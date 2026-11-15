// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic eMMC 私有分区表 MPT（MMC Partition Table）
 *
 * 厂商 u-boot 把分区表存在 eMMC user 区 reserved 分区的起点（偏移见 CONFIG_AML_PARTITION_OFFSET；联通 M12 的扇区 0 是 0xFF，没有 MBR/GPT）。格式与校验和取自厂商 drivers/mmc/aml_emmc_partition.c（struct ptbl_rsv、get_ptbl_rsv、_calc_iptbl_check_v1/_v2），并以联通 M12 出厂表的真字节核对：
 *
 *   0x00 magic[4]     "MPT\0"
 *   0x04 version[12]  "01.00.00\0"（v1）或 "01.02.00\0"（v2）
 *   0x10 count        u32 LE，1..32
 *   0x14 checksum     u32 LE
 *   0x18 entries[]    每项 40 B：name[16] size(u64) offset(u64) mask_flags(u32) + 4 B 对齐填充（64 位厂商 u-boot 的结构体布局）
 *
 * 校验和是 32 位字的回绕和：v2 对前 count 项的全部字求和；v1 有厂商注明“do not fix”的缺陷，内层指针每轮都从第 0 项起算，结果是 count 乘以第 0 项 10 个字之和，第 1 项以后不受校验和保护。所以 v1 表的其余各项靠结构检查兜底：名字非空、偏移与大小按块对齐、不越过盘尾、按偏移升序且互不重叠（厂商 _calculate_offset 逐项顺排，表天然满足）。任何一项不过整张表都不认。
 *
 * 探测顺序由链接表按驱动名排序决定（a_efi、amiga、dos、iso、mac、mpt……），本驱动以 mpt 为名，排在 EFI 与 DOS 之后。GPT 先于 MPT：GPT 自带头与表项 CRC；厂商树开 CONFIG_AML_GPT 时由 MPT 生成 GPT，GPT_PRIORITY 下再以 GPT 覆盖 MPT（厂商 mmc_device_init）。DOS 先于 MPT：有效的 MBR 是主线 Linux 与 distro 启动认的分区表，而 MPT 在 12 MiB 处、重新分区不会清掉它——把 eMMC 镜像整盘 dd 到 U 盘或 SD 卡后重新分区、或在 eMMC 上改写 MPT 后再建 MBR 装主线系统，盘上都同时有有效 MBR 与残留 MPT。代价是厂商 CONFIG_PTBL_MBR 写的 MBR（由 MPT 生成的镜像，没有分区名）与 MPT 同在时取 MBR，分区名用不了；扇区 0 没有 MBR 的盘（联通 M12 是 0xFF）不受影响。
 *
 * 魔数对上而其余校验不过时打印原因。part_init() 与每次按未知类型查表（disk/part.c 的 part_driver_lookup_type()）都会重跑各驱动的 test，坏表的盘上每条 part、fs 命令都会走到这里，所以同一块盘的同一个原因只打印一次；原因变了、表又能认出或魔数没了之后，再出现的原因照常打印。
 */

#include <blk.h>
#include <memalign.h>
#include <part.h>
#include <stdio.h>
#include <vsprintf.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <asm/byteorder.h>

#define AML_MPT_MAGIC		"MPT"	/* 连同结尾 NUL 共 4 字节 */
#define AML_MPT_VERSION_1	"01.00.00"
#define AML_MPT_VERSION_2	"01.02.00"
#define AML_MPT_MAX_ENTRIES	32	/* 厂商 MAX_MMC_PART_NUM */
#define AML_MPT_NAME_LEN	16

struct aml_mpt_entry {
	char name[AML_MPT_NAME_LEN];
	__le64 size;
	__le64 offset;
	__le32 mask_flags;
	__le32 pad;
} __packed;

struct aml_mpt {
	char magic[4];
	char version[12];
	__le32 count;
	__le32 checksum;
	struct aml_mpt_entry entries[AML_MPT_MAX_ENTRIES];
} __packed;

/* 读表用的缓冲：按块向上取整、按 cache 行对齐 */
#define AML_MPT_BUF(name, desc) \
	ALLOC_CACHE_ALIGN_BUFFER(u8, name, \
				 roundup(sizeof(struct aml_mpt), (desc)->blksz))

static u32 aml_mpt_sum(const struct aml_mpt_entry *e, uint n)
{
	const __le32 *w = (const __le32 *)e;
	u32 sum = 0;
	uint i;

	for (i = 0; i < n * sizeof(*e) / sizeof(*w); i++)
		sum += le32_to_cpu(w[i]);

	return sum;
}

/* 最近打印过的拒绝原因与它所在的盘（uclass、设备号、硬件分区） */
static struct {
	enum uclass_id uclass_id;
	int devnum;
	int hwpart;
	char msg[192];
} aml_mpt_last;

static bool aml_mpt_same_disk(const struct blk_desc *desc)
{
	return aml_mpt_last.uclass_id == desc->uclass_id &&
	       aml_mpt_last.devnum == desc->devnum &&
	       aml_mpt_last.hwpart == desc->hwpart;
}

/* 打印拒绝原因（带盘的名字），与这块盘上一次打印的相同就不再打印 */
static __printf(2, 3) void aml_mpt_complain(struct blk_desc *desc,
					    const char *fmt, ...)
{
	char msg[sizeof(aml_mpt_last.msg)];
	va_list args;
	int n;

	n = snprintf(msg, sizeof(msg), "MPT on %s %d: ",
		     blk_get_uclass_name(desc->uclass_id), desc->devnum);
	va_start(args, fmt);
	vsnprintf(msg + n, sizeof(msg) - n, fmt, args);
	va_end(args);
	if (aml_mpt_same_disk(desc) && !strcmp(msg, aml_mpt_last.msg))
		return;
	aml_mpt_last.uclass_id = desc->uclass_id;
	aml_mpt_last.devnum = desc->devnum;
	aml_mpt_last.hwpart = desc->hwpart;
	strlcpy(aml_mpt_last.msg, msg, sizeof(aml_mpt_last.msg));
	printf("%s\n", msg);
}

/**
 * aml_mpt_read() - 读出并校验 MPT
 *
 * @desc: 块设备
 * @buf: 至少 roundup(sizeof(struct aml_mpt), blksz) 字节、cache 对齐
 * @loud: 魔数对上但校验不过时打印原因（同一原因只打印一次，见文件头；魔数不对时总是安静）
 * Return: 表项数（1..32），或负的 errno
 */
static int aml_mpt_read(struct blk_desc *desc, u8 *buf, bool loud)
{
	const struct aml_mpt *mpt = (const struct aml_mpt *)buf;
	u64 disk_end, prev_end = 0, off, size;
	lbaint_t start, cnt;
	u32 count, sum;
	int version;
	uint i;

	if (!desc->blksz || CONFIG_AML_PARTITION_OFFSET % desc->blksz)
		return -EINVAL;
	start = CONFIG_AML_PARTITION_OFFSET / desc->blksz;
	cnt = DIV_ROUND_UP(sizeof(*mpt), desc->blksz);
	if (start + cnt > desc->lba)
		return -ENOSPC;
	if (blk_dread(desc, start, cnt, buf) != cnt)
		return -EIO;

	if (memcmp(mpt->magic, AML_MPT_MAGIC, sizeof(AML_MPT_MAGIC))) {
		if (aml_mpt_same_disk(desc))
			aml_mpt_last.msg[0] = '\0';
		return -ENOENT;
	}

	if (!memcmp(mpt->version, AML_MPT_VERSION_1, sizeof(AML_MPT_VERSION_1))) {
		version = 1;
	} else if (!memcmp(mpt->version, AML_MPT_VERSION_2,
			   sizeof(AML_MPT_VERSION_2))) {
		version = 2;
	} else {
		if (loud)
			aml_mpt_complain(desc, "unknown version '%.12s'", mpt->version);
		return -EPROTONOSUPPORT;
	}

	count = le32_to_cpu(mpt->count);
	if (!count || count > AML_MPT_MAX_ENTRIES) {
		if (loud)
			aml_mpt_complain(desc, "bad entry count %u", count);
		return -EINVAL;
	}

	if (version == 1)
		sum = count * aml_mpt_sum(mpt->entries, 1);
	else
		sum = aml_mpt_sum(mpt->entries, count);
	if (sum != le32_to_cpu(mpt->checksum)) {
		if (loud)
			aml_mpt_complain(desc, "checksum %08x, computed %08x (v%d)",
					 le32_to_cpu(mpt->checksum), sum, version);
		return -EBADMSG;
	}

	disk_end = (u64)desc->lba * desc->blksz;
	for (i = 0; i < count; i++) {
		const struct aml_mpt_entry *e = &mpt->entries[i];

		off = le64_to_cpu(e->offset);
		size = le64_to_cpu(e->size);
		if (!e->name[0] || off % desc->blksz || size % desc->blksz ||
		    off < prev_end || off > disk_end || size > disk_end - off) {
			if (loud)
				aml_mpt_complain(desc, "entry %u '%.16s' offset %#llx size %#llx invalid (previous end %#llx, disk end %#llx)",
						 i + 1, e->name, off, size, prev_end,
						 disk_end);
			return -EINVAL;
		}
		prev_end = off + size;
	}
	if (aml_mpt_same_disk(desc))
		aml_mpt_last.msg[0] = '\0';

	return count;
}

static int part_test_aml(struct blk_desc *desc)
{
	AML_MPT_BUF(buf, desc);

	return aml_mpt_read(desc, buf, true) > 0 ? 0 : -1;
}

static int part_get_info_aml(struct blk_desc *desc, int part,
			     struct disk_partition *info)
{
	AML_MPT_BUF(buf, desc);
	const struct aml_mpt_entry *e;
	size_t len;
	int count;

	count = aml_mpt_read(desc, buf, false);
	if (count < 0)
		return count;
	if (part < 1 || part > count)
		return -ENOENT;

	e = &((const struct aml_mpt *)buf)->entries[part - 1];
	info->start = le64_to_cpu(e->offset) / desc->blksz;
	info->size = le64_to_cpu(e->size) / desc->blksz;
	info->blksz = desc->blksz;
	/*
	 * type 不是盘上的分区类型，而是 u-boot 内部的标记：blk_get_device_part_str()（load、ls、fstype、fatload、ums 等 <dev>[:<n>] 写法都经它取分区）只接受 BOOT_PART_TYPE，DOS、EFI、ISO 驱动都填这个值。
	 */
	strcpy((char *)info->type, BOOT_PART_TYPE);
	/*
	 * disk_partition_clr() 只清 name、type、uuid、type_guid（MPT 没有 UUID 与类型 GUID，保持清空），下面几个字段不写就是调用者栈上的残值（part list -bootable、part_get_bootable() 会读 bootable）。MPT 没有可启动标志与分区类型码：mask_flags 在厂商树里是存储类别（include/storage.h 的 STORE_CODE/STORE_CACHE/STORE_DATA；common/fdt_support.c 把位 0 当只读），与 MBR 的 0x80、GPT 的 legacy_bios_bootable 都不对应。
	 */
	info->bootable = 0;
	info->type_flags = 0;
#ifdef CONFIG_DOS_PARTITION
	info->sys_ind = 0;
#endif
	/* 名字可以占满 16 字节而不带 NUL */
	len = strnlen(e->name, sizeof(e->name));
	memcpy(info->name, e->name, len);
	info->name[len] = '\0';

	return 0;
}

static void part_print_aml(struct blk_desc *desc)
{
	AML_MPT_BUF(buf, desc);
	const struct aml_mpt *mpt = (const struct aml_mpt *)buf;
	int count, i;

	count = aml_mpt_read(desc, buf, true);
	if (count < 0)
		return;

	printf("MPT version %.8s at %#x, %d entries\n", mpt->version,
	       CONFIG_AML_PARTITION_OFFSET, count);
	printf("Part\tStart LBA\tEnd LBA\t\tMask\tName\n");
	for (i = 0; i < count; i++) {
		const struct aml_mpt_entry *e = &mpt->entries[i];
		u64 start = le64_to_cpu(e->offset) / desc->blksz;
		u64 blks = le64_to_cpu(e->size) / desc->blksz;

		printf("%3d\t0x%08llx\t0x%08llx\t%u\t\"%.16s\"\n", i + 1,
		       start, blks ? start + blks - 1 : start,
		       le32_to_cpu(e->mask_flags), e->name);
	}
}

U_BOOT_PART_TYPE(mpt) = {
	.name		= "AML",
	.part_type	= PART_TYPE_AML,
	.max_entries	= AML_MPT_MAX_ENTRIES,
	.get_info	= part_get_info_ptr(part_get_info_aml),
	.print		= part_print_ptr(part_print_aml),
	.test		= part_test_aml,
};
