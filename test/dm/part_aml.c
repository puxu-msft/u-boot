// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic MPT 分区驱动（disk/part_aml.c）的测试
 *
 * 用 blkmap 把一块 16 MiB 的内存映射成磁盘，在 CONFIG_AML_PARTITION_OFFSET 处按字节偏移手写 MPT（不复用驱动的结构体定义，布局错了测试能看出来），先验证正常表的探测、取信息、按名字查找与经 blk_get_device_part_str() 按分区号取分区，再逐个构造坏表，确认每一种都因预期的那项检查被拒绝、同一块盘的同一个原因只打印一次，最后核对与 DOS、EFI 的探测先后（有效 MBR 与残留 MPT 同在时取 DOS）。
 */

#include <blk.h>
#include <blkmap.h>
#include <command.h>
#include <dm.h>
#include <env.h>
#include <malloc.h>
#include <part.h>
#include <u-boot/uuid.h>
#include <asm/unaligned.h>
#include <dm/test.h>
#include <linux/sizes.h>
#include <test/test.h>
#include <test/ut.h>

#define BLKSZ		512
#define DISK_SIZE	SZ_16M
#define MPT_OFF		CONFIG_AML_PARTITION_OFFSET
#define ENT(i)		(MPT_OFF + 24 + (i) * 40)

struct mpt_ent {
	const char *name;
	u64 offset;
	u64 size;
	u32 mask;
};

/* 与 M12 出厂表同样的排法：bootloader、间隙、reserved（表本身在它起点）、其余顺排到盘尾 */
static const struct mpt_ent ents[] = {
	{ "bootloader", 0, SZ_4M, 0 },
	{ "reserved", MPT_OFF, SZ_1M, 0 },
	{ "boot", MPT_OFF + SZ_1M, SZ_1M, 1 },
	{ "data", MPT_OFF + SZ_2M, DISK_SIZE - MPT_OFF - SZ_2M, 4 },
};

static u32 mpt_sum(const u8 *disk, uint n)
{
	u32 sum = 0;
	uint i;

	for (i = 0; i < n * 10; i++)
		sum += get_unaligned_le32(disk + ENT(0) + i * 4);

	return sum;
}

/* 按版本重算校验和；count 取表头里的值 */
static void mpt_fix_checksum(u8 *disk)
{
	u32 count = get_unaligned_le32(disk + MPT_OFF + 16);
	u32 sum;

	if (!memcmp(disk + MPT_OFF + 4, "01.02.00", 9))
		sum = mpt_sum(disk, count);
	else
		sum = count * mpt_sum(disk, 1);
	put_unaligned_le32(sum, disk + MPT_OFF + 20);
}

static void mpt_write(u8 *disk, const char *version)
{
	uint i;

	memset(disk + MPT_OFF, '\0', SZ_16K);
	memcpy(disk + MPT_OFF, "MPT", 4);
	strcpy((char *)disk + MPT_OFF + 4, version);
	put_unaligned_le32(ARRAY_SIZE(ents), disk + MPT_OFF + 16);
	for (i = 0; i < ARRAY_SIZE(ents); i++) {
		strncpy((char *)disk + ENT(i), ents[i].name, 16);
		put_unaligned_le64(ents[i].size, disk + ENT(i) + 16);
		put_unaligned_le64(ents[i].offset, disk + ENT(i) + 24);
		put_unaligned_le32(ents[i].mask, disk + ENT(i) + 32);
	}
	mpt_fix_checksum(disk);
}

/* 重新探测，断言不是 MPT，且拒绝原因（盘名之后的部分）以 @reason 开头（NULL 表示不出声：魔数不对，或同一原因已经打印过） */
static int mpt_rejected(struct unit_test_state *uts, struct blk_desc *desc,
			const char *reason)
{
	struct disk_partition info;

	ut_assert_console_end();
	/* 当作新插入的盘探测：part_init 不先复测上次认出的类型 */
	desc->part_type = PART_TYPE_UNKNOWN;
	part_init(desc);
	ut_assert(desc->part_type != PART_TYPE_AML);
	if (reason)
		ut_assert_nextlinen("MPT on blkmap %d: %s", desc->devnum, reason);
	ut_assert_console_end();
	ut_assert(part_get_info_by_type(desc, 1, PART_TYPE_AML, &info) < 0);
	console_record_reset();

	return 0;
}

static int dm_test_part_aml(struct unit_test_state *uts)
{
	struct disk_partition info;
	struct udevice *dev, *blk;
	struct blk_desc *desc, *found;
	char spec[16];
	u8 *disk;
	uint i;

	disk = calloc(1, DISK_SIZE);
	ut_assertnonnull(disk);
	ut_assertok(blkmap_create("mpt", &dev));
	ut_assertok(blkmap_map_mem(dev, 0, DISK_SIZE / BLKSZ, disk));
	ut_assertok(blk_get_from_parent(dev, &blk));
	desc = dev_get_uclass_plat(blk);

	/* 正常 v1 表：四项的起点、大小、名字，type 是 blk_get_device_part_str() 要求的 BOOT_PART_TYPE，按名字查找，越界的项号；MPT 没有的字段（可启动、类型码、UUID、类型 GUID）不得带出调用者的残值 */
	mpt_write(disk, "01.00.00");
	part_init(desc);
	ut_asserteq(PART_TYPE_AML, desc->part_type);
	for (i = 0; i < ARRAY_SIZE(ents); i++) {
		memset(&info, 0xff, sizeof(info));
		ut_assertok(part_get_info(desc, i + 1, &info));
		ut_asserteq(ents[i].offset / BLKSZ, info.start);
		ut_asserteq(ents[i].size / BLKSZ, info.size);
		ut_asserteq(BLKSZ, info.blksz);
		ut_asserteq_str(ents[i].name, (char *)info.name);
		ut_asserteq_str(BOOT_PART_TYPE, (char *)info.type);
		ut_asserteq(0, info.bootable);
		ut_asserteq(0, info.type_flags);
		if (IS_ENABLED(CONFIG_DOS_PARTITION))
			ut_asserteq(0, disk_partition_sys_ind(&info));
		if (CONFIG_IS_ENABLED(PARTITION_UUIDS))
			ut_asserteq_str("", disk_partition_uuid(&info));
		if (IS_ENABLED(CONFIG_PARTITION_TYPE_GUID))
			ut_asserteq_str("", disk_partition_type_guid(&info));
	}
	ut_asserteq(0, part_get_bootable(desc));
	ut_assert(part_get_info(desc, ARRAY_SIZE(ents) + 1, &info) < 0);
	ut_asserteq(3, part_get_info_by_name(desc, "boot", &info));
	ut_asserteq((MPT_OFF + SZ_1M) / BLKSZ, info.start);

	/* 按分区号取分区：part_get_info_by_dev_and_name_or_num()（fs 命令与 ums 的入口）不带 '#' 时转给 blk_get_device_part_str()，后者只接受 type 为 BOOT_PART_TYPE 的分区。写法有 <dev>:<n>、不带分区号（取第 1 项）、<dev>:auto（没有可启动项时取第 1 项） */
	console_record_reset();
	snprintf(spec, sizeof(spec), "%d:3", desc->devnum);
	ut_asserteq(3, part_get_info_by_dev_and_name_or_num("blkmap", spec, &found, &info, 1));
	ut_asserteq_ptr(desc, found);
	ut_asserteq((MPT_OFF + SZ_1M) / BLKSZ, info.start);
	ut_asserteq_str("boot", (char *)info.name);
	snprintf(spec, sizeof(spec), "%d", desc->devnum);
	ut_asserteq(1, part_get_info_by_dev_and_name_or_num("blkmap", spec, &found, &info, 1));
	ut_asserteq_str("bootloader", (char *)info.name);
	snprintf(spec, sizeof(spec), "%d:auto", desc->devnum);
	ut_asserteq(1, part_get_info_by_dev_and_name_or_num("blkmap", spec, &found, &info, 1));
	ut_asserteq_str("bootloader", (char *)info.name);
	ut_assert_console_end();

	/* 经命令行的 <dev>#<名字> 路径 */
	ut_assertok(run_commandf("part start blkmap %d boot mpt_start",
				 desc->devnum));
	ut_asserteq((MPT_OFF + SZ_1M) / BLKSZ, env_get_hex("mpt_start", 0));
	ut_assertok(run_commandf("part size blkmap %d data mpt_size",
				 desc->devnum));
	ut_asserteq(ents[3].size / BLKSZ, env_get_hex("mpt_size", 0));

	/* 名字占满 16 字节、不带 NUL */
	memcpy(disk + ENT(2), "boot-with-16-chr", 16);
	mpt_fix_checksum(disk);
	part_init(desc);
	ut_asserteq(PART_TYPE_AML, desc->part_type);
	ut_assertok(part_get_info(desc, 3, &info));
	ut_asserteq_str("boot-with-16-chr", (char *)info.name);

	/* 正常 v2 表：校验和覆盖全部项，改第 3 项就不认（v1 只保护第 1 项） */
	mpt_write(disk, "01.02.00");
	part_init(desc);
	ut_asserteq(PART_TYPE_AML, desc->part_type);
	disk[ENT(2)] ^= 0x20;
	ut_assertok(mpt_rejected(uts, desc, "checksum"));

	/* 坏魔数：安静地不认 */
	mpt_write(disk, "01.00.00");
	disk[MPT_OFF] = 'X';
	ut_assertok(mpt_rejected(uts, desc, NULL));

	/* 未知版本 */
	mpt_write(disk, "01.01.00");
	ut_assertok(mpt_rejected(uts, desc, "unknown version"));

	/* 坏校验和（v1 覆盖第 1 项） */
	mpt_write(disk, "01.00.00");
	disk[ENT(0) + 16] ^= 0x01;
	ut_assertok(mpt_rejected(uts, desc, "checksum"));

	/* 同一块盘、同一个原因只打印一次：再探测、按未知类型查表（每条 part/fs 命令都走这里）都不再出声 */
	ut_assertok(mpt_rejected(uts, desc, NULL));
	ut_assert(part_get_info(desc, 1, &info) < 0);
	ut_assertok(run_commandf("part list blkmap %d mpt_parts", desc->devnum));
	ut_assert_console_end();
	ut_asserteq_str("", env_get("mpt_parts") ?: "");
	ut_assertok(run_commandf("part list blkmap %d", desc->devnum));
	ut_assert_nextline("## Unknown partition table type 0");
	ut_assert_console_end();
	/* 表又能认出之后，同样的坏法再出现时照常打印 */
	disk[ENT(0) + 16] ^= 0x01;
	desc->part_type = PART_TYPE_UNKNOWN;
	part_init(desc);
	ut_asserteq(PART_TYPE_AML, desc->part_type);
	disk[ENT(0) + 16] ^= 0x01;
	ut_assertok(mpt_rejected(uts, desc, "checksum"));

	/* 项数为 0、超过 32（校验和按伪造的项数重算，只让项数检查失败） */
	mpt_write(disk, "01.00.00");
	put_unaligned_le32(0, disk + MPT_OFF + 16);
	mpt_fix_checksum(disk);
	ut_assertok(mpt_rejected(uts, desc, "bad entry count 0"));
	put_unaligned_le32(33, disk + MPT_OFF + 16);
	mpt_fix_checksum(disk);
	ut_assertok(mpt_rejected(uts, desc, "bad entry count 33"));

	/* 越界：最后一项多出一个块 */
	mpt_write(disk, "01.02.00");
	put_unaligned_le64(ents[3].size + BLKSZ, disk + ENT(3) + 16);
	mpt_fix_checksum(disk);
	ut_assertok(mpt_rejected(uts, desc, "entry 4 'data'"));

	/* 偏移大到加大小会回绕 */
	mpt_write(disk, "01.02.00");
	put_unaligned_le64(~0ULL - BLKSZ + 1, disk + ENT(3) + 24);
	mpt_fix_checksum(disk);
	ut_assertok(mpt_rejected(uts, desc, "entry 4 'data'"));

	/* 与前一项重叠 */
	mpt_write(disk, "01.02.00");
	put_unaligned_le64(MPT_OFF + SZ_1M - BLKSZ, disk + ENT(2) + 24);
	mpt_fix_checksum(disk);
	ut_assertok(mpt_rejected(uts, desc, "entry 3 'boot'"));

	/* 偏移不按块对齐 */
	mpt_write(disk, "01.02.00");
	put_unaligned_le64(MPT_OFF + SZ_1M + 1, disk + ENT(2) + 24);
	mpt_fix_checksum(disk);
	ut_assertok(mpt_rejected(uts, desc, "entry 3 'boot'"));

	/* 空名字 */
	mpt_write(disk, "01.02.00");
	memset(disk + ENT(1), '\0', 16);
	mpt_fix_checksum(disk);
	ut_assertok(mpt_rejected(uts, desc, "entry 2 ''"));

	/*
	 * 与 DOS/EFI 的先后：有效 MBR 与完好的 MPT 同在（eMMC 镜像 dd 到别的盘后重新分区、或改写 MPT 后再建 MBR 的盘）时取 DOS，分区是 MBR 的那一项；去掉 MBR 签名（正对照）则归 MPT
	 */
	mpt_write(disk, "01.00.00");
	disk[0x1be + 4] = 0x83;
	put_unaligned_le32(2048, disk + 0x1be + 8);
	put_unaligned_le32(2048, disk + 0x1be + 12);
	disk[0x1fe] = 0x55;
	disk[0x1ff] = 0xaa;
	desc->part_type = PART_TYPE_UNKNOWN;
	part_init(desc);
	ut_asserteq(PART_TYPE_DOS, desc->part_type);
	ut_assertok(part_get_info(desc, 1, &info));
	ut_asserteq(2048, info.start);
	ut_asserteq(2048, info.size);
	ut_assert(part_get_info(desc, 2, &info) < 0);
	disk[0x1fe] = 0;
	desc->part_type = PART_TYPE_UNKNOWN;
	part_init(desc);
	ut_asserteq(PART_TYPE_AML, desc->part_type);

	/* GPT 与 MPT 同在时 GPT 优先 */
	if (IS_ENABLED(CONFIG_EFI_PARTITION)) {
		struct disk_partition gpt = {
			.start = 48, .size = 16, .name = "gpt1",
		};
		char guid[UUID_STR_LEN + 1] =
			"8d60b397-1bb6-4d33-80ee-b1587d24c2f8";

		disk_partition_set_uuid(&gpt, "c5bce7a2-03f0-4d03-9048-01ff23b9d527");
		ut_assertok(gpt_restore(desc, guid, &gpt, 1));
		desc->part_type = PART_TYPE_UNKNOWN;
		part_init(desc);
		ut_asserteq(PART_TYPE_EFI, desc->part_type);
		ut_assertok(part_get_info(desc, 1, &info));
		ut_asserteq_str("gpt1", (char *)info.name);
	}
	console_record_reset();

	ut_assertok(blkmap_destroy(dev));
	free(disk);

	return 0;
}
DM_TEST(dm_test_part_aml, UTF_CONSOLE);
