// SPDX-License-Identifier: GPL-2.0+
/*
 * store：Amlogic 线刷主机发给 u-boot 的 store 命令词的只读子集
 *
 * 厂商 u-boot（common/store_interface.c）的 store 是线刷协议依赖的存储抽象：主机按厂商分区表的分区名或 user 区的 store 坐标读写 eMMC。这里实现主机实际用到的只读子集，语义与数字写法照厂商：
 *   store init [<flag>]                     解析 store 设备与分区表；flag 按 16 进制，0、1 不擦除，厂商 2、3 会擦除整个存储，这里拒绝
 *   store read <分区名> <地址> <偏移> <长度>   分区内 [偏移, 偏移 + 长度) 读到内存，偏移相对分区头
 *   store rom_read <地址> <偏移> <长度>        user 区 store 坐标读：物理偏移 = 偏移 + 0x200（GXL 起厂商 off += 512，M12 实测）
 * 地址按 16 进制（厂商 simple_strtoul(…, 16)）；偏移与长度按 16 进制、可带 k/m/g 后缀（厂商 str2longlong），整串都要是数字。读越过分区或设备末尾即报错、不读，不截断。
 * 不实现写类子命令（store write、rom_write、erase 等）：主机写分区走 Optimus 协议的 download store（drivers/optimus），写物理偏移走 mmc write（红线门）。
 * 厂商在 0–4 MiB 的 rom_read 之后还会切到 boot0、boot1 读一遍（MMC_BOOT_PARTITION_SUPPORT 时读回的是 boot1），这里只读 user 区。
 */

#include <aml_store.h>
#include <blk.h>
#include <command.h>
#include <env.h>
#include <errno.h>
#include <limits.h>
#include <malloc.h>
#include <mapmem.h>
#include <memalign.h>
#include <part.h>
#include <vsprintf.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

#define BOUNCE		SZ_64K

struct blk_desc *aml_store_dev(void)
{
	const char *spec = env_get("store_dev");
	char buf[32], *dev;
	struct blk_desc *desc;
	int ret;

	if (!spec)
		spec = CONFIG_CMD_AML_STORE_DEV;
	if (strlcpy(buf, spec, sizeof(buf)) >= sizeof(buf)) {
		printf("store: store_dev '%s' too long\n", spec);
		return NULL;
	}
	dev = strchr(buf, ' ');
	if (!dev) {
		printf("store: store_dev '%s' is not '<interface> <device>'\n", spec);
		return NULL;
	}
	*dev++ = '\0';
	ret = blk_get_device_by_str(buf, dev, &desc);
	if (ret < 0) {
		printf("store: no device %s\n", spec);
		return NULL;
	}
	if (desc->part_type == PART_TYPE_UNKNOWN)
		part_init(desc);

	return desc;
}

int aml_store_part(const char *name, struct blk_desc **descp,
		   struct disk_partition *info)
{
	struct blk_desc *desc = aml_store_dev();

	if (!desc)
		return -ENODEV;
	if (part_get_info_by_name(desc, name, info) < 0) {
		printf("store: no partition '%s'\n", name);
		return -ENOENT;
	}
	*descp = desc;

	return 0;
}

/* [@off, @off + @len) 在设备里（不回绕） */
static bool aml_store_fits(struct blk_desc *desc, u64 off, u64 len)
{
	u64 size = (u64)desc->lba * desc->blksz;

	return off <= size && len <= size - off;
}

/* 头尾不足一块的部分：读出那一块，按方向拷入或拷出，写时再写回 */
static int aml_store_partial(struct blk_desc *desc, u64 off, u64 len, void *buf,
			     bool write, u8 *blk)
{
	lbaint_t lba = off / desc->blksz;
	uint skip = off % desc->blksz;

	if (blk_dread(desc, lba, 1, blk) != 1)
		return -EIO;
	if (!write) {
		memcpy(buf, blk + skip, len);
		return 0;
	}
	memcpy(blk + skip, buf, len);

	return blk_dwrite(desc, lba, 1, blk) == 1 ? 0 : -EIO;
}

static int aml_store_rw(struct blk_desc *desc, u64 off, u64 len, void *buf,
			bool write)
{
	ulong bsz = desc->blksz;
	u8 *bounce;
	int ret = 0;

	if (!aml_store_fits(desc, off, len))
		return -EINVAL;
	bounce = memalign(ARCH_DMA_MINALIGN, BOUNCE);
	if (!bounce)
		return -ENOMEM;
	while (len && !ret) {
		u64 n;

		if (off % bsz || len < bsz) {
			n = min_t(u64, len, bsz - off % bsz);
			ret = aml_store_partial(desc, off, n, buf, write, bounce);
		} else {
			lbaint_t cnt, got;

			/* 整块部分经对齐的中转缓冲：调用方的内存地址不一定满足 DMA 对齐 */
			n = min_t(u64, len - len % bsz, BOUNCE);
			cnt = n / bsz;
			if (write) {
				memcpy(bounce, buf, n);
				got = blk_dwrite(desc, off / bsz, cnt, bounce);
			} else {
				got = blk_dread(desc, off / bsz, cnt, bounce);
				memcpy(buf, bounce, n);
			}
			if (got != cnt)
				ret = -EIO;
		}
		off += n;
		len -= n;
		buf += n;
	}
	free(bounce);

	return ret;
}

int aml_store_read(struct blk_desc *desc, u64 off, u64 len, void *buf)
{
	return aml_store_rw(desc, off, len, buf, false);
}

int aml_store_write(struct blk_desc *desc, u64 off, u64 len, const void *buf)
{
	return aml_store_rw(desc, off, len, (void *)buf, true);
}

/* 厂商 str2longlong：16 进制，可带 k/m/g 后缀，整串都要用掉 */
static int store_num(const char *s, u64 *val)
{
	char *end;
	int shift = 0;

	*val = simple_strtoull(s, &end, 16);
	switch (*end) {
	case 'g':
	case 'G':
		shift = 30;
		end++;
		break;
	case 'm':
	case 'M':
		shift = 20;
		end++;
		break;
	case 'k':
	case 'K':
		shift = 10;
		end++;
		break;
	}
	if (end == s || *end || (shift && *val >> (64 - shift))) {
		printf("store: '%s' is not a number\n", s);
		return -EINVAL;
	}
	*val <<= shift;

	return 0;
}

static int store_to_mem(struct blk_desc *desc, const char *addr_s, u64 off,
			u64 len)
{
	ulong addr = hextoul(addr_s, NULL);
	void *buf;
	int ret;

	buf = map_sysmem(addr, len);
	ret = aml_store_read(desc, off, len, buf);
	unmap_sysmem(buf);
	if (ret) {
		printf("store: read 0x%llx bytes at 0x%llx failed (%d)\n", len,
		       off, ret);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

static int do_store_init(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	struct blk_desc *desc;
	ulong init_flag = argc > 1 ? hextoul(argv[1], NULL) : 0;

	if (init_flag > 1) {
		printf("store: init flag %lu would erase the storage; not supported\n",
		       init_flag);
		return CMD_RET_FAILURE;
	}
	desc = aml_store_dev();
	if (!desc)
		return CMD_RET_FAILURE;
	printf("store: %s %d, %lu-byte blocks, 0x" LBAF " blocks, partition table %s\n",
	       blk_get_uclass_name(desc->uclass_id), desc->devnum, desc->blksz,
	       desc->lba, desc->part_type == PART_TYPE_UNKNOWN ? "unknown" : "found");

	return CMD_RET_SUCCESS;
}

static int do_store_read(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	struct disk_partition info;
	struct blk_desc *desc;
	u64 off, len, size;

	if (argc != 5)
		return CMD_RET_USAGE;
	if (store_num(argv[3], &off) || store_num(argv[4], &len))
		return CMD_RET_FAILURE;
	if (aml_store_part(argv[1], &desc, &info))
		return CMD_RET_FAILURE;
	size = (u64)info.size * info.blksz;
	if (off > size || len > size - off) {
		printf("store: 0x%llx+0x%llx is past the end of '%s' (0x%llx bytes)\n",
		       off, len, argv[1], size);
		return CMD_RET_FAILURE;
	}

	return store_to_mem(desc, argv[2], (u64)info.start * info.blksz + off, len);
}

static int do_store_rom_read(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{
	struct blk_desc *desc;
	u64 off, len;

	if (argc != 4)
		return CMD_RET_USAGE;
	if (store_num(argv[2], &off) || store_num(argv[3], &len))
		return CMD_RET_FAILURE;
	desc = aml_store_dev();
	if (!desc)
		return CMD_RET_FAILURE;
	if (off > U64_MAX - AML_STORE_ROM_SKEW ||
	    !aml_store_fits(desc, off + AML_STORE_ROM_SKEW, len)) {
		printf("store: 0x%llx+0x%llx (store offset) is past the end of the device\n",
		       off, len);
		return CMD_RET_FAILURE;
	}

	return store_to_mem(desc, argv[1], off + AML_STORE_ROM_SKEW, len);
}

U_BOOT_LONGHELP(store,
	"init [<flag>]\n"
	"    - find the store device ($store_dev, default \"" CONFIG_CMD_AML_STORE_DEV "\")\n"
	"      and its partition table; flags 2 and 3 (erase) are refused\n"
	"store read <part> <addr> <off> <len>\n"
	"    - read <len> bytes at <off> of partition <part> to <addr>\n"
	"store rom_read <addr> <off> <len>\n"
	"    - read <len> bytes of the user area at physical <off> + 0x200\n"
	"<addr> is hex; <off> and <len> are hex and may end in k, m or g");

U_BOOT_CMD_WITH_SUBCMDS(store, "Amlogic burning-tool storage commands (read-only)",
	store_help_text,
	U_BOOT_SUBCMD_MKENT(init, 2, 0, do_store_init),
	U_BOOT_SUBCMD_MKENT(read, 5, 0, do_store_read),
	U_BOOT_SUBCMD_MKENT(rom_read, 4, 0, do_store_rom_read));
