// SPDX-License-Identifier: GPL-2.0+
/*
 * 存储写入类命令的红线确认
 *
 * cmd_process() 在执行每条命令前调 redline_gate()。命令（按命令表里的规范名匹配，缩写与 .后缀同样命中）与子命令落在 rl_rules 里时不执行，而是打印红线动作清单并返回失败：完整命令行；动作；写入的源数据（地址、长度与 crc32；eFuse 写——sm efusewrite 的源缓冲、smc 0x82000031 的安全监控输入共享缓冲——直接列出要烧的字节）；块设备写入时的接口、设备号、硬件分区、起止块、字节范围与重叠的分区（经分区层枚举，Amlogic eMMC 上即 MPT 分区）；不可逆的动作单独标出。被拦的动作记为唯一的待确认动作，新被拦的动作替换它。
 * redline yes 执行待确认动作一次，执行前就作废。它只接受在更早的一次输入里被拦的动作：交互解析器每读进一行命令（cli_simple 与两种 hush 的读行处）、fastboot 每处理一个执行命令的请求（oem run、UCmd、ACmd），都调 redline_input_next() 让输入代次加一；同一行、同一个 run 变量、同一段 bootcmd/preboot/source 脚本或同一个 fastboot 请求里的 redline yes 一律拒绝——这些路径上清单还没有人看过。执行前按当时的状态重新生成清单（源数据的 crc32 重新计算），与展示过的不同（期间换了 mmc 当前设备或硬件分区、分区表变了、源数据改了等）就不执行，改为展示新清单、等再次确认。redline show 再打印清单，redline cancel 丢弃。
 * 子命令判定：子命令表按前缀匹配的命令（mmc、sm、env）用与 find_cmd_tbl() 相同的规则解析，规则里列全该命令在上游的全部子命令，只读的放行，写的与解析不出的（歧义、未知）都拦；子命令按全名比较的命令（usb、gpt、mbr）只拦写的那几个；smc/hvc 按 fid 白名单放行。
 * 清单读源数据之前先确认它整段在内存里、且不碰 lmb 里带 no-map 的保留区（控制 DT /reserved-memory 的 no-map 节点，Amlogic 上是 BL31/BL32 的安全区：CPU 读安全内存可能总线错误挂死），不满足就不读、在清单里注明。
 * 清单解析设备时与命令调同一个函数：blk_get_device_by_str() 一族会按 <dev>[.<hwpart>] 切换 MMC 的当前硬件分区（不带 .<hwpart> 时切回 user 区），被拦、未执行的命令同样留下这一切换。
 * 覆盖范围是“经 cmd_process() 按名字执行的命令”：内核、EFI 程序、go/bootm 跳进去的代码，fastboot flash（FASTBOOT_FLASH）这类不经命令层、直接写盘的路径，以及 cli_secure_boot_cmd()（控制 DT 的 /config 有 bootsecure 时，common/cli.c 直接调 bootcmd 那条命令的函数）不在此列。
 */

#include <blk.h>
#include <command.h>
#include <div64.h>
#include <lmb.h>
#include <malloc.h>
#include <mapmem.h>
#include <mmc.h>
#include <part.h>
#include <sparse_format.h>
#include <usb.h>
#include <vsprintf.h>
#include <asm/byteorder.h>
#include <asm/global_data.h>
#include <asm/unaligned.h>
#include <linux/arm-smccc.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <u-boot/crc.h>

DECLARE_GLOBAL_DATA_PTR;

/* 清单正文：按行追加，展示与 redline yes 时的复核比较都用它 */
struct rl_text {
	char *buf;
	size_t len, size;
	bool oom;
};

static void rl_puts(struct rl_text *t, const char *s)
{
	size_t n = strlen(s), size;
	char *p;

	if (t->oom)
		return;
	if (t->len + n + 1 > t->size) {
		size = max(t->size * 2, t->len + n + 1 + 256);
		p = realloc(t->buf, size);
		if (!p) {
			t->oom = true;
			return;
		}
		t->buf = p;
		t->size = size;
	}
	memcpy(t->buf + t->len, s, n + 1);
	t->len += n;
}

static __printf(2, 3) void rl_printf(struct rl_text *t, const char *fmt, ...)
{
	char line[256];
	va_list args;

	va_start(args, fmt);
	vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	rl_puts(t, line);
}

static const char *rl_arg(int argc, char *const argv[], int i)
{
	return i < argc ? argv[i] : "?";
}

/*
 * 与 find_cmd_tbl() 相同的子命令解析：只比 '.' 之前的部分，全名优先，否则要求恰好一个前缀匹配
 * Return: 解析出的子命令名；歧义、未知或缺参数时为 NULL
 */
static const char *rl_resolve(const char *arg, const char *const *names, int n)
{
	const char *hit = NULL, *dot;
	int i, len, found = 0;

	if (!arg)
		return NULL;
	dot = strchr(arg, '.');
	len = dot ? dot - arg : strlen(arg);
	for (i = 0; i < n; i++) {
		if (strncmp(arg, names[i], len))
			continue;
		if (len == strlen(names[i]))
			return names[i];
		hit = names[i];
		found++;
	}

	return found == 1 ? hit : NULL;
}

static bool rl_in(const char *name, const char *const *names, int n)
{
	int i;

	for (i = 0; i < n && name; i++)
		if (!strcmp(name, names[i]))
			return true;

	return false;
}

/* [@addr, @addr + @len) 整段在内存（gd->ram_base 起 gd->ram_size）里 */
static bool rl_in_ram(ulong addr, u64 len)
{
	return addr >= gd->ram_base && len <= gd->ram_size &&
	       addr - gd->ram_base <= gd->ram_size - len;
}

/* [@addr, @addr + @len)（已知在内存里）与 lmb 里带 no-map 的保留区重叠 */
static bool rl_nomap(ulong addr, u64 len)
{
	const struct lmb_region *r;

	if (!IS_ENABLED(CONFIG_LMB))
		return false;
	alist_for_each(r, &lmb_get()->used_mem)
		if ((r->flags & LMB_NOMAP) && r->base < addr + len &&
		    addr < r->base + r->size)
			return true;

	return false;
}

/*
 * 清单能不能由 CPU 读 [@addr, @addr + @len)：不全在内存里的，命令自己会出错，清单不该先替它读到总线错误；落在 no-map 保留区里的是安全世界或固件的内存，读了可能挂死
 * Return: NULL 可以读；否则是不读的原因
 */
static const char *rl_unreadable(ulong addr, u64 len)
{
	if (!rl_in_ram(addr, len))
		return "not entirely inside RAM";
	if (rl_nomap(addr, len))
		return "inside a no-map reserved-memory region";

	return NULL;
}

/* 写入的源数据 [@addr, @addr + @len)：与命令读的是同一段，crc32 进清单，redline yes 前重算，展示后改了源数据就与展示过的清单不同 */
static void rl_source(struct rl_text *t, ulong addr, u64 len)
{
	const char *why = rl_unreadable(addr, len);
	const u8 *p;
	u64 done;
	u32 crc = 0;
	uint n;

	if (why) {
		rl_printf(t, "  source  : 0x%llx bytes at 0x%lx (%s: not read, not checksummed)\n",
			  len, addr, why);
		return;
	}
	p = map_sysmem(addr, len);
	for (done = 0; done < len; done += n) {
		n = min_t(u64, len - done, SZ_1G);
		crc = crc32_wd(crc, p + done, n, SZ_64K);
	}
	unmap_sysmem(p);
	rl_printf(t, "  source  : 0x%llx bytes at 0x%lx, crc32 0x%08x\n", len, addr,
		  crc);
}

static void rl_device(struct rl_text *t, struct blk_desc *desc)
{
	static const char *const hwparts[] = {
		"user area", "boot0", "boot1", "rpmb", "gp1", "gp2", "gp3", "gp4",
	};
	char hw[32] = "";

	if (desc->uclass_id == UCLASS_MMC)
		snprintf(hw, sizeof(hw), ", hwpart %d (%s)", desc->hwpart,
			 desc->hwpart < ARRAY_SIZE(hwparts) ?
			 hwparts[desc->hwpart] : "?");
	rl_printf(t, "  device  : %s %d%s, %lu-byte blocks, 0x" LBAF " blocks\n",
		  blk_get_uclass_name(desc->uclass_id), desc->devnum, hw,
		  desc->blksz, desc->lba);
}

static bool rl_overlap(struct disk_partition *info, lbaint_t start,
		       lbaint_t end)
{
	return info->start < end && start < info->start + info->size;
}

/*
 * [start, end) 与设备上各分区的重叠；硬件分区不是 user 区时分区表不覆盖它。
 * @sure_end 之前是一定会写的部分，[@sure_end, @end) 只是可能写到（gzwrite 写到流结束）：只与后者重叠的分区单独注明
 */
static void rl_overlaps(struct rl_text *t, struct blk_desc *desc,
			lbaint_t start, lbaint_t end, lbaint_t sure_end)
{
	struct disk_partition info;
	int p, total = 0, hit = 0;

	if (desc->hwpart) {
		rl_printf(t, "  overlaps: none (hwpart %d is outside the partition table)\n",
			  desc->hwpart);
		return;
	}
	if (desc->part_type == PART_TYPE_UNKNOWN) {
		rl_puts(t, "  overlaps: no partition table recognised on the device\n");
		return;
	}
	for (p = 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		if (part_get_info(desc, p, &info))
			continue;
		total++;
		if (rl_overlap(&info, start, end))
			hit++;
	}
	if (!hit) {
		rl_printf(t, "  overlaps: none of the %d partitions\n", total);
		return;
	}
	rl_printf(t, "  overlaps: %d of the %d partitions\n", hit, total);
	for (p = 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		if (part_get_info(desc, p, &info) || !rl_overlap(&info, start, end))
			continue;
		rl_printf(t, "    #%d %s: blocks 0x" LBAF "..0x" LBAF "%s\n", p,
			  (char *)info.name, info.start, info.start + info.size,
			  rl_overlap(&info, start, sure_end) ? "" :
			  " (only if the write runs past the declared size)");
	}
}

/* 块设备 @desc 上 [@start, @start + @count) 一段：设备、起止块、字节范围、重叠的分区 */
static void rl_range(struct rl_text *t, struct blk_desc *desc, lbaint_t start,
		     lbaint_t count)
{
	rl_device(t, desc);
	rl_printf(t, "  blocks  : 0x" LBAF "..0x" LBAF " (0x" LBAF " blocks)\n",
		  start, start + count, count);
	rl_printf(t, "  bytes   : 0x%llx..0x%llx\n", (u64)start * desc->blksz,
		  (u64)(start + count) * desc->blksz);
	if (start + count > desc->lba)
		rl_puts(t, "  note    : the range runs past the end of the device\n");
	rl_overlaps(t, desc, start, start + count, start + count);
}

/* 只给设备、不给范围的动作（改 eMMC 配置）：@devnum 号 MMC 设备 */
static void rl_mmc_whole(struct rl_text *t, int devnum)
{
	struct blk_desc *desc;

	if (devnum < 0 || blk_get_desc(UCLASS_MMC, devnum, &desc))
		rl_printf(t, "  device  : mmc %d (not available now)\n", devnum);
	else
		rl_device(t, desc);
}

static void rl_unclassified(struct rl_text *t, const char *cmd,
			    const char *arg)
{
	rl_printf(t, "  action  : '%s %s' does not resolve to a known read-only subcommand (held: it cannot be classified)\n",
		  cmd, arg ? arg : "");
}

/* ---- mmc：子命令表按前缀解析（cmd/mmc.c 的 cmd_mmc[]） ---- */
static const char *const mmc_subs[] = {
	"info", "read", "wp", "write", "erase", "swrite", "rescan", "part",
	"dev", "list", "hwpartition", "bootbus", "bootpart-resize", "partconf",
	"rst-function", "rpmb", "setdsr", "bkops-enable", "bkops", "reg",
};

/* 只读（含只改主机侧或易失状态的 setdsr、rescan、dev）：放行 */
static const char *const mmc_readonly[] = {
	"info", "read", "rescan", "part", "dev", "list", "setdsr", "reg",
};

static const char *const mmc_rpmb_subs[] = { "key", "read", "write", "counter" };

/* mmc 命令下一条子命令将作用的块设备（mmc dev 选定的设备与硬件分区）；取不到时在清单里注明并返回 NULL */
static struct blk_desc *rl_mmc_desc(struct rl_text *t)
{
	struct blk_desc *desc;
	int dev = mmc_cmd_get_dev();

	if (dev < 0 || blk_get_desc(UCLASS_MMC, dev, &desc)) {
		rl_printf(t, "  device  : mmc %d (not available now)\n", dev);
		return NULL;
	}

	return desc;
}

static void rl_mmc_current(struct rl_text *t)
{
	rl_mmc_whole(t, mmc_cmd_get_dev());
}

/*
 * mmc erase 的实际范围：起点或块数不对齐擦除组、卡又不支持 TRIM 时，卡按整组擦（drivers/mmc/mmc_write.c 的 mmc_berase()），范围向两头扩到组边界
 */
static void rl_mmc_erase(struct rl_text *t, struct blk_desc *desc, u32 blk,
			 u32 cnt)
{
	struct mmc *mmc = find_mmc_device(desc->devnum);
	u64 start = blk, end = (u64)blk + cnt;
	u32 grp;

	if (!mmc || !mmc->has_init || !mmc->erase_grp_size) {
		rl_puts(t, "  note    : erase group unknown (EXT_CSD not read): the card may erase up to its erase-group boundaries around this range\n");
	} else if ((blk % mmc->erase_grp_size || cnt % mmc->erase_grp_size) &&
		   !mmc->can_trim) {
		grp = mmc->erase_grp_size;
		start = blk - blk % grp;
		end = lldiv(end + grp - 1, grp) * grp;
		rl_printf(t, "  note    : blocks 0x%x..0x%llx are not aligned to the 0x%x-block erase group and the card cannot TRIM: whole groups are erased\n",
			  blk, (u64)blk + cnt, grp);
	}
	rl_range(t, desc, start, end - start);
}

/* mmc swrite 的 sparse 块：do_mmc_sparse_write() 把 sparse.blksz 固定为 512 */
#define RL_SPARSE_BLKSZ		512

/*
 * mmc swrite <addr> <blk> 的源数据与盘上范围，照 lib/image-sparse.c 的 write_sparse_image() 逐个 chunk 走一遍：
 * 文件头之后先跳 file_hdr_sz，比 sparse_header_t 长时多出的部分再跳一次（上游如此）；每个 chunk 先读 chunk_header_t，chunk_hdr_sz 更长时多出的部分跳过；RAW 带 blk_sz × chunk_sz 字节数据，FILL 带 4 字节，CRC32 跳过 blk_sz × chunk_sz 字节，DONT_CARE 不带数据；RAW、FILL、DONT_CARE 让盘上位置前进 blk_sz × chunk_sz / 512 块。
 * 命令先查魔数与主版本（is_sparse_image()）、blk_sz 是 512 的倍数，不符时一块都不写；RAW、FILL、CRC32 的 total_sz 与应有长度不符、类型未知、RAW/FILL 越过盘尾（命令给的范围是起点到盘尾）时，在该 chunk 报错返回，前面的 chunk 已经写了；走完后各 chunk 的块数之和与头里的 total_blks 不符时，写完全部再报错。
 */
static void rl_swrite(struct rl_text *t, struct blk_desc *desc, ulong addr,
		      u32 start)
{
	const sparse_header_t *sh;
	const chunk_header_t *ch;
	const char *why, *bad = NULL;
	u32 chunks, i, total = 0, total_blks, blk_sz, type, csz, tsz;
	u64 off, blk = start, data;
	u16 fhdr, chdr;

	why = rl_unreadable(addr, sizeof(*sh));
	if (why) {
		rl_printf(t, "  source  : sparse header at 0x%lx (%s: not read)\n",
			  addr, why);
		rl_puts(t, "  note    : the image cannot be parsed: what mmc swrite writes is unknown, the range is shown up to the end of the device\n");
		rl_range(t, desc, start, desc->lba > start ? desc->lba - start : 0);
		return;
	}
	sh = map_sysmem(addr, sizeof(*sh));
	fhdr = le16_to_cpu(sh->file_hdr_sz);
	chdr = le16_to_cpu(sh->chunk_hdr_sz);
	blk_sz = le32_to_cpu(sh->blk_sz);
	total_blks = le32_to_cpu(sh->total_blks);
	chunks = le32_to_cpu(sh->total_chunks);
	if (le32_to_cpu(sh->magic) != SPARSE_HEADER_MAGIC ||
	    le16_to_cpu(sh->major_version) != 1)
		bad = "  note    : no Android sparse image at the source (magic or major version): mmc swrite refuses before writing anything\n";
	else if (blk_sz % RL_SPARSE_BLKSZ)
		bad = "  note    : the sparse block size is not a multiple of 512: mmc swrite refuses before writing anything\n";
	unmap_sysmem(sh);
	if (bad) {
		rl_puts(t, bad);
		rl_mmc_whole(t, desc->devnum);
		return;
	}

	off = fhdr + (fhdr > sizeof(*sh) ? fhdr - sizeof(*sh) : 0);
	for (i = 0; i < chunks; i++) {
		why = rl_unreadable(addr + off, sizeof(*ch));
		if (why) {
			rl_printf(t, "  source  : chunk %u header at 0x%llx (%s: not read)\n",
				  i + 1, addr + off, why);
			rl_puts(t, "  note    : the image cannot be parsed: what mmc swrite writes is unknown, the range is shown up to the end of the device\n");
			rl_range(t, desc, start, desc->lba > start ? desc->lba - start : 0);
			return;
		}
		ch = map_sysmem(addr + off, sizeof(*ch));
		type = le16_to_cpu(ch->chunk_type);
		csz = le32_to_cpu(ch->chunk_sz);
		tsz = le32_to_cpu(ch->total_sz);
		unmap_sysmem(ch);
		off += max_t(u64, chdr, sizeof(*ch));
		data = (u64)blk_sz * csz;
		if ((type == CHUNK_TYPE_RAW && tsz != chdr + data) ||
		    ((type == CHUNK_TYPE_FILL || type == CHUNK_TYPE_CRC32) &&
		     tsz != chdr + sizeof(u32))) {
			bad = "total_sz does not match the chunk type";
		} else if (type != CHUNK_TYPE_RAW && type != CHUNK_TYPE_FILL &&
			   type != CHUNK_TYPE_DONT_CARE && type != CHUNK_TYPE_CRC32) {
			bad = "unknown chunk type";
		} else if ((type == CHUNK_TYPE_RAW || type == CHUNK_TYPE_FILL) &&
			   blk + data / RL_SPARSE_BLKSZ > desc->lba) {
			bad = "it would run past the end of the device";
		}
		if (bad)
			break;
		if (type == CHUNK_TYPE_RAW || type == CHUNK_TYPE_CRC32)
			off += data;
		else if (type == CHUNK_TYPE_FILL)
			off += sizeof(u32);
		if (type != CHUNK_TYPE_CRC32)
			blk += data / RL_SPARSE_BLKSZ;
		/* FILL 按 DIV_ROUND_UP(blk_sz × chunk_sz, blk_sz) 计，blk_sz 为 0 时 arm64 的除法得 0 */
		total += type == CHUNK_TYPE_FILL && !blk_sz ? 0 : csz;
	}
	rl_source(t, addr, off);
	if (bad)
		rl_printf(t, "  note    : chunk %u (type 0x%x) is rejected (%s): mmc swrite writes the chunks before it, then fails\n",
			  i + 1, type, bad);
	else if (total != total_blks)
		rl_printf(t, "  note    : the chunks add up to 0x%x sparse blocks, the header declares 0x%x: mmc swrite writes them all, then reports failure\n",
			  total, total_blks);
	rl_range(t, desc, start, blk - start);
}

static bool rl_mmc(struct rl_text *t, const char *name, int argc,
		   char *const argv[])
{
	const char *sub, *sub2;
	struct blk_desc *desc;
	struct disk_partition info;
	int i, part;

	if (!IS_ENABLED(CONFIG_CMD_MMC) || argc < 2)
		return false;
	sub = rl_resolve(argv[1], mmc_subs, ARRAY_SIZE(mmc_subs));
	if (rl_in(sub, mmc_readonly, ARRAY_SIZE(mmc_readonly)))
		return false;
	if (!sub) {
		rl_unclassified(t, name, argv[1]);
		rl_mmc_current(t);
	} else if (!strcmp(sub, "write")) {
		rl_printf(t, "  action  : MMC block write from %s\n",
			  rl_arg(argc, argv, 2));
		desc = rl_mmc_desc(t);
		if (desc) {
			/* do_mmc_write() 把块号与块数截成 u32，清单按同样的值算 */
			rl_source(t, hextoul(rl_arg(argc, argv, 2), NULL),
				  (u64)(u32)hextoul(rl_arg(argc, argv, 4), NULL) *
				  desc->blksz);
			rl_range(t, desc, (u32)hextoul(rl_arg(argc, argv, 3), NULL),
				 (u32)hextoul(rl_arg(argc, argv, 4), NULL));
		}
	} else if (!strcmp(sub, "erase")) {
		rl_puts(t, "  action  : MMC erase\n");
		desc = rl_mmc_desc(t);
		if (desc && argc != 3)
			rl_mmc_erase(t, desc, hextoul(rl_arg(argc, argv, 2), NULL),
				     hextoul(rl_arg(argc, argv, 3), NULL));
		else if (desc && part_get_info_by_name(desc, argv[2], &info) < 0)
			rl_printf(t, "  device  : mmc %d has no partition '%s' now\n",
				  desc->devnum, argv[2]);
		else if (desc)	/* do_mmc_erase() 同样把分区起点与大小截成 u32 */
			rl_mmc_erase(t, desc, info.start, info.size);
	} else if (!strcmp(sub, "swrite")) {
		rl_printf(t, "  action  : MMC sparse image write from %s\n",
			  rl_arg(argc, argv, 2));
		desc = rl_mmc_desc(t);
		if (argc != 4)
			rl_puts(t, "  note    : mmc swrite takes exactly <addr> <blk> (it will print its usage)\n");
		else if (desc)	/* do_mmc_sparse_write() 把起始块截成 u32 */
			rl_swrite(t, desc, hextoul(argv[2], NULL),
				  hextoul(argv[3], NULL));
	} else if (!strcmp(sub, "wp")) {
		/* do_mmc_boot_wp()：带一个参数时取 (int)dectoul()，0 只保护 boot0（不是数字的写法也得 0）、1 只保护 boot1，其余值经 mmc_boot_wp_single_partition() 保护全部；不带参数保护全部；多于一个参数只打印用法 */
		part = argc == 3 ? (int)dectoul(argv[2], NULL) : -1;
		rl_printf(t, "  action  : power-on write protection of %s\n",
			  argc > 3 ? "nothing (too many arguments: mmc prints its usage)" :
			  part == 0 ? "boot partition 0" :
			  part == 1 ? "boot partition 1" : "all boot partitions");
		rl_mmc_current(t);
		rl_puts(t, "  note    : software cannot lift it; it lasts until the next power cycle\n");
	} else if (!strcmp(sub, "hwpartition")) {
		for (i = 2; i < argc; i++)
			if (!strcmp(argv[i], "set") || !strcmp(argv[i], "complete"))
				break;
		if (i == argc)
			return false;	/* 只有 check 模式：只校验、不写 */
		rl_puts(t, "  action  : eMMC hardware partitioning (EXT_CSD)\n");
		rl_mmc_current(t);
		for (i = 2; i < argc; i++)
			if (!strcmp(argv[i], "complete"))
				rl_puts(t, "  IRREVERSIBLE: 'complete' sets PARTITION_SETTING_COMPLETED; the layout can never be changed again\n");
	} else if (!strcmp(sub, "rpmb")) {
		sub2 = rl_resolve(argc > 2 ? argv[2] : NULL, mmc_rpmb_subs,
				  ARRAY_SIZE(mmc_rpmb_subs));
		if (sub2 && (!strcmp(sub2, "read") || !strcmp(sub2, "counter")))
			return false;
		if (sub2 && !strcmp(sub2, "key")) {
			rl_puts(t, "  action  : program the RPMB authentication key\n");
			rl_puts(t, "  IRREVERSIBLE: the RPMB key can be programmed only once\n");
		} else if (sub2) {
			rl_puts(t, "  action  : RPMB authenticated write\n");
		} else {
			rl_unclassified(t, "mmc rpmb", argc > 2 ? argv[2] : NULL);
		}
		rl_mmc_current(t);
	} else if (!strcmp(sub, "partconf")) {
		if (argc == 3 || argc == 4)
			return false;	/* 只显示（可存进变量） */
		rl_puts(t, "  action  : write PARTITION_CONFIG (boot partition enable, boot ack, access)\n");
		rl_mmc_whole(t, dectoul(rl_arg(argc, argv, 2), NULL));
	} else {
		/* 带 <dev> 参数改 EXT_CSD 的子命令 */
		rl_printf(t, "  action  : eMMC configuration change '%s'\n", sub);
		rl_mmc_whole(t, dectoul(rl_arg(argc, argv, 2), NULL));
		if (!strcmp(sub, "rst-function") || !strcmp(sub, "bkops-enable") ||
		    (!strcmp(sub, "bkops") && argc > 4 &&
		     !strcmp(argv[3], "manual") && !strcmp(argv[4], "enable")))
			rl_puts(t, "  IRREVERSIBLE: this EXT_CSD field is write-once\n");
		else if (!strcmp(sub, "bootpart-resize"))
			rl_puts(t, "  IRREVERSIBLE: boot/RPMB partition sizes are changed by a one-time vendor sequence\n");
	}

	return true;
}

/* ---- usb：块读写落到 blk_common_cmd()，write/erase 按全名比较 ---- */
static bool rl_usb(struct rl_text *t, const char *name, int argc,
		   char *const argv[])
{
	struct blk_desc *desc;
	bool erase;
	int dev;

	if (argc < 2 || (strcmp(argv[1], "write") && strcmp(argv[1], "erase")))
		return false;
	erase = !strcmp(argv[1], "erase");
	if (erase)
		rl_puts(t, "  action  : USB storage erase\n");
	else
		rl_printf(t, "  action  : USB storage block write from %s\n",
			  rl_arg(argc, argv, 2));
	dev = IS_ENABLED(CONFIG_USB_STORAGE) ? usb_stor_get_curr_dev() : -1;
	if (dev < 0 || blk_get_desc(UCLASS_USB, dev, &desc)) {
		rl_printf(t, "  device  : usb %d (no current USB storage device)\n",
			  dev);
		return true;
	}
	if (!erase)
		rl_source(t, hextoul(rl_arg(argc, argv, 2), NULL),
			  (u64)hextoul(rl_arg(argc, argv, 4), NULL) * desc->blksz);
	rl_range(t, desc, hextoul(rl_arg(argc, argv, erase ? 2 : 3), NULL),
		 hextoul(rl_arg(argc, argv, erase ? 3 : 4), NULL));

	return true;
}

/*
 * ---- gzwrite <interface> <dev> <addr> <length> [wbuf [offs [outsize]]] ----
 * gzwrite() 解压写盘直到 deflate 流结束，大小只在写完后与声明值核对（lib/gunzip.c）：声明的大小（outsize，没给就取 gzip 尾部的 ISIZE）只是“至少写到”，流比声明长就一直写下去，最远到盘尾
 */
static bool rl_gzwrite(struct rl_text *t, const char *name, int argc,
		       char *const argv[])
{
	struct blk_desc *desc;
	ulong addr = hextoul(rl_arg(argc, argv, 3), NULL);
	ulong len = hextoul(rl_arg(argc, argv, 4), NULL);
	u64 offs = argc > 6 ? simple_strtoull(argv[6], NULL, 16) : 0;
	u64 size = argc > 7 ? simple_strtoull(argv[7], NULL, 16) : 0;
	lbaint_t start, sure, end;
	u32 isize = 0;
	u8 *p;

	rl_printf(t, "  action  : decompress gzip data at %s and write it to the device at byte 0x%llx\n",
		  rl_arg(argc, argv, 3), offs);
	if (argc < 5 || blk_get_device_by_str(argv[1], argv[2], &desc) < 0) {
		rl_printf(t, "  device  : %s %s (not available now)\n",
			  rl_arg(argc, argv, 1), rl_arg(argc, argv, 2));
		return true;
	}
	rl_source(t, addr, len);
	if (len >= 18 && !rl_unreadable(addr, len)) {
		p = map_sysmem(addr + len - 4, 4);
		isize = get_unaligned_le32(p);
		unmap_sysmem(p);
	}
	if (size) {
		rl_printf(t, "  declared: 0x%llx bytes (outsize)\n", size);
		if ((u32)size != isize)
			rl_printf(t, "  note    : outsize does not match the gzip trailer (ISIZE 0x%x); gzwrite will refuse\n",
				  isize);
	} else {
		size = isize;
		rl_printf(t, "  declared: 0x%llx bytes (gzip trailer ISIZE, modulo 4 GiB)\n",
			  size);
	}
	rl_puts(t, "  note    : gzwrite writes until the deflate stream ends and checks the size only afterwards: the write may run past the declared size, up to the end of the device\n");
	start = lldiv(offs, desc->blksz);
	sure = lldiv(offs + size + desc->blksz - 1, desc->blksz);
	end = max(sure, desc->lba);
	if (sure > desc->lba)
		rl_puts(t, "  note    : the declared size runs past the end of the device (gzwrite will refuse)\n");
	rl_device(t, desc);
	rl_printf(t, "  blocks  : 0x" LBAF "..0x" LBAF " at least, possibly up to 0x" LBAF "\n",
		  start, sure, end);
	rl_printf(t, "  bytes   : 0x%llx..0x%llx at least, possibly up to 0x%llx\n",
		  (u64)start * desc->blksz, (u64)sure * desc->blksz,
		  (u64)end * desc->blksz);
	rl_overlaps(t, desc, start, end, sure);

	return true;
}

/* ---- ums <USB_controller> [<devtype>] <dev[:part]>[,<dev[:part]>...] ---- */
static bool rl_ums(struct rl_text *t, const char *name, int argc,
		   char *const argv[])
{
	const char *devtype = argc >= 4 ? argv[2] : "mmc";
	struct disk_partition info;
	struct blk_desc *desc;
	char *list, *s, *tok;
	int part;

	rl_puts(t, "  action  : USB mass storage: the USB host gets raw read/write access until the command is stopped\n");
	if (argc < 3)
		return true;
	list = strdup(argc >= 4 ? argv[3] : argv[2]);
	if (!list) {
		t->oom = true;
		return true;
	}
	for (s = list; (tok = strsep(&s, ","));) {
		/* 与 ums_init() 相同：不带 ':' 的写法暴露整个设备 */
		part = part_get_info_by_dev_and_name_or_num(devtype, tok, &desc,
							    &info, 1);
		if (part < 0) {
			rl_printf(t, "  device  : %s %s (not available now)\n",
				  devtype, tok);
		} else if (!strchr(tok, ':') || !part) {
			rl_printf(t, "  exposed : WHOLE DEVICE %s %s\n", devtype, tok);
			rl_range(t, desc, 0, desc->lba);
		} else {
			rl_printf(t, "  exposed : partition %d '%s' of %s %s\n", part,
				  (char *)info.name, devtype, tok);
			rl_range(t, desc, info.start, info.size);
		}
	}
	free(list);

	return true;
}

/* ---- 文件系统写：<interface> <dev[.hwpart][:part]>|<dev>#<分区名> ...；设备与分区用 fs_set_blk_dev() 调的同一个函数 part_get_info_by_dev_and_name_or_num() 解析 ---- */
static bool rl_fs(struct rl_text *t, const char *name, int argc,
		  char *const argv[])
{
	struct disk_partition info;
	struct blk_desc *desc = NULL;
	int part;

	if (!strcmp(name, "rm") || !strcmp(name, "fatrm")) {
		rl_printf(t, "  action  : filesystem write: delete %s\n",
			  rl_arg(argc, argv, 3));
	} else if (!strcmp(name, "mkdir") || !strcmp(name, "fatmkdir")) {
		rl_printf(t, "  action  : filesystem write: create directory %s\n",
			  rl_arg(argc, argv, 3));
	} else if (!strcmp(name, "mv")) {
		rl_printf(t, "  action  : filesystem write: rename %s to %s\n",
			  rl_arg(argc, argv, 3), rl_arg(argc, argv, 4));
	} else if (!strcmp(name, "ln")) {
		rl_printf(t, "  action  : filesystem write: symlink %s -> %s\n",
			  rl_arg(argc, argv, 4), rl_arg(argc, argv, 3));
	} else {
		/* save、fatwrite、ext4write 都落到 do_save()：<addr> <filename> <bytes> [pos]，数字按十六进制 */
		rl_printf(t, "  action  : filesystem write: file %s, %s bytes from %s\n",
			  rl_arg(argc, argv, 4), rl_arg(argc, argv, 5),
			  rl_arg(argc, argv, 3));
		if (argc >= 6)
			rl_source(t, hextoul(argv[3], NULL), hextoul(argv[5], NULL));
	}
	part = argc < 3 ? -1 :
	       part_get_info_by_dev_and_name_or_num(argv[1], argv[2], &desc,
						    &info, 1);
	if (part < 0 || !desc) {
		rl_printf(t, "  device  : %s %s (not available now)\n",
			  rl_arg(argc, argv, 1), rl_arg(argc, argv, 2));
	} else if (!part) {
		rl_puts(t, "  scope   : whole device (no partition table)\n");
		rl_range(t, desc, 0, desc->lba);
	} else {
		rl_printf(t, "  scope   : anywhere inside partition %d '%s'\n", part,
			  (char *)info.name);
		rl_range(t, desc, info.start, info.size);
	}

	return true;
}

/* ---- sm（Amlogic 安全监控）：子命令表按前缀解析（cmd/meson/sm.c 的 cmd_sm_sub[]） ---- */
static const char *const sm_subs[] = {
	"serial", "reboot_reason", "efuseread", "efusewrite", "efusedump",
};

/* 安全监控输入共享缓冲的大小：meson_sm_call_write() 拒绝更长的写（drivers/sm/meson-sm.c 的 meson_sm_gxbb_data.shmem_size） */
#define RL_SM_SHMEM_SIZE	SZ_4K

/* 要烧进 eFuse 偏移 @offset 起的字节（@addr 起 @size 个，调用者已确认可读）逐个列出：清单因此与数据绑定，展示后改了数据，redline yes 时清单不同、不执行 */
static void rl_efuse_bytes(struct rl_text *t, ulong offset, ulong addr,
			   ulong size)
{
	char hex[16 * 3 + 1];
	const u8 *p;
	ulong i, j;

	p = map_sysmem(addr, size);
	for (i = 0; i < size; i += 16) {
		for (j = 0; j < 16 && i + j < size; j++)
			snprintf(hex + j * 3, 4, " %02x", p[i + j]);
		rl_printf(t, "  data    : eFuse 0x%03lx:%s\n", offset + i, hex);
	}
	unmap_sysmem(p);
}

static bool rl_sm(struct rl_text *t, const char *name, int argc,
		  char *const argv[])
{
	ulong offset, size, addr;
	const char *sub, *why;

	if (argc < 2)
		return false;
	sub = rl_resolve(argv[1], sm_subs, ARRAY_SIZE(sm_subs));
	if (sub && strcmp(sub, "efusewrite"))
		return false;
	if (!sub) {
		rl_unclassified(t, name, argv[1]);
		return true;
	}
	/* 与 do_efuse_write() 相同：三个数都按 simple_strtoul(…, 0) 解析（0x 前缀十六进制，否则十进制，0 开头八进制） */
	offset = simple_strtoul(rl_arg(argc, argv, 2), NULL, 0);
	size = simple_strtoul(rl_arg(argc, argv, 3), NULL, 0);
	addr = simple_strtoul(rl_arg(argc, argv, 4), NULL, 0);
	rl_printf(t, "  action  : eFuse write: 0x%lx bytes at eFuse offset 0x%lx from 0x%lx\n",
		  size, offset, addr);
	if (argc < 5) {
		rl_puts(t, "  note    : missing arguments (the command will print its usage)\n");
	} else if (size > RL_SM_SHMEM_SIZE) {
		rl_puts(t, "  note    : more than the 4 KiB secure monitor input buffer: the call is refused\n");
	} else if ((why = rl_unreadable(addr, size))) {
		rl_printf(t, "  data    : the source is %s (not shown)\n", why);
	} else {
		rl_efuse_bytes(t, offset, addr, size);
	}
	rl_puts(t, "  IRREVERSIBLE: eFuse bits are one-time programmable\n");

	return true;
}

/*
 * ---- smc/hvc <fid> [arg1 ... arg6] [id]：cmd/smccc.c 不过滤 fid，按 fid 白名单放行 ----
 * fid 与命令相同按 hextoul() 解析、按整个 unsigned long 比较：SMCCC 规定 function ID 在 W0（厂商 BL31 所本的 TF-A v1.3 psci_smc_handler() 的 smc_fid 是 uint32_t），高 32 位不为 0 的写法不算白名单项，清单按 W0 叫出名字
 */
struct rl_fid {
	u32 fid;
	const char *what;
};

/*
 * 放行：0x84000009 PSCI SYSTEM_RESET（Amlogic BL31 把 x1 当重启原因；m12.env 的 m12_boot_reset 与开发循环回线刷态用）、0x82000049 Amlogic 设重启原因（m12.env 的 m12_android_reason）；它们只改重启原因与复位，不碰存储。
 * 以及只返回信息的查询：PSCI_VERSION 与 PSCI_FEATURES（TF-A v1.3 psci_main.c 里分别返回常量与 psci_caps 位）、SMCCC_VERSION 与 SMCCC_ARCH_FEATURES（SMCCC 的发现调用，drivers/firmware/psci.c 探测时就这样用；没有该服务的固件返回 NOT_SUPPORTED）。
 * 以及 Amlogic 厂商 BL31 安全存储的只读调用，供手工做 RAM 级实验（在控制台上 mw 填输入共享缓冲、smc、md 看输出缓冲）：取四块共享缓冲的地址与块大小（返回常量或读全局）、QUERY/READ/TELL/VERIFY/STATUS/LIST（只读 key 表，只往输出共享缓冲写）、GET_ENCTYPE（经 SCP 邮箱读 eFuse，只读）、VERSION；依据 M12 厂商 BL31 v1.3:079690f10 的反汇编，它们没有计数、锁定等副作用。改存储与 BL31 运行态的 WRITE 0x82000062、REMOVE 0x82000068、NOTIFY 0x82000066、NOTIFY_EX 0x82000069、SET_ENCTYPE 0x8200006a、SET_STORAGE_INFO 0x82000028 不放行
 */
static const struct rl_fid smc_allowed[] = {
	{ 0x84000009, "PSCI SYSTEM_RESET" },
	{ 0x82000049, "Amlogic SiP: set reboot reason" },
	{ 0x84000000, "PSCI_VERSION" },
	{ 0x8400000a, "PSCI_FEATURES" },
	{ 0x80000000, "SMCCC_VERSION" },
	{ 0x80000001, "SMCCC_ARCH_FEATURES" },
	{ 0x82000023, "Amlogic secure storage: input buffer address" },
	{ 0x82000024, "Amlogic secure storage: output buffer address" },
	{ 0x82000025, "Amlogic secure storage: block buffer address" },
	{ 0x82000026, "Amlogic secure storage: message area address" },
	{ 0x82000027, "Amlogic secure storage: block size" },
	{ 0x82000060, "Amlogic secure storage: query" },
	{ 0x82000061, "Amlogic secure storage: read" },
	{ 0x82000063, "Amlogic secure storage: tell (length)" },
	{ 0x82000064, "Amlogic secure storage: verify (SHA-256)" },
	{ 0x82000065, "Amlogic secure storage: status (attributes)" },
	{ 0x82000067, "Amlogic secure storage: list" },
	{ 0x8200006b, "Amlogic secure storage: get encryption type" },
	{ 0x8200006c, "Amlogic secure storage: version" },
};

/* 拦下时能叫出名字的 fid：Amlogic SiP 调用取自 drivers/sm/meson-sm.c 的 meson_sm_gxbb_data 与 M12 厂商 BL31 的反汇编，PSCI 取自 include/linux/psci.h */
static const struct rl_fid smc_known[] = {
	{ 0x82000020, "Amlogic SiP: get the shared input buffer address" },
	{ 0x82000021, "Amlogic SiP: get the shared output buffer address" },
	{ 0x82000030, "Amlogic SiP: eFuse read into the shared output buffer" },
	{ 0x82000031, "Amlogic SiP: eFuse write from the shared input buffer" },
	{ 0x82000044, "Amlogic SiP: get chip ID" },
	{ 0x82000093, "Amlogic SiP: power domain set" },
	{ 0x82000028, "Amlogic SiP: set storage info (AO register)" },
	{ 0x82000062, "Amlogic secure storage: write (re-encrypts the block buffer)" },
	{ 0x82000066, "Amlogic secure storage: notify (rebuilds the key table)" },
	{ 0x82000068, "Amlogic secure storage: remove (re-encrypts the block buffer)" },
	{ 0x82000069, "Amlogic secure storage: notify_ex (rebuilds the key table)" },
	{ 0x8200006a, "Amlogic secure storage: set encryption type" },
	{ 0x84000008, "PSCI SYSTEM_OFF" },
	{ 0x84000012, "PSCI SYSTEM_RESET2" },
	{ 0xc4000012, "PSCI SYSTEM_RESET2 (64-bit)" },
};

#define RL_SMC_SHMEM_IN		0x82000020
#define RL_SMC_EFUSE_WRITE	0x82000031

/*
 * Amlogic 安全监控输入共享缓冲的地址：smc 0x82000020 只返回地址（drivers/sm/meson-sm.c 的 meson_sm_probe() 同样这样取）；取不到时为 0。
 * 弱函数：sandbox 没有安全监控，测试给出一块内存当缓冲
 */
__weak ulong redline_sm_input_buffer(void)
{
#if IS_ENABLED(CONFIG_ARCH_MESON) && IS_ENABLED(CONFIG_ARM_SMCCC)
	struct arm_smccc_res res;

	arm_smccc_smc(RL_SMC_SHMEM_IN, 0, 0, 0, 0, 0, 0, 0, &res);

	return res.a0;
#else
	return 0;
#endif
}

/*
 * smc 0x82000031 <x1> <x2>：安全监控从输入共享缓冲的起点取 x2 字节烧到 eFuse 偏移 x1（meson_sm_call_write() 先把数据复制进这块缓冲，再发同一个调用）。缓冲普通世界可写，清单逐字节列出它此刻的内容，与 sm efusewrite 同样绑定数据。
 * 缓冲是安全监控交给普通世界读写的共享内存，可以落在 /reserved-memory 的 no-map 区里，所以只要求它在内存里；meson-sm 用的缓冲是 4 KiB，更长的 x2 只列前 4 KiB
 */
static void rl_smc_efuse(struct rl_text *t, ulong offset, ulong size)
{
	ulong buf = redline_sm_input_buffer();

	if (!buf || !rl_in_ram(buf, min_t(ulong, size, RL_SM_SHMEM_SIZE))) {
		rl_printf(t, "  data    : the secure monitor input buffer (0x%lx) is not readable (not shown)\n",
			  buf);
		return;
	}
	rl_printf(t, "  data    : 0x%lx bytes from the secure monitor input buffer at 0x%lx\n",
		  size, buf);
	if (size > RL_SM_SHMEM_SIZE) {
		rl_puts(t, "  note    : x2 is larger than the 4 KiB input buffer: what the secure monitor does past it is not known; only the first 4 KiB are listed\n");
		size = RL_SM_SHMEM_SIZE;
	}
	rl_efuse_bytes(t, offset, buf, size);
}

static const char *rl_fid_name(const struct rl_fid *tbl, int n, u32 fid)
{
	int i;

	for (i = 0; i < n; i++)
		if (tbl[i].fid == fid)
			return tbl[i].what;

	return NULL;
}

static bool rl_smc(struct rl_text *t, const char *name, int argc,
		   char *const argv[])
{
	ulong fid, a[7];
	const char *what;
	int i;

	if (argc < 2)
		return false;	/* 只打印用法 */
	fid = hextoul(argv[1], NULL);
	for (i = 0; i < ARRAY_SIZE(smc_allowed); i++)
		if (fid == smc_allowed[i].fid)
			return false;
	for (i = 0; i < ARRAY_SIZE(a); i++)
		a[i] = argc > i + 2 ? hextoul(argv[i + 2], NULL) : 0;
	what = rl_fid_name(smc_known, ARRAY_SIZE(smc_known), fid);
	if (!what)
		what = rl_fid_name(smc_allowed, ARRAY_SIZE(smc_allowed), fid);
	rl_printf(t, "  action  : %s, function ID 0x%lx%s%s\n",
		  strcmp(name, "hvc") ? "secure monitor call (smc)" :
		  "hypervisor call (hvc)", fid,
		  fid != (u32)fid ? " (upper 32 bits set; SMCCC passes the function ID in W0)" : "",
		  what ? "" : ": not on the allow list");
	if (what)
		rl_printf(t, "  function: W0 0x%08x = %s\n", (u32)fid, what);
	rl_printf(t, "  args    : x1 0x%lx, x2 0x%lx, x3 0x%lx, x4 0x%lx, x5 0x%lx, x6 0x%lx, w7 0x%lx\n",
		  a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
	rl_puts(t, "  allowed : PSCI SYSTEM_RESET 0x84000009, Amlogic set reboot reason 0x82000049, PSCI_VERSION, PSCI_FEATURES, SMCCC_VERSION, SMCCC_ARCH_FEATURES, Amlogic secure storage read-only calls 0x82000023-0x82000027, 0x82000060/61/63/64/65/67/6b/6c\n");
	if ((u32)fid == RL_SMC_EFUSE_WRITE) {
		rl_smc_efuse(t, a[0], a[1]);
		rl_puts(t, "  IRREVERSIBLE: eFuse bits are one-time programmable\n");
	}

	return true;
}

/* ---- eficonfig：改 UEFI 启动项，非易失变量随之落盘 ---- */
static bool rl_eficonfig(struct rl_text *t, const char *name, int argc,
			 char *const argv[])
{
	if (IS_ENABLED(CONFIG_EFI_VARIABLE_FILE_STORE))
		rl_puts(t, "  action  : edit UEFI boot options; non-volatile UEFI variables are saved as ubootefi.var on the first EFI system partition found\n");
	else if (IS_ENABLED(CONFIG_EFI_VARIABLE_SF_STORE))
		rl_puts(t, "  action  : edit UEFI boot options; non-volatile UEFI variables are saved to SPI flash\n");
	else
		return false;

	return true;
}

/* ---- 环境：只有环境放在存储上（不是 ENV_IS_DEFAULT）时才有 save/erase ---- */
static const char *const env_subs[] = {
	"ask", "default", "delete", "edit", "callbacks", "flags", "export",
	"grep", "import", "indirect", "info", "load", "print", "run", "save",
	"erase", "select", "set", "exists",
};

static bool rl_env(struct rl_text *t, const char *name, int argc,
		   char *const argv[])
{
	const char *sub;

	if (!IS_ENABLED(CONFIG_CMD_SAVEENV) || IS_ENABLED(CONFIG_ENV_IS_DEFAULT))
		return false;
	if (strcmp(name, "env")) {
		sub = name;	/* saveenv、eraseenv */
	} else {
		if (argc < 2)
			return false;
		sub = rl_resolve(argv[1], env_subs, ARRAY_SIZE(env_subs));
		if (sub && strcmp(sub, "save") && strcmp(sub, "erase"))
			return false;
		if (!sub) {
			rl_unclassified(t, name, argv[1]);
			return true;
		}
	}
	rl_printf(t, "  action  : %s the environment on its storage\n",
		  strstr(sub, "erase") ? "erase" : "save");

	return true;
}

/* ---- dfu：编进了存储后端时，主机可经它写任一 dfu_alt_info 项 ---- */
static bool rl_dfu(struct rl_text *t, const char *name, int argc,
		   char *const argv[])
{
	if (!IS_ENABLED(CONFIG_DFU_MMC) && !IS_ENABLED(CONFIG_DFU_NAND) &&
	    !IS_ENABLED(CONFIG_DFU_SF) && !IS_ENABLED(CONFIG_DFU_MTD) &&
	    !IS_ENABLED(CONFIG_DFU_SCSI) && !IS_ENABLED(CONFIG_DFU_VIRT))
		return false;
	if (argc > 1 && !strcmp(argv[argc - 1], "list"))
		return false;
	rl_puts(t, "  action  : DFU: the USB host may write any storage alt setting in dfu_alt_info\n");

	return true;
}

/* ---- gpt/mbr <op> <interface> <dev> ...：改写整个设备的分区表 ---- */
static const char *const gpt_writes[] = {
	"write", "repair", "swap", "rename", "transpose", "set-bootable",
};

static bool rl_ptable(struct rl_text *t, const char *name, int argc,
		      char *const argv[])
{
	struct blk_desc *desc;

	if (argc < 2 || !rl_in(argv[1], gpt_writes, ARRAY_SIZE(gpt_writes)) ||
	    (!strcmp(name, "mbr") && strcmp(argv[1], "write")))
		return false;
	rl_printf(t, "  action  : rewrite the %s partition table ('%s')\n", name,
		  argv[1]);
	/* 与命令相同：blk_get_dev() 取整个设备（MMC 会切回 user 区） */
	desc = argc > 3 ? blk_get_dev(argv[2], dectoul(argv[3], NULL)) : NULL;
	if (desc)
		rl_device(t, desc);
	else
		rl_printf(t, "  device  : %s %s (not available now)\n",
			  rl_arg(argc, argv, 2), rl_arg(argc, argv, 3));

	return true;
}

struct rl_rule {
	const char *name;	/* 命令表里的规范名 */
	/* 要拦就把清单正文写进 @t 并返回 true；放行返回 false */
	bool (*judge)(struct rl_text *t, const char *name, int argc,
		      char *const argv[]);
};

static const struct rl_rule rl_rules[] = {
	{ "mmc", rl_mmc },
	{ "usb", rl_usb },
	{ "gzwrite", rl_gzwrite },
	{ "ums", rl_ums },
	{ "fatwrite", rl_fs },
	{ "fatmkdir", rl_fs },
	{ "fatrm", rl_fs },
	{ "ext4write", rl_fs },
	{ "save", rl_fs },
	{ "mkdir", rl_fs },
	{ "rm", rl_fs },
	{ "mv", rl_fs },
	{ "ln", rl_fs },
	{ "sm", rl_sm },
	{ "smc", rl_smc },
	{ "hvc", rl_smc },
	{ "eficonfig", rl_eficonfig },
	{ "env", rl_env },
	{ "saveenv", rl_env },
	{ "eraseenv", rl_env },
	{ "dfu", rl_dfu },
	{ "gpt", rl_ptable },
	{ "mbr", rl_ptable },
};

/* 输入代次：redline_input_next() 每次加一；待确认动作记下被拦时的代次，redline yes 只接受更早代次的 */
static uint rl_input_gen;

void redline_input_next(void)
{
	rl_input_gen++;
}

/* 唯一的待确认动作：argv 的指针数组与字符串同一块分配；text 是展示过的清单正文，gen 是被拦时的输入代次 */
static struct {
	int argc;
	char **argv;
	char *text;
	uint gen;
} rl_pending;

static void rl_drop(void)
{
	free(rl_pending.argv);
	free(rl_pending.text);
	rl_pending.argc = 0;
	rl_pending.argv = NULL;
	rl_pending.text = NULL;
}

static char **rl_dup_argv(int argc, char *const argv[])
{
	size_t size = (argc + 1) * sizeof(char *);
	char **v, *p;
	int i;

	for (i = 0; i < argc; i++)
		size += strlen(argv[i]) + 1;
	v = malloc(size);
	if (!v)
		return NULL;
	p = (char *)(v + argc + 1);
	for (i = 0; i < argc; i++) {
		v[i] = strcpy(p, argv[i]);
		p += strlen(p) + 1;
	}
	v[argc] = NULL;

	return v;
}

/* 按当前状态生成清单；返回 true 表示要拦，清单在 @t（内存不够时 t->oom） */
static bool rl_judge(struct cmd_tbl *cmdtp, int argc, char *const argv[],
		     struct rl_text *t)
{
	int i;

	if (!cmdtp)
		return false;
	for (i = 0; i < ARRAY_SIZE(rl_rules); i++) {
		if (strcmp(cmdtp->name, rl_rules[i].name))
			continue;
		rl_puts(t, "  command :");
		for (int j = 0; j < argc; j++) {
			rl_puts(t, " ");
			rl_puts(t, argv[j]);
		}
		rl_puts(t, "\n");
		if (rl_rules[i].judge(t, cmdtp->name, argc, argv))
			return true;
		break;
	}
	free(t->buf);
	memset(t, '\0', sizeof(*t));

	return false;
}

static void rl_hint(void)
{
	puts("Next: 'redline yes' as a separate input runs it once, 'redline show' lists it again, 'redline cancel' drops it\n");
}

/* 把 @argv 记为待确认动作（替换原有的，记下当前输入代次）并展示 @t；@t 的缓冲归待确认动作所有 */
static void rl_hold(int argc, char *const argv[], struct rl_text *t)
{
	char **v = t->oom ? NULL : rl_dup_argv(argc, argv);

	rl_drop();
	if (!v) {
		free(t->buf);
		puts("REDLINE: storage write held, not executed (out of memory: nothing is pending)\n");
		return;
	}
	rl_pending.argc = argc;
	rl_pending.argv = v;
	rl_pending.text = t->buf;
	rl_pending.gen = rl_input_gen;
	puts("REDLINE: storage write held, not executed\n");
	puts(rl_pending.text);
	rl_hint();
}

int redline_gate(struct cmd_tbl *cmdtp, int argc, char *const argv[])
{
	struct rl_text t = {};

	if (!rl_judge(cmdtp, argc, argv, &t))
		return 0;
	rl_hold(argc, argv, &t);

	return CMD_RET_FAILURE;
}

static int rl_yes(void)
{
	struct rl_text t = {};
	int argc = rl_pending.argc, rep = 1, ret;
	char **argv = rl_pending.argv;

	if (!argv) {
		puts("redline: nothing pending\n");
		return CMD_RET_FAILURE;
	}
	/* 同一次输入里被拦的：清单与确认出自同一行、同一段脚本或同一个 fastboot 请求，没有人在两者之间看过清单 */
	if (rl_pending.gen == rl_input_gen) {
		puts("REDLINE: the held action was listed during this same input (command line, script, bootcmd or fastboot request); not executed, it stays pending for 'redline yes' as a separate input\n");
		return CMD_RET_FAILURE;
	}
	if (!rl_judge(find_cmd(argv[0]), argc, argv, &t)) {
		puts("redline: the held action no longer needs confirmation; dropped\n");
		rl_drop();
		return CMD_RET_FAILURE;
	}
	if (t.oom || strcmp(t.buf, rl_pending.text)) {
		puts("REDLINE: the target or the source data changed since the list was shown; not executed, new list follows\n");
		rl_hold(argc, argv, &t);
		return CMD_RET_FAILURE;
	}
	free(t.buf);
	/* 单次有效：执行前就作废，执行中再被拦的动作成为新的待确认动作 */
	free(rl_pending.text);
	rl_pending.argc = 0;
	rl_pending.argv = NULL;
	rl_pending.text = NULL;
	printf("redline: running");
	for (int i = 0; i < argc; i++)
		printf(" %s", argv[i]);
	puts("\n");
	ret = cmd_process(CMD_FLAG_REDLINE_CONFIRMED, argc, argv, &rep, NULL);
	free(argv);

	return ret;
}

static int do_redline(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	if (argc == 2 && !strcmp(argv[1], "yes"))
		return rl_yes();
	if (argc == 2 && !strcmp(argv[1], "cancel")) {
		if (rl_pending.argv)
			printf("redline: dropped %s\n", rl_pending.argv[0]);
		else
			puts("redline: nothing pending\n");
		rl_drop();
		return CMD_RET_SUCCESS;
	}
	if (argc == 1 || !strcmp(argv[1], "show")) {
		if (!rl_pending.argv) {
			puts("redline: nothing pending\n");
			return CMD_RET_FAILURE;
		}
		puts("REDLINE: pending storage write, not executed\n");
		puts(rl_pending.text);
		rl_hint();
		return CMD_RET_SUCCESS;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(redline, 2, 0, do_redline,
	   "confirm a held storage write",
	   "[show] - list the held storage write\n"
	   "redline yes    - run the held storage write once (only from a later input than the one that held it)\n"
	   "redline cancel - drop the held storage write\n"
	   "Commands that write storage or change eMMC configuration are held\n"
	   "instead of run; the latest held one is the only one 'yes' runs.");
