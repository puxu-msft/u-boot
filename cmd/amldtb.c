// SPDX-License-Identifier: GPL-2.0+
/*
 * amldtb：从 Amlogic 厂商 DTB 镜像里按名字选出一份 FDT
 *
 * 厂商 DTB 的字节在三处出现，外层包装不同，按下列顺序逐层剥开（每层可选）：
 *
 * 1. Android boot image（头版本 0..2）的 second 区：取 page + ALIGN(kernel) + ALIGN(ramdisk) 起的 second_size 字节。
 * 2. eMMC reserved 分区 +4 MiB / +4.25 MiB 的两份 DTB 槽：各 256 KiB，尾部 16 B 是 magic "A~D\0"、version、timestamp、checksum（厂商 common/cmd_aml_mmc.c 的 struct aml_dtb_rsv）。checksum 是槽内前 65535 个 LE 32 位字的回绕和（_calc_dtb_checksum）。只在长度恰为 256 KiB 且尾部 magic 对上时按槽处理，校验和不对就拒绝。
 * 3. gzip：厂商 get_multi_dt_entry 认 1f 8b 开头的数据。这里用 zlib 的 gzip 封装解压，CRC32 与 ISIZE 尾部一并校验；流结束后的填充（DTB 槽里旧内容的残留、boot image 的页对齐）不参与。
 *
 * 剥完后是 AML_ 多 DTB 容器或单个 FDT。AML_ 容器（厂商 common/aml_dt.c）：
 *
 *   0x00 "AML_"  0x04 version（1 或 2）  0x08 count  0x0c 起 count 项：
 *   3 个 id 字段（v1 每个 4 B，v2 每个 16 B），后接 offset(u32) size(u32)
 *
 * id 是 soc、platform、variant，按 32 位字存放且字内字节逆序，空格补齐；名字是三者用 '_' 连接（厂商 env aml_dt 的写法，如 sm1_ac213v2_4g）。offset 相对容器起点，size 是该 FDT 占的槽长。容器没有自己的校验和，所以每一项都要过结构检查：在头部之后、8 字节对齐（libfdt 的要求；M12 真字节里各项按 2 KiB 对齐）、不越过容器尾、id 为非空的可打印字符、槽里是 totalsize 不超过槽长的合法 FDT；任何一项不过整个容器都不认。同名多项时与厂商一样取最后一项。选中的 FDT 再过 fdt_check_full。
 */

#include <android_image.h>
#include <command.h>
#include <env.h>
#include <errno.h>
#include <malloc.h>
#include <mapmem.h>
#include <vsprintf.h>
#include <asm/unaligned.h>
#include <linux/ctype.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <u-boot/zlib.h>

#define AML_RSV_SLOT_SIZE	SZ_256K
#define AML_RSV_TAIL_SIZE	16
#define AML_RSV_MAGIC		0x00447e41	/* "A~D\0" */
#define AML_DT_MAGIC		0x5f4c4d41	/* "AML_" */
#define AML_DT_HDR_SIZE		12
#define AML_DT_IDS		3
/* 解压上限与厂商 GUNZIP_BUF_SIZE 相同 */
#define AMLDTB_GZIP_MAX		0x500000

struct amldtb_img {
	const u8 *buf;		/* 当前层的数据 */
	ulong len;
	void *map;		/* map_sysmem 的原始映射 */
	void *gz;		/* gzip 解压缓冲，非空时由 amldtb_close 释放 */
};

struct amldtb_entry {
	char name[AML_DT_IDS * 17];
	u32 offset;
	u32 size;
};

static int amldtb_android(struct amldtb_img *img)
{
	u32 page, ksize, rsize, ssize, ver;
	u64 off;

	if (img->len < sizeof(struct andr_boot_img_hdr_v0))
		goto trunc;
	ver = get_unaligned_le32(img->buf +
			offsetof(struct andr_boot_img_hdr_v0, header_version));
	if (ver > 2) {
		printf("Android boot image v%u has no second area\n", ver);
		return -EPROTONOSUPPORT;
	}
	page = get_unaligned_le32(img->buf +
			offsetof(struct andr_boot_img_hdr_v0, page_size));
	ksize = get_unaligned_le32(img->buf +
			offsetof(struct andr_boot_img_hdr_v0, kernel_size));
	rsize = get_unaligned_le32(img->buf +
			offsetof(struct andr_boot_img_hdr_v0, ramdisk_size));
	ssize = get_unaligned_le32(img->buf +
			offsetof(struct andr_boot_img_hdr_v0, second_size));
	if (!page || (page & (page - 1))) {
		printf("Android boot image: bad page size %#x\n", page);
		return -EINVAL;
	}
	if (!ssize) {
		printf("Android boot image: empty second area\n");
		return -ENOENT;
	}
	off = page + ALIGN((u64)ksize, page) + ALIGN((u64)rsize, page);
	if (off > img->len || ssize > img->len - off)
		goto trunc;

	printf("Android boot image v%u: second at +%#llx, %u bytes\n", ver,
	       off, ssize);
	img->buf += off;
	img->len = ssize;
	return 0;

trunc:
	printf("Android boot image truncated (%lu bytes)\n", img->len);
	return -ENODATA;
}

static int amldtb_rsv_slot(struct amldtb_img *img)
{
	const u8 *tail = img->buf + img->len - AML_RSV_TAIL_SIZE;
	u32 sum = 0, want = get_unaligned_le32(tail + 12);
	ulong i;

	for (i = 0; i < img->len - 4; i += 4)
		sum += get_unaligned_le32(img->buf + i);
	if (sum != want) {
		printf("reserved DTB slot: checksum %08x, computed %08x\n",
		       want, sum);
		return -EBADMSG;
	}

	printf("reserved DTB slot: version %u, timestamp %u, checksum %08x OK\n",
	       get_unaligned_le32(tail + 4), get_unaligned_le32(tail + 8), sum);
	img->len -= AML_RSV_TAIL_SIZE;
	return 0;
}

static int amldtb_gunzip(struct amldtb_img *img)
{
	z_stream s = {};
	void *dst;
	int r;

	dst = malloc(AMLDTB_GZIP_MAX);
	if (!dst)
		return -ENOMEM;
	s.zalloc = gzalloc;
	s.zfree = gzfree;
	if (inflateInit2(&s, 16 + MAX_WBITS) != Z_OK) {
		free(dst);
		return -EIO;
	}
	s.next_in = (u8 *)img->buf;
	s.avail_in = min_t(ulong, img->len, UINT_MAX);
	s.next_out = dst;
	s.avail_out = AMLDTB_GZIP_MAX;
	r = inflate(&s, Z_FINISH);
	inflateEnd(&s);
	if (r != Z_STREAM_END) {
		if (!s.avail_out)
			printf("gzip: data exceeds %#x bytes\n", AMLDTB_GZIP_MAX);
		else
			printf("gzip: inflate failed (%d: %s) after %lu input bytes\n",
			       r, s.msg ? s.msg : "truncated", s.total_in);
		free(dst);
		return -EBADMSG;
	}

	printf("gzip: %lu -> %lu bytes, CRC32 and ISIZE OK\n", s.total_in,
	       s.total_out);
	img->gz = dst;
	img->buf = dst;
	img->len = s.total_out;
	return 0;
}

/* 剥掉 boot image、DTB 槽与 gzip 三层外包装（都可选） */
static int amldtb_unwrap(struct amldtb_img *img)
{
	int ret;

	if (img->len >= ANDR_BOOT_MAGIC_SIZE &&
	    !memcmp(img->buf, ANDR_BOOT_MAGIC, ANDR_BOOT_MAGIC_SIZE)) {
		ret = amldtb_android(img);
		if (ret)
			return ret;
	}
	if (img->len == AML_RSV_SLOT_SIZE &&
	    get_unaligned_le32(img->buf + img->len - AML_RSV_TAIL_SIZE) ==
	    AML_RSV_MAGIC) {
		ret = amldtb_rsv_slot(img);
		if (ret)
			return ret;
	}
	if (img->len >= 2 && img->buf[0] == 0x1f && img->buf[1] == 0x8b)
		return amldtb_gunzip(img);

	return 0;
}

static bool amldtb_fdt_ok(const void *fdt, ulong room)
{
	return room >= sizeof(struct fdt_header) && !fdt_check_header(fdt) &&
		fdt_totalsize(fdt) <= room;
}

/* 解出第 idx 项；id 不可打印或为空时返回 false */
static bool amldtb_entry(const struct amldtb_img *img, u32 idlen, u32 idx,
			 struct amldtb_entry *e)
{
	const u8 *p = img->buf + AML_DT_HDR_SIZE +
		(ulong)idx * (AML_DT_IDS * idlen + 8);
	char *s = e->name;
	u32 f, j;

	for (f = 0; f < AML_DT_IDS; f++, p += idlen) {
		char *start;

		if (f)
			*s++ = '_';
		start = s;
		for (j = 0; j < idlen; j++) {
			char c = p[(j & ~3) + 3 - (j & 3)];

			if (c == ' ' || !c)
				break;
			if (!isprint(c))
				return false;
			*s++ = c;
		}
		if (s == start)
			return false;
	}
	*s = '\0';
	e->offset = get_unaligned_le32(p);
	e->size = get_unaligned_le32(p + 4);
	return true;
}

/**
 * amldtb_container() - 校验 AML_ 容器的每一项，可选列出或按名字选择
 *
 * @img: 剥完外层的数据，开头是 "AML_"
 * @name: 要选的名字；NULL 表示只校验
 * @list: 逐项打印
 * @fdtp: 返回选中的 FDT
 * Return: 选中项的下标，只校验时为 0；失败为负的 errno
 */
static int amldtb_container(const struct amldtb_img *img, const char *name,
			    bool list, const void **fdtp)
{
	struct amldtb_entry e;
	u32 ver, idlen, count, i;
	int found = -ENOENT;
	u64 hdr_end;

	if (img->len < AML_DT_HDR_SIZE)
		return -ENODATA;
	ver = get_unaligned_le32(img->buf + 4);
	count = get_unaligned_le32(img->buf + 8);
	if (ver != 1 && ver != 2) {
		printf("AML_: unknown version %u\n", ver);
		return -EPROTONOSUPPORT;
	}
	idlen = ver == 1 ? 4 : 16;
	hdr_end = AML_DT_HDR_SIZE + (u64)count * (AML_DT_IDS * idlen + 8);
	if (!count || hdr_end > img->len) {
		printf("AML_: %u entries do not fit in %lu bytes\n", count,
		       img->len);
		return -EINVAL;
	}
	if (list)
		printf("AML_ v%u, %u DTBs\n", ver, count);

	for (i = 0; i < count; i++) {
		const void *fdt;

		if (!amldtb_entry(img, idlen, i, &e)) {
			printf("AML_: entry %u has a bad id\n", i);
			return -EINVAL;
		}
		fdt = img->buf + e.offset;
		if (e.offset < hdr_end || e.offset % 8 || e.offset > img->len ||
		    e.size > img->len - e.offset || !amldtb_fdt_ok(fdt, e.size)) {
			printf("AML_: entry %u '%s' offset %#x size %#x invalid in %lu bytes\n",
			       i, e.name, e.offset, e.size, img->len);
			return -EINVAL;
		}
		if (list)
			printf("%3u  %-24s offset %#08x size %#07x fdt %u bytes\n",
			       i, e.name, e.offset, e.size, fdt_totalsize(fdt));
		if (name && !strcmp(name, e.name)) {
			found = i;
			*fdtp = fdt;
		}
	}

	return name ? found : 0;
}

static int amldtb_open(char *const argv[], struct amldtb_img *img)
{
	ulong addr = hextoul(argv[0], NULL);

	img->len = hextoul(argv[1], NULL);
	img->map = map_sysmem(addr, img->len);
	img->buf = img->map;
	img->gz = NULL;
	/*
	 * libfdt 要求 FDT 8 字节对齐，这里先管起点。剥层后不一定还对齐：page_size 小于 8 的 Android 启动映像（1、2、4 都合法）second 区可能落在不对齐处，gzip 层解压进 malloc 缓冲后又是对齐的；不对齐处的 FDT 与 AML_ 项过不了 fdt_check_header()（FDT_ERR_ALIGNMENT），整体被拒，不会被误读
	 */
	if (addr % 8) {
		printf("address %#lx is not 8-byte aligned\n", addr);
		return -EINVAL;
	}
	return amldtb_unwrap(img);
}

static void amldtb_close(struct amldtb_img *img)
{
	free(img->gz);
	unmap_sysmem(img->map);
}

static int do_amldtb_list(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct amldtb_img img;
	int ret;

	if (argc != 3)
		return CMD_RET_USAGE;
	ret = amldtb_open(argv + 1, &img);
	if (!ret) {
		if (img.len >= 4 &&
		    get_unaligned_le32(img.buf) == AML_DT_MAGIC) {
			ret = amldtb_container(&img, NULL, true, NULL);
		} else if (amldtb_fdt_ok(img.buf, img.len)) {
			printf("single FDT, %u bytes\n", fdt_totalsize(img.buf));
		} else {
			printf("neither AML_ container nor FDT\n");
			ret = -EINVAL;
		}
	}
	amldtb_close(&img);

	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

static int do_amldtb_select(struct cmd_tbl *cmdtp, int flag, int argc,
			    char *const argv[])
{
	const char *name;
	const void *fdt = NULL;
	struct amldtb_img img;
	ulong dst, max = ULONG_MAX;
	void *dstp;
	u32 size;
	int ret;

	if (argc >= 3 && !strcmp(argv[1], "-m")) {
		max = hextoul(argv[2], NULL);
		argc -= 2;
		argv += 2;
	}
	if (argc < 4 || argc > 5)
		return CMD_RET_USAGE;
	name = argc > 4 ? argv[4] : env_get("aml_dt");
	dst = hextoul(argv[3], NULL);
	ret = amldtb_open(argv + 1, &img);
	if (ret)
		goto out;

	if (img.len >= 4 && get_unaligned_le32(img.buf) == AML_DT_MAGIC) {
		if (!name) {
			printf("no DTB name: give <name> or set aml_dt\n");
			ret = -EINVAL;
			goto out;
		}
		ret = amldtb_container(&img, name, false, &fdt);
		if (ret == -ENOENT)
			printf("AML_: no DTB named '%s'\n", name);
		if (ret < 0)
			goto out;
		printf("AML_: selected #%d %s\n", ret, name);
	} else if (amldtb_fdt_ok(img.buf, img.len)) {
		fdt = img.buf;
		printf("single FDT\n");
	} else {
		printf("neither AML_ container nor FDT\n");
		ret = -EINVAL;
		goto out;
	}

	ret = fdt_check_full(fdt, fdt_totalsize(fdt));
	if (ret) {
		printf("FDT check failed: %s\n", fdt_strerror(ret));
		ret = -EINVAL;
		goto out;
	}
	size = fdt_totalsize(fdt);
	if (size > max) {
		printf("%u-byte FDT exceeds the %#lx bytes allowed at %#lx\n",
		       size, max, dst);
		ret = -E2BIG;
		goto out;
	}
	dstp = map_sysmem(dst, size);
	memmove(dstp, fdt, size);
	unmap_sysmem(dstp);
	env_set_hex("filesize", size);
	printf("%u bytes -> %#lx\n", size, dst);
	ret = 0;
out:
	amldtb_close(&img);

	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_LONGHELP(amldtb,
	"list <addr> <len>\n"
	"    - show the layers and DTB entries of an Amlogic DTB image\n"
	"amldtb select [-m <max>] <addr> <len> <dst> [<name>]\n"
	"    - copy the DTB named <name> ($aml_dt by default, ignored for a\n"
	"      single FDT) to <dst> and set filesize to its size; with -m,\n"
	"      fail without writing if it is larger than <max> bytes\n"
	"<addr>/<len> may hold an Android boot image (v0-v2, its second area\n"
	"is used), a 256 KiB reserved-partition DTB slot, gzip data, an AML_\n"
	"multi-DTB container or a plain FDT; layers are unwrapped in this order");

U_BOOT_CMD_WITH_SUBCMDS(amldtb, "Amlogic multi-DTB image", amldtb_help_text,
	U_BOOT_SUBCMD_MKENT(list, 3, 1, do_amldtb_list),
	U_BOOT_SUBCMD_MKENT(select, 7, 0, do_amldtb_select));
