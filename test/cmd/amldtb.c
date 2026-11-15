// SPDX-License-Identifier: GPL-2.0+
/*
 * amldtb 命令的测试
 *
 * 在内存里按厂商格式手工构造输入（不复用 cmd/amldtb.c 的解析代码）：AML_ v1/v2 容器、gzip 封装、reserved 分区 256 KiB DTB 槽、Android boot image second 区与单个 FDT；正常输入要选中预期的那份 FDT 并逐字节相同，截断、伪造项数/偏移、坏 id、坏 FDT、名字不存在、gzip CRC 错、DTB 槽校验和错、FDT 超出 -m 给的容量、page_size 小于 8 使 second 区不对齐都必须失败。
 */

#include <android_image.h>
#include <command.h>
#include <env.h>
#include <gzip.h>
#include <malloc.h>
#include <mapmem.h>
#include <asm/unaligned.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>
#include <test/cmd.h>
#include <test/test.h>
#include <test/ut.h>

#define SLOT		0x200	/* 容器里每份 FDT 的槽长 */
#define DATA_OFF	0x800	/* 第一份 FDT 的偏移，与厂商工具一样按 2 KiB 对齐 */

struct amldtb_test {
	ulong src, dst;		/* 输入与输出的 sysmem 地址 */
	u8 *in;			/* 输入缓冲 */
};

static const char *const ids3[][3] = {
	{ "sm1", "ac213", "4g" },
	{ "sm1", "ac213v2", "2g" },
	{ "sm1", "ac213v2", "4g" },
};

/* 写一个 id 字段：按 32 位字存放、字内字节逆序、空格补齐 */
static void put_id(u8 *p, const char *id, uint idlen)
{
	uint j, n = strlen(id);

	for (j = 0; j < idlen; j++)
		p[(j & ~3) + 3 - (j & 3)] = j < n ? id[j] : ' ';
}

/* 在 @buf 造一份 model 为 @model 的最小 FDT，返回 totalsize */
static uint mkfdt(void *buf, const char *model)
{
	fdt_create_empty_tree(buf, SLOT);
	fdt_setprop_string(buf, 0, "model", model);
	fdt_pack(buf);

	return fdt_totalsize(buf);
}

/* 构造 AML_ 容器，返回总长；每份 FDT 的 model 是它的名字 */
static ulong mkaml(u8 *buf, uint version, const char *const (*ids)[3], uint n)
{
	uint idlen = version == 1 ? 4 : 16, ent = 3 * idlen + 8, i, f;
	char name[64];

	memset(buf, '\0', DATA_OFF + n * SLOT);
	put_unaligned_le32(0x5f4c4d41, buf);
	put_unaligned_le32(version, buf + 4);
	put_unaligned_le32(n, buf + 8);
	for (i = 0; i < n; i++) {
		u8 *p = buf + 12 + i * ent;

		for (f = 0; f < 3; f++)
			put_id(p + f * idlen, ids[i][f], idlen);
		put_unaligned_le32(DATA_OFF + i * SLOT, p + 3 * idlen);
		put_unaligned_le32(SLOT, p + 3 * idlen + 4);
		snprintf(name, sizeof(name), "%s_%s_%s", ids[i][0], ids[i][1],
			 ids[i][2]);
		mkfdt(buf + DATA_OFF + i * SLOT, name);
	}

	return DATA_OFF + n * SLOT;
}

/* 断言 dst 处是 model 为 @model 的 FDT，filesize 等于它的 totalsize */
static int check_out(struct unit_test_state *uts, struct amldtb_test *t,
		     const char *model)
{
	const void *fdt = map_sysmem(t->dst, SLOT);

	ut_assertok(fdt_check_header(fdt));
	ut_asserteq(fdt_totalsize(fdt), env_get_hex("filesize", 0));
	ut_asserteq_str(model, fdt_getprop(fdt, 0, "model", NULL));
	unmap_sysmem(fdt);

	return 0;
}

static int sel(struct amldtb_test *t, ulong len, const char *name)
{
	memset(map_sysmem(t->dst, SLOT), '\0', SLOT);
	env_set("filesize", NULL);
	return run_commandf("amldtb select %lx %lx %lx %s", t->src, len, t->dst,
			    name ? name : "");
}

static int setup(struct amldtb_test *t)
{
	ulong loadaddr = env_get_hex("loadaddr", 0);

	t->src = loadaddr;
	t->dst = loadaddr + SZ_1M;
	t->in = map_sysmem(t->src, SZ_1M);
	env_set("aml_dt", NULL);
	return 0;
}

/* AML_ 容器本身：选择、默认名字、名字不存在、同名、v1 */
static int cmd_test_amldtb_container(struct unit_test_state *uts)
{
	static const char *const dup[][3] = {
		{ "sm1", "ac213", "4g" }, { "g12a", "u200", "1g" },
		{ "g12a", "u200", "1g" },
	};
	static const char *const v1[][3] = {
		{ "g12a", "u200", "1g" }, { "sm1", "s905", "2g" },
	};
	struct amldtb_test t;
	ulong len;

	setup(&t);
	len = mkaml(t.in, 2, ids3, 3);

	ut_assertok(run_commandf("amldtb list %lx %lx", t.src, len));
	ut_assert_nextline("AML_ v2, 3 DTBs");
	ut_assert_nextlinen("  0  sm1_ac213_4g ");
	ut_assert_nextlinen("  1  sm1_ac213v2_2g ");
	ut_assert_nextlinen("  2  sm1_ac213v2_4g ");
	ut_assert_console_end();

	ut_assertok(sel(&t, len, "sm1_ac213v2_4g"));
	ut_assert_nextline("AML_: selected #2 sm1_ac213v2_4g");
	ut_assertok(check_out(uts, &t, "sm1_ac213v2_4g"));
	console_record_reset();

	/* 默认取 env aml_dt；两者都没有就失败 */
	ut_asserteq(1, sel(&t, len, NULL));
	ut_assert_nextline("no DTB name: give <name> or set aml_dt");
	env_set("aml_dt", "sm1_ac213_4g");
	ut_assertok(sel(&t, len, NULL));
	ut_assertok(check_out(uts, &t, "sm1_ac213_4g"));
	env_set("aml_dt", NULL);
	console_record_reset();

	/* 名字不存在、只差前缀或后缀都不算匹配 */
	ut_asserteq(1, sel(&t, len, "sm1_ac213v2_8g"));
	ut_assert_nextline("AML_: no DTB named 'sm1_ac213v2_8g'");
	ut_asserteq(1, sel(&t, len, "sm1_ac213v2"));
	ut_asserteq(1, sel(&t, len, "sm1_ac213v2_4g_x"));
	ut_asserteq(0, env_get_hex("filesize", 0));
	console_record_reset();

	/* 同名多项取最后一项（与厂商 get_multi_dt_entry 相同） */
	len = mkaml(t.in, 2, dup, 3);
	ut_assertok(sel(&t, len, "g12a_u200_1g"));
	ut_assert_nextline("AML_: selected #2 g12a_u200_1g");
	console_record_reset();

	/* v1：id 每个 4 字节 */
	len = mkaml(t.in, 1, v1, 2);
	ut_assertok(run_commandf("amldtb list %lx %lx", t.src, len));
	ut_assert_nextline("AML_ v1, 2 DTBs");
	ut_assertok(sel(&t, len, "sm1_s905_2g"));
	ut_assertok(check_out(uts, &t, "sm1_s905_2g"));
	console_record_reset();

	return 0;
}
CMD_TEST(cmd_test_amldtb_container, UTF_CONSOLE);

/* -m 给出 dst 的容量：FDT 比它大时报错，dst 与 filesize 都不动；正好放得下时照常 */
static int cmd_test_amldtb_max(struct unit_test_state *uts)
{
	struct amldtb_test t;
	ulong len;
	uint size;
	u8 *dst;

	setup(&t);
	len = mkaml(t.in, 2, ids3, 3);
	size = fdt_totalsize(t.in + DATA_OFF + 2 * SLOT);
	dst = map_sysmem(t.dst, SLOT);
	memset(dst, '\0', SLOT);
	env_set("filesize", NULL);

	ut_asserteq(1, run_commandf("amldtb select -m %x %lx %lx %lx sm1_ac213v2_4g",
				    size - 1, t.src, len, t.dst));
	ut_assert_nextline("AML_: selected #2 sm1_ac213v2_4g");
	ut_assert_nextline("%u-byte FDT exceeds the %#x bytes allowed at %#lx",
			   size, size - 1, t.dst);
	ut_assert_console_end();
	ut_asserteq(0, env_get_hex("filesize", 0));
	ut_assertnull(memchr_inv(dst, '\0', SLOT));

	ut_assertok(run_commandf("amldtb select -m %x %lx %lx %lx sm1_ac213v2_4g",
				 size, t.src, len, t.dst));
	ut_assertok(check_out(uts, &t, "sm1_ac213v2_4g"));
	console_record_reset();

	/* 只给 -m 不给值 */
	ut_asserteq(1, run_command("amldtb select -m", 0));
	console_record_reset();
	unmap_sysmem(dst);

	return 0;
}
CMD_TEST(cmd_test_amldtb_max, UTF_CONSOLE);

/* 坏容器：截断、伪造项数/偏移/大小、坏 id、坏版本、未选中项的坏 FDT 都整个不认 */
static int cmd_test_amldtb_bad(struct unit_test_state *uts)
{
	struct amldtb_test t;
	ulong len, e1;

	setup(&t);
	e1 = 12 + 56;	/* v2 第 1 项（从 0 数）的起点 */

	len = mkaml(t.in, 2, ids3, 3);
	ut_asserteq(1, sel(&t, len - 1, "sm1_ac213_4g"));
	ut_assert_nextline("AML_: entry 2 'sm1_ac213v2_4g' offset 0xc00 size 0x200 invalid in %lu bytes",
			   len - 1);
	ut_asserteq(1, sel(&t, 100, "sm1_ac213_4g"));
	ut_assert_nextline("AML_: 3 entries do not fit in 100 bytes");
	console_record_reset();

	put_unaligned_le32(0x10000000, t.in + 8);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_nextline("AML_: 268435456 entries do not fit in %lu bytes", len);
	put_unaligned_le32(0, t.in + 8);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_nextline("AML_: 0 entries do not fit in %lu bytes", len);
	console_record_reset();

	/* 偏移指进头部、越过尾部、不按 8 字节对齐；大小回绕 */
	len = mkaml(t.in, 2, ids3, 3);
	put_unaligned_le32(0, t.in + e1 + 48);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_nextlinen("AML_: entry 1 'sm1_ac213v2_2g' offset 0x0 ");
	put_unaligned_le32(len, t.in + e1 + 48);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	put_unaligned_le32(DATA_OFF + SLOT + 4, t.in + e1 + 48);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	put_unaligned_le32(DATA_OFF + SLOT, t.in + e1 + 48);
	put_unaligned_le32(0xffffffff, t.in + e1 + 52);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_skip_to_line("AML_: entry 1 'sm1_ac213v2_2g' offset 0xa00 size 0xffffffff invalid in %lu bytes",
			       len);
	console_record_reset();

	/* 槽长比 FDT 的 totalsize 小 */
	len = mkaml(t.in, 2, ids3, 3);
	put_unaligned_le32(16, t.in + e1 + 52);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	console_record_reset();

	/* 未选中那一项的 FDT 坏了，整个容器也不认 */
	len = mkaml(t.in, 2, ids3, 3);
	t.in[DATA_OFF + SLOT] ^= 0xff;
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_nextlinen("AML_: entry 1 ");
	console_record_reset();

	/* id 里有不可打印字符、整段为空 */
	len = mkaml(t.in, 2, ids3, 3);
	t.in[e1 + 16 + 3] = 0x01;
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_nextline("AML_: entry 1 has a bad id");
	len = mkaml(t.in, 2, ids3, 3);
	put_id(t.in + e1 + 16, "", 16);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_nextline("AML_: entry 1 has a bad id");
	console_record_reset();

	/* 未知版本 */
	len = mkaml(t.in, 2, ids3, 3);
	put_unaligned_le32(3, t.in + 4);
	ut_asserteq(1, sel(&t, len, "sm1_ac213_4g"));
	ut_assert_nextline("AML_: unknown version 3");
	console_record_reset();

	/* 既不是 AML_ 也不是 FDT；起点不对齐 */
	memset(t.in, 0x5a, 64);
	ut_asserteq(1, sel(&t, 64, "sm1_ac213_4g"));
	ut_assert_nextline("neither AML_ container nor FDT");
	ut_asserteq(1, run_commandf("amldtb select %lx 40 %lx x", t.src + 4,
				    t.dst));
	ut_assert_nextline("address %#lx is not 8-byte aligned", t.src + 4);
	console_record_reset();

	return 0;
}
CMD_TEST(cmd_test_amldtb_bad, UTF_CONSOLE);

/* 外层包装：单个 FDT、gzip、reserved 分区 DTB 槽、Android boot image */
static int cmd_test_amldtb_layers(struct unit_test_state *uts)
{
	struct andr_boot_img_hdr_v0 *hdr;
	ulong len, gzlen, i, page = 2048;
	struct amldtb_test t;
	u8 *aml, *gz, *slot;
	u32 sum, bad;

	if (!IS_ENABLED(CONFIG_GZIP_COMPRESSED))
		return -EAGAIN;
	setup(&t);
	aml = malloc(SZ_64K);
	gz = malloc(SZ_64K);
	ut_assertnonnull(aml);
	ut_assertnonnull(gz);
	len = mkaml(aml, 2, ids3, 3);
	gzlen = SZ_64K;
	ut_assertok(gzip(gz, &gzlen, aml, len));

	/* 单个 FDT 原样通过，名字不参与 */
	mkfdt(t.in, "single");
	ut_assertok(sel(&t, SLOT, "whatever"));
	ut_assert_nextline("single FDT");
	ut_assertok(check_out(uts, &t, "single"));
	console_record_reset();

	/* gzip：正常；尾部 CRC 错；截断 */
	memcpy(t.in, gz, gzlen);
	ut_assertok(sel(&t, gzlen, "sm1_ac213v2_2g"));
	ut_assert_nextline("gzip: %lu -> %lu bytes, CRC32 and ISIZE OK", gzlen,
			   len);
	ut_assertok(check_out(uts, &t, "sm1_ac213v2_2g"));
	console_record_reset();
	t.in[gzlen - 8] ^= 0x01;
	ut_asserteq(1, sel(&t, gzlen, "sm1_ac213v2_2g"));
	ut_assert_nextlinen("gzip: inflate failed (-3: incorrect data check)");
	t.in[gzlen - 8] ^= 0x01;
	ut_asserteq(1, sel(&t, gzlen - 1, "sm1_ac213v2_2g"));
	ut_assert_nextlinen("gzip: inflate failed (-5: truncated)");
	console_record_reset();

	/* DTB 槽：gzip + 残留垃圾 + 尾部 "A~D\0"、版本、时间戳、校验和 */
	slot = t.in;
	memset(slot, '\0', SZ_256K);
	memcpy(slot, gz, gzlen);
	memset(slot + gzlen + 64, 0xa5, 1000);
	put_unaligned_le32(0x00447e41, slot + SZ_256K - 16);
	put_unaligned_le32(1, slot + SZ_256K - 12);
	put_unaligned_le32(7, slot + SZ_256K - 8);
	for (sum = 0, i = 0; i < SZ_256K - 4; i += 4)
		sum += get_unaligned_le32(slot + i);
	put_unaligned_le32(sum, slot + SZ_256K - 4);
	ut_assertok(sel(&t, SZ_256K, "sm1_ac213_4g"));
	ut_assert_nextline("reserved DTB slot: version 1, timestamp 7, checksum %08x OK",
			   sum);
	ut_assertok(check_out(uts, &t, "sm1_ac213_4g"));
	console_record_reset();
	slot[gzlen + 100] ^= 0x01;
	for (bad = 0, i = 0; i < SZ_256K - 4; i += 4)
		bad += get_unaligned_le32(slot + i);
	ut_asserteq(1, sel(&t, SZ_256K, "sm1_ac213_4g"));
	ut_assert_nextline("reserved DTB slot: checksum %08x, computed %08x",
			   sum, bad);
	console_record_reset();

	/* Android boot image v1：second 区是 gzip 的容器 */
	memset(t.in, '\0', 6 * page + gzlen);
	hdr = (struct andr_boot_img_hdr_v0 *)t.in;
	memcpy(hdr->magic, ANDR_BOOT_MAGIC, ANDR_BOOT_MAGIC_SIZE);
	hdr->kernel_size = 3 * page - 100;
	hdr->ramdisk_size = page + 1;
	hdr->second_size = gzlen;
	hdr->page_size = page;
	hdr->header_version = 1;
	memcpy(t.in + 6 * page, gz, gzlen);
	len = 6 * page + ALIGN(gzlen, page);
	ut_assertok(sel(&t, len, "sm1_ac213v2_4g"));
	ut_assert_nextline("Android boot image v1: second at +%#lx, %lu bytes",
			   6 * page, gzlen);
	ut_assertok(check_out(uts, &t, "sm1_ac213v2_4g"));
	console_record_reset();
	ut_asserteq(1, sel(&t, 6 * page + gzlen - 1, "sm1_ac213v2_4g"));
	ut_assert_nextline("Android boot image truncated (%lu bytes)",
			   6 * page + gzlen - 1);
	hdr->header_version = 3;
	ut_asserteq(1, sel(&t, len, "sm1_ac213v2_4g"));
	ut_assert_nextline("Android boot image v3 has no second area");
	console_record_reset();

	/* page_size 4：second 区落在 +2004，不按 8 字节对齐，其中未压缩的 FDT 被 libfdt 拒绝 */
	len = mkfdt(aml, "unaligned");
	memset(t.in, '\0', 2004 + len);
	memcpy(hdr->magic, ANDR_BOOT_MAGIC, ANDR_BOOT_MAGIC_SIZE);
	hdr->kernel_size = 2000;
	hdr->second_size = len;
	hdr->page_size = 4;
	hdr->header_version = 1;
	memcpy(t.in + 2004, aml, len);
	ut_asserteq(1, sel(&t, 2004 + len, "x"));
	ut_assert_nextline("Android boot image v1: second at +0x7d4, %lu bytes", len);
	ut_assert_nextline("neither AML_ container nor FDT");
	console_record_reset();

	free(gz);
	free(aml);

	return 0;
}
CMD_TEST(cmd_test_amldtb_layers, UTF_CONSOLE);
