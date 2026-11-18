// SPDX-License-Identifier: GPL-2.0+
/*
 * unifykey 客户端（drivers/sm/meson-unifykey.c）与 unifykey 命令的测试
 *
 * 安全监控是厂商 BL31 安全存储的 sandbox 模型（drivers/sm/sandbox-meson-sm.c，test.dts 的 meson-secure-monitor）。测试按 BL31 的格式自己打包加密块（密钥派生与模型共用 sandbox_meson_sm_key()；格式另由主机侧 gen-unifykey-blob.py 独立实现、经 host 盘交叉核对），用 blkmap 把一块带 Amlogic MPT 的内存盘当作 eMMC，在 reserved 分区里放两份副本与 info，经 unifykey load blkmap <n>#reserved 走与实机相同的路径。
 * 每个用例结束时断言：模型不实现的调用（写、删、eFuse 等）一次也没有发生，整块盘的内容与放好副本时相同。
 */

#include <blk.h>
#include <blkmap.h>
#include <command.h>
#include <dm.h>
#include <env.h>
#include <malloc.h>
#include <mapmem.h>
#include <part.h>
#include <sandbox-meson-sm.h>
#include <uboot_aes.h>
#include <u-boot/crc.h>
#include <u-boot/sha256.h>
#include <asm/cache.h>
#include <asm/unaligned.h>
#include <dm/test.h>
#include <linux/sizes.h>
#include <meson/unifykey.h>
#include <test/test.h>
#include <test/ut.h>

#define BLKSZ		512
#define DISK_SIZE	SZ_16M
#define MPT_OFF		CONFIG_AML_PARTITION_OFFSET
#define RSV		MPT_OFF		/* reserved 分区起点；MPT 本身在它的开头 */
#define KEY_OFF		CONFIG_MESON_UNIFYKEY_OFFSET
#define BLOB		CONFIG_MESON_UNIFYKEY_SIZE
#define INFO_OFF	(KEY_OFF + 2 * BLOB)
#define MEM_ADDR	0x1000

struct tkey {
	const char *name;
	const char *val;
	u32 len;	/* 0 = strlen(val) */
	u32 attr;
};

#define MAC_A	"14:80:1F:CD:FE:50"
#define MAC_B	"14:80:1F:00:00:0B"
#define DEVID	"49070000M0065111200688"
#define AUTHKEY	"0123456789ABCDEFFEDCBA98765432100123456789ABCDEFFEDCBA9876543210"	/* 合成值，与本机 authkey 同长同格式 */

static const struct tkey keys_a[] = {
	{ "mac", MAC_A },
	{ "deviceid", DEVID },
	{ "authkey", AUTHKEY },
	{ "hdcp", "secure-data", 0, MESON_UNIFYKEY_ATTR_SECURE },
	{ "nonascii", "ab\x80" },
	{ "mac_raw", "\x02\x11\x22\x33\x44\x55", 6 },
	{ "mac_mcast", "01:00:5E:00:00:01" },
};

static const struct tkey keys_b[] = {
	{ "mac", MAC_B },
};

/* 打包选项：缺省为本机真块的样子（版本 1、类型 0、内层头声明 BLOB） */
struct pack_opt {
	u32 enctype;
	u32 hdr_enctype;	/* 写进块头的类型；与 enctype 不同即用错密钥 */
	s32 version;
	u32 declared;
};

static const struct pack_opt def_opt = { 0, 0, 1, BLOB };

struct uk_env {
	u8 *disk;
	u8 *blk[2];
	struct udevice *bm;
	struct udevice *sm;
	struct blk_desc *desc;
	char dev[24];
	int devnum;
	u32 crc;
};

/* 静态而不是 malloc：用例失败提前返回时不漏掉 16 MiB，后面的用例照样能跑 */
static u8 uk_disk[DISK_SIZE] __aligned(ARCH_DMA_MINALIGN);
static u8 uk_blk[2][BLOB] __aligned(ARCH_DMA_MINALIGN);

static u32 put_val(u8 *p, u32 tag, u32 val)
{
	put_unaligned_le32(tag, p);
	put_unaligned_le32(4, p + 4);
	put_unaligned_le32(val, p + 8);

	return 12;
}

static u32 put_buf(u8 *p, u32 tag, const void *val, u32 len)
{
	u32 n = ALIGN(len, 4);

	put_unaligned_le32(tag, p);
	put_unaligned_le32(n, p + 4);
	memset(p + 8, '\0', n);
	memcpy(p + 8, val, len);

	return 8 + n;
}

/* 按厂商 BL31 的格式打包一块：块头 + 两条各自从 IV 起的 CBC 链（内层头 0x200 B、TLV 区补齐到 16 B）+ 块头 SHA-256 */
static void pack(u8 *blk, const struct tkey *keys, int n, const struct pack_opt *o)
{
	u8 key[AES256_KEY_LENGTH], iv[AES_BLOCK_LENGTH], chain[AES_BLOCK_LENGTH];
	u8 exp[AES256_EXPAND_KEY_LENGTH], sha[SHA256_SUM_LEN];
	u32 total = 0, len, i;
	u8 *plain, *tlv, *p, *q;

	plain = calloc(1, BLOB);
	tlv = plain + 0x200;
	for (i = 0; i < n; i++) {
		const struct tkey *k = &keys[i];

		len = k->len ? k->len : strlen(k->val);
		sha256_csum_wd((const u8 *)k->val, len, sha, CHUNKSZ_SHA256);
		p = tlv + total;
		q = p + 8;
		q += put_val(q, 4, strlen(k->name));
		q += put_buf(q, 5, k->name, strlen(k->name));
		q += put_val(q, 6, len);
		q += put_buf(q, 7, k->val, len);
		q += put_buf(q, 0xa, sha, sizeof(sha));
		q += put_val(q, 8, 0xa00000bf);
		q += put_val(q, 9, k->attr);
		put_unaligned_le32(3, p);
		put_unaligned_le32(q - p - 8, p + 4);
		total += q - p;
	}
	put_val(plain, 1, 0);
	put_unaligned_le32(0x18, plain + 4);
	put_val(plain + 8, 2, total);
	put_val(plain + 20, 0xb, o->declared);

	memset(blk, '\0', BLOB);
	memcpy(blk, "AMLSECURITY", 12);
	put_unaligned_le32(o->version, blk + 0x10);
	put_unaligned_le32(o->hdr_enctype, blk + 0x14);
	sandbox_meson_sm_key(o->enctype, key, iv);
	aes_expand_key(key, AES256_KEY_LENGTH, exp);
	memcpy(chain, iv, sizeof(chain));
	aes_cbc_encrypt_blocks(AES256_KEY_LENGTH, exp, chain, plain, blk + 0x200,
			       0x200 / AES_BLOCK_LENGTH);
	memcpy(chain, iv, sizeof(chain));
	aes_cbc_encrypt_blocks(AES256_KEY_LENGTH, exp, chain, tlv, blk + 0x400,
			       ALIGN(total, AES_BLOCK_LENGTH) / AES_BLOCK_LENGTH);
	sha256_csum_wd(blk + 0x200, BLOB - 0x200, blk + 0x18, CHUNKSZ_SHA256);
	free(plain);
}

static u64 sum64(const u8 *p)
{
	u64 sum = 0;
	u32 i;

	for (i = 0; i < BLOB; i += 4)
		sum += get_unaligned_le32(p + i);

	return sum;
}

/* 把 @blk 作为副本 @i 放进 reserved 分区，info 按它算 */
static void put_copy(struct uk_env *e, int i, const u8 *blk, u32 stamp, u32 magic)
{
	u8 *info = e->disk + RSV + INFO_OFF + i * 512;

	memcpy(e->disk + RSV + KEY_OFF + i * BLOB, blk, BLOB);
	memset(info, '\0', 512);
	put_unaligned_le64(sum64(blk), info);
	put_unaligned_le32(stamp, info + 8);
	put_unaligned_le32(magic, info + 12);
}

static u8 *copy_at(struct uk_env *e, int i)
{
	return e->disk + RSV + KEY_OFF + i * BLOB;
}

/* MPT v1（与 M12 出厂表同样的排法）：bootloader、reserved（表在它起点）、data */
static void mpt_write(u8 *disk, u64 rsv_size)
{
	static const char *const names[] = { "bootloader", "reserved", "data" };
	u64 offs[] = { 0, RSV, RSV + rsv_size };
	u64 sizes[] = { SZ_4M, rsv_size, DISK_SIZE - RSV - rsv_size };
	u8 *t = disk + MPT_OFF;
	u32 sum = 0;
	int i;

	memset(t, '\0', SZ_16K);
	memcpy(t, "MPT", 4);
	strcpy((char *)t + 4, "01.00.00");
	put_unaligned_le32(ARRAY_SIZE(names), t + 16);
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		strncpy((char *)t + 24 + i * 40, names[i], 16);
		put_unaligned_le64(sizes[i], t + 24 + i * 40 + 16);
		put_unaligned_le64(offs[i], t + 24 + i * 40 + 24);
	}
	for (i = 0; i < 10; i++)
		sum += get_unaligned_le32(t + 24 + i * 4);
	put_unaligned_le32(ARRAY_SIZE(names) * sum, t + 20);
}

static int uk_setup(struct unit_test_state *uts, struct uk_env *e, u64 rsv_size)
{
	struct udevice *blk;

	memset(e, '\0', sizeof(*e));
	e->disk = uk_disk;
	e->blk[0] = uk_blk[0];
	e->blk[1] = uk_blk[1];
	memset(e->disk, '\0', DISK_SIZE);
	mpt_write(e->disk, rsv_size);
	ut_assertok(blkmap_create("unifykey", &e->bm));
	ut_assertok(blkmap_map_mem(e->bm, 0, DISK_SIZE / BLKSZ, e->disk));
	ut_assertok(blk_get_from_parent(e->bm, &blk));
	e->desc = dev_get_uclass_plat(blk);
	blkcache_invalidate(e->desc->uclass_id, e->desc->devnum);
	part_init(e->desc);
	ut_asserteq(PART_TYPE_AML, e->desc->part_type);
	e->devnum = e->desc->devnum;
	snprintf(e->dev, sizeof(e->dev), "%d#reserved", e->devnum);
	ut_assertok(uclass_get_device_by_driver(UCLASS_SM,
						DM_DRIVER_GET(sandbox_meson_sm), &e->sm));
	console_record_reset();

	return 0;
}

/* 副本放好之后、执行命令之前：记下整块盘；盘是测试直接改的内存，块缓存里可能还有上一轮读过的块 */
static void uk_seal(struct uk_env *e)
{
	blkcache_invalidate(e->desc->uclass_id, e->desc->devnum);
	e->crc = crc32(0, e->disk, DISK_SIZE);
}

static int uk_teardown(struct unit_test_state *uts, struct uk_env *e)
{
	ut_asserteq(0, sandbox_meson_sm_unknown_calls(e->sm));
	ut_asserteq(e->crc, crc32(0, e->disk, DISK_SIZE));
	ut_assertok(blkmap_destroy(e->bm));

	return 0;
}

static int run_load(struct uk_env *e)
{
	return run_commandf("unifykey load blkmap %s", e->dev);
}

static int assert_copy_line(struct unit_test_state *uts, int i, u32 stamp,
			    const u8 *blk, const char *verdict)
{
	ut_assert_nextline("unifykey: copy %d: stamp 0x%x, magic 9, checksum 0x%llx, %s",
			   i, stamp, sum64(blk), verdict);

	return 0;
}

/* 读 mac 的值，证明 BL31 装进的是哪一份 */
static int assert_mac(struct unit_test_state *uts, const char *mac)
{
	ut_assertok(run_command("unifykey read mac " __stringify(MEM_ADDR) " str", 0));
	ut_asserteq_str(mac, env_get("mac"));
	console_record_reset();

	return 0;
}

/* 正例：两份相同、有效的副本，三种加密类型各一遍；list、read（含 str）、只读放内存 */
static int dm_test_unifykey_load(struct unit_test_state *uts)
{
	static const u32 lens[] = { 17, 22, 64, 11, 3, 6, 17 };
	struct uk_env e;
	struct pack_opt o = def_opt;
	char *mem;
	int t, i;

	ut_assertok(uk_setup(uts, &e, SZ_1M));
	for (t = 0; t < 3; t++) {
		o.enctype = o.hdr_enctype = t;
		pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
		put_copy(&e, 0, e.blk[0], 0x20, 9);
		put_copy(&e, 1, e.blk[0], 0x20, 9);
		uk_seal(&e);

		ut_assertok(run_load(&e));
		ut_assertok(assert_copy_line(uts, 0, 0x20, e.blk[0], "ok"));
		ut_assertok(assert_copy_line(uts, 1, 0x20, e.blk[0], "ok"));
		ut_assert_nextline("unifykey: loading copy 0");
		ut_assert_nextline("unifykey: block header: version 1, encryption type %d, SHA-256 ok", t);
		ut_assert_nextline("unifykey: BL31 loaded 7 keys");
		ut_assert_console_end();

		ut_assertok(run_command("unifykey list", 0));
		for (i = 0; i < ARRAY_SIZE(keys_a); i++)
			ut_assert_nextline("%-24s %6u%s", keys_a[i].name, lens[i],
					   keys_a[i].attr ? "  secure" : "");
		ut_assert_nextline("unifykey: 7 keys");
		ut_assert_console_end();
	}

	/* 分区号写法（reserved 是 MPT 第 2 项）同样可用 */
	ut_assertok(run_commandf("unifykey load blkmap %d:2", e.devnum));
	ut_assert_skip_to_line("unifykey: BL31 loaded 7 keys");
	ut_assert_console_end();

	/* str：值进同名变量，数据也落在地址上，filesize 是长度 */
	env_set("mac", NULL);
	ut_assertok(run_command("unifykey read mac " __stringify(MEM_ADDR) " str", 0));
	ut_assert_console_end();
	ut_asserteq_str(MAC_A, env_get("mac"));
	ut_asserteq(17, env_get_hex("filesize", 0));
	mem = map_sysmem(MEM_ADDR, 64);
	ut_asserteq_mem(MAC_A, mem, 17);
	ut_assertok(run_command("unifykey read deviceid " __stringify(MEM_ADDR) " str", 0));
	ut_asserteq_str(DEVID, env_get("deviceid"));
	ut_assertok(run_command("unifykey read authkey " __stringify(MEM_ADDR) " str", 0));
	ut_asserteq_str(AUTHKEY, env_get("authkey"));
	ut_asserteq_mem(AUTHKEY, mem, 64);

	/* 不带 str：只落内存，不碰同名变量 */
	env_set("deviceid", "untouched");
	memset(mem, 0xee, 64);
	ut_assertok(run_command("unifykey read deviceid " __stringify(MEM_ADDR), 0));
	ut_asserteq_mem(DEVID, mem, 22);
	ut_asserteq(0xee, (u8)mem[22]);
	ut_asserteq(22, env_get_hex("filesize", 0));
	ut_asserteq_str("untouched", env_get("deviceid"));
	unmap_sysmem(mem);
	ut_assert_console_end();
	env_set("mac", NULL);
	env_set("deviceid", NULL);
	env_set("authkey", NULL);

	return uk_teardown(uts, &e);
}
DM_TEST(dm_test_unifykey_load, UTF_SCAN_FDT | UTF_CONSOLE);

/* 选副本，与厂商 mmc_key_read 相同：副本 0 内容是 A、副本 1 是 B，读 mac 判断交给 BL31 的是哪份 */
static int dm_test_unifykey_copies(struct unit_test_state *uts)
{
	static const struct {
		const char *what;
		u32 stamp0, stamp1, magic0, magic1;
		int corrupt;	/* -1 不改；否则改坏该副本的一个字节（info 不变） */
		const char *line0, *line1, *note;
		int pick;
	} cases[] = {
		{ "副本 0 校验和不符", 0x20, 0x20, 9, 9, 0, NULL, "ok", NULL, 1 },
		{ "副本 1 校验和不符", 0x20, 0x20, 9, 9, 1, "ok", NULL, NULL, 0 },
		{ "副本 1 更新", 0x20, 0x21, 9, 9, -1, "ok", "ok", NULL, 1 },
		{ "副本 0 更新", 0x21, 0x20, 9, 9, -1, "ok", "ok", NULL, 0 },
		{ "stamp 有符号回绕：0xffffffff 之后是 0", 0xffffffff, 0, 9, 9, -1, "ok", "ok", NULL, 1 },
		{ "stamp 差恰为 0x80000000：厂商 (s32)(stamp0 - stamp1) < 0 取副本 1", 0x80000000, 0, 9, 9, -1, "ok", "ok", NULL, 1 },
		{ "stamp 相同、内容不同取副本 0", 0x20, 0x20, 9, 9, -1, "ok", "ok", NULL, 0 },
		{ "副本 0 magic 为 0", 0x20, 0x20, 0, 9, -1, "bad (magic 0)", "ok", NULL, 1 },
		{ "两份 magic 都为 0、内容完好：照厂商交副本 0，BL31 照常装载", 0x20, 0x20, 0, 0, -1, "bad (magic 0)", "bad (magic 0)",
		  "unifykey: no valid copy, handing copy 0 to BL31 as the vendor u-boot does", 0 },
	};
	struct pack_opt o = def_opt;
	struct uk_env e;
	int i, j;

	ut_assertok(uk_setup(uts, &e, SZ_1M));
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	pack(e.blk[1], keys_b, ARRAY_SIZE(keys_b), &o);
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		console_record_reset();
		put_copy(&e, 0, e.blk[0], cases[i].stamp0, cases[i].magic0);
		put_copy(&e, 1, e.blk[1], cases[i].stamp1, cases[i].magic1);
		if (cases[i].corrupt >= 0)
			copy_at(&e, cases[i].corrupt)[0x1000] ^= 0x5a;
		uk_seal(&e);

		ut_assertok(run_load(&e));
		for (j = 0; j < 2; j++) {
			const char *line = j ? cases[i].line1 : cases[i].line0;
			u32 stamp = j ? cases[i].stamp1 : cases[i].stamp0;
			u32 magic = j ? cases[i].magic1 : cases[i].magic0;
			const u8 *on_disk = copy_at(&e, j);

			if (line)
				ut_assert_nextline("unifykey: copy %d: stamp 0x%x, magic %u, checksum 0x%llx, %s",
						   j, stamp, magic, sum64(e.blk[j]), line);
			else
				ut_assert_nextline("unifykey: copy %d: stamp 0x%x, magic %u, checksum 0x%llx, bad (computed 0x%llx)",
						   j, stamp, magic, sum64(e.blk[j]), sum64(on_disk));
		}
		ut_assert_nextline("unifykey: the two copies differ (left as they are: nothing is written)");
		if (cases[i].note)
			ut_assert_nextline(cases[i].note);
		ut_assert_nextline("unifykey: loading copy %d", cases[i].pick);
		ut_assert_skip_to_line("unifykey: BL31 loaded %d keys",
				       cases[i].pick ? 1 : 7);
		ut_assert_console_end();
		ut_assertok(assert_mac(uts, cases[i].pick ? MAC_B : MAC_A));
	}
	env_set("mac", NULL);

	return uk_teardown(uts, &e);
}
DM_TEST(dm_test_unifykey_copies, UTF_SCAN_FDT | UTF_CONSOLE);

/* BL31 不收的块：每种都要说清原因、命令失败、表是空的；块大小填错之后再给好块能恢复（客户端不因 BL31 改记的块大小拒绝装载） */
static int dm_test_unifykey_rejected(struct unit_test_state *uts)
{
	struct pack_opt o = def_opt;
	struct uk_env e;

	ut_assertok(uk_setup(uts, &e, SZ_1M));

	/* 两份都坏：照厂商交副本 0，BL31 的块头 SHA-256 不符 */
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	copy_at(&e, 0)[0x1000] ^= 1;
	copy_at(&e, 1)[0x1000] ^= 1;
	uk_seal(&e);
	ut_asserteq(1, run_load(&e));
	/* 模型（BL31）在装载时打印，命令在装载返回后才打印经过；两份坏得一样，不报不一致 */
	ut_assert_nextline("ERROR:   !!!!!storage data is corrupt!");
	ut_assert_nextline("unifykey: copy 0: stamp 0x20, magic 9, checksum 0x%llx, bad (computed 0x%llx)",
			   sum64(e.blk[0]), sum64(copy_at(&e, 0)));
	ut_assert_nextline("unifykey: copy 1: stamp 0x20, magic 9, checksum 0x%llx, bad (computed 0x%llx)",
			   sum64(e.blk[0]), sum64(copy_at(&e, 1)));
	ut_assert_nextline("unifykey: no valid copy, handing copy 0 to BL31 as the vendor u-boot does");
	ut_assert_nextline("unifykey: loading copy 0");
	ut_assert_nextline("unifykey: block header: version 1, encryption type 0, SHA-256 mismatch");
	ut_assert_nextline("unifykey: BL31 rejected the block: SHA-256 mismatch");
	ut_assert_console_end();
	ut_assertok(run_command("unifykey list", 0));
	ut_assert_nextline("unifykey: 0 keys");
	ut_asserteq(1, run_command("unifykey read mac " __stringify(MEM_ADDR) " str", 0));
	ut_assert_nextline("unifykey: mac: no such key");
	ut_assert_console_end();

	/* 魔数不对 */
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	e.blk[0][0] = 'X';
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	uk_seal(&e);
	ut_asserteq(1, run_load(&e));
	ut_assert_skip_to_line("unifykey: block header: bad magic");
	ut_assert_nextline("unifykey: BL31 did not recognise the block (storage version -1)");
	ut_assert_console_end();

	/* 内层头声明的块大小与交给 BL31 的不同：BL31 改记该大小、静默清表，只能从读回的块大小看出来 */
	o.declared = SZ_128K;
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	uk_seal(&e);
	ut_asserteq(1, run_load(&e));
	ut_assert_skip_to_line("unifykey: block header: version 1, encryption type 0, SHA-256 ok");
	ut_assert_nextline("unifykey: BL31 block size is 0x20000 after loading, not 0x40000: the inner header declares another size and BL31 dropped every key");
	ut_assert_console_end();
	o.declared = BLOB;
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	uk_seal(&e);
	ut_assertok(run_load(&e));
	ut_assert_skip_to_line("unifykey: BL31 loaded 7 keys");
	ut_assert_console_end();

	/* 块头写的加密类型与加密时用的不同：BL31 用错密钥，内层头解析失败，表空 */
	o.hdr_enctype = 2;
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	uk_seal(&e);
	ut_asserteq(1, run_load(&e));
	ut_assert_skip_to_line("unifykey: block header: version 1, encryption type 2, SHA-256 ok");
	ut_assert_nextline("unifykey: BL31 has no keys after loading (decryption with this chip's key or the key records failed)");
	ut_assert_console_end();
	ut_assertok(run_command("unifykey list", 0));
	ut_assert_nextline("unifykey: 0 keys");
	o.hdr_enctype = 0;

	/* 块头版本 0：BL31 不验 SHA-256，块头哈希错了照样装载 */
	o.version = 0;
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	e.blk[0][0x18] ^= 1;
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	uk_seal(&e);
	ut_assertok(run_load(&e));
	ut_assert_skip_to_line("unifykey: block header: version 0, encryption type 0, SHA-256 not checked by BL31 (version <= 0)");
	ut_assert_nextline("unifykey: BL31 loaded 7 keys");
	ut_assert_console_end();

	/* 分区放不下 key 区（OFFSET + 2 份 + info）：不读、不交 BL31，BL31 里上面装好的表原样留着 */
	ut_assertok(uk_teardown(uts, &e));
	ut_assertok(uk_setup(uts, &e, INFO_OFF));
	uk_seal(&e);
	ut_asserteq(1, run_load(&e));
	ut_assert_nextline("unifykey: the partition cannot hold the key area at 0x%x (2 copies of 0x%x and their info) in whole blocks",
			   KEY_OFF, BLOB);
	ut_assert_console_end();
	ut_assertok(run_command("unifykey list", 0));
	ut_assert_skip_to_line("unifykey: 7 keys");

	return uk_teardown(uts, &e);
}
DM_TEST(dm_test_unifykey_rejected, UTF_SCAN_FDT | UTF_CONSOLE);

static int unifykey_efuse_cases(struct unit_test_state *uts, const u8 *saved)
{
	u8 other[SANDBOX_MESON_SM_EFUSE_4_16_LEN];
	struct pack_opt o = def_opt;
	struct uk_env e;
	int t, i;

	for (i = 0; i < sizeof(other); i++)
		other[i] = saved[sizeof(other) - 1 - i];
	ut_assert(memcmp(other, saved, sizeof(other)));
	ut_assertok(uk_setup(uts, &e, SZ_1M));
	for (t = 0; t < 3; t++) {
		o.enctype = o.hdr_enctype = t;
		sandbox_meson_sm_set_efuse_4_16(other);
		pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
		put_copy(&e, 0, e.blk[0], 0x20, 9);
		put_copy(&e, 1, e.blk[0], 0x20, 9);
		uk_seal(&e);
		ut_assertok(run_load(&e));
		ut_assert_skip_to_line("unifykey: BL31 loaded 7 keys");
		ut_assert_console_end();
		ut_assertok(assert_mac(uts, MAC_A));

		sandbox_meson_sm_set_efuse_4_16(saved);
		if (t == 2) {
			ut_assertok(run_load(&e));
			ut_assert_skip_to_line("unifykey: BL31 loaded 7 keys");
			ut_assert_console_end();
			ut_assertok(assert_mac(uts, MAC_A));
		} else {
			ut_asserteq(1, run_load(&e));
			ut_assert_skip_to_line("unifykey: block header: version 1, encryption type %d, SHA-256 ok", t);
			ut_assert_nextline("unifykey: BL31 has no keys after loading (decryption with this chip's key or the key records failed)");
			ut_assert_console_end();
		}
	}
	env_set("mac", NULL);

	return uk_teardown(uts, &e);
}

/*
 * eFuse 字节 [4,16) 换了值（命令行 --meson_sm_efuse_4_16 给真机的值时就是这样）：模型按设定的值派生类型 0、1 的密钥，类型 2 是固定密钥、与它无关。每种类型先在另一个值（原值逆序）下打包并装载，应装得上；再换回原值装载同一块，类型 0、1 解不开、表空，类型 2 照常装上。用例结束时（含失败提前返回）换回原值
 */
static int dm_test_unifykey_efuse(struct unit_test_state *uts)
{
	u8 saved[SANDBOX_MESON_SM_EFUSE_4_16_LEN];
	int ret;

	memcpy(saved, sandbox_meson_sm_efuse_4_16(), sizeof(saved));
	ret = unifykey_efuse_cases(uts, saved);
	sandbox_meson_sm_set_efuse_4_16(saved);

	return ret;
}
DM_TEST(dm_test_unifykey_efuse, UTF_SCAN_FDT | UTF_CONSOLE);

/* 读不到的情形：不存在、安全属性、名字长度、缓冲太小、str 遇到 ≥ 0x80 的字节；失败时内存、环境变量与 filesize 都不动 */
static int dm_test_unifykey_read_errors(struct unit_test_state *uts)
{
	char name80[81], buf[32];
	struct pack_opt o = def_opt;
	struct uk_env e;
	u32 len, attr;
	u8 *mem;

	ut_assertok(uk_setup(uts, &e, SZ_1M));
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	uk_seal(&e);
	ut_assertok(run_load(&e));
	console_record_reset();

	/* 客户端接口 */
	memset(buf, 0xaa, sizeof(buf));
	ut_asserteq(-E2BIG, meson_unifykey_read("mac", buf, 16, &len));
	ut_asserteq(17, len);
	ut_asserteq(0xaa, (u8)buf[0]);
	ut_assertok(meson_unifykey_read("mac", buf, 17, &len));
	ut_asserteq_mem(MAC_A, buf, 17);
	ut_asserteq(-ENOENT, meson_unifykey_read("nosuch", buf, sizeof(buf), &len));
	ut_asserteq(-ENOENT, meson_unifykey_read("ma", buf, sizeof(buf), &len));
	ut_asserteq(-EACCES, meson_unifykey_read("hdcp", buf, sizeof(buf), &len));
	memset(name80, 'k', 80);
	name80[79] = '\0';
	ut_asserteq(-ENOENT, meson_unifykey_read(name80, buf, sizeof(buf), &len));
	name80[79] = 'k';
	name80[80] = '\0';
	ut_asserteq(-EINVAL, meson_unifykey_read(name80, buf, sizeof(buf), &len));
	ut_asserteq(-EINVAL, meson_unifykey_read("", buf, sizeof(buf), &len));
	ut_assertok(meson_unifykey_stat("hdcp", &len, &attr));
	ut_asserteq(11, len);
	ut_asserteq(MESON_UNIFYKEY_ATTR_SECURE, attr);

	/* 命令：失败不写内存、不设变量、不改 filesize */
	mem = map_sysmem(MEM_ADDR, 16);
	memset(mem, 0xee, 16);
	env_set("filesize", "1234");
	env_set("nonascii", NULL);
	ut_asserteq(1, run_command("unifykey read nosuch " __stringify(MEM_ADDR) " str", 0));
	ut_assert_nextline("unifykey: nosuch: no such key");
	ut_asserteq(1, run_command("unifykey read hdcp " __stringify(MEM_ADDR), 0));
	ut_assert_nextline("unifykey: hdcp: secure key, not readable from the normal world");
	ut_asserteq(1, run_commandf("unifykey read %s " __stringify(MEM_ADDR), name80));
	ut_assert_nextline("unifykey: %s: name must be 1..79 bytes", name80);
	ut_asserteq(1, run_command("unifykey read nonascii " __stringify(MEM_ADDR) " str", 0));
	ut_assert_nextline("unifykey: nonascii: byte 0x80 at offset 2 is not ASCII, not set");
	/* 地址变量为空时 str 顶到地址的位置：拒绝，不当成地址 0 */
	ut_asserteq(1, run_command("unifykey read mac str", 0));
	ut_assert_nextline("unifykey: str is not an address");
	console_record_reset();
	ut_assert_console_end();
	ut_asserteq(0xee, mem[0]);
	ut_assertnull(env_get("nonascii"));
	ut_assertnull(env_get("nosuch"));
	ut_asserteq_str("1234", env_get("filesize"));
	/* 不带 str 时不查字节 */
	ut_assertok(run_command("unifykey read nonascii " __stringify(MEM_ADDR), 0));
	ut_asserteq_mem("ab\x80", mem, 3);
	ut_asserteq(3, env_get_hex("filesize", 0));
	ut_assert_console_end();
	unmap_sysmem(mem);

	return uk_teardown(uts, &e);
}
DM_TEST(dm_test_unifykey_read_errors, UTF_SCAN_FDT | UTF_CONSOLE);

/* ethaddr：17 字节文本与 6 字节两种存法；ethaddr 是写一次的变量，普通 setenv 改不了，出厂 MAC 强制取代；不合格的值不改它 */
static int dm_test_unifykey_ethaddr(struct unit_test_state *uts)
{
	struct pack_opt o = def_opt;
	char old[20], *flags;
	struct uk_env e;
	bool had;

	ut_assertok(uk_setup(uts, &e, SZ_1M));
	pack(e.blk[0], keys_a, ARRAY_SIZE(keys_a), &o);
	put_copy(&e, 0, e.blk[0], 0x20, 9);
	put_copy(&e, 1, e.blk[0], 0x20, 9);
	uk_seal(&e);
	ut_assertok(run_load(&e));
	console_record_reset();

	/*
	 * 与实机一样让 ethaddr 是写一次的变量：经 .flags 显式设定（别的用例改过 .flags 之后，已有的 ethaddr 会失去按通配符给的 mo 标志），不在就先建一个（写一次的变量允许新建，与 meson 板 misc_init_r 设过的情形相同）
	 */
	flags = env_get(".flags") ? strdup(env_get(".flags")) : NULL;
	ut_assertok(env_set(".flags", "ethaddr:mo"));
	had = env_get("ethaddr");
	if (!had)
		ut_assertok(env_set("ethaddr", "02:00:00:00:00:01"));
	strlcpy(old, env_get("ethaddr"), sizeof(old));
	ut_asserteq(1, run_command("setenv ethaddr 02:00:00:00:00:99", 0));
	ut_asserteq_str(old, env_get("ethaddr"));
	console_record_reset();

	ut_assertok(run_command("unifykey ethaddr", 0));
	ut_assert_nextline("unifykey: ethaddr 14:80:1f:cd:fe:50 (from key mac)");
	ut_assert_console_end();
	ut_asserteq_str("14:80:1f:cd:fe:50", env_get("ethaddr"));

	ut_assertok(run_command("unifykey ethaddr mac_raw", 0));
	ut_assert_nextline("unifykey: ethaddr 02:11:22:33:44:55 (from key mac_raw)");
	ut_asserteq_str("02:11:22:33:44:55", env_get("ethaddr"));

	ut_asserteq(1, run_command("unifykey ethaddr mac_mcast", 0));
	ut_assert_nextline("unifykey: mac_mcast: 01:00:5e:00:00:01 is not a valid unicast address");
	ut_asserteq(1, run_command("unifykey ethaddr deviceid", 0));
	ut_assert_nextline("unifykey: deviceid: 22 bytes, not a MAC address");
	ut_asserteq(1, run_command("unifykey ethaddr authkey", 0));
	ut_assert_nextline("unifykey: authkey: 64 bytes, not a MAC address");
	ut_asserteq(1, run_command("unifykey ethaddr nonascii", 0));
	ut_assert_nextline("unifykey: nonascii: 3 bytes, not a MAC address");
	ut_asserteq(1, run_command("unifykey ethaddr nosuch", 0));
	ut_assert_nextline("unifykey: nosuch: no such key");
	ut_assert_console_end();
	ut_asserteq_str("02:11:22:33:44:55", env_get("ethaddr"));

	if (had)
		ut_assertok(run_commandf("env set -f ethaddr %s", old));
	else
		ut_assertok(run_command("env set -f ethaddr", 0));
	ut_assertok(env_set(".flags", flags));
	free(flags);

	return uk_teardown(uts, &e);
}
DM_TEST(dm_test_unifykey_ethaddr, UTF_SCAN_FDT | UTF_CONSOLE);

/* 安全监控没有存储调用（上游 TF-A 一类的 BL31）：报告不支持，不读盘也不发装载 */
static int dm_test_unifykey_no_storage(struct unit_test_state *uts)
{
	struct uk_env e;

	ut_assertok(uk_setup(uts, &e, SZ_1M));
	uk_seal(&e);
	sandbox_meson_sm_set_storage(e.sm, false);
	ut_asserteq(1, run_load(&e));
	ut_assert_nextline("unifykey: the secure monitor has no Amlogic secure storage calls (not the vendor BL31?)");
	ut_asserteq(1, run_command("unifykey list", 0));
	ut_assert_nextline("unifykey: the secure monitor has no Amlogic secure storage calls (not the vendor BL31?)");
	ut_asserteq(1, run_command("unifykey read mac " __stringify(MEM_ADDR), 0));
	ut_assert_nextline("unifykey: the secure monitor has no Amlogic secure storage calls (not the vendor BL31?)");
	ut_assert_console_end();

	/* 不带分区、空分区号与 auto 都会落到第 1 个或第一个可启动分区（厂商 eMMC 上是 bootloader），拒绝 */
	sandbox_meson_sm_set_storage(e.sm, true);
	ut_asserteq(1, run_commandf("unifykey load blkmap %d", e.devnum));
	ut_assert_nextline("unifykey: name the partition, e.g. mmc 0#reserved");
	console_record_reset();
	ut_asserteq(1, run_commandf("unifykey load blkmap %d:", e.devnum));
	ut_assert_nextline("unifykey: name the partition, e.g. mmc 0#reserved");
	console_record_reset();
	ut_asserteq(1, run_commandf("unifykey load blkmap %d:auto", e.devnum));
	ut_assert_nextline("unifykey: name the partition, e.g. mmc 0#reserved");

	return uk_teardown(uts, &e);
}
DM_TEST(dm_test_unifykey_no_storage, UTF_SCAN_FDT | UTF_CONSOLE);
