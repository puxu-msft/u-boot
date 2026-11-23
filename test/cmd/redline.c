// SPDX-License-Identifier: GPL-2.0+
/*
 * redline（存储写入类命令的红线确认）的测试
 *
 * 用 sandbox 的内存 MMC（test.dts 的 mmc0、mmc2，各 1 MiB）做真写入：mmc0 上先手写一张 DOS 分区表与已知数据，确认被拦的命令第一次不执行、存储内容不变，redline yes 作为单独的一次输入执行一次、内容变，第二次 yes 报错；同一次输入里（同一行、||、run 变量、多行脚本、source 脚本）的自确认一律拒绝，动作留待下一次输入确认；新被拦的命令替换待确认动作；run、多行脚本（bootcmd 的路径）、source 脚本与命令缩写同样被拦；只读命令照常执行且不动待确认动作；展示后切换 mmc 当前设备或改了源数据，yes 不执行而是重新展示；源数据落在 lmb 带 no-map 的保留区时清单不读它。另用 blkmap 映射一块带 Amlogic MPT 的内存盘，核对清单里的设备、起止块、字节范围与重叠的 MPT 分区名（gzwrite 的范围标到盘尾，文件系统写与 ums 的 <dev>#<名字> 写法）；mmc erase 按擦除组扩大的范围；mmc swrite 按上游逐 chunk 的规则得出的源数据与范围；sandbox 没有 smc 与 sm 命令，用伪造的命令表项直接调 redline_gate() 核对 smc 的 fid 白名单、smc 与 sm 两条 eFuse 写的字节清单（smc 的字节取自安全监控输入共享缓冲，测试覆盖弱函数 redline_sm_input_buffer() 给出一块内存）。
 * 交互解析器读进一行、fastboot 收到一个请求时调 redline_input_next()；测试里的 run_command() 不经读行，用例在需要“下一次输入”的地方显式调它。
 * CMD_REDLINE 关闭时只跑 cmd_test_redline_off：mmc write 第一次就执行、没有 redline 命令，与上游一致。
 */

#include <blk.h>
#include <blkmap.h>
#include <command.h>
#include <env.h>
#include <fastboot.h>
#include <image.h>
#include <lmb.h>
#include <malloc.h>
#include <mapmem.h>
#include <memalign.h>
#include <mmc.h>
#include <part.h>
#include <sparse_format.h>
#include <asm/unaligned.h>
#include <dm/device.h>
#include <linux/sizes.h>
#include <test/cmd.h>
#include <test/test.h>
#include <test/ut.h>
#include <u-boot/crc.h>

#define BLKSZ		512
#define HINT		"Next: 'redline yes' as a separate input runs it once, 'redline show' lists it again, 'redline cancel' drops it"
#define HELD		"REDLINE: storage write held, not executed"
#define SAME		"REDLINE: the held action was listed during this same input (command line, script, bootcmd or fastboot request); not executed, it stays pending for 'redline yes' as a separate input"
#define CHANGED		"REDLINE: the target or the source data changed since the list was shown; not executed, new list follows"
/* 一块全是 0xa5、0x33 的 crc32（Python zlib.crc32 算出，与被测代码无关） */
#define CRC_A5		0xc906d311
#define CRC_33		0xdc20faa6
#define GZ_NOTE		"  note    : gzwrite writes until the deflate stream ends and checks the size only afterwards: the write may run past the declared size, up to the end of the device"
#define PAST		" (only if the write runs past the declared size)"

/* 安全监控输入共享缓冲：sandbox 没有安全监控，用例把一块内存的地址放在这里（0 表示取不到） */
static ulong rl_sm_buf;

#if IS_ENABLED(CONFIG_CMD_REDLINE)
/* 覆盖 cmd/redline.c 的弱函数 */
ulong redline_sm_input_buffer(void)
{
	return rl_sm_buf;
}
#endif

struct rl_env {
	struct blk_desc *mmc0;
	u8 *a, *b, *rd;		/* 两份写入源与一份读回缓冲，各一块 */
	ulong aa, ba, rda;	/* 它们的 sysmem 地址 */
};

/* 一块填 @v */
static void fill(u8 *p, u8 v)
{
	memset(p, v, BLKSZ);
}

/* mmc0 的第 @blk 块是否整块等于 @v（绕过命令层直接读） */
static int blk_is(struct unit_test_state *uts, struct blk_desc *desc,
		  lbaint_t blk, u8 v)
{
	u8 buf[BLKSZ];
	int i;

	ut_asserteq(1, blk_dread(desc, blk, 1, buf));
	for (i = 0; i < BLKSZ; i++)
		ut_asserteq(v, buf[i]);

	return 0;
}

/* mmc0：块 0 写 DOS 分区表（#1 0x20+0x40、#2 0x100+0x100），块 0x40、0x41 写已知数据 0x11；mmc 当前设备设为 0，清掉待确认动作 */
static int rl_setup(struct unit_test_state *uts, struct rl_env *e)
{
	u8 mbr[BLKSZ] = {};
	u8 *p;

	ut_assertok(run_command("mmc dev 0", 0));
	ut_assertok(blk_get_desc(UCLASS_MMC, 0, &e->mmc0));
	p = mbr + 0x1be;
	p[4] = 0x83;
	put_unaligned_le32(0x20, p + 8);
	put_unaligned_le32(0x40, p + 12);
	p += 16;
	p[4] = 0x83;
	put_unaligned_le32(0x100, p + 8);
	put_unaligned_le32(0x100, p + 12);
	mbr[0x1fe] = 0x55;
	mbr[0x1ff] = 0xaa;
	ut_asserteq(1, blk_dwrite(e->mmc0, 0, 1, mbr));
	part_init(e->mmc0);
	ut_asserteq(PART_TYPE_DOS, e->mmc0->part_type);

	e->a = memalign(BLKSZ, 3 * BLKSZ);
	ut_assertnonnull(e->a);
	e->b = e->a + BLKSZ;
	e->rd = e->b + BLKSZ;
	fill(e->a, 0xa5);
	fill(e->b, 0x5a);
	fill(e->rd, 0x11);
	ut_asserteq(1, blk_dwrite(e->mmc0, 0x40, 1, e->rd));
	ut_asserteq(1, blk_dwrite(e->mmc0, 0x41, 1, e->rd));
	e->aa = map_to_sysmem(e->a);
	e->ba = map_to_sysmem(e->b);
	e->rda = map_to_sysmem(e->rd);
	run_command("redline cancel", 0);
	console_record_reset();

	return 0;
}

/* 清掉 mmc0 的分区表，还原 test.dts 的空盘 */
static void rl_teardown(struct rl_env *e)
{
	u8 zero[BLKSZ] = {};

	run_command("redline cancel", 0);
	run_command("mmc dev 0", 0);
	blk_dwrite(e->mmc0, 0, 1, zero);
	part_init(e->mmc0);
	free(e->a);
	console_record_reset();
}

/* 作为单独的一次输入提交 redline yes（交互解析器读进新的一行时同样调 redline_input_next()） */
static int yes(void)
{
	redline_input_next();

	return run_command("redline yes", 0);
}

/* 被拦的 mmc write 0x40 的清单（源 @src，一块内容的 crc32 是 @crc） */
static int expect_held_write40(struct unit_test_state *uts, const char *cmd,
			       ulong src, u32 crc)
{
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : %s", cmd);
	ut_assert_nextline("  action  : MMC block write from %lx", src);
	ut_assert_nextline("  source  : 0x200 bytes at 0x%lx, crc32 0x%08x", src, crc);
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x41 (0x1 blocks)");
	ut_assert_nextline("  bytes   : 0x8000..0x8200");
	ut_assert_nextline("  overlaps: 1 of the 2 partitions");
	ut_assert_nextline("    #1 mmcsda1: blocks 0x20..0x60");
	ut_assert_nextline(HINT);
	ut_assert_console_end();

	return 0;
}

/* 第一次不执行、内容不变；yes 执行一次、内容变；再 yes 报错 */
static int cmd_test_redline_hold(struct unit_test_state *uts)
{
	struct rl_env e;
	char cmd[64];

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	snprintf(cmd, sizeof(cmd), "mmc write %lx 40 1", e.aa);

	ut_asserteq(1, run_command(cmd, 0));
	ut_assertok(expect_held_write40(uts, cmd, e.aa, CRC_A5));
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	/* show 再打印同一份清单，不执行 */
	ut_assertok(run_command("redline show", 0));
	ut_assert_nextline("REDLINE: pending storage write, not executed");
	ut_assert_nextline("  command : %s", cmd);
	ut_assert_skip_to_line(HINT);
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	ut_assertok(yes());
	ut_assert_nextline("redline: running %s", cmd);
	ut_assert_nextline("MMC write: dev # 0, block # 64, count 1 ... 1 blocks written: OK");
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0xa5));

	/* 单次有效：源换成 b 也不会再写 */
	fill(e.a, 0x5a);
	ut_asserteq(1, yes());
	ut_assert_nextline("redline: nothing pending");
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0xa5));

	/* cancel 丢弃待确认动作 */
	ut_asserteq(1, run_command(cmd, 0));
	console_record_reset();
	ut_assertok(run_command("redline cancel", 0));
	ut_assert_nextline("redline: dropped mmc");
	ut_asserteq(1, yes());
	ut_assert_nextline("redline: nothing pending");
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0xa5));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_hold, UTF_CONSOLE);

/* 新被拦的命令替换待确认动作：yes 只执行后一条 */
static int cmd_test_redline_replace(struct unit_test_state *uts)
{
	struct rl_env e;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));

	ut_asserteq(1, run_commandf("mmc write %lx 40 1", e.aa));
	ut_asserteq(1, run_commandf("mmc write %lx 41 1", e.ba));
	console_record_reset();
	ut_assertok(run_command("redline show", 0));
	ut_assert_nextline("REDLINE: pending storage write, not executed");
	ut_assert_nextline("  command : mmc write %lx 41 1", e.ba);
	console_record_reset();

	ut_assertok(yes());
	ut_assert_nextline("redline: running mmc write %lx 41 1", e.ba);
	ut_assert_nextline("MMC write: dev # 0, block # 65, count 1 ... 1 blocks written: OK");
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	ut_assertok(blk_is(uts, e.mmc0, 0x41, 0x5a));
	ut_asserteq(1, yes());
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_replace, UTF_CONSOLE);

/* 待确认动作的命令行，确认它没被别的命令替换或清掉 */
static int expect_pending(struct unit_test_state *uts, const char *fmt, ulong arg)
{
	console_record_reset();
	ut_assertok(run_command("redline show", 0));
	ut_assert_nextline("REDLINE: pending storage write, not executed");
	ut_assert_nextline(fmt, arg);
	console_record_reset();

	return 0;
}

/* 传统格式脚本映像（source 用）：@script 包成 IH_TYPE_SCRIPT，返回映像，调用者 free */
static u8 *script_image(const char *script)
{
	struct legacy_img_hdr *hdr;
	uint slen = strlen(script), dlen = 8 + slen;
	u8 *img;

	img = calloc(1, sizeof(*hdr) + dlen);
	if (!img)
		return NULL;
	hdr = (struct legacy_img_hdr *)img;
	put_unaligned_be32(slen, img + sizeof(*hdr));
	memcpy(img + sizeof(*hdr) + 8, script, slen);
	image_set_magic(hdr, IH_MAGIC);
	image_set_type(hdr, IH_TYPE_SCRIPT);
	image_set_size(hdr, dlen);
	image_set_dcrc(hdr, crc32(0, img + sizeof(*hdr), dlen));
	image_set_hcrc(hdr, crc32(0, img, sizeof(*hdr)));

	return img;
}

/* run、多行脚本、source 脚本与缩写同样被拦 */
static int cmd_test_redline_paths(struct unit_test_state *uts)
{
	struct rl_env e;
	char script[64];
	u8 *img;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));

	/* run <变量> */
	snprintf(script, sizeof(script), "mmc write %lx 40 1", e.aa);
	ut_assertok(env_set("rl_cmd", script));
	ut_asserteq(1, run_command("run rl_cmd", 0));
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : %s", script);
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	ut_assertok(expect_pending(uts, "  command : mmc write %lx 40 1", e.aa));
	ut_assertok(run_command("redline cancel", 0));

	/* 多行脚本（bootcmd、fastboot oem run 经 run_command_list/run_command 走同一处） */
	snprintf(script, sizeof(script), "echo before\nmmc write %lx 41 1\necho after\n",
		 e.ba);
	console_record_reset();
	run_command_list(script, -1, 0);
	ut_assert_nextline("before");
	ut_assert_nextline(HELD);
	ut_assert_skip_to_line(HINT);
	ut_assert_nextline("after");
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x41, 0x11));
	ut_assertok(expect_pending(uts, "  command : mmc write %lx 41 1", e.ba));
	ut_assertok(run_command("redline cancel", 0));

	/* source：传统格式脚本映像 */
	if (IS_ENABLED(CONFIG_CMD_SOURCE) && IS_ENABLED(CONFIG_LEGACY_IMAGE_FORMAT)) {
		snprintf(script, sizeof(script), "mmc write %lx 40 1\n", e.aa);
		img = script_image(script);
		ut_assertnonnull(img);
		console_record_reset();
		ut_asserteq(1, run_commandf("source %lx", (ulong)map_to_sysmem(img)));
		ut_assert_skip_to_line(HELD);
		ut_assert_nextline("  command : mmc write %lx 40 1", e.aa);
		ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
		free(img);
		ut_assertok(run_command("redline cancel", 0));
	}

	/* 缩写与 .后缀：按命令表的规范名与子命令解析结果判定 */
	console_record_reset();
	ut_asserteq(1, run_commandf("mmc wr %lx 40 1", e.aa));
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : mmc wr %lx 40 1", e.aa);
	ut_asserteq(1, run_commandf("mmc.x write %lx 40 1", e.aa));
	ut_assert_skip_to_line(HELD);
	ut_assert_nextline("  command : mmc.x write %lx 40 1", e.aa);
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	if (IS_ENABLED(CONFIG_FAT_WRITE)) {
		ut_asserteq(1, run_commandf("fatw mmc 0:1 %lx x 10", e.aa));
		ut_assert_skip_to_line(HELD);
		ut_assert_nextline("  command : fatw mmc 0:1 %lx x 10", e.aa);
		ut_assert_nextline("  action  : filesystem write: file x, 10 bytes from %lx", e.aa);
		ut_assert_nextline("  source  : 0x10 bytes at 0x%lx, crc32 0xbbb56b1b", e.aa);
		ut_assert_nextline("  scope   : anywhere inside partition 1 'mmcsda1'");
		ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
		ut_assert_nextline("  blocks  : 0x20..0x60 (0x40 blocks)");
	}
	if (IS_ENABLED(CONFIG_CMD_USB_MASS_STORAGE)) {
		console_record_reset();
		ut_asserteq(1, run_command("ums 0 mmc 0:2", 0));
		ut_assert_nextline(HELD);
		ut_assert_nextline("  command : ums 0 mmc 0:2");
		ut_assert_nextline("  action  : USB mass storage: the USB host gets raw read/write access until the command is stopped");
		ut_assert_nextline("  exposed : partition 2 'mmcsda2' of mmc 0:2");
		ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
		ut_assert_nextline("  blocks  : 0x100..0x200 (0x100 blocks)");
		ut_assert_nextline("  bytes   : 0x20000..0x40000");
		ut_assert_nextline("  overlaps: 1 of the 2 partitions");
		ut_assert_nextline("    #2 mmcsda2: blocks 0x100..0x200");
		ut_assert_nextline(HINT);
		ut_assert_console_end();
	}

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_paths, UTF_CONSOLE);

/* 只读命令照常执行，不替换、不清掉待确认动作 */
static int cmd_test_redline_readonly(struct unit_test_state *uts)
{
	struct rl_env e;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	ut_asserteq(1, run_commandf("mmc write %lx 40 1", e.aa));

	fill(e.rd, 0);
	console_record_reset();
	ut_assertok(run_commandf("mmc read %lx 40 1", e.rda));
	ut_assert_nextline("MMC read: dev # 0, block # 64, count 1 ... 1 blocks read: OK");
	ut_assert_console_end();
	ut_asserteq(0x11, e.rd[0]);
	ut_asserteq(0x11, e.rd[BLKSZ - 1]);
	ut_assertok(run_command("part list mmc 0", 0));
	ut_assert_skip_to_linen("Partition Map for mmc device 0");
	ut_assertok(run_command("mmc dev 0", 0));
	ut_assertok(run_command("mmc part", 0));
	ut_assertok(run_command("mmc list", 0));
	ut_assertok(run_command("mmc info", 0));
	ut_assertok(run_command("mmc rescan", 0));
	/* mmc0:1 不是 FAT：fatls 自己报错，但它确实执行了、没进红线门 */
	if (IS_ENABLED(CONFIG_CMD_FAT))
		run_command("fatls mmc 0:1", 0);
	ut_assertok(expect_pending(uts, "  command : mmc write %lx 40 1", e.aa));
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_readonly, UTF_CONSOLE);

/* 展示后换了 mmc 当前设备：yes 不执行，按新状态重新展示，再 yes 才写到新设备 */
static int cmd_test_redline_changed(struct unit_test_state *uts)
{
	struct blk_desc *mmc2;
	struct rl_env e;
	char cmd[64];

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	ut_assertok(blk_get_desc(UCLASS_MMC, 2, &mmc2));
	fill(e.rd, 0x22);
	ut_asserteq(1, blk_dwrite(mmc2, 0x40, 1, e.rd));
	snprintf(cmd, sizeof(cmd), "mmc write %lx 40 1", e.aa);

	ut_asserteq(1, run_command(cmd, 0));
	ut_assertok(run_command("mmc dev 2", 0));
	console_record_reset();
	ut_asserteq(1, yes());
	ut_assert_nextline(CHANGED);
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : %s", cmd);
	ut_assert_nextline("  action  : MMC block write from %lx", e.aa);
	ut_assert_nextline("  source  : 0x200 bytes at 0x%lx, crc32 0x%08x", e.aa, CRC_A5);
	ut_assert_nextline("  device  : mmc 2, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_skip_to_line(HINT);
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	ut_assertok(blk_is(uts, mmc2, 0x40, 0x22));

	/* 重新展示的清单在这次输入里才第一次给出：同一次输入里再 yes 仍被拒绝 */
	ut_asserteq(1, run_command("redline yes", 0));
	ut_assert_nextline(SAME);
	ut_assert_console_end();
	ut_assertok(yes());
	ut_assert_nextline("redline: running %s", cmd);
	ut_assert_nextline("MMC write: dev # 2, block # 64, count 1 ... 1 blocks written: OK");
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	ut_assertok(blk_is(uts, mmc2, 0x40, 0xa5));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_changed, UTF_CONSOLE);

/* 执行 @cmd，断言它被拦下且清单的动作行是 @action；清单其余行留给调用者核对 */
static int expect_held(struct unit_test_state *uts, const char *cmd,
		       const char *action)
{
	console_record_reset();
	ut_asserteq(1, run_command(cmd, 0));
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : %s", cmd);
	ut_assert_nextline("%s", action);

	return 0;
}

/* 执行 @cmd，断言它没进红线门（待确认动作仍是 mmc write ... 40 1） */
static int expect_passed(struct unit_test_state *uts, struct rl_env *e,
			 const char *cmd)
{
	console_record_reset();
	run_command(cmd, 0);
	ut_assertok(expect_pending(uts, "  command : mmc write %lx 40 1", e->aa));

	return 0;
}

/* 子命令判定：写的、歧义与未知的拦，只读的与 hwpartition 的 check 模式放行 */
static int cmd_test_redline_classify(struct unit_test_state *uts)
{
	struct rl_env e;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	ut_assertok(expect_held(uts, "mmc erase 40 1", "  action  : MMC erase"));
	ut_assertok(expect_held(uts, "mmc erase mmcsda1", "  action  : MMC erase"));
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x20..0x60 (0x40 blocks)");
	console_record_reset();
	ut_assertok(expect_held(uts, "mmc wp", "  action  : power-on write protection of all boot partitions"));
	/* do_mmc_boot_wp() 用 dectoul()：0 与不是数字的写法只保护 boot0，1 只保护 boot1，其余值保护全部 */
	ut_assertok(expect_held(uts, "mmc wp 0", "  action  : power-on write protection of boot partition 0"));
	ut_assertok(expect_held(uts, "mmc wp x", "  action  : power-on write protection of boot partition 0"));
	ut_assertok(expect_held(uts, "mmc wp 1", "  action  : power-on write protection of boot partition 1"));
	ut_assertok(expect_held(uts, "mmc wp 2", "  action  : power-on write protection of all boot partitions"));
	ut_assertok(expect_held(uts, "mmc wp 0 1", "  action  : power-on write protection of nothing (too many arguments: mmc prints its usage)"));
	ut_assertok(expect_held(uts, "mmc hwpartition user enh 0 800 set",
				"  action  : eMMC hardware partitioning (EXT_CSD)"));
	ut_assertok(expect_held(uts, "mmc hwpartition gp1 800 0 0 complete",
				"  action  : eMMC hardware partitioning (EXT_CSD)"));
	ut_assert_skip_to_line("  IRREVERSIBLE: 'complete' sets PARTITION_SETTING_COMPLETED; the layout can never be changed again");
	console_record_reset();
	ut_assertok(expect_held(uts, "mmc w 0 0 0",
				"  action  : 'mmc w' does not resolve to a known read-only subcommand (held: it cannot be classified)"));
	ut_assertok(expect_held(uts, "mmc bogus",
				"  action  : 'mmc bogus' does not resolve to a known read-only subcommand (held: it cannot be classified)"));
	if (IS_ENABLED(CONFIG_CMD_GPT))
		ut_assertok(expect_held(uts, "gpt write mmc 0 name=a,size=64K",
					"  action  : rewrite the gpt partition table ('write')"));
	if (IS_ENABLED(CONFIG_CMD_SAVEENV) && !IS_ENABLED(CONFIG_ENV_IS_DEFAULT))
		ut_assertok(expect_held(uts, "env sa",
					"  action  : save the environment on its storage"));

	/* 放行的：待确认动作保持为最后一条被拦的，先把它换回 mmc write */
	ut_asserteq(1, run_commandf("mmc write %lx 40 1", e.aa));
	ut_assertok(expect_passed(uts, &e, "mmc hwpartition user enh 0 800 check"));
	ut_assertok(expect_passed(uts, &e, "mmc dev 0 0"));
	ut_assertok(expect_passed(uts, &e, "mmc setdsr 404"));
	ut_assertok(expect_passed(uts, &e, "usb info"));
	ut_assertok(expect_passed(uts, &e, "env print rl_none"));
	ut_assertok(expect_passed(uts, &e, "gpt verify mmc 0"));
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_classify, UTF_CONSOLE);

/* Amlogic MPT（与补丁 0201 的测试同一种手写法）：bootloader、reserved（表本身在它起点）、boot、data */
#define MPT_DISK	SZ_16M
#define MPT_OFF		0xc00000
#define MPT_ENT(i)	(MPT_OFF + 24 + (i) * 40)

static void mpt_write(u8 *disk)
{
	static const struct {
		const char *name;
		u64 offset, size;
	} ents[] = {
		{ "bootloader", 0, SZ_4M },
		{ "reserved", MPT_OFF, SZ_1M },
		{ "boot", MPT_OFF + SZ_1M, SZ_1M },
		{ "data", MPT_OFF + SZ_2M, MPT_DISK - MPT_OFF - SZ_2M },
	};
	u32 sum = 0;
	uint i;

	memcpy(disk + MPT_OFF, "MPT", 4);
	strcpy((char *)disk + MPT_OFF + 4, "01.00.00");
	put_unaligned_le32(ARRAY_SIZE(ents), disk + MPT_OFF + 16);
	for (i = 0; i < ARRAY_SIZE(ents); i++) {
		strncpy((char *)disk + MPT_ENT(i), ents[i].name, 16);
		put_unaligned_le64(ents[i].size, disk + MPT_ENT(i) + 16);
		put_unaligned_le64(ents[i].offset, disk + MPT_ENT(i) + 24);
	}
	/* v1 校验和：项数 × 第 0 项 10 个 LE 字之和 */
	for (i = 0; i < 10; i++)
		sum += get_unaligned_le32(disk + MPT_ENT(0) + i * 4);
	put_unaligned_le32(ARRAY_SIZE(ents) * sum, disk + MPT_OFF + 20);
}

/* 清单的设备、范围与重叠分区：MPT 盘上的 gzwrite 与 ums（文件系统写的分区解析在 cmd_test_redline_paths 的 DOS 盘上核对） */
static int cmd_test_redline_describe(struct unit_test_state *uts)
{
	struct udevice *dev, *blk;
	struct blk_desc *desc;
	u8 *disk, *gz;
	ulong gza;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !IS_ENABLED(CONFIG_BLKMAP) ||
	    CONFIG_AML_PARTITION_OFFSET != MPT_OFF)
		return -EAGAIN;
	disk = calloc(1, MPT_DISK);
	ut_assertnonnull(disk);
	mpt_write(disk);
	ut_assertok(blkmap_create("rl", &dev));
	ut_assertok(blkmap_map_mem(dev, 0, MPT_DISK / BLKSZ, disk));
	ut_assertok(blk_get_from_parent(dev, &blk));
	desc = dev_get_uclass_plat(blk);
	part_init(desc);
	ut_asserteq(PART_TYPE_AML, desc->part_type);
	run_command("redline cancel", 0);

	/*
	 * gzwrite：声明的输出长度取 gzip 尾部 ISIZE（这里伪造为 1.5 MiB），从 reserved 起点至少写到 boot 中段；gzwrite() 写到流结束才核对大小，所以范围标到盘尾，只在越过声明长度时才碰到的分区单独注明。
	 * 源数据是 0x100 字节（除 ISIZE 外全 0），crc32 由 Python zlib.crc32 算出
	 */
	gz = calloc(1, 0x100);
	ut_assertnonnull(gz);
	put_unaligned_le32(0x180000, gz + 0x100 - 4);
	gza = map_to_sysmem(gz);
	console_record_reset();
	ut_asserteq(1, run_commandf("gzwrite blkmap %d %lx 100 100000 c00000",
				    desc->devnum, gza));
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : gzwrite blkmap %d %lx 100 100000 c00000",
			   desc->devnum, gza);
	ut_assert_nextline("  action  : decompress gzip data at %lx and write it to the device at byte 0xc00000",
			   gza);
	ut_assert_nextline("  source  : 0x100 bytes at 0x%lx, crc32 0x8f8d1d01", gza);
	ut_assert_nextline("  declared: 0x180000 bytes (gzip trailer ISIZE, modulo 4 GiB)");
	ut_assert_nextline(GZ_NOTE);
	ut_assert_nextline("  device  : blkmap %d, 512-byte blocks, 0x8000 blocks", desc->devnum);
	ut_assert_nextline("  blocks  : 0x6000..0x6c00 at least, possibly up to 0x8000");
	ut_assert_nextline("  bytes   : 0xc00000..0xd80000 at least, possibly up to 0x1000000");
	ut_assert_nextline("  overlaps: 3 of the 4 partitions");
	ut_assert_nextline("    #2 reserved: blocks 0x6000..0x6800");
	ut_assert_nextline("    #3 boot: blocks 0x6800..0x7000");
	ut_assert_nextline("    #4 data: blocks 0x7000..0x8000" PAST);
	ut_assert_nextline(HINT);
	ut_assert_console_end();

	/* 边界：声明长度恰好盖满 reserved，终点等于 boot 的起点：boot 只在越过声明长度时才碰到 */
	put_unaligned_le32(0x100000, gz + 0x100 - 4);
	ut_asserteq(1, run_commandf("gzwrite blkmap %d %lx 100 100000 c00000",
				    desc->devnum, gza));
	ut_assert_nextline(HELD);
	ut_assert_skip_to_line("  declared: 0x100000 bytes (gzip trailer ISIZE, modulo 4 GiB)");
	ut_assert_skip_to_line("  blocks  : 0x6000..0x6800 at least, possibly up to 0x8000");
	ut_assert_nextline("  bytes   : 0xc00000..0xd00000 at least, possibly up to 0x1000000");
	ut_assert_nextline("  overlaps: 3 of the 4 partitions");
	ut_assert_nextline("    #2 reserved: blocks 0x6000..0x6800");
	ut_assert_nextline("    #3 boot: blocks 0x6800..0x7000" PAST);
	ut_assert_nextline("    #4 data: blocks 0x7000..0x8000" PAST);
	ut_assert_nextline(HINT);
	ut_assert_console_end();

	/* 给了 outsize：以它为声明长度，与尾部 ISIZE 的低 32 位不符时 gzwrite 会拒绝 */
	ut_asserteq(1, run_commandf("gzwrite blkmap %d %lx 100 100000 c00000 200000",
				    desc->devnum, gza));
	ut_assert_nextline(HELD);
	ut_assert_skip_to_line("  declared: 0x200000 bytes (outsize)");
	ut_assert_nextline("  note    : outsize does not match the gzip trailer (ISIZE 0x100000); gzwrite will refuse");
	ut_assert_nextline(GZ_NOTE);
	ut_assert_skip_to_line("  blocks  : 0x6000..0x7000 at least, possibly up to 0x8000");
	ut_assert_skip_to_line(HINT);
	ut_assert_console_end();

	/* 文件系统写的 <dev>#<名字>：fs_set_blk_dev() 经 part_get_info_by_dev_and_name_or_num() 按名字找到分区，清单是该分区 */
	if (IS_ENABLED(CONFIG_FAT_WRITE)) {
		ut_asserteq(1, run_commandf("fatwrite blkmap %d#boot %lx x 10",
					    desc->devnum, gza));
		ut_assert_nextline(HELD);
		ut_assert_nextline("  command : fatwrite blkmap %d#boot %lx x 10",
				   desc->devnum, gza);
		ut_assert_nextline("  action  : filesystem write: file x, 10 bytes from %lx", gza);
		ut_assert_nextline("  source  : 0x10 bytes at 0x%lx, crc32 0xecbb4b55", gza);
		ut_assert_nextline("  scope   : anywhere inside partition 3 'boot'");
		ut_assert_nextline("  device  : blkmap %d, 512-byte blocks, 0x8000 blocks", desc->devnum);
		ut_assert_nextline("  blocks  : 0x6800..0x7000 (0x800 blocks)");
		ut_assert_nextline("  bytes   : 0xd00000..0xe00000");
		ut_assert_nextline("  overlaps: 1 of the 4 partitions");
		ut_assert_nextline("    #3 boot: blocks 0x6800..0x7000");
		ut_assert_nextline(HINT);
		ut_assert_console_end();
	}

	/* ums：不带 ':' 的 <dev>#<名字> 与 ums_init() 一样暴露整个设备；带 ':' 的分区写法在 cmd_test_redline_paths 的 DOS 盘上核对 */
	if (IS_ENABLED(CONFIG_CMD_USB_MASS_STORAGE)) {
		ut_asserteq(1, run_commandf("ums 0 blkmap %d#boot", desc->devnum));
		ut_assert_nextline(HELD);
		ut_assert_nextline("  command : ums 0 blkmap %d#boot", desc->devnum);
		ut_assert_nextline("  action  : USB mass storage: the USB host gets raw read/write access until the command is stopped");
		ut_assert_nextline("  exposed : WHOLE DEVICE blkmap %d#boot", desc->devnum);
		ut_assert_nextline("  device  : blkmap %d, 512-byte blocks, 0x8000 blocks", desc->devnum);
		ut_assert_nextline("  blocks  : 0x0..0x8000 (0x8000 blocks)");
		ut_assert_nextline("  bytes   : 0x0..0x1000000");
		ut_assert_nextline("  overlaps: 4 of the 4 partitions");
		ut_assert_nextline("    #1 bootloader: blocks 0x0..0x2000");
		ut_assert_nextline("    #2 reserved: blocks 0x6000..0x6800");
		ut_assert_nextline("    #3 boot: blocks 0x6800..0x7000");
		ut_assert_nextline("    #4 data: blocks 0x7000..0x8000");
		ut_assert_nextline(HINT);
		ut_assert_console_end();
	}

	ut_assertok(run_command("redline cancel", 0));
	ut_assertok(blkmap_destroy(dev));
	free(gz);
	free(disk);
	console_record_reset();

	return 0;
}
CMD_TEST(cmd_test_redline_describe, UTF_CONSOLE);

/* @input 在同一次输入里先被拦、再 redline yes：yes 被拒绝，动作留着、存储不变 */
static int expect_same_input(struct unit_test_state *uts, struct rl_env *e,
			     const char *cmd)
{
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : %s", cmd);
	ut_assert_skip_to_line(HINT);
	ut_assert_nextline(SAME);
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e->mmc0, 0x40, 0x11));
	ut_assertok(expect_pending(uts, "  command : mmc write %lx 40 1", e->aa));

	return 0;
}

/* 同一次输入里的自确认：同一行（; 与 ||）、run 变量、多行脚本（bootcmd/preboot 的路径）、source 脚本都拒绝；单独一次输入的 yes 照常执行 */
static int cmd_test_redline_same_input(struct unit_test_state *uts)
{
	struct rl_env e;
	char cmd[64], script[96];
	u8 *img;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	snprintf(cmd, sizeof(cmd), "mmc write %lx 40 1", e.aa);

	ut_asserteq(1, run_commandf("%s; redline yes", cmd));
	ut_assertok(expect_same_input(uts, &e, cmd));
	ut_asserteq(1, run_commandf("%s || redline yes", cmd));
	ut_assertok(expect_same_input(uts, &e, cmd));

	snprintf(script, sizeof(script), "%s; redline yes", cmd);
	ut_assertok(env_set("rl_cmd", script));
	ut_asserteq(1, run_command("run rl_cmd", 0));
	ut_assertok(expect_same_input(uts, &e, cmd));

	/* run_command_list() 的返回值不反映最后一条命令（hush 的 parse_string_outer()），只看输出与存储 */
	snprintf(script, sizeof(script), "%s\nredline yes\n", cmd);
	run_command_list(script, -1, 0);
	ut_assertok(expect_same_input(uts, &e, cmd));

	if (IS_ENABLED(CONFIG_CMD_SOURCE) && IS_ENABLED(CONFIG_LEGACY_IMAGE_FORMAT)) {
		img = script_image(script);
		ut_assertnonnull(img);
		run_commandf("source %lx", (ulong)map_to_sysmem(img));
		free(img);
		ut_assert_skip_to_line(HELD);
		ut_assert_nextline("  command : %s", cmd);
		ut_assert_skip_to_line(HINT);
		ut_assert_nextline(SAME);
		ut_assert_console_end();
		ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	}

	/* 动作仍待确认：下一次输入的 yes 执行它 */
	ut_assertok(yes());
	ut_assert_nextline("redline: running %s", cmd);
	ut_assert_nextline("MMC write: dev # 0, block # 64, count 1 ... 1 blocks written: OK");
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0xa5));
	ut_assertok(env_set("rl_cmd", NULL));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_same_input, UTF_CONSOLE);

/* 展示后改了源数据：yes 不执行，按新内容（新的 crc32）重新展示，再 yes 才写进新内容 */
static int cmd_test_redline_source_changed(struct unit_test_state *uts)
{
	struct rl_env e;
	char cmd[64];

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	snprintf(cmd, sizeof(cmd), "mmc write %lx 40 1", e.aa);

	ut_asserteq(1, run_command(cmd, 0));
	ut_assertok(expect_held_write40(uts, cmd, e.aa, CRC_A5));
	fill(e.a, 0x33);
	ut_asserteq(1, yes());
	ut_assert_nextline(CHANGED);
	ut_assertok(expect_held_write40(uts, cmd, e.aa, CRC_33));
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	ut_assertok(yes());
	ut_assert_nextline("redline: running %s", cmd);
	ut_assert_skip_to_line("MMC write: dev # 0, block # 64, count 1 ... 1 blocks written: OK");
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x33));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_source_changed, UTF_CONSOLE);

/* 源数据碰到 lmb 里带 no-map 的保留区（Amlogic 上是 BL31/BL32 的安全区，CPU 读了可能挂死）：清单不读它、注明原因（整段在区里与跨进区里都算）；保留区释放后照常读、算 crc32 */
static int cmd_test_redline_nomap(struct unit_test_state *uts)
{
	struct rl_env e;
	phys_addr_t addr;
	ulong a;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !IS_ENABLED(CONFIG_LMB))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	ut_assertok(lmb_alloc_mem(LMB_MEM_ALLOC_ANY, SZ_4K, &addr, SZ_4K,
				  LMB_NOMAP | LMB_NONOTIFY));
	a = addr;
	memset(map_sysmem(a - 0x100, SZ_4K), 0xa5, SZ_4K);

	ut_asserteq(1, run_commandf("mmc write %lx 40 1", a));
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : mmc write %lx 40 1", a);
	ut_assert_nextline("  action  : MMC block write from %lx", a);
	ut_assert_nextline("  source  : 0x200 bytes at 0x%lx (inside a no-map reserved-memory region: not read, not checksummed)", a);
	console_record_reset();
	ut_asserteq(1, run_commandf("mmc write %lx 40 1", a - 0x100));
	ut_assert_skip_to_line("  source  : 0x200 bytes at 0x%lx (inside a no-map reserved-memory region: not read, not checksummed)", a - 0x100);
	console_record_reset();

	ut_assertok(lmb_free(addr, SZ_4K, LMB_NOMAP | LMB_NONOTIFY));
	ut_asserteq(1, run_commandf("mmc write %lx 40 1", a));
	ut_assert_skip_to_line("  source  : 0x200 bytes at 0x%lx, crc32 0x%08x", a, CRC_A5);
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_nomap, UTF_CONSOLE);

/* mmc erase：不对齐擦除组、卡又不支持 TRIM 时清单给出扩到组边界的实际范围（sandbox 的卡按 SD 处理，擦除组为 1，这里临时改 struct mmc 模拟） */
static int cmd_test_redline_erase_group(struct unit_test_state *uts)
{
	struct rl_env e;
	struct mmc *mmc;
	uint grp, init;
	bool trim;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	mmc = find_mmc_device(0);
	ut_assertnonnull(mmc);
	grp = mmc->erase_grp_size;
	trim = mmc->can_trim;
	init = mmc->has_init;
	mmc->erase_grp_size = 0x10;
	mmc->can_trim = false;

	ut_assertok(expect_held(uts, "mmc erase 41 2", "  action  : MMC erase"));
	ut_assert_nextline("  note    : blocks 0x41..0x43 are not aligned to the 0x10-block erase group and the card cannot TRIM: whole groups are erased");
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x50 (0x10 blocks)");
	ut_assert_nextline("  bytes   : 0x8000..0xa000");
	ut_assert_nextline("  overlaps: 1 of the 2 partitions");
	console_record_reset();

	/* 按分区名：#1 是 0x20+0x40，对齐 0x10，不扩 */
	ut_assertok(expect_held(uts, "mmc erase mmcsda1", "  action  : MMC erase"));
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x20..0x60 (0x40 blocks)");
	console_record_reset();

	/* 支持 TRIM：按写块擦，范围不扩 */
	mmc->can_trim = true;
	ut_assertok(expect_held(uts, "mmc erase 41 2", "  action  : MMC erase"));
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x41..0x43 (0x2 blocks)");
	console_record_reset();

	/* 擦除组未知 */
	mmc->erase_grp_size = 0;
	ut_assertok(expect_held(uts, "mmc erase 41 2", "  action  : MMC erase"));
	ut_assert_nextline("  note    : erase group unknown (EXT_CSD not read): the card may erase up to its erase-group boundaries around this range");
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x41..0x43 (0x2 blocks)");
	console_record_reset();

	mmc->erase_grp_size = grp;
	mmc->can_trim = trim;
	mmc->has_init = init;
	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_erase_group, UTF_CONSOLE);

#define SMC_ALLOWED	"  allowed : PSCI SYSTEM_RESET 0x84000009, Amlogic set reboot reason 0x82000049, PSCI_VERSION, PSCI_FEATURES, SMCCC_VERSION, SMCCC_ARCH_FEATURES, Amlogic secure storage read-only calls 0x82000023-0x82000027, 0x82000060/61/63/64/65/67/6b/6c"

/* 伪造命令表项 @name 直接过红线门：放行时返回 0、不打印、不动待确认动作 */
static int expect_gate_passes(struct unit_test_state *uts, const char *name,
			      int argc, char *argv[])
{
	struct cmd_tbl cmd = { .name = (char *)name };

	console_record_reset();
	ut_asserteq(0, redline_gate(&cmd, argc, argv));
	ut_assert_console_end();

	return 0;
}

static int expect_gate_holds(struct unit_test_state *uts, const char *name,
			     int argc, char *argv[])
{
	struct cmd_tbl cmd = { .name = (char *)name };

	console_record_reset();
	ut_asserteq(CMD_RET_FAILURE, redline_gate(&cmd, argc, argv));
	ut_assert_nextline(HELD);

	return 0;
}

/*
 * smc/hvc 的 fid 白名单（sandbox 没有这两条命令：按命令表规范名直接调 redline_gate()）：m12.env 的 smc 0x84000009 ${m12_fallback_reason}（含变量为空）与 smc 0x82000049 0、PSCI/SMCCC 查询放行；eFuse 写 0x82000031 拦、标不可逆并逐字节列出安全监控输入共享缓冲里要烧的数据（取不到缓冲时注明、x2 超过 4 KiB 时只列前 4 KiB）；高 32 位不为 0、未知 fid 拦
 */
static int cmd_test_redline_smc(struct unit_test_state *uts)
{
	char *reset[] = { "smc", "0x84000009", "3" };
	char *reset_empty[] = { "smc", "0x84000009" };
	char *reason[] = { "smc", "0x82000049", "0" };
	char *version[] = { "smc", "84000000" };
	char *features[] = { "smc", "0x8400000a", "0x84000009" };
	char *smccc[] = { "smc", "0x80000000" };
	char *smccc_feat[] = { "smc", "0x80000001", "0x80008000" };
	char *usage[] = { "smc" };
	char *efuse[] = { "smc", "0x82000031", "0", "10" };
	char *efuse_at[] = { "smc", "0x82000031", "8", "14" };
	char *efuse_big[] = { "smc", "0x82000031", "0", "1001" };
	char *wide[] = { "smc", "0x184000009", "3" };
	char *unknown[] = { "smc", "0xc2000000" };
	char *hvc[] = { "hvc", "0x82000031" };
	u8 *shm;
	int i;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	run_command("redline cancel", 0);
	shm = malloc(SZ_4K);
	ut_assertnonnull(shm);
	for (i = 0; i < SZ_4K; i++)
		shm[i] = i;
	rl_sm_buf = 0;
	ut_assertok(expect_gate_passes(uts, "smc", 3, reset));
	ut_assertok(expect_gate_passes(uts, "smc", 2, reset_empty));
	ut_assertok(expect_gate_passes(uts, "smc", 3, reason));
	ut_assertok(expect_gate_passes(uts, "smc", 2, version));
	ut_assertok(expect_gate_passes(uts, "smc", 3, features));
	ut_assertok(expect_gate_passes(uts, "smc", 2, smccc));
	ut_assertok(expect_gate_passes(uts, "smc", 3, smccc_feat));
	ut_assertok(expect_gate_passes(uts, "smc", 1, usage));
	ut_assertok(expect_gate_passes(uts, "hvc", 3, reset));
	ut_asserteq(1, run_command("redline show", 0));
	ut_assert_nextline("redline: nothing pending");

	ut_assertok(expect_gate_holds(uts, "smc", 4, efuse));
	ut_assert_nextline("  command : smc 0x82000031 0 10");
	ut_assert_nextline("  action  : secure monitor call (smc), function ID 0x82000031");
	ut_assert_nextline("  function: W0 0x82000031 = Amlogic SiP: eFuse write from the shared input buffer");
	ut_assert_nextline("  args    : x1 0x0, x2 0x10, x3 0x0, x4 0x0, x5 0x0, x6 0x0, w7 0x0");
	ut_assert_nextline(SMC_ALLOWED);
	ut_assert_nextline("  data    : the secure monitor input buffer (0x0) is not readable (not shown)");
	ut_assert_nextline("  IRREVERSIBLE: eFuse bits are one-time programmable");
	ut_assert_nextline(HINT);
	ut_assert_console_end();

	/* 缓冲里此刻的字节逐个列出（烧到 eFuse 偏移 x1 起）；改了缓冲，清单随之不同 */
	rl_sm_buf = map_to_sysmem(shm);
	ut_assertok(expect_gate_holds(uts, "smc", 4, efuse_at));
	ut_assert_skip_to_line(SMC_ALLOWED);
	ut_assert_nextline("  data    : 0x14 bytes from the secure monitor input buffer at 0x%lx", rl_sm_buf);
	ut_assert_nextline("  data    : eFuse 0x008: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");
	ut_assert_nextline("  data    : eFuse 0x018: 10 11 12 13");
	ut_assert_nextline("  IRREVERSIBLE: eFuse bits are one-time programmable");
	ut_assert_nextline(HINT);
	ut_assert_console_end();
	shm[0x13] = 0xee;
	ut_assertok(expect_gate_holds(uts, "smc", 4, efuse_at));
	ut_assert_skip_to_line("  data    : eFuse 0x018: 10 11 12 ee");

	/* x2 超过 4 KiB：只列前 4 KiB */
	ut_assertok(expect_gate_holds(uts, "smc", 4, efuse_big));
	ut_assert_skip_to_line("  data    : 0x1001 bytes from the secure monitor input buffer at 0x%lx", rl_sm_buf);
	ut_assert_nextline("  note    : x2 is larger than the 4 KiB input buffer: what the secure monitor does past it is not known; only the first 4 KiB are listed");
	ut_assert_skip_to_line("  data    : eFuse 0xff0: f0 f1 f2 f3 f4 f5 f6 f7 f8 f9 fa fb fc fd fe ff");
	ut_assert_nextline("  IRREVERSIBLE: eFuse bits are one-time programmable");
	rl_sm_buf = 0;
	free(shm);

	ut_assertok(expect_gate_holds(uts, "smc", 3, wide));
	ut_assert_nextline("  command : smc 0x184000009 3");
	ut_assert_nextline("  action  : secure monitor call (smc), function ID 0x184000009 (upper 32 bits set; SMCCC passes the function ID in W0)");
	ut_assert_nextline("  function: W0 0x84000009 = PSCI SYSTEM_RESET");
	ut_assert_nextline("  args    : x1 0x3, x2 0x0, x3 0x0, x4 0x0, x5 0x0, x6 0x0, w7 0x0");
	ut_assert_nextline(SMC_ALLOWED);
	ut_assert_nextline(HINT);
	ut_assert_console_end();

	ut_assertok(expect_gate_holds(uts, "smc", 2, unknown));
	ut_assert_nextline("  command : smc 0xc2000000");
	ut_assert_nextline("  action  : secure monitor call (smc), function ID 0xc2000000: not on the allow list");
	ut_assert_nextline("  args    : x1 0x0, x2 0x0, x3 0x0, x4 0x0, x5 0x0, x6 0x0, w7 0x0");
	ut_assert_skip_to_line(HINT);
	ut_assert_console_end();

	ut_assertok(expect_gate_holds(uts, "hvc", 2, hvc));
	ut_assert_nextline("  command : hvc 0x82000031");
	ut_assert_nextline("  action  : hypervisor call (hvc), function ID 0x82000031");
	ut_assert_skip_to_line("  IRREVERSIBLE: eFuse bits are one-time programmable");
	ut_assert_nextline(HINT);
	ut_assert_console_end();

	ut_assertok(run_command("redline cancel", 0));
	console_record_reset();

	return 0;
}
CMD_TEST(cmd_test_redline_smc, UTF_CONSOLE);

/*
 * Amlogic 厂商 BL31 安全存储：只读调用（取缓冲与块大小、QUERY/READ/TELL/VERIFY/STATUS/LIST、GET_ENCTYPE、VERSION）放行，供按规格手工做 RAM 级实验；改存储或 BL31 运行态的 WRITE、REMOVE、NOTIFY、NOTIFY_EX、SET_ENCTYPE 与 SET_STORAGE_INFO 拦，并叫出名字
 */
static int cmd_test_redline_smc_storage(struct unit_test_state *uts)
{
	static const char *const allowed[] = {
		"0x82000023", "0x82000024", "0x82000025", "0x82000026", "0x82000027",
		"0x82000060", "0x82000061", "0x82000063", "0x82000064", "0x82000065",
		"0x82000067", "0x8200006b", "0x8200006c",
	};
	static const struct {
		const char *fid;
		const char *what;
	} held[] = {
		{ "0x82000062", "Amlogic secure storage: write (re-encrypts the block buffer)" },
		{ "0x82000068", "Amlogic secure storage: remove (re-encrypts the block buffer)" },
		{ "0x82000066", "Amlogic secure storage: notify (rebuilds the key table)" },
		{ "0x82000069", "Amlogic secure storage: notify_ex (rebuilds the key table)" },
		{ "0x8200006a", "Amlogic secure storage: set encryption type" },
		{ "0x82000028", "Amlogic SiP: set storage info (AO register)" },
	};
	char *argv[] = { "smc", NULL, "0x40000" };
	int i;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	run_command("redline cancel", 0);
	for (i = 0; i < ARRAY_SIZE(allowed); i++) {
		argv[1] = (char *)allowed[i];
		ut_assertok(expect_gate_passes(uts, "smc", 2, argv));
	}
	ut_asserteq(1, run_command("redline show", 0));
	ut_assert_nextline("redline: nothing pending");

	for (i = 0; i < ARRAY_SIZE(held); i++) {
		argv[1] = (char *)held[i].fid;
		ut_assertok(expect_gate_holds(uts, "smc", 3, argv));
		ut_assert_nextline("  command : smc %s 0x40000", held[i].fid);
		ut_assert_nextline("  action  : secure monitor call (smc), function ID %s", held[i].fid);
		ut_assert_nextline("  function: W0 %s = %s", held[i].fid, held[i].what);
		ut_assert_nextline("  args    : x1 0x40000, x2 0x0, x3 0x0, x4 0x0, x5 0x0, x6 0x0, w7 0x0");
		ut_assert_nextline(SMC_ALLOWED);
		ut_assert_nextline(HINT);
		ut_assert_console_end();
	}

	ut_assertok(run_command("redline cancel", 0));
	console_record_reset();

	return 0;
}
CMD_TEST(cmd_test_redline_smc_storage, UTF_CONSOLE);

/* sm efusewrite：清单逐字节列出要烧的数据（参数按 simple_strtoul(…, 0) 解析：0x 十六进制、否则十进制）；超过 4 KiB 输入缓冲时注明会被拒绝；只读子命令放行，歧义的拦 */
static int cmd_test_redline_efuse(struct unit_test_state *uts)
{
	char addr[24], *write[] = { "sm", "efusew", "0x10", "20", addr };
	char *big[] = { "sm", "efusewrite", "0", "0x1001", addr };
	char *read[] = { "sm", "efuseread", "0", "20", addr };
	char *ambiguous[] = { "sm", "efuse", "0", "20", addr };
	u8 *buf;
	int i;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	run_command("redline cancel", 0);
	buf = malloc(20);
	ut_assertnonnull(buf);
	for (i = 0; i < 20; i++)
		buf[i] = i;
	snprintf(addr, sizeof(addr), "0x%lx", (ulong)map_to_sysmem(buf));

	ut_assertok(expect_gate_holds(uts, "sm", 5, write));
	ut_assert_nextline("  command : sm efusew 0x10 20 %s", addr);
	ut_assert_nextline("  action  : eFuse write: 0x14 bytes at eFuse offset 0x10 from %s", addr);
	ut_assert_nextline("  data    : eFuse 0x010: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");
	ut_assert_nextline("  data    : eFuse 0x020: 10 11 12 13");
	ut_assert_nextline("  IRREVERSIBLE: eFuse bits are one-time programmable");
	ut_assert_nextline(HINT);
	ut_assert_console_end();

	ut_assertok(expect_gate_holds(uts, "sm", 5, big));
	ut_assert_skip_to_line("  note    : more than the 4 KiB secure monitor input buffer: the call is refused");
	ut_assert_nextline("  IRREVERSIBLE: eFuse bits are one-time programmable");

	ut_assertok(expect_gate_holds(uts, "sm", 5, ambiguous));
	ut_assert_nextline("  command : sm efuse 0 20 %s", addr);
	ut_assert_nextline("  action  : 'sm efuse' does not resolve to a known read-only subcommand (held: it cannot be classified)");
	ut_assertok(run_command("redline cancel", 0));
	ut_assertok(expect_gate_passes(uts, "sm", 5, read));

	free(buf);
	console_record_reset();

	return 0;
}
CMD_TEST(cmd_test_redline_efuse, UTF_CONSOLE);

/* sparse 映像头：块大小 4 KiB，@fhdr 字节的文件头、12 字节的 chunk 头，@blks 块、@chunks 个 chunk */
static void sparse_hdr(u8 *img, u16 fhdr, u32 blks, u32 chunks)
{
	put_unaligned_le32(SPARSE_HEADER_MAGIC, img);
	put_unaligned_le16(1, img + 4);
	put_unaligned_le16(fhdr, img + 8);
	put_unaligned_le16(12, img + 10);
	put_unaligned_le32(4096, img + 12);
	put_unaligned_le32(blks, img + 16);
	put_unaligned_le32(chunks, img + 20);
}

/* chunk 头写在 @p，RAW 时再跟一块 4 KiB 的 0x5a；返回下一个 chunk 的位置 */
static u8 *sparse_chunk(u8 *p, u16 type, u32 total_sz)
{
	put_unaligned_le16(type, p);
	put_unaligned_le32(1, p + 4);
	put_unaligned_le32(total_sz, p + 8);
	if (type != CHUNK_TYPE_RAW)
		return p + 12;
	memset(p + 12, 0x5a, 4096);

	return p + 12 + 4096;
}

/* 执行 mmc swrite @ia @blk，断言被拦，核对动作行，后面的行留给调用者 */
static int expect_swrite(struct unit_test_state *uts, ulong ia, uint blk)
{
	console_record_reset();
	ut_asserteq(1, run_commandf("mmc swrite %lx %x", ia, blk));
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : mmc swrite %lx %x", ia, blk);
	ut_assert_nextline("  action  : MMC sparse image write from %lx", ia);

	return 0;
}

/*
 * mmc swrite：源数据与盘上范围照 lib/image-sparse.c 的 write_sparse_image() 逐 chunk 走出（一个 RAW chunk 一块 4 KiB 的 0x5a + 一个 DONT_CARE；crc32 由 Python zlib.crc32 对同样构造的字节算出）。
 * 上游不查 DONT_CARE 的 total_sz；文件头比 sparse_header_t 长时多出的部分跳两次；坏 chunk 之前的照写；越过盘尾的 chunk 报错；块数与头不符时写完再报错；魔数不对时一块不写
 */
static int cmd_test_redline_swrite(struct unit_test_state *uts)
{
	struct rl_env e;
	u8 *img, *c2;
	ulong ia;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !IS_ENABLED(CONFIG_CMD_MMC_SWRITE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	img = calloc(1, 0x1034);
	ut_assertnonnull(img);
	sparse_hdr(img, 28, 2, 2);
	c2 = sparse_chunk(img + 28, CHUNK_TYPE_RAW, 12 + 4096);
	sparse_chunk(c2, CHUNK_TYPE_DONT_CARE, 12);
	ia = map_to_sysmem(img);

	ut_assertok(expect_swrite(uts, ia, 0x40));
	ut_assert_nextline("  source  : 0x1034 bytes at 0x%lx, crc32 0xa0ca6025", ia);
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x50 (0x10 blocks)");

	/* DONT_CARE 的 total_sz 比 chunk 头还短：上游不查，照样写 */
	put_unaligned_le32(4, c2 + 8);
	ut_assertok(expect_swrite(uts, ia, 0x40));
	ut_assert_nextline("  source  : 0x1034 bytes at 0x%lx, crc32 0x657e48ca", ia);
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x50 (0x10 blocks)");
	put_unaligned_le32(12, c2 + 8);

	/* 第 2 个 chunk 类型未知：第 1 个照写，命令在第 2 个报错 */
	put_unaligned_le16(0xcaff, c2);
	ut_assertok(expect_swrite(uts, ia, 0x40));
	ut_assert_nextline("  source  : 0x1034 bytes at 0x%lx, crc32 0xb1e91ba0", ia);
	ut_assert_nextline("  note    : chunk 2 (type 0xcaff) is rejected (unknown chunk type): mmc swrite writes the chunks before it, then fails");
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x48 (0x8 blocks)");
	put_unaligned_le16(CHUNK_TYPE_DONT_CARE, c2);

	/* 头里的块数与各 chunk 之和不符：全部写完再报错 */
	put_unaligned_le32(3, img + 16);
	ut_assertok(expect_swrite(uts, ia, 0x40));
	ut_assert_nextline("  source  : 0x1034 bytes at 0x%lx, crc32 0xca6ec7e5", ia);
	ut_assert_nextline("  note    : the chunks add up to 0x2 sparse blocks, the header declares 0x3: mmc swrite writes them all, then reports failure");
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x50 (0x10 blocks)");
	put_unaligned_le32(2, img + 16);

	/* RAW 越过盘尾（0x800 块）：在第 1 个 chunk 报错，一块不写；源数据只读到第 1 个 chunk 头 */
	ut_assertok(expect_swrite(uts, ia, 0x7fc));
	ut_assert_nextline("  source  : 0x28 bytes at 0x%lx, crc32 0x82480fc2", ia);
	ut_assert_nextline("  note    : chunk 1 (type 0xcac1) is rejected (it would run past the end of the device): mmc swrite writes the chunks before it, then fails");
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x7fc..0x7fc (0x0 blocks)");

	/* 魔数不对：一块不写 */
	img[0] ^= 1;
	ut_assertok(expect_swrite(uts, ia, 0x40));
	ut_assert_nextline("  note    : no Android sparse image at the source (magic or major version): mmc swrite refuses before writing anything");
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline(HINT);
	img[0] ^= 1;

	/* 文件头 32 字节：上游跳过 file_hdr_sz 后把多出的 4 字节再跳一次，第一个 chunk 在偏移 36 */
	memset(img, '\0', 0x1034);
	sparse_hdr(img, 32, 1, 1);
	sparse_chunk(img + 36, CHUNK_TYPE_RAW, 12 + 4096);
	ut_assertok(expect_swrite(uts, ia, 0x40));
	ut_assert_nextline("  source  : 0x1030 bytes at 0x%lx, crc32 0xd8ca8058", ia);
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x48 (0x8 blocks)");

	free(img);
	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_swrite, UTF_CONSOLE);

/* CMD_REDLINE 关闭：mmc write 第一次就执行，没有 redline 命令 */
static int cmd_test_redline_off(struct unit_test_state *uts)
{
	struct blk_desc *desc;
	u8 *buf;

	if (IS_ENABLED(CONFIG_CMD_REDLINE))
		return -EAGAIN;
	ut_assertok(run_command("mmc dev 0", 0));
	ut_assertok(blk_get_desc(UCLASS_MMC, 0, &desc));
	buf = memalign(BLKSZ, BLKSZ);
	ut_assertnonnull(buf);
	fill(buf, 0x11);
	ut_asserteq(1, blk_dwrite(desc, 0x40, 1, buf));
	fill(buf, 0xa5);
	console_record_reset();
	ut_assertok(run_commandf("mmc write %lx 40 1", (ulong)map_to_sysmem(buf)));
	ut_assert_nextline("MMC write: dev # 0, block # 64, count 1 ... 1 blocks written: OK");
	ut_assert_console_end();
	ut_assertok(blk_is(uts, desc, 0x40, 0xa5));
	ut_asserteq(1, run_command("redline yes", 0));
	ut_assert_nextline("Unknown command 'redline' - try 'help'");
	ut_assert_console_end();
	free(buf);

	return 0;
}
CMD_TEST(cmd_test_redline_off, UTF_CONSOLE);

/*
 * fastboot flash/erase（补丁 0403）：每个请求被拦下、回 FAIL，清单按 fastboot 自己的解析给出目标（这里用 fastboot_raw_partition_ 原始区段，块 0x40 起 0x20 块）；同一请求里不能确认；单独一次输入的 redline yes 执行一次；展示后下载缓冲变了就重新展示；sparse 镜像按展开后的范围列（RAW 写、DONT_CARE 跳过）；erase 列整个分区；fastboot 不认识的名字照样拦、清单注明。sandbox 没开 FASTBOOT_OEM_RUN，“单独一次输入”用 redline_input_next() 加 redline yes 模拟（oem run 正是这样做的）
 */
#define FB_HINT	"Next: 'redline yes' as a separate input (fastboot oem run:redline yes) runs it once, 'redline show' lists it again, 'redline cancel' drops it"
#define FB_FAIL	"FAILREDLINE held; confirm: oem run:redline yes"

static int fb_download(struct unit_test_state *uts, const void *data, u32 len)
{
	char resp[FASTBOOT_RESPONSE_LEN], cmd[32];

	snprintf(cmd, sizeof(cmd), "download:%08x", len);
	fastboot_handle_command(cmd, resp);
	ut_assert(!strncmp(resp, "DATA", 4));
	fastboot_data_download(data, len, resp);
	fastboot_data_complete(resp);
	ut_asserteq_str("OKAY", resp);

	return 0;
}

static int fb_held(struct unit_test_state *uts, const char *cmd)
{
	char resp[FASTBOOT_RESPONSE_LEN], c[64];

	strlcpy(c, cmd, sizeof(c));
	console_record_reset();
	fastboot_handle_command(c, resp);
	ut_asserteq_str(FB_FAIL, resp);
	/* fastboot 解析不认识的名字时自己会先打印一行（Bad device specification） */
	ut_assert_skip_to_line(HELD);

	return 0;
}

static int cmd_test_redline_fastboot(struct unit_test_state *uts)
{
	struct rl_env e;
	u8 *buf, *img, *c2;
	ulong bufa;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	buf = memalign(ARCH_DMA_MINALIGN, SZ_64K);
	ut_assertnonnull(buf);
	bufa = map_to_sysmem(buf);
	fastboot_init(buf, SZ_64K);
	ut_assertok(env_set("fastboot_raw_partition_rlfb", "0x40 0x20"));

	/* 一块 0xa5：拦下，块 0x40 不变 */
	ut_assertok(fb_download(uts, e.a, BLKSZ));
	ut_assertok(fb_held(uts, "flash:rlfb"));
	ut_assert_nextline("  command : fastboot flash rlfb (0x200 bytes downloaded)");
	ut_assert_nextline("  target  : 'rlfb', blocks 0x40..0x60 (0x20 blocks)");
	ut_assert_nextline("  source  : 0x200 bytes at 0x%lx, crc32 0x%08x", bufa, CRC_A5);
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x41 (0x1 blocks)");
	ut_assert_nextline("  bytes   : 0x8000..0x8200");
	ut_assert_nextline("  overlaps: 1 of the 2 partitions");
	ut_assert_nextline("    #1 mmcsda1: blocks 0x20..0x60");
	ut_assert_nextline(FB_HINT);
	ut_assert_console_end();
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));

	/* 同一次输入里确认被拒；单独一次输入执行一次，再来一次没有可执行的 */
	ut_asserteq(1, run_command("redline yes", 0));
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	redline_input_next();
	ut_assertok(run_command("redline yes", 0));
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0xa5));
	ut_assertok(blk_is(uts, e.mmc0, 0x41, 0x11));
	redline_input_next();
	ut_asserteq(1, run_command("redline yes", 0));

	/* 展示之后下载缓冲变了（主机又下载了别的）：不执行，展示新清单 */
	ut_assertok(fb_download(uts, e.b, BLKSZ));
	ut_assertok(fb_held(uts, "flash:rlfb"));
	memset(buf, 0x33, BLKSZ);
	redline_input_next();
	console_record_reset();
	ut_asserteq(1, run_command("redline yes", 0));
	ut_assert_nextline(CHANGED);
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : fastboot flash rlfb (0x200 bytes downloaded)");
	ut_assert_nextline("  target  : 'rlfb', blocks 0x40..0x60 (0x20 blocks)");
	ut_assert_nextline("  source  : 0x200 bytes at 0x%lx, crc32 0x%08x", bufa, CRC_33);
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0xa5));
	ut_assertok(run_command("redline cancel", 0));

	/* sparse：RAW 一块 4 KiB、DONT_CARE 一块 4 KiB，范围按展开后的 0x10 块列 */
	img = calloc(1, 0x1034);
	ut_assertnonnull(img);
	sparse_hdr(img, 28, 2, 2);
	c2 = sparse_chunk(img + 28, CHUNK_TYPE_RAW, 12 + 4096);
	sparse_chunk(c2, CHUNK_TYPE_DONT_CARE, 12);
	ut_assertok(fb_download(uts, img, 0x1034));
	ut_assertok(fb_held(uts, "flash:rlfb"));
	ut_assert_nextline("  command : fastboot flash rlfb (0x1034 bytes downloaded)");
	ut_assert_nextline("  target  : 'rlfb', blocks 0x40..0x60 (0x20 blocks)");
	ut_assert_nextline("  image   : Android sparse; the range below is the expanded one (RAW and FILL chunks are written, DONT_CARE is skipped)");
	ut_assert_nextline("  source  : 0x1034 bytes at 0x%lx, crc32 0xa0ca6025", bufa);
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x50 (0x10 blocks)");
	/* 分区只有 4 块：第 1 个 RAW chunk（8 块）就越过分区末尾，上游在它之前停下 */
	ut_assertok(env_set("fastboot_raw_partition_rlfb", "0x40 0x4"));
	ut_assertok(fb_held(uts, "flash:rlfb"));
	ut_assert_nextline("  command : fastboot flash rlfb (0x1034 bytes downloaded)");
	ut_assert_nextline("  target  : 'rlfb', blocks 0x40..0x44 (0x4 blocks)");
	ut_assert_nextline("  image   : Android sparse; the range below is the expanded one (RAW and FILL chunks are written, DONT_CARE is skipped)");
	ut_assert_skip_to_line("  note    : chunk 1 (type 0xcac1) is rejected (it would run past the end of the partition): fastboot flash writes the chunks before it, then fails");
	ut_assertok(env_set("fastboot_raw_partition_rlfb", "0x40 0x20"));
	free(img);
	ut_assertok(run_command("redline cancel", 0));

	/* erase：整个分区；不认识的名字照样拦下、清单注明 */
	ut_assertok(fb_held(uts, "erase:rlfb"));
	ut_assert_nextline("  command : fastboot erase rlfb");
	ut_assert_nextline("  target  : 'rlfb', blocks 0x40..0x60 (0x20 blocks)");
	ut_assert_nextline("  device  : mmc 0, hwpart 0 (user area), 512-byte blocks, 0x800 blocks");
	ut_assert_nextline("  blocks  : 0x40..0x60 (0x20 blocks)");
	ut_assertok(fb_held(uts, "flash:nosuch"));
	ut_assert_nextline("  command : fastboot flash nosuch (0x1034 bytes downloaded)");
	ut_assert_nextline("  target  : 'nosuch' is not a partition, alias or raw partition fastboot knows: fastboot will fail");
	console_record_reset();
	ut_assertok(run_command("redline cancel", 0));
	ut_assert_nextline("redline: dropped fastboot flash");

	env_set("fastboot_raw_partition_rlfb", NULL);
	free(buf);
	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_fastboot, UTF_CONSOLE);

/* ums -r（补丁 0404）：只读 LUN 不是存储写入，红线门放行、不动待确认动作；不带 -r 的照旧拦下 */
static int cmd_test_redline_ums_ro(struct unit_test_state *uts)
{
	struct rl_env e;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !IS_ENABLED(CONFIG_CMD_USB_MASS_STORAGE))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	console_record_reset();
	ut_asserteq(1, run_command("ums 0 mmc 0", 0));
	ut_assert_nextline(HELD);
	ut_assert_nextline("  command : ums 0 mmc 0");
	/* sandbox 没有 UDC：放行后 ums 自己在取控制器时失败，但不是被红线门拦下 */
	console_record_reset();
	run_command("ums -r 0 mmc 0", 0);
	/* 第一行不是红线清单（是 ums 自己取控制器失败的报错） */
	ut_assert(ut_check_console_line(uts, HELD));
	console_record_reset();
	ut_assertok(run_command("redline show", 0));
	ut_assert_nextline("REDLINE: pending storage write, not executed");
	ut_assert_nextline("  command : ums 0 mmc 0");
	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_ums_ro, UTF_CONSOLE);

#ifdef CONFIG_REDLINE_FASTBOOT_RELAXED
/* fastboot 的放宽分区（补丁 0405，sandbox 配置 REDLINE_FASTBOOT_RELAXED="mmcsda2"）：分区表里的同名分区、清单无附注时照常写；同名的原始区段、超出分区的镜像、表里没有的分区照旧拦下 */
static int cmd_test_redline_fastboot_relaxed(struct unit_test_state *uts)
{
	char resp[FASTBOOT_RESPONSE_LEN];
	u8 mbr[BLKSZ];
	struct rl_env e;
	u8 *buf;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC) ||
	    !IS_ENABLED(CONFIG_CMD_GPT) ||
	    strcmp(CONFIG_REDLINE_FASTBOOT_RELAXED, "mmcsda2 gpt"))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	ut_asserteq(1, blk_dread(e.mmc0, 0, 1, mbr));
	buf = memalign(ARCH_DMA_MINALIGN, SZ_256K);
	ut_assertnonnull(buf);
	fastboot_init(buf, SZ_256K);
	ut_assertok(fb_download(uts, e.a, BLKSZ));
	console_record_reset();
	fastboot_handle_command((char[]){"flash:mmcsda2"}, resp);
	ut_asserteq_str("OKAY", resp);
	ut_assert_nextline("redline: fastboot flash mmcsda2: relaxed partition (REDLINE_FASTBOOT_RELAXED), not held");
	ut_assertok(blk_is(uts, e.mmc0, 0x100, 0xa5));
	/* 不在表里的分区、同名但只是原始区段的写照旧拦下 */
	ut_assertok(fb_held(uts, "flash:mmcsda1"));
	run_command("redline cancel", 0);
	ut_assertok(env_set("fastboot_raw_partition_rlfb", "0x100 0x100"));
	ut_assertok(fb_held(uts, "flash:rlfb"));
	run_command("redline cancel", 0);
	env_set("fastboot_raw_partition_rlfb", NULL);
	/* 超出分区：清单带附注，照旧拦下 */
	ut_assertok(fb_download(uts, buf, 0x200 * 0x101));
	ut_assertok(fb_held(uts, "flash:mmcsda2"));
	run_command("redline cancel", 0);
	/* 名单里的名字若只是原始区段（盘上没有同名分区）：照旧拦下 */
	ut_assertok(fb_download(uts, e.a, BLKSZ));
	memset(buf, 0, BLKSZ);
	ut_asserteq(1, blk_dwrite(e.mmc0, 0, 1, buf));
	part_init(e.mmc0);
	ut_assertok(env_set("fastboot_raw_partition_mmcsda2", "0x100 0x100"));
	ut_assertok(fb_held(uts, "flash:mmcsda2"));
	env_set("fastboot_raw_partition_mmcsda2", NULL);
	run_command("redline cancel", 0);

	/* 块设备停在别的硬件分区上（前面的写留下的）：fastboot 会写进那个硬件分区，照旧拦下 */
	ut_asserteq(1, blk_dwrite(e.mmc0, 0, 1, mbr));
	part_init(e.mmc0);
	e.mmc0->hwpart = 1;
	ut_assertok(fb_held(uts, "flash:mmcsda2"));
	e.mmc0->hwpart = 0;
	run_command("redline cancel", 0);

	/*
	 * GPT 上的分区名：mmcsda 只是名单里 mmcsda2 的前缀，按整词比不在名单里；gpt 在名单里，但 fastboot 先把它当“写分区表”的特殊目标，
	 * 写的不是同名分区，照旧拦下
	 */
	ut_asserteq(1, run_command("gpt write mmc 0 \"name=mmcsda,start=64KiB,size=64KiB;name=gpt,start=128KiB,size=64KiB\"", 0));
	ut_assertok(yes());
	/* part_init() 先按已知类型复核，保护性 MBR 照样过 DOS 的检查：清掉类型重新识别 */
	e.mmc0->part_type = PART_TYPE_UNKNOWN;
	part_init(e.mmc0);
	ut_asserteq(PART_TYPE_EFI, e.mmc0->part_type);
	ut_assertok(fb_held(uts, "flash:mmcsda"));
	run_command("redline cancel", 0);
	ut_assertok(fb_held(uts, "flash:gpt"));
	run_command("redline cancel", 0);
	/* 清掉主、备两份 GPT，恢复 rl_setup 的 DOS 表 */
	memset(buf, 0, 34 * BLKSZ);
	ut_asserteq(34, blk_dwrite(e.mmc0, 0, 34, buf));
	ut_asserteq(33, blk_dwrite(e.mmc0, e.mmc0->lba - 33, 33, buf));
	ut_asserteq(1, blk_dwrite(e.mmc0, 0, 1, mbr));
	part_init(e.mmc0);
	ut_asserteq(PART_TYPE_DOS, e.mmc0->part_type);
	free(buf);
	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_fastboot_relaxed, UTF_CONSOLE);
#endif

/* fastboot.is-logical:<名>（补丁 0505）：标为逻辑分区的名字 flash/erase 一律回 FAIL，不进红线清单、不写；表里没有的照常走红线门 */
static int cmd_test_redline_fastboot_logical(struct unit_test_state *uts)
{
	char resp[FASTBOOT_RESPONSE_LEN];
	struct rl_env e;
	u8 *buf;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC))
		return -EAGAIN;
	ut_assertok(rl_setup(uts, &e));
	buf = memalign(ARCH_DMA_MINALIGN, SZ_64K);
	ut_assertnonnull(buf);
	fastboot_init(buf, SZ_64K);
	ut_assertok(env_set("fastboot_raw_partition_rlfb", "0x40 0x20"));
	ut_assertok(env_set("fastboot.is-logical:rlfb", "yes"));
	ut_assertok(fb_download(uts, e.a, BLKSZ));
	console_record_reset();
	fastboot_handle_command((char[]){"flash:rlfb"}, resp);
	ut_asserteq_str("FAILlogical partition: flash it from fastbootd", resp);
	fastboot_handle_command((char[]){"erase:rlfb"}, resp);
	ut_asserteq_str("FAILlogical partition: erase it from fastbootd", resp);
	ut_assert_console_end();
	fastboot_handle_command((char[]){"getvar:is-logical:rlfb"}, resp);
	ut_asserteq_str("OKAYyes", resp);
	ut_assertok(blk_is(uts, e.mmc0, 0x40, 0x11));
	ut_assertok(env_set("fastboot.is-logical:rlfb", "no"));
	ut_assertok(fb_held(uts, "flash:rlfb"));
	env_set("fastboot.is-logical:rlfb", NULL);
	env_set("fastboot_raw_partition_rlfb", NULL);
	free(buf);
	rl_teardown(&e);

	return 0;
}
CMD_TEST(cmd_test_redline_fastboot_logical, UTF_CONSOLE);
