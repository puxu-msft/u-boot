// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic USB 线刷（Optimus）协议核心的测试
 *
 * sandbox 没有 UDC：用一个假的传输层接住核心在 bulk IN 上发出的字节与 bulk OUT 上要收的长度，直接喂 SETUP 包、控制数据与 bulk 数据。核对每个实现的请求与命令词，以及错误路径：identify 的 8 字节；不实现的请求与方向不对的请求 STALL；bulk 命令的成功、失败、空命令、超长命令（SETUP 即 STALL）、前导空格、low_power；TPL 命令只在子码 1 执行、结果由 0x31 读出；download mem 的帧校验（校验和错不推进、可重发）、越过声明长度、没有声明、旧格式 16 字节控制头、get_status；内存目标越出 DRAM、压 lmb 保留区、数字不合格；upload mem 按主机给的帧长分帧、读完回零长度；download store 按 MPT 分区名写、非对齐尾帧、未知分区、超过分区、bootloader 与不支持的类型被拒；每条主机命令是红线门的一次新输入。
 * 黄金矢量（控制头字节与累加和）由仓库的主机侧协议库 homeiot.amlogic.usb.frames 算出（build_write_media_header、addsum），与被测代码无关；identify 的 8 字节同厂商 BL33 线刷态的应答数组（0x10ca6d0）。
 */

#include <aml_store.h>
#include <blk.h>
#include <blkmap.h>
#include <command.h>
#include <env.h>
#include <lmb.h>
#include <malloc.h>
#include <mapmem.h>
#include <memalign.h>
#include <optimus.h>
#include <part.h>
#include <asm/global_data.h>
#include <asm/unaligned.h>
#include <dm/device.h>
#include <linux/sizes.h>
#include <linux/usb/ch9.h>
#include <test/cmd.h>
#include <test/test.h>
#include <test/ut.h>

DECLARE_GLOBAL_DATA_PTR;

#define VIN	(USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE)
#define VOUT	(USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE)
/* homeiot.amlogic.usb.frames：数据 (i*7+3)&0xff 共 0x1003 字节，seq 0，应答 0x200 */
#define GOLD1_LEN	0x1003
#define GOLD1_SUM	0xfe130e03
static const u8 gold1_head[32] = {
	0x00, 0x00, 0x00, 0x00, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x0e, 0x13, 0xfe, 0xef, 0x00, 0x00, 0x02,
};
/* 数据 (i*13+5)&0xff 共 0x10000 字节，seq 1，应答 0x40 */
static const u8 gold2_head[32] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0xc0, 0x1f, 0x60, 0xef, 0x00, 0x40, 0x00,
};

struct fake {
	struct optimus o;
	u8 in[OPTIMUS_FRAME_MAX];	/* 最近一次 bulk IN 的内容 */
	uint in_len, in_count;
	uint out_len, out_count;	/* 最近一次要收的 bulk OUT 长度 */
	u8 ep0[OPTIMUS_CMD_MAX];
};

static int fake_bulk_in(struct optimus *o, const void *buf, unsigned int len)
{
	struct fake *f = o->priv;

	memcpy(f->in, buf, len);
	f->in_len = len;
	f->in_count++;

	return 0;
}

static int fake_bulk_out(struct optimus *o, unsigned int len)
{
	struct fake *f = o->priv;

	f->out_len = len;
	f->out_count++;

	return 0;
}

static const struct optimus_ops fake_ops = {
	.bulk_in = fake_bulk_in,
	.bulk_out = fake_bulk_out,
};

static struct fake *fake_new(void)
{
	struct fake *f = calloc(1, sizeof(*f));

	if (f && optimus_init(&f->o, &fake_ops, f)) {
		free(f);
		return NULL;
	}

	return f;
}

static void fake_free(struct fake *f)
{
	optimus_release(&f->o);
	free(f);
}

static struct usb_ctrlrequest req(u8 type, u8 request, u16 value, u16 index,
				  u16 length)
{
	struct usb_ctrlrequest c = {
		.bRequestType = type,
		.bRequest = request,
		.wValue = cpu_to_le16(value),
		.wIndex = cpu_to_le16(index),
		.wLength = cpu_to_le16(length),
	};

	return c;
}

/* 一条 OUT 控制请求（SETUP + 数据阶段）；SETUP 被 STALL 时返回其负值 */
static int ctrl_out(struct fake *f, u8 request, u16 index, const void *data,
		    uint len)
{
	struct usb_ctrlrequest c = req(VOUT, request, 0, index, len);
	int n = optimus_setup(&f->o, &c, f->ep0);

	if (n < 0)
		return n;
	memcpy(f->ep0, data, len);
	optimus_setup_data(&f->o, &c, f->ep0, n);

	return n;
}

/* 发一条 bulk 命令（wIndex 2，与主机 bulk_command 相同），返回应答串 */
static const char *bulkcmd(struct fake *f, const char *cmd)
{
	f->in_len = 0;
	if (ctrl_out(f, OPTIMUS_REQ_BULKCMD, 2, cmd, strlen(cmd) + 1) < 0)
		return "<stall>";
	f->in[OPTIMUS_REPLY_LEN - 1] = '\0';

	return f->in_len == OPTIMUS_REPLY_LEN ? (const char *)f->in : "<no reply>";
}

/* 一帧媒体写：控制头 @head，随后把 @data 放进帧缓冲、报告收完；返回应答串 */
static const char *media(struct fake *f, const u8 *head, uint headlen,
			 const void *data, uint len)
{
	f->in_len = 0;
	f->out_count = 0;
	if (ctrl_out(f, OPTIMUS_REQ_WRITE_MEDIA, 0xffff, head, headlen) < 0)
		return "<stall>";
	if (f->out_count) {
		memcpy(f->o.frame, data, min_t(uint, len, f->out_len));
		optimus_bulk_out_done(&f->o, min_t(uint, len, f->out_len));
	}
	f->in[OPTIMUS_REPLY_LEN - 1] = '\0';

	return (const char *)f->in;
}

/* 控制头：<IIII HH> 重发次数、帧长、序号、校验和、算法、应答长度 */
static void head(u8 *h, u32 len, u32 sum, u16 alg, u16 ack)
{
	memset(h, '\0', 32);
	put_unaligned_le32(len, h + 4);
	put_unaligned_le32(sum, h + 12);
	put_unaligned_le16(alg, h + 16);
	put_unaligned_le16(ack, h + 18);
}

static u32 addsum(const u8 *p, uint len)
{
	u32 s = 0;
	uint i;

	for (i = 0; i < len; i++)
		s += (u32)p[i] << ((i % 4) * 8);

	return s;
}

static int cmd_test_optimus_identify(struct unit_test_state *uts)
{
	static const u8 id[8] = { 0, 7, 0, 16, 0, 0, 0, 0 };
	struct fake *f = fake_new();
	struct usb_ctrlrequest c;
	static const u8 rejected[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
				       0x11, 0x12, 0x35, 0x36, 0x50, 0x60, 0xff };
	uint i;

	ut_assertnonnull(f);
	c = req(VIN, OPTIMUS_REQ_IDENTIFY, 0, 0, 8);
	ut_asserteq(8, optimus_setup(&f->o, &c, f->ep0));
	ut_asserteq_mem(id, f->ep0, 8);
	c = req(VIN, OPTIMUS_REQ_IDENTIFY, 0, 0, 4);
	ut_asserteq(4, optimus_setup(&f->o, &c, f->ep0));
	c = req(VIN, OPTIMUS_REQ_IDENTIFY, 0, 0, 64);
	ut_asserteq(8, optimus_setup(&f->o, &c, f->ep0));
	/* 方向不对、不是厂商类型、不实现的请求一律 STALL */
	c = req(VOUT, OPTIMUS_REQ_IDENTIFY, 0, 0, 8);
	ut_assert(optimus_setup(&f->o, &c, f->ep0) < 0);
	c = req(USB_DIR_IN, OPTIMUS_REQ_IDENTIFY, 0, 0, 8);
	ut_assert(optimus_setup(&f->o, &c, f->ep0) < 0);
	c = req(VIN, OPTIMUS_REQ_BULKCMD, 0, 2, 8);
	ut_assert(optimus_setup(&f->o, &c, f->ep0) < 0);
	c = req(VOUT, OPTIMUS_REQ_TPL_STAT, 0, 0, 64);
	ut_assert(optimus_setup(&f->o, &c, f->ep0) < 0);
	c = req(VOUT, OPTIMUS_REQ_WRITE_MEDIA, 1, 0xffff, 24);
	ut_assert(optimus_setup(&f->o, &c, f->ep0) < 0);
	for (i = 0; i < ARRAY_SIZE(rejected); i++) {
		c = req(VOUT, rejected[i], 0, 0, 16);
		ut_assert(optimus_setup(&f->o, &c, f->ep0) < 0);
		c = req(VIN, rejected[i], 0, 0, 16);
		ut_assert(optimus_setup(&f->o, &c, f->ep0) < 0);
	}
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_identify, 0);

static int cmd_test_optimus_bulkcmd(struct unit_test_state *uts)
{
	struct fake *f = fake_new();
	char *cmd;

	ut_assertnonnull(f);
	env_set("opt_t", NULL);
	ut_asserteq_str("success", bulkcmd(f, "setenv opt_t abc"));
	ut_asserteq_str("abc", env_get("opt_t"));
	ut_asserteq(OPTIMUS_REPLY_LEN, f->in_len);
	ut_asserteq_str("success", bulkcmd(f, "    echo 1234"));
	ut_asserteq_str("success", bulkcmd(f, "    low_power"));
	ut_asserteq_str("failed:", bulkcmd(f, "false"));
	ut_asserteq_str("failed:", bulkcmd(f, "no_such_command_x"));
	ut_asserteq_str("failed:", bulkcmd(f, "low_power 1"));
	ut_asserteq_str("failed:no command", bulkcmd(f, "   "));
	/* test 的成败就是主机区分主线与厂商 u-boot 的办法（m12_mainline） */
	ut_asserteq_str("success", bulkcmd(f, "test -n \"${opt_t}\""));
	ut_asserteq_str("failed:", bulkcmd(f, "test -n \"${opt_none}\""));

	/* 511 个字符加 NUL 正好 512 字节可以；513 字节在 SETUP 阶段就 STALL、不执行 */
	cmd = calloc(1, 600);
	ut_assertnonnull(cmd);
	strcpy(cmd, "setenv opt_t ");
	memset(cmd + strlen(cmd), 'x', 511 - strlen(cmd));
	ut_asserteq_str("success", bulkcmd(f, cmd));
	ut_asserteq(511 - 13, strlen(env_get("opt_t")));
	strcpy(cmd, "setenv opt_t ");
	memset(cmd + strlen(cmd), 'y', 512 - strlen(cmd));
	ut_asserteq_str("<stall>", bulkcmd(f, cmd));
	ut_asserteq('x', env_get("opt_t")[0]);
	free(cmd);
	env_set("opt_t", NULL);
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_bulkcmd, 0);

static int cmd_test_optimus_tplcmd(struct unit_test_state *uts)
{
	struct fake *f = fake_new();
	struct usb_ctrlrequest c;
	static const char set1[] = "setenv opt_t2 one";
	static const char set0[] = "setenv opt_t2 zero";

	ut_assertnonnull(f);
	env_set("opt_t2", NULL);
	/* 子码 0 不执行；子码 1 执行，不走 bulk IN，结果由 0x31 读 */
	ut_asserteq(sizeof(set0), ctrl_out(f, OPTIMUS_REQ_TPL_CMD, 0, set0, sizeof(set0)));
	ut_assertnull(env_get("opt_t2"));
	ut_asserteq(sizeof(set1), ctrl_out(f, OPTIMUS_REQ_TPL_CMD, 1, set1, sizeof(set1)));
	ut_asserteq_str("one", env_get("opt_t2"));
	ut_asserteq(0, f->in_count);
	c = req(VIN, OPTIMUS_REQ_TPL_STAT, 0, 0, 0x40);
	ut_asserteq(0x40, optimus_setup(&f->o, &c, f->ep0));
	ut_asserteq_str("success", (char *)f->ep0);
	ut_asserteq(6, ctrl_out(f, OPTIMUS_REQ_TPL_CMD, 1, "false", 6));
	ut_asserteq(0x40, optimus_setup(&f->o, &c, f->ep0));
	ut_asserteq_str("failed:", (char *)f->ep0);
	env_set("opt_t2", NULL);
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_tplcmd, 0);

static int cmd_test_optimus_download_mem(struct unit_test_state *uts)
{
	struct fake *f = fake_new();
	u8 *data, *mem, h[32];
	ulong addr = 0x8000000;
	uint i;

	ut_assertnonnull(f);
	data = malloc(OPTIMUS_FRAME_MAX);
	ut_assertnonnull(data);
	mem = map_sysmem(addr, 0x20000);
	memset(mem, 0xee, 0x20000);

	/* 黄金矢量 1：帧头字节、累加和都由主机侧协议库算出 */
	for (i = 0; i < GOLD1_LEN; i++)
		data[i] = (i * 7 + 3) & 0xff;
	ut_asserteq(GOLD1_SUM, addsum(data, GOLD1_LEN));
	ut_asserteq_str("success", bulkcmd(f, "download mem 0x8000000 normal 0x11003"));
	ut_asserteq_str("OK!!", media(f, gold1_head, 32, data, GOLD1_LEN));
	ut_asserteq(GOLD1_LEN, f->out_len);
	ut_asserteq(0x200, f->in_len);
	ut_asserteq_mem(data, mem, GOLD1_LEN);
	ut_asserteq(0xee, mem[GOLD1_LEN]);
	ut_asserteq_str("failed:download not complete", bulkcmd(f, "download get_status"));

	/* 校验和错：不写、不推进，主机重发同一帧后接着写 */
	for (i = 0; i < OPTIMUS_FRAME_MAX; i++)
		data[i] = (i * 13 + 5) & 0xff;
	memcpy(h, gold2_head, 32);
	put_unaligned_le32(get_unaligned_le32(h + 12) + 1, h + 12);
	ut_asserteq_str("failed:checksum mismatch", media(f, h, 32, data, OPTIMUS_FRAME_MAX));
	ut_asserteq(0x40, f->in_len);
	ut_asserteq(0xee, mem[GOLD1_LEN]);
	ut_asserteq(GOLD1_LEN, f->o.done);
	/* 黄金矢量 2：64 KiB 帧、应答长度 0x40 */
	ut_asserteq_str("OK!!", media(f, gold2_head, 32, data, OPTIMUS_FRAME_MAX));
	ut_asserteq(0x40, f->in_len);
	ut_asserteq_mem(data, mem + GOLD1_LEN, OPTIMUS_FRAME_MAX);
	ut_asserteq_str("success", bulkcmd(f, "download get_status"));

	/* 越过声明长度、帧长为 0 或超过 64 KiB、没有声明都拒收 */
	ut_asserteq_str("success", bulkcmd(f, "download mem 0x8000000 normal 0x100"));
	head(h, 0x101, addsum(data, 0x101), 0xef, 0x200);
	ut_asserteq_str("failed:frame runs past the declared size", media(f, h, 32, data, 0x101));
	head(h, 0x10001, 0, 0xee, 0x200);
	ut_asserteq_str("failed:frame length out of range", media(f, h, 32, data, OPTIMUS_FRAME_MAX));
	head(h, 0, 0, 0xee, 0x200);
	ut_asserteq_str("failed:frame length out of range", media(f, h, 32, data, 0));
	head(h, 0x100, 0, 0xf0, 0x200);
	ut_asserteq_str("failed:unsupported checksum algorithm", media(f, h, 32, data, 0x100));
	/* 少收了字节 */
	head(h, 0x100, addsum(data, 0x100), 0xef, 0x200);
	f->out_count = 0;
	ut_asserteq(32, ctrl_out(f, OPTIMUS_REQ_WRITE_MEDIA, 0xffff, h, 32));
	optimus_bulk_out_done(&f->o, 0x80);
	ut_asserteq_str("failed:short frame", (char *)f->in);
	/* 不校验（0xee）与旧格式 16 字节控制头（缺省累加和、应答 512 字节） */
	head(h, 0x80, 0, 0xee, 0x10);
	ut_asserteq_str("OK!!", media(f, h, 32, data, 0x80));
	ut_asserteq(0x10, f->in_len);
	head(h, 0x80, addsum(data + 0x80, 0x80), 0, 0);
	ut_asserteq_str("OK!!", media(f, h, 16, data + 0x80, 0x80));
	ut_asserteq(OPTIMUS_REPLY_LEN, f->in_len);
	ut_asserteq_mem(data, mem, 0x100);
	ut_asserteq_str("success", bulkcmd(f, "download get_status"));
	ut_asserteq_str("success", bulkcmd(f, "upload mem 0x8000000 normal 0x100"));
	ut_asserteq_str("failed:no download declared", media(f, h, 16, data, 0x80));
	ut_asserteq_str("failed:download not complete", bulkcmd(f, "download get_status"));

	/* 控制头按旧声明核过、数据还没到时又来了新的声明（换了地址）：这一帧作废，不写到新地址 */
	ut_asserteq_str("success", bulkcmd(f, "download mem 0x8000000 normal 0x100"));
	head(h, 0x100, addsum(data, 0x100), 0xef, 0x200);
	f->out_count = 0;
	ut_asserteq(32, ctrl_out(f, OPTIMUS_REQ_WRITE_MEDIA, 0xffff, h, 32));
	ut_asserteq(1, f->out_count);
	ut_asserteq_str("success", bulkcmd(f, "download mem 0x8010000 normal 0x100"));
	memset(mem + 0x10000, 0xee, 0x100);
	memcpy(f->o.frame, data, 0x100);
	optimus_bulk_out_done(&f->o, 0x100);
	ut_asserteq_str("failed:a new transfer was declared before the frame data arrived", (char *)f->in);
	ut_asserteq(0xee, mem[0x10000]);
	ut_asserteq(0, f->o.done);

	unmap_sysmem(mem);
	free(data);
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_download_mem, 0);

static const char *bulkcmdf(struct fake *f, const char *fmt, ...)
{
	static char cmd[OPTIMUS_CMD_MAX];
	va_list args;

	va_start(args, fmt);
	vsnprintf(cmd, sizeof(cmd), fmt, args);
	va_end(args);

	return bulkcmd(f, cmd);
}

static int cmd_test_optimus_mem_range(struct unit_test_state *uts)
{
	struct fake *f = fake_new();
	ulong top = gd->ram_base + gd->ram_size;
	phys_addr_t rsv = gd->ram_base + 0x9000000;

	ut_assertnonnull(f);
	/* 越出 DRAM：尾部越过、起点在 DRAM 之下、长度回绕 */
	ut_asserteq_str("failed:range not inside DRAM",
			bulkcmdf(f, "download mem 0x%lx normal 0x20", top - 0x10));
	ut_asserteq_str("failed:range not inside DRAM",
			bulkcmdf(f, "upload mem 0x%lx normal 0x20", top - 0x10));
	ut_asserteq_str("failed:range not inside DRAM",
			bulkcmd(f, "upload mem 0x10 normal 0xfffffffffffffff8"));
	if (gd->ram_base)
		ut_asserteq_str("failed:range not inside DRAM",
				bulkcmdf(f, "download mem 0x%lx normal 0x10", gd->ram_base - 0x8));
	/* 数字与类型不合格；新的声明作废上一次的 */
	ut_asserteq_str("failed:bad size", bulkcmd(f, "download mem 0x8000000 normal 0"));
	ut_asserteq_str("failed:bad size", bulkcmd(f, "download mem 0x8000000 normal 0x10q"));
	ut_asserteq_str("failed:bad address", bulkcmd(f, "download mem 0x80g normal 0x10"));
	ut_asserteq_str("failed:only image type normal is supported",
			bulkcmd(f, "download mem 0x8000000 sparse 0x10"));
	ut_asserteq(OPTIMUS_XFER_NONE, f->o.xfer);
	ut_asserteq_str("failed:unsupported media", bulkcmd(f, "download nand system normal 0x10"));
	ut_assert(!strncmp("failed:usage", bulkcmd(f, "download mem 0x8000000 normal"), 12));
	ut_asserteq_str("failed:too many arguments",
			bulkcmd(f, "download mem 0x8000000 normal 0x10 1 2"));
	/* 压住 lmb 里 U-Boot 或固件的保留区（NOOVERWRITE/NOMAP） */
	ut_assertok(lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &rsv, 0x1000, LMB_NOOVERWRITE));
	ut_asserteq_str("failed:range overlaps memory reserved for U-Boot or firmware",
			bulkcmdf(f, "download mem 0x%llx normal 0x10", (u64)rsv + 0xff8));
	ut_asserteq_str("failed:range overlaps memory reserved for U-Boot or firmware",
			bulkcmdf(f, "upload mem 0x%llx normal 0x2000", (u64)rsv - 0x1000));
	ut_asserteq_str("success", bulkcmdf(f, "download mem 0x%llx normal 0x1000", (u64)rsv + 0x1000));
	ut_assertok(lmb_free(rsv, 0x1000, LMB_NOOVERWRITE));
	ut_asserteq_str("success", bulkcmdf(f, "download mem 0x%llx normal 0x10", (u64)rsv));
	/* 固件保留、不映射的区（NOMAP，如 BL31/BL32） */
	ut_assertok(lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &rsv, 0x1000, LMB_NOMAP));
	ut_asserteq_str("failed:range overlaps memory reserved for U-Boot or firmware",
			bulkcmdf(f, "download mem 0x%llx normal 0x10", (u64)rsv + 0x800));
	ut_assertok(lmb_free(rsv, 0x1000, LMB_NOMAP));
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_mem_range, 0);

/* 0x33：16 字节帧信息 + bulk IN 数据，帧长取主机 wValue（0 即 64 KiB）与剩余字节的较小者 */
static int read_media(struct unit_test_state *uts, struct fake *f, u16 value,
		      uint want)
{
	struct usb_ctrlrequest c = req(VIN, OPTIMUS_REQ_READ_MEDIA, value,
				       (want + 0xfff) / 0x1000, 16);

	f->in_count = 0;
	ut_asserteq(16, optimus_setup(&f->o, &c, f->ep0));
	ut_asserteq(0xefe8, get_unaligned_le32(f->ep0));
	ut_asserteq(want, get_unaligned_le32(f->ep0 + 4));
	ut_asserteq(1, f->in_count);
	ut_asserteq(want, f->in_len);

	return 0;
}

static int cmd_test_optimus_upload_mem(struct unit_test_state *uts)
{
	struct fake *f = fake_new();
	ulong addr = 0x8100000;
	u8 *mem;
	uint i;

	ut_assertnonnull(f);
	mem = map_sysmem(addr, 0x20000);
	for (i = 0; i < 0x20000; i++)
		mem[i] = (i * 31 + 7) & 0xff;
	/* 没有声明时回零长度，主机按短读报错而不是一直等 */
	ut_assertok(read_media(uts, f, 0, 0));
	ut_asserteq_str("success", bulkcmd(f, "upload mem 0x8100000 normal 0x19000"));
	ut_assertok(read_media(uts, f, 0, 0x10000));
	ut_asserteq_mem(mem, f->in, 0x10000);
	ut_assertok(read_media(uts, f, 0x1000, 0x1000));
	ut_asserteq_mem(mem + 0x10000, f->in, 0x1000);
	ut_assertok(read_media(uts, f, 0, 0x8000));
	ut_asserteq_mem(mem + 0x11000, f->in, 0x8000);
	ut_assertok(read_media(uts, f, 0, 0));
	ut_assertok(read_media(uts, f, 0x200, 0));
	unmap_sysmem(mem);
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_upload_mem, 0);

#define BLKSZ		512
#define MPT_DISK	SZ_16M
#define MPT_OFF		0xc00000
#define MPT_ENT(i)	(MPT_OFF + 24 + (i) * 40)
#define BOOT_OFF	(MPT_OFF + SZ_1M)

/* Amlogic MPT（与补丁 0201、0401 的测试同一种手写法）；分区名随盘上的表，不写死 */
static void mpt_write(u8 *disk)
{
	static const struct {
		const char *name;
		u64 offset, size;
	} ents[] = {
		{ "bootloader", 0, SZ_4M },
		{ "reserved", MPT_OFF, SZ_1M },
		{ "boot", BOOT_OFF, SZ_1M },
		{ "vendor_boot", MPT_OFF + SZ_2M, MPT_DISK - MPT_OFF - SZ_2M },
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
	for (i = 0; i < 10; i++)
		sum += get_unaligned_le32(disk + MPT_ENT(0) + i * 4);
	put_unaligned_le32(ARRAY_SIZE(ents) * sum, disk + MPT_OFF + 20);
}

static int cmd_test_optimus_download_store(struct unit_test_state *uts)
{
	struct udevice *dev, *blk;
	struct blk_desc *desc;
	struct fake *f;
	u8 *disk, *before, *data, h[32];
	uint i;

	if (CONFIG_AML_PARTITION_OFFSET != MPT_OFF)
		return -EAGAIN;
	f = fake_new();
	ut_assertnonnull(f);
	disk = calloc(1, MPT_DISK);
	before = malloc(MPT_DISK);
	data = malloc(OPTIMUS_FRAME_MAX);
	ut_assertnonnull(disk);
	ut_assertnonnull(before);
	ut_assertnonnull(data);
	memset(disk, 0x5a, MPT_DISK);
	memset(disk + MPT_OFF, 0, SZ_16K);
	mpt_write(disk);
	ut_assertok(blkmap_create("optimus", &dev));
	ut_assertok(blkmap_map_mem(dev, 0, MPT_DISK / BLKSZ, disk));
	ut_assertok(blk_get_from_parent(dev, &blk));
	desc = dev_get_uclass_plat(blk);
	ut_assertok(run_commandf("setenv store_dev blkmap %d", desc->devnum));
	ut_asserteq_str("success", bulkcmd(f, "store init"));
	memcpy(before, disk, MPT_DISK);

	/* 两个 64 KiB 帧加 5 字节的尾帧（尾块读-改-写） */
	ut_asserteq_str("success", bulkcmd(f, "download store boot normal 0x20005"));
	for (i = 0; i < 3; i++) {
		uint len = i < 2 ? OPTIMUS_FRAME_MAX : 5;
		uint k;

		for (k = 0; k < len; k++)
			data[k] = (k * 3 + i * 101 + 1) & 0xff;
		head(h, len, addsum(data, len), 0xef, 0x200);
		ut_asserteq_str("OK!!", media(f, h, 32, data, len));
		ut_asserteq_mem(data, disk + BOOT_OFF + i * OPTIMUS_FRAME_MAX, len);
	}
	ut_asserteq_str("success", bulkcmd(f, "download get_status"));
	ut_asserteq_mem(before, disk, BOOT_OFF);
	ut_asserteq_mem(before + BOOT_OFF + 0x20005, disk + BOOT_OFF + 0x20005,
			MPT_DISK - BOOT_OFF - 0x20005);
	/* 按名字读回（store read）与写进去的一致 */
	ut_asserteq_str("success", bulkcmd(f, "store read boot 0x8200000 0x20000 5"));
	ut_asserteq_mem(disk + BOOT_OFF + 0x20000, map_sysmem(0x8200000, 5), 5);

	/* 帧之间有命令切了硬件分区：拒收、之后整次下载作废，盘不变 */
	ut_asserteq_str("success", bulkcmd(f, "download store boot normal 0x400"));
	head(h, 0x200, addsum(data, 0x200), 0xef, 0x200);
	ut_asserteq_str("OK!!", media(f, h, 32, data, 0x200));
	memcpy(before, disk, MPT_DISK);
	desc->hwpart = 1;
	ut_asserteq_str("failed:the eMMC hardware partition changed during the download",
			media(f, h, 32, data, 0x200));
	desc->hwpart = 0;
	ut_asserteq_str("failed:an earlier media write failed", media(f, h, 32, data, 0x200));
	ut_asserteq_str("failed:download not complete", bulkcmd(f, "download get_status"));
	ut_asserteq_mem(before, disk, MPT_DISK);

	/* 写存储失败（设备比分区表说的小，写越出设备末尾）：应答报错，之后的帧都拒收，get_status 失败 */
	ut_asserteq_str("success", bulkcmd(f, "download store boot normal 0x400"));
	desc->lba = BOOT_OFF / BLKSZ;
	ut_asserteq_str("failed:storage write failed", media(f, h, 32, data, 0x200));
	desc->lba = MPT_DISK / BLKSZ;
	ut_asserteq_str("failed:an earlier media write failed", media(f, h, 32, data, 0x200));
	ut_asserteq_str("failed:download not complete", bulkcmd(f, "download get_status"));
	ut_asserteq_mem(before, disk, MPT_DISK);

	/* 新名字的分区照样按盘上的 MPT 找到；超过分区、未知分区、bootloader、upload store 被拒 */
	ut_asserteq_str("success", bulkcmd(f, "download store vendor_boot normal 0x10"));
	ut_asserteq_str("failed:size is larger than the partition",
			bulkcmd(f, "download store boot normal 0x100001"));
	ut_asserteq_str("failed:no such partition on the store device",
			bulkcmd(f, "download store super normal 0x10"));
	ut_assert(!strncmp("failed:bootloader has a vendor-specific layout",
			   bulkcmd(f, "download store bootloader normal 0x200"), 46));
	ut_asserteq_str("failed:unsupported media", bulkcmd(f, "upload store boot normal 0x10"));
	ut_asserteq(OPTIMUS_XFER_NONE, f->o.xfer);
	memcpy(before, disk, MPT_DISK);
	head(h, 0x10, addsum(data, 0x10), 0xef, 0x200);
	ut_asserteq_str("failed:no download declared", media(f, h, 32, data, 0x10));
	ut_asserteq_mem(before, disk, MPT_DISK);

	env_set("store_dev", NULL);
	ut_assertok(blkmap_destroy(dev));
	free(data);
	free(before);
	free(disk);
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_download_store, 0);

/* 每条主机命令是红线门的一次新输入：被拦的 mmc write 由下一条 bulk 命令 redline yes 执行，同一条命令里的自确认被拒 */
static int cmd_test_optimus_redline_input(struct unit_test_state *uts)
{
	struct blk_desc *desc;
	struct fake *f;
	u8 *blk, pat;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !IS_ENABLED(CONFIG_MMC_WRITE))
		return -EAGAIN;
	f = fake_new();
	ut_assertnonnull(f);
	ut_assertok(run_command("mmc dev 0", 0));
	ut_assertok(blk_get_device_by_str("mmc", "0", &desc));
	blk = memalign(ARCH_DMA_MINALIGN, BLKSZ);
	ut_assertnonnull(blk);
	run_command("redline cancel", 0);
	ut_asserteq(1, blk_dread(desc, 0x40, 1, blk));
	pat = blk[0] ^ 0xff;
	ut_asserteq_str("success", bulkcmdf(f, "mw.b 1000000 %x 200", pat));
	ut_asserteq_str("failed:", bulkcmd(f, "mmc write 1000000 40 1"));
	ut_asserteq(1, blk_dread(desc, 0x40, 1, blk));
	ut_assert(blk[0] != pat);
	/* 同一条命令里先被拦再确认：红线门拒绝 */
	ut_asserteq_str("failed:", bulkcmd(f, "mmc write 1000000 40 1; redline yes"));
	ut_asserteq(1, blk_dread(desc, 0x40, 1, blk));
	ut_assert(blk[0] != pat);
	ut_asserteq_str("success", bulkcmd(f, "redline yes"));
	ut_asserteq(1, blk_dread(desc, 0x40, 1, blk));
	ut_asserteq(pat, blk[0]);
	ut_asserteq_str("failed:", bulkcmd(f, "redline yes"));
	free(blk);
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_redline_input, 0);

static int cmd_test_optimus_leave(struct unit_test_state *uts)
{
	struct fake *f = fake_new();

	ut_assertnonnull(f);
	ut_asserteq_ptr(&f->o, optimus_current());
	ut_assert(!f->o.leave);
	ut_assertok(optimus_request_leave());
	ut_assert(f->o.leave);
	fake_free(f);
	ut_assertnull(optimus_current());
	ut_asserteq(-ENODEV, optimus_request_leave());

	return 0;
}
CMD_TEST(cmd_test_optimus_leave, 0);

/* 线刷会话里 ums 被拦、redline yes 确认：推迟到离开 USB 之后（回 success、请求离开、记下命令，待确认动作还在），在会话外执行时红线门放行；确认 mmc write 时照常就地执行（补丁 0507） */
static int cmd_test_optimus_defer(struct unit_test_state *uts)
{
	struct fake *f;

	if (!IS_ENABLED(CONFIG_CMD_REDLINE) || !IS_ENABLED(CONFIG_CMD_USB_MASS_STORAGE))
		return -EAGAIN;
	f = fake_new();
	ut_assertnonnull(f);
	run_command("redline cancel", 0);
	ut_asserteq_str("failed:", bulkcmd(f, "ums 0 mmc 0"));
	ut_assert(redline_pending_needs_udc());
	ut_asserteq_str("success", bulkcmd(f, "redline yes"));
	ut_assert(f->o.leave);
	ut_assertnonnull(f->o.deferred);
	ut_asserteq_str("redline yes", f->o.deferred);
	ut_assertok(run_command("redline show", 0));
	/* 照 f_optimus 的做法在线刷会话之外执行记下的命令：红线门照常放行（同一次输入的代次，被拦的在更早一次），ums 随之执行并作废待确认动作；sandbox 没有 UDC，ums 自己取控制器失败 */
	run_command(f->o.deferred, 0);
	ut_asserteq(1, run_command("redline show", 0));
	/* 待确认的是 mmc write：不推迟 */
	f->o.leave = false;
	ut_asserteq_str("failed:", bulkcmd(f, "mmc write 1000000 40 1"));
	ut_assert(!redline_pending_needs_udc());
	ut_asserteq_str("success", bulkcmd(f, "redline yes"));
	ut_assert(!f->o.leave);
	run_command("redline cancel", 0);
	fake_free(f);

	return 0;
}
CMD_TEST(cmd_test_optimus_defer, 0);

/* optimus 命令把结果写进 optimus_result：参数不对是 usage；sandbox 没有 UDC，合格的参数走到 error（超时与主机离开要上机） */
static int cmd_test_optimus_result(struct unit_test_state *uts)
{
	static const char *const usage[] = {
		"optimus -t 0 0", "optimus -t 3000x 0", "optimus -t 0o10 0", "optimus x", "optimus",
	};
	int i;

	if (!IS_ENABLED(CONFIG_CMD_OPTIMUS))
		return -EAGAIN;
	for (i = 0; i < ARRAY_SIZE(usage); i++) {
		env_set("optimus_result", NULL);
		ut_asserteq(1, run_command(usage[i], 0));
		ut_assertnonnull(env_get("optimus_result"));
		ut_asserteq_str("usage", env_get("optimus_result"));
	}
	env_set("optimus_result", NULL);
	ut_asserteq(1, run_command("optimus -t 0xbb8 0", 0));
	ut_asserteq_str("error", env_get("optimus_result"));
	env_set("optimus_result", NULL);
	ut_asserteq(1, run_command("optimus 0", 0));
	ut_asserteq_str("error", env_get("optimus_result"));
	env_set("optimus_result", NULL);

	return 0;
}
CMD_TEST(cmd_test_optimus_result, 0);
