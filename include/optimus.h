/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic USB 线刷（WorldCup/Optimus）协议的设备侧核心（drivers/optimus），与传输无关
 *
 * 线刷主机（厂商 USB Burning Tool、pyamlboot、本仓库的 usbdl-mux）对 u-boot 线刷态（identify 的阶段为 TPL）发厂商控制请求：0x20 identify、0x30/0x31 TPL 命令与状态、0x34 bulk 命令（应答从 bulk IN 读 512 字节）、0x32 媒体写（控制头 + bulk OUT 数据 + bulk IN 应答）、0x33 媒体读（16 字节控制应答 + bulk IN 数据）。命令词 download/upload 是协议层的数据面，其余命令词交给 run_command()。
 * 核心只做协议判决，USB 收发经 struct optimus_ops 交给传输层（drivers/usb/gadget/f_optimus.c）；测试直接喂控制请求与 bulk 数据（test/cmd/optimus.c）。
 */

#ifndef __OPTIMUS_H
#define __OPTIMUS_H

#include <linux/types.h>

struct usb_ctrlrequest;
struct blk_desc;

#define OPTIMUS_REQ_IDENTIFY	0x20
#define OPTIMUS_REQ_TPL_CMD	0x30
#define OPTIMUS_REQ_TPL_STAT	0x31
#define OPTIMUS_REQ_WRITE_MEDIA	0x32
#define OPTIMUS_REQ_READ_MEDIA	0x33
#define OPTIMUS_REQ_BULKCMD	0x34

/* 命令（含结尾 NUL）与应答的上限：厂商 CMD_BUFF_SIZE、AM_BULK_REPLY_LEN */
#define OPTIMUS_CMD_MAX		512
#define OPTIMUS_REPLY_LEN	512
/* 一帧媒体读写的上限（厂商 transferUnitSz，主机按 64 KiB 分帧） */
#define OPTIMUS_FRAME_MAX	0x10000
/* 0x33 的控制应答长度：[0] 0xefe8，[4] 本帧字节数（厂商 optimus_buf_manager_get_command_data_for_upload_transfer） */
#define OPTIMUS_UPLOAD_INFO_LEN	16

struct optimus;

struct optimus_ops {
	/* 在 bulk IN 上发出 @buf 的 @len 字节（可以为 0）；返回 0 或负的错误码 */
	int (*bulk_in)(struct optimus *o, const void *buf, unsigned int len);
	/* 准备从 bulk OUT 收 @len 字节，收完由传输层调 optimus_bulk_out_done() */
	int (*bulk_out)(struct optimus *o, unsigned int len);
};

enum optimus_xfer {
	OPTIMUS_XFER_NONE,
	OPTIMUS_XFER_DOWNLOAD,
	OPTIMUS_XFER_UPLOAD,
};

struct optimus {
	const struct optimus_ops *ops;
	void *priv;			/* 传输层自用 */
	u8 *frame;			/* OPTIMUS_FRAME_MAX 字节，媒体帧的中转 */
	char reply[OPTIMUS_REPLY_LEN];	/* 最近一条命令的结果：0x31 读它，bulk 命令从 bulk IN 发它 */

	/* download/upload 的声明与进度 */
	enum optimus_xfer xfer;
	bool to_store;			/* 目标是 store 分区，否则是内存 */
	ulong addr;			/* 内存：起点 */
	struct blk_desc *desc;		/* store：块设备 */
	u64 base;			/* store：分区起点的字节偏移 */
	u64 total, done;		/* 声明的字节数、已确认的字节数 */
	bool failed;			/* 写存储出过错（download get_status 报失败，之后的帧都拒收） */

	/* 当前 0x32 帧 */
	u32 frame_len, frame_sum;
	u16 frame_alg, ack_len;
	const char *frame_err;		/* 控制头就不合格时的原因：照收数据，应答报错 */

	bool leave;			/* 命令请求离开线刷态（optimus leave） */
	char *deferred;			/* 离开 USB 之后要执行的命令（redline yes 确认的动作要占用 UDC 时），由传输层取走执行 */
};

/**
 * optimus_init() - 初始化核心并登记为当前实例（optimus leave 作用于它）
 *
 * Return: 0 或 -ENOMEM
 */
int optimus_init(struct optimus *o, const struct optimus_ops *ops, void *priv);

/** optimus_release() - 释放核心并撤销当前实例登记 */
void optimus_release(struct optimus *o);

/**
 * optimus_setup() - 控制请求的 SETUP 阶段
 *
 * @ctrl:	线上格式（小端）的 SETUP 包
 * @buf:	ep0 数据缓冲（至少 OPTIMUS_CMD_MAX 字节）；IN 请求的应答写在这里
 * Return: IN 请求为应答的字节数；OUT 请求为要收的字节数（收完由传输层调 optimus_setup_data()，0 字节时也调）；负数表示 STALL
 */
int optimus_setup(struct optimus *o, const struct usb_ctrlrequest *ctrl, void *buf);

/** optimus_setup_data() - OUT 控制请求的数据阶段收完：执行命令、解析媒体写控制头 */
void optimus_setup_data(struct optimus *o, const struct usb_ctrlrequest *ctrl,
			const void *buf, unsigned int len);

/** optimus_bulk_out_done() - 媒体写的数据收完（@len 字节已在 o->frame）：校验、写入，在 bulk IN 上应答 */
void optimus_bulk_out_done(struct optimus *o, unsigned int len);

/**
 * optimus_run() - 执行一条命令词（bulk 命令与 TPL 命令共用），结果写进 o->reply
 *
 * download/upload/low_power 由协议层处理；其余交给 run_command()，执行前调 redline_input_next()：每条主机命令是一次新输入。redline yes 而待确认的动作要占用 UDC（ums、fastboot）时不在这里执行：记进 o->deferred 并请求离开，传输层离开 USB 之后执行它
 * Return: 0 成功
 */
int optimus_run(struct optimus *o, const char *cmd);

/**
 * optimus_request_leave() - 请当前实例离开线刷态（optimus leave 调它）
 *
 * Return: 0；没有在线刷态时 -ENODEV
 */
int optimus_request_leave(void);

/** optimus_current() - 当前实例；不在线刷态时为 NULL */
struct optimus *optimus_current(void);

/**
 * f_optimus_run() - 在 USB 设备控制器 @controller 上进入线刷态（drivers/usb/gadget/f_optimus.c）
 *
 * @probe_ms:	非 0 时是探测窗口：这么久里主机没有设配置就退出
 * Return: 0 离开（命令请求离开、Ctrl-C）；-ETIMEDOUT 探测窗口内没有主机；其余负数是 USB 初始化失败
 */
int f_optimus_run(int controller, ulong probe_ms);

#endif /* __OPTIMUS_H */
