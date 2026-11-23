// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic USB 线刷（WorldCup/Optimus）协议的设备侧核心，与传输无关（接口与请求一览见 include/optimus.h）
 *
 * 语义对照厂商 GPL 实现（Khadas u-boot v2015.01 drivers/usb/gadget/v2_burning/v2_usb_tool/usb_pcd.c 的 do_vendor_request、bulk_transfer_reply，optimus_transform.c 的 optimus_working，v2_common/optimus_download.c 的 optimus_parse_download_cmd）与线刷主机的客户端语义（本仓库 src/homeiot/amlogic/usb）：
 * - identify 回厂商 u-boot 线刷态的 8 字节 00 07 00 10 00 00 00 00（ROM 版本 0.7、阶段 0.16 即 TPL；厂商 BL33 的应答数组 0x10ca6d0 起就是这 8 字节），主机据此认作 u-boot 线刷态。
 * - bulk 命令（0x34）执行后在 bulk IN 上回 512 字节：成功是 "success"，失败是 "failed:" 加原因（厂商用 memcpy 把令牌盖在原因前面，令牌后的内容主机不解析）。TPL 命令（0x30，wIndex 为 1 才执行）不回 bulk，结果留给 0x31 读。
 * - download <mem|store> <地址|分区名> normal <长度> 与 upload mem <地址> normal <长度> 只声明，数据随后经 0x32/0x33 分帧传；数字照厂商 simple_strtoull(…, 0)，这里另要求整串都是数字。download get_status 在声明的字节全部写完、没有出错时成功。
 * - 0x32 控制头 <IIII HH>：重发次数、帧长、序号、校验和、算法（0xef 按 4 字节小端累加，0xee 不校验）、应答长度；收完帧后在 bulk IN 上回应答长度字节：写入成功是 "OK!!"，否则是 "failed:" 加原因。校验和不符不推进进度，主机可以重发同一帧；写存储失败之后本次下载的帧一律拒收。
 * - 0x33 回 16 字节：[0] 0xefe8，[4] 本帧字节数（主机 wValue 给出的帧长，0 即 64 KiB，与剩余字节数取小），随后在 bulk IN 上发出这么多字节。
 * 不实现、按 STALL 处理的请求：0x01/0x02 任意地址读写、0x05 跳转执行、0x11/0x12 大块读写、0x35 口令、0x36 等（线刷主机只在 ROM 与 BL2 阶段用它们）。download store bootloader 拒绝：厂商按启动区专门的布局写 bootloader（BL2 多份拷贝等），这里只会照分区偏移原样写，落点不同；写启动区走 mmc write（物理偏移、经红线门）。
 * 内存目标（download/upload mem）须整段在 DRAM 里、不碰 lmb 里 U-Boot 自身与固件的保留区（NOOVERWRITE/NOMAP）；存储目标不越过分区。确认门在主机侧：download store 这类协议层写入不是 u-boot 命令，不经 cmd_process() 的红线门（usbdl-mux 按设备策略列清单确认）；经 run_command() 执行的命令照常经红线门。
 */

#include <aml_store.h>
#include <alist.h>
#include <blk.h>
#include <command.h>
#include <errno.h>
#include <lmb.h>
#include <malloc.h>
#include <mapmem.h>
#include <memalign.h>
#include <optimus.h>
#include <part.h>
#include <vsprintf.h>
#include <asm/global_data.h>
#include <asm/unaligned.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/usb/ch9.h>

DECLARE_GLOBAL_DATA_PTR;

#define ALG_ADDSUM	0x00ef
#define ALG_NONE	0x00ee
#define UPLOAD_TAG	0xefe8

/* 厂商 u-boot 线刷态 identify 的应答（ROM 0.7、阶段 0.16 = TPL） */
static const u8 optimus_id[8] = { 0, 7, 0, 16, 0, 0, 0, 0 };

static struct optimus *optimus_cur;

struct optimus *optimus_current(void)
{
	return optimus_cur;
}

int optimus_request_leave(void)
{
	if (!optimus_cur)
		return -ENODEV;
	optimus_cur->leave = true;

	return 0;
}

int optimus_init(struct optimus *o, const struct optimus_ops *ops, void *priv)
{
	memset(o, '\0', sizeof(*o));
	o->frame = memalign(ARCH_DMA_MINALIGN, OPTIMUS_FRAME_MAX);
	if (!o->frame)
		return -ENOMEM;
	o->ops = ops;
	o->priv = priv;
	optimus_cur = o;

	return 0;
}

void optimus_release(struct optimus *o)
{
	free(o->frame);
	o->frame = NULL;
	free(o->deferred);
	o->deferred = NULL;
	if (optimus_cur == o)
		optimus_cur = NULL;
}

/* 按 4 字节小端累加，尾部不足 4 字节按小端取实际字节数（主机 frames.addsum、厂商 add_sum） */
static u32 optimus_addsum(const u8 *p, u32 len)
{
	u32 sum = 0, i, tail = 0;

	for (i = 0; i + 4 <= len; i += 4)
		sum += get_unaligned_le32(p + i);
	for (; i < len; i++)
		tail |= (u32)p[i] << ((i % 4) * 8);

	return sum + tail;
}

/* 内存目标 [@addr, @addr + @len) 不合格的原因；合格时为 NULL */
static const char *optimus_mem_bad(u64 addr, u64 len)
{
	if (!len)
		return "zero length";
	if (addr < gd->ram_base || len > gd->ram_size ||
	    addr - gd->ram_base > gd->ram_size - len)
		return "range not inside DRAM";
	if (CONFIG_IS_ENABLED(LMB)) {
		const struct lmb_region *r;

		alist_for_each(r, &lmb_get()->used_mem)
			if ((r->flags & (LMB_NOMAP | LMB_NOOVERWRITE)) &&
			    r->base < addr + len && addr < r->base + r->size)
				return "range overlaps memory reserved for U-Boot or firmware";
	}

	return NULL;
}

static int optimus_num(const char *s, u64 *val)
{
	char *end;

	*val = simple_strtoull(s, &end, 0);

	return end == s || *end ? -EINVAL : 0;
}

static int optimus_xfer_cmd(struct optimus *o, int argc, char *const argv[],
			    const char **why)
{
	bool up = !strcmp(argv[0], "upload");
	struct disk_partition info;
	struct blk_desc *desc;
	u64 size, addr;

	if (!up && argc == 2 && !strcmp(argv[1], "get_status")) {
		if (o->xfer == OPTIMUS_XFER_DOWNLOAD && !o->failed &&
		    o->done == o->total)
			return 0;
		*why = "download not complete";
		return -EIO;
	}
	if (argc != 5) {
		*why = "usage: download|upload <mem|store> <address|partition> normal <size>";
		return -EINVAL;
	}
	/* 新的声明取代上一次的，不论是否合格；控制头已收、数据还没到的那一帧是按旧声明核过的，作废 */
	o->xfer = OPTIMUS_XFER_NONE;
	o->failed = false;
	o->done = 0;
	o->total = 0;
	o->frame_err = "a new transfer was declared before the frame data arrived";
	if (strcmp(argv[3], "normal")) {
		*why = "only image type normal is supported";
		return -EOPNOTSUPP;
	}
	if (optimus_num(argv[4], &size) || !size) {
		*why = "bad size";
		return -EINVAL;
	}
	if (!strcmp(argv[1], "mem")) {
		if (optimus_num(argv[2], &addr)) {
			*why = "bad address";
			return -EINVAL;
		}
		*why = optimus_mem_bad(addr, size);
		if (*why)
			return -EFAULT;
		o->to_store = false;
		o->addr = addr;
	} else if (!up && !strcmp(argv[1], "store")) {
		if (!strcmp(argv[2], "bootloader")) {
			*why = "bootloader has a vendor-specific layout; write it with mmc write at its physical offset";
			return -EOPNOTSUPP;
		}
		if (aml_store_part(argv[2], &desc, &info)) {
			*why = "no such partition on the store device";
			return -ENOENT;
		}
		if (size > (u64)info.size * info.blksz) {
			*why = "size is larger than the partition";
			return -EFBIG;
		}
		o->to_store = true;
		o->desc = desc;
		o->hwpart = desc->hwpart;
		o->base = (u64)info.start * info.blksz;
	} else {
		*why = "unsupported media";
		return -EOPNOTSUPP;
	}
	o->xfer = up ? OPTIMUS_XFER_UPLOAD : OPTIMUS_XFER_DOWNLOAD;
	o->total = size;
	printf("optimus: %s %s %s 0x%llx bytes\n", argv[0], argv[1], argv[2], size);

	return 0;
}

int optimus_run(struct optimus *o, const char *cmd)
{
	char line[OPTIMUS_CMD_MAX], *argv[6], *p = line;
	const char *why = "";
	int argc = 0, ret;

	/* 512 字节的数据阶段里没有 NUL：解析副本放不下，run_command 却会拿到全长，两者不一致，拒绝 */
	if (strlen(cmd) >= sizeof(line)) {
		why = "command too long";
		ret = -E2BIG;
		goto reply;
	}
	strlcpy(line, cmd, sizeof(line));
	while (*p && argc < ARRAY_SIZE(argv)) {
		while (isspace(*p))
			p++;
		if (!*p)
			break;
		argv[argc++] = p;
		while (*p && !isspace(*p))
			p++;
		if (*p)
			*p++ = '\0';
	}
	while (isspace(*p))
		p++;
	if (!argc) {
		why = "no command";
		ret = -EINVAL;
	} else if (!strcmp(argv[0], "download") || !strcmp(argv[0], "upload")) {
		if (*p) {
			why = "too many arguments";
			ret = -EINVAL;
		} else {
			ret = optimus_xfer_cmd(o, argc, argv, &why);
		}
	} else if (argc == 1 && !strcmp(argv[0], "low_power")) {
		/* 厂商用它关背光、限 VBUS 电流；本机没有要做的 */
		ret = 0;
	} else {
		/* 每条主机命令是一次新输入：红线门据此区分“同一次输入里的自确认” */
		redline_input_next();
		if (argc == 2 && !strcmp(argv[0], "redline") && !strcmp(argv[1], "yes") &&
		    redline_pending_needs_udc()) {
			/* 被拦的是 ums、fastboot 这类要占用 UDC 的命令：先回应答、离开 USB，再由传输层执行（同一次输入，代次不再加） */
			free(o->deferred);
			o->deferred = strdup(cmd);
			o->leave = true;
			ret = o->deferred ? 0 : -ENOMEM;
			why = "out of memory";
			if (!ret)
				printf("optimus: '%s' runs after leaving USB burning mode\n", cmd);
		} else {
			ret = run_command(cmd, 0);
		}
	}
reply:
	memset(o->reply, '\0', sizeof(o->reply));
	if (ret)
		snprintf(o->reply, sizeof(o->reply), "failed:%s", why);
	else
		strcpy(o->reply, "success");

	return ret;
}

/* 0x33：16 字节的帧信息写进 @buf，本帧数据经 bulk IN 发出 */
static int optimus_read_media(struct optimus *o, u16 value, u16 length, u8 *buf)
{
	u32 n = 0;
	void *src;

	if (length < 8)
		return -EINVAL;
	if (o->xfer == OPTIMUS_XFER_UPLOAD) {
		n = value ? value : OPTIMUS_FRAME_MAX;
		n = min_t(u64, n, o->total - o->done);
	}
	if (n) {
		src = map_sysmem(o->addr + o->done, n);
		memcpy(o->frame, src, n);
		unmap_sysmem(src);
		o->done += n;
	}
	length = min_t(u16, length, OPTIMUS_UPLOAD_INFO_LEN);
	memset(buf, '\0', length);
	put_unaligned_le32(UPLOAD_TAG, buf);
	put_unaligned_le32(n, buf + 4);
	o->ops->bulk_in(o, o->frame, n);

	return length;
}

/* 认得的厂商请求的 SETUP 阶段：IN 请求返回应答字节数，OUT 请求返回要收的字节数，不认得或不合格的返回负数（STALL） */
static int optimus_setup_vendor(struct optimus *o, const struct usb_ctrlrequest *ctrl, void *buf)
{
	u16 length = le16_to_cpu(ctrl->wLength);
	u16 value = le16_to_cpu(ctrl->wValue);
	bool in = ctrl->bRequestType & USB_DIR_IN;

	switch (ctrl->bRequest) {
	case OPTIMUS_REQ_IDENTIFY:
		if (!in)
			return -EINVAL;
		length = min_t(u16, length, sizeof(optimus_id));
		memcpy(buf, optimus_id, length);
		return length;
	case OPTIMUS_REQ_TPL_STAT:
		if (!in)
			return -EINVAL;
		length = min_t(u16, length, OPTIMUS_REPLY_LEN);
		memcpy(buf, o->reply, length);
		return length;
	case OPTIMUS_REQ_READ_MEDIA:
		if (!in)
			return -EINVAL;
		return optimus_read_media(o, value, length, buf);
	case OPTIMUS_REQ_TPL_CMD:
	case OPTIMUS_REQ_BULKCMD:
		if (in)
			return -EINVAL;
		if (length > OPTIMUS_CMD_MAX)
			return -E2BIG;
		return length;
	case OPTIMUS_REQ_WRITE_MEDIA:
		/* 32 字节是带算法与应答长度的新格式，16 字节是旧格式（厂商 do_vendor_out_complete 两种都认） */
		if (in || (length != 32 && length != 16))
			return -EINVAL;
		return length;
	default:
		return -EOPNOTSUPP;
	}
}

int optimus_setup(struct optimus *o, const struct usb_ctrlrequest *ctrl, void *buf)
{
	int ret;

	if ((ctrl->bRequestType & USB_TYPE_MASK) != USB_TYPE_VENDOR)
		return -EOPNOTSUPP;
	ret = optimus_setup_vendor(o, ctrl, buf);
	/* 主机在 = 收到过认得、且合格的线刷请求：未知请求、被 STALL 的请求与标准请求（枚举、设配置）都不算 */
	if (ret >= 0)
		o->host_seen = true;

	return ret;
}

static void optimus_media_reply(struct optimus *o, const char *err)
{
	memset(o->reply, '\0', sizeof(o->reply));
	if (err) {
		snprintf(o->reply, sizeof(o->reply), "failed:%s", err);
		printf("optimus: media write: %s\n", err);
	} else {
		memcpy(o->reply, "OK!!", 4);
	}
	o->ops->bulk_in(o, o->reply, o->ack_len);
}

static void optimus_write_media(struct optimus *o, const u8 *head, uint len)
{
	u32 frame = get_unaligned_le32(head + 4);

	o->frame_len = frame;
	o->frame_sum = get_unaligned_le32(head + 12);
	o->frame_alg = ALG_ADDSUM;
	o->ack_len = OPTIMUS_REPLY_LEN;
	if (len >= 20) {
		o->frame_alg = get_unaligned_le16(head + 16);
		o->ack_len = get_unaligned_le16(head + 18);
		if (!o->ack_len || o->ack_len > OPTIMUS_REPLY_LEN)
			o->ack_len = OPTIMUS_REPLY_LEN;
	}
	if (o->xfer != OPTIMUS_XFER_DOWNLOAD)
		o->frame_err = "no download declared";
	else if (o->failed)
		o->frame_err = "an earlier media write failed";
	else if (!frame || frame > OPTIMUS_FRAME_MAX)
		o->frame_err = "frame length out of range";
	else if (frame > o->total - o->done)
		o->frame_err = "frame runs past the declared size";
	else if (o->frame_alg != ALG_ADDSUM && o->frame_alg != ALG_NONE)
		o->frame_err = "unsupported checksum algorithm";
	else
		o->frame_err = NULL;
	if (!frame) {
		optimus_media_reply(o, o->frame_err);
		return;
	}
	/* 头不合格也照收数据（至多一帧），免得主机卡在 bulk OUT 上，应答里报原因 */
	o->ops->bulk_out(o, min_t(u32, frame, OPTIMUS_FRAME_MAX));
}

void optimus_setup_data(struct optimus *o, const struct usb_ctrlrequest *ctrl,
			const void *buf, unsigned int len)
{
	char cmd[OPTIMUS_CMD_MAX + 1];

	switch (ctrl->bRequest) {
	case OPTIMUS_REQ_BULKCMD:
	case OPTIMUS_REQ_TPL_CMD:
		len = min_t(uint, len, OPTIMUS_CMD_MAX);
		memcpy(cmd, buf, len);
		cmd[len] = '\0';
		if (ctrl->bRequest == OPTIMUS_REQ_TPL_CMD) {
			/* 子码 1 才执行（厂商 do_vendor_out_complete），结果留给 0x31 */
			if (le16_to_cpu(ctrl->wIndex) == 1)
				optimus_run(o, cmd);
			return;
		}
		optimus_run(o, cmd);
		o->ops->bulk_in(o, o->reply, OPTIMUS_REPLY_LEN);
		return;
	case OPTIMUS_REQ_WRITE_MEDIA:
		optimus_write_media(o, buf, len);
		return;
	}
}

void optimus_bulk_out_done(struct optimus *o, unsigned int len)
{
	const char *err = o->frame_err;
	void *dst;

	if (!err && len != o->frame_len)
		err = "short frame";
	if (!err && o->frame_alg == ALG_ADDSUM &&
	    optimus_addsum(o->frame, len) != o->frame_sum)
		err = "checksum mismatch";
	if (err) {
		optimus_media_reply(o, err);
		return;
	}
	if (o->to_store) {
		/* 帧之间有命令切了硬件分区（mmc dev <n> <part>）：照分区偏移写会落到别的硬件分区上 */
		if (o->desc->hwpart != o->hwpart) {
			o->failed = true;
			optimus_media_reply(o, "the eMMC hardware partition changed during the download");
			return;
		}
		if (aml_store_write(o->desc, o->base + o->done, len, o->frame)) {
			o->failed = true;
			optimus_media_reply(o, "storage write failed");
			return;
		}
	} else {
		dst = map_sysmem(o->addr + o->done, len);
		memcpy(dst, o->frame, len);
		unmap_sysmem(dst);
	}
	o->done += len;
	optimus_media_reply(o, NULL);
}
