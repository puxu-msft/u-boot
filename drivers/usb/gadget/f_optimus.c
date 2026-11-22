// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic USB 线刷（Optimus）协议的 USB 设备函数：把 drivers/optimus 的协议核心接到 g_dnl 与 UDC 上（M12 上是 DWC2）
 *
 * 枚举形态照厂商 u-boot 线刷态（Khadas u-boot v2015.01 v2_usb_tool/usb_pcd.c 的 device_desc 与 _configDescTotal）：VID:PID 1b8e:c003、bcdDevice 0x0007（主机 homeiot.amlogic.usb 的 BCD_STAGE 据此认作 u-boot 线刷态）、一个厂商类接口（0xff/0/0）带一对 bulk 端点。g_dnl 的设备描述符是全局共用的：g_dnl_bind_fixup() 只在绑定 usb_dnl_optimus 时换成上面的 VID:PID 与 bcdDevice，绑定别的函数（fastboot、ums、ACM）时恢复 Kconfig 的 VID:PID 与 g_dnl 的缺省 bcdDevice。这两个钩子是 g_dnl 的弱函数，板级代码另有定义时会在链接时冲突，届时把这里的逻辑并进板级那份。
 * 控制请求全是 bmRequestType 0x40/0xc0 的设备级厂商请求，wIndex 不是接口号（bulk 命令带 2，媒体读写带块数），所以 composite 对单接口配置的设备级请求要交给唯一的接口（补丁同改 composite.c）。
 * OUT 帧按端点最大包长取整后收，主机最后一个短包结束传输；bulk IN 的上一条应答主机还没取走时，丢掉它、改发新的。
 * 离开：命令执行中请求了离开（optimus leave，见 m12.env 的 storeboot 与 m12_rescue_*）时，等这条命令的应答被主机取走再退出；主机一直不取，LEAVE_GRACE_MS 后照样退出。
 */

#include <command.h>
#include <console.h>
#include <dm.h>
#include <errno.h>
#include <g_dnl.h>
#include <malloc.h>
#include <memalign.h>
#include <optimus.h>
#include <time.h>
#include <usb.h>
#include <watchdog.h>
#include <linux/kernel.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#define OPTI_VID		0x1b8e
#define OPTI_PID		0xc003
#define OPTI_BCD		0x0007
#define OPTI_BUF		(OPTIMUS_FRAME_MAX + 1024)
#define LEAVE_GRACE_MS		2000

struct f_optimus {
	struct usb_function func;
	struct usb_ep *in_ep, *out_ep;
	struct usb_request *in_req, *out_req;
	struct optimus core;
	struct usb_ctrlrequest ctrl;	/* 数据阶段还没收完的 OUT 控制请求 */
	bool seen_host;			/* 主机设过配置（探测窗口据此判定主机在） */
	bool in_busy;
	bool leave_on_in;		/* 请求了离开：这条应答被取走就退出 */
	bool leaving;
	ulong leave_at;
};

static struct f_optimus *opti;
static bool opti_binding;

static struct usb_interface_descriptor opti_intf = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bNumEndpoints		= 2,
	.bInterfaceClass	= USB_CLASS_VENDOR_SPEC,
};

static struct usb_endpoint_descriptor fs_in = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(64),
};

static struct usb_endpoint_descriptor fs_out = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(64),
};

static struct usb_endpoint_descriptor hs_in = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_out = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= cpu_to_le16(512),
};

static struct usb_descriptor_header *opti_fs[] = {
	(struct usb_descriptor_header *)&opti_intf,
	(struct usb_descriptor_header *)&fs_in,
	(struct usb_descriptor_header *)&fs_out,
	NULL,
};

static struct usb_descriptor_header *opti_hs[] = {
	(struct usb_descriptor_header *)&opti_intf,
	(struct usb_descriptor_header *)&hs_in,
	(struct usb_descriptor_header *)&hs_out,
	NULL,
};

/* g_dnl 的弱钩子：只在绑定 Optimus 时换成厂商线刷态的 VID:PID 与 bcdDevice */
int g_dnl_bind_fixup(struct usb_device_descriptor *dev, const char *name)
{
	opti_binding = !strcmp(name, "usb_dnl_optimus");
	dev->idVendor = cpu_to_le16(opti_binding ? OPTI_VID :
				    CONFIG_USB_GADGET_VENDOR_NUM);
	dev->idProduct = cpu_to_le16(opti_binding ? OPTI_PID :
				     CONFIG_USB_GADGET_PRODUCT_NUM);

	return 0;
}

int g_dnl_get_board_bcd_device_number(int gcnum)
{
	return opti_binding ? OPTI_BCD : gcnum;
}

static void opti_note_leave(void)
{
	if (!opti->core.leave || opti->leaving)
		return;
	opti->leaving = true;
	opti->leave_at = get_timer(0);
	if (opti->in_busy)
		opti->leave_on_in = true;
	else
		g_dnl_trigger_detach();
}

static void opti_in_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (!opti)
		return;
	opti->in_busy = false;
	if (opti->leave_on_in)
		g_dnl_trigger_detach();
}

static void opti_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (!opti || req->status)
		return;
	memcpy(opti->core.frame, req->buf,
	       min_t(uint, req->actual, OPTIMUS_FRAME_MAX));
	optimus_bulk_out_done(&opti->core, req->actual);
	opti_note_leave();
}

static int opti_bulk_in(struct optimus *o, const void *buf, unsigned int len)
{
	struct f_optimus *f = o->priv;
	int ret;

	if (!f->in_req)
		return -ENODEV;
	if (f->in_busy)
		usb_ep_dequeue(f->in_ep, f->in_req);
	memcpy(f->in_req->buf, buf, len);
	f->in_req->length = len;
	f->in_req->zero = 0;
	f->in_busy = true;
	ret = usb_ep_queue(f->in_ep, f->in_req, 0);
	if (ret)
		f->in_busy = false;

	return ret;
}

static int opti_bulk_out(struct optimus *o, unsigned int len)
{
	struct f_optimus *f = o->priv;
	uint maxp;

	if (!f->out_req)
		return -ENODEV;
	maxp = f->out_ep->maxpacket ? f->out_ep->maxpacket : 512;
	f->out_req->length = ALIGN(len, maxp);

	return usb_ep_queue(f->out_ep, f->out_req, 0);
}

static const struct optimus_ops opti_ops = {
	.bulk_in = opti_bulk_in,
	.bulk_out = opti_bulk_out,
};

static void opti_ep0_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	if (!opti || req->status)
		return;
	optimus_setup_data(&opti->core, &opti->ctrl, req->buf, req->actual);
	opti_note_leave();
}

static int opti_setup(struct usb_function *func,
		      const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = func->config->cdev;
	struct usb_request *req = cdev->req;
	bool in = ctrl->bRequestType & USB_DIR_IN;
	int n, ret;

	n = optimus_setup(&opti->core, ctrl, req->buf);
	if (n < 0)
		return n;
	req->length = n;
	req->zero = in && n < le16_to_cpu(ctrl->wLength);
	if (!in && n) {
		opti->ctrl = *ctrl;
		req->complete = opti_ep0_out_complete;
	}
	ret = usb_ep_queue(cdev->gadget->ep0, req, 0);
	if (ret)
		return ret;
	if (!in && !n) {
		optimus_setup_data(&opti->core, ctrl, req->buf, 0);
		opti_note_leave();
	}

	return n;
}

static struct usb_request *opti_alloc_req(struct usb_ep *ep,
					  void (*complete)(struct usb_ep *,
							   struct usb_request *))
{
	struct usb_request *req = usb_ep_alloc_request(ep, 0);

	if (!req)
		return NULL;
	req->buf = memalign(CONFIG_SYS_CACHELINE_SIZE, OPTI_BUF);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}
	req->complete = complete;

	return req;
}

static void opti_free_req(struct usb_ep *ep, struct usb_request **req)
{
	if (!*req)
		return;
	free((*req)->buf);
	usb_ep_free_request(ep, *req);
	*req = NULL;
}

static void opti_disable(struct usb_function *func)
{
	usb_ep_disable(opti->out_ep);
	usb_ep_disable(opti->in_ep);
	opti_free_req(opti->out_ep, &opti->out_req);
	opti_free_req(opti->in_ep, &opti->in_req);
	opti->in_busy = false;
}

static int opti_set_alt(struct usb_function *func, unsigned int intf,
			unsigned int alt)
{
	struct usb_gadget *g = func->config->cdev->gadget;
	bool hs = gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH;
	int ret;

	ret = usb_ep_enable(opti->in_ep, hs ? &hs_in : &fs_in);
	if (ret)
		return ret;
	ret = usb_ep_enable(opti->out_ep, hs ? &hs_out : &fs_out);
	if (ret)
		goto err;
	opti->in_req = opti_alloc_req(opti->in_ep, opti_in_complete);
	opti->out_req = opti_alloc_req(opti->out_ep, opti_out_complete);
	if (!opti->in_req || !opti->out_req) {
		ret = -ENOMEM;
		goto err;
	}
	opti->seen_host = true;

	return 0;
err:
	opti_disable(func);

	return ret;
}

static int opti_bind(struct usb_configuration *c, struct usb_function *func)
{
	struct usb_gadget *g = c->cdev->gadget;
	int id;

	id = usb_interface_id(c, func);
	if (id < 0)
		return id;
	opti_intf.bInterfaceNumber = id;
	opti->in_ep = usb_ep_autoconfig(g, &fs_in);
	opti->out_ep = usb_ep_autoconfig(g, &fs_out);
	if (!opti->in_ep || !opti->out_ep)
		return -ENODEV;
	opti->in_ep->driver_data = c->cdev;
	opti->out_ep->driver_data = c->cdev;
	func->descriptors = opti_fs;
	if (gadget_is_dualspeed(g)) {
		hs_in.bEndpointAddress = fs_in.bEndpointAddress;
		hs_out.bEndpointAddress = fs_out.bEndpointAddress;
		func->hs_descriptors = opti_hs;
	}

	return 0;
}

static int opti_add(struct usb_configuration *c)
{
	if (!opti)
		return -ENODEV;
	opti->func.name = "f_optimus";
	opti->func.bind = opti_bind;
	opti->func.set_alt = opti_set_alt;
	opti->func.disable = opti_disable;
	opti->func.setup = opti_setup;

	return usb_add_function(c, &opti->func);
}

DECLARE_GADGET_BIND_CALLBACK(usb_dnl_optimus, opti_add);

int f_optimus_run(int controller, ulong probe_ms)
{
	struct f_optimus *f;
	struct udevice *udc;
	ulong start;
	int ret;

	if (opti) {
		puts("optimus: already running\n");
		return -EBUSY;
	}
	f = calloc(1, sizeof(*f));
	if (!f)
		return -ENOMEM;
	ret = optimus_init(&f->core, &opti_ops, f);
	if (ret)
		goto out_free;
	opti = f;
	ret = udc_device_get_by_index(controller, &udc);
	if (ret)
		goto out_core;
	g_dnl_clear_detach();
	ret = g_dnl_register("usb_dnl_optimus");
	if (ret)
		goto out_put;
	start = get_timer(0);
	if (probe_ms)
		printf("optimus: USB burning mode, waiting %lu ms for a host\n", probe_ms);
	else
		puts("optimus: USB burning mode, waiting for the host\n");
	while (!g_dnl_detach()) {
		if (ctrlc()) {
			puts("optimus: interrupted\n");
			break;
		}
		if (probe_ms && !f->seen_host && get_timer(start) > probe_ms) {
			ret = -ETIMEDOUT;
			break;
		}
		if (f->core.leave && !f->leaving)
			opti_note_leave();
		if (f->leaving && get_timer(f->leave_at) > LEAVE_GRACE_MS)
			break;
		schedule();
		dm_usb_gadget_handle_interrupts(udc);
	}
	g_dnl_unregister();
	g_dnl_clear_detach();
out_put:
	udc_device_put(udc);
out_core:
	optimus_release(&f->core);
	opti = NULL;
out_free:
	free(f);

	return ret;
}
