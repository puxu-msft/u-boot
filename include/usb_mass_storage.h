/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2011 Samsung Electrnoics
 * Lukasz Majewski <l.majewski@samsung.com>
 */

#ifndef __USB_MASS_STORAGE_H__
#define __USB_MASS_STORAGE_H__

#include <part.h>
#include <vsprintf.h>
#include <linux/string.h>
#include <linux/usb/composite.h>

/* Wait at maximum 60 seconds for cable connection */
#define UMS_CABLE_READY_TIMEOUT	60

struct ums {
	int (*read_sector)(struct ums *ums_dev,
			   ulong start, lbaint_t blkcnt, void *buf);
	int (*write_sector)(struct ums *ums_dev,
			    ulong start, lbaint_t blkcnt, const void *buf);
	unsigned int start_sector;
	unsigned int num_sectors;
	const char *name;
	struct blk_desc block_dev;
	int hwpart;
	bool ro;	/* 只读：主机看到写保护介质，写命令一律以 WRITE PROTECTED 拒绝 */
};

/*
 * ums 的选项，顺序不限：-r 只读；-t <ms> 主机 ms 毫秒内没有发命令（一直没发，或发过之后拔线、走开）就离开，十进制、不得为 0。
 * 选项之后只收 <控制器> [<类型>] <设备> 两个或三个参数：命令表的最大参数个数为容下选项放到了 7，不带选项时多出来的参数只有这里能拒绝。
 * 命令本身与红线门（cmd/redline.c 判断是不是只读）共用这份解析。返回选项占的参数个数；选项或参数个数不对返回 -1（命令按用法错误返回，红线门放行它去报用法）
 */
static inline int ums_parse_opts(int argc, char *const argv[], bool *ro, ulong *idle_ms)
{
	char *end;
	int i;

	*ro = false;
	*idle_ms = 0;
	for (i = 1; i < argc && argv[i][0] == '-'; i++) {
		if (!strcmp(argv[i], "-r")) {
			*ro = true;
		} else if (!strcmp(argv[i], "-t") && i + 1 < argc) {
			*idle_ms = simple_strtoul(argv[++i], &end, 10);
			if (*end || !*idle_ms)
				return -1;
		} else {
			return -1;
		}
	}
	if (argc - i < 2 || argc - i > 3)
		return -1;

	return i - 1;
}

/* @idle_ms：ums -t 的空闲时限，0 = 不限 */
int fsg_init(struct ums *ums_devs, int count, struct udevice *udc, ulong idle_ms);
void fsg_cleanup(void);
int fsg_main_thread(void *);
int fsg_add(struct usb_configuration *c);
#endif /* __USB_MASS_STORAGE_H__ */
