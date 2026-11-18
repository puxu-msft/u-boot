/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2023 SberDevices, Inc.
 *
 * Author: Alexey Romanov <avromanov@salutedevices.com>
 */

#ifndef __MESON_SM_CMD_H__
#define __MESON_SM_CMD_H__

enum meson_smc_cmd {
	MESON_SMC_CMD_EFUSE_READ,  /* read efuse memory */
	MESON_SMC_CMD_EFUSE_WRITE, /* write efuse memory */
	MESON_SMC_CMD_CHIP_ID_GET, /* readh chip unique id */
	MESON_SMC_CMD_PWRDM_SET,   /* do command at specified power domain */
	/*
	 * 厂商 BL31 的安全存储（unifykey）：取共享缓冲、装载与只读查询。写、删与改加密类型等改状态的调用不设索引，经 UCLASS_SM 在结构上就发不出来
	 */
	MESON_SMC_CMD_STORAGE_IN_BASE,		/* 存储类请求缓冲的地址 */
	MESON_SMC_CMD_STORAGE_OUT_BASE,		/* 存储类应答缓冲的地址 */
	MESON_SMC_CMD_STORAGE_BLOCK_BASE,	/* 整块加密镜像缓冲的地址 */
	MESON_SMC_CMD_STORAGE_BLOCK_SIZE,	/* 块大小 */
	MESON_SMC_CMD_STORAGE_NOTIFY_EX,	/* x1 = 块大小：按块缓冲重建 BL31 内存里的 key 表 */
	MESON_SMC_CMD_STORAGE_READ,		/* 按名字读数据 */
	MESON_SMC_CMD_STORAGE_TELL,		/* 按名字取数据长度 */
	MESON_SMC_CMD_STORAGE_STATUS,		/* 按名字取属性 */
	MESON_SMC_CMD_STORAGE_LIST,		/* 列出全部名字 */
	MESON_SMC_CMD_STORAGE_VERSION,		/* 最近一次装载认出的块版本 */
	MESON_SMC_CMD_COUNT,
};

#endif
