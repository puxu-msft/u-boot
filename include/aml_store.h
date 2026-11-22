/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic 线刷命令词 store 的存储后端（cmd/aml_store.c）
 *
 * Amlogic 的 USB 线刷主机（厂商烧录工具、本仓库的 usbdl-mux）对 u-boot 发 store init、store read <分区名> …、store rom_read … 这类命令词，按厂商分区表（eMMC 上是 MPT，补丁 0201 的 part_aml）的分区名或 user 区的 store 坐标读写存储。这里给出这些命令词与 Optimus 协议的 download store 共用的设备与分区解析、按字节读写。
 * store 设备取环境变量 store_dev（"<接口> <设备号>"，如 "mmc 0"），没有就取 CONFIG_CMD_AML_STORE_DEV；经 blk_get_device_by_str() 解析，MMC 因此切回 user 区（厂商 store 只在 user 区按名字定位）。
 */

#ifndef __AML_STORE_H
#define __AML_STORE_H

#include <linux/types.h>

struct blk_desc;
struct disk_partition;

/* store rom_read 的坐标：GXL 起厂商 do_store_rom_read 把偏移加 512 再读 user 区，store 坐标 0 = 物理 0x200 */
#define AML_STORE_ROM_SKEW	0x200

/**
 * aml_store_dev() - 解析 store 设备，分区表未识别时先探测一次
 *
 * Return: 块设备；解析不出时为 NULL（原因已打印）
 */
struct blk_desc *aml_store_dev(void);

/**
 * aml_store_part() - 在 store 设备上按名字查分区
 *
 * @name:	分区名
 * @descp:	返回块设备
 * @info:	返回分区信息
 * Return: 0 成功；-ENODEV 没有 store 设备；-ENOENT 没有这个分区
 */
int aml_store_part(const char *name, struct blk_desc **descp,
		   struct disk_partition *info);

/**
 * aml_store_read() - 按字节读块设备，头尾不对齐时读整块再拷
 *
 * Return: 0 成功；-EINVAL 越过设备末尾；-EIO 读失败；-ENOMEM
 */
int aml_store_read(struct blk_desc *desc, u64 off, u64 len, void *buf);

/**
 * aml_store_write() - 按字节写块设备，头尾不对齐时读出整块、合并后写回
 *
 * Return: 0 成功；-EINVAL 越过设备末尾；-EIO 读写失败；-ENOMEM
 */
int aml_store_write(struct blk_desc *desc, u64 off, u64 len, const void *buf);

#endif /* __AML_STORE_H */
