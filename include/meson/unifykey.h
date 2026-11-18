/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * 经 Amlogic 厂商 BL31 的安全存储读 unifykey（mac、deviceid、authkey 等出厂写入的 key）
 *
 * BL31 自己解密、校验并在内存里建 key 表，普通世界只做三件事：把 eMMC 上的加密块原样交进 BL31 的块缓冲、发 NOTIFY_EX 让它装载、之后按名字读。本客户端只读块设备、只发 include/meson/sm.h 里的只读与装载调用，不写回 eMMC（厂商 u-boot 在两份副本不一致时会写盘修复，这里不做）。
 */

#ifndef __MESON_UNIFYKEY_H__
#define __MESON_UNIFYKEY_H__

#include <linux/bitops.h>
#include <linux/types.h>

struct blk_desc;
struct disk_partition;

/* BL31 拒绝更长的名字（存储类调用 cmp w1, #0x4f） */
#define MESON_UNIFYKEY_NAME_MAX		79

/* STATUS 给出的属性位：安全属性的 key 普通世界读不到 */
#define MESON_UNIFYKEY_ATTR_SECURE	BIT(0)
#define MESON_UNIFYKEY_ATTR_ENCRYPT	BIT(8)

/* eMMC 上一份副本：info 扇区记的值与现算的结果 */
struct meson_unifykey_copy {
	u64 checksum;	/* info 记的：整份副本按小端 u32 零扩展累加的 64 位和 */
	u64 sum;	/* 现算的 */
	u32 stamp;	/* 厂商每写一次加 1，两份都有效时新者胜 */
	u32 magic;	/* 厂商写 9，只判非 0 */
	bool valid;	/* magic 非 0 且 sum == checksum，与厂商同一判据 */
};

/*
 * meson_unifykey_load() 的经过：两份副本的判定、交给 BL31 的是哪份、该份块头的预检（BL31 自己还会再验一次，这里只为说清失败原因），以及装载后从 BL31 读回的状态
 */
struct meson_unifykey_load {
	struct meson_unifykey_copy copy[2];
	bool same;		/* 两份副本与两块 info 的前 16 B 都逐字节相同 */
	int pick;		/* 交给 BL31 的副本 */
	bool hdr_magic;		/* 块头 "AMLSECURITY\0" */
	s32 hdr_version;	/* 块头 +0x10；BL31 只在它 > 0 时验 SHA-256 */
	u32 hdr_enctype;	/* 块头 +0x14：BL31 按它选密钥 */
	bool hdr_sha_ok;	/* sha256(块[0x200, 块大小)) 与块头 +0x18 相符 */
	u32 block_size;		/* 装载后 BL31 的块大小：内层头声明的大小与交给它的不同时会变，同时清表 */
	s32 version;		/* 装载后 BL31 的 VERSION：块头魔数没对上时为 -1 */
	u32 keys;		/* 装载后表里的 key 数（LIST） */
};

/**
 * meson_unifykey_load() - 从分区读出 unifykey 块，按厂商规则选出副本，交给 BL31 装载
 *
 * 布局（相对分区起点，CONFIG_MESON_UNIFYKEY_OFFSET/SIZE）：副本 i 在 OFFSET + i * SIZE，其后两块 512 B 的 info，info i 在 OFFSET + 2 * SIZE + i * 512。
 * 两份都有效时取 stamp 新的（有符号差，相等取副本 0），只有一份有效取它，都无效时与厂商一样交副本 0，由 BL31 的 SHA-256 判定。
 * NOTIFY_EX 恒返回 0，成败按装载后读回的块大小（须仍等于 SIZE）与表里的 key 数（须大于 0）判定。
 *
 * @desc:	块设备
 * @part:	厂商 reserved 分区
 * @res:	经过与结果；出错时填到出错那一步为止
 * Return: 0 表里有 key；-ENODEV 没有安全监控设备；-ENOSYS 安全监控没有存储调用（例如不是厂商 BL31）；-ERANGE 分区放不下或布局不按块对齐；-ENOMEM；-EIO 读盘失败；-EPROTO 安全监控的应答不合协议；-ENODATA 已交 BL31 但表是空的或块大小变了
 */
int meson_unifykey_load(struct blk_desc *desc, const struct disk_partition *part,
			struct meson_unifykey_load *res);

/**
 * meson_unifykey_list() - BL31 表里全部 key 的名字
 *
 * @buf:	名字依次写入，各以 NUL 结尾
 * @size:	@buf 的大小
 * @len:	写入的字节数
 * Return: 0；-ENODEV；-ENOSYS；-E2BIG @buf 放不下；-EPROTO 应答不合协议
 */
int meson_unifykey_list(char *buf, size_t size, size_t *len);

/**
 * meson_unifykey_stat() - 按名字取数据长度与属性
 *
 * Return: 0；-EINVAL 名字长度不在 1..MESON_UNIFYKEY_NAME_MAX；-ENOENT 不存在；-ENODEV；-ENOSYS；-EPROTO
 */
int meson_unifykey_stat(const char *name, u32 *len, u32 *attr);

/**
 * meson_unifykey_read() - 按名字读数据：STATUS（不存在、安全属性）、TELL（长度）、READ。厂商 keyman read 在这之前还发 QUERY，它的判据（存在且数据非空）被 STATUS 与 TELL 覆盖，这里不发
 *
 * @name:	key 名
 * @buf:	数据写到这里
 * @size:	@buf 的大小
 * @len:	数据长度；返回 -E2BIG 时同样给出
 * Return: 0；-EINVAL；-ENOENT 未烧录；-EACCES 安全属性（普通世界读不到）；-E2BIG @buf 小于数据；-ENODEV；-ENOSYS；-EPROTO
 */
int meson_unifykey_read(const char *name, void *buf, size_t size, u32 *len);

#endif
