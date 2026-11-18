/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic 厂商 BL31 安全存储的 sandbox 模型（drivers/sm/sandbox-meson-sm.c）给测试的接口
 */

#ifndef __SANDBOX_MESON_SM_H__
#define __SANDBOX_MESON_SM_H__

#include <linux/types.h>

struct udevice;

/*
 * 代替 eFuse 的测试密钥材料：真 BL31 的默认加密类型用 eFuse 字节 [4,16) 派生密钥，普通世界读不到。主机侧 tools/u-boot/gen-unifykey-blob.py（M12 设备仓库）的默认值与此相同，它打出的块能被本模型装载
 */
#define SANDBOX_MESON_SM_EFUSE_4_16	"m12-test-efu"	/* 12 字节 */
#define SANDBOX_MESON_SM_EFUSE_40_48	"efuse-40"	/* 8 字节，加密类型 1 用 */
#define SANDBOX_MESON_SM_EFUSE_4_16_LEN	12

/**
 * sandbox_meson_sm_key() - 按块头加密类型给出 AES-256-CBC 的密钥与 IV（真 BL31 0x5120718 的规则；eFuse 字节 [4,16) 取当前设定，[0x40,0x48) 取上面的测试值）
 */
void sandbox_meson_sm_key(u32 enctype, u8 key[32], u8 iv[16]);

/**
 * sandbox_meson_sm_efuse_4_16() - 模型当前用的 eFuse 字节 [4,16)（SANDBOX_MESON_SM_EFUSE_4_16_LEN 字节）；缺省是 SANDBOX_MESON_SM_EFUSE_4_16
 */
const u8 *sandbox_meson_sm_efuse_4_16(void);

/**
 * sandbox_meson_sm_set_efuse_4_16() - 换掉模型的 eFuse 字节 [4,16)，此后的装载与 sandbox_meson_sm_key() 都按新值派生密钥
 *
 * 命令行 --meson_sm_efuse_4_16 <24 位 hex> 在启动时调它：给出真机的值（主线 `sm serial` 写进内存的 12 字节原样即是）时，模型能装载真机 reserved 分区里的块。这是全进程的设定，与芯片的 eFuse 一样只有一份
 */
void sandbox_meson_sm_set_efuse_4_16(const u8 efuse[SANDBOX_MESON_SM_EFUSE_4_16_LEN]);

/**
 * sandbox_meson_sm_unknown_calls() - 模型不实现的调用（写、删、eFuse 等一切改状态或模型之外的索引）被调用的次数；测试断言客户端从不调用它们
 */
uint sandbox_meson_sm_unknown_calls(struct udevice *dev);

/**
 * sandbox_meson_sm_set_storage() - 关掉存储服务时存储类调用一律返回 SMC_UNK，模拟没有这些服务的 BL31（如上游 TF-A）
 */
void sandbox_meson_sm_set_storage(struct udevice *dev, bool present);

#endif
