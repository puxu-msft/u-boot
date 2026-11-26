// SPDX-License-Identifier: GPL-2.0
/*
 * 佰维 BIWIN BWET08U SPI NAND（厂商 0xBC、器件 0xB2）：256 MB SLC，每页 2048 字节、每块 64 页、2048 块，片上 ECC；中国移动 GS2107（EcoNet EN7528）板载。
 *
 * 读 ID：手册图 5-1 是 9FH 后跟 1 个地址字节，本板的干净读法是 OPCODE_ADDR，读出 BC B2 BC B2 BC。只发 9FH 不发地址就读，读出 00 BC B2 BC B2：头一个 00 落在地址周期（图 5-1 里这 8 个时钟 SO 是高阻），不是 ID。
 *
 * 纠错能力与备用区布局取 BWET08U 手册（1.1 节“每 512 字节 + 32 字节 8 位 ECC”、表 1-2 ECC 保护与备用区），与原厂芯片表的可用区 ooblayout_type10 一致；状态位 ECCS 的取值按同家族 Etron EM73 手册表 13-1。与 Linux 驱动相同，依据见 devices/openwrt/gs2107/docs/35-VENDOR-LAYOUT-AUDIT.md N31。
 */

#include <linux/bitops.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_BWET 0xbc

#define BWET_FR_BLOCK_LOCK 0xa0
#define BWET_FR_BLOCK_LOCK_UNLOCK_MASK 0x38
#define BWET_FR_ECC 0xb0
#define BWET_FR_ECC_ON_DIE BIT(4)

/* 状态寄存器 C0H 的 ECCS1:ECCS0（bit5:4）：00 无错；01 已纠正、位数低于纠错上限；10 不可纠；11 已纠正、位数等于纠错上限。同家族 Etron 手册没有列出读纠正位数的寄存器，只能按档报。原厂驱动只判 10（(status & 0x30) == 0x20）。解码与 Linux 驱动相同。 */
#define BWET_STATUS_ECC_MAX_BITFLIPS (3 << 4)

/* 读缓存有双线 0x3B（地址单线、1 个哑元字节、数据双线，即原厂芯片表的 read_mode = DUAL、dummy_mode = APPEND）与单线两种；spinand 核心逐个变体核控制器是否支持、算整页传输时长，取最快的（spinand_select_op_variant），设备树没给 rx 双线或控制器不支持时落到单线。 */
static SPINAND_OP_VARIANTS(read_cache_variants,
		SPINAND_PAGE_READ_FROM_CACHE_1S_1S_2S_OP(0, 1, NULL, 0, 0),
		SPINAND_PAGE_READ_FROM_CACHE_1S_1S_1S_OP(0, 1, NULL, 0, 0),
		SPINAND_PAGE_READ_FROM_CACHE_FAST_1S_1S_1S_OP(0, 1, NULL, 0, 0));

static SPINAND_OP_VARIANTS(write_cache_variants,
		SPINAND_PROG_LOAD_1S_1S_1S_OP(true, 0, NULL, 0));

static SPINAND_OP_VARIANTS(update_cache_variants,
		SPINAND_PROG_LOAD_1S_1S_1S_OP(false, 0, NULL, 0));

static int bwet08u_cfg(struct spinand_device *spinand)
{
	u8 val;
	int ret;

	ret = spinand_read_reg_op(spinand, BWET_FR_ECC, &val);
	if (ret)
		return ret;
	ret = spinand_write_reg_op(spinand, BWET_FR_ECC, val | BWET_FR_ECC_ON_DIE);
	if (ret)
		return ret;

	ret = spinand_read_reg_op(spinand, BWET_FR_BLOCK_LOCK, &val);
	if (ret)
		return ret;
	ret = spinand_write_reg_op(spinand, BWET_FR_BLOCK_LOCK, val & ~BWET_FR_BLOCK_LOCK_UNLOCK_MASK);
	return ret;
}

/* 备用区每 512 字节扇区 32 字节：前 18 字节 metadata（受 ECC 保护）、后 14 字节 ECC 校验（手册表 1-2）。MTD 只用前 64 字节（原厂 3.18 nand_scan_ident 也按页大小 / 32 取 64），所以只描述前两段；可用区从第 4 字节起，0–1 是坏块标记，2–3 是 EcoNet BMT 在备用块上记的原块号（原厂 LINUX_USE_OOB_START_OFFSET = 4）。 */
static int bwet_ooblayout_ecc(struct mtd_info *mtd, int section,
			      struct mtd_oob_region *region)
{
	if (section > 1)
		return -ERANGE;
	region->offset = 32 * section + 18;
	region->length = 14;
	return 0;
}

static int bwet_ooblayout_free(struct mtd_info *mtd, int section,
			       struct mtd_oob_region *region)
{
	switch (section) {
	case 0:
		region->offset = 4;
		region->length = 14;
		return 0;
	case 1:
		region->offset = 32;
		region->length = 18;
		return 0;
	}
	return -ERANGE;
}

static const struct mtd_ooblayout_ops bwet_ooblayout = {
	.ecc = bwet_ooblayout_ecc,
	.rfree = bwet_ooblayout_free,
};

static int bwet08u_ecc_get_status(struct spinand_device *spinand, u8 status)
{
	struct nand_device *nand = spinand_to_nand(spinand);

	switch (status & STATUS_ECC_MASK) {
	case STATUS_ECC_NO_BITFLIPS:
		return 0;

	case STATUS_ECC_UNCOR_ERROR:
		return -EBADMSG;

	case STATUS_ECC_HAS_BITFLIPS:
		/* 位数未知、低于上限：报上限减一（与 Linux 的 etron.c、alliancememory.c 同口径），达到 bitflip_threshold，mtd_read() 返回 -EUCLEAN；mtd read（local 0021）与 env（0023）把它当读成功，本 U-Boot 的 EcoNet BMT 只读、不因此重映射 */
		return nanddev_get_ecc_conf(nand)->strength - 1;

	case BWET_STATUS_ECC_MAX_BITFLIPS:
		return nanddev_get_ecc_conf(nand)->strength;
	}

	return -EINVAL;
}

static const struct spinand_info bwet_spinand_table[] = {
	SPINAND_INFO("BWET08U",
		     SPINAND_ID(SPINAND_READID_METHOD_OPCODE_ADDR, 0xb2),
		     NAND_MEMORG(1, 2048, 64, 64, 2048, 20, 1, 1, 1),
		     NAND_ECCREQ(8, 512),
		     SPINAND_INFO_OP_VARIANTS(&read_cache_variants,
					      &write_cache_variants,
					      &update_cache_variants),
		     0,
		     SPINAND_CONFIGURE_CHIP(bwet08u_cfg),
		     SPINAND_ECCINFO(&bwet_ooblayout, bwet08u_ecc_get_status)),
};

static const struct spinand_manufacturer_ops bwet_spinand_manuf_ops = {
};

const struct spinand_manufacturer bwet_spinand_manufacturer = {
	.id = SPINAND_MFR_BWET,
	.name = "BWET",
	.chips = bwet_spinand_table,
	.nchips = ARRAY_SIZE(bwet_spinand_table),
	.ops = &bwet_spinand_manuf_ops,
};
