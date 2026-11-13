// SPDX-License-Identifier: GPL-2.0
/*
 * BWET08U SPI NAND support for the ChinaMobile GS2107 (EcoNet EN7528).
 *
 * The plain RDID method returns one residual leading byte on this board
 * (00 BC B2 BC B2); the stock kernel's clean path is OPCODE_ADDR, which
 * returns BC B2 BC B2 BC. The device is 256MB SLC with 2048-byte pages,
 * 64 pages per erase block, 2048 blocks and on-die ECC.
 */

#include <linux/bitops.h>
#include <linux/mtd/spinand.h>

#define SPINAND_MFR_BWET 0xbc

#define BWET_FR_BLOCK_LOCK 0xa0
#define BWET_FR_BLOCK_LOCK_UNLOCK_MASK 0x38
#define BWET_FR_ECC 0xb0
#define BWET_FR_ECC_ON_DIE BIT(4)

/* ECC status, status register bits 5:4; no register with the exact bitflip count is known. There is no datasheet, and only 10 = uncorrectable is backed by evidence: the stock EcoNet driver fails a read only on (status & 0x30) == 0x20. The meaning of 11 is unknown. The stock driver uses the data, but its table applies the same check to W25N01GV, MX35LF1GE4AB and F50L1G41LB, whose Linux and U-Boot drivers fail the read on 11, while GD5FxGQ4xA, PN26G0xA, W25N0xKV and the Linux HYF1GQ4UDACAE driver use 11 for their highest corrected class. Same decoding as the Linux driver. */
#define BWET_STATUS_ECC_11 (3 << 4)

static SPINAND_OP_VARIANTS(read_cache_variants,
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

static int bwet_ooblayout_ecc(struct mtd_info *mtd, int section,
			      struct mtd_oob_region *region)
{
	return -ERANGE;
}

static int bwet_ooblayout_free(struct mtd_info *mtd, int section,
			       struct mtd_oob_region *region)
{
	if (section > 0)
		return -ERANGE;
	region->offset = 0;
	region->length = 64;
	return 0;
}

static const struct mtd_ooblayout_ops bwet_ooblayout = {
	.ecc = bwet_ooblayout_ecc,
	.rfree = bwet_ooblayout_free,
};

static int bwet08u_ecc_get_status(struct spinand_device *spinand, u8 status)
{
	struct nand_device *nand = spinand_to_nand(spinand);
	static bool status_11_seen;

	switch (status & STATUS_ECC_MASK) {
	case STATUS_ECC_NO_BITFLIPS:
		return 0;

	case STATUS_ECC_UNCOR_ERROR:
		return -EBADMSG;

	case STATUS_ECC_HAS_BITFLIPS:
		/* Corrected, count unknown. Stay below mtd->bitflip_threshold (3/4 of the strength): mtd_read() turns reaching it into -EUCLEAN, and Linux into a UBI scrub or a permanent BMT remap. Both values are relative to the strength, so they stay on their side of the threshold while NAND_ECCREQ is a placeholder. */
		return nanddev_get_ecc_conf(nand)->strength / 2;

	case BWET_STATUS_ECC_11:
		/* Meaning unknown (see above). Like the stock driver, use the data; like the Linux HYF1GQ4UDACAE driver for its refresh code, report the full strength. Printed once per boot (the console is the serial port) so that real occurrences can be checked against the data. */
		if (!status_11_seen) {
			status_11_seen = true;
			printf("BWET08U: ECC status 11 (meaning unknown), data used as corrected\n");
		}
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
