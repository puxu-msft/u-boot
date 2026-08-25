// SPDX-License-Identifier: GPL-2.0
/*
 * EcoNet/Airoha EN75xx vendor BMT/BBT translation layer (READ-ONLY).
 *
 * The vendor bootbase and the Linux en75_bmt driver do not "skip" bad NAND
 * blocks the way stock MTD does - they *remap* them into a reserve pool kept at
 * the top of the chip, so that a logical block keeps a stable address after a
 * block goes bad. A bootloader that ignores this reads the wrong data on any
 * device that has ever remapped a block.
 *
 * This is a deliberately read-only port of that scheme:
 *
 *   - the factory Bad Block Table ("RAWB") and the Block Mapping Table ("BMT")
 *     are located, checksum-verified and parsed,
 *   - every MTD access is translated logical -> physical exactly as the vendor
 *     does (BBT shift-up, then BMT lookup),
 *   - mtd->size is reduced to the user area so partitions land where Linux and
 *     the bootbase expect them,
 *   - *nothing is ever written to the tables*. If a block turns bad we report
 *     the error upward instead of allocating a replacement; leave repair to
 *     Linux, which implements the full scheme.
 *
 * Deliberately NOT ported: BBT reconstruction / factory-BBT writing. Rewriting
 * the factory BBT shifts the physical address of every block above the first
 * bad one, which in a bootloader means bricking the board. Omission is the
 * safest form of "disabled".
 *
 * Derived from the Linux drivers by Caleb James DeLisle:
 *   target/linux/econet/files/drivers/mtd/nand/en75_bmt.c   (back end)
 *   target/linux/generic/files/drivers/mtd/nand/mtk_bmt.c   (MTD glue)
 */

#include <malloc.h>
#include <asm/cache.h>
#include <dm/ofnode.h>
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mtd/mtd.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/types.h>

#include "en75_bmt.h"

#define LOG_PFX				"en75_bmt"

/* Vendor firmware calls this POOL_GOOD_BLOCK_PERCENT. Must agree with the
 * bootbase, which prints the resulting figure as "bmt pool size: N".
 */
#define REQUIRED_GOOD_BLOCKS(total)	((total) * 8 / 100)

#define MAX_BMT_SIZE			256
#define MAX_BBT_SIZE			1000
#define DEFAULT_BBT_TABLE_SIZE		1000
#define INITIAL_READ_TRIES		3

/* Marked bad by the Linux driver (the bootbase uses 0x00 for everything). */
#define BLOCK_WORN_MARK			0x55

/*
 * On-disk structures. All multi-byte fields are host-order little-endian.
 */

struct bmt_table_header {
	char signature[3];	/* "BMT" */
	u8 version;		/* 1 */
	u8 bad_count;		/* unused */
	u8 size;		/* number of mappings */
	u8 checksum;		/* bmt_checksum() */
	u8 reserved[13];
};

static_assert(sizeof(struct bmt_table_header) == 20);

struct bmt_entry {
	u16 from;		/* physical (post-BBT-shift) block being replaced */
	u16 to;			/* reserve block standing in for it */
};

struct bmt_table {
	struct bmt_table_header header;
	struct bmt_entry table[MAX_BMT_SIZE];
};

struct bbt_table_header {
	char signature[4];	/* "RAWB" */
	u32 checksum;		/* bbt_checksum(), only 16 bits used */
	u8 version;		/* 1 */
	u8 size;		/* number of bad blocks */
	u8 reserved[2];		/* 0xffff */
};

static_assert(sizeof(struct bbt_table_header) == 12);

struct bbt_table {
	struct bbt_table_header header;
	u16 table[MAX_BBT_SIZE];	/* ascending order required */
};

static_assert(sizeof(struct bbt_table) == 2012);

/*
 * Runtime state. Single instance: there is one NAND chip on these boards, and
 * the Linux driver makes the same assumption.
 */
struct en75_bmt {
	struct mtd_info *mtd;

	/* saved raw ops - internal I/O must bypass our own translation */
	int (*_read_oob)(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops);
	int (*_write_oob)(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops);
	int (*_erase)(struct mtd_info *mtd, struct erase_info *instr);
	int (*_block_isbad)(struct mtd_info *mtd, loff_t ofs);
	int (*_block_markbad)(struct mtd_info *mtd, loff_t ofs);

	struct bbt_table bbt;
	struct bmt_table bmt;

	u16 bbt_table_size;
	u16 total_blks;
	u16 reserve_area_begin;
	u32 blk_size;
	u32 pg_size;
	u16 blk_shift;
	u16 pg_shift;

	u8 *data_buf;
	bool attached;
};

static struct en75_bmt bmtd;

static inline u32 blk_pg(u16 block)
{
	return (u32)(block << (bmtd.blk_shift - bmtd.pg_shift));
}

/* Read via the *saved* op: reserve-area offsets sit above the shrunken
 * mtd->size and would be rejected by the bounds-checked wrappers, and going
 * through our own translation here would recurse.
 */
static int bbt_nand_read(u32 page, u8 *dat, int dat_len, u8 *fdm, int fdm_len)
{
	struct mtd_oob_ops ops = {
		.mode = MTD_OPS_PLACE_OOB,
		.ooboffs = 0,
		.oobbuf = fdm,
		.ooblen = fdm_len,
		.datbuf = dat,
		.len = dat_len,
	};
	int ret;

	ret = bmtd._read_oob(bmtd.mtd, (loff_t)page << bmtd.pg_shift, &ops);
	if (ret < 0)
		return ret;
	return 0;
}

/*
 * In-memory helpers (no I/O)
 */

static u16 bbt_checksum(const struct bbt_table *bbt, u16 table_size)
{
	const u8 *data = (const u8 *)bbt->table;
	u16 checksum = bbt->header.version + bbt->header.size;
	int i;

	for (i = 0; i < table_size * (int)sizeof(bbt->table[0]); i++)
		checksum += data[i];

	return checksum;
}

static u8 bmt_checksum(const struct bmt_table *bmt, int check_entries)
{
	const u8 *data = (const u8 *)&bmt->table;
	u8 checksum = bmt->header.version + bmt->header.size;
	int length, i;

	if (check_entries > MAX_BMT_SIZE)
		check_entries = MAX_BMT_SIZE;
	length = check_entries * (int)sizeof(bmt->table[0]);
	for (i = 0; i < length; i++)
		checksum += data[i];

	return checksum;
}

static void sort_bbt(struct bbt_table *bbt)
{
	int i, j;

	/* Insertion sort: at most a few hundred u16s, and it avoids pulling in
	 * qsort() for something this small. Ascending order is required both by
	 * the shift algorithm below and by the vendor firmware.
	 */
	for (i = 1; i < bbt->header.size; i++) {
		u16 key = bbt->table[i];

		for (j = i - 1; j >= 0 && bbt->table[j] > key; j--)
			bbt->table[j + 1] = bbt->table[j];
		bbt->table[j + 1] = key;
	}
}

/*
 * Stage 1: each factory-bad block shifts everything above it up by one. This
 * is a shift, not a skip-list, which is why the BBT must be sorted and why
 * changing it after the fact moves every block above the change.
 */
static int get_mapping_block_bbt(int block)
{
	int size = bmtd.bbt.header.size;
	int i;

	for (i = 0; i < size; i++)
		if (bmtd.bbt.table[i] <= block)
			block++;

	if (block >= bmtd.reserve_area_begin || block < 0)
		return -EINVAL;

	return block;
}

/* Stage 2: BMT lookup, scanned backwards so a later duplicate 'from' wins. */
static int get_mapping_block(int block)
{
	int i;

	block = get_mapping_block_bbt(block);
	if (block < 0)
		return block;

	for (i = bmtd.bmt.header.size - 1; i >= 0; i--)
		if (bmtd.bmt.table[i].from == block)
			return bmtd.bmt.table[i].to;

	return block;
}

enum block_is_bad {
	BB_GOOD,
	BB_FACTORY_BAD,
	BB_WORN,
	BB_UNKNOWN_BAD,
};

static enum block_is_bad fdm_is_bad(const u8 fdm[4])
{
	if (fdm[0] == 0xff && fdm[1] == 0xff)
		return BB_GOOD;
	if (fdm[0] == BLOCK_WORN_MARK)
		return BB_WORN;
	if (fdm[0] == 0x00 || fdm[1] == 0x00)
		return BB_FACTORY_BAD;
	return BB_UNKNOWN_BAD;
}

static int try_parse_bbt(struct bbt_table *out, const u8 *buf, int len, u16 table_size)
{
	static struct bbt_table workspace;
	size_t bbt_size = sizeof(workspace.header) +
			  table_size * sizeof(workspace.table[0]);

	if (len < (int)bbt_size)
		return -EINVAL;

	memcpy(&workspace, buf, bbt_size);

	if (strncmp(workspace.header.signature, "RAWB", 4))
		return -EINVAL;

	if ((u16)workspace.header.checksum != bbt_checksum(&workspace, table_size))
		return -EINVAL;

	sort_bbt(&workspace);
	memcpy(out, &workspace, bbt_size);

	return 0;
}

static int try_parse_bmt(struct bmt_table *out, const u8 *buf, int len)
{
	static struct bmt_table workspace;

	if (len < (int)sizeof(*out))
		return -EINVAL;

	memcpy(&workspace, buf, sizeof(workspace));

	if (strncmp(workspace.header.signature, "BMT", 3))
		return -EINVAL;

	if (workspace.header.checksum !=
	    bmt_checksum(&workspace, workspace.header.size))
		return -EINVAL;

	memcpy(out, &workspace, sizeof(workspace));

	return 0;
}

/*
 * Walk down from the last block until REQUIRED_GOOD_BLOCKS good ones have been
 * seen; that boundary is where the reserve pool starts. The BBT and BMT are
 * found by signature+checksum along the way, not at fixed addresses.
 */
static int scan_reserve(void)
{
	u16 total_blks = bmtd.total_blks;
	int required = REQUIRED_GOOD_BLOCKS(total_blks);
	int good_blocks = 0;
	bool found_bbt = false, found_bmt = false;
	int cursor;

	for (cursor = total_blks - 1; cursor > 0; cursor--) {
		u8 fdm[4] = { 0xff, 0xff, 0xff, 0xff };
		int ret = -EIO;
		int i;

		for (i = 0; i < INITIAL_READ_TRIES; i++) {
			ret = bbt_nand_read(blk_pg(cursor), bmtd.data_buf,
					    bmtd.pg_size, fdm, sizeof(fdm));
			if (!ret)
				break;
		}

		if (ret || fdm_is_bad(fdm)) {
			pr_info("%s: skipping bad block %d in reserve area\n",
				LOG_PFX, cursor);
		} else {
			good_blocks++;
			if (!found_bbt &&
			    !try_parse_bbt(&bmtd.bbt, bmtd.data_buf,
					   bmtd.pg_size, bmtd.bbt_table_size)) {
				printf("%s: found BBT in block %d\n", LOG_PFX, cursor);
				found_bbt = true;
			} else if (!found_bmt &&
				   !try_parse_bmt(&bmtd.bmt, bmtd.data_buf,
						  bmtd.pg_size)) {
				printf("%s: found BMT in block %d\n", LOG_PFX, cursor);
				found_bmt = true;
			}
		}

		if (good_blocks >= required)
			break;
	}

	if (!cursor) {
		pr_err("%s: not enough valid blocks found, need %d got %d\n",
		       LOG_PFX, required, good_blocks);
		return -ENOSPC;
	}

	bmtd.reserve_area_begin = cursor;

	/*
	 * Refuse to run without a valid BBT. The alternative - reconstructing
	 * it - is exactly the operation that can shift every block on the chip,
	 * so a bootloader must not attempt it.
	 */
	if (!found_bbt) {
		pr_err("%s: no valid BBT found, refusing to translate\n", LOG_PFX);
		return -ENOENT;
	}
	if (!found_bmt)
		pr_info("%s: no BMT found, assuming no remapped blocks\n", LOG_PFX);

	return 0;
}

/*
 * MTD operation wrappers. Each translates then defers to the saved raw op.
 * Unlike the Linux driver these never allocate a replacement block: a failure
 * is reported to the caller instead.
 */

static int en75_bmt_read(struct mtd_info *mtd, loff_t from, struct mtd_oob_ops *ops)
{
	struct mtd_oob_ops cur_ops = *ops;
	int max_bitflips = 0;
	int ret = 0;

	ops->retlen = 0;
	ops->oobretlen = 0;

	while (ops->retlen < ops->len || ops->oobretlen < ops->ooblen) {
		u32 offset = from & (bmtd.blk_size - 1);
		u32 block = from >> bmtd.blk_shift;
		int cur_block, cur_ret;
		loff_t cur_from;

		cur_block = get_mapping_block(block);
		if (cur_block < 0)
			return -EIO;

		cur_from = ((loff_t)cur_block << bmtd.blk_shift) + offset;

		cur_ops.oobretlen = 0;
		cur_ops.retlen = 0;
		cur_ops.len = min_t(u32, mtd->erasesize - offset,
				    ops->len - ops->retlen);

		cur_ret = bmtd._read_oob(mtd, cur_from, &cur_ops);
		if (cur_ret < 0) {
			ret = cur_ret;
			if (!mtd_is_bitflip(cur_ret))
				goto out;
		} else {
			max_bitflips = max_t(int, max_bitflips, cur_ret);
		}

		ops->retlen += cur_ops.retlen;
		ops->oobretlen += cur_ops.oobretlen;

		cur_ops.ooboffs = 0;
		cur_ops.datbuf += cur_ops.retlen;
		cur_ops.oobbuf += cur_ops.oobretlen;
		cur_ops.ooblen -= cur_ops.oobretlen;

		if (!cur_ops.len)
			cur_ops.len = mtd->erasesize - offset;

		from += cur_ops.len;
	}

out:
	if (ret < 0 && !mtd_is_bitflip(ret))
		return ret;

	return max_bitflips;
}

static int en75_bmt_write(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	struct mtd_oob_ops cur_ops = *ops;
	int ret;

	ops->retlen = 0;
	ops->oobretlen = 0;

	while (ops->retlen < ops->len || ops->oobretlen < ops->ooblen) {
		u32 offset = to & (bmtd.blk_size - 1);
		u32 block = to >> bmtd.blk_shift;
		int cur_block;
		loff_t cur_to;

		cur_block = get_mapping_block(block);
		if (cur_block < 0)
			return -EIO;

		cur_to = ((loff_t)cur_block << bmtd.blk_shift) + offset;

		cur_ops.oobretlen = 0;
		cur_ops.retlen = 0;
		cur_ops.len = min_t(u32, bmtd.blk_size - offset,
				    ops->len - ops->retlen);

		ret = bmtd._write_oob(mtd, cur_to, &cur_ops);
		if (ret < 0) {
			/* No remap-on-failure here: the block stays bad and the
			 * caller finds out. Repairing it is Linux's job.
			 */
			pr_err("%s: write failed at block %u (physical %d); "
			       "not remapping (read-only BMT)\n",
			       LOG_PFX, block, cur_block);
			return ret;
		}

		ops->retlen += cur_ops.retlen;
		ops->oobretlen += cur_ops.oobretlen;

		cur_ops.ooboffs = 0;
		cur_ops.datbuf += cur_ops.retlen;
		cur_ops.oobbuf += cur_ops.oobretlen;
		cur_ops.ooblen -= cur_ops.oobretlen;

		if (!cur_ops.len)
			cur_ops.len = mtd->erasesize - offset;

		to += cur_ops.len;
	}

	return 0;
}

static int en75_bmt_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct erase_info mapped_instr = {
		.len = bmtd.blk_size,
	};
	u64 start_addr, end_addr;
	int ret = 0;

	start_addr = instr->addr & (~mtd->erasesize_mask);
	end_addr = instr->addr + instr->len;

	while (start_addr < end_addr) {
		u16 orig_block = start_addr >> bmtd.blk_shift;
		int block = get_mapping_block(orig_block);

		if (block < 0)
			return -EIO;

		mapped_instr.addr = (loff_t)block << bmtd.blk_shift;
		ret = bmtd._erase(mtd, &mapped_instr);
		if (ret) {
			pr_err("%s: erase failed at block %u (physical %d); "
			       "not remapping (read-only BMT)\n",
			       LOG_PFX, orig_block, block);
			instr->fail_addr = start_addr;
			break;
		}
		start_addr += mtd->erasesize;
	}

	return ret;
}

static int en75_bmt_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	u16 orig_block = ofs >> bmtd.blk_shift;
	int block = get_mapping_block(orig_block);

	if (block < 0)
		return -EIO;

	return bmtd._block_isbad(mtd, (loff_t)block << bmtd.blk_shift);
}

static int en75_bmt_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	u16 orig_block = ofs >> bmtd.blk_shift;
	int block = get_mapping_block(orig_block);

	if (block < 0)
		return -EIO;

	return bmtd._block_markbad(mtd, (loff_t)block << bmtd.blk_shift);
}

static void en75_bmt_replace_ops(struct mtd_info *mtd)
{
	bmtd._read_oob		= mtd->_read_oob;
	bmtd._write_oob		= mtd->_write_oob;
	bmtd._erase		= mtd->_erase;
	bmtd._block_isbad	= mtd->_block_isbad;
	bmtd._block_markbad	= mtd->_block_markbad;

	mtd->_read_oob		= en75_bmt_read;
	mtd->_write_oob		= en75_bmt_write;
	mtd->_erase		= en75_bmt_erase;
	mtd->_block_isbad	= en75_bmt_block_isbad;
	mtd->_block_markbad	= en75_bmt_block_markbad;
}

static void en75_bmt_restore_ops(struct mtd_info *mtd)
{
	mtd->_read_oob		= bmtd._read_oob;
	mtd->_write_oob		= bmtd._write_oob;
	mtd->_erase		= bmtd._erase;
	mtd->_block_isbad	= bmtd._block_isbad;
	mtd->_block_markbad	= bmtd._block_markbad;
}

int en75_bmt_attach(struct mtd_info *mtd)
{
	ofnode node = mtd_get_ofnode(mtd);
	u32 assert_reserve = 0;
	u32 val;
	int ret;

	if (!ofnode_valid(node) || !ofnode_read_bool(node, "econet,bmt"))
		return 0;

	if (bmtd.attached) {
		pr_err("%s: already attached to an MTD device\n", LOG_PFX);
		return -EBUSY;
	}

	memset(&bmtd, 0, sizeof(bmtd));
	bmtd.mtd = mtd;
	bmtd.blk_size = mtd->erasesize;
	bmtd.pg_size = mtd->writesize;
	bmtd.blk_shift = ffs(mtd->erasesize) - 1;
	bmtd.pg_shift = ffs(mtd->writesize) - 1;
	/* Must be taken before the size is shrunk below. */
	bmtd.total_blks = mtd->size >> bmtd.blk_shift;

	bmtd.bbt_table_size = DEFAULT_BBT_TABLE_SIZE;
	if (!ofnode_read_u32(node, "econet,bbt-table-size", &val)) {
		if (val == 0 || val > MAX_BBT_SIZE) {
			pr_err("%s: econet,bbt-table-size %u out of range\n",
			       LOG_PFX, val);
			return -EINVAL;
		}
		bmtd.bbt_table_size = val;
	}
	ofnode_read_u32(node, "econet,assert-reserve-size", &assert_reserve);

	bmtd.data_buf = memalign(ARCH_DMA_MINALIGN, bmtd.pg_size);
	if (!bmtd.data_buf)
		return -ENOMEM;

	en75_bmt_replace_ops(mtd);

	ret = scan_reserve();
	if (ret)
		goto err_restore;

	if (assert_reserve &&
	    assert_reserve != bmtd.total_blks - bmtd.reserve_area_begin) {
		/* Mismatch means our idea of the pool boundary differs from the
		 * bootbase's, so every translation would be wrong.
		 */
		pr_err("%s: reserve size %d != asserted %u, refusing\n", LOG_PFX,
		       bmtd.total_blks - bmtd.reserve_area_begin, assert_reserve);
		ret = -EINVAL;
		goto err_restore;
	}

	/* Hide the reserve area and the factory-bad shift from everything
	 * above, so DT partitions are validated against the same size Linux
	 * sees. Must happen before partitions are parsed.
	 */
	mtd->size = (loff_t)(bmtd.reserve_area_begin - bmtd.bbt.header.size)
		    << bmtd.blk_shift;

	bmtd.attached = true;

	printf("%s: blocks: total: %d, user: %d, factory_bad: %d, worn: %d reserve: %d\n",
	       LOG_PFX, bmtd.total_blks, bmtd.reserve_area_begin,
	       bmtd.bbt.header.size, bmtd.bmt.header.size,
	       bmtd.total_blks - bmtd.reserve_area_begin);
	printf("%s: %llu MiB usable space (read-only BMT: no remapping)\n",
	       LOG_PFX, (u64)mtd->size >> 20);

	return 0;

err_restore:
	en75_bmt_restore_ops(mtd);
	free(bmtd.data_buf);
	bmtd.data_buf = NULL;
	bmtd.mtd = NULL;
	return ret;
}

void en75_bmt_detach(struct mtd_info *mtd)
{
	if (!bmtd.attached || bmtd.mtd != mtd)
		return;

	en75_bmt_restore_ops(mtd);
	free(bmtd.data_buf);
	bmtd.data_buf = NULL;
	bmtd.mtd = NULL;
	bmtd.attached = false;
}
