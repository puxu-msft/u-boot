/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EcoNet/Airoha EN75xx vendor BMT/BBT translation layer (read-only).
 */
#ifndef __EN75_BMT_H
#define __EN75_BMT_H

struct mtd_info;

#if CONFIG_IS_ENABLED(MTD_EN75_BMT)
/*
 * Install the translation layer on a NAND whose DT node carries "econet,bmt".
 * Must be called before add_mtd_device(), because it shrinks mtd->size to the
 * user area and partitions are validated against that.
 * Returns 0 when attached *or* when the node opts out; negative on error.
 */
int en75_bmt_attach(struct mtd_info *mtd);
void en75_bmt_detach(struct mtd_info *mtd);
#else
static inline int en75_bmt_attach(struct mtd_info *mtd) { return 0; }
static inline void en75_bmt_detach(struct mtd_info *mtd) { }
#endif

#endif /* __EN75_BMT_H */
