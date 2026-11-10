/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __CONFIG_GS2107_H
#define __CONFIG_GS2107_H

#define CFG_SYS_SDRAM_BASE 0x80000000
/* Above the FIT scratch area (0x82000000) and the kernel load address (0x80020000, up to ~32 MiB). */
#define CFG_SYS_INIT_SP_OFFSET 0x04000000

#endif
