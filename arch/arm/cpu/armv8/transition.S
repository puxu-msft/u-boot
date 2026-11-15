/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2013
 * David Feng <fenghua@phytium.com.cn>
 */

#include <asm-offsets.h>
#include <config.h>
#include <linux/linkage.h>
#include <asm/macro.h>

.pushsection .text.armv8_switch_to_el2_prep, "ax"
WEAK(armv8_switch_to_el2_prep)
	ret
ENDPROC(armv8_switch_to_el2_prep)
.popsection

.pushsection .text.armv8_switch_to_el2, "ax"
ENTRY(armv8_switch_to_el2)
	bl	armv8_switch_to_el2_prep
	nop
	switch_el x6, 1f, 0f, 0f
0:
	cmp x5, #ES_TO_AARCH64
	b.eq 2f
	/*
	 * 32 位内核：armv8_el2_to_aarch32 切到 AArch32 后进入内核，不返回
	 */
	bl armv8_el2_to_aarch32
2:
	/*
	 * x4 is kernel entry point or switch_to_el1
	 * if CONFIG_ARMV8_SWITCH_TO_EL1 is defined.
         * When running in EL2 now, jump to the
	 * address saved in x4.
	 */
	br x4
1:	armv8_switch_to_el2_m x4, x5, x6
ENDPROC(armv8_switch_to_el2)
.popsection

.pushsection .text.armv8_switch_to_el1, "ax"
ENTRY(armv8_switch_to_el1)
	switch_el x6, 0f, 1f, 0f
0:
	/* x4 is kernel entry point. When running in EL1
	 * now, jump to the address saved in x4.
	 */
	br x4
1:	armv8_switch_to_el1_m x4, x5, x6, x7
ENDPROC(armv8_switch_to_el1)
.popsection

.pushsection .text.armv8_el2_to_aarch32, "ax"
/*
 * 在 EL2 把 32 位内核交给 EL1 AArch32 SVC 模式（IRQ/FIQ/SError 屏蔽，EL1 的 MMU 与缓存关），不返回：EL2 自己就能把当前核的下一级设成 AArch32（HCR_EL2.RW=0），不必求助安全固件。这只管当前核：副核由 PSCI CPU_ON 经 EL3 拉起，进什么执行状态由 EL3 固件决定。标准 TF-A 按 SCR_EL3.RW（psci_get_ns_ep_info()），EL2 是 AArch64 时副核进 AArch64 EL2；Amlogic BL31 改为锁存首次 CPU_ON/CPU_SUSPEND/SYSTEM_SUSPEND 时调用核的 HCR_EL2.RW，为 0 时副核以 AArch32 SVC 起来。所以只有后一类固件上本函数才足以引导 SMP 的 32 位内核，否则副核起不来。寄存器约定同 armv8_switch_to_el2() 的 ES_TO_AARCH32 调用：x0..x3 原样成为内核的 r0..r3（0、machid、FDT 地址），x4 是入口，x5 = ES_TO_AARCH32；破坏 x6、x7。HCR_EL2 另置 HCD（EL1 的 HVC 成为未定义指令；Amlogic 厂商 BL33 写 0，PSCI 走 SMC 时没有差别）。不在 EL2 时直接返回（EL1 改不了自己的执行状态）。安全固件提供切换服务的平台（NXP sec_firmware、MediaTek、i.MX8M）用强符号覆盖本函数，经 SMC 切到 AArch32。
 */
WEAK(armv8_el2_to_aarch32)
	/* 标号避开 1：armv8_switch_to_el1_m 内部用 1 标 AArch32 分支 */
	switch_el x6, 3f, 2f, 3f
2:	armv8_switch_to_el1_m x4, x5, x6, x7
3:	ret
ENDPROC(armv8_el2_to_aarch32)
.popsection
