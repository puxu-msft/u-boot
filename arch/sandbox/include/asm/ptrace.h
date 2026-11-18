/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2011 The Chromium OS Authors.
 */

#ifndef __ASM_SANDBOX_PTRACE_H
#define __ASM_SANDBOX_PTRACE_H

#ifndef __ASSEMBLY__
/*
 * sandbox 没有处理器异常；这里只承载 UCLASS_SM 的调用参数（sm_call() 的 args，下标与 arm64 的 x0..x30 同名），供 sandbox 的安全监控模型取 x1 起的参数
 */
struct pt_regs {
	unsigned long regs[31];
};

#ifdef __KERNEL__
extern void show_regs(struct pt_regs *);

#endif

#endif /* __ASSEMBLY__ */

#endif
