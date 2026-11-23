// SPDX-License-Identifier: GPL-2.0+
/*
 * fastboot 的重启标记（补丁 0508）：Amlogic 的原因映射（asm/arch-meson/reboot.h，板上由 mach-meson 的 fastboot_set_reboot_flag() 使用），与会话没有复位就结束时撤销标记的钩子
 */

#include <command.h>
#include <fastboot.h>
#include <test/cmd.h>
#include <test/ut.h>
#include "../../arch/arm/include/asm/arch-meson/reboot.h"

static int fb_cleared;

/* sandbox 没有板级覆盖：这里只计数，核对 fastboot 命令结束时调了它 */
void fastboot_clear_reboot_flag(void)
{
	fb_cleared++;
}

/* reboot-bootloader、reboot-fastboot 记 fastboot（4）；reboot-recovery 在厂商那边是 factory_reset（2），不支持 */
static int cmd_test_fastboot_reboot_reason(struct unit_test_state *uts)
{
	ut_asserteq(REBOOT_REASON_FASTBOOT,
		    meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASON_BOOTLOADER));
	ut_asserteq(REBOOT_REASON_FASTBOOT,
		    meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASON_FASTBOOTD));
	ut_asserteq(-EINVAL, meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASON_RECOVERY));
	ut_asserteq(-EINVAL, meson_fastboot_reboot_reason(FASTBOOT_REBOOT_REASONS_COUNT));

	return 0;
}
CMD_TEST(cmd_test_fastboot_reboot_reason, 0);

/* 会话没起来（没有这个 USB 控制器）也算结束：标记作废 */
static int cmd_test_fastboot_reboot_clear(struct unit_test_state *uts)
{
	fb_cleared = 0;
	ut_asserteq(1, run_command("fastboot usb 99", 0));
	ut_asserteq(1, fb_cleared);

	return 0;
}
CMD_TEST(cmd_test_fastboot_reboot_clear, 0);
