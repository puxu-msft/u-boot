// SPDX-License-Identifier: GPL-2.0+
/*
 * optimus：进入 Amlogic USB 线刷态（协议见 drivers/optimus，USB 设备函数见 drivers/usb/gadget/f_optimus.c）
 *
 * optimus [-t <毫秒>] <USB 控制器>  进入线刷态，直到主机经命令请求离开（optimus leave）、Ctrl-C 或出错；-t 是探测窗口，窗口内主机没有设配置就返回失败（冷启动时“主机在就进线刷态，否则继续启动”）
 * optimus leave                       只在线刷态执行的命令里有效：请线刷态在这条命令的应答送出后退出；不在线刷态时返回失败
 * 离开之后做什么由调用方的脚本决定（m12.env 的救援循环按 m12_rescue_next 分派）。
 */

#include <command.h>
#include <errno.h>
#include <optimus.h>
#include <vsprintf.h>
#include <linux/string.h>

static int do_optimus(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	ulong probe = 0;
	char *end;
	int ctrl, ret;

	if (argc == 2 && !strcmp(argv[1], "leave")) {
		if (optimus_request_leave()) {
			puts("optimus: not in USB burning mode\n");
			return CMD_RET_FAILURE;
		}
		return CMD_RET_SUCCESS;
	}
	if (argc == 4 && !strcmp(argv[1], "-t")) {
		probe = simple_strtoul(argv[2], &end, 10);
		if (*end || !probe)
			return CMD_RET_USAGE;
		argc -= 2;
		argv += 2;
	}
	if (argc != 2)
		return CMD_RET_USAGE;
	ctrl = simple_strtoul(argv[1], &end, 10);
	if (*end)
		return CMD_RET_USAGE;
	ret = f_optimus_run(ctrl, probe);
	if (ret == -ETIMEDOUT) {
		printf("optimus: no USB host within %lu ms\n", probe);
		return CMD_RET_FAILURE;
	}
	if (ret) {
		printf("optimus: USB burning mode failed (%d)\n", ret);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_LONGHELP(optimus,
	"[-t <ms>] <usb_controller>\n"
	"    - enter Amlogic USB burning mode until a command asks to leave;\n"
	"      with -t, give up if no host configures the device within <ms>\n"
	"optimus leave\n"
	"    - (inside burning mode) leave once this command's reply is sent");

U_BOOT_CMD(optimus, 4, 0, do_optimus, "Amlogic USB burning (Optimus) mode",
	   optimus_help_text);
