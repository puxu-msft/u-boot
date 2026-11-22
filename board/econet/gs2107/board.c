/* SPDX-License-Identifier: GPL-2.0+ */

#include <dm.h>
#include <init.h>
#include <linux/errno.h>
#include <stdio.h>

#if CONFIG_IS_ENABLED(AIROHA_ETH)
extern U_BOOT_DRIVER(airoha_eth);
#endif

int board_init(void)
{
	return 0;
}

int board_late_init(void)
{
#if CONFIG_IS_ENABLED(AIROHA_ETH)
	struct udevice *dev;
	int ret;

	/* Probe the EN751221-family Ethernet parent after the console is up and just before initr_net() enumerates UCLASS_ETH devices. */
	ret = uclass_get_device_by_driver(UCLASS_MISC, DM_DRIVER_GET(airoha_eth), &dev);
	if (ret && ret != -ENODEV)
		printf("GS2107 Ethernet probe failed: %d\n", ret);
#endif
	return 0;
}
