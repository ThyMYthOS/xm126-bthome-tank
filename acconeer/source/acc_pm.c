// Copyright (c) Acconeer AB, 2023
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.


#include <stddef.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>


int acc_pm_suspend_uart(const struct device *uart)
{
	if (!device_is_ready(uart))
	{
		printk("%s: device not ready.\n", uart->name);
		return -EBUSY;
	}

	pm_device_action_run(uart, PM_DEVICE_ACTION_SUSPEND);

	return 0;
}
