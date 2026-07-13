// Copyright (c) Acconeer AB, 2023
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "acc_mcu_temperature.h"

static const struct device *temp_dev = DEVICE_DT_GET_ANY(nordic_nrf_temp);


bool acc_mcu_temperature_read_die_temp(float *temperature)
{
	struct sensor_value temp_value;
	int                 ret = -1;

	if (temp_dev != NULL && device_is_ready(temp_dev))
	{
		ret = sensor_sample_fetch(temp_dev);

		if (ret)
		{
			printk("sensor_sample_fetch failed return: %d\n", ret);
		}
	}

	if (ret == 0)
	{
		ret = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP,
		                         &temp_value);
		if (ret)
		{
			printk("sensor_channel_get failed return: %d\n", ret);
		}
	}

	if (ret == 0)
	{
		*temperature = sensor_value_to_float(&temp_value);
	}

	return ret == 0;
}
