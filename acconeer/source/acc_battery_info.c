// Copyright (c) Acconeer AB, 2023-2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <math.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "acc_battery_info.h"

LOG_MODULE_REGISTER(BATTERY_INFO, CONFIG_ADC_LOG_LEVEL);

#if !DT_NODE_EXISTS(DT_PATH(vbatt)) || !DT_NODE_HAS_PROP(DT_PATH(vbatt), io_channels)
#error "No suitable devicetree overlay for vbatt specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {DT_FOREACH_PROP_ELEM(DT_PATH(vbatt), io_channels, DT_SPEC_AND_COMMA)};

#define VBATT_LEN DT_PROP_LEN(DT_PATH(vbatt), io_channels)
#if VBATT_LEN > 1
#error "VBATT_LEN > 1"
#endif

// This is safe since we have already checked that there is exactly one
// vbatt node defined in DT
static const struct adc_dt_spec adc_channel = adc_channels[0];

#define VBATT DT_PATH(vbatt)

static const struct gpio_dt_spec vbat_power_gpio = GPIO_DT_SPEC_GET_OR(VBATT, power_gpios, {0});
static const uint32_t            vbat_output_ohm = DT_PROP_OR(VBATT, output_ohms, 0);
static const uint32_t            vbat_full_ohm   = DT_PROP_OR(VBATT, full_ohms, 0);

static int16_t raw_buffer;

static struct adc_sequence adc_seq = {
    .buffer      = &raw_buffer,
    .buffer_size = sizeof(raw_buffer),
};

static bool measure_enable(bool enable)
{
	int rc = -ENOENT;

	if (vbat_power_gpio.port)
	{
		rc = gpio_pin_set_dt(&vbat_power_gpio, enable);
	}

	return rc == 0;
}

bool acc_battery_info_init(void)
{
	int rc;

	if (!device_is_ready(adc_channel.dev))
	{
		LOG_ERR("ADC device is not ready %s", adc_channel.dev->name);
		return false;
	}

	if (vbat_power_gpio.port)
	{
		if (!device_is_ready(vbat_power_gpio.port))
		{
			LOG_ERR("%s: device not ready", vbat_power_gpio.port->name);
			return false;
		}

		rc = gpio_pin_configure_dt(&vbat_power_gpio, GPIO_OUTPUT_INACTIVE);
		if (rc != 0)
		{
			LOG_ERR("Failed to control feed %s.%u: %d", vbat_power_gpio.port->name, vbat_power_gpio.pin, rc);
			return false;
		}
	}

	rc = adc_channel_setup_dt(&adc_channel);

	return rc == 0;
}

bool acc_battery_info_sample_voltage(float *voltage, bool recalibrate)
{
	int rc = -ENOENT;

	rc = adc_sequence_init_dt(&adc_channel, &adc_seq);
	if (rc != 0)
	{
		LOG_ERR("Failed to init ADC sequence from dt, error %d", rc);
		return false;
	}

	if (measure_enable(true))
	{
		// Wait 300 microseconds after asserting VIN_ADC_EN before trying to sample
		// VIN_ADC to accommodate for delay in the power switch
		k_usleep(300);
		adc_seq.calibrate = recalibrate;
		rc                = adc_read(adc_channel.dev, &adc_seq);
		if (rc == 0)
		{
			int32_t adc_val_mv = raw_buffer;

			rc = adc_raw_to_millivolts_dt(&adc_channel, &adc_val_mv);
			if (rc == 0)
			{
				if (vbat_output_ohm != 0)
				{
					*voltage = (float)(adc_val_mv * (uint64_t)vbat_full_ohm / vbat_output_ohm);
				}
				else
				{
					*voltage = (float)adc_val_mv;
				}

				// Return battery voltage in Volts
				*voltage /= 1000.0f;
			}
			else
			{
				LOG_ERR("Failed to convert adc to mV, error %d", rc);
			}
		}

		(void)measure_enable(false);
	}

	return rc == 0;
}
