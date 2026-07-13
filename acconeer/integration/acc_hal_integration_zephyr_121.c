// Copyright (c) Acconeer AB, 2024-2025
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "acc_definitions_common.h"
#include "acc_hal_definitions_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/pm/device.h>

#define MODULE "hal_integration_zephyr"

/********************************************************************/
/* Acconeer A121 HAL Integration Driver for Zephyr OS               */
/*                                                                  */
/* This code implements a low-level driver for the Acconeer A121    */
/* radar sensor.  In addition to the functions implemented in this  */
/* file, an application using the Acconeer radar requires           */
/* libraries, include files, and other files provided in the        */
/* Acconeer SDK packages                                            */
/*                                                                  */
/* Note:                                                            */
/*  - This code has not been designed to be multithread safe        */
/*  - No support for sensors on multiple SPI busses                 */
/*                                                                  */
/********************************************************************/

//  Local Macros and Constants

#define ACC_PROP_NODE DT_PATH(acc_rss_hal)

#define ACC_PROP(x)     DT_PROP(ACC_PROP_NODE, x)
#define ACC_PROP_LEN(x) DT_PROP_LEN(ACC_PROP_NODE, x)

// Each sensor must have an enable pin gpio so by counting the number of enable gpios
// defined in the device tree we get the number of sensors.
#define SENSOR_COUNT ACC_PROP_LEN(acc_sensor_enable_gpios)

#define MAX_SPI_TRANSFER_SIZE ACC_PROP(acc_max_spi_transfer_size)

#if DT_NODE_HAS_PROP(ACC_PROP_NODE, acc_interrupt_polling_delay_us)
// Poll the interrupt pin at the specified delay interval
#define POLLING_DELAY_US ACC_PROP(acc_interrupt_polling_delay_us)
#else
// Use MCU interrupt if polling delay is not specified in the device tree
#define USE_INTERRUPT (1)
#endif

static const int sensor_stab_time_us = ACC_PROP(acc_sensor_stab_time_us);

#define SENSOR_SELECT_GPIOS_LEN ACC_PROP_LEN(acc_sensor_select_gpios)
#if SENSOR_SELECT_GPIOS_LEN > 0
static const struct gpio_dt_spec sensor_select_gpio_specs[] = {
    DT_FOREACH_PROP_ELEM_SEP(ACC_PROP_NODE, acc_sensor_select_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#endif

static const struct gpio_dt_spec sensor_interrupt_gpio_specs[] = {
    DT_FOREACH_PROP_ELEM_SEP(ACC_PROP_NODE, acc_sensor_interrupt_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

static const struct gpio_dt_spec sensor_enable_gpio_specs[] = {
    DT_FOREACH_PROP_ELEM_SEP(ACC_PROP_NODE, acc_sensor_enable_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

#if SENSOR_COUNT != ACC_PROP_LEN(acc_sensor_interrupt_gpios)
#error "Each sensor must have an enable gpio and an interrupt gpio. Check device tree."
#endif

#define POWER_ENABLE_GPIOS_LEN DT_PROP_LEN_OR(ACC_PROP_NODE, acc_power_enable_gpios, 0)
#if POWER_ENABLE_GPIOS_LEN == SENSOR_COUNT || POWER_ENABLE_GPIOS_LEN == 1
const struct gpio_dt_spec power_enable_gpios_specs[] = {
    DT_FOREACH_PROP_ELEM_SEP(ACC_PROP_NODE, acc_power_enable_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};
#endif

#if USE_INTERRUPT
K_SEM_DEFINE(sensor_interrupt_sem, 0, 1);
static void sensor_interrupt_callback_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);

static struct gpio_callback interrupt_gpio_callback_data[SENSOR_COUNT];
#endif

// SPI device configuration

static const struct device *spi_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(acc_a121_device)));

// For Zephyr 3.3.x compatibility, SPI_CS_CONTROL_INIT was introduced in Zephyr 3.4.0
#ifdef SPI_CS_CONTROL_PTR_DT
#define SPI_CS_CONTROL_INIT SPI_CS_CONTROL_PTR_DT
#endif

static const struct spi_config spi_cfg = {
    .frequency = ACC_PROP(acc_spi_freq),
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
#if DT_SPI_DEV_HAS_CS_GPIOS(DT_NODELABEL(acc_a121_device)) > 0
    .cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(acc_a121_device), 0),
#else
    .cs = {{0}},
#endif
};

#if defined(CONFIG_BT) && defined(CONFIG_NRF52_ANOMALY_198_WORKAROUND)
/*
 * Workaround for nRF52840 anomaly [198] SPIM: SPIM3 transmit data might be corrupted
 * Symptoms:
 * Data accessed by CPU location in the same RAM block as where the SPIM3 TXD.PTR is pointing,
 * and CPU does a read or write operation at the same clock cycle as the SPIM3 EasyDMA is fetching data.
 * Workaround:
 * Reserve dedicated RAM blocks for the SPIM3 transmit buffer, not overlapping with application data
 * used by the CPU. In addition, synchronize so that the CPU is not writing data to the transmit buffer
 * while SPIM is transmitting data.
 *
 * One RAM block in nRF52840 is 8k bytes.
 *
 * See more in the Errata document located at https://infocenter.nordicsemi.com/
 */
uint8_t tx_buffer[0x2000] __attribute__((section("spim3_tx_buffer"), aligned(0x2000))) = {0};
#endif

// Local Functions

static int acc_init_peripherals(void)
{
	int ret = 0;

#if SENSOR_SELECT_GPIOS_LEN > 0
	for (uint32_t i = 0; i < SENSOR_SELECT_GPIOS_LEN; i++)
	{
		gpio_pin_configure_dt(&sensor_select_gpio_specs[i], GPIO_OUTPUT_HIGH);
	}

#endif

	for (uint32_t i = 0; i < SENSOR_COUNT; i++)
	{
		gpio_pin_configure_dt(&sensor_enable_gpio_specs[i], GPIO_OUTPUT_INACTIVE);
		ret = gpio_pin_configure_dt(&sensor_interrupt_gpio_specs[i], GPIO_INPUT);
#if USE_INTERRUPT
		gpio_init_callback(&interrupt_gpio_callback_data[i], sensor_interrupt_callback_handler, BIT(sensor_interrupt_gpio_specs[i].pin));
#endif
#if POWER_ENABLE_GPIOS_LEN == SENSOR_COUNT
		ret = gpio_pin_configure_dt(&power_enable_gpios_specs[i], GPIO_OUTPUT_INACTIVE);
#endif
	}

#if POWER_ENABLE_GPIOS_LEN == 1 && SENSOR_COUNT > 1
	ret = gpio_pin_configure_dt(&power_enable_gpios_specs[0], GPIO_OUTPUT_INACTIVE);
#endif

	return ret;
}

static inline uint32_t sensor_index(acc_sensor_id_t sensor_id)
{
	if ((sensor_id < 1) || (sensor_id > SENSOR_COUNT))
	{
		ACC_LOG_ERROR("Sensor id out of range");
		return 0;
	}
	else
	{
		return (uint32_t)sensor_id - 1;
	}
}

// HAL Integration Functions

void acc_hal_integration_sensor_supply_on(acc_sensor_id_t sensor_id)
{
#if POWER_ENABLE_GPIOS_LEN != SENSOR_COUNT
	(void)sensor_id;

#if POWER_ENABLE_GPIOS_LEN == 1
	// Use common power control for all sensors if there is only one
	// power enable GPIO
	gpio_pin_set_dt(&power_enable_gpios_specs[0], 1);
	k_usleep(sensor_stab_time_us);
#endif

#else
	// Individual power control for each sensor
	gpio_pin_set_dt(&power_enable_gpios_specs[sensor_index(sensor_id)], 1);
	k_usleep(sensor_stab_time_us);
#endif
}

void acc_hal_integration_sensor_supply_off(acc_sensor_id_t sensor_id)
{
#if POWER_ENABLE_GPIOS_LEN != SENSOR_COUNT
	(void)sensor_id;
#else
	gpio_pin_set_dt(&power_enable_gpios_specs[sensor_index(sensor_id)], 0);
	k_usleep(sensor_stab_time_us);
#endif
}

void acc_hal_integration_sensor_enable(acc_sensor_id_t sensor_id)
{
#ifdef CONFIG_PM_DEVICE
	pm_device_action_run(spi_dev, PM_DEVICE_ACTION_RESUME);
#endif

	gpio_pin_set_dt(&sensor_enable_gpio_specs[sensor_index(sensor_id)], 1);
	k_usleep(sensor_stab_time_us);

#if USE_INTERRUPT
	for (uint32_t i = 0; i < SENSOR_COUNT; i++)
	{
		gpio_pin_interrupt_configure_dt(&sensor_interrupt_gpio_specs[i], GPIO_INT_EDGE_TO_ACTIVE);
	}

#endif
}

void acc_hal_integration_sensor_disable(acc_sensor_id_t sensor_id)
{
#if USE_INTERRUPT
	for (uint32_t i = 0; i < SENSOR_COUNT; i++)
	{
		gpio_pin_interrupt_configure_dt(&sensor_interrupt_gpio_specs[i], GPIO_INT_DISABLE);
	}

#endif

	gpio_pin_set_dt(&sensor_enable_gpio_specs[sensor_index(sensor_id)], 0);

#ifdef CONFIG_PM_DEVICE
	pm_device_action_run(spi_dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#if defined(CONFIG_SOC_NRF52840_QIAA)
	// Force SPIM3 to release the HFCLK (Anomaly 195). This is needed since NCS 3.2.0
	*((volatile uint32_t *)0x4002F004) = 1;
#endif

	k_usleep(sensor_stab_time_us);
}

static void acc_hal_integration_sensor_transfer(acc_sensor_id_t sensor_id, uint8_t *buffer, size_t buffer_length)
{
#if SENSOR_SELECT_GPIOS_LEN > 0
	// XE121 style sensor selection
	for (uint32_t i = 0; i < SENSOR_SELECT_GPIOS_LEN; i++)
	{
		gpio_pin_set_dt(&sensor_select_gpio_specs[i], ((sensor_id - 1) & (1U << i)) != 0);
	}

#endif

#if defined(CONFIG_BT) && defined(CONFIG_NRF52_ANOMALY_198_WORKAROUND)
	memcpy(tx_buffer, buffer, buffer_length);
	const struct spi_buf tx = {.buf = tx_buffer, .len = buffer_length};
#else
	const struct spi_buf tx = {.buf = buffer, .len = buffer_length};
#endif
	const struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};

	const struct spi_buf     rx      = {.buf = buffer, .len = buffer_length};
	const struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);

	if (ret < 0)
	{
		ACC_LOG_ERROR("SPI transceive error: %d", ret);
	}
}

static bool check_interrupt_pin(acc_sensor_id_t sensor_id)
{
	int pin_value = gpio_pin_get_dt(&sensor_interrupt_gpio_specs[sensor_index(sensor_id)]);

	return pin_value > 0;
}

#if USE_INTERRUPT

static void sensor_interrupt_callback_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	k_sem_give(&sensor_interrupt_sem);
}

#endif

bool acc_hal_integration_wait_for_sensor_interrupt(acc_sensor_id_t sensor_id, uint32_t timeout_ms)
{
#if USE_INTERRUPT
	// Check the interrupt pin first, so we can return as quickly as possible if the
	// pin already is high.
	if (check_interrupt_pin(sensor_id))
	{
		return true;
	}

	// Store the current time so we can calculate for how long we have waited.
	uint32_t wait_start = k_uptime_get_32();

	// Reset the semaphore as we are not interested in old interrupt signals. We will check
	// the interrupt pin again before going to sleep waiting for the semaphore to change state.
	k_sem_reset(&sensor_interrupt_sem);

	struct gpio_callback *gpio_callback_data = &interrupt_gpio_callback_data[sensor_index(sensor_id)];
	gpio_add_callback(sensor_interrupt_gpio_specs->port, gpio_callback_data);

	while (!check_interrupt_pin(sensor_id))
	{
		// The interrupt signal for this sensor is still low. Calculate the time
		// left until we reach timeout.
		int32_t max_sleep_time_ms = timeout_ms - (k_uptime_get_32() - wait_start);

		if (max_sleep_time_ms <= 0)
		{
			// The timeout period has expired, so let's give up and return to RSS.
			gpio_remove_callback(sensor_interrupt_gpio_specs->port, gpio_callback_data);
			return false;
		}

		// Wait for the ISR to give the semaphore to this thread.
		k_sem_take(&sensor_interrupt_sem, K_MSEC(max_sleep_time_ms));

		// TODO: Consider using an event or some other signalling mechanism instead of a
		// semaphore to ensure that all waiting threads are woken up if an interrupt pin
		// changes state to high. The current solution is not multithread safe as the thread
		// that gets the semaphore may not be the thread that is waiting for the
		// interrupt signal.
	}

	gpio_remove_callback(sensor_interrupt_gpio_specs->port, gpio_callback_data);

	// The interrupt signal is high as we managed to exit the while loop.
	return true;

#else // Use polling
	uint32_t start_time = k_uptime_get_32();
	while (!check_interrupt_pin(sensor_id) && (k_uptime_get_32() - start_time) < timeout_ms)
	{
		k_usleep(POLLING_DELAY_US);
	}
	return check_interrupt_pin(sensor_id);
#endif
}

const acc_hal_a121_t *acc_hal_rss_integration_get_implementation(void)
{
	static bool initialized = false;

	if (!initialized)
	{
		// Only initialize peripherals once
		acc_init_peripherals();
		initialized = true;
	}

	static acc_hal_a121_t hal = {
	    .max_spi_transfer_size = MAX_SPI_TRANSFER_SIZE,

	    .mem_alloc = k_malloc,
	    .mem_free  = k_free,

	    .transfer = acc_hal_integration_sensor_transfer,
	    .log      = acc_integration_log,

	    .optimization.transfer16 = NULL,
	};

	return &hal;
}

uint16_t acc_hal_integration_sensor_count(void)
{
	return SENSOR_COUNT;
}
