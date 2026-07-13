// SPDX-License-Identifier: BSD-3-Clause

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/retention/bootmode.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

#include "acc_serial_console_commands.h"
#include "acc_tank_config_ble.h"

struct console_trigger
{
	const char *str;
	uint8_t     len; /* excludes the terminating NUL */
	uint8_t     pos;
};

/* Typed on the console UART, each followed by Enter, to trigger the matching action. */
static struct console_trigger dfu_trigger      = {.str = "dfu\n", .len = 4U};
static struct console_trigger resetcfg_trigger = {.str = "resetcfg\n", .len = 9U};

static bool trigger_feed(struct console_trigger *trigger, uint8_t byte)
{
	if (byte == trigger->str[trigger->pos])
	{
		trigger->pos++;

		if (trigger->pos == trigger->len)
		{
			trigger->pos = 0U;
			return true;
		}
	}
	else
	{
		trigger->pos = (byte == trigger->str[0]) ? 1U : 0U;
	}

	return false;
}

static void uart_rx_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev))
	{
		return;
	}

	uint8_t byte;

	while (uart_fifo_read(dev, &byte, 1) == 1)
	{
		if (trigger_feed(&dfu_trigger, byte))
		{
			/* Same flag the physical DFU button sets; MCUboot enters serial
			 * recovery on the next boot instead of loading the app.
			 */
			bootmode_set(BOOT_MODE_TYPE_BOOTLOADER);
			sys_reboot(SYS_REBOOT_COLD);
		}

		if (trigger_feed(&resetcfg_trigger, byte))
		{
			acc_tank_config_ble_reset_to_defaults();
			sys_reboot(SYS_REBOOT_COLD);
		}
	}
}

void acc_serial_console_commands_init(void)
{
	const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	uart_irq_callback_user_data_set(uart, uart_rx_isr, NULL);
	uart_irq_rx_enable(uart);
}
