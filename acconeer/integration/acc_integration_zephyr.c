// Copyright (c) Acconeer AB, 2022-2025
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include "acc_integration.h"

static unsigned int lock_key;

K_SEM_DEFINE(timer_expired, 0, 1);

static struct k_timer periodic_timer;

static void timer_expired_handler(struct k_timer *timer)
{
	k_sem_give(&timer_expired);
}

void acc_integration_set_periodic_wakeup(uint32_t time_msec)
{
	k_timer_stop(&periodic_timer);
	k_timer_init(&periodic_timer, timer_expired_handler, NULL);
	k_timer_start(&periodic_timer, K_MSEC(time_msec), K_MSEC(time_msec));
}

void acc_integration_sleep_until_periodic_wakeup(void)
{
	k_sem_take(&timer_expired, K_FOREVER);
}

void acc_integration_sleep_us(uint32_t time_usec)
{
	k_usleep(time_usec);
}

void acc_integration_sleep_ms(uint32_t time_msec)
{
	k_msleep(time_msec);
}

uint32_t acc_integration_get_time(void)
{
	return k_uptime_get_32();
}

void *acc_integration_mem_alloc(size_t size)
{
	return k_malloc(size);
}

void *acc_integration_mem_calloc(size_t nmemb, size_t size)
{
	return k_calloc(nmemb, size);
}

void acc_integration_mem_free(void *ptr)
{
	k_free(ptr);
}

void acc_integration_critical_section_enter(void)
{
	lock_key = irq_lock();
}

void acc_integration_critical_section_exit(void)
{
	irq_unlock(lock_key);
}

