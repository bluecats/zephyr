/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <device.h>
#include <counter.h>
#include <misc/printk.h>

#define ALARM 2000000

struct counter_alarm_cfg alarm_cfg;

static void test_counter_interrupt_fn(struct device *counter_dev,
				const struct counter_alarm_cfg *cfg,
				u32_t counter)
{
	u32_t ticks = counter_read(counter_dev);
	u64_t now_usec = counter_ticks_to_us(counter_dev, ticks);

	printk("!!! Alarm !!!\n");
	printk("Time: %d\n", (int)(now_usec / USEC_PER_SEC));

	alarm_cfg.ticks = 2 * alarm_cfg.ticks;

	counter_set_ch_alarm(counter_dev, &alarm_cfg);
}

void main(void)
{
	struct device *counter_dev;
	int err = 0;

	printk("Test counter driver\n");
	counter_dev = device_get_binding(DT_RTC_0_NAME);

	counter_start(counter_dev);

	alarm_cfg.channel_id = 0;
	alarm_cfg.absolute = false;
	alarm_cfg.ticks = counter_us_to_ticks(counter_dev, ALARM);
	alarm_cfg.handler = test_counter_interrupt_fn;

	err = counter_set_ch_alarm(counter_dev, &alarm_cfg);

	if (err) {
		printk("Alarm set error\n");
	}

	while (1) {
	}
}
