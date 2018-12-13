/*
 * Misc LED triggers for the PA18 project
 *
 * Copyright 2018 Exor Int. S.p.a.
 *
 * Author: G. Pavoni
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/leds.h>

DEFINE_LED_TRIGGER(ledtrig_usbstick_wr);
static unsigned long long_delay = 300;
static unsigned long short_delay = 300;

void ledtrig_usbstick_wr_activity(void)
{
	led_trigger_blink_oneshot(ledtrig_usbstick_wr, &long_delay, &short_delay, 0);
}
EXPORT_SYMBOL(ledtrig_usbstick_wr_activity);

static int __init ledtrig_pa18_init(void)
{
	led_trigger_register_simple("usb-disk-wr", &ledtrig_usbstick_wr);
	return 0;
}

static void __exit ledtrig_pa18_exit(void)
{
	led_trigger_unregister_simple(ledtrig_usbstick_wr);
}

module_init(ledtrig_pa18_init);
module_exit(ledtrig_pa18_exit);

MODULE_AUTHOR("Richard Purdie <rpurdie@openedhand.com>");
MODULE_DESCRIPTION("LED IDE Disk Activity Trigger");
MODULE_LICENSE("GPL");
