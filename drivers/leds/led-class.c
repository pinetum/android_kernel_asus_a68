/*
 * LED Class Core
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005-2007 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include "leds.h"

#define LED_BUFF_SIZE 50

//ASUS_BSP +++ Maggie Lee "Backlight Porting"
#ifdef CONFIG_ASUS_BACKLIGHT
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#endif
#ifdef CONFIG_MICROP_NOTIFIER_CONTROLLER
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
#endif

extern void asus_set_bl_brightness(int);
extern int backlight_mode_state;
static unsigned long g_brightness = 102;
struct led_classdev *g_led_cdev;

enum device_mode {
    phone = 0,
    pad,
};

#include <linux/module.h>
/* Debug levels */
#define NO_DEBUG		0
#define DEBUG_POWER	1
#define DEBUG_INFO		2
#define DEBUG_VERBOSE	5
#define DEBUG_RAW		8
#define DEBUG_TRACE		10

static int debug = DEBUG_INFO;

module_param(debug, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");

#define led_debug(level, ...) \
		if (debug >= (level)) \
			pr_info(__VA_ARGS__);
#endif
//ASUS_BSP --- Maggie Lee "Backlight Porting"

static struct class *leds_class;

static void led_update_brightness(struct led_classdev *led_cdev)
{
	if (led_cdev->brightness_get)
		led_cdev->brightness = led_cdev->brightness_get(led_cdev);
}

static ssize_t led_brightness_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	/* no lock needed for this */
	led_update_brightness(led_cdev);

	return snprintf(buf, LED_BUFF_SIZE, "%u\n", led_cdev->brightness);
}

static ssize_t led_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	//ASUS_BSP +++ Maggie Lee "Backlight Porting"
	led_debug(NO_DEBUG, "[BL] %s +++: led_cdev = %s value = %d\n", __func__, led_cdev->name, (int)state);
	#ifdef CONFIG_ASUS_BACKLIGHT
	g_brightness = state;
	#endif
	//ASUS_BSP --- Maggie Lee "Backlight Porting"

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		if (state == LED_OFF)
			led_trigger_remove(led_cdev);
		//ASUS_BSP +++ Maggie Lee "Backlight Porting"
		#ifdef CONFIG_ASUS_BACKLIGHT
		if (!strcmp(led_cdev->name, "lcd-backlight")) {
			led_cdev->brightness = state;
			asus_set_bl_brightness(state);
		}
		else
			led_set_brightness(led_cdev, state);
		#else
		led_set_brightness(led_cdev, state);
		#endif
		//ASUS_BSP --- Maggie Lee "Backlight Porting"
	}

	return ret;
}

static ssize_t led_max_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long state = 0;

	ret = strict_strtoul(buf, 10, &state);
	if (!ret) {
		ret = size;
		if (state > LED_FULL)
			state = LED_FULL;
		led_cdev->max_brightness = state;
		led_set_brightness(led_cdev, led_cdev->brightness);
	}

	return ret;
}

static ssize_t led_max_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return snprintf(buf, LED_BUFF_SIZE, "%u\n", led_cdev->max_brightness);
}

static struct device_attribute led_class_attrs[] = {
	__ATTR(brightness, 0644, led_brightness_show, led_brightness_store),
	__ATTR(max_brightness, 0644, led_max_brightness_show,
			led_max_brightness_store),
#ifdef CONFIG_LEDS_TRIGGERS
	__ATTR(trigger, 0644, led_trigger_show, led_trigger_store),
#endif
	__ATTR_NULL,
};

static void led_timer_function(unsigned long data)
{
	struct led_classdev *led_cdev = (void *)data;
	unsigned long brightness;
	unsigned long delay;

	if (!led_cdev->blink_delay_on || !led_cdev->blink_delay_off) {
		led_set_brightness(led_cdev, LED_OFF);
		return;
	}

	brightness = led_get_brightness(led_cdev);
	if (!brightness) {
		/* Time to switch the LED on. */
		brightness = led_cdev->blink_brightness;
		delay = led_cdev->blink_delay_on;
	} else {
		/* Store the current brightness value to be able
		 * to restore it when the delay_off period is over.
		 */
		led_cdev->blink_brightness = brightness;
		brightness = LED_OFF;
		delay = led_cdev->blink_delay_off;
	}

	led_set_brightness(led_cdev, brightness);

	mod_timer(&led_cdev->blink_timer, jiffies + msecs_to_jiffies(delay));
}

/**
 * led_classdev_suspend - suspend an led_classdev.
 * @led_cdev: the led_classdev to suspend.
 */
void led_classdev_suspend(struct led_classdev *led_cdev)
{
	led_cdev->flags |= LED_SUSPENDED;
	led_cdev->brightness_set(led_cdev, 0);
}
EXPORT_SYMBOL_GPL(led_classdev_suspend);

/**
 * led_classdev_resume - resume an led_classdev.
 * @led_cdev: the led_classdev to resume.
 */
void led_classdev_resume(struct led_classdev *led_cdev)
{
	led_cdev->brightness_set(led_cdev, led_cdev->brightness);
	led_cdev->flags &= ~LED_SUSPENDED;
}
EXPORT_SYMBOL_GPL(led_classdev_resume);

static int led_suspend(struct device *dev, pm_message_t state)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (led_cdev->flags & LED_CORE_SUSPENDRESUME)
		led_classdev_suspend(led_cdev);

	return 0;
}

static int led_resume(struct device *dev)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (led_cdev->flags & LED_CORE_SUSPENDRESUME)
		led_classdev_resume(led_cdev);

	return 0;
}

/**
 * led_classdev_register - register a new object of led_classdev class.
 * @parent: The device to register.
 * @led_cdev: the led_classdev structure for this device.
 */
int led_classdev_register(struct device *parent, struct led_classdev *led_cdev)
{
	led_cdev->dev = device_create(leds_class, parent, 0, led_cdev,
				      "%s", led_cdev->name);
	if (IS_ERR(led_cdev->dev))
		return PTR_ERR(led_cdev->dev);

#ifdef CONFIG_LEDS_TRIGGERS
	init_rwsem(&led_cdev->trigger_lock);
#endif
	/* add to the list of leds */
	down_write(&leds_list_lock);
	list_add_tail(&led_cdev->node, &leds_list);
	up_write(&leds_list_lock);

	if (!led_cdev->max_brightness)
		led_cdev->max_brightness = LED_FULL;

	led_update_brightness(led_cdev);

	init_timer(&led_cdev->blink_timer);
	led_cdev->blink_timer.function = led_timer_function;
	led_cdev->blink_timer.data = (unsigned long)led_cdev;

#ifdef CONFIG_LEDS_TRIGGERS
	led_trigger_set_default(led_cdev);
#endif

	printk(KERN_DEBUG "Registered led device: %s\n",
			led_cdev->name);

	//ASUS_BSP +++ Maggie Lee "Backlight Porting"
	#ifdef CONFIG_ASUS_BACKLIGHT
	if(!strcmp(led_cdev->name, "lcd-backlight"))
		g_led_cdev = dev_get_drvdata(led_cdev->dev);
	#endif
	//ASUS_BSP --- Maggie Lee "Backlight Porting"

	return 0;
}
EXPORT_SYMBOL_GPL(led_classdev_register);

/**
 * led_classdev_unregister - unregisters a object of led_properties class.
 * @led_cdev: the led device to unregister
 *
 * Unregisters a previously registered via led_classdev_register object.
 */
void led_classdev_unregister(struct led_classdev *led_cdev)
{
#ifdef CONFIG_LEDS_TRIGGERS
	down_write(&led_cdev->trigger_lock);
	if (led_cdev->trigger)
		led_trigger_set(led_cdev, NULL);
	up_write(&led_cdev->trigger_lock);
#endif

	/* Stop blinking */
	led_brightness_set(led_cdev, LED_OFF);

	device_unregister(led_cdev->dev);

	down_write(&leds_list_lock);
	list_del(&led_cdev->node);
	up_write(&leds_list_lock);
}
EXPORT_SYMBOL_GPL(led_classdev_unregister);

//ASUS_BSP +++ Maggie Lee "Backlight Porting"
#if defined(CONFIG_ASUS_BACKLIGHT) && defined(CONFIG_EEPROM_NUVOTON)
static int change_backlight_mode(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk("%s ++, event=%d\r\n", __FUNCTION__, (int)event);
	switch (event) {
		case P01_ADD:
                	backlight_mode_state = pad;
			printk("[BL][mod] %s change to Pad\n",__func__);
			asus_set_bl_brightness(g_brightness);
                	break;
		case P01_REMOVE:
                	backlight_mode_state = phone;
			printk("[BL][mod] %s change to Phone\n",__func__);
			asus_set_bl_brightness(g_brightness);
                	
           	default:
                	break;
        }
	printk("%s --, event=%d\r\n", __FUNCTION__, (int)event);
	return NOTIFY_DONE;
}

static struct notifier_block my_hs_notifier = {
        .notifier_call = change_backlight_mode,
        .priority = VIBRATOR_MP_NOTIFY,
};
#endif
//ASUS_BSP --- Maggie Lee "Backlight Porting"

static int __init leds_init(void)
{
	leds_class = class_create(THIS_MODULE, "leds");
	if (IS_ERR(leds_class))
		return PTR_ERR(leds_class);
	leds_class->suspend = led_suspend;
	leds_class->resume = led_resume;
	leds_class->dev_attrs = led_class_attrs;

	//ASUS_BSP +++ Maggie Lee "Backlight Porting"
	#ifdef CONFIG_ASUS_BACKLIGHT
	#ifdef CONFIG_EEPROM_NUVOTON
	register_microp_notifier(&my_hs_notifier);
	#endif
	#ifdef CONFIG_MICROP_NOTIFIER_CONTROLLER
	notify_register_microp_notifier(&my_hs_notifier, "led_class"); //ASUS_BSP Lenter+
    	#endif
	#endif
	//ASUS_BSP --- Maggie Lee "Backlight Porting"

	return 0;
}

static void __exit leds_exit(void)
{
	class_destroy(leds_class);
}

subsys_initcall(leds_init);
module_exit(leds_exit);

MODULE_AUTHOR("John Lenz, Richard Purdie");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LED Class Interface");
