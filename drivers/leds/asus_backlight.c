/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/pmic.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/time.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <mach/board.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/irqs.h>
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif

#define calc_div 1000000

//Debug Masks +++
#include <linux/module.h>

#define NO_DEBUG       0
#define DEBUG_POWER     1
#define DEBUG_INFO  2
#define DEBUG_VERBOSE 5
#define DEBUG_RAW      8
#define DEBUG_TRACE   10

static int debug = DEBUG_INFO;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");

#define backlight_debug(level, ...) \
		if (debug >= (level)) \
			pr_info(__VA_ARGS__);
//Debug Masks ---

DECLARE_COMPLETION(brightness_comp);

#ifdef CONFIG_EEPROM_NUVOTON
extern int AX_MicroP_setPWMValue(uint8_t);
static struct delayed_work turn_off_panel_work;
static struct delayed_work turn_on_panel_work;
#endif

extern void renesas_set_backlight(int);
extern int sharp_set_brightness(int);
#ifndef ASUS_A80_PROJECT
extern void sharp_set_cabc(int);
#endif

static struct asus_backlight_data *pdata;
static struct workqueue_struct *backlight_workqueue;

int backlight_mode_state = 0;
int g_backlight = 0;

extern int g_HAL_als_switch_on;

enum backlight_mode {
    phone = 0,
    pad,
};

enum brightness_mode {
	NORMAL = 0,
	AUTO,
	OUTDOOR,
};

static int mipi_set_backlight(int value)
{
	int index;
	//int div = ((pdata->max_value-pdata->min_value)*calc_div/(pdata->phone.max_index-pdata->phone.min_index));
	#ifndef ASUS_A80_PROJECT
	static bool CABC_On = true;
	#endif

	if (value == 0) {
        	backlight_debug(DEBUG_VERBOSE, "[BL] %s turn off Phone backlight\n",__func__);
		index = 0;
    	}
	else if (value >= pdata->max_value) {
        	index = pdata->phone.max_index;
	}
	else if (value > 0 && value <= pdata->min_value) {
		index = pdata->phone.min_index;
	}
	else if (value > pdata->min_value && value < pdata->max_value) {
		//index = value*calc_div/div+pdata->phone.shift;			//add 6 to match min_index
		index = (value*calc_div/pdata->max_value)*pdata->phone.max_index/calc_div;
		backlight_debug(DEBUG_VERBOSE, "value(%d) max_value(%d) max_index(%d) index(%d)\n", value, pdata->max_value, pdata->phone.max_index, index);
	}
	else {
		backlight_debug(DEBUG_VERBOSE,"[BL] value (%d) not within spec, set to default brightness\n", value);
		index = pdata->phone.default_index;//value not in spec, do set default value
	}
	if (index > 0 && index < pdata->phone.min_index)
		index = pdata->phone.min_index;

	backlight_debug(DEBUG_INFO,"[BL] %s: value(%d), index(%d) \n", __func__, value, index);
	#ifdef ASUS_A80_PROJECT
	renesas_set_backlight(index);
	#else
	if (index <= 28 && CABC_On == true)
        {
            sharp_set_cabc(0);
            CABC_On = false;
        }
        else if (index > 28 && CABC_On == false)
        {
            sharp_set_cabc(3);
            CABC_On = true;
        }
	sharp_set_brightness(index);
	#endif
	complete(&brightness_comp);

	return 0;
}

#ifdef CONFIG_EEPROM_NUVOTON
int pad_set_backlight(int value)
{
	int ret = 0;
	int index = 0;
	static int previous_value;
	//int div = ((pdata->max_value-pdata->min_value)*calc_div/(pdata->pad.max_index-pdata->pad.min_index));

	if(g_HAL_als_switch_on)
		value = value * ((pdata->phone.max*pdata->phone.max_index/pdata->phone.resolution*calc_div )/(pdata->pad.max*pdata->pad.max_index/pdata->pad.resolution)) / calc_div;			//compensation gain is only applied for autobrightness

	if (value == 0) {
        	backlight_debug(DEBUG_VERBOSE,"[BL] turn off pad backlight\n");
        	if (delayed_work_pending(&turn_off_panel_work))
			cancel_delayed_work_sync(&turn_off_panel_work);
        	AX_MicroP_setPWMValue(0);
        	queue_delayed_work(backlight_workqueue, &turn_off_panel_work, msecs_to_jiffies(3000));
		previous_value = 0;
		return 0;
	}

	if (value >= pdata->max_value)
        	index = pdata->pad.max_index;
	else if (value > 0 && value <= pdata->min_value) 
		index = pdata->pad.min_index;
	else if (value > pdata->min_value && value < pdata->max_value) 
		//index = value*calc_div/div+pdata->pad.shift;
		index = (value*calc_div/pdata->max_value)*pdata->pad.max_index/calc_div;
	else {
		backlight_debug(DEBUG_VERBOSE,"[BL] value (%d) not within spec, set to default brightness\n", value);
		index = pdata->pad.default_index;//value not in spec, do set default value
	}

	if (index > 0 && index < pdata->pad.min_index)
		index = pdata->pad.min_index;

	g_backlight = index;

	if ((previous_value == 0) && (index > 0)) {
        	if (delayed_work_pending(&turn_off_panel_work))
			cancel_delayed_work_sync(&turn_off_panel_work);
		queue_delayed_work(backlight_workqueue, &turn_on_panel_work, 0);
	} else {
		backlight_debug(DEBUG_INFO,"(%s): pad set backlight %d \n", __func__,index);
		ret = AX_MicroP_setPWMValue(index);
		if (ret < 0)
        		backlight_debug(DEBUG_INFO,"(%s): pad set backlight fail\n", __func__);
	}

	previous_value = index;

	return ret;
}

static void set_pad_panel_off_func(struct work_struct *work)
{
	backlight_debug(DEBUG_VERBOSE,"[BL] (%s): pad turn off panel\n", __func__);
	AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_EN,0);
	AX_MicroP_setGPIOOutputPin(OUT_uP_PAD_LOW_BAT,0);
}

static void set_pad_panel_on_func(struct work_struct *work)
{	
	int ret = 0;
	int i = 0;

	backlight_debug(DEBUG_INFO,"(%s): pad set backlight %d \n", __func__,g_backlight);
	AX_MicroP_setPWMValue(g_backlight);
	if (ret < 0) {
		backlight_debug(DEBUG_INFO,"[BL] (%s): pad set backlight fail\n", __func__);
		for(i=0; i<5; i++) {
			AX_MicroP_setPWMValue(g_backlight);
			if (!ret)
				break;
			msleep(5);
		}
	}
	AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_EN,1);	
	backlight_debug(DEBUG_VERBOSE,"[BL] (%s): pad turn on panel\n", __func__);
}
#endif

void asus_set_bl_brightness(int value)
{
	backlight_debug(DEBUG_INFO,"[BL] (%s): %d\n", __func__, value);

       	if (backlight_mode_state == phone)
		mipi_set_backlight(value);
	#ifdef CONFIG_EEPROM_NUVOTON
	else if (backlight_mode_state == pad)
		pad_set_backlight(value);
	#endif
}

static struct led_classdev asus_backlight_led = {
	.name       = "lcd-backlight",
	.brightness = 255,
};

static int __devinit asus_backlight_probe(struct platform_device *pdev)
{
	backlight_debug(DEBUG_INFO,"[BL](%s) +++\n", __func__);

	pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
        	pr_err("%s.invalid platform data.\n", __func__);
        	return -ENODEV;
	}

	backlight_debug(DEBUG_INFO,"[BL](%s): register led_classdev\n", __func__);
	//led_classdev_register(&pdev->dev, &asus_backlight_led);			//Moved led_classdev to msm_fb.c
	
	backlight_workqueue  = create_singlethread_workqueue("PADBACKLIGHTWORKQUEUE");
	#ifdef CONFIG_EEPROM_NUVOTON
	INIT_DELAYED_WORK(&turn_off_panel_work, set_pad_panel_off_func);
	INIT_DELAYED_WORK(&turn_on_panel_work, set_pad_panel_on_func);
	#endif

	backlight_debug(DEBUG_INFO,"[BL](%s) ---\n", __func__);

	return 0;
}

static int asus_backlight_remove(struct platform_device *pdev)
{
        led_classdev_unregister(&asus_backlight_led);

	return 0;
}

static struct platform_driver bl_driver = {
	.probe  = asus_backlight_probe,
	.remove = asus_backlight_remove,
	.driver = {
        	.name   = "asus_backlight",
    	},
};

static int __init msm_pmic_led_init(void)
{
	return platform_driver_register(&bl_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&bl_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC8921 A68 backlight driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:board-8064");
