/* AP3212C.c - AP3212C proximity/light sensor driver
 *
 * Copyright (C) 2013 ASUSTek Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>

#include <asm/gpio.h>
#include <linux/of_gpio.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include "linux/cm36283.h"
#include "linux/proximity_class.h"

// Select model here
#define AP3212C_DRV_NAME	"ap3212c"
#define DRIVER_VERSION		1

#define PROXIMITY_CALIBRATION_FILE_HI		PSENSOR_CALIBRATION_HI_ASUS_NV_FILE
#define PROXIMITY_CALIBRATION_FILE_LO		PSENSOR_CALIBRATION_LO_ASUS_NV_FILE
#define LIGHT_CALIBRATION_FILE_GAIN			LSENSOR_CALIBRATION_ASUS_NV_FILE
#define LIGHT_CALIBRATION_FILE_SHIFT			LSENSOR_CALIBRATION_SHIFT_ASUS_NV_FILE

#define DEBUG_TEST_DELAY 1
#define AP3212C_GPIO_PROXIMITY_INT			65
#define AP3212C_NUM_CACHABLE_REGS			26
#define AP3212C_THRES_OVERHEAD 			10

#define AP3212C_RAN_COMMAND	0x10
#define AP3212C_RAN_MASK		0x30
#define AP3212C_RAN_SHIFT		(4)
#define AP3212C_PERS_MASK		0x0f
#define AP3212C_PERS_SHIFT		(0)

#define AP3212C_MODE_COMMAND	0x00
#define AP3212C_MODE_SHIFT		(0)
#define AP3212C_MODE_MASK		0x07

#define AP3212C_ADC_LSB			0x0c
#define AP3212C_ADC_MSB			0x0d

#define AP3212C_PX_LSB			0x0e
#define AP3212C_PX_MSB			0x0f
#define AP3212C_PX_LSB_MASK	0x0f
#define AP3212C_PX_MSB_MASK	0x3f

#define AP3212C_OBJ_COMMAND	0x0f
#define AP3212C_OBJ_MASK		0x80
#define AP3212C_OBJ_SHIFT		(7)

#define AP3212C_INT_COMMAND	0x01
#define AP3212C_INT_SHIFT		(0)
#define AP3212C_INT_MASK		0x03
#define AP3212C_INT_PMASK		0x02
#define AP3212C_INT_AMASK		0x01

#define AP3212C_ALS_LTHL			0x1a
#define AP3212C_ALS_LTHL_SHIFT		(0)
#define AP3212C_ALS_LTHL_MASK		0xff

#define AP3212C_ALS_LTHH			0x1b
#define AP3212C_ALS_LTHH_SHIFT		(0)
#define AP3212C_ALS_LTHH_MASK		0xff

#define AP3212C_ALS_HTHL			0x1c
#define AP3212C_ALS_HTHL_SHIFT		(0)
#define AP3212C_ALS_HTHL_MASK		0xff

#define AP3212C_ALS_HTHH			0x1d
#define AP3212C_ALS_HTHH_SHIFT		(0)
#define AP3212C_ALS_HTHH_MASK		0xff

#define AP3212C_PX_CONFIGURE		0x20
#define AP3212C_PX_IT_MASK			0xf0
#define AP3212C_PX_IT_SHIFT			(4)
#define AP3212C_PX_GAIN_MASK		0x0c
#define AP3212C_PX_GAIN_SHIFT		(2)
#define AP3212C_PX_PERS_MASK		0x03
#define AP3212C_PX_PERS_SHIFT		(0)

#define AP3212C_LED_CONFIGURE		0x21
#define AP3212C_LED_PULSE_MASK		0x30
#define AP3212C_LED_PULSE_SHIFT		(4)
#define AP3212C_LED_DUTY_MASK		0x03
#define AP3212C_LED_DUTY_SHIFT		(0)

#define AP3212C_PX_INT				0x22
#define AP3212C_PX_INT_MASK			0x01
#define AP3212C_PX_INT_SHIFT		(0)

#define AP3212C_PX_MEAN				0x23
#define AP3212C_PX_MEAN_MASK		0x03
#define AP3212C_PX_MEAN_SHIFT		(0)

#define AP3212C_PX_WAIT				0x24
#define AP3212C_PX_WAIT_MASK		0x1f
#define AP3212C_PX_WAIT_SHIFT		(0)

#define AP3212C_PX_CAL_LOW			0x28
#define AP3212C_PX_CAL_LOW_MASK	0x01
#define AP3212C_PX_CAL_LOW_SHIFT	(0)

#define AP3212C_PX_CAL_HI			0x29
#define AP3212C_PX_CAL_HI_MASK		0xff
#define AP3212C_PX_CAL_HI_SHIFT		(0)

#define AP3212C_PX_LTHL				0x2a
#define AP3212C_PX_LTHL_SHIFT		(0)
#define AP3212C_PX_LTHL_MASK		0x03

#define AP3212C_PX_LTHH				0x2b
#define AP3212C_PX_LTHH_SHIFT		(0)
#define AP3212C_PX_LTHH_MASK		0xff

#define AP3212C_PX_HTHL				0x2c
#define AP3212C_PX_HTHL_SHIFT		(0)
#define AP3212C_PX_HTHL_MASK		0x03

#define AP3212C_PX_HTHH				0x2d
#define AP3212C_PX_HTHH_SHIFT		(0)
#define AP3212C_PX_HTHH_MASK		0xff

/* AP3212 User Settings */
#define AP3212C_POW_DOWN   	0x00			//Power off
#define AP3212C_ALS_ON  			0x01			//ALS on only
#define AP3212C_PS_ON  			0x02			//PS on only
#define AP3212C_POW_UP  		0x03			//ALS and PS on
#define AP3212C_RESET    			0x04			//software reset: clears all registers

#define AP3212C_ALS_RAN			0x03			//0=65535, 1=16383, 2=4095, 3=1023
#define AP3212C_ALS_PERS		0x02			//default value: 1 consecutive conversion time out of range
#define AP3212C_ALS_CAL			0x40			//default value: calibration factor 64/64 (use asus calibration method)

#define AP3212C_PS_IT			0x0e			//PS/IR integrated time select (default: 0000) 0x01 = 2T
#define AP3212C_PS_GAIN			0x01			//PS gain (default: 0x01) 0x01 = 2
#define AP3212C_PS_PERS			0x00			//PS persist (default: 0x01) 0x03 = 8 conversion time
#define AP3212C_PS_PULSE		0x03			//LED pulse (default: 0x01) 0x01 = 1 pulse
#define AP3212C_PS_DUTY			0x03			//LED driver ration (default: 0x03) 0x03 = 100%
#define AP3212C_PS_INT			0x01			//PS INT mode (default: 0x01) 0 = zone type, 1 = hysteresis type
#define AP3212C_PS_MEAN			0x03			//PS mean time (default:0x00) 0 = 12.5ms, 1=25ms, 2=37.5ms, 3=50ms
#define AP3212C_PS_WAIT			0x00			//PS waiting time (default: 0x00) 0=0 mean time....3f=63 mean time

#define ENABLE				0X01
#define DISABLE				0X00

#define ALS_ACTIVE    0x01
#define PS_ACTIVE    0x02

extern int g_ASUS_hw_sku_id;
extern bool I2C_BUS2_SUSPENDING;

static int g_ps_threshold_lo = 300;
static int g_ps_threshold_hi = 850;

static int report_proxm = 0;		//Use backdoor to enable/disable ps event report

struct ap3212c_data {
	struct i2c_client *client;
	struct mutex lock;
	struct mutex reg_lock;
	u8 reg_cache[AP3212C_NUM_CACHABLE_REGS];
	int irq;

	struct workqueue_struct *wq;
	struct workqueue_struct *delay_wq;
	struct workqueue_struct *debug_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notif;
	int fb_register_fail;
#endif

	struct input_dev *light_input_dev;
	struct work_struct work_light;
	struct delayed_work delay_work_light;
	struct delayed_work delay_work_proximity;
	struct delayed_work als_test_work;
	struct hrtimer light_timer;
	ktime_t light_poll_delay;

	struct input_dev *proximity_input_dev;
	struct work_struct work_proximity;
	struct delayed_work ps_test_work;
	struct hrtimer proximity_timer;
	ktime_t proximity_poll_delay;
	struct wake_lock prx_wake_lock;

	struct work_struct ISR_work;
};

// ap3212c register
static u8 ap3212c_reg[AP3212C_NUM_CACHABLE_REGS] = 
	{0x00,0x01,0x02,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
	 0x10,0x19,0x1a,0x1b,0x1c,0x1d,
	 0x20,0x21,0x22,0x23,0x24,0x28,0x29,0x2a,0x2b,0x2c,0x2d};

static int ap3212c_range[4] = {65535,16383,4095,1023};

int cali = 100;
u8 suspend_mode;

// Set the polling mode or int mode here.
u8 als_polling = 0;
u8 ps_polling = 0;

static int ap3212c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ap3212c_remove(struct i2c_client *client);

// r/w calibration value ++
static u32 g_ap3212c_light_shift_calibration = DEFAULT_ALS_SHIFT;
static u16 g_ap3212c_light_calibration_fval_x1000 = DEFAULT_ALS_GAIN;

void ps_adc_report_event(int data);
static int ap3212c_turn_onoff_als(bool);
static int ap3212c_turn_onoff_proxm(bool);
static int atd_write_status_and_adc_2(int *, int *, int *);
static int get_adc_calibrated_lux_from_ap3212c(int);
static bool read_lightsensor_shiftvalue(char *);
static bool read_lightsensor_calibrationvalue(char *);
static bool read_prox_hi_calibrationvalue(void);
static bool read_prox_lo_calibrationvalue(void);
#ifdef ASUS_FACTORY_BUILD
static void write_lightsensor_calibrationvalue_work(struct work_struct *work);
static struct write_calvalue {
    struct work_struct write_calvalue_work;
    int calvalue;
} *ap3212c_write_calvalue;

static void write_lightsensor_shiftvalue_work(struct work_struct *work);
static struct write_shift {
    struct work_struct write_shift_work;
    int calvalue;
} *ap3212c_write_shift;

static void write_prox_hi_calibrationvalue_work(struct work_struct *work);
static struct write_prox_hi {
    struct work_struct write_prox_hi_work;
    int calvalue;
} *ap3212c_write_prox_hi;


static void write_prox_lo_calibrationvalue_work(struct work_struct *work);
static struct write_prox_lo {
    struct work_struct write_prox_lo_work;
    int calvalue;
} *ap3212c_write_prox_lo;

static int a_als_low_calibration_adc = 0;
static int a_als_high_calibration_adc = 0;
static int a_ps_hi_calibration_adc = 0;
static int a_ps_lo_calibration_adc = 0;
#endif
// for r/w calibration value --

static int a_als_calibration_ratio = 100;

struct i2c_client *ap3212c_client;
struct ap3212c_data *g_ap3212c_data;
extern int g_HAL_als_switch_on;				// For all lightsensor trun on/off global flag
static bool g_ap3212c_ps_switch_on = 0;
static bool g_ap3212c_als_switch_on = 0;
static int g_ambient_suspended = 0;
u16 g_lux_light=0;
static u16 g_last_ap3212c_light=0;
static int g_ap3212c_light_first=1;
static int g_max_light_level = 17;
static int g_ap3212c_light_map[17] = {0,50,100,200,300,400,500,650,800,1000,1500,2000,3000,4000,5000,7000,10000};
static int g_proxm_dbg = 0; /* Add for debug only */
static int g_ambient_dbg = 0; /* Add for debug only */
static int g_interval = 100;
static int g_ap3212c_reset = 0;
static bool g_proxm_state = 0;			/*For check proximity near/far state*/
static int g_als_threshold_lo = 0;
static int g_als_threshold_hi = 0;
static int g_ap3212c_on_fail=0;
static int g_ap3212c_earlysuspend_int = 0;

#define ADD_TO_IDX(addr,idx)	{														\
									int i;												\
									for(i = 0; i < AP3212C_NUM_CACHABLE_REGS; i++)						\
									{													\
										if (addr == ap3212c_reg[i])						\
										{												\
											idx = i;									\
											break;										\
										}												\
									}													\
								}

/*
 * register access helpers
 */

static int __ap3212c_read_reg(struct i2c_client *client, u32 reg, u8 mask, u8 shift)
{
	u8 idx = 0xff;

	ADD_TO_IDX(reg,idx)
	return (g_ap3212c_data->reg_cache[idx] & mask) >> shift;
}

static int __ap3212c_write_reg(struct i2c_client *client, u32 reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0;
	u8 tmp;
	u8 idx = 0xff;

	ADD_TO_IDX(reg,idx)
	if (idx >= AP3212C_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&g_ap3212c_data->reg_lock);

	tmp = g_ap3212c_data->reg_cache[idx];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		g_ap3212c_data->reg_cache[idx] = tmp;

	mutex_unlock(&g_ap3212c_data->reg_lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap3212c_get_range(struct i2c_client *client)
{
	u8 idx = __ap3212c_read_reg(client, AP3212C_RAN_COMMAND, AP3212C_RAN_MASK, AP3212C_RAN_SHIFT); 
	return ap3212c_range[idx];
}

static int ap3212c_set_range(struct i2c_client *client, int range)
{
	return __ap3212c_write_reg(client, AP3212C_RAN_COMMAND, AP3212C_RAN_MASK, AP3212C_RAN_SHIFT, range);
}

/* persist */
static int ap3212c_set_persist(struct i2c_client *client, int persist)
{
	return __ap3212c_write_reg(client, AP3212C_RAN_COMMAND, AP3212C_PERS_MASK, AP3212C_PERS_SHIFT, persist);
}

/* mode */
static int ap3212c_get_mode(struct i2c_client *client)
{
	int ret;

	ret = __ap3212c_read_reg(client, AP3212C_MODE_COMMAND, AP3212C_MODE_MASK, AP3212C_MODE_SHIFT);
	return ret;
}

static int ap3212c_set_mode(struct i2c_client *client, int mode)
{  
  if (mode != ap3212c_get_mode(client))
  {
    __ap3212c_write_reg(client, AP3212C_MODE_COMMAND, AP3212C_MODE_MASK, AP3212C_MODE_SHIFT, mode);
	
	if (als_polling)
	{
		/* Enable/Disable ALS */
		if (ALS_ACTIVE & mode)
			hrtimer_start(&g_ap3212c_data->light_timer, g_ap3212c_data->light_poll_delay, HRTIMER_MODE_REL);
		else
		{
			hrtimer_cancel(&g_ap3212c_data->light_timer);
			cancel_work_sync(&g_ap3212c_data->work_light);
		}
	}

	if (ps_polling)
	{
		/* Enable/Disable PS */
		if (PS_ACTIVE & mode)
		{
			wake_lock(&g_ap3212c_data->prx_wake_lock);
			hrtimer_start(&g_ap3212c_data->proximity_timer, g_ap3212c_data->proximity_poll_delay, HRTIMER_MODE_REL);
		}
		else
		{
			wake_unlock(&g_ap3212c_data->prx_wake_lock);
			hrtimer_cancel(&g_ap3212c_data->proximity_timer);
			cancel_work_sync(&g_ap3212c_data->work_proximity);
		}
	}
  }
  return 0;
}

/* ALS low threshold */
static int ap3212c_get_althres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3212c_read_reg(client, AP3212C_ALS_LTHL, AP3212C_ALS_LTHL_MASK, AP3212C_ALS_LTHL_SHIFT);
	msb = __ap3212c_read_reg(client, AP3212C_ALS_LTHH, AP3212C_ALS_LTHH_MASK, AP3212C_ALS_LTHH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3212c_set_althres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AP3212C_ALS_LTHL_MASK;

	err = __ap3212c_write_reg(client, AP3212C_ALS_LTHL, AP3212C_ALS_LTHL_MASK, AP3212C_ALS_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3212c_write_reg(client, AP3212C_ALS_LTHH, AP3212C_ALS_LTHH_MASK, AP3212C_ALS_LTHH_SHIFT, msb);

	return err;
}

/* ALS high threshold */
static int ap3212c_get_ahthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3212c_read_reg(client, AP3212C_ALS_HTHL, AP3212C_ALS_HTHL_MASK, AP3212C_ALS_HTHL_SHIFT);
	msb = __ap3212c_read_reg(client, AP3212C_ALS_HTHH, AP3212C_ALS_HTHH_MASK, AP3212C_ALS_HTHH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3212c_set_ahthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AP3212C_ALS_HTHL_MASK;
	
	err = __ap3212c_write_reg(client, AP3212C_ALS_HTHL, AP3212C_ALS_HTHL_MASK, AP3212C_ALS_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3212c_write_reg(client, AP3212C_ALS_HTHH, AP3212C_ALS_HTHH_MASK, AP3212C_ALS_HTHH_SHIFT, msb);

	return err;
}

/* PX low threshold */
static int ap3212c_get_plthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3212c_read_reg(client, AP3212C_PX_LTHL, AP3212C_PX_LTHL_MASK, AP3212C_PX_LTHL_SHIFT);
	msb = __ap3212c_read_reg(client, AP3212C_PX_LTHH, AP3212C_PX_LTHH_MASK, AP3212C_PX_LTHH_SHIFT);
	return ((msb << 2) | lsb);
}

static int ap3212c_set_plthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 2;
	lsb = val & AP3212C_PX_LTHL_MASK;
	
	err = __ap3212c_write_reg(client, AP3212C_PX_LTHL, AP3212C_PX_LTHL_MASK, AP3212C_PX_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3212c_write_reg(client, AP3212C_PX_LTHH, AP3212C_PX_LTHH_MASK, AP3212C_PX_LTHH_SHIFT, msb);

	return err;
}

/* PX high threshold */
static int ap3212c_get_phthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3212c_read_reg(client, AP3212C_PX_HTHL, AP3212C_PX_HTHL_MASK, AP3212C_PX_HTHL_SHIFT);
	msb = __ap3212c_read_reg(client, AP3212C_PX_HTHH, AP3212C_PX_HTHH_MASK, AP3212C_PX_HTHH_SHIFT);
	return ((msb << 2) | lsb);
}

static int ap3212c_set_phthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 2;
	lsb = val & AP3212C_ALS_HTHL_MASK;
	
	err = __ap3212c_write_reg(client, AP3212C_PX_HTHL, AP3212C_PX_HTHL_MASK, AP3212C_PX_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3212c_write_reg(client, AP3212C_PX_HTHH, AP3212C_PX_HTHH_MASK, AP3212C_PX_HTHH_SHIFT, msb);

	return err;
}

static int ap3212c_get_adc_value(struct i2c_client *client, int lock)
{
	unsigned int lsb, msb, range;
	unsigned long tmp;

	if (!lock)	mutex_lock(&g_ap3212c_data->lock);
	lsb = i2c_smbus_read_byte_data(client, AP3212C_ADC_LSB);

	if (lsb < 0) {
		if (!lock)	mutex_unlock(&g_ap3212c_data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AP3212C_ADC_MSB);
	if (!lock)	mutex_unlock(&g_ap3212c_data->lock);

	if (msb < 0)
		return msb;

	range = ap3212c_get_range(client);

	tmp = ((msb<<8) | lsb);
	tmp *= cali;

	return (tmp / 100);
}

static int ap3212c_get_object(struct i2c_client *client, int lock)
{
	int val;

	if (!lock)	mutex_lock(&g_ap3212c_data->lock);
	val = i2c_smbus_read_byte_data(client, AP3212C_OBJ_COMMAND);
	val &= AP3212C_OBJ_MASK;

	if (!lock)	mutex_unlock(&g_ap3212c_data->lock);

	return val >> AP3212C_OBJ_SHIFT;
}

static int ap3212c_get_intstat(struct i2c_client *client)
{
	int val;

	while (I2C_BUS2_SUSPENDING) {
		printk("[ap3212c] I2C not ready, waiting...\n");
		msleep(3);
	}
	val = i2c_smbus_read_byte_data(client, AP3212C_INT_COMMAND);

	val &= AP3212C_INT_MASK;

	return val >> AP3212C_INT_SHIFT;
}

static int ap3212c_set_ps_it(struct i2c_client *client, int it)
{
	return __ap3212c_write_reg(client, AP3212C_PX_CONFIGURE, AP3212C_PX_IT_MASK, AP3212C_PX_IT_SHIFT, it);
}

static int ap3212c_set_ps_gain(struct i2c_client *client, int gain)
{
	return __ap3212c_write_reg(client, AP3212C_PX_CONFIGURE, AP3212C_PX_GAIN_MASK, AP3212C_PX_GAIN_SHIFT, gain);
}

static int ap3212c_set_ps_pers(struct i2c_client *client, int pers)
{
	return __ap3212c_write_reg(client, AP3212C_PX_CONFIGURE, AP3212C_PX_PERS_MASK, AP3212C_PX_PERS_SHIFT, pers);
}

static int ap3212c_set_ps_pulse(struct i2c_client *client, int pulse)
{
	return __ap3212c_write_reg(client, AP3212C_LED_CONFIGURE, AP3212C_LED_PULSE_MASK, AP3212C_LED_PULSE_SHIFT, pulse);
}

static int ap3212c_set_ps_duty(struct i2c_client *client, int duty)
{
	return __ap3212c_write_reg(client, AP3212C_LED_CONFIGURE, AP3212C_LED_DUTY_MASK, AP3212C_LED_DUTY_SHIFT, duty);
}

static int ap3212c_set_ps_int(struct i2c_client *client, int interrupt)
{
	return __ap3212c_write_reg(client, AP3212C_PX_INT, AP3212C_PX_INT_MASK, AP3212C_PX_INT_SHIFT, interrupt);
}

static int ap3212c_set_ps_mean(struct i2c_client *client, int mean)
{
	return __ap3212c_write_reg(client, AP3212C_PX_MEAN, AP3212C_PX_MEAN_MASK, AP3212C_PX_MEAN_SHIFT, mean);
}

static int ap3212c_get_px_value(struct i2c_client *client, int lock)
{
	u8 lsb, msb;

	if (!lock) mutex_lock(&g_ap3212c_data->lock);
	
	lsb = i2c_smbus_read_byte_data(client, AP3212C_PX_LSB);
	msb = i2c_smbus_read_byte_data(client, AP3212C_PX_MSB);

	if (!lock)	mutex_unlock(&g_ap3212c_data->lock);

	return (u32)(((msb & AP3212C_PX_MSB_MASK) << 4) | (lsb & AP3212C_PX_LSB_MASK));
}

#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int i2c_stresstest_turn_on_als(bool bOn)
{
	int err = 0;

	//bOn always on
	if(g_ap3212c_als_switch_on != bOn) {		
		if(g_ap3212c_ps_switch_on)
			ap3212c_set_mode(ap3212c_client, 3);
		else
			ap3212c_set_mode(ap3212c_client, 1);

		if(err < 0)
			return err;
		g_ap3212c_als_switch_on = 1;
	}
	return 0;
}

static int i2c_stresstest_turn_on_proxm(bool bOn)
{
	int err = 0;

	if ( bOn == 1 )	{
		if (g_ap3212c_als_switch_on) 
			ap3212c_set_mode(ap3212c_client, 3);		//enable ALS and PS+IR functions
		else 
			ap3212c_set_mode(ap3212c_client, 2);		//enable PS+IR functions only

		if(err < 0)
			return -1;
	}
	return 0;
}

static int TestAP3212CSensorI2C (struct i2c_client *apClient)
{
	int lnResult = I2C_TEST_PASS;
	int err = 0;

	i2c_log_in_test_case("TestLSensorI2C ++\n");

	/* Light sensor */
	g_HAL_als_switch_on = 1;
	printk(DBGMSK_PRX_G3"[ap3212c][als] Turn on ap3212c\n");
	err = i2c_stresstest_turn_on_als(g_HAL_als_switch_on);
	if(err < 0)	{
		g_HAL_als_switch_on = 0;
		i2c_log_in_test_case("Fail to turn on lsensor\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		goto error_1;
	}
	else
		ap3212c_turn_onoff_als(0);
	g_HAL_als_switch_on = 0;
		/* Proximity sensor */
	if ( g_ap3212c_ps_switch_on) {
		ap3212c_turn_onoff_proxm(0);
		printk(DBGMSK_PRX_G3"[ap3212c][ps] Turn on ap3212c\n");
		err = i2c_stresstest_turn_on_proxm(1);
		if(err < 0)	{
			i2c_log_in_test_case("Fail to turn on Psensor\n");
			lnResult = I2C_TEST_Psensor_FAIL;
			goto error_1;
		}
		else
			ap3212c_turn_onoff_proxm(0);
	}else {
		printk(DBGMSK_PRX_G3"[ap3212c][ps] Turn on ap3212c\n");
		err = i2c_stresstest_turn_on_proxm(1);
		if(err < 0)	{
			i2c_log_in_test_case("Fail to turn on Psensor\n");
			lnResult = I2C_TEST_Psensor_FAIL;
			goto error_1;
		}
		else
			ap3212c_turn_onoff_proxm(0);
	}

	i2c_log_in_test_case("TestLSensorI2C --\n");

error_1:
	return lnResult;
}

static struct i2c_test_case_info gLSensorTestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(TestAP3212CSensorI2C),
};
#endif

/*
 * sysfs layer
 */

/* adc */
static ssize_t ap3212c_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* No ADC data if power down */
	if (ap3212c_get_mode(ap3212c_client) == 0x00)
		return sprintf((char*) buf, "%s\n", "Please power up first!");

	return sprintf(buf, "%d\n", ap3212c_get_adc_value(ap3212c_client,0));
}

static DEVICE_ATTR(adc, S_IRUGO, ap3212c_show_adc, NULL);


/* Px data */
static ssize_t ap3212c_show_pxvalue(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* No Px data if power down */
	if (ap3212c_get_mode(ap3212c_client) == 0x00)
		return -EBUSY;

	return sprintf(buf, "%d\n", ap3212c_get_px_value(ap3212c_client,0));
}

static DEVICE_ATTR(proxm, S_IRUGO, ap3212c_show_pxvalue, NULL);


/* proximity object detect */
static ssize_t ap3212c_show_object(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3212c_get_object(ap3212c_client,0));
}

static DEVICE_ATTR(object, S_IRUGO, ap3212c_show_object, NULL);

#if defined(ASUS_USERDEBUG_BUILD) || defined(ASUS_ENG_BUILD)
/* enable/disable ps event reporting */
static ssize_t ap3212c_store_ps_report_event_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if ( (kstrtoul(buf, 10, &val) < 0) )
		return -EINVAL;

	report_proxm = (int)val;

	printk("[ap3212c] PS report event : %s\n", report_proxm ? "on" : "off");

	return count;
}

static DEVICE_ATTR(ps_report, S_IRWXU | S_IRWXG| S_IRWXO, NULL, ap3212c_store_ps_report_event_onoff);
#endif

static enum proximity_property ap3212c_proxmdev_properties[] = {
    /* int */
    SENSORS_PROP_INTERVAL,
    SENSORS_PROP_HI_THRESHOLD,
    SENSORS_PROP_LO_THRESHOLD,
    SENSORS_PROP_MAXRANGE,      /* read only */
    SENSORS_PROP_RESOLUTION,    /* read only */
    SENSORS_PROP_VERSION,       /* read only */
    SENSORS_PROP_CURRENT,       /* read only */
    SENSORS_PROP_DBG, /* Add for debug only */
    /* char */
    SENSORS_PROP_SWITCH,
    SENSORS_PROP_VENDOR,        /* read only */
    SENSORS_PROP_ADC,           /* adc raw data */
    SENSORS_PROP_ATD_STATUS     /* for atd mode only */
};

atomic_t proxm_update;

static int proxmdev_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	printk(DBGMSK_PRX_G4 "[ap3212c][ps] proxmdev_dev_open.\n");

	if (file->f_flags & O_NONBLOCK)
		printk(DBGMSK_PRX_G4 "[ap3212c][ps] proxmdl_dev_open (O_NONBLOCK)\n");

	atomic_set(&proxm_update, 0); //initialize atomic.

	ret = nonseekable_open(inode, file);

	return ret;
}

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
};

static int ap3212c_proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	switch( property ) 
	{
		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_ps_threshold_hi);
			val->intval = g_ps_threshold_hi;
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_ps_threshold_lo);
			val->intval = g_ps_threshold_lo;
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_INTERVAL.\n");
			val->intval = g_interval;
			break;

		case SENSORS_PROP_MAXRANGE:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_MAXRANGE.\n");
			val->intval = 1; //keep it 1.0 for OMS.
			break;

		case SENSORS_PROP_RESOLUTION:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_RESOLUTION.\n");
			val->intval = 1;
			break;

		case SENSORS_PROP_VERSION:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_VERSION.\n");
			sprintf(val->strval, "Ver 0");
			break;

		case SENSORS_PROP_CURRENT:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_CURRENT.\n");
			val->intval = 30;
			break;

		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] get switch = %d.\n", g_ap3212c_ps_switch_on);
			val->intval = g_ap3212c_ps_switch_on;
			break;

		case SENSORS_PROP_VENDOR:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] SENSORS_PROP_VENDOR.\n");
			sprintf(val->strval, "LITEON");
			break;

		/* Add for debug only */
		case SENSORS_PROP_DBG:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] dbg = %d.\n", g_proxm_dbg);
			val->intval = g_proxm_dbg;
			break;

		case SENSORS_PROP_ADC:
			val->intval = ap3212c_get_px_value(ap3212c_client,0);
			printk(DBGMSK_PRX_G4"[ap3212c][ps] get adc property: %d\n", val->intval);
			break;

		case SENSORS_PROP_ATD_STATUS:
	        {
			int als_sts =0, als_adc =0, ps_sts =0;

			atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
			val->intval = ps_sts;
			printk(DBGMSK_PRX_G4"[ap3212c][ps] get atd status: %d\n", val->intval);
			break;
		}		
		default:
			printk(DBGMSK_PRX_G0"[ap3212c][ps] default.\n");
			return -EINVAL;
	}
	return 0;
}

static int ap3212c_proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
	int ret = 0;
	static bool bFirst = true;
    	static bool openfilp = true;

	switch (property) 
	{
		case SENSORS_PROP_SWITCH:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] put SENSORS_PROP_SWITCH (%d,%d).\n", (val->intval), g_ap3212c_ps_switch_on);
			if(bFirst) {
				printk(DBGMSK_PRX_G4"[ap3212c][ps] put switch 1st read calvalue\n");
				openfilp = read_prox_hi_calibrationvalue();			

				if (openfilp == false )	{
					printk("[ap3212c][ps] Get golden calvalue : thd_hi = %d thd_lo = %d or fail\n", g_ps_threshold_hi, g_ps_threshold_lo);
					g_ps_threshold_hi = DEFAULT_PS_THRESHOLD_HI;
					g_ps_threshold_lo = DEFAULT_PS_THRESHOLD_LO;
				}
				else
					read_prox_lo_calibrationvalue();

				printk("[ap3212c] Set prox threshold hi and lo: %d , %d\n",g_ps_threshold_hi, g_ps_threshold_lo);
				bFirst = false;
			}

			if((g_ap3212c_ps_switch_on != val->intval))	{			
				if(val->intval==1)	{					//turn on PS
					ps_adc_report_event(0);			//report initial far value
					ret = ap3212c_turn_onoff_proxm(1);
					g_ap3212c_ps_switch_on = 1;
					printk(DBGMSK_PRX_G4"[ap3212c][ps] proximity on.\n");
				}else {								//turn off PS if val->intval==0 or other
					g_ap3212c_ps_switch_on = 0;
					ap3212c_turn_onoff_proxm(0);
					printk(DBGMSK_PRX_G4"[ap3212c][ps] proximity off.\n");
				}
			}
			break;

		case SENSORS_PROP_HI_THRESHOLD:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] config THRESHOLD (%d).\n", val->intval);
			if(val->intval>=3 && val->intval<=255) {
				if(g_ps_threshold_hi != val->intval) {
					g_ps_threshold_hi = val->intval;
					ret = ap3212c_set_phthres(ap3212c_client, g_ps_threshold_hi);
				}
			}
			else
				printk(DBGMSK_PRX_G0"[ap3212c][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
			break;

		case SENSORS_PROP_LO_THRESHOLD:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] config low THRESHOLD (%d).\n", val->intval);
			if(val->intval>=0 && val->intval<=255) {
				if(g_ps_threshold_lo != val->intval) {
					g_ps_threshold_lo = val->intval;
					ret = ap3212c_set_plthres(ap3212c_client, g_ps_threshold_lo);
				}
			}
			else
				printk(DBGMSK_PRX_G0"[ap3212c][ps] ERROR!!! OUT OF THRESHOLD (0~255)!!! \n");
			break;

		case SENSORS_PROP_INTERVAL:
			printk(DBGMSK_PRX_G4"[ap3212c][ps] set interval (0x%x)\n", val->intval);
			break;
		/* Add for debug only */
		case SENSORS_PROP_DBG:
			g_proxm_dbg = val->intval;
			if ( g_proxm_dbg == 1 && g_ap3212c_ps_switch_on == 1)
				queue_delayed_work(g_ap3212c_data->debug_wq, &g_ap3212c_data->ps_test_work, HZ * DEBUG_TEST_DELAY);
			else if ( g_proxm_dbg == 2 && g_ap3212c_ps_switch_on == 1)
				queue_delayed_work(g_ap3212c_data->debug_wq, &g_ap3212c_data->ps_test_work, HZ * DEBUG_TEST_DELAY);
			if ( g_proxm_dbg == 0 )
				cancel_delayed_work(&g_ap3212c_data->ps_test_work);
			printk(DBGMSK_PRX_G4"[ap3212c][ps] dbg = %d.\n", g_proxm_dbg);
			break;

		default:
			printk(DBGMSK_PRX_G0"[ap3212c][ps] put default.\n");
			return -EINVAL;
	}
	return 0;
}

struct proximity_class_dev ap3212c_proxmDev = {
	.id = SENSORS_PROXIMITY,
	.name = "psensor",
	.num_properties = ARRAY_SIZE(ap3212c_proxmdev_properties),
	.properties = ap3212c_proxmdev_properties,
	.get_property = ap3212c_proxmdev_get_property,
	.put_property = ap3212c_proxmdev_put_property,
	.fops = &proxmdev_fops
};

static int ap3212c_turn_onoff_proxm(bool bOn)
{
	if(bOn == 1 && g_ap3212c_ps_switch_on==0)	{	//power on
		printk(DBGMSK_PRX_G4"[ap3212c][ps] sensor switch, turn on proximity sensor ++.\n");
			
		/* Clear INT*/
		i2c_smbus_write_byte_data(ap3212c_client, AP3212C_INT_COMMAND, AP3212C_INT_PMASK);

		/* Init client */		
		ap3212c_set_ps_it(ap3212c_client, AP3212C_PS_IT);
		ap3212c_set_ps_gain(ap3212c_client, AP3212C_PS_GAIN);
		ap3212c_set_ps_pers(ap3212c_client, AP3212C_PS_PERS);
		ap3212c_set_ps_pulse(ap3212c_client, AP3212C_PS_PULSE);
		ap3212c_set_ps_duty(ap3212c_client, AP3212C_PS_DUTY);
		ap3212c_set_ps_int(ap3212c_client, AP3212C_PS_INT);
		ap3212c_set_ps_mean(ap3212c_client, AP3212C_PS_MEAN);
		ap3212c_set_plthres(ap3212c_client, g_ps_threshold_lo);
		ap3212c_set_phthres(ap3212c_client, g_ps_threshold_hi);
		
		/* Enable PS */
		if (g_ap3212c_als_switch_on) 
			ap3212c_set_mode(ap3212c_client, 3);		//enable ALS and PS+IR functions
		else 
			ap3212c_set_mode(ap3212c_client, 2);		//enable PS+IR functions only

		g_ap3212c_ps_switch_on = 1;

		/* Enable wake lock */
		wake_lock(&g_ap3212c_data->prx_wake_lock);

		/* report distance immediately after ps switched on */
		queue_delayed_work(g_ap3212c_data->wq, &g_ap3212c_data->delay_work_proximity,10);

		printk("[ap3212c][ps]turn on proximity sensor --. lo_thresh = %d, hi_thresh = %d\n", ap3212c_get_plthres(ap3212c_client), ap3212c_get_phthres(ap3212c_client));
	}
	else		//power off
	{
		printk(DBGMSK_PRX_G4"[ap3212c][ps] sensor switch, turn on proximity sensor ++.\n");
		
		/* Disable PS & turn on als (if it was originally on)*/
		if (g_ap3212c_als_switch_on) 
			ap3212c_set_mode(ap3212c_client, 1);
		else 
			ap3212c_set_mode(ap3212c_client, 0);

		g_ap3212c_ps_switch_on = 0;
		
		/* Disable wake lock */
		wake_unlock(&g_ap3212c_data->prx_wake_lock);

		printk("[ap3212c][ps] turn off proximity sensor --.\n");
	}

	return 0;
}

void ps_adc_report_event(int data)
{
	printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
	printk(DBGMSK_PRX_G4"[ap3212c][ps] PS_data = %d\n",data);
	
	if(data >= g_ps_threshold_hi) {  //panel off
		input_report_abs(g_ap3212c_data->proximity_input_dev, ABS_DISTANCE, 0);
		input_sync(g_ap3212c_data->proximity_input_dev);
		g_proxm_state = 1;
		printk("[ap3212c][ps] trigger panel off\n");
	}else	{
		input_report_abs(g_ap3212c_data->proximity_input_dev, ABS_DISTANCE, 1);
		input_sync(g_ap3212c_data->proximity_input_dev);
		wake_unlock(&g_ap3212c_data->prx_wake_lock);
		g_proxm_state = 0;
		printk("[ap3212c][ps] trigger panel on\n");
	}

	#ifndef INPUT_EVENT_MODE
		atomic_inc(&proxm_update);
		printk(DBGMSK_PRX_G4"[ap3212c][ps] proxm_interrupt_handler, state_change\n");
		printk(DBGMSK_PRX_G4"[ap3212c][ps] proxm_interrupt_handler fire(%d)\n",atomic_read(&proxm_update));
		printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
	#endif
}

static void proximity_test_delayed_work(struct work_struct *work)
{
	if ( g_proxm_dbg == 2 )
		queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_proximity);
	else if ( g_proxm_dbg == 1 )
		printk(DBGMSK_PRX_G4"[ap3212c][ps] PS_data = %d\n", ap3212c_get_px_value(ap3212c_client, 0));

	if ( g_proxm_dbg > 0 )
		queue_delayed_work(g_ap3212c_data->debug_wq, &g_ap3212c_data->ps_test_work, HZ * DEBUG_TEST_DELAY);
	else
		cancel_delayed_work(&g_ap3212c_data->ps_test_work);
}

static enum proximity_property ambientDev_properties[] = {
    /* int */
    SENSORS_PROP_INTERVAL,
    SENSORS_PROP_HI_THRESHOLD,
    SENSORS_PROP_LO_THRESHOLD,
    SENSORS_PROP_MAXRANGE,      /* read only */
    SENSORS_PROP_RESOLUTION,    /* read only */
    SENSORS_PROP_VERSION,       /* read only */
    SENSORS_PROP_CURRENT,       /* read only */
    SENSORS_PROP_DBG,           /* Add for debug only */
    /* char */
    SENSORS_PROP_SWITCH,
    SENSORS_PROP_VENDOR,        /* read only */
    SENSORS_PROP_CALIBRATION,   /* Old_calibration value */
    SENSORS_PROP_ADC,           /* adc raw data */
    SENSORS_PROP_K_ADC,         /* adc raw data w/ calibrated */
    SENSORS_PROP_LUX,            /* lux data (calibrated) */
    SENSORS_PROP_ATD_STATUS,    /* for atd mode only */
    SENSORS_PROP_ATD_ADC,        /* for atd mode only */
};

atomic_t ambient_update;

static int ambientDev_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    printk(DBGMSK_PRX_G3"[ap3212c][als] ambientdl_dev_open \n");

    if (file->f_flags & O_NONBLOCK) {
        printk(DBGMSK_PRX_G2"[ap3212c][als] ambientdl_dev_open (O_NONBLOCK)\n");
    }
    atomic_set(&ambient_update, 0); //initialize atomic.

    ret = nonseekable_open(inode, file);

    return ret;
}

static struct file_operations ambientDev_fops = {
    .owner = THIS_MODULE,
    .open = ambientDev_open,
};

static int ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    printk(DBGMSK_PRX_G3"[ap3212c][als] ambientdl_get_property +.\n");

    switch( property ) 
    {
        case SENSORS_PROP_HI_THRESHOLD:
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_HI_THRESHOLD. (%d)\n", g_als_threshold_hi);
            val->intval = g_als_threshold_hi;
            break;

        case SENSORS_PROP_LO_THRESHOLD:
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_LO_THRESHOLD. (%d)\n", g_als_threshold_lo);
            val->intval = g_als_threshold_lo;
            break;

        case SENSORS_PROP_INTERVAL:
            printk(DBGMSK_PRX_G3"[ap3212c][als] config GAIN_ALS by using \"echo XXX > /sys/class/sensors/sensor4/interval\" XXX is only 0~3. \n");
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_INTERVAL.\n");
            val->intval = g_interval;
            break;

        case SENSORS_PROP_MAXRANGE:
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_MAXRANGE.\n");
            val->intval = 128;  
            break;

        case SENSORS_PROP_RESOLUTION:
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_RESOLUTION.\n");
            val->intval = 1;
            break;

        case SENSORS_PROP_VERSION:
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_VERSION.\n");
            val->intval = 1;    
            break;

        case SENSORS_PROP_CURRENT:
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_CURRENT.\n");
            val->intval = 30;   
            break;

        case SENSORS_PROP_SWITCH:
            printk(DBGMSK_PRX_G3"[ap3212c][als] get switch = %d.\n", g_HAL_als_switch_on);
            val->intval = g_HAL_als_switch_on;
            break;

        case SENSORS_PROP_VENDOR:
            printk(DBGMSK_PRX_G3"[ap3212c][als] SENSORS_PROP_VENDOR.\n");
            sprintf(val->strval, "LITEON");
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            val->intval = g_ambient_dbg;
            printk(DBGMSK_PRX_G3"[ap3212c][als] dbg = %d.\n", g_ambient_dbg);
            break;
        case SENSORS_PROP_ADC:
            val->intval = ap3212c_get_adc_value(ap3212c_client,0);
            printk(DBGMSK_PRX_G3"[ap3212c][als] get adc property: %d\n", val->intval);
            break;
        case SENSORS_PROP_K_ADC:
            val->intval = get_adc_calibrated_lux_from_ap3212c(ap3212c_get_adc_value(ap3212c_client,0));
            printk(DBGMSK_PRX_G3"[ap3212c][als] get k_adc property: %d\n", val->intval);
	    break;
        case SENSORS_PROP_LUX:
            val->intval = g_lux_light;
            printk(DBGMSK_PRX_G3"[ap3212c][als] get lux property: %d\n", val->intval);
            break;

       case SENSORS_PROP_ATD_STATUS:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = als_sts;
            printk(DBGMSK_PRX_G3"[ap3212c][als] get atd status: %d\n", val->intval);
            break;
        }
        case SENSORS_PROP_ATD_ADC:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = als_adc;
            printk(DBGMSK_PRX_G3"[ap3212c][als] get atd adc: %d\n", val->intval);
            break;
        }
       default:
            printk(DBGMSK_PRX_G0"[ap3212c][als] default\n");
            return -EINVAL;
    }

    printk( DBGMSK_PRX_G3"[ap3212c]: ambientdl_get_property -.\n");

    return 0;
}

static int ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    static bool bFirst = true;
    static bool openfilp = true;

    switch (property) 
    {
        case SENSORS_PROP_SWITCH:
		printk(DBGMSK_PRX_G3"[ap3212c][als] put SENSORS_PROP_SWITCH (%d,%d).\n", 
                    (val->intval), g_HAL_als_switch_on);

		//read calibration value
		if(bFirst) {
			printk(DBGMSK_PRX_G3"[ap3212c][als] put switch 1st read calvalue\n");
			openfilp = read_lightsensor_calibrationvalue(LIGHT_CALIBRATION_FILE_GAIN);
			/*Support old calibration value*/
			if ( openfilp == false) {
				printk("[ap3212c][als] Get calvalue fail, use default settings\n" );
			}
			else {
				printk(DBGMSK_PRX_G3"[ap3212c][als] Get calvalue successful\n" );
				read_lightsensor_shiftvalue(LIGHT_CALIBRATION_FILE_SHIFT);
			}
			printk("[ap3212c] Set calibration and shift: %d , %d\n",g_ap3212c_light_calibration_fval_x1000, g_ap3212c_light_shift_calibration );
			bFirst = false;
		}

		if(val->intval > 0)
			g_HAL_als_switch_on = 1;
		else
			g_HAL_als_switch_on = 0;

              	printk(DBGMSK_PRX_G3"[ap3212c][als] sensor switch, turn on/off ap3212c: %d\n", g_HAL_als_switch_on);
		ap3212c_turn_onoff_als(g_HAL_als_switch_on);

		break;

        case SENSORS_PROP_LO_THRESHOLD:
            break;

        case SENSORS_PROP_HI_THRESHOLD:        
            break;

        case SENSORS_PROP_INTERVAL:
            printk(DBGMSK_PRX_G3"[ap3212c][als] put SENSORS_PROP_INTERVAL. %d\n", val->intval);
            if(val->intval < 100) {
                g_interval = 100;
            }
            else {
                g_interval = val->intval;
            }
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
	    g_ambient_dbg = val->intval;
	    if ( g_ambient_dbg == 1 && g_ap3212c_als_switch_on == 1)
		queue_delayed_work(g_ap3212c_data->debug_wq, &g_ap3212c_data->als_test_work, HZ * DEBUG_TEST_DELAY);
	    else if ( g_ambient_dbg == 2 && g_ap3212c_als_switch_on == 1)
		queue_delayed_work(g_ap3212c_data->debug_wq, &g_ap3212c_data->als_test_work, HZ * DEBUG_TEST_DELAY);
	    if ( g_ambient_dbg == 0 )
		cancel_delayed_work(&g_ap3212c_data->als_test_work);
            printk(DBGMSK_PRX_G3"[ap3212c][als] dbg = %d.\n", g_ambient_dbg);
            break;
        case SENSORS_PROP_CALIBRATION:
            g_ap3212c_light_calibration_fval_x1000 = val->intval;
            printk(DBGMSK_PRX_G3"[ap3212c][als] calibration val x1000= %d\n", g_ap3212c_light_calibration_fval_x1000);
            break;

        default:
            printk(DBGMSK_PRX_G0"[ap3212c][als] put default.\n");
            return -EINVAL;
    }
    return 0;
}

struct proximity_class_dev ap3212c_ambientDev = {
    .id = SENSORS_LIGHT,
    .name = "lsensor",
    .num_properties = ARRAY_SIZE(ambientDev_properties),
    .properties = ambientDev_properties,
    .get_property = ambientDev_get_property,
    .put_property = ambientDev_put_property,
    .fops = &ambientDev_fops
};

static int ap3212c_turn_onoff_als(bool bOn)
{
	printk(DBGMSK_PRX_G3"[ap3212c][als]++Turn onoff ambient sensor\n");

	if (g_ap3212c_als_switch_on != bOn || g_ap3212c_reset==1) {
		if ( bOn == 1 )	{
			printk(DBGMSK_PRX_G3"[ap3212c][als] turn on light sensor ++.\n");

			g_ap3212c_light_first = 1;

			/* Clear ara INT*/
			i2c_smbus_write_byte_data(ap3212c_client, AP3212C_INT_COMMAND, AP3212C_INT_AMASK);

			ap3212c_set_range(ap3212c_client, AP3212C_ALS_RAN);
			ap3212c_set_persist(ap3212c_client, AP3212C_ALS_PERS);
			ap3212c_set_althres(ap3212c_client, 0);
			ap3212c_set_ahthres(ap3212c_client, 0);

			/* Enable ALS */
			if(g_ap3212c_ps_switch_on)
				ap3212c_set_mode(ap3212c_client, 3);
			else
				ap3212c_set_mode(ap3212c_client, 1);
			
			g_ap3212c_als_switch_on = 1;

			/* report lux immediately after als switched on */
			queue_delayed_work(g_ap3212c_data->delay_wq, &g_ap3212c_data->delay_work_light, 20);
		
			printk("[ap3212c][als] turn on light sensor --. lo_thresh = %d, hi_thresh = %d\n", ap3212c_get_althres(ap3212c_client), ap3212c_get_ahthres(ap3212c_client));
		}
		else	{
			printk(DBGMSK_PRX_G3"[ap3212c][als] turn off light sensor ++.\n");

			g_ap3212c_light_first = 1;

			/* Disable ALS & interrupt */
			if(g_ap3212c_ps_switch_on)
				ap3212c_set_mode(ap3212c_client, 2);
			else
				ap3212c_set_mode(ap3212c_client, 0);

			g_ap3212c_als_switch_on = 0;
		
			printk("[ap3212c][als] turn off light sensor --.\n");
		}
	}
	return 0;
}

static void light_test_delayed_work(struct work_struct *work)
{
	if ( g_ambient_dbg == 2 )
		queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_light);
	else if ( g_ambient_dbg == 1 )
		printk(DBGMSK_PRX_G3"[ap3212c][als] ALS_data = %d\n", ap3212c_get_adc_value(ap3212c_client, 0));

	if ( g_ambient_dbg > 0 )
		queue_delayed_work(g_ap3212c_data->debug_wq, &g_ap3212c_data->als_test_work, HZ * DEBUG_TEST_DELAY);
	else
		cancel_delayed_work(&g_ap3212c_data->als_test_work);
}

/*Support ATD light sensor test ++ */
static int proximity_als_turn_on(int bOn)
{ 
    int err = 0;
    int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

    printk(DBGMSK_PRX_G5"[ap3212c] proximity and light sensor test, turn_on:%d\n", bOn);

    err = ap3212c_turn_onoff_als(bOn);
    if(err < 0) {
        printk(DBGMSK_PRX_G0"[ap3212c] (%s): turn %s light sensor error!\n" ,__FUNCTION__, bOn ? "on" : "off");
    }
    else {
        printk(DBGMSK_PRX_G5"[ap3212c] (%s): light sensor %s OK!\n" ,__FUNCTION__, bOn ? "on" : "off");
        status |= 0x1;  //als ok
    }

    err = ap3212c_turn_onoff_proxm(bOn);
    if(err < 0) {
        printk(DBGMSK_PRX_G0"[ap3212c] (%s): turn %s proximity sensor error!\n",__FUNCTION__, bOn ? "on" : "off");
    }
    else {
        printk(DBGMSK_PRX_G5"[ap3212c] (%s): proximity sensor %s OK!\n",__FUNCTION__, bOn ? "on" : "off");
        status |= 0x02; //ps OK
    }

    printk(DBGMSK_PRX_G5"[ap3212c]turn %s, status:0x%x (bitwise)\n", bOn ? "on" : "off", status);
	
    return status;
}

int atd_read_P_L_sensor_adc(int *adc)
{
    int status = 0;
    int lux = 0;
    int idx = 0;

    printk(DBGMSK_PRX_G5"[ap3212c][atd]readadc: trying to turn on lsensor\n");

    status = proximity_als_turn_on(1);

    if(0 == status) {
        printk(DBGMSK_PRX_G2"[ap3212c][atd]readadc: lsensor is not on\n");
        return status;
    }

    *adc = 0;

    for(idx = 0; idx < 5; idx++)
    {
	*adc = ap3212c_get_adc_value(ap3212c_client, 0);
        msleep(100);
    }

    lux = (u32)((*adc) * g_ap3212c_light_calibration_fval_x1000 / 100  + g_ap3212c_light_shift_calibration); 

    printk(DBGMSK_PRX_G5"[ap3212c][atd]readadc steps=%d lux=%d\n", *adc, lux);

    proximity_als_turn_on(0);

    return status;
}

static int atd_write_status_and_adc_2(int *als_sts, int *als_adc, int *ps_sts)
{
    int adc = 0;
    int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

    printk(DBGMSK_PRX_G5"[ap3212c][atd]writests_2: started\n");

    status = atd_read_P_L_sensor_adc(&adc);

    if(status & 0x01) {
        *als_sts = 1;
        *als_adc = adc;
    }
    else {
        *als_sts = 0;
        *als_adc = 0;
    }

    if(status & 0x02) {
        *ps_sts = 1;
    }
    else {
        *ps_sts = 0;
    }

    printk(DBGMSK_PRX_G5"[ap3212c][atd]writests_2: get adc, als_sts:%d, als_adc:%d, ps_sts:%d\n", 
            *als_sts, *als_adc, *ps_sts);

    return status;
}

/* Support ATD light sensor calibration process */
static bool read_lightsensor_calibrationvalue(char *filename)
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G5"[ap3212c] ++read_lsensor_calvalue open\n");

	fp = filp_open(filename, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G5"[ap3212c] read_lsensor_calvalue open (%s) fail\n", filename);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G5"[ap3212c] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[ap3212c] read_lsensor_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_ap3212c_light_calibration_fval_x1000 = ori_val;

	printk("[ap3212c] read_lsensor_calvalue: Ori: %d, Cal: %d\n", ori_val, g_ap3212c_light_calibration_fval_x1000);

	printk(DBGMSK_PRX_G5"[ap3212c] --read_lsensor_calvalue open\n");
	return true;
}

static bool read_lightsensor_shiftvalue(char *filename)
{
    struct file *fp = NULL; 
    loff_t pos_lsts = 0;
    char readstr[8];
    int ori_val = 0, readlen =0;
    mm_segment_t old_fs;

    printk(DBGMSK_PRX_G5"[ap3212c] ++read_lsensor_shift open\n");

    fp = filp_open(filename, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    if(IS_ERR_OR_NULL(fp)) {
        printk(DBGMSK_PRX_G5"[ap3212c] read_lsensor_shift open (%s) fail\n", filename);
        return false;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if(fp->f_op != NULL && fp->f_op->read != NULL) {
        pos_lsts = 0;
        readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
        readstr[readlen] = '\0';

        printk(DBGMSK_PRX_G5"[ap3212c] strlen:%s(%d)\n", readstr, strlen(readstr));
    }
    else {
        printk(DBGMSK_PRX_G0"[ap3212c] read_lsensor_shift, f_op=NULL or op->read=NULL\n");
    }

    set_fs(old_fs);
    filp_close(fp, NULL);

    sscanf(readstr, "%d", &ori_val);

    //limit the calibration value range
    g_ap3212c_light_shift_calibration = ori_val;

    printk("[ap3212c] read_lsensor_shift: Ori: %d, Cal: %d\n", ori_val, g_ap3212c_light_shift_calibration);

    printk(DBGMSK_PRX_G5"[ap3212c] --read_lsensor_calvalue open\n");
    return true;
}

#ifdef ASUS_FACTORY_BUILD
static void write_lightsensor_shiftvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_shift *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_shift, write_shift_work);

	fp = filp_open(LIGHT_CALIBRATION_FILE_SHIFT, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G0"[ap3212c] write_lsensor_shift open (%s) fail\n", LIGHT_CALIBRATION_FILE_SHIFT);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G5"[ap3212c] write_lsensor_shift = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[ap3212c] write_lsensor_shift fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

/*Light Sensor Calibration */
static int ap3212c_show_calibration_200(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_lightsensor_calibrationvalue(LIGHT_CALIBRATION_FILE_GAIN);
	
	printk(DBGMSK_PRX_G5"[ap3212c] Show_gait_calibration: %d\n", g_ap3212c_light_calibration_fval_x1000 );
	
	return sprintf(buf, "%d\n", g_ap3212c_light_calibration_fval_x1000 );
}

static ssize_t ap3212c_store_calibration_200(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (kstrtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_low_calibration_adc = (int)val;

	printk("[ap3212c] Get low calibration adc value : %d\n", a_als_low_calibration_adc );

	return count;
	
}

static DEVICE_ATTR(calibration_200, S_IRWXU | S_IRWXG| S_IRWXO,
		   ap3212c_show_calibration_200, ap3212c_store_calibration_200);


static int ap3212c_show_calibration_1000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_lightsensor_shiftvalue(LIGHT_CALIBRATION_FILE_SHIFT);

	printk(DBGMSK_PRX_G5"[ap3212c] Show_shift_calibration: %d\n", g_ap3212c_light_shift_calibration );
	
	return sprintf(buf, "%d\n", g_ap3212c_light_shift_calibration );
}

static ssize_t ap3212c_store_calibration_1000(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (kstrtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get low brightness adc*/
	a_als_high_calibration_adc = (int)val;

	printk("[ap3212c] Get High calibration adc value : %d\n", a_als_high_calibration_adc );

	/*Calibration operation*/
	g_ap3212c_light_calibration_fval_x1000 = 
		(1000-200)*a_als_calibration_ratio / ( a_als_high_calibration_adc - a_als_low_calibration_adc );

	g_ap3212c_light_shift_calibration = 
		1000 - ( a_als_high_calibration_adc*g_ap3212c_light_calibration_fval_x1000/a_als_calibration_ratio );

	/*Write Calibration value*/
	ap3212c_write_calvalue = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&ap3212c_write_calvalue->write_calvalue_work, write_lightsensor_calibrationvalue_work);

	ap3212c_write_calvalue ->calvalue = g_ap3212c_light_calibration_fval_x1000;

	queue_work(g_ap3212c_data->wq, &ap3212c_write_calvalue->write_calvalue_work);
	
	/*Write shift value*/
	ap3212c_write_shift = kmalloc(sizeof(struct write_shift), GFP_KERNEL);

	INIT_WORK(&ap3212c_write_shift->write_shift_work, write_lightsensor_shiftvalue_work);

	ap3212c_write_shift ->calvalue = g_ap3212c_light_shift_calibration;

	queue_work(g_ap3212c_data->wq, &ap3212c_write_shift->write_shift_work);

	return count;
}

static DEVICE_ATTR(calibration_1000, S_IRWXU | S_IRWXG | S_IRWXO,
		   ap3212c_show_calibration_1000, ap3212c_store_calibration_1000);

static void write_lightsensor_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_calvalue *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_calvalue, write_calvalue_work);

	fp = filp_open(LIGHT_CALIBRATION_FILE_GAIN, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G0"[ap3212c] write_lsensor_calvalue open (%s) fail\n", LIGHT_CALIBRATION_FILE_GAIN);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G5"[ap3212c] write_lsensor_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[ap3212c] write_lsensor_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}
#endif

/* Support ATD proximity sensor calibration function */
static bool read_prox_hi_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G3"[ap3212c] ++read_psensor_hi_calvalue open\n");

	fp = filp_open(PROXIMITY_CALIBRATION_FILE_HI, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	//fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[ap3212c] read_psensor_hi_calvalue open (%s) fail\n", PROXIMITY_CALIBRATION_FILE_HI);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[ap3212c] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[ap3212c] read_psensor_hi_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_ps_threshold_hi = ori_val;

	printk("[ap3212c] read_psensor_hi_calvalues: Ori: %d, Cal: %d\n", ori_val, g_ps_threshold_hi);

	printk("[ap3212c] --read_psensor_hi_calvalue open\n");
	return true;
}

#ifdef ASUS_FACTORY_BUILD
static void write_prox_hi_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_prox_hi *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_prox_hi, write_prox_hi_work);

	fp = filp_open(PROXIMITY_CALIBRATION_FILE_HI, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G0"[ap3212c] write_psensor_hi_calvalue open (%s) fail\n", PROXIMITY_CALIBRATION_FILE_HI);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[ap3212c] write_psensor_hi_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[ap3212c] write_psensor_hi_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}

static int ap3212c_show_calibration_prox_hi(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_prox_hi_calibrationvalue();
	
	printk(DBGMSK_PRX_G2"[ap3212c] Show_prox_hi_calibration: %d\n", g_ps_threshold_hi);
	
	return sprintf(buf, "%d\n", g_ps_threshold_hi);
}

static int ap3212c_store_calibration_prox_hi(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get hi threshold adc*/
	a_ps_hi_calibration_adc = (int)val;

	printk("[ap3212c] Get calibration_prox_hi value : %d\n", a_ps_hi_calibration_adc );

	/*Write Calibration value*/
	ap3212c_write_prox_hi = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&ap3212c_write_prox_hi->write_prox_hi_work, write_prox_hi_calibrationvalue_work);

	ap3212c_write_prox_hi->calvalue = a_ps_hi_calibration_adc;

	queue_work(g_ap3212c_data->wq, &ap3212c_write_prox_hi->write_prox_hi_work);

	return a_ps_hi_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_hi, S_IRWXU | S_IRWXG| S_IRWXO,
		   ap3212c_show_calibration_prox_hi, ap3212c_store_calibration_prox_hi);
#endif

static bool read_prox_lo_calibrationvalue()
{
	struct file *fp = NULL; 
	loff_t pos_lsts = 0;
	char readstr[8];
	int ori_val = 0, readlen =0;
	mm_segment_t old_fs;
	
	printk(DBGMSK_PRX_G3"[ap3212c] ++read_psensor_low_calvalue open\n");

	fp = filp_open(PROXIMITY_CALIBRATION_FILE_LO, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	//fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G3"[ap3212c] read_psensor_low_calvalue open (%s) fail\n", PROXIMITY_CALIBRATION_FILE_LO);
		return false;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	if(fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
		readstr[readlen] = '\0';

		printk(DBGMSK_PRX_G3"[ap3212c] strlen:%s(%d)\n", readstr, strlen(readstr));
	}
	else
		printk(DBGMSK_PRX_G0"[ap3212c] read_psensor_low_calvalue, f_op=NULL or op->read=NULL\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(readstr, "%d", &ori_val);

	//limit the calibration value range
	g_ps_threshold_lo = ori_val;

	printk("[ap3212c] read_psensor_low_calvalue: Ori: %d, Cal: %d\n", ori_val, g_ps_threshold_lo);
	printk("[ap3212c] --read_psensor_low_calvalue open\n");
	return true;
}

#ifdef ASUS_FACTORY_BUILD
static void write_prox_lo_calibrationvalue_work(struct work_struct *work)
{
	struct file *fp = NULL; 
	struct write_prox_lo *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[8];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	this = container_of(work, struct write_prox_lo, write_prox_lo_work);

	fp = filp_open(PROXIMITY_CALIBRATION_FILE_LO, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR_OR_NULL(fp)) {
		printk(DBGMSK_PRX_G0"[ap3212c] write_psensor_low_calvalue open (%s) fail\n", PROXIMITY_CALIBRATION_FILE_LO);
		return;
	}

	sprintf(writestr, "%d", this->calvalue);

	printk(DBGMSK_PRX_G3"[ap3212c] write_psensor_low_calvalue = %d[%s(%d)]\n", 
		this->calvalue, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk(DBGMSK_PRX_G0"[ap3212c] write_psensor_low_calvalue fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}


static int ap3212c_show_calibration_prox_lo(struct device *dev, struct device_attribute *attr, char *buf)
{	
	read_prox_lo_calibrationvalue();

	printk(DBGMSK_PRX_G2"[ap3212c] Show_prox_lo_calibration: %d\n", g_ps_threshold_lo);
	
	return sprintf(buf, "%d\n", g_ps_threshold_lo);
}

static int ap3212c_store_calibration_prox_lo(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	
	if ( (strict_strtoul(buf, 10, &val) < 0) )
		return -EINVAL;
	
	/*Get hi threshold adc*/
	a_ps_lo_calibration_adc = (int)val;

	printk("[ap3212c] Get calibration_prox_hi value : %d\n", a_ps_lo_calibration_adc );

	/*Write Calibration value*/
	ap3212c_write_prox_lo = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);

	INIT_WORK(&ap3212c_write_prox_lo->write_prox_lo_work, write_prox_lo_calibrationvalue_work);

	ap3212c_write_prox_lo->calvalue = a_ps_lo_calibration_adc;

	queue_work(g_ap3212c_data->wq, &ap3212c_write_prox_lo->write_prox_lo_work);

	return a_ps_lo_calibration_adc;
}

static DEVICE_ATTR(calibration_prox_lo, S_IRWXU | S_IRWXG | S_IRWXO,
		   ap3212c_show_calibration_prox_lo, ap3212c_store_calibration_prox_lo);
#endif

static struct attribute *ap3212c_attributes[] = {
	&dev_attr_adc.attr,
	&dev_attr_object.attr,
	&dev_attr_proxm.attr,
#if defined(ASUS_USERDEBUG_BUILD) || defined(ASUS_ENG_BUILD)
	&dev_attr_ps_report.attr,
#endif
#ifdef ASUS_FACTORY_BUILD
	&dev_attr_calibration_200.attr,
	&dev_attr_calibration_1000.attr,
	&dev_attr_calibration_prox_hi.attr,
	&dev_attr_calibration_prox_lo.attr,
#endif
	NULL
};

static const struct attribute_group ap3212c_attr_group = {
	.name = "ap3212c",
	.attrs = ap3212c_attributes,
};

static int ap3212c_init_client(struct i2c_client *client)
{
	int i;

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < AP3212C_NUM_CACHABLE_REGS; i++) {
		int v = i2c_smbus_read_byte_data(ap3212c_client, ap3212c_reg[i]);
		if (v < 0)
			return -ENODEV;

		g_ap3212c_data->reg_cache[i] = v;
	}

	/* set defaults */
	ap3212c_set_mode(ap3212c_client, 0);

	/* set int clear manner to "write to clear" */
	i2c_smbus_write_byte_data(ap3212c_client, 0x02, 1);

	mdelay(15);

	return 0;
}

static void ap3212c_work_func_proximity(struct work_struct *work)
{
	int Pval;

	mutex_lock(&g_ap3212c_data->lock);
	
	Pval = ap3212c_get_px_value(ap3212c_client,1);
	printk("[ap3212c] work_func_proximity: PS dtat = [%d]\n", Pval);

	if (report_proxm || (g_ASUS_hwID != ME175KG_SR1))
		ps_adc_report_event(Pval);
	
	mutex_unlock(&g_ap3212c_data->lock);
	
	/* Clear INT */
	i2c_smbus_write_byte_data(ap3212c_client, AP3212C_INT_COMMAND, AP3212C_INT_PMASK);
}

static int get_adc_calibrated_lux_from_ap3212c(int adc)
{
	int i = 0;
	int lux = 0;

	lux = (u32)(adc * g_ap3212c_light_calibration_fval_x1000 / a_als_calibration_ratio  + g_ap3212c_light_shift_calibration);     //apply calibration value ( Because the panel level, calibration number probably over 10000)

	/*Get Lux*/
	for(i=1;i<g_max_light_level;i++) {
		if( lux < g_ap3212c_light_map[i] ) {
			g_lux_light = g_ap3212c_light_map[ i -1 ];
			break;
		}
		else if( lux > g_ap3212c_light_map[g_max_light_level - 1] )	{
			g_lux_light = g_ap3212c_light_map[ g_max_light_level -1 ];
			break;
		}
	}
	if ( lux > g_ap3212c_light_map[ g_max_light_level -1 ])
		g_lux_light = g_ap3212c_light_map[ g_max_light_level -1 ];
	
	printk(DBGMSK_PRX_G3"[ap3212c][als] adc=%d, k_adc=%d, lux=%d, last=%d\n", adc, lux, g_lux_light, g_last_ap3212c_light);

    return lux;
}

static void set_als_thd(int adc)
{
	g_als_threshold_hi = adc*(100+AP3212C_THRES_OVERHEAD)/100;
	g_als_threshold_lo = adc*(100-AP3212C_THRES_OVERHEAD)/100;
	printk(DBGMSK_PRX_G3"[ap3212c][als] Setting threshold: adc=%d, g_als_threshold_hi=%d, g_als_threshold_lo=%d\n", adc, g_als_threshold_hi, g_als_threshold_lo);
	
	/* Set als threshold*/
	ap3212c_set_ahthres(ap3212c_client, g_als_threshold_hi);
	ap3212c_set_althres(ap3212c_client, g_als_threshold_lo);

	return;
}

static void ap3212c_work_func_light(struct work_struct *work)
{
	int adc, lux;

	mutex_lock(&g_ap3212c_data->lock);
	
	adc = ap3212c_get_adc_value(ap3212c_client,1);
	lux = get_adc_calibrated_lux_from_ap3212c(adc);
	set_als_thd(adc);

	printk("[ap3212c] work_func_light: adc=%d, lux=%d\n", adc, lux);

	if(g_lux_light != g_last_ap3212c_light || g_ap3212c_light_first) {
		g_last_ap3212c_light = g_lux_light;
		if(g_ASUS_hw_sku_id!=1)						//Do not report als event for ME175KG HW_SKU_ID_1
			als_lux_report_event( g_lux_light);
		g_ap3212c_light_first=0;
	}

	/* Clear INT */
	i2c_smbus_write_byte_data(ap3212c_client, AP3212C_INT_COMMAND, AP3212C_INT_AMASK);

	mutex_unlock(&g_ap3212c_data->lock);

	if(g_ap3212c_ps_switch_on==1) {
		if (g_ambient_suspended==1) {
			printk(DBGMSK_PRX_G3"[ap3212c][isr] setting g_ap3212c_earlysuspend_int = %d\n", g_ap3212c_earlysuspend_int);
			g_ap3212c_earlysuspend_int = 1;
		}
		if (g_ap3212c_reset != 1)
			queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_proximity);
	}
	
	if(g_proxm_state == 1) 
		wake_unlock(&g_ap3212c_data->prx_wake_lock);	
}

static enum hrtimer_restart ap3212c_light_timer_func(struct hrtimer *timer)
{
	queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_light);
	hrtimer_forward_now(&g_ap3212c_data->light_timer, g_ap3212c_data->light_poll_delay);
	return HRTIMER_RESTART;
}

static enum hrtimer_restart ap3212c_pxy_timer_func(struct hrtimer *timer)
{
	queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_proximity);
	hrtimer_forward_now(&g_ap3212c_data->proximity_timer, g_ap3212c_data->proximity_poll_delay);
	return HRTIMER_RESTART;
}

static void ap3212c_timer_init(struct ap3212c_data *data)
{
	if (ps_polling)
	{
		/* proximity hrtimer settings. */
		hrtimer_init(&g_ap3212c_data->proximity_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_ap3212c_data->proximity_poll_delay = ns_to_ktime(50 * NSEC_PER_MSEC);
		g_ap3212c_data->proximity_timer.function = ap3212c_pxy_timer_func;
	}

	if (als_polling)
	{
		/* light hrtimer settings. */
		hrtimer_init(&g_ap3212c_data->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_ap3212c_data->light_poll_delay = ns_to_ktime(100 * NSEC_PER_MSEC);
		g_ap3212c_data->light_timer.function = ap3212c_light_timer_func;
	}
}

static int ap3212c_input_init(struct ap3212c_data *data)
{
    struct input_dev *input_dev;
    int ret;

    /* allocate proximity input_device */
    input_dev = input_allocate_device();
    if (!input_dev) {
        printk(DBGMSK_PRX_G0"could not allocate input device\n");
        goto err_pxy_all;
    }
    input_dev->name = "ASUS Proximitysensor";
    input_dev->id.bustype = BUS_I2C;
    input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(ABS_DISTANCE, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
    input_set_drvdata(input_dev, g_ap3212c_data);

    ret = input_register_device(input_dev);
    if (ret < 0) {
        printk(DBGMSK_PRX_G0"could not register input device\n");
        goto err_pxy_reg;
    }
    g_ap3212c_data->proximity_input_dev = input_dev;
    ret = proximity_dev_register(&ap3212c_proxmDev);
    if (ret)
	printk(DBGMSK_PRX_G0"[ap3212c] ambientdl create sysfile fail.\n");

    /* allocate light input_device */
    input_dev = input_allocate_device();
    if (!input_dev) {
        printk(DBGMSK_PRX_G0"could not allocate input device\n");
        goto err_light_all;
    }	
    input_dev->name = "ASUS Lightsensor";
    input_dev->id.bustype = BUS_I2C;
    input_set_capability(input_dev, EV_ABS, ABS_MISC);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(ABS_MISC, input_dev->absbit);
    input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);
    input_set_drvdata(input_dev, g_ap3212c_data);

    ret = input_register_device(input_dev);
    if (ret < 0) {
        printk(DBGMSK_PRX_G0"could not register input device\n");
        goto err_light_reg;
    }
    g_ap3212c_data->light_input_dev = input_dev;
    ret = als_lux_report_event_register(g_ap3212c_data->light_input_dev );

    ret = sysfs_create_group(&ap3212c_client->dev.kobj, &ap3212c_attr_group);
    if (ret) {
        printk(DBGMSK_PRX_G0"could not create sysfs group\n");
        goto err_light_sys;
    }	
    ret = proximity_dev_register(&ap3212c_ambientDev);
    if (ret)
	printk(DBGMSK_PRX_G0"[ap3212c] ambientdl create sysfile fail.\n");

    return 0;

err_light_sys:
    input_unregister_device(g_ap3212c_data->light_input_dev);
err_light_reg:
    input_free_device(input_dev);
err_pxy_reg:
    input_free_device(input_dev);
err_pxy_all:
err_light_all:
    return (-1);   
}

static void ap3212c_input_fini(struct ap3212c_data *data)
{
    struct input_dev *dev = g_ap3212c_data->light_input_dev;

    input_unregister_device(dev);
    input_free_device(dev);
    
    dev = g_ap3212c_data->proximity_input_dev;
    input_unregister_device(dev);
    input_free_device(dev);
}

static int ap3212c_gpio_init(void)
{
	int rc = -EINVAL;
	printk(DBGMSK_PRX_G2"[ap3212c][board]ap3212c_gpio_init++\n");

	/* configure Phone Lightsensor interrupt gpio */
	rc = gpio_request(AP3212C_GPIO_PROXIMITY_INT, "ap3212c-irq");
	if (rc) {
		printk(DBGMSK_PRX_G0"%s: unable to request gpio %d (ap3212c-irq)\n",__func__, AP3212C_GPIO_PROXIMITY_INT);
		goto err;
	}

	rc = gpio_direction_input(AP3212C_GPIO_PROXIMITY_INT);
	if (rc < 0) {
		printk(DBGMSK_PRX_G0"%s: unable to set the direction of gpio %d\n",__func__, AP3212C_GPIO_PROXIMITY_INT);
		goto err;
	}

	printk(DBGMSK_PRX_G2"[ap3212c][board]ap3212c_gpio_init--\n");
	return 0;
err:
	gpio_free(AP3212C_GPIO_PROXIMITY_INT);
	return rc;
}


/*
 * I2C layer
 */

static irqreturn_t ap3212c_irq(int irq, void *data_)
{
	printk(DBGMSK_PRX_G2"[ap3212c][isr] interrupt handler ++\n");	

	queue_work(g_ap3212c_data->wq, &g_ap3212c_data->ISR_work);
	if (g_proxm_state == 1)	
		wake_lock_timeout(&g_ap3212c_data->prx_wake_lock, 1 * HZ);

	printk(DBGMSK_PRX_G2"[ap3212c][isr] interrupt handler --\n");
	return IRQ_HANDLED;
}	
	
static void ap3212c_interrupt_handler(struct work_struct *work)
{
	u8 int_stat;
 	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_irq: +++\n");

	if ((g_ap3212c_ps_switch_on == 1) ||(g_ap3212c_als_switch_on == 1) || (g_ambient_suspended == 1)) {
		int_stat = ap3212c_get_intstat(ap3212c_client);
 		printk(DBGMSK_PRX_G2"[ap3212c] int_stat (%d)\n", int_stat);

		// ALS int
		if (int_stat & AP3212C_INT_AMASK) {
 			printk(DBGMSK_PRX_G3"[ap3212c] ap3212c_irq: als int triggered\n");
			queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_light);
		}
	
		// PX int
		if (int_stat & AP3212C_INT_PMASK) {
 			printk(DBGMSK_PRX_G4"[ap3212c] ap3212c_irq: ps int triggered\n");
			if (g_ambient_suspended==1) {
				printk(DBGMSK_PRX_G2"[ap3212c][isr] setting g_ap3212c_earlysuspend_int = %d\n", g_ap3212c_earlysuspend_int);
				g_ap3212c_earlysuspend_int = 1;
			}
			queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_proximity);
		}
	}

 	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_irq: ---\n");

	return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ap3212c_early_suspend(struct early_suspend *h)
{
	return;
}

static void ap3212c_late_resume(struct early_suspend *h)
{
	printk("[ap3212c] ++ap3212c_late_resume, g_ap3212c_on_fail:%d\n", g_ap3212c_on_fail);

	if(g_ap3212c_on_fail==1) {
		ap3212c_turn_onoff_als(1);
		printk(DBGMSK_PRX_G2"[ap3212c][als] late_resume: apply ALS interrupt mode\n");
	}

	printk(DBGMSK_PRX_G2"[ap3212c]--ap3212c_late_resume\n");
}
#elif defined(CONFIG_FB)
static void ap3212c_early_suspend_fb(void)
{
	return;
}

static void ap3212c_late_resume_fb(void)
{
	printk("[ap3212c] ++ap3212c_late_resume_fb, g_ap3212c_on_fail:%d\n", g_ap3212c_on_fail);

	if(g_ap3212c_on_fail==1) {
		ap3212c_turn_onoff_als(1);
		printk(DBGMSK_PRX_G2"[ap3212c][als] late_resume: apply ALS interrupt mode\n");
	}

	printk(DBGMSK_PRX_G2"[ap3212c]--ap3212c_late_resume_fb\n");
}

static int ap3212c_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	static int blank_old = 0;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if (blank_old == FB_BLANK_POWERDOWN) {
				blank_old = FB_BLANK_UNBLANK;
				ap3212c_late_resume_fb();
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			if (blank_old == 0 || blank_old == FB_BLANK_UNBLANK) {
				blank_old = FB_BLANK_POWERDOWN;
				ap3212c_early_suspend_fb();
			}
		}
	}

	return 0;
}
#endif

static int ap3212c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("[ap3212c] ++ap3212c_suspend, psensor:%d, als:%d\n", g_ap3212c_ps_switch_on, g_ap3212c_als_switch_on);

	enable_irq_wake(g_ap3212c_data->irq);

	if(g_ap3212c_als_switch_on) {
		//In case upper layer doesn't switch off ambient before early_suspend.
		printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_suspend, turn off ambient\n");
		g_ambient_suspended = 1;
		ap3212c_turn_onoff_als(0);
	}
	printk(DBGMSK_PRX_G2"[ap3212c] --ap3212c_suspend\n");
	return 0 ;
}

static int ap3212c_resume(struct i2c_client *client)
{
	printk("[ap3212c]++ap3212c_resume, gpio %d : %d\n",AP3212C_GPIO_PROXIMITY_INT,gpio_get_value(AP3212C_GPIO_PROXIMITY_INT) );

	if (g_proxm_state == 1) {
		queue_work(g_ap3212c_data->wq, &g_ap3212c_data->work_proximity);		//obtain current proximity state
		wake_lock_timeout(&g_ap3212c_data->prx_wake_lock, 1 * HZ);
	}

	printk(DBGMSK_PRX_G2"[ap3212c][als] resume: g_ambient_suspended = %d first_light=%d\n",g_ambient_suspended, g_ap3212c_light_first);
	if(g_ambient_suspended==1) {
		ap3212c_turn_onoff_als(1);
		g_ambient_suspended = 0;
		printk(DBGMSK_PRX_G2"[ap3212c][als] resume: apply ALS interrupt mode\n");
	}

	disable_irq_wake(g_ap3212c_data->irq);

	g_ap3212c_earlysuspend_int = 0;

	printk(DBGMSK_PRX_G2"[ap3212c]--ap3212c_resume\n");
	return 0 ;
}

static const struct i2c_device_id ap3212c_id[] = {
	{ "ap3212c", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ap3212c_id);

static struct of_device_id ap3212c_match_table[] = {
	{ .compatible = "liteon,ap3212c",},
	{},
};

static struct i2c_driver ap3212c_driver = {
	.driver = {
		.name	= AP3212C_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ap3212c_match_table,
	},
	.probe = ap3212c_probe,
	.resume = ap3212c_resume,
	.suspend = ap3212c_suspend,
	.remove = ap3212c_remove,
	.id_table = ap3212c_id,
};

static int ap3212c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int err = 0;
	int gpio = 0;

	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_probe +.\n");

	/*if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;
*/
	g_ap3212c_data = kzalloc(sizeof(struct ap3212c_data), GFP_KERNEL);
	if (!g_ap3212c_data)
		return -ENOMEM;

	g_ap3212c_data->client = client;
	ap3212c_client= client;
	i2c_set_clientdata(ap3212c_client, g_ap3212c_data);
	ap3212c_client->driver = &ap3212c_driver;
	ap3212c_client->flags = 1;
	strlcpy(ap3212c_client->name, AP3212C_DRV_NAME, I2C_NAME_SIZE);
	mutex_init(&g_ap3212c_data->lock);
	mutex_init(&g_ap3212c_data->reg_lock);
	wake_lock_init(&g_ap3212c_data->prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");

	ap3212c_gpio_init();

	ap3212c_timer_init(g_ap3212c_data);

	/* initialize the ap3212c chip */
	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_init_client+++\n");
	err = ap3212c_init_client(ap3212c_client);
	if (err)
		goto exit_kfree;

	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_input_init+++\n");
	err = ap3212c_input_init(g_ap3212c_data);
	if (err)
		goto exit_kfree;
	
	if (!als_polling || !ps_polling)
	{
		gpio = of_get_named_gpio_flags(ap3212c_client->dev.of_node, "ap3212c,irq-gpio",0,NULL);
		g_ap3212c_data->irq =gpio_to_irq(gpio);
		if( g_ap3212c_data->irq < 0 )
			printk(DBGMSK_PRX_G0"[ap3212c] gpio_to_irq fail (g_ap3212c_data->irq)irq=%d.\n", g_ap3212c_data->irq);
		else	{
			printk(DBGMSK_PRX_G2"[ap3212c] (g_ap3212c_data->irq) irq=%d gpio=%d.\n", g_ap3212c_data->irq, gpio);
		
			err = request_irq(g_ap3212c_data->irq, ap3212c_irq,
				IRQF_TRIGGER_FALLING  | IRQF_TRIGGER_FALLING,
				"ap3212c_INT", &ap3212c_client->dev );
                               
			if (err) {
				printk(DBGMSK_PRX_G0"[ap3212c] ret: %d, could not get IRQ %d\n",err,g_ap3212c_data->irq);
				goto exit_irq;
			}
			else	{
				printk(DBGMSK_PRX_G2"[ap3212c] (g_ap3212c_data->irq ) request_irq ok.\n");
				disable_irq(g_ap3212c_data->irq);
				msleep(5);
				enable_irq(g_ap3212c_data->irq);
			}
		}
	}

	g_ap3212c_data->delay_wq = create_singlethread_workqueue("ap3212c_delay_wq");
	g_ap3212c_data->wq = create_singlethread_workqueue("ap3212c_wq");
	g_ap3212c_data->debug_wq = create_workqueue("ap3212c_debug_test_wq");

	INIT_WORK(&g_ap3212c_data->ISR_work, ap3212c_interrupt_handler);
	INIT_WORK(&g_ap3212c_data->work_proximity, ap3212c_work_func_proximity);
	INIT_WORK(&g_ap3212c_data->work_light, ap3212c_work_func_light);
	INIT_DELAYED_WORK(&g_ap3212c_data->delay_work_proximity, ap3212c_work_func_proximity);
	INIT_DELAYED_WORK(&g_ap3212c_data->delay_work_light, ap3212c_work_func_light);
	INIT_DELAYED_WORK(&g_ap3212c_data->ps_test_work, proximity_test_delayed_work);
	INIT_DELAYED_WORK(&g_ap3212c_data->als_test_work, light_test_delayed_work);

	#ifdef CONFIG_I2C_STRESS_TEST
		printk("LSenor add test case+\n");
		i2c_add_test_case(client, "LightSensorTest",ARRAY_AND_SIZE(gLSensorTestCaseInfo));
		printk("LSensor add test case-\n");
	#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	printk(DBGMSK_PRX_G2"[ap3212c] Register early suspend\n");
	g_ap3212c_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	g_ap3212c_data->early_suspend.suspend = ap3212c_early_suspend;
	g_ap3212c_data->early_suspend.resume = ap3212c_late_resume;
	register_early_suspend(&g_ap3212c_data->early_suspend);
#elif defined(CONFIG_FB)
	printk(DBGMSK_PRX_G2"[ap3212c] Register fb callback early suspend\n");
	g_ap3212c_data->fb_notif.notifier_call = ap3212c_fb_notifier_callback;
	g_ap3212c_data->fb_register_fail = fb_register_client(&g_ap3212c_data->fb_notif);
	if (g_ap3212c_data->fb_register_fail)
		printk(DBGMSK_PRX_G0"[ap3212c] Unable to register fb_notifier: %d\n", g_ap3212c_data->fb_register_fail);
#endif

	dev_info(&ap3212c_client->dev, "[ap3212c] Driver version: %d enabled\n", DRIVER_VERSION);
	
	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_probe -.\n");
	return 0;

exit_irq:
	ap3212c_input_fini(g_ap3212c_data);
	
exit_kfree:
	wake_lock_destroy(&g_ap3212c_data->prx_wake_lock);
	mutex_destroy(&g_ap3212c_data->lock);
	mutex_destroy(&g_ap3212c_data->reg_lock);
	kfree(g_ap3212c_data);
	return err;
}

static int ap3212c_remove(struct i2c_client *client)
{	
	if (!als_polling || !ps_polling)
		free_irq(g_ap3212c_data->irq, g_ap3212c_data);
	else {
		if (g_ap3212c_data->reg_cache[0] & PS_ACTIVE) 
		{
			hrtimer_cancel(&g_ap3212c_data->proximity_timer);
			cancel_work_sync(&g_ap3212c_data->work_proximity);
			cancel_delayed_work_sync(&g_ap3212c_data->delay_work_proximity);
		}
		if (g_ap3212c_data->reg_cache[0] & ALS_ACTIVE) {
			hrtimer_cancel(&g_ap3212c_data->light_timer);
			cancel_work_sync(&g_ap3212c_data->work_light);
			cancel_delayed_work_sync(&g_ap3212c_data->delay_work_light);
		}
	}
	sysfs_remove_group(&g_ap3212c_data->light_input_dev->dev.kobj, &ap3212c_attr_group);
	input_unregister_device(g_ap3212c_data->light_input_dev);
	input_unregister_device(g_ap3212c_data->proximity_input_dev);
	destroy_workqueue(g_ap3212c_data->wq);
	destroy_workqueue(g_ap3212c_data->delay_wq);
	destroy_workqueue(g_ap3212c_data->debug_wq);
	mutex_destroy(&g_ap3212c_data->lock);
	mutex_destroy(&g_ap3212c_data->reg_lock);
	wake_lock_destroy(&g_ap3212c_data->prx_wake_lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&g_ap3212c_data->early_suspend);
#elif defined(CONFIG_FB)
	if(!g_ap3212c_data->fb_register_fail)
		fb_unregister_client(&g_ap3212c_data->fb_notif);
#endif
	kfree(g_ap3212c_data);
	
	return 0;
}

static int ap3212c_platform_suspend_noirq( struct device *dev )
{
	printk("[ap3212c][suspend_noirq] g_earlysuspend_int = %d\n",  g_ap3212c_earlysuspend_int);
	if(g_ap3212c_earlysuspend_int == 1) {
		g_ap3212c_earlysuspend_int = 0;
		return -EBUSY;
	}

        return 0;
}

static int ap3212c_platform_resume_noirq( struct device *dev )
{
        return 0;
}

static const struct dev_pm_ops ap3212c_pm_ops = {
        .suspend_noirq  = ap3212c_platform_suspend_noirq,
	.resume_noirq = ap3212c_platform_resume_noirq,
};

static int ap3212c_platform_probe( struct platform_device *pdev )
{
	int err = 0;
	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_platform_probe ++ \n");

	err = i2c_add_driver(&ap3212c_driver);
	if ( err != 0 )
		printk(DBGMSK_PRX_G0"[ap3212c] i2c_add_driver fail:  %d\n",err);

	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_platform_probe -- \n");
	return 0;
}

static int __devexit ap3212c_platform_remove( struct platform_device *pdev )
{
	i2c_del_driver(&ap3212c_driver);

	return 0;
}

static struct of_device_id ap3212c_of_match[] = {
	{ .compatible = "ap3212c", },
	{ },
};

static struct platform_driver  ap3212c_platform_driver = {
	.probe 	= ap3212c_platform_probe,
	.remove   = ap3212c_platform_remove,

	.driver = {
		.name = "ap3212c",
		.owner = THIS_MODULE,
		.pm = &ap3212c_pm_ops,
		.of_match_table = ap3212c_of_match,
	},
};

static int __init ap3212c_init(void)
{
	int err = 0;

	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_platform_init +.\n");
	err = platform_driver_register(&ap3212c_platform_driver);
	if ( err != 0 )
		printk(DBGMSK_PRX_G0"[ap3212c] platform_driver_register fail, Error : %d\n",err);
	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_platform_init -.\n");

	return err;
}

static void __exit ap3212c_exit(void)
{
	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_platform_exit +.\n");
	platform_driver_unregister(&ap3212c_platform_driver);
	printk(DBGMSK_PRX_G2"[ap3212c] ap3212c_platform_exit -.\n");
}

MODULE_AUTHOR("Maggie Lee @ ASUS BSP");
MODULE_DESCRIPTION("AP3212C driver.");
MODULE_LICENSE("GPL v2");

module_init(ap3212c_init);
module_exit(ap3212c_exit);
