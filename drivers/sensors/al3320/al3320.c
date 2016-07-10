/* al3320.c - AL3320A ambient light sensor driver
 *
 * Copyright (C) 2012 ASUSTek Inc.
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
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/switch.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
static struct notifier_block al3320_fb_notif;
static int al3320_fb_register_fail = 0;
#endif

#if 0
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif

#include "linux/cm36283.h"
#include "linux/proximity_class.h"
#include "linux/al3320.h"

#define AL3320_DRV_NAME		"al3320_light_sensor"
#define DRIVER_VERSION		"1.0"

#define threshold_overhead		10		//define threshold range as 10 %

#define P01_EVENT_NOTIFY_LIGHTSENSOR_NO_ERROR (0)
#define P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR (-1)

#define AL3320_NUM_CACHABLE_REGS		15

/* AL3320 Register Definitions*/
#define AL3320_MODE_COMMAND			0x00
#define AL3320_MODE_MASK				0x01
#define AL3320_MODE_SHIFT				(0)

#define AL3320_INT_COMMAND				0x01
#define AL3320_INT_MASK					0x01
#define AL3320_INT_SHIFT					(0)

#define AL3320_INT_ENABLE				0x02
#define AL3320_INT_ENABLE_MASK			0x08
#define AL3320_INT_ENABLE_SHIFT			(3)
#define AL3320_SUS_ENABLE_MASK			0x04
#define AL3320_SUS_ENABLE_SHIFT			(2)

#define AL3320_WAITING_TIME				0x06
#define AL3320_WAITING_MASK			0xff
#define AL3320_WAITING_SHIFT			(0)

#define AL3320_RAN_COMMAND				0x07
#define AL3320_RAN_MASK					0x06
#define AL3320_RAN_SHIFT				(1)
#define AL3320_EXGAIN_MASK				0x01
#define AL3320_EXGAIN_SHIFT				(0)

#define AL3320_ALS_PERSIST				0x08
#define AL3320_PERSIST_MASK				0x3f
#define AL3320_PERSIST_SHIFT				(0)

#define AL3320_ALS_MEANTIME				0x09
#define AL3320_MEANTIME_MASK			0x0f
#define AL3320_MEANTIME_SHIFT			(0)

#define AL3320_ALS_ADUMMY				0x0a
#define AL3320_ADUMMY_MASK				0xff
#define AL3320_ADUMMY_SHIFT			(0)

#define AL3320_ADC_LSB					0x22
#define AL3320_ADC_MSB					0x23

#define AL3320_ALS_LTHL					0x30
#define AL3320_ALS_LTHL_MASK			0xff
#define AL3320_ALS_LTHL_SHIFT			(0)

#define AL3320_ALS_LTHH					0x31
#define AL3320_ALS_LTHH_MASK			0xff
#define AL3320_ALS_LTHH_SHIFT			(0)

#define AL3320_ALS_HTHL					0x32
#define AL3320_ALS_HTHL_MASK			0xff
#define AL3320_ALS_HTHL_SHIFT			(0)

#define AL3320_ALS_HTHH					0x33
#define AL3320_ALS_HTHH_MASK			0xff
#define AL3320_ALS_HTHH_SHIFT			(0)


#define ALS_RAN_0			0x00
#define ALS_RAN_1			0x01
#define ALS_RAN_2			0x02
#define ALS_RAN_3			0x03

#define ALS_ADUMMY_0		(72)
#define ALS_ADUMMY_1		(62)
#define ALS_ADUMMY_2		(22)
#define ALS_ADUMMY_3		(0)

#define ALS_MEAN_0			(12)
#define ALS_MEAN_1			(12)
#define ALS_MEAN_2			(12)
#define ALS_MEAN_3			(8)

/* AL3320 User Settings */
#define AL3320_POW_DOWN   	0x00			//ALS off
#define AL3320_POW_UP  		0x01			//ALS on
#define AL3320_RESET    		0x04			//software reset: clears all registers
#define AL3320_INT			0x01			//interrupt function: 0=polling, 1=interrupt
#define AL3320_SUS			0x00			//suspend function: 0=disable, 1=enable
#define AL3320_WAITING		0x00			//default value: 0 waiting time between each hw sensing
#define AL3320_RAN			0x02			//low gain: 0=0.65K, 1=2.08K, 2=8.32K, 3=33.28K; high gain: 0=1.95K, 1=6.25K, 2=25K, 3=100K
#define AL3320_GAIN			0x01			//0=low gain, 1=extended 3 times gain
#define AL3320_PERS			0x00			//default value: 1 consecutive conversion time out of range

#define ENABLE				0X01
#define DISABLE				0X00

// AL3320 register
static u8 al3320_reg[AL3320_NUM_CACHABLE_REGS] = {0x00,0x01,0x02,0x06,0x07,0x08,0x09,0x0a,0x22,0x23,0x30,0x31,0x32,0x33,0x34};

// AL3320 range
static long al3320_range[8] = {33280,8320,2080,650,100000,25000,6250,1950};

extern int g_HAL_als_switch_on;

struct al3320_data {
	struct i2c_client *client;
	struct mutex lock;
        struct input_dev   *input_dev;
	u8 reg_cache[AL3320_NUM_CACHABLE_REGS];
	int irq;
	int adc;
	int lux_last;
};

struct al3320_data *g_al3320_data_as;
bool g_al3320_switch_on = false;
bool g_bIsP07Attached = false;
static int  g_AlsP07ProbeError = 0xff;

static struct workqueue_struct *al3320_light_workqueue = NULL;
//static struct delayed_work al3320_attached_P07_work;
static struct work_struct al3320_ISR_work;
static struct delayed_work al3320_ISR_delay_work;

/*For resume and debounce I2C issue*/
static struct delayed_work al3320_light_resume_work;
static struct workqueue_struct *al3320_light_delay_workqueue = NULL;
bool g_al3320_suspend_switch_on = false;
bool al3320_interrupt_busy = false;
static int g_al3320_switch_earlysuspend = 0;

static int al3320_map_max_level = 12;
static int g_al3320_light_map[12] = {0,50,100,200,300,450,700,850,1050,1250,1500,2200} ;
/*
static int al3320_threshold_max_level = 42;
static int g_al3320_thd[42] = 
     {0,50,100,150,200,350,450,550,650,750,850,1000,1350,1700,2000,2500,3000,3500,4000,4500,5000,6000,7000,8000,9000,10000,11000,12000,13000,14000,16000,18000,20000,22000,24000,28000,31000,34000,37000,40000,50000,65535};
     */

/* For Calibration*/
#ifdef ASUS_FACTORY_BUILD
static int p_als_calibration_lux = 80000;
static int p_als_low_calibration_adc = 0;
static int p_als_high_calibration_adc = 0;
#endif
static u32 g_al3320_light_calibration = 84;
static int g_al3320_light_shift_calibration = 35;

static struct switch_dev al3320_switch_dev ={ 
        .name = AL3320_DRV_NAME,
        .index = 0,
};

/*register access helpers */
static int __al3320_read_reg(struct i2c_client *client, u32 reg, u8 mask, u8 shift)
{
	return (g_al3320_data_as->reg_cache[reg] & mask) >> shift;
}

static int __al3320_write_reg(struct i2c_client *client, u32 reg, u8 mask, u8 shift, u8 val)
{
	int ret = 0;
	u8 tmp;

	if (reg >= AL3320_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&g_al3320_data_as->lock);

	tmp = g_al3320_data_as->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		g_al3320_data_as->reg_cache[reg] = tmp;

	mutex_unlock(&g_al3320_data_as->lock);
	return ret;
}

/*internally used functions */
/* mode */
static int al3320_get_mode(struct i2c_client *client)
{
	return __al3320_read_reg(client, AL3320_MODE_COMMAND, AL3320_MODE_MASK, AL3320_MODE_SHIFT);
}

static int al3320_set_mode(struct i2c_client *client, int mode)
{
	if(AL3320_POW_UP == (mode & AL3320_MODE_MASK)) {
		g_al3320_switch_on = true;
		//AX_MicroP_enablePinInterrupt(INTR_EN_ALS_INT, 1);
	}
	else if(AL3320_POW_DOWN == (mode & AL3320_MODE_MASK)) {
		g_al3320_switch_on = false;
		//AX_MicroP_enablePinInterrupt(INTR_EN_ALS_INT, 0);
	}

	printk("[als_P07] al3320_set_mode: %s\n", g_al3320_switch_on ? "on" : "off");
	
  	if (mode != al3320_get_mode(client))
		return __al3320_write_reg(client, AL3320_MODE_COMMAND, AL3320_MODE_MASK, AL3320_MODE_SHIFT, mode);
	else
		return 0;
}

/* waiting time */
static int al3320_set_waiting_time(struct i2c_client *client, int wait_time)
{
	return __al3320_write_reg(client, AL3320_WAITING_TIME, AL3320_WAITING_MASK, AL3320_WAITING_SHIFT, wait_time);
}

/* INT enable */
static int al3320_set_int_enable(struct i2c_client *client, int flag)
{
	return __al3320_write_reg(client, AL3320_INT_ENABLE, AL3320_INT_ENABLE_MASK, AL3320_INT_ENABLE_SHIFT, flag);
}

/* suspend enable */
static int al3320_set_sus_enable(struct i2c_client *client, int flag)
{
	return __al3320_write_reg(client, AL3320_INT_ENABLE, AL3320_SUS_ENABLE_MASK, AL3320_SUS_ENABLE_SHIFT, flag);
}

/* meantime */
static int al3320_set_meantime(struct i2c_client *client, int meantime)
{
	return __al3320_write_reg(client, AL3320_ALS_MEANTIME, AL3320_MEANTIME_MASK, AL3320_MEANTIME_SHIFT, meantime);
}

/* a-dummy */
static int al3320_set_adummy(struct i2c_client *client, int adummy)
{
	return __al3320_write_reg(client, AL3320_ALS_ADUMMY, AL3320_ADUMMY_MASK, AL3320_ADUMMY_SHIFT, adummy);
}

/* range */
static long al3320_get_range(struct i2c_client *client)
{
	u8 idx, exgain;
  
	exgain = __al3320_read_reg(client, AL3320_RAN_COMMAND, AL3320_EXGAIN_MASK, AL3320_EXGAIN_SHIFT);    
	idx = __al3320_read_reg(client, AL3320_RAN_COMMAND, AL3320_RAN_MASK, AL3320_RAN_SHIFT);
     
	return (exgain ? al3320_range[idx] : al3320_range[idx+4]);
}

static int al3320_set_range(struct i2c_client *client, int range)
{
	int adummy, mean, ret;

	switch(range)
	{
		case ALS_RAN_0:	adummy = ALS_ADUMMY_0; 
						mean = ALS_MEAN_0;
						break;
		case ALS_RAN_1:	adummy = ALS_ADUMMY_1; 
						mean = ALS_MEAN_1;
						break;
		case ALS_RAN_2:	adummy = ALS_ADUMMY_2; 
						mean = ALS_MEAN_2;
						break;
		case ALS_RAN_3:	adummy = ALS_ADUMMY_3;
						mean = ALS_MEAN_3;
						break;
		default:	adummy = 0;
				mean = 0;
				break;
	}

	ret = al3320_set_adummy(client, adummy);
	if(ret)
		return ret;

	ret = al3320_set_meantime(client, mean);
	if(ret)
		return ret;
	
	return __al3320_write_reg(client, AL3320_RAN_COMMAND, AL3320_RAN_MASK, AL3320_RAN_SHIFT, range);
}

/* exgain */
static int al3320_set_exgain(struct i2c_client *client, int exgain)
{
	return __al3320_write_reg(client, AL3320_RAN_COMMAND, AL3320_EXGAIN_MASK, AL3320_EXGAIN_SHIFT, exgain);
}

/* persist */
static int al3320_set_persist(struct i2c_client *client, int persist)
{
	return __al3320_write_reg(client, AL3320_ALS_PERSIST, AL3320_PERSIST_MASK, AL3320_PERSIST_SHIFT, persist);
}

/* ALS low threshold */
static int al3320_set_althres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AL3320_ALS_LTHL_MASK;

	err = __al3320_write_reg(client, AL3320_ALS_LTHL, AL3320_ALS_LTHL_MASK, AL3320_ALS_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __al3320_write_reg(client, AL3320_ALS_LTHH,	AL3320_ALS_LTHH_MASK, AL3320_ALS_LTHH_SHIFT, msb);
	
	printk("[als_P07] al3320_set_low_threshold_value: %d ,msb=%d, lsb=%d\n", ((msb << 8) | lsb), msb, lsb);

	return err;
}

/* ALS high threshold */
static int al3320_set_ahthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AL3320_ALS_HTHL_MASK;
	
	err = __al3320_write_reg(client, AL3320_ALS_HTHL, AL3320_ALS_HTHL_MASK, AL3320_ALS_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __al3320_write_reg(client, AL3320_ALS_HTHH, AL3320_ALS_HTHH_MASK, AL3320_ALS_HTHH_SHIFT, msb);

	printk("[als_P07] al3320_set_high_threshold_value: %d ,msb=%d, lsb=%d\n", ((msb << 8) | lsb), msb, lsb);

	return err;
}

/* Clear INT pin */
static int al3320_clean_int(struct i2c_client *client)
{
	return __al3320_write_reg(client, AL3320_INT_COMMAND, AL3320_INT_MASK, AL3320_INT_SHIFT, 0);
}

/* get ADC value */
static int al3320_get_adc(struct i2c_client *client, int lock)
{
	unsigned int lsb, msb;
	
	printk("[als_P07] al3320_get_adc++\n");

	if (!lock) mutex_lock(&g_al3320_data_as->lock);

	lsb = i2c_smbus_read_byte_data(client, AL3320_ADC_LSB);
	if (lsb < 0) {
		if (!lock)	mutex_unlock(&g_al3320_data_as->lock);
		switch_set_state(&al3320_switch_dev,P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
		al3320_interrupt_busy = false;
		return lsb;
	}
	
	msb = i2c_smbus_read_byte_data(client, AL3320_ADC_MSB);
	if (msb < 0) {
		if (!lock)	mutex_unlock(&g_al3320_data_as->lock);
		al3320_interrupt_busy = false;
		switch_set_state(&al3320_switch_dev,P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
		return msb;
	}

	if (!lock)	mutex_unlock(&g_al3320_data_as->lock);

	printk("[als_P07]****al3320_get_adc: msb=%d, lsb=%d, adc=%d\n", msb, lsb, (u32)(msb << 8 | lsb));
	
	return (u32)(msb << 8 | lsb);
}

static int al3320_get_lux(int adc)
{
	int i;
	int k_adc = 0;
	int lux = 0;

	if ( adc == 0 )
		k_adc = 0;
	else
		k_adc = g_al3320_light_shift_calibration + (adc * g_al3320_light_calibration/100);
	for( i=1 ; i < al3320_map_max_level ; i++) {
		if( k_adc < g_al3320_light_map[i] ) {
			lux = g_al3320_light_map[ i -1 ];
			break;
		}
		else if( k_adc > g_al3320_light_map[al3320_map_max_level - 1] )	{
			lux = g_al3320_light_map[ al3320_map_max_level -1 ];
			break;
		}
	}	
	if( lux > g_al3320_light_map[al3320_map_max_level - 1] )
		lux = g_al3320_light_map[al3320_map_max_level - 1];

	printk("[als_P07] raw adc= %d, cal_adc= %d, lux = %d\n", adc, k_adc, lux);
	
	return lux;
}

#if 0
static int al3320_get_adc_thd(int adc)
{
	int i;
	int thd_lvl = 0;
	
	for( i = 0 ; i < al3320_threshold_max_level ; i++) {
		if( adc < g_al3320_thd[i] ) {
			thd_lvl = i;
			break;
		}
		else if (adc > g_al3320_thd[al3320_threshold_max_level - 1])	{
			thd_lvl = al3320_threshold_max_level;
			break;
		}
	}

	return thd_lvl;
}
#endif

/* sysfs layer */
static ssize_t al3320_show_range(struct device *dev, struct device_attribute *attr, char *buf)
{
	if ( g_AlsP07ProbeError == 0 && g_bIsP07Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		printk("[als_P07] al3320_show_range: %ld\n", al3320_get_range(client));
		
		return sprintf(buf, "%ld\n", al3320_get_range(client));
	}else
		return 0;
}

static ssize_t al3320_store_range(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if ( g_AlsP07ProbeError == 0 && g_bIsP07Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		unsigned long val;
		int ret;

		printk("[als_P07] al3320_store_range\n");

		if ((kstrtoul(buf, 10, &val) < 0) || (val > 3))
			return -EINVAL;

		printk("[als_P07] al3320_store_range: %lu\n", val);
		ret = al3320_set_range(client, val);
		if (ret < 0)
			printk("[als_P07] al3320_set_range failed\n");
	}

	return count;	
}
static DEVICE_ATTR(range, S_IWUSR | S_IRUGO, al3320_show_range, al3320_store_range);

#ifdef ASUS_FACTORY_BUILD
static int al3320_show_calibration_200(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 p0_calibration_data = 0;
	
	if ( g_bIsP07Attached )	{
		//p0_calibration_data = AX_MicroP_readKDataOfLightSensor();
		
		printk("[als_P07] al3320_show_gait_calibration: %d.%d\n", p0_calibration_data & 0x0000ff00, p0_calibration_data & 0x000000ff);
		
		return sprintf(buf, "%d.%d\n", p0_calibration_data & 0x0000ff00, p0_calibration_data & 0x000000ff);
	}else	{
			printk("[als_P07] Without P07\n");
			return -1;
	}
}

static ssize_t al3320_store_calibration_200(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	p_als_low_calibration_adc = 0;

	if ( g_bIsP07Attached )	{
		printk("[als_P07] al3320_store_resolution\n");

		if ( (kstrtoul(buf, 10, &val) < 0) )
			return -EINVAL;

		p_als_low_calibration_adc = (int)val;

		printk("[als_P07] al3320 Get low calibration adc value : %d\n", p_als_low_calibration_adc );
	}
	else
		printk("[als_P07] Without P07\n");

	return count;
}
static DEVICE_ATTR(calibration_200, S_IRWXU | S_IRWXG | S_IRWXO,
		   al3320_show_calibration_200, al3320_store_calibration_200);

static ssize_t al3320_show_calibration_1000(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 p0_calibration_data = 0;
	
	if ( g_bIsP07Attached )	{
		//p0_calibration_data = AX_MicroP_readKDataOfLightSensor();

		if ( (0xf << 28 ) & p0_calibration_data )	{
			printk("[als_P07] al3320_show_shift_calibration: %d\n", 
				(p0_calibration_data >> 16) | (0xffff << 16 ) );
			return sprintf(buf, "%d\n", (int)((p0_calibration_data >> 16) | (0xffff << 16)) );
		}else	{
			printk("[als_P07] al3320_show_shift_calibration: %d\n", 
				(p0_calibration_data >> 16) );
			return sprintf(buf, "%d\n", (int)(p0_calibration_data >> 16) );
		}
	}else	{
			printk("[als_P07] Without P07\n");
			return -1;
	}
}

static ssize_t al3320_store_calibration_1000(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long val;
	u32 p0_calibration_data = 0;
	p_als_high_calibration_adc = 0;

	if ( g_bIsP07Attached )	{
		printk("[als_P07] al3320_calibration_final\n");
		
		if ( (kstrtoul(buf, 10, &val) < 0) )
			return -EINVAL;
		
		p_als_high_calibration_adc = (int)val;

		printk("[als_P07] al3320 Get Hight calibration adc value : %d\n", p_als_high_calibration_adc );

		/*Calibration operation*/
		g_al3320_light_calibration = 
			p_als_calibration_lux / ( p_als_high_calibration_adc - p_als_low_calibration_adc );

		g_al3320_light_shift_calibration = 
			1000 - ( p_als_high_calibration_adc*g_al3320_light_calibration/100);

		if ( g_al3320_light_calibration > 65535)
			g_al3320_light_calibration = 65535;
		
		printk("[als_P07] al3320 Set shift calibration value : %d\n", g_al3320_light_shift_calibration);

		if ( g_al3320_light_shift_calibration >= 0 )
			p0_calibration_data = ( (0x0 << 28 ) | ((u32)g_al3320_light_shift_calibration << 16) | g_al3320_light_calibration) ;
		else
			p0_calibration_data = ( (0xf << 28 ) | ((u32)g_al3320_light_shift_calibration << 16) | g_al3320_light_calibration) ;

		printk("[als_P07] al3320 Set P07 calibration value : 0x%x\n", (uint32_t)p0_calibration_data);

		//err = AX_MicroP_writeKDataOfLightSensor( (uint32_t)p0_calibration_data );
		if ( err == 0 )
			printk("[als_P07] al3320 calibration success\n");
		else
			printk("[als_P07] al3320 calibration fail\n");
	}
	else
		printk("[als_P07] Without P07\n");

	return count;
}
static DEVICE_ATTR(calibration_1000, S_IRWXU | S_IRWXG | S_IRWXO,
		   al3320_show_calibration_1000, al3320_store_calibration_1000);
#endif

/* mode */
static ssize_t al3320_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	if ( g_AlsP07ProbeError == 0 && g_bIsP07Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		printk("[als_P07] al3320_show_mode: %d\n", al3320_get_mode(client));

		return sprintf(buf, "%d\n", al3320_get_mode(client));
	}else
		return 0;
}

static ssize_t al3320_store_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if ( g_AlsP07ProbeError == 0 && g_bIsP07Attached )	{
		struct i2c_client *client = to_i2c_client(dev);
		unsigned long val;
		int ret;
		printk("[als_P07] al3320_store_mode\n");

		if ((kstrtoul(buf, 10, &val) < 0) || (val > 4))
			return -EINVAL;

		printk("[als_P07] al3320_store_mode: %lu\n", val);
		ret = al3320_set_mode(client, val);
		if (ret < 0)
			printk("[als_P07] al3320_set_mode failed\n");
	}

	return count;
}
static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO, al3320_show_mode, al3320_store_mode);

/* adc */
static int al3320_show_adc(struct device *dev, struct device_attribute *attr, char *buf)
{
	if ( g_AlsP07ProbeError == 0 && g_bIsP07Attached )	{
		int adc = 0;
		struct i2c_client *client = to_i2c_client(dev);
		printk("[als_P07] al3320_show_adc\n");

		/* No LUX data if not operational */
		if (al3320_get_mode(client) != 0x01)
			return -EBUSY;

		adc = al3320_get_adc(client, 0);
		printk("[als_P07] al3320_show_adc: %d\n", adc );

		return sprintf(buf, "%d\n", adc);
	}else
		return 0;
}
static DEVICE_ATTR(adc, S_IRWXU | S_IRWXG | S_IROTH, al3320_show_adc, NULL);

static struct attribute *al3320_attributes[] = {
	&dev_attr_range.attr,
#ifdef ASUS_FACTORY_BUILD
	&dev_attr_calibration_200.attr,
	&dev_attr_calibration_1000.attr,
#endif
	&dev_attr_mode.attr,
	&dev_attr_adc.attr,
	NULL
};

static const struct attribute_group al3320_attr_group = {
    .name = "al3320",
	.attrs = al3320_attributes,
};

static int al3320_init_client(struct i2c_client *client)
{
	int i, ret;

	/* read all the registers once to fill the cache. If one of the reads fails, we consider the init failed */
	for (i = 0; i < AL3320_NUM_CACHABLE_REGS; i++) {
		ret = i2c_smbus_read_byte_data(client, al3320_reg[i]);
		if (ret < 0)
			return -ENODEV;
		g_al3320_data_as->reg_cache[i] = ret;
	}

	/* set defaults */
	al3320_set_mode(client, AL3320_RESET);
	mdelay(15);
	al3320_set_waiting_time(client, AL3320_WAITING);
	al3320_set_range(client, AL3320_RAN);
	al3320_set_exgain(client, AL3320_GAIN);
	al3320_set_sus_enable(client, DISABLE);
	al3320_set_int_enable(client, ENABLE);
	al3320_set_persist(client, AL3320_PERS);
	al3320_set_althres(client, 0);
	al3320_set_ahthres(client, 0);
	
	return 0;
}

int al3320_als_turn_onoff(int state)
{
	int ret = 0;
	int indx = 0;
	int microp_state = -1;
	printk("[als_P07]set_als_pwr_state: %d\n", state );

	#if 0
	if(!AX_MicroP_IsP01Connected())	{
		printk("[als_P07]Without P07 plug in\n");
		return -1;		
	}
	#endif
	
	if( g_al3320_switch_earlysuspend == 1 )	{
		g_al3320_suspend_switch_on = state;
		printk("[als_P07][als] al3320 without resume, by pass; state:%d\n", g_al3320_switch_earlysuspend);
		return 0;
	}else
		g_al3320_switch_earlysuspend = false;

	al3320_interrupt_busy = true;

	#if 0
	/*Check microp state before al3320 initiation*/
	for(indx = 0; indx<5; indx++) {
		microp_state = AX_MicroP_getOPState();
		if(microp_state == st_MICROP_Active)	{
			printk(DBGMSK_PRX_G2"[al3320][als] Microp in Active mode\n");
			break;
		}else	{
			printk("[al3320][als] Microp still in sleep mode, retry = %d\n",indx);
			msleep( 250 );
		}
		if ( indx >= 4 ){
			al3320_interrupt_busy = false;
			return microp_state;
		}
	}
	#endif
	printk("[als_P07]microp power state: %d\n", microp_state );
	
	/*Inital al3320*/
	for(indx = 0; indx<5; indx++) {
		ret = al3320_init_client(g_al3320_data_as->client);
		if(!ret) {
			printk("[al3320][als] init al3320 success\n");
			break;
		}else	{
			printk("[al3320][als] init_client error retry = %d\n",indx);
			msleep( 10 );
		}
		if ( indx >= 4 ){
			al3320_interrupt_busy = false;
			return ret;
		}
	}


	/*Turn on al3320*/
	for(indx = 0; indx<5; indx++) {
		ret = al3320_set_mode(g_al3320_data_as->client, state? AL3320_POW_UP:AL3320_POW_DOWN);
		if(!ret) {
			printk("[al3320][als] switch on al3320 success\n");
			break;
		}else
			printk("[al3320][als] i2c error retry = %d\n",indx);

		if (indx >= 4) {
			al3320_interrupt_busy = false;
			return ret;
		}
	}
	
	/*Release interrupt trigger*/
	al3320_clean_int(g_al3320_data_as->client);
	al3320_interrupt_busy = false;

	if (state == 1)
		printk("[al3320][als] P07 light sensor dev_open\n");
	else
		printk("[al3320][als] P07 light sensor dev_close\n");

	queue_delayed_work(al3320_light_delay_workqueue, &al3320_ISR_delay_work, 10);

	return ret;
}
EXPORT_SYMBOL(al3320_als_turn_onoff);

static void mp_als_interrupt_handler(struct work_struct *work)
{
	//int thd_lvl = 0;
	int lux = 0;
	int ret = 0;
	int als_threshold_hi = 0;
	int als_threshold_lo = 0;
	
	if( g_bIsP07Attached && g_al3320_switch_on ) {
		mutex_lock(&g_al3320_data_as->lock);
		al3320_interrupt_busy = true;

		/*Suspend al3320*/
		ret = al3320_set_sus_enable(g_al3320_data_as->client, ENABLE);

		/*Get ADC value*/
		g_al3320_data_as->adc = al3320_get_adc(g_al3320_data_as->client, 1);
		
		printk("/********************************************************/\n");
		printk("[als_P07] al3320 raw adc value: %d\n", g_al3320_data_as->adc);

		/*Get calibrated adc and determine lux*/
		lux = al3320_get_lux(g_al3320_data_as->adc);

		/* Report Lux*/
		if(lux != g_al3320_data_as->lux_last) {
			g_al3320_data_as->lux_last = lux;
			//report lux	(CM3628 did not register in A11_EVB and A22_EVB)
			if(g_ASUS_hwID != A11_EVB && g_ASUS_hwID != A22_EVB) 
				als_lux_report_event(lux);
		}
		printk("[als_P07][als] last=%d light=%d\n", g_al3320_data_as->lux_last, lux);


		/*Get threshold level of adc*/
		als_threshold_hi = g_al3320_data_as->adc*(100+threshold_overhead)/100;
		als_threshold_lo = g_al3320_data_as->adc*(100-threshold_overhead)/100;
		//thd_lvl = al3320_get_adc_thd(g_al3320_data_as->adc);

		/* Set als threshold*/
		al3320_set_althres(g_al3320_data_as->client, als_threshold_lo);
		al3320_set_ahthres(g_al3320_data_as->client, als_threshold_hi);
		//al3320_set_althres(g_al3320_data_as->client, g_al3320_thd[thd_lvl]);
		//al3320_set_ahthres(g_al3320_data_as->client, g_al3320_thd[thd_lvl + 1]);

		/*Resume al3320*/
		ret = al3320_set_sus_enable(g_al3320_data_as->client, DISABLE);
	
		/*Check interrupt state (Read only)*/
		ret = i2c_smbus_read_byte_data(g_al3320_data_as->client, AL3320_INT_COMMAND);
		if (ret == 0) {
			printk("[al3320][als] P07 light sensor interrupt is cleared\n");
			al3320_interrupt_busy = false;
		}
		else{
			printk("[al3320][als] P07 light sensor interrupt is triggered\n");
			al3320_clean_int(g_al3320_data_as->client);
		}
		
		mutex_unlock(&g_al3320_data_as->lock);
	}
}

#if 0
/////////////////////////////////////////////////////////////////////////////////
//---Pad feature part---
//
static void lightsensor_attached_pad(struct work_struct *work)
{
	u32 p0_calibration_data = 0;

	printk("[als_P07] lightsensor_attached_pad()++\n");	

	/*Get calibration data*/
	//p0_calibration_data = AX_MicroP_readKDataOfLightSensor();

	g_al3320_light_calibration = (p0_calibration_data & 0x0000ffff);

	if ( (0xf << 28 ) & p0_calibration_data )
		g_al3320_light_shift_calibration = (int)( (p0_calibration_data >> 16) | (0xffff << 16 ) );
	else
		g_al3320_light_shift_calibration = (int)(p0_calibration_data >> 16);

	/*Check Calibration value*/
	if ( g_al3320_light_calibration > 255  || g_al3320_light_calibration <= 0 )
		g_al3320_light_calibration = 85;

	if ( g_al3320_light_shift_calibration >= 255  || g_al3320_light_shift_calibration <= -255 )
		g_al3320_light_shift_calibration = 35;
			
	printk("[als_P07] al3320 set calibration and shift: %d , %d\n",g_al3320_light_calibration, g_al3320_light_shift_calibration);
			
	g_bIsP07Attached = true;
	if ( g_AlsP07ProbeError != 0 )	{
		printk("[als_P07] Lightsensor add i2c error!\n");

		//report uevent if prob error
		printk("[als_P07] al3320 probe error, report uevent to framework\n");
		switch_set_state(&al3320_switch_dev, P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
		g_AlsP07ProbeError = -1;

		return;
	}

	if (g_HAL_als_switch_on) {
		g_al3320_suspend_switch_on = 0;

		/*Switch Phone light value to Pad*/
		//g_al3320_data_as->lux_last = g_cm36283_light;
		
		/*Wait al3320 stable*/
		queue_delayed_work(al3320_light_delay_workqueue, &al3320_light_resume_work, 500 );
	}

	printk("[als_P07] lightsensor_attached_pad()--\n");

	return;
}
EXPORT_SYMBOL(lightsensor_attached_pad);

int lightsensor_detached_pad(void)
{
	printk("[als_P07] lightsensor_detached_pad()++\n");

	if( g_al3320_switch_on ) {
		al3320_als_turn_onoff(0);
		g_al3320_switch_on = false;
	}

	g_bIsP07Attached = false;

	printk("[als_P07] lightsensor_detached_pad()--\n");

	return 0;
}
EXPORT_SYMBOL(lightsensor_detached_pad);

static int lightsensor_pad_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	switch (event) {
		case P01_ADD:
			printk("[als_P07][MicroP] P07_ADD \r\n");                
			queue_delayed_work(al3320_light_workqueue, &al3320_attached_P07_work, HZ);
			return NOTIFY_DONE;

		case P01_REMOVE:
			printk("[als_P07][MicroP] P07_REMOVE \r\n");
			lightsensor_detached_pad();
			return NOTIFY_DONE;

		case P01_LIGHT_SENSOR:
			printk("[als_P07][MicroP] P07_ISR \r\n");
			if (work_pending(&al3320_ISR_work)){
				printk("[als_P07] Begin cancel work \r\n");
				cancel_work_sync(&al3320_ISR_work);
				printk("[als_P07] Finish cancel work \r\n");
			}
			if ( !al3320_interrupt_busy )
				queue_work(al3320_light_workqueue ,&al3320_ISR_work);
			else
				printk("[als_P07] Interrupt busy \r\n");

			return NOTIFY_DONE;
		default:
			return NOTIFY_DONE;
		}
}

static struct notifier_block lightsensor_pad_mp_notifier = {
       .notifier_call = lightsensor_pad_mp_event,
        .priority = AL3320_LIGHTSENSOR_MP_NOTIFY,
};
#endif

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_FAIL_SENSOR (-1)

static int Test_Al3320_SensorI2C(struct i2c_client *apClient)
{
	int err = 0;
	int lnResult = I2C_TEST_PASS;

	i2c_log_in_test_case("Test_Al3320_SensorI2C++\n");
	
	if( g_AlsP07ProbeError == 0 && g_bIsP07Attached )	{
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(100));

		if (!g_HAL_als_switch_on)	{
			err = al3320_als_turn_onoff(1);
			if ( err < 0 )	{
				i2c_log_in_test_case("Fail to turn on al3320 lsensor\n");
				lnResult = I2C_TEST_FAIL_SENSOR;
			}
			err = al3320_get_adc(apClient, 0);
			if ( err < 0 )	{
				i2c_log_in_test_case("Fail to read al3320 data\n");
				lnResult = I2C_TEST_FAIL_SENSOR;
			}
			else	{
				err = al3320_als_turn_onoff(0);
				if ( err < 0 )	{
					i2c_log_in_test_case("Fail to turn off al3320 lsensor\n");
					lnResult = I2C_TEST_FAIL_SENSOR;
				}
			}
		}
		else	{
			err = al3320_get_adc(apClient, 0);
			if ( err < 0 )	{
				i2c_log_in_test_case("Fail to read al3320 data\n");
				lnResult = I2C_TEST_FAIL_SENSOR;
			}
			else	{
				err = al3320_als_turn_onoff(0);
				if ( err < 0 )	{
					i2c_log_in_test_case("Fail to turn off al3320 lsensor\n");
					lnResult = I2C_TEST_FAIL_SENSOR;
				}
			}
		}
	}else	{
		i2c_log_in_test_case("Fail to lsensor test\n");
		lnResult = I2C_TEST_FAIL_SENSOR;
	}

	i2c_log_in_test_case("Test_Al3320_SensorI2C--\n");

	return lnResult;
};

static struct i2c_test_case_info gSensorTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(Test_Al3320_SensorI2C),
};
#endif

static void al3320_late_resume_delayed_work(struct work_struct *work)
{
	printk("[als_P07] al3320_late_resume, resume ALS\n");
	if (g_HAL_als_switch_on)
		al3320_als_turn_onoff(g_HAL_als_switch_on);
	else
		al3320_als_turn_onoff(g_al3320_suspend_switch_on || g_al3320_switch_on);

	/*Release interrupt trigger*/
	al3320_clean_int(g_al3320_data_as->client);
	al3320_interrupt_busy = false;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void al3320_early_suspend(struct early_suspend *handler)
{
	printk("[als_P07] ++al3320_early_suspend, als:%d\n", g_al3320_switch_on);
	g_al3320_switch_earlysuspend = 1;
	
	if(1 == g_al3320_switch_on) {
		printk(DBGMSK_PRX_G2"[als_P07] al3320_early_suspend, turn off ambient\n");
		al3320_als_turn_onoff(0);
		
	}

	//force report 0 lux	(CM3628 did not register in A11_EVB and A22_EVB)
	if(g_ASUS_hwID != A11_EVB && g_ASUS_hwID != A22_EVB) 
		als_lux_report_event(0);

	printk("[als_P07] --al3320_early_suspend\n");
}


static void al3320_late_resume(struct early_suspend *handler)
{
	printk("[als_P07] ++al3320_late_resume, als:%d\n", g_al3320_switch_on);

	if(1 == g_al3320_switch_earlysuspend) {
		printk(DBGMSK_PRX_G2"[als_P07] al3320_late_resume, P07 attached: %d\n", g_bIsP07Attached);

		if( g_bIsP07Attached && ( g_al3320_suspend_switch_on || g_al3320_switch_on )) {
			printk(DBGMSK_PRX_G2"[als_P07] al3320_late_resume, resume ALS +++\n");
			queue_delayed_work(al3320_light_delay_workqueue, &al3320_light_resume_work, 150 );
		}
	}

	g_al3320_switch_earlysuspend=0;
	printk("[als_P07]--al3320_late_resume\n");
}

static struct early_suspend al3320_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = al3320_early_suspend,
    .resume = al3320_late_resume,
};

#elif defined(CONFIG_FB)
static void al3320_early_suspend_fb(void)
{
	printk("[als_P07] ++al3320_early_suspend, als:%d\n", g_al3320_switch_on);
	g_al3320_switch_earlysuspend = 1;
	
	if(1 == g_al3320_switch_on) {
		printk(DBGMSK_PRX_G2"[als_P07] al3320_early_suspend, turn off ambient\n");
		al3320_als_turn_onoff(0);
		
	}

	///force report 0 lux	(CM3628 did not register in A11_EVB and A22_EVB)
	if(g_ASUS_hwID != A11_EVB && g_ASUS_hwID != A22_EVB) 
		als_lux_report_event(0);

	printk("[als_P07] --al3320_early_suspend\n");
}


static void al3320_late_resume_fb(void)
{
	printk("[als_P07] ++al3320_late_resume, als:%d\n", g_al3320_switch_on);

	if(1 == g_al3320_switch_earlysuspend) {
		printk(DBGMSK_PRX_G2"[als_P07] al3320_late_resume, P07 attached: %d\n", g_bIsP07Attached);

		if( g_bIsP07Attached && ( g_al3320_suspend_switch_on || g_al3320_switch_on )) {
			printk(DBGMSK_PRX_G2"[als_P07] al3320_late_resume, resume ALS +++\n");
			queue_delayed_work(al3320_light_delay_workqueue, &al3320_light_resume_work, 10 );
		}
	}

	g_al3320_switch_earlysuspend=0;
	printk("[als_P07]--al3320_late_resume\n");
}

static int al3320_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	static int blank_old = 0;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if (blank_old == FB_BLANK_POWERDOWN) {
				blank_old = FB_BLANK_UNBLANK;
				al3320_late_resume_fb();
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			if (blank_old == 0 || blank_old == FB_BLANK_UNBLANK) {
				blank_old = FB_BLANK_POWERDOWN;
				al3320_early_suspend_fb();
			}
		}
	}

	return 0;
}
#endif

static int al3320_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("[als_P07] ++al3320_suspend, als:%d\n", g_al3320_switch_on);
	if (delayed_work_pending(&al3320_light_resume_work)){
		printk("[als_P07] Resume work still runing, begin cancel resume work \r\n");
		cancel_delayed_work_sync(&al3320_light_resume_work);
		printk("[als_P07] Finish cancel work \r\n");
	}
	printk("[als_P07] --al3320_suspend\n");

	return 0;
}

static int al3320_resume(struct i2c_client *client)
{
    return 0;
}

static int al3320_init(void)
{
	int err = 0;
	
	printk("[als_P07] al3320_init++n");
	
	al3320_light_workqueue = create_singlethread_workqueue("al3320_light_wq");
	//INIT_DELAYED_WORK(&al3320_attached_P07_work, lightsensor_attached_pad);
	INIT_WORK(&al3320_ISR_work, mp_als_interrupt_handler);

	//For resume and debounce I2C issue
	al3320_light_delay_workqueue = create_singlethread_workqueue("al3320_light_delay_wq");
	INIT_DELAYED_WORK( &al3320_light_resume_work, al3320_late_resume_delayed_work);
	INIT_DELAYED_WORK(&al3320_ISR_delay_work, mp_als_interrupt_handler);

	#if 0
	//Disable P07 attached temporarily for 1st ICS check-in
	register_microp_notifier(&lightsensor_pad_mp_notifier);
	notify_register_microp_notifier(&lightsensor_pad_mp_notifier, "al3320");
	#endif

	// registered as switch device
	err = switch_dev_register(&al3320_switch_dev);
	if (err < 0)
		goto err_init_sreg;
	
	printk("[als_P07] al3320_init--\n");
	return 0;

err_init_sreg:
	switch_dev_unregister(&al3320_switch_dev);

	return (-1);   
}

static int al3320_input_init(void)
{
	struct input_dev *input_dev = NULL;
	int ret;

	/* allocate light input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		printk("[al3320]: Failed to allocate input_data device\n");
	        goto err_light_all;
    	}
	input_dev->name = "al3320_als";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_MISC, input_dev->absbit);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);
	input_set_drvdata(input_dev, g_al3320_data_as);
	
	ret = input_register_device(input_dev);
	if (ret < 0) {
		printk("[al3320]: Failed to register input_data device\n");
        	goto err_light_reg;
    	}
   	g_al3320_data_as->input_dev = input_dev;
	
	printk("[als_P07] al3320 create_group --\n");
	ret = sysfs_create_group(&input_dev->dev.kobj, &al3320_attr_group);
	if (ret) {
        	printk("could not create sysfs group\n");
		goto err_light_sys;
	}
	printk("[als_P07] al3320 create_group --\n");

	return 0;

err_light_sys:
	input_unregister_device(input_dev);
err_light_reg:
	input_free_device(input_dev);
err_light_all:

	return (-1);   
}

static int __devinit al3320_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int err = 0;
	g_AlsP07ProbeError = -1;

	printk("[als_P07]++al3320_probe\n");

	// Check adapter supports everything 
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))	{
		printk("[als_P07] i2c_check_functionality fail\n");
		g_AlsP07ProbeError = -EIO;
		goto exit_kfree;
	}

	g_al3320_data_as = kzalloc(sizeof(struct al3320_data), GFP_KERNEL);
	if (!g_al3320_data_as)	{
		g_AlsP07ProbeError = -ENOMEM;
		return -ENOMEM;
	}

	g_al3320_data_as->client = client;
	i2c_set_clientdata(client, g_al3320_data_as);
	g_al3320_data_as->adc = 0;
	g_al3320_data_as->lux_last= 0;
	mutex_init(&g_al3320_data_as->lock);

	if (al3320_init() != 0)
		goto exit_kfree;
	if( al3320_input_init()!=0)
		goto exit_kfree;

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend( &al3320_early_suspend_desc );
#elif defined(CONFIG_FB)
	al3320_fb_notif.notifier_call = al3320_fb_notifier_callback;
	al3320_fb_register_fail = fb_register_client(&al3320_fb_notif);
	if (al3320_fb_register_fail)
		printk("[al3320] Unable to register fb_notifier: %d\n", al3320_fb_register_fail);
#endif

#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(client, "Sensor_AL3320",ARRAY_AND_SIZE(gSensorTestCaseInfo));
#endif

	g_AlsP07ProbeError = 0;

	printk("[als_P07]--al3320_probe\n");
	dev_info(&client->dev, "Driver version %s enabled \n", DRIVER_VERSION);
	
	return 0;

exit_kfree:
	g_AlsP07ProbeError = err;
	kfree(g_al3320_data_as);
	printk("[als_P07]--al3320_probe fail : %d\n", err);
	return err;
}

static int __devexit al3320_remove(struct i2c_client *client)
{
	al3320_set_mode(client, 0);
	destroy_workqueue(al3320_light_workqueue);
	destroy_workqueue(al3320_light_delay_workqueue);
	sysfs_remove_group(&client->dev.kobj, &al3320_attr_group);
	mutex_destroy(&g_al3320_data_as->lock);
	switch_dev_unregister(&al3320_switch_dev);
	kfree(g_al3320_data_as);


#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend( &al3320_early_suspend_desc );
#elif defined(CONFIG_FB)
	if(!al3320_fb_register_fail)
		fb_unregister_client(&al3320_fb_notif);
#endif

	return 0;
}

static const struct i2c_device_id al3320_id[] = {
	{ "al3320", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3320_id);

static struct of_device_id al3320_match_table[] = {
	{ .compatible = "liteon,al3320",},
	{},
};

static struct i2c_driver al3320_driver = {
	.driver = {
		.name	= AL3320_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = al3320_match_table,
	},
	.suspend = al3320_suspend,
	.resume	= al3320_resume,
	.probe	= al3320_probe,
	.remove	= __devexit_p(al3320_remove),
	.id_table = al3320_id,
};

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("Version v1.0");

module_i2c_driver(al3320_driver);
