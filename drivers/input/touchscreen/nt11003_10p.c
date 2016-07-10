 /* drivers/input/touchscreen/nt11003_touch.c
 *
 * Copyright (C) 2010 - 2011 Novatek, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/microp_notify.h>
//#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+

#include <linux/wait.h>
#include <linux/wakelock.h>

#include <linux/proc_fs.h>
#include <linux/nt11003.h>
#include "nt11003_firmware.h"
#include "nt11306_firmware.h"

//ASUS_BSP HANS: add for led icon +++
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/leds-pm8xxx.h>
#include <linux/pwm.h>

#define LED_MAX_LEVEL 100
#define LEVEL_TO_TABLE(x) (2*((x)/5))

static struct pwm_device *pwmb;

static int PERIOD = 50;
static int duty_time = 25;

static int duty_table[] = {0,50,7,50,10,50,12,50,15,50,17,50,
                               19,50,22,50,25,50,27,50,30,50,
                               32,50,35,50,37,50,40,50,42,50,
                               45,50,47,50,50,50,100,100,1000000,1000000};

struct workqueue_struct *wq_led_icon;
struct delayed_work icon_led_off_w, icon_led_on_w;

static int mv_led_level = 10;
static int user_duration = 2000; //in microseconds
static int user_mode = 0;
static int screen_unlock = 0;
//ASUS_BSP HANS: add for led icon ---

struct nt11003_ts_data *nvts; // ASUS_BSP Deeo : put nt11003_ts_data to global variable

//ASUS_BSP Deeo : add for On/Off touch in P05 +++
static struct workqueue_struct *g_nv_wq_attach_detach;
static struct work_struct g_mp_attach_work_nv;
static struct delayed_work g_mp_detach_work_nv;
//ASUS_BSP Deeo : add for On/Off touch in P05 ---

#define BABBAGE_NT11003_TS_RST1		62
#define BABBAGE_NT11003_TS_INT1		12

#define INT_PORT BABBAGE_NT11003_TS_INT1
#define TS_INT gpio_to_irq(INT_PORT)
#define MAX(a,b) ((a) < (b) ? (b) : (a))

#define NVT_TOUCH_CTRL_DRIVER 1
#define UPDATE_FIRMWARE 1
static int FW_VERSION = 0 ; //ASUS_BSP : Add for firmware version
static int rc_cal_fail = 1; //ASUS_BSP HANS: some fw update need RC cal default set 1 ( fail ) 
static int driver_version = 0xFF;
static int tpid=0xFF;
static int chipid=0xFF;

//ASUS_BSP : Add for wake_lock when update firmware +++
static struct wake_lock touch_wake_lock;
static int tp_state_update =0;
static int tp_det_suspend =0;
static int tp_early_suspend =0;
static int tp_pad_attach =0;
//ASUS_BSP : Add for wake_lock when update firmware ---

//ASUS_BSP : Add report location +++
#define REPORT_LOCATION 0
#if REPORT_LOCATION
static int REPORT_COUNT = 100;
static int finger_count[MAX_FINGER_NUM];
#endif
//ASUS_BSP : Add report location ---

#define MAX_LEN		200 //ASUS_BSP Deeo : add for creating virtual_key_maps ++

//ASUS_BSP HANS: To switch touch chip NT11306 +++
static int switch_shift = 0;
static const uint8_t* BUFFER_DATA;
//ASUS_BSP HANS: To switch touch chip NT11306 ---

//ASUS_BSP Deeo: Get USB state +++
static int usb_state = 0;
//ASUS_BSP Deeo: Get USB state ---

#if NVT_TOUCH_CTRL_DRIVER
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME		"NVTflash"

struct nvt_flash_data {
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	struct i2c_client *client;
};

struct nvt_flash_data *flash_priv;
#endif

//ASUS BSP : Add debug Mask HANS +++
enum{
    DEBUG_MV_REPORT = 1U << 1,
	  DEBUG_LED = 1U << 2,
};

//ASUS_BSP : deflault open uart log in Factory_Build
#ifdef ASUS_FACTORY_BUILD
static int debug_mask = 2;
#define NV_RW_ATTR (S_IWUGO | S_IRUGO)
#define NV_WO_ATTR S_IWUGO
#else
static int debug_mask = 0;
#define NV_RW_ATTR (S_IWUSR | S_IWGRP | S_IRUGO)
#define NV_WO_ATTR (S_IWUSR | S_IWGRP)
#endif

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define A80_DP(mask, message, ...) \
	do { \
		if ((mask) & debug_mask) \
			printk(message, ## __VA_ARGS__); \
	} while (0)
//ASUS BSP : Add debug Mask HANS ---


//ASUS_BSP HANS: add for led icon +++
DEFINE_LED_TRIGGER(keypad_led_trigger);

void led_icon_trigger(struct led_trigger *trigger, enum led_brightness event){

    int dutytime = 0;

	if(duty_time < 0){
        dutytime = duty_table[LEVEL_TO_TABLE(event)];
        PERIOD = duty_table[LEVEL_TO_TABLE(event) + 1];
    }
	else
        dutytime = duty_time;

    A80_DP(DEBUG_LED,"[touch] trigger LED duty: %d period: %d\n", dutytime,PERIOD);
    pwm_config(pwmb,dutytime,PERIOD);
	
    if(user_mode == 3 || tp_early_suspend)
        event = 0; 

    A80_DP(DEBUG_LED,"[touch] trigger LED (%d)\n", event);
    led_trigger_event(trigger, event);
}

void led_icon_turnoff(struct work_struct *work){
	
	if(user_mode == 2)
		return;

    led_icon_trigger(keypad_led_trigger, 0);
}

void led_icon_turnon(struct work_struct *work){
	led_icon_trigger(keypad_led_trigger, mv_led_level);
}


static ssize_t led_icon_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "mv_led_level=%d\n", mv_led_level);
}

static ssize_t led_icon_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(sscanf(buf, "%d", &mv_led_level) != 1)
		return -EINVAL;

	if(mv_led_level > LED_MAX_LEVEL)
		mv_led_level = LED_MAX_LEVEL;

    duty_time = -1;
    
	pwm_config(pwmb, (duty_table[LEVEL_TO_TABLE(mv_led_level)]),
                      duty_table[LEVEL_TO_TABLE(mv_led_level) + 1]);
	
	return count;
}

static ssize_t led_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "user_mode=%d\n", user_mode);
}

static ssize_t led_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(sscanf(buf, "%d", &user_mode) != 1)
		return -EINVAL;

	switch(user_mode){
		case 0:{
		user_duration = 2000;
		break;
		}
		case 1:{
		user_duration = 10000;
		break;
		}
		case 2:
		case 3:{
		led_icon_trigger(keypad_led_trigger, mv_led_level);
		return count;
		}
	}
	
	if(delayed_work_pending(&icon_led_off_w)) {
		cancel_delayed_work_sync(&icon_led_off_w);
	}	
	queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(user_duration));

	return count;
}

static ssize_t screen_unlocked_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(sscanf(buf, "%d", &screen_unlock) != 1)
		return -EINVAL;
	
	if((screen_unlock == 1) && delayed_work_pending(&icon_led_off_w)) {
		cancel_delayed_work_sync(&icon_led_off_w);
		queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(10000));
		screen_unlock = 0;
	}

	return count;
}

static ssize_t period_duty_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf((char *)buf, PAGE_SIZE, "period=%d, duty_time=%d\n", PERIOD, duty_time);
}

static ssize_t period_duty_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if(sscanf(buf, "%d %d", &PERIOD, &duty_time) != 2)
		return -EINVAL;

	if(duty_time < 0)
		PERIOD = 50;
	else if(duty_time > PERIOD)
		duty_time = PERIOD;
	
	led_icon_trigger(keypad_led_trigger, mv_led_level);

	if(delayed_work_pending(&icon_led_off_w)) {
		cancel_delayed_work_sync(&icon_led_off_w);
	}
	queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(user_duration));

	return count;
}
//ASUS_BSP HANS: add for led icon ---

#if NVT_TOUCH_CTRL_DRIVER
/*******************************************************
Description:
	Novatek touchscreen control driver initialize function.

Parameter:
	priv:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
int nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
 struct i2c_msg msgs[2];	
 char *str;
 int ret=-1;
 int retries = 0;
 file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 str = file->private_data;
 ret=copy_from_user(str, buff, count);

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = str[1];
	msgs[0].buf   = &str[2];

	while(retries < 20)
	{
		ret = i2c_transfer(flash_priv->client->adapter, msgs, 1);
		if(ret == 1)	break;
		else
			printk("[Touch_N] nvt_flash_write %d\n", retries);
		retries++;
	}
 return ret;
}

int nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
 struct i2c_msg msgs[2];	 
 char *str;
 int ret = -1;
 int retries = 0;
 file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 str = file->private_data;
 if(copy_from_user(str, buff, count))
	return -EFAULT;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = str[0];
	msgs[0].len   = 1;
	msgs[0].buf   = &str[2];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = str[0];
	msgs[1].len   = str[1]-1;
	msgs[1].buf   = &str[3];

	while(retries < 20)
	{
		ret = i2c_transfer(flash_priv->client->adapter, msgs, 2);
		if(ret == 2)	break;
		else
			printk("[Touch_N] nvt_flash_read %d\n", retries);
		retries++;
	}
	ret=copy_to_user(buff, str, count);
 return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev) {
		kfree(dev);
	}
	return 0;   
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};

static int nvt_flash_init(struct nt11003_ts_data *ts)
{		
	int ret=0;
  	NVT_proc_entry = create_proc_entry(DEVICE_NAME, 0666, NULL);
	if(NVT_proc_entry == NULL)
	{
		printk("[Touch_N] Couldn't create proc entry!\n");
		ret = -ENOMEM;
		return ret ;
	}
	else
	{
		printk("[Touch_N] Create proc entry success!\n");
		NVT_proc_entry->proc_fops = &nvt_flash_fops;
	}
	flash_priv=kzalloc(sizeof(*flash_priv),GFP_KERNEL);	
	if (ts == NULL) {
                ret = -ENOMEM;
                goto error;
	}
	flash_priv->client = ts->client;
	printk("============================================================\n");
	printk("[Touch_N] NVT_flash driver loaded\n");
	printk("============================================================\n");	
	return 0;
error:
	if(ret != 0)
	{
	printk("[Touch_N] flash_priv error!\n");
	}
	return -1;
}

#endif

//ASUS_BSP Deeo : add for creating virtual_key_maps +++
static ssize_t novaTP_virtual_keys_register(struct kobject *kobj,
		     struct kobj_attribute *attr, char *buf)
{
	char *virtual_keys = 	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":270:1960:30:30" "\n" \
				__stringify(EV_KEY) ":" __stringify(KEY_HOME) ":540:1960:30:30" "\n" \
				__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":810:1960:30:30" "\n" ;

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",	virtual_keys);
}

static struct kobj_attribute novaTP_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.elan-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &novaTP_virtual_keys_register,
};

static struct attribute *virtual_key_properties_attrs[] = {
	&novaTP_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group virtual_key_properties_attr_group = {
	.attrs = virtual_key_properties_attrs,
};

struct kobject *nova_virtual_key_properties_kobj;
//ASUS_BSP Deeo : add for creating virtual_key_maps ---

//ASUS_BSP Deeo : Add for HWrst +++

static void nova_hwrst(void)
{
	int ret=1;
	int rec=1;

	ret= gpio_request(BABBAGE_NT11003_TS_RST1, "Novatek_HWrst");
	if (ret) {
		pr_err("%s: Failed to get reset gpio %d. Code: %d.",
			__func__, BABBAGE_NT11003_TS_RST1, ret);
	}
	rec = gpio_direction_output(BABBAGE_NT11003_TS_RST1, 1);
	if (rec) {
		pr_err("%s: Failed to setup reset gpio %d. Code: %d.",
			__func__, BABBAGE_NT11003_TS_RST1, rec);
		gpio_free(BABBAGE_NT11003_TS_RST1);
	}

	if(!ret && !rec)
	{
		gpio_set_value( BABBAGE_NT11003_TS_RST1 , 0 );
		msleep(3);
		gpio_set_value( BABBAGE_NT11003_TS_RST1 , 1 );
		printk("[Touch_N] Send hardware reset succes!!\n");
	}
	else
		printk("[Touch_N] Send hardware reset fail!!\n");
}

//ASUS_BSP Deeo : Add for HWrst ---

//ASUS_BSP Deeo : Add for write RC flag to  asusdata +++
#ifdef ASUS_FACTORY_BUILD
#define Novatek_RC_FLAG  "/data/asusdata/A80_TP_RC"

static void nv_write_rc_calibration_work(void)
{
	struct file *fp = NULL; 
	//struct write_rcvalue *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[16];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	//this = container_of(work, struct write_rcvalue, write_rcvalue_work);

	fp = filp_open( Novatek_RC_FLAG, O_RDWR|O_CREAT|O_TRUNC, 0666 );
	if(IS_ERR_OR_NULL(fp)) {
		printk("[Touch_N] nv_write_rc_calibration_work open (%s) fail\n", Novatek_RC_FLAG);
		return;
	}

	sprintf(writestr, "V%d %d", driver_version , rc_cal_fail);

	printk("[Touch_N] Ver %d RC flag = %d[%s(%d)]\n", 
		driver_version ,rc_cal_fail, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk("[Touch_N] nv_write_rc_calibration_work fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}
#endif
//ASUS_BSP Deeo : Add for write RC flag to asusdata ---

/*******************************************************	
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_bytes(struct i2c_client *client, uint8_t *buf, int len)
{
	struct i2c_msg msgs[2];
	int ret=-1;
	int retries = 0;

	msgs[0].flags=!I2C_M_RD;
	msgs[0].addr=client->addr;
	msgs[0].len=1;
	msgs[0].buf=&buf[0];

	msgs[1].flags=I2C_M_RD;
	msgs[1].addr=client->addr;
	msgs[1].len=len-1;
	msgs[1].buf=&buf[1];

	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,msgs, 2);
		if(ret == 2)break;
		retries++;
	}
	return ret;
}

/*******************************************************	
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.
	
return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=!I2C_M_RD;
	msg.addr=client->addr;
	msg.len=len;
	msg.buf=data;		
	
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}
	return ret;
}

static int i2c_write_bytes_addr(struct i2c_client *client,uint8_t *data,int len, unsigned char addr)
{
	struct i2c_msg msg;
	int ret=-1;
	int retries = 0;

	msg.flags=!I2C_M_RD;
	msg.addr=addr;
	msg.len=len;
	msg.buf=data;		
	
	while(retries<5)
	{
		ret=i2c_transfer(client->adapter,&msg, 1);
		if(ret == 1)break;
		retries++;
	}
	return ret;
}

#if UPDATE_FIRMWARE
void CTP_I2C_READ(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	break;
		retries++;
	}
	return;	
}


void CTP_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = data;		
	
	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return;
}
#endif

//ASUS_BSP HANS: To switch touch chip NT11306 +++
static void switchChip(void){
/*
	struct nt11003_ts_data *ts = nvts;
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0xF0;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);

    if(I2C_Buf[1] == 0x6){
        switch_shift = 32;
	printk("[Touch_N] NT11306!!\n");
        BUFFER_DATA = BUFFER_DATA_306;
    }
    else{
        switch_shift = 0;
	printk("[Touch_N] NT11003!!\n");
        BUFFER_DATA = BUFFER_DATA_003;
    }
   */
  if( g_A68_hwID >= A80_SR4 )
  {
	switch_shift = 32;
	BUFFER_DATA = BUFFER_DATA_306;
	printk("[Touch_N] NT11306!!\n");
  }
  else{
	switch_shift = 0;
	BUFFER_DATA = BUFFER_DATA_003;
	printk("[Touch_N] NT11003!!\n");
  }
}
//ASUS_BSP HANS: To switch touch chip NT11306---



//ASUS_BSP HANS: temporary way to notify usb state +++
#if 1
void nv_touch_mode(int state)
{

	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	struct nt11003_ts_data *ts = nvts;
	if(ts==NULL)
		return;
	if(tp_pad_attach){
		printk("[Touch_N] In Pad, skip USB notify!!\n");
		return;
	}

	usb_state = state;

	if(tp_state_update == 1){
		printk("[Touch_N] Updating TP FW, Block USB notify!!\n");
		return;
	}

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8F - switch_shift;
	I2C_Buf[2] = 0xFF;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
    
	if(usb_state == 0){
	        printk("[Touch_N] Usb Power OFF.\n");
		I2C_Buf[1] = 0xF0;
	}
	else if(usb_state == 2){
	        printk("[Touch_N] Usb Power ON DC.\n");
	        I2C_Buf[1] = 0xF1;
	}
	else if(usb_state == 4){
	        printk("[Touch_N] Usb Power ON AC.\n");
	        I2C_Buf[1] = 0xF2;
	}
	else{
		printk("[Touch_N] Use USB default value %d\n",usb_state);
		usb_state = 0;
		I2C_Buf[1] = 0xF0;
	}
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8B - switch_shift;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
}

EXPORT_SYMBOL(nv_touch_mode);
#endif
//ASUS_BSP HANS: temporary way to notify usb state ---

static int nt11003_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
    unsigned char i2c_control_buf[3] = {0xFF,  (0x8F - switch_shift), 0xFF};		//suspend cmd, (I2C buffer = 255, data = 0)
	unsigned char i2c_control_buf2[2] = {0x00,  0xAF};		//suspend cmd, (I2C buffer = 255, data = 0)
    if (ts->use_irq)
		disable_irq(client->irq);
	i2c_write_bytes(ts->client, i2c_control_buf, 3);
	i2c_write_bytes(ts->client, i2c_control_buf2, 2);
	return 0;
}

static int nt11003_ts_resume(struct i2c_client *client)
{
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	unsigned char i2c_swrst_buf[2] = {0x00, 0x5A};		//suspend cmd, (I2C buffer = 255, data = 0)
	i2c_write_bytes_addr(client, i2c_swrst_buf, 2, 0x7F);

	if (ts->use_irq)
		enable_irq(client->irq);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nt11003_ts_early_suspend(struct early_suspend *h)
{
	struct nt11003_ts_data *ts;

	if (tp_early_suspend == 1 || tp_pad_attach == 1 )
	{
		printk("[Touch_N] Skip suspend!!! %d %d\n",tp_early_suspend,tp_pad_attach);
		return;
	}

	if (tp_state_update == 0 )
	{
		tp_early_suspend = 1;
		//ASUS_BSP HANS: add for led icon +++
		if(delayed_work_pending(&icon_led_off_w)){
			cancel_delayed_work_sync(&icon_led_off_w);
		}

		led_icon_trigger(keypad_led_trigger,0);
		//ASUS_BSP HANS: add for led icon ---

		printk("[Touch_N] Novatek early_suspend!!!\n");
		ts = container_of(h, struct nt11003_ts_data, early_suspend);
		nt11003_ts_suspend(ts->client, PMSG_SUSPEND);
	}
	else{
		tp_det_suspend = 1;
		printk("[Touch_N] Updating FW, Pause early_suspend!!\n");
	}
}

static void nt11003_ts_late_resume(struct early_suspend *h)
{
	struct nt11003_ts_data *ts;

	if(tp_pad_attach == 1 )
	{
		printk("[Touch_N] Skip resume!!! %d %d\n",tp_early_suspend,tp_pad_attach);
		return;
	}

	if(tp_early_suspend == 1 )
	{
		tp_early_suspend = 0;
		//ASUS_BSP HANS: add for led icon +++
		if(delayed_work_pending(&icon_led_on_w)){
			cancel_delayed_work_sync(&icon_led_on_w);
		}
		queue_delayed_work(wq_led_icon, &icon_led_on_w, msecs_to_jiffies(900));

		if(delayed_work_pending(&icon_led_off_w)){
			cancel_delayed_work_sync(&icon_led_off_w);
		}

		queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(user_duration));
		//ASUS_BSP HANS: add for led icon ---

		printk("[Touch_N] Novatek late_resume!!!\n");
		ts = container_of(h, struct nt11003_ts_data, early_suspend);
		nt11003_ts_resume(ts->client);

		msleep(200); //ASUS_BSP Deeo: wait soft calibration finish

		nv_touch_mode(usb_state);	// ASUS_BSP Deeo: Add for resume to get USB state

	}
	else
		printk("[Touch_N] Skip resume!!! %d %d\n",tp_early_suspend,tp_pad_attach);
}
#endif

//ASUS_BSP Deeo : add for On/Off touch in P05 +++
static void attach_nv_padstation_work(struct work_struct *work)
{
	printk("[Touch_N] attach_padstation_work()++\n");

	if(tp_early_suspend == 1 )
		printk("[Touch_N] Skip suspend!!! %d %d\n",tp_early_suspend,tp_pad_attach);
	else if(tp_state_update ==1 )
		printk("[Touch_N] Skip suspend!!! Updating FW %d",tp_state_update);
	else{
		printk("[Touch_N] Pad attach suspend!!!\n");
		nt11003_ts_suspend(nvts->client, PMSG_SUSPEND);
		tp_early_suspend = 1;
	}

	//ASUS_BSP HANS: add for led icon +++
	if(delayed_work_pending(&icon_led_off_w)){
		cancel_delayed_work_sync(&icon_led_off_w);
	}
	led_icon_trigger(keypad_led_trigger,0);
	//ASUS_BSP HANS: add for led icon ---

	tp_pad_attach = 1;
	printk("[Touch_N] attach_padstation_work()--\n");
}

static void detach_nv_padstation_work(struct work_struct *work)
{
	printk("[Touch_N] detach_padstation_work()++\n");

	if(tp_early_suspend == 1 )
	{
		printk("[Touch_N] Pad detach resume!!!\n");
		nt11003_ts_resume(nvts->client);
		tp_early_suspend = 0;
	}
	else
		printk("[Touch_N] Skip resume!!! %d %d\n",tp_early_suspend,tp_pad_attach);

	tp_pad_attach = 0;

	msleep(200); //ASUS_BSP Deeo: wait soft calibration finish
	usb_state = 0;
	printk("[Touch_N] Force switch USB state to NO POWER %d\n",usb_state);
	nv_touch_mode(usb_state);	// ASUS_BSP Deeo: Add for resume to get USB state

	printk("[Touch_N] detach_padstation_work()--\n");
}
//ASUS_BSP Deeo : add for On/Off touch in P05 ---

//ASUS_BSP Deeo : add suspend wake lock +++
static void detect_suspend (void)
{
	if (tp_det_suspend == 1){
		printk("[Touch_N] Continues suspend!!!\n");
		tp_early_suspend = 1; 
		tp_det_suspend = 0;
		nt11003_ts_suspend(nvts->client, PMSG_SUSPEND);
	}
}
//ASUS_BSP Deeo : add suspend wake lock ---

//ASUS_BSP Deeo : Add touch deriver attribute +++
static void get_driver_ver(void)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	//I2C_Buf[0] = 0x78;
	//CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);

	//switchChip(); //ASUS_BSP HANS: To switch touch chip NT11306 for save ++

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8B - switch_shift;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(nvts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x78;
	CTP_I2C_READ(nvts->client, 0x01, I2C_Buf, 3);

	msleep(500); //delay 500ms


	if ((I2C_Buf[1] & I2C_Buf[2]) == 0){
		printk("[Touch_N] Driver FW_VERSION: %d\n",I2C_Buf[1]);
		//sprintf(tmpstr,"%d\n",I2C_Buf[1]);
		driver_version = I2C_Buf[1];
	}
	else{
		printk("[Touch_N] I2C_Buf[1]: %d, I2C_Buf[2]: %d, ERROR!!!\n",I2C_Buf[1] , I2C_Buf[2]);
		//sprintf(tmpstr,"%d\n",0xFF);
		driver_version = 0xFF;
	} 
}

static ssize_t driver_ver(struct device *dev, struct device_attribute *devattr,char *buf)
{
	char tmpstr[10];
/*	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);

	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	//I2C_Buf[0] = 0x78;
	//CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);

	switchChip(); //ASUS_BSP HANS: To switch touch chip NT11306 for save ++

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8B - switch_shift;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x78;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 3);

	msleep(500); //delay 500ms


	if ((I2C_Buf[1] & I2C_Buf[2]) == 0){
		printk("[Touch_N] Driver FW_VERSION: %d\n",I2C_Buf[1]);
		sprintf(tmpstr,"%d\n",I2C_Buf[1]);
		driver_version = I2C_Buf[1];
	}
	else{
		printk("[Touch_N] I2C_Buf[1]: %d, I2C_Buf[2]: %d, ERROR!!!\n",I2C_Buf[1] , I2C_Buf[2]);
		sprintf(tmpstr,"%d\n",0xFF);
		driver_version = 0xFF;
	}
*/

	get_driver_ver();

	sprintf(tmpstr,"%d\n",driver_version);
	strncat(buf,tmpstr,strlen(tmpstr));
	
	return strlen(buf);
}

static ssize_t package_ver(struct device *dev, struct device_attribute *devattr,char *buf)
{
	char tmpstr[10];
 	int index = 0x6700;

	FW_VERSION = BUFFER_DATA[index];
	printk("[Touch_N] Package FW_VERSION: %d\n",FW_VERSION);
	sprintf(tmpstr,"%d\n",FW_VERSION);
	strncat(buf,tmpstr,strlen(tmpstr));

	return strlen(buf);
}

static void get_tpid(void)
{	
	int cmd_ack=0xFF;
	int check=0xFF;
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0xE8;
	I2C_Buf[1] = 0xC4;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(nvts->client, 0x01, I2C_Buf, 3);

	msleep(200);

	I2C_Buf[0] = 0xE9;
	CTP_I2C_READ(nvts->client, 0x01, I2C_Buf, 4);

	cmd_ack = I2C_Buf[1];
	tpid = I2C_Buf[2];
	check = tpid + cmd_ack;
	
	printk("[Touch_N]status:0x%x Data:0x%x Result:0x%x Check:0x%x\n",I2C_Buf[1],I2C_Buf[2],I2C_Buf[3],check);

}

static ssize_t TPID(struct device *dev, struct device_attribute *devattr,char *buf)
{
	char tmpstr[10];
/*	int tpid=0xFF;
	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8B - switch_shift;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x85;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);

	tpid = I2C_Buf[1];
	sprintf(tmpstr,"%d %d\n",I2C_Buf[1],tpid>>5);
*/
	get_tpid();
	tpid = tpid >> 4;
	sprintf(tmpstr,"0x%x\n",tpid);
	strncat(buf,tmpstr,strlen(tmpstr));

	return strlen(buf);
}

static void get_chipid(void)
{
/*	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0xF0;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(nvts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(nvts->client, 0x01, I2C_Buf, 2);

	chipid=I2C_Buf[1];
*/
	if( g_A68_hwID >= A80_SR4 ){
		chipid=0x6;
	}
	else{
		chipid=0x3;
	}
	printk("[Touch_N] ChipID 0x%x\n",chipid);
}

static ssize_t ChipID(struct device *dev, struct device_attribute *devattr,char *buf)
{
	char tmpstr[10];
/*	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0xF0;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);
	sprintf(tmpstr,"0x%x\n",I2C_Buf[1] );
*/
	get_chipid();

	sprintf(tmpstr,"0x%x\n",chipid);
	strncat(buf,tmpstr,strlen(tmpstr));

	return strlen(buf);
}

static ssize_t about_phone(struct device *dev, struct device_attribute *devattr,char *buf)
{
	char tmpstr[36];

	sprintf(tmpstr,"C%x -T%x -V%d\n",chipid,tpid,driver_version);
	strncat(buf,tmpstr,strlen(tmpstr));

	return strlen(buf);
}

static ssize_t get_hwid(struct device *dev, struct device_attribute *devattr,char *buf)
{
	char tmpstr[4];

	sprintf(tmpstr,"%d\n", g_A68_hwID);
	strncat(buf,tmpstr,strlen(tmpstr));

	return strlen(buf);
}

extern bool asus_padstation_exist_realtime(void);
static ssize_t get_pad(struct device *dev, struct device_attribute *devattr,char *buf)
{
	char tmpstr[4];

	sprintf(tmpstr,"%d\n", asus_padstation_exist_realtime());
	strncat(buf,tmpstr,strlen(tmpstr));

	return strlen(buf);
}

static ssize_t soft_reset(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	int cfg;

	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);

	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	sscanf(buf, "%d\n",&cfg);

	printk("[Touch_N] cfg=%d\n",cfg);

	if ( cfg!=1 )
		return count;

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);

	printk("[Touch_N] NT11003 soft-reset!!!\n");

	return count;
}

static ssize_t force_update(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{

	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t i = 0;
	uint8_t j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	uint8_t CheckSum[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// 128/8 = 16 times ;
	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	
	int cfg;
	sscanf(buf, "%d\n",&cfg);
	printk("[Touch_N] cfg=%d\n",cfg);

	if ( cfg!=1 )
		return count;
	
	printk("[Touch_N] Start Update Firmware!!!\n");
	printk("[Touch_N] Init touchFW_wake_lock\n");

	tp_state_update = 1;
	wake_lock(&touch_wake_lock);
	
	//-------------------------------
	// Step1 --> initial BootLoader
 	// Note. 0x7F -> 0x00 -> 0x00 ;
 	// 須配合 Reset Pin 
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a “A5H” to NT1100x

	msleep(2);	// Delay.2mS
	//mdelay(2);

	printk("[Touch_N] inital Bootloader!!\n");  

	//Step 1 : Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a 0x00 to NT1100x

	msleep(20);	// Delay
	//mdelay(20);

	printk("[Touch_N] inital Flash Block!!\n");

	// Read NT1100x status
	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);
	// if return “AAH” then going next step
	if (I2C_Buf[1] != 0xAA)
	{
		dev_info(&client->dev, "[Touch_N] Program : init get status(0x%2X) error\n", I2C_Buf[1]);
		goto error;
	}
	dev_info(&client->dev, "[Touch_N] Program : init get status(0x%2X) success\n", I2C_Buf[1]);

	//---------------------------------------------------------
 	// Step 2 : Erase 26K bytes via Row Erase Command ( 30H )  
 	//---------------------------------------------------------
 	for (i = 0 ; i < (208 + switch_shift) ; i++)	// 26K equals 208 Rows 
 	{
 		Row_Address = i * 128; 															
 						 					
 		I2C_Buf [0] = 0x00;
 		I2C_Buf [1] = 0x30;	// Row Erase command : 30H  
 		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte  
 		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte 
 						
 		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 4);	// Write 30H, Addr_H & Addr_L to NT11003
 
 		msleep(10);	// Delay 15 ms
                  
    	// Read NT11003 status
 		CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2); 
                  
    	// if NT1003 return AAH then going next step          
		if (I2C_Buf[1] != 0xAA)
		{
			dev_info(&client->dev, "[Touch_N] Program : erase(0x%2X) error\n", I2C_Buf[1]);
			goto error;
		}
 	}
	dev_info(&client->dev, "[Touch_N] Program : erase(0x%2X) success\n", I2C_Buf[1]);

	Flash_Address = 0;
        		
	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step3. Host write 128 bytes to NT11003  
	// Step4. Host read checksum to verify
	//----------------------------------------
	dev_info(&client->dev, "[Touch_N] Program : write begin, please wait ...\n");
	for (j = 0 ; j < (208 + switch_shift); j++)	// Write/ Read 208 times 
	{
		Flash_Address = j * 128 ; 						     		 

	    for (i = 0 ; i < 16 ; i++)	// 128/8 = 16 times for One Row program 
		{
    		// Step 3 : write binary data to NT11003  
  			I2C_Buf[0] = 0x00;
			I2C_Buf[1] = 0x55;	//Flash write command
			I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
			I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
			I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
			I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
			I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
			I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
			I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
			I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
			I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
			I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
			I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8
           
			// Calculate a check sum by Host controller. 
			// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
			//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
			//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1 
			CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] + 
            	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
                	      I2C_Buf[13]) + 1; 
           		
			I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer 
			CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 14);	//Host write I2C_Buf[0…12] to NT1100x.

            if(i == 15)
                msleep(7);

			// Read NT1100x status
   			I2C_Buf[0] = 0x00;
			CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);

			// if return “AAH” then going next step
			if (I2C_Buf[1] != 0xAA)
			{
				dev_info(&client->dev, "[Touch_N] Program : write(j=%d, i=%d, 0x%2X) error\n", j, i, I2C_Buf[1]);
      			goto error;
			}
			Flash_Address += 8 ;	// Increase Flash Address. 8 bytes for 1 time
		}
           	
		msleep(10);	// Each Row program --> Need 15ms delay time
	}
	dev_info(&client->dev, "[Touch_N] Program : write finish ~~\n");
	/////////////////////////////////////////////////////////////////////
	
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	

	dev_info(&client->dev, "Program : OK\n");

	printk("[Touch_N] Soft reset!!! \n");

	printk("[Touch_N] Finish Update Firmware!!!\n");
	
	tp_state_update = 0;
	wake_unlock(&touch_wake_lock);
	detect_suspend();
	
	return count;
	
error:

	printk("[Touch_N] Update touch FW fail!!!\n");
	
	tp_state_update = 0;
	wake_unlock(&touch_wake_lock);
	detect_suspend();
	
	return count;

}

//ASUS_BSP HANS: some fw update need RC cal +++
static ssize_t rc_calibration(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{

	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	//int rc_count = 0;
	int cfg = 0;

	sscanf(buf, "%d\n",&cfg);
	printk("[Touch_N] cfg=%d\n",cfg);

	if ( cfg!=1 )
		return count;

	wake_lock(&touch_wake_lock);
	rc_cal_fail = 2;
	tp_state_update = 1;	// Avoid suspend

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	

	msleep(500);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8C - switch_shift;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);	

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x06;
	I2C_Buf[2] = 0x09;
	I2C_Buf[3] = 0x0A;
	I2C_Buf[4] = 0x05;
	I2C_Buf[5] = 0xE6;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 6);	

	msleep(10);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8F - switch_shift;
	I2C_Buf[2] = 0xFF;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);	

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xBE;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);	

    //do{
       msleep(2000);
	
	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x8F - switch_shift;
       I2C_Buf[2] = 0xFE;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);	

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);
        
        if (I2C_Buf[1] == 0xAA){
		dev_info(&client->dev, "[Touch_N] Program : rc_cal get status(0x%2X) success\n", I2C_Buf[1]);

		I2C_Buf[0] = 0x00;
		I2C_Buf[1] = 0x5A;
        	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	

		rc_cal_fail = 0;

#ifdef ASUS_FACTORY_BUILD
		nv_write_rc_calibration_work();
#endif

		tp_state_update = 0;

		msleep(200); //ASUS_BSP Deeo : wait soft calibration finish
		nv_touch_mode(usb_state);	//ASUS_BSP Deeo: Add for Get Usb state after rc_calibartion to avoid usb plugin while update FW

		wake_unlock(&touch_wake_lock);
		detect_suspend();
		
		return count;
        }
        dev_info(&client->dev, "[Touch_N] Program : rc_cal get status(0x%2X), fail\n", I2C_Buf[1]);
    //} while( rc_count++ < 2);

	//ASUS_BSP Deeo : do soft_reset either RC K fail
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
     	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);

	rc_cal_fail = 1;

#ifdef ASUS_FACTORY_BUILD
	nv_write_rc_calibration_work();
#endif

	printk("[Touch_N] RC Calibration Fail!!!\n");

	tp_state_update = 0;
	wake_unlock(&touch_wake_lock);
	detect_suspend();
	
	return count;
}

static ssize_t rc_calibration_result(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", rc_cal_fail);
}
//ASUS_BSP HANS: some fw update need RC cal ---

//ASUS BSP HANS: touch function switch +++
static ssize_t touch_onoff(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
    
    int cfg = 0;

    sscanf(buf, "%d\n",&cfg);
	printk("[Touch_N] cfg=%d\n",cfg);

    if(cfg == 0){
        if(delayed_work_pending(&g_mp_detach_work_nv)) {
		    cancel_delayed_work_sync(&g_mp_detach_work_nv);
            printk("[Touch_N] cancel last detach_work\n");
        }
        queue_work(g_nv_wq_attach_detach, &g_mp_attach_work_nv);
    }
    else{
        
    	if(delayed_work_pending(&g_mp_detach_work_nv)) {
	    	cancel_delayed_work_sync(&g_mp_detach_work_nv);
    	    printk("[Touch_N] cancel last detach_work\n");
	    }
    	queue_delayed_work(g_nv_wq_attach_detach, &g_mp_detach_work_nv, msecs_to_jiffies(2000));
    }
    
	return count;
}
//ASUS BSP HANS: touch function switch ---

//ASUS_BSP Deeo : load bin file to updae FW from user-space +++
//#define Novatek_TP_FW "/sdcard/A80_TP.bin"

int readFile(struct file *fp,uint8_t *buf,int readlen)
{
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos);
	else
		return -1;
}

static ssize_t load_fw(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct file *fp = NULL;
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t i = 0;
	uint8_t j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	uint8_t CheckSum[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// 128/8 = 16 times ;
	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	uint8_t buffer[8];
	int ret = 0;
	char cfg[60];

	mm_segment_t old_fs;
	sscanf(buf, "%s", cfg);;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	printk("[Touch_N] cfg (%s) \n",cfg);

	fp = filp_open( cfg , O_RDONLY, 0 );
	if(IS_ERR_OR_NULL(fp)) {
		printk("[Touch_N] load_fw open (%s) fail\n",cfg);
		return count;
	}

	printk("[Touch_N] Start Update Firmware!!!\n");
	printk("[Touch_N] Init touchFW_wake_lock\n");
	tp_state_update = 1;
	wake_lock(&touch_wake_lock);

	//-------------------------------
	// Step1 --> initial BootLoader
	// Note. 0x7F -> 0x00 -> 0x00 ;
	// 須配合 Reset Pin
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a “A5H” to NT1100x

	msleep(2);	// Delay.2mS
	//mdelay(2);

	printk("[Touch_N] inital Bootloader!!\n");

	//Step 1 : Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a 0x00 to NT1100x

	msleep(20);	// Delay
	//mdelay(20);

	printk("[Touch_N] inital Flash Block!!\n");

	// Read NT1100x status
	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);
	// if return “AAH” then going next step
	if (I2C_Buf[1] != 0xAA)
	{
		dev_info(&client->dev, "[Touch_N] Program : init get status(0x%2X) error\n", I2C_Buf[1]);
		goto error;
	}
	dev_info(&client->dev, "[Touch_N] Program : init get status(0x%2X) success\n", I2C_Buf[1]);

	//---------------------------------------------------------
	// Step 2 : Erase 26K bytes via Row Erase Command ( 30H )
	//---------------------------------------------------------
	for (i = 0 ; i < (208 + switch_shift) ; i++)	// 26K equals 208 Rows
	{
		Row_Address = i * 128;

		I2C_Buf [0] = 0x00;
		I2C_Buf [1] = 0x30;	// Row Erase command : 30H
		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte
		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte

		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 4);	// Write 30H, Addr_H & Addr_L to NT11003

		msleep(10);	// Delay 15 ms

	// Read NT11003 status
 		CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);

	// if NT1003 return AAH then going next step
		if (I2C_Buf[1] != 0xAA)
		{
			dev_info(&client->dev, "[Touch_N] Program : erase(0x%2X) error\n", I2C_Buf[1]);
			goto error;
		}
	}
	dev_info(&client->dev, "[Touch_N] Program : erase(0x%2X) success\n", I2C_Buf[1]);

	Flash_Address = 0;

	////////////////////////////////////////////////////////////////////////////////////
	//----------------------------------------
	// Step3. Host write 128 bytes to NT11003  
	// Step4. Host read checksum to verify
	//----------------------------------------
	dev_info(&client->dev, "[Touch_N] Program : write begin, please wait ...\n");
	for (j = 0 ; j < (208 + switch_shift); j++)	// Write/ Read 208 times 
	{
		Flash_Address = j * 128 ; 						     		 

	    for (i = 0 ; i < 16 ; i++)	// 128/8 = 16 times for One Row program 
		{

			memset(buffer,0,8);
			ret=readFile(fp,buffer,8);

    		// Step 3 : write binary data to NT11003
  			I2C_Buf[0] = 0x00;
			I2C_Buf[1] = 0x55;	//Flash write command
			I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
			I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
			I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
			I2C_Buf[6] = buffer[0];		//Binary data 1
			I2C_Buf[7] = buffer[1];		//Binary data 2
			I2C_Buf[8] = buffer[2];		//Binary data 3
			I2C_Buf[9] = buffer[3];		//Binary data 4
			I2C_Buf[10] = buffer[4];	//Binary data 5
			I2C_Buf[11] = buffer[5];	//Binary data 6
			I2C_Buf[12] = buffer[6];	//Binary data 7
			I2C_Buf[13] = buffer[7];	//Binary data 8
           
			// Calculate a check sum by Host controller. 
			// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
			//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
			//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1 
			CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] + 
            	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
                	      I2C_Buf[13]) + 1; 
           		
			I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer 
			CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 14);	//Host write I2C_Buf[0…12] to NT1100x.

            if(i == 15)
                msleep(7);

			// Read NT1100x status
   			I2C_Buf[0] = 0x00;
			CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);

			// if return “AAH” then going next step
			if (I2C_Buf[1] != 0xAA)
			{
				dev_info(&client->dev, "[Touch_N] Program : write(j=%d, i=%d, 0x%2X) error\n", j, i, I2C_Buf[1]);
      			goto error;
			}
			Flash_Address += 8 ;	// Increase Flash Address. 8 bytes for 1 time
		}
           	
		msleep(10);	// Each Row program --> Need 15ms delay time
	}
	dev_info(&client->dev, "[Touch_N] Program : write finish ~~\n");
	/////////////////////////////////////////////////////////////////////

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	

	dev_info(&client->dev, "Program : OK\n");

	printk("[Touch_N] Soft reset!!! \n");

	printk("[Touch_N] Finish Update Firmware!!!\n");
	
	tp_state_update = 0;
	wake_unlock(&touch_wake_lock);
	detect_suspend();

	set_fs(old_fs);
	filp_close(fp, NULL);

	return count;

error:
	printk("[Touch_N] Update touch FW fail!!!\n");
	
	tp_state_update = 0;
	wake_unlock(&touch_wake_lock);
	detect_suspend();

	set_fs(old_fs);
	filp_close(fp, NULL);

	return count;
}
//ASUS_BSP Deeo : load bin file to updae FW from user-space ---

DEVICE_ATTR(soft_reset, NV_WO_ATTR, NULL, soft_reset);
DEVICE_ATTR(package_ver, S_IRUGO, package_ver, NULL);
DEVICE_ATTR(driver_ver, S_IRUGO, driver_ver, NULL);
DEVICE_ATTR(force_update, NV_RW_ATTR, NULL, force_update);
DEVICE_ATTR(load_fw, NV_RW_ATTR, NULL, load_fw);
DEVICE_ATTR(TPID, S_IRUGO,TPID, NULL);
DEVICE_ATTR(ChipID, S_IRUGO,ChipID, NULL);
DEVICE_ATTR(HWID, S_IRUGO,get_hwid, NULL);
DEVICE_ATTR(PAD, S_IRUGO,get_pad, NULL);
DEVICE_ATTR(about_phone, S_IRUGO,about_phone, NULL);
DEVICE_ATTR(rc_calibration, NV_RW_ATTR, rc_calibration_result, rc_calibration); //ASUS_BSP HANS: fw v23 need RC cal++
//ASUS_BSP HANS: add for led +++
DEVICE_ATTR(key_led, NV_RW_ATTR, led_icon_show, led_icon_store);
DEVICE_ATTR(user_mode, NV_RW_ATTR, led_mode_show, led_mode_store);
DEVICE_ATTR(screen_unlocked, NV_WO_ATTR, NULL, screen_unlocked_store);
DEVICE_ATTR(period_duty, NV_RW_ATTR, period_duty_show, period_duty_store);
//ASUS_BSP HANS: add for led ---
DEVICE_ATTR(touch_onoff, NV_WO_ATTR , NULL, touch_onoff);//ASUS BSP HANS: touch function switch ++

static struct attribute *nt_attrs[] = {
	&dev_attr_package_ver.attr,
	&dev_attr_driver_ver.attr,
	&dev_attr_soft_reset.attr,
	&dev_attr_force_update.attr,
	&dev_attr_load_fw.attr,
	&dev_attr_TPID.attr,
	&dev_attr_ChipID.attr,
	&dev_attr_HWID.attr,
	&dev_attr_PAD.attr,
	&dev_attr_about_phone.attr,
	&dev_attr_rc_calibration.attr, //ASUS_BSP HANS: fw v23 need RC cal ++
//ASUS_BSP HANS: add for led +++
	&dev_attr_key_led.attr,
	&dev_attr_user_mode.attr,
	&dev_attr_screen_unlocked.attr,
	&dev_attr_period_duty.attr,
//ASUS_BSP HANS: add for led ---
    &dev_attr_touch_onoff.attr,//ASUS BSP HANS: touch function switch ++
	NULL
};

static struct attribute_group nt11003_attr_group = {
        .attrs = nt_attrs,
};
//ASUS_BSP Deeo : Add touch deriver attribute ---


/*******************************************************
Description:
	Novatek touchscreen initialize function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static int nt11003_init_panel(struct nt11003_ts_data *ts)
{
	ts->abs_x_max = TOUCH_MAX_WIDTH;
	ts->abs_y_max = TOUCH_MAX_HEIGHT;
	ts->max_touch_num = MAX_FINGER_NUM;
	ts->int_trigger_type = INT_TRIGGER;

	msleep(10);
	return 0; 
}

static unsigned char touch_cunt_old;
/*******************************************************
Description:
	Novatek touchscreen work function.

Parameter:
	ts:	i2c client private struct.
	
return:
	Executive outcomes.0---succeed.
*******************************************************/
static void nt11003_ts_work_func(struct work_struct *work)
{	
	int ret=-1;
	uint8_t  point_data[1+MAX_FINGER_NUM*6]={0};
	unsigned int position = 0;	
	uint8_t track_id[MAX_FINGER_NUM] = {0};
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0;
	unsigned char run_count = 0, lift_count = 0;
	unsigned char touch_cunt_now = 0;
	unsigned char Up_Buf[MAX_FINGER_NUM] = {1,1,1,1,1,1,1,1,1,1};
	unsigned char Up_Index = 0;
	unsigned char tpid=99;

	struct nt11003_ts_data *ts = container_of(work, struct nt11003_ts_data, work);
	
	ret=i2c_read_bytes(ts->client, point_data,  sizeof(point_data)/sizeof(point_data[0]));

  	for (index = 0; index < MAX_FINGER_NUM; index++) //0~9 (10 points)
  	{
		position = 1 + 6*index;
		if ( ( ( (point_data[1+index*6]>>3)&0x1F) > 0) && ( ((point_data[1+index*6]>>3)&0x1F)  <= MAX_FINGER_NUM) )  
		{
			track_id[index] = ( (point_data[1+index*6]>>3)&0x0F )-1; 
			touch_cunt_now++;
		}
	}

	run_count = MAX(touch_cunt_old, touch_cunt_now);

  	for (index = 0; index < run_count; index++) //0~9 (10 points)
  	{
		position = 1 + 6*index;

		if ( (point_data[position]&0x03) == 0x03 )     // Touch UP    Up or No event
		{
			if( (Up_Buf[Up_Index]==1) && (Up_Index != tpid) )  //up for initial state   
			{
				input_mt_slot(ts->input_dev,Up_Index);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
				lift_count++;
			}
			Up_Index ++;
			if ( Up_Buf[Up_Index] == 0 )
			{
				Up_Index ++;
			}
			if(Up_Buf[track_id[index]] == 1) 
			{
			    input_mt_slot(ts->input_dev, track_id[index]);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			}

			//ASUS_BSP HANS: add for led icon +++
			if(delayed_work_pending(&icon_led_off_w)) {
				cancel_delayed_work_sync(&icon_led_off_w);
			}

			if(screen_unlock == 1){
				queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(10000));
				screen_unlock = 0;
			}
			
			queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(user_duration));
			//ASUS_BSP HANS: add for led icon ---
#if REPORT_LOCATION			
			if(finger_count[index] != 0)
				printk("[Touch_N][%d]Touch up!! \n",index);

			finger_count[index] = 0;
#endif            
		} 
		else 
		{
			input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
			input_w = (unsigned int) (point_data[position+4])+10;
			
			if (input_x < 0)	input_x = 0;
			if (input_y < 0)	input_y = 0;

			// ASUS_BSP : Add report location +++
#if REPORT_LOCATION			
			if (finger_count[index]%REPORT_COUNT == 0)
				//A80_DP(DEBUG_MV_REPORT,"[Touch_N][%d][x,y,w][ %d, %d, %d]\n",index,input_x,input_y,input_w);
				printk("[Touch_N][%d][x,y,w][ %d, %d, %d]-[%d] \n",index,input_x,input_y,input_w,finger_count[index]);

			finger_count[index]++;
#endif            
			// ASUS_BSP : Add report location ---

			//ASUS_BSP HANS: add for led icon +++
			if(delayed_work_pending(&icon_led_off_w)){
				cancel_delayed_work_sync(&icon_led_off_w);
			}

			led_icon_trigger(keypad_led_trigger, mv_led_level);
			//ASUS_BSP HANS: add for led icon ---


			if((input_x > 1090 )||(input_y > 1970))	continue;	//ASUS_BSP Deeo : Fix conditition +++
			input_mt_slot(ts->input_dev, track_id[index]);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
		    	        
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);

			//input_mt_sync(ts->input_dev);

			Up_Buf[track_id[index]] = 0;  
			tpid = index;
		}
	}//end for loop
         	
	input_sync(ts->input_dev);

	if ( (run_count - lift_count) == 0)
		touch_cunt_old = touch_cunt_now;
	else
		touch_cunt_old = run_count;

	goto END_WORK_FUNC;


END_WORK_FUNC:
	if(ts->use_irq)
		enable_irq(ts->client->irq);

}

/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.
	
return:
	irq execute status.
*******************************************************/
static irqreturn_t nt11003_ts_irq_handler(int irq, void *dev_id)
{
	struct nt11003_ts_data *ts = dev_id;
	//ts->client = dev_id;
	disable_irq_nosync(ts->client->irq);
	queue_work(nt11003_wq, &ts->work);
	
	return IRQ_HANDLED;
}


//ASUS_BSP Deeo : add for On/Off touch in P05 +++
static int touch_mp_nv_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk("%s ++, event=%d\r\n", __FUNCTION__, (int)event);
	printk("[Touch_N] Mircop event %d\n",(unsigned int)event);
	switch (event) {

		case P01_ADD:{

			if(delayed_work_pending(&g_mp_detach_work_nv)) {
				cancel_delayed_work_sync(&g_mp_detach_work_nv);
				printk("[Touch_N] cancel last detach_work\n");
			}
			queue_work(g_nv_wq_attach_detach, &g_mp_attach_work_nv);
			break;
		}
		case P01_REMOVE:{

			if(delayed_work_pending(&g_mp_detach_work_nv)) {
				cancel_delayed_work_sync(&g_mp_detach_work_nv);
				printk("[Touch_N] cancel last detach_work\n");
			}
			queue_delayed_work(g_nv_wq_attach_detach, &g_mp_detach_work_nv, msecs_to_jiffies(2000));
			break;
		}

	default:
		break;
    }
    printk("%s --, event=%d\r\n", __FUNCTION__, (int)event);
    return NOTIFY_DONE;
}

static struct notifier_block touch_mp_notifier_nv = {
        .notifier_call = touch_mp_nv_event,
        .priority = TOUCH_MP_NOTIFY,
};
//ASUS_BSP Deeo : add for On/Off touch in P05 ---

/*******************************************************
Description:
	Novatek touchscreen probe function.

Parameter:
	client:	i2c device struct.
	id:device id.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int nt11003_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry=0;
	int error=0;
	struct nt11003_ts_data *ts;
	struct nt11003_platform_data *platform;
	//char *version_info = NULL;
	uint8_t test_data[7] = {0x00,};
	const char irq_table[4] = {IRQ_TYPE_EDGE_RISING,
							   IRQ_TYPE_EDGE_FALLING,
							   IRQ_TYPE_LEVEL_LOW,
							   IRQ_TYPE_LEVEL_HIGH};
	//unsigned char i2c_swrst_buf[2] = {0x00, 0x5A};		//suspend cmd, (I2C buffer = 255, data = 0)

	//struct nt11003_i2c_rmi_platform_data *pdata;
	dev_info(&client->dev, "[Touch_N] Install touch driver.\n");
	printk("[Touch_N] nt11003_ts_probe +++\n");

	printk("[Touch_N] Device HWID : %d\n",g_A68_hwID);
	if (g_A68_hwID < A80_EVB){
		printk("[Touch_N] Disable Novatek nt11003 in A68 device\n");
		return -EINVAL;
	}

//ASUS_BSP : add for platform init +++
	platform = client->dev.platform_data;
	if (platform == NULL) {
		dev_err(&client->dev, "[Touch_N] platform data is required!\n");
		ret = -EINVAL;
		goto err_alloc_data_failed;
	}

	if (platform->init_platform_hw) {
		ret = platform->init_platform_hw(client);
		if (ret) {
			dev_err(&client->dev, "[Touch_N] hw init failed");
			//goto err_init_hw;
			goto err_alloc_data_failed;
		}
	}
/*
	if (platform->power_on) {
		ret = platform->power_on(true);
		if (ret) {
			dev_err(&client->dev, "[Touch_N] power on failed");
			goto err_pwr_on;
		}
	}
*/
//ASUS_BSP : add for platform init ---

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		dev_info(&client->dev,  "[Touch_N] Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	INIT_WORK(&ts->work, nt11003_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	//pdata = client->dev.platform_data;
	
#if UPDATE_FIRMWARE
	nvts = ts ;
	switchChip(); //ASUS_BSP HANS: To switch touch chip NT11306 ++
 	FW_VERSION = BUFFER_DATA[26368];
#endif
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_info(&client->dev, "[Touch_N] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

#if 1	
	for(retry=0; retry<3; retry++)
	{
		ret=nt11003_init_panel(ts);
		msleep(2);
		if(ret != 0)
			continue;
		else
			break;
	}
	if(ret != 0) {
		ts->bad_data=1;
		goto err_init_godix_ts;
	}
#endif
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 						// absolute coor (x,y)
	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM);

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

#ifdef NT11003_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif	

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = nt11003_ts_name;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0306;
	ts->input_dev->id.product = 0xFF2F;

//ASUS_BSP : Add touch deriver attribute +++
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_info(&client->dev, "[Touch_N] Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
//ASUS_BSP : Add touch deriver attribute ---

//ASUS_BSP HANS: add for led icon +++
	led_trigger_register_simple("keypad-touched", &keypad_led_trigger);
	pwmb = pwm_get(5);

	wq_led_icon = create_singlethread_workqueue("ICON_LED_WQ");
	INIT_DELAYED_WORK(&icon_led_off_w,led_icon_turnoff);
	INIT_DELAYED_WORK(&icon_led_on_w,led_icon_turnon);
//ASUS_BSP HANS: add for led icon ---

#ifdef INT_PORT
	client->irq=TS_INT;
	
	ts->int_trigger_type = 1; //ASUS_BPS Deeo : set edge fulling

	if (client->irq)
	{
		if (ret < 0) 
		{
			dev_info(&client->dev, "[Touch_N] Failed to request GPIO:%d, ERRNO:%d\n",(int)INT_PORT,ret);
			goto err_gpio_request_failed;
		}
		dev_info(&client->dev, "[Touch_N] ts->int_trigger_type=%d\n",ts->int_trigger_type);
		ret  = request_irq(client->irq, nt11003_ts_irq_handler ,  irq_table[ts->int_trigger_type],
			client->name, ts);
		if (ret != 0) {
			dev_info(&client->dev, "[Touch_N] Cannot allocate ts INT!ERRNO:%d\n", ret);
			gpio_direction_input(INT_PORT);
			gpio_free(INT_PORT);
			goto err_gpio_request_failed;
		}
		else 
		{	
			disable_irq(client->irq);
			ts->use_irq = 1;
			dev_info(&client->dev, "[Touch_N]Reques EIRQ %d succesd on GPIO:%d\n",TS_INT,INT_PORT);
		}

		//ret = i2c_write_bytes_addr(client, i2c_swrst_buf, 2, 0x7F);
		//dev_info(&client->dev, "[Touch_N] Send software reset %s\n", ret == 1 ? "success" : "failed");

		nova_hwrst(); //add hw reset
	}    //End of "if (client->irq)"

#endif

	i2c_connect_client_nt11003 = client;
	for(retry=0;retry < 30; retry++)
	{
		disable_irq(client->irq);
		msleep(5);
		enable_irq(client->irq);
		ret =i2c_read_bytes(client, test_data, 5);
		dev_info(&client->dev, "[Touch_N] test_data[1]=%d,test_data[2]=%d,test_data[3]=%d,test_data[4]=%d,test_data[5]=%d\n",test_data[1],test_data[2],test_data[3],test_data[4],test_data[5]);
		if (ret > 0)
			break;
		dev_info(&client->dev, "[Touch_N] nt11003 i2c test failed!\n");
	}
	if(ret <= 0)
	{
		dev_info(&client->dev,  "[Touch_N] I2C communication ERROR!nt11003 touchscreen driver become invalid\n");
		goto err_i2c_failed;
	}	

	ts->bad_data = 0;

	if(ts->use_irq)
		enable_irq(client->irq);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = nt11003_ts_early_suspend;
	ts->early_suspend.resume = nt11003_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#if NVT_TOUCH_CTRL_DRIVER
	nvt_flash_init(ts);
#endif

	//ASUS_BSP : get information +++
	//get_driver_ver();
	//get_chipid();
	//get_tpid();
	//ASUS_BSP : get information +++

	//ASUS_BSP : Add attribute +++
	error = sysfs_create_group(&client->dev.kobj, &nt11003_attr_group);
	if(error)
		printk("[Touch_N] Creat Touch IC Attribute Fail!!!\n");
	//ASUS_BSP : Add attribute ---

	//ASUS_BSP Deeo: add for creating virtual_key_maps +++
	nova_virtual_key_properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (nova_virtual_key_properties_kobj)
		ret = sysfs_create_group(nova_virtual_key_properties_kobj, &virtual_key_properties_attr_group);
	if (!nova_virtual_key_properties_kobj || ret)
		pr_err("[Touch_N] failed to create novaTP virtual key map!\n");
	//ASUS_BSP Deeo: add for creating virtual_key_maps ---

	dev_info(&client->dev, "[Touch_N] Start %s in %s mode\n", 
	ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	dev_info(&client->dev, "[Touch_N] Driver Modify Date:2013-02-07 by Deeo\n");

	//ASUS_BSP Deeo : add for On/Off touch in P05 +++
	g_nv_wq_attach_detach = create_singlethread_workqueue("g_nv_wq_attach_detach");
	if (!g_nv_wq_attach_detach) {
		printk("[Touch_N] %s: create workqueue failed: g_nv_wq_attach_detach\n", __func__);
	}
	INIT_WORK(&g_mp_attach_work_nv, attach_nv_padstation_work);
	INIT_DELAYED_WORK(&g_mp_detach_work_nv, detach_nv_padstation_work);

	//ASUS_BSP Deeo: add wake lock
	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch_wake_lock");

	register_microp_notifier(&touch_mp_notifier_nv);
//	notify_register_microp_notifier(&touch_mp_notifier_nv, "nt11003_10p"); //ASUS_BSP Lenter+
	//ASUS_BSP Deeo : add for On/Off touch in P05 ---

	return 0;

err_init_godix_ts:
	if(ts->use_irq)
	{
		ts->use_irq = 0;
		free_irq(client->irq,ts);
	#ifdef INT_PORT	
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
	#endif	
	}
err_i2c_failed:
err_gpio_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);

	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}


/*******************************************************
Description:
	Novatek touchscreen driver release function.

Parameter:
	client:	i2c device struct.
	
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int nt11003_ts_remove(struct i2c_client *client)
{
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts && ts->use_irq) 
	{
	#ifdef INT_PORT
		gpio_direction_input(INT_PORT);
		gpio_free(INT_PORT);
	#endif	
		free_irq(client->irq, ts);
	}	
	
	dev_notice(&client->dev,"[Touch_N] The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}


static const struct i2c_device_id nt11003_ts_id[] = {
	{ NT11003_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver nt11003_ts_driver = {
	.probe		= nt11003_ts_probe,
	.remove		= nt11003_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= nt11003_ts_suspend,
	.resume		= nt11003_ts_resume,
#endif
	.id_table	= nt11003_ts_id,
	.driver = {
		.name	= NT11003_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

/*******************************************************	
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit nt11003_ts_init(void)
{
	int ret;
	
	nt11003_wq = create_workqueue("nt11003_wq");		//create a work queue and worker thread
	if (!nt11003_wq) {
		printk(KERN_ALERT "[Touch_N] creat workqueue faiked\n");
		return -ENOMEM;
	}
	ret=i2c_add_driver(&nt11003_ts_driver);
	return ret; 
}

/*******************************************************	
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit nt11003_ts_exit(void)
{
	printk(KERN_ALERT "[Touch_N] Touchscreen driver of guitar exited.\n");

	destroy_workqueue(g_nv_wq_attach_detach);
	unregister_microp_notifier(&touch_mp_notifier_nv);
//	notify_unregister_microp_notifier(&touch_mp_notifier_nv, "nt11003_10p"); //ASUS_BSP Lenter+

	i2c_del_driver(&nt11003_ts_driver);
	if (nt11003_wq)
		destroy_workqueue(nt11003_wq);		//release our work queue
}

late_initcall(nt11003_ts_init);
module_exit(nt11003_ts_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
