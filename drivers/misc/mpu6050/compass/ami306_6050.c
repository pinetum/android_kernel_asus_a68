/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
 */

/**
 *  @addtogroup COMPASSDL
 *
 *  @{
 *      @file   ami306.c
 *      @brief  Magnetometer setup and handling methods for Aichi AMI306
 *              compass.
 */

/* -------------------------------------------------------------------------- */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/err.h>
#include <linux/fs.h>
#include "mpu-dev.h"

#include "ami_hw.h"
#include "ami_sensor_def.h"

#include <log.h>
#include <linux/mpu_6050.h>
#include "mlsl.h"
#include "mldl_cfg.h"
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"

//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Spec] support P05 e-compass"
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_api.h>
#endif
#ifdef CONFIG_MICROP_NOTIFIER_CONTROLLER
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
#endif
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Spec] support P05 e-compass"

//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
#define AMI_NORMAL_MODE				(0x0)
#define AMI_PHONE_ONLY_MODE			(0x1)
#define AMI_PAD_ONLY_MODE			(0x2)
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"

static struct workqueue_struct *ami306_wq = NULL;
extern void ami306_phone_compass_reinit(struct work_struct *work);
extern void ami306_pad_compass_reinit(struct work_struct *work);
static struct delayed_work ami306_reinit_phone_compass_work;
static struct delayed_work ami306_reinit_pad_compass_work;
extern void ami306_phone_compass_remove(void);
extern void ami306_pad_compass_remove(void);


static int g_isA80 = 0;
static int g_inP05 = 0;
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
static int g_e_compass_mode = 0;
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation
//static int g_e_compass_orientation = 1;
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation

//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation
int asus_generate_compass_orientation_file(char *filename, char *str);
static int asus_change_current_compass_orientation_file(char *src, char *str);
int asus_switch_compass_orientation_file_6050(void);
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation

extern struct ext_slave_descr *p05_ami306_get_slave_descr(void);

int isPhoneInPad(void)
{
    return g_inP05;
}
EXPORT_SYMBOL_GPL(isPhoneInPad);

#ifdef CONFIG_EEPROM_NUVOTON
static int asus_e_compass_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk("%s ++, event=%d\r\n", __FUNCTION__, (int)event);
    if((g_A68_hwID >= A80_SR1) && (g_A68_hwID <= A80_PR))
    {
        switch (event) {
        case P01_REMOVE:
            g_inP05 = 0;
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation
            asus_switch_compass_orientation_file_6050();
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
            if(g_e_compass_mode != AMI_PHONE_ONLY_MODE)
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
            {
                ami306_pad_compass_remove();
                queue_delayed_work(ami306_wq, &ami306_reinit_phone_compass_work, 3*HZ);
            }
            printk(KERN_INFO "[AMI306][PAD] remove !\n");
            break;
        case P01_ADD:
            g_inP05 = 1;
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation
            asus_switch_compass_orientation_file_6050();
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
            if(g_e_compass_mode != AMI_PHONE_ONLY_MODE)
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
            {
                ami306_phone_compass_remove();
                queue_delayed_work(ami306_wq, &ami306_reinit_pad_compass_work, 3*HZ);
            }
            printk(KERN_INFO "[AMI306][PAD] add !\n");
            break;
#ifdef CONFIG_EEPROM_NUVOTON_A80
        case P05_ECOMPASREADY:
            if(g_inP05 == 1)
            {
                printk(KERN_INFO "[AMI306][PAD] receive data !\n");
            }
            break;
#endif
        default:
            break;
        }   
    }
    printk("%s --, event=%d\r\n", __FUNCTION__, (int)event);
    return NOTIFY_DONE;
}

static struct notifier_block asus_e_compass_notifier = {
    .notifier_call = asus_e_compass_event,
#ifdef CONFIG_EEPROM_NUVOTON_A80
    .priority = AMI306_ECOMPASS_MP_NOTIFY,
#endif
};
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ASUS_BSP +++ Jason Chang "check and recover compass calibration file when compass resume"
#define AMI_INTER_OFFSET_PAD "7 -24 -17 -34 -59 12\n"     //AICHI-JP Add for ASUS PadFone
#define AMI_INTER_OFFSET_PHONE "0 0 0 0 0 0\n"
#define AMI_CALIBRATION_FILE "/data/amit/AMI304_Config.ini"
#define ASUS_PAD_CALIBRATION_FILE "/data/amit/AMI304_Config_PAD.ini"
#define ASUS_PHONE_CALIBRATION_FILE "/data/amit/AMI304_Config_PHONE.ini"
#define ASUS_COMPASS_CALIBRATION_FILE "/data/asusdata/AMI304_Config.ini"

//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] rm A68 PAD e-compass inter offset"
#define AMI_INTER_OFFSET_P05 "0 0 0 0 0 0\n"     //AICHI-JP Add for ASUS PadFone
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] rm A68 PAD e-compass inter offset"

int ami306_file_read_6050(char * file_name, unsigned char *buf, int read_len)
{
    int ret=0;
    struct file *fp = NULL;
    mm_segment_t oldfs;
    unsigned char local_buf[256];

    memset(local_buf,0,sizeof(local_buf));

    oldfs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(file_name, O_RDONLY, 0775);
    if(!IS_ERR(fp))
    {
        fp->f_op->read(fp, local_buf, read_len, &fp->f_pos);
        printk("[ami306] read file %s to buffer\n",file_name);
        if(filp_close(fp,NULL))
            printk("[ami306] filp_close failed\n");
    }
    else
    {
        printk("[ami306] open file %s failed\n",file_name);
        ret = -1;
    }

    memcpy(buf,local_buf,sizeof(local_buf));

    set_fs(oldfs);

    return ret;
}
EXPORT_SYMBOL_GPL(ami306_file_read_6050);

int ami306_file_write_6050(char * file_name, unsigned char *buf, int len)
{
    int ret=0;
    struct file *fp = NULL;
    mm_segment_t oldfs;

    oldfs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(file_name, O_CREAT | O_RDWR | O_SYNC, S_IROTH | S_IWOTH | S_IWGRP | S_IRGRP | S_IWUSR | S_IRUSR);
    if(!IS_ERR(fp))
    {
        fp->f_op->write(fp, buf, len, &fp->f_pos);
        printk("[ami306] write %d byte to file %s\n", strlen(buf), file_name);
        if(filp_close(fp,NULL))
            printk("[ami306] filp_close failed\n");
    }
    else
    {
        printk("[ami306] open file %s failed\n",file_name);
        ret =  -1;
    }

    set_fs(oldfs);

    return ret;
}
EXPORT_SYMBOL_GPL(ami306_file_write_6050);

int ami306_recover_compass_calibration_file_6050(void)
{
    int ret = 0;
    unsigned char *cali_buf = NULL;
    unsigned char *temp_buf = NULL;

    if(!(cali_buf = kmalloc(256, GFP_KERNEL)) || !(temp_buf = kmalloc(256, GFP_KERNEL)))
    {
        ret = 7;
        printk("[ami306] Allocate calibration data buffer failed\n");
    }

    if(!ami306_file_read_6050(ASUS_COMPASS_CALIBRATION_FILE,cali_buf,256) && !ret)
    {
            printk("[ami306] create compass calibration file\n");

            memset(temp_buf, 0, 256);
            memcpy(temp_buf, cali_buf, 7);
            memcpy(temp_buf+7,cali_buf+8, 151);
            strcat(temp_buf, AMI_INTER_OFFSET_PHONE);
            if(0 != ami306_file_write_6050(AMI_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
            {
                ret = 2;
                printk("[ami306] Error: create %s failed\n",AMI_CALIBRATION_FILE);
            }

            memset(temp_buf, 0, 256);
            memcpy(temp_buf, cali_buf, 7);
            memcpy(temp_buf+7,cali_buf+8, 151);
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] rm A68 PAD e-compass inter offset"
            if((g_A68_hwID >= A80_SR1) && (g_A68_hwID <= A80_PR))
                strcat(temp_buf, AMI_INTER_OFFSET_P05);
            else
                strcat(temp_buf, AMI_INTER_OFFSET_PAD);
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] rm A68 PAD e-compass inter offset"            
            if(0 != ami306_file_write_6050(ASUS_PAD_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
            {
                ret = 3;
                printk("[ami306] Error: create %s failed\n",ASUS_PAD_CALIBRATION_FILE);
            }

            memset(temp_buf, 0, 256);
            memcpy(temp_buf, cali_buf, 7);
            memcpy(temp_buf+7,cali_buf+8, 151);
            strcat(temp_buf, AMI_INTER_OFFSET_PHONE);
            if(0 != ami306_file_write_6050(ASUS_PHONE_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
            {
                ret = 4;
                printk("[ami306] Error: create %s failed\n",ASUS_PHONE_CALIBRATION_FILE);
            }
    }
    else
    {
        ret =1;
        printk("[ami306] Error: read %s failed\n",ASUS_COMPASS_CALIBRATION_FILE);
    }

    if(cali_buf)
        kfree(cali_buf);
    if(temp_buf)
        kfree(temp_buf);

    return ret;
}
EXPORT_SYMBOL_GPL(ami306_recover_compass_calibration_file_6050);

int Check_and_recover_Calibration_file_6050(void)
{
    int ret = 0;
    int mem_index;
    int count = 0;
    char temp_string[64];
    unsigned char *temp_buf = NULL;

    temp_buf = kmalloc(256, GFP_KERNEL);
    if(!temp_buf)
    {
        return -ENOMEM;
        printk("[ami306] Allocate calibration data buffer failed\n");
    }

    if(0 == ami306_file_read_6050(AMI_CALIBRATION_FILE,temp_buf,256))
    {
            //parser inter offset
            for(mem_index = 0; mem_index<256; mem_index++)
            {
                if(temp_buf[mem_index] == 0x0a)
                {
                    count++;
                    if(count == 9)  // line 9
                    {
                        memset(temp_string, 0xff, sizeof(temp_string));
                        memcpy(temp_string, (temp_buf+mem_index+1), sizeof(temp_string));
                        printk("[ami306] inter offset = [%s]\n",temp_string);

                        if(!memcmp(temp_string,AMI_INTER_OFFSET_PHONE,strlen(temp_string)) || !memcmp(temp_string,AMI_INTER_OFFSET_PAD,strlen(temp_string)))
                        {
                            ret = 0;
                            printk("[ami306] calibraiton file is valid\n");
                        }
                        else
                        {
                            ret = 1;
                            printk("[ami306] calibraiton file is invalid\n");
                        }
                    }
                }
            }
    }
    kfree(temp_buf);
    return ret;
}
EXPORT_SYMBOL_GPL(Check_and_recover_Calibration_file_6050);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation
#define ASUS_ORIENTATION_SETTING_FILE "/data/amit/Orientation.ini"

#define ASUS_PII_PHONE_ORIENTATION_SETTING "6 7\n"
#define ASUS_PIII_1_PHONE_ORIENTATION_SETTING "24 5\n"
#define ASUS_PIII_1_PAD_ORIENTATION_SETTING "18 7\n"
#define ASUS_PIII_1_PAD_SR4_SETTING "6 7\n"
#define ASUS_PIII_2_PAD_ORIENTATION_SETTING "6 7\n"
#define ASUS_ORIENTATION_SETTING "6 7\n"

#define BUF_SZ 32

int asus_generate_compass_orientation_file(char *filename, char *str)
{
	unsigned char *ori_buf = NULL;
	unsigned char *temp_buf = NULL;
	int ret = 0;
	
	if(!(ori_buf = kmalloc(BUF_SZ, GFP_KERNEL)) || !(temp_buf = kmalloc(BUF_SZ, GFP_KERNEL)))
    {
        ret = 7;
        printk(KERN_INFO "[AMI306] Allocate orientation data buffer failed\n");
    }
	memset(temp_buf, 0, BUF_SZ);
	memcpy(temp_buf, str, strlen(str));
	if(0 != ami306_file_write_6050(filename,temp_buf,strlen(temp_buf)))
	{
		ret = 2;
		printk("[AMI306] Error: create %s failed\n",AMI_CALIBRATION_FILE);
	}
	
	if(ori_buf)
		kfree(ori_buf);
	if(temp_buf)
		kfree(temp_buf);
		
	return ret;
}
EXPORT_SYMBOL_GPL(asus_generate_compass_orientation_file);

static int asus_change_current_compass_orientation_file(char *src, char *str)
{
	int ret = 0;
	
	asus_generate_compass_orientation_file(src, str);
	
	return ret;
}

int asus_switch_compass_orientation_file_6050(void)
{
	// A80 project
	if((g_A68_hwID >= A80_SR1) && (g_A68_hwID <= A80_PR))
	{
		// pad
		if(g_inP05 == 1)
		{
			// normal mode
			if(g_e_compass_mode != AMI_PHONE_ONLY_MODE)
			{
				printk(KERN_INFO "[AMI306][A80] orientation setting = sensor 2 pad mode\n");
				asus_change_current_compass_orientation_file(ASUS_ORIENTATION_SETTING_FILE, ASUS_PIII_2_PAD_ORIENTATION_SETTING);
			}
			// phone only mode
			else
			{
// ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] support SR4
				if(g_A68_hwID >= A80_SR4){
					printk(KERN_INFO "[AMI306][A80] orientation setting = sensor 1 pad mode (HW_ID > SR4)\n");
					asus_change_current_compass_orientation_file(ASUS_ORIENTATION_SETTING_FILE, ASUS_PIII_1_PAD_SR4_SETTING);
				}
// ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] support SR4
				else{
					printk(KERN_INFO "[AMI306][A80] orientation setting = sensor 1 pad mode\n");
					asus_change_current_compass_orientation_file(ASUS_ORIENTATION_SETTING_FILE, ASUS_PIII_1_PAD_ORIENTATION_SETTING);
				}
			}
		}
		// phone
		else
		{
			printk(KERN_INFO "[AMI306][A80] orientation setting = sensor 1 phone mode\n");
			asus_change_current_compass_orientation_file(ASUS_ORIENTATION_SETTING_FILE, ASUS_PIII_1_PHONE_ORIENTATION_SETTING);
		}
	}
	// A68 project
	else
	{
		printk(KERN_INFO "[AMI306][A68] orientation setting = sensor 1 phone mode\n");
		asus_change_current_compass_orientation_file(ASUS_ORIENTATION_SETTING_FILE, ASUS_PII_PHONE_ORIENTATION_SETTING);	
	}
	return 0;
}
EXPORT_SYMBOL_GPL(asus_switch_compass_orientation_file_6050);
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] dynamic change e-compass orientation

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ASUS_BSP +++ Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
int get_compass_mode(void)
{
	return g_e_compass_mode;
}
EXPORT_SYMBOL_GPL(get_compass_mode);

int set_compass_mode(int value)
{	
	printk(KERN_INFO "[AMI306] 0 : normal | 1 : phone only | 2 : pad only\n");
	
	if (value == 0)
		g_e_compass_mode = AMI_NORMAL_MODE;
	else if (value == 1)
		g_e_compass_mode = AMI_PHONE_ONLY_MODE;
	else if (value == 2)
		g_e_compass_mode = AMI_PAD_ONLY_MODE;
	else
		printk(KERN_INFO "[AMI306] Not supported\n");
		
	return g_e_compass_mode;
}
EXPORT_SYMBOL_GPL(set_compass_mode);
//ASUS_BSP --- Jiunhau_Wang "[A80][Sensor][NA][Others] support E-compass phone/pad change commnd"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int __init ami306_mod_init(void)
{
	pr_info("%s: Probe name %s\n", __func__, "ami306_mod_6050");

	if((g_A68_hwID >= A80_SR1) && (g_A68_hwID <= A80_PR))
	{
		g_isA80 = 1;
		ami306_wq = create_singlethread_workqueue("ami306_wq");
		if(!ami306_wq){
			printk(KERN_INFO "[AMI306][P05] add ami306 wq fail \n");
			destroy_workqueue(ami306_wq);
			return -ESRCH;
		}
		INIT_DELAYED_WORK(&ami306_reinit_phone_compass_work, ami306_phone_compass_reinit);
		INIT_DELAYED_WORK(&ami306_reinit_pad_compass_work, ami306_pad_compass_reinit);
#ifdef CONFIG_EEPROM_NUVOTON
		register_microp_notifier(&asus_e_compass_notifier);
#endif
#ifdef CONFIG_MICROP_NOTIFIER_CONTROLLER
		notify_register_microp_notifier(&asus_e_compass_notifier, "ami306_6050"); //ASUS_BSP Lenter+
#endif
	}
	return 0;
}

static void __exit ami306_mod_exit(void)
{
	pr_info("%s\n", __func__);
	if(g_isA80 == 1)
	{
#ifdef CONFIG_EEPROM_NUVOTON
		unregister_microp_notifier(&asus_e_compass_notifier);
#endif
#ifdef CONFIG_MICROP_NOTIFIER_CONTROLLER
		notify_unregister_microp_notifier(&asus_e_compass_notifier, "ami306_6050"); //ASUS_BSP Lenter+
#endif
	}
}

module_init(ami306_mod_init);
module_exit(ami306_mod_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Driver to integrate AMI306 sensor with the MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ami306_mod_6050");


