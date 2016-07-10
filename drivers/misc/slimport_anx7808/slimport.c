/*
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/slimport_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include "slimport_tx_drv.h"
#include <linux/slimport.h>
#include "slimport_tx_reg.h"
#include <linux/err.h>
#include <linux/kdev_t.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif

#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif


struct i2c_client *anx7808_client;
int g_i2c_error_count = 0;
unchar g_slimport_bypass_check_mdp4 = 0;  // by pass check mdp4 ready flag
unchar g_hdmi_rx_vsync_change = 0;

int dp_pd_value=1;
//#ifdef EYE_TEST
int read_swing_value=0;
int write_swing_value=1;
//#endif
int myDP_TV_mode=0;
int g_dump_7730_reg = 0;
int myDP_force_pad_mode=0;
int g_dump_7808_reg = 0;
int g_force_swing_value = 1;
int g_swing_value=2;  // 600mv
int g_pre_emphis_value=1;  // 3.5db
int myDP_DP_Dongle = 0;
int g_tx_recovery_process = 0;
static bool g_otg_state=false;
struct anx7808_data *g_anx7808;
bool get_otg_state(void){
	return g_otg_state;
}
void sp_tx_hardware_recovery(struct i2c_client *client);
int sp_tx_hardware_power_set(int on);

int dump_7808_reg_info(void)
{
	unchar ch, cl;
	uint n;

	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;

	printk("ANX7808 RX cur_h_res = 0x%x\n", n);
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	printk("ANX7808 RX cur_v_res = 0x%x\n", n);

	sp_read_reg(RX_P0, HDMI_RX_VID_PCLK_CNTR_REG, &cl);
	printk("ANX7808 RX cur_pix_clk = 0x%x\n", cl);

	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &cl);
	printk("ANX7808 RX dvi_status = 0x%x\n",  ((cl & HDMI_MODE) == HDMI_MODE));	
	
	sp_read_reg(RX_P0, 0x14, &cl);
	printk("ANX7808 0x7e:14 = 0x%x\n", cl);
	return 0;

}

int dump_7808_reg_pclk(void)
{
	ulong str_clk = 0;
	unchar c;
 ulong pclk;
 ulong m_val, n_val;


		str_clk = 162;

	sp_read_reg(TX_P0, M_VID_2, &c);
	m_val = c * 0x10000;
	sp_read_reg(TX_P0, M_VID_1, &c);
	m_val = m_val + c * 0x100;
	sp_read_reg(TX_P0, M_VID_0, &c);
	m_val = m_val + c;

	sp_read_reg(TX_P0, N_VID_2, &c);
	n_val = c * 0x10000;
	sp_read_reg(TX_P0, N_VID_1, &c);
	n_val = n_val + c * 0x100;
	sp_read_reg(TX_P0, N_VID_0, &c);
	n_val = n_val + c;

	str_clk = str_clk * m_val;
	pclk = str_clk;
	pclk = pclk / n_val;


	DEV_DBG("   M = %lu, N = %lu, PCLK = %ld MHz\n", m_val, n_val, pclk);
	return 0;
}
//ASUS_BSP+++ larry : check 480p for VGA dongle
bool myDP_check_7808_rx_under_480p(void)
{
	unchar ch, cl;
	uint n1, n2;
	bool ret = false;

	sp_read_reg(RX_P0, HDMI_RX_HACT_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HACT_HIGH, &ch);
	n1 = ch;
	n1 = (n1 << 8) + cl;
	
	sp_read_reg(RX_P0, HDMI_RX_VACT_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VACT_HIGH, &ch);
	n2 = ch;
	n2 = (n2 << 8) + cl;

	if ( ((n1 == 640) || (n1 == 720)) && (n2 == 480))
		ret = true;
	else
		printk("(VGA dongle, not allow output) active RX H x V = (%d x %d)\n", n1, n2);
		
	return ret;

}
//ASUS_BSP--- larry : check 480p for VGA dongle


extern void pad_sp_tx_lt_done_int_handler(void);

static int dp_reset_pd_function(const char *val, struct kernel_param *kp)
{
	int ret=0;
	int old_val = dp_pd_value;
	struct anx7808_platform_data *pdata = anx7808_client->dev.platform_data;
	if (ret)
		return ret;

	if (dp_pd_value > 0xf)  {
		dp_pd_value = old_val;
		return -EINVAL;
	}

	ret = param_set_int(val, kp);
	if(dp_pd_value==0){
/*		
		gpio_set_value(pdata->gpio_usb_select, 0);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 1);
		msleep(500);
		gpio_set_value(pdata->gpio_usb_select, 0);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 1);		
		msleep(500);		
*/
	unchar ch, cl;
	uint n;

	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;

	printk("ANX7808 RX cur_h_res = 0x%x\n", n);
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	printk("ANX7808 RX cur_v_res = 0x%x\n", n);

	sp_read_reg(RX_P0, HDMI_RX_VID_PCLK_CNTR_REG, &cl);
	printk("ANX7808 RX cur_pix_clk = 0x%x\n", cl);

	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &cl);
	printk("ANX7808 RX dvi_status = 0x%x\n",  ((cl & HDMI_MODE) == HDMI_MODE));	

	}else if (dp_pd_value==1) {
		gpio_set_value(pdata->gpio_usb_select, 0);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 1);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 0);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 1);		
		msleep(20);		
		gpio_set_value(pdata->gpio_usb_select, 0);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 1);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 0);
		msleep(20);
		gpio_set_value(pdata->gpio_usb_select, 1);		
		}
	else if (dp_pd_value==2) {
sp_tx_hardware_power_set(1);				
		}
	else if (dp_pd_value==3) {
sp_tx_hardware_power_set(0);		
		}	
	else if (dp_pd_value==4) {
sp_tx_hardware_poweron(anx7808_client);				
		}
	else if (dp_pd_value==5) {
sp_tx_hardware_powerdown(anx7808_client);		
		}	
	else if (dp_pd_value==6) {
		unchar  c1=0;
/*		
		unchar addr_tmp = 0x50;
		switch(addr_tmp)
		{
			case 0x50: c= 0; break;
			case 0x8c: c= 1; break;
			case 0x70: c= 7; break;
			case 0x72: c = 5; break;
			case 0x7a: c = 6; break;
			default : break;

		}
*/		
		i2c_master_read_reg(0, 0x0B, &c1);
		printk("[myDP] 7730reg 0x50, 0x0B=%x\n", (uint)c1);
		msleep(1);
		i2c_master_read_reg(5, 0xE3, &c1);
		printk("[myDP] 7730reg 0x72, 0xE3=%x\n", (uint)c1);		
		msleep(1);
		i2c_master_read_reg(0, 0x06, &c1);
		printk("[myDP] 7730reg 0x50, 0x06=%x\n", (uint)c1);
		msleep(1);
		i2c_master_read_reg(5, 0x06, &c1);
		printk("[myDP] 7730reg 0x72, 0x06=%x\n", (uint)c1);				
		i2c_master_read_reg(0, 0x05, &c1);
		printk("[myDP] 7730reg 0x50, 0x05=%x\n", (uint)c1);				
		}	
	else if (dp_pd_value==7) {
		unchar c=0;
		
		printk("[myDP] write 7730reg 0, 0x06 bit5\n");				
		
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x20; 
		i2c_master_write_reg(0,0x06, c);
		
		}	
	else if (dp_pd_value==8) {
		unchar c=0;
		
		printk("[myDP] write 7730reg 0, 0x06 bit3\n");				
		
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x08; 
		i2c_master_write_reg(0,0x06, c);
		
		}	
	else if (dp_pd_value==9) {
		unchar c=0;
		
		printk("[myDP] write 7730reg 0, 0x06 bit2\n");				
		
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x04; 
		i2c_master_write_reg(0,0x06, c);
		
		}		
	else if (dp_pd_value==10) {
		
		unchar c=0;

		printk("[myDP] write 7730reg 0, 0x06 bit0\n");				
		
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x01; 
		i2c_master_write_reg(0,0x06, c);

		}		
	else if (dp_pd_value==11) {				
		unchar c=0;
		
		printk("[myDP] write 7730reg 5, 0x06 bit5\n");				
		
		i2c_master_read_reg(5,0x06, &c);
		c = c | 0x20; 
		i2c_master_write_reg(5,0x06, c);		
		}	
	else if (dp_pd_value==12) {
		myDP_TV_mode = 1;
	}			
	else if (dp_pd_value==13) {
		myDP_TV_mode = 0;
	}			
	else if (dp_pd_value==14) {
//		printk("[myDP shell] switch to myDP\n");
//		gpio_set_value(pdata->gpio_usb_select, 1);
		myDP_DP_Dongle = 1;
	}			
	else if (dp_pd_value==15) {
//		printk("[myDP shell] switch to USB\n");
//		gpio_set_value(pdata->gpio_usb_select, 0);
		myDP_DP_Dongle = 0;
	}			
	return 0;
}
//#ifdef EYE_TEST
extern void sp_tx_phy_auto_test(void);

static int read_swing_function(const char *val, struct kernel_param *kp)
{
	int ret=0;
	unchar c=0;
	int old_val = read_swing_value;
	//struct anx7808_platform_data *pdata = anx7808_client->dev.platform_data;
	if (ret)
		return ret;

	if (read_swing_value > 0xf)  {
		read_swing_value = old_val;
		return -EINVAL;
	}


	ret = param_set_int(val, kp);
	if (read_swing_value==1)
	{
		myDP_force_pad_mode = 1;				
	}
	else if (read_swing_value==2)  // test tx phy 
	{
		myDP_force_pad_mode = 0;				
	}
	else if (read_swing_value==3) 
	{
		g_pre_emphis_value	= 2;	
		printk("7808 6db\n");
		
	}
	else if (read_swing_value==4)
	{
		g_pre_emphis_value	= 3;	
		printk("7808 9db\n");
		
	}	
	else if (read_swing_value==5)
	{
		g_dump_7730_reg = 1;
	}
	else if (read_swing_value==6)
	{
		g_dump_7730_reg = 0;
	}
	else if (read_swing_value==7)
	{
		unchar c=0;
		
		printk("[myDP] HW reset === 7730reg 0x72, 0x06 bit0\n");				
		
		i2c_master_read_reg(5,0x06, &c);
		c = c | 0x01; 
		i2c_master_write_reg(5,0x06, c);		
	}
	else if (read_swing_value==8)
	{
		unchar c=0;
		int i;
		
		printk("******* ANX7730 reg 0x50 DUMP ============\n");
		for (i=0; i <=0xff; i++)
		{
			i2c_master_read_reg(0, i , &c);
			if  (i%0xf)
				printk("0x%x = (%x), ", i, c);										
			else	
				printk("0x%x = (%x)\n", i, c);										
				
		}

		printk("******* ANX7730 reg 0x72 DUMP ============\n");
		for (i=0; i <=0xff; i++)
		{
			i2c_master_read_reg(5, i , &c);
			if  (i%0xf)
				printk("0x%x = (%x), ", i, c);										
			else	
				printk("0x%x = (%x)\n", i, c);										
		}		
	}
	else if (read_swing_value==9)
	{
		g_force_swing_value = 1;	
	}
	else if (read_swing_value==10)
	{
		g_force_swing_value = 0;
	}
	else if (read_swing_value==11)
	{
		g_swing_value	= 0;
		printk("7808 swing = 200mv\n");
	}
	else if (read_swing_value==12)
	{
		g_swing_value	= 1;
		printk("7808 swing = 400mv\n");
		
	}
	else if (read_swing_value==13)
	{
		g_swing_value	= 2;
		printk("7808 swing = 600mv\n");

	}
	else if (read_swing_value==14)
	{
		g_swing_value	= 3;
		printk("7808 swing = 800mv\n");
		
	}
	else if (read_swing_value==15)
	{
		sp_read_reg(TX_P0, 0xa3, &c);
		printk("=== TX_P0  0xa3: %x ===\n",c);		
	}

	
	
	return 0;
}
static int write_swing_function(const char *val, struct kernel_param *kp)
{
	int ret=0;
	unchar c=0;
	int old_val = write_swing_value;
	//struct anx7808_platform_data *pdata = anx7808_client->dev.platform_data;
	if (ret)
		return ret;

	if (write_swing_value > 0xf)  {
		write_swing_value = old_val;
		return -EINVAL;
	}


	ret = param_set_int(val, kp);
	if (write_swing_value ==1){
		sp_write_reg(TX_P0, 0xa3, write_swing_value);
		printk("TX_P0  0xa3: %x\n",write_swing_value);
	}
	else if (write_swing_value==2)
	{
		g_dump_7808_reg = 1;		
	}
	else if (write_swing_value==3) 
	{
		g_dump_7808_reg = 0;		
	}	
	else if (write_swing_value ==4)
	{
                sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x06, 1, &c);
                printk("sp_tx_config_hdmi_pad , ADJUST_REQUEST_LANE0_1 = 0x%x\n", c);		
	}	
	else if(write_swing_value==5){
		sp_tx_aux_dpcdread_bytes(0x00, 0x01, 0x03, 1, &c);		
		printk("DPCD , LANE0_SET = 0x%x\n", c);	
	}
	
	return 0;
}
//#endif

module_param_call(dp_pd_value, dp_reset_pd_function, param_get_int,
			&dp_pd_value, 0644);

#ifdef CONFIG_HAS_EARLYSUSPEND

static void dp7808_early_suspend(struct early_suspend *handler){

struct anx7808_data *anx7808 = i2c_get_clientdata(anx7808_client);

	return ; //now force , still implement

	if(gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det)==1){
		cancel_delayed_work_sync(&anx7808->work);
		wake_unlock(&anx7808->slimport_lock);
		//sp_tx_hardware_powerdown(anx7808_client);
	}
	printk("[myDP]dp7808_early_suspend+++++++++++++ \n");

}

static void dp7808_late_resume(struct early_suspend *handler){

struct anx7808_data *anx7808 = i2c_get_clientdata(anx7808_client);

	return ; //now force , still implement

	if(gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det)==1){

			wake_lock(&anx7808->slimport_lock);
			//sp_tx_set_sys_state(STATE_CABLE_PLUG);
			//sp_tx_hardware_poweron(anx7808_client);
			queue_delayed_work(anx7808->workqueue, &anx7808->work, 0);
		
	}
	printk("[myDP]dp7808_late_resume++++++++++++++++  \n");
}

static struct early_suspend dp7808_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = dp7808_early_suspend,
    .resume = dp7808_late_resume,
};
#endif
//ASUS BSP wei lai ---

#ifdef ANX7808_PM_SUPPORT
static int anx7808_suspend(struct i2c_client *client, pm_message_t mesg)
{	

	printk("anx7808_suspend (not implement) +++\n");

	printk("anx7808_suspend (not implement) ---\n");	
	
	return 0;
}
static int anx7808_resume(struct i2c_client *client)
{

	printk("anx7808_resume (not implement) ++++\n");
	   
	printk("anx7808_resume (not implement) ----\n");
	
	return 0;
}
#endif



//#ifdef EYE_TEST
module_param_call(read_swing_value, read_swing_function, param_get_int,
			&read_swing_value, 0644);
module_param_call(write_swing_value, write_swing_function, param_get_int,
			&write_swing_value, 0644);
//#endif			
//ASUS BSP wei lai ---

//ASUS BSP wei lai +++
static void (*notify_carkit_in_out_func_ptr)(int) = NULL;
//bool isCarkitConnected=false;
void dp_switch_carkit(bool enable)
{
	//isCarkitConnected=enable;	
	g_otg_state=enable;
	DEV_DBG("%s: dp_switch_carkit is %d++++\n", __func__,enable);
        if(NULL != notify_carkit_in_out_func_ptr)
	{
             printk("[dp] carkit cable notify (%d)\n", enable);
             (*notify_carkit_in_out_func_ptr) (enable);
        }
	
}
//ASUS BSP wei lai ---


int dp_registerCarkitInOutNotificaition(void (*callback)(int))
{
    printk("[myDP]%s +++\n",__FUNCTION__);
    
    notify_carkit_in_out_func_ptr = callback;

//ASUS_BSP: for Booting with otg plug-in issue
    /* if (g_b_SwitchCarkitBootPlugin)
   {
           printk("[MHL] Boot plug-in initial carkit cable notify\n");
	    (*notify_carkit_in_out_func_ptr) (1);
    }*/
    printk("[myDP]%s ---\n",__FUNCTION__);

    return 0;
}








extern  struct regulator *devm_regulator_get(struct device *dev, const char *id);
extern int regulator_set_voltage(struct regulator *regulator, int min_uV, int max_uV);

#ifndef EYE_TEST
//Top tree, default enable HDCP
static bool hdcp_enable = 1;
#endif


int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;
//printk("=======sp_read_reg ====%x====%x\n",slave_addr,offset);
       if ( (g_i2c_error_count > 20) || (g_tx_recovery_process))
               return -1;
	anx7808_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx7808_client, offset);
	if (ret < 0) {
		DEV_ERR("%s: failed to read i2c addr=%x\n",
			__func__, slave_addr);
		if ((++g_i2c_error_count) > 20)
		{
			printk("myDP read i2c error, power down\n");
			sp_tx_hardware_recovery(anx7808_client);
		}	
		return ret;
	}
	else
	{
		g_i2c_error_count = 0;
        g_tx_recovery_process = 0;
	}		
	*buf = (uint8_t) ret;
	return 0;
}

int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value)
{
	int ret = 0;
//printk("=======sp_write_reg ====%x====%x\n",slave_addr,offset);

       if ( (g_i2c_error_count > 20) || (g_tx_recovery_process))
               return -1;
	anx7808_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_write_byte_data(anx7808_client, offset, value);
	if (ret < 0) {
		DEV_ERR("%s: failed to write i2c addr=%x\n",
			__func__, slave_addr);
		if ((++g_i2c_error_count) > 20)
		{
			printk("myDP write i2c error, power down\n");
			sp_tx_hardware_recovery(anx7808_client);
		}		
	}
	else
	{
		g_i2c_error_count = 0;
        g_tx_recovery_process = 0;
	}	
	return ret;
}

int sp_tx_hardware_power_set(int on)
{
	struct regulator *anx7808_v10;
	int ret = 0;
	
	printk("+++ sp_tx_hardware_power_set (%d)\n", on);
	anx7808_v10=regulator_get(NULL,"8921_l12");
	 if (IS_ERR(anx7808_v10)) {
		printk("unable to get anx7808_v10\n");
		return PTR_ERR(anx7808_v10);
	}
      ret = regulator_set_voltage(anx7808_v10, 1000000, 1000000);
	if (ret) {
		printk("%s: unable to set the voltage for regulator "
			"anx7808_v10\n", __func__);
		return ret;
	}

	if (on)
		ret= regulator_enable(anx7808_v10);
	else	
		ret = regulator_disable(anx7808_v10);

	printk("--- sp_tx_hardware_power_set (%d)\n", ret);
	
	return ret;	
}

void sp_tx_hardware_poweron(struct i2c_client *client)
{
	struct anx7808_platform_data *pdata = client->dev.platform_data;
	gpio_set_value(pdata->gpio_reset, 0);
	msleep(2);

#ifdef CONFIG_EEPROM_NUVOTON
	if (!AX_MicroP_IsP01Connected() || !sp_tx_asus_pad) {
		printk("[myDP] switch to myDP\n");
		gpio_set_value(pdata->gpio_usb_select, 1);
		msleep(2);		
	}
#else	
	printk("[myDP] switch to myDP\n");
	gpio_set_value(pdata->gpio_usb_select, 1);
	msleep(2);
#endif	

	
	gpio_set_value(pdata->gpio_p_dwn, 0);
	msleep(2);
	//gpio_set_value(pdata->gpio_v10_ctrl, 1);
	sp_tx_hardware_power_set(1);
	mdelay(20);
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(2);

        g_i2c_error_count = 0;  
        g_tx_recovery_process = 0; 
	sp_tx_pd_mode = 0;
	DEV_DBG("%s: anx7808 power on\n", __func__);
}

void sp_tx_hardware_powerdown(struct i2c_client *client)
{

	struct anx7808_platform_data *pdata = client->dev.platform_data;
	//return;
	gpio_set_value(pdata->gpio_reset, 0);
	msleep(2);
	//gpio_set_value(pdata->gpio_v10_ctrl, 0);
	sp_tx_hardware_power_set(0);	
	msleep(5);

	gpio_set_value(pdata->gpio_p_dwn, 1);
	msleep(2);


#ifdef CONFIG_EEPROM_NUVOTON
	if (!AX_MicroP_IsP01Connected() || !sp_tx_asus_pad) {
		printk("[myDP] switch to USB\n");		
		gpio_set_value(pdata->gpio_usb_select, 0);
	}
#else	
	printk("[myDP] switch to USB\n");
	gpio_set_value(pdata->gpio_usb_select, 0);
#endif	

	g_i2c_error_count = 0;
	g_tx_recovery_process = 0;
	sp_tx_pd_mode = 1;
	DEV_DBG("%s: anx7808 power down\n", __func__);
}

void sp_tx_hardware_recovery(struct i2c_client *client)
{

       struct anx7808_platform_data *pdata = client->dev.platform_data;
       //return;

       gpio_set_value(pdata->gpio_reset, 0);
       msleep(20);
       //gpio_set_value(pdata->gpio_v10_ctrl, 0);
       //msleep(5);
       gpio_set_value(pdata->gpio_p_dwn, 1);
       msleep(20);
 
        DEV_DBG("%s: anx7808 power recovery\n", __func__);
 }

void myDP_tx_hardware_recovery(void)
{
   g_tx_recovery_process = 1;
   sp_tx_hardware_recovery(anx7808_client);
}
//ASUS_BSP+++ larry : check 480p for VGA dongle
int myDP_cable_type(void)
{
	return sp_tx_rx_type;
}
//ASUS_BSP+++ larry : check 480p for VGA dongle
#ifndef EYE_TEST
extern void nv_touch_mode(int); //ASUS_BSP HANS: temporary way to notify usb state ++
static void slimport_cable_plug_proc(struct anx7808_data *anx7808)
{
	int Pad_HW_ID = -1;
#ifdef CONFIG_EEPROM_NUVOTON_A80	
	int MicroP_tryCount=5;
#endif	
	pr_debug("+++++++++++++slimport_cable_plug_proc+++++++++++++++++++\n");
	if (gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det) ) {
		msleep(50);
		if (gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det) ) {
			pr_debug("slimport_cable_plug_proc start\n");
			if (sp_tx_pd_mode) {
				pr_debug("=================slimport_cable_plug_proc (1) ===============\n");
//ASUS_BSP +++ : larry lai for pad solution
				if(sp_tx_get_asus_id()) 
				{
#ifdef CONFIG_EEPROM_NUVOTON_A80	
						while(MicroP_tryCount--){
							Pad_HW_ID = AX_MicroP_IsMydpNewSKU();
							if(Pad_HW_ID==-1)
								msleep(100);
							else 
								break;
						}
							
#else
					Pad_HW_ID = -1;  // default TV mode
#endif
					if (Pad_HW_ID == 1)
					{
						printk("### [myDP] DP Pad detect ###\n");				
						sp_tx_asus_pad = 1;	
						myDP_DP_Dongle = 1;
					}
					else if (Pad_HW_ID == 0)
					{
						printk("### [myDP] HDMI Pad detect ###\n");				
						sp_tx_asus_pad = 1;
						myDP_DP_Dongle = 0;
					}
					else
					{
						if (myDP_force_pad_mode)
						{
							if (myDP_DP_Dongle)
							{
								printk("[myDP] DP Pad detect ###\n");				
								sp_tx_asus_pad = 1;	
								myDP_DP_Dongle = 1;
							}
							else
							{
								printk("[myDP] HDMI Pad detect ###\n");				
								sp_tx_asus_pad = 1;
								myDP_DP_Dongle = 0;							
							}
						}
						else
						{
							printk("### [myDP] Fail detect Pad , force TV mode ###\n");									
							sp_tx_asus_pad = 0;
							myDP_DP_Dongle = 0;											
						}
					}
				}
				else{
					nv_touch_mode(4);	//ASUS_BSP Deeo : notify touch while plug out HDMI 4: AC
				}

//ASUS_BSP --- : larry lai for pad solution

				//sp_tx_hardware_chip_enable(anx7808_client);
				sp_tx_hardware_poweron(anx7808_client);

				sp_tx_pd_mode = 0;

//ANX : (ver:20130105) diff with ANX slimport driver, comment it ??? 			
////				msleep(200);
				sp_tx_power_on(SP_TX_PWR_REG);
				sp_tx_power_on(SP_TX_PWR_TOTAL);
				pr_debug("=================slimport_cable_plug_proc (2) ===============\n");
//ANX : (ver:0.2)
				sp_tx_pull_down_id(TRUE);

				hdmi_rx_initialization();
				pr_debug("=================slimport_cable_plug_proc (3) ===============\n");
				sp_tx_initialization();
				pr_debug("=================slimport_cable_plug_proc (4) ===============\n");
//ASUS_BSP +++ : larry lai for pad solution
				if (!sp_tx_asus_pad)
				{
					sp_tx_vbus_poweron();
					msleep(200);
				}
				else
				{
					msleep(20);					
				}
//ASUS_BSP --- : larry lai for pad solution				
				if (!sp_tx_get_cable_type()) {
					DEV_ERR("%s:AUX ERR\n", __func__);
					sp_tx_vbus_powerdown();
//ANX : (ver:0.2)					
					sp_tx_pull_down_id(FALSE);
					sp_tx_power_down(SP_TX_PWR_REG);
					sp_tx_power_down(SP_TX_PWR_TOTAL);
//					sp_tx_hardware_chip_disable(anx7808_client);
					sp_tx_hardware_powerdown(anx7808_client);
					sp_tx_pd_mode = 1;
					sp_tx_link_config_done = 0;
					sp_tx_hw_lt_enable = 0;
					sp_tx_hw_lt_done = 0;
					sp_tx_rx_type = RX_NULL;
					sp_tx_rx_type_backup = RX_NULL;
//ANX +++: (ver:20130105) pad solution
					sp_tx_asus_pad = 0;
//ANX ---: (ver:20130105) pad solution
					sp_tx_set_sys_state(STATE_CABLE_PLUG);
					return;
				}
				pr_debug("=================slimport_cable_plug_proc (5) ===============\n");
				sp_tx_rx_type_backup = sp_tx_rx_type;
			}
			switch(sp_tx_rx_type) {
			case RX_HDMI:
				pr_debug("=================slimport_cable_plug_proc (RX_HDMI) ===============\n");
				if(sp_tx_get_hdmi_connection()){
					printk("==== (RX_HDMI) ===\n");
//ANX +++: (ver:20130105) pad solution					
					if(sp_tx_asus_pad) {
						//skip EDID read
						hdmi_rx_set_hpd(1);
						hdmi_rx_set_termination(1);
						sp_tx_set_sys_state(STATE_CONFIG_HDMI);
					} else {
						sp_tx_set_sys_state(STATE_PARSE_EDID);
					}
//ANX ---: (ver:20130105) pad solution					
					}
				break;
			case RX_DP:
				if(sp_tx_get_dp_connection())
				{
					printk("==== (RX_DP) ===\n");				
					if(sp_tx_asus_pad) {
						//skip EDID read
						hdmi_rx_set_hpd(1);
						hdmi_rx_set_termination(1);
						sp_tx_set_sys_state(STATE_CONFIG_HDMI);
					} else {				
						sp_tx_set_sys_state(STATE_PARSE_EDID);
					}
				}
				break;
			case RX_VGA:
				if(sp_tx_get_vga_connection()){
					sp_tx_send_message(MSG_CLEAR_IRQ); 
					sp_tx_set_sys_state(STATE_PARSE_EDID);
				}
				break;
			case RX_NULL:
			default:
				break;
			}
		}
	} else if (sp_tx_pd_mode == 0) {
		sp_tx_vbus_powerdown();
//ANX : (ver:0.2)		
		sp_tx_pull_down_id(FALSE);
		sp_tx_power_down(SP_TX_PWR_REG);
		sp_tx_power_down(SP_TX_PWR_TOTAL);
		sp_tx_hardware_powerdown(anx7808_client);
//		sp_tx_hardware_chip_disable(anx7808_client);
		sp_tx_pd_mode = 1;
		sp_tx_link_config_done = 0;
		sp_tx_hw_lt_enable = 0;
		sp_tx_hw_lt_done = 0;
		sp_tx_rx_type = RX_NULL;
		sp_tx_rx_type_backup = RX_NULL;
//ANX +++: (ver:20130105) pad solution		
		sp_tx_asus_pad = 0;
//ANX ---: (ver:20130105) pad solution		
		sp_tx_set_sys_state(STATE_CABLE_PLUG);
	}
}
#endif

#ifndef EYE_TEST
static void slimport_edid_proc(void)
{
	sp_tx_aux_polling_enable(0); //+++ ASUS BSP Bernard: ANX 4.0
	sp_tx_edid_read();

	if (bedid_break)
		DEV_ERR("%s: EDID corruption!\n", __func__);
	sp_tx_aux_polling_enable(1); //+++ ASUS BSP Bernard: ANX 4.0
	hdmi_rx_set_hpd(1);
	hdmi_rx_set_termination(1);
	sp_tx_set_sys_state(STATE_CONFIG_HDMI);
}
#endif

//ANX +++: (ver:20130105) EDID read issue with QCOM solution
int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	if (block == 0) {
		memcpy(edid_buf, bedid_firstblock, sizeof(bedid_firstblock));
	} else if (block == 1) {
		memcpy(edid_buf, bedid_extblock, sizeof(bedid_extblock));
	} else {
		pr_err("%s: block number %d is invalid\n", __func__, block);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(slimport_read_edid_block);

unchar slimport_get_link_bw(void)
{
	return slimport_link_bw;
}
EXPORT_SYMBOL(slimport_get_link_bw);
//ANX ---: (ver:20130105) EDID read issue with QCOM solution

#ifndef EYE_TEST
static void slimport_config_output(void)
{
	sp_tx_clean_hdcp();
	sp_tx_set_colorspace();
	sp_tx_avi_setup();
	sp_tx_config_packets(AVI_PACKETS);
	sp_tx_enable_video_input(1);
	sp_tx_set_sys_state(STATE_HDCP_AUTH);
}

extern void sp_tx_show_infomation(void);

extern uint sp_tx_link_err_check_1(void);

static void slimport_playback_proc(void)
{
	unchar c1 = 0;
	unchar c=0;
	unchar c2=0;

	if (g_dump_7808_reg)
	{
	//0x7E, 8B
	
		dump_7808_reg_info();
//		dump_7808_reg_pclk();
		sp_tx_show_infomation();
	}
	
	if (g_dump_7730_reg)
	{
		sp_tx_link_err_check_1();

		sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_L, &c);
		sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_H, &c1);

		if(c1!=0x7||c!=0xd0)
		     printk("ANX7808 TX cur_h_res = 0x%x,0x%x\n", c,c1);

		sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_L, &c);
		sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_H, &c1);

		if(c1!=0x4||c!=0xd3)
		     printk("ANX7808 TX cur_v_res = 0x%x\n,0x%x\n", c,c1);


        if ( (sp_tx_asus_pad) && (!myDP_DP_Dongle) )
		{
//========================================	
// disable 7730 OCM

		i2c_master_write_reg(7,0xf0, 0);
		i2c_master_read_reg(7,0xf0, &c);

		if(c != 0 )
		    i2c_master_write_reg(7,0xf0, 0);
//========================================

		i2c_master_read_reg(0, 0xcb, &c1);															
		i2c_master_read_reg(1, 0xd1, &c);					
		//i2c_master_read_reg(0, 0x31, &c2);				
		
		i2c_master_write_reg(0, 0xcb, c1);					
		i2c_master_write_reg(1, 0xd1, c);					
	
		if ((c1 & 0x08) || (c & 0x80))
		{
			printk("[myDP] # 7730 video FIFO error , 0xcb= (%x), RX ISR6 = (%x)\n",  c1, c);
//			ASUSEvtlog("[myDP]7730 video FIFO error , 0xcb= (%x), RX ISR6 = (%x)\n", c1, c);
		}
		
//		printk("========== ANX7730 reg 0x50 DUMP ============\n");
/*
		for (i=0x20; i <=0x30; i++)
		{
			i2c_master_read_reg(0, i , &c);
			if  (i%0xf)
				printk("0x%x = (%x), ", i, c);										
			else	
				printk("0x%x = (%x)\n", i, c);										
						
		}
*/
		i2c_master_read_reg(0, 0x24 , &c1);
		i2c_master_read_reg(0, 0x25 , &c2);

		if ((c1 != 0x7) || (c2 != 0xd0))
		{
			printk("Fail 7730 0x50 H total = (%x, %x)", c1, c2);
		}

						
//		printk("========== ANX7730 reg 0x72 DUMP ============\n");
/*		
		for (i=0x24; i <=0x34; i++)
		{
			i2c_master_read_reg(5, i , &c);
			if  (i%0xf)
				printk("0x%x = (%x), ", i, c);										
			else	
				printk("0x%x = (%x)\n", i, c);										
		}
*/		
		i2c_master_read_reg(5, 0x2b , &c1);
		i2c_master_read_reg(5, 0x2c , &c2);

		if ((c1 != 0xd0) || (c2 != 0x7))
		{
			printk("Fail 7730 0x72 H total = (%x, %x)", c1, c2);				
		}
//		
//========================================
// re-enable 7730 OCM
		i2c_master_write_reg(7,0xf0, 0xe6);
		i2c_master_write_reg(7,0xf0, 0xe6);
//========================================		
		}
	}
}
#endif
//extern int check_mdp4_dtv(void);

#ifndef EYE_TEST
static void slimport_main_proc(struct anx7808_data *anx7808)
{
//unchar c1, c2;
pr_debug("=================slimport_main_proc================\n");
	mutex_lock(&anx7808->lock);
	//ASUS BSP Wei Lai +++
	//sp_tx_chip_located();
pr_debug("(1) slimport_main_proc \n");


	//sp_tx_hardware_poweron(anx7808_client);
	//msleep(200);
	//ASUS BSP Wei Lai ---
	if (!sp_tx_pd_mode) {
		pr_debug("(1-1)slimport_main_proc \n");
		
		sp_tx_int_irq_handler();
		hdmi_rx_int_irq_handler();
		pr_debug("(1-2)slimport_main_proc \n");

	}
	//sp_tx_hardware_chip_enable(anx7808_client);

pr_debug("=================slimport_main_proc END================\n");
pr_debug("(2)slimport_main_proc \n");
	if (sp_tx_system_state == STATE_CABLE_PLUG)
		slimport_cable_plug_proc(anx7808);
pr_debug("(3)slimport_main_proc \n");
	if (sp_tx_system_state == STATE_PARSE_EDID)
		slimport_edid_proc();
//ANX +++: (ver0.4)
	if (sp_tx_system_state == STATE_CONFIG_HDMI) {
// 		if((sp_tx_asus_pad) && (!myDP_DP_Dongle)) 
 		if(sp_tx_asus_pad) 
		{
			int try_count= 50;
			while (try_count) {
				if (!sp_tx_config_hdmi_input())
					break;
				else	
					msleep(20);
				try_count --;
			};

			printk("[myDP] try_count = %d\n", try_count);
			
			if ( (/*!check_mdp4_dtv() ||*/ !try_count) && (g_slimport_bypass_check_mdp4) )
			{
				printk("[myDP] after check RX input, still TMDS not ready, go link , then power down\n");
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
				
			}
			else
			{
				printk("[myDP] TMDS ready, go to next process\n");			
				g_slimport_bypass_check_mdp4 = 1;
			}
			if (sp_tx_system_state == STATE_LINK_TRAINING)
				sp_tx_sw_error_power_down();						
			else	
				sp_tx_config_hdmi_pad();
		}
		else 
		{
			if (sp_tx_rx_type == RX_VGA)
			{
				// check need under 480p
				if (myDP_check_7808_rx_under_480p())					
				{
					sp_tx_config_hdmi_input();
				}
			}
			else
			{
				sp_tx_config_hdmi_input();
			}
		}
	}
	if (sp_tx_system_state == STATE_LINK_TRAINING) {
		if (!sp_tx_asus_pad) {
			if (!sp_tx_lt_pre_config())
				sp_tx_hw_link_training();
		}
		else
		{// pad mode not do link training, so power down chip
			sp_tx_sw_error_power_down();			
		}
	}
//ANX ---: (ver0.4)
pr_debug("=================slimport_main_proc================\n");
pr_debug("=================slimport_main_proc END================\n");
	if (sp_tx_system_state == STATE_CONFIG_OUTPUT)
		slimport_config_output();

	if (sp_tx_system_state == STATE_HDCP_AUTH) {
		if (hdcp_enable && 
			((sp_tx_rx_type == RX_HDMI) ||
			( sp_tx_rx_type ==RX_DP) || ( sp_tx_rx_type ==RX_VGA)) ) {
//ANX +++: (ver:20130105) pad solution			
//			if((sp_tx_asus_pad) && (!myDP_DP_Dongle))
			if(sp_tx_asus_pad)
				sp_tx_sw_hdcp_process();
			else
				sp_tx_hdcp_process();
//ANX ---: (ver:20130105) pad solution
		} else {
			sp_tx_power_down(SP_TX_PWR_HDCP);
			sp_tx_video_mute(0);
			sp_tx_show_infomation();
			sp_tx_set_sys_state(STATE_PLAY_BACK);
		}
	}

	if (sp_tx_system_state == STATE_PLAY_BACK)
		slimport_playback_proc();
		//ASUS BSP Wei Lai +++
	//sp_tx_hardware_powerdown(anx7808_client);
		//ASUS BSP Wei Lai ---
	mutex_unlock(&anx7808->lock);
		pr_debug("=================slimport_main_proc END================\n");
}
#endif

static uint8_t anx7808_chip_detect(void)
{
	return sp_tx_chip_located();
}

#ifdef EYE_TEST
extern void sp_tx_eye_diagram_test(void);
#endif
static void anx7808_chip_initial(void)
{
#ifdef EYE_TEST
	sp_tx_eye_diagram_test();
#else
	sp_tx_variable_init();
	sp_tx_vbus_powerdown();
	sp_tx_hardware_powerdown(anx7808_client);

//ASUS_BSP +++: workaround for turn off 1.0V after read chip id
	msleep(5);
	sp_tx_hardware_power_set(0);
//ASUS_BSP ---: workaround for turn off 1.0V after read chip id
	
	sp_tx_set_sys_state(STATE_CABLE_PLUG);
#endif
}

static void anx7808_free_gpio(struct anx7808_data *anx7808)
{
	gpio_free(anx7808->pdata->gpio_cbl_det);
	gpio_free(anx7808->pdata->gpio_int);
	gpio_free(anx7808->pdata->gpio_reset);
	gpio_free(anx7808->pdata->gpio_p_dwn);
	gpio_free(anx7808->pdata->gpio_usb_select);
	gpio_free(anx7808->pdata->gpio_usb_id);		
}
static int anx7808_init_gpio(struct anx7808_data *anx7808)
{
	int ret = 0;

	DEV_DBG("anx7808 init gpio\n");



	ret = gpio_request(anx7808->pdata->gpio_usb_select, "anx7808_usb_select");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_usb_select);
		goto err_0;
	}
	gpio_direction_output(anx7808->pdata->gpio_usb_select, 0);

	ret = gpio_request(anx7808->pdata->gpio_usb_id, "anx7808_usb_id");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_usb_id);
		goto err;
	}
	gpio_direction_input(anx7808->pdata->gpio_usb_id);


	ret = gpio_request(anx7808->pdata->gpio_p_dwn, "anx_p_dwn_ctl_2");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_p_dwn);
		goto err0;
	}
	gpio_direction_output(anx7808->pdata->gpio_p_dwn, 1);


	ret = gpio_request(anx7808->pdata->gpio_reset, "anx7808_reset_n");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_reset);
		goto err1;
	}

	gpio_direction_output(anx7808->pdata->gpio_reset, 0);

	ret = gpio_request(anx7808->pdata->gpio_int, "anx7808_int_n");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_int);
		goto err2;
	}
	else
	{
		printk("[myDP] check INT pin = %d\n", anx7808->pdata->gpio_int);
	}
	gpio_direction_input(anx7808->pdata->gpio_int);
	
	ret = gpio_request(anx7808->pdata->gpio_cbl_det, "anx7808_cbl_det");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_cbl_det);
		goto err3;
	}
	gpio_direction_input(anx7808->pdata->gpio_cbl_det);

#if 0
	ret = gpio_request(anx7808->pdata->gpio_v10_ctrl, "anx7808_v10_ctrl");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_v10_ctrl);
		goto out;
	}
	gpio_direction_output(anx7808->pdata->gpio_v10_ctrl, 0);
#endif
	//gpio_set_value(anx7808->pdata->gpio_reset, 0);
	//gpio_set_value(anx7808->pdata->gpio_p_dwn, 1);
	goto out;

err3:
	gpio_free(anx7808->pdata->gpio_cbl_det);
err2:
	gpio_free(anx7808->pdata->gpio_int);
err1:
	gpio_free(anx7808->pdata->gpio_reset);
err0:
	gpio_free(anx7808->pdata->gpio_p_dwn);
err:
	gpio_free(anx7808->pdata->gpio_usb_id);
err_0:
	gpio_free(anx7808->pdata->gpio_usb_select);	
out:
	return ret;
}

static int  anx7808_system_init(void)
{
	int ret = 0;

	ret = anx7808_chip_detect();
	if (ret == 0) {
		DEV_ERR("%s : failed to detect anx7808\n", __func__);
		return -ENODEV;
	}

	anx7808_chip_initial();
	return 0;
}

static irqreturn_t anx7808_cbl_det_isr(int irq, void *data)
{
	//static int i=0;
	struct anx7808_data *anx7808 = (struct anx7808_data *)data;
	int status;
	int gpio_cbl_det = gpio_get_value(anx7808->pdata->gpio_cbl_det);
	printk("+++++++++++++++++anx7808_cbl_det_isr++++++++++++++ (gpio_cbl_det=%d,g_i2c_error_count=%d)\n",gpio_cbl_det,g_i2c_error_count);
	//disable_irq(irq);

	if ( gpio_cbl_det && (g_i2c_error_count<=20))  {
            // i=1;
		wake_lock(&anx7808->slimport_lock);
		printk("[myDP]%s:wake_lock(&anx7808->slimport_lock),detect cable insertion!!!\n",__func__);
		queue_delayed_work(anx7808->workqueue, &anx7808->work, 0);
	} else {
//ANX +++: (ver0.4)		
		status = cancel_delayed_work_sync(&anx7808->work);
		if(status == 0)
			flush_workqueue(anx7808 ->workqueue);
		wake_unlock(&anx7808->slimport_lock);
		printk("[myDP]%s:wake_unlock(&anx7808->slimport_lock),detect cable removal...\n",__func__);

//		nv_touch_mode(0);	//ASUS_BSP Deeo : notify touch while plug out HDMI 0:USB off
		g_hdmi_rx_vsync_change = 0; // clear hdmi rx vsync count variable

		//wake_lock_timeout(&anx7808->slimport_lock, 2*HZ);
//ANX ---: (ver0.4)		
	}
	//enable_irq(irq);
	return IRQ_HANDLED;
}

static void anx7808_work_func(struct work_struct *work)
{
#ifndef EYE_TEST
	struct anx7808_data *td = container_of(work, struct anx7808_data,
								work.work);
	pr_debug("##############anx7808_work_func++++++++++++++++++++\n");
	//return ;
	slimport_main_proc(td);

		pr_debug("queue_delayed_work for 300 ms\n");
	queue_delayed_work(td->workqueue, &td->work,
			msecs_to_jiffies(300));	
	pr_debug("##############anx7808_work_func--------------------------\n");
#endif
	
}

#if 1
static irqreturn_t dp_usb_id_detect_handler(int irq, void *dev_id){
	struct anx7808_platform_data *pdata = anx7808_client->dev.platform_data;
	struct anx7808_data *anx7808 = (struct anx7808_data *)dev_id;
	if(gpio_get_value(pdata->gpio_cbl_det)==1)goto exit;
	DEV_DBG("%s: ++++\n", __func__);
	if(gpio_get_value(pdata->gpio_usb_id)==0)
		queue_delayed_work(anx7808->workqueue, &anx7808->carKitwork, 50);
	else
		dp_switch_carkit(false);
		
exit:
	return IRQ_HANDLED;
}
#endif
static void anx7808_carKitwork_func(struct work_struct *work)
{
#ifndef EYE_TEST
	
	struct anx7808_data *td = container_of(work, struct anx7808_data,
		 						carKitwork.work);
	//printk("##############anx7808_carKitwork_func++++++++++++++++++++ %x,%d\n",(unsigned int)td,(unsigned int)td ->pdata);
	if (gpio_get_value_cansleep(td ->pdata->gpio_usb_id) ==0 ){
		msleep(50);
		if(gpio_get_value_cansleep(td ->pdata->gpio_cbl_det) ==0){
			dp_switch_carkit(true);
		}
	}
	
#endif
}

//ASUS_BSP larry lai :for ATD test +++
static struct class *myDPClass = 0;
static uint8_t g_myDP_init_status = 0;
struct device	 *g_myDP_pDevice = 0;			// pointer to the driver's device object
static int32_t 	 g_myDP_devMajor = 0;    /**< default device major number */
#define MYDP_DEVICE_NAME "anx7808"

ssize_t read_myDP_status(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int		status = 0;

	status = scnprintf(buf, PAGE_SIZE, "%s",
			g_myDP_init_status ? "1" : "0");

	return status;
}
/*
 * Sysfs attribute files supported by this driver.
 */ 
struct device_attribute mydp_driver_attribs[] = {
//ASUS_BSP larry lai :for ATD test +++		
		__ATTR(myDP_status, 0664, read_myDP_status, NULL),		
//ASUS_BSP larry lai :for ATD test ---		
		__ATTR_NULL
};
//ASUS_BSP larry lai :for ATD test ---


static int anx7808_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct anx7808_data *anx7808;
	struct regulator *anx7808_v10;
	int ret = 0;
	int irq=0;
	printk("##########anx7808_i2c_probe###################\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		DEV_ERR("%s: i2c bus does not support the anx7808\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	anx7808 = kzalloc(sizeof(struct anx7808_data), GFP_KERNEL);
	if (!anx7808) {
		DEV_ERR("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
	g_anx7808=anx7808;
	anx7808->pdata = client->dev.platform_data;

	memcpy(&anx7808_client, &client, sizeof(client));
	
	mutex_init(&anx7808->lock);
	if (!anx7808->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	// Early wake lock init before usage by ISR
	wake_lock_init(&anx7808->slimport_lock, WAKE_LOCK_SUSPEND, "slimport_wake_lock");

// ASUS_BSP +++ Tingyi "[A80][HDMI] Avoid dev_uevent+0x17c/0x270 data abort"
if(1)
{
	myDPClass = class_create(THIS_MODULE, "myDP");
	if (IS_ERR(myDPClass)) {
		pr_info("myDP class_create failed %d\n", ret);
	      ret = PTR_ERR(myDPClass);
		goto err0;
	}

      myDPClass->dev_attrs = mydp_driver_attribs;

      g_myDP_pDevice  = device_create(myDPClass, NULL,
    									 MKDEV(g_myDP_devMajor, 0),  NULL,
    									 "%s", MYDP_DEVICE_NAME);
    if (IS_ERR(g_myDP_pDevice)) {
    	 pr_info("myDP class_device_create failed %s %d\n", MYDP_DEVICE_NAME, ret);
        ret = PTR_ERR(g_myDP_pDevice);
	  goto free_class;
    }
}
// ASUS_BSP --- Tingyi "[A80][HDMI] Avoid dev_uevent+0x17c/0x270 data abort"

	ret = anx7808_init_gpio(anx7808);
	if (ret) {
		DEV_ERR("%s: failed to initialize gpio\n", __func__);
		goto err1;
	}

	anx7808_v10=regulator_get(NULL,"8921_l12");
	 if (IS_ERR(anx7808_v10)) {
		printk("unable to get anx7808_v10\n");
		return PTR_ERR(anx7808_v10);
	}
      ret = regulator_set_voltage(anx7808_v10, 1000000, 1000000);
	if (ret) {
		printk("%s: unable to set the voltage for regulator "
			"anx7808_v10\n", __func__);
		return ret;
	}
	regulator_enable(anx7808_v10);
	
//ASUS BSP wei lai +++
#if 1
	irq = MSM_GPIO_TO_INT(anx7808->pdata->gpio_usb_id);
	if (irq < 0) {
		printk( "%s: could not get USB_ID_DETECT IRQ resource, error=%d ", __func__, irq);		
	}
	ret = request_irq(irq, dp_usb_id_detect_handler,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING , "dp usb id mode", anx7808);

	if (ret < 0) {
		printk( "%s: FACTORY USB IRQ#%d request failed with error=%d ", __func__, irq, ret);				
	}
	disable_irq(irq);
#endif
//ASUS BSP wei lai ---

	INIT_DELAYED_WORK(&anx7808->work, anx7808_work_func);
//ASUS BSP wei lai +++
	INIT_DELAYED_WORK(&anx7808->carKitwork, anx7808_carKitwork_func);
//ASUS BSP wei lai ---
	anx7808->workqueue = create_singlethread_workqueue("anx7808_work");
	if (anx7808->workqueue == NULL) {
		DEV_ERR("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err2;
	}
//ASUS BSP wei lai +++
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend( &dp7808_early_suspend_desc );
#endif
//ASUS BSP wei lai ---
	ret = anx7808_system_init();
	if (ret) {
		DEV_ERR("%s: failed to initialize anx7808\n", __func__);
		goto err2;
	}
//ASUS_BSP larry lai :for ATD test +++	
	g_myDP_init_status = 1;
//ASUS_BSP larry lai :for ATD test ---

	client->irq = gpio_to_irq(anx7808->pdata->gpio_cbl_det);
	if (client->irq < 0) {
		DEV_ERR("%s : failed to get gpio irq\n", __func__);
		goto err3;
	}

	ret = request_threaded_irq(client->irq, NULL, anx7808_cbl_det_isr,
					IRQF_TRIGGER_RISING
					| IRQF_TRIGGER_FALLING,
					"anx7808_cabel_det", anx7808);
	if (ret  < 0) {
		DEV_ERR("%s : failed to request irq\n", __func__);
		goto err3;
	}
//wei debug irq
#if 1
//	ret = set_irq_wake(client->irq, 1);
//ASUS Wei_Lai +++
	ret = irq_set_irq_wake(client->irq, 1);
//ASUS Wei_Lai ---
	if (ret  < 0) {
		pr_err("%s : Request irq for cable detect"
			"interrupt wake set fail\n", __func__);
		goto err3;
	}
#endif
//wei debug irq
#if 1
	ret = enable_irq_wake(client->irq);
	if (ret  < 0) {
		DEV_ERR("%s : Enable irq for cable detect", __func__);
		DEV_ERR("interrupt wake enable fail\n");
		goto err3;
	}
#endif	

	//enable_irq(client->irq);
//ASUS BSP wei lai +++
	enable_irq(irq);
	i2c_set_clientdata(anx7808_client,anx7808);

	printk("########## #####anx7808_i2c_probe END###################\n");

	if (gpio_get_value(anx7808->pdata->gpio_cbl_det) && (g_i2c_error_count==0))  {
		msleep(10);
		if(gpio_get_value(anx7808->pdata->gpio_cbl_det)==1){
			wake_lock(&anx7808->slimport_lock);
			printk("[myDP]%s:wake_lock(&anx7808->slimport_lock),detect cable insertion\n",__func__);
			queue_delayed_work(anx7808->workqueue, &anx7808->work, 1000);
		}
	}
	if (gpio_get_value_cansleep(anx7808 ->pdata->gpio_usb_id) ==0 ){
		if(gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det) ==0){
			g_otg_state=true;
		}
	}
	
//ASUS BSP wei lai ---
	goto exit;

err3:
	free_irq(client->irq, anx7808);
err2:
	destroy_workqueue(anx7808->workqueue);
err1:
	anx7808_free_gpio(anx7808);
free_class:
	if (myDPClass) class_destroy(myDPClass);
err0:
	kfree(anx7808);
exit:
	return ret;
}

// Tingyi +++
bool slimport_is_connected(void)
{
	struct anx7808_platform_data *pdata = NULL;
	bool result = false;

	if (!anx7808_client)
		return false;

	pdata = anx7808_client->dev.platform_data;
	if (!pdata)
		return false;

//	spin_lock(&pdata->lock);

	if (gpio_get_value_cansleep(pdata->gpio_cbl_det)) {
		mdelay(10);
		if (gpio_get_value_cansleep(pdata->gpio_cbl_det)) {
			pr_info("%s : Slimport Dongle is detected\n", __func__);
			result = true;
		}
	}

//	spin_unlock(&pdata->lock);

	return result;
}
EXPORT_SYMBOL(slimport_is_connected);

//unchar sp_get_link_bw(void)
//{
//	return slimport_link_bw;
//}
//EXPORT_SYMBOL(sp_get_link_bw);

void sp_set_link_bw(unchar link_bw)
{
	slimport_link_bw = link_bw;
}
EXPORT_SYMBOL(sp_set_link_bw);
// Tingyi ---

static int anx7808_i2c_remove(struct i2c_client *client)
{
	struct anx7808_data *anx7808 = i2c_get_clientdata(client);

	free_irq(client->irq, anx7808);
	anx7808_free_gpio(anx7808);
	destroy_workqueue(anx7808->workqueue);
	wake_lock_destroy(&anx7808->slimport_lock);
	kfree(anx7808);
	return 0;
}

static const struct i2c_device_id anx7808_id[] = {
	{ "anx7808_i2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, anx7808_id);

static struct i2c_driver anx7808_driver = {
	.driver  = {
		.name  = "anx7808_i2c",
		.owner  = THIS_MODULE,
	},
	.probe  = anx7808_i2c_probe,
	.remove  = anx7808_i2c_remove,
	.id_table  = anx7808_id,
#ifdef ANX7808_PM_SUPPORT	
      .suspend = anx7808_suspend,
      .resume = anx7808_resume,	
#endif	  
};

static int __init anx7808_init(void)
{
	int ret = -1;

//	if(g_A68_hwID < A80_SR3)
//		goto exit;  // MHL solution, so exit myDP driver

	ret = i2c_add_driver(&anx7808_driver);
	if (ret < 0)
		DEV_ERR("%s: failed to register anx7808 i2c drivern", __func__);
//exit:	
	return ret;
}

static void __exit anx7808_exit(void)
{
	i2c_del_driver(&anx7808_driver);
}

module_init(anx7808_init);
module_exit(anx7808_exit);

MODULE_DESCRIPTION("Slimport  transmitter ANX7808 driver");
MODULE_AUTHOR("FeiWang <fwang@analogixsemi.com>");
MODULE_LICENSE("GPL");


