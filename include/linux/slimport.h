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
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#ifndef _SLIMPORT_H
#define _SLIMPORT_H

#ifdef DEBUG
#define DEV_DBG(args... )  pr_info("[myDP]"args)
#else
#define DEV_DBG(args... ) (void)0
#endif

#define DEV_NOTICE(args... ) pr_notice(args)
#define DEV_ERR(args... ) pr_err(args)


#define SSC_EN
//#define HDCP_EN

#if 0
#define SSC_1
#define EYE_TEST
#define EDID_DEBUG_PRINT
#endif

#define AUX_ERR  1
#define AUX_OK   0

extern bool  sp_tx_hw_lt_done;
extern bool  sp_tx_hw_lt_enable;
extern bool	sp_tx_link_config_done ;
extern enum SP_TX_System_State sp_tx_system_state;
extern enum RX_CBL_TYPE sp_tx_rx_type;
extern enum RX_CBL_TYPE  sp_tx_rx_type_backup;
extern unchar sp_tx_pd_mode;
//ANX +++: (ver:20130105) EDID read issue with QCOM solution
extern bool sp_tx_asus_pad;
//ANX ---: (ver:20130105) EDID read issue with QCOM solution
extern unchar bedid_break;
extern struct i2c_client *anx7808_client;

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf);
int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value);
void sp_tx_hardware_poweron(struct i2c_client *client);
void sp_tx_hardware_powerdown(struct i2c_client *client);
//ANX +++: (ver:20130105) pad solution
int slimport_read_edid_block(int block, uint8_t *edid_buf);
unchar slimport_get_link_bw(void);
//ANX +++: (ver:20130105) pad solution

//ASUS BSP Wei_Lai +++
/* GPIO parameters */
/* direction */
#define	PM_GPIO_DIR_OUT			0x01
#define	PM_GPIO_DIR_IN			0x02
#define	PM_GPIO_DIR_BOTH		(PM_GPIO_DIR_OUT | PM_GPIO_DIR_IN)

/* output_buffer */
#define	PM_GPIO_OUT_BUF_OPEN_DRAIN	1
#define	PM_GPIO_OUT_BUF_CMOS		0

/* pull */
#define	PM_GPIO_PULL_UP_30		0
#define	PM_GPIO_PULL_UP_1P5		1
#define	PM_GPIO_PULL_UP_31P5		2
#define	PM_GPIO_PULL_UP_1P5_30		3
#define	PM_GPIO_PULL_DN			4
#define	PM_GPIO_PULL_NO			5

/* vin_sel: Voltage Input Select */
#define	PM_GPIO_VIN_VPH			0 /* 3v ~ 4.4v */
#define	PM_GPIO_VIN_BB			1 /* ~3.3v */
#define	PM_GPIO_VIN_S4			2 /* 1.8v */
#define	PM_GPIO_VIN_L15			3
#define	PM_GPIO_VIN_L4			4
#define	PM_GPIO_VIN_L3			5
#define	PM_GPIO_VIN_L17			6

/* vin_sel: Voltage Input select on PM8058 */
#define PM8058_GPIO_VIN_VPH		0
#define PM8058_GPIO_VIN_BB		1
#define PM8058_GPIO_VIN_S3		2
#define PM8058_GPIO_VIN_L3		3
#define PM8058_GPIO_VIN_L7		4
#define PM8058_GPIO_VIN_L6		5
#define PM8058_GPIO_VIN_L5		6
#define PM8058_GPIO_VIN_L2		7

/* vin_sel: Voltage Input Select on PM8038*/
#define PM8038_GPIO_VIN_VPH		0
#define PM8038_GPIO_VIN_BB		1
#define PM8038_GPIO_VIN_L11		2
#define PM8038_GPIO_VIN_L15		3
#define PM8038_GPIO_VIN_L4		4
#define PM8038_GPIO_VIN_L3		5
#define PM8038_GPIO_VIN_L17		6

/* out_strength */
#define	PM_GPIO_STRENGTH_NO		0
#define	PM_GPIO_STRENGTH_HIGH		1
#define	PM_GPIO_STRENGTH_MED		2
#define	PM_GPIO_STRENGTH_LOW		3

/* function */
#define	PM_GPIO_FUNC_NORMAL		0
#define	PM_GPIO_FUNC_PAIRED		1
#define	PM_GPIO_FUNC_1			2
#define	PM_GPIO_FUNC_2			3
#define	PM_GPIO_DTEST1			4
#define	PM_GPIO_DTEST2			5
#define	PM_GPIO_DTEST3			6
#define	PM_GPIO_DTEST4			7

 struct pm_gpio {
	int		direction;
	int		output_buffer;
	int		output_value;
	int		pull;
	int		vin_sel;
	int		out_strength;
	int		function;
	int		inv_int_pol;
	int		disable_pin;
};

struct anx7808_data {
	struct anx7808_platform_data    *pdata;
	struct delayed_work    work;
//ASUS BSP wei lai +++
	struct delayed_work    carKitwork;
//ASUS BSP wei lai ---
	struct workqueue_struct    *workqueue;
	struct mutex    lock;
	struct wake_lock slimport_lock;
};
extern int pm8xxx_gpio_config(int gpio, struct pm_gpio *param);
//ASUS BSP Wei_Lai ---


#ifdef CONFIG_SLIMPORT_ANX7808
bool slimport_is_connected(void);
unchar sp_get_link_bw(void);
void sp_set_link_bw(unchar link_bw);
#else
static inline bool slimport_is_connected(void)
{
	return false;
}
static inline unchar sp_get_link_bw(void)
{
	return 0;
}
static inline void sp_set_link_bw(unchar link_bw)
{
	return;
}
#endif

#endif
