#ifndef __a68_er2_GPIO_PINMUX_H
#define __a68_er2_GPIO_PINMUX_H

//#include "a68_gpio_pinmux_setting.h"
#include "a80_gpio_pinmux_setting.h"

static struct msm_gpiomux_config a68_er2_msm8960_gpio_configs[] = {
        {
                .gpio = 0,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,//Mickey
                        [GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,//Mickey
                },
        },
        {       //GP1_NFC_IRQ
                .gpio = 1,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_NP_cfg,
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //C-PEN_IRQ_N
                .gpio = 2,
                .settings = {
                        [GPIOMUX_ACTIVE] = &I_NP_cfg,
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //PROXM_PWR_EN
                .gpio = 3,
                .settings = {
                        [GPIOMUX_ACTIVE] = &cm3623_pwr_en,
                        [GPIOMUX_SUSPENDED] = &cm3623_pwr_en,
                },
        },
        {       //SMB346_EN_N
                .gpio = 4,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &O_L_cfg,
                        [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },

//ASUS_BSP +++ Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
		{
				.gpio = 5,	//A68: FUNC1: CAM_MCLK0
				.settings = {
					[GPIOMUX_ACTIVE]    = &cam_settings[1],
					[GPIOMUX_SUSPENDED] = &cam_settings[0],
				},
		},
//ASUS_BSP --- Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
        {
                .gpio = 6,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &hs_detect,
                },
        },
        {       //P03_IRQ_N
                .gpio = 7,
                .settings = {
                        [GPIOMUX_ACTIVE] = &microp_intr_cfg,
                        [GPIOMUX_SUSPENDED] = &microp_intr_cfg,
                },
        },
        {
                .gpio      = 8,                 /* GSBI3 I2C QUP SDA */
                .settings = {
                        [GPIOMUX_SUSPENDED] = &gsbi3_suspended,
                        [GPIOMUX_ACTIVE] = &gsbi3_active,
                },
        },
        {
                .gpio      = 9,                 /* GSBI3 I2C QUP SCL */
                .settings = {
                        [GPIOMUX_SUSPENDED] = &gsbi3_suspended,
                        [GPIOMUX_ACTIVE] = &gsbi3_active,
                },
        },
        {
                .gpio      = 10,                 /* GSBI4 I2C SDA */
                .settings = {
                        [GPIOMUX_SUSPENDED] = &gsbi4_suspended,
                        [GPIOMUX_ACTIVE] = &gsbi4_sda_active,
                },
        },
        {
                .gpio      = 11,                 /* GSBI4 I2C SCL */
                .settings = {
                        [GPIOMUX_SUSPENDED] = &gsbi4_suspended,
                        [GPIOMUX_ACTIVE] = &gsbi4_scl_active,
                },
        },
//joe1_++
        {
                .gpio = 12, //TS INTERRUPT: a68_SR1_2=TP_IRQ_N
                .settings = {
                        [GPIOMUX_ACTIVE]    = &ts_int_cfg,
                        [GPIOMUX_SUSPENDED] = &ts_int_cfg,
                },
        },
//joe1_--
        {     //APQ2MDM_IPC2
/*	        .gpio = 13,
	        .settings = {
                        [GPIOMUX_ACTIVE]    = &cam_settings[2],
                        [GPIOMUX_SUSPENDED] = &cam_settings[0],
	        },*/
        },
        {       //FM_SSBI
                .gpio = 14,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &fm_ssbi_active,
                        [GPIOMUX_SUSPENDED] = &fm_ssbi_suspended,
                },
        },
        {       //FM_DATA
                .gpio = 15,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &fm_sdi_active,
                        [GPIOMUX_SUSPENDED] = &fm_sdi_suspended,
                },
        },
	{       //BT_CTL
/*		.gpio      = 16,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3,
		},*/
	},
	{       //BT_DATA
/*		.gpio      = 17,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3,
		},*/
	},
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        {       //AP2MDM_ERR_FATAL
                .gpio = 18,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
                },
        },
        {       //MDM2AP_ERR_FATAL
                .gpio = 19,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &mdm2ap_errfatal_cfg,
                },
        },
        //ASUS_BSP--- BennyCheng "config apq-mdm gpio default settings"
        {
                .gpio      = 20,
                .settings = {
                        [GPIOMUX_ACTIVE] = &gsbi1_active,
                        [GPIOMUX_SUSPENDED] = &gsbi1_suspended,
                },
        },
        {
                .gpio      = 21,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &gsbi1_suspended,
                        [GPIOMUX_ACTIVE] = &gsbi1_active,
                },
        },
        {       //SMB346_IRQ_N
                .gpio = 22,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_NP_cfg,
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //SMB346_INOK_N
                .gpio = 23,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_NP_cfg,
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {
                .gpio      = 24,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &gsbi2_suspended,
                        [GPIOMUX_ACTIVE] = &gsbi2_active,
                },
        },
        {
                .gpio      = 25,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &gsbi2_suspended,
                        [GPIOMUX_ACTIVE] = &gsbi2_active,
                },
        },
        {       //AP_PWR_KEY_N
                .gpio = 26,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &gpio_power_keys_cfg,
                        [GPIOMUX_SUSPENDED] = &NC_cfg,//ASUS BSP Austin
                },
        },
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        {       //AP2MDM_SOFT_RESET, aka AP2MDM_PON_RESET_N
                .gpio = 27,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &ap2mdm_soft_reset_cfg,
                },
        },
        //ASUS_BSP--- BennyCheng "config apq-mdm gpio default settings"
        {       //GYRO_INT_N
                .gpio = 28,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_NP_cfg,
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //BATT_LOW_N
                .gpio = 29,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_PU_cfg,
                        [GPIOMUX_SUSPENDED] = &I_PU_cfg,
                },
        },
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        /*
        {       //AP2MDM_VDDMIN
                .gpio = 30,
                .settings = {
                        [GPIOMUX_ACTIVE] = &NC_cfg,
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        */
        //ASUS_BSP--- BennyCheng "config apq-mdm gpio default settings"
//ASUS_BSP +++ Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
		{
				.gpio = 31,	//A68: ISP_INT
				.settings = {
                    [GPIOMUX_ACTIVE]    = &cam_settings[9],
                    [GPIOMUX_SUSPENDED] = &cam_settings[9],
				},
		},
//ASUS_BSP --- Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
        {       //MHL_IRQ_N
                .gpio = 32,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_NP_cfg,
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        /*
        {
                .gpio = 33,
                .settings = {
                        [GPIOMUX_ACTIVE] = &NC_cfg,
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        */
//ASUS_BSP +++ Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
		{
				.gpio = 34,	//A68: ISP_SUSPEND
				.settings = {
                    [GPIOMUX_ACTIVE]    = &cam_settings[9],
                    [GPIOMUX_SUSPENDED] = &cam_settings[9],
				},
		},
//ASUS_BSP --- Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        {       //AP2MDM_WAKEUP
                .gpio = 35,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &ap2mdm_wakeup,
                },
        },
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        {       //FM_I2S_WS
                .gpio = 36,
                .settings = {
                    //Mickey+++
                    [GPIOMUX_ACTIVE] = &I_NP_cfg,
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                    //Mickey---
                },
        },
//ASUS_BSP: joe1_++
        {       //C-PEN_RST_N
                .gpio = 37,
                .settings = {
                        [GPIOMUX_ACTIVE] = &O_H_cfg,
                        [GPIOMUX_SUSPENDED] = &O_H_cfg,
                },
        },
//ASUS_BSP: joe1_++
        {       //PROXM_OUT
                .gpio = 38,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &cm3623_interrupt,
                },
        },
        {       //SLIMBUS1_MCLK
                .gpio = 39,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &cdc_mclk,
                },
        },
        {       //SLIMBUS1_CLK
                .gpio = 40,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &slimbus,
                },
        },
        {       //SLIMBUS1_DATA
                .gpio = 41,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &slimbus,
                },
        },
        {       //CODEC_MAD_INT_N
                .gpio = 42,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
//ASUS_BSP larry lai : for MHL Reset setting +++        
        {       //MHL RST 
                .gpio = 43,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },
//ASUS_BSP larry lai : for MHL Reset setting ---        
        {    //BT_PCM_IN
                .gpio      = 44,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &bt_pcm,
                },
        },
        //ASUS_BSP+++ BennyCheng "default set APQ_MDM_SW_SEL to low for switching to MDM USB"
        {       //APQ_MDM_SW_SEL
                .gpio      = 45,
                .settings = {
                        [GPIOMUX_ACTIVE] = &O_L_cfg,
                        [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },
        //ASUS_BSP--- BennyCheng "default set APQ_MDM_SW_SEL to low for switching to MDM USB"
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        {       //MDM2AP_PBL_READY
                .gpio = 46,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &mdm2ap_pblrdy,
                },
        },
        //ASUS_BSP--- BennyCheng "config apq-mdm gpio default settings"
        //ASUS_BSP+++ BennyCheng "config hsic gpio default settings"
        {       //AP2MDM_SOFT_RESET
                .gpio = 47,
                .settings = {
                        [GPIOMUX_ACTIVE] = &hsic_wakeup_act_cfg,
                        [GPIOMUX_SUSPENDED] = &hsic_wakeup_sus_cfg,
                },
        },
        //ASUS_BSP--- BennyCheng "config hsic gpio default settings"
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        {       //AP2MDM_STATUS
                .gpio = 48,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
                },
        },
        {       //MDM2AP_STATUS
                .gpio = 49,
                .settings = {
                        [GPIOMUX_ACTIVE] = &mdm2ap_status_cfg,
                        [GPIOMUX_SUSPENDED] = &mdm2ap_status_cfg,
                },
        },
        //ASUS_BSP--- BennyCheng "config apq-mdm gpio default settings"
        {    //NFC_VEN
                .gpio = 50,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },
        {    //LCD_RST_N
                .gpio = 51,
                .settings = {
                    [GPIOMUX_ACTIVE] = &O_H_cfg,
                    [GPIOMUX_SUSPENDED] = &O_H_cfg,
                },
        },
        {       //TP_RST_N
                .gpio = 52,
                .settings = {
                    [GPIOMUX_ACTIVE] = &ts_reset_cfg,
                    [GPIOMUX_SUSPENDED] = &ts_reset_cfg,
                },
        },
        {       //BUT_VOL_UP
                .gpio = 53,
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_volup_keys_cfg,
                        [GPIOMUX_SUSPENDED] = &NC_cfg,//Austin
                },
        },
        {       //BUT_VOL_DOWN
                .gpio = 54,
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_voldown_keys_cfg,
                        [GPIOMUX_SUSPENDED] = &NC_cfg,//Austin
                },
        },
//ASUS_BSP +++ Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
		{
				.gpio = 55,	//A68: FLED_DRIVER_ENT
				.settings = {
                    [GPIOMUX_ACTIVE]    = &cam_settings[9],
                    [GPIOMUX_SUSPENDED] = &cam_settings[9],
				},
		},
//ASUS_BSP --- Stimber "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
        {       //C-PEN_DET
                .gpio = 56,
                .settings = {
                        [GPIOMUX_ACTIVE] = &I_NP_cfg, //ASUS_BSP: joe1_++
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //HW_ID0
                .gpio = 57,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {    //WCN_PRIORITY
/*                .gpio = 58,
                .settings = {
                    [GPIOMUX_ACTIVE]    = &,
                    [GPIOMUX_SUSPENDED] = &,
                },*/
        },
        {       //HW_ID1
                .gpio = 59,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
 	{       //MDM_LTE_FRAME_SYNC
//                 .gpio	= 60,
//                 .settings = {
//                         [GPIOMUX_SUSPENDED] = &slimbus,
//                 },
        },
        {       //MDM_LTE_ACTIVE
//                 .gpio	= 61,
//                 .settings = {
//                         [GPIOMUX_SUSPENDED] = &slimbus,
//                 },
        },
        {
                .gpio = 62,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &hs_button_detect,
                },
        },
        {    //BT_SSBI

                .gpio = 63,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //WL_CMD_DATA2
                .gpio = 64,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
                        [GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
                },
        },
        {       //WL_CMD_DATA1
                .gpio = 65,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
                        [GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
                },
        },
        {       //WL_CMD_DATA0
                .gpio = 66,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
                        [GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
                },
        },
        {       //WL_CMD_SET
                .gpio = 67,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
                        [GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
                },
        },
        {       //WL_CMD_CLK
                .gpio = 68,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
                        [GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
                },
        },
        {       //HW_ID2
                .gpio = 69,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //HDMI_DDC_SCL
                .gpio = 70,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
                        [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
                },
        },
        {    //HDMI_DDC_SDA
                .gpio = 71,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
                        [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
                },
        },

        {    //HDMI_INT
                .gpio = 72,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
                        [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
                },
        },

//         {    //PM8921_APC_SEC_IRQ_N
//                 .gpio      = 73,
//                 .settings = {
//                         [GPIOMUX_SUSPENDED] = &,
//                 },
//         },
//         {    //PM8921_APC_USR_IRQ_N
//                 .gpio      = 74,
//                 .settings = {
//                         [GPIOMUX_SUSPENDED] = &,
//                 },
//         },
//         {       //PM8921_MDM_IRQ_N
//                 .gpio = 75,
//                 .settings = {
//                     [GPIOMUX_SUSPENDED] = &,
//                 },
//         },
//         {    //PM8821_APC_SEC_IRQ_N
//                 .gpio = 76,
//                 .settings = {
//                     [GPIOMUX_SUSPENDED] = &,
//                 },
//         },
        //ASUS_BSP+++ BennyCheng "implement factory usb mode"
        {       //USB_HS_ID
                .gpio = 77,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &I_PU_cfg,
                },
        },
        //ASUS_BSP--- BennyCheng "implement factory usb mode"
//         {       //PS_HOLD
//                 .gpio = 78,
//                 .settings = {
//                     [GPIOMUX_SUSPENDED] = &NC_cfg,
//                 },
//         },
//         {       //SSBI_PM8821
//                 .gpio = 79,
//                 .settings = {
//                     [GPIOMUX_SUSPENDED] = &NC_cfg,
//                 },
//         },
        //ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
        /*
        {       //MDM2AP_VDDMIN
                .gpio = 80,
                .settings = {
                        [GPIOMUX_ACTIVE] = &NC_cfg,
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        */
        //ASUS_BSP--- BennyCheng "config apq-mdm gpio default settings"
//         {       //APQ2MDM_IPC1
//                 .gpio = 81,
//                 .settings = {
//                     [GPIOMUX_SUSPENDED] = &NC_cfg,
//                 },
//         },
        {       //UART_TXD_DBG
                .gpio = 82,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &gsbi7_func2,
                },
        },
        {       //UART_RXD_DBG
                .gpio = 83,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &gsbi7_func1,
                },
        },
        {       //APQ2MDM_IPC3
/*                .gpio = 84,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &,
                },*/
        },
        {       //ECOMP_RDY
                .gpio = 85,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //USB_SW_SEL
                .gpio = 86,
                .settings = {
                    [GPIOMUX_ACTIVE] = &O_L_cfg,
                    [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },
        {       //1.2M_MCLK_EN_N
                .gpio = 87,
                .settings = {
                    [GPIOMUX_ACTIVE] = &O_L_cfg,
                    [GPIOMUX_SUSPENDED] = &O_H_cfg,
                },
        },
        //ASUS_BSP+++ BennyCheng "config hsic gpio default settings"
        {       //HSIC_STROBE
                .gpio = 88,
                .settings = {
                        [GPIOMUX_ACTIVE] = &hsic_act_cfg,
                        [GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
                },
        },
        {       //HSIC_DATA
                .gpio = 89,
                .settings = {
                        [GPIOMUX_ACTIVE] = &hsic_act_cfg,
                        [GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
                },
        },
        //ASUS_BSP--- BennyCheng "config hsic gpio default settings"
};

#endif  /* __a68_ER2_GPIO_PINMUX_H  */
