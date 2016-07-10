#ifndef __A80_EVB_GPIO_SETTING_PINMUX_H__
#define __A80_EVB_GPIO_SETTING_PINMUX_H__

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <mach/gpiomux.h>

//ASUS_BSP +++ Stimber "[A68][8M][NA][Others]Full porting for 8M camera with ISP"
static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 1*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 2*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*active 3*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_5, /*active 4*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_6, /*active 5*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_2, /*active 6*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_3, /*active 7*/
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*i2c suspend*/
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

				{
			.func = GPIOMUX_FUNC_GPIO,
			.drv = GPIOMUX_DRV_2MA,
			.pull = GPIOMUX_PULL_NONE,
			.dir = GPIOMUX_OUT_LOW,
				},
				{
			.func = GPIOMUX_FUNC_GPIO,
			.drv = GPIOMUX_DRV_2MA,
			.pull = GPIOMUX_PULL_NONE,
			.dir = GPIOMUX_OUT_LOW,
				},
};
//ASUS_BSP --- Stimber "[A68][8M][NA][Others]Full porting for 8M camera with ISP"

static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

//Bruno:change SPK_AMP_EN gpio from PM8921 18,19 to msm8960 46
// static struct gpiomux_setting spk_amp_en = {
//     .func = GPIOMUX_FUNC_GPIO,
//     .drv = GPIOMUX_DRV_2MA,
//     .pull = GPIOMUX_PULL_NONE,
//     .dir = GPIOMUX_OUT_HIGH,
// };
// 
// static struct gpiomux_setting spk_amp_dis = {
//     .func = GPIOMUX_FUNC_GPIO,
//     .drv = GPIOMUX_DRV_2MA,
//     .pull = GPIOMUX_PULL_NONE,
//     .dir = GPIOMUX_OUT_LOW,
// };
//Bruno:change SPK_AMP_EN gpio from PM8921 18,19 to msm8960 46
//Rice: added for hs detect and button detect
// static struct gpiomux_setting mic2_bias_en = {
//     .func = GPIOMUX_FUNC_GPIO,
//     .drv = GPIOMUX_DRV_2MA,
//     .pull = GPIOMUX_PULL_NONE,
// };
// static struct gpiomux_setting hs_path_en = {
//     .func = GPIOMUX_FUNC_GPIO,
//     .drv = GPIOMUX_DRV_2MA,
//     .pull = GPIOMUX_PULL_NONE,
//     .dir = GPIOMUX_OUT_LOW,
// };
static struct gpiomux_setting hs_button_detect = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting hs_detect = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};

//++ Ledger
static struct gpiomux_setting gsbi1_suspended = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_2MA,
        .pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi1_active = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting gsbi2_suspended = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_2MA,
        .pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi2_active = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting gsbi3_suspended = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_2MA,
        .pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi3_active = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};
//ASUS_BSP larry lai : GSBI4 porting +++
static struct gpiomux_setting gsbi4_suspended = {
        .func = GPIOMUX_FUNC_GPIO,
        .drv = GPIOMUX_DRV_2MA,
        .pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting gsbi4_sda_active = {
        .func = GPIOMUX_FUNC_9,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};
 
static struct gpiomux_setting gsbi4_scl_active = {
        .func = GPIOMUX_FUNC_A,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};
//ASUS_BSP larry lai : GSBI4 porting ---
static struct gpiomux_setting gsbi7_func1 = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gsbi7_func2 = {
        .func = GPIOMUX_FUNC_2,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting fm_ssbi_suspended = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_2MA,
        .pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting fm_ssbi_active = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_6MA,
        .pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting fm_sdi_suspended = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_2MA,
        .pull = GPIOMUX_PULL_KEEPER,
};

static struct gpiomux_setting fm_sdi_active = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_6MA,
        .pull = GPIOMUX_PULL_NONE,
};

#if 0
static struct gpiomux_setting fm_i2s = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_KEEPER,
};
#endif

static struct gpiomux_setting cdc_mclk = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting slimbus = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_KEEPER,
};
static struct gpiomux_setting bt_pcm = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_KEEPER,
};
//-- Ledger

//ASUS_BSP+++ BennyCheng "config hsic gpio default settings"
static struct gpiomux_setting hsic_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hsic_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting hsic_wakeup_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting hsic_wakeup_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
//ASUS_BSP--- BennyCheng "config hsic gpio default settings"

//ASUS_BSP+++ BennyCheng "config apq-mdm gpio default settings"
static struct gpiomux_setting ap2mdm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_status_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_errfatal_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdm2ap_pblrdy = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ap2mdm_soft_reset_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting ap2mdm_wakeup = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};
//ASUS_BSP--- BennyCheng "config apq-mdm gpio default settings"

// static struct gpiomux_setting gsbi3 = {
// 	.func = GPIOMUX_FUNC_1,
// 	.drv = GPIOMUX_DRV_8MA,
// 	.pull = GPIOMUX_PULL_NONE,
// };

//ASUS_BSP +++ Stimber "[A68][8M][NA][Others]Full porting for 8M camera with ISP"
/*
static struct gpiomux_setting gsbi4 = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};
*/
//ASUS_BSP --- Stimber "[A68][8M][NA][Others]Full porting for 8M camera with ISP"

// static struct gpiomux_setting gsbi5 = {
// 	.func = GPIOMUX_FUNC_1,
// 	.drv = GPIOMUX_DRV_8MA,
// 	.pull = GPIOMUX_PULL_NONE,
// };

// static struct gpiomux_setting gsbi10 = {
// 	.func = GPIOMUX_FUNC_2,
// 	.drv = GPIOMUX_DRV_8MA,
// 	.pull = GPIOMUX_PULL_NONE,
// };
// 
// static struct gpiomux_setting gsbi12 = {
// 	.func = GPIOMUX_FUNC_1,
// 	.drv = GPIOMUX_DRV_8MA,
// 	.pull = GPIOMUX_PULL_NONE,
// };
// 
static struct gpiomux_setting hdmi_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,      //ext pull
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,      //ext pull, GPIOMUX_PULL_UP,
};

 static struct gpiomux_setting hdmi_active_2_cfg = {
 	.func = GPIOMUX_FUNC_1,
 	.drv = GPIOMUX_DRV_16MA,
 	.pull = GPIOMUX_PULL_DOWN,
 };

// static struct gpiomux_setting hdmi_active_2_cfg = {
// 	.func = GPIOMUX_FUNC_1,
// 	.drv = GPIOMUX_DRV_2MA,
// 	.pull = GPIOMUX_PULL_DOWN,
// };
//Mickey+++
// static struct gpiomux_setting hdmi_active_3_cfg = {
//     .func = GPIOMUX_FUNC_GPIO,
//     .drv = GPIOMUX_DRV_2MA,
//     .pull = GPIOMUX_PULL_NONE,
// };
//Mickey---

//joe1_++
static struct gpiomux_setting ts_reset_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting ts_int_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

/*static struct gpiomux_setting ts_pad_int_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};*/
//joe1_--

// //ASUS_BSP +++ Josh_Liao "add asus battery driver"
// static struct gpiomux_setting bat_low_cfg = {
// 	.func = GPIOMUX_FUNC_GPIO,
// 	.pull = GPIOMUX_PULL_UP,
// 	.dir = GPIOMUX_IN,
// };
// //ASUS_BSP --- Josh_Liao "add asus battery driver"
// 
// static struct gpiomux_setting hall_sensor_cfg = {
// 	.func = GPIOMUX_FUNC_GPIO,
// 	.pull = GPIOMUX_PULL_UP,
// 	.dir = GPIOMUX_IN,
// };

//ASUS BSP TIM-2011.08.12++
static struct gpiomux_setting gpio_volup_keys_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting gpio_voldown_keys_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting gpio_power_keys_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_UP,
};
//ASUS BSP TIM-2011.08.12--

//[PSensor] Add cm3623 support
static struct gpiomux_setting cm3623_pwr_en = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cm3623_interrupt = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
    .dir = GPIOMUX_IN,
};

// static struct gpiomux_setting rf_switch_cfg = {
//     .func = GPIOMUX_FUNC_GPIO,
// };
//Ledger
static struct gpiomux_setting O_L_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
    .dir = GPIOMUX_OUT_LOW,
};
//Ledger
static struct gpiomux_setting O_H_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
    .dir = GPIOMUX_OUT_HIGH,
};
// Ledger
static struct gpiomux_setting I_NP_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
    .dir = GPIOMUX_IN,
};
//Ledger
static struct gpiomux_setting I_PU_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_UP,
    .dir = GPIOMUX_IN,
};
//Ledger

//+++for NFC's irq GPIO+++
static struct gpiomux_setting nfc_intr_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
    .dir = GPIOMUX_IN,
};
//---for NFC's irq GPIO---

static struct gpiomux_setting NC_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
    .dir = GPIOMUX_IN,
};
// ASUS_BSP +++ sinachou
/*static struct gpiomux_setting microp_intr_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};*/
// ASUS_BSP --- 
//Mickey+++
static struct gpiomux_setting mdp_vsync_suspend_cfg = {
    .func = GPIOMUX_FUNC_1, //Mickey+++
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting mdp_vsync_active_cfg = {
    .func = GPIOMUX_FUNC_1,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
};
// static struct gpiomux_setting gpio_out = {
//     .func = GPIOMUX_FUNC_GPIO,
//     .drv = GPIOMUX_DRV_2MA,
//     .pull = GPIOMUX_PULL_NONE,
//     .dir = GPIOMUX_OUT_LOW,
// };
//Mickey---
//Desmond ++
static struct gpiomux_setting microp_intr_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
//Desmond --

//+++ ASUS_BSP: for A68
static struct gpiomux_setting fm_i2s = {
        .func = GPIOMUX_FUNC_1,
        .drv = GPIOMUX_DRV_8MA,
        .pull = GPIOMUX_PULL_KEEPER,
};

//--- ASUS_BSP: for A68
#endif  /* __A80_EVB_GPIO_SETTING_PINMUX_H__  */
