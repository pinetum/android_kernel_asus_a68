/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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
#include "msm_sensor.h"
#define SENSOR_NAME "mi1040"
#define PLATFORM_DRIVER_NAME "msm_camera_mi1040"
#define mi1040_obj mi1040_##obj

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
//#include <linux/a68_evb_gpio_pinmux.h>
#include <linux/gpio.h>
#include "iCatch7002a.h"
#include "msm_ispif.h"
#include "msm.h"
#include <linux/regulator/consumer.h>
#include <linux/mfd/pm8xxx/pm8921.h>

#include <linux/proc_fs.h>	//ASUS_BSP Stimber "Add ATD proc interface"
#include <linux/time.h>

/* Macros assume PMIC GPIOs and MPPs start at 1 */
#define PM8921_GPIO_BASE		NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_GPIO_BASE)

unsigned char g_mi1040_power = false; 
static struct timeval g_mi1040_power_tv;
extern bool iCatch_first_open;  // LiJen: for iCatch ISP used
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"

extern char g_camera_status;	//ASUS_BSP Stimber "Add ATD proc interface"
extern int g_cur_res;
extern bool g_streamon;

//ASUS_BSP +++ Stimber "Implement for calibration"
extern u32 is_calibration;
bool g_front_calibrating = false;
//ASUS_BSP --- Stimber "Implement for calibration"

DEFINE_MUTEX(mi1040_mut);
static struct msm_sensor_ctrl_t imx091_s_ctrl; //ASUS_BSP LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
#if 0	//LiJen: ISP dosen't  need
static struct msm_camera_i2c_reg_conf imx091_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf imx091_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf imx091_groupon_settings[] = {
	{0x0104, 0x01},
};

static struct msm_camera_i2c_reg_conf imx091_groupoff_settings[] = {
	{0x0104, 0x00},
};

static struct msm_camera_i2c_reg_conf imx091_prev_settings[] = {
	/* 30fps 1/2 * 1/2 */
	/* PLL setting */
	{0x0305, 0x02}, /* pre_pll_clk_div[7:0] */
	{0x0307, 0x2F}, /* pll_multiplier[7:0] */
	{0x30A4, 0x02},
	{0x303C, 0x4B},
	/* mode setting */
	{0x0340, 0x06}, /* frame_length_lines[15:8] */
	{0x0341, 0x5A}, /* frame_length_lines[7:0] */
	{0x0342, 0x12}, /* line_length_pck[15:8] */
	{0x0343, 0x0C}, /* line_length_pck[7:0] */
	{0x0344, 0x00}, /* x_addr_start[15:8] */
	{0x0345, 0x08}, /* x_addr_start[7:0] */
	{0x0346, 0x00}, /* y_addr_start[15:8] */
	{0x0347, 0x30}, /* y_addr_start[7:0] */
	{0x0348, 0x10}, /* x_addr_end[15:8] */
	{0x0349, 0x77}, /* x_addr_end[7:0] */
	{0x034A, 0x0C}, /* y_addr_end[15:8] */
	{0x034B, 0x5F}, /* y_addr_end[7:0] */
	{0x034C, 0x08}, /* x_output_size[15:8] */
	{0x034D, 0x38}, /* x_output_size[7:0] */
	{0x034E, 0x06}, /* y_output_size[15:8] */
	{0x034F, 0x18}, /* y_output_size[7:0] */
	{0x0381, 0x01}, /* x_even_inc[3:0] */
	{0x0383, 0x03}, /* x_odd_inc[3:0] */
	{0x0385, 0x01}, /* y_even_inc[7:0] */
	{0x0387, 0x03}, /* y_odd_inc[7:0] */
	{0x3040, 0x08},
	{0x3041, 0x97},
	{0x3048, 0x01},
	{0x3064, 0x12},
	{0x309B, 0x28},
	{0x309E, 0x00},
	{0x30D5, 0x09},
	{0x30D6, 0x01},
	{0x30D7, 0x01},
	{0x30D8, 0x64},
	{0x30D9, 0x89},
	{0x30DE, 0x02},
	{0x3102, 0x10},
	{0x3103, 0x44},
	{0x3104, 0x40},
	{0x3105, 0x00},
	{0x3106, 0x0D},
	{0x3107, 0x01},
	{0x310A, 0x0A},
	{0x315C, 0x99},
	{0x315D, 0x98},
	{0x316E, 0x9A},
	{0x316F, 0x99},
	{0x3318, 0x73},
};

static struct msm_camera_i2c_reg_conf imx091_snap_settings[] = {
	/* full size */
	/* PLL setting */
	{0x0305, 0x02}, /* pre_pll_clk_div[7:0] */
	{0x0307, 0x2B}, /* pll_multiplier[7:0] */
	{0x30A4, 0x02},
	{0x303C, 0x4B},
	/* mode setting */
	{0x0340, 0x0C}, /* frame_length_lines[15:8] */
	{0x0341, 0x8C}, /* frame_length_lines[7:0] */
	{0x0342, 0x12}, /* line_length_pck[15:8] */
	{0x0343, 0x0C}, /* line_length_pck[7:0] */
	{0x0344, 0x00}, /* x_addr_start[15:8] */
	{0x0345, 0x08}, /* x_addr_start[7:0] */
	{0x0346, 0x00}, /* y_addr_start[15:8] */
	{0x0347, 0x30}, /* y_addr_start[7:0] */
	{0x0348, 0x10}, /* x_addr_end[15:8] */
	{0x0349, 0x77}, /* x_addr_end[7:0] */
	{0x034A, 0x0C}, /* y_addr_end[15:8] */
	{0x034B, 0x5F}, /* y_addr_end[7:0] */
	{0x034C, 0x10}, /* x_output_size[15:8] */
	{0x034D, 0x70}, /* x_output_size[7:0] */
	{0x034E, 0x0C}, /* y_output_size[15:8] */
	{0x034F, 0x30}, /* y_output_size[7:0] */
	{0x0381, 0x01}, /* x_even_inc[3:0] */
	{0x0383, 0x01}, /* x_odd_inc[3:0] */
	{0x0385, 0x01}, /* y_even_inc[7:0] */
	{0x0387, 0x01}, /* y_odd_inc[7:0] */
	{0x3040, 0x08},
	{0x3041, 0x97},
	{0x3048, 0x00},
	{0x3064, 0x12},
	{0x309B, 0x20},
	{0x309E, 0x00},
	{0x30D5, 0x00},
	{0x30D6, 0x85},
	{0x30D7, 0x2A},
	{0x30D8, 0x64},
	{0x30D9, 0x89},
	{0x30DE, 0x00},
	{0x3102, 0x10},
	{0x3103, 0x44},
	{0x3104, 0x40},
	{0x3105, 0x00},
	{0x3106, 0x0D},
	{0x3107, 0x01},
	{0x310A, 0x0A},
	{0x315C, 0x99},
	{0x315D, 0x98},
	{0x316E, 0x9A},
	{0x316F, 0x99},
	{0x3318, 0x64},
};

static struct msm_camera_i2c_reg_conf imx091_recommend_settings[] = {
	/* global setting */
	{0x3087, 0x53},
	{0x309D, 0x94},
	{0x30A1, 0x08},
	{0x30C7, 0x00},
	{0x3115, 0x0E},
	{0x3118, 0x42},
	{0x311D, 0x34},
	{0x3121, 0x0D},
	{0x3212, 0xF2},
	{0x3213, 0x0F},
	{0x3215, 0x0F},
	{0x3217, 0x0B},
	{0x3219, 0x0B},
	{0x321B, 0x0D},
	{0x321D, 0x0D},
	/* black level setting */
	{0x3032, 0x40},
};
#endif //LiJen: ISP dosen't  need

static struct v4l2_subdev_info mi1040_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,  //V4L2_MBUS_FMT_SBGGR10_1X10 // LiJen: format ???
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

#if 0	//LiJen: ISP dosen't  need
static struct msm_camera_i2c_conf_array imx091_init_conf[] = {
	{&imx091_recommend_settings[0],
	ARRAY_SIZE(imx091_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array imx091_confs[] = {
	{&imx091_snap_settings[0],
	ARRAY_SIZE(imx091_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx091_prev_settings[0],
	ARRAY_SIZE(imx091_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};
#endif //LiJen: ISP dosen't  need
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"

#ifdef ASUS_A80_PROJECT
static struct msm_sensor_output_info_t mi1040_dimensions[] = {
//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	{	//Capture
		.x_output = 1920,
		.y_output = 1088,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 216000000,	//ASUS_BSP LiJen: This condifuration only for 3A, YUV sensor didn't need
		.op_pixel_clk = 320000000,	//ASUS_BSP LiJen: ISP MIPI speed is 576Mbits/Lane, 4x576/8=288M
		.binning_factor = 1,
	},
	{	//Preview
		.x_output = 1920,
		.y_output = 1088,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 216000000,	//ASUS_BSP LiJen: This condifuration only for 3A, YUV sensor didn't need
		.op_pixel_clk = 320000000,	//ASUS_BSP LiJen: ISP MIPI speed is 576Mbits/Lane, 4x576/8=288M
		.binning_factor = 1,
	},
			{ //video
				.x_output = 1920,
				.y_output = 1088,
				.line_length_pclk = 0x85c,
				.frame_length_lines = 0x460,
				.vt_pixel_clk = 216000000,	//ASUS_BSP LiJen: This condifuration only for 3A, YUV sensor didn't need
				.op_pixel_clk = 320000000,	//ASUS_BSP LiJen: ISP MIPI speed is 576Mbits/Lane, 4x576/8=288M
				.binning_factor = 1,
			},
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
};

#endif 

#ifdef ASUS_A68_PROJECT
static struct msm_sensor_output_info_t mi1040_dimensions[] = {
//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	{	//Capture
		.x_output = 1280,
		.y_output = 960,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 216000000,	//ASUS_BSP LiJen: This condifuration only for 3A, YUV sensor didn't need
		.op_pixel_clk = 320000000,	//ASUS_BSP LiJen: ISP MIPI speed is 576Mbits/Lane, 4x576/8=288M
		.binning_factor = 1,
	},
	{	//Preview
		.x_output = 1280,
		.y_output = 960,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 216000000,	//ASUS_BSP LiJen: This condifuration only for 3A, YUV sensor didn't need
		.op_pixel_clk = 320000000,	//ASUS_BSP LiJen: ISP MIPI speed is 576Mbits/Lane, 4x576/8=288M
		.binning_factor = 1,
	},
			{ //video
				.x_output = 1280,
				.y_output = 960,
				.line_length_pclk = 0x85c,
				.frame_length_lines = 0x460,
				.vt_pixel_clk = 216000000,	//ASUS_BSP LiJen: This condifuration only for 3A, YUV sensor didn't need
				.op_pixel_clk = 320000000,	//ASUS_BSP LiJen: ISP MIPI speed is 576Mbits/Lane, 4x576/8=288M
				.binning_factor = 1,
			},
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
};
#endif 
#if 0
static struct msm_camera_csid_vc_cfg mi1040_cid_cfg[] = {
	{0, 0x1E, CSI_DECODE_8BIT}, //ASUS_BSP LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params mi1040_csi_params = {
	.csid_params = {
		.lane_cnt = 4,
		.lut_params = {
			.num_cid = ARRAY_SIZE(mi1040_cid_cfg),
			.vc_cfg = mi1040_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 4,
		.settle_cnt = 0x14,//0x12, //ASUS_BSP LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	},
};

static struct msm_camera_csi2_params *mi1040_csi_params_array[] = {
	&mi1040_csi_params,
	&mi1040_csi_params,
};
#endif 

static struct msm_sensor_output_reg_addr_t mi1040_reg_addr = {
	.x_output = 0x034C,
	.y_output = 0x034E,
	.line_length_pclk = 0x0342,
	.frame_length_lines = 0x0340,
};

static struct msm_sensor_id_info_t mi1040_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0x0091,
};

static struct msm_sensor_exp_gain_info_t mi1040_exp_gain_info = {
	.coarse_int_time_addr = 0x0202,
	.global_gain_addr = 0x0204,
	.vert_offset = 5,
};

static struct pm_gpio pm_isp_gpio_high = {
	.direction		  = PM_GPIO_DIR_OUT,
	.output_buffer	  = PM_GPIO_OUT_BUF_CMOS,
	.output_value	  = 1,
	.pull			  = PM_GPIO_PULL_NO,
	.vin_sel		  = PM_GPIO_VIN_S4,
	.out_strength	  = PM_GPIO_STRENGTH_HIGH,
	.function		  = PM_GPIO_FUNC_PAIRED,
	.inv_int_pol	  = 0,
	.disable_pin	  = 0,
};

	static struct pm_gpio pm_isp_gpio_low = {
	.direction		  = PM_GPIO_DIR_OUT,
	.output_buffer	  = PM_GPIO_OUT_BUF_CMOS,
	.output_value	  = 0,
	.pull			  = PM_GPIO_PULL_NO,
	.vin_sel		  = PM_GPIO_VIN_S4,
	.out_strength	  = PM_GPIO_STRENGTH_HIGH,
	.function		  = PM_GPIO_FUNC_PAIRED,
	.inv_int_pol	  = 0,
	.disable_pin	  = 0,
};


//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
static int mi1040_regulator_init(bool on)
{
	static struct regulator *reg_8921_l8, *reg_8921_l6;
	static int prev_on = false;
	int rc;

	pr_info("%s +++\n",__func__);

	if (on) {
		if (on == prev_on) {
			pr_info("on(%d) == prev_on(%d)\n",on,prev_on);
			pr_info("%s ---\n",__func__);
			return 0;
		}
		prev_on = true;

		if (!reg_8921_l8) {
			reg_8921_l8 = regulator_get(&imx091_s_ctrl.sensor_i2c_client->client->dev, "8921_l8");
			if (IS_ERR(reg_8921_l8)) {
				pr_info("PTR_ERR(reg_8921_l8)=%ld\n", PTR_ERR(reg_8921_l8));
				return -ENODEV;
			}
		}

        	if (!reg_8921_l6) {
        		reg_8921_l6 = regulator_get(&imx091_s_ctrl.sensor_i2c_client->client->dev, "8921_l6");
        		if (IS_ERR(reg_8921_l6)) {
        			pr_info("PTR_ERR(reg_8921_l8)=%ld\n", PTR_ERR(reg_8921_l6));
        			return -ENODEV;
        		}
        	}                  

		pr_info("Turn on the regulators\n");
        
		rc = regulator_set_voltage(reg_8921_l8, 2800000, 2800000);
		if (!rc) {
			pr_info("reg_8921_l8 regulator_set_voltage, !rc is true, rc=%d--\n", rc);
			rc = regulator_enable(reg_8921_l8);
		}

		if (rc) {
			pr_info("8921_l8 regulator enable failed, rc=%d--\n", rc);
			return rc;
		}
		pr_info("Turn on reg_8921_l8 success\n");
		pr_info("reg_8921_l8(%d)",regulator_get_voltage(reg_8921_l8));
		pr_info("reg_8921_l8 enable(%d)",regulator_is_enabled(reg_8921_l8));

            	rc = regulator_set_voltage(reg_8921_l6, 2700000, 2700000);
        	if (!rc) {
        		pr_info("reg_8921_l6 regulator_set_voltage, !rc is true, rc=%d--\n", rc);
        		rc = regulator_enable(reg_8921_l6);
        	}

        	if (rc) {
        		pr_info("8921_l6 regulator enable failed, rc=%d--\n", rc);
        		return rc;
        	}
        	pr_info("Turn on reg_8921_l6 success\n");
        	pr_info("reg_8921_l6(%d)",regulator_get_voltage(reg_8921_l6));
        	pr_info("reg_8921_l6 enable(%d)",regulator_is_enabled(reg_8921_l6));
	} else {           //(on == false) /* Turn off the regulators */

		if (on == prev_on) {
			pr_info("on(%d) == prev_on(%d)\n",on,prev_on);
			pr_info("%s ---\n",__func__);
			return 0;
		}

		pr_info("Turn off the regulators\n");
		prev_on = false;

		regulator_disable(reg_8921_l8);
              regulator_disable(reg_8921_l6);
	}
	pr_info("%s ---\n",__func__);
	return 0;
}

void mi1040_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_info("%s +++ \n",__func__);

    if(g_front_calibrating){	//ASUS_BSP Stimber "Implement for calibration"
        pr_info("%s ignore during calibration \n",__func__);
    }else{
       if(true == iCatch_first_open){    //ASUS_BSP Stimber
           pr_info("%s first open camera \n",__func__);
#ifdef ASUS_A68_PROJECT		   
           sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x00);//preview mode
#endif 
#ifdef ASUS_A80_PROJECT		   
           sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7106, 0x0d);//preview mode
#endif 
           sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);  
	     sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7121, 0x01);	 //ASUS_BSP LiJen "[A86][Camera][NA][Others]Camera mini porting"
           wait_for_AWB_ready();
       }
    }
    
	//ASUS_BSP Stimber "Implement for calibration"
    if(is_calibration){
         g_front_calibrating = true;
    }
    g_streamon = true;
              
	pr_info("%s --- \n",__func__);
}

void mi1040_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct timeval tv;
       long gap =0;
       const long l_delay = 70; //ISP power up to MIPI pull high at least need 70 ms
       
    
	pr_info("%s +++ \n",__func__);
       if(true == iCatch_first_open){
           //wait for isp load code
    	    do_gettimeofday(&tv);
           gap = tv.tv_sec*1000-g_mi1040_power_tv.tv_sec*1000 + tv.tv_usec/1000-g_mi1040_power_tv.tv_usec/1000;
           if(gap > 0 && gap < l_delay){
                msleep(l_delay-gap);
                pr_info("%s delay(%ld)\n",__func__,gap);
           }else{
                //pr_info("%s no delay(%ld)\n",__func__,gap);
           }
           sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x7121, 0x00);  
           msleep(3); //wait for stream off command (MIPI pull high)                
       }  
       g_streamon = false;
	pr_info("%s --- \n",__func__);
}

int32_t mi1040_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	pr_info("%s +++ \n",__func__);

	pr_info("%s --- \n",__func__);
       return rc;
}

late_initcall(icatch_i2c_debuginit);

u8 is_front_camera = 1;
 

static int32_t mi1040_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
				int update_type, int res)
{
	int32_t rc = 0;
 //      struct sensor_mode mode;
 //      struct sensor_info info;
        
	pr_info("%s +++ res(%d)\n",__func__,res);    

#if 0	//LiJen: ISP dosen't  need                     
	s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
	msleep(30);
#endif

	if (update_type == MSM_SENSOR_REG_INIT) {
		pr_err("%s MSM_SENSOR_REG_INIT\n",__func__);	               
              iCatch_first_open = true; //LiJen: Ignore wiat staus when ISP first open

		//ASUS_BSP +++ Peter_Lu For i2c bus for protect loading ISP code
#ifdef ASUS_A80_PROJECT		
		sw_i2c_bus_lock( imx091_s_ctrl.sensor_i2c_client->client->adapter, true, 80, I2C_BUS_LOCK_TYPE2 );
#endif		
              //ISP load code - switch to front camera
              sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1011, 0x01);//cpu reset
              sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x001C, 0x08);
              sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x001C, 0x00);
              sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1010, 0x02);
              sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1010, 0x00);
              sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1306, 0x01);//front camera
              sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, 0x1011, 0x00);
              msleep(10);    

              iCatch_enable_exif(true, false);
		//ASUS_BSP +++ Peter_Lu Lock i2c bus for protect runing ISP code
#ifdef ASUS_A80_PROJECT		
		sw_i2c_bus_lock( imx091_s_ctrl.sensor_i2c_client->client->adapter, true, 70, I2C_BUS_LOCK_TYPE1 );
#endif
              g_front_calibrating = false;	//ASUS_BSP Stimber "Implement for calibration"
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
            v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
                    NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
                    output_settings[0].op_pixel_clk);
            g_cur_res=res;
	}
              
	pr_info("%s ---\n",__func__);
	return rc;
}

int32_t mi1040_sensor_mode_init(struct msm_sensor_ctrl_t *s_ctrl,
			int mode, struct sensor_init_cfg *init_info)
{
	int32_t rc = 0;
	pr_info("%s +++ return\n",__func__);
	s_ctrl->fps_divider = Q10;
	s_ctrl->cam_mode = MSM_SENSOR_MODE_INVALID;

	CDBG("%s: %d\n", __func__, __LINE__);
	if (mode != s_ctrl->cam_mode) {

#if 0	//LiJen: ISP dosen't  need			
		if (init_info->prev_res >=
			s_ctrl->msm_sensor_reg->num_conf ||
			init_info->pict_res >=
			s_ctrl->msm_sensor_reg->num_conf) {
			CDBG("Resolution does not exist");
			return -EINVAL;
		}
#endif		
              g_cur_res = MSM_SENSOR_INVALID_RES;
		s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
		s_ctrl->cam_mode = mode;

		rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
			MSM_SENSOR_REG_INIT, 0);
	}
	pr_info("%s --- \n",__func__);
	return rc;
}

static int mi1040_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	pr_info("%s +++ \n",__func__);
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
    
	       mutex_lock(s_ctrl->msm_sensor_mutex);
        	CDBG("msm_sensor_config: cfgtype = %d\n",cdata.cfgtype);\
                
		switch (cdata.cfgtype) {
		case CFG_SET_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_sensor_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_set_sensor_mode(
					s_ctrl,
					cdata.mode,
					cdata.rs);
			break;

		case CFG_SET_EFFECT:
			break;

		case CFG_SENSOR_INIT:
			if (s_ctrl->func_tbl->
			sensor_mode_init == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_mode_init(
				s_ctrl,
				cdata.mode,
				&(cdata.cfg.init_info));
			break;

		case CFG_GET_OUTPUT_INFO:
			if (s_ctrl->func_tbl->
			sensor_get_output_info == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_get_output_info(
				s_ctrl,
				&cdata.cfg.output_info);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_START_STREAM:
			if (s_ctrl->func_tbl->sensor_start_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
			break;

		case CFG_STOP_STREAM:
			if (s_ctrl->func_tbl->sensor_stop_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			break;

		case CFG_GET_CSI_PARAMS:
			if (s_ctrl->func_tbl->sensor_get_csi_params == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->sensor_get_csi_params(
				s_ctrl,
				&cdata.cfg.csi_lane_params);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
            
//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Mini porting for 13M camera with ISP"
#if 0 //LiJen don't need
		case CFG_GET_EEPROM_DATA:
			if (s_ctrl->sensor_eeprom_client == NULL ||
				s_ctrl->sensor_eeprom_client->
				func_tbl.eeprom_get_data == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->sensor_eeprom_client->
				func_tbl.eeprom_get_data(
				s_ctrl->sensor_eeprom_client,
				&cdata.cfg.eeprom_data);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_eeprom_data_t)))
				rc = -EFAULT;
			break;
#endif 

//ASUS_BSP +++ 
		case CFG_SET_ISP_WB_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_wb_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_WB_MODE mode(%d)\n",cdata.cfg.wb);
			s_ctrl->func_tbl->
			sensor_set_isp_wb_mode(cdata.cfg.wb);						
			break;

		case CFG_SET_ISP_EV_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_ev_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_EV_MODE mode(%d)\n",cdata.cfg.ev);
			s_ctrl->func_tbl->
			sensor_set_isp_ev_mode(cdata.cfg.ev);						
			break;

		case CFG_SET_ISP_ISO_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_iso_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_ISO_MODE mode(%d)\n",cdata.cfg.iso);
			s_ctrl->func_tbl->
			sensor_set_isp_iso_mode(cdata.cfg.iso);						
			break;      

		case CFG_SET_ISP_FLICKER_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_flicker_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_FLICKER_MODE mode(%d)\n",cdata.cfg.flicker);
			s_ctrl->func_tbl->
			sensor_set_isp_flicker_mode(cdata.cfg.flicker);						
			break;   

		case CFG_SET_ISP_AECLOCK_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_aeclock_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_AECLOCK_MODE mode(%d)\n",cdata.cfg.aeclock);
			s_ctrl->func_tbl->
			sensor_set_isp_aeclock_mode(cdata.cfg.aeclock);						
			break;

		case CFG_SET_ISP_AWBLOCK_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_awblock_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_AWBLOCK_MODE mode(%d)\n",cdata.cfg.awblock);
			s_ctrl->func_tbl->
			sensor_set_isp_awblock_mode(cdata.cfg.awblock);						
			break;

		case CFG_SET_ISP_CAF_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_caf_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_CAF_MODE mode(%d)\n",cdata.cfg.focus.af_continue);
			s_ctrl->func_tbl->
			sensor_set_isp_caf_mode(cdata.cfg.focus.af_continue);						
			break;

			pr_info("CFG_SET_ISP_FLICKER_MODE mode(%d)\n",cdata.cfg.flicker);
			s_ctrl->func_tbl->
			sensor_set_isp_flicker_mode(cdata.cfg.flicker);						
			break;   

            		case CFG_SET_ISP_SCENE_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_scene_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_SCENE_MODE mode(%d)\n",cdata.cfg.scene);
			s_ctrl->func_tbl->
			sensor_set_isp_scene_mode(cdata.cfg.scene);						
			break;

		case CFG_SET_ISP_EFFECT_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_effect_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_EFFECT_MODE mode(%d)\n",cdata.cfg.effect);
			s_ctrl->func_tbl->
			sensor_set_isp_effect_mode(cdata.cfg.effect);						
			break;

 		case CFG_SET_ISP_AURA_VALUE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_aura_value == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_AURA_VALUE mode(%d)\n",cdata.cfg.aura);
			s_ctrl->func_tbl->
			sensor_set_isp_aura_value(cdata.cfg.aura);						
			break;

//ASUS_BSP +++ Stimber "Implement EXIF info for camera with ISP"
		case CFG_GET_ISP_EXIF:
			if (s_ctrl->func_tbl->sensor_get_isp_exif == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_GET_ISP_EXIF\n");
			s_ctrl->func_tbl->sensor_get_isp_exif(&cdata.cfg.exif);
			if (copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data))){
				rc = -EFAULT;
			}
			break;
//ASUS_BSP --- Stimber "Implement EXIF info for camera with ISP"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement general command"
 		case CFG_SET_ISP_GENERAL_CMD:
			if (s_ctrl->func_tbl->
			sensor_set_isp_general_cmd == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_GENERAL_CMD\n");
			s_ctrl->func_tbl->
			sensor_set_isp_general_cmd(&cdata.cfg.general_cmd);						
			break;
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement general command"

#if 0 //TODO: for iCatch
		case CFG_ISP_AF_START:
			if (s_ctrl->func_tbl->
			sensor_isp_af_start == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_ISP_AF_START afParam(%d,%d,%d,%d,%d,%d)\n",cdata.cfg.focus.af_enable,cdata.cfg.focus.mode,cdata.cfg.focus.coordinate_x,cdata.cfg.focus.coordinate_y,cdata.cfg.focus.rectangle_h,cdata.cfg.focus.rectangle_w);
			s_ctrl->func_tbl->
				sensor_isp_af_start(
				cdata.cfg.focus.af_enable,
				cdata.cfg.focus.mode,
				cdata.cfg.focus.coordinate_x,
				cdata.cfg.focus.coordinate_y,
				cdata.cfg.focus.rectangle_h,
				cdata.cfg.focus.rectangle_w);
			break;

		case CFG_GET_ISP_AF_RESULT:
			if (s_ctrl->func_tbl->
			sensor_get_isp_af_result == NULL) {
				rc = -EFAULT;
				break;
			}
			cdata.cfg.focus.result =
				s_ctrl->func_tbl->
				sensor_get_isp_af_result
				(s_ctrl);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			
			pr_info("CFG_GET_ISP_AF_RESULT: %d\n",cdata.cfg.focus.result);
			break;		
            
		case CFG_SET_ISP_LED_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_led_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_LED_MODE mode(%d)\n",cdata.cfg.is_autoflash);
			s_ctrl->func_tbl->
			sensor_set_isp_led_mode(cdata.cfg.is_autoflash);						
			break;   

//ASUS_BSP +++ LiJen "[A68][13M][NA][Other] Implement EXIF info for 13M camera with ISP"
		case CFG_GET_ISP_EXIF:
			if (s_ctrl->func_tbl->sensor_get_isp_exif == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_GET_ISP_EXIF\n");
			s_ctrl->func_tbl->sensor_get_isp_exif(&cdata.cfg.exif);
			if (copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data))){
				rc = -EFAULT;
			}
			break;
//ASUS_BSP --- LiJen "[A68][13M][NA][Other] Implement EXIF info for 13M camera with ISP"
#endif
// ASUS BSP ---
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Mini porting for 13M camera with ISP"
		default:
			pr_info("%s: cfgtype = %d is not supported!!\n",__func__,cdata.cfgtype);
			rc = -EFAULT;
			break;
		}

	mutex_unlock(s_ctrl->msm_sensor_mutex);
	pr_info("%s --- \n",__func__);
	return rc;
}


#ifdef ASUS_A80_PROJECT

static int mi1040_power_down(const struct msm_camera_sensor_info *data)
{
       int rc=0;
    	static struct pm_gpio pm_isp_gpio_low = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 0,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_S4,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_PAIRED,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};
        
	pr_info("%s +++\n",__func__);

       if(g_mi1040_power == false){
            pr_info("%s --- power has disabled\n", __func__);
            return -1;
       }
       
	if(!data)
	{
		pr_info("data is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	if(!imx091_s_ctrl.sensordata)
	{
		pr_info("imx091_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	switch (g_A68_hwID)
	{
		case A68_EVB:
		case A68_SR1_1:
		case A68_SR1_2:
		case A68_SR2:
		case A68_ER1:
		case A68_ER2:
		case A68_ER3:
		case A68_PR:
		case A68_PR2:
		case A68_MP:
		case A68_CD:
		default:
		//mutex_lock(imx091_s_ctrl.msm_sensor_mutex);    //ASUS_BSP Stimber "Fix the issue which fail to re-open camera"
		// Switch CLK to 8M
			gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 0);

	    //ISP_RST reset low
	       //gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 0);
	       rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), &pm_isp_gpio_low);
	       if (rc != 0){
				pr_err("%s: sensor_reset failed\n", __func__);
	       }       
	    
	    //disable MCLK
	       msm_sensor_power_down(&imx091_s_ctrl);

	    //PMIC regulator - ISP 2.8V OFF
		   mi1040_regulator_init(false);

	    //ISP 1.8V OFF
			//gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, 0);
		   rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), &pm_isp_gpio_low);
	       if (rc != 0){
				pr_err("%s: isp_1p8_en failed\n", __func__);
	       }    
	       		   
	    //ISP 1.2V OFF
	       //gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, 0);
	       rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), &pm_isp_gpio_low);
	       if (rc != 0){
				pr_err("%s: isp_1p2_en failed\n", __func__);
	       }     

		   //gpio_set_value_cansleep(imx091_s_ctrl.sensordata->sensor_reset, 0);
		   gpio_set_value_cansleep(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_reset), 0);
		   
		   gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 0);   	

		   msleep(20);
		   gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset));
		   gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en));
		   gpio_free(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en);
		   gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en));	
	     gpio_free(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend);
		   break;
	}

       g_mi1040_power = false;

       g_front_calibrating = false;	//ASUS_BSP Stimber "Implement for calibration"
	pr_info("%s ---\n",__func__);
	return 0;
}

static int mi1040_gpio_request(void)
{
	int32_t rc = 0;
    
    pr_info("%s +++\n",__func__);

	switch (g_A68_hwID)
	{
		case A68_EVB:
	        // Power on ISP module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p2_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, rc);
	        	goto init_probe_fail0;
	        }

	        // Power on 8M camera OV8830 module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p8_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, rc);
	        	goto init_probe_fail1;
	        }

	        rc = gpio_request(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio vga_mclk_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, rc);
	        	goto init_probe_fail2;
	        }

	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio sensor_reset %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, rc);
	        	goto init_probe_fail3;
	        }

	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio 8M_WP %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, rc);
	        	goto init_probe_fail4;
	        }
			break;
			
		case A68_SR1_1:
		case A68_SR1_2:
		case A68_SR2:
		case A68_ER1:
		case A68_ER2:
		case A68_ER3:
		case A68_PR:
		//case A68_MP:
		default:
			// Power on ISP module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p2_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, rc);
	        	goto init_probe_fail0;
	        }

	        // Power on 8M camera OV8830 module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p8_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, rc);
	        	goto init_probe_fail1;
	        }

	        rc = gpio_request(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio vga_mclk_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, rc);
	        	goto init_probe_fail2;
	        }

	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio sensor_reset %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, rc);
	        	goto init_probe_fail3;
	        }

	        rc = gpio_request(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio 8M_WP %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, rc);
	        	goto init_probe_fail4;
	        }

			break;
	}
        
	pr_info("%s ---\n",__func__);
	return rc;


init_probe_fail4:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset));

init_probe_fail3:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en));

init_probe_fail2:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en));

init_probe_fail1:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en));

init_probe_fail0:    
	pr_info("%s ---\n",__func__);
	return rc;
}

static int mi1040_power_up(const struct msm_camera_sensor_info *data)
{
	int ret = -1, rc=0;
        
	pr_info("%s +++\n",__func__);

       if(g_mi1040_power == true){
            pr_info("%s --- power has enabled\n", __func__);
            return -1;	            
       }
       
	if(!data)
	{
		pr_err("data is NULL, return\n");
		pr_err("%s ---\n",__func__);
		return -1;
	}

	if(!imx091_s_ctrl.sensordata)
	{
		pr_err("imx091_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

       iCatch_init();
       
	ret = mi1040_gpio_request();
	if(ret < 0)
	{
		pr_err("8M Camera GPIO request fail!!\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	switch (g_A68_hwID)
	{
		case A68_EVB:
		case A68_SR1_1:
		case A68_SR1_2:
		case A68_SR2:
		case A68_ER1:
		case A68_ER2:
		case A68_ER3:
		case A68_PR:
		case A68_PR2:			
		case A68_MP:
		default:
		// ISP power on +++

		//ASUS_BSP +++ Peter_Lu Lock i2c bus for protect camera ISP
		sw_i2c_bus_lock( imx091_s_ctrl.sensor_i2c_client->client->adapter, true, 200, I2C_BUS_LOCK_TYPE1 );

		//ISP_RST reset low
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 0);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), &pm_isp_gpio_low);
		    if (rc != 0){
				pr_err("%s: sensor_reset low failed\n", __func__);
		    }	
		    //msleep(25); 

		// ISP 1.2V ON
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, 1);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), &pm_isp_gpio_high);
		    if (rc != 0){
				pr_err("%s: isp_1p2_en high failed\n", __func__);
		    }	
		    //msleep(0); //t2 
		    
		// ISP 1.8V on
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, 1);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), &pm_isp_gpio_high);
		    if (rc != 0){
				pr_err("%s: isp_1p8_en high failed\n", __func__);
		    }
		    msleep(1);   //t1

		//PMIC regulator - ISP 2.8V ON    
		    mi1040_regulator_init(true);
		    
		// enable MCLK
		    msm_sensor_power_up(&imx091_s_ctrl);
		    //t3

		//ISP SUSPEND high
			rc = gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 1);
		    if (rc != 0){
				pr_err("%s: isp_suspend high failed\n", __func__);
		    }
		          
		//ISP_RST reset high
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 1);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), &pm_isp_gpio_high);
		    if (rc != 0){
				pr_err("%s: sensor_reset high failed\n", __func__);
		    }	
		    msleep(6);

		//ISP SUSPEND low
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 0);
		    gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 0);
	
		// ISP power on ---

		//ASUS_BSP +++ Peter_Lu Lock i2c bus for protect camera ISP
		sw_i2c_bus_lock( imx091_s_ctrl.sensor_i2c_client->client->adapter, true, 70, I2C_BUS_LOCK_TYPE1 );

		// Sensor power on +++
		// vga_mclk_en switch to 8M, ensure initial state
			gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 1);

		// Switch CLK to 8M
		if (!is_front_camera) {
			// vga_mclk_en switch to VGA, ensure initial state
				gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 0);
		// Switch CLK to 8M
					gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 1);
		// Sensor power on ---
			} else {
				// vga_mclk_en switch to VGA, ensure initial state
					gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 1);
			// Switch CLK to 8M
						gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 0);
			}
		// Sensor power on ---

                do_gettimeofday(&g_mi1040_power_tv);
		
		// Wait for I2C ready
		//ASUS_BSP +++ Peter_Lu Already lock i2c bus 70ms 
		//  msleep(10); //LiJen: tmp

			pr_info("gpio sensor_reset(%d)\n",gpio_get_value(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset)));
			pr_info("gpio isp_1p2_en(%d)\n",gpio_get_value(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en)));
			pr_info("gpio vga_mclk_en(%d)\n",gpio_get_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en));
			pr_info("gpio isp_1p8_en(%d)\n",gpio_get_value(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en)));
			pr_info("gpio ISP_SUSPEND(%d)\n",gpio_get_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend));
			pr_info("gpio GPIO5(%d)\n",gpio_get_value(5));
			break;
	}

       g_mi1040_power = true;
	pr_info("%s ---\n",__func__);
	return 0;	
}



#endif
#ifdef ASUS_A68_PROJECT

static int mi1040_power_down(const struct msm_camera_sensor_info *data)
{
       int rc=0;
    	static struct pm_gpio pm_isp_gpio_low = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 0,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_S4,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_PAIRED,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};
        
	pr_info("%s +++\n",__func__);

       if(g_mi1040_power == false){
            pr_info("%s --- power has disabled\n", __func__);
            return -1;
       }
       
	if(!data)
	{
		pr_info("data is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	if(!imx091_s_ctrl.sensordata)
	{
		pr_info("imx091_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	switch (g_A68_hwID)
	{
		case A68_EVB:
		case A68_SR1_1:
		case A68_SR1_2:
		case A68_SR2:
		case A68_ER1:
		case A68_ER2:
		case A68_ER3:
		case A68_PR:
		case A68_PR2:			
		case A68_MP:
		case A68_CD:
		default:
		//mutex_lock(imx091_s_ctrl.msm_sensor_mutex);    //ASUS_BSP Stimber "Fix the issue which fail to re-open camera"
		// Switch CLK to 8M
			gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 0);

	    //ISP_RST reset low
	       //gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 0);
	       rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), &pm_isp_gpio_low);
	       if (rc != 0){
				pr_err("%s: sensor_reset failed\n", __func__);
	       }       
	    
	    //disable MCLK
	       msm_sensor_power_down(&imx091_s_ctrl);

	    //PMIC regulator - ISP 2.8V OFF
		   mi1040_regulator_init(false);

	    //ISP 1.8V OFF
			//gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, 0);
		   rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), &pm_isp_gpio_low);
	       if (rc != 0){
				pr_err("%s: isp_1p8_en failed\n", __func__);
	       }    
	       		   
	    //ISP 1.2V OFF
	       //gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, 0);
	       rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), &pm_isp_gpio_low);
	       if (rc != 0){
				pr_err("%s: isp_1p2_en failed\n", __func__);
	       }     

		   //gpio_set_value_cansleep(imx091_s_ctrl.sensordata->sensor_reset, 0);
		   gpio_set_value_cansleep(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_reset), 0);
		   
		   gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 0);   	

		   msleep(20);
		   gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset));
		   gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en));
		   gpio_free(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en);
		   gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en));	
	     gpio_free(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend);
		   break;
	}

       g_mi1040_power = false;

       g_front_calibrating = false;	//ASUS_BSP Stimber "Implement for calibration"
	pr_info("%s ---\n",__func__);
	return 0;
}

static int mi1040_gpio_request(void)
{
	int32_t rc = 0;
    
    pr_info("%s +++\n",__func__);
    printk("%s +++ g_A68_hwID=%d\n",__func__,g_A68_hwID);
	switch (g_A68_hwID)
	{
		case A68_EVB:
	        // Power on ISP module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p2_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, rc);
	        	goto init_probe_fail0;
	        }

	        // Power on 8M camera OV8830 module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p8_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, rc);
	        	goto init_probe_fail1;
	        }

	        rc = gpio_request(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio vga_mclk_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, rc);
	        	goto init_probe_fail2;
	        }

	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio sensor_reset %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, rc);
	        	goto init_probe_fail3;
	        }

	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio 8M_WP %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, rc);
	        	goto init_probe_fail4;
	        }
			break;
			
		case A68_SR1_1:
		case A68_SR1_2:
		case A68_SR2:
		case A68_ER1:
		case A68_ER2:
		case A68_ER3:
		case A68_PR:
             case A68_PR2:
		case A68_MP:
		case A68_CD:
		default:
			// Power on ISP module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p2_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, rc);
	        	goto init_probe_fail0;
	        }

	        // Power on 8M camera OV8830 module:
	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio isp_1p8_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, rc);
	        	goto init_probe_fail1;
	        }

	        rc = gpio_request(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio vga_mclk_en %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, rc);
	        	goto init_probe_fail2;
	        }

	        rc = gpio_request(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio sensor_reset %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, rc);
	        	goto init_probe_fail3;
	        }

	        rc = gpio_request(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, "mi1040");
	        if (rc) {
	        	pr_err("%s: gpio 8M_WP %d, rc(%d)fail\n",__func__, imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, rc);
	        	goto init_probe_fail4;
	        }

			break;
	}
        
	pr_info("%s ---\n",__func__);
	return rc;


init_probe_fail4:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset));

init_probe_fail3:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en));

init_probe_fail2:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en));

init_probe_fail1:
	gpio_free(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en));

init_probe_fail0:    
	pr_info("%s ---\n",__func__);
	return rc;
}



static int mi1040_power_up(const struct msm_camera_sensor_info *data)
{
	int ret = -1, rc=0;
        
	pr_info("%s +++\n",__func__);

       if(g_mi1040_power == true){
            pr_info("%s --- power has enabled\n", __func__);
            return -1;	            
       }
       
	if(!data)
	{
		pr_err("data is NULL, return\n");
		pr_err("%s ---\n",__func__);
		return -1;
	}

	if(!imx091_s_ctrl.sensordata)
	{
		pr_err("imx091_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

       iCatch_init();
       
	ret = mi1040_gpio_request();
	if(ret < 0)
	{
		pr_err("8M Camera GPIO request fail!!\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	switch (g_A68_hwID)
	{
		case A68_EVB:
		case A68_SR1_1:
		case A68_SR1_2:
		case A68_SR2:
		case A68_ER1:
		case A68_ER2:
		case A68_ER3:
		case A68_PR:
		case A68_PR2:			
		case A68_MP:
		case A68_CD:
		default:
		// ISP power on +++

		//ASUS_BSP +++ Peter_Lu Lock i2c bus for protect camera ISP
//		sw_i2c_bus_lock( imx091_s_ctrl.sensor_i2c_client->client->adapter, true, 200, I2C_BUS_LOCK_TYPE1 );

		//ISP_RST reset low
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 0);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), &pm_isp_gpio_low);
		    if (rc != 0){
				pr_err("%s: sensor_reset low failed\n", __func__);
		    }	
		    //msleep(25); 

		// ISP 1.2V ON
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en, 1);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en), &pm_isp_gpio_high);
		    if (rc != 0){
				pr_err("%s: isp_1p2_en high failed\n", __func__);
		    }	
		    //msleep(0); //t2 
		    
		// ISP 1.8V on
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en, 1);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en), &pm_isp_gpio_high);
		    if (rc != 0){
				pr_err("%s: isp_1p8_en high failed\n", __func__);
		    }
		    msleep(1);   //t1

		//PMIC regulator - ISP 2.8V ON    
		    mi1040_regulator_init(true);
		    
		// enable MCLK
		    msm_sensor_power_up(&imx091_s_ctrl);
		    //t3

		//ISP SUSPEND high
			rc = gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 1);
		    if (rc != 0){
				pr_err("%s: isp_suspend high failed\n", __func__);
		    }
		          
		//ISP_RST reset high
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 1);
		    rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset), &pm_isp_gpio_high);
		    if (rc != 0){
				pr_err("%s: sensor_reset high failed\n", __func__);
		    }	
		    msleep(6);

		//ISP SUSPEND low
			//gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 0);
		    gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend, 0);
	
		// ISP power on ---

		//ASUS_BSP +++ Peter_Lu Lock i2c bus for protect camera ISP
//		sw_i2c_bus_lock( imx091_s_ctrl.sensor_i2c_client->client->adapter, true, 70, I2C_BUS_LOCK_TYPE1 );

		// Sensor power on +++
		// vga_mclk_en switch to 8M, ensure initial state
			gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 1);

		// Switch CLK to 8M
		if (!is_front_camera) {
			// vga_mclk_en switch to VGA, ensure initial state
				gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 0);
		// Switch CLK to 8M
					gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 1);
		// Sensor power on ---
			} else {
				// vga_mclk_en switch to VGA, ensure initial state
					gpio_direction_output(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 1);
			// Switch CLK to 8M
						gpio_set_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en, 0);
			}
		// Sensor power on ---

                do_gettimeofday(&g_mi1040_power_tv);
		
		// Wait for I2C ready
		//ASUS_BSP +++ Peter_Lu Already lock i2c bus 70ms 
		//  msleep(10); //LiJen: tmp

			pr_info("gpio sensor_reset(%d)\n",gpio_get_value(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset)));
			pr_info("gpio isp_1p2_en(%d)\n",gpio_get_value(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en)));
			pr_info("gpio vga_mclk_en(%d)\n",gpio_get_value(imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en));
			pr_info("gpio isp_1p8_en(%d)\n",gpio_get_value(PM8921_GPIO_PM_TO_SYS(imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en)));
			pr_info("gpio ISP_SUSPEND(%d)\n",gpio_get_value(imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend));
			pr_info("gpio GPIO5(%d)\n",gpio_get_value(5));
			break;
	}

       g_mi1040_power = true;
	pr_info("%s ---\n",__func__);
	return 0;	
}


#endif 




//ASUS_BSP +++ LiJen [A68][13M][NA][Others]modify camera power error handling
int32_t mi1040_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc =0;
	//	pr_err("%s: randy power has enabled\n", __func__);
    if(g_mi1040_power == false){
        // Condif mi1040 GPIO
        rc = mi1040_power_up(s_ctrl->sensordata);
        if(rc < 0){
            pr_err("%s: config mi1040 gpio failed\n", __func__);
            goto fail;
        }
#if 0        
        // Config common GPIO
        rc = msm_sensor_power_up(s_ctrl);
        if(rc < 0){
            pr_err("%s: config common gpio failed\n", __func__);
            goto fail;
        }
#endif                 
    }else{
        pr_info("%s: power has enabled\n", __func__);
    }
    
fail:    
    return rc;
}

int32_t mi1040_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc =0;

    if(g_mi1040_power == true){
        // Disable common GPIO
        //msm_sensor_power_down(s_ctrl);
        // Disable mi1040 GPIO
        mi1040_power_down(s_ctrl->sensordata);
        msleep(100); // wait for regulator output stop
    }else{
        pr_info("%s: power has disabled\n", __func__);
    }
    
    return rc;
}
//ASUS_BSP --- LiJen [A68][13M][NA][Others]modify camera power error handling

//ASUS_BSP +++ Stimber "Implement the interface for calibration"
static int mi1040_sensor_i2c_rw(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	asus_sensor_i2c_rw_t i2c_reg_data;
	long   rc = 0;
	int ret=0;
	//pr_info("%s +++ \n",__func__);
	ret = copy_from_user(&i2c_reg_data,
		(void *)argp,
		sizeof(asus_sensor_i2c_rw_t));
	if (ret){
		pr_err("[Camera-cal] mi1040_sensor_config copy_from_user error");
		return -EFAULT;
	}
    
	//mutex_lock(s_ctrl->msm_sensor_mutex);
	//pr_info("msm_sensor_config: cfgtype = %d\n",i2c_reg_data.i2c_reg_rw_type);
	        
	switch (i2c_reg_data.i2c_reg_rw_type) {

		case ASUS_SET_I2C_REG:
			if (s_ctrl->func_tbl->sensor_set_custom_ioctl_reg == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_set_custom_ioctl_reg(i2c_reg_data.reg);	
			pr_info("[Camera-cal] I2C_REG_SET(0x%x-->0x%x)\n", i2c_reg_data.reg.addr, i2c_reg_data.reg.val);

			break;
		case ASUS_GET_I2C_REG:
			if (s_ctrl->func_tbl->sensor_get_custom_ioctl_reg == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_get_custom_ioctl_reg(&i2c_reg_data.reg);
			pr_info("[Camera-cal] I2C_REG_GET(0x%x-->0x%x)\n", i2c_reg_data.reg.addr, i2c_reg_data.reg.val);

			if (copy_to_user((void *)argp, &i2c_reg_data, sizeof(asus_sensor_i2c_rw_t))){
				rc = -EFAULT;
			}
			
			break;
		default:
			pr_info("[Camera-cal] cfgtype = %d is not supported!!\n", i2c_reg_data.i2c_reg_rw_type);
			rc = -EFAULT;
			break;
	}

	//mutex_unlock(s_ctrl->msm_sensor_mutex);
	//pr_info("%s --- \n",__func__);
	return rc;
}

int32_t mi1040_sensor_set_custom_ioctl_reg(register_setting_A68 reg)
{
	 int32_t err;
	 
	 err = sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client, reg.addr, reg.val);        
	 if (err){
	 	return err;
	 }  
	 
	 return 0;
}

int32_t mi1040_sensor_get_custom_ioctl_reg(register_setting_A68 *reg)
{
	int32_t err;
	u16 val = 0;

	err = sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client, reg->addr, &val);
	
	if (err){
		return err;
    } 
	
	reg->val = val;		

	return 0;
}
//ASUS_BSP --- Stimber "Implement the interface for calibration"


int32_t mi1040_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;  
//       u16 val = 0xff;
//       u16 val2 = 0xab;
	struct msm_sensor_ctrl_t *s_ctrl;
	pr_info("%s +++ \n",__func__);
	CDBG("%s_i2c_probe called\n", client->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		rc = -EFAULT;
              goto probe_fail;
	}

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensor_i2c_addr;
	} else {
		rc = -EFAULT;
              goto probe_fail;
	}

	s_ctrl->sensordata = client->dev.platform_data;
       imx091_s_ctrl.sensordata = client->dev.platform_data;

	switch (g_A68_hwID)
	{
		case A68_EVB:
			imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en = 87;
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en = 10;	  //PM(10)
			imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset = 12; 	  //PM(12)
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en = 11; 	  //PM(11)
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend= 1; 	  //PM(1)
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_int = 13; 		  //PM(13)
			imx091_s_ctrl.sensordata->sensor_platform_info->fled_driver_ent = 42; //PM(42)
			pr_info("In %s, g_A68_hwID==A60K_EVB\n", __func__);
			break;
		case A68_SR1_1:
		case A68_SR1_2:
		case A68_SR2:
		case A68_ER1:
		case A68_ER2:
		case A68_ER3:
		case A68_PR:
		case A68_PR2:
		case A68_MP:
		case A68_CD:
			imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en = 87;
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en = 10; 	  //PM(10)
			imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset = 12;    //PM(12)
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en = 11;      //PM(11)
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend= 34;
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_int = 31;
			imx091_s_ctrl.sensordata->sensor_platform_info->fled_driver_ent = 55;
			pr_info("In %s, g_A68_hwID>=A68_SR1_1\n", __func__);
			break;
                case A80_EVB:
                case A80_SR1:
                case A80_SR2:
                case A80_SR3:
                case A80_SR4:
                case A80_SR5:                    
                case A80_ER:
                case A80_PR:                    
                default:
			imx091_s_ctrl.sensordata->sensor_platform_info->vga_mclk_en = 87;
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p2_en = 10; 	  //PM(10)
			imx091_s_ctrl.sensordata->sensor_platform_info->sensor_reset = 12;    //PM(12)
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_1p8_en = 11;      //PM(11)
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_suspend= 54;
			imx091_s_ctrl.sensordata->sensor_platform_info->isp_int = 31;
			imx091_s_ctrl.sensordata->sensor_platform_info->fled_driver_ent = 55;
			//_info("In %s, g_A68_hwID>=A68_SR1_1\n", __func__);
			break;            
	}		
			
#if 0
	rc = s_ctrl->func_tbl->sensor_power_up(&imx091_s_ctrl);
	if (rc < 0)
		goto probe_fail;

       //LiJen: check icatch ISP I2C R/W
       get_fw_version_in_isp();

       sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client , 0x1300, &val);
       pr_info("%s LiJen check icatch ISP i2c read val=0x00? (%x)\n",__func__,val);

       sensor_write_reg(imx091_s_ctrl.sensor_i2c_client->client , 0x1300, val2);
       sensor_read_reg(imx091_s_ctrl.sensor_i2c_client->client , 0x1300, &val);
       pr_info("%s LiJen check icatch ISP i2c write val=0xab?(%x)\n",__func__,val);         
#endif              
        
	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);

	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	goto power_down;
    
probe_fail:
	CDBG("%s_i2c_probe failed\n", client->name);
power_down:
#if 0
        s_ctrl->func_tbl->sensor_power_down(&imx091_s_ctrl);
#endif
        pr_info("%s --- \n",__func__);
	return rc;
}

static int32_t mi1040_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
			uint16_t gain, uint32_t line, int32_t luma_avg, uint16_t fgain)		
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);
	int_time[0] = line >> 12;
	int_time[1] = line >> 4;
	int_time[2] = line << 4;
	msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr-1,
		&int_time[0], 3);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"

static const struct i2c_device_id mi1040_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&imx091_s_ctrl},
	{ }
};

static struct i2c_driver mi1040_i2c_driver = {
	.id_table = mi1040_i2c_id,
	.probe  = mi1040_sensor_i2c_probe,  //ASUS_BSP LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client mi1040_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

void create_mi1040_proc_file(void);	//ASUS_BSP Stimber "Add ATD proc interface"

static int __init mi1040_sensor_init_module(void)
{
	return i2c_add_driver(&mi1040_i2c_driver);
}

static struct v4l2_subdev_core_ops mi1040_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops mi1040_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops mi1040_subdev_ops = {
	.core = &mi1040_subdev_core_ops,
	.video  = &mi1040_subdev_video_ops,
};


static struct msm_sensor_fn_t mi1040_func_tbl = {
//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	.sensor_start_stream = mi1040_sensor_start_stream,
	.sensor_stop_stream = mi1040_sensor_stop_stream,
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
       .sensor_match_id = mi1040_sensor_match_id,
	.sensor_write_exp_gain = mi1040_write_exp_gain,
	.sensor_write_snapshot_exp_gain = mi1040_write_exp_gain,	
	.sensor_csi_setting = msm_sensor_setting,
       .sensor_setting = mi1040_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,//mi1040_sensor_set_sensor_mode,
	.sensor_mode_init = mi1040_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = mi1040_sensor_config,
	.sensor_power_up = mi1040_sensor_power_up,
	.sensor_power_down = mi1040_sensor_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_set_isp_wb_mode = iCatch_set_wb_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement WB mode"
	.sensor_set_isp_ev_mode =  iCatch_set_ev_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement EV mode"
	.sensor_set_isp_iso_mode =  iCatch_set_iso_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement ISO mode"
	.sensor_set_isp_flicker_mode =  iCatch_set_flicker_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement FLICKER mode"
	.sensor_set_isp_aeclock_mode =  iCatch_set_acelock_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement AECLOCK mode"
	.sensor_set_isp_awblock_mode =  iCatch_set_awblock_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement AWBLOCK mode"
	.sensor_set_isp_caf_mode =  iCatch_set_caf_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement CAF mode"	
	.sensor_set_isp_scene_mode =  iCatch_set_scene_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement SCENE mode"	
	.sensor_set_isp_effect_mode = iCatch_set_effect_mode, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement EFFECT mode"	
	.sensor_set_isp_aura_value = iCatch_set_aura_value,
	.sensor_set_isp_general_cmd = iCatch_set_general_cmd, //ASUS_BSP LiJen "[A68][13M][NA][Others]implement general command"
	.sensor_get_isp_exif = iCatch_get_exif,	//ASUS_BSP Stimber "Implement EXIF info for camera with ISP"
	
//ASUS_BSP +++ Stimber "Implement the interface for calibration"
	.sensor_i2c_rw = mi1040_sensor_i2c_rw,
	.sensor_set_custom_ioctl_reg = mi1040_sensor_set_custom_ioctl_reg,
	.sensor_get_custom_ioctl_reg = mi1040_sensor_get_custom_ioctl_reg,
//ASUS_BSP --- Stimber "Implement the interface for calibration"

#if 0 //TODO: for iCatch	
	.sensor_isp_af_start = fjm6mo_start_AF,
	.sensor_get_isp_af_result = fjm6mo_get_AF_result,
	.sensor_get_isp_exif = fjm6mo_get_exif,	//ASUS_BSP Stimber "[A68][13M][NA][Other] Implement EXIF info for 13M camera with ISP"
#endif	
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
};

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
#if 0	//LiJen: ISP dosen't  need
static struct msm_sensor_reg_t mi1040_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = mi1040_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(mi1040_start_settings),
	.stop_stream_conf = mi1040_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(mi1040_stop_settings),
	.group_hold_on_conf = mi1040_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(mi1040_groupon_settings),
	.group_hold_off_conf = mi1040_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(mi1040_groupoff_settings),
	.init_settings = &mi1040_init_conf[0],
	.init_size = ARRAY_SIZE(mi1040_init_conf),
	.mode_settings = &mi1040_confs[0],
	.output_settings = &mi1040_dimensions[0],
	.num_conf = ARRAY_SIZE(mi1040_confs),
};
#endif
static struct msm_sensor_reg_t mi1040_regs = {
	.output_settings = &mi1040_dimensions[0],
	.num_conf = 3,
};
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"

static struct msm_sensor_ctrl_t imx091_s_ctrl = { //ASUS_BSP LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	.msm_sensor_reg = &mi1040_regs,
	.sensor_i2c_client = &mi1040_sensor_i2c_client,
	.sensor_i2c_addr = (0x78 >> 1),		//ASUS_BSP LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
	.sensor_output_reg_addr = &mi1040_reg_addr,
	.sensor_id_info = &mi1040_id_info,
	.sensor_exp_gain_info = &mi1040_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	//.csi_params = &mi1040_csi_params_array[0],
	.msm_sensor_mutex = &mi1040_mut,
	.sensor_i2c_driver = &mi1040_i2c_driver,
	.sensor_v4l2_subdev_info = mi1040_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mi1040_subdev_info),
	.sensor_v4l2_subdev_ops = &mi1040_subdev_ops,
	.func_tbl = &mi1040_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_12HZ, //ASUS_BSP LiJen "[A68][13M][NA][Others]Full porting for 13M camera with ISP"
};

module_init(mi1040_sensor_init_module);
MODULE_DESCRIPTION("Aptina 1.2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
