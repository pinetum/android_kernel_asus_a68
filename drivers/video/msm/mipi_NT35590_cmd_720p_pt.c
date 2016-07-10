/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_NT35590.h"

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
	/* DSI Bit Clock at 485 MHz, 4 lane, RGB888 */
	/* regulator */
	{0x03, 0x0a, 0x04, 0x00, 0x20},
	/* timing   */
	{0xC2, 0x26, 0x25, 0x00, 0x55, 0x99, 0x27,
	0x92, 0x29, 0x03, 0x04, 0xa0},
	/* phy ctrl */
	{0x5f, 0x00, 0x00, 0x10},
	/* strength */
	{0xff, 0x00, 0x06, 0x00},
	/* pll control */
	{0x0, 0xf9, 0xb0, 0xda, 
    0x00, 0x50, 0x48, 0x63,
    0x30, 0x07, 0x00,
    0x00, 0x14, 0x03, 0x00, 0x02, 
    0x00, 0x20, 0x00, 0x01 },
};

static int mipi_cmd_nt35590_720p_pt_init(void)
{
	int ret;

	//if (msm_fb_detect_client("mipi_cmd_nt35590_720p"))
		//return 0;
printk("JASON:%s+\n",__func__);
	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 96;
	pinfo.lcdc.h_front_porch = 96;
	pinfo.lcdc.h_pulse_width = 8;
	pinfo.lcdc.v_back_porch = 4;
	pinfo.lcdc.v_front_porch = 14;
	pinfo.lcdc.v_pulse_width = 2;

	pinfo.lcdc.border_clr = 0;	/* blk */
//ASUS_BSP +++ Jason Chang "[A68][Display] change the color of buffer underrun in user build"
#ifdef ASUS_SHIP_BUILD
	pinfo.lcdc.underflow_clr = 0x0;	/* black */
#else
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
#endif
//ASUS_BSP +++ Jason Chang "[A68][Display] change the color of buffer underrun in user build"
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.clk_rate = 575000000;
	pinfo.mipi.esc_byte_ratio = 4;

	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
// ASUS_BSP +++ Tingyi "[A68][LCM] Adjust vsync timming to avoid tearing"
	pinfo.lcd.refx100 = 6200; /* adjust refx100 to prevent tearing */
// ASUS_BSP --- Tingyi "[A68][LCM] Adjust vsync timming to avoid tearing"

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = FALSE;
	pinfo.mipi.t_clk_post = 0x0C;
	pinfo.mipi.t_clk_pre = 0x28;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.te_sel = 1; /* TE from vsync gpio */
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;
	//pinfo.mipi.tx_eot_append = 0x01;
	//pinfo.mipi.rx_eot_ignore = 0x0;

	ret = mipi_nt35590_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_720P_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);
printk("JASON:%s-\n",__func__);
	return ret;
}

module_init(mipi_cmd_nt35590_720p_pt_init);
