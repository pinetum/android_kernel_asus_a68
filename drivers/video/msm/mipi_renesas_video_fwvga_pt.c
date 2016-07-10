/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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
#include "mipi_renesas.h"

static struct msm_panel_info pinfo;
#if 0
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
#ifdef CONFIG_FB_MSM_MDP303
	/* DSI Bit Clock at 500 MHz, 2 lane, RGB888 */
	/* regulator */
	{0x03, 0x01, 0x01, 0x00},
	/* timing   */
	{0xb9, 0x8e, 0x1f, 0x00, 0x98, 0x9c, 0x22, 0x90,
	0x18, 0x03, 0x04},
	/* phy ctrl */
	{0x7f, 0x00, 0x00, 0x00},
	/* strength */
	{0xbb, 0x02, 0x06, 0x00},
	/* pll control */
	{0x00, 0xec, 0x31, 0xd2, 0x00, 0x40, 0x37, 0x62,
	0x01, 0x0f, 0x07,
	0x05, 0x14, 0x03, 0x0, 0x0, 0x0, 0x20, 0x0, 0x02, 0x0},
#else
	/* DSI_BIT_CLK at 400MHz, 1 lane, RGB888 */
	/* regulator */
	{0x03, 0x01, 0x01, 0x00},
	/* timing   */
	{0xaa, 0x3b, 0x1b, 0x00, 0x52, 0x58, 0x20, 0x3f,
	0x2e, 0x03, 0x04},
	/* phy ctrl */
	{0x7f, 0x00, 0x00, 0x00},
	/* strength */
	{0xee, 0x00, 0x86, 0x00},
	/* pll control */
	{0x40, 0xc7, 0xb0, 0xda, 0x00, 0x50, 0x48, 0x63,
#if defined(RENESAS_FWVGA_TWO_LANE)
	0x30, 0x07, 0x03,
#else
	/* default set to 1 lane */
	0x30, 0x07, 0x07,
#endif
	0x05, 0x14, 0x03, 0x0, 0x0, 0x54, 0x06, 0x10, 0x04, 0x0},
#endif
};
#else
static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
	    /* 1920*1080, RGB888, 4 Lane 60 fps video mode */
	    /* regulator */
		{0x09, 0x08, 0x05, 0x00, 0x20},
	    /* timing */
	    {0xE8, 0x9B, 0x3B, 0x00, 0x78, 0xA6, 0x3D, 0x9C,
	    0x42, 0x03, 0x04, 0xa0},
	    /* phy ctrl */
	    {0x5f, 0x00, 0x00, 0x10},
	    /* strength */
	    {0xff, 0x00, 0x06, 0x00},
	    /* pll control */
	    {0x0, 0xB3, 0x1, 0x19, 0x00, 0x50, 0x48, 0x63,
	    0x41, 0x0f, 0x01,
	    0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
};
#endif

static int __init mipi_video_renesas_a80_init(void)
{
	int ret;

	//if (msm_fb_detect_client("mipi_video_renesas_a80"))
	//	return 0;
	printk("[Display]:mipi_video_renesas_a80_init+++\n");

	pinfo.xres = 1080;
	pinfo.yres = 1920;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
    pinfo.lcdc.h_back_porch = 52;
    pinfo.lcdc.h_front_porch = 160;
    pinfo.lcdc.h_pulse_width = 10;
    pinfo.lcdc.v_back_porch = 4; //4
    pinfo.lcdc.v_front_porch = 4;
    pinfo.lcdc.v_pulse_width = 2;
	pinfo.lcdc.border_clr = 0;	/* blk */
//AUS_BSP +++ Jason Chang "[A80][Display] chang the color of buffer underrun in user build"
#ifdef ASUS_SHIP_BUILD
	pinfo.lcdc.underflow_clr = 0x0;	/* black */
#else
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
#endif
//ASUS_BSP --- Jason Chang "[A80][Display] chang the color of buffer underrun in user build"
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 905000000;//981560000;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = FALSE;
	pinfo.mipi.hbp_power_stop = FALSE;
	pinfo.mipi.hsa_power_stop = FALSE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;


	pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_BGR;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.data_lane3 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x1A;//0x04;
	pinfo.mipi.t_clk_pre = 0x32;//0x1c;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.esc_byte_ratio = 7;

	ret = mipi_renesas_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_renesas_a80_init);
