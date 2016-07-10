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
#include <mach/socinfo.h>

#define RENESAS_CMD_DELAY 0 /* 50 */
#define RENESAS_SLEEP_OFF_DELAY 50
static struct msm_panel_common_pdata *mipi_renesas_pdata;

static struct dsi_buf renesas_tx_buf;
static struct dsi_buf renesas_rx_buf;

static int mipi_renesas_lcd_init(void);
#if 0
static char config_sleep_out[2] = {0x11, 0x00};
static char config_CMD_MODE[2] = {0x40, 0x01};
static char config_WRTXHT[7] = {0x92, 0x16, 0x08, 0x08, 0x00, 0x01, 0xe0};
static char config_WRTXVT[7] = {0x8b, 0x02, 0x02, 0x02, 0x00, 0x03, 0x60};
static char config_PLL2NR[2] = {0xa0, 0x24};
static char config_PLL2NF1[2] = {0xa2, 0xd0};
static char config_PLL2NF2[2] = {0xa4, 0x00};
static char config_PLL2BWADJ1[2] = {0xa6, 0xd0};
static char config_PLL2BWADJ2[2] = {0xa8, 0x00};
static char config_PLL2CTL[2] = {0xaa, 0x00};
static char config_DBICBR[2] = {0x48, 0x03};
static char config_DBICTYPE[2] = {0x49, 0x00};
static char config_DBICSET1[2] = {0x4a, 0x1c};
static char config_DBICADD[2] = {0x4b, 0x00};
static char config_DBICCTL[2] = {0x4e, 0x01};
/* static char config_COLMOD_565[2] = {0x3a, 0x05}; */
/* static char config_COLMOD_666PACK[2] = {0x3a, 0x06}; */
static char config_COLMOD_888[2] = {0x3a, 0x07};
static char config_MADCTL[2] = {0x36, 0x00};
static char config_DBIOC[2] = {0x82, 0x40};
static char config_CASET[7] = {0x2a, 0x00, 0x00, 0x00, 0x00, 0x01, 0xdf };
static char config_PASET[7] = {0x2b, 0x00, 0x00, 0x00, 0x00, 0x03, 0x5f };
static char config_TXON[2] = {0x81, 0x00};
static char config_BLSET_TM[2] = {0xff, 0x6c};
static char config_DSIRXCTL[2] = {0x41, 0x01};
static char config_TEON[2] = {0x35, 0x00};
static char config_TEOFF[1] = {0x34};

static char config_AGCPSCTL_TM[2] = {0x56, 0x08};

static char config_DBICADD70[2] = {0x4b, 0x70};
static char config_DBICSET_15[2] = {0x4a, 0x15};
static char config_DBICADD72[2] = {0x4b, 0x72};

static char config_Power_Ctrl_2a_cmd[3] = {0x4c, 0x40, 0x10};
static char config_Auto_Sequencer_Setting_a_cmd[3] = {0x4c, 0x00, 0x00};
static char Driver_Output_Ctrl_indx[3] = {0x4c, 0x00, 0x01};
static char Driver_Output_Ctrl_cmd[3] = {0x4c, 0x03, 0x10};
static char config_LCD_drive_AC_Ctrl_indx[3] = {0x4c, 0x00, 0x02};
static char config_LCD_drive_AC_Ctrl_cmd[3] = {0x4c, 0x01, 0x00};
static char config_Entry_Mode_indx[3] = {0x4c, 0x00, 0x03};
static char config_Entry_Mode_cmd[3] = {0x4c, 0x00, 0x00};
static char config_Display_Ctrl_1_indx[3] = {0x4c, 0x00, 0x07};
static char config_Display_Ctrl_1_cmd[3] = {0x4c, 0x00, 0x00};
static char config_Display_Ctrl_2_indx[3] = {0x4c, 0x00, 0x08};
static char config_Display_Ctrl_2_cmd[3] = {0x4c, 0x00, 0x04};
static char config_Display_Ctrl_3_indx[3] = {0x4c, 0x00, 0x09};
static char config_Display_Ctrl_3_cmd[3] = {0x4c, 0x00, 0x0c};
static char config_Display_IF_Ctrl_1_indx[3] = {0x4c, 0x00, 0x0c};
static char config_Display_IF_Ctrl_1_cmd[3] = {0x4c, 0x40, 0x10};
static char config_Display_IF_Ctrl_2_indx[3] = {0x4c, 0x00, 0x0e};
static char config_Display_IF_Ctrl_2_cmd[3] = {0x4c, 0x00, 0x00};

static char config_Panel_IF_Ctrl_1_indx[3] = {0x4c, 0x00, 0x20};
static char config_Panel_IF_Ctrl_1_cmd[3] = {0x4c, 0x01, 0x3f};
static char config_Panel_IF_Ctrl_3_indx[3] = {0x4c, 0x00, 0x22};
static char config_Panel_IF_Ctrl_3_cmd[3] = {0x4c, 0x76, 0x00};
static char config_Panel_IF_Ctrl_4_indx[3] = {0x4c, 0x00, 0x23};
static char config_Panel_IF_Ctrl_4_cmd[3] = {0x4c, 0x1c, 0x0a};
static char config_Panel_IF_Ctrl_5_indx[3] = {0x4c, 0x00, 0x24};
static char config_Panel_IF_Ctrl_5_cmd[3] = {0x4c, 0x1c, 0x2c};
static char config_Panel_IF_Ctrl_6_indx[3] = {0x4c, 0x00, 0x25};
static char config_Panel_IF_Ctrl_6_cmd[3] = {0x4c, 0x1c, 0x4e};
static char config_Panel_IF_Ctrl_8_indx[3] = {0x4c, 0x00, 0x27};
static char config_Panel_IF_Ctrl_8_cmd[3] = {0x4c, 0x00, 0x00};
static char config_Panel_IF_Ctrl_9_indx[3] = {0x4c, 0x00, 0x28};
static char config_Panel_IF_Ctrl_9_cmd[3] = {0x4c, 0x76, 0x0c};


static char config_gam_adjust_00_indx[3] = {0x4c, 0x03, 0x00};
static char config_gam_adjust_00_cmd[3] = {0x4c, 0x00, 0x00};
static char config_gam_adjust_01_indx[3] = {0x4c, 0x03, 0x01};
static char config_gam_adjust_01_cmd[3] = {0x4c, 0x05, 0x02};
static char config_gam_adjust_02_indx[3] = {0x4c, 0x03, 0x02};
static char config_gam_adjust_02_cmd[3] = {0x4c, 0x07, 0x05};
static char config_gam_adjust_03_indx[3] = {0x4c, 0x03, 0x03};
static char config_gam_adjust_03_cmd[3] = {0x4c, 0x00, 0x00};
static char config_gam_adjust_04_indx[3] = {0x4c, 0x03, 0x04};
static char config_gam_adjust_04_cmd[3] = {0x4c, 0x02, 0x00};
static char config_gam_adjust_05_indx[3] = {0x4c, 0x03, 0x05};
static char config_gam_adjust_05_cmd[3] = {0x4c, 0x07, 0x07};
static char config_gam_adjust_06_indx[3] = {0x4c, 0x03, 0x06};
static char config_gam_adjust_06_cmd[3] = {0x4c, 0x10, 0x10};
static char config_gam_adjust_07_indx[3] = {0x4c, 0x03, 0x07};
static char config_gam_adjust_07_cmd[3] = {0x4c, 0x02, 0x02};
static char config_gam_adjust_08_indx[3] = {0x4c, 0x03, 0x08};
static char config_gam_adjust_08_cmd[3] = {0x4c, 0x07, 0x04};
static char config_gam_adjust_09_indx[3] = {0x4c, 0x03, 0x09};
static char config_gam_adjust_09_cmd[3] = {0x4c, 0x07, 0x07};
static char config_gam_adjust_0A_indx[3] = {0x4c, 0x03, 0x0a};
static char config_gam_adjust_0A_cmd[3] = {0x4c, 0x00, 0x00};
static char config_gam_adjust_0B_indx[3] = {0x4c, 0x03, 0x0b};
static char config_gam_adjust_0B_cmd[3] = {0x4c, 0x00, 0x00};
static char config_gam_adjust_0C_indx[3] = {0x4c, 0x03, 0x0c};
static char config_gam_adjust_0C_cmd[3] = {0x4c, 0x07, 0x07};
static char config_gam_adjust_0D_indx[3] = {0x4c, 0x03, 0x0d};
static char config_gam_adjust_0D_cmd[3] = {0x4c, 0x10, 0x10};
static char config_gam_adjust_10_indx[3] = {0x4c, 0x03, 0x10};
static char config_gam_adjust_10_cmd[3] = {0x4c, 0x01, 0x04};
static char config_gam_adjust_11_indx[3] = {0x4c, 0x03, 0x11};
static char config_gam_adjust_11_cmd[3] = {0x4c, 0x05, 0x03};
static char config_gam_adjust_12_indx[3] = {0x4c, 0x03, 0x12};
static char config_gam_adjust_12_cmd[3] = {0x4c, 0x03, 0x04};
static char config_gam_adjust_15_indx[3] = {0x4c, 0x03, 0x15};
static char config_gam_adjust_15_cmd[3] = {0x4c, 0x03, 0x04};
static char config_gam_adjust_16_indx[3] = {0x4c, 0x03, 0x16};
static char config_gam_adjust_16_cmd[3] = {0x4c, 0x03, 0x1c};
static char config_gam_adjust_17_indx[3] = {0x4c, 0x03, 0x17};
static char config_gam_adjust_17_cmd[3] = {0x4c, 0x02, 0x04};
static char config_gam_adjust_18_indx[3] = {0x4c, 0x03, 0x18};
static char config_gam_adjust_18_cmd[3] = {0x4c, 0x04, 0x02};
static char config_gam_adjust_19_indx[3] = {0x4c, 0x03, 0x19};
static char config_gam_adjust_19_cmd[3] = {0x4c, 0x03, 0x05};
static char config_gam_adjust_1C_indx[3] = {0x4c, 0x03, 0x1c};
static char config_gam_adjust_1C_cmd[3] = {0x4c, 0x07, 0x07};
static char config_gam_adjust_1D_indx[3] = {0x4c, 0x03, 0x1D};
static char config_gam_adjust_1D_cmd[3] = {0x4c, 0x02, 0x1f};
static char config_gam_adjust_20_indx[3] = {0x4c, 0x03, 0x20};
static char config_gam_adjust_20_cmd[3] = {0x4c, 0x05, 0x07};
static char config_gam_adjust_21_indx[3] = {0x4c, 0x03, 0x21};
static char config_gam_adjust_21_cmd[3] = {0x4c, 0x06, 0x04};
static char config_gam_adjust_22_indx[3] = {0x4c, 0x03, 0x22};
static char config_gam_adjust_22_cmd[3] = {0x4c, 0x04, 0x05};
static char config_gam_adjust_27_indx[3] = {0x4c, 0x03, 0x27};
static char config_gam_adjust_27_cmd[3] = {0x4c, 0x02, 0x03};
static char config_gam_adjust_28_indx[3] = {0x4c, 0x03, 0x28};
static char config_gam_adjust_28_cmd[3] = {0x4c, 0x03, 0x00};
static char config_gam_adjust_29_indx[3] = {0x4c, 0x03, 0x29};
static char config_gam_adjust_29_cmd[3] = {0x4c, 0x00, 0x02};

static char config_Power_Ctrl_1_indx[3] = {0x4c, 0x01, 0x00};
static char config_Power_Ctrl_1b_cmd[3] = {0x4c, 0x36, 0x3c};
static char config_Power_Ctrl_2_indx[3] = {0x4c, 0x01, 0x01};
static char config_Power_Ctrl_2b_cmd[3] = {0x4c, 0x40, 0x03};
static char config_Power_Ctrl_3_indx[3] = {0x4c, 0x01, 0x02};
static char config_Power_Ctrl_3a_cmd[3] = {0x4c, 0x00, 0x01};
static char config_Power_Ctrl_4_indx[3] = {0x4c, 0x01, 0x03};
static char config_Power_Ctrl_4a_cmd[3] = {0x4c, 0x3c, 0x58};
static char config_Power_Ctrl_6_indx[3] = {0x4c, 0x01, 0x0c};
static char config_Power_Ctrl_6a_cmd[3] = {0x4c, 0x01, 0x35};

static char config_Auto_Sequencer_Setting_b_cmd[3] = {0x4c, 0x00, 0x02};

static char config_Panel_IF_Ctrl_10_indx[3] = {0x4c, 0x00, 0x29};
static char config_Panel_IF_Ctrl_10a_cmd[3] = {0x4c, 0x03, 0xbf};
static char config_Auto_Sequencer_Setting_indx[3] = {0x4c, 0x01, 0x06};
static char config_Auto_Sequencer_Setting_c_cmd[3] = {0x4c, 0x00, 0x03};
static char config_Power_Ctrl_2c_cmd[3] = {0x4c, 0x40, 0x10};

static char config_VIDEO[2] = {0x40, 0x00};

static char config_Panel_IF_Ctrl_10_indx_off[3] = {0x4C, 0x00, 0x29};

static char config_Panel_IF_Ctrl_10b_cmd_off[3] = {0x4C, 0x00, 0x02};

static char config_Power_Ctrl_1a_cmd[3] = {0x4C, 0x30, 0x00};

static struct dsi_cmd_desc renesas_sleep_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, RENESAS_SLEEP_OFF_DELAY,
		sizeof(config_sleep_out), config_sleep_out }
};

static struct dsi_cmd_desc renesas_display_off_cmds[] = {
	/* Choosing Command Mode */
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_CMD_MODE), config_CMD_MODE },

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_indx),
			config_Auto_Sequencer_Setting_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_b_cmd),
			config_Auto_Sequencer_Setting_b_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY * 2,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	/* After waiting >= 5 frames, turn OFF RGB signals
	This is done by on DSI/MDP (depends on Vid/Cmd Mode.  */
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_indx),
			config_Auto_Sequencer_Setting_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_a_cmd),
			config_Auto_Sequencer_Setting_a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_10_indx_off),
			config_Panel_IF_Ctrl_10_indx_off},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_10b_cmd_off),
				config_Panel_IF_Ctrl_10b_cmd_off},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_1_indx),
				config_Power_Ctrl_1_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_1a_cmd),
				config_Power_Ctrl_1a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_TEOFF), config_TEOFF},
};

static struct dsi_cmd_desc renesas_display_on_cmds[] = {
	/* Choosing Command Mode */
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_CMD_MODE), config_CMD_MODE },
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_WRTXHT), config_WRTXHT },
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_WRTXVT), config_WRTXVT },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_PLL2NR), config_PLL2NR },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_PLL2NF1), config_PLL2NF1 },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_PLL2NF2), config_PLL2NF2 },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_PLL2BWADJ1), config_PLL2BWADJ1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_PLL2BWADJ2), config_PLL2BWADJ2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_PLL2CTL), config_PLL2CTL},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICBR), config_DBICBR},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICTYPE), config_DBICTYPE},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET1), config_DBICSET1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD), config_DBICADD},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICCTL), config_DBICCTL},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_COLMOD_888), config_COLMOD_888},
	/* Choose config_COLMOD_565 or config_COLMOD_666PACK for other modes */
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_MADCTL), config_MADCTL},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBIOC), config_DBIOC},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_CASET), config_CASET},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_PASET), config_PASET},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DSIRXCTL), config_DSIRXCTL},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_TEON), config_TEON},
	{DTYPE_DCS_WRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_TXON), config_TXON},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_BLSET_TM), config_BLSET_TM},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_AGCPSCTL_TM), config_AGCPSCTL_TM},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_1_indx), config_Power_Ctrl_1_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_1a_cmd), config_Power_Ctrl_1a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_2_indx), config_Power_Ctrl_2_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_2a_cmd), config_Power_Ctrl_2a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_indx),
			config_Auto_Sequencer_Setting_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_a_cmd),
			config_Auto_Sequencer_Setting_a_cmd },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(Driver_Output_Ctrl_indx), Driver_Output_Ctrl_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(Driver_Output_Ctrl_cmd),
			Driver_Output_Ctrl_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_LCD_drive_AC_Ctrl_indx),
			config_LCD_drive_AC_Ctrl_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_LCD_drive_AC_Ctrl_cmd),
			config_LCD_drive_AC_Ctrl_cmd },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Entry_Mode_indx),
			config_Entry_Mode_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Entry_Mode_cmd),
			config_Entry_Mode_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_Ctrl_1_indx),
			config_Display_Ctrl_1_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_Ctrl_1_cmd),
			config_Display_Ctrl_1_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_Ctrl_2_indx),
			config_Display_Ctrl_2_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_Ctrl_2_cmd),
			config_Display_Ctrl_2_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_Ctrl_3_indx),
			config_Display_Ctrl_3_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_Ctrl_3_cmd),
			config_Display_Ctrl_3_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_IF_Ctrl_1_indx),
			config_Display_IF_Ctrl_1_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_IF_Ctrl_1_cmd),
			config_Display_IF_Ctrl_1_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_IF_Ctrl_2_indx),
			config_Display_IF_Ctrl_2_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Display_IF_Ctrl_2_cmd),
			config_Display_IF_Ctrl_2_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_1_indx),
			config_Panel_IF_Ctrl_1_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_1_cmd),
			config_Panel_IF_Ctrl_1_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_3_indx),
			config_Panel_IF_Ctrl_3_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_3_cmd),
			config_Panel_IF_Ctrl_3_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_4_indx),
			config_Panel_IF_Ctrl_4_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_4_cmd),
			config_Panel_IF_Ctrl_4_cmd },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_5_indx),
			config_Panel_IF_Ctrl_5_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_5_cmd),
			config_Panel_IF_Ctrl_5_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_6_indx),
			config_Panel_IF_Ctrl_6_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_6_cmd),
			config_Panel_IF_Ctrl_6_cmd },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_8_indx),
			config_Panel_IF_Ctrl_8_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_8_cmd),
			config_Panel_IF_Ctrl_8_cmd },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_9_indx),
			config_Panel_IF_Ctrl_9_indx },
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_9_cmd),
			config_Panel_IF_Ctrl_9_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_00_indx),
			config_gam_adjust_00_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_00_cmd),
			config_gam_adjust_00_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_01_indx),
			config_gam_adjust_01_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_01_cmd),
			config_gam_adjust_01_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_02_indx),
			config_gam_adjust_02_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_02_cmd),
			config_gam_adjust_02_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_03_indx),
			config_gam_adjust_03_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_03_cmd),
			config_gam_adjust_03_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_04_indx), config_gam_adjust_04_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_04_cmd), config_gam_adjust_04_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},


	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_05_indx), config_gam_adjust_05_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_05_cmd), config_gam_adjust_05_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_06_indx), config_gam_adjust_06_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_06_cmd), config_gam_adjust_06_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_07_indx), config_gam_adjust_07_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_07_cmd), config_gam_adjust_07_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_08_indx), config_gam_adjust_08_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_08_cmd), config_gam_adjust_08_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_09_indx), config_gam_adjust_09_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_09_cmd), config_gam_adjust_09_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0A_indx), config_gam_adjust_0A_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0A_cmd), config_gam_adjust_0A_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0B_indx), config_gam_adjust_0B_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0B_cmd), config_gam_adjust_0B_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0C_indx), config_gam_adjust_0C_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0C_cmd), config_gam_adjust_0C_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0D_indx), config_gam_adjust_0D_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_0D_cmd), config_gam_adjust_0D_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_10_indx), config_gam_adjust_10_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_10_cmd), config_gam_adjust_10_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_11_indx), config_gam_adjust_11_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_11_cmd), config_gam_adjust_11_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_12_indx), config_gam_adjust_12_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_12_cmd), config_gam_adjust_12_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_15_indx), config_gam_adjust_15_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_15_cmd), config_gam_adjust_15_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_16_indx), config_gam_adjust_16_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_16_cmd), config_gam_adjust_16_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_17_indx), config_gam_adjust_17_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_17_cmd), config_gam_adjust_17_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_18_indx), config_gam_adjust_18_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_18_cmd), config_gam_adjust_18_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_19_indx), config_gam_adjust_19_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_19_cmd), config_gam_adjust_19_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_1C_indx), config_gam_adjust_1C_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_1C_cmd), config_gam_adjust_1C_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_1D_indx), config_gam_adjust_1D_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_1D_cmd), config_gam_adjust_1D_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_20_indx), config_gam_adjust_20_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_20_cmd), config_gam_adjust_20_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_21_indx), config_gam_adjust_21_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_21_cmd), config_gam_adjust_21_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},


	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_22_indx), config_gam_adjust_22_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_22_cmd), config_gam_adjust_22_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_27_indx), config_gam_adjust_27_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_27_cmd), config_gam_adjust_27_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_28_indx), config_gam_adjust_28_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_28_cmd), config_gam_adjust_28_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_29_indx), config_gam_adjust_29_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_gam_adjust_29_cmd), config_gam_adjust_29_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_1_indx), config_Power_Ctrl_1_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_1b_cmd), config_Power_Ctrl_1b_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_2_indx), config_Power_Ctrl_2_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_2b_cmd), config_Power_Ctrl_2b_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_3_indx), config_Power_Ctrl_3_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_3a_cmd), config_Power_Ctrl_3a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_4_indx), config_Power_Ctrl_4_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_4a_cmd), config_Power_Ctrl_4a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_6_indx), config_Power_Ctrl_6_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_6a_cmd), config_Power_Ctrl_6a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_indx),
			config_Auto_Sequencer_Setting_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_b_cmd),
			config_Auto_Sequencer_Setting_b_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_10_indx),
			config_Panel_IF_Ctrl_10_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Panel_IF_Ctrl_10a_cmd),
			config_Panel_IF_Ctrl_10a_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_indx),
			config_Auto_Sequencer_Setting_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Auto_Sequencer_Setting_c_cmd),
			config_Auto_Sequencer_Setting_c_cmd},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},

	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD70), config_DBICADD70},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_2_indx),
			config_Power_Ctrl_2_indx},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICSET_15), config_DBICSET_15},
	{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_DBICADD72), config_DBICADD72},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_Power_Ctrl_2c_cmd),
			config_Power_Ctrl_2c_cmd},

	{DTYPE_DCS_WRITE1, 1, 0, 0, 0/* RENESAS_CMD_DELAY */,
		sizeof(config_DBICSET_15), config_DBICSET_15},

};

static char config_WRTXHT2[7] = {0x92, 0x15, 0x05, 0x0F, 0x00, 0x01, 0xe0};
static char config_WRTXVT2[7] = {0x8b, 0x14, 0x01, 0x14, 0x00, 0x03, 0x60};

static struct dsi_cmd_desc renesas_hvga_on_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_WRTXHT2), config_WRTXHT2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_WRTXVT2), config_WRTXVT2},
};

static struct dsi_cmd_desc renesas_video_on_cmds[] = {
{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_VIDEO), config_VIDEO}
};

static struct dsi_cmd_desc renesas_cmd_on_cmds[] = {
{DTYPE_DCS_WRITE1, 1, 0, 0, RENESAS_CMD_DELAY,
		sizeof(config_CMD_MODE), config_CMD_MODE},
};
#else
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/file.h>
static mm_segment_t oldfs;

#define SHARP_CABC_PROC_FILE  "driver/cabc"
//+++ ASUS_BSP: for mini-porting
#include <linux/mutex.h>
//static char bl_value[] = {0x51, 0x64};      //write brightness
static struct mutex cmd_mutex;//Mickey+++
struct dcs_cmd_req bl_cmdreq;
struct platform_device *g_pdev;
extern int g_fb0_dsi_block;
extern int asus_set_bl_brightness(int);
extern void asus_mdp_write_lut(int mode);

#ifdef ASUS_A80_PROJECT
extern void nvt71890_cabc_set(bool bOn);
#endif
extern bool asus_padstation_exist_realtime(void);
int g_displayOn = false;
int g_frame_update = 0;
int g_CE_update = 0;
//--- ASUS_BSP: for mini-porting
//ASUS_BSP +++ Jason Chang "[A80][Backlight] Add interface for backlight driver"
static bool bl_on = FALSE;
int g_backlightValue = 0;
static struct proc_dir_entry *cabc_proc_file;
struct msm_fb_data_type *g_mfd;
static char nop[2] = {0x0, 0x0};
static char unlock_manufacture[2] = {0xB0, 0x04};
static char remove_NVM_reload[2] = {0xD6, 0x01};
static char cabc_bk_ctrl1[8] = {0xB9, 0x03, 0x30, 0x10, 0x30, 0x9F, 0x1F, 0x80}; //for movie & still
static char cabc_bk_ctrl2[8] = {0xCE, 0x0, 0x3, 0x4, 0xC1, 0x5E, 0xB2, 0x3}; //20k, tri-linear (lower_mask is set to 0x2 for DBV range 0x000 ~ 0x3FF)
static char cabc_bl_min[3] = {0x5E, 0x0, 0x1E};
static char cabc_ctrl[2] = {0x55, 0x3};
static char a80_init_bl_val[3] = {0x51, 0x1, 0x91};
static char a80_bl_val[3] = {0x51, 0x1, 0x91};			//maximum brightness is 1023 (0x3FF)
//ASUS_BSP --- Jason Chang "[A80][Backlight] Add interface for backlight driver"
//static char a80_write_brightness[3] = {0x51, 0x0F, 0xFF};
static char a80_ctrl_display[2] = {0x53, 0x24};
static char a80_hsync[4] = {0xc3, 0x1, 0x0, 0xa};   //hsync signal for touch
//static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */

static struct dsi_cmd_desc a80_cabc_cmd[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(cabc_ctrl), cabc_ctrl},
};
char a80_ce_setting[33] = {0xCA, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x08,
     0x20, 0x80, 0x80, 0x0A, 0x4A, 0x37, 0xA0, 0x55, 0xF8, 0x0C,
     0x0C, 0x20, 0x10, 0x3F, 0x3F, 0x00, 0x00, 0x10, 0x10, 0x3F,
     0x3F, 0x3F, 0x3F};

//ASUS_BSP +++ Jason Chang "[A80][Backlight] Add interface for backlight driver"
static struct dsi_cmd_desc renesas_brightness_set[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(a80_bl_val), a80_bl_val},
//ASUS_BSP --- Jason Chang "[A80][Backlight] Add interface for backlight driver"
};
static struct dsi_cmd_desc a80_init_cmds[] = {
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(unlock_manufacture), unlock_manufacture}, //must be generic write
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(nop), nop},
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(nop), nop},
    {DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(remove_NVM_reload), remove_NVM_reload},   //must be generic write
    //{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(a80_ce_setting), a80_ce_setting},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(a80_ctrl_display), a80_ctrl_display},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(a80_init_bl_val), a80_init_bl_val},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(cabc_ctrl), cabc_ctrl},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(cabc_bl_min), cabc_bl_min},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(cabc_bk_ctrl1), cabc_bk_ctrl1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(cabc_bk_ctrl2), cabc_bk_ctrl2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(a80_hsync), a80_hsync},
    {DTYPE_DCS_WRITE,  1, 0, 0, 0, sizeof(display_on), display_on},
    {DTYPE_DCS_WRITE,  1, 0, 0, 120, sizeof(exit_sleep), exit_sleep},    //wait 6 frame
};

static struct dsi_cmd_desc a80_display_off_cmds[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 20,
        sizeof(display_off), display_off},
    {DTYPE_DCS_WRITE, 1, 0, 0, 40,
        sizeof(enter_sleep), enter_sleep},
};
#endif

//ASUS_BSP +++ Jason Chang "[A80][Backlight] Add interface for backlight driver"
int renesas_set_backlight(int value)
{
    a80_bl_val[2] = value;

    if (value > 255)					//maximum brightness is 0x3FF, calculate third bit.
	a80_bl_val[1] = value/256;
    else							//if value is less that 255, third bit must be 0
	a80_bl_val[1] = 0;

    if (value == 0) {
        gpio_set_value(2, 0);
	    bl_on = FALSE;
        return 0;
    }

    if (bl_on == FALSE) {
        gpio_set_value(2, 1);
	    bl_on = TRUE;
    }

    if (value >= 84) {  // ignore for timeout suspend bl, min duty is 8.24% at 0x3FF DBV, min value is 84 (dim value is 80)
	if (value > 255)					//maximum brightness is 0x3FF, calculate third bit.
		a80_init_bl_val[1] = value/256;
	else								//if value is less that 255, third bit must be 0
		a80_init_bl_val[1] = 0;
	a80_init_bl_val[2] = value; 			//keep it for resume initial bl value
    }

    //Mickey+++, ignore the same backlight value command
    if (value == g_backlightValue) {
        printk("[BL] got same backlight value, ignore!!\n");
        return 0;
    }
    //Mickey---

    mutex_lock(&cmd_mutex);//Mickey+++
    if (!g_fb0_dsi_block && g_mfd->panel_power_on){// && g_frame_update) { //Mickey+++, don't allow any backlight command when panel is off

        //Mickey+++, force using high speed mode to issue backlight command
        mipi_set_tx_power_mode(0);
        bl_cmdreq.cmds = renesas_brightness_set;
        bl_cmdreq.cmds_cnt = ARRAY_SIZE(renesas_brightness_set);
        bl_cmdreq.flags = CMD_REQ_COMMIT;
        bl_cmdreq.rlen = 0;
        bl_cmdreq.cb = NULL;

        mipi_dsi_cmdlist_put(&bl_cmdreq);
        mipi_set_tx_power_mode(1);
        printk("[BL] brightness(%d) done\n", value);
        //Mickey---
        g_backlightValue = value;//Mickey+++
    }
    else {
        printk("[BL] bk(%d) return due to dsi_block(%d), panel_power_on(%d), g_frame_update(%d)\n", value, g_fb0_dsi_block, g_mfd->panel_power_on, g_frame_update);
    }
    mutex_unlock(&cmd_mutex);//Mickey+++

    return 0;
}
//ASUS_BSP --- Jason Chang "[A80][Backlight] Add interface for backlight driver"
//ASUS_BSP +++ Jason Chang
void renesas_set_ce(char *ce_params)
{
    static int first_ce_count = 0;
    struct dcs_cmd_req cmdreq;
    struct dsi_cmd_desc a80_ce_cmd[] = {
        {DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(unlock_manufacture), unlock_manufacture},
        {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(nop), nop},
        {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(nop), nop},
        {DTYPE_GEN_WRITE1, 1, 0, 0, 0, sizeof(remove_NVM_reload), remove_NVM_reload},
        {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(a80_ce_setting), a80_ce_setting},
    };

    memcpy(a80_ce_setting, ce_params, sizeof(a80_ce_setting));

    first_ce_count++;   //AP set vivid, saturation and hue at booting, we only apply one

    mutex_lock(&cmd_mutex);//Mickey+++

    printk("[Display][CE] %s +++\n", __func__);

    if (g_mfd->panel_power_on) {
        if (!g_fb0_dsi_block && (first_ce_count >= 3)) {

            mipi_set_tx_power_mode(0);

            cmdreq.cmds = a80_ce_cmd;
            cmdreq.cmds_cnt = ARRAY_SIZE(a80_ce_cmd);
            cmdreq.flags = CMD_REQ_COMMIT;
            cmdreq.rlen = 0;
            cmdreq.cb = NULL;
            mipi_dsi_cmdlist_put(&cmdreq);

            mipi_set_tx_power_mode(1);

            g_CE_update = 1;        //must set here to make sure pan display never update again
        }
        else {
            printk("[Display] Set CE fail due to dsi_block(%d), first_ce_count(%d)\n", g_fb0_dsi_block, first_ce_count);
        }
    }

    printk("[Display][CE] %s ---\n", __func__);

    mutex_unlock(&cmd_mutex);//Mickey+++

    return;
}
EXPORT_SYMBOL(renesas_set_ce);
//ASUS_BSP --- Jason Chang
static int mipi_renesas_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	//struct msm_panel_info *pinfo;
	struct dcs_cmd_req cmdreq;

	static bool bFirst = true; // +++ ASUS_BSP : miniporting

	mfd = platform_get_drvdata(pdev);
	mipi  = &mfd->panel_info.mipi;

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
#if 0
	mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_sleep_off_cmds,
			ARRAY_SIZE(renesas_sleep_off_cmds));

	mipi_set_tx_power_mode(1);
	mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_display_on_cmds,
			ARRAY_SIZE(renesas_display_on_cmds));

	if (cpu_is_msm7x25a() || cpu_is_msm7x25aa() || cpu_is_msm7x25ab()) {
		mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_hvga_on_cmds,
			ARRAY_SIZE(renesas_hvga_on_cmds));
	}

	if (mipi->mode == DSI_VIDEO_MODE)
		mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_video_on_cmds,
			ARRAY_SIZE(renesas_video_on_cmds));
	else
		mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_cmd_on_cmds,
			ARRAY_SIZE(renesas_cmd_on_cmds));

	mipi_set_tx_power_mode(0);
#else

	mutex_lock(&cmd_mutex); //Mickey+++
	mipi = &mfd->panel_info.mipi;

	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

    if (bFirst) {
        if (g_A68_hwID >= A80_EVB) {
            unlock_manufacture[1] = 0x4;

	    printk("[Display] A80 first initial command ++.\n");
            mipi_set_tx_power_mode(0);
            mipi_dsi_cmds_tx(&renesas_tx_buf, a80_init_cmds, ARRAY_SIZE(a80_init_cmds));
            mipi_set_tx_power_mode(1);

            g_backlightValue = a80_init_bl_val[1]<<8 | a80_init_bl_val[2];//Mickey+++
            printk("[Display] A80 first initial command --. BL(%d) \n", g_backlightValue);
        }
        printk("[Display] sharp panel init cmd\n");

        bFirst = false;
    }
    else {
        unlock_manufacture[1] = 0x4;

        printk("[Display] A80 display on ++.\n");
        mipi_set_tx_power_mode(0);
        mipi_dsi_cmds_tx(&renesas_tx_buf, a80_init_cmds, ARRAY_SIZE(a80_init_cmds));
        mipi_set_tx_power_mode(1);

        g_backlightValue = a80_init_bl_val[1]<<8 | a80_init_bl_val[2];//Mickey+++
        printk("[Display] A80 display on --. BL(%d)\n", g_backlightValue);

        gpio_set_value(2, 1);
    }

    g_displayOn = true;
	mutex_unlock(&cmd_mutex);   //Mickey+++
#endif
	return 0;
}

static int mipi_renesas_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct dcs_cmd_req cmdreq;


	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
#if 0
	mipi_dsi_cmds_tx(&renesas_tx_buf, renesas_display_off_cmds,
			ARRAY_SIZE(renesas_display_off_cmds));
#else
//+++ ASUS_BSP: for mini-porting
	mutex_lock(&cmd_mutex);//Mickey+++
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

        if (g_displayOn) {
            unlock_manufacture[1] = 0x2;

            printk("[Display] A80 display off ++\n");
            mipi_set_tx_power_mode(0);
            mipi_dsi_cmds_tx(&renesas_tx_buf, a80_display_off_cmds,
                ARRAY_SIZE(a80_display_off_cmds));
            mipi_set_tx_power_mode(1);
            printk("[Display] A80 display off --\n");
        }

    gpio_set_value(2, 0);
    g_displayOn = false;
    g_CE_update = 0;
//--- ASUS_BSP: for mini-porting
#endif
    mutex_unlock(&cmd_mutex);//Mickey+++
	return 0;
}

//ASUS_BSP +++ Jason Chang "[A80][Backlight]enable CABC"
//ASUS_BSP +++ Jason Chang "send display off in HS"
void asus_mipi_display_off(void)
{
    mipi_renesas_lcd_off(g_pdev);
}
EXPORT_SYMBOL(asus_mipi_display_off);
//ASUS_BSP --- Jason Chang "send display off in HS"
static void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
    set_fs(oldfs);
}

static ssize_t cabc_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char messages[256];

    memset(messages, 0, sizeof(messages));

    if (len > 256)
        len = 256;
    if (copy_from_user(messages, buff, len))
        return -EFAULT;

    initKernelEnv();

        if(strncmp(messages, "0", 1) == 0)  //off
            sharp_set_cabc(0);
        else if(strncmp(messages, "1", 1) == 0) //ui
            sharp_set_cabc(1);
        else if(strncmp(messages, "2", 1) == 0) //still
            sharp_set_cabc(2);
        else if(strncmp(messages, "3", 1) == 0) //moving
            sharp_set_cabc(3);
        else if(strncmp(messages, "off", 3) == 0) //turn off cabc all function
            sharp_set_cabc(4);

    deinitKernelEnv(); 
    return len;
}

static struct file_operations cabc_proc_ops = {
    .write = cabc_proc_write,
};

static void create_cabc_proc_file(void)
{
    cabc_proc_file = create_proc_entry(SHARP_CABC_PROC_FILE, 0666, NULL);

    if (cabc_proc_file) {
        cabc_proc_file->proc_fops = &cabc_proc_ops;
    }
}

void sharp_set_cabc(int mode)
{
    struct dcs_cmd_req cmdreq;

    if (mode == 0) {
        a80_ctrl_display[1] = 0x24;
    }
    else {
        a80_ctrl_display[1] = 0x2C;
    }

    cabc_ctrl[1] = cabc_ctrl[1] & 0xf0;
    cabc_ctrl[1] += mode;
    printk("[Display][CABC] write cabc mode = 0x%x\n", cabc_ctrl[1]);

    cmdreq.cmds = a80_cabc_cmd;
    cmdreq.cmds_cnt = ARRAY_SIZE(a80_cabc_cmd);

    cmdreq.flags = CMD_REQ_COMMIT;
    cmdreq.rlen = 0;
    cmdreq.cb = NULL;
    mipi_dsi_cmdlist_put(&cmdreq);

}
//ASUS_BSP --- Jason Chang "[A80][Backlight]enable CABC"
static int __devinit mipi_renesas_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	//static struct mipi_dsi_phy_ctrl *phy_settings;
	//static char dlane_swap;

	if (pdev->id == 0) {
		mipi_renesas_pdata = pdev->dev.platform_data;
/*
		if (mipi_renesas_pdata
			&& mipi_renesas_pdata->phy_ctrl_settings) {
			phy_settings = (mipi_renesas_pdata->phy_ctrl_settings);
		}

		if (mipi_renesas_pdata
			&& mipi_renesas_pdata->dlane_swap) {
			dlane_swap = (mipi_renesas_pdata->dlane_swap);
		}
*/
		return 0;
	}

	current_pdev = msm_fb_add_device(pdev);

	if (current_pdev) {
		mfd = platform_get_drvdata(current_pdev);
		if (!mfd)
			return -ENODEV;
		if (mfd->key != MFD_KEY)
			return -EINVAL;

		mipi  = &mfd->panel_info.mipi;
/*
		if (phy_settings != NULL)
			mipi->dsi_phy_db = phy_settings;

		if (dlane_swap)
			mipi->dlane_swap = dlane_swap;
*/
        g_mfd = mfd;// +++ ASUS_BSP : miniporting
	}
//ASUS_BSP +++ Jason Chang "[A80][Backlight]enable CABC"
    g_pdev = pdev;
    //printk("[A80][Display] panel_id(%d)\n", gpio_get_value(PM8921_GPIO_PM_TO_SYS(5)));
	create_cabc_proc_file();
//ASUS_BSP --- Jason Chang "[A80][Backlight]enable CABC"
	return 0;
}

static void mipi_renesas_set_backlight(struct msm_fb_data_type *mfd)
{
	int ret = -EPERM;
	int bl_level;

	bl_level = mfd->bl_level;

	if (mipi_renesas_pdata && mipi_renesas_pdata->pmic_backlight)
		ret = mipi_renesas_pdata->pmic_backlight(bl_level);
	else
		pr_err("%s(): Backlight level set failed", __func__);

}

static struct platform_driver this_driver = {
	.probe  = mipi_renesas_lcd_probe,
	.driver = {
		.name   = "mipi_renesas",
	},
};

static struct msm_fb_panel_data renesas_panel_data = {
	.on		= mipi_renesas_lcd_on,
	.off	= mipi_renesas_lcd_off,
	.set_backlight = mipi_renesas_set_backlight,
};

static int ch_used[3];

int mipi_renesas_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_renesas_lcd_init();
	if (ret) {
		pr_err("mipi_renesas_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_renesas", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	renesas_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &renesas_panel_data,
		sizeof(renesas_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_renesas_lcd_init(void)
{
	mipi_dsi_buf_alloc(&renesas_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&renesas_rx_buf, DSI_BUF_SIZE);
	//ASUS_BSP +++ Jason Chang "[A80][Backlight] Add interface for backlight driver"
	mutex_init(&cmd_mutex);
	//ASUS_BSP --- Jason Chang "[A80][Backlight] Add interface for backlight driver"
	return platform_driver_register(&this_driver);
}
