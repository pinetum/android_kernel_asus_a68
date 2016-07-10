/*

SiI8240 Linux Driver

Copyright (C) 2011-2012 Silicon Image Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.

This program is distributed .as is. WITHOUT ANY WARRANTY of any
kind, whether express or implied; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the
GNU General Public License for more details.

*/


/*
 *****************************************************************************
 * @file  si_8240_regs.h
 *
 * @brief Implementation of the SiI 8240 internal registers
 *
 *****************************************************************************
*/




#define REG_DEV_IDL     (TX_PAGE_L0| 0x0002)
#define REG_DEV_IDH     (TX_PAGE_L0| 0x0003)
#define REG_DEV_REV     (TX_PAGE_L0| 0x0004)

#define REG_SYS_CTRL1   (TX_PAGE_L0| 0x0008)

#define REG_DCTL        (TX_PAGE_L0| 0x000D)
typedef enum{
     BIT_DCTL_TCLK3X_PHASE          = 0x01
    ,BIT_DCTL_TLCK_PHASE_MASK       = 0x02
    ,BIT_DCTL_TLCK_PHASE_NORMAL     = 0x00
    ,BIT_DCTL_TLCK_PHASE_INVERTED   = 0x02

    ,BIT_DCTL_TRANSCODE_MASK    = 0x08
    ,BIT_DCTL_TRANSCODE_OFF     = 0x00
    ,BIT_DCTL_TRANSCODE_ON      = 0x08

    ,BIT_DCTL_EXT_DDC_SEL       = 0x10
}DctlBits_e;

#define REG_PWD_SRST    (TX_PAGE_L0 | 0x000E)
typedef enum
{
     BIT_PWD_SRST_SW_RST_MASK    = 0x01
    ,BIT_PWD_SRST_SW_RST_NORMAL  = 0x00
    ,BIT_PWD_SRST_SW_RST_RESET   = 0x01
    ,BIT_PWD_MHLFIFO_RST_MASK    = 0x10
    ,BIT_PWD_MHLFIFO_RST_normal  = 0x00
    ,BIT_PWD_MHLFIFO_RST_reset   = 0x10
}BitsPwdSrst_e;

#define REG_HDCP_CTRL    (TX_PAGE_L0 | 0x000F)
typedef enum
{
      BIT_CP_RESETN_MASK           = 0x04	// bit 2
     ,BIT_CP_RESETN_RESET          = 0x00	// bit 2
     ,BIT_CP_RESETN_RELEASE        = 0x04	// bit 2
}BitsHdcpResetn_e;


#define TPI_HDCP_RI_LOW_REG					(TX_PAGE_L0 | 0x0022)
#define TPI_HDCP_RI_HIGH_REG				(TX_PAGE_L0 | 0x0023)

#define REG_RI_CMD                          (TX_PAGE_L0 | 0x0027)
typedef enum
{
     BIT_RI_CMD_ENABLE_RI_CHECK_MASK    = 0x01
    ,BIT_RI_CMD_ENABLE_RI_CHECK_DISABLE = 0x00
    ,BIT_RI_CMD_ENABLE_RI_CHECK_ENABLE  = 0x01
}BitsRiCmd_e;

#define REG_PEER_RI_RX_1                    (TX_PAGE_L0 | 0x0029)


#define REG_HDCP_DEBUG    (TX_PAGE_L0 | 0x002B)
typedef enum
{
      BIT_CP_DEBUG_MASK           = 0x80	// bit 7
     ,BIT_CP_DEBUG_RESET          = 0x00	// bit 7
     ,BIT_CP_DEBUG_RELEASE        = 0x80	// bit 7

}BitsHdcpDebug_e;

#define REG_HRESL           (TX_PAGE_L0 | 0x003A)
#define REG_HRESH           (TX_PAGE_L0 | 0x003B)
#define REG_VRESL           (TX_PAGE_L0 | 0x003C)
#define REG_VRESH           (TX_PAGE_L0 | 0x003D)

#define REG_VID_MODE    (TX_PAGE_L0 | 0x004A)
typedef enum
{
     BIT_VID_MODE_m1080p_MASK    = 0x40
    ,BIT_VID_MODE_m1080p_DISABLE    = 0x00
    ,BIT_VID_MODE_m1080p_ENABLE    = 0x40

}VidModeBits_e;

#define REG_VID_OVRRD   (TX_PAGE_L0 | 0x0051)
typedef enum
{
     BIT_VID_OVRRD_3DCONV_EN_MASK       = 0x10
    ,BIT_VID_OVRRD_3DCONV_EN_NORMAL     = 0x00
    ,BIT_VID_OVRRD_3DCONV_EN_FRAME_PACK = 0x10

    ,BIT_VID_OVRRD_M1080p_ovrrd_MASK    = 0x40
    ,BIT_VID_OVRRD_M1080p_ovrrd_DISABLE = 0x00
    ,BIT_VID_OVRRD_M1080p_ovrrd_ENABLE  = 0x40
}VidOvrRdBits_e;

#define REG_INTR_STATE  (TX_PAGE_L0| 0x0070)
#define REG_INTR1       (TX_PAGE_L0| 0x0071)
typedef enum{
     BIT_INTR1_RSEN_CHG        = 0x20
    ,BIT_INTR1_HPD_CHG         = 0x40
}Intr1Bits_e;
#define REG_INTR2       (TX_PAGE_L0| 0x0072)
#define     INTR2_PSTABLE 0x02

#define REG_INTR3       (TX_PAGE_L0| 0x0073)

#define REG_INTR5		(TX_PAGE_L0| 0x0074)
typedef enum{
	 BIT_INTR5_SCDT_CHANGE		= 0x01
	,BIT_INTR5_CKDT_CHANGE		= 0x02
	,BIT_INTR5_MHL_FIFO_OVERFLOW	= 0x04
	,BIT_INTR5_MHL_FIFO_UNDERFLOW= 0x08
	,BIT_INTR5_RPWR5V_CHG		= 0x10
	,BIT_INTR5_PQ_OVERFLOW		= 0x80
}enIntr5Bits_e;

#define REG_INTR1_MASK  (TX_PAGE_L0| 0x0075)
#define REG_INTR2_MASK  (TX_PAGE_L0| 0x0076)
#define REG_INTR3_MASK  (TX_PAGE_L0| 0x0077)
#define REG_INTR5_MASK	(TX_PAGE_L0| 0x0078)

#define REG_HPD_CTRL	(TX_PAGE_L0| 0x0079)
typedef enum{
     BIT_HPD_CTRL_HPD_OUT_OVR_EN_MASK = 0x10
    ,BIT_HPD_CTRL_HPD_OUT_OVR_EN_OFF  = 0x00
    ,BIT_HPD_CTRL_HPD_OUT_OVR_EN_ON   = 0x10

    ,BIT_HPD_CTRL_HPD_OUT_OVR_VAL_MASK = 0x20
    ,BIT_HPD_CTRL_HPD_OUT_OVR_VAL_OFF  = 0x00
    ,BIT_HPD_CTRL_HPD_OUT_OVR_VAL_ON   = 0x20

}HpdCtrlBits_e;

#define REG_INTR7   (TX_PAGE_L0 | 0x007B)
typedef enum
{
     BIT_INTR7_MUTE_ON      = 0x01
    ,BIT_INTR7_CP_NEW_CP    = 0x02
    ,BIT_INTR7_CP_SET_MUTE  = 0x04
    ,BIT_INTR7_CP_CLR_MUTE  = 0x08

    ,BIT_INTR7_CEA_NO_AVI   = 0x40
    ,BIT_INTR7_CEA_NO_VSI   = 0x80

}Intr7Bits_e;

#define REG_INTR8   (TX_PAGE_L0 | 0x007C)
typedef enum
{
     BIT_INTR8_CEA_DET_AIF      = 0x01
    ,BIT_INTR8_CEA_NEW_AVI      = 0x02
    ,BIT_INTR8_CEA_NEW_VSI      = 0x04
    ,BIT_INTR8_CEA_DET_ACP      = 0x08
    ,BIT_INTR8_CEA_DET_SPD      = 0x10
    ,BIT_INTR8_CEA_DET_IS_RC1   = 0x20
    ,BIT_INTR8_CEA_DET_IS_RC2   = 0x40

}Intr8Bits_e;


#define REG_INTR7_MASK   (TX_PAGE_L0 | 0x007D)
#define REG_INTR8_MASK   (TX_PAGE_L0 | 0x007E)

#define REG_TMDS_CCTRL  (TX_PAGE_L0| 0x0080)
typedef enum{
     BIT_TMDS_CCTRL_BGRCTL_MASK = 0x07
    ,BIT_TMDS_CCTRL_SEL_BGR     = 0x08
    ,BIT_TMDS_CCTRL_TMDS_OE     = 0x10
    ,BIT_TMDS_CCTRL_CKDT_EN     = 0x20
}BitsTmdsCCtrl_e;
#define REG_TMDS_CSTAT  (TX_PAGE_L0| 0x0081)
typedef enum{
	 BIT_TMDS_CSTAT_RPWR5V_STATUS	= 0x04
}BitsTmdsCStat_e;

#define REG_TMDS_CTRL4  (TX_PAGE_L0| 0x0085)

#define REG_TPI_SEL     (TX_PAGE_L0 | 0x00C7)
typedef enum{
     BIT_TPI_SEL_SW_TPI_EN_MASK = 0x80
    ,BIT_TPI_SEL_SW_TPI_EN_HW_TPI = 0x00
    ,BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI = 0x80
}RegTpiSelBits_e;


#define REG_EPCM        (TX_PAGE_L0| 0x00FA)
typedef enum
{
     BIT_EPCM_LD_KSV_MASK = 0x20
    ,BIT_EPCM_LD_KSV_DISABLE  = 0x00
    ,BIT_EPCM_LD_KSV_ENABLE  = 0x20
}BitsEpcm_e;


#define REG_INF_CTRL1        (TX_PAGE_L1 | 0x003E)

#define REG_TMDS0_CCTRL2    (TX_PAGE_2 | 0x0000)
typedef enum
{
     BIT_TMDS0_CCTRL2_DC0_CTL_MASK          = 0x0F
    ,BIT_TMDS0_CCTRL2_DC0_CTL_8BPP_1X_CLK               = 0x00
    ,BIT_TMDS0_CCTRL2_DC0_CTL_8BPP_2X_CLK               = 0x02
    ,BIT_TMDS0_CCTRL2_DC0_CTL_10BPP_1X_DEEP_COLOR_CLK   = 0x04
    ,BIT_TMDS0_CCTRL2_DC0_CTL_12BPP_1X_DEEP_COLOR_CLK   = 0x05
    ,BIT_TMDS0_CCTRL2_DC0_CTL_10BPP_2X_DEEP_COLOR_CLK   = 0x06
    ,BIT_TMDS0_CCTRL2_DC0_CTL_12BPP_2X_DEEP_COLOR_CLK   = 0x07
    ,BIT_TMDS0_CCTRL2_RI_ENABLE_MASK        = 0x10
    ,BIT_TMDS0_CCTRL2_OFFSET_COMP_EN_MASK   = 0x20
}BitsTmds0CCtrl2_e;


#define REG_DVI_CTRL3	(TX_PAGE_2 | 0x0005)
typedef enum
{
    BIT_DVI_CTRL3_RI_RST_WDT_MASK = 0xFF
}BitsDviCtrl3_e;

#define REG_TMDS_CLK_EN (TX_PAGE_2 | 0x0011)
#define REG_TMDS_CH_EN  (TX_PAGE_2 | 0x0012)
#define REG_TMDS_TERMCTRL1 (TX_PAGE_2 | 0x0013)

#define REG_PLL_CALREFSEL  (TX_PAGE_2 | 0x0017)
#define REG_PLL_VCOCAL  (TX_PAGE_2 | 0x001A)

#define REG_EQ_DATA0    (TX_PAGE_2 | 0x0022)
#define REG_EQ_DATA1    (TX_PAGE_2 | 0x0023)
#define REG_EQ_DATA2    (TX_PAGE_2 | 0x0024)
#define REG_EQ_DATA3    (TX_PAGE_2 | 0x0025)
#define REG_EQ_DATA4    (TX_PAGE_2 | 0x0026)
#define REG_EQ_DATA5    (TX_PAGE_2 | 0x0027)
#define REG_EQ_DATA6    (TX_PAGE_2 | 0x0028)
#define REG_EQ_DATA7    (TX_PAGE_2 | 0x0029)

#define REG_BW_I2C      (TX_PAGE_2 | 0x0031)

#define REG_EQ_PLL_CTRL1    (TX_PAGE_2 | 0x0045)

#define REG_MON_USE_COMP_EN (TX_PAGE_2 | 0x004B)
#define REG_ZONE_CTRL_SW_RST (TX_PAGE_2 | 0x004C)
//SII8240_VER75	+++	
typedef enum
{
    BIT_ZONE_CTRL_SW_RST_SZONE_I2C_MASK = 0x30
}ZoneCtrlSwRstBits_e;
//SII8240_VER75	---

#define REG_MODE_CONTROL    (TX_PAGE_2 | 0x004D)

#define REG_MHLTX_CTL1	(TX_PAGE_2 | 0x0080)		/* Was 0x00A0 */
typedef enum
{
     BIT_MHLTX_CTL1_VCO_FMAX_CTL_MASK   = 0x0F
    ,BIT_MHLTX_CTL1_DISC_OVRIDE_MASK    = 0x10
    ,BIT_MHLTX_CTL1_DISC_OVRIDE_OFF     = 0x00
    ,BIT_MHLTX_CTL1_DISC_OVRIDE_ON      = 0x10
    ,BIT_MHLTX_CTL1_TX_TERM_MODE_MASK   = 0xC0
    ,BIT_MHLTX_CTL1_TX_TERM_MODE_100DIFF= 0x00
    ,BIT_MHLTX_CTL1_TX_TERM_MODE_150DIFF= 0x40
    ,BIT_MHLTX_CTL1_TX_TERM_MODE_300DIFF= 0x80
    ,BIT_MHLTX_CTL1_TX_TERM_MODE_OFF    = 0xC0
}MhlTxCtl1Bits_e;

#define REG_MHLTX_CTL2	(TX_PAGE_2 | 0x0081)		/* Was 0x00A1 */
typedef enum
{
     BIT_MHL_TX_CTL2_TX_OE_MASK         = 0x3F
}MhlTxCtl2Bits_e;

#define REG_MHLTX_CTL3	(TX_PAGE_2 | 0x0082)		/* Was 0x00A2 */
typedef enum
{
     BIT_MHLTX_CTL3_DAMPING_SEL_MASK    = 0x30
    ,BIT_MHLTX_CTL3_DAMPING_SEL_75_OHM  = 0x00
    ,BIT_MHLTX_CTL3_DAMPING_SEL_100_OHM = 0x10
    ,BIT_MHLTX_CTL3_DAMPING_SEL_150_OHM = 0x20
    ,BIT_MHLTX_CTL3_DAMPING_SEL_OFF     = 0x30
}MhlTxCtl3Bits_e;

#define REG_MHLTX_CTL4	(TX_PAGE_2 | 0x0083)		/* Was 0x00A3 */
typedef enum
{
     BIT_DATA_SWING_CTL_MASK             = 0x07
    ,BIT_CLK_SWING_CTL_MASK              = 0x38
    ,BIT_MHLTX_CTL4_MHL_CLK_RATIO_MASK   = 0x40
    ,BIT_MHLTX_CTL4_MHL_CLK_RATIO_2X     = 0x00
    ,BIT_MHLTX_CTL4_MHL_CLK_RATIO_3X     = 0x40
}MhlTxCtl4Bits_e;

#define REG_MHLTX_CTL5	(TX_PAGE_2 | 0x0084)		/* Was 0x00A4 */
#define REG_MHLTX_CTL6	(TX_PAGE_2 | 0x0085)		/* Was 0x00A5 */
#define REG_MHLTX_CTL7	(TX_PAGE_2 | 0x0086)		/* Was 0x00A6 */
#define REG_MHLTX_CTL8	(TX_PAGE_2 | 0x0087)
typedef enum
{
     BIT_MHLTX_CTL8_PLL_BW_CTL_MASK  = 0x07
}BitsMhlTxCtl8_e;

#define REG_PKT_FILTER_0    (TX_PAGE_2 | 0x0090)
typedef enum
{
     BIT_PKT_FILTER_0_DROP_GCP_PKT          = 0x01
    ,BIT_PKT_FILTER_0_DROP_CTS_PKT          = 0x02
    ,BIT_PKT_FILTER_0_DROP_AVI_PKT          = 0x04
    ,BIT_PKT_FILTER_0_DROP_AIF_PKT          = 0x08
    ,BIT_PKT_FILTER_0_DROP_SPIF_PKT         = 0x10
    ,BIT_PKT_FILTER_0_DROP_MPEG_PKT         = 0x20
    ,BIT_PKT_FILTER_0_DROP_CEA_CP_PKT       = 0x40
    ,BIT_PKT_FILTER_0_DROP_CEA_GAMUT_PKT    = 0x80
}BitsPktFilter0_e;

#define REG_PKT_FILTER_1    (TX_PAGE_2 | 0x0091)

#define REG_TMDS_CSTAT_P3	(TX_PAGE_2 | 0x00A0)
typedef enum
{
     BIT_TMDS_CSTAT_P3_PDO_MASK              = 0x01
    ,BIT_TMDS_CSTAT_P3_PDO_CLOCK_DETECTED    = 0x00
    ,BIT_TMDS_CSTAT_P3_PDO_CLOCK_STOPPED     = 0x01

    ,BIT_TMDS_CSTAT_P3_SCDT                 = 0x02
}BitsTmdsCStateP3_e;
#define     TMDS_CSTAT_P3_SCDT 0x02

#define REG_RX_HDMI_CTRL0   (TX_PAGE_2 | 0x00A1)
typedef enum
{
     BIT_REG_RX_HDMI_CTRL0_rx_hdmi_hdmi_mode_MASK           = 0x01
    ,BIT_REG_RX_HDMI_CTRL0_rx_hdmi_hdmi_mode_en_MASK        = 0x02

    ,BIT_REG_RX_HDMI_CTRL0_hdmi_mode_overwrite_MASK         = 0x04
    ,BIT_REG_RX_HDMI_CTRL0_hdmi_mode_overwrite_HW_CTRL      = 0x00
    ,BIT_REG_RX_HDMI_CTRL0_hdmi_mode_overwrite_SW_CTRL      = 0x04

    ,BIT_REG_RX_HDMI_CTRL0_hdmi_mode_sw_value_MASK          = 0x08
    ,BIT_REG_RX_HDMI_CTRL0_hdmi_mode_sw_value_DVI           = 0x00
    ,BIT_REG_RX_HDMI_CTRL0_hdmi_mode_sw_value_HDMI          = 0x08

    ,BIT_REG_RX_HDMI_CTRL0_ri_hdmi_mode_en_itself_clr_MASK  = 0x10
    ,BIT_REG_RX_HDMI_CTRL0_byp_dvifilt_sync_MASK            = 0x20
}BitRxHdmiCtrl0_e;


#define REG_RX_HDMI_CTRL2   (TX_PAGE_2 | 0x00A3)
typedef enum
{
     BIT_RX_HDMI_CTRL2_VSI_MON_SEL_MASK             = 0x01
    ,BIT_RX_HDMI_CTRL2_VSI_MON_SEL_AVI_INFOFRAME    = 0x00
    ,BIT_RX_HDMI_CTRL2_VSI_MON_SEL_VS_INFOFRAME     = 0x01

    ,BIT_RX_HDMI_CTRL2_USE_AV_MUTE_SUPPORT_MASK = 0x08
    ,BIT_RX_HDMI_CTRL2_USE_AV_MUTE_SUPPORT_DISABLE = 0x00
    ,BIT_RX_HDMI_CTRL2_USE_AV_MUTE_SUPPORT_ENABLE = 0x08

    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_MASK       = 0xF0
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_0      = 0x00
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_1      = 0x10
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_2      = 0x20
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_3      = 0x30
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_4      = 0x40
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_5      = 0x50
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_6      = 0x60
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_7      = 0x70
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_8      = 0x80
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_9      = 0x90
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_A      = 0xA0
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_B      = 0xB0
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_C      = 0xC0
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_D      = 0xD0
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_E      = 0xE0
    ,BIT_RX_HDMI_CTRL2_IDLE_CNT_3_0_VAL_F      = 0xF0
}BitsRxHdmiCtrl2_e;

#define REG_RX_HDMI_CTRL3   (TX_PAGE_2 | 0x00A4)
#ifdef SII8240_VER68
#define REG_HDMI_CLR_BUFFER (TX_PAGE_2 | 0x00AC)
typedef enum
{
     BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_EN_MASK    = 0x01
    ,BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_EN_STALE   = 0x00
    ,BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_EN_CLEAR   = 0x01

    ,BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_W_AVI_EN_MASK    = 0x10
    ,BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_W_AVI_EN_STALE   = 0x00
    ,BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_W_AVI_EN_CLEAR   = 0x10

}BitsHdmiClrBuffer_e;
#endif
#define REG_RX_HDMI_MON_PKT_HEADER1 (TX_PAGE_2 | 0x00B8)


#define REG_INTR9	(TX_PAGE_2 | 0x00E0)
typedef enum
{
      BIT_INTR9_DEVCAP_DONE_MASK    = 0x10
     ,BIT_INTR9_DEVCAP_DONE         = 0x10
	 ,BIT_INTR9_EDID_DONE_MASK      = 0x20	// Bit 5
     ,BIT_INTR9_EDID_DONE           = 0x20	// Bit 5
     ,BIT_INTR9_EDID_ERROR          = 0x40


}BitsEdidHwAssistStatus_e;


#define REG_INTR9_MASK   (TX_PAGE_2  | 0x00E1)

#define REG_TPI_CBUS_START	(TX_PAGE_2 | 0x00E2)
typedef enum
{
       BIT_TPI_CBUS_START_DEVCAP_READ_START = 0x01
      ,BIT_TPI_CBUS_START_EDID_READ_BLOCK_0 = 0x02

}BitsEdidHwAssistPage_e;

//SII8240_VER75	+++	
#define REG_EDID_CTRL       (TX_PAGE_2 | 0x00E3)
typedef enum
{
     BIT_EDID_CTRL_EDID_MODE_EN_MASK    = 0x01
    ,BIT_EDID_CTRL_EDID_MODE_EN_DISABLE = 0x00
    ,BIT_EDID_CTRL_EDID_MODE_EN_ENABLE  = 0x01

    ,BIT_EDID_CTRL_INVALID_BKSV_MASK    = 0x02
    ,BIT_EDID_CTRL_INVALID_BKSV_NAK     = 0x00
    ,BIT_EDID_CTRL_INVALID_BKSV_ENABLE  = 0x02

    ,BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_MASK     = 0x10
    ,BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_DISABLE  = 0x00
    ,BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE   = 0x10

    ,BIT_EDID_CTRL_DEVCAP_SELECT_MASK   = 0x20
    ,BIT_EDID_CTRL_DEVCAP_SELECT_EDID   = 0x00
    ,BIT_EDID_CTRL_DEVCAP_SELECT_DEVCAP = 0x20

    ,BIT_EDID_CTRL_EDID_PRIME_VALID_MASK    = 0x80
    ,BIT_EDID_CTRL_EDID_PRIME_VALID_DISABLE = 0x00
    ,BIT_EDID_CTRL_EDID_PRIME_VALID_ENABLE  = 0x80
}BitsEdidCtrl_e;
//SII8240_VER75	---
#define REG_EDID_STAT       (TX_PAGE_2 | 0x00E4)
typedef enum
{
     BIT_EDID_STAT_DEVCAP_VALID_MASK    = 0x02
    ,BIT_EDID_STAT_DEVCAP_VALID_VALID   = 0x02
    ,BIT_EDID_STAT_DEVCAP_VALID_INVALID = 0x00
    ,BIT_EDID_STAT_TPI_CBUS_BUSY_MASK   = 0x10
    ,BIT_EDID_STAT_TPI_CBUS_BUSY_BUSY   = 0x10
    ,BIT_EDID_STAT_TPI_CBUS_BUSY_IDLE   = 0x10
}BitsEdidStat_e;

#define REG_CBUS_LINK_MODE       (TX_PAGE_2 | 0x00E5)

#define REG_EDID_FIFO_ADDR       (TX_PAGE_2 | 0x00E9)
#define REG_EDID_FIFO_WR_DATA    (TX_PAGE_2 | 0x00EA)
#define REG_EDID_FIFO_RD_DATA    (TX_PAGE_2 | 0x00EC)

#define REG_EDID_HW_ASSIST_READ_BLOCK_ADDR    (TX_PAGE_2 | 0x00ED)
typedef enum
{
      BIT_HW_ASSIST_EDID_READ_BLOCK1     = 0x01		// Bit 0 = block 1
     ,BIT_HW_ASSIST_EDID_READ_BLOCK2     = 0x02
     ,BIT_HW_ASSIST_EDID_READ_BLOCK3     = 0x04
}BitsEdidHwAssistBlockAddr_e;

#define REG_RI_LOCAL_OR_REMOTE       		(TX_PAGE_2  | 0x00FC)


#define REG_SRST		(TX_PAGE_3 | 0x0000)		/* Was 0x0005 */
#define REG_DPD         (TX_PAGE_3 | 0x0001)
typedef enum
{
     BIT_DPD_PDIDCK_MASK              = 0x04
    ,BIT_DPD_PDIDCK_POWER_DOWN        = 0x00
    ,BIT_DPD_PDIDCK_NORMAL_OPERATION  = 0x04

    ,BIT_DPD_PDNRX12_MASK               = 0x20
    ,BIT_DPD_PDNRX12_POWER_DOWN         = 0x00
    ,BIT_DPD_PDNRX12_NORMAL_OPERATION   = 0x20

    ,BIT_DPD_TX_CORE_MASK               = 0x40
    ,BIT_DPD_TX_CORE_POWER_DOWN         = 0x40
    ,BIT_DPD_TX_CORE_POWER_UP           = 0x40

    ,BITS_DPD_UPSTREAM_POWER_MASK   = BIT_DPD_PDIDCK_MASK | BIT_DPD_PDNRX12_MASK
    ,BITS_DPD_UPSTREAM_POWER_DOWN   = BIT_DPD_PDIDCK_POWER_DOWN | BIT_DPD_PDNRX12_POWER_DOWN
    ,BITS_DPD_UPSTREAM_POWER_UP     = BIT_DPD_PDIDCK_NORMAL_OPERATION | BIT_DPD_PDNRX12_NORMAL_OPERATION

}BitsDPD_e;

#define REG_DISC_CTRL1	(TX_PAGE_3 | 0x0010)		/* Was 0x0090 */
#define REG_DISC_CTRL2	(TX_PAGE_3 | 0x0011)		/* Was 0x0091 */
#define REG_DISC_CTRL3	(TX_PAGE_3 | 0x0012)		/* Was 0x0092 */
typedef enum{
     BIT_DC3_DLYTRG_SEL_MASK = 0x07
    ,BIT_DC3_DLYTRG_SEL_133us= 0x00
    ,BIT_DC3_DLYTRG_SEL_534us= 0x01
    ,BIT_DC3_DLYTRG_SEL_002ms= 0x02
    ,BIT_DC3_DLYTRG_SEL_008ms= 0x03
    ,BIT_DC3_DLYTRG_SEL_016ms= 0x04
    ,BIT_DC3_DLYTRG_SEL_032ms= 0x05
    ,BIT_DC3_DLYTRG_SEL_064ms= 0x06
    ,BIT_DC3_DLYTRG_SEL_128ms= 0x07

    ,BIT_DC3_USB_EN_MASK        = 0x08
    ,BIT_DC3_USB_EN_OFF         = 0x00
    ,BIT_DC3_USB_EN_ON          = 0x08

    ,BIT_DC3_FORCE_USB_MASK     = 0x10
    ,BIT_DC3_FORCE_USB_OFF      = 0x00
    ,BIT_DC3_FORCE_USB_ON       = 0x10

    ,BIT_DC3_DISC_SIMODE_MASK   = 0x20
    ,BIT_DC3_DISC_SIMODE_OFF    = 0x00
    ,BIT_DC3_DISC_SIMODE_ON     = 0x20

    ,BIT_DC3_FORCE_MHL_MASK     = 0x40
    ,BIT_DC3_FORCE_MHL_OFF      = 0x00
    ,BIT_DC3_FORCE_MHL_ON       = 0x40

    ,BIT_DC3_COMM_IMME_MASK     = 0x80
    ,BIT_DC3_COMM_IMME_OFF      = 0x00
    ,BIT_DC3_COMM_IMME_ON       = 0x80
}DiscCtrl3Bits_e;
#define REG_DISC_CTRL4	(TX_PAGE_3 | 0x0013)		/* Was 0x0093 */
#define REG_DISC_CTRL5	(TX_PAGE_3 | 0x0014)		/* Was 0x0094 */
#define REG_DISC_CTRL6	(TX_PAGE_3 | 0x0015)		/* Was 0x0095 */
typedef enum{
     BIT_DC6_USB_OVERRIDE_MASK  = 0x40
    ,BIT_DC6_USB_OVERRIDE_OFF   = 0x00
    ,BIT_DC6_USB_OVERRIDE_ON    = 0x40
}DiscCtrl6Bits_e;
#define REG_DISC_CTRL7	(TX_PAGE_3 | 0x0016)		/* Was 0x0096 */
#define REG_DISC_CTRL8	(TX_PAGE_3 | 0x0017)		/* Was 0x0097 */
#define REG_DISC_CTRL9	(TX_PAGE_3 | 0x0018)
typedef enum{
     BIT_DC9_VBUS_OUTPUT_CAPABILITY_SRC = 0x01
    ,BIT_DC9_WAKE_PULSE_BYPASS          = 0x02
    ,BIT_DC9_DISC_PULSE_PROCEED         = 0x04
    ,BIT_DC9_USB_EST                    = 0x08
    ,BIT_DC9_WAKE_DRVFLT                = 0x10
    ,BIT_DC9_CBUS_LOW_TO_DISCONNECT     = 0x20
    ,BIT_DC9_VBUS_EN_OVERRIDE           = 0x40
    ,BIT_DC9_VBUS_EN_OVERRIDE_VAL       = 0x80
}DiscCtrl9Bits_e;
#define REG_DISC_CTRL10	(TX_PAGE_3 | 0x0019)
#define REG_DISC_CTRL11	(TX_PAGE_3 | 0x001A)
#define REG_DISC_STAT	(TX_PAGE_3 | 0x001B)		/* Was 0x0098 */
#define REG_DISC_STAT2	(TX_PAGE_3 | 0x001C)		/* Was 0x0099 */

#define REG_INT_CTRL	(TX_PAGE_3 | 0x0020)		/* Was 0x0079 */
#define REG_INTR4		(TX_PAGE_3 | 0x0021)		/* Was 0x0074 */
#define BIT_INTR4_VBUS_CHG			0x01
#define BIT_INTR4_MHL_EST			0x04
#define BIT_INTR4_NON_MHL_EST		0x08
#define BIT_INTR4_CBUS_LKOUT			0x10
#define BIT_INTR4_CBUS_DISCONNECT	0x20
#define BIT_INTR4_RGND_DETECTION		0x40
#define BIT_INTR4_SOFT
#define REG_INTR4_MASK	(TX_PAGE_3 | 0x0022)		/* Was 0x0078 */

#define REG_USB_CHARGE_PUMP (TX_PAGE_3 | 0x0030)

#define REG_MHLTX_CTRL_AON  (TX_PAGE_3 | 0x0032)


// ===================================================== //
///////////////////////////////////////////////////////////////////////////////
//
// CBUS register definitions
//

#define REG_CBUS_DEVICE_CAP_0           (TX_PAGE_CBUS | 0x0000)      /* Used */   // New :   MHL_DEVCAP_*
#define REG_CBUS_DEVICE_CAP_1           (TX_PAGE_CBUS | 0x0001)
#define REG_CBUS_DEVICE_CAP_2           (TX_PAGE_CBUS | 0x0002)
#define REG_CBUS_DEVICE_CAP_3           (TX_PAGE_CBUS | 0x0003)
#define REG_CBUS_DEVICE_CAP_4           (TX_PAGE_CBUS | 0x0004)
#define REG_CBUS_DEVICE_CAP_5           (TX_PAGE_CBUS | 0x0005)
#define REG_CBUS_DEVICE_CAP_6           (TX_PAGE_CBUS | 0x0006)
#define REG_CBUS_DEVICE_CAP_7           (TX_PAGE_CBUS | 0x0007)
#define REG_CBUS_DEVICE_CAP_8           (TX_PAGE_CBUS | 0x0008)
#define REG_CBUS_DEVICE_CAP_9           (TX_PAGE_CBUS | 0x0009)
#define REG_CBUS_DEVICE_CAP_A           (TX_PAGE_CBUS | 0x000A)
#define REG_CBUS_DEVICE_CAP_B           (TX_PAGE_CBUS | 0x000B)
#define REG_CBUS_DEVICE_CAP_C           (TX_PAGE_CBUS | 0x000C)
#define REG_CBUS_DEVICE_CAP_D           (TX_PAGE_CBUS | 0x000D)
#define REG_CBUS_DEVICE_CAP_E           (TX_PAGE_CBUS | 0x000E)
#define REG_CBUS_DEVICE_CAP_F           (TX_PAGE_CBUS | 0x000F)
#define REG_CBUS_COMMON_CONFIG          (TX_PAGE_CBUS | 0x0007)


#define REG_CBUS_SET_INT_0				(TX_PAGE_CBUS | 0x0020)      /* Used */   // New :   MHL_INT_*
#define REG_CBUS_SET_INT_1				(TX_PAGE_CBUS | 0x0021)
#define REG_CBUS_SET_INT_2				(TX_PAGE_CBUS | 0x0022)
#define REG_CBUS_SET_INT_3				(TX_PAGE_CBUS | 0x0023)

#define REG_CBUS_WRITE_STAT_0        	(TX_PAGE_CBUS | 0x0030)      /* Used */   // New :   MHL_STAT_*
#define REG_CBUS_WRITE_STAT_1        	(TX_PAGE_CBUS | 0x0031)
#define REG_CBUS_WRITE_STAT_2        	(TX_PAGE_CBUS | 0x0032)
#define REG_CBUS_WRITE_STAT_3        	(TX_PAGE_CBUS | 0x0033)


#define REG_CBUS_MHL_SCRPAD_0               (TX_PAGE_CBUS | 0x0040)      /* Used */   // New :   WB_XMIT_DATA_* (0x40), MHL_SCRPAD_* ( 0x60 )
#define REG_CBUS_WB_XMIT_DATA_0             (TX_PAGE_CBUS | 0x0060)      /* Used */   // New :   WB_XMIT_DATA_* (0x40), MHL_SCRPAD_* ( 0x60 )

#define REG_CBUS_SET_INT_ENABLE_0    (TX_PAGE_CBUS | 0x0080)         /* Used */   // New :   MHL_INT_*_MASK
#define REG_CBUS_SET_INT_ENABLE_1    (TX_PAGE_CBUS | 0x0081)
#define REG_CBUS_SET_INT_ENABLE_2    (TX_PAGE_CBUS | 0x0082)
#define REG_CBUS_SET_INT_ENABLE_3    (TX_PAGE_CBUS | 0x0083)


#define REG_CBUS_STATUS                     (TX_PAGE_CBUS | 0x0091)
typedef enum{
     BIT_CBUS_CONNECTED     = 0x0001
    ,BIT_MHL_MODE           = 0x0002
    ,BIT_CBUS_HPD           = 0x0004
    ,BIT_MSC_HB_SUCCESS     = 0x0008
    ,BIT_MHL_CABLE_PRESENT  = 0x0010
}CBusStatusBits_e;

#define REG_CBUS_INT_0                  (TX_PAGE_CBUS | 0x0092)     /* Used */    // New :   CBUS_INT_0
typedef enum{
	 BIT_CBUS_CNX_CHG				= BIT0
	,BIT_CBUS_MSC_MT_DONE			= BIT1
	,BIT_CBUS_HPD_RCVD				= BIT2
	,BIT_CBUS_MSC_MR_WRITE_STAT		= BIT3                        /* Used */  // New :   CBUS_INT_0
	,BIT_CBUS_MSC_MR_MSC_MSG        = BIT4    /* Responder sent a VS_MSG packet (response data or command.) */
	,BIT_CBUS_MSC_MR_WRITE_BURST	= BIT5                        /* Used */  // New :   CBUS_INT_0
	,BIT_CBUS_MSC_MR_SET_INT		= BIT6
	,BIT_CBUS_MSC_MT_DONE_NACK		= BIT7
}CBusInt0Bits_e;

#define REG_CBUS_INT_0_MASK				(TX_PAGE_CBUS | 0x0093)

#define REG_CBUS_INT_1					(TX_PAGE_CBUS | 0x0094)
typedef enum{
	 BIT_CBUS_CEC_ABRT              = 0x02    /* Responder aborted DDC command at translation layer */
	,BIT_CBUS_DDC_ABRT              = 0x04    /* Responder sent a VS_MSG packet (response data or command.) */
    ,BIT_CBUS_MSC_ABORT_RCVD        = 0x08
    ,BIT_CBUS_MSC_SET_CAP_ID_RCVD   = 0x10
    ,BIT_CBUS_RCV_VALID             = 0x20
    ,BIT_CBUS_CMD_ABORT             = 0x40
    ,BIT_CBUS_MHL_CABLE_CNX_CHG     = 0x80
}CBusInt1Bits_e;
#define REG_CBUS_INT_1_MASK				(TX_PAGE_CBUS | 0x0095)


#define REG_CBUS_DDC_ABORT_INT          (TX_PAGE_CBUS | 0x0098)
typedef enum{
     BIT_CBUS_DDC_MAX_FAIL      = 0x01
    ,BIT_CBUS_DDC_PROTO_ERR     = 0x02
    ,BIT_CBUS_DDC_TIMEOUT       = 0x04

    ,BIT_CBUS_DDC_PEER_ABORT    = 0x80
}CBusDDCAbortIntBits_e;


#define REG_CBUS_MSC_MT_ABORT_INT       (TX_PAGE_CBUS | 0x009A)
typedef enum{
     BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL             = 0x01
    ,BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR            = 0x02
    ,BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT              = 0x04
    ,BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD            = 0x08

    ,BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT    = 0x80

}MscMtAbortIntBits_e;

//SII8240_VER75	+++	
#define REG_CBUS_LINK_XMIT_BIT_TIME (TX_PAGE_CBUS | 0x00A7)
//SII8240_VER75	---
#define REG_CBUS_MSC_COMMAND_START          (TX_PAGE_CBUS | 0x00B8)
typedef enum{
     BIT_CBUS_MSC_PEER_CMD              = 0x01
    ,BIT_CBUS_MSC_MSG                   = 0x02
    ,BIT_CBUS_MSC_READ_DEVCAP           = 0x04
    ,BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT = 0x08
    ,BIT_CBUS_MSC_WRITE_BURST           = 0x10
}CBusMscCommandStartBits_e;

#define REG_CBUS_MSC_CMD_OR_OFFSET      (TX_PAGE_CBUS | 0x00B9)
#define REG_CBUS_MSC_1ST_TRANSMIT_DATA  (TX_PAGE_CBUS | 0x00BA)
#define REG_CBUS_MSC_2ND_TRANSMIT_DATA  (TX_PAGE_CBUS | 0x00BB)
#define REG_CBUS_PRI_RD_DATA_1ST        (TX_PAGE_CBUS | 0x00BC)      /* Used */    // New :  MSC_MT_RCVD_DATA0
#define REG_CBUS_PRI_RD_DATA_2ND        (TX_PAGE_CBUS | 0x00BD)

#define REG_CBUS_MSC_MR_MSC_MSG_RCVD_1ST_DATA   (TX_PAGE_CBUS | 0x00BF)
#define REG_CBUS_MSC_MR_MSC_MSG_RCVD_2ND_DATA   (TX_PAGE_CBUS | 0x00C0)


#define	REG_CBUS_MSC_WRITE_BURST_DATA_LEN   (TX_PAGE_CBUS | 0x00C6)      /* Used */   /* only for WRITE_BURST */   // New :   MSC_WRITE_BURST_DATA_LEN
#define     MSC_WRITE_BURST_LEN_MASK    0x0F

#define REG_CBUS_DDC_TIMEOUT			(TX_PAGE_CBUS | 0x00D1)
#define REG_CBUS_MISC_CONTROL			(TX_PAGE_CBUS | 0x00D8)

#if 0 //(

#define BIT_CBUS_MSC_MR_MSC_MSG BIT3    /* Responder sent ACK packet (not VS_MSG) */
#define BIT_CBUS_MSC_MT_DONE    BIT4    /* Command send aborted on TX side */
#define BIT_CBUS_MSC_MT_ABRT    BIT5    /* Responder aborted MSC command at translation layer */
#define BIT_CBUS_MSC_MR_ABRT    BIT6
#define BIT_CBUS_LINK_SOFT_ERR  BIT7

#define BIT_DDC_ABORT                   BIT_CBUS_DDC_ABRT            /* Used */   // New :   CBUS_INT_1
#define BIT_MSC_MSG_RCV                 BIT_CBUS_MSC_MR_MSC_MSG
#define BIT_MSC_XFR_DONE                BIT_CBUS_MSC_MT_DONE
#define BIT_MSC_XFR_ABORT               BIT_CBUS_MSC_MT_ABRT         /* Used */   // New :   CBUS_INT_1
#define BIT_MSC_ABORT                   BIT_CBUS_MSC_MR_ABRT         /* Used */

#define REG_CBUS_INTR_ENABLE            (TX_PAGE_CBUS | 0x0009)      /* Used */
#define BIT_CBUS_INTRP_CNX_CHG_EN        BIT0
#define BIT_CBUS_INTRP_CEC_ABRT_EN       BIT1
#define BIT_CBUS_INTRP_DDC_ABRT_EN       BIT2                        /* Used */
#define BIT_CBUS_INTRP_MSC_MR_MSC_MSG_EN BIT3                        /* Used */
#define BIT_CBUS_INTRP_MSC_MT_DONE_EN    BIT4                        /* Used */
#define BIT_CBUS_INTRP_MSC_MT_ABRT_EN    BIT5                        /* Used */
#define BIT_CBUS_INTRP_MSC_MR_ABRT_EN    BIT6                        /* Used */
#define BIT_CBUS_INTRP_LINK_SOFT_ERR_EN  BIT7

#define REG_DDC_ABORT_REASON        	(TX_PAGE_CBUS | 0x000B)      /* Used */   // New :	DDC_ABORT_INT
#define REG_PRI_XFR_ABORT_REASON        (TX_PAGE_CBUS | 0x000D)      /* Used */   // New :	MSC_MT_ABORT_INT

#define REG_CBUS_PRI_FWR_ABORT_REASON   (TX_PAGE_CBUS | 0x000E)      /* Used */
#define	BIT_CBUS_MSC_MT_MAX_FAIL                    BIT0
#define	BIT_CBUS_MSC_MT_PRORO_ERR                   BIT1
#define	BIT_CBUS_MSC_MT_TIMEOUT                     BIT2
#define	BIT_CBUS_MSC_UNDEF_CMD                      BIT3

#define BIT_CBUS_MSC_MT_CMD_LAT_FLUSHED_BY_HB_FAIL  BIT5
#define	BIT_CBUS_HPD                                BIT6
#define	BIT_CBUS_MSC_MT_ABRT_BY_PEER                BIT7

#define	CBUSABORT_BIT_REQ_MAXFAIL		BIT_CBUS_MSC_MT_MAX_FAIL     /* Used */
#define	CBUSABORT_BIT_PROTOCOL_ERROR	BIT_CBUS_MSC_MT_PRORO_ERR    /* Used */
#define	CBUSABORT_BIT_REQ_TIMEOUT	    BIT_CBUS_MSC_MT_TIMEOUT      /* Used */
#define	CBUSABORT_BIT_UNDEFINED_OPCODE  BIT_CBUS_MSC_UNDEF_CMD       /* Used */


#define	CBUSSTATUS_BIT_CONNECTED		BIT_CBUS_HPD
#define	CBUSABORT_BIT_PEER_ABORTED		BIT_CBUS_MSC_MT_ABRT_BY_PEER /* Used */



#define	REG_CBUS_MSC_RETRY_INTERVAL		(TX_PAGE_CBUS | 0x001A)		 /* default is 16 */
#define	REG_CBUS_DDC_FAIL_LIMIT			(TX_PAGE_CBUS | 0x001C)		 /* default is 5  */
#define	REG_CBUS_MSC_FAIL_LIMIT			(TX_PAGE_CBUS | 0x001D)		 /* default is 5  */
#define	REG_CBUS_MSC_INT2_STATUS        (TX_PAGE_CBUS | 0x001E)      /* Used */     // New :   CBUS_INT_0

#ifdef OLD_CBUS_MAPPING //{
#define BIT_CBUS_MSC_MR_WRITE_BURST     BIT0                         /* Used */  // New :   CBUS_INT_0
#endif //}

#define BIT_CBUS_MSC_HB_MAX_FAIL        BIT1
#define BIT_CBUS_MSC_MR_SET_INT         BIT2                         /* Used */   // New :   CBUS_INT_0
#define BIT_CBUS_MSC_MR_WRITE_STATE     BIT3                         /* Used */   // New :   CBUS_INT_0

#define BIT_CBUS_MSC_PEER_STATE_CHG     BIT4



#define REG_CBUS_MSC_INT2_ENABLE        (TX_PAGE_CBUS | 0x001F)      /* Used */
#define BIT_CBUS_INTRP_MSC_MR_WRITE_BURST_EN     BIT0
#define BIT_CBUS_INTRP_MSC_HB_MAX_FAIL_EN        BIT1
#define BIT_CBUS_INTRP_MSC_MR_SET_INT_EN         BIT2                /* Used */
#define BIT_CBUS_INTRP_MSC_MR_WRITE_STATE_EN     BIT3                /* Used */   // New :   CBUS_INT_0_MASK
#define BIT_CBUS_INTRP_MSC_PEER_STATE_CHG_EN     BIT4


#define	REG_MSC_WRITE_BURST_LEN         (TX_PAGE_CBUS | 0x0020)      /* Used */   /* only for WRITE_BURST */   // New :   MSC_WRITE_BURST_DATA_LEN
#define     MSC_MT_DONE_NACK_MASK       BIT6                         /* Used */   // New :   CBUS_INT_0
#define     MSC_WRITE_BURST_LEN_MASK    0x0F

#define REG_MSC_TIMEOUT_LIMIT           (TX_PAGE_CBUS | 0x0022)      /* Used */   // New :   MSC_TIMEOUT
#define	MSC_TIMEOUT_LIMIT_MSB_MASK	        (0x0F)	                               /* default is 1           */
#define	MSC_LEGACY_BIT					    (0x01 << 7)	    /* This should be cleared.*/

#define REG_CBUS_MSC_COMPATIBILITY_CTRL (TX_PAGE_CBUS | 0x002E)      /* Used */   // New :   CBUS_MSC_COMPATIBILITY_CONTROL
#define BIT_CBUS_CEC_DISABLE            BIT4                         /* Used */   // New :   CBUS_MSC_COMPATIBILITY_CONTROL

#define	REG_CBUS_LINK_CONTROL_1			(TX_PAGE_CBUS | 0x00A0)      /* Used */   // New :   CBUS_LINK_CONTROL_1
#define BIT_CBUS_PACKET_LIMIT_MASK      0x3F

#define	REG_CBUS_LINK_CONTROL_2			(TX_PAGE_CBUS | 0x00A1)      /* Used */   // New :   CBUS_LINK_CONTROL_2
#define BIT_CBUS_INITIATOR_TIMEOUT_MASK 0x0F                         /* Used */
#define BIT_CBUS_LNK_GLITCH_FILTER_SEL  0xF0                         /* Used */

#define	REG_CBUS_LINK_CONTROL_3			(TX_PAGE_CBUS | 0x00A2)
typedef enum{
     BIT_LNK_RCV_BIT_CHECK_OPT_MASK = 0x07
    ,BIT_LNK_FWR_ACK_OFFSET_MASK    = 0x38
    ,BIT_LNK_ARB_ID                 = 0x40
    ,BIT_LNK_SYNC_FILTER_EN         = 0x80
}CBusLinkCtrl3Bits_e;

#define	REG_CBUS_LINK_CONTROL_4			(TX_PAGE_CBUS | 0x00A3)
typedef enum{
    BIT_LNK_XMIT_OPP_TIMEOUT_MAX_MASK       = 0x07
    BIT_REG_LNK_RCV_OPP_TIMEOUT_MAX_MASK    = 0xF8
}CBusLinkCtrl4Bits_e;

#define	REG_CBUS_LINK_CONTROL_5			(TX_PAGE_CBUS | 0x00A4)
typedef enum{
     BIT_LNK_HIGH_TIMEOUT_MAX_MSB_MASK  = 0x0F
    ,BIT_LNK_LOW_TIMEOUT_MAX_MSB_MASK   = 0xF0
}CBusLinkCtrl5Bits_e;

#define	REG_CBUS_LINK_CONTROL_6			(TX_PAGE_CBUS | 0x00A5)
typedef enum{
     BIT_LNK_DRV_HIGH_LIMIT_MASK    = 0x07
    ,BIT_LNK_CHECK_HIGH_LIMIT_MASK  = 0x38
}CBusLinkCtrl6Bits_e;

#define	REG_CBUS_LINK_CONTROL_7			(TX_PAGE_CBUS | 0x00A6)      /* Used */   // New :   CBUS_LINK_CONTROL_6
typedef enum{
     BIT_LNK_XMIT_IDLE_TIMEOUT_MASK = 0x1F
    ,BIT_LNK_XMIT_BIT_TIME_SEL0     = 0x20
}CBusLinkCtrl7Bits_e;

#define REG_CBUS_LINK_CONTROL_8         (TX_PAGE_CBUS | 0x00A7)
typedef enum{
     BIT_LNK_XMIT_BIT_TIME_MASK     = 0xFF
}CBusLinkCtrl8Bits_e;

#define REG_CBUS_LINK_CONTROL_9         (TX_PAGE_CBUS | 0x00A8)
typedef enum{
     BIT_LNK_CAL_BYTE_CNT_MIN_MASK  = 0x07
    ,BIT_LNK_CAL_FIFO_DEPTH_MASK    = 0x38
    ,BIT_LNK_INV_POLARITY           = 0x40
    ,BIT_LNK_XMIT_BIT_TIME_SEL1     = 0x80
}CBusLinkCtrl9Bits_e;

#define REG_CBUS_LINK_CONTROL_10		(TX_PAGE_CBUS | 0x00A9)      /* Used */   // New :   CBUS_LINK_CONTROL_7
typedef enum{
     BIT_LNK_CAL_SYNC_OFFSET_MASK   = 0x07
    ,BIT_LNK_CAL_BIT_OFFSET_MASK    = 0x38
    ,BIT_LNK_RCV_BIT_TIME_SEL       = 0x40
    ,BIT_LNK_CAL_ERROR_CHECK        = 0x80
}CBusLinkCtrl10Bits_e;

#define REG_CBUS_LINK_CONTROL_11		(TX_PAGE_CBUS | 0x00AA)
typedef enum{
     BIT_LNK_CAL_BDY_CTL            = 0x01
    ,BIT_LNK_NORM_BDY_CTL           = 0x02
    ,BIT_LNK_XMIT_RETRY_LIMIT_MASK  = 0xFC
}CBusLinkCtrl10Bits_e;


//(
        (TX_PAGE_CBUS | 0x00AB)
		(TX_PAGE_CBUS | 0x00AC)
		(TX_PAGE_CBUS | 0x00AD)
//)

#define REG_CBUS_LINK_STATUS_1
#define REG_CBUS_LINK_STATUS_2


#define REG_CBUS_LINK_CTRL9_0			(TX_PAGE_CBUS | 0x003A)
#define REG_CBUS_LINK_CTRL9_1           (TX_PAGE_CBUS | 0x00BA)

#define	REG_CBUS_DRV_STRENGTH_0			(TX_PAGE_CBUS | 0x0040)      /* Used */
#define	REG_CBUS_DRV_STRENGTH_1			(TX_PAGE_CBUS | 0x0041)
#define	REG_CBUS_ACK_CONTROL			(TX_PAGE_CBUS | 0x0042)
#define	REG_CBUS_CAL_CONTROL			(TX_PAGE_CBUS | 0x0043)	/* Calibration */

//DNE
//DNE
#endif //)

