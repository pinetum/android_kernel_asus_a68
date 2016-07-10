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
//SII8240_VER81
#define BUILD_NUMBER	81
#define BUILD_STRING	"Intermediate release for protocol/system tests"

#define BUILD_CONFIG 1  /* CP8240 Starter Kit  -- see si_app_gpio.h */
//#define BUILD_CONFIG 2  /* CP8558 FPGA  -- see si_app_gpio.h */
#define BISHOP_DEVICE_ID 0x8558
#define WOLV60_DEVICE_ID 0x8240

#define ONE_EDID_BLOCK_PER_REQUEST
//SII8240_VER81_1 +++
//#define MHL2_0_SUPPORT  //Keno20120810, update
//SII8240_VER81_1 ---
#define PACKED_PIXEL_SUPPORT
#define DEVCAP_CACHE
#define NEW_TIMER_API
#define NEW_SERIAL_CODE
#define eTMDS_SOURCE
#define	GET_DVI_MODE_FROM_SINK_EDID_NOT_REGISTER
#define	HW_ASSISTED_EDID_READ
#define THREE_D_SUPPORT
//SII8240_VER81_1 +++
#define Fix_SiiMhlTxPruneDTDList_Issue//Keno20120810, double confirmabtion and decision DTD is available.
#define Fix_SiiHdmiTxLiteReadEdid_Issue//Keno20120810, SiiHdmiTxLiteReadEdid() no exist.
//SII8240_VER81_1 ---
//#define APPLY_PLL_RECOVERY

//#define EXTRA_TMDS_TOGGLE_FOR_HDCP_START
#if BUILD_CONFIG >= 1 //( 8558
#define ASSERT_PUSH_PULL
#endif //)

// debug only -- fake out the 3D write burst data
//#define FAKE_3D_DATA

#if 1 //(

#define BUILD_VARIATION "Drive Upstream HPD - Transcode mode supported"

#else //)(

#define BUILD_VARIATION "Release Upstream HPD control"
#define TRANSCODE_HPD_CONTROL

#endif //)
//#define AVI_PASSTHROUGH

// define this only if SYSTEM_BOARD == SB_EPV5_MARK_II
//#define ENABLE_OCS_OVERRIDE

#define SII_DEBUG_CONFIG_RESOURCE_CONSTRAINED  /* ver75 */
//#define ENABLE_APP_DEBUG_PRINT
#define ENABLE_TX_DEBUG_PRINT	 /* ver75 */
#define ENABLE_PP_DEBUG_PRINT
#define ENABLE_EDID_DEBUG_PRINT  /* ver75 */
#define ENABLE_EDID_TRACE_PRINT
#define ENABLE_TX_EDID_PRINT
//#define ENABLE_PIXCLK_DEBUG_PRINT
#define ENABLE_TX_PRUNE_PRINT
#define ENABLE_DUMP_INFOFRAME
#define ENABLE_COLOR_SPACE_DEBUG_PRINT  /* ver75 */
#define ENABLE_CBUS_DEBUG_PRINT
#define ENABLE_SCRPAD_DEBUG_PRINT
#define ENABLE_INFO_DEBUG_PRINT
#define ENABLE_HDCP_DEBUG_PRINT
#define ENABLE_LITE_DEBUG_PRINT

#define EDID_DUMP_SW_BUFFER   //SII8240_VER81
//#define EDID_CLEAR_HW_BUFFER


//#define	EDID_CHANGE_NOTIFICATION_REQUIRED
//#define	US_HPD_CONTROL_FROM_CBUS_INTERRUPT
//#define	SACHIKO_HDCP_OFF
//#define	EDID_DUMP_8240_BUFFER  /* ver75 */ /* SII8240_VER81 , comment */
//#define	EDID_READ_FIFO_BYTE_MODE
//#define DUMP_INTERRUPT_STATUS  /* SII8240_VER81 , comment */
//#define	DEBUG_RI_VALUES
#define	HANDLE_GCP_TO_MUTE  /* ver75 */
//#define	LEIS_REQUEST_TO_MAKE_2E3_0
#define	CTS_1A_09_FIX  /* ver75 */
//#define	MDT_TESTER

//SII8240_VER75 +++
/************************Source Code Issue************************/
//#define Disable_SWWA_FLAKY_US_HPD  /* SII8240_VER81_1 , comment */
#define Modification_INT4_Clear_Place

/************************For Asus Issue************************/
//#define CI2CA_PullHigh          /* no use for asus build */
#define linux_interrput_handle

//MHL_CTS 20120907+++
//workaround for high resolution color issue +++
#define AVI_bypass_USE_QCOM_AVI
//#define AVI_bypass
//workaround for high resolution color issue ---
//MHL_CTS 20120907---
//#define mute_issue_when_HDCP_enable    //SII8240_VER81	+++ : remove from ver81 ??	
//SII8240_VER81	---
#define Disable_VBUS_Interrupt
//ASUS_BSP : ignore MHL link not stable , for some A68 in pad reboot issue
#define Disable_INTR5_MHL_FIFO_OVERFLOW
#define CP9687_CHECK_DEVCAP_1
#define HAMMER_HDCP_ISSUE
#define HAMMER_CP9687_NO_RESOLUTION_ISSUE

#define KENO_DONGLE_DOWNSTREAM1

//for P03
/*
#define UP_HPD_Pull_High
#define DONTCheckVideoReady  // SII8240_VER81_1 //hammer update for HDCP support
//SII8240_VER81_1 ---
#define Force_EDID_To_HDMI
#define DONT_Check_DevCap
#define Force_PP_Mode
*/

/*
#define bypass_Wake_pulse
#define open_MHL_TMDS
#define send_link_mode
#define remove_some_cbus_behavior
*/
//SII8240_VER75 ---



