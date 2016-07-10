/*
 *	si_drv_mhl_tx.c <Firmware or Driver>
 *
 *

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
 *   !file     si_drv_mhl_tx.c
 *   !brief    Silicon Image implementation of MHL driver.
 *
 */
#include "si_common.h"
#ifndef	__KERNEL__ //(
#include "hal_timers.h"
#endif //)

#include "si_cra.h"
#include "si_cra_cfg.h"
#include "si_bitdefs.h"
#include "si_mhl_defs.h"
#include "si_mhl_tx_api.h"
#include "si_mhl_tx_base_drv_api.h"  // generic driver interface to MHL tx component
#include "si_hdmi_tx_lite_api.h"
#include "si_hdmi_tx_lite_private.h"
#include "si_8240_regs.h"
#include "si_tpi_regs.h"
#include "si_drv_mhl_tx.h"
#include "si_osscheduler.h"
#include "si_edid.h"
#include "si_drvisrconfig.h"

#include "sii_hal_priv.h"

#include <linux/switch.h>
//MHL_CTS 20120907+++
#define SILICON_IMAGE_ADOPTER_ID 1324
//MHL_CTS 20120907---
#define TRANSCODER_DEVICE_ID TARGET_DEVICE_ID


//
// Software power states are a little bit different than the hardware states but
// a close resemblance exists.
//
// D3 matches well with hardware state. In this state we receive RGND interrupts
// to initiate wake up pulse and device discovery
//
// Chip wakes up in D2 mode and interrupts MCU for RGND. Firmware changes the TX
// into D0 mode and sets its own operation mode as POWER_STATE_D0_NO_MHL because
// MHL connection has not yet completed.
//
// For all practical reasons, firmware knows only two states of hardware - D0 and D3.
//
// We move from POWER_STATE_D0_NO_MHL to POWER_STATE_D0_MHL only when MHL connection
// is established.
/*
//
//                             S T A T E     T R A N S I T I O N S
//
//
//                    POWER_STATE_D3                      POWER_STATE_D0_NO_MHL
//                   /--------------\                        /------------\
//                  /                \                      /     D0       \
//                 /                  \                \   /                \
//                /   DDDDDD  333333   \     RGND       \ /   NN  N  OOO     \
//                |   D     D     33   |-----------------|    N N N O   O     |
//                |   D     D  3333    |      IRQ       /|    N  NN  OOO      |
//                \   D     D      33  /               /  \                  /
//                 \  DDDDDD  333333  /                    \   CONNECTION   /
//                  \                /\                     /\             /
//                   \--------------/  \  TIMEOUT/         /  -------------
//                         /|\          \-------/---------/        ||
//                        / | \            500ms\                  ||
//                          |                     \                ||
//                          |  RSEN_LOW                            || MHL_EST
//                           \ (STATUS)                            ||  (IRQ)
//                            \                                    ||
//                             \      /------------\              //
//                              \    /              \            //
//                               \  /                \          //
//                                \/                  \ /      //
//                                 |    CONNECTED     |/======//
//                                 |                  |\======/
//                                 \   (OPERATIONAL)  / \
//                                  \                /
//                                   \              /
//                                    \-----------/
//                                   POWER_STATE_D0_MHL
//
//
//
*/
#define	POWER_STATE_D3				3
#define	POWER_STATE_D0_NO_MHL		2
#define	POWER_STATE_D0_MHL			0
#define	POWER_STATE_FIRST_INIT		0xFF

//#define TX_HW_RESET_PERIOD		10	// 10 ms.
//#define TX_HW_RESET_DELAY			100

#define TX_HW_RESET_PERIOD		10	// 10 ms.
#define TX_HW_RESET_DELAY			10


typedef struct _StateFlags_t
{
    uint8_t     upStreamMuted  : 1;
    uint8_t     reserved       : 7;
}StateFlags_t,*PStateFlags_t;


int g_pad_tv_mode = MHL_NONE;
//ASUS_BSP +++ : for lock ISR in disconnecting progress
int g_TxDisconnect_lock = 0;
//ASUS_BSP --- : for lock ISR in disconnecting progress
bool_t MHL_Resume = false;

//
// To remember the current power state.
//
uint8_t	fwPowerState = POWER_STATE_FIRST_INIT;
//SII8240_VER81 +++
#define UpdatePowerState(x) \
{ \
    fwPowerState = x; \
    TX_DEBUG_PRINT(("fwPowerState = %d\n",(uint16_t)x)); \
}
//SII8240_VER81 ---
//
// To serialize the RCP commands posted to the CBUS engine, this flag
// is maintained by the function SiiMhlTxDrvSendCbusCommand()
//
static bool_t	mscCmdInProgress;	// false when it is okay to send a new command

#ifdef DEFERRED_HDCP_START //(
static bool_t   hdcpStartPending=false;
#endif

static uint8_t  cache_TPI_SYSTEM_CONTROL_DATA_REG=0;
#define WriteTpiSystemControlDataReg(value) \
{   \
    SiiRegWrite(TPI_SYSTEM_CONTROL_DATA_REG,cache_TPI_SYSTEM_CONTROL_DATA_REG=value); \
    TX_DEBUG_PRINT(("TPI_SYSTEM_CONTROL_DATA_REG: %02x\n",(uint16_t) cache_TPI_SYSTEM_CONTROL_DATA_REG)); \
}

#define ReadTpiSystemControlDataReg(dummy) (cache_TPI_SYSTEM_CONTROL_DATA_REG= SiiRegRead(TPI_SYSTEM_CONTROL_DATA_REG))
#define ModifyTpiSystemControlDataReg(mask,value) \
{ \
uint8_t holder = cache_TPI_SYSTEM_CONTROL_DATA_REG; \
    cache_TPI_SYSTEM_CONTROL_DATA_REG &= ~(mask); \
    cache_TPI_SYSTEM_CONTROL_DATA_REG |= ((mask) & value); \
    if (holder != cache_TPI_SYSTEM_CONTROL_DATA_REG) \
    { \
        SiiRegWrite(TPI_SYSTEM_CONTROL_DATA_REG,cache_TPI_SYSTEM_CONTROL_DATA_REG); \
        TX_DEBUG_PRINT(("TPI_SYSTEM_CONTROL_DATA_REG: %02x\n",(uint16_t) cache_TPI_SYSTEM_CONTROL_DATA_REG)); \
    } \
}
//
// Preserve Downstream HPD status
//
static	uint8_t	dsHpdStatus = 0;
static  uint8_t upStreamHPD = 0;
//ASUS_BSP+++ larry lai : Non-transcode mode
#ifdef SII8240_NO_TRANSCODE_MODE
static  uint8_t g_modeFlags = 0;
#else
static  uint8_t g_modeFlags = mfdrvTranscodeMode;
#endif
//ASUS_BSP--- larry lai : Non-transcode mode
static  StateFlags_t stateFlags ={0,0};

static uint8_t hwInputColorSpace  = BIT_TPI_INPUT_FORMAT_RGB;
static uint8_t hwOutputColorSpace = BIT_TPI_OUTPUT_FORMAT_HDMI_TO_RGB;
static QuantizationSettings_e hwInputQuantizationSettings = qsAutoSelectByColorSpace;
static QuantizationSettings_e hwOutputQuantizationSettings = qsAutoSelectByColorSpace;
static AviInfoFrameDataByte2_t colorimetryAspectRatio = {0x8,0x01,0x00 };//0x18;
static AviInfoFrameDataByte4_t inputVideoCode = {2,0}; //2
uint8_t packedPixelStatus=0;
//SII8240_VER75 +++
static uint32_t pixelClockFrequency=0;
//SII8240_VER75 ---
int PackedPixelAvailable =0;

unsigned int g_b_isCarkitConnected=0;
struct switch_dev switch_carkit_cable;
//ASUS_BSP , for A68 otg feature, check carkit cable in/out in MHL and msm otg driver +++
uint8_t g_b_SwitchCarkitInitial=0;
uint8_t g_b_SwitchCarkitBootPlugin=0;
//ASUS_BSP , for A68 otg feature, check carkit cable in/out in MHL and msm otg driver ---

#ifdef CONFIG_HAS_EARLYSUSPEND
extern int g_mhl_early_suspend_flag;
#endif
extern unsigned char g_mhl_i2c_error_not_connected_count;

extern int get_pad_tv_mode(void);
extern int get_p03_plug_in(void);
extern void sii8240drv_wakelock(void);
extern void sii8240drv_wakeunlock(void);
extern void MHL_disable_irq(void);


// for MHL TX debug
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_Sii8240_Reg_dump(struct seq_file *s, void *unused)
{
	int i;
	SiiResultCodes_t i2c_ret = 0;
	
	seq_printf(s, "\n============== TX_PAGE_TPI Reg dump =============\n");
	for (i=0; i<0x100; i++)
	{
		if (!(i%16))
			seq_printf(s, "\n");
		
		i2c_ret = 0;
		seq_printf(s, "[%02x]=0x%02x, ", i, SiiRegRead_check(TX_PAGE_TPI | (i),  &i2c_ret) );
		if (i2c_ret !=SII_SUCCESS)
			return -1;
	}
	seq_printf(s, "\n============== TX_PAGE_L0 Reg dump =============\n");
	for (i=0; i<0x100; i++)
	{
		if (!(i%16))
			seq_printf(s,"\n");
		
		i2c_ret = 0;
		seq_printf(s,"[%02x]=0x%02x, ", i, SiiRegRead_check(TX_PAGE_L0 | (i) ,  &i2c_ret) );
		if (i2c_ret !=SII_SUCCESS)
			return -1;		
	}
	seq_printf(s, "\n============== TX_PAGE_L1 Reg dump =============\n");
	for (i=0; i<0x100; i++)
	{
		if (!(i%16))
			seq_printf(s,"\n");

		i2c_ret = 0;
		seq_printf(s, "[%02x]=0x%02x, ", i, SiiRegRead_check(TX_PAGE_L1 | (i) ,  &i2c_ret) );
		if (i2c_ret !=SII_SUCCESS)
			return -1;		
	}
	seq_printf(s, "\n============== TX_PAGE_2 Reg dump =============\n");
	for (i=0; i<0x100; i++)
	{
		if (!(i%16))
			seq_printf(s, "\n");

	i2c_ret = 0;
	seq_printf(s, "[%02x]=0x%02x, ", i, SiiRegRead_check(TX_PAGE_2 | (i) ,  &i2c_ret) );
		if (i2c_ret !=SII_SUCCESS)
			return -1;	
	}
	seq_printf(s, "\n============== TX_PAGE_3 Reg dump =============\n");
	for (i=0; i<0x100; i++)
	{
		if (!(i%16))
			seq_printf(s, "\n");
		
		i2c_ret = 0;
		seq_printf(s, "[%02x]=0x%02x, ", i, SiiRegRead_check(TX_PAGE_3 | (i) ,  &i2c_ret) );
		if (i2c_ret !=SII_SUCCESS)
			return -1;		
	}
	seq_printf(s, "\n============== TX_PAGE_CBUS Reg dump =============\n");
	for (i=0; i<0x100; i++)
	{
		if (!(i%16))
			seq_printf(s, "\n");
		
		i2c_ret = 0;
		seq_printf(s, "[%02x]=0x%02x, ", i, SiiRegRead_check(TX_PAGE_CBUS | (i) ,  &i2c_ret) );
		if (i2c_ret !=SII_SUCCESS)
			return -1;		
	}
/*
	seq_printf("\n============== TX_PAGE_DDC_EDID Reg dump =============\n");
	for (i=0; i<0xff; i++)
	{
		seq_printf("[%02x]=0x%02x, ", i, SiiRegRead(TX_PAGE_DDC_EDID | (i) ) );
		if (!(i%16))
			seq_printf("\n");
	}
	seq_printf("\n============== TX_PAGE_DDC_SEGM Reg dump =============\n");
	for (i=0; i<0xff; i++)
	{
		seq_printf("[%02x]=0x%02x, ", i, SiiRegRead(TX_PAGE_DDC_SEGM | (i) ) );
		if (!(i%16))
			seq_printf("\n");
	}
*/
      return 0;

}

static int dbg_Sii8240_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_Sii8240_Reg_dump, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_Sii8240_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_Sii8240_TPISysCtrl_off(struct seq_file *s, void *unused)
{
//            SiiRegModify(TPI_SYSTEM_CONTROL_DATA_REG,
//				TMDS_OUTPUT_CONTROL_MASK,
//				TMDS_OUTPUT_CONTROL_POWER_DOWN);

//		SiiRegModify(REG_HPD_CTRL
//	        , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_MASK
//	        , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_OFF
  //          );

		SiiRegModify(REG_TPI_OUTPUT
	        , BIT_TPI_OUTPUT_FORMAT_MASK
	        , BIT_TPI_OUTPUT_FORMAT_HDMI_TO_RGB
          );


		return 0;			
}

static int dbg_Sii8240_TPISysCtrl_off_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_Sii8240_TPISysCtrl_off, &inode->i_private);
}

static const struct file_operations debug_TPISysCtrl_off_fops = {
	.open		= dbg_Sii8240_TPISysCtrl_off_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_Sii8240_TPISysCtrl_on(struct seq_file *s, void *unused)
{          
			
      //SiiRegModify(TPI_SYSTEM_CONTROL_DATA_REG,
	//			TMDS_OUTPUT_CONTROL_MASK,
	//			TMDS_OUTPUT_CONTROL_ACTIVE);
	
//		SiiRegModify(REG_HPD_CTRL
//	        , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_MASK
//	        , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_ON
  //          );

      SiiRegWrite(REG_TPI_AVI_BYTE13, SiiRegRead(REG_TPI_AVI_BYTE13));
	  
      return 0;			
}

static int dbg_Sii8240_TPISysCtrl_on_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_Sii8240_TPISysCtrl_on, &inode->i_private);
}

static const struct file_operations debug_TPISysCtrl_on_fops = {
	.open		= dbg_Sii8240_TPISysCtrl_on_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_Sii8240_AVI_InfoFrame_on(struct seq_file *s, void *unused)
{          
			
      SiiRegWrite(REG_TPI_AVI_BYTE13, 0x07);
      return 0;			
}

static int dbg_Sii8240_AVI_InfoFrame_on_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_Sii8240_AVI_InfoFrame_on, &inode->i_private);
}

static const struct file_operations debug_AVI_fops = {
	.open		= dbg_Sii8240_AVI_InfoFrame_on_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

//Larry debug#1080p , color issue,  not sure to fix +++
void toggletxoutput_on(void)
{
	      printk(("##toggletxoutput_on +++\n"));

		if (fwPowerState == POWER_STATE_D3)
		{
			printk("power state D3, By pass toggletxoutput_on\n");
			return;
		}

		if ((!packedPixelStatus) && (g_pad_tv_mode == MHL_TV_MODE)) 
		{
	      		printk(("toggletxoutput_on (1)\n"));
		
			SiiRegWrite(REG_VID_MODE, 0x12);  // KH, Enable Demuxing data
			printk("[toggletxoutput_on], Enable Demuxing data\n");
			SiiRegWrite(REG_VID_MODE, 0x00);  // KH, Disable Demuxing data
			printk("[toggletxoutput_on], Disable Demuxing data\n");
			MHL_Resume = true;
		}

	      printk(("toggletxoutput_on (2)\n"));

            SiiRegModify(TPI_SYSTEM_CONTROL_DATA_REG,
				TMDS_OUTPUT_CONTROL_MASK,
				TMDS_OUTPUT_CONTROL_ACTIVE);
			
	      printk(("toggletxoutput_on ---\n"));
}

void toggletxoutput_off(void)
{
	      printk(("toggletxoutput_off +++\n"));

		if (fwPowerState == POWER_STATE_D3)
		{
			printk("power state D3, By pass toggletxoutput_off\n");
			return;
		}

	      printk(("toggletxoutput_off TPI power down\n"));
		  
            SiiRegModify(TPI_SYSTEM_CONTROL_DATA_REG,
				TMDS_OUTPUT_CONTROL_MASK,
				TMDS_OUTPUT_CONTROL_POWER_DOWN);
					
	      printk(("toggletxoutput_off ---\n"));
}
//Larry debug#1080p , color issue,  not sure to fix ---

static ssize_t carkit_switch_name(struct switch_dev *sdev, char *buf)
{
       
	return sprintf(buf, "Asus-carkit\n");
}


static ssize_t carkit_switch_state(struct switch_dev *sdev, char *buf)
{
       return sprintf(buf, "%d\n", g_b_isCarkitConnected);
}

uint8_t SiiDrvMipiGetSourceStatus( void )
{
    return 1;
}

static uint8_t colorSpaceTranslateInfoFrameToHw[]=
    {
         BIT_TPI_INPUT_FORMAT_RGB
        ,BIT_TPI_INPUT_FORMAT_YCbCr422
        ,BIT_TPI_INPUT_FORMAT_YCbCr444
        ,BIT_TPI_INPUT_FORMAT_INTERNAL_RGB // reserved for future
    };
static uint8_t colorSpaceTranslateHwToInfoFrame[] =
    {
         acsRGB
        ,acsYCbCr444
        ,acsYCbCr422
        ,acsFuture
    };
//SII8240_VER75 +++
void SiiMhlTxDrvSetPixelClockFrequency(uint32_t pixelClockFrequencyParm)
{
    pixelClockFrequency = pixelClockFrequencyParm;
}
//SII8240_VER75 ---
void SiiMhlTxDrvSetOutputColorSpaceImpl(uint8_t  outputClrSpc)
{
    hwOutputColorSpace = colorSpaceTranslateInfoFrameToHw[outputClrSpc];
}
void SiiMhlTxDrvSetInputColorSpaceImpl(uint8_t inputClrSpc)
{
    hwInputColorSpace = colorSpaceTranslateInfoFrameToHw[inputClrSpc];
}

#ifdef ENABLE_COLOR_SPACE_DEBUG_PRINT //(

void PrintColorSpaceSettingsImpl(char *pszId,int iLine)
{
    COLOR_SPACE_DEBUG_PRINT(("\n%s:%d\n"
            "\t\tICS: sw:%02x hw:%02x \n"
            "\t\tOCS: sw:%02x hw:%02x\n\n"
    		,pszId,iLine
            ,colorSpaceTranslateHwToInfoFrame[hwInputColorSpace]
            ,hwInputColorSpace
            ,colorSpaceTranslateHwToInfoFrame[hwOutputColorSpace]
            ,hwOutputColorSpace
            ));
}

#define PrintColorSpaceSettings(id,line) PrintColorSpaceSettingsImpl(id,line);

void SiiMhlTxDrvSetOutputColorSpaceWrapper(char *pszId,int iLine,uint8_t  outputClrSpc)
{
    SiiMhlTxDrvSetOutputColorSpaceImpl(outputClrSpc);
    PrintColorSpaceSettings(pszId,iLine)
}

void SiiMhlTxDrvSetInputColorSpaceWrapper(char *pszId,int iLine,uint8_t inputClrSpc)
{
    SiiMhlTxDrvSetInputColorSpaceImpl(inputClrSpc);
    PrintColorSpaceSettings(pszId,iLine)
}

#else //)(

#define PrintColorSpaceSettings(id,line)

#endif //)
//SII8240_VER75 +++
void InitTranscodeMode(void)
{
	SiiRegWrite(REG_TMDS_CLK_EN, 0x01); // Enable TxPLL clock
	SiiRegWrite(REG_TMDS_CH_EN, 0x11); // Enable Tx clock path & Equalizer
    SiiRegWrite(REG_TMDS_CCTRL, BIT_TMDS_CCTRL_BGRCTL_MASK & 4); // Enable Rx PLL clock
	SiiRegWrite(REG_DISC_CTRL1, 0x27); // Enable CBUS discovery
}
//SII8240_VER75 ---

//-------------------------------------------------------------------------------------------------
//! @brief      Driver API to set or clear MHL 2 enhancement of higher modes/freq. Called only when dcap read is done.
//!
//! @param[in]  supportPackedPixel - indicate if video link will be supporting packed pixel as per peer's devcap
//! @retval     None
//-------------------------------------------------------------------------------------------------
void	SiiMhlTxDrvSetPackedPixelStatus( int supportPackedPixel )
{
	EDID_DEBUG_PRINT(("Setting Packed Pixel = %02X\n", supportPackedPixel));
    packedPixelStatus = supportPackedPixel;
}

uint8_t SiiMhlTxDrvGetPackedPixelStatus( void )
{
    return packedPixelStatus;
}

void SiiMhlTxDrvSetInputQuantizationRange(QuantizationSettings_e qsData)
{
    hwInputQuantizationSettings = qsData;
}

void SiiMhlTxDrvSetOutputQuantizationRange(QuantizationSettings_e qsData)
{
    hwOutputQuantizationSettings = qsData;
}

#ifdef THREE_D_SUPPORT //(
void SiiMhlTxDrvSet3DMode(uint8_t do3D,uint8_t three3ModeParm)
{

    if (do3D)
    {
    ThreeDStructure_e three3Mode = (ThreeDStructure_e) three3ModeParm;
        if (tdsFramePacking == three3Mode)
        {
            PP_DEBUG_PRINT(("Drv: using frame packing\n"));
            SiiRegModify(REG_VID_OVRRD,BIT_VID_OVRRD_3DCONV_EN_MASK,BIT_VID_OVRRD_3DCONV_EN_FRAME_PACK);
        }
        else
        {
            PP_DEBUG_PRINT(("Drv: NOT using frame packing\n"));
            SiiRegModify(REG_VID_OVRRD,BIT_VID_OVRRD_3DCONV_EN_MASK,BIT_VID_OVRRD_3DCONV_EN_NORMAL);
        }
    }
    else
    {
        PP_DEBUG_PRINT(("Drv: NOT using frame packing\n"));
        SiiRegModify(REG_VID_OVRRD,BIT_VID_OVRRD_3DCONV_EN_MASK,BIT_VID_OVRRD_3DCONV_EN_NORMAL);
    }
}
#endif //)
//SII8240_VER81 +++
void SiiMhlTxDrvSetHdmiMode( void )
{
	// Page2.0xA1[2] = 1
	SiiRegModify(REG_RX_HDMI_CTRL0
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_overwrite_MASK
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_overwrite_SW_CTRL
	    );

	// Page2.0xA1[3] = 1
	SiiRegModify(REG_RX_HDMI_CTRL0
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_sw_value_MASK
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_sw_value_HDMI
	    );

	TX_DEBUG_PRINT(("SiiMhlTxDrvSetHdmiMode (HDMI)\n"));
       ModifyTpiSystemControlDataReg(TMDS_OUTPUT_MODE_MASK, TMDS_OUTPUT_MODE_HDMI)
	// Change packet filters to drop AIF and GCP.
//ASUS_BSP+++ larry lai : high resolution color issue
#ifdef AVI_bypass	
			SiiRegWrite(REG_PKT_FILTER_0, 0xA1);
#else	
//keno20121001  // workaround for hdcp
//	SiiRegWrite(REG_PKT_FILTER_0, 0xA5);
#endif	
//ASUS_BSP--- larry lai : high resolution color issue
	SiiRegWrite(REG_PKT_FILTER_1, 0x0);
    SiiRegModify(REG_RX_HDMI_CTRL2
        ,BIT_RX_HDMI_CTRL2_USE_AV_MUTE_SUPPORT_MASK
        ,BIT_RX_HDMI_CTRL2_USE_AV_MUTE_SUPPORT_ENABLE
        );
}

void SiiMhlTxDrvSetDviMode( void )
{
	// Page2.0xA1[2] = 1
	SiiRegModify(REG_RX_HDMI_CTRL0
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_overwrite_MASK
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_overwrite_SW_CTRL
	    );

	// Page2.0xA1[3] = 0
	SiiRegModify(REG_RX_HDMI_CTRL0
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_sw_value_MASK
	    , BIT_REG_RX_HDMI_CTRL0_hdmi_mode_sw_value_DVI
	    );

	TX_DEBUG_PRINT(("SiiMhlTxDrvSetDviMode (DVI)\n"));
    ModifyTpiSystemControlDataReg(TMDS_OUTPUT_MODE_MASK, TMDS_OUTPUT_MODE_DVI)
	// Change packet filters to drop all packets
	SiiRegWrite(REG_PKT_FILTER_0, 0xFF);
	SiiRegWrite(REG_PKT_FILTER_1, 0xFF);
    SiiRegModify(REG_RX_HDMI_CTRL2
        ,BIT_RX_HDMI_CTRL2_USE_AV_MUTE_SUPPORT_MASK
        ,BIT_RX_HDMI_CTRL2_USE_AV_MUTE_SUPPORT_DISABLE
        );
}
//SII8240_VER81 ---

void SiiMhlTxDrvApplyColorSpaceSettings(void)
{
//SII8240_VER75 +++
    //( 2012-05-25 bugzilla 24790
    if (mfdrvTranscodeMode & g_modeFlags)
    {
        PP_DEBUG_PRINT(("Drv: Transcode zone control: D0\n"));
        SiiRegWrite(REG_ZONE_CTRL_SW_RST,0xD0);
    }
    else
    {
        if (pixelClockFrequency > 75000000)
        {
            PP_DEBUG_PRINT(("Drv: zone control: D0\n"));
            SiiRegWrite(REG_ZONE_CTRL_SW_RST,0xD0);
        }
        else
        {
            PP_DEBUG_PRINT(("Drv: zone control: E0\n"));
            SiiRegWrite(REG_ZONE_CTRL_SW_RST,0xE0);
        }
    }
    //)
//SII8240_VER75 ---
//SII8240_VER81 +++
	if(SiiMhlTxOutputModeIsHDMI())
	{
        SiiMhlTxDrvSetHdmiMode();
	}
	else
	{
        SiiMhlTxDrvSetDviMode();
	}
//SII8240_VER81 ---	
    if (packedPixelStatus)
	{
		// PackedPixel Mode
		SiiRegModify(REG_VID_MODE,BIT_VID_MODE_m1080p_MASK,BIT_VID_MODE_m1080p_ENABLE); // Packed Pixel mode enabled.
        SiiRegModify(REG_MHLTX_CTL4,BIT_MHLTX_CTL4_MHL_CLK_RATIO_MASK,BIT_MHLTX_CTL4_MHL_CLK_RATIO_2X);  // Bit6 set to 0 for PP Mode clock 2x.

		SiiRegWrite (REG_MHLTX_CTL6, 0x60);  // rcommon mode clock
        PP_DEBUG_PRINT(("Drv: Using 16-bit mode (Packed Pixel)\n"));
	}
    else
    {
		// normal Mode
        // Packed Pixel mode disabled.
#ifndef KENO_DONGLE_DOWNSTREAM1
        SiiRegModify(REG_VID_MODE,BIT_VID_MODE_m1080p_MASK,BIT_VID_MODE_m1080p_DISABLE);
#else
	if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
	{
	        SiiRegModify(REG_VID_MODE,BIT_VID_MODE_m1080p_MASK,BIT_VID_MODE_m1080p_DISABLE);		
	}
#endif
        // Bit6 set to 1 for normal Mode clock 3x.
        SiiRegModify(REG_MHLTX_CTL4,BIT_MHLTX_CTL4_MHL_CLK_RATIO_MASK,BIT_MHLTX_CTL4_MHL_CLK_RATIO_3X);

		SiiRegWrite (REG_MHLTX_CTL6, 0xA0);  // rcommon mode clock
        PP_DEBUG_PRINT(("Drv: Using 24-bit mode (non-Packed Pixel)\n"));
    }
    // Set input color space
//ASUS_BSP+++ larry lai : high resolution color issue
#ifdef AVI_bypass_USE_QCOM_AVI

	SiiRegModify(REG_TPI_INPUT
	            , BIT_TPI_INPUT_FORMAT_MASK  | BIT_TPI_INPUT_QUAN_RANGE_MASK
	            , BIT_TPI_INPUT_FORMAT_RGB          | (hwInputQuantizationSettings << 2)
	            );
#else
	SiiRegModify(REG_TPI_INPUT
	            , BIT_TPI_INPUT_FORMAT_MASK  | BIT_TPI_INPUT_QUAN_RANGE_MASK
	            , hwInputColorSpace          | (hwInputQuantizationSettings << 2)
	            );
#endif
//ASUS_BSP--- larry lai : high resolution color issue
    // Set output color space
    SiiRegModify(REG_TPI_OUTPUT
                , BIT_TPI_OUTPUT_FORMAT_MASK | BIT_TPI_OUTPUT_QUAN_RANGE_MASK
                , hwOutputColorSpace         | (hwOutputQuantizationSettings<< 2)
                );

    PrintColorSpaceSettings("SiiMhlTxDrvApplyColorSpaceSettings",__LINE__)
}

uint8_t SiiMhlTxDrvGetOutputColorSpace(void)
{
uint8_t retVal;
    retVal = colorSpaceTranslateHwToInfoFrame[hwOutputColorSpace];
    COLOR_SPACE_DEBUG_PRINT(("hwOutputColorSpace:0x%02x retVal:0x%02x\n",hwOutputColorSpace,retVal));
    return retVal;
}

uint8_t SiiMhlTxDrvGetInputColorSpace( void )
{
uint8_t retVal;
    retVal = colorSpaceTranslateHwToInfoFrame[hwInputColorSpace];
    COLOR_SPACE_DEBUG_PRINT(("hwInputColorSpace:0x%02x retVal:0x%02x\n",hwInputColorSpace,retVal));
    return retVal;
}

void SiiMhlTxDrvSetColorimetryAspectRatio (AviInfoFrameDataByte2_t colAspRat )
{
    colorimetryAspectRatio = colAspRat;
}

AviInfoFrameDataByte2_t SiiMhlTxDrvGetColorimetryAspectRatio ( void )
{
    return colorimetryAspectRatio;
}

void SiiMhlTxDrvSetInputVideoCode (AviInfoFrameDataByte4_t inputVIC)
{
    inputVideoCode = inputVIC;
}

AviInfoFrameDataByte4_t SiiMhlTxDrvGetInputVideoCode ( void )
{
    return inputVideoCode;
}

uint16_t SiiMhlTxDrvGetIncomingHorzTotal(void)
{
uint16_t retVal;
    retVal = (((uint16_t)SiiRegRead(REG_HRESH)) <<8) | (uint16_t)SiiRegRead(REG_HRESL);
    return retVal;
}

uint16_t SiiMhlTxDrvGetIncomingVertTotal(void)
{
uint16_t retVal;
    retVal = (((uint16_t)SiiRegRead(REG_VRESH)) <<8) | (uint16_t)SiiRegRead(REG_VRESL);
    return retVal;
}


#define	SET_BIT(offset,bitnumber)		SiiRegModify(offset,(1<<bitnumber),(1<<bitnumber))
#define	CLR_BIT(offset,bitnumber)		SiiRegModify(offset,(1<<bitnumber),0x00)
//
//
#define	DISABLE_DISCOVERY				SiiRegModify(REG_DISC_CTRL1,BIT0,0);
#define	ENABLE_DISCOVERY				SiiRegModify(REG_DISC_CTRL1,BIT0,BIT0);

#define STROBE_POWER_ON					SiiRegModify(REG_DISC_CTRL1,BIT1,0);

typedef struct _InterruptEnableMaskInfo_t
{
    uint16_t regName;
    uint8_t  value;
}InterruptEnableMaskInfo_t,*PInterruptEnableMaskInfo_t;

InterruptEnableMaskInfo_t  g_TranscodeInterruptMasks[]=
{
     {REG_INTR7_MASK            , 0 }
    ,{REG_INTR8_MASK            , 0 }
    ,{REG_TPI_INTR_ST0_ENABLE   , 0 }
};

InterruptEnableMaskInfo_t  g_NonTranscodeInterruptMasks[]=
{
     {REG_INTR7_MASK            ,(
                                      BIT_INTR7_CEA_NO_AVI
                                    | BIT_INTR7_CEA_NO_VSI
                                  //  | BIT_INTR7_CP_NEW_CP
                                    | BIT_INTR7_CP_SET_MUTE
                                    | BIT_INTR7_CP_CLR_MUTE
                                )}
    ,{REG_INTR8_MASK            ,(
                                      BIT_INTR8_CEA_NEW_AVI
                                    | BIT_INTR8_CEA_NEW_VSI
                                )}
//ASUS_BSP Tom Chu : workaround to mask all HDCP interrupt for pad mode i2c busy issue
    ,{REG_TPI_INTR_ST0_ENABLE   , 0 /*(
                                      BIT_TPI_INTR_ST0_HDCP_AUTH_STATUS_CHANGE_EVENT
                                    | BIT_TPI_INTR_ST0_HDCP_VPRIME_VALUE_READY_EVENT
                                    | BIT_TPI_INTR_ST0_HDCP_SECURITY_CHANGE_EVENT
                                    | BIT_TPI_INTR_ST0_BKSV_DONE
                                    | BIT_TPI_INTR_ST0_BKSV_ERR
                                )*/}

};

InterruptEnableMaskInfo_t g_InterruptMasks[]=
{
     {REG_INTR1_MASK            ,(
                                      BIT_INTR1_HPD_CHG
                                    | BIT_INTR1_RSEN_CHG
                                )}
    ,{REG_INTR2_MASK            ,0}
    ,{REG_INTR3_MASK            ,0}
    ,{REG_INTR4_MASK            ,(
//ASUS_BSP+++ larry lai : fix Pad mode TX hange issue
#ifdef Disable_VBUS_Interrupt	
//disable. when pwron, VBUS_IN high to 1.8v, this interupt will happen event.		
                                    /*  BIT_INTR4_VBUS_CHG
                                    |*/ 
                                    BIT_INTR4_MHL_EST
#else
 					    BIT_INTR4_VBUS_CHG
                                    | BIT_INTR4_MHL_EST
#endif
//ASUS_BSP--- larry lai : fix Pad mode TX hange issue
                                    |BIT_INTR4_NON_MHL_EST
                                    | BIT_INTR4_CBUS_LKOUT
                                    | BIT_INTR4_CBUS_DISCONNECT
                                    | BIT_INTR4_RGND_DETECTION
                                )}
#ifdef Disable_INTR5_MHL_FIFO_OVERFLOW
    ,{REG_INTR5_MASK            ,(
                                    BIT_INTR5_CKDT_CHANGE
                                    | BIT_INTR5_SCDT_CHANGE
                                )}
#else
    ,{REG_INTR5_MASK            ,(
                                      BIT_INTR5_MHL_FIFO_UNDERFLOW
                                    | BIT_INTR5_MHL_FIFO_OVERFLOW
                                    | BIT_INTR5_CKDT_CHANGE
                                    | BIT_INTR5_SCDT_CHANGE
                                )}
#endif
//SII8240_VER81 +++									
    ,{REG_INTR9_MASK            ,(
                                      BIT_INTR9_DEVCAP_DONE_MASK
//SII8240_VER81 ---									  
                                )}
    ,{REG_CBUS_INT_0_MASK       ,(
                                      BIT_CBUS_CNX_CHG
                                    | BIT_CBUS_MSC_MT_DONE
                                    | BIT_CBUS_HPD_RCVD
                                    | BIT_CBUS_MSC_MR_WRITE_STAT
//SII8240_VER81_1 +++									
                                    #if 1//Enable MSC_MSG interrupt  //hammer RCP test
                                    | BIT_CBUS_MSC_MR_MSC_MSG
                                    #endif 
//SII8240_VER81_1 ---									
                                    | BIT_CBUS_MSC_MR_WRITE_BURST
                                    | BIT_CBUS_MSC_MR_SET_INT
                                    | BIT_CBUS_MSC_MT_DONE_NACK
                                    )}
    ,{REG_CBUS_INT_1_MASK       ,(
                                      BIT_CBUS_DDC_ABRT
                                    | BIT_CBUS_CEC_ABRT
                                    | BIT_CBUS_CMD_ABORT
                                )}
};


#define I2C_INACCESSIBLE -1
#define I2C_ACCESSIBLE 1

//
// Local scope functions.
//
static void Int1Isr (void);
static int  Int4Isr (void);
static void Int5Isr (void);
static void MhlCbusIsr (void);

static void CbusReset (void);
static void SwitchToD0 (void);
static void SwitchToD3 (void);
static void WriteInitialRegisterValuesPartOne (void);
static void WriteInitialRegisterValuesPartTwo (void);
static void InitCBusRegs (void);
static void ForceUsbIdSwitchOpen (void);
static void ReleaseUsbIdSwitchOpen (void);
static void MhlTxDrvProcessConnection (void);
static void MhlTxDrvProcessDisconnection (void);

uint8_t g_chipRevId;
uint16_t g_chipDeviceId;

    // make sure that TMDS is not enabled prior to enabling TPI
#define SetTPIMode \
{ \
uint8_t tpiSel; \
    tpiSel =SiiRegRead(REG_TPI_SEL); \
	TX_DEBUG_PRINT(("Drv: REG_TPI_SEL:%02x\n",tpiSel)); \
    tpiSel &= ~BIT_TPI_SEL_SW_TPI_EN_MASK;   \
    tpiSel |= BIT_TPI_SEL_SW_TPI_EN_HW_TPI;   \
    SiiRegWrite(REG_TPI_SEL, tpiSel);\
	TX_DEBUG_PRINT(("Drv: REG_TPI_SEL:%02x\n",tpiSel)); \
}
#define ClrTPIMode SiiRegModify(REG_TPI_SEL,BIT_TPI_SEL_SW_TPI_EN_MASK ,BIT_TPI_SEL_SW_TPI_EN_NON_HW_TPI);

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvAcquireUpstreamHPDControl
//
// Acquire the direct control of Upstream HPD.
//
void SiiMhlTxDrvAcquireUpstreamHPDControl (void)
{
	// set reg_hpd_out_ovr_en to first control the hpd

	SiiRegModify(REG_HPD_CTRL
	    , BIT_HPD_CTRL_HPD_OUT_OVR_EN_MASK
	    , BIT_HPD_CTRL_HPD_OUT_OVR_EN_ON
	    );
	TX_DEBUG_PRINT(("Drv: Upstream HPD Acquired.\n"));
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow
//
// Acquire the direct control of Upstream HPD.
//
void SiiMhlTxDrvAcquireUpstreamHPDControlDriveLowImpl (char *pszFile,int iLine)
{
//ASUS_BSP+++ larry lai : fix Pad mode TX hange issue
#ifdef Disable_SWWA_FLAKY_US_HPD
#else
//#define SWWA_FLAKY_US_HPD
#endif
//ASUS_BSP--- larry lai : fix Pad mode TX hange issue

#ifdef SWWA_FLAKY_US_HPD //(
    {
    uint8_t temp,holder;
//	if (g_pad_tv_mode == MHL_TV_MODE)
	{
        temp = SiiRegRead(REG_HPD_CTRL);
        temp &= ~(BIT_HPD_CTRL_HPD_OUT_OVR_VAL_MASK | BIT_HPD_CTRL_HPD_OUT_OVR_EN_MASK);
        temp |=  (BIT_HPD_CTRL_HPD_OUT_OVR_VAL_OFF  | BIT_HPD_CTRL_HPD_OUT_OVR_EN_ON);
        do
        {
        	SiiRegWrite(REG_HPD_CTRL,temp);
            // write it twice to be sure
        	holder = SiiRegRead(REG_HPD_CTRL);
        } while (temp != holder);
        // one more time to be sure
        SiiRegWrite(REG_HPD_CTRL,temp);
	}
    }
#else //)(
//	if (g_pad_tv_mode == MHL_TV_MODE)
	{
	// set reg_hpd_out_ovr_en to first control the hpd and clear reg_hpd_out_ovr_val
	SiiRegModify(REG_HPD_CTRL
	    , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_MASK | BIT_HPD_CTRL_HPD_OUT_OVR_EN_MASK
	    , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_OFF  | BIT_HPD_CTRL_HPD_OUT_OVR_EN_ON
	    );
	}
#endif //)

    upStreamHPD = 0;
    SiiMhlTxNotifyUpStreamHPD(upStreamHPD);
	TX_DEBUG_PRINT(("Drv: Upstream HPD Acquired - driven low.called from %s:%d\n",pszFile,iLine));
}

#define SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow() SiiMhlTxDrvAcquireUpstreamHPDControlDriveLowImpl(__FILE__,__LINE__)

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvAcquireUpstreamHPDControlDriveHigh
//
// Acquire the direct control of Upstream HPD.
//
void SiiMhlTxDrvAcquireUpstreamHPDControlDriveHighImpl (char *pszFile,int iLine)
{
//	if (g_pad_tv_mode == MHL_TV_MODE)
	{

	// set reg_hpd_out_ovr_en to first control the hpd and clear reg_hpd_out_ovr_val
	SiiRegModify(REG_HPD_CTRL
	    , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_MASK | BIT_HPD_CTRL_HPD_OUT_OVR_EN_MASK
	    , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_ON   | BIT_HPD_CTRL_HPD_OUT_OVR_EN_ON
	    );
	}

    upStreamHPD = 1;
    SiiMhlTxNotifyUpStreamHPD(upStreamHPD);
	TX_DEBUG_PRINT(("Drv: Upstream HPD Acquired - driven high. called from %s:%d\n",pszFile,iLine));
}
#define SiiMhlTxDrvAcquireUpstreamHPDControlDriveHigh() SiiMhlTxDrvAcquireUpstreamHPDControlDriveHighImpl(__FILE__,__LINE__)


///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvReleaseUpstreamHPDControl
//
// Release the direct control of Upstream HPD.
//
void SiiMhlTxDrvReleaseUpstreamHPDControlImpl (char *pszFile,int iLine)
{
   	// Un-force HPD (it was kept low, now propagate to source)
	// let HPD float by clearing reg_hpd_out_ovr_en

	SiiRegModify(REG_HPD_CTRL
	    , BIT_HPD_CTRL_HPD_OUT_OVR_EN_MASK
	    , BIT_HPD_CTRL_HPD_OUT_OVR_EN_OFF
	    );
	TX_DEBUG_PRINT(("Drv: Upstream HPD released. called from %s:%d\n",pszFile,iLine));
}
#define SiiMhlTxDrvReleaseUpstreamHPDControl() SiiMhlTxDrvReleaseUpstreamHPDControlImpl (__FILE__,__LINE__)

static void Int1Isr(void)
{
uint8_t regIntr1;
    regIntr1 = SiiRegRead(REG_INTR1);
    if (regIntr1)
    {
/*	  if (regIntr1 & BIT_INTR1_RSEN_CHG)
	  {
	  	printk("[MHL] Int1Isr BIT_INTR1_RSEN_CHG\n");
	  }
*/	  
        // Clear all interrupts coming from this register.
        SiiRegWrite(REG_INTR1,regIntr1);

    }
}

void    SiiMhlTxDrvTimerHDCPCallBack(void)
{
    TX_DEBUG_PRINT(("TIMER_HDCP expired\n"));
}

//SII8240_VER81 +++
#ifdef ENABLE_HDCP_DEBUG_PRINT //(

void DumpTPIDebugRegsImpl(void)
{
uint8_t debugStatus[7];
    SiiRegReadBlock(REG_TPI_HW_DBG1,debugStatus,sizeof(debugStatus));
    ERROR_DEBUG_PRINT((
        "HDCP: debugStatus[]={%02x %02x %02x %02x %02x %02x %02x}\n\n"
                , (uint16_t)debugStatus[0]
                , (uint16_t)debugStatus[1]
                , (uint16_t)debugStatus[2]
                , (uint16_t)debugStatus[3]
                , (uint16_t)debugStatus[4]
                , (uint16_t)debugStatus[5]
                , (uint16_t)debugStatus[6]
                ));
}
#define DumpTPIDebugRegs DumpTPIDebugRegsImpl();

#else //)(

#define DumpTPIDebugRegs /* nothing */

#endif //)
//SII8240_VER81 ---

static void TpiIsr(void)
{
uint8_t tpiIntStatus;
    tpiIntStatus = SiiRegRead(REG_TPI_INTR_ST0);
    if (tpiIntStatus)
    {
//SII8240_VER75 +++
    uint8_t queryData = SiiRegRead(TPI_HDCP_QUERY_DATA_REG);
//SII8240_VER81 +++	
#ifdef ENABLE_HDCP_DEBUG_PRINT //(
    uint8_t debugStatus[7];
        SiiRegReadBlock(REG_TPI_HW_DBG1,debugStatus,sizeof(debugStatus));
#endif //)
//SII8240_VER81 ---
        // ALWAYS clear interrupt status BEFORE processing!
        SiiRegWrite(REG_TPI_INTR_ST0,tpiIntStatus);
//SII8240_VER81 +++
        HDCP_DEBUG_PRINT(("HDCP: debugStatus[]={%02x %02x %02x %02x %02x %02x %02x}\n\n"
                    , (uint16_t)debugStatus[0]
                    , (uint16_t)debugStatus[1]
                    , (uint16_t)debugStatus[2]
                    , (uint16_t)debugStatus[3]
                    , (uint16_t)debugStatus[4]
                    , (uint16_t)debugStatus[5]
                    , (uint16_t)debugStatus[6]
                    ));

#if 0 //def ENABLE_HDCP_DEBUG_PRINT //(
        while (0x10 == debugStatus[5])
        {
        uint8_t riStatus[2];
        uint8_t temp = SiiRegRead(REG_INTR2);
        uint8_t intStatus = SiiRegRead(REG_TPI_INTR_ST0);
        uint8_t riPeerStatus[2];
            SiiRegReadBlock(TPI_HDCP_RI_LOW_REG,riStatus,sizeof(riStatus));
            SiiRegReadBlock(REG_PEER_RI_RX_1,riPeerStatus,sizeof(riPeerStatus));
            SiiRegReadBlock(REG_TPI_HW_DBG1,debugStatus,sizeof(debugStatus));
            if (0x10 == debugStatus[5])
            {
                ERROR_DEBUG_PRINT(("Intr2 status:%02x intStatus:%02x riLow: %02x riHigh:%02x\n"
                    ,(uint16_t)temp
                    ,(uint16_t)intStatus
                    ,(uint16_t)riStatus[0]
                    ,(uint16_t)riStatus[1]
                    ));
                HalTimerWait(10);
            }
        }
#endif //)
//SII8240_VER81 ---
		TX_DEBUG_PRINT(("REG_TPI_INTR_ST0: %02bx qd: %02bx\n",tpiIntStatus,queryData));
//SII8240_VER81_1 +++		
//	#ifdef DONTCheckVideoReady
	if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
	{
	
	        uint8_t temp;
	            temp = cache_TPI_SYSTEM_CONTROL_DATA_REG;
	//SII8240_VER81
	            TX_DEBUG_PRINT(("TpiIsr\n"));
	            if (0==(TMDS_OUTPUT_CONTROL_POWER_DOWN & temp))
	            {
	//SII8240_VER81			
	                TX_DEBUG_PRINT(("TpiIsr\n"));
	                SiiHdmiTxLiteHandleEvents(tpiIntStatus,queryData);
	            }		            
    		TX_DEBUG_PRINT(("Pad mode, no handle HDCP interrupt\n"));	
	}
	else
	{
	        if (SiiMhlTxReadyForVideo())
	        {
	        uint8_t temp;
	            temp = cache_TPI_SYSTEM_CONTROL_DATA_REG;
	//SII8240_VER81
	            TX_DEBUG_PRINT(("TpiIsr\n"));
	            if (0==(TMDS_OUTPUT_CONTROL_POWER_DOWN & temp))
	            {
	//SII8240_VER81			
	                TX_DEBUG_PRINT(("TpiIsr\n"));
	                SiiHdmiTxLiteHandleEvents(tpiIntStatus,queryData);
	            }
	        }
	        else
	        {
	    		TX_DEBUG_PRINT(("Unexpected HDCP interrupt\n"));
	        }        
	}
//SII8240_VER81_1 ---
    }
//SII8240_VER75 ---

}

////////////////////////////////////////////////////////////////////
//
// E X T E R N A L L Y    E X P O S E D   A P I    F U N C T I O N S
//
////////////////////////////////////////////////////////////////////

#ifdef	MDT_TESTER //(
static	uint8_t	outBuffer[18] = {0x0f};
static	uint8_t	inBuffer [18] = {0x0f};

void	MdtErrorHalt(void)
{
	int	i;

	SiiOsDebugPrintAlwaysShort(("\n\n\n\n\MDT_ERROR: HALTED\n\n\n\nRegister values...."));

	SiiOsDebugPrintAlwaysShort("Reg 0x86 = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x86));
	SiiOsDebugPrintAlwaysShort("Reg 0x87 = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x87));
	SiiOsDebugPrintAlwaysShort("Reg 0x88 = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x88));
	SiiOsDebugPrintAlwaysShort("Reg 0x8A = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x8A));
	SiiOsDebugPrintAlwaysShort("Reg 0x8B = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x8B));
	SiiOsDebugPrintAlwaysShort("Reg 0x8C = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x8C));
	SiiOsDebugPrintAlwaysShort("Reg 0x8D = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x8D));
	SiiOsDebugPrintAlwaysShort("Reg 0x8E = %02X\n", (int)SiiRegRead(TX_PAGE_CBUS | 0x8E));

	SiiOsDebugPrintAlwaysShort("Incoming buffer is.. LENGTH = %03d\n", (int)inBuffer[0] );
	for(i = 1 ; i < 17; i++)
	{
		SiiOsDebugPrintAlwaysShort("%02X ", (int) inBuffer[ i ] );
	}

	// Completely stall to avoid any I2C traffic
	while(true);
}
//
//
//

void	MdtDataCompare(uint8_t enableMdtCode)
{
#define	STOP_BYTE		0xF0
#define	WAIT_FOR_ROCKY_IN_SECONDS	10

	uint8_t	currentByteOut = 0;
	uint8_t	currentByteIn  = 0;
	uint8_t	thisLength = 16;

	uint8_t	status;

	int	iterationCount = 0;
	int	sendCount = 0;

	int	i;
	int	printed = false;
	int	lastTime = HalTimerElapsed(ELAPSED_TIMER);
	int	newTime;

	// Once MDT tester starts, nothing else will work. We DO NOT return from here.
	SiiOsDebugPrintAlwaysShort("MdtDataCompare(%02X)\n\n.", (int)enableMdtCode);
	SiiOsDebugPrintAlwaysShort("Removed WAIT_FOR_ROCKY_IN_SECONDS delay before starting MDT. \n");
	SiiOsDebugPrintAlwaysShort("Looks at CBUS page register 0x00 to define whether to start MDT or not\n");
	SiiOsDebugPrintAlwaysShort("\n\n\n\n\n\n\n\n\n\n\n\n\nStarting MDT Tester Application. Will NOT stop at offset %04X Shutting down all others.\n\n\n", (int)STOP_BYTE);

//	SiiOsDebugPrintAlwaysShort("MDT: Wait for Rocky to be up and running %02X. May not be needed.\n", (int)WAIT_FOR_ROCKY_IN_SECONDS);
//	HalTimerWait( WAIT_FOR_ROCKY_IN_SECONDS * 1000 );
//	SiiOsDebugPrintAlwaysShort("MDT: HalTimerElapsed(ELAPSED_TIMER) %04X\n", (int)HalTimerElapsed(ELAPSED_TIMER));

	// Initialize
	// 0x8C = 0xFF	to clear the interrupt bits	(TX_PAGE_CBUS | 0x0000)
	// 0x88[7] = 1	to enable the MDT Transmitter
	// 0x86[7] = 1	to enable the MDT Receiver
    SiiRegWrite ((TX_PAGE_CBUS | 0x8C), 0xFF);			// Clear the interrupt bits
    SiiRegWrite ((TX_PAGE_CBUS | 0x88), 0x80);			// 0x88[7] = 1	to enable the MDT Transmitter
    SiiRegWrite ((TX_PAGE_CBUS | 0x86), 0x80);			// 0x86[7] = 1	to enable the MDT Receiver
    SiiRegWrite ((TX_PAGE_CBUS | 0x84), 0x02);			// Timeout = 2
    SiiRegWrite ((TX_PAGE_CBUS | 0x85), 0x02);			//

//	if(enableMdtCode == 0xaa)
	{
	   	currentByteOut = 0;
		SiiOsDebugPrintAlwaysShort("Incrementing pattern will be sent since CBUS:0 is %02X\n", (int)enableMdtCode);
	}
//	else
//	{
//	   	currentByteOut = 0xff;
//		SiiOsDebugPrintAlwaysShort("Decrementing pattern will be sent since CBUS:0 is %02X\n", (int)enableMdtCode);
//	}

	// Forever do
	//	If 8B[7:5] > 0 it means we can write
	//		Write 1+16 bytes in a loop on [89]
	//	if 8C[0] == 1 it means we got data on receiver
	//		Read [87]. if not 16, go to error
	//		Read 1+16 bytes in a loop from [87]
	//		 if(unexpected byte) go to error
	//		Move pointer to the next 86[0] = 0x81
	// done
	// error: dump registers 86 to 8F, except 87 and 89?
	//		  dump 16 bytes from 87
	//
	while(enableMdtCode)
	{
		// print a dot every 1 second of run.
		newTime = HalTimerElapsed(ELAPSED_TIMER);
		if (newTime > (lastTime + 1000))
		{
			SiiOsDebugPrintAlwaysShort(".");
			lastTime = newTime;
		}
		// Send when you can
		status = SiiRegRead(TX_PAGE_CBUS | 0x8B);
	    if( status != 0xFF)
		{
			if(status &0xE0)
			{
				sendCount++;
				lastTime = newTime;

//				if(currentByteOut < STOP_BYTE)
				{
					SiiOsDebugPrintAlwaysShort("MDT(sendCount = %5d): Writing current bytes from %02X\n", (int)sendCount, (int)currentByteOut);

					outBuffer[0] = 15;

					for(i = 1 ; i < 17; i++)
					{
						outBuffer[ i ] = currentByteOut++;
					}
					if(enableMdtCode != 0xff)
					{
						SiiRegWriteBlock((TX_PAGE_CBUS | 0x89), outBuffer, 17);
					}
					else
					{
						SiiOsDebugPrintAlwaysShort("MDT(sendCount = %5d): Packet not written since CBUS:0 = 0xFF\n");
					}
				}
/*				else if(currentByteOut == STOP_BYTE && !printed)
				{
					printed = true;
					SiiOsDebugPrintAlwaysShort("MDT: Stopping after %02d writes.\n", (int)currentByteOut >> 4);
				}*/
			}
		}
		//
		// Receive when you can
		// if 8C[0] == 1 it means we got data on receiver
		//
		status = SiiRegRead(TX_PAGE_CBUS | 0x8C);
	    if( status != 0xFF)
		{
			if(0x01 & status)
			{
				lastTime = newTime;

				if( 0 == currentByteIn)
				{
					ERROR_DEBUG_PRINT(("MDT: Iteration Count = %d\n", (int)iterationCount++));
				}

				for(i = 0 ; i < 17; i++)
				{
					inBuffer[ i ] = 0xa5;
				}

				ERROR_DEBUG_PRINT(("MDT: Reading bytes Expecting from %02X\n", (int)currentByteIn));

				SiiRegReadBlock((TX_PAGE_CBUS | 0x87), inBuffer, 17);

				//
				// Check length
				//
				if( inBuffer[ 0 ] != 16)
				{
					ERROR_DEBUG_PRINT(("MDT_ERROR: LENGTH MISMATCH. Expected %02d, Got %02d\n", (int)16, (int)inBuffer[ 0 ]));
					MdtErrorHalt();
				}
				//
				// Check data
				//
				for(i = 1 ; i < 17; i++)
				{
					if( inBuffer[i] != currentByteIn)
					{
						ERROR_DEBUG_PRINT(("MDT_ERROR: DATA MISMATCH. Expected %02X, Got %02X\n", (int)currentByteIn, (int)inBuffer[i]));
						MdtErrorHalt();
					}
					currentByteIn++;
				}
				//		Move pointer to the next 86[0] = 0x81
		    	SiiRegModify((TX_PAGE_CBUS | 0x86), 0x81, 0x81);	//		Move pointer to the next 86[0] = 0x81

				// Clear interrupt
      			SiiRegWrite ((TX_PAGE_CBUS | 0x8C), 0x01);			// Clear the interrupt bit
			}
		}

		if(enableMdtCode < 0xaa)
		{
			if(enableMdtCode == sendCount)
			{
				enableMdtCode = false;
			}
		}
	}
}
#endif	//)

///////////////////////////////////////////////////////////////////////////////
//
// SiiAnyPollingDebug
//
// Alert the driver that the peer's POW bit has changed so that it can take
// action if necessary.
//
#ifdef	DEBUG_RI_VALUES //(
static	uint8_t	counter = 0;
#endif //)
int		g_ri_display = false;

void	SiiAnyPollingDebug( void )
{

#ifdef	MDT_TESTER
	uint8_t	enableMdtCode = SiiRegRead(TX_PAGE_CBUS | 0x00);

	if(enableMdtCode)
	{
		SiiRegWrite((TX_PAGE_CBUS | 0x00), 0);
		MdtDataCompare(enableMdtCode);
	}
#endif	//	MDT_TESTER


#ifdef	DEBUG_RI_VALUES
//SII8240_VER75 +++
    if (!(mfdrvTranscodeMode & g_modeFlags))
    {
	uint8_t		remoteRiLow, remoteRiHigh;
	uint8_t		localRiLow, localRiHigh;
    if(g_ri_display)
    {
    	SiiRegWrite(REG_RI_LOCAL_OR_REMOTE, 0x40);
    	remoteRiLow  = SiiRegRead(TPI_HDCP_RI_LOW_REG);
    	remoteRiHigh = SiiRegRead(TPI_HDCP_RI_HIGH_REG);

    	SiiRegWrite(REG_RI_LOCAL_OR_REMOTE, 0x00);
    	localRiLow   = SiiRegRead(TPI_HDCP_RI_LOW_REG);
    	localRiHigh  = SiiRegRead(TPI_HDCP_RI_HIGH_REG);

    	if( 0 == (0x20 & (counter++)))
    	{
    		TX_DEBUG_PRINT(("Remote Ri = %02X, %02X. Local  Ri = %02X, %02X\n",
    				(int) remoteRiLow,
    				(int) remoteRiHigh,
    				(int) localRiLow,
    				(int) localRiHigh));
    	}
    }
    }
//SII8240_VER75 ---	
#endif //	DEBUG_RI_VALUES
}


int CarKitInitialize(void)	
{
       int ret=0;

#ifdef CONFIG_DEBUG_FS
       debugfs_create_file("sii8240", S_IRUGO, NULL, NULL, &debug_fops);
       debugfs_create_file("sii8240_tpi_on", S_IRUGO, NULL, NULL, &debug_TPISysCtrl_on_fops);
       debugfs_create_file("sii8240_tpi_off", S_IRUGO, NULL, NULL, &debug_TPISysCtrl_off_fops);
       debugfs_create_file("sii8240_avi", S_IRUGO, NULL, NULL, &debug_AVI_fops);
#endif   	   
	   
//ASUS_BSP , force return for OTG mode
	return 0;

	switch_carkit_cable.name="carkitcable";
	switch_carkit_cable.print_state=carkit_switch_state;
	switch_carkit_cable.print_name=carkit_switch_name;
	ret=switch_dev_register(&switch_carkit_cable);
	if (ret < 0){
	    printk("%s: Unable to register switch dev! %d\n", __FUNCTION__,ret);
	    return -1;	
	}	

 
//ASUS_BSP , for debug usage, set carkit uevent in MHL driver +++
	g_b_SwitchCarkitInitial = 1;
//ASUS_BSP , for debug usage, set carkit uevent in MHL driver ---

	return 0;

}

//////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxChipInitialize
//
// Chip specific initialization.
// This function is for SiI 8332/8336 Initialization: HW Reset, Interrupt enable.
//
//////////////////////////////////////////////////////////////////////////////

bool_t SiiMhlTxChipInitialize (void)
{	   
	mscCmdInProgress = false;	// false when it is okay to send a new command
//SII8240_VER81 +++	
#ifdef DEFERRED_HDCP_START //(
    hdcpStartPending=false;
#endif //)
//SII8240_VER81 ---
	dsHpdStatus = 0;
//SII8240_VER81 +++		
	UpdatePowerState(POWER_STATE_D0_MHL);
//SII8240_VER81 ---	
    SI_OS_DISABLE_DEBUG_CHANNEL(SII_OSAL_DEBUG_SCHEDULER);

	SiiOsDebugPrintAlwaysShort("\nFor MHL CTS HDCP Testing,\n"
	                           "\talways use a source that \n"
	                           "\tparses the EDID that we present upstream\n"
	                           "\tand sends HDMI or DVI appropriately\n\n");

#if TARGET_DEVICE_ID == WOLV60_DEVICE_ID //)(

    g_chipRevId = SiiMhlTxDrvGetDeviceRev();
    g_chipDeviceId = SiiMhlTxDrvGetDeviceId();
    TX_DEBUG_PRINT (("SiiMhlTxChipInitialize g_chipRevId = 0x%x \n", g_chipRevId));

	SiiMhlTxHwReset(TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);  // call up through the stack to accomplish reset.
	ERROR_DEBUG_PRINT(("Drv: SiiMhlTxChipInitialize: chip rev: %02X"
                    " chip id: %04X"
	                " in %s Mode\n"
	                    ,(int)g_chipRevId
                        ,(uint16_t)SiiMhlTxDrvGetDeviceId()
						,(mfdrvTranscodeMode & g_modeFlags)?"Transcode":"Non-Transcode"
					));
 	// setup device registers. Ensure RGND interrupt would happen.
	WriteInitialRegisterValuesPartOne();

#elif TARGET_DEVICE_ID == BISHOP_DEVICE_ID //(
	SiiMhlTxHwReset(TX_HW_RESET_PERIOD,TX_HW_RESET_DELAY);  // call up through the stack to accomplish reset.
	WriteInitialRegisterValuesPartOne();

    g_chipRevId = SiiMhlTxDrvGetDeviceRev();
    g_chipDeviceId = SiiMhlTxDrvGetDeviceId();

	TX_DEBUG_PRINT(("HdmiTxDrv: HDCP revision:0x%02x\n",(uint16_t)SiiRegRead(TPI_HDCP_REVISION_DATA_REG) ));
	ERROR_DEBUG_PRINT(("Drv: SiiMhlTxChipInitialize:"
                    " device: 0x%04x"
	                " rev: %02x"
	                " in %s Mode\n"
                        ,(uint16_t)g_chipDeviceId
	                    ,(uint16_t)g_chipRevId
						,(mfdrvTranscodeMode & g_modeFlags)?"Transcode":"Non-Transcode"
					));

 	// setup device registers. Ensure RGND interrupt would happen.
#endif//)
	WriteInitialRegisterValuesPartTwo();

	// setup TX
//SII8240_VER75	
    if (!(mfdrvTranscodeMode & g_modeFlags))
    {
	    SiiHdmiTxLiteInitialize(PlatformGPIOGet(pinDoHdcp)?true:false);
    }
    SiiOsMhlTxInterruptEnable();

    // check of PlatformGPIOGet(pinAllowD3) is done inside SwitchToD3
	SwitchToD3();

	TX_DEBUG_PRINT(("SiiMhlTxChipInitialize --\n"));
	return true;
}

void	HDCP_On (void);

void SiiMhlTxDrvGetAviInfoFrame(PAVIInfoFrame_t pAviInfoFrame)
{
    SiiRegModify(REG_RX_HDMI_CTRL2
        ,BIT_RX_HDMI_CTRL2_VSI_MON_SEL_MASK
        ,BIT_RX_HDMI_CTRL2_VSI_MON_SEL_AVI_INFOFRAME
   	    );
   	SiiRegReadBlock(REG_RX_HDMI_MON_PKT_HEADER1,(uint8_t *)pAviInfoFrame,sizeof(*pAviInfoFrame));
}

/*
    SiiMhlTxDrvReadIncomingInfoFrame
*/
static void SiiMhlTxDrvReadIncomingInfoFrame(void)
{
AVIInfoFrame_t aviInfoFrame;
//SII8240_VER81
    INFO_DEBUG_PRINT(("Process AVIF\n"));

    SiiMhlTxDrvGetAviInfoFrame(&aviInfoFrame);
   	//process information here

    SiiMhlTxProcessAviInfoFrameChange(&aviInfoFrame);

}

//-------------------------------------------------------------------------------------------------
//! @brief      Interrupt Service Routine for INTR 7
//!
//! @param[in]  None
//! @retval     None
//-------------------------------------------------------------------------------------------------

void Int7Isr(void)
{
uint8_t statusIntr7;
    statusIntr7 = SiiRegRead(REG_INTR7);
    if (statusIntr7)
    {
// 		TX_DEBUG_PRINT(("-----------------> GOT statusIntr7 = %02X\n\n", (int)statusIntr7));
        // clear all
        SiiRegWrite(REG_INTR7,statusIntr7);
        if (BIT_INTR7_CEA_NO_AVI & statusIntr7)
        {
            TX_DEBUG_PRINT(("Drv: NO_AVI\n"));
        }
        if (BIT_INTR7_CEA_NO_VSI & statusIntr7)
        {
            TX_DEBUG_PRINT(("Drv: NO_VSI\n"));
        }
        if (BIT_INTR7_CP_NEW_CP & statusIntr7)
        {
        }
        if (BIT_INTR7_CP_SET_MUTE & statusIntr7)
        {
	 		TX_DEBUG_PRINT(("-----------------> Got AV MUTE statusIntr7 = %02X\n\n", (int)statusIntr7));

            if (!stateFlags.upStreamMuted)
            {
                SiiMhlTxNotifyUpstreamMute();
                stateFlags.upStreamMuted = 1;
            }
        }
        if (BIT_INTR7_CP_CLR_MUTE & statusIntr7)
        {
            if (upStreamHPD)
            {
    	 		TX_DEBUG_PRINT(("-----------------> Got AV UNMUTE statusIntr7 = %02X\n\n", (int)statusIntr7));

                //	Wait for stability.
    	    	HalTimerWait(500);

           		// process AVIF
           	    TX_DEBUG_PRINT(("Drv: BIT_INTR7_CP_CLR_MUTE %s\n",stateFlags.upStreamMuted?"upstream mute":"already unmuted"));
           	    SiiMhlTxDrvReadIncomingInfoFrame();

                //if (stateFlags.upStreamMuted)
                {
                // if it is time yet, this will restart HDCP
                    SiiMhlTxNotifyUpstreamUnMute();
                    stateFlags.upStreamMuted = 0;
                }
            }
        }
    }
}


void SiiMhlTxDrvGetVendorSpecificInfoFrame(PVendorSpecificInfoFrame_t pVendorSpecificInfoFrame)
{
    SiiRegModify(REG_RX_HDMI_CTRL2
       ,BIT_RX_HDMI_CTRL2_VSI_MON_SEL_MASK
       ,BIT_RX_HDMI_CTRL2_VSI_MON_SEL_VS_INFOFRAME
       );
   SiiRegReadBlock(REG_RX_HDMI_MON_PKT_HEADER1,(uint8_t *)pVendorSpecificInfoFrame,sizeof(*pVendorSpecificInfoFrame));
}
//-------------------------------------------------------------------------------------------------
//! @brief      Interrupt Service Routine for INTR 8
//!
//! @param[in]  None
//! @retval     None
//-------------------------------------------------------------------------------------------------
void Int8Isr(void)
{
uint8_t statusIntr8;
    statusIntr8 = SiiRegRead(REG_INTR8);
    if (statusIntr8)
    {
        // clear all interrupts
        SiiRegWrite(REG_INTR8,statusIntr8);

// 		TX_DEBUG_PRINT(("\n-----------------> GOT statusIntr8 = %02X\n\n", (int)statusIntr8));
        if (BIT_INTR8_CEA_NEW_AVI & statusIntr8)
        {
            if (!upStreamHPD)
            {
          		// process avif at AVUNMUTE interrupt as well
                //  component layer will take care of timing issues.
                ERROR_DEBUG_PRINT(("Drv: UNEXPECTED BIT_INTR8_CEA_NEW_AVI\n"));

                //SiiMhlTxDrvAcquireUpstreamHPDControlDriveHigh(); // SWWA flaky HW
                //SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow();  // SWWA flaky HW
            }
            if(upStreamHPD)
            {
          		// process avif at AVUNMUTE interrupt as well
                //  component layer will take care of timing issues.
                TX_DEBUG_PRINT(("Drv: BIT_INTR8_CEA_NEW_AVI\n"));

                SiiMhlTxDrvReadIncomingInfoFrame();
            }
        }

        if (BIT_INTR8_CEA_NEW_VSI & statusIntr8)
        {
         	VendorSpecificInfoFrame_t vendorSpecificInfoFrame;
        //#define BITFIELD_TEST
        #ifdef BITFIELD_TEST //(
            {
            int i;
            extern void *memset  (void *s, char val, int n);
                memset(&vendorSpecificInfoFrame,0,sizeof(vendorSpecificInfoFrame));
                for(i = 0; i < 8;++i)
                {
                    vendorSpecificInfoFrame.pb4.HDMI_Video_Format=i;
                    vendorSpecificInfoFrame.pb5.ThreeDStructure.threeDStructure=i;
                    DumpIncomingInfoFrame(&vendorSpecificInfoFrame,sizeof(vendorSpecificInfoFrame));
                }
                TX_DEBUG_PRINT(("avi.checksum offset:%x\n",SII_OFFSETOF(InfoFrame_t,body.avi.checksum) ));
                TX_DEBUG_PRINT(("avi.byte_4 offset:%x\n",SII_OFFSETOF(InfoFrame_t,body.avi.byte_4) ));
                TX_DEBUG_PRINT(("vendorSpecific.checksum offset:%x\n",SII_OFFSETOF(InfoFrame_t,body.vendorSpecific.checksum) ));
                TX_DEBUG_PRINT(("vendorSpecific.pb4 offset:%x\n",SII_OFFSETOF(InfoFrame_t,body.vendorSpecific.pb4) ));
                TX_DEBUG_PRINT(("vendorSpecific.pb5 offset:%x\n",SII_OFFSETOF(InfoFrame_t,body.vendorSpecific.pb5) ));
                TX_DEBUG_PRINT(("vendorSpecific.pb6 offset:%x\n",SII_OFFSETOF(InfoFrame_t,body.vendorSpecific.pb6) ));
                TX_DEBUG_PRINT(("vendorSpecific.pb7 offset:%x\n",SII_OFFSETOF(InfoFrame_t,body.vendorSpecific.pb7) ));
            }
        #endif //)
            TX_DEBUG_PRINT(("Drv: NEW_VSI\n"));
            SiiMhlTxDrvGetVendorSpecificInfoFrame(&vendorSpecificInfoFrame);
            SiiMhlTxProcessVendorSpecificInfoFrameChange(&vendorSpecificInfoFrame);
        }
	}
}

//SII8240_VER81 +++
void Int9Isr(void)
{
uint8_t int9Status;
    int9Status = SiiRegRead(REG_INTR9);
    if (0xFF == int9Status)
    {
        ERROR_DEBUG_PRINT(("I2C failure!!!\n"));
    }
    else if (int9Status)
    {
        TX_DEBUG_PRINT(("Int9Isr: 0x%02x\n",(uint16_t)int9Status));
        SiiRegWrite(REG_INTR9,int9Status);
        if (BIT_INTR9_DEVCAP_DONE & int9Status)
        {
    		mscCmdInProgress = false;
            TX_DEBUG_PRINT(("Int9Isr:\n"));

            // Notify the component layer
            SiiMhlTxMscCommandDone(0);

        }
        if (BIT_INTR9_EDID_DONE & int9Status)
        {
            ERROR_DEBUG_PRINT(("Int9Isr: Expected to see this in-line\n"));
        }
    }
}

void SiiMhlTxDrvGetPeerDevCapRegs(PMHLDevCap_u pDevCap)
{

    TX_DEBUG_PRINT(("SiiMhlTxDrvGetPeerDevCapRegs:\n"));
    // choose devcap instead of EDID to appear at the FIFO
    SiiRegModify(REG_EDID_CTRL
            ,BIT_EDID_CTRL_DEVCAP_SELECT_MASK
            ,BIT_EDID_CTRL_DEVCAP_SELECT_DEVCAP
            );
    // set the starting address
	SiiRegWrite (REG_EDID_FIFO_ADDR, 0);	// 2E9 = Starting address of FIFO

    TX_DEBUG_PRINT(("SiiMhlTxDrvGetPeerDevCapRegs:\n"));

    // read the DEVCAP into the FIFO
	SiiRegReadBlock ( REG_EDID_FIFO_RD_DATA, pDevCap->aucDevCapCache, sizeof(*pDevCap));

}
//SII8240_VER81 ---
#ifdef DUMP_INTERRUPT_STATUS //(

void DumpInterruptStatus( void )
{
int i = 0;
char *szFormats[]=
{
      "\tREG_INTR_STATE             0x%02x\n"
    , "\tREG_INTR1                  0x%02x\n"
    , "\tREG_INTR2                  0x%02x\n"
    , "\tREG_INTR3                  0x%02x\n"
    , "\tREG_INTR4                  0x%02x\n"
    , "\tREG_INTR5                  0x%02x\n"
    , "\tREG_INTR7                  0x%02x\n"
    , "\tREG_INTR8                  0x%02x\n"
    , "\tREG_CBUS_INT_0             0x%02x\n"
    , "\tREG_CBUS_INT_1             0x%02x\n"
    , "\tREG_TPI_INTR_ST0_ENABLE    0x%02x\n"
    , "\tREG_TPI_INTR_ST0           0x%02x\n"
};
  
uint8_t intStatus[12];
    intStatus[i++] =SiiRegRead(REG_INTR_STATE          );
    intStatus[i++] =SiiRegRead(REG_INTR1               );
    intStatus[i++] =SiiRegRead(REG_INTR2               );
    intStatus[i++] =SiiRegRead(REG_INTR3               );
    intStatus[i++] =SiiRegRead(REG_INTR4               );
    intStatus[i++] =SiiRegRead(REG_INTR5               );
    intStatus[i++] =SiiRegRead(REG_INTR7               );
    intStatus[i++] =SiiRegRead(REG_INTR8               );
    intStatus[i++] =SiiRegRead(REG_CBUS_INT_0          );
    intStatus[i++] =SiiRegRead(REG_CBUS_INT_1          );
    intStatus[i++] =SiiRegRead(REG_TPI_INTR_ST0_ENABLE );
    intStatus[i++] =SiiRegRead(REG_TPI_INTR_ST0        );

    TX_DEBUG_PRINT(("Drv: REG_DISC_CTRL1:%02x\n",SiiRegRead(REG_DISC_CTRL1)));
#ifdef FPGA_DEBUG //(
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_1:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_1)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_2:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_2)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_3:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_3)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_4:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_4)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_5:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_5)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_6:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_6)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_7:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_7)));
#endif //)
    for ( i=0; i < sizeof(intStatus)/sizeof(intStatus[0]); ++i)
    {
        if (intStatus[i])
        {
            SiiOsDebugPrintAlwaysShort(szFormats[i],intStatus[i]);
        }
    }

}


void DumpInterruptStatus_clear( void )
{
int i = 0;
int reg_status;
char *szFormats[]=
{
      "\t##REG_INTR_STATE             0x%02x\n"
    , "\t##REG_INTR1                  0x%02x\n"
    , "\t##REG_INTR2                  0x%02x\n"
    , "\t##REG_INTR3                  0x%02x\n"
    , "\t##REG_INTR4                  0x%02x\n"
    , "\t##REG_INTR5                  0x%02x\n"
    , "\t##REG_INTR7                  0x%02x\n"
    , "\t##REG_INTR8                  0x%02x\n"
    , "\t##REG_CBUS_INT_0             0x%02x\n"
    , "\t##REG_CBUS_INT_1             0x%02x\n"
    , "\t##REG_TPI_INTR_ST0_ENABLE    0x%02x\n"
    , "\t##REG_TPI_INTR_ST0           0x%02x\n"
};

int Reg_array[12] = {
	REG_INTR_STATE,
	REG_INTR1,
	REG_INTR2,
	REG_INTR3,
	REG_INTR4,
	REG_INTR5,
	REG_INTR7,
	REG_INTR8,
	REG_CBUS_INT_0,
	REG_CBUS_INT_1,
	REG_TPI_INTR_ST0_ENABLE,
	REG_TPI_INTR_ST0
};
  
uint8_t intStatus[12];
    intStatus[i++] =SiiRegRead(REG_INTR_STATE          );
    intStatus[i++] =SiiRegRead(REG_INTR1               );
    intStatus[i++] =SiiRegRead(REG_INTR2               );
    intStatus[i++] =SiiRegRead(REG_INTR3               );
    intStatus[i++] =SiiRegRead(REG_INTR4               );
    intStatus[i++] =SiiRegRead(REG_INTR5               );
    intStatus[i++] =SiiRegRead(REG_INTR7               );
    intStatus[i++] =SiiRegRead(REG_INTR8               );
    intStatus[i++] =SiiRegRead(REG_CBUS_INT_0          );
    intStatus[i++] =SiiRegRead(REG_CBUS_INT_1          );
    intStatus[i++] =SiiRegRead(REG_TPI_INTR_ST0_ENABLE );
    intStatus[i++] =SiiRegRead(REG_TPI_INTR_ST0        );

    TX_DEBUG_PRINT(("(clear)Drv: REG_DISC_CTRL1:%02x\n",SiiRegRead(REG_DISC_CTRL1)));
#ifdef FPGA_DEBUG //(
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_1:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_1)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_2:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_2)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_3:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_3)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_4:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_4)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_5:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_5)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_6:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_6)));
    TX_DEBUG_PRINT(("Drv: REG_TPI_DEBUG_STATUS_7:%02x\n",SiiRegRead( REG_TPI_DEBUG_STATUS_7)));
#endif //)
    for ( i=0; i < sizeof(intStatus)/sizeof(intStatus[0]); ++i)
    {
        if (intStatus[i])
        {
            SiiOsDebugPrintAlwaysShort(szFormats[i],intStatus[i]);
		{
	      		SiiRegWrite(Reg_array[i], intStatus[i]); // clear all interrupts  	
		      reg_status = SiiRegRead(Reg_array[i]);  		      
                  SiiOsDebugPrintAlwaysShort(szFormats[i],reg_status);	      

		}
        }
    }

}




#endif //)

uint8_t SiiMhlTxDrvGetOverallInterruptStatus(uint8_t i)
{
uint8_t intMStatus;
i=i;
//ASUS_BSP +++ : for lock ISR in disconnecting progress
	if (g_TxDisconnect_lock)
	{
		TX_DEBUG_PRINT(("[MHL] SiiMhlTxDrvGetOverallInterruptStatus Disconnect lock\n"));
		return 0;
	}
//ASUS_BSP --- : for lock ISR in disconnecting progress

	#if 0	
   	intMStatus = SiiRegRead(REG_INTR_STATE);	// read status
   	if(0xFF == intMStatus)
   	{
   		intMStatus = 0;
   	    TX_DEBUG_PRINT(("\nDrv: EXITING ISR DUE TO intMStatus - 0xFF loop = [%02X] intMStatus = [%02X] \n\n", (int) i, (int)intMStatus));
           return 0;
   	}
//	else
//	{
//		printk("[MHL] Linux loop intMStatus = 0x%x", intMStatus);
//	}
	#else
//ASUS_BSP+++ use INT Low to determine exit ISR function to solve TX hang issue	
	      if (gpio_get_value(W_INT_GPIO))
	      	{
			intMStatus = 0;
			TX_DEBUG_PRINT(("[MHL] Linux loop INT high return \n"));
	      	}	
		else
		{
			if  (g_mhl_i2c_error_not_connected_count > MAX_MHL_I2C_ERROR_COUNT)
			{
				intMStatus = 0;
				printk("[MHL] Linux loop INT, but i2c error over max timers\n");
				MHL_disable_irq();
			}
			else
			{		
				intMStatus = 1;
				
				if (g_pad_tv_mode == MHL_TV_MODE)
                	msleep(100); 
					
				//TX_DEBUG_PRINT(("[MHL] Linux loop INT low continue ..... \n"));		
			}
		}
//ASUS_BSP--- use INT Low to determine exit ISR function to solve TX hang issue			
	#endif
	
    return intMStatus & 0x01;
}
//SII8240_VER75 +++

#ifdef	APPLY_PLL_RECOVERY
///////////////////////////////////////////////////////////////////////////
// FUNCTION:	ApplyPllRecovery
//
// PURPOSE:		This function helps recover PLL.
//
///////////////////////////////////////////////////////////////////////////
static void ApplyPllRecovery ( void  )
{
    if (!(mfdrvTranscodeMode & g_modeFlags))
    {
    void HDCP_Restart (void);
    uint8_t ucPwdReset,temp;
    #ifdef PIN_PROBE_AS_PLL_RECOVERY //(
    	pinProbe = 1;
    #endif //)
        ucPwdReset = SiiRegRead(REG_PWD_SRST);


    	// MHL FIFO Reset here
        temp = (ucPwdReset & ~(BIT_PWD_MHLFIFO_RST_MASK)) | (BIT_PWD_MHLFIFO_RST_MASK & BIT_PWD_MHLFIFO_RST_reset);
    	SiiRegWrite(REG_PWD_SRST, temp);

        // clear these two interrupts
        SiiRegWrite(REG_INTR5, (BIT_INTR5_MHL_FIFO_OVERFLOW | BIT_INTR5_MHL_FIFO_UNDERFLOW));

        HDCP_Restart();

    	// followed by a 10ms settle time
    	HalTimerWait(10);

        // bring MHL FIFO out of reset
        temp = (ucPwdReset & ~(BIT_PWD_MHLFIFO_RST_MASK)) | (BIT_PWD_MHLFIFO_RST_MASK & BIT_PWD_MHLFIFO_RST_normal);
    	SiiRegWrite(REG_PWD_SRST, temp ); 	 // MHL Auto FIFO Reset

        if (PlatformGPIOGet(pinDoHdcp) == false)
       	{
            SiiMhlTxDrvVideoUnmute();
        }
    #ifdef PIN_PROBE_AS_PLL_RECOVERY //(
    	pinProbe = 0;
    #endif //)

    	TX_DEBUG_PRINT(("Drv: Applied PLL Recovery\n"));

    }


}

#endif // APPLY_PLL_RECOVERY
//SII8240_VER75 ---
///////////////////////////////////////////////////////////////////////////////
// SiiMhlTxDeviceIsr
//
// This function must be called from a master interrupt handler or any polling
// loop in the host software if during initialization call the parameter
// interruptDriven was set to true. SiiMhlTxGetEvents will not look at these
// events assuming firmware is operating in interrupt driven mode. MhlTx component
// performs a check of all its internal status registers to see if a hardware event
// such as connection or disconnection has happened or an RCP message has been
// received from the connected device. Due to the interruptDriven being true,
// MhlTx code will ensure concurrency by asking the host software and hardware to
// disable interrupts and restore when completed. Device interrupts are cleared by
// the MhlTx component before returning back to the caller. Any handling of
// programmable interrupt controller logic if present in the host will have to
// be done by the caller after this function returns back.

// This function has no parameters and returns nothing.
//
// This is the master interrupt handler for 9244. It calls sub handlers
// of interest. Still couple of status would be required to be picked up
// in the monitoring routine (Sii9244TimerIsr)
//
// To react in least amount of time hook up this ISR to processor's
// interrupt mechanism.
//
// Just in case environment does not provide this, set a flag so we
// call this from our monitor (Sii9244TimerIsr) in periodic fashion.
//
// Device Interrupts we would look at
//		RGND		= to wake up from D3
//		MHL_EST 	= connection establishment
//		CBUS_LOCKOUT= Service USB switch
//		CBUS 		= responder to peer messages
//					  Especially for DCAP etc time based events
//
#ifdef linux_interrput_handle
#define EXHAUST_PENDING_INTERRUPTS_LINUX //hammer modify for match Asus use linux-driver 0.63
#endif

extern int check_mdp4_dtv(void);
extern bool_t MHL_Check_QcomTmdsHVTotal(void);


void SiiMhlTxDeviceIsr (void)
{
#ifdef EXHAUST_PENDING_INTERRUPTS_LINUX //(
	uint8_t i; //count the number of times that another interrupt is still pending
                // at the end of interrupt processing.
	//
	// Look at discovery interrupts if not yet connected.
	//
	i=0;

//   	TX_DEBUG_PRINT(("\n## SiiMhlTxDeviceIsr +++\n"));
	
	do
	{
#endif //)

	//SII_MHL_TX_DUMP_INFO
//ASUS_BSP +++ : for lock ISR in disconnecting progress	
	if (g_TxDisconnect_lock)
	{
		TX_DEBUG_PRINT(("[MHL] SiiMhlTxDeviceIsr Disconnect lock\n"));
		return;
	}
//ASUS_BSP --- : for lock ISR in disconnecting progress	
    	if( POWER_STATE_D0_MHL != fwPowerState )
    	{
    		//
    		// Check important RGND, MHL_EST, CBUS_LOCKOUT and SCDT interrupts
    		// During D3 we only get RGND but same ISR can work for both states
    		//
    		if (I2C_INACCESSIBLE == Int4Isr())
    		{
                return; // don't do any more I2C traffic until the next interrupt.
    		}
    	}
    	else if( POWER_STATE_D0_MHL == fwPowerState )
    	{

    		if (I2C_INACCESSIBLE == Int4Isr())
    		{
    			return; // don't do any more I2C traffic until the next interrupt.
    		}
    		// it is no longer necessary to check if the call to Int4Isr()
    		//  put us into D3 mode, since we now return immediately in that case

//SII8240_VER81 +++
  			// Check for any peer messages for DCAP_CHG etc
  			// Dispatch to have the CBUS module working only once connected.

            //  Check CBUS first for performance and downstream HPD reasons
            MhlCbusIsr();
//SII8240_VER81 ---

            if (mfdrvTranscodeMode & g_modeFlags)
            {
      			Int5Isr();
            }
            else
            {
//MHL_CTS 20120907+++		
	
uint8_t H_RES_L; // 0x6A
uint8_t H_RES_H; //0x6B
uint8_t V_RES_L;  // 0x6C
uint8_t V_RES_H;  //0x6D
uint8_t HDMI_MON_PKT_HEADER1;
uint8_t HDMI_MON_PKT_HEADER2;
//MHL_CTS 20120907---
            
//SII8240_VER75 +++			
      			Int5Isr();				
//MHL_CTS 20120907+++				
//SII8240 +++ Keno : filter HDCP while QCOM TMDS not ready

		if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
		{
//ASUS_BSP +++: larry lai : for by-pass check QCOM TMDS H V total while booting in PAD mode
			if (MHL_Check_QcomTmdsHVTotal())
			{
				if (check_mdp4_dtv())
				{		
					H_RES_L = SiiRegRead(REG_TPI_HW_6A); // 0x6A
					H_RES_H = SiiRegRead(REG_TPI_HW_6B); //0x6B
					V_RES_L = SiiRegRead(REG_TPI_HW_6C);  // 0x6C
					V_RES_H = SiiRegRead(REG_TPI_HW_6D);  //0x6D								   
					
				    	if ( ((H_RES_L == 0xa0) && (H_RES_H == 0x05) 
						&& (V_RES_L  == 0x37) && (V_RES_H== 0x03)) ||   // 1280x800		

						((H_RES_L == 0x98) && (H_RES_H == 0x08) 
						&& (V_RES_L  == 0x65) && (V_RES_H== 0x04))    // 1080p60   1125x2200						
						)  
			    		{
						if ((SiiRegRead(REG_RX_HDMI_MON_PKT_HEADER1) == 0x82) && 
						    (SiiRegRead(REG_RX_HDMI_MON_PKT_HEADER1+1) == 0x02))
						{
			 				TpiIsr();		
						}
			    		}
					else
					{
						TX_DEBUG_PRINT(("[Pad mode Not Ready] ## REG_TPI_HW_6A = 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", 
								H_RES_L, H_RES_H, V_RES_L, V_RES_H));			
					}						
				}			
			}
			else
			{
				TpiIsr();		
			}
//ASUS_BSP ---: larry lai : for by-pass check QCOM TMDS H V total while booting in PAD mode			
		}
		else
		{


#ifdef CONFIG_HAS_EARLYSUSPEND	
//if hdcp on, then stand-by , tmds off will re-trigger TpiIsr
      	    		if (g_mhl_early_suspend_flag)	
      	    		{
				//printk("MHL_ISR_SKIP_INT9ISR\n");      	    		
      	    			goto MHL_ISR_SKIP_INT9ISR;
      	    		}
#endif						

			if (!MHL_Check_QcomTmdsHVTotal())
			{
      	    			goto MHL_ISR_SKIP_INT9ISR;			
			}

			if (check_mdp4_dtv())
			{
			
			H_RES_L = SiiRegRead(REG_TPI_HW_6A); // 0x6A
			H_RES_H = SiiRegRead(REG_TPI_HW_6B); //0x6B
			V_RES_L = SiiRegRead(REG_TPI_HW_6C);  // 0x6C
			V_RES_H = SiiRegRead(REG_TPI_HW_6D);  //0x6D

		    if  ((H_RES_L == 0x5a) && (H_RES_H == 0x03) 
				&& (V_RES_L  == 0x0d) && (V_RES_H== 0x02))   // 480p
		    	{
			    SiiRegWrite(REG_MHLTX_CTL7, 0x03);			    		
		    	}
			else
			{
			    SiiRegWrite(REG_MHLTX_CTL7, 0x09);			    					
			}

#ifndef KENO_DONGLE_DOWNSTREAM1//KH, skip checking H/V Total, just check AVI InfoFrame is fine.
		    if ( ((H_RES_L == 0x5a) && (H_RES_H == 0x03) 
				&& (V_RES_L  == 0x0d) && (V_RES_H== 0x02)) ||   // 480p
				
				((H_RES_L == 0x72) && (H_RES_H == 0x06) 
				&& (V_RES_L  == 0xee) && (V_RES_H== 0x02)) ||	// 720p	
				
				((H_RES_L == 0xbc) && (H_RES_H == 0x07) 
				&& (V_RES_L  == 0xee) && (V_RES_H== 0x02)) ||	// 720p50
			
				((H_RES_L == 0xbe) && (H_RES_H == 0x0a) 
				&& (V_RES_L  == 0x65) && (V_RES_H== 0x04))  ||   // 1080p24   1125x2750
				
				((H_RES_L == 0x50) && (H_RES_H == 0x0a) 
				&& (V_RES_L  == 0x65) && (V_RES_H== 0x04))  ||   // 1080p25   1125x2640
				
				((H_RES_L == 0x98) && (H_RES_H == 0x08) 
				&& (V_RES_L  == 0x65) && (V_RES_H== 0x04))    // 1080p30/60   1125x2200
				)	
#endif
	         	{

			HDMI_MON_PKT_HEADER1 = SiiRegRead(REG_RX_HDMI_MON_PKT_HEADER1); // 0xB8
			HDMI_MON_PKT_HEADER2 = SiiRegRead(REG_RX_HDMI_MON_PKT_HEADER1+1); //0xB9

//    printk("HDMI_MON_PKT_HEADER1 : 0x%02x\n",HDMI_MON_PKT_HEADER1);
//    printk("HDMI_MON_PKT_HEADER2 : 0x%02x\n",HDMI_MON_PKT_HEADER2);
	         		
				if ((HDMI_MON_PKT_HEADER1 == 0x82) && 
				    (HDMI_MON_PKT_HEADER2 == 0x02) )
					{
					
		  			    TpiIsr();
					}
					else
					{
						TX_DEBUG_PRINT(("[Not Ready] HDMI_MON_PKT_HEADER1 : 0x%02x, 0x%02x\n",HDMI_MON_PKT_HEADER1, HDMI_MON_PKT_HEADER2));
//						msleep(50);					
					}
				}
#ifndef KENO_DONGLE_DOWNSTREAM1//KH, skip checking H/V Total, just check AVI InfoFrame is fine.
				else
				{
					TX_DEBUG_PRINT(("[Not Ready] ## REG_TPI_HW_6A = 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", 
							H_RES_L, H_RES_H, V_RES_L, V_RES_H));			
//					msleep(50);					
			}
#endif				
			}
		}
//MHL_CTS 20120907---			   
//SII8240 --- Keno : filter HDCP while QCOM TMDS not ready			
//SII8240_VER75 ---				
MHL_ISR_SKIP_INT9ISR:
                Int7Isr();
                Int8Isr();
            }

      		Int1Isr();
//SII8240_VER81 +++			
            Int9Isr();
//SII8240_VER81 ---			
    	}

    	if( POWER_STATE_D3 != fwPowerState )
    	{
    		// Call back into the MHL component to give it a chance to
    		// take care of any message processing caused by this interrupt.
    		MhlTxProcessEvents();
    	}

#ifdef EXHAUST_PENDING_INTERRUPTS_LINUX //(
		i++;

	}while (SiiMhlTxDrvGetOverallInterruptStatus(i));
	
#endif //)
//   	TX_DEBUG_PRINT(("\n## SiiMhlTxDeviceIsr ---\n"));     
}



//-------------------------------------------------------------------------------------------------
//! @brief      DriverAPI function to mute and unmute video
//!
//! @param[in]  mute - non zero will mute the video else unmute.
//! @retval     None
//-------------------------------------------------------------------------------------------------
#ifdef	DEBUG_RI_VALUES
extern	int	g_ri_display;
#endif	// DEBUG_RI_VALUES

void	SiiMhlTxDrvVideoMute( void )
{
#ifdef	DEBUG_RI_VALUES
		g_ri_display = false;
#endif	// DEBUG_RI_VALUES

	// Print always
	HDCP_DEBUG_PRINT(("SiiDrvVideoMute: AV muted\n"));
    ModifyTpiSystemControlDataReg(AV_MUTE_MASK,AV_MUTE_MUTED)
}

//SII8240_VER81 +++
void	SiiMhlTxDrvVideoUnmute( void )
{
#ifdef	DEBUG_RI_VALUES
	g_ri_display = true;
#endif	// DEBUG_RI_VALUES

	HDCP_DEBUG_PRINT(("AV unmuted.\n"));
    // Change register
    ModifyTpiSystemControlDataReg(AV_MUTE_MASK,AV_MUTE_NORMAL)
}


void	SiiToggleTmdsForHdcpAuthentication(void)
{
#ifdef DEFERRED_HDCP_START //(
    if (mscCmdInProgress)
    {
        TX_DEBUG_PRINT(("deferring HDCP start...\n"));
        hdcpStartPending = true;
    }
    else
    {
#endif //(
        // a three step process is mandatory
        ModifyTpiSystemControlDataReg(AV_MUTE_MASK,AV_MUTE_MUTED)
        ModifyTpiSystemControlDataReg(TMDS_OUTPUT_CONTROL_MASK,TMDS_OUTPUT_CONTROL_POWER_DOWN)
        ModifyTpiSystemControlDataReg(TMDS_OUTPUT_CONTROL_MASK,TMDS_OUTPUT_CONTROL_ACTIVE)
#ifdef DEFERRED_HDCP_START //(
    }
#endif //)
}
//SII8240_VER81 ---

void	SiiMhlTxDrvSetOutputType( void )
{
	// select output mode (HDMI/DVI) according to what chip register says
	// isHdmi =  (CONNECTOR_TYPE_HDMI == SiiRegRead(TPI_HDCP_QUERY_DATA_REG) & CONNECTOR_TYPE_MASK;
//SII8240_VER81 +++
// VER81 move code to SiiMhlTxDrvSetHdmiMode()
//SII8240_VER81 ---
	
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvTmdsControl
//
// Control the TMDS output. MhlTx uses this to support RAP content on and off.
//
void SiiMhlTxDrvTmdsControl (bool_t enable)
{

	if( enable )
	{
        SiiRegModify(REG_MHLTX_CTL2,BIT_MHL_TX_CTL2_TX_OE_MASK ,0x3C); //2012-04-17-1600 PDT
        SiiRegModify(REG_MHLTX_CTL8,BIT_MHLTX_CTL8_PLL_BW_CTL_MASK,0x3); //2012-04-17-1600 PDT

//SII8240_VER75 +++
        if ( mfdrvTranscodeMode & g_modeFlags )
        {
    		SET_BIT(REG_TMDS_CCTRL, 4);
    	    TX_DEBUG_PRINT(("Drv: TMDS Output Enabled\n"));
            SiiMhlTxDrvReleaseUpstreamHPDControl();  // this triggers an EDID read

        }
        else
//SII8240_VER75 ---		
        {
		//
		// Now control the AV Mute and toggle TMDS_OE to kick start HDCP authentication
		//
        if (PlatformGPIOGet(pinDoHdcp) == false)
   		{
	        TX_DEBUG_PRINT(("SiiMhlTxDrvTmdsControl: TMDS Output Enabled (Video UnMuted, DS does not support HDCP)\n"));
//SII8240_VER75 +++
                // must set UNMUTE and output enable in same write to avoid starting HDCP
                ModifyTpiSystemControlDataReg(
                              (TMDS_OUTPUT_CONTROL_MASK   |  AV_MUTE_MASK)
                            , (TMDS_OUTPUT_CONTROL_ACTIVE | AV_MUTE_NORMAL)
                            )
//MHL_CTS 20120918+++, for MHL CTS Test
		if  (g_pad_tv_mode == MHL_TV_MODE)
		{
			//keno20120903  // workaround for try to re-enable AVI Inforframe    
	   		SiiRegWrite(REG_PKT_FILTER_0, 0xA1);
		}
//MHL_CTS 20120918---

                SiiMhlTxNotifyAuthentication();
//SII8240_VER75 ---
        }
        else	// Downstream supports HDCP. Toggle TMDS to kick off HDCP Authentication
        {
        uint8_t tmds_oe;
//SII8240_VER75		
			tmds_oe = cache_TPI_SYSTEM_CONTROL_DATA_REG;
	        TX_DEBUG_PRINT(("SiiMhlTxDrvTmdsControl: Examining TMDS_OE:%02x\n",(uint16_t) tmds_oe));
            if (TMDS_OUTPUT_CONTROL_POWER_DOWN == (TMDS_OUTPUT_CONTROL_MASK & tmds_oe))
            {

    	        TX_DEBUG_PRINT(("SiiMhlTxDrvTmdsControl: TMDS_OE indicated power down\n"));
    			SiiToggleTmdsForHdcpAuthentication();
            }
            else
            {
                uint8_t currentStatus,linkStatus;
                    currentStatus = SiiRegRead(TPI_HDCP_QUERY_DATA_REG);
                    linkStatus = LINK_STATUS_MASK & currentStatus;
        	        TX_DEBUG_PRINT(("SiiMhlTxDrvTmdsControl: Examining TPI_HDCP_QUERY_DATA_REG: 0x%02x\n",(uint16_t)currentStatus));

                    switch(linkStatus)
                    {
                    case LINK_STATUS_NORMAL:
                        if ( AV_MUTE_MUTED & tmds_oe)
                        {
#ifdef KENO_DONGLE_DOWNSTREAM1                        		
				  if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
				  {
                            	SiiMhlTxDrvVideoUnmute();
				  }
#else
                         	 SiiMhlTxDrvVideoUnmute();
#endif
                        }
                        break;
                    case LINK_STATUS_LINK_LOST:
                        break;
                    case LINK_STATUS_RENEGOTIATION_REQ:
                        break;
                    case LINK_STATUS_LINK_SUSPENDED:
                        break;
                    }
                }
            }
        }
	}
	else
	{

        SiiRegModify(REG_MHLTX_CTL2,BIT_MHL_TX_CTL2_TX_OE_MASK ,0x0); //2012-04-17-1600 PDT
        SiiRegModify(REG_MHLTX_CTL8,BIT_MHLTX_CTL8_PLL_BW_CTL_MASK,0x3); //2012-04-17-1600 PDT

//SII8240_VER75	+++
        if ( mfdrvTranscodeMode & g_modeFlags )
        {
    		CLR_BIT(REG_TMDS_CCTRL, 4);
    	    TX_DEBUG_PRINT(("Drv: TMDS Ouput Disabled\n"));
        }
        else
        {
		// Disable HDCP
		SiiHdmiTxLiteDisableEncryption();
	    TX_DEBUG_PRINT(("SiiMhlTxDrvTmdsControl: Turn off TMDS OE, AV mute and Disable HDCP\n"));
            ModifyTpiSystemControlDataReg(
                  (TMDS_OUTPUT_CONTROL_MASK          | AV_MUTE_MASK)
    		    , (TMDS_OUTPUT_CONTROL_POWER_DOWN    | AV_MUTE_MUTED)
    		    )
        }
//SII8240_VER75	---		
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvNotifyEdidChange
//
// MhlTx may need to inform upstream device of an EDID change. This can be
// achieved by toggling the HDMI HPD signal or by simply calling EDID read
// function.
//
void SiiMhlTxDrvNotifyEdidChange (void)
{
    TX_DEBUG_PRINT(("Drv: SiiMhlTxDrvNotifyEdidChange\n"));
    if ( mfdrvTranscodeMode & g_modeFlags )
    {
    	//
    	// Prepare to toggle HPD to upstream
    	//
        SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow();

    	// wait a bit
    	HalTimerWait(110);

    	// drive HPD back to high by reg_hpd_out_ovr_val = HIGH
    	SiiRegModify(REG_HPD_CTRL
	        , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_MASK
	        , BIT_HPD_CTRL_HPD_OUT_OVR_VAL_ON
            );

        // release control to allow transcoder to modulate for CLR_HPD and SET_HPD
        SiiMhlTxDrvReleaseUpstreamHPDControl();
    }
}
//------------------------------------------------------------------------------
// Function:    SiiMhlTxDrvSendCbusCommand
//
// Write the specified Sideband Channel command to the CBUS.
// Command can be a MSC_MSG command (RCP/RAP/RCPK/RCPE/RAPK), or another command
// such as READ_DEVCAP, SET_INT, WRITE_STAT, etc.
//
// Parameters:
//              pReq    - Pointer to a cbus_req_t structure containing the
//                        command to write
// Returns:     true    - successful write
//              false   - write failed
//------------------------------------------------------------------------------

bool_t SiiMhlTxDrvSendCbusCommand (cbus_req_t *pReq)
{
    bool_t  success = true;
//SII8240_VER81 +++
#if 1 //(

	//
	// If not connected, return with error
	//
	if( (POWER_STATE_D0_MHL != fwPowerState ) || (mscCmdInProgress))
	{
	    TX_DEBUG_PRINT(("Error: Drv: fwPowerState: %02X, or CBUS(0x0A):%02X mscCmdInProgress = %d\n",
			(int) fwPowerState,
			(int) SiiRegRead(REG_CBUS_STATUS),
			(int) mscCmdInProgress));

   		return false;
	}
	// Now we are getting busy
	mscCmdInProgress	= true;


    switch ( pReq->command )
    {
		case MHL_SET_INT:
            CBUS_DEBUG_PRINT(("Drv: Sending SET_INT command %02X, %02X, %02X\n"
                , (int)pReq->command
                , (int)(pReq->offsetData)
    			, (int)pReq->payload_u.msgData[0]
    		 	));
        	SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->offsetData); 	// set offset
        	SiiRegWrite(REG_CBUS_MSC_1ST_TRANSMIT_DATA, pReq->payload_u.msgData[0]);
            SiiRegWrite(REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
			break;

        case MHL_WRITE_STAT:
            CBUS_DEBUG_PRINT(("Drv: Sending WRITE_STATE command %02X, %02X, %02X\n"
                , (int)pReq->command
                , (int)(pReq->offsetData)
    			, (int)pReq->payload_u.msgData[0]
    		 	));
        	SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->offsetData); 	// set offset
        	SiiRegWrite(REG_CBUS_MSC_1ST_TRANSMIT_DATA, pReq->payload_u.msgData[0]);
            SiiRegWrite(REG_CBUS_MSC_COMMAND_START,  BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);
            break;

        case MHL_READ_DEVCAP:
            CBUS_DEBUG_PRINT(("Reading DEVCAP\n"));
            // read the entire DEVCAP array in one command (new in 8240)
            SiiRegWrite(REG_TPI_CBUS_START,BIT_TPI_CBUS_START_DEVCAP_READ_START);
            break;


 		case MHL_GET_STATE:			// 0x62 -
		case MHL_GET_VENDOR_ID:		// 0x63 - for vendor id
		case MHL_SET_HPD:			// 0x64	- Set Hot Plug Detect in follower
		case MHL_CLR_HPD:			// 0x65	- Clear Hot Plug Detect in follower
		case MHL_GET_SC1_ERRORCODE:		// 0x69	- Get channel 1 command error code
		case MHL_GET_DDC_ERRORCODE:		// 0x6A	- Get DDC channel command error code.
		case MHL_GET_MSC_ERRORCODE:		// 0x6B	- Get MSC command error code.
		case MHL_GET_SC3_ERRORCODE:		// 0x6D	- Get channel 3 command error code.
            CBUS_DEBUG_PRINT(("Drv: Sending misc. command %02X, %02X\n"
                , (int)pReq->command
    			, (int)pReq->payload_u.msgData[0]
    		 	));
			SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->command );
        	SiiRegWrite(REG_CBUS_MSC_1ST_TRANSMIT_DATA, pReq->payload_u.msgData[0]);
            SiiRegWrite(REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_PEER_CMD);
            break;

        case MHL_MSC_MSG:
            CBUS_DEBUG_PRINT(("Drv: Sending MSC_MSG command %02X, %02X, %02X\n"
                , (int)pReq->command
    			, (int)pReq->payload_u.msgData[0]
    		 	, (int)pReq->payload_u.msgData[1]
    		 	));
			SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->command );
        	SiiRegWrite(REG_CBUS_MSC_1ST_TRANSMIT_DATA, pReq->payload_u.msgData[0]);
			SiiRegWrite(REG_CBUS_MSC_2ND_TRANSMIT_DATA, pReq->payload_u.msgData[1]);
            SiiRegWrite(REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_MSG);
            break;

        case MHL_WRITE_BURST:

            // Now copy all bytes from array to local scratchpad
            if (NULL == pReq->payload_u.pdatabytes)
            {
                TX_DEBUG_PRINT(("\nDrv: Put pointer to WRITE_BURST data in req.pdatabytes!!!\n\n"));
                success = false;
            }
            else
            {
	        uint8_t *pData = pReq->payload_u.pdatabytes;
                CBUS_DEBUG_PRINT(("Drv: Sending WRITE_BURST command %02X, %02X, %02\n"
                    , (int)pReq->offsetData
        			, (int)pReq->payload_u.msgData[0]
        		 	, (int)pReq->length -1
        		 	));
            	SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->offsetData); 	// set offset
            	SiiRegWrite(REG_CBUS_MSC_1ST_TRANSMIT_DATA, pReq->payload_u.msgData[0]);
                SiiRegWrite(REG_CBUS_MSC_WRITE_BURST_DATA_LEN, pReq->length -1 );
                TX_DEBUG_PRINT(("\nDrv: Writing data into scratchpad\n\n"));
                SiiRegWriteBlock(REG_CBUS_WB_XMIT_DATA_0,  pData,pReq->length);
                SiiRegWrite(REG_CBUS_MSC_COMMAND_START, BIT_CBUS_MSC_WRITE_BURST);
			}
            break;

        default:
            success = false;
            break;
    }
    return( success );
#else //)(
//SII8240_VER81 ---
    uint8_t startbit;

	//
	// If not connected, return with error
	//
	if( (POWER_STATE_D0_MHL != fwPowerState ) || (mscCmdInProgress))
	{
	    TX_DEBUG_PRINT(("Error: Drv: fwPowerState: %02X, or CBUS(0x0A):%02X mscCmdInProgress = %d\n",
			(int) fwPowerState,
			(int) SiiRegRead(REG_CBUS_STATUS),
			(int) mscCmdInProgress));

   		return false;
	}
	// Now we are getting busy
	mscCmdInProgress	= true;

    CBUS_DEBUG_PRINT(("Drv: Sending MSC command %02X, %02X, %02X, %02X\n",
			(int)pReq->command,
			(int)(pReq->offsetData),
		 	(int)pReq->payload_u.msgData[0],
		 	(int)pReq->payload_u.msgData[1]));

    /****************************************************************************************/
    /* Setup for the command - write appropriate registers and determine the correct        */
    /*                         start bit.                                                   */
    /****************************************************************************************/

	// Set the offset and outgoing data byte right away
	SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->offsetData); 	// set offset
	SiiRegWrite(REG_CBUS_MSC_1ST_TRANSMIT_DATA, pReq->payload_u.msgData[0]);

    startbit = 0x00;
    switch ( pReq->command )
    {
		case MHL_SET_INT:
			startbit = BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT;
			break;

        case MHL_WRITE_STAT:
            startbit = BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT;
            break;

        case MHL_READ_DEVCAP:	// Read one device capability register = 0x61

            startbit = BIT_CBUS_MSC_READ_DEVCAP;
            break;

 		case MHL_GET_STATE:			// 0x62 -
		case MHL_GET_VENDOR_ID:		// 0x63 - for vendor id
		case MHL_SET_HPD:			// 0x64	- Set Hot Plug Detect in follower
		case MHL_CLR_HPD:			// 0x65	- Clear Hot Plug Detect in follower
		case MHL_GET_SC1_ERRORCODE:		// 0x69	- Get channel 1 command error code
		case MHL_GET_DDC_ERRORCODE:		// 0x6A	- Get DDC channel command error code.
		case MHL_GET_MSC_ERRORCODE:		// 0x6B	- Get MSC command error code.
		case MHL_GET_SC3_ERRORCODE:		// 0x6D	- Get channel 3 command error code.
			SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->command );
            startbit = BIT_CBUS_MSC_PEER_CMD;
            break;

        case MHL_MSC_MSG:
			SiiRegWrite(REG_CBUS_MSC_2ND_TRANSMIT_DATA, pReq->payload_u.msgData[1]);
			SiiRegWrite(REG_CBUS_MSC_CMD_OR_OFFSET, pReq->command );
            startbit = BIT_CBUS_MSC_MSG;
            break;

        case MHL_WRITE_BURST:
            SiiRegWrite(REG_CBUS_MSC_WRITE_BURST_DATA_LEN, pReq->length -1 );

            // Now copy all bytes from array to local scratchpad
            if (NULL == pReq->payload_u.pdatabytes)
            {
                TX_DEBUG_PRINT(("\nDrv: Put pointer to WRITE_BURST data in req.pdatabytes!!!\n\n"));
                success = false;
            }
            else
            {
	            uint8_t *pData = pReq->payload_u.pdatabytes;
                TX_DEBUG_PRINT(("\nDrv: Writing data into scratchpad\n\n"));
                SiiRegWriteBlock(REG_CBUS_WB_XMIT_DATA_0,  pData,pReq->length);
                startbit = BIT_CBUS_MSC_WRITE_BURST;
			}
            break;

        default:
            success = false;
            break;
    }

    /****************************************************************************************/
    /* Trigger the CBUS command transfer using the determined start bit.                    */
    /****************************************************************************************/

    if ( success )
    {
        SiiRegWrite(REG_CBUS_MSC_COMMAND_START, startbit );
    }
    else
    {
        TX_DEBUG_PRINT(("\nDrv: SiiMhlTxDrvSendCbusCommand failed\n\n"));
    }

    return( success );
#endif //)
}

bool_t SiiMhlTxDrvCBusBusy (void)
{
    return mscCmdInProgress ? true : false;
}

///////////////////////////////////////////////////////////////////////////
// WriteInitialRegisterValuesPartOne
//
//
///////////////////////////////////////////////////////////////////////////

static void WriteInitialRegisterValuesPartOne (void)
{
	TX_DEBUG_PRINT(("Drv: WriteInitialRegisterValuesPartOne\n"));
	// Power Up
    #if (TARGET_DEVICE_ID == BISHOP_DEVICE_ID) //)(
    {
    	SiiRegWrite(REG_DPD, 0x7D);
    }
    #elif (TARGET_DEVICE_ID == WOLV60_DEVICE_ID) //)(
    {

        SiiRegWrite(REG_DPD, 0x75);			// Power up CVCC 1.2V core
    }
    #else //)(
     this should not compile
    #endif //)
}
///////////////////////////////////////////////////////////////////////////
// WriteInitialRegisterValuesPartTwo
//
//
///////////////////////////////////////////////////////////////////////////
//#define VD_EMI_TEST
static void WriteInitialRegisterValuesPartTwo (void)
{
	TX_DEBUG_PRINT(("Drv: WriteInitialRegisterValuesPartTwo ++\n"));
//SII8240_VER75	+++
    if (mfdrvTranscodeMode & g_modeFlags)
    {
        InitTranscodeMode();
        // Don't force HPD to 0 during wake-up from D3
    	if (fwPowerState != TX_POWER_STATE_D3)
    	{
            //SiiRegModify(REG_DPD,BIT_DPD_PDIDCK_MASK,BIT_DPD_PDIDCK_POWER_DOWN);
            SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow(); // Force HPD to 0 when not in MHL mode.
    	}

    	if (g_chipDeviceId == BISHOP_DEVICE_ID)
    	{
       		SiiRegWrite(REG_DCTL, BIT_DCTL_EXT_DDC_SEL | BIT_DCTL_TRANSCODE_ON  | BIT_DCTL_TLCK_PHASE_INVERTED );
    	}
        // 8240 has correct default falue for REG_DCTL
        // wake pulses necessary in transcode mode
        // No OTG, Discovery pulse proceed, Wake pulse not bypassed
        SiiRegWrite(REG_DISC_CTRL9
                    , BIT_DC9_CBUS_LOW_TO_DISCONNECT
                    | BIT_DC9_WAKE_DRVFLT
                    | BIT_DC9_DISC_PULSE_PROCEED
    //bugzilla 24170                | BIT_DC9_VBUS_OUTPUT_CAPABILITY_SRC
                 );
#ifdef VD_EMI_TEST
	   if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
	   {
//EMI workaround for pad mode +++   	  
		    SiiRegModify(REG_MHLTX_CTL4
		        ,BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK
		            ,0x0
		            ); //2012-05-10-bugzilla 24790

		    SiiRegWrite(REG_MHLTX_CTL7, 0x00);	
//EMI workaround for pad mode ---		  
	   }
	   else
	   {
//for pass CTS eye diagram, optimize swing value	   
		    SiiRegModify(REG_MHLTX_CTL4
		        ,BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK
		            ,0x37
		            ); //2012-05-10-bugzilla 24790	   
	   }
#endif	   

    }
//SII8240_VER75	---	
    else
    {
    	SiiRegWrite(REG_TMDS_CLK_EN, 0x01);			// Enable TxPLL Clock
    	SiiRegWrite(REG_TMDS_CH_EN, 0x11);			// Enable Tx Clock Path & Equalizer

    SetTPIMode // Enable TPI Mode

	SiiRegWrite(TPI_HDCP_CONTROL_DATA_REG, 0);
	// ksv exchange will not occur on tmds_oe ->0, unless av mute is already on
//SII8240_VER75	+++	
    cache_TPI_SYSTEM_CONTROL_DATA_REG = 0x10;  //default value;
    ModifyTpiSystemControlDataReg(AV_MUTE_MASK,  AV_MUTE_MUTED)

    SiiRegWrite(REG_TPI_HW_OPT3,0x76);
    SiiRegWrite(REG_TPI_INTR_ST0,0xFF); // clear stale HDCP interrupt status
//SII8240_VER75	---

        SiiRegWrite(REG_RX_HDMI_CTRL3,0x00);
//SII8240_VER81 +++
    	SiiRegWrite(REG_MHLTX_CTL1, BIT_MHLTX_CTL1_TX_TERM_MODE_100DIFF | BIT_MHLTX_CTL1_DISC_OVRIDE_ON); // TX Source termination ON
//SII8240_VER81 ---
	// MHLTX_CTL6 was split into two registers
//default value 0xA0	SiiRegWrite(REG_MHLTX_CTL6, 0xBC); // Enable 1X MHL clock output
//default value 0x1E	SiiRegWrite(REG_MHLTX_CTRL_AON,0x1E);

//default value 0x3C	SiiRegWrite(REG_MHLTX_CTL2, 0x3C); // TX Differential Driver Config
    SiiRegModify(REG_MHLTX_CTL3
        , BIT_MHLTX_CTL3_DAMPING_SEL_MASK
        , BIT_MHLTX_CTL3_DAMPING_SEL_150_OHM
        );

//SII8240_VER75	+++
#ifdef VD_EMI_TEST
   if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
   {
//EMI workaround for pad mode +++   
	    SiiRegModify(REG_MHLTX_CTL4
	        ,BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK
	            ,0x0
	            ); //2012-05-10-bugzilla 24790

	    SiiRegWrite(REG_MHLTX_CTL7, 0x00);	
//EMI workaround for pad mode ---  						
   }
   else
   {   
//for pass CTS eye diagram, optimize swing value   
	    SiiRegModify(REG_MHLTX_CTL4
	        ,BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK
	            ,0x37
	            ); //2012-05-10-bugzilla 24790
    }
#else
	   if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
	   {
//for pass EMI	   
		    SiiRegModify(REG_MHLTX_CTL4
		        ,BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK
		            ,0x0
		            ); //2012-05-10-bugzilla 24790
	   }
	   else
	   {

		   switch (g_A68_hwID)
		   {
		        case A68_PR:
		        case A68_MP:
					
//for pass CTS eye diagram, optimize swing value
			    SiiRegModify(REG_MHLTX_CTL4
			        ,BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK
			            ,0x37
			            ); //2012-05-10-bugzilla 24790
			            
			    SiiRegWrite(REG_MHLTX_CTL7, 0x09);	
			    break;	
			    
			 default:
// for ER			 
//for pass CTS eye diagram, optimize swing value
			    SiiRegModify(REG_MHLTX_CTL4
			        ,BIT_CLK_SWING_CTL_MASK | BIT_DATA_SWING_CTL_MASK
			            ,0x37
			            ); //2012-05-10-bugzilla 24790
			    SiiRegWrite(REG_MHLTX_CTL7, 0x09);				            
		               break;
		   }
	   }
#endif
//SII8240_VER75	---

//SII8240_VER75	+++
	SiiRegWrite(REG_CBUS_LINK_XMIT_BIT_TIME,0x1C); //2012-05-25 bugzilla 24790
//SII8240_VER75	---
//default value 0x03	SiiRegWrite(REG_MHLTX_CTL7, 0x03); // 2011-10-10
	//2011-12-08 bit 3 soon to be defaulted on, but not yet
	SiiRegWrite(REG_MHLTX_CTL8, 0x0A); // PLL bias current, PLL BW Control

	// Analog PLL Control
//default value 0x08    SiiRegWrite(REG_TMDS_CCTRL, 0x08);			// Enable Rx PLL clock 2011-10-10 - select BGR circuit for voltage references

    //default value 0x8C	SiiRegWrite(REG_USB_CHARGE_PUMP, 0x8C);		// 2011-10-10 USB charge pump clock
        SiiRegWrite(REG_TMDS_CTRL4, 0x02);

    SiiRegWrite(REG_TMDS0_CCTRL2, 0x00);
//default value 0x07	SiiRegModify(REG_DVI_CTRL3, BIT5, 0);      // 2011-10-10
//dafault value 0x60	SiiRegWrite(REG_TMDS_TERMCTRL1, 0x60);

//default value 0x03	SiiRegWrite(REG_PLL_CALREFSEL, 0x03);			// PLL Calrefsel
//default value 0x20	SiiRegWrite(REG_PLL_VCOCAL, 0x20);			// VCO Cal
//default value 0xE0	SiiRegWrite(REG_EQ_DATA0, 0xE0);			// Auto EQ
//default value 0xC0	SiiRegWrite(REG_EQ_DATA1, 0xC0);			// Auto EQ
//default value 0xA0	SiiRegWrite(REG_EQ_DATA2, 0xA0);			// Auto EQ
//default value 0x80	SiiRegWrite(REG_EQ_DATA3, 0x80);			// Auto EQ
//default value 0x60	SiiRegWrite(REG_EQ_DATA4, 0x60);			// Auto EQ
//default value 0x40	SiiRegWrite(REG_EQ_DATA5, 0x40);			// Auto EQ
//default value 0x20	SiiRegWrite(REG_EQ_DATA6, 0x20);			// Auto EQ
//default value 0x00	SiiRegWrite(REG_EQ_DATA7, 0x00);			// Auto EQ

//default value 0x0A	SiiRegWrite(REG_BW_I2C, 0x0A);			// Rx PLL BW ~ 4MHz
//default value 0x06	SiiRegWrite(REG_EQ_PLL_CTRL1, 0x06);			// Rx PLL BW value from I2C

//default value 0x06	SiiRegWrite(REG_MON_USE_COMP_EN, 0x06);

    // synchronous s/w reset
	SiiRegWrite(REG_ZONE_CTRL_SW_RST, 0x60);			// Manual zone control
//SII8240_VER75	+++	
    SiiRegWrite(REG_ZONE_CTRL_SW_RST, 0xD0);			// Manual zone control 2012-05-30 0xE0->0xD0 as initial value.
//SII8240_VER75	---

//default value	SiiRegWrite(REG_MODE_CONTROL, 0x00);			// PLL Mode Value

//no longer needed SiiRegWrite(REG_SYS_CTRL1, 0x35);			// bring out from power down (script moved this here from above)

//SII8240_VER81_1 +++
    //default value 0xAD
//    #ifdef reduce_RGND_detect_time
	if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
	 SiiRegWrite(REG_DISC_CTRL2, 0xA0);
	else
        SiiRegWrite(REG_DISC_CTRL2, 0xA5);
//SII8240_VER81_1 ---	
    //experiment default value 0x57    SiiRegWrite(REG_DISC_CTRL5, 0x0x66);				// 1.8V CBUS VTH 5K pullup for MHL state

	if (g_chipDeviceId == BISHOP_DEVICE_ID)
	{
//SII8240_VER75	+++	
    //		SiiRegWrite(REG_DISC_CTRL5, 0x02);				// 1.8V CBUS VTH 5K pullup for MHL state
//SII8240_VER75	---	
		SiiRegWrite(REG_DISC_CTRL6, 0x01);				// RGND & single discovery attempt (RGND blocking)
		SiiRegWrite(REG_DISC_CTRL8, 0x81);				// Ignore VBUS
	}
	else if (g_chipDeviceId == WOLV60_DEVICE_ID)
	{
    	SiiRegWrite(REG_DISC_CTRL6, 0x11);				// RGND & single discovery attempt (RGND blocking)
    	// SiiRegWrite(REG_DISC_CTRL8, 0x82);				// Ignore VBUS
	}

	if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
	{
    // No OTG, Discovery pulse proceed, Wake pulse not bypassed
    SiiRegWrite(REG_DISC_CTRL9
                , BIT_DC9_CBUS_LOW_TO_DISCONNECT
                | BIT_DC9_WAKE_DRVFLT
                | BIT_DC9_DISC_PULSE_PROCEED
//SII8240_VER75	+++					
//bypass_Wake_pulse//hammer20120703
                | BIT_DC9_WAKE_PULSE_BYPASS
//SII8240_VER75	---			   
//bugzilla 24170                | BIT_DC9_VBUS_OUTPUT_CAPABILITY_SRC
             );
	}
	else
	{
	    // No OTG, Discovery pulse proceed, Wake pulse not bypassed
	    SiiRegWrite(REG_DISC_CTRL9
	                , BIT_DC9_CBUS_LOW_TO_DISCONNECT
	                | BIT_DC9_WAKE_DRVFLT
	                | BIT_DC9_DISC_PULSE_PROCEED
	//bugzilla 24170                | BIT_DC9_VBUS_OUTPUT_CAPABILITY_SRC
	             );	
	}

    // leave bit 3 reg_usb_en at its default value of 1
//default value 0x8C    SiiRegWrite(REG_DISC_CTRL4, 0x8C);				// Pull-up resistance off for IDLE state and 10K for discovery state.
    if (g_chipDeviceId == WOLV60_DEVICE_ID)
    {
    	SiiRegWrite(REG_DISC_CTRL1, 0x27);				// Enable CBUS discovery
//        	#ifdef reduce_time_from_wake_to_discover
			if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))			
        		SiiRegWrite(REG_DISC_CTRL3, 0x80);				// MHL CBUS discovery
			else
        		SiiRegWrite(REG_DISC_CTRL3, 0x86);				// MHL CBUS discovery
    }
    else if (g_chipDeviceId == BISHOP_DEVICE_ID)
    {
    	SiiRegWrite(REG_DISC_CTRL1, 0x25);				// Enable CBUS discovery
    }
//default value 0x20 SiiRegWrite(REG_DISC_CTRL7, 0x20);				// use 1K only setting

//keno hdcp test 20121001 +++
///    SiiRegWrite(REG_PKT_FILTER_0, BIT_PKT_FILTER_0_DROP_GCP_PKT | BIT_PKT_FILTER_0_DROP_AVI_PKT | BIT_PKT_FILTER_0_DROP_MPEG_PKT | BIT_PKT_FILTER_0_DROP_CEA_GAMUT_PKT);
    SiiRegWrite(REG_PKT_FILTER_0, 0x00);//BIT_PKT_FILTER_0_DROP_GCP_PKT | BIT_PKT_FILTER_0_DROP_AVI_PKT | BIT_PKT_FILTER_0_DROP_MPEG_PKT | BIT_PKT_FILTER_0_DROP_CEA_GAMUT_PKT);
//keno hdcp test 20121001 ---

//ASUS BSP +++ enable HPD push-pull
#ifdef ASSERT_PUSH_PULL
    SiiRegModify(REG_HPD_CTRL,BIT6,0);	// Assert Push/Pull		
	TX_DEBUG_PRINT(("Drv: ### Assert HPD Push/Pull trans mode(%d)	 \n", g_modeFlags));
    
#else
	TX_DEBUG_PRINT(("Drv: NO NO Assert HPD Push/Pull trans mode(%d)	 \n" g_modeFlags));

#endif
//ASUS BSP --- enable HPD push-pull


//SII8240_VER75	+++		
//UP_HPD_Pull_High//hammer20120703
	if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
	{
		SiiRegModify(REG_DPD,BIT_DPD_PDIDCK_MASK,BIT_DPD_PDIDCK_NORMAL_OPERATION);
		SiiRegModify( REG_HPD_CTRL   , BIT7	    , BIT7   );
		SiiMhlTxDrvAcquireUpstreamHPDControlDriveHigh();
	} 
	else
	{
		// do this for non-trancode mode
		SiiRegModify(REG_DPD,BIT_DPD_PDIDCK_MASK,BIT_DPD_PDIDCK_POWER_DOWN);
		SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow(); // Force HPD to 0 when not in MHL mode.
	}
//SII8240_VER75	---

        SiiRegModify(REG_HDMI_CLR_BUFFER
            ,BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_W_AVI_EN_MASK  | BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_EN_MASK
            ,BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_W_AVI_EN_CLEAR | BIT_HDMI_CLR_BUFFER_RX_HDMI_VSI_CLR_EN_CLEAR
            );

    //TODO re-investigate
	SiiRegWrite(REG_SRST, 0x84); 					// Enable Auto soft reset on SCDT = 0

        if (g_chipDeviceId == BISHOP_DEVICE_ID)
    	{
    		SiiRegWrite(REG_PWD_SRST, 0x80); 	 // MHL Auto FIFO Reset
    	}

        SiiRegWrite(REG_DCTL, BIT_DCTL_TRANSCODE_OFF | BIT_DCTL_TLCK_PHASE_INVERTED );
    }


	if (g_chipDeviceId == BISHOP_DEVICE_ID)
	{
		SiiRegWrite(REG_PWD_SRST, 0x80); 	 // MHL Auto FIFO Reset
	}
	
	CbusReset(); // TPI mode is enabled in SiiMhlTxChipInitialize

	InitCBusRegs();

	TX_DEBUG_PRINT(("Drv: WriteInitialRegisterValuesPartTwo --\n"));	
}

///////////////////////////////////////////////////////////////////////////
// InitCBusRegs
//
///////////////////////////////////////////////////////////////////////////
//SII8240_VER81 +++
PLACE_IN_CODE_SEG uint8_t devCap[16]=
{
     DEVCAP_VAL_DEV_STATE
    ,DEVCAP_VAL_MHL_VERSION
    ,DEVCAP_VAL_DEV_CAT
    ,DEVCAP_VAL_ADOPTER_ID_H
    ,DEVCAP_VAL_ADOPTER_ID_L
    ,DEVCAP_VAL_VID_LINK_MODE
    ,DEVCAP_VAL_AUD_LINK_MODE
    ,DEVCAP_VAL_VIDEO_TYPE
    ,DEVCAP_VAL_LOG_DEV_MAP
    ,DEVCAP_VAL_BANDWIDTH
    ,DEVCAP_VAL_FEATURE_FLAG
    ,DEVCAP_VAL_DEVICE_ID_H
    ,DEVCAP_VAL_DEVICE_ID_L
    ,DEVCAP_VAL_SCRATCHPAD_SIZE
    ,DEVCAP_VAL_INT_STAT_SIZE
    ,DEVCAP_VAL_RESERVED
};
//SII8240_VER81 ---
static void InitCBusRegs (void)
{
	TX_DEBUG_PRINT(("Drv: InitCBusRegs\n"));

//2011-12-08 no need to write	SiiRegWrite(REG_CBUS_COMMON_CONFIG, 0xF2); 			 // Increase DDC translation layer timer
#ifdef OLD_LINK_CONTROL //(
	SiiRegWrite(REG_CBUS_LINK_CONTROL_7, 0x0B);   	 // Drive High Time. -- changed from 0x03 on 2011-11-21 -- changed from 0x0C on 2011-10-03 - 17:00
	SiiRegWrite(REG_CBUS_LINK_CONTROL_8, 0x30);   	 // Use programmed timing.
	SiiRegWrite(REG_CBUS_DRV_STRENGTH_0, 0x03); 			// CBUS Drive Strength
#endif //)

//SII8240_VER81 +++
	// Setup our devcap
    // make devcap loading quicker by using block write
    SiiRegWriteBlock(REG_CBUS_DEVICE_CAP_0,devCap,sizeof(devCap));
//SII8240_VER81 ---

#ifdef OLD_LINK_CONTROL //(
	// Make bits 2,3 (initiator timeout) to 1,1 for register CBUS_LINK_CONTROL_2
    SiiRegModify(REG_CBUS_LINK_CONTROL_2 ,BIT_CBUS_INITIATOR_TIMEOUT_MASK,BIT_CBUS_INITIATOR_TIMEOUT_MASK);

	 // Clear legacy bit on Wolverine TX. and set timeout to 0xF
    SiiRegWrite(REG_MSC_TIMEOUT_LIMIT, 0x0F);   // New :   MSC_TIMEOUT

	// Set NMax to 1
	SiiRegWrite(REG_CBUS_LINK_CONTROL_1, 0x01);   // New :   CBUS_LINK_CONTROL_1
    SiiRegModify(REG_CBUS_MSC_COMPATIBILITY_CTRL /* // New :   CBUS_MSC_COMPATIBILITY_CONTROL */, BIT_CBUS_CEC_DISABLE /* // New :   CBUS_MSC_COMPATIBILITY_CONTROL */, BIT_CBUS_CEC_DISABLE);  // disallow vendor specific commands
#endif //)
}

///////////////////////////////////////////////////////////////////////////
//
// ForceUsbIdSwitchOpen
//
///////////////////////////////////////////////////////////////////////////
static void ForceUsbIdSwitchOpen (void)
{
	DISABLE_DISCOVERY
	SiiRegModify(REG_DISC_CTRL6, BIT_DC6_USB_OVERRIDE_MASK, BIT_DC6_USB_OVERRIDE_ON);				// Force USB ID switch to open
	SiiRegWrite(REG_DISC_CTRL3,  BIT_DC3_COMM_IMME_ON
	                           | BIT_DC3_FORCE_MHL_OFF
	                           | BIT_DC3_DISC_SIMODE_OFF
	                           | BIT_DC3_FORCE_USB_OFF
	                           | BIT_DC3_USB_EN_OFF
	                           | BIT_DC3_DLYTRG_SEL_064ms
	                           );
    // Force HPD to 0 when not in Mobile HD mode.
    SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow ();
}
///////////////////////////////////////////////////////////////////////////
//
// ReleaseUsbIdSwitchOpen
//
///////////////////////////////////////////////////////////////////////////
static void ReleaseUsbIdSwitchOpen (void)
{
	HalTimerWait(50); // per spec
	SiiRegModify(REG_DISC_CTRL6, BIT6, 0x00);
	ENABLE_DISCOVERY
}
/*
	SiiMhlTxDrvProcessMhlConnection
		optionally called by the MHL Tx Component after giving the OEM layer the
		first crack at handling the event.
*/
void SiiMhlTxDrvProcessRgndMhl( void )
{
	//perhaps this could be done in the future, but not now SiiRegModify(REG_DISC_CTRL9, BIT_DC9_VBUS_OUTPUT_CAPABILITY_SRC, BIT_DC9_VBUS_OUTPUT_CAPABILITY_SRC);
}

//ASUS_BSP , for A68 otg feature, check carkit cable in/out in MHL and msm otg driver +++
static void (*notify_carkit_in_out_func_ptr)(int) = NULL;
int dp_registerCarkitInOutNotificaition(void (*callback)(int))
{
    printk("[MHL]%s +++\n",__FUNCTION__);
    
    notify_carkit_in_out_func_ptr = callback;

//ASUS_BSP: for Booting with otg plug-in issue
    if (g_b_SwitchCarkitBootPlugin)
    {
           printk("[MHL] Boot plug-in initial carkit cable notify\n");
	    (*notify_carkit_in_out_func_ptr) (1);
    }
    printk("[MHL]%s ---\n",__FUNCTION__);

    return 0;
}

void mhl_switch_carkit(bool_t enable)
{
	g_b_isCarkitConnected=enable;	
        if(NULL != notify_carkit_in_out_func_ptr)
	{
             printk("[MHL] carkit cable notify (%d)\n", enable);
             (*notify_carkit_in_out_func_ptr) (enable);
        }
       else
     	{
		if (g_b_SwitchCarkitInitial)
		{
//ASUS_BSP , for debug usage, set carkit uevent in MHL driver
			printk("[MHL] nobody carkit registed, so switch carkit cable in MHL driver\n");
			switch_set_state(&switch_carkit_cable, enable);
			printk("[MHL] %d: %s\n", switch_carkit_cable.state, __func__);
		}
		else
		{
//ASUS_BSP: for Booting with otg plug-in issue
			printk("[MHL] %s : Boot plug-in, but not initial switch carkit cable\n", __func__);
			g_b_SwitchCarkitBootPlugin = 1;
		}
	}	
}
//ASUS_BSP , for A68 otg feature, check carkit cable in/out in MHL and msm otg driver ---

bool_t mhl_check_carkit_mode(void)
{
	if (g_pad_tv_mode == MHL_CARKIT)
		return true;
	else	
	{
		printk("[MHL]  mhl_check_carkit_mode (%d)\n", g_pad_tv_mode);		
		return false;
	}
}
///////////////////////////////////////////////////////////////////////////
// ProcessRgnd
//
// H/W has detected impedance change and interrupted.
// We look for appropriate impedance range to call it MHL and enable the
// hardware MHL discovery logic. If not, disable MHL discovery to allow
// USB to work appropriately.
//
// In current chip a firmware driven slow wake up pulses are sent to the
// sink to wake that and setup ourselves for full D0 operation.
///////////////////////////////////////////////////////////////////////////
uint8_t RgndResultIsMhl (void)
{
	if (g_chipDeviceId == BISHOP_DEVICE_ID)
	{
#if 0 //(
		uint8_t int4Status;
		int4Status = SiiRegRead(REG_INTR4);	// read status
		if(int4Status & BIT_INTR4_MHL_EST) // MHL_EST_INT
		{
			TX_DEBUG_PRINT(("(MHL Device)\n"));
			ENABLE_DISCOVERY;
			return 1;
		}
		else
		{
			SiiRegModify(REG_INTR4, BIT3, BIT3);	// USB Established
			TX_DEBUG_PRINT(("(Non-MHL Device)\n"));
			return 0;
		}
#else //)(
        return 1;
#endif //)
	}
	else if (g_chipDeviceId == WOLV60_DEVICE_ID)
	{
	uint8_t rgndImpedance;
	uint8_t  mode;
    	//
    	// Impedance detection has completed - process interrupt
    	//
    	rgndImpedance = SiiRegRead(REG_DISC_STAT2) & 0x03;
    	TX_DEBUG_PRINT(("Drv: RGND = %02X : \n", (int)rgndImpedance));

    	//
    	// 00, 01 or 11 means USB.
    	// 10 means 1K impedance (MHL)
    	//
    	// If 1K, then only proceed with wake up pulses
    	if (0x02 == rgndImpedance)
    	{
    		TX_DEBUG_PRINT(("(MHL Device)\n"));

		mdelay(40); //for PR change workaround, delay to p03_plug_in stable
		
		//switch to USB	
		HalGpioSetMHLSwitchPin(1);
		mode = get_pad_tv_mode();
		if (mode == MHL_TV_MODE)
		{
			g_pad_tv_mode = MHL_TV_MODE;		
			printk("###MHL_TV_MODE ###\n");
		}		
		else if (mode == MHL_PAD_MODE_L)
		{
			g_pad_tv_mode = MHL_PAD_MODE_L;
			printk("###MHL_PAD_MODE_L ###\n");
			PackedPixelAvailable = 0;			
		}
		else if (mode == MHL_PAD_MODE_H)
		{
			g_pad_tv_mode = MHL_PAD_MODE_H;
			printk("###MHL_PAD_MODE_H ###\n");							
			PackedPixelAvailable = 1;			
		}
		else
		{
			g_pad_tv_mode = MHL_NONE;		
			printk("###MHL_NONE ###\n");			
		}
			
            return 1;
    	}
    	else
    	{

    		SiiRegModify(REG_DISC_CTRL9, BIT_DC9_USB_EST, BIT_DC9_USB_EST);	// USB Established
		//switch to USB	
		HalGpioSetMHLSwitchPin(0);

		if (rgndImpedance == 0x03)   // shorted, asus carkit
		{
			printk("+++ MHL CarKit detect (%d)+++\n", g_b_isCarkitConnected);		
			g_pad_tv_mode = MHL_CARKIT;		
				
			printk("[MHL] carkit detect , lunch CarHome\n");
			mhl_switch_carkit(true);

			printk("--- MHL CarKit detect (%d)---\n", g_b_isCarkitConnected);		
		}
		else
		{
			// carkit detect ---
//	    		TX_DEBUG_PRINT(("(Non-MHL Device)\n"));		
	    		printk(" Non-MHL Device %d\n", rgndImpedance);		
		}
            return 0;
    	}
    }
    return 0;
}


////////////////////////////////////////////////////////////////////
// SwitchToD0
// This function performs s/w as well as h/w state transitions.
//
// Chip comes up in D2. Firmware must first bring it to full operation
// mode in D0.
////////////////////////////////////////////////////////////////////
void SwitchToD0 (void)
{
	TX_DEBUG_PRINT(("Drv: Switch to D0 ++\n"));

	//
	// WriteInitialRegisterValuesPartOne switches the chip to full power mode.
	//
    WriteInitialRegisterValuesPartOne();

    // Force Power State to ON

      STROBE_POWER_ON; // Force Power State to ON

	WriteInitialRegisterValuesPartTwo();
//SII8240_VER81 +++	
	UpdatePowerState(POWER_STATE_D0_NO_MHL);
//SII8240_VER81 ---	
	TX_DEBUG_PRINT(("Drv: Switch to D0 --\n"));	

#ifdef KENO_DONGLE_DOWNSTREAM1
	MHL_Resume = false;
	if ((!packedPixelStatus) && (g_pad_tv_mode == MHL_TV_MODE) && (!MHL_Resume)) 
	{
		SiiRegModify(0x01C7,0x20, 0x20);//display black image
		SiiRegWrite(REG_VID_MODE, 0x12);  // KH, Enable Demuxing data
	}
#endif
}



////////////////////////////////////////////////////////////////////
// SwitchToD3
//
// This function performs s/w as well as h/w state transitions.
//
////////////////////////////////////////////////////////////////////
void SwitchToD3 (void)
{
	TX_DEBUG_PRINT(("Drv: SwitchToD3, power state =(%d) ++ \n",fwPowerState ));
//ASUS_BSP+++ larry lai : fix Pad mode TX hange issue
#ifdef Modification_INT4_Clear_Place
//clear INTR4
	SiiRegWrite(REG_INTR4, SiiRegRead(REG_INTR4));
	TX_DEBUG_PRINT(("Drv: SwitchToD3,after clear REG_INTR4 = 0x%x\n", SiiRegRead(REG_INTR4)));
#endif	
//ASUS_BSP--- larry lai : fix Pad mode TX hange issue	

	if(POWER_STATE_D3 != fwPowerState)
	{
		TX_DEBUG_PRINT(("Drv: Switch To D3: pinAllowD3 = %d\n",(int) PlatformGPIOGet(pinAllowD3) ));

#ifndef	__KERNEL__ //(
		pinM2uVbusCtrlM = 1;
		pinMhlConn = 1;
		pinUsbConn = 0;
#endif	//)

		// Force HPD to 0 when not in MHL mode.
        SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow();

		// Change TMDS termination to high impedance on disconnection
		// Bits 1:0 set to 11

//SII8240_VER81	+++
		SiiRegWrite(REG_MHLTX_CTL1, BIT_MHLTX_CTL1_TX_TERM_MODE_OFF | BIT_MHLTX_CTL1_DISC_OVRIDE_ON) ;
//SII8240_VER81	---

//SII8240_VER75	+++
//SII8240_VER81_1 +++
//#ifdef reduce_RGND_detect_time
	if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
        SiiRegWrite(REG_DISC_CTRL2, 0xA8); // set back to default value.
	else        
        SiiRegWrite(REG_DISC_CTRL2, 0xAD); // set back to default value.
//SII8240_VER81_1 ---

//SII8240_VER75	---

//SII8240_VER81	+++
        //FP1798  clear discovery interrupts before going to sleep.
        {
        uint16_t temp;
            temp = SiiRegRead(REG_INTR4);
            TX_DEBUG_PRINT(("temp: %02x\n",temp));
            SiiRegWrite(REG_INTR4, 0xFB); // clear all interrupts except MHL_EST
        }
//SII8240_VER81	---
		//
		// GPIO controlled from SiIMon can be utilized to disallow
		// low power mode, thereby allowing SiIMon to read/write register contents.
		// Otherwise SiIMon reads all registers as 0xFF
		//
		TX_DEBUG_PRINT(("pinAllowD3 = %d\n", pinAllowD3));
		if(PlatformGPIOGet(pinAllowD3))
		{
//ASUS_BSP+++ larry lai : fix Pad mode TX hange issue			
			//SII_MHL_TX_DUMP_INFO_CLEAR		
//ASUS_BSP--- larry lai : fix Pad mode TX hange issue

			// wait Tsrc:cbus_float
			if ((g_pad_tv_mode == MHL_TV_MODE))
			{
				HalTimerWait(50);
			}
			//
			// Change state to D3 by clearing bit 0 of 3D (SW_TPI, Page 1) register
			//
//SII8240_VER75	+++			
#if (TARGET_DEVICE_ID == BISHOP_DEVICE_ID) //(
			SiiRegWrite(REG_DPD, 0x00);
#else //)(
			CLR_BIT(REG_DPD, 0);
#endif //)
//SII8240_VER75	---
//ASUS BSP +++ : fix not switch to USB issue
			HalGpioSetMHLSwitchPin(0);
//ASUS BSP ---
//SII8240_VER81	+++
			UpdatePowerState( POWER_STATE_D3 );
//SII8240_VER81	---
		}
		else
		{
//SII8240_VER81	+++		
			UpdatePowerState(POWER_STATE_D0_NO_MHL);
//SII8240_VER81	---			
		}
	}
		TX_DEBUG_PRINT(("## Drv: SwitchToD3 power state =(%d) --5 \n",fwPowerState ));
	
}

////////////////////////////////////////////////////////////////////
// Int4Isr
//
//
//	Look for interrupts on INTR4 (Register 0x21)
//		7 = RSVD		(reserved)
//		6 = RGND Rdy	(interested)
//		5 = CBUS Disconnect	(interested)
//		4 = CBUS LKOUT	(interested)
//		3 = USB EST		(interested)
//		2 = MHL EST		(interested)
//		1 = RPWR5V Change	(ignore)
//		0 = SCDT Change	(interested during D0)
//
////////////////////////////////////////////////////////////////////
static int Int4Isr (void)
{
	uint8_t int4Status;

// 	TX_DEBUG_PRINT(("\n +++ Int4Isr ++\n"));
	
	int4Status = SiiRegRead(REG_INTR4);	// read status
	msleep(15);	//ASUS_BSP +++ Jason chang "[A68][MHL][TT425876]add delay for workaround"
// 	TX_DEBUG_PRINT(("## Int4Isr status 0x%02x, power state=%d\n", int4Status, fwPowerState));

	// When I2C is inoperational (D3) and a previous interrupt brought us here, do nothing.
	if(0xFF != int4Status)
	{
		if(int4Status)
		{
			msleep(15);	//ASUS_BSP +++ Jason chang "[A68][MHL][TT425876]add delay for workaround"
			TX_DEBUG_PRINT(("Drv: INT4 Status = %02X\n", (int) int4Status));
//ASUS_BSP+++ larry lai : fix Pad mode TX hange issue
#ifndef Modification_INT4_Clear_Place//move clear interupt activity to this function last line.            
            /* Now that RGND interrupts are deferred until the chip has
                awoken sufficiently to allow I2C reads to succeed,
                we can clear the interupt right away.
            */
            SiiRegWrite(REG_INTR4, int4Status); // clear all interrupts
			TX_DEBUG_PRINT(("Drv: INT4 Status = %02X\n", (int) SiiRegRead(REG_INTR4)));
#endif
//ASUS_BSP--- larry lai : fix Pad mode TX hange issue
    		if (int4Status & BIT_INTR4_RGND_DETECTION)
    		{
                if (POWER_STATE_D0_MHL != fwPowerState)
                {

                    if (RgndResultIsMhl())
                    {
            			// Switch to full power mode.
            			SwitchToD0();

                		SiiMhlTxNotifyRgndMhl(); // this will call the application and then optionally call
                    }
                }
    		}


    		if(int4Status & BIT_INTR4_MHL_EST) // MHL_EST_INT
    		{
//SII8240_VER75	+++			
		
#if BUILD_CONFIG == 2
				// Switch to full power mode.
				SwitchToD0();
#endif
//SII8240_VER75	---
    			MhlTxDrvProcessConnection();
    		}

    		else if(int4Status & BIT_INTR4_NON_MHL_EST)
    		{
    			TX_DEBUG_PRINT(("Drv: Non- MHL device detected.\n"));
    			SiiRegWrite(REG_DISC_STAT2, 0x80);	// Exit D3 via CBUS falling edge
    			SwitchToD3();
    			return I2C_INACCESSIBLE;
    		}

    		if (int4Status & BIT_INTR4_CBUS_DISCONNECT)
    		{
    			TX_DEBUG_PRINT(("Drv: CBUS Disconnect.\n"));
    			MhlTxDrvProcessDisconnection();

    			return I2C_INACCESSIBLE;
    		}

        	// Can't succeed at these in D3
        	if(fwPowerState != POWER_STATE_D3)
        	{
        		// CBUS Lockout interrupt?
        		if (int4Status & BIT_INTR4_CBUS_LKOUT)
        		{
        			TX_DEBUG_PRINT(("Drv: CBus Lockout\n"));

        			ForceUsbIdSwitchOpen();
        			ReleaseUsbIdSwitchOpen();
        		}
            }
		}
//ASUS_BSP+++ larry lai : fix Pad mode TX hange issue
#ifdef Modification_INT4_Clear_Place//clear interrupt
		SiiRegWrite(REG_INTR4, int4Status); // clear all interrupts
		//ASUS_BSP +++ Jason chang "[A68][MHL][TT425876]add delay for workaround"
		SiiRegRead(REG_INTR4);
		msleep(10);
		//ASUS_BSP --- Jason chang "[A68][MHL][TT425876]add delay for workaround"
//		TX_DEBUG_PRINT(("## Drv: INT4 Status = %02X\n", (int) SiiRegRead(REG_INTR4)));
#endif		
//ASUS_BSP--- larry lai : fix Pad mode TX hange issue		
		
	}

// 	TX_DEBUG_PRINT(("\nInt4Isr --\n"));		
	return I2C_ACCESSIBLE;
}

////////////////////////////////////////////////////////////////////
// Int5Isr
//
//
//	Look for interrupts on INTR5
//		7 =
//		6 =
//		5 =
//		4 =
//		3 =
//		2 =
//		1 =
//		0 =
////////////////////////////////////////////////////////////////////
static void Int5Isr (void)
{
	uint8_t int5Status;

	int5Status = SiiRegRead(REG_INTR5);

    if (int5Status)
    {
//SII8240_VER75	+++			
#ifdef	APPLY_PLL_RECOVERY //(
        if ((BIT_INTR5_MHL_FIFO_OVERFLOW | BIT_INTR5_MHL_FIFO_UNDERFLOW) & int5Status)
        {
    		TX_DEBUG_PRINT(("Drv: %s %s\n"
    		    ,(int5Status & BIT_INTR5_MHL_FIFO_OVERFLOW )?"BIT_INTR5_MHL_FIFO_OVERFLOW" :""
    		    ,(int5Status & BIT_INTR5_MHL_FIFO_UNDERFLOW)?"BIT_INTR5_MHL_FIFO_UNDERFLOW":""
    		    ));
            ApplyPllRecovery ( );
        }

#endif	//)
//SII8240_VER75	---
    	SiiRegWrite(REG_INTR5, int5Status);	// clear all interrupts
    	if(int5Status & BIT_INTR5_CKDT_CHANGE)
    	{

        uint8_t temp;
    		TX_DEBUG_PRINT(("Drv: BIT_INTR5_CKDT_CHANGE.\n"));
            temp = SiiRegRead(REG_TMDS_CSTAT_P3);
    		TX_DEBUG_PRINT(("Drv: BIT_INTR5_CKDT_CHANGE. CSTAT_P3:0x%x\n",(uint16_t)temp ));
            if (BIT_TMDS_CSTAT_P3_PDO_CLOCK_DETECTED== (temp & BIT_TMDS_CSTAT_P3_PDO_MASK))
            {
    		// todo HERE
    		/*
    			video is now stable
    			look at upstream AVI info frame and
    			program timing and other mode related
    			register according to the information
    			found therein.
    		*/
    		    TX_DEBUG_PRINT(("Drv: clock is stable.\n"));


            }

    	}
        if (int5Status & BIT_INTR5_SCDT_CHANGE)
        {
        uint8_t temp;
    		TX_DEBUG_PRINT(("Drv: BIT_INTR5_SCDT_CHANGE.\n"));
            temp = SiiRegRead(REG_TMDS_CSTAT_P3);
    		TX_DEBUG_PRINT(("Drv: BIT_INTR5_SCDT_CHANGE. CSTAT_P3:0x%x\n",(uint16_t)temp ));

#ifdef KENO_DONGLE_DOWNSTREAM1
		if ((!packedPixelStatus) && (g_pad_tv_mode == MHL_TV_MODE) && (!MHL_Resume)) 
		{
			SiiRegModify(0x01C7,0x20, 0x20);//display black image
			SiiRegWrite(REG_VID_MODE, 0x12);  // KH, Enable Demuxing data
		}
#endif

//SII8240_VER75	+++						
            if (!(mfdrvTranscodeMode & g_modeFlags))
            {
                SiiMhlTxNotifySyncDetect((BIT_TMDS_CSTAT_P3_SCDT & temp) ? 1 : 0);
            }
//SII8240_VER75	---
        }

    	if (int5Status & BIT_INTR5_RPWR5V_CHG)
    	{
    	/*
    		todo: upstream transmitter is ready to read EDID.
    		Set a flag here to check later when downstream HPD
    		comes in.
    	*/
    	uint8_t tmdsCStatus;
    		tmdsCStatus = SiiRegRead(REG_TMDS_CSTAT);
    		if (tmdsCStatus & BIT_TMDS_CSTAT_RPWR5V_STATUS)
    		{
    			TX_DEBUG_PRINT(("Drv: RPWR5V low to high xition.\n"));
    			/* todo: refresh EDID on transition from low to high */
//SII8240_VER75	+++
/*				
			if ((g_pad_tv_mode == MHL_PAD_MODE_L) || (g_pad_tv_mode == MHL_PAD_MODE_H))
			{
				// power up the receiver
				SiiRegModify(REG_DPD, BIT_DPD_PDIDCK_MASK, BIT_DPD_PDIDCK_NORMAL_OPERATION);
				SiiRegModify(REG_HPD_CTRL, BIT7, BIT7);  // bypass RX setHPD,  force pull HPD 
				SiiMhlTxDrvAcquireUpstreamHPDControlDriveHigh();
			}
*/
//SII8240_VER75	---					
    		}
    	}
    }

}

///////////////////////////////////////////////////////////////////////////
//
// MhlTxDrvProcessConnection
//
///////////////////////////////////////////////////////////////////////////
static void MhlTxDrvProcessConnection (void)
{
	TX_DEBUG_PRINT (("Drv: MHL Cable Connected. CBUS:0x0A = %02X fwPowerState:%d\n", (int) SiiRegRead(REG_CBUS_STATUS),(unsigned int)fwPowerState));  // New :  CBUS_STATUS

	if( POWER_STATE_D0_MHL == fwPowerState )
	{
    	TX_DEBUG_PRINT (("Drv: Already in D0_MHL fwPowerState:%d\n", (unsigned int)fwPowerState));
		return;
	}
//( FP1798
    /*
        if we're called when power state is D3,
            then we abort and wait for RGND
    */
    if ( POWER_STATE_D3 == fwPowerState)
    {
    	TX_DEBUG_PRINT (("Drv: I2C registers inaccessible in fwPowerState:%d\n", (unsigned int)fwPowerState));
        return;
    }
//) FP1798

#ifndef	__KERNEL__ //(
    TX_DEBUG_PRINT (("Drv: Controlling VBUS fwPowerState:%d\n", (unsigned int)fwPowerState));
	// VBUS control gpio
	pinM2uVbusCtrlM = 0;
	pinMhlConn = 0;
	pinUsbConn = 1;
#endif	// )

    TX_DEBUG_PRINT (("Drv: trying to control upstream HDP\n"));

//SII8240_VER75	+++
//#ifndef UP_HPD_Pull_High//hammer20120703
//	if (g_pad_tv_mode == MHL_TV_MODE)
	{
		SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow();
	}	
//SII8240_VER75	---
	//
	// Discovery over-ride: reg_disc_ovride
	//
//SII8240_VER81	+++	
	SiiRegWrite(REG_MHLTX_CTL1, BIT_MHLTX_CTL1_TX_TERM_MODE_100DIFF | BIT_MHLTX_CTL1_DISC_OVRIDE_ON);

	UpdatePowerState(POWER_STATE_D0_MHL);
//SII8240_VER81	---
	//
	// Increase DDC translation layer timer (uint8_t mode)
    //SiiRegWrite(REG_CBUS_DDC_TIMEOUT,0x0F);


	// Keep the discovery enabled. Need RGND interrupt
	ENABLE_DISCOVERY

	// Notify upper layer of cable connection
	SiiMhlTxNotifyConnection(true);

	if (g_pad_tv_mode == MHL_TV_MODE)
	{
		sii8240drv_wakelock();		
	}	
}

///////////////////////////////////////////////////////////////////////////
//
// MhlTxDrvProcessDisconnection
//
///////////////////////////////////////////////////////////////////////////
static void MhlTxDrvProcessDisconnection (void)
{

	TX_DEBUG_PRINT(("Drv: MhlTxDrvProcessDisconnection\n"));

	if (g_pad_tv_mode == MHL_TV_MODE)
		sii8240drv_wakeunlock();

	// clear all interrupts
	SiiRegWrite(REG_INTR4, SiiRegRead(REG_INTR4));
//SII8240_VER81	+++
	SiiRegWrite(REG_MHLTX_CTL1, BIT_MHLTX_CTL1_TX_TERM_MODE_OFF | BIT_MHLTX_CTL1_DISC_OVRIDE_ON);
//SII8240_VER81	---

	dsHpdStatus &= ~BIT_CBUS_HPD;  //cable disconnect implies downstream HPD low
	SiiRegWrite(REG_CBUS_STATUS, dsHpdStatus);    // New :	 MSC_MT_ABORT_INT
	SiiMhlTxNotifyDsHpdChange(0);

	if( POWER_STATE_D0_MHL == fwPowerState )
	{
		// Notify upper layer of cable removal
		SiiMhlTxNotifyConnection(false);
	}

	// Now put chip in sleep mode
	SwitchToD3();	
}
//ASUS BSP +++
void MhlTxDisconnect(void)
{
  printk("[MHL] MhlTxDisconnect++\n");
//ASUS_BSP +++ : for lock ISR in disconnecting progress  
  g_TxDisconnect_lock = 1;
  MhlTxDrvProcessDisconnection();
  g_TxDisconnect_lock = 0;  
//ASUS_BSP --- : for lock ISR in disconnecting progress  
  printk("[MHL] MhlTxDisconnect--\n"); 
}
//ASUS BSP +++

///////////////////////////////////////////////////////////////////////////
//
// CbusReset
//
///////////////////////////////////////////////////////////////////////////
void CbusReset (void)
{
uint8_t enable[4]={0xff,0xff,0xff,0xff};// must write 0xFF to clear regardless!

	mscCmdInProgress = false;

    {
    int i;
        for (i =0; i <sizeof(g_InterruptMasks)/sizeof(g_InterruptMasks[0]);++i)
        {
            SiiRegWrite(g_InterruptMasks[i].regName,g_InterruptMasks[i].value);
        }
    }

    if (mfdrvTranscodeMode & g_modeFlags)
    {
    int i;
        for (i =0; i <sizeof(g_TranscodeInterruptMasks)/sizeof(g_TranscodeInterruptMasks[0]);++i)
        {
            SiiRegWrite(g_TranscodeInterruptMasks[i].regName,g_TranscodeInterruptMasks[i].value);
        }
    }
    else
    {
    int i;
        for (i =0; i <sizeof(g_NonTranscodeInterruptMasks)/sizeof(g_NonTranscodeInterruptMasks[0]);++i)
        {
            SiiRegWrite(g_NonTranscodeInterruptMasks[i].regName,g_NonTranscodeInterruptMasks[i].value);
        }
    }
	// Enable SET_INT interrupt for writes to all 4 MSC Interrupt registers.
    SiiRegWriteBlock(REG_CBUS_SET_INT_ENABLE_0,enable,4);
}

///////////////////////////////////////////////////////////////////////////
//
// CBusProcessErrors
//
//
///////////////////////////////////////////////////////////////////////////
static uint8_t CBusProcessErrors (uint8_t int1Status)
{
    uint8_t result          = 0;
//    uint8_t mscAbortReason  = 0;
	uint8_t ddcAbortReason  = 0;



    /* At this point, we only need to look at the abort interrupts. */

    int1Status &= (BIT_CBUS_MSC_ABORT_RCVD| BIT_CBUS_DDC_ABRT | BIT_CBUS_CMD_ABORT);

    if ( int1Status )
    {
//      result = ERROR_CBUS_ABORT;		// No Retry will help

        /* If transfer abort or MSC abort, clear the abort reason register. */
		if( int1Status & BIT_CBUS_DDC_ABRT)
		{
			result = ddcAbortReason = SiiRegRead(REG_CBUS_DDC_ABORT_INT);
			TX_DEBUG_PRINT(("CBUS DDC ABORT happened, reason:: %02X\n", (int)(ddcAbortReason)));
		}

        if (BIT_CBUS_CMD_ABORT & int1Status)
        {
            // last command sent was not successful
        }

        if ( int1Status & BIT_CBUS_MSC_ABORT_RCVD)  // New :   CBUS_INT_1
        {
            uint8_t mscMtAbortIntStatus   = 0;

            mscMtAbortIntStatus = SiiRegRead(REG_CBUS_MSC_MT_ABORT_INT);
            if (mscMtAbortIntStatus)
            {
                TX_DEBUG_PRINT(("CBUS:: MSC Transfer ABORTED. Clearing 0x0D\n"));
                SiiRegWrite(REG_CBUS_MSC_MT_ABORT_INT,mscMtAbortIntStatus);
                if (BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL & mscMtAbortIntStatus)
                {
                    TX_DEBUG_PRINT(("Requestor MAXFAIL - retry threshold exceeded\n"));
                }
                if (BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR & mscMtAbortIntStatus)
                {
                    TX_DEBUG_PRINT(("Protocol Error\n"));
                }
                if (BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT & mscMtAbortIntStatus)
                {
                    TX_DEBUG_PRINT(("Requestor translation layer timeout\n"));
                }
                if (BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD & mscMtAbortIntStatus)
                {
                    TX_DEBUG_PRINT(("Undefined opcode\n"));
                }
                if (BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT & mscMtAbortIntStatus)
                {
                    TX_DEBUG_PRINT(("CBUS:: MSC Peer sent an ABORT.\n"));
                }
            }
        }
    }
    return( result );
}

void SiiMhlTxDrvGetScratchPad (uint8_t startReg,uint8_t *pData,uint8_t length)
{
uint8_t offset=startReg & 0xF;
uint8_t count = 16-offset;

    TX_DEBUG_PRINT(("Drv: Getting Scratch Pad.\n"));
    while (length > 0)
    {
    	SiiRegReadBlock(REG_CBUS_MHL_SCRPAD_0+offset, pData, count);
        length -=count;
        count = (length > 16)? 16 : length;
    }
}

static uint8_t lastCBusInt0Status=0;
/*
 SiiMhlTxDrvMscMsgNacked
    returns:
        0 - message was not NAK'ed
        non-zero message was NAK'ed
 */
int SiiMhlTxDrvMscMsgNacked()
{
    if (BIT_CBUS_MSC_MT_DONE_NACK & lastCBusInt0Status)
    {

        TX_DEBUG_PRINT(("MSC MSG NAK'ed - retrying...\n\n"));
        return 1;
    }
    return 0;
}
///////////////////////////////////////////////////////////////////////////
//
// MhlCbusIsr
//
// Only when MHL connection has been established. This is where we have the
// first looks on the CBUS incoming commands or returned data bytes for the
// previous outgoing command.
//
// It simply stores the event and allows application to pick up the event
// and respond at leisure.
//
// Look for interrupts on CBUS:CBUS_INTR_STATUS [0xC8:0x08]
//		7 = RSVD			(reserved)
//		6 = MSC_RESP_ABORT	(interested)
//		5 = MSC_REQ_ABORT	(interested)
//		4 = MSC_REQ_DONE	(interested)
//		3 = MSC_MSG_RCVD	(interested)
//		2 = DDC_ABORT		(interested)
//		1 = RSVD			(reserved)
//		0 = rsvd			(reserved)
///////////////////////////////////////////////////////////////////////////
static void MhlCbusIsr (void)
{
uint8_t		cbusInt;
#if 0 //20120718 hammer modify for avoid lose BIT_CBUS_MSC_MR_SET_INT
uint8_t DRC_int[4];// device register change interrupt
#endif
	//
	// Main CBUS interrupts on CBUS_INTR_STATUS
	//
	cbusInt = SiiRegRead(REG_CBUS_INT_0);   // New :   CBUS_INT_0

#if 0 //20120718 hammer modify for avoid lose BIT_CBUS_MSC_MR_SET_INT
	SiiRegReadBlock(REG_CBUS_SET_INT_0, DRC_int, 4);
	 
	if ((DRC_int[0])|(DRC_int[1]))
	{
		SiiRegWriteBlock(REG_CBUS_SET_INT_0, DRC_int, 4);   // New :   MHL_INT_*
		SiiOsDebugPrintAlwaysShort ("cbusInt:%2X,  int0:%02X,  int1:%02X\n",(int)cbusInt, (int) DRC_int[0], (int) DRC_int[1] );         
	}
#endif

	// When I2C is inoperational (D3) and a previous interrupt brought us here, do nothing.
	if (cbusInt == 0xFF)
	{
		return;
	}

	if (cbusInt)
	{
	    lastCBusInt0Status = cbusInt; // save for SiiMhlTxDrvMscMsgNacked()
		//
		// Clear all interrupts that were raised even if we did not process
		//

            SiiRegWrite(REG_CBUS_INT_0, cbusInt); // New :   CBUS_INT_0

            CBUS_DEBUG_PRINT(("Drv: Clear CBUS INTR_1: %02X\n", (int) cbusInt));
#if 0//hammer test
if(SiiRegRead(REG_CBUS_INT_0))
	SiiOsDebugPrintAlwaysShort("@@@1 Drv: REG_CBUS_INT_0 : %02X\n", (int) SiiRegRead(REG_CBUS_INT_0));// hammer test
#endif

        // Downstream HPD handline is high priority
        if (BIT_CBUS_HPD_RCVD & cbusInt)
        {
        uint8_t cbusStatus;
        	//
        	// Check if a SET_HPD came from the downstream device.
        	//
        	cbusStatus = SiiRegRead(REG_CBUS_STATUS);

    	// CBUS_HPD status bit
    	if(BIT_CBUS_HPD & (dsHpdStatus ^ cbusStatus))
    	{
	        uint8_t status = cbusStatus & BIT_CBUS_HPD;
    		TX_DEBUG_PRINT(("Drv: Downstream HPD changed to: %02X\n", (int) cbusStatus));

       		// Inform upper layer of change in Downstream HPD
       		SiiMhlTxNotifyDsHpdChange( status );

            if ( mfdrvTranscodeMode & g_modeFlags )
            {
                if (status)
                {
                    SiiMhlTxDrvReleaseUpstreamHPDControl();  // this triggers an EDID read
                }
            }
                else
                {
                    if (!status)
                    {
                        SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow();
                    }
                }
        		// Remember
        		dsHpdStatus = cbusStatus;
        	}
        }

    	// MSC_REQ_DONE received.
    	if (BIT_CBUS_MSC_MT_DONE & cbusInt)
    	{
    	    CBUS_DEBUG_PRINT(("Drv: MSC_REQ_DONE\n"));

    		mscCmdInProgress = false;

            // only do this after cBusInt interrupts are cleared above
    		SiiMhlTxMscCommandDone( SiiRegRead(REG_CBUS_PRI_RD_DATA_1ST) );    // New :  MSC_MT_RCVD_DATA0
#ifdef DEFERRED_HDCP_START //(
            if (hdcpStartPending)
            {
                hdcpStartPending = false;
                TX_DEBUG_PRINT(("executing deferred HDCP start...\n"));
                SiiToggleTmdsForHdcpAuthentication();
            }
#endif //)
    	}

    	if (BIT_CBUS_MSC_MR_WRITE_STAT & cbusInt)   // New :   CBUS_INT_0
    	{
        uint8_t status[4];
        //uint8_t clear[4]={0xff,0xff,0xff,0xff};// must write 0xFF to clear regardless!
            //re-visit

		// don't put debug output here, it just creates confusion.
	    SiiRegReadBlock(REG_CBUS_WRITE_STAT_0, status, 4);   // New :   MHL_STAT_*

	    //SiiRegWriteBlock(REG_CBUS_WRITE_STAT_0, clear, 4);   // New :   MHL_STAT_*
		SiiMhlTxGotMhlStatus( status[0], status[1] );
	}
	// MSC_MSG (RCP/RAP)
	if ((BIT_CBUS_MSC_MR_MSC_MSG & cbusInt))
	{
    	uint8_t mscMsg[2];
	    TX_DEBUG_PRINT(("Drv: MSC_MSG Received\n"));
		//
		// Two bytes arrive at registers formerly mapped at 0x18 and 0x19
		//
        mscMsg[0] = SiiRegRead(REG_CBUS_MSC_MR_MSC_MSG_RCVD_1ST_DATA);  // New :   MSC_MR_MSC_MSG_RCVD_1ST_DATA
        mscMsg[1] = SiiRegRead(REG_CBUS_MSC_MR_MSC_MSG_RCVD_2ND_DATA);  // New :   MSC_MR_MSC_MSG_RCVD_2ND_DATA

	    TX_DEBUG_PRINT(("Drv: MSC MSG: %02X %02X\n", (int)mscMsg[0], (int)mscMsg[1] ));
		SiiMhlTxGotMhlMscMsg( mscMsg[0], mscMsg[1] );
	}

    if ( BIT_CBUS_MSC_MR_WRITE_BURST & cbusInt)
    {
        // WRITE_BURST complete
        SiiMhlTxMscWriteBurstDone( cbusInt );
    }

	if(BIT_CBUS_MSC_MR_SET_INT & cbusInt)
	{
#if 0 //20120718 hammer modify for avoid lose BIT_CBUS_MSC_MR_SET_INT
           SiiMhlTxGotMhlIntr( DRC_int[0], DRC_int[1] );
#else
    uint8_t intr[4];

	    TX_DEBUG_PRINT(("Drv: MHL INTR Received\n"));
	    SiiRegReadBlock(REG_CBUS_SET_INT_0, intr, 4);   // New :   MHL_INT_*
	    SiiRegWriteBlock(REG_CBUS_SET_INT_0, intr, 4);   // New :   MHL_INT_*

//           ERROR_DEBUG_PRINT(("@@@2 Drv: REG_CBUS_INT_0 : %02x\n", SiiRegRead(REG_CBUS_INT_0))); //hammer test
	    
    	// We are interested only in first two bytes. and the scratch pad index
    	SiiMhlTxGotMhlIntr( intr[0], intr[1] );
#endif
    	}
	}
#if 0 //20120718 hammer modify for avoid lose BIT_CBUS_MSC_MR_SET_INT
else if((DRC_int[0])|(DRC_int[1]))
{
	     SiiOsDebugPrintAlwaysShort ("#### check (DRC_int[0])|(DRC_int[1]) not zero ####");         

           SiiMhlTxGotMhlIntr( DRC_int[0], DRC_int[1] );
}
#endif

    cbusInt =SiiRegRead(REG_CBUS_INT_1);
	if(cbusInt)
	{
		//
		// Clear all interrupts that were raised even if we did not process
		//
		SiiRegWrite(REG_CBUS_INT_1, cbusInt);   // New :   CBUS_INT_0

	    CBUS_DEBUG_PRINT(("Drv: Clear CBUS INTR_2: %02X\n", (int) cbusInt));
    	CBusProcessErrors(cbusInt);
	}

}


///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvPowBitChange
//
// Alert the driver that the peer's POW bit has changed so that it can take
// action if necessary.
//
void SiiMhlTxDrvPowBitChange (bool_t enable)
{
	// MHL peer device has it's own power
	if (enable)
	{
//SII8240_VER81	+++	
		// this does not exist in 8240 SiiRegModify(REG_DISC_CTRL8, 0x04, 0x04);
//SII8240_VER81	---
        TX_DEBUG_PRINT(("Drv: POW bit 0->1, set DISC_CTRL8[2] = 1\n"));
	}
}
/*
	SiMhlTxDrvSetClkMode
	-- Set the hardware this this clock mode.
 */
void SiMhlTxDrvSetClkMode(uint8_t clkMode)
{
	TX_DEBUG_PRINT(("SiMhlTxDrvSetClkMode:0x%02x\n",(int)clkMode));

	// nothing to do here since we only suport MHL_STATUS_CLK_MODE_NORMAL
	// if we supported SUPP_PPIXEL, this would be the place to write the register
}
/*
    SiiMhlTxDrvGetDeviceId
    returns chip Id
 */

uint16_t SiiMhlTxDrvGetDeviceId(void)
{
uint16_t retVal;
    retVal =  SiiRegRead(REG_DEV_IDH);
	retVal <<= 8;
	retVal |= SiiRegRead(REG_DEV_IDL);
    return retVal;
}
/*
    SiiMhlTxDrvGetDeviceRev
    returns chip revision
 */
uint8_t SiiMhlTxDrvGetDeviceRev(void)
{
    return SiiRegRead(REG_DEV_REV);
}


/*
    SiiMhlTxDrvPreInitSetModeFlags
*/
void SiiMhlTxDrvPreInitSetModeFlags(uint8_t flags)
{
    g_modeFlags = flags;
}

#ifdef	EDID_DUMP_8240_BUFFER //(
void	DumpEdidFifoImpl( int blockNumber )
{
    if (SiiOsDebugChannelIsEnabled(SII_OSAL_DEBUG_EDID_DBG))
    {
	uint8_t j;
    uint8_t temp;
        temp = SiiRegRead(REG_EDID_CTRL);
    	SiiRegWrite (REG_EDID_CTRL,							// 2E3 = 11
               BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
               | BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
    	    );


    SiiOsDebugPrintAlwaysShort("\nShowing the 8240 EDID SRAM blockNumber = %02X\n", (int)blockNumber);
	SiiRegWrite (REG_EDID_FIFO_ADDR, (blockNumber << 7));				// 2E9 Starting address of FIFO

	for (j = 0; j < 128; j++)
	{
		if ((j & 0x0F) == 0x00)
		{
            SiiOsDebugPrintAlwaysShort("\n");
		}
        SiiOsDebugPrintAlwaysShort("%02X ", (int) SiiRegRead (REG_EDID_FIFO_RD_DATA) );
	}
    SiiOsDebugPrintAlwaysShort("\n");
        SiiRegWrite(REG_EDID_CTRL,temp);
    }
}
#endif //)


#ifdef EDID_DUMP_SW_BUFFER //(

void DumpEdidBlockImpl1(char *pszFile, int iLineNum,uint8_t *pData,uint16_t length)
{
    if (SiiOsDebugChannelIsEnabled(SII_OSAL_DEBUG_EDID_DBG))
    {
    uint16_t j;
        SiiOsDebugPrintAlwaysShort(pszFile,iLineNum,SII_OSAL_DEBUG_TX, "EDID DATA:");
    	for (j = 0; j < length; j++)
    	{
    		if ((j & 0x0F) == 0x00)
    		{
                SiiOsDebugPrintAlwaysShort("\n");
    		}
            SiiOsDebugPrintAlwaysShort("%02X ", (int) pData[j]);
    	}
        SiiOsDebugPrintAlwaysShort("\n");
    }
}


void DumpEdidBlockImpl(char *pszFile, int iLineNum,uint8_t *pData,uint16_t length)
{
    if (SiiOsDebugChannelIsEnabled(SII_OSAL_DEBUG_EDID_DBG))
    {
    uint16_t j;
        SiiOsDebugPrint(pszFile,iLineNum,SII_OSAL_DEBUG_TX, "EDID DATA:");
    	for (j = 0; j < length; j++)
    	{
    		if ((j & 0x0F) == 0x00)
    		{
                SiiOsDebugPrintAlwaysShort("\n");
    		}
            SiiOsDebugPrintAlwaysShort("%02X ", (int) pData[j]);
    	}
        SiiOsDebugPrintAlwaysShort("\n");
    }
}
#endif //)

#ifdef EDID_CLEAR_HW_BUFFER //(

void ClearEdidBlockImpl(uint8_t *pData)
{
uint8_t j;

	for (j = 0; j < EDID_BLOCK_SIZE; j++)
	{
        pData[j] = j;
	}

#ifdef	EDID_READ_FIFO_BYTE_MODE //(
	{
		int	i;

		EDID_DEBUG_PRINT(("ClearEdidBlockImpl: Copy 128 bytes - byte mode\n"));
		for(i = 0; i < EDID_BLOCK_SIZE; i++)
		{
			SiiRegWrite (REG_EDID_FIFO_WR_DATA, pData[i]);
		}
	}
#else //)(
	SiiRegWriteBlock ( REG_EDID_FIFO_WR_DATA, pData, EDID_BLOCK_SIZE );
#endif //)
}
#endif //)

SiiMhlTxDrvGetEdidBlockResult_e SiiMhlTxDrvGetEdidBlock(uint8_t *pBufEdid,uint8_t blockNumber,uint8_t blockSize)
{

	uint16_t	offset = (blockNumber << 7);
    uint16_t    counter=0;


	EDID_DEBUG_PRINT(("EDID HW Assist: Read EDID block number %02X\n", (int)blockNumber));
//SII8240_VER81	+++
	SiiRegModify (REG_INTR9, BIT_INTR9_EDID_DONE_MASK, BIT_INTR9_EDID_DONE);		// CLEAR 2E0[5]
//SII8240_VER81	---
	// Setup auto increment and kick off read
	SiiRegWrite (REG_EDID_CTRL,							// 2E3 = 11
           BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
           | BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
	    );

	// Setup which block to read
	if( 0 == blockNumber)
	{
//SII8240_VER81	+++	
		SiiRegWrite (REG_TPI_CBUS_START, BIT_TPI_CBUS_START_EDID_READ_BLOCK_0);		// 2E2[1] shared with RCP etc
//SII8240_VER81	---
	}
	else
	{
            uint8_t param= (1 << (blockNumber-1));
		EDID_DEBUG_PRINT(("EDID HW Assist: Programming Reg %02X to %02X\n", (int)REG_EDID_HW_ASSIST_READ_BLOCK_ADDR, (int)(1 << (blockNumber-1)) ));
//MHL_CTS 20120907 +++		
//SII8240_VER81_2 +++ 20120901 Hammer fix edid 		
        //SiiRegWrite (REG_EDID_HW_ASSIST_READ_BLOCK_ADDR, (1 << (blockNumber-1)));         // 2ED
    	SiiRegWrite (REG_EDID_HW_ASSIST_READ_BLOCK_ADDR,param);       // 2ED
//SII8240_VER81_2 --- 20120901 Hammer fix edid
//MHL_CTS 20120907 ---
	}

	EDID_DEBUG_PRINT(("EDID HW Assist: Waiting for Completion\n" ));
	// Wait for completion
//SII8240_VER81	+++	
	for (counter =0; counter < 100;++counter)
	{
    uint8_t temp = SiiRegRead(REG_INTR9);
        if (temp)
        {
    		if( BIT_INTR9_EDID_DONE & temp )	// 2E0[5]
    		{
                // clear the interrupt
                SiiRegWrite(REG_INTR9,BIT_INTR9_EDID_DONE);
    			break;
    		}
            ERROR_DEBUG_PRINT(("intr9 status: %02x\n",(uint16_t)temp));
            if (BIT_INTR9_EDID_ERROR & temp)
            {
                // clear the interrupt
                SiiRegWrite(REG_INTR9,BIT_INTR9_EDID_ERROR);
                ERROR_DEBUG_PRINT(("EDID read error, retrying\n"));
                DumpTPIDebugRegs
            	// Setup which block to read
            	if( 0 == blockNumber)
            	{
            		SiiRegWrite (REG_TPI_CBUS_START, BIT_TPI_CBUS_START_EDID_READ_BLOCK_0);		// 2E2[1] shared with RCP etc
            	}
            	else
            	{
//MHL_CTS 20120907 +++						
                    uint8_t param= (1 << (blockNumber-1));
            		EDID_DEBUG_PRINT(("EDID HW Assist: Programming Reg %02X to %02X\n", (int)REG_EDID_HW_ASSIST_READ_BLOCK_ADDR, (int)(1 << (blockNumber-1)) ));
//SII8240_VER81_2 +++ 20120901 Hammer fix edid 		
                   //SiiRegWrite (REG_EDID_HW_ASSIST_READ_BLOCK_ADDR, (1 << (blockNumber-1)));         // 2ED
                   SiiRegWrite (REG_EDID_HW_ASSIST_READ_BLOCK_ADDR,param);           // 2ED
//SII8240_VER81_2 --- 20120901 Hammer fix edid 		
//MHL_CTS 20120907 ---				   
            	}
            }
        }
  		// wait a bit
  		HalTimerWait(1);
	}
    if (counter >= 100)
    {
     ERROR_DEBUG_PRINT(("EDID HW Assist: Timed Out. counter:%d\n",counter));
        return gebTimedOut;
    }
	EDID_DEBUG_PRINT(("EDID HW Assist: Setting address for read from buffer to %02X counter:%d\n", (int)offset,(uint16_t)counter));
//SII8240_VER81	---
	SiiRegWrite (REG_EDID_FIFO_ADDR, offset);				// 2E9 = Starting address of FIFO

#ifdef	EDID_READ_FIFO_BYTE_MODE //(
	{
		int	i;

		// Copy data into the internal buffer for parsing as we did in the past
		for(i = 0; i < blockSize; i++)
		{
			pBufEdid[ i ] = SiiRegRead (REG_EDID_FIFO_RD_DATA);	// from 2EC
		}
	}
#else //)( #ifdef	EDID_READ_FIFO_BYTE_MODE
	SiiRegReadBlock ( REG_EDID_FIFO_RD_DATA, pBufEdid, blockSize );
#endif //)	EDID_READ_FIFO_BYTE_MODE

	EDID_DEBUG_PRINT(("Done reading EDID from FIFO using HW_ASSIST. It never fails.\n"));
//SII8240_VER81	+++
    DumpEdidFifo(blockNumber);
    DumpEdidFifo(blockNumber+1);
//SII8240_VER81	---
    DumpEdidBlock1(pBufEdid,blockSize)  // no semicolon needed here
	return gebSuccess;
}

void SiiMhlTxDrvSetUpstreamEDID(uint8_t *pEDID,uint16_t length)
{
    DumpEdidBlock(pEDID,length)  // no semicolon needed here

	SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow();
	SiiRegWrite (REG_EDID_FIFO_ADDR, 0);				// 2E9 = 0		// Starting address of FIFO as 0

#ifdef	EDID_READ_FIFO_BYTE_MODE //(
    TX_DEBUG_PRINT(("SiiMhlTxDrvSetUpstreamEDID: Copy %d bytes - byte mode\n",length));
    	{
    		int	i;

    		for(i = 0; i < length;++i)
    		{
    			SiiRegWrite (REG_EDID_FIFO_WR_DATA, pEDID[i]);
    		}
    	}
#else //)(
    TX_DEBUG_PRINT(("SiiMhlTxDrvSetUpstreamEDID: Copy %d bytes - block mode\n",length));
	SiiRegWriteBlock ( REG_EDID_FIFO_WR_DATA, pEDID, length );
#endif //)
	TX_DEBUG_PRINT(("%d bytes written to 8240 upstream EDID SRAM\n",length));

	DumpEdidFifo( 0 ) // no semicolon needed here
	DumpEdidFifo( 1 ) // no semicolon needed here
    SiiRegWrite (REG_EDID_CTRL,
//SII8240_VER75	+++
                BIT_EDID_CTRL_EDID_PRIME_VALID_ENABLE
              | BIT_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
              | BIT_EDID_CTRL_EDID_MODE_EN_ENABLE
//SII8240_VER75	---
        );
    EDID_DEBUG_PRINT(("Open EDID access to the upstream device. 2E3 = %02X\n", (int)SiiRegRead(REG_EDID_CTRL) ));

    // power up the receiver
    SiiRegModify(REG_DPD,BIT_DPD_PDIDCK_MASK,BIT_DPD_PDIDCK_NORMAL_OPERATION);

    // toggle to make sure that the EDID gets read by the upstream device.
    SiiMhlTxDrvAcquireUpstreamHPDControlDriveHigh();

#ifdef CP9687_CHECK_DEVCAP_1
    if (g_pad_tv_mode == MHL_TV_MODE)
    {
    	msleep(500);
    }
#endif

}


#ifdef MDT_STATE_MACHINE //(
#include "queue.h"

typedef enum
{    mdteNoOp
    ,mdteLegacyModeRequest
    ,mdteLegacyModeGrant
    ,mdteMdtRequest
    ,mdteMdtQueueExhausted
    ,mdteREQ_WRT_FromPeer
    ,mdteREQ_WRT
    ,mdteGRT_WRT_FromPeer
    ,mdteGRT_WRT
    ,mdteLegacyRequest
    ,mdteLegacyCommandDone
}MdtEvent_e;

typedef enum
{
     mdtsMdtModeIdle
    ,mdtsMdtModeIdleLegacyRequestPending
    ,mdtsMdtModeActive
    ,mdtsMdtModeActiveLegacyRequestPending
    ,mdtsLegacyModeIdle
    ,mdtsLegacyModeActive
    ,mdtsLegacyModeWriteBurstPending
    ,mdtsLegacyModeWriteBurstActive

    ,mdtsNumStates  // this entry must be last

}MdtState_e;
typedef struct _MdtStateTableEntry_t
{
    MdtEvent_e        event;
    MdtState_e        newState;
    void (*pfnTransitionHandler)(void);  // note that the size of this structure, on an 8051, should be exactly 4 bytes.
}MdtStateTableEntry_t,*PMdtStateTableEntry_t;

typedef struct _MdtStateTableRowHeader_t
{
    uint8_t numValidEvents;
    void (*pfnEventGatherer)(void);  // note that the size of this structure, on an 8051, should be exactly 4 bytes.
    PMdtStateTableEntry_t  pStateRow;
}MdtStateTableRowHeader_t,*PMdtStateTableRowHeader_t;

#define NUM_TX_EVENT_QUEUE_EVENTS 8
typedef struct _MdtEventQueue_t
{
    QueueHeader_t header;
    MdtEvent_e queue[NUM_TX_EVENT_QUEUE_EVENTS];
}MdtEventQueue_t,*PMdtEventQueue_t;

MdtEventQueue_t MdtEventQueue={0,0,{0}};

MdtEvent_e GetNextMdtEvent()
{
    if (0==QUEUE_DEPTH(MdtEventQueue))
    {
        return mdteNoOp;
    }
    else
    {
    MdtEvent_e retVal;
        retVal = MdtEventQueue.queue[MdtEventQueue.header.head];
        ADVANCE_QUEUE_HEAD(MdtEventQueue)
        return retVal;
    }
}



bool_t PutNextMdtEventImpl(MdtEvent_e event)
{
    if (QUEUE_FULL(MdtEventQueue))
    {
        //queue is full
        return false;
    }
    // at least one slot available
    MdtEventQueue.queue[MdtEventQueue.header.tail] = event;
    ADVANCE_QUEUE_TAIL(MdtEventQueue)
    return true;
}
// use this wrapper to do debugging output for the routine above.
bool_t PutNextMdtEventWrapper(MdtEvent_e event,char *pszEvent,int iLine)
{
	bool_t retVal;

    TX_DEBUG_PRINT(("PutNextMdtEventWrapper: called from line:%d event: %s(%d) depth:%d head: %d tail:%d\n"
                ,iLine
                ,pszEvent
                ,(int)event
                ,(int)QUEUE_DEPTH(MdtEventQueue)
                ,(int)MdtEventQueue.header.head
                ,(int)MdtEventQueue.header.tail
                ));
    retVal = PutNextMdtEventImpl(event);

    if (!retVal)
    {
        TX_DEBUG_PRINT(("queue full, when adding event %d called from line:%d\n",(int)event,iLine));
    }
    return retVal;
}
#define PutNextTxEvent(event) PutNextTxEventWrapper(event,#event,__LINE__);


void MdtModeIdleToMdtModeActiveViaMdtRequest(void)
{
}

void MdtModeIdleToMdtModeIdleLegacyRequestPendingViaLegacyModeRequest(void)
{
}

void MdtModeIdleLegacyRequestPendingToLegacyModeIdleViaLegacyModeRequest(void)
{
}

void MdtModeIdleLegacyRequestPendingToLegacyModeIdleViaLegacyModeGrant(void)
{
}

void MdtModeActiveToMdtModeIdleViaMdtQueueExhausted(void)
{
}

void MdtModeActiveToMdtModeActiveLegacyRequestPendingViaLegacyModeRequest(void)
{
}

void MdtModeActiveLegacyRequestPendingToLegacyModeIdleViaLegacyModeGrant(void)
{
}

void mdteMdtModeRequest(void)
{
}

void LegacyModeIdleToMdtModeIdleViaMdtModeRequest(void)
{
}

void LegacyModeIdleToLegacyModeWriteBurstPendingViaREQ_WRT_FromPeer(void)
{
}

void LegacyModeIdleToLegacyModeActiveViaLegacyRequest(void)
{
}

void LegacyModeActiveToLegacyModeIdleViaLegacyCommandDone(void)
{
}

void mdtsLegacyWriteBurstActive(void)
{
}

void LegacyModeWriteBurstPendingToLegacyWriteBurstActiveViaGRT_WRT(void)
{
}

void LegacyModeWriteBurstActiveToLegacyModeIdleViaLegacyCommandDone(void)
{
}

void GathererMdtModeIdle(void)
{
}

void GathererMdtModeIdleLegacyRequestPending(void)
{
}

void GathererMdtModeActive(void)
{
}

void GathererMdtModeActiveLegacyRequestPending(void)
{
}

void GathererLegacyModeIdle(void)
{
}

void GathererLegacyModeActive(void)
{
}

void GathererLegacyModeWriteBurstPending(void)
{
}

void GathererLegacyModeWriteBurstActive(void)
{
}


#define StateTransitionEntry(state,event,newState) {mdte##event,mdts##newState,state##To##newState##Via##event}
PLACE_IN_CODE_SEG MdtStateTableEntry_t  mdtste_MdtModeIdle[] =
{
     StateTransitionEntry(MdtModeIdle,MdtRequest,MdtModeActive)
    ,StateTransitionEntry(MdtModeIdle,LegacyModeRequest,MdtModeIdleLegacyRequestPending)
};

PLACE_IN_CODE_SEG MdtStateTableEntry_t mdtste_MdtModeIdleLegacyRequestPending[] =
{
     StateTransitionEntry(MdtModeIdleLegacyRequestPending,LegacyModeGrant,LegacyModeIdle)
};

PLACE_IN_CODE_SEG MdtStateTableEntry_t mdtste_MdtModeActive[] =
{
     StateTransitionEntry(MdtModeActive,MdtQueueExhausted,MdtModeIdle)
    ,StateTransitionEntry(MdtModeActive,LegacyModeRequest,MdtModeActiveLegacyRequestPending)
};

PLACE_IN_CODE_SEG MdtStateTableEntry_t mdtste_MdtModeActiveLegacyRequestPending[] =
{
     StateTransitionEntry(MdtModeActiveLegacyRequestPending,LegacyModeGrant,LegacyModeIdle)
};

PLACE_IN_CODE_SEG MdtStateTableEntry_t mdtste_LegacyModeIdle[] =
{
     StateTransitionEntry(LegacyModeIdle,MdtModeRequest,MdtModeIdle)
    ,StateTransitionEntry(LegacyModeIdle,REQ_WRT_FromPeer,LegacyModeWriteBurstPending)
    ,StateTransitionEntry(LegacyModeIdle,LegacyRequest,LegacyModeActive)
};

PLACE_IN_CODE_SEG MdtStateTableEntry_t mdtste_LegacyModeActive[] =
{
     StateTransitionEntry(LegacyModeActive,LegacyCommandDone,LegacyModeIdle)
};

PLACE_IN_CODE_SEG MdtStateTableEntry_t mdtste_LegacyModeWriteBurstPending[] =
{
     StateTransitionEntry(LegacyModeWriteBurstPending,GRT_WRT,LegacyWriteBurstActive)
};

PLACE_IN_CODE_SEG MdtStateTableEntry_t mdtste_LegacyModeWriteBurstActive[] =
{
     StateTransitionEntry(LegacyModeWriteBurstActive,LegacyCommandDone,LegacyModeIdle)
};

// The order of the following table is CRITICAL.
//  Do NOT insert anything without a corresponding insertion into MdtState_e
//   See statetable.h for the definition of MdtState_e
PLACE_IN_CODE_SEG MdtStateTableRowHeader_t MdtStateTransitionAndResponseTable[mdtsNumStates]=
{
#define MDT_RowEntry(state)  {sizeof(mdtste_##state)/sizeof(mdtste_##state[0]), Gatherer##state, mdtste_##state }
      MDT_RowEntry(MdtModeIdle)
    , MDT_RowEntry(MdtModeIdleLegacyRequestPending)
    , MDT_RowEntry(MdtModeActive)
    , MDT_RowEntry(MdtModeActiveLegacyRequestPending)
    , MDT_RowEntry(LegacyModeIdle)
    , MDT_RowEntry(LegacyModeActive)
    , MDT_RowEntry(LegacyModeWriteBurstPending)
    , MDT_RowEntry(LegacyModeWriteBurstActive)
};

PMdtStateTableEntry_t LookupNextState(MdtState_e state, uint8_t event)
{
PMdtStateTableEntry_t pMdtStateTableEntry = MdtStateTransitionAndResponseTable[state].pStateRow;
uint8_t i,limit;
    limit = MdtStateTransitionAndResponseTable[state].numValidEvents;
    for (i=0; i < limit;++i)
    {
        if (event == pMdtStateTableEntry->event)
        {
            TX_DEBUG_PRINT(("lookup state:%d event:%d\n",(int)state,(int)event));

            return pMdtStateTableEntry;
        }
        pMdtStateTableEntry++;
    }
    TX_DEBUG_PRINT(("lookup failed. event:%d\n",(int)event));
    return NULL;
}
#endif //)


