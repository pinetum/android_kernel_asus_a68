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



/**
 * @file mhl_linuxdrv_main.c
 *
 * @brief Main entry point of the Linux driver for Silicon Image MHL transmitters.
 *
 * $Author: Dave Canfield
 * $Rev: $
 * $Date: Jan. 20, 2011
 *
 *****************************************************************************/

#define MHL_LINUXDRV_MAIN_C

/***** #include statements ***************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include "si_c99support.h"
#include "si_common.h"
#include "mhl_linuxdrv.h"
#include "osal/include/osal.h"
#include "si_mhl_defs.h"  //TB - added
#include "si_mhl_tx_api.h"
#include "si_drvisrconfig.h"
#include "si_cra.h"
#include "si_osdebug.h"
#include "si_hdmi_tx_lite_api.h"   //TB : Added 3/9/12 : 2:00 pm

// for GPIO debug use +++
#include <linux/gpio.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
// for GPIO debug use ---
#include "sii_hal_priv.h"

#ifdef SII8240_PM_SUPPORT
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif
#include "si_cra_cfg.h"
#include "si_8240_regs.h"
#include "si_tpi_regs.h"


//TB  - added  {
// EDID_SNOOPING
void	SiiDrvSetPackedPixel( int supportPackedPixel );
void	SiiDrvClearDcapEdidFlags( void );
//TB }

/***** local macro definitions ***********************************************/
#define GPIO_CONFIG_DBG(gpio)         (MSM_TLMM_BASE + 0x1000 + (0x10 * (gpio)))
#define GPIO_IN_OUT_DBG(gpio)         (MSM_TLMM_BASE + 0x1004 + (0x10 * (gpio)))


/***** local variable declarations *******************************************/
static int32_t devMajor = 0;    /**< default device major number */
static struct cdev siiMhlCdev;
static struct class *siiMhlClass;
static char *buildTime = "Build: " __DATE__"-" __TIME__ "\n";
static char *buildVersion = "1.00.";

//static struct device *siiMhlClassDevice;
static int pad_tv_mode = MHL_PAD_MODE_L;  // default Pad mode
//static int pad_tv_mode = MHL_PAD_MODE_H;  // default Pad mode
static int p03_plug_in = MHL_DETECT_FROM_P03_PLUG;
//static int p03_plug_in = MHL_DETECT_FROM_PARAMETER;
#ifdef SII8240_PM_SUPPORT
//static int mhl_cbus_enable = 1;  // default enable cbus enable
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
int g_mhl_early_suspend_flag = 0;
#endif

int mhl_reset_set = 0;

static int mhl_swing_base_value = 0;  // default enable cbus enable

//ASUS_BSP +++: larry lai : for by-pass check QCOM TMDS H V total while booting in PAD mode
static int P03_QcomTmdsHVTotalByPassCheck = 1;
//ASUS_BSP ---: larry lai : for by-pass check QCOM TMDS H V total while booting in PAD mode
\
static int pad_tv_mode_set(const char *val, struct kernel_param *kp);
static int p03_plug_in_set(const char *val, struct kernel_param *kp);
#ifdef SII8240_PM_SUPPORT
//static int mhl_cbus_set(const char *val, struct kernel_param *kp);
#endif
static int mhl_swing_base_set(const char *val, struct kernel_param *kp);
static int mhl_reset_function(const char *val, struct kernel_param *kp);

int get_p03_plug_in(void);

extern bool asus_padstation_exist(void);
extern bool asus_padstation_exist_realtime(void);

/***** global variable declarations *******************************************/

MHL_DRIVER_CONTEXT_T gDriverContext;


//#if defined(DEBUG)
#if 1
unsigned char DebugChannelMasks[SII_OSAL_DEBUG_NUM_CHANNELS/8+1]=
{
	 0xFF,0xFF,0xFF,0xFF
	,0xFF,0xFF,0xFF,0xFF
	,0xFF,0xFF,0xFF,0xFF
	,0xFF,0xFF
};
//ulong DebugChannelMask = 0xFFFFFFFF;
module_param_array(DebugChannelMasks, byte, NULL, S_IRUGO | S_IWUSR);

ushort DebugFormat = SII_OS_DEBUG_FORMAT_FILEINFO;
module_param(DebugFormat, ushort, S_IRUGO | S_IWUSR);
#endif

module_param_call(pad_tv_mode, pad_tv_mode_set, param_get_int,
			&pad_tv_mode, 0644);

module_param_call(p03_plug_in, p03_plug_in_set, param_get_int,
			&p03_plug_in, 0644);

module_param_call(mhl_reset_set, mhl_reset_function, param_get_int,
			&mhl_reset_set, 0644);

#ifdef SII8240_PM_SUPPORT
//module_param_call(mhl_cbus_enable, mhl_cbus_set, param_get_int,
//			&mhl_cbus_enable, 0644);
#endif

module_param_call(mhl_swing_base_value, mhl_swing_base_set, param_get_int,
			&mhl_swing_base_value, 0644);


/***** local functions *******************************************************/
static int pad_tv_mode_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = pad_tv_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If tv mode is great than 2, ignore. */
	if (pad_tv_mode > MHL_CARKIT) {
		pad_tv_mode = old_val;
		return -EINVAL;
	}
/*
	printk("+++gMHL_USB_ID_DETECT config = 0x%x\n",  __raw_readl(GPIO_CONFIG_DBG(MHL_USB_ID_DETECT)) );
	printk("+++GPIO_IN_OUT_DBG config = 0x%x\n",  __raw_readl(GPIO_IN_OUT_DBG(MHL_USB_ID_DETECT)) );

	printk("+++++ MHL_USB_ID_DETECT= %d, set to %d\n", gpio_get_value(MHL_USB_ID_DETECT), (*val & 0x01));			

	__raw_writel(0x2C0, GPIO_CONFIG_DBG(MHL_USB_ID_DETECT));
      gpio_set_value(MHL_USB_ID_DETECT, (*val & 0x01) );

	printk("----gMHL_USB_ID_DETECT config = 0x%x\n",  __raw_readl(GPIO_CONFIG_DBG(MHL_USB_ID_DETECT)) );
	printk("---GPIO_IN_OUT_DBG config = 0x%x\n",  __raw_readl(GPIO_IN_OUT_DBG(MHL_USB_ID_DETECT)) );
	printk("----- MHL_USB_ID_DETECT= %d\n", gpio_get_value(MHL_USB_ID_DETECT));			
*/	
	printk("MHL pad_tv_mode_set :0x%x\n", pad_tv_mode);

	return 0;
}


static int mhl_swing_base_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = mhl_swing_base_value;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If tv mode is great than 2, ignore. */
	if (mhl_swing_base_value > 0xf) {
		mhl_swing_base_value = old_val;
		return -EINVAL;
	}
    SiiRegWrite(REG_MHLTX_CTL7, mhl_swing_base_value);	


	printk("MHL mhl_swing_base_value :0x%x\n", mhl_swing_base_value);

	return 0;

}



#ifdef SII8240_PM_SUPPORT
#if 0
static int mhl_cbus_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = mhl_cbus_enable;
	bool enable = 0;
	
	if (AX_MicroP_IsP01Connected() && asus_padstation_exist_realtime()) {

		ret = param_set_int(val, kp);

		if (ret)
			return ret;

		/* If download_mode is not zero or one, ignore. */
		if (mhl_cbus_enable >> 1) {
			mhl_cbus_enable = old_val;
			return -EINVAL;
		}

		printk("MHL mhl_cbus_enable :0x%x\n", mhl_cbus_enable);

		if (mhl_cbus_enable)
			enable = 1;
		else
			enable = 0;

		ret = AX_MicroP_setGPIOOutputPin(OUT_uP_MHL_CBUS_EN, enable);
		if (ret < 0) {
			printk( "fail to set mhl cbus enable (%d)(%d)\n", enable, ret);
		} else {
			printk("set mhl cbus enable (%d)\n", enable);
		}
	} else {
		printk("not in pad, skip set mhl cbus enable control! (%d)(%d)\n", enable, AX_MicroP_IsP01Connected());
	}	


	return 0;
}
#endif

#endif

#ifdef CONFIG_EEPROM_NUVOTON
//extern int AX_IsPadUsing_MHL_H(void);
#endif

//ASUS_BSP +++: larry lai : for by-pass check QCOM TMDS H V total while booting in PAD mode
bool_t MHL_Check_QcomTmdsHVTotal(void)
{
	if (P03_QcomTmdsHVTotalByPassCheck)
	{
		P03_QcomTmdsHVTotalByPassCheck = 0;
		printk("[MHL] initial by pass Check QcomTmds\n");
		return false;
	}

	return true;		
}

void MHL_Assert_Check_QcomTmdsHVTotal(void)
{
		P03_QcomTmdsHVTotalByPassCheck = 0;
		printk("[MHL] assert check QcomTmds\n");
}

//ASUS_BSP ---: larry lai : for by-pass check QCOM TMDS H V total while booting in PAD mode

int get_pad_tv_mode(void)
{
	if (get_p03_plug_in() ==MHL_DETECT_FROM_P03_PLUG)  // check from P03 plug-in
	{
//	    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,"get_pad_tv_mode : check from HDMI\n");

		printk("get_pad_tv_mode : HPD GPIO_CONFIG = 0x%x\n",  __raw_readl(GPIO_CONFIG_DBG(72)) );
		printk("get_pad_tv_mode : HPD value = %d\n", gpio_get_value(72));			
		printk("get_pad_tv_mode : check from HDMI\n");	

		if (asus_padstation_exist_realtime())	
		{
#ifdef CONFIG_EEPROM_NUVOTON		
//			if (AX_IsPadUsing_MHL_H())
			if (0)  // always return PAD LOW
				return MHL_PAD_MODE_H;  // pad mode High
			else	
				return MHL_PAD_MODE_L;  // pad mode Low
#else			
			return pad_tv_mode;
#endif				
		}
		else
		{
			return MHL_TV_MODE;  // TV mode
		}
	}
	else
	{
//	    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,"get_pad_tv_mode : check from parameters\n\n");

		printk("get_pad_tv_mode : HPD GPIO_CONFIG = 0x%x\n",  __raw_readl(GPIO_CONFIG_DBG(72)) );
		printk("get_pad_tv_mode : HPD value = %d\n", gpio_get_value(72));
		printk("get_pad_tv_mode : check from parameters\n");

		return pad_tv_mode;		
	}
}	

static int p03_plug_in_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = p03_plug_in;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (p03_plug_in >> 1) {
		p03_plug_in = old_val;
		return -EINVAL;
	}

	printk("MHL p03_plug_in :0x%x\n", p03_plug_in);
	return 0;
}

int get_p03_plug_in(void)
{
	return p03_plug_in;
}	

static int mhl_reset_function(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = mhl_reset_set;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not zero or one, ignore. */
	if (mhl_reset_set > 0xf)  {
		mhl_reset_set = old_val;
		return -EINVAL;
	}

	switch (mhl_reset_set)
	{
		case 0:
			printk("HalGpioSetTxResetPin high +++\n");
			HalGpioSetTxResetPin(HIGH);
			printk("HalGpioSetTxResetPin high ---\n");
			break;
		case 1:
			printk("HalGpioSetTxResetPin Low +++\n");
			HalGpioSetTxResetPin(LOW);
			printk("HalGpioSetTxResetPin Low ---\n");
			break;
		case 2:
			SiiMhlTxInitialize(EVENT_POLL_INTERVAL_30_MS);
			break;
		case 3:
			printk("[MHL] REG_PKT_FILTER_0 =0xA5 --> 0xA1\n");			
			SiiRegWrite(REG_PKT_FILTER_0, 0xA5);
			mdelay(50);
			SiiRegWrite(REG_PKT_FILTER_0, 0xA1);						
			break;
		case 4:	
			printk("[MHL] REG_PKT_FILTER_0 =0xA1\n");			
			SiiRegWrite(REG_PKT_FILTER_0, 0xA1);			
			break;
		case 5:	
			printk("[MHL] REG_PKT_FILTER_0 =0xA5\n");			
			SiiRegWrite(REG_PKT_FILTER_0, 0xA5);			
			break;
			
		default:	
			break;
	}
	printk("MHL mhl_reset_set :0x%x\n", mhl_reset_set);
	return 0;
}



//ASUS_BSP +++: larry : for VD suspend/Voff test
#ifdef CONFIG_HAS_EARLYSUSPEND

extern void toggletxoutput_on(void);
extern void toggletxoutput_off(void);
extern void MhlTxDisconnect(void);
extern bool asus_padstation_exist_realtime(void);

void sii8240drv_wakelock(void)
{
    	printk("[mhl] +++ sii8240drv_wakelock\n");

	wake_lock(&gDriverContext.mhl_wlock);	

    	printk("[mhl] --- sii8240drv_wakelock\n");
	
}

void sii8240drv_wakeunlock(void)
{
    	printk("[mhl] +++ sii8240drv unlock\n");

	wake_unlock(&gDriverContext.mhl_wlock);	

    	printk("[mhl] --- sii8240drv unlock\n");
	
}
	
static void sii8240drv_early_suspend(struct early_suspend *handler)
{

#if 1
	printk("[mhl] ++sii8240drv_early_suspend\n");

//joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
	if (!AX_MicroP_IsP01Connected() || !asus_padstation_exist_realtime()) {
		printk("[MHL] early_suspend, not in pad, skip DISABLE mhl cbus \n");
		toggletxoutput_off();			
	}
#endif //#ifdef CONFIG_EEPROM_NUVOTON
//joe1_--

#else
	printk("[mhl] ++sii8240drv_early_suspend\n");

//joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
	if (AX_MicroP_IsP01Connected() && asus_padstation_exist_realtime()) 
	{
		printk("[MHL] early_suspend in pad mode\n");			
	}
	else
	{		
		printk("[MHL] early_suspend toggletxoutput_off \n");
		toggletxoutput_off();			
	}
#endif //#ifdef CONFIG_EEPROM_NUVOTON
//joe1_--

#endif
	g_mhl_early_suspend_flag = 1;

    printk("[mhl] --sii8240drv_early_suspend\n");
}


static void sii8240drv_late_resume(struct early_suspend *handler)
{	
#if 1
    	printk("[mhl] ++sii8240drv_late_resume\n");

//joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
	if (!AX_MicroP_IsP01Connected() || !asus_padstation_exist_realtime()) {
		printk("[MHL] late_resume, not in pad, skip ENABLE mhl cbus \n");
		toggletxoutput_on();	
	}
#endif //#ifdef CONFIG_EEPROM_NUVOTON
//joe1_--

#else
	printk("[mhl] ++sii8240drv_late_resume\n");

//joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
	if (AX_MicroP_IsP01Connected() && asus_padstation_exist_realtime()) 
	{
		printk("[MHL] late_resume in pad mode\n");			
	}
	else
	{
		printk("[MHL] sii8240drv_late_resume toggletxoutput_on \n");
		toggletxoutput_on();			
	}
#endif //#ifdef CONFIG_EEPROM_NUVOTON
//joe1_--

#endif
	g_mhl_early_suspend_flag = 0;		

    printk("[mhl] --sii8240drv_late_resume\n");
}


static struct early_suspend sii8240drv_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = sii8240drv_early_suspend,
    .resume = sii8240drv_late_resume,
};
#endif

extern int CarKitInitialize(void);

//ASUS_BSP ---: larry : for VD suspend/Voff test

/**
 *  @brief Start the MHL transmitter device
 *
 *  This function is called during driver startup to initialize control of the
 *  MHL transmitter device by the driver.
 *
 *  @return     0 if successful, negative error code otherwise
 *
 *****************************************************************************/
int32_t StartMhlTxDevice(void)
{
	halReturn_t		halStatus;
	SiiOsStatus_t	osalStatus;

    pr_info("Starting %s\n", MHL_PART_NAME);

    // Initialize the Common Register Access (CRA) layer.
    if(!SiiCraInitialize())
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Initialization of CRA layer failed!\n");
    	return -EIO;
    }

    // Initialize the OS Abstraction Layer (OSAL) support.
    osalStatus = SiiOsInit(0);
    if (osalStatus != SII_OS_STATUS_SUCCESS)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"Initialization of OSAL failed, error code: %d\n",osalStatus);
    	return -EIO;
    }


    halStatus = HalInit();
    if (halStatus != HAL_RET_SUCCESS)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"Initialization of HAL failed, error code: %d\n",halStatus);
    	SiiOsTerm();
    	return -EIO;
    }

    halStatus = HalOpenI2cDevice(MHL_PART_NAME, MHL_DRIVER_NAME);
    if (halStatus != HAL_RET_SUCCESS)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"Opening of I2c device %s failed, error code: %d\n",
    			MHL_PART_NAME, halStatus);
    	HalTerm();
    	SiiOsTerm();
    	return -EIO;
    }

    halStatus = HalInstallIrqHandler(SiiMhlTxDeviceIsr);
    if (halStatus != HAL_RET_SUCCESS)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"Initialization of HAL interrupt support failed, error code: %d\n",
    			halStatus);
    	HalCloseI2cDevice();
    	HalTerm();
    	SiiOsTerm();
    	return -EIO;
    }

    /* Initialize the MHL Tx code a polling interval of 30ms. */
	HalAcquireIsrLock();
	SiiMhlTxInitialize(EVENT_POLL_INTERVAL_30_MS);
    HalReleaseIsrLock();

	CarKitInitialize();
//ASUS_BSP +++: larry : for VD suspend/Voff test
#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend( &sii8240drv_early_suspend_desc );
#endif
//ASUS_BSP ---: larry : for VD suspend/Voff test
    wake_lock_init(&gDriverContext.mhl_wlock, WAKE_LOCK_SUSPEND, "mhl_tx_lock");

    pr_info(" ----- Starting %s ----\n", MHL_PART_NAME);

    return 0;
}



/**
 *  @brief Stop the MHL transmitter device
 *
 *  This function shuts down control of the transmitter device so that
 *  the driver can exit
 *
 *  @return     0 if successful, negative error code otherwise
 *
 *****************************************************************************/
int32_t StopMhlTxDevice(void)
{
	halReturn_t		halStatus;

	pr_info("Stopping %s\n", MHL_PART_NAME);

	HalRemoveIrqHandler();

	halStatus = HalCloseI2cDevice();
    if (halStatus != HAL_RET_SUCCESS)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"Closing of I2c device failed, error code: %d\n",halStatus);
    	return -EIO;
    }

	halStatus = HalTerm();
    if (halStatus != HAL_RET_SUCCESS)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"Termination of HAL failed, error code: %d\n",halStatus);
    	return -EIO;
    }

	SiiOsTerm();
	return 0;
}




/***** public functions ******************************************************/

/**
 * @brief Handle read request to the connection_state attribute file.
 */
ssize_t ShowConnectionState(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	if(gDriverContext.flags & MHL_STATE_FLAG_CONNECTED) {
		return scnprintf(buf, PAGE_SIZE, "connected %s_ready",
				gDriverContext.flags & MHL_STATE_FLAG_RCP_READY? "rcp" : "not_rcp");
	} else {
		return scnprintf(buf, PAGE_SIZE, "not connected");
	}
}


/**
 * @brief Handle read request to the rcp_keycode attribute file.
 */
ssize_t ShowRcp(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int		status = 0;

	if(HalAcquireIsrLock() != HAL_RET_SUCCESS)
	{
		return -ERESTARTSYS;
	}

	if(gDriverContext.flags &
		(MHL_STATE_FLAG_RCP_SENT | MHL_STATE_FLAG_RCP_RECEIVED))
	{
		status = scnprintf(buf, PAGE_SIZE, "0x%02x %s",
				gDriverContext.keyCode,
				gDriverContext.flags & MHL_STATE_FLAG_RCP_SENT? "sent" : "received");
	}

	HalReleaseIsrLock();
	return status;
}



/**
 * @brief Handle write request to the rcp_keycode attribute file.
 */
ssize_t SendRcp(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long	keyCode;
	int				status = -EINVAL;

	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "SendRcp received string: ""%s""\n", buf);

	if(HalAcquireIsrLock() != HAL_RET_SUCCESS)
	{
		return -ERESTARTSYS;
	}

	while(gDriverContext.flags & MHL_STATE_FLAG_RCP_READY) {

		if(strict_strtoul(buf, 0, &keyCode)) {
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Unable to convert keycode string\n");
			break;
		}

		if(keyCode >= 0xFE) {
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
					"keycode (0x%x) is too large to be valid\n", keyCode);
			break;
		}

		gDriverContext.flags &= ~(MHL_STATE_FLAG_RCP_RECEIVED |
								  MHL_STATE_FLAG_RCP_ACK |
								  MHL_STATE_FLAG_RCP_NAK);
		gDriverContext.flags |= MHL_STATE_FLAG_RCP_SENT;
		gDriverContext.keyCode = (uint8_t)keyCode;
		SiiMhlTxRcpSend((uint8_t)keyCode);
		status = count;
		break;
	}

	HalReleaseIsrLock();

	return status;
}


/**
 * @brief Handle write request to the rcp_ack attribute file.
 *
 * This file is used to send either an ACK or NAK for a received
 * Remote Control Protocol (RCP) key code.
 *
 * The format of the string in buf must be:
 * 	"keycode=<keyvalue> errorcode=<errorvalue>
 * 	where:	<keyvalue>		is replaced with value of the RCP to be ACK'd or NAK'd
 * 			<errorvalue>	0 if the RCP key code is to be ACK'd
 * 							non-zero error code if the RCP key code is to be NAK'd
 */
ssize_t SendRcpAck(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long	keyCode = 0x100;	// initialize with invalid values
	unsigned long	errCode = 0x100;
	char			*pStr;
	int				status = -EINVAL;

	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "SendRcpAck received string: ""%s""\n", buf);

	// Parse the input string and extract the RCP key code and error code
	do {
		pStr = strstr(buf, "keycode=");
		if(pStr != NULL) {
			if(strict_strtoul(pStr + 8, 0, &keyCode)) {
				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Unable to convert keycode string\n");
				break;
			}
		} else {
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Invalid string format, can't "\
							"find ""keycode"" value\n");
			break;
		}

		pStr = strstr(buf, "errorcode=");
		if(pStr != NULL) {
			if(strict_strtoul(pStr + 10, 0, &errCode)) {
				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Unable to convert errorcode string\n");
				break;
			}
		} else {
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Invalid string format, can't "\
							"find ""errorcode"" value\n");
			break;
		}
	} while(false);

	if((keyCode > 0xFF) || (errCode > 0xFF)) {
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Invalid key code or error code "\
						"specified, key code: 0x%02x  error code: 0x%02x\n",
						keyCode, errCode);
		return status;
	}

	if(HalAcquireIsrLock() != HAL_RET_SUCCESS)
	{
		return -ERESTARTSYS;
	}

	while(gDriverContext.flags & MHL_STATE_FLAG_RCP_READY) {

		if((keyCode != gDriverContext.keyCode)
			|| !(gDriverContext.flags & MHL_STATE_FLAG_RCP_RECEIVED)) {

			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
					"Attempting to ACK a key code that was not received!\n");
			break;
		}

		if(errCode == 0) {
			SiiMhlTxRcpkSend((uint8_t)keyCode);
		} else {
			SiiMhlTxRcpeSend((uint8_t)errCode);
		}

		status = count;
		break;
	}

	HalReleaseIsrLock();

	return status;
}



/**
 * @brief Handle read request to the rcp_ack attribute file.
 *
 * Reads from this file return a string detailing the last RCP
 * ACK or NAK received by the driver.
 *
 * The format of the string returned in buf is:
 * 	"keycode=<keyvalue> errorcode=<errorvalue>
 * 	where:	<keyvalue>		is replaced with value of the RCP key code for which
 * 							an ACK or NAK has been received.
 * 			<errorvalue>	0 if the last RCP key code was ACK'd or
 * 							non-zero error code if the RCP key code was NAK'd
 */
ssize_t ShowRcpAck(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	int				status = -EINVAL;

	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "ShowRcpAck called\n");

	if(HalAcquireIsrLock() != HAL_RET_SUCCESS)
	{
		return -ERESTARTSYS;
	}

	if(gDriverContext.flags & (MHL_STATE_FLAG_RCP_ACK | MHL_STATE_FLAG_RCP_NAK)) {

		status = scnprintf(buf, PAGE_SIZE, "keycode=0x%02x errorcode=0x%02x",
				gDriverContext.keyCode, gDriverContext.errCode);
	}

	HalReleaseIsrLock();

	return status;
}



/**
 * @brief Handle write request to the devcap attribute file.
 *
 * Writes to the devcap file are done to set the offset of a particular
 * Device Capabilities register to be returned by a subsequent read
 * from this file.
 *
 * All we need to do is validate the specified offset and if valid
 * save it for later use.
 */
ssize_t SelectDevCap(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned long	devCapOffset;
	int				status = -EINVAL;

	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "SelectDevCap received string: ""%s""\n", buf);

	do {

		if(strict_strtoul(buf, 0, &devCapOffset)) {
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
							"Unable to convert register offset string\n");
			break;
		}

		if(devCapOffset >= 0x0F) {
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
					"dev cap offset (0x%x) is too large to be valid\n",
					devCapOffset);
			break;
		}

		gDriverContext.devCapOffset = (uint8_t)devCapOffset;

		status = count;

	} while(false);

	return status;
}



/**
 * @brief Handle read request to the devcap attribute file.
 *
 * Reads from this file return the hexadecimal string value of the last
 * Device Capability register offset written to this file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 *
 * The format of the string returned in buf is:
 * 	"offset:<offset>=<regvalue>
 * 	where:	<offset>	is the last Device Capability register offset
 * 						written to this file
 * 			<regvalue>	the currentl value of the Device Capability register
 * 						specified in offset
 */
ssize_t ReadDevCap(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	uint8_t		regValue;
	int			status = -EINVAL;

	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "ReadDevCap called\n");

	if(HalAcquireIsrLock() != HAL_RET_SUCCESS)
	{
		return -ERESTARTSYS;
	}

	do {
		if(gDriverContext.flags & MHL_STATE_FLAG_CONNECTED) {

			status = SiiTxGetPeerDevCapEntry(gDriverContext.devCapOffset,
											 &regValue);
			if(status != 0) {
				// Driver is busy and cannot provide the requested DEVCAP
				// register value right now so inform caller they need to
				// try again later.
				status = -EAGAIN;
				break;
			}
			status = scnprintf(buf, PAGE_SIZE, "offset:0x%02x=0x%02x",
								gDriverContext.devCapOffset, regValue);
		}
	} while(false);

	HalReleaseIsrLock();

	return status;
}

#define MAX_EVENT_STRING_LEN 40
/*****************************************************************************/
/**
 * @brief Handler for MHL hot plug detect status change notifications
 *  from the MhlTx layer.
 *
 *****************************************************************************/
void  AppNotifyMhlDownStreamHPDStatusChange(bool_t connected)
{
	char	event_string[MAX_EVENT_STRING_LEN];
	char	*envp[] = {event_string, NULL};


	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
			"AppNotifyMhlDownStreamHPDStatusChange called, "\
			"HPD status is: %s\n", connected? "CONNECTED" : "NOT CONNECTED");

	snprintf(event_string, MAX_EVENT_STRING_LEN, "MHLEVENT=%s",
			connected? "HPD" : "NO_HPD");
	kobject_uevent_env(&gDriverContext.pDevice->kobj,
						KOBJ_CHANGE, envp);
}


/*****************************************************************************/
/**
 * @brief Handler for most of the event notifications from the MhlTx layer.
 *
 *****************************************************************************/

                    //TB - Commented out temporarily -- use the FW switch and later use this to make it Linux driver compliant.
#if 1 //(
MhlTxNotifyEventsStatus_e  AppNotifyMhlEvent(uint8_t eventCode, uint8_t eventParam, void *pRefData)  //TB
{
	char	event_string[MAX_EVENT_STRING_LEN];
	char	*envp[] = {event_string, NULL};
	MhlTxNotifyEventsStatus_e rtnStatus = MHL_TX_EVENT_STATUS_PASSTHROUGH;
	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
			"AppNotifyEvent called, eventCode: 0x%02x eventParam: 0x%02x\n",
			eventCode, eventParam);

	// Save the info on the most recent event.  This is done to support the
	// SII_MHL_GET_MHL_TX_EVENT IOCTL.  If at some point in the future the
	// driver's IOCTL interface is abandoned in favor of using sysfs attributes
	// this can be removed.
	gDriverContext.pendingEvent = eventCode;
	gDriverContext.pendingEventData = eventParam;

	switch(eventCode)
	{
		case MHL_TX_EVENT_CONNECTION:
			gDriverContext.flags |= MHL_STATE_FLAG_CONNECTED;
		   	SiiMhlTxSetPreferredPixelFormat(MHL_STATUS_CLK_MODE_NORMAL);

			strncpy(event_string, "MHLEVENT=connected", MAX_EVENT_STRING_LEN);
			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);
#ifdef	BYPASS_VBUS_HW_SUPPORT //(
        	// turn off VBUS power here
#endif //)
			break;

		case MHL_TX_EVENT_RCP_READY:
			gDriverContext.flags |= MHL_STATE_FLAG_RCP_READY;
			strncpy(event_string, "MHLEVENT=rcp_ready", MAX_EVENT_STRING_LEN);
			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);
			break;

		case MHL_TX_EVENT_DISCONNECTION:
			gDriverContext.flags = 0;
			gDriverContext.keyCode = 0;
			gDriverContext.errCode = 0;
			#ifdef	BYPASS_VBUS_HW_SUPPORT //(
			        pinTx2MhlRxPwrM = 1; //disable VBUS power on cable pull
			#endif //)

			strncpy(event_string, "MHLEVENT=disconnected", MAX_EVENT_STRING_LEN);
			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);
			break;

		case MHL_TX_EVENT_RCP_RECEIVED:
			gDriverContext.flags &= ~MHL_STATE_FLAG_RCP_SENT;
			gDriverContext.flags |= MHL_STATE_FLAG_RCP_RECEIVED;
			gDriverContext.keyCode = eventParam;

			switch (gDriverContext.keyCode)
			{
				case 0x0:
					input_report_key(gDriverContext.input_dev_key, KEY_ENTER, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_ENTER, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_SELECT down\n");
					break;					
				case 0x80:
//					input_report_key(gDriverContext.input_dev_key, KEY_ENTER, 0);
//					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_SELECT up\n");
					break;
				case 0x01:
					input_report_key(gDriverContext.input_dev_key, KEY_UP, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_UP, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_UP down\n");
					break;					
				case 0x81:
//					input_report_key(gDriverContext.input_dev_key, KEY_UP, 0);
//					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_UP up\n");
					break;		
				case 0x02:
					input_report_key(gDriverContext.input_dev_key, KEY_DOWN, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_DOWN, 0);					
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_DOWN down\n");
					break;					
				case 0x82:
//					input_report_key(gDriverContext.input_dev_key, KEY_DOWN, 0);
//					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_DOWN up\n");
					break;	
				case 0x03:
					input_report_key(gDriverContext.input_dev_key, KEY_LEFT, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_LEFT, 0);					
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_LEFT down\n");
					break;					
				case 0x83:
//					input_report_key(gDriverContext.input_dev_key, KEY_LEFT, 0);
//					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_LEFT up\n");
					break;	
				case 0x04:
					input_report_key(gDriverContext.input_dev_key, KEY_RIGHT, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_RIGHT, 0);					
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_RIGHT down\n");
					break;					
				case 0x84:
//					input_report_key(gDriverContext.input_dev_key, KEY_RIGHT, 0);
//					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_RIGHT up\n");
					break;							
				case 0x09:
					input_report_key(gDriverContext.input_dev_key, KEY_HOMEPAGE, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_HOMEPAGE, 0);					
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_HOMEPAGE down\n");
					break;					
				case 0x89:
//					input_report_key(gDriverContext.input_dev_key, KEY_HOMEPAGE, 0);
//					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_HOMEPAGE up\n");
					break;
				case 0x0D:
					input_report_key(gDriverContext.input_dev_key, KEY_BACK, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_BACK, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_BACK down\n");
					break;					
				case 0x8D:
//					input_report_key(gDriverContext.input_dev_key, KEY_BACK, 0);
//					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_BACK up\n");
					break;
//ASUS BSP Wei_Lai +++ 
				case 0x20:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_0, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_0, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_0 down\n");
					break;
				case 0x21:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_1, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_1, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_1 down\n");
					break;
				case 0x22:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_2, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_2, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_2 down\n");
					break;
				case 0x23:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_3, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_3, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_3 down\n");
					break;
				case 0x24:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_4, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_4, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_4 down\n");
					break;
				case 0x25:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_5, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_5, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_5 down\n");
					break;
				case 0x26:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_6, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_6, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_6 down\n");
					break;
				case 0x27:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_7, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_7, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_7 down\n");
					break;
				case 0x28:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_8, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_8, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_8 down\n");
					break;
				case 0x29:
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_9, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NUMERIC_9, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NUMERIC_9 down\n");
					break;
				case 0x44:
					input_report_key(gDriverContext.input_dev_key, KEY_PLAYPAUSE, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_PLAYPAUSE, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_PLAY down\n");
					break;
				case 0x45:
					input_report_key(gDriverContext.input_dev_key, KEY_STOP, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_STOP, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_STOP down\n");
					break;
				case 0x46:
					input_report_key(gDriverContext.input_dev_key, KEY_PAUSE, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_PAUSE, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_PAUSE down\n");
					break;
				case 0x48:
					input_report_key(gDriverContext.input_dev_key, KEY_REWIND, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_REWIND, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_REWIND down\n");
					break;
				case 0x49:
					input_report_key(gDriverContext.input_dev_key, KEY_FASTFORWARD, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_FASTFORWARD, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_FASTFORWARD down\n");
					break;
				case 0x4B:
					input_report_key(gDriverContext.input_dev_key, KEY_NEXTSONG, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_NEXTSONG, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_NEXTSONG down\n");
					break;
				case 0x4C:
					input_report_key(gDriverContext.input_dev_key, KEY_PREVIOUSSONG, 1);
					input_report_key(gDriverContext.input_dev_key, KEY_PREVIOUSSONG, 0);
					input_sync(gDriverContext.input_dev_key);
					printk("[MHL RCP] KEY_PREVIOUSSONG down\n");
					break;
					
//ASUS BSP Wei_Lai --- 
				default:
					printk("[MHL RCP] default key =0x%x\n", gDriverContext.keyCode);
					
					break;
			}


			
			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_RCP key code=0x%02x", eventParam);
			printk("[MHL RCP] MHLEVENT=received_RCP key code=0x%02x", eventParam);
			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);
			
			break;

		case MHL_TX_EVENT_RCPK_RECEIVED:
			if((gDriverContext.flags & MHL_STATE_FLAG_RCP_SENT)
				&& (gDriverContext.keyCode == eventParam)) {

				gDriverContext.flags |= MHL_STATE_FLAG_RCP_ACK;

				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
						"Generating RCPK received event, keycode: 0x%02x\n",
						eventParam);
				snprintf(event_string, MAX_EVENT_STRING_LEN,
						"MHLEVENT=received_RCPK key code=0x%02x", eventParam);
				kobject_uevent_env(&gDriverContext.pDevice->kobj,
									KOBJ_CHANGE, envp);
			} else {
				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
						"Ignoring unexpected RCPK received event, keycode: 0x%02x\n",
						eventParam);
			}
			break;

		case MHL_TX_EVENT_RCPE_RECEIVED:
			if(gDriverContext.flags & MHL_STATE_FLAG_RCP_SENT) {

				gDriverContext.errCode = eventParam;
				gDriverContext.flags |= MHL_STATE_FLAG_RCP_NAK;

				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
						"Generating RCPE received event, error code: 0x%02x\n",
						eventParam);
				snprintf(event_string, MAX_EVENT_STRING_LEN,
						"MHLEVENT=received_RCPE error code=0x%02x", eventParam);
				kobject_uevent_env(&gDriverContext.pDevice->kobj,
									KOBJ_CHANGE, envp);
			} else {
				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
						"Ignoring unexpected RCPE received event, error code: 0x%02x\n",
						eventParam);
			}
			break;

		case MHL_TX_EVENT_DCAP_CHG:
			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=DEVCAP change");
			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);

			break;

		case MHL_TX_EVENT_DSCR_CHG:	// Scratch Pad registers have changed
			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=SCRATCHPAD change");
			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);
			break;

		case MHL_TX_EVENT_POW_BIT_CHG:	// Peer's power capability has changed
			if (eventParam)
			{
#ifdef BYPASS_VBUS_HW_SUPPORT //(
				// Since downstream device is supplying VBUS power we should
				// turn off our VBUS power here.  If the platform application
				// can control VBUS power it should turn off it's VBUS power
				// now and return status of MHL_TX_EVENT_STATUS_HANDLED.  If
				// platform cannot control VBUS power it should return
				// MHL_TX_EVENT_STATUS_PASSTHROUGH to allow the MHL layer to
				// try to turn it off.
				rtnStatus = MHL_TX_EVENT_STATUS_HANDLED;
#else //)(
				// In this sample driver all that is done is to report an
				// event describing the requested state of VBUS power and
				// return MHL_TX_EVENT_STATUS_PASSTHROUGH to allow lower driver
				// layers to control VBUS power if possible.
#endif //)
				snprintf(event_string, MAX_EVENT_STRING_LEN,
						"MHLEVENT=MHL VBUS power OFF");
			}
			else
			{
				snprintf(event_string, MAX_EVENT_STRING_LEN,
						"MHLEVENT=MHL VBUS power ON");
#ifdef BYPASS_VBUS_HW_SUPPORT //(
				rtnStatus = MHL_TX_EVENT_STATUS_HANDLED;
#else //)(
#endif //)
			}

			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);
			break;
		case MHL_TX_EVENT_RGND_MHL:
#ifdef BYPASS_VBUS_HW_SUPPORT //(
			// RGND measurement has determine that the peer is an MHL device.
			// If platform application can determine that the attached device
			// is not supplying VBUS power it should turn on it's VBUS power
			// here and return MHL_TX_EVENT_STATUS_HANDLED to indicate to
			// indicate to the caller that it handled the notification.
			rtnStatus = MHL_TX_EVENT_STATUS_HANDLED;
#else //)(
			// In this sample driver all that is done is to report the event
			// and return MHL_TX_EVENT_STATUS_PASSTHROUGH to allow lower driver
			// layers to control VBUS power if possible.
#endif //)
			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=MHL device detected");
			kobject_uevent_env(&gDriverContext.pDevice->kobj,
								KOBJ_CHANGE, envp);
			break;

		case MHL_TX_EVENT_TMDS_ENABLED:  //TB : Added 3/9/12 : 2:00 pm
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_APP_TRACE,"MHL_TX_EVENT_TMDS_ENABLED!\n");
			SiiHdmiTxLiteTmdsActive();
	        break;

		case MHL_TX_EVENT_TMDS_DISABLED: //TB : Added 3/9/12 : 2:00 pm
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_APP_TRACE,"MHL_TX_EVENT_TMDS_DISABLED!\n");
	        SiiHdmiTxLiteTmdsInactive();
	        break;

		case MHL_TX_EVENT_INFO_FRAME_CHANGE:
		    if (!PlatformGPIOGet(pinTranscodeMode))
		    {
		        MhlTxLiteSetInfoFrame(eventParam,pRefData);
		    }
		    break;
		 default:
			SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
					"AppNotifyEvent called with unrecognized event code!\n");
	}
	return rtnStatus;
}


#else //)( //TB - changed to FW

///////////////////////////////////////////////////////////////////////////////
//
// AppNotifyMhlEvent
//
//  This function is invoked from the MhlTx component to notify the application
//  about detected events that may be of interest to it.
//
// Application module must provide this function.
//
MhlTxNotifyEventsStatus_e AppNotifyMhlEvent(uint8_t eventCode, uint8_t eventParam,void *pRefData)
{
	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
		"AppNotifyEvent called, eventCode: 0x%02x eventParam: 0x%02x\n",
		eventCode, eventParam);

	MhlTxNotifyEventsStatus_e retVal = MHL_TX_EVENT_STATUS_PASSTHROUGH;

	switch(eventCode)
	{
	case MHL_TX_EVENT_DISCONNECTION:
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "App: Got event = MHL_TX_EVENT_DISCONNECTION\n");
#ifdef	BYPASS_VBUS_HW_SUPPORT //(
        pinTx2MhlRxPwrM = 1; //disable VBUS power on cable pull
#endif //)
		pinSourceVbusOn = 0;
		break;
	case MHL_TX_EVENT_CONNECTION:
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "App: Got event = MHL_TX_EVENT_CONNECTION\n");
    	SiiMhlTxSetPreferredPixelFormat(MHL_STATUS_CLK_MODE_NORMAL);
        pinSourceVbusOn = pinVbusEnM;
		break;
	case MHL_TX_EVENT_RCP_READY:
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "MHL connection is ready for RCP");
		break;
	case MHL_TX_EVENT_RCP_RECEIVED :
        //
        // Check if we got an RCP. Application can perform the operation here
        // and send RCPK or RCPE. For now, we send the RCPK
        //
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "App: Received an RCP key code = %02X\n", (int)eventParam );

        SiiMhlTxRcpkSend(eventParam);
		break;
	case MHL_TX_EVENT_RCPK_RECEIVED:
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "App: Received an RCPK = %02X\n", (int)eventParam);
		break;
	case MHL_TX_EVENT_RCPE_RECEIVED:
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "App: Received an RCPE = %02X\n", (int)eventParam);
		break;
	case MHL_TX_EVENT_DCAP_CHG:
        {
        uint8_t myData;
        	// EDID_SNOOPING
            SiiTxGetPeerDevCapEntry(DEVCAP_OFFSET_VID_LINK_MODE,&myData);
        	SiiDrvSetPackedPixel( (myData & 0x03) == 0x03 );
            // debug output
            {
            uint8_t i;
                SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "App: MHL_TX_EVENT_DCAP_CHG:");
            	for(i=0;i<16;++i)
            	{
        			if (0 == SiiTxGetPeerDevCapEntry(i,&myData))
        			{
        				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "0x%02x ",(int)myData);

        			}
        			else
        			{
        				SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "busy ");
        			}
            	}
            	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "\n");
            }
        }
		break;
	case MHL_TX_EVENT_POW_BIT_CHG:
		// handle the LEDs here
		if (eventParam)
		{
			// POW bit 0->1 transition
            pinSourceVbusOn = 0;
		}
		else
		{
			// POW bit 1->0 transition
			// do nothing
		}
		// Let the lower layers handle the rest.
#ifdef BYPASS_VBUS_HW_SUPPORT //(
		if (eventParam) // power bit changed
		{
		    pinTx2MhlRxPwrM = 1;
		}
		// indicate to lower layers that we have handled this.
		retVal = MHL_TX_EVENT_STATUS_HANDLED;
#endif //)
		break;
	case MHL_TX_EVENT_RGND_MHL:
#ifdef BYPASS_VBUS_HW_SUPPORT //(
		if (!pinVbusEnM)
		{
			// no power coming from peer, so supply it.
	        pinTx2MhlRxPwrM = 0;
		}

		// The important thing is the return value
		retVal = MHL_TX_EVENT_STATUS_HANDLED;
#else
		// Let the lower layers handle this.
#endif //)
		break;
    case MHL_TX_EVENT_TMDS_ENABLED:
        SiiHdmiTxLiteHdmiDrvTmdsActive();
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_APP_TRACE,"MHL_TX_EVENT_TMDS_ENABLED!\n");
        break;
    case MHL_TX_EVENT_TMDS_DISABLED:
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_APP_TRACE,"MHL_TX_EVENT_TMDS_DISABLED!\n");
        break;
    case MHL_TX_EVENT_INFO_FRAME_CHANGE:
        MhlTxLiteSetInfoFrame(eventParam,pRefData);
        break;
	default:
		SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "Unknown event: 0x%02x\n",(int)eventCode);

	}
	return retVal;
}
#endif //)

/*****************************************************************************/
/**
 * @brief Handler for MHL transmitter reset requests.
 *
 * This function is called by the MHL layer to request that the MHL transmitter
 * chip be reset.  Since the MHL layer is platform agnostic and therefore doesn't
 * know how to control the transmitter's reset pin each platform application is
 * required to implement this function to perform the requested reset operation.
 *
 * @param[in]	hwResetPeriod	Time in ms. that the reset pin is to be asserted.
 * @param[in]	hwResetDelay	Time in ms. to wait after reset pin is released.
 *
 *****************************************************************************/
void AppResetMhlTx(uint16_t hwResetPeriod,uint16_t hwResetDelay)
{

	// Reset the TX chip,
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"AppResetMhlTx +++\n");	
	
	HalTimerWait(hwResetDelay);
	
	HalGpioSetTxResetPin(LOW);
	HalTimerWait(hwResetPeriod);
	
	HalGpioSetTxResetPin(HIGH);
	HalTimerWait(hwResetDelay);
	
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"AppResetMhlTx ---\n");	
}

/**
 *  File operations supported by the MHL driver
 */
static const struct file_operations siiMhlFops = {
    .owner			= THIS_MODULE,
    .open			= SiiMhlOpen,
    .release		= SiiMhlRelease,
    .unlocked_ioctl	= SiiMhlIoctl
};

//ASUS_BSP larry lai :for ATD test +++
ssize_t read_mhl_status(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int		status = 0;

	if(HalAcquireIsrLock() != HAL_RET_SUCCESS)
	{
		return -ERESTARTSYS;
	}

	status = scnprintf(buf, PAGE_SIZE, "%s",
			gDriverContext.mhl_status ? "1" : "0");


	HalReleaseIsrLock();
	return status;
}
//ASUS_BSP larry lai :for ATD test ---


/*
 * Sysfs attribute files supported by this driver.
 */
struct device_attribute driver_attribs[] = {
		__ATTR(connection_state, 0444, ShowConnectionState, NULL),
		__ATTR(rcp_keycode, 0664, ShowRcp, SendRcp),
		__ATTR(rcp_ack, 0664, ShowRcpAck, SendRcpAck),
		__ATTR(devcap, 0664, ReadDevCap, SelectDevCap),
//ASUS_BSP larry lai :for ATD test +++		
		__ATTR(MHL_status, 0664, read_mhl_status, NULL),		
//ASUS_BSP larry lai :for ATD test ---		
		__ATTR_NULL
};

static int __init SiiMhlInit(void)
{
    int32_t	ret;
    dev_t	devno;

#ifdef ASUS_A68_PROJECT
	printk("MHL:%s:+++\n",__func__);
#else
#ifdef ASUS_A80_PROJECT
	printk("MHL:%s:Oh no! A80 have no MHL! Just return...\n",__func__);
	return 0;
#else
	#error Neither ASUS_A68_PROJECT nor ASUS_A80_PROJECT is defined!!
#endif
#endif

	pr_info("%s driver starting!\n", MHL_DRIVER_NAME);
	pr_info("Version: %s%d\n", buildVersion,BUILDNUM);
	pr_info("%s", buildTime);

    /* register chrdev */
    pr_info("register_chrdev %s\n", MHL_DRIVER_NAME);

    pr_info("TX Driver version = %d\n", BUILD_NUMBER);
	
    /* If a major device number has already been selected use it,
     * otherwise dynamically allocate one.
     */
    if (devMajor) {
        devno = MKDEV(devMajor, 0);
        ret = register_chrdev_region(devno, MHL_DRIVER_MINOR_MAX,
                MHL_DRIVER_NAME);
    } else {
        ret = alloc_chrdev_region(&devno,
                        0, MHL_DRIVER_MINOR_MAX,
                        MHL_DRIVER_NAME);
        devMajor = MAJOR(devno);
    }
    if (ret) {
    	pr_info("register_chrdev %d, %s failed, error code: %d\n",
    					devMajor, MHL_DRIVER_NAME, ret);
        return ret;
    }

    cdev_init(&siiMhlCdev, &siiMhlFops);
    siiMhlCdev.owner = THIS_MODULE;
    ret = cdev_add(&siiMhlCdev, devno, MHL_DRIVER_MINOR_MAX);
    if (ret) {
    	pr_info("cdev_add %s failed %d\n", MHL_DRIVER_NAME, ret);
        goto free_chrdev;
    }

    siiMhlClass = class_create(THIS_MODULE, "mhl");
    if (IS_ERR(siiMhlClass)) {
    	pr_info("class_create failed %d\n", ret);
        ret = PTR_ERR(siiMhlClass);
        goto free_cdev;
    }

    siiMhlClass->dev_attrs = driver_attribs;

    gDriverContext.pDevice  = device_create(siiMhlClass, NULL,
    									 MKDEV(devMajor, 0),  NULL,
    									 "%s", MHL_DEVICE_NAME);
    if (IS_ERR(gDriverContext.pDevice)) {
    	pr_info("class_device_create failed %s %d\n", MHL_DEVICE_NAME, ret);
        ret = PTR_ERR(gDriverContext.pDevice);
        goto free_class;
    }

    gDriverContext.input_dev_key = input_allocate_device();
    if (!gDriverContext.input_dev_key) {
		ret = -ENOMEM;
		goto free_class;
    }
    	
    gDriverContext.input_dev_key->name = "mhl_rcp_key";
    gDriverContext.input_dev_key->phys = "mhl_8240/input0";
	
    set_bit(EV_SYN, gDriverContext.input_dev_key->evbit);
    set_bit(EV_KEY, gDriverContext.input_dev_key->evbit);
//    set_bit(EV_REP, gDriverContext.input_dev_key->evbit);	
	
//ASUS_BSP +++ larry lai: support RCP LD_GUI Key
    set_bit(KEY_BACK,gDriverContext.input_dev_key->keybit);
    set_bit(KEY_HOMEPAGE,gDriverContext.input_dev_key->keybit);
    set_bit(KEY_ENTER,gDriverContext.input_dev_key->keybit);
    set_bit(KEY_UP,gDriverContext.input_dev_key->keybit);
    set_bit(KEY_DOWN,gDriverContext.input_dev_key->keybit);
    set_bit(KEY_LEFT,gDriverContext.input_dev_key->keybit);
    set_bit(KEY_RIGHT,gDriverContext.input_dev_key->keybit);	
//ASUS_BSP --- larry lai: support RCP LD_GUI Key
//ASUS BSP Wei_Lai support RCP LD_VIDEO Key	+++
  set_bit(KEY_NUMERIC_1,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_2,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_3,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_4,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_5,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_6,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_7,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_8,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_9,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NUMERIC_0,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_CLEAR,gDriverContext.input_dev_key->keybit);
 // set_bit(KEY_PLAY,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_PLAYPAUSE,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_STOP,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_PAUSE,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_REWIND,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_FASTFORWARD,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_NEXTSONG,gDriverContext.input_dev_key->keybit);
  set_bit(KEY_PREVIOUSSONG,gDriverContext.input_dev_key->keybit);
  
//ASUS BSP Wei_Lai support RCP LD_VIDEO Key	---

    ret = input_register_device(gDriverContext.input_dev_key);
    if (ret < 0) {
    	input_free_device(gDriverContext.input_dev_key);
    	gDriverContext.input_dev_key = NULL;
    	goto free_class;
    }

    ret = StartMhlTxDevice();
    if(ret == 0) {
//ASUS_BSP larry lai :for ATD test +++		
	gDriverContext.mhl_status = 1;
//ASUS_BSP larry lai :for ATD test ---	
    	return 0;

    } else {
    	// Transmitter startup failed so fail the driver load.
    	device_destroy(siiMhlClass, MKDEV(devMajor, 0));
    }

free_class:
	class_destroy(siiMhlClass);

free_cdev:
	cdev_del(&siiMhlCdev);

free_chrdev:
	unregister_chrdev_region(MKDEV(devMajor, 0), MHL_DRIVER_MINOR_MAX);

	return ret;
}



static void __exit SiiMhlExit(void)
{
	pr_info("%s driver exiting!\n", MHL_DRIVER_NAME);
#ifdef ASUS_A80_PROJECT
	printk("MHL:%s:Oh no! A80 have no MHL! Just return...\n",__func__);
	return;
#endif

	StopMhlTxDevice();

	if (gDriverContext.input_dev_key)
	{
		input_unregister_device(gDriverContext.input_dev_key);
	}

	wake_lock_destroy(&gDriverContext.mhl_wlock);
	device_destroy(siiMhlClass, MKDEV(devMajor, 0));
    class_destroy(siiMhlClass);
    unregister_chrdev_region(MKDEV(devMajor, 0), MHL_DRIVER_MINOR_MAX);
}

module_init(SiiMhlInit);
module_exit(SiiMhlExit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Silicon Image <http://www.siliconimage.com>");
MODULE_DESCRIPTION(MHL_DRIVER_DESC);
