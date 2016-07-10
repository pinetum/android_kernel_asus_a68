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
 * @file sii_hal_linux_isr.c
 *
 * @brief Linux implementation of interrupt support used by Silicon Image
 *        MHL devices.
 *
 * $Author: Dave Canfield
 * $Rev: $
 * $Date: Jan. 31, 2011
 *
 *****************************************************************************/

#define SII_HAL_LINUX_ISR_C

/***** #include statements ***************************************************/
#include "sii_hal.h"
#include "sii_hal_priv.h"
#include "si_c99support.h"
#include "si_osdebug.h"
// for GPIO debug use +++
#include <linux/gpio.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
// for GPIO debug use ---
/***** local macro definitions ***********************************************/
#define GPIO_CONFIG_DBG(gpio)         (MSM_TLMM_BASE + 0x1000 + (0x10 * (gpio)))
#define GPIO_IN_OUT_DBG(gpio)         (MSM_TLMM_BASE + 0x1004 + (0x10 * (gpio)))

/***** local type definitions ************************************************/

/***** local variable declarations *******************************************/

/***** local function prototypes *********************************************/

/***** global variable declarations *******************************************/


/***** local functions *******************************************************/

/*****************************************************************************/
/*
 *  @brief Interrupt handler for MHL transmitter interrupts.
 *
 *  @param[in]		irq		The number of the asserted IRQ line that caused
 *  						this handler to be called.
 *  @param[in]		data	Data pointer passed when the interrupt was enabled,
 *  						which in this case is a pointer to the
 *  						MhlDeviceContext of the I2c device.
 *
 *  @return     Always returns IRQ_HANDLED.
 *
 *****************************************************************************/
static irqreturn_t HalThreadedIrqHandler(int irq, void *data)
{
	pMhlDeviceContext	pMhlDevContext = (pMhlDeviceContext)data;


	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "HalThreadedIrqHandler called\n");
	if (HalAcquireIsrLock() == HAL_RET_SUCCESS) {
		(pMhlDevContext->irqHandler)();
		HalReleaseIsrLock();
	}

	return IRQ_HANDLED;
}

//ASUS_BSP+++ larry lai : carkit gpio un-plug detect
#ifndef ASUS_FACTORY_BUILD
extern void mhl_switch_carkit(bool_t enable);
extern bool_t mhl_check_carkit_mode(void);
extern bool_t g_b_isCarkitConnected;

static irqreturn_t mhl_usb_id_detect_handler(int irq, void *dev_id)
{
	int value;

	printk("[MHL] mhl_usb_id_detect_handler+++\n");

	printk("gMHL_USB_ID_DETECT config = 0x%x\n",  __raw_readl(GPIO_CONFIG_DBG(MHL_USB_ID_DETECT)) );
	printk("GPIO_IN_OUT_DBG config = 0x%x\n",  __raw_readl(GPIO_IN_OUT_DBG(MHL_USB_ID_DETECT)) );

 	if ( (!mhl_check_carkit_mode()) || (!g_b_isCarkitConnected))
 	{
		return IRQ_HANDLED;		
	}
	
	value = gpio_get_value(MHL_USB_ID_DETECT);
	printk("[MHL] MHL_USB_ID_DETECT change interrupt (%d)\n", value);
	if (value == 0) {
//		printk("[MHL] carkit detect , lunch CarHome\n");
//		mhl_switch_carkit(true);   // do this switch in MHL INT detect
	} else if (value == 1) {
		printk("[MHL] carkit remove , exit CarHome\n");
		mhl_switch_carkit(false);
	}
	printk("[MHL] mhl_usb_id_detect_handler---\n");
	
	return IRQ_HANDLED;
}
#endif
//ASUS_BSP--- larry lai : carkit gpio un-plug detect



/***** public functions ******************************************************/


/*****************************************************************************/
/**
 * @brief Install IRQ handler.
 *
 *****************************************************************************/
halReturn_t HalInstallIrqHandler(fwIrqHandler_t irqHandler)
{
	int				retStatus;
	halReturn_t 	halRet;
//ASUS_BSP+++ larry lai : carkit gpio un-plug detect
#ifndef ASUS_FACTORY_BUILD
	int irq;
#endif
//ASUS_BSP--- larry lai : carkit gpio un-plug detect
	
	if(irqHandler == NULL)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"HalInstallIrqHandler: irqHandler cannot be NULL!\n");
		return HAL_RET_PARAMETER_ERROR;
	}

	halRet = I2cAccessCheck();
	if (halRet != HAL_RET_SUCCESS)
	{
		return halRet;
	}

	if(gMhlDevice.pI2cClient->irq == 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"HalInstallIrqHandler: No IRQ assigned to I2C device!\n");
		return HAL_RET_FAILURE;
	}

	gMhlDevice.irqHandler = irqHandler;

//ASUS_BSP +++ : use INT Low trigger to handle all event to solve TX hang issue	
	retStatus = request_threaded_irq(gMhlDevice.pI2cClient->irq, NULL,
									 HalThreadedIrqHandler,
									 IRQF_TRIGGER_LOW | IRQF_ONESHOT,
//									  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
									 gMhlI2cIdTable[0].name,
									 &gMhlDevice);
//ASUS_BSP --- : use INT Low trigger to handle all event to solve TX hang issue	

	if(retStatus != 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"HalInstallIrqHandler: request_threaded_irq failed, status: %d\n",
    			retStatus);
		gMhlDevice.irqHandler = NULL;
		return HAL_RET_FAILURE;
	}
//	retStatus = enable_irq_wake(gMhlDevice.pI2cClient->irq);
//	if (retStatus != 0) {
//		ERROR_DEBUG_PRINT(("[MHL] Can't enable IRQ as wake source: %d\n", retStatus));
//	}

//ASUS_BSP+++ larry lai : carkit gpio un-plug detect
#ifndef ASUS_FACTORY_BUILD
	irq = MSM_GPIO_TO_INT(MHL_USB_ID_DETECT);
	if (irq < 0) {
		ERROR_DEBUG_PRINT(( "%s: could not get MHL_USB_ID_DETECT IRQ resource, error=%d ", __func__, irq));		
	}
	retStatus = request_irq(irq, mhl_usb_id_detect_handler,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT , "mhl usb id mode", NULL);

	if (retStatus < 0) {
		ERROR_DEBUG_PRINT(( "%s: FACTORY USB IRQ#%d request failed with error=%d ", __func__, irq, retStatus));				
	}
#endif	
//ASUS_BSP--- larry lai : carkit gpio un-plug detect
	
	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Remove IRQ handler.
 *
 *****************************************************************************/
halReturn_t HalRemoveIrqHandler(void)
{
	halReturn_t 	halRet;


	halRet = I2cAccessCheck();
	if (halRet != HAL_RET_SUCCESS)
	{
		return halRet;
	}

	if(gMhlDevice.irqHandler == NULL)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"HalRemoveIrqHandler: no irqHandler installed!\n");
		return HAL_RET_FAILURE;
	}

	free_irq(gMhlDevice.pI2cClient->irq, &gMhlDevice);

	gMhlDevice.irqHandler = NULL;

	return HAL_RET_SUCCESS;
}
