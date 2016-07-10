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
 * @file sii_hal_linux_i2c.c
 *
 * @brief Linux implementation of I2c access functions required by Silicon Image
 *        MHL devices.
 *
 * $Author: Dave Canfield
 * $Rev: $
 * $Date: Jan. 24, 2011
 *
 *****************************************************************************/

#define SII_HAL_LINUX_I2C_C

/***** #include statements ***************************************************/
#include <linux/i2c.h>
#include <linux/slab.h>
#include "sii_hal.h"
#include "sii_hal_priv.h"
#include "si_c99support.h"
#include "si_common.h"
#include "si_cra_cfg.h"
#include "si_cra_internal.h"
#include "si_osdebug.h"
#include <linux/module.h> //joe1_++

//ASUS BSP +++ : register i2c driver PM function
//joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif //#ifdef CONFIG_EEPROM_NUVOTON
//joe1_--
//ASUS BSP --- : register i2c driver PM function
//joe1_++
#ifndef CONFIG_ASUS_DEBUG
#define ASUSEvtlog(...) do{}while(0);
#endif
//joe1_--

#include "si_drvisrconfig.h"
/***** local macro definitions ***********************************************/


/***** local type definitions ************************************************/
#define	POWER_STATE_D3				3
#define	POWER_STATE_D0_NO_MHL		2
#define	POWER_STATE_D0_MHL			0
#define	POWER_STATE_FIRST_INIT		0xFF

#define APQ_8064_GSBI1_QUP_I2C_BUS_ID 0
#define APQ_8064_GSBI2_QUP_I2C_BUS_ID 1
#define APQ_8064_GSBI3_QUP_I2C_BUS_ID 3
#define APQ_8064_GSBI4_QUP_I2C_BUS_ID 4
#define APQ_8064_GSBI5_QUP_I2C_BUS_ID 5
/***** local variable declarations *******************************************/


/***** local function prototypes *********************************************/

static int32_t MhlI2cProbe(struct i2c_client *client, const struct i2c_device_id *id);
static int32_t MhlI2cRemove(struct i2c_client *client);

#ifdef SII8240_PM_SUPPORT
//ASUS BSP +++ : register i2c driver PM function
static int MhlI2c_suspend(struct i2c_client *client, pm_message_t mesg);
static int MhlI2c_resume(struct i2c_client *client);
//ASUS BSP --- : register i2c driver PM function
#endif

/***** global variable declarations *******************************************/
#ifdef SII8240_PM_SUPPORT
int g_mhl_suspend_flag = 0;
#endif
int g_mhl_irq_disable = 0;

unsigned char g_mhl_i2c_not_ready_count=0;
unsigned char g_mhl_i2c_error_not_connected_count=0;
/***** local functions *******************************************************/
extern uint8_t	  fwPowerState;


#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>
#define I2C_TEST_FAIL_MHL_READ_I2C (-1)
#define I2C_TEST_FAIL_MHL_WROND_CHIP_ID (-2)
#define I2C_TEST_FAIL_MHL_WRITE_I2C (-3)
int32_t g_mhl_slave_addr = 0x5b;  // TPI slave address 0xB6

static int mhl_i2c_read(u8 addr, int len, void *data);
static int mhl_i2c_write(u8 addr, int len, void *data);

static int TestMHLChipID(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
       unsigned int l_MHLID=0;

      return I2C_TEST_PASS;

	i2c_log_in_test_case("TestMHLChipID++\n");
//	lnResult = mhl_i2c_read(0x00, 4,&l_MHLID);
	lnResult = mhl_i2c_read(0x1a, 1,&l_MHLID);
	if(lnResult <= 0){

		i2c_log_in_test_case("Fail to get MHL id\n");

		lnResult = I2C_TEST_FAIL_MHL_READ_I2C;
	} /* else if(l_MHLID != MHL_hw_ID){

		i2c_log_in_test_case("Get wrong chip id=%d",l_MHLID);

		lnResult = I2C_TEST_FAIL_MHL_WROND_CHIP_ID;
	} */ else
	{
	    i2c_log_in_test_case("Get MHL 0x1a=0x%x",l_MHLID);	  
            lnResult=I2C_TEST_PASS;
	}
	i2c_log_in_test_case("TestMHLChipID--\n");

	return lnResult;
};

static int TestMHLChipID_DUMP_TPI(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;

	i2c_log_in_test_case("TestMHLChipID++\n");
//	DumpPage0Reg();
	i2c_log_in_test_case("TestMHLChipID--\n");

	return lnResult;
};



static int TestMHLChipID1(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
//       unsigned int l_MHLID=0;
       unsigned char i2cdata=0;
 
      return I2C_TEST_PASS;


	i2c_log_in_test_case("TestMHLChipID 0x1a = 0x11 +++\n");

        i2cdata = 0x11;
       lnResult=mhl_i2c_write(0x1a, 1, &i2cdata);
	if(lnResult <= 0){

		i2c_log_in_test_case("Fail to write mhl 0x1a = 0x11\n");

		lnResult = I2C_TEST_FAIL_MHL_WRITE_I2C;
	}  else
	{
            lnResult=I2C_TEST_PASS;
	}
	i2c_log_in_test_case("TestMHLChipID 0x1a = 0x11 ---\n");

	return lnResult;
};



static int TestMHLChipID2(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
       unsigned char i2cdata=0;

      return I2C_TEST_PASS;
      
	i2c_log_in_test_case("TestMHLChipID 0x1a = 0x1 +++\n");
       i2cdata=0x1;
       lnResult=mhl_i2c_write(0x1a, 1, &i2cdata);
	if(lnResult <= 0){

		i2c_log_in_test_case("Fail to write mhl 0x1a = 0x1\n");

		lnResult = I2C_TEST_FAIL_MHL_WRITE_I2C;
	}  else
	{
            lnResult=I2C_TEST_PASS;
	}
	i2c_log_in_test_case("TestMHLChipID 0x1a = 0x1 ---\n");

	return lnResult;
};


static struct i2c_test_case_info gMHLTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestMHLChipID_DUMP_TPI),
	__I2C_STRESS_TEST_CASE_ATTR(TestMHLChipID),  // just by pass	
	__I2C_STRESS_TEST_CASE_ATTR(TestMHLChipID1),// just by pass	
	__I2C_STRESS_TEST_CASE_ATTR(TestMHLChipID2),	// just by pass
};


static int mhl_i2c_read(u8 addr, int len, void *data)
{
        int i=0;
        int retries=6;
        int status=0;

	struct i2c_msg msg[] = {
		{
			.addr = g_mhl_slave_addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = g_mhl_slave_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};
    
            do{    
        		status = i2c_transfer(gMhlDevice.pI2cClient->adapter,
        			msg, ARRAY_SIZE(msg));
        		if ((status < 0) && (i < retries)){
                                printk("### %s retry %d I2C status=0x%x\r\n", __FUNCTION__, i, status);                                
					msleep(15);
                                i++;
                     }
        	    } while ((status < 0) && (i < retries));

        if(status < 0){
            printk("### MHL: i2c read error\n");
        }
    
        return status;
        
}

static int mhl_i2c_write(u8 addr, int len, void *data)
{
        int i=0;
	int status=0;
	u8 buf[len + 1];
	struct i2c_msg msg[] = {
		{
			.addr = g_mhl_slave_addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		},
	};
	int retries = 6;

	buf[0] = addr;
	memcpy(buf + 1, data, len);

	do {
		status = i2c_transfer(gMhlDevice.pI2cClient->adapter,
			msg, ARRAY_SIZE(msg));
        		if ((status < 0) && (i < retries)){
                                printk("### %s retry %d I2C status=0x%x\r\n", __FUNCTION__, i, status);                                
					msleep(15);
                                i++;
                     }
       } while ((status < 0) && (i < retries));

        if(status < 0){
            printk("### MHL: i2c write error\n");
        }
		
	return status;
}



#endif
// for test ---


void MHL_disable_irq(void)
{
	if (!g_mhl_irq_disable)
	{
		printk("MHL_disable_irq\n");
//		disable_irq(gMhlDevice.pI2cClient->irq);	
		disable_irq_nosync(gMhlDevice.pI2cClient->irq);
		g_mhl_irq_disable = 1;
	}
}

void MHL_enable_irq(void)
{
	if (g_mhl_irq_disable)
	{
		printk("MHL_enable_irq\n");	
		enable_irq(gMhlDevice.pI2cClient->irq);
		g_mhl_irq_disable = 0;	
	}
}

#ifdef SII8240_PM_SUPPORT
extern void MhlTxDisconnect(void);
extern bool asus_padstation_exist_realtime(void);

//ASUS BSP +++ : register i2c driver PM function
static int MhlI2c_suspend(struct i2c_client *client, pm_message_t mesg)
{	

	printk("[MHL] MhlI2c_suspend +++\n");
#if 0
	if (AX_MicroP_IsP01Connected() && asus_padstation_exist_realtime()) {

//ASUS BSP +++ : for suspend , cbus always pull low issue
		MhlTxDisconnect();
//ASUS BSP -
		//check cbus pin is high, then pull low cbus
		if (AX_MicroP_getGPIOOutputPinLevel(OUT_uP_MHL_CBUS_EN))
		{
			if (AX_MicroP_setGPIOOutputPin(OUT_uP_MHL_CBUS_EN, 0) < 0)
			{
				printk("fail to disable mhl cbus \n");
			} 
			else 
			{
				printk("success to disable mhl cbus\n");
			}			
		}		
	} 
	else 
	{
		printk("[MHL] suspend, not in pad, skip DISABLE mhl cbus \n");
	}	

	if (!g_mhl_suspend_flag)
	{
		disable_irq(gMhlDevice.pI2cClient->irq);
	}
#endif	

//ASUS_BSP+++: mhl disable irq while mhl_suspend	
	MHL_disable_irq();
//ASUS_BSP---: mhl disable irq while mhl_suspend
	g_mhl_suspend_flag = 1;

	printk("[MHL] MhlI2c_suspend---\n");	
	return 0;
}
static int MhlI2c_resume(struct i2c_client *client)
{

	printk("[MHL] MhlI2c_resume ++++\n");
#if 0
	if (AX_MicroP_IsP01Connected() && asus_padstation_exist_realtime()) {

		//check cbus pin is low, then pull high cbus
		if (!AX_MicroP_getGPIOOutputPinLevel(OUT_uP_MHL_CBUS_EN))
		{
			if (AX_MicroP_setGPIOOutputPin(OUT_uP_MHL_CBUS_EN, 1) < 0)
			{
				printk("fail to enable mhl cbus \n");
			} 
			else 
			{
				printk("success to enable mhl cbus\n");
			}			
		}		
	} else
	{
		printk("[MHL] resume, not in pad, skip ENABLE mhl cbus \n");
	}	

//	if (g_mhl_suspend_flag)
//	{
//		enable_irq(gMhlDevice.pI2cClient->irq);
//	}
#endif	
//ASUS_BSP+++: mhl enable irq while mhl_resume	
	MHL_enable_irq();
//ASUS_BSP---: mhl enable irq while mhl_resume

	g_mhl_suspend_flag = 0;
	   
	printk("[MHL] MhlI2c_resume ----\n");
	return 0;
}
//ASUS BSP --- : register i2c driver PM function
#endif

/**
 *  @brief Standard Linux probe callback.
 *  
 *  Probe is called if the I2C device name passed to HalOpenI2cDevice matches
 *  the name of an I2C device on the system.
 *  
 *  All we need to do is store the passed in i2c_client* needed when performing
 *  I2C bus transactions with the device.
 *
 *  @param[in]      client     		Pointer to i2c client structure of the matching
 *  								I2C device.
 *  @param[in]      i2c_device_id	Index within MhlI2cIdTable of the matching
 *  								I2C device.
 *
 *  @return     Always returns zero to indicate success.
 *
 *****************************************************************************/
static int32_t MhlI2cProbe(struct i2c_client *client, const struct i2c_device_id *id)
{
	gMhlDevice.pI2cClient = client;

#ifdef CONFIG_I2C_STRESS_TEST
//	g_mhl_slave_addr = client->addr;
#endif	
    return 0;
}


/**
 *  @brief Standard Linux remove callback.
 *  
 *  Remove would be called if the I2C device were removed from the system (very unlikley).
 *  
 *  All we need to do is clear our copy of the i2c_client pointer to indicate we no longer
 *  have an I2C device to work with.
 *
 *  @param[in]	client	Pointer to the client structure representing the I2C device that
 *  					was removed.
 *
 *  @return     Always returns zero to indicate success.
 *
 *****************************************************************************/
static int32_t MhlI2cRemove(struct i2c_client *client)
{
	gMhlDevice.pI2cClient = NULL;
    return 0;
}


/***** public functions ******************************************************/


/*****************************************************************************/
/**
 *  @brief Check if I2c access is allowed.
 *
 *****************************************************************************/
halReturn_t I2cAccessCheck(void)
{
	halReturn_t		retStatus;

	retStatus = HalInitCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}

	if(gMhlDevice.pI2cClient == NULL)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "I2C device not currently open\n");
		retStatus = HAL_RET_DEVICE_NOT_OPEN;
	}
	return retStatus;
}



/*****************************************************************************/
/**
 * @brief Request access to the specified I2c device.
 *
 *****************************************************************************/
halReturn_t HalOpenI2cDevice(char const *DeviceName, char const *DriverName)
{
	halReturn_t		retStatus;
    int32_t 		retVal;


	retStatus = HalInitCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}

    retVal = strnlen(DeviceName, I2C_NAME_SIZE);
    if (retVal >= I2C_NAME_SIZE)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "I2c device name too long!\n");
    	return HAL_RET_PARAMETER_ERROR;
    }
    
    memcpy(gMhlI2cIdTable[0].name, DeviceName, retVal);
    gMhlI2cIdTable[0].name[retVal] = 0;
    gMhlI2cIdTable[0].driver_data = 0;

    gMhlDevice.driver.driver.name = DriverName;
    gMhlDevice.driver.id_table = gMhlI2cIdTable;
    gMhlDevice.driver.probe = MhlI2cProbe;
    gMhlDevice.driver.remove = MhlI2cRemove;
    #ifdef SII8240_PM_SUPPORT 
    gMhlDevice.driver.suspend = MhlI2c_suspend;
    gMhlDevice.driver.resume = MhlI2c_resume;	
    #endif
 
    retVal = i2c_add_driver(&gMhlDevice.driver);
    if (retVal != 0)
    {
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "I2C driver add failed\n");
        retStatus = HAL_RET_FAILURE;
    }
    else
    {
    	if (gMhlDevice.pI2cClient == NULL)
        {
            i2c_del_driver(&gMhlDevice.driver);
        	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "I2C driver add failed\n");
            retStatus = HAL_RET_NO_DEVICE;
        }
    	else
    	{
    		retStatus = HAL_RET_SUCCESS;

#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(gMhlDevice.pI2cClient, "MHL_Sii8240",ARRAY_AND_SIZE(gMHLTestCaseInfo));
#endif
			
    	}
    }
    return retStatus;
}



/*****************************************************************************/
/**
 * @brief Terminate access to the specified I2c device.
 *
 *****************************************************************************/
halReturn_t HalCloseI2cDevice(void)
{
	halReturn_t		retStatus;


	retStatus = HalInitCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}

	if(gMhlDevice.pI2cClient == NULL)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "I2C device not currently open\n");
        retStatus = HAL_RET_DEVICE_NOT_OPEN;
	}
	else
	{
		i2c_del_driver(&gMhlDevice.driver);
		gMhlDevice.pI2cClient = NULL;
		retStatus = HAL_RET_SUCCESS;
	}
	return retStatus;
}


#if 0
/*****************************************************************************/
/**
 * @brief Read a byte from an I2c device using SMBus protocol.
 *
 *****************************************************************************/
halReturn_t HalSmbusReadByteData(uint8_t command, uint8_t *pRetByteRead)
{
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}
	
	status = i2c_smbus_read_byte_data(gMhlDevice.pI2cClient, command);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"i2c_smbus_read_byte_data returned error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	*pRetByteRead = (uint8_t)status;
	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Write a byte to an I2c device using SMBus protocol.
 *
 *****************************************************************************/
halReturn_t HalSmbusWriteByteData(uint8_t command, uint8_t writeByte)
{
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}
	
	status = i2c_smbus_write_byte_data(gMhlDevice.pI2cClient,
									   command, writeByte);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"i2c_smbus_write_byte_data returned error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Read a word from an I2c device using SMBus protocol.
 *
 *****************************************************************************/
halReturn_t HalSmbusReadWordData(uint8_t command, uint16_t *pRetWordRead)
{
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}
	
	status = i2c_smbus_read_word_data(gMhlDevice.pI2cClient, command);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"i2c_smbus_read_word_data returned error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	*pRetWordRead = (uint16_t)status;
	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Write a word to an I2c device using SMBus protocol.
 *
 *****************************************************************************/
halReturn_t HalSmbusWriteWordData(uint8_t command, uint16_t wordData)
{
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}
	
	status = i2c_smbus_write_word_data(gMhlDevice.pI2cClient,
									   command, wordData);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"i2c_smbus_write_word_data returned error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Read a series of bytes from an I2c device using SMBus protocol.
 *
 *****************************************************************************/
halReturn_t HalSmbusReadBlock(uint8_t command, uint8_t *buffer, uint8_t *bufferLen)
{
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}

	if(*bufferLen > I2C_SMBUS_BLOCK_MAX)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"HalSmbusReadBlock, bufferLen param too big (%d) max size (%d)!\n",
    			*bufferLen, I2C_SMBUS_BLOCK_MAX);
        return HAL_RET_PARAMETER_ERROR;

	}
	status = i2c_smbus_read_i2c_block_data(gMhlDevice.pI2cClient, command,
										   *bufferLen, buffer);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"i2c_smbus_read_i2c_block_data returned error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	*bufferLen = (uint8_t)status;	/* return # of bytes read */
	
	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Write a series of bytes to an I2c device using SMBus protocol.
 *
 *****************************************************************************/
halReturn_t HalSmbusWriteBlock(uint8_t command, uint8_t const *blockData,
								  uint8_t length)
{
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}

	if(length > I2C_SMBUS_BLOCK_MAX)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"HalSmbusWriteBlock, bufferLen param too big (%d) max size (%d)!\n",
    			length, I2C_SMBUS_BLOCK_MAX);
        return HAL_RET_PARAMETER_ERROR;

	}
	status = i2c_smbus_write_i2c_block_data(gMhlDevice.pI2cClient, command,
											length, blockData);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"i2c_smbus_write_i2c_block_data returned error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Write a series of bytes to an I2c device.
 *
 *****************************************************************************/
halReturn_t HalI2cMasterWrite(uint8_t i2cAddr, uint8_t length, uint8_t *buffer)
{
    struct i2c_msg	i2cMsg;
    
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}

    i2cMsg.addr = (i2cAddr >> 1);
    i2cMsg.flags = 0;
    i2cMsg.len = length;
    i2cMsg.buf = buffer;

    status = i2c_transfer(gMhlDevice.pI2cClient->adapter, &i2cMsg, 1);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"HalI2cMasterWrite, i2c_transfer error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	return HAL_RET_SUCCESS;
}



/*****************************************************************************/
/**
 * @brief Read a series of bytes from an I2c device.
 *
 *****************************************************************************/
halReturn_t HalI2cMasterRead(uint8_t i2cAddr, uint8_t length,
								uint8_t *buffer)
{
    struct i2c_msg	i2cMsg;
    
	halReturn_t		retStatus;
	int32_t			status;
	
	retStatus = I2cAccessCheck();
	if (retStatus != HAL_RET_SUCCESS)
	{
		return retStatus;
	}

    i2cMsg.addr = (i2cAddr >> 1);
    i2cMsg.flags = I2C_M_RD;
    i2cMsg.len = length;
    i2cMsg.buf = buffer;

    status = i2c_transfer(gMhlDevice.pI2cClient->adapter, &i2cMsg, 1);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE, "i2c_transfer error: %d\n",status);
		return HAL_RET_FAILURE;
	}

	return HAL_RET_SUCCESS;
}


/*****************************************************************************/
/**
 * @brief Read a single byte from a register within an I2c device.
 *
 *****************************************************************************/
uint8_t I2C_ReadByte(uint8_t deviceID, uint8_t offset)
{
	uint8_t					accessI2cAddr;
	uint8_t					addrOffset;
	union i2c_smbus_data	data;
	int32_t					status;


	if (I2cAccessCheck() != HAL_RET_SUCCESS)
	{
		/* Driver expects failed I2C reads to return 0xFF */
		return 0xFF;
	}

	// Figure I2c address offset from base of I2c device to requested
	// I2c address.
	addrOffset = deviceID - BASE_I2C_ADDR;

	// Get REAL base address of the I2c device on the platform.
	accessI2cAddr = (uint8_t)(gMhlDevice.pI2cClient->addr << 1);

	// Calculate REAL I2c access address.
	accessI2cAddr += addrOffset;

	accessI2cAddr >>= 1;

    status = i2c_smbus_xfer(gMhlDevice.pI2cClient->adapter, accessI2cAddr,
    						0, I2C_SMBUS_READ, offset, I2C_SMBUS_BYTE_DATA,
    						&data);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"I2C_ReadByte(0x%02x, 0x%02x), i2c_transfer error: %d\n",
    			deviceID, offset, status);
		data.byte = 0xFF;
	}

	return data.byte;
}



/*****************************************************************************/
/**
 * @brief Write a single byte to a register within an I2c device.
 *
 *****************************************************************************/
void I2C_WriteByte(uint8_t deviceID, uint8_t offset, uint8_t value)
{
	uint8_t					accessI2cAddr;
	uint8_t					addrOffset;
	union i2c_smbus_data	data;
	int32_t					status;


	if (I2cAccessCheck() != HAL_RET_SUCCESS)
	{
		return;
	}

	// Figure I2c address offset from base of I2c device to requested
	// I2c address.
	addrOffset = deviceID - BASE_I2C_ADDR;

	// Get REAL base address of the I2c device on the platform.
	accessI2cAddr = (uint8_t)(gMhlDevice.pI2cClient->addr << 1);

	// Calculate REAL I2c access address.
	accessI2cAddr += addrOffset;

	accessI2cAddr >>= 1;

	data.byte = value;

    status = i2c_smbus_xfer(gMhlDevice.pI2cClient->adapter, accessI2cAddr,
    						0, I2C_SMBUS_WRITE, offset, I2C_SMBUS_BYTE_DATA,
    						&data);
	if (status < 0)
	{
    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    			"I2C_WriteByte(0x%02x, 0x%02x, 0x%02x), i2c_transfer error: %d\n",
    			deviceID, offset, value, status);
	}
}
#endif // #if 0

extern unsigned int qup_check_suspended_flag(int i2c_index);

/*****************************************************************************/
/**
 * @brief Linux implementation of CRA driver platform interface function
 * 		  SiiMasterI2cTransfer.
 *
 *****************************************************************************/
SiiPlatformStatus_t SiiMasterI2cTransfer(deviceAddrTypes_t busIndex,
										 SiiI2cMsg_t *pMsgs, uint8_t msgNum)
{
	uint8_t				idx;
	uint8_t				msgCount = 0;
    struct i2c_msg		i2cMsg[MAX_I2C_MESSAGES];
	uint8_t				*pBuffer = NULL;
    SiiPlatformStatus_t	siiStatus = PLATFORM_FAIL;
    int					i2cStatus;


    do {
    	if (I2cAccessCheck() != HAL_RET_SUCCESS)
    	{
    		break;
    	}

    	if(busIndex != DEV_I2C_0)
    	{
        	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
        			"SiiMasterI2cTransfer error: implementation supports" \
        			"only one I2C bus\n");
    		break;
    	}

    	if(msgNum > MAX_I2C_MESSAGES)
    	{
        	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
        			"SiiMasterI2cTransfer error: implementation supports" \
        			"only %d message segments\n", MAX_I2C_MESSAGES);
    		break;
    	}

    	// Function parameter checks passed, assume at this point that the
    	// function will complete successfully.
    	siiStatus = PLATFORM_SUCCESS;

    	for(idx=0; idx < msgNum; idx++) {
    		i2cMsg[idx].addr	= pMsgs[idx].addr >> 1;
    		i2cMsg[idx].buf		= pMsgs[idx].pBuf;
    		i2cMsg[idx].len		= pMsgs[idx].len;
    		i2cMsg[idx].flags	= (pMsgs[idx].cmdFlags & SII_MI2C_RD) ? I2C_M_RD : 0;
    		if(pMsgs[idx].cmdFlags & SII_MI2C_TEN) {
    			pMsgs[idx].cmdFlags |= I2C_M_TEN;
    		}
    		if(pMsgs[idx].cmdFlags & SII_MI2C_APPEND_NEXT_MSG) {
    			// Caller is asking that we append the buffer from the next
    			// message to this one.  We will do this IF there is a next
    			// message AND the direction of the two messages is the same
    			// AND we haven't already appended a message.

    			siiStatus = PLATFORM_INVALID_PARAMETER;
    			if(idx+1 < msgNum && pBuffer == NULL) {
    				if(!((pMsgs[idx].cmdFlags ^ pMsgs[idx+1].cmdFlags) & SII_MI2C_RD)) {

    					i2cMsg[idx].len += pMsgs[idx+1].len;

    				    pBuffer = kmalloc(i2cMsg[idx].len, GFP_KERNEL);
    				    if(pBuffer == NULL) {
    				    	siiStatus = PLATFORM_FAIL;
    				    	break;
    				    }

    				    i2cMsg[idx].buf = pBuffer;
    				    memmove(pBuffer, pMsgs[idx].pBuf, pMsgs[idx].len);
    				    memmove(&pBuffer[pMsgs[idx].len], pMsgs[idx+1].pBuf, pMsgs[idx+1].len);

    				    idx += 1;
    				    siiStatus = PLATFORM_SUCCESS;
    				}
    			}
    		}
    		msgCount++;
    	}

    	if(siiStatus != PLATFORM_SUCCESS) {
        	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
        			"SiiMasterI2cTransfer failed, returning error: %d\n", siiStatus);

        	printk("SiiMasterI2cTransfer failed, returning error: %d\n", siiStatus);


    		if(pBuffer != NULL) {
        		kfree(pBuffer);
        	}

    		return siiStatus;
    	}

	if (qup_check_suspended_flag(APQ_8064_GSBI4_QUP_I2C_BUS_ID) == 1)
	{
		if (g_mhl_i2c_not_ready_count > MAX_MHL_I2C_BUS_NOT_READY)
		{
//			panic("[MHL] i2c bus not ready over times\n");
			printk("[MHL i2c error] i2c bus not ready over times (%d), disable irq\n", g_mhl_i2c_not_ready_count);
			MHL_disable_irq();
			ASUSEvtlog("[MHL i2c error] i2c bus not ready over times, disable irq\n");
		}
		else
		{
			g_mhl_i2c_not_ready_count++;
		}
			
	    	if(pBuffer != NULL) {
	    		kfree(pBuffer);
	    	}	
  		siiStatus = PLATFORM_FAIL;	
             printk("## %s i2c bus-%d not ready\n", __FUNCTION__, APQ_8064_GSBI4_QUP_I2C_BUS_ID);
             msleep(100); 

		//SII_MHL_TX_DUMP_INFO

			 
		return siiStatus;
	}
	else
	{
		g_mhl_i2c_not_ready_count = 0;
	}
	
//	printk("[MHL] i2c_transfer (0x%x) (0x%x)+++ \n", pMsgs->addr, *pMsgs->pBuf); 
    	i2cStatus = i2c_transfer(gMhlDevice.pI2cClient->adapter, i2cMsg, msgCount);
//	printk("[MHL] i2c_transfer (0x%x) (0x%x)--- \n", pMsgs->addr, *pMsgs->pBuf);
    	if(pBuffer != NULL) {
    		kfree(pBuffer);
    	}

    	if(i2cStatus < msgCount)
    	{
    		// All the messages were not transferred, some sort of error occurred.
    		// Try to return the most appropriate error code to the caller.
    		if (i2cStatus < 0)
    		{
    	    	SII_DEBUG_PRINT(SII_OSAL_DEBUG_TRACE,
    	    			"SiiMasterI2cTransfer, i2c_transfer error: %d  " \
    	    			"deviceId: 0x%02x regOffset: 0x%02x\n",
    	    			i2cStatus, pMsgs->addr, *pMsgs->pBuf);
						
//ASUS_BSP +++ 			
		
			if (g_mhl_i2c_error_not_connected_count > MAX_MHL_I2C_ERROR_COUNT)
			{
				printk("[MHL i2c error (%d)] %d mode , over error times %d, disable irq\n", i2cStatus,  fwPowerState, g_mhl_i2c_error_not_connected_count);
				MHL_disable_irq();
				ASUSEvtlog("[MHL] i2c error (%d) mode=(%d) , disable irq\n", i2cStatus,  fwPowerState);
			}
			else
			{
				g_mhl_i2c_error_not_connected_count++;
			}

//ASUS_BSP ---
    			siiStatus = PLATFORM_FAIL;
    		}
    		else
    		{
    			// One or more messages transferred so error probably occurred on the
    			// first unsent message.  Look to see if the message was a read or write
    			// and set the appropriate return code.
    			if(pMsgs[i2cStatus].cmdFlags & SII_MI2C_RD)
    			{
    				siiStatus = PLATFORM_I2C_READ_FAIL;
    			}
    			else
    			{
    				siiStatus = PLATFORM_I2C_WRITE_FAIL;
    			}
    		}
    	}

	} while(0);

	if (siiStatus == PLATFORM_SUCCESS)
	{
		g_mhl_i2c_error_not_connected_count = 0;
	}
	return siiStatus;
}
