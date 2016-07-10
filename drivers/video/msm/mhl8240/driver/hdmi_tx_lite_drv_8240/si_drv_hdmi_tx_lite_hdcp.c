/*
  Copyright (c) 2002-2011, Silicon Image, Inc.  All rights reserved.
  No part of this work may be reproduced, modified, distributed, transmitted,
  transcribed, or translated into any language or computer format, in any form
  or by any means without written permission of: Silicon Image, Inc.,
  1140 East Arques Avenue, Sunnyvale, California 94085
*/

/*
 *****************************************************************************
 * @file  HDCP.c
 *
 * @brief Implementation of the HDCP Module.
 *
 *****************************************************************************
*/

#include "si_memsegsupport.h"
#include "si_common.h"
#ifdef __KERNEL__ //(
#include "sii_hal.h"
#else //)(
#include "hal_timers.h"
#endif //)

#include "si_platform.h"
#include "si_cra.h"
#include "si_cra_cfg.h"
#include "si_common_regs.h"
#include "si_hdmi_tx_lite_private.h"
#include "si_drv_hdmi_tx_lite_hdcp.h"
#include "si_bitdefs.h"
#include "si_mhl_defs.h"
#include "si_mhl_tx_api.h"
#include "si_mhl_tx_base_drv_api.h"
#include <linux/delay.h>


#define AKSV_SIZE              5
#define NUM_OF_ONES_IN_KSV    20

#ifdef KENO_DONGLE_DOWNSTREAM1
//keno20121009 +++ un-display while plug HDMI cable from Dongle downstream
extern uint8_t packedPixelStatus;
//keno20121009 --- un-display while plug HDMI cable from Dongle downstream
extern int g_pad_tv_mode;
extern bool_t MHL_Resume;
#endif

//ASUS_BSP +++ Jason Chang "[A68][MHL]fix battery no show when system in charger mode with pad"
#if defined(ASUS_CN_CHARGER_BUILD) && !defined(ASUS_FACTORY_BUILD)
extern int g_CHG_mode;
#endif
//ASUS_BSP --- Jason Chang "[A68][MHL]fix battery no show when system in charger mode with pad"

void HDCP_On (void);
void HDCP_Restart (void);

#ifdef CHECKREVOCATIONLIST //(
static bool_t CheckRevocationList (void);
#endif //)

#ifdef READKSV //(
static bool_t GetKSV (void);
#endif //)


void SiiDrvHdmiTxLiteHdcpInitialize (void)
{
#ifdef KSVFORWARD //(
	// Enable the KSV Forwarding feature and the KSV FIFO Intererrupt
	ReadModifyWriteTPI(TPI_HDCP_CONTROL_DATA_REG, KSV_FORWARD_MASK, KSV_FORWARD_ENABLE);
	ReadModifyWriteTPI(TPI_KSV_FIFO_READY_INT_EN, KSV_FIFO_READY_EN_MASK, KSV_FIFO_READY_ENABLE);
#endif //)
}

//===========================================================
// Madhuresh
#ifdef	DEBUG_RI_VALUES
extern	int	g_ri_display;
#endif	// DEBUG_RI_VALUES


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   IsRepeater()
//
// PURPOSE      :   Test if sink is a repeater
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE if sink is a repeater. FALSE if not.
//
//////////////////////////////////////////////////////////////////////////////
int IsRepeater(uint8_t queryData)
{
    uint8_t RegImage;

	HDCP_DEBUG_PRINT(("Drv: >>IsRepeater()\n"));

    RegImage = queryData;

    if (RegImage & HDCP_REPEATER_MASK)
        return true;

    return false;           // not a repeater
}


//===========================================================

void SiiDrvHdmiTxLiteHandleHdcpEvents (uint8_t HdcpIntStatus,uint8_t queryData)
{
	uint8_t LinkStatus;
	uint8_t RegImage;
	uint8_t NewLinkProtectionLevel;
#ifdef READKSV //(
	uint8_t RiCnt;
#endif //)
#ifdef KSVFORWARD //(
	uint8_t ksv;
#endif //)
    static uint8_t renegPending=0;

    HDCP_DEBUG_PRINT(("HDCP: SiiDrvHdmiTxLiteHandleHdcpEvents HdcpIntStatus: 0x%02x\n",(int)HdcpIntStatus));
    HDCP_DEBUG_PRINT(("SiiDrvHdmiTxLiteHandleHdcpEvents :  REG_TPI_HW_6A = 0x%02x\n", SiiRegRead(REG_TPI_HW_6A)));
    HDCP_DEBUG_PRINT(("SiiDrvHdmiTxLiteHandleHdcpEvents :  REG_TPI_HW_6B = 0x%02x\n", SiiRegRead(REG_TPI_HW_6B)));
    HDCP_DEBUG_PRINT(("SiiDrvHdmiTxLiteHandleHdcpEvents :  REG_TPI_HW_6C = 0x%02x\n", SiiRegRead(REG_TPI_HW_6C)));
    HDCP_DEBUG_PRINT(("SiiDrvHdmiTxLiteHandleHdcpEvents :  REG_TPI_HW_6D = 0x%02x\n", SiiRegRead(REG_TPI_HW_6D)));
	
    if (!renegPending)
    {
    	// Check if Link Status has changed:
    	if (BIT_TPI_INTR_ST0_HDCP_SECURITY_CHANGE_EVENT & HdcpIntStatus )
    	{
    #ifdef ENABLE_HDCP_DEBUG_PRINT //(
        PLACE_IN_CODE_SEG char szFormatString[]="HDCP (%2X) -> Link = (%2X) %s";
    #endif //)

//ASUS_BSP +++ Jason Chang "[A68][MHL]fix battery no show when system in charger mode with pad"
//This is workaround about battery baby no show in charger mode with pad.
#if defined(ASUS_CN_CHARGER_BUILD) && !defined(ASUS_FACTORY_BUILD)
           if(g_CHG_mode && (g_pad_tv_mode == MHL_PAD_MODE_H || g_pad_tv_mode == MHL_PAD_MODE_L)){
	         printk("Skip handle HDCP_SECURITY_CHANGE_EVENT in charger mode with pad!\n");
	         return;
           }
#endif
//ASUS_BSP --- Jason Chang "[A68][MHL]fix battery no show when system in charger mode with pad"
           
            HDCP_DEBUG_PRINT(("HDCP: \n"));

            LinkStatus = queryData;
    		LinkStatus &= LINK_STATUS_MASK;

    		// SiiRegWrite(TPI_INTERRUPT_STATUS_REG, HDCP_SECURITY_CHANGE_EVENT_MASK);


    		switch (LinkStatus)
    		{
    			case LINK_STATUS_NORMAL:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Normal\n"));
    				break;

    			case LINK_STATUS_LINK_LOST:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Lost\n"));

    				// AV Unmute if DVI and no HDCP in 29 and Link Lost
    				if(    (0 == SiiMhlTxOutputModeIsHDMI())
    				    && (0 == (queryData& 0x08)         )
    				    )

    				{
    					ERROR_DEBUG_PRINT(("Unmute AV coz it is DVI and no HDCP is supported\n"));
                        SiiMhlTxNotifyAuthentication();
						
    					// Do nothing, send out video
    					return;
    				}
    				HDCP_Restart();
                    // we've just reset the HDCP engine, wait until next interrupt to do more processing.
                    return;
    				break;

    			case LINK_STATUS_RENEGOTIATION_REQ:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Renegotiation Required\n"));
    #if 1 //(
                    // old code set the AV_MUTE bit,
                    //  then cleared 2A
                    //  then re-enabled encryption
                    SiiMhlTxDrvVideoMute();
                    SiiDrvHdmiTxLiteDisableEncryption();
    #define WAIT_FOR_BKSV_DONE_ON_RENEG
    #ifdef WAIT_FOR_BKSV_DONE_ON_RENEG //(

                    renegPending = 1;
    #else //)(
                    HDCP_On();
    #endif //)

    #else //)(
    				HDCP_Restart();
    #endif //)

                    // we've just reset the HDCP engine, wait until next interrupt to do more processing.
                    return;
    				break;

    			case LINK_STATUS_LINK_SUSPENDED:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Suspended\n"));
#ifdef KENO_DONGLE_DOWNSTREAM1
				if ((!packedPixelStatus) && (g_pad_tv_mode == MHL_TV_MODE) && (!MHL_Resume)) 
				{
//        				SiiRegModify(0x01C7,0x20, 0x20);//display black image
//					SiiRegWrite(REG_VID_MODE, 0x12);  // KH, Enable Demuxing data
					printk("[LINK_STATUS_LINK_SUSPENDED], Enable Demuxing data\n");
					//HalTimerWait(100);
					msleep(1500);
					SiiRegWrite(REG_VID_MODE, 0x00);  // KH, Disable Demuxing data
        				SiiRegModify(0x01C7,0x20, 0x00);//recovery from black image
					printk("[LINK_STATUS_LINK_SUSPENDED], Disable Demuxing data\n");
				}
#endif			

    				HDCP_On();
//MHL_CTS 20120916+++	
//keno20120903  // workaround for try to re-enable AVI Inforframe    
				msleep(100);					
		    		SiiRegWrite(REG_PKT_FILTER_0, 0xA1);						
//MHL_CTS 20120916---	
    				break;
    		}
    	}

    	// Check if HDCP state has changed:
    	if (BIT_TPI_INTR_ST0_HDCP_AUTH_STATUS_CHANGE_EVENT & HdcpIntStatus)
    	{

            RegImage = queryData;

            HDCP_DEBUG_PRINT(("HDCP: QueryData (TPI:29) = %02X\n", (int)RegImage));
    		NewLinkProtectionLevel = RegImage & (EXTENDED_LINK_PROTECTION_MASK | LOCAL_LINK_PROTECTION_MASK);
    		{
    #ifdef ENABLE_HDCP_DEBUG_PRINT //(
            PLACE_IN_CODE_SEG char szFormatString[]="HDCP (%2X) -> Protection = (%2X) %s";
    #endif //)

                HDCP_DEBUG_PRINT(("HDCP: New prot level = %02X\n", (int)NewLinkProtectionLevel));

    			switch (NewLinkProtectionLevel)
    			{
    				case (EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE):
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"None\n"));
    					HDCP_Restart();
                        return;

    				case LOCAL_LINK_PROTECTION_SECURE:
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"Local, Unmuting\n"));
					printk("SiiDrvHdmiTxLiteHandleHdcpEvents : Local, Unmuting\n");	
                        SiiMhlTxNotifyAuthentication();
//MHL_CTS 20120916+++	
//keno20120903  // workaround for try to re-enable AVI Inforframe    
///		    		SiiRegWrite(REG_PKT_FILTER_0, 0xA1);							
//MHL_CTS 20120916---	
						
    #ifdef	DEBUG_RI_VALUES //(
    					g_ri_display = true;
    #endif	//)
    					break;

    				case (EXTENDED_LINK_PROTECTION_SECURE | LOCAL_LINK_PROTECTION_SECURE):
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"Extended\n"));
    					if (IsRepeater(queryData))
    					{
    #ifdef READKSV //(
    						RiCnt = ReadIndexedRegister(INDEXED_PAGE_0, 0x25);
    						while (RiCnt > 0x70)  // Frame 112
    						{
    							RiCnt = ReadIndexedRegister(INDEXED_PAGE_0, 0x25);
    						}
    						SiI_RegisterReadModifyWrite(TPI_SYSTEM_CONTROL_DATA_REG
    						    , DDC_BUS_REQUEST_MASK      | DDC_BUS_GRANT_MASK
    						    , DDC_BUS_REQUEST_REQUESTED | DDC_BUS_GRANT_GRANTED
    						    );
    						GetKSV();
    						RiCnt = SiiRegRead(TPI_SYSTEM_CONTROL_DATA_REG);
    #endif //)
                            SiiMhlTxNotifyAuthentication();
    					}
    					break;

    				default:
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"Extended but not Local?\n"));
    					HDCP_Restart();
                        return;
    			}
    		}

    #ifdef KSVFORWARD //(
    		// Check if KSV FIFO is ready and forward - Bug# 17892
    		// If interrupt never goes off:
    	 	//   a- KSV formwarding is not enabled
    		//   b- not a repeater
    		//   c- a repeater with device count == 0
    		// and therefore no KSV list to forward
    		if ((SiI_RegisterRead(TPI_KSV_FIFO_READY_INT) & KSV_FIFO_READY_MASK) == KSV_FIFO_READY_YES)
    		{
    			ReadModifyWriteTPI(TPI_KSV_FIFO_READY_INT, KSV_FIFO_READY_MASK, KSV_FIFO_READY_YES);
    			HDCP_DEBUG_PRINT(("KSV Fwd: KSV FIFO has data...\n"));
    			{
    				// While !(last byte has been read from KSV FIFO)
    				// if (count = 0) then a byte is not in the KSV FIFO yet, do not read
    				//   else read a byte from the KSV FIFO and forward it or keep it for revocation check
    				do
    				{
    					ksv = SiiRegRead(TPI_KSV_FIFO_STATUS_REG);
    					if (ksv & KSV_FIFO_COUNT_MASK)
    					{
    						TPI_DEBUG_PRINT((TPI_DEBUG_CHANNEL,"KSV Fwd: KSV FIFO Count = %d, ", (int)(ksv & KSV_FIFO_COUNT_MASK)));
    						ksv = SiI_RegisterRead(TPI_KSV_FIFO_VALUE_REG);	// Forward or store for revocation check
    						HDCP_DEBUG_PRINT(("Value = %d\n", (int)ksv));
    					}
    				} while ((ksv & KSV_FIFO_LAST_MASK) == KSV_FIFO_LAST_NO);
    				HDCP_DEBUG_PRINT(("KSV Fwd: Last KSV FIFO forward complete\n"));
    			}
    		}
    #endif //)
    		// SiiRegWrite(TPI_INTERRUPT_STATUS_REG, HDCP_AUTH_STATUS_CHANGE_EVENT_MASK);
    	}
    }
    if ( BIT_TPI_INTR_ST0_BKSV_DONE & HdcpIntStatus)
    {
        HDCP_DEBUG_PRINT(("HDCP: BKSV read done\n"));

        HDCP_DEBUG_PRINT(("HDCP: QueryData: 0x%02x 2A==%02bx\n"
            ,(int)queryData
            ,SiiRegRead(TPI_HDCP_CONTROL_DATA_REG)
            ));

#ifdef KENO_DONGLE_DOWNSTREAM1
		//Set black display twice to cover BULE display issue.
		if ((!packedPixelStatus) && (g_pad_tv_mode == MHL_TV_MODE) && (!MHL_Resume)) 
	        	SiiRegModify(0x01C7,0x20, 0x20);//black image
#endif

        if (queryData & PROTECTION_TYPE_MASK)   // Is HDCP available
        {
            HDCP_DEBUG_PRINT(("HDCP: \n"));
            renegPending = 0;

#ifdef KENO_DONGLE_DOWNSTREAM1
				if ((!packedPixelStatus) && (g_pad_tv_mode == MHL_TV_MODE) && (!MHL_Resume)) 
				{
//        				SiiRegModify(0x01C7,0x20, 0x20);//display black image
//					SiiRegWrite(REG_VID_MODE, 0x12);  // KH, Enable Demuxing data
					printk("[PROTECTION_TYPE_MASK], Enable Demuxing data\n");
					//HalTimerWait(100);
					msleep(1500);
					SiiRegWrite(REG_VID_MODE, 0x00);  // KH, Disable Demuxing data
        				SiiRegModify(0x01C7,0x20, 0x00);//recovery from black image
					printk("[PROTECTION_TYPE_MASK], Disable Demuxing data\n");
				}
#endif			

        	HDCP_On();
//MHL_CTS 20120916+++	
//keno20120903  // workaround for try to re-enable AVI Inforframe    
				msleep(100);					
		    		SiiRegWrite(REG_PKT_FILTER_0, 0xA1);						
//MHL_CTS 20120916---				
        }
    }
    if ( BIT_TPI_INTR_ST0_BKSV_ERR & HdcpIntStatus)
    {
        HDCP_DEBUG_PRINT(("HDCP: BKSV error - be sure that your sink supports HDCP - restarting\n"));
//ASUS_BSP +++ : for non-support HDCP TV issue, re-start HDCP		
	  HDCP_Restart();
//ASUS_BSP ---
        return;
    }

}

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SiiDrvHdmiTxLiteIsHdcpSupported()
//
// PURPOSE      :   Check TX HDCP revision number to determine if HDCP is
//                  supported by reading TPI_HDCP_REVISION_DATA_REG.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   True if TX supports HDCP, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////

bool_t SiiDrvHdmiTxLiteIsHdcpSupported (void)
{
    uint8_t revision;
	bool_t supported;

	supported = true;

	// Check Device ID
    revision = SiiRegRead(TPI_HDCP_REVISION_DATA_REG);

    if (revision != (HDCP_MAJOR_REVISION_VALUE | HDCP_MINOR_REVISION_VALUE))
	{
    	supported = false;
		HDCP_DEBUG_PRINT(("%sHdmiTxDrv: HDCP not supported. revision:0x%02x\n%s",PlatformGPIOGet(pinDoHdcp)?"\n\n\n":"",(uint16_t)revision,PlatformGPIOGet(pinDoHdcp)?"\n\n\n":"" ));
	}

	return supported;
}



//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   HDCP_On()
//
// PURPOSE      :   Switch hdcp on.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void HDCP_On (void)
{
   	HDCP_DEBUG_PRINT(("HDCP -> Started. TPI:2A = 05\n"));
	printk("HDCP -> Started. TPI:2A = 05\n");	

#if 1 //(
   	SiiRegModify(TPI_HDCP_CONTROL_DATA_REG
   	    , BIT_TPI_HDCP_CONTROL_DATA_DOUBLE_RI_CHECK_MASK   | BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MASK
   	    , BIT_TPI_HDCP_CONTROL_DATA_DOUBLE_RI_CHECK_ENABLE | BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MAX
           );
#else //)(
   	SiiRegModify(TPI_HDCP_CONTROL_DATA_REG
   	    , BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MASK
   	    , BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MAX
           );
#endif //)

}

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SiiDrvHdmiTxLiteDisableEncryption()
//
// PURPOSE      :   Switch hdcp off.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void SiiDrvHdmiTxLiteDisableEncryption (void)
{
    HDCP_DEBUG_PRINT(("HDCP -> Stopped. 2A = 0\n"));
    printk("HDCP -> Stopped. 2A = 0\n");	
	
    SiiRegWrite(TPI_HDCP_CONTROL_DATA_REG, 0);
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   HDCP_Restart()
//
// PURPOSE      :   Switch hdcp off and then back on.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void HDCP_Restart (void)
{
	HDCP_DEBUG_PRINT(("HDCP -> Restart --- TOGGLE TMDS\n"));
    printk("HDCP -> Restart --- TOGGLE TMDS\n");	

	SiiDrvHdmiTxLiteDisableEncryption();
	SiiToggleTmdsForHdcpAuthentication();
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   GetKSV()
//
// PURPOSE      :   Collect all downstrean KSV for verification
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   KSV_Array[]
//
// RETURNS      :   true if KSVs collected successfully. False if not.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// The buffer is limited to KSV_ARRAY_SIZE due to the 8051 implementation.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//////////////////////////////////////////////////////////////////////////////
#ifdef READKSV //(
static bool_t GetKSV(void)
{
	uint8_t i;
    uint16_t KeyCount;
    uint8_t KSV_Array[KSV_ARRAY_SIZE];

    ReadBlockHDCP(DDC_BSTATUS_ADDR_L, 1, &i);
    KeyCount = (i & DEVICE_COUNT_MASK) * 5;
	if (KeyCount != 0)
	{
		ReadBlockHDCP(DDC_KSV_FIFO_ADDR, KeyCount, KSV_Array);
	}

	/*
	TPI_TRACE_PRINT((TPI_TRACE_CHANNEL,"KeyCount = %d\n", (int) KeyCount));
	for (i=0; i<KeyCount; i++)
	{
		TPI_TRACE_PRINT((TPI_TRACE_CHANNEL,"KSV[%2d] = %02X\n", (int) i, (int) KSV_Array[i]));
	}
	*/

	 return true;
}
#endif //)

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  SiiDrvHdmiTxLiteIsAksvValid()
//
// PURPOSE       :  Check if AKSVs contain 20 '0' and 20 '1'
//
// INPUT PARAMS  :  None
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  TBD
//
// RETURNS       :  true if 20 zeros and 20 ones found in AKSV. false OTHERWISE
//
//////////////////////////////////////////////////////////////////////////////
bool_t SiiDrvHdmiTxLiteIsAksvValid (void)
{
    uint8_t B_Data[AKSV_SIZE];
    uint8_t NumOfOnes = 0;
    uint8_t i;

    SiiRegReadBlock(TPI_AKSV_1_REG, B_Data, AKSV_SIZE);

    for (i=0; i < AKSV_SIZE; i++)
    {
        while (B_Data[i] != 0x00)
        {
            if (B_Data[i] & 0x01)
            {
                NumOfOnes++;
            }
            B_Data[i] >>= 1;
        }
    }

    if (NumOfOnes != NUM_OF_ONES_IN_KSV)
	{
        HDCP_DEBUG_PRINT(("HDCP -> Illegal AKSV\n"));
       printk("HDCP -> Illegal AKSV\n");	
		
        return false;
	}

    return true;
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  CheckRevocationList()
//
// PURPOSE       :  Compare KSVs to those included in a revocation list
//
// INPUT PARAMS  :  None
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  TBD
//
// RETURNS       :  true if no illegal KSV found in BKSV list
//
// NOTE			 :	Currently this function is implemented as a place holder only
//
//////////////////////////////////////////////////////////////////////////////
#ifdef CHECKREVOCATIONLIST //(
static bool CheckRevocationList(void)
{
    return true;
}
#endif //)

void SiiHdmiTxLiteHdcpDrvTmdsActive(void)
{
}

void SiiHdmiTxLiteHdcpDrvTmdsInactive(void)
{
}
