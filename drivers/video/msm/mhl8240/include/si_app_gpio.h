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
  @file  si_app_gpio.h
 
  @brief  Definitions for GPIO on Linux
 
*/
 

extern	bool_t	pinDoHdcp;
extern	bool_t	pinTxHwReset;
extern	bool_t	pinDataLaneL;
extern	bool_t	pinDataLaneH;
extern	bool_t	pinMhlConn;
extern	bool_t	pinUsbConn;
extern	bool_t	pinM2uVbusCtrlM;
extern	bool_t	pinDbgMsgs;
extern	bool_t	pinAllowD3;
extern	bool_t	pinOverrideTiming;
extern	bool_t	pinVbusEnM;
extern	bool_t	pinSourceVbusOn;
extern	bool_t	pinSinkVbusOn;
extern  bool_t  pinMhlVbusSense;
extern  bool_t  pinForcePackedPixel;
extern  bool_t	pinTranscodeMode;

#define SB_NONE				(0)
#define SB_EPV5_MARK_II		(1)
#define SB_STARTER_KIT_X01	(2)

#if BUILD_CONFIG == 0 //(
#define TARGET_CHIP "8240"
#define TARGET_DEVICE_ID WOLV60_DEVICE_ID
#define SYSTEM_BOARD		(SB_EPV5_MARK_II)
#elif BUILD_CONFIG == 1 //)(
#define TARGET_CHIP "8240"
#define TARGET_DEVICE_ID WOLV60_DEVICE_ID
#define SYSTEM_BOARD		(SB_STARTER_KIT_X01)
#elif BUILD_CONFIG == 2 //)(
#define TARGET_CHIP "8558"
#define TARGET_DEVICE_ID BISHOP_DEVICE_ID
#define SYSTEM_BOARD		(SB_EPV5_MARK_II)
#elif BUILD_CONFIG == 3 //)(
#define TARGET_CHIP "8558"
#define TARGET_DEVICE_ID BISHOP_DEVICE_ID
#define SYSTEM_BOARD		(SB_STARTER_KIT_X01)
#endif

#if (SYSTEM_BOARD == SB_EPV5_MARK_II) //(
#define SiI_TARGET_STRING(s)       "SiI"s" EPV5 MARK II"
#elif (SYSTEM_BOARD == SB_STARTER_KIT_X01) //)(
#define SiI_TARGET_STRING(s)       "SiI"s" Starter Kit A00"
#else //)(
#error "Unknown SYSTEM_BOARD definition."
#endif //)
