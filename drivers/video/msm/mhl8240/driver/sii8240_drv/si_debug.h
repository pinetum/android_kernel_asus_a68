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


//***************************************************************************
//!file     si_debug.h
//!brief    Silicon Image Debug header.
//
//***************************************************************************/
#ifndef __SI_DEBUG_H__
#define __SI_DEBUG_H__
#include "si_common.h"

//------------------------------------------------------------------------------
// Debug Flags
//------------------------------------------------------------------------------

#define RCP_DEBUG           0       // 1 - Enable RCP debug mode
#define API_DEBUG_CODE      1       // Set to 0 to eliminate debug print messages from code
                                    // Set to 1 to allow debug print messages in code
//------------------------------------------------------------------------------
// Debug Macros
//------------------------------------------------------------------------------

    // Debug print message level

#define MSG_ALWAYS              0x00
#define MSG_ERR                 0x00
#define MSG_STAT                0x01
#define MSG_DBG                 0x02
/*
#if (API_DEBUG_CODE==1)

    #ifndef DEBUG_PRINT
    #define TDEBUG_PRINT(x,y)   SkDebugPrintf( x, y )
    #define DEBUG_PRINT(x,...)  SkDebugPrintf(x,__VA_ARGS__)
    #endif
    #define _DEBUG_(x)              x

#else
    #ifndef DEBUG_PRINT
        #define DEBUG_PRINT(...)
    #endif
    #define _DEBUG_(x)
#endif
*/
//------------------------------------------------------------------------------
// Debug Status Codes
//------------------------------------------------------------------------------

enum
{
    I2C_SUCCESS,
    I2C_READ_FAIL,
    I2C_WRITE_FAIL,
};

//------------------------------------------------------------------------------
// Debug Trace
//------------------------------------------------------------------------------

#if (API_DEBUG_CODE==1)
    #define DF1_SW_HDCPGOOD         0x01
    #define DF1_HW_HDCPGOOD         0x02
    #define DF1_SCDT_INT            0x04
    #define DF1_SCDT_HI             0x08
    #define DF1_SCDT_STABLE         0x10
    #define DF1_HDCP_STABLE         0x20
    #define DF1_NON_HDCP_STABLE     0x40
    #define DF1_RI_CLEARED          0x80

    #define DF2_MP_AUTH             0x01
    #define DF2_MP_DECRYPT          0x02
    #define DF2_HPD                 0x04
    #define DF2_HDMI_MODE           0x08
    #define DF2_MUTE_ON             0x10
    #define DF2_PORT_SWITCH_REQ     0x20
    #define DF2_PIPE_PWR            0x40
    #define DF2_PORT_PWR_CHG        0x80

#endif

//------------------------------------------------------------------------------
//  From si_platform.h
//------------------------------------------------------------------------------
#if !defined __PLATFORM_STATUS_CODES__
#define __PLATFORM_STATUS_CODES__

enum
{
    DBG_BRD,
    DBG_DEV,
    DBG_CBUS,
    DBG_CDC ,
    DBG_CEC ,
    DBG_CPI ,
    DBG_EDID,
    DBG_HEAC,
    DBG_RPTR,
    DBG_SWCH,
    DBG_VSIF,
    DBG_TX,
    DBG_EDID_TX,

    DBG_MAX_COMPONENTS
};

typedef enum _SkDebugPrintFlags_t
{
    DBGF_TS     = 0x0100,       // Add Time Stamp to debug message output
    DBGF_CN     = 0x0200,       // Add Component Name to debug message output
                                // (must include component name index as first
                                // parameter after format string)
    DBGF_CP     = 0x0400,       // Add Component Name to debug message output
                                // (must include pointer to component instance
                                // data as first parameter after format string)
    DBGF_CS     = 0x0800,       // Add Component Name to debug message output
                                // (must include pointer to display string
                                // as first parameter after format string)
} SkDebugPrintFlags_t;

#endif // __PLATFORM_STATUS_CODES__

//------------------------------------------------------------------------------
//  Debug Print Function.  This function declaration is actually from the
//                         Board component, but will always have the same
//                         interface, so we will not have to include the
//                         si_platform.h file, which would be ugly.
//------------------------------------------------------------------------------

//void SkDebugPrintf ( uint_t printFlags, ... );
//void SkDebugSetMessageLevel( uint_t msgLevel );

#endif // __SI_DEBUG_H__
