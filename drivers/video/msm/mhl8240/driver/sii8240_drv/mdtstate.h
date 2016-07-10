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

typedef struct _StateTableEntry_t
{
    TxPowerStateEvent_e   event;
    TxPowerState_e        newState;
    void (*pfnTransitionHandler)(void);  // note that the size of this structure, on an 8051, should be exactly 4 bytes.
}StateTableEntry_t,*PStateTableEntry_t;

typedef struct _StateTableRowHeader_t
{

    void (*pfnEventGatherer)(void);  // note that the size of this structure, on an 8051, should be exactly 4 bytes.
    PStateTableEntry_t  pStateRow;
}StateTableRowHeader_t,*PStateTableRowHeader_t;

