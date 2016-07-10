/*
 * SiIxxxx <Firmware or Driver>
 *
 * Copyright (C) 2011 Silicon Image Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the
 * GNU General Public License for more details.
*/



void SiiHdmiTxLiteHdmiDrvHpdStatusChange(bool_t connected);
void SiiHdmiTxLiteHdmiDrvTmdsActive(void);
void SiiHdmiTxLiteHdcpDrvTmdsActive(void);
void SiiHdmiTxLiteHdmiDrvTmdsInactive(void);
void SiiHdmiTxLiteHdcpDrvTmdsInactive(void);

void SiiHdmiTxLiteDrvProcessAviInfoFrameChange(PAVIInfoFrame_t pAviInfoFrame);
void SiiHdmiTxLiteDrvSendVendorSpecificInfoFrame( PVendorSpecificInfoFrame_t pVendorSpecificInfoFrame);
