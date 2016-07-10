/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//ASUS_BSP +++ Jason Chang "[A80][Backlight] Add interface for backlight driver"
#include <linux/gpio.h>
int renesas_set_backlight(int value);
void sharp_set_cabc(int mode);	//ASUS_BSP + Jason Chang "[A80][Backlight]enable CABC"
//ASUS_BSP --- Jason Chang "[A80][Backlight] Add interface for backlight driver"
#ifndef MIPI_RENESAS_H
#define MIPI_RENESAS_H

#define RENESAS_FWVGA_TWO_LANE

int mipi_renesas_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

#endif  /* MIPI_RENESAS_H */
