/*
 * al3010 driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

extern int set_als_power_state_of_P01(int state);

#define DEFAULT_AL3010_GAIN		85
#define DEFAULT_AL3010_SHIFT	35
#define GOLDEN_P05E_GAIN		85
#define GOLDEN_P05E_SHIFT		35
#define GOLDEN_P05C_GAIN		87
#define GOLDEN_P05C_SHIFT		11