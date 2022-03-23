/*
 * ST VTSens thermal sensor driver
 *
 * Copyright (C) STMicroelectronics 2018
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ST_VTSENS_KERNEL_H
#define __ST_VTSENS_KERNEL_H

int vtsens_read_temperature(unsigned int sensor, int *temperature);

#endif
