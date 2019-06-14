/** @file
 */

/*

   Flexibilis Real-Time Clock Linux driver

   Copyright (C) 2009 Flexibilis Oy

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License version 2
   as published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef DEIPCE_CLOCK_NCO_H
#define DEIPCE_CLOCK_NCO_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/io.h>

#include "deipce_clock_types.h"

int flx_frtc_nco_read_time(struct flx_frtc_dev_priv *dp,
                           struct flx_frtc_time_data *time);
int flx_frtc_nco_read_time_locked(struct flx_frtc_dev_priv *dp,
                                  struct flx_frtc_time_data *time);
int flx_frtc_nco_adj_time(struct flx_frtc_dev_priv *dp,
                          int sign, const struct timespec64 *ts);
int flx_frtc_nco_adj_freq(struct flx_frtc_dev_priv *dp, int32_t ppb);
int flx_frtc_nco_init_registers(struct flx_frtc_dev_priv *dp);

#endif
