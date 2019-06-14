/** @file
 */

/*

   Flexibilis time driver for Linux

   Copyright (C) 2016 Flexibilis Oy

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

#ifndef DEIPCE_CLOCK_PTP_H
#define DEIPCE_CLOCK_PTP_H

#include "deipce_clock_types.h"

#ifdef CONFIG_PTP_1588_CLOCK

int flx_frtc_ptp_init(struct flx_frtc_dev_priv *dp);
int flx_frtc_ptp_init_devxtstamp(struct flx_frtc_dev_priv *dp);
void flx_frtc_ptp_cleanup(struct flx_frtc_dev_priv *dp);
int deipce_clock_get_phc_index(struct flx_frtc_dev_priv *dp);

#else

static inline int flx_frtc_ptp_init(struct flx_frtc_dev_priv *dp)
{
    return 0;
}

static inline int flx_frtc_ptp_init_devxtstamp(struct flx_frtc_dev_priv *dp)
{
    return 0;
}

static inline void flx_frtc_ptp_cleanup(struct flx_frtc_dev_priv *dp)
{ }

static inline struct ptp_clock_info *deipce_clock_of_get_phc(
        struct device_node *node)
{
    return NULL;
}

static inline int deipce_clock_get_phc_index(struct flx_frtc_dev_priv *dp)
{
    return -1;
}

#endif // CONFIG_PTP_1588_CLOCK

#endif
