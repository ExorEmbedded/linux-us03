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

#ifndef DEIPCE_CLOCK_MAIN_H
#define DEIPCE_CLOCK_MAIN_H

struct device_node;
struct flx_frtc_dev_priv;

int __init deipce_clock_init_driver(void);
void deipce_clock_cleanup_driver(void);
struct flx_frtc_dev_priv *deipce_clock_of_get_clock(struct device_node *node);
void deipce_clock_get_granularity(struct flx_frtc_dev_priv *dp,
                                  uint32_t *granularity);

#endif
