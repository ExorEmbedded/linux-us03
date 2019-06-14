/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2013 Flexibilis Oy

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

#ifndef DEIPCE_NETDEV_H
#define DEIPCE_NETDEV_H

#include "deipce_types.h"
#include "deipce_hw_type.h"

/// Link check interval in jiffies
#define DEIPCE_LINK_CHECK_INTERVAL (1*HZ)

struct sk_buff;
struct deipce_cfg;
struct deipce_port_cfg;

int deipce_netdev_init_switch(struct deipce_dev_priv *dp,
                              struct deipce_switch_config *config);
int deipce_netdev_init_port(struct deipce_dev_priv *dp,
                            struct deipce_port_priv *pp,
                            struct deipce_port_config *config);
void deipce_netdev_cleanup_switch(struct deipce_dev_priv *dp);
void deipce_netdev_cleanup_port(struct deipce_dev_priv *dp,
                                struct deipce_port_priv *pp);
void deipce_rx_frame(struct sk_buff *rx_frame);
int deipce_update_port_mode(struct net_device *netdev,
                            enum link_mode link_mode);
bool deipce_is_port(struct net_device *netdev);

#endif
