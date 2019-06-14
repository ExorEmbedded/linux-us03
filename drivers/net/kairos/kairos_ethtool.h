/** @file
 */

/*

   DE-IP Core Edge Linux driver

   Copyright (C) 2007 Flexibilis Oy

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

#ifndef DEIPCE_ETHTOOL_H
#define DEIPCE_ETHTOOL_H

#include <linux/ethtool.h>

#include "deipce_types.h"

enum link_mode deipce_get_phy_link_mode(struct phy_device *phy);

void deipce_update_port_stats(struct deipce_port_priv *pp);

extern const struct ethtool_ops deipce_ethtool_ops;
extern const struct ethtool_ops deipce_ep_ethtool_ops;

#endif
