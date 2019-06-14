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

#ifndef DEIPCE_ADAPTER_H
#define DEIPCE_ADAPTER_H

#include "deipce_types.h"

// Common adapter registers

/// ID and version
#define ADAPTER_REG_ID                          0x00
#define ADAPTER_ID_ID_SHIFT                     0
#define ADAPTER_ID_ID_MASK                      0xff
#define ADAPTER_ID_VERSION_SHIFT                8
#define ADAPTER_ID_VERSION_MASK                 0xff
#define ADAPTER_ID_ALT_TSE                      0xb2
#define ADAPTER_ID_1000BASE_X                   0xb3
#define ADAPTER_ID_100BASE_FX                   0xb4
#define ADAPTER_ID_MII                          0xb0
#define ADAPTER_ID_RGMII                        0xb1
#define ADAPTER_ID_RMII                         0xb5
#define ADAPTER_ID_SGMII_1000BASEX              0xb8
#define ADAPTER_ID_SGMII_1000BASEX_100BASEFX    0xb9

#define ADAPTER_ID_100BASE_FX_EXT_TX_PLL        0xa4
#define ADAPTER_ID_SGMII_1000BASEX_EXT_TX_PLL   0xa8
#define ADAPTER_ID_SGMII_1000BASEX_100BASEFX_EXT_TX_PLL 0xa9

/// Link status
#define ADAPTER_REG_LINK_STATUS                 0x01
#define ADAPTER_LINK_STATUS_UP                  (1u << 0)
#define ADAPTER_RGMII_SPEED_MASK                ((1u << 2) | (1u << 1))
#define ADAPTER_RGMII_SPEED_1000M               ((0u << 2) | (1u << 1))
#define ADAPTER_RGMII_SPEED_100M                ((1u << 2) | (0u << 1))
#define ADAPTER_RGMII_SPEED_10M                 ((1u << 2) | (1u << 1))

/// RX delay register
#define ADAPTER_REG_RX_DELAY                    0x10
/// TX delay register
#define ADAPTER_REG_TX_DELAY                    0x11

// Altera triple-speed Ethernet registers

/// PCS control register
#define ALT_TSE_PCS_CONTROL                     0x20

/// PCS status register
#define ALT_TSE_PCS_STATUS                      0x21
#define ALT_TSE_PCS_STATUS_AUTONEG_COMPLETE     (1u << 5)
#define ALT_TSE_PCS_STATUS_LINK_UP              (1u << 2)

/// PCS device ability register
#define ALT_TSE_PCS_DEV_ABILITY                 0x24

/// PCS parter ability register
#define ALT_TSE_PCS_PARTNER_ABILITY             0x25

// PCS device ability and partner ability bits in 1000Base-X mode
#define ALT_TSE_PCS_1000BASEX_ACK               (1u << 14)

// PCS device ability and partner ability bits in SGMII mode
#define ALT_TSE_PCS_SGMII_SPEED_MASK            (1u << 11 | 1u << 10)
#define ALT_TSE_PCS_SGMII_SPEED_10M             (0u << 11 | 0u << 10)
#define ALT_TSE_PCS_SGMII_SPEED_100M            (0u << 11 | 1u << 10)
#define ALT_TSE_PCS_SGMII_SPEED_1000M           (1u << 11 | 0u << 10)
#define ALT_TSE_PCS_SGMII_DUPLEX_FULL           (1u << 12)
#define ALT_TSE_PCS_SGMII_ACK                   (1u << 14)
#define ALT_TSE_PCS_SGMII_LINK_UP               (1u << 15)

/// PCS interface mode register
#define ALT_TSE_PCS_IFMODE                      0x34

// SGMII/1000Base-X and SGMII/1000Base-X/100Base-FX adapter registers

/// Link status
#define SGMII_1000BASEX_LINK_STATUS_CONTROL             (1u << 1)
#define SGMII_1000BASEX_LINK_STATUS_SGMII_1000BASEX_UP  (1u << 2)
#define SGMII_1000BASEX_LINK_STATUS_100BASEFX_UP        (1u << 3)

/// PCS control register
#define SGMII_1000BASEX_REG_PCS_CONTROL                 0x08
#define SGMII_1000BASEX_PCS_CONTROL_IF_1000BASEX        ((0u << 0) | (0u << 1))
#define SGMII_1000BASEX_PCS_CONTROL_IF_SGMII            (1u << 0)
#define SGMII_1000BASEX_PCS_CONTROL_IF_100BASEFX        (1u << 1)
#define SGMII_1000BASEX_PCS_CONTROL_AUTONEG_ENABLE      (1u << 4)
#define SGMII_1000BASEX_PCS_CONTROL_AUTONEG_RESTART     (1u << 8)

/// PCS status register
#define SGMII_1000BASEX_REG_PCS_STATUS                  0x07
#define SGMII_1000BASEX_PCS_STATUS_AUTONEG_COMPLETE     (1u << 8)
#define SGMII_1000BASEX_PCS_STATUS_SPEED_MASK           ((1u << 10) | (1u << 9))
#define SGMII_1000BASEX_PCS_STATUS_SPEED_10M            ((0u << 10) | (0u << 9))
#define SGMII_1000BASEX_PCS_STATUS_SPEED_100M           ((0u << 10) | (1u << 9))
#define SGMII_1000BASEX_PCS_STATUS_SPEED_1000M          ((1u << 10) | (0u << 9))

/// PCS SGMII control register
#define SGMII_1000BASEX_REG_PCS_SGMII_CONTROL           0x09
#define SGMII_1000BASEX_PCS_SGMII_CONTROL_SPEED_MASK    ((1u << 1) | (1u << 0))
#define SGMII_1000BASEX_PCS_SGMII_CONTROL_SPEED_10M     ((0u << 1) | (0u << 0))
#define SGMII_1000BASEX_PCS_SGMII_CONTROL_SPEED_100M    ((0u << 1) | (1u << 0))
#define SGMII_1000BASEX_PCS_SGMII_CONTROL_SPEED_1000M   ((1u << 1) | (0u << 0))
#define SGMII_1000BASEX_PCS_SGMII_CONTROL_MODE_MAC      (0u << 4)
#define SGMII_1000BASEX_PCS_SGMII_CONTROL_MODE_PHY      (1u << 4)

/// PCS SGMII device configuration register
#define SGMII_1000BASEX_REG_PCS_SGMII_DEV_CONFIG        0x0a
#define SGMII_1000BASEX_PCS_SGMII_DEV_CONFIG_SPEED_MASK ((1u << 11) | (1u << 10))
#define SGMII_1000BASEX_PCS_SGMII_DEV_CONFIG_SPEED_10M  ((0u << 11) | (0u << 10))
#define SGMII_1000BASEX_PCS_SGMII_DEV_CONFIG_SPEED_100M ((0u << 11) | (1u << 10))
#define SGMII_1000BASEX_PCS_SGMII_DEV_CONFIG_SPEED_1000M        ((1u << 11) | (0u << 10))
#define SGMII_1000BASEX_PCS_SGMII_DEV_CONFIG_LINK_UP    (1u << 15)

/// PCS SGMII partner configuration register
#define SGMII_1000BASEX_REG_PCS_SGMII_PARTNER_CONFIG    0x0b

int deipce_init_adapter(struct deipce_port_priv *pp);
bool deipce_is_supported_by_adapter(struct deipce_port_priv *pp,
                                    enum link_mode link_mode);
enum link_mode deipce_best_adapter_link_mode(struct deipce_port_priv *pp);
void deipce_get_adapter_delays(struct deipce_port_priv *pp,
                               struct deipce_delay *delay);
void deipce_cleanup_adapter(struct deipce_port_priv *pp);

#endif
