/** @file
 */

/*

   Kairos Linux driver

*/

#ifndef KAIROS_HW_TYPE_H
#define KAIROS_HW_TYPE_H

#include <linux/types.h>
#include <linux/if_ether.h>

/**
 * Link modes.
 */
enum link_mode {
    LM_DOWN,
    LM_10FULL,
    LM_100FULL,
    LM_1000FULL,
};

/// Number of link modes
#define KAIROS_LINK_MODE_COUNT (LM_1000FULL + 1)

/**
 * FRS dynamic MAC address table entry.
 */
struct kairos_dmac_entry {
    uint16_t port_num;                  ///< port number
    uint8_t mac_address[ETH_ALEN];      ///< MAC address
    uint16_t fid;                       ///< FID number
};

/**
 * Configuration time switch information.
 */
struct kairos_switch_config {
    const char *mac_name;               ///< underlying MAC net device name
    const char *ep_name;                ///< default endpoint net device name
};

/**
 * Configuration time switch port information.
 */
struct kairos_port_config {
    const char *name;                   ///< default switch port name
};

#endif
