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

#ifndef DEIPCE_HW_H
#define DEIPCE_HW_H

#include <linux/etherdevice.h>

#include "deipce_types.h"
#include "deipce_hw_type.h"

/// SMAC usage tracking bitmap bit number of SMAC entry
#define DEIPCE_SMAC_USED_BIT(row,col) ((row)*FRS_SMAC_TABLE_COLS + (col))

int deipce_init_crc40(struct deipce_drv_priv *drv);
void deipce_cleanup_crc40(struct deipce_drv_priv *drv);
uint16_t deipce_get_smac_row(struct deipce_dev_priv *dp,
                             const struct frs_smac_table_entry *entry);
void deipce_update_smac_usage(struct deipce_dev_priv *dp,
                              uint16_t row, uint16_t col,
                              uint16_t new_config,
                              uint16_t old_config);

static inline bool deipce_is_smac_used(struct deipce_dev_priv *dp,
                                       uint16_t row, uint16_t col)
{
    return test_bit(DEIPCE_SMAC_USED_BIT(row, col), dp->smac.used);
}

int deipce_init_vlan_fids(struct deipce_dev_priv *dp);
void deipce_cleanup_vlan_fids(struct deipce_dev_priv *dp);
bool deipce_is_vlan_fid_valid(struct deipce_dev_priv *dp, uint16_t fid);
void deipce_drop_all_vlans(struct deipce_dev_priv *dp);
uint16_t deipce_get_pvid(struct deipce_port_priv *pp);
void deipce_set_pvid(struct deipce_port_priv *pp, uint16_t vid);
int deipce_prepare_add_vlans(struct deipce_dev_priv *dp,
                             uint16_t vid_begin, uint16_t vid_end,
                             uint16_t flags);
void deipce_commit_add_vlans(struct deipce_dev_priv *dp,
                             struct deipce_port_priv *pp,
                             uint16_t vid_begin, uint16_t vid_end,
                             uint16_t flags);
void deipce_del_vlans(struct deipce_dev_priv *dp,
                      struct deipce_port_priv *pp,
                      uint16_t vid_begin, uint16_t vid_end);
int deipce_get_new_vlan_fid(struct deipce_dev_priv *dp, uint16_t *fid);
uint16_t deipce_get_vlan_fid(struct deipce_dev_priv *dp, uint16_t vid);
void deipce_add_vlan_fid(struct deipce_dev_priv *dp, uint16_t vid,
                         uint16_t fid);
void deipce_del_vlan_fid(struct deipce_dev_priv *dp, uint16_t vid,
                         uint16_t fid);
int deipce_set_vlan_fid(struct deipce_dev_priv *dp, uint16_t vid, uint16_t fid);
uint16_t deipce_get_vlan_ports(struct deipce_dev_priv *dp, uint16_t vid);
int deipce_fill_fid_vid_map(struct deipce_dev_priv *dp,
                            unsigned int *fid_vid_map,
                            unsigned int first,
                            unsigned int count);

void deipce_reset_port_vlan_config(struct deipce_port_priv *port,
                                   bool member_default);
unsigned int deipce_get_default_vlan_pcp(struct deipce_port_priv *pp);
int deipce_set_default_vlan_pcp(struct deipce_port_priv *pp, unsigned int pcp);

enum link_mode deipce_get_ext_link_mode(struct deipce_port_priv *pp);

int deipce_set_port_state(struct net_device *netdev, enum link_mode link_mode);

int deipce_set_port_stp_state(struct net_device *netdev, uint8_t stp_state);
uint8_t deipce_get_port_stp_state(struct net_device *netdev);

int deipce_get_mac_table(struct deipce_dev_priv *dp,
                         void (*new_entry)(struct deipce_dev_priv *dp,
                                           struct deipce_dmac_entry *dmac,
                                           void *arg),
                         void *arg);
int deipce_clear_mac_table(struct deipce_dev_priv *dp, uint16_t fid,
                           uint16_t port_mask);

int deipce_read_smac_entry(struct deipce_dev_priv *dp,
                           uint16_t row, uint16_t col,
                           struct frs_smac_table_entry *entry);
int deipce_write_smac_entry(struct deipce_dev_priv *dp,
                            uint16_t row, uint16_t col,
                            const struct frs_smac_table_entry *entry);
bool deipce_match_smac_entries(const struct frs_smac_table_entry *entry,
                               const struct frs_smac_table_entry *ref);
int deipce_get_smac_pos(struct deipce_dev_priv *dp,
                        struct frs_smac_table_entry *entry,
                        struct frs_smac_table_entry *cur_entry,
                        uint16_t *row,
                        uint16_t *col);

int deipce_read_policer(struct deipce_port_priv *pp,
                        unsigned int policer,
                        uint16_t *limit, uint16_t *rate_status);
int deipce_write_policer(struct deipce_port_priv *pp,
                         unsigned int policer,
                         uint16_t limit, uint16_t rate);

int deipce_write_port_ipo(struct deipce_port_priv *pp,
                          uint16_t entry,
                          uint16_t flags0,
                          uint16_t flags1,
                          uint16_t allow_mask,
                          uint16_t mirror_mask,
                          uint8_t priority,
                          const uint8_t addr[IFHWADDRLEN],
                          uint8_t compare_length);

bool deipce_get_cutthrough(struct deipce_port_priv *pp);
void deipce_set_cutthrough(struct deipce_port_priv *pp, bool enable);

void deipce_set_delays(struct deipce_port_priv *pp,
                       enum deipce_delay_type delay_type,
                       enum deipce_dir dir,
                       const unsigned int *delays);
void deipce_get_delays(struct deipce_port_priv *pp,
                       enum deipce_delay_type delay_type,
                       enum deipce_dir dir,
                       unsigned int *delays);
void deipce_update_delays(struct deipce_port_priv *pp);

#endif
