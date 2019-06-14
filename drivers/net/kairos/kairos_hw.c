/** @file
 */

/*

   Kairos Linux driver

*/

/// Uncomment to enable debug messages
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/if_bridge.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include "kairos_main.h"
#include "kairos_types.h"
#include "kairos_if.h"
#include "kairos_fsc_main.h"
#include "kairos_adapter.h"
#include "kairos_hw.h"

/**
 * String representations of link modes.
 */
static const char *kairos_link_mode_str[] = {
    [LM_DOWN] = "DOWN",
    [LM_1000FULL] = "UP at 1000 Mbps",
    [LM_100FULL] = "UP at 100 Mbps",
    [LM_10FULL] = "UP at 10 Mbps",
};

/**
 * Calculate small (values for 4-bit data nibbles) CRC40 table.
 * It is used for SMAC hash.
 * @param drv Driver privates.
 */
int __init kairos_init_crc40(struct kairos_drv_priv *drv)
{
    const uint64_t polynomial = 0x0104c11db7ull;
    uint64_t c;
    unsigned int i;
    unsigned int j;

    // Allocate table for precalculated CRC40 values of each 4-bit data nibble.
    drv->crc40_table = kmalloc(16*sizeof(uint64_t), GFP_KERNEL);
    if (!drv->crc40_table) {
        pr_err(DRV_NAME ": Failed to allocate CRC40 table\n");
        return -ENOMEM;
    }

    // Calculate CRC40 data.
    for (i = 0; i < 16; i++) {
        c = (uint64_t)i << 36;
        for (j = 0; j < 4; j++) {
            if (c & 0x8000000000ull)
                c = (c << 1) ^ polynomial;
            else
                c <<= 1;
        }
        drv->crc40_table[i] = c & 0xffffffffffull;
    }

    return 0;
}

/**
 * Free CRC40 table.
 * @param drv Driver privates.
 */
void kairos_cleanup_crc40(struct kairos_drv_priv *drv)
{
    kfree(drv->crc40_table);
    drv->crc40_table = NULL;

    return;
}

/**
 * Calculate CRC40 value for data block using lookup table.
 * @param drv Driver privates.
 * @param crc CRC40 value thus far, or ~0ull to start a new.
 * @param data Data for which to calculate CRC40.
 * @param length Number of bytes in data.
 * @return Updated CRC40 value.
 */
static uint64_t kairos_calc_crc40(struct kairos_drv_priv *drv,
                                  uint64_t crc,
                                  const void *data, size_t length)
{
    const uint64_t *table = drv->crc40_table;
    const uint8_t *buf = data;
    unsigned int i;

    for (i = 0; i < length; i++) {
        crc = (crc << 4) ^
            table[(((uint8_t)(crc >> 36) & 0xfu) ^ (buf[i] >> 4))];
        crc = (crc << 4) ^
            table[(((uint8_t)(crc >> 36) & 0xfu) ^ (buf[i] & 0xfu))];
    }

    return crc & 0xffffffffffull;
}

/**
 * Initialize switch FID support.
 * @param dp Switch privates.
 */
int kairos_init_vlan_fids(struct kairos_dev_priv *dp)
{
    if (dp->features.fids > 0) {
        size_t alloc_size =
            (dp->features.fids + 1) * sizeof(*dp->vlan.fid_vlan_count);

        dp->vlan.fid_vlan_count = kzalloc(alloc_size, GFP_KERNEL);
        if (!dp->vlan.fid_vlan_count) {
            dev_err(dp->this_dev, "Failed to allocate FID VLAN counters\n");
            return -ENOMEM;
        }
    }

    return 0;
}

/**
 * Cleanup switch FID support.
 * @param dp Switch privates.
 */
void kairos_cleanup_vlan_fids(struct kairos_dev_priv *dp)
{
    if (dp->vlan.fid_vlan_count) {
        kfree(dp->vlan.fid_vlan_count);
        dp->vlan.fid_vlan_count = NULL;
    }

    return;
}

/**
 * Get VLAN memberships.
 * @param dp Switch privates.
 * @param vid VID number.
 * @return Bitmask of all ports in VLAN vid.
 */
uint16_t kairos_get_vlan_ports(struct kairos_dev_priv *dp, uint16_t vid)
{
    return kairos_read_switch_reg(dp, FRS_VLAN_CFG(vid));
}

/**
 * Set VLAN memberships.
 * @param dp Switch privates.
 * @param vid VID number.
 * @param port_mask Bitmask of all ports in VLAN vid.
 */
static inline void kairos_set_vlan_ports(struct kairos_dev_priv *dp,
                                         uint16_t vid, uint16_t port_mask)
{
    kairos_write_switch_reg(dp, FRS_VLAN_CFG(vid), port_mask);
}

/**
 * Check that FID value is valid.
 * @param dp Switch privates.
 * @param fid FID value to check.
 * @return True if fid is valid, false otherwise.
 */
bool kairos_is_vlan_fid_valid(struct kairos_dev_priv *dp, uint16_t fid)
{
    if (fid == 0 || fid > dp->features.fids)
        return -EINVAL;

    return true;
}

/**
 * Drop all VLAN memberships.
 * @param dp Switch privates.
 */
void kairos_drop_all_vlans(struct kairos_dev_priv *dp)
{
    uint16_t vid;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    for (vid = 0; vid <= VLAN_VID_MASK; vid++) {
        kairos_set_vlan_ports(dp, vid, 0);

        if (dp->features.fids > 0)
            kairos_write_switch_reg(dp, FRS_VLAN_FID(vid), 0);
    }

    return;
}

/**
 * Get current PVID for port.
 * @param pp Port privates.
 * @return VLAN ID value.
 */
uint16_t kairos_get_pvid(struct kairos_port_priv *pp)
{
    uint16_t port_vlan = kairos_read_port_reg(pp, PORT_REG_VLAN);

    return port_vlan & VLAN_VID_MASK;
}

/**
 * Configure new PVID for port.
 * @param pp Port privates.
 * @param vid VLAN ID.
 */
void kairos_set_pvid(struct kairos_port_priv *pp, uint16_t vid)
{
    // PVID defines also VLAN ID for priority-tagged frames.
    uint16_t port_vlan0_map = vid & VLAN_VID_MASK;
    uint16_t port_vlan;

    netdev_dbg(pp->netdev, "%s() Set PVID %hu\n", __func__, vid);

    port_vlan = kairos_read_port_reg(pp, PORT_REG_VLAN);

    port_vlan &= ~VLAN_VID_MASK;
    port_vlan |= port_vlan0_map;
    port_vlan |= PORT_VLAN_TAGGED;

    kairos_write_port_reg(pp, PORT_REG_VLAN, port_vlan);
    kairos_write_port_reg(pp, PORT_REG_VLAN0_MAP, port_vlan0_map);

    return;
}

/**
 * Prepare to add a range of VLANs.
 * After success, call kairos_commit_add_vlans to write changes to HW.
 * @param dp Switch privates.
 * @param vid_begin First VID.
 * @param vid_end Last VID.
 * @param flags BRIDGE_VLAN_INFO_xxx values.
 */
int kairos_prepare_add_vlans(struct kairos_dev_priv *dp,
                             uint16_t vid_begin, uint16_t vid_end,
                             uint16_t flags)
{
    unsigned int vlan_count = dp->vlan.count;
    uint16_t port_mask;
    uint16_t vid;

    dev_dbg(dp->this_dev, "%s() VID %hu .. %hu flags 0x%hx %s%s\n",
            __func__, vid_begin, vid_end, flags,
            flags & BRIDGE_VLAN_INFO_PVID ? "PVID " : "",
            flags & BRIDGE_VLAN_INFO_UNTAGGED ? "untagged " : "");

    // Pretend everything is allocated to FID 1 in case of no support for FIDs.
    if (dp->features.fids == 0)
        return 0;

    // Check tagging untagged frames at ingress configuration.
    if (flags & BRIDGE_VLAN_INFO_PVID) {
        // There can be only one.
        if (vid_begin != vid_end)
            return -EINVAL;

        // PVID tagged at egress is not supported.
        if (!(flags & BRIDGE_VLAN_INFO_UNTAGGED))
            return -EINVAL;
    }

    // Record new VIDs not already in use.
    bitmap_zero(dp->vlan.new_vids, VLAN_N_VID);

    /*
     * Enforce VLAN count limit so that each used VID can be allocated to any
     * available FID at any time.
     */
    for (vid = vid_begin; vid <= vid_end; vid++) {
        port_mask = kairos_get_vlan_ports(dp, vid);

        dev_dbg(dp->this_dev,
                "%s()  check VID %hu port_mask 0x%hx vlan_count %u\n",
                __func__, vid, port_mask, vlan_count);

        if (port_mask == 0) {
            if (vlan_count++ >= dp->features.fids)
                return -ENOSPC;

            set_bit(vid, dp->vlan.new_vids);
        }
    }

    return 0;
}

/**
 * Commit new VLANs and update tagging at ingress and egress configuration.
 * @param dp Switch privates.
 * @param pp Port privates.
 * @param vid_begin First VID.
 * @param vid_end Last VID.
 * @param flags BRIDGE_VLAN_INFO_xxx values.
 */
void kairos_commit_add_vlans(struct kairos_dev_priv *dp,
                             struct kairos_port_priv *pp,
                             uint16_t vid_begin, uint16_t vid_end,
                             uint16_t flags)
{
    uint16_t port_mask = 1u << pp->port_num;
    uint16_t old_port_mask;
    uint16_t vid;
    uint16_t fid;
    uint16_t port_tag;
    uint16_t old_pvid;

    dev_dbg(dp->this_dev, "%s() %s VID %hu .. %hu flags 0x%hx\n",
            __func__, netdev_name(pp->netdev), vid_begin, vid_end, flags);

    /*
     * Configure tagging untagged frames at ingress.
     * VLAN ID 4095 is used internally for "no PVID" case.
     */
    old_pvid = kairos_get_pvid(pp);
    if (flags & BRIDGE_VLAN_INFO_PVID)
        kairos_set_pvid(pp, vid_begin);
    else if (old_pvid >= vid_begin && old_pvid <= vid_end)
        kairos_set_pvid(pp, VLAN_VID_MASK);

    // Configure VLAN membership, FID and untagging at egress.
    for (vid = vid_begin; vid <= vid_end; vid++) {
        // Untagging at egress.
        port_tag = kairos_read_switch_reg(dp, FRS_VLAN_TAG(vid));
        if (flags & BRIDGE_VLAN_INFO_UNTAGGED)
            port_tag |= port_mask;
        else
            port_tag &= ~port_mask;
        kairos_write_switch_reg(dp, FRS_VLAN_TAG(vid), port_tag);

        if (test_bit(vid, dp->vlan.new_vids)) {
            // New VID. We know there are no ports in this VLAN yet.
            kairos_set_vlan_ports(dp, vid, port_mask);
            kairos_get_new_vlan_fid(dp, &fid);
            kairos_add_vlan_fid(dp, vid, fid);
        }
        else {
            // VID already in use.
            old_port_mask = kairos_get_vlan_ports(dp, vid);
            kairos_set_vlan_ports(dp, vid, old_port_mask | port_mask);
        }
    }

    return;
}

/**
 * Remove port from VLANs.
 * @param dp Switch privates.
 * @param pp Port privates.
 * @param vid_begin First VID.
 * @param vid_end Last VID.
 * @param flags BRIDGE_VLAN_INFO_xxx values.
 */
void kairos_del_vlans(struct kairos_dev_priv *dp,
                      struct kairos_port_priv *pp,
                      uint16_t vid_begin, uint16_t vid_end)
{
    uint16_t old_pvid;
    uint16_t old_port_mask;
    uint16_t new_port_mask;
    uint16_t vid;
    uint16_t fid;

    netdev_dbg(pp->netdev, "%s() DEL %hu .. %hu\n",
               __func__, vid_begin, vid_end);

    // VLAN membership configuration.
    for (vid = vid_begin; vid <= vid_end; vid++) {
        old_port_mask = kairos_get_vlan_ports(dp, vid);
        new_port_mask = old_port_mask & ~(1u << pp->port_num);
        kairos_set_vlan_ports(dp, vid, new_port_mask);
        if (old_port_mask == 1u << pp->port_num) {
            fid = kairos_get_vlan_fid(dp, vid);
            kairos_del_vlan_fid(dp, vid, fid);
        }
    }

    /*
     * Handle deleting current PVID, VLAN ID 4095 is used internally
     * for "no PVID" case.
     */
    old_pvid = kairos_get_pvid(pp);
    if (old_pvid >= vid_begin && old_pvid <= vid_end)
        kairos_set_pvid(pp, VLAN_VID_MASK);

    return;
}

/**
 * Get FID for a new VLAN.
 * Internally FID numbering starts from zero, but to user it starts from 1.
 * @param dp Switch privates.
 * @param fid Place for new FID.
 */
int kairos_get_new_vlan_fid(struct kairos_dev_priv *dp, uint16_t *fid)
{
    // Pretend everything is allocated to FID 1 in case of no support for FIDs.
    if (dp->features.fids == 0) {
        *fid = 1;
        return 0;
    }

    /*
     * Enforce VLAN count limit so that each used VID can be allocated to any
     * available FID at any time.
     */
    if (dp->vlan.count >= dp->features.fids)
        return -ENOSPC;

    // Allocate each new VLAN from first FID that does not yet have any VLANs.
    for (*fid = 1; *fid <= dp->features.fids; (*fid)++) {
        if (dp->vlan.fid_vlan_count[*fid] == 0)
            return 0;
    }

    return -ENOSPC;
}

/**
 * Get FID a VID is allocated to.
 * @param dp Switch privates.
 * @param vid VID number whose FID to get.
 */
uint16_t kairos_get_vlan_fid(struct kairos_dev_priv *dp, uint16_t vid)
{
    // Pretend everything is allocated to FID 1 in case of no support for FIDs.
    uint16_t fid = 1;

    if (dp->features.fids > 0) {
        fid = kairos_read_switch_reg(dp, FRS_VLAN_FID(vid));
        fid &= dp->features.fids - 1;
        fid++;
    }

    return fid;
}

/**
 * Add new VLAN to FID.
 * @param dp Switch privates.
 * @param vid VID number.
 * @param fid FID number to allocated VID to.
 */
void kairos_add_vlan_fid(struct kairos_dev_priv *dp, uint16_t vid,
                         uint16_t fid)
{
    // Pretend everything is allocated to FID 1 in case of no support for FIDs.
    if (dp->features.fids == 0)
        return;

    dp->vlan.count++;
    dp->vlan.fid_vlan_count[fid]++;
    kairos_write_switch_reg(dp, FRS_VLAN_FID(vid), fid - 1);
    kairos_switchdev_add_vlan_fid(dp, vid, fid);

    dev_dbg(dp->this_dev, "%s() add vid %u to fid %u vlans %u total %u\n",
            __func__, vid, fid, dp->vlan.fid_vlan_count[fid], dp->vlan.count);

    return;
}

/**
 * Remove VLAN from FID.
 * @param dp Switch privates.
 * @param vid VID number.
 * @param fid FID number VID is allocated to.
 */
void kairos_del_vlan_fid(struct kairos_dev_priv *dp, uint16_t vid,
                         uint16_t fid)
{
    if (dp->features.fids == 0)
        return;

    dp->vlan.count--;
    dp->vlan.fid_vlan_count[fid]--;
    kairos_switchdev_del_vlan_fid(dp, vid, fid);

    dev_dbg(dp->this_dev, "%s() del vid %u from fid %u vlans %u total %u\n",
            __func__, vid, fid, dp->vlan.fid_vlan_count[fid], dp->vlan.count);

    return;
}

/*
 * Reallocate VID to a new FID.
 * @param dp Switch privates.
 * @param vid VID number.
 * @param fid FID number VID is allocated to.
 */
int kairos_set_vlan_fid(struct kairos_dev_priv *dp, uint16_t vid, uint16_t fid)
{
    uint16_t old_fid;

    if (!kairos_is_vlan_fid_valid(dp, fid))
        return -EINVAL;

    if (kairos_get_vlan_ports(dp, vid) == 0)
        return -EINVAL;

    old_fid = kairos_get_vlan_fid(dp, vid);

    dev_dbg(dp->this_dev, "%s() move vid %u from fid %u to fid %u\n",
            __func__, vid, old_fid, fid);

    if (old_fid == fid)
        return 0;

    // Order is significant.
    kairos_add_vlan_fid(dp, vid, fid);
    kairos_del_vlan_fid(dp, vid, old_fid);

    return 0;
}

/**
 * Fill VLAN FID to VID map.
 * @param dp Switch privates.
 * @param fid_vid_map Map to fill. Map contains fixed sized list of VIDs
 * in each FID, like in array fid_vid_map[FID][VLAN], where
 * FID = 1,2,3 .. FIDS and VLAN = 0,1,2, .. FIDS - 1.
 * Values are VID numbers, zero denoting an unused entry.
 * @param first Index of first entry in map.
 * @param count Number of entries to fill.
 */
int kairos_fill_fid_vid_map(struct kairos_dev_priv *dp,
                            unsigned int *fid_vid_map,
                            unsigned int first,
                            unsigned int count)
{
    const unsigned int fid_elems = dp->features.fids + 1;
    const unsigned int end = first + count;
    unsigned int index;
    uint16_t vid;
    uint16_t fid;
    uint16_t vlan;

    dev_dbg(dp->this_dev, "%s() first %u count %u end %u\n",
            __func__, first, count, end);

    // FID 0 is invalid.
    if (first < dp->features.fids)
        return -EINVAL;

    memset(dp->vlan.fid_vlan_count, 0,
           fid_elems * sizeof(*dp->vlan.fid_vlan_count));
    memset(fid_vid_map, 0, count * sizeof(*fid_vid_map));

    for (vid = 1; vid < VLAN_VID_MASK; vid++) {
        if (kairos_get_vlan_ports(dp, vid) == 0)
            continue;

        fid = kairos_get_vlan_fid(dp, vid);
        vlan = dp->vlan.fid_vlan_count[fid]++;
        index = fid * dp->features.fids + vlan;
        if (index >= first && index < end)
            fid_vid_map[index - first] = vid;
    }

    return 0;
}

/**
 * Initialize port VLAN configuration to driver defaults.
 * @param port Port privates.
 * @param member_default Whether or not to make port member of default_vlan.
 */
void kairos_reset_port_vlan_config(struct kairos_port_priv *port,
                                   bool member_default)
{
    struct kairos_dev_priv *dp = port->dp;
    const uint16_t default_vlan = 1;
    const uint16_t vlan0_map = default_vlan;
    uint16_t port_vlan = PORT_VLAN_TAGGED | default_vlan;
    uint16_t new_port_mask = 0;
    uint16_t old_port_mask;
    uint16_t vid;
    uint16_t fid = 0;

    dev_dbg(dp->this_dev, "%s() port %u member %s\n",
            __func__, port->port_num, member_default ? "default" : "none");

    // Retain PCP bits.
    port_vlan = kairos_read_port_reg(port, PORT_REG_VLAN);
    port_vlan |= PORT_VLAN_TAGGED;
    port_vlan &= ~VLAN_VID_MASK;
    port_vlan |= default_vlan;

    kairos_write_port_reg(port, PORT_REG_VLAN, port_vlan);
    kairos_write_port_reg(port, PORT_REG_VLAN0_MAP, vlan0_map);

    for (vid = 0; vid <= VLAN_VID_MASK; vid++) {
        old_port_mask = kairos_get_vlan_ports(dp, vid);

        if ((vid == default_vlan) && member_default)
            new_port_mask = old_port_mask | (1u << port->port_num);
        else
            new_port_mask = old_port_mask & ~(1u << port->port_num);

        if (new_port_mask != old_port_mask) {
            kairos_set_vlan_ports(dp, vid, new_port_mask);

            if (dp->features.fids > 0) {
                if (new_port_mask == 0) {
                    // VID no longer in use.
                    fid = kairos_get_vlan_fid(dp, vid);
                    kairos_del_vlan_fid(dp, vid, fid);
                }
                else if (old_port_mask == 0) {
                    // Default VID taken into use.
                    kairos_get_new_vlan_fid(dp, &fid);
                    kairos_add_vlan_fid(dp, vid, fid);
                }
            }
        }
    }

    return;
}

/**
 * Get default VLAN priority code point for port.
 * @param pp Port privates.
 * @return Port default PCP.
 */
unsigned int kairos_get_default_vlan_pcp(struct kairos_port_priv *pp)
{
    uint16_t port_vlan = kairos_read_port_reg(pp, PORT_REG_VLAN);

    return PORT_VLAN_PCP_GET(port_vlan);
}

/**
 * Set default VLAN priority code point for port.
 * @param pp Port privates.
 * @param pcp New default PCP value.
 */
int kairos_set_default_vlan_pcp(struct kairos_port_priv *pp, unsigned int pcp)
{
    uint16_t port_vlan;

    if (pcp > PORT_VLAN_PCP_MAX)
        return -EINVAL;

    port_vlan = kairos_read_port_reg(pp, PORT_REG_VLAN);
    port_vlan &= ~PORT_VLAN_PCP_MASK;
    port_vlan |= PORT_VLAN_PCP(pcp);
    kairos_write_port_reg(pp, PORT_REG_VLAN, port_vlan);

    return 0;
}

/**
 * Get current link mode from external/automatic speed detection signals.
 * @param pp Port privates.
 * @return Link mode.
 */
enum link_mode kairos_get_ext_link_mode(struct kairos_port_priv *pp)
{
    uint16_t data = kairos_read_port_reg(pp, PORT_REG_STATE);

    switch (data & PORT_STATE_CURRENT_MASK) {
    case PORT_STATE_CURRENT_1000MBPS:
        return LM_1000FULL;
    case PORT_STATE_CURRENT_100MBPS:
        return LM_100FULL;
    case PORT_STATE_CURRENT_10MBPS:
        return LM_10FULL;
    }

    return LM_DOWN;
}

/**
 * Adjust FRS port link state to account for link status, speed and STP state.
 * Link mode mutex must be held when calling this.
 * @param netdev Netdevice associated with an FRS port.
 * @param link_mode New link mode for port.
 * @return 0 on success or negative error code
 */
int kairos_set_port_state(struct net_device *netdev, enum link_mode link_mode)
{
    struct kairos_netdev_priv *np = netdev_priv(netdev);
    struct kairos_port_priv *pp = np->pp;
    uint16_t orig_state = 0;
    uint16_t state = 0;

    netdev_dbg(netdev, "Set link mode %i previous mode %i PHY %s\n",
               link_mode, np->link_mode,
               pp->ext_phy.phydev ? pp->ext_phy.phydev->drv->name : "none");

    orig_state = kairos_read_port_reg(pp, PORT_REG_STATE);
    state = orig_state;
    state &= ~PORT_STATE_SPEED_MASK;
    state &= ~PORT_STATE_STATE_MASK;
    state &= ~PORT_STATE_GMII;

    // Speed selection.
    if (pp->flags & kairos_PORT_SPEED_EXT) {
        state |= PORT_STATE_SPEED_EXT;
    }
    else {
        switch (link_mode) {
        case LM_1000FULL:
            state |= PORT_STATE_1000MBPS | PORT_STATE_GMII;
            break;

        case LM_100FULL:
            state |= PORT_STATE_100MBPS | PORT_STATE_MII;
            break;

        case LM_10FULL:
            state |= PORT_STATE_10MBPS | PORT_STATE_MII;
            break;

        case LM_DOWN:
        default:
            // Do not fall back to external/automatic speed selection.
            state |= PORT_STATE_10MBPS | PORT_STATE_MII;
        }
    }

    // Port forwarding state selection.
    if (link_mode == LM_DOWN) {
        state |= PORT_STATE_DISABLED;
    }
    else {
        switch (np->stp_state) {
        case BR_STATE_FORWARDING:
            state |= PORT_STATE_FORWARDING;
            break;
        case BR_STATE_LEARNING:
            state |= PORT_STATE_LEARNING;
            break;
        case BR_STATE_DISABLED:
        case BR_STATE_BLOCKING:
        case BR_STATE_LISTENING:
        default:
            state |= PORT_STATE_DISABLED;
            break;
        }
    }

    np->link_mode = link_mode;

    if (state != orig_state)
        kairos_write_port_reg(pp, PORT_REG_STATE, state);

    // Update adapter link status.
    if (pp->adapter.ops.update_link)
        pp->adapter.ops.update_link(pp);

    // Update scheduler.
    if (pp->sched.fsc)
        kairos_fsc_update_link(pp->sched.fsc, pp->sched.num, netdev);

    if (link_mode == LM_DOWN) {
        netif_carrier_off(netdev);
    }
    else {
        netif_carrier_on(netdev);
    }

    if (state != orig_state) {
        netif_info(np, link, netdev,
                   "Link is %s (PORT_STATE: 0x%x)\n",
                   kairos_link_mode_str[link_mode],
                   state);
    }

    return 0;
}

/**
 * Set FES port forwarding state according to STP state.
 * @param netdev FES port netdevice.
 * @param stp_state New port forwarding state, BR_STATE_xxx.
 * Type is uint8_t for direct compatibility with netdevice operation callback.
 */
int kairos_set_port_stp_state(struct net_device *netdev, uint8_t stp_state)
{
    struct kairos_netdev_priv *np = netdev_priv(netdev);
    int ret;

    netdev_dbg(netdev, "%s() Set STP state to %u\n", __func__, stp_state);

    mutex_lock(&np->link_mode_lock);
    np->stp_state = stp_state;
    ret = kairos_set_port_state(netdev, np->link_mode);
    mutex_unlock(&np->link_mode_lock);

    return ret;
}

/**
 * Get FES port STP state.
 * @param netdev FES port netdevice.
 * @return Port STP state, BR_STATE_xxx.
 */
uint8_t kairos_get_port_stp_state(struct net_device *netdev)
{
    struct kairos_netdev_priv *np = netdev_priv(netdev);
    uint8_t stp_state;

    netdev_dbg(netdev, "%s() Set STP state to %u\n", __func__, stp_state);

    mutex_lock(&np->link_mode_lock);
    stp_state = np->stp_state;
    mutex_unlock(&np->link_mode_lock);

    return stp_state;
}

/**
 * Helper function to wait for FRS to complete fetching next MAC address.
 * @param dp FRS device privates.
 * @return Register MAC_TABLE0 value when finished, negative on error.
 */
static int kairos_wait_mac_table_transfer(struct kairos_dev_priv *dp)
{
    unsigned int timeout = 100;
    uint16_t data;

    for (;;) {
        if (timeout-- == 0) {
            dev_err(dp->this_dev, "MAC table transfer timeout\n");
            break;
        }

        data = kairos_read_switch_reg(dp, FRS_REG_MAC_TABLE(0));
        if (!(data & FRS_MAC_TABLE0_TRANSFER))
            return data;

        /*
         * It takes some time in the order of 0 .. 100 us before
         * result is available. Have enough slack.
         */
        usleep_range(100, 200);
    }

    return -EBUSY;
}

/**
 * Helper function to read one FRS MAC address table entry.
 * @param dp FRS device privates.
 * @param port_number Place for port number the MAC address was seen.
 * @param addr Place for MAC address, must be ETH_ALEN long.
 * @return Zero on success, negative on error, positive at end of table.
 */
static int kairos_read_next_mac_addr(struct kairos_dev_priv *dp,
                                     struct kairos_dmac_entry *dmac)
{
    static const uint8_t end_mac_addr[ETH_ALEN] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff
    };
    int ret;
    uint16_t data;
    unsigned int i;

    kairos_write_switch_reg(dp, FRS_REG_MAC_TABLE(0),
                            FRS_MAC_TABLE0_TRANSFER);

    ret = kairos_wait_mac_table_transfer(dp);
    if (ret < 0)
        return ret;

    dmac->port_num = ret & FRS_MAC_TABLE0_PORT_MASK;
    for (i = 0; i < 3; i++) {
        data = kairos_read_switch_reg(dp, FRS_REG_MAC_TABLE(i + 1));
        dmac->mac_address[i*2 + 0] = (data >> 0) & 0xff;
        dmac->mac_address[i*2 + 1] = (data >> 8) & 0xff;
    }

    if (dp->features.fids > 0) {
        dmac->fid = kairos_read_switch_reg(dp, FRS_REG_MAC_TABLE(4)) &
            FRS_MAC_TABLE4_FID_MASK;
        dmac->fid++;
    }
    else {
        dmac->fid = 0;
    }

    if (memcmp(dmac->mac_address, end_mac_addr, ETH_ALEN) == 0)
        return 1;

    return 0;
}

/**
 * Get FRS MAC table.
 * @param dp Device data
 * @param new_entry Callback function to call for all entries, or NULL
 * @param arg Argument for callback function
 * @return Negative on error or total number of found entries
 */
int kairos_get_mac_table(struct kairos_dev_priv *dp,
                         void (*new_entry)(struct kairos_dev_priv *dp,
                                           struct kairos_dmac_entry *dmac,
                                           void *arg),
                         void *arg)
{
    struct kairos_dmac_entry dmac = { .port_num = 0 };
    // Absolute maximum number of MAC addresses, just in case.
    const unsigned int max_addr_count = 1*1024*1024;
    unsigned int total_count = 0;
    int ret = 0;

    mutex_lock(&dp->common_reg_lock);

    for (;;) {
        if (total_count >= max_addr_count) {
            dev_err(dp->this_dev, "Too many MAC address table entries\n");
            break;
        }

        ret = kairos_read_next_mac_addr(dp, &dmac);
        if (ret)
            break;

        if (new_entry) {
            new_entry(dp, &dmac, arg);
        }

        total_count++;
    }

    mutex_unlock(&dp->common_reg_lock);

    if (ret < 0)
        return ret;
    return total_count;
}

/**
 * Clear MAC table entries for given port or ports.
 * @param dp Device privates.
 * @param fid FID number whose entries to clear.
 * @param port_mask Bitmask of ports whose MAC entries to clear.
 * @return Zero or negative error code.
 */
int kairos_clear_mac_table(struct kairos_dev_priv *dp, uint16_t fid,
                           uint16_t port_mask)
{
    unsigned int timeout = 1000;
    uint16_t data;
    int ret;

    // Synchronize accesses to switch general register.
    mutex_lock(&dp->common_reg_lock);

    kairos_write_switch_reg(dp, FRS_REG_MAC_TABLE_CLEAR_MASK, port_mask);
    kairos_write_switch_reg(dp, FRS_REG_MAC_TABLE_CLEAR_FID, fid - 1);

    data = kairos_read_switch_reg(dp, FRS_REG_GEN);
    kairos_write_switch_reg(dp, FRS_REG_GEN, data | FRS_GEN_CLEAR_MAC_TABLE);

    // Wait clear completion.
    do {
        if (timeout-- == 0) {
            dev_err(dp->this_dev, "Failed to clear MAC address table\n");
            ret = -EBUSY;
            goto out;
        }
        schedule();

        data = kairos_read_switch_reg(dp, FRS_REG_GEN);
    } while (data & FRS_GEN_CLEAR_MAC_TABLE);

    ret = 0;

out:
    mutex_unlock(&dp->common_reg_lock);

    return ret;
}

/**
 * Helper function to wait for FRS to complete fetching/storing policer entry.
 * @param pp FRS port privates.
 * @return Register POLICER_CMD value when finished, negative on error.
 */
static int kairos_wait_policer_transfer(struct kairos_port_priv *pp)
{
    unsigned int timeout = 1000;
    uint16_t data;

    for (;;) {
        if (timeout-- == 0) {
            netdev_err(pp->netdev, "Policer transfer timeout\n");
            break;
        }

        data = kairos_read_port_reg(pp, PORT_REG_POLICER_CMD);
        if (!(data & PORT_POLICER_CMD_TRANSFER))
            return data;

        // No such delay as with dynamic MAC address table.
        cpu_relax();
    }

    return -EBUSY;
}

/**
 * Read port policer registers.
 * @param pp Port privates.
 * @param policer Policer number to whose registers to read.
 * @param limit Place for limit register (POLICER0).
 * @param rate_status Place for rate and drop status register (POLICER1).
 */
int kairos_read_policer(struct kairos_port_priv *pp,
                        unsigned int policer,
                        uint16_t *limit, uint16_t *rate_status)
{
    struct kairos_dev_priv *dp = pp->dp;
    int ret;

    *limit = 0xffffu;
    *rate_status = 0xffffu;

    if (dp->features.policers == 0)
        return -EOPNOTSUPP;
    if (policer >= dp->features.policers)
        return -EINVAL;

    mutex_lock(&pp->port_reg_lock);

    kairos_write_port_reg(pp, PORT_REG_POLICER_CMD,
                          policer |
                          PORT_POLICER_CMD_READ |
                          PORT_POLICER_CMD_TRANSFER);

    ret = kairos_wait_policer_transfer(pp);
    if (ret < 0)
        goto out;

    *limit = kairos_read_port_reg(pp, PORT_REG_POLICER0);
    *rate_status = kairos_read_port_reg(pp, PORT_REG_POLICER1);

    ret = 0;

out:
    mutex_unlock(&pp->port_reg_lock);

    return ret;
}

/**
 * Write port policer registers.
 * @param pp Port privates.
 * @param policer Policer number to whose registers to write.
 * @param limit Limit register (POLICER0).
 * @param rate Rate register (POLICER1).
 */
int kairos_write_policer(struct kairos_port_priv *pp,
                         unsigned int policer,
                         uint16_t limit, uint16_t rate)
{
    struct kairos_dev_priv *dp = pp->dp;
    int ret;

    if (dp->features.policers == 0)
        return -EOPNOTSUPP;
    if (policer >= dp->features.policers)
        return -EINVAL;

    mutex_lock(&pp->port_reg_lock);

    kairos_write_port_reg(pp, PORT_REG_POLICER0, limit);
    kairos_write_port_reg(pp, PORT_REG_POLICER1, rate);
    kairos_write_port_reg(pp, PORT_REG_POLICER_CMD,
                          policer |
                          PORT_POLICER_CMD_WRITE |
                          PORT_POLICER_CMD_TRANSFER);

    ret = kairos_wait_policer_transfer(pp);
    if (ret < 0)
        goto out;

    ret = 0;

out:
    mutex_unlock(&pp->port_reg_lock);

    return ret;
}

/**
 * Calculate SMAC row for a MAC address (hash based SMAC only).
 * @param dp FES device privates.
 * @param mac_address MAC address whose row number to calculate.
 * @param column SMAC table column number for which to determine row number.
 * @param vlan VLAN ID to use in hash, ignored if configured row selection
 * algorithm for the column does not use it.
 * @return SMAC row number of MAC address, column and VLAN tuple.
 */
static uint16_t kairos_calc_smac_row(struct kairos_dev_priv *dp,
                                     const uint8_t *mac_address,
                                     uint16_t column,
                                     uint16_t vlan)
{
    struct kairos_drv_priv *drv = kairos_get_drv_priv();

    // Calculate CRC40 of the MAC address.
    uint64_t crc40 = kairos_calc_crc40(drv, ~(uint64_t)0,
                                       mac_address, ETH_ALEN);

    /*
     * Split the 12-bit column hashes of CRC40 to a 64-bit value.
     * Use 16 bits (not 12) for each column hash.
     */
    uint64_t hash =
        ((crc40 << 18) & ((uint64_t)0x3ffu << 48)) |
        ((crc40 << 12) & ((uint64_t)0x3ffu << 32)) |
        ((crc40 << 6) & ((uint64_t)0x3ffu << 16)) |
        ((crc40 << 0) & ((uint64_t)0x3ffu << 0));

    // Add the two extra bits from MAC address to each column hash.
    uint64_t extra = (mac_address[3] ^ mac_address[4]) & 0x3u;

    hash |=
        ((extra << (10 + 3*16))) |
        ((extra << (10 + 2*16))) |
        ((extra << (10 + 1*16))) |
        ((extra << (10 + 0*16)));

    // XOR each column hash with VLAN ID.
    if (dp->smac.cfg.row_sel[column] == FLX_FRS_SMAC_ROW_SEL_VLAN) {
        hash ^=
            ((uint64_t)vlan << 48) |
            ((uint64_t)vlan << 32) |
            ((uint64_t)vlan << 16) |
            ((uint64_t)vlan << 0);
    }

    return (hash >> (column*16)) & (dp->features.smac_rows - 1u);
}

/**
 * Determine row for an SMAC table entry, regardless of SMAC type.
 * @param dp FES device privates.
 * @param entry SMAC table entry.
 * @return Row that should be used for SMAC table entry.
 */
uint16_t kairos_get_smac_row(struct kairos_dev_priv *dp,
                             const struct frs_smac_table_entry *entry)
{
    return kairos_calc_smac_row(dp, entry->mac_address, entry->column,
                                entry->vlan);
}

/**
 * Update SMAC entry usage information after writing an entry.
 * @param dp FES device privates.
 * @param row SMAC row number.
 * @param col SMAC column number.
 * @param new_config New SMAC entry config bits.
 * @param old_config Old SMAC entry config bits.
 */
void kairos_update_smac_usage(struct kairos_dev_priv *dp,
                              uint16_t row, uint16_t col,
                              uint16_t new_config,
                              uint16_t old_config)
{
    if (old_config & FRS_SMAC_CONFIG_ENABLED) {
        if (old_config & FRS_SMAC_CONFIG_VLAN)
            dp->smac.col_count[col].vlan--;
        else
            dp->smac.col_count[col].no_vlan--;
    }

    if (new_config & FRS_SMAC_CONFIG_ENABLED) {
        if (new_config & FRS_SMAC_CONFIG_VLAN)
            dp->smac.col_count[col].vlan++;
        else
            dp->smac.col_count[col].no_vlan++;
        set_bit(DEIPCE_SMAC_USED_BIT(row, col), dp->smac.used);
    }
    else {
        clear_bit(DEIPCE_SMAC_USED_BIT(row, col), dp->smac.used);
    }

    return;
}

/**
 * Helper function to wait for FRS to complete fetching/storing SMAC entry.
 * @param dp FRS device privates.
 * @return Register SMAC_CMD value when finished, negative on error.
 */
static int kairos_wait_smac_table_transfer(struct kairos_dev_priv *dp)
{
    unsigned int timeout = 1000;
    uint16_t data;

    for (;;) {
        if (timeout-- == 0) {
            dev_err(dp->this_dev, "SMAC table transfer timeout\n");
            break;
        }

        data = kairos_read_switch_reg(dp, FRS_REG_SMAC_CMD);
        if (!(data & FRS_SMAC_CMD_TRANSFER))
            return data;

        // No such delay as with dynamic MAC address table.
        cpu_relax();
    }

    return -EBUSY;
}

/**
 * Read one static MAC address table entry from registers.
 * smac_table_lock must be held when calling this.
 * @param dp FRS device privates.
 * @param row Row in the SMAC table.
 * @param column Column in the SMAC table.
 * @param entry Place for SMAC table entry values.
 * @return Zero on success, negative on error.
 */
int kairos_read_smac_entry(struct kairos_dev_priv *dp,
                           uint16_t row, uint16_t column,
                           struct frs_smac_table_entry *entry)
{
    int ret = -EINVAL;
    uint16_t data = FRS_SMAC_CMD_TRANSFER
        | FRS_SMAC_CMD_READ
        | (row << FRS_SMAC_CMD_ROW_SHIFT)
        | (column << FRS_SMAC_CMD_COLUMN_SHIFT);
    unsigned int i;

    if (row >= dp->features.smac_rows || column >= FRS_SMAC_TABLE_COLS) {
        dev_dbg(dp->this_dev, "SMAC entry (row %u, col %u) out of bounds\n",
                row, column);
        return -EINVAL;
    }

    kairos_write_switch_reg(dp, FRS_REG_SMAC_CMD, data);

    ret = kairos_wait_smac_table_transfer(dp);
    if (ret < 0)
        return ret;

    for (i = 0; i < 3; i++) {
        data = kairos_read_switch_reg(dp, FRS_REG_SMAC_ADDR(i));
        entry->mac_address[i*2 + 0] = (data >> 0) & 0xff;
        entry->mac_address[i*2 + 1] = (data >> 8) & 0xff;
    }

    entry->column = column;
    entry->config = kairos_read_switch_reg(dp, FRS_REG_SMAC_CONFIG);
    entry->fwd_mask = kairos_read_switch_reg(dp, FRS_REG_SMAC_FWD_MASK);
    if (dp->features.policers > 0) {
        entry->policed_mask =
            kairos_read_switch_reg(dp, FRS_REG_SMAC_POLICED_MASK);
        entry->policer =
            kairos_read_switch_reg(dp, FRS_REG_SMAC_POLICER);
    }
    else {
        entry->policed_mask = 0xffffu;
        entry->policer = 0xffffu;
    }
    entry->vlan = kairos_read_switch_reg(dp, FRS_REG_SMAC_VLAN) &
        VLAN_VID_MASK;

    dev_dbg(dp->this_dev, "Read SMAC row %u col %u: %pM VLAN %u\n",
            row, column, entry->mac_address, entry->vlan);

    return 0;
}

/**
 * Write one static MAC address table entry to registers.
 * smac_table_lock must be held when calling this.
 * @param dp FRS device privates.
 * @param row Row in the SMAC table.
 * @param column Column in the SMAC table.
 * @param entry SMAC table entry values to write.
 * @return Zero on success, negative on error.
 */
int kairos_write_smac_entry(struct kairos_dev_priv *dp,
                            uint16_t row, uint16_t column,
                            const struct frs_smac_table_entry *entry)
{
    int ret = -EINVAL;
    uint16_t cmd = FRS_SMAC_CMD_TRANSFER |
        FRS_SMAC_CMD_WRITE |
        (row << FRS_SMAC_CMD_ROW_SHIFT) |
        (column << FRS_SMAC_CMD_COLUMN_SHIFT);
    uint16_t data = 0;
    unsigned int i;

    if (row >= dp->features.smac_rows || column > FRS_SMAC_TABLE_COLS) {
        dev_err(dp->this_dev, "SMAC entry (row %u, col %u) out of bounds\n",
                row, column);
        return -EINVAL;
    }

    dev_dbg(dp->this_dev, "Write SMAC row %u col %u: %pM VLAN %u\n",
            row, column, entry->mac_address, entry->vlan);

    for (i = 0; i < 3; i++) {
        data =
            ((uint16_t)entry->mac_address[i*2 + 0] << 0) |
            ((uint16_t)entry->mac_address[i*2 + 1] << 8);
        kairos_write_switch_reg(dp, FRS_REG_SMAC_ADDR(i), data);
    }

    kairos_write_switch_reg(dp, FRS_REG_SMAC_CONFIG, entry->config);
    kairos_write_switch_reg(dp, FRS_REG_SMAC_FWD_MASK, entry->fwd_mask);

    // NOTE: Must not write to reserved registers.
    if (dp->features.policers > 0) {
        kairos_write_switch_reg(dp, FRS_REG_SMAC_POLICED_MASK,
                                entry->policed_mask);
        kairos_write_switch_reg(dp, FRS_REG_SMAC_POLICER,
                                entry->policer);
    }
    kairos_write_switch_reg(dp, FRS_REG_SMAC_VLAN,
                            entry->vlan & VLAN_VID_MASK);

    kairos_write_switch_reg(dp, FRS_REG_SMAC_CMD, cmd);

    ret = kairos_wait_smac_table_transfer(dp);
    if (ret < 0)
        return ret;

    return 0;
}

/**
 * Determine if an SMAC entry matches another.
 * Takes into account MAC address, match VLAN flag and VLAN ID.
 * Enable flags are ignored by this function.
 * @param entry SMAC entry to compare.
 * @param ref Reference SMAC entry.
 * @return True if entry and ref would match the same frames.
 */
bool kairos_match_smac_entries(const struct frs_smac_table_entry *entry,
                               const struct frs_smac_table_entry *ref)
{
    if (!ether_addr_equal(entry->mac_address, ref->mac_address))
        return false;
    if ((entry->config & FRS_SMAC_CONFIG_VLAN) !=
        (ref->config & FRS_SMAC_CONFIG_VLAN))
        return false;
    if ((entry->config & FRS_SMAC_CONFIG_VLAN) &&
        (entry->vlan != ref->vlan))
        return false;

    return true;
}

/**
 * Get location (row, column) for an SMAC table entry.
 * @param dp FES device privates.
 * @param entry SMAC table entry to add or modify, with the following set:
 * - MAC address
 * - VLAN ID
 * - config flags
 * Field column will also be updated to column number to use.
 * @param cur_entry Place for current entry information in case an existing
 * entry would be updated. Config flags (use entry bit) can be used to detect.
 * Can be NULL.
 * @param row Place for row number to use.
 * @param col Place for column number to use.
 */
int kairos_get_smac_pos(struct kairos_dev_priv *dp,
                        struct frs_smac_table_entry *entry,
                        struct frs_smac_table_entry *cur_entry,
                        uint16_t *row,
                        uint16_t *col)
{
    int ret = -EIO;
    struct frs_smac_table_entry old_entry = { .config = 0 };
    // SMAC entry position information.
    struct kairos_smac_pos {
        uint16_t row;
        uint16_t col;
    };
    // SMAC entry position to use.
    struct kairos_smac_pos pos = { .row = 0 };
    // Alternate positions.
    enum { no_vlan, vlan, free, count };
    struct kairos_smac_pos alt_pos[count] = {
        // No VLAN is for columns whose row selection does not depend on VLAN.
        [no_vlan] = { .col = FRS_SMAC_TABLE_COLS },
        // VLAN is for columns whose row selection depends on VLAN,
        // VLAN match should be enabled in all columns.
        [vlan] = { .col = FRS_SMAC_TABLE_COLS },
        // Free means that column is not yet in use at all.
        [free] = { .col = FRS_SMAC_TABLE_COLS },
    };

    if (cur_entry)
        *cur_entry = old_entry;

    // Reuse existing column, if possible, or use first free column.
    for (pos.col = 0; pos.col < FRS_SMAC_TABLE_COLS; pos.col++) {
        entry->column = pos.col;
        pos.row = kairos_get_smac_row(dp, entry);
        ret = kairos_read_smac_entry(dp, pos.row, pos.col, &old_entry);
        if (ret < 0)
            return ret;

        /*
         * If entry is not in use, it can possibly be used.
         * Record such positions for later, but still look for perfect match.
         */
        if (!(old_entry.config & FRS_SMAC_CONFIG_ENABLED)) {
            // Usability depends on SMAC hash function for column.
            enum frs_smac_row_sel row_sel = dp->smac.cfg.row_sel[pos.col];

            if (entry->config & FRS_SMAC_CONFIG_VLAN) {
                // Columns whose row selection depends on VLAN can be used.
                if (row_sel == FLX_FRS_SMAC_ROW_SEL_VLAN) {
                    if (alt_pos[vlan].col >= FRS_SMAC_TABLE_COLS)
                        alt_pos[vlan] = pos;
                }
            }

            /*
             * Columns whose row selection does not depend on VLAN
             * can always be used.
             */
            if (row_sel == FLX_FRS_SMAC_ROW_SEL_NO_VLAN) {
                if (alt_pos[no_vlan].col >= FRS_SMAC_TABLE_COLS)
                    alt_pos[no_vlan] = pos;
            }

            /*
             * Entirely unused (free) column can also be taken into use,
             * but may require changing column row selection method.
             */
            if (dp->smac.col_count[pos.col].no_vlan +
                dp->smac.col_count[pos.col].vlan == 0) {
                if (alt_pos[free].col >= FRS_SMAC_TABLE_COLS)
                    alt_pos[free] = pos;
            }

            continue;
        }

        // Reuse entry if it is already used for this MAC/VLAN tuple.
        if (!kairos_match_smac_entries(entry, &old_entry))
            continue;

        // It really is for this MAC/VLAN tuple, use this column.
        if (cur_entry)
            *cur_entry = old_entry;

        break;
    }

    // Order of preference: existing, VLAN match, no VLAN match and free.
    if (pos.col >= FRS_SMAC_TABLE_COLS)
        pos = alt_pos[vlan];
    if (pos.col >= FRS_SMAC_TABLE_COLS)
        pos = alt_pos[no_vlan];
    // Disable automatic column row selection function setting for now.
#if 0
    if (pos.col >= FRS_SMAC_TABLE_COLS)
        pos = alt_pos[free];
#endif
    if (pos.col >= FRS_SMAC_TABLE_COLS) {
        // No suitable column found.
        return -ENOMEM;
    }

    *row = pos.row;
    *col = pos.col;
    entry->column = pos.col;

    return 0;
}

/**
 * Write port IPO config.
 * @param pp Port private
 * @param entry entry ID
 * @param flags IPO flags
 * @param allow_mask Allow mask
 * @param mirror_mask Mirror mask
 * @param priority Frame priority
 * @param addr MAC address
 * @param compare_length Compare length for address.
 */
int kairos_write_port_ipo(struct kairos_port_priv *pp,
                          uint16_t entry,
                          uint16_t flags0,
                          uint16_t flags1,
                          uint16_t allow_mask,
                          uint16_t mirror_mask,
                          uint8_t priority,
                          const uint8_t addr[IFHWADDRLEN],
                          uint8_t compare_length)
{
    uint16_t data = 0;
    uint16_t ipo_data = 0;

    netdev_dbg(pp->netdev,
               "%s() IPO(%i) f:0x%04x 0x%04x a:0x%04x m:0x%04x p:%i %pM/%i\n",
               __func__, entry, flags0, flags1, allow_mask, mirror_mask,
               priority, addr, compare_length);

    if (flags0 != 0) {
        ipo_data = flags0 |
            PORT_ETH_ADDR_FROM_PRIO(priority) |
            PORT_ETH_ADDR_CMP_LENGTH(compare_length);

        data = addr[0] | (addr[1] << 8);
        kairos_write_port_reg(pp, PORT_REG_ETH_ADDR_0(entry), data);

        data = addr[2] | (addr[3] << 8);
        kairos_write_port_reg(pp, PORT_REG_ETH_ADDR_1(entry), data);

        data = addr[4] | (addr[5] << 8);
        kairos_write_port_reg(pp, PORT_REG_ETH_ADDR_2(entry), data);

        kairos_write_port_reg(pp, PORT_REG_ETH_ADDR_FWD_ALLOW(entry),
                              allow_mask);
        kairos_write_port_reg(pp, PORT_REG_ETH_ADDR_FWD_MIRROR(entry),
                              mirror_mask);
        kairos_write_port_reg(pp, PORT_REG_ETH_ADDR_CFG1(entry), flags1);
    }

    kairos_write_port_reg(pp, PORT_REG_ETH_ADDR_CFG0(entry), ipo_data);

    return 0;
}

/**
 * Get port cut-through status.
 * @param pp Port privates.
 * @return True when cut-through is enabled on port.
 */
bool kairos_get_cutthrough(struct kairos_port_priv *pp)
{
    struct kairos_netdev_priv *np = netdev_priv(pp->netdev);
    bool enable;

    mutex_lock(&np->link_mode_lock);
    enable = kairos_read_port_reg(pp, PORT_REG_STATE) & PORT_STATE_RX_CT;
    mutex_unlock(&np->link_mode_lock);

    return enable;
}

/**
 * Enable/disable cut-through on port.
 * @param pp Port privates.
 * @param enable True to enable cut-through on port.
 */
void kairos_set_cutthrough(struct kairos_port_priv *pp, bool enable)
{
    struct kairos_dev_priv *dp = pp->dp;
    struct kairos_netdev_priv *np = netdev_priv(pp->netdev);
    uint16_t port_state;
    uint16_t tx_ct = 0;

    if (enable)
        tx_ct = (1u << dp->features.prio_queues) - 1u;

    mutex_lock(&np->link_mode_lock);

    port_state = kairos_read_port_reg(pp, PORT_REG_STATE);
    if (enable)
        port_state |= PORT_STATE_RX_CT;
    else
        port_state &= ~PORT_STATE_RX_CT;
    kairos_write_port_reg(pp, PORT_REG_STATE, port_state);
    kairos_write_port_reg(pp, PORT_REG_TX_CT, tx_ct);

    mutex_unlock(&np->link_mode_lock);

    return;
}

/**
 * Get pointer to specific delays.
 * @param pp Port privates.
 * @param delay_type Delay selector.
 * @return Pointer to delays.
 */
static struct kairos_delay *kairos_delay_type_to_delay(
        struct kairos_port_priv *pp,
        enum kairos_delay_type delay_type)
{
    struct kairos_delay *delay = NULL;

    switch (delay_type) {
    case DEIPCE_DELAY_PHY:
        delay = pp->ext_phy.delay;
        break;
    case DEIPCE_DELAY_ADAPTER:
        delay = pp->adapter.delay;
        break;
    }

    return delay;
}

/**
 * Set port delays.
 * @param pp Port privates.
 * @param delay_type Delay selection.
 * @param dir Direction on which to set the delays.
 * @param new_delays Array of new delay values in nanoseconds,
 * indexed by enum link_mode.
 * Delays for LM_10FULL are always used when link is down (LM_DOWN).
 */
void kairos_set_delays(struct kairos_port_priv *pp,
                       enum kairos_delay_type delay_type,
                       enum kairos_dir dir,
                       const unsigned int new_delays[DEIPCE_LINK_MODE_COUNT])
{
    struct kairos_delay *delay = kairos_delay_type_to_delay(pp, delay_type);
    struct kairos_netdev_priv *np = netdev_priv(pp->netdev);

    mutex_lock(&np->link_mode_lock);

    if (delay_type == DEIPCE_DELAY_ADAPTER)
        pp->flags |= DEIPCE_ADAPTER_DELAY_OVERRIDE;

    switch (dir) {
    case DEIPCE_DIR_TX:
        delay[LM_10FULL].tx = new_delays[LM_10FULL];
        delay[LM_100FULL].tx = new_delays[LM_100FULL];
        delay[LM_1000FULL].tx = new_delays[LM_1000FULL];
        delay[LM_DOWN].tx = delay[LM_10FULL].tx;
        break;
    case DEIPCE_DIR_RX:
        delay[LM_10FULL].rx = new_delays[LM_10FULL];
        delay[LM_100FULL].rx = new_delays[LM_100FULL];
        delay[LM_1000FULL].rx = new_delays[LM_1000FULL];
        delay[LM_DOWN].rx = delay[LM_10FULL].tx;
        break;
    }

    kairos_update_delays(pp);

    mutex_unlock(&np->link_mode_lock);

    return;
}

/**
 * Get port delays.
 * @param pp Port privates.
 * @param delay_type Delay selection.
 * @param dir Direction for which to get the delays.
 * @param cur_delays Array of current delay values in nanoseconds,
 * indexed by enum link_mode.
 */
void kairos_get_delays(struct kairos_port_priv *pp,
                       enum kairos_delay_type delay_type,
                       enum kairos_dir dir,
                       unsigned int cur_delays[DEIPCE_LINK_MODE_COUNT])
{
    const struct kairos_delay *delay =
        kairos_delay_type_to_delay(pp, delay_type);
    struct kairos_netdev_priv *np = netdev_priv(pp->netdev);

    mutex_lock(&np->link_mode_lock);

    switch (dir) {
    case DEIPCE_DIR_TX:
        cur_delays[LM_DOWN] = delay[LM_DOWN].tx;
        cur_delays[LM_10FULL] = delay[LM_10FULL].tx;
        cur_delays[LM_100FULL] = delay[LM_100FULL].tx;
        cur_delays[LM_1000FULL] = delay[LM_1000FULL].tx;
        break;
    case DEIPCE_DIR_RX:
        cur_delays[LM_DOWN] = delay[LM_DOWN].rx;
        cur_delays[LM_10FULL] = delay[LM_10FULL].rx;
        cur_delays[LM_100FULL] = delay[LM_100FULL].rx;
        cur_delays[LM_1000FULL] = delay[LM_1000FULL].rx;
        break;
    }

    mutex_unlock(&np->link_mode_lock);

    return;
}

/**
 * Take delay values for current link mode into use.
 * @param pp Port privates.
 */
void kairos_update_delays(struct kairos_port_priv *pp)
{
    struct kairos_netdev_priv *np = netdev_priv(pp->netdev);
    struct kairos_delay adapter_delay;

    kairos_get_adapter_delays(pp, &adapter_delay);
    pp->cur_delay.tx = adapter_delay.tx + pp->ext_phy.delay[np->link_mode].tx;
    pp->cur_delay.rx = adapter_delay.rx + pp->ext_phy.delay[np->link_mode].rx;

    netdev_dbg(pp->netdev, "%s() TX/RX delays adapter %lu/%lu PHY %lu/%lu"
               " total %lu/%lu ns\n",
               __func__, adapter_delay.tx, adapter_delay.rx,
               pp->ext_phy.delay[np->link_mode].tx,
               pp->ext_phy.delay[np->link_mode].rx,
               pp->cur_delay.tx, pp->cur_delay.rx);

    return;
}

