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
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/hardirq.h>
#include <linux/if_vlan.h>
#include <linux/of.h>
#include <linux/of_net.h>

#include "kairos_types.h"
#include "kairos_if.h"
#include "kairos_netdev.h"
#include "kairos_netdevif.h"
#include "kairos_hw.h"
#include "kairos_switchdev.h"
#include "kairos_time.h"
#include "kairos_clock_main.h"
#include "kairos_main.h"

// Module description information
MODULE_DESCRIPTION("Kairos driver");
MODULE_AUTHOR("EXOR Embedded");
MODULE_LICENSE("Proprietary");
MODULE_VERSION(DRV_VERSION);

// Module parameters

/// bitmask of devices, 1 means use port timestamper (for 802.1AS)
static unsigned int port_ts = ~0u;
module_param(port_ts, uint, S_IRUGO);
MODULE_PARM_DESC(port_ts,
                 "Select between 802.1AS (1) and 1588 (0)"
                 " (bitmask of switch devices, 802.1AS is the default)");

// Driver private data
static struct kairos_drv_priv kairos_drv_priv = {
    .wq_low = NULL,
};

/**
 * Get access to driver privates.
 */
struct kairos_drv_priv *kairos_get_drv_priv(void)
{
    return &kairos_drv_priv;
}

/**
 * Set switch features from switch generics registers.
 * @param dp Switch privates.
 */
static int kairos_read_switch_parameters(struct kairos_dev_priv *dp)
{
    unsigned int column;
    uint16_t value;

    dev_dbg(dp->this_dev, "%s() read generics\n", __func__);

    dp->num_of_ports = 2;
        kairos_read_generic(dp, FRS_REG_GENERIC_PORT_HIGH,
                            FRS_REG_GENERIC_PORT_HIGH_MASK) + 1;

    dp->features.mgmt_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_MGMT_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);
    dp->features.hsr_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_HSR_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);
    dp->features.prp_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_PRP_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);
    dp->features.ts_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_TS_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);
    dp->features.sched_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_SCHED_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);
    dp->features.ct_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_CT_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);
    dp->features.preempt_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_PREEMPT_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);
    dp->features.macsec_ports =
        kairos_read_generic(dp, FRS_REG_GENERIC_MACSEC_PORTS,
                            FRS_REG_GENERIC_PORT_MASK);

    dp->features.fids =
        kairos_read_generic(dp, FRS_REG_GENERIC_FIDS,
                            FRS_REG_GENERIC_FIDS_MASK);
    if (dp->features.fids)
        dp->features.fids = 1u << dp->features.fids;

    dp->features.clock_freq =
        kairos_read_generic(dp, FRS_REG_GENERIC_CLK_FREQ,
                            FRS_REG_GENERIC_CLK_FREQ_MASK) * 1000000;
    dp->features.prio_queues =
        kairos_read_generic(dp, FRS_REG_GENERIC_PRIO_QUEUES,
                            FRS_REG_GENERIC_PRIO_QUEUES_MASK);

    // Number of policers is only valid when policing is enabled.
    value = kairos_read_generic(dp, FRS_REG_GENERIC_POLICING,
                                FRS_REG_GENERIC_POLICING_MASK);
    if (value) {
        dp->features.policers = 1u <<
            kairos_read_generic(dp, FRS_REG_GENERIC_POLICERS,
                                FRS_REG_GENERIC_POLICERS_MASK);
        if (dp->features.policers < DEIPCE_MIN_POLICERS ||
            dp->features.policers > DEIPCE_MAX_POLICERS) {
            dev_err(dp->this_dev, "Number of policers %u is invalid\n",
                    dp->features.policers);
            return -EINVAL;
        }
    }
    else {
        dp->features.policers = 0;
    }

    // SMAC is required.
    value = kairos_read_generic(dp, FRS_REG_GENERIC_SMAC_ROWS,
                                FRS_REG_GENERIC_SMAC_ROWS_MASK);
    if (!value) {
        dev_err(dp->this_dev, "SMAC is required\n");
        return -EINVAL;
    }
    dp->features.smac_rows = 1u << value;
    if (dp->features.smac_rows < KAIROS_MIN_SMAC_ROWS ||
        dp->features.smac_rows > KAIROS_MAX_SMAC_ROWS) {
        dev_err(dp->this_dev, "Number of SMAC rows %u is invalid\n",
                dp->features.smac_rows);
        return -EINVAL;
    }

    dp->features.flags = 0;
    if (kairos_read_generic(dp, FRS_REG_GENERIC_GIGABIT,
                            FRS_REG_GENERIC_BOOL_MASK))
        dp->features.flags |= FLX_FRS_FEAT_GIGABIT;
    if (kairos_read_generic(dp, FRS_REG_GENERIC_COUNTERS,
                            FRS_REG_GENERIC_BOOL_MASK))
        dp->features.flags |= FLX_FRS_FEAT_STATS;
    if (kairos_read_generic(dp, FRS_REG_GENERIC_SHAPERS,
                            FRS_REG_GENERIC_BOOL_MASK))
        dp->features.flags |= FLX_FRS_FEAT_SHAPER;

    for (column = 0; column < FRS_SMAC_TABLE_COLS; column++)
        dp->smac.cfg.row_sel[column] = FLX_FRS_SMAC_ROW_SEL_NO_VLAN;

    return 0;
}

/**
 * Do a software reset to switch.
 * @param dp Switch privates.
 */
static int kairos_sw_reset(struct deipce_dev_priv *dp)
{
    unsigned int timeout = 100;
    uint16_t data;

    // SW Reset
    deipce_write_switch_reg(dp, FRS_REG_GEN, FRS_GEN_RESET);

    /*
     * Wait reset to complete. It may take up to a few hundred microseconds,
     * depending on features. Have enough room for changes.
     */
    do {
        if (timeout-- == 0) {
            dev_err(dp->this_dev, "SW reset failed: timeout\n");
            return -EBUSY;
        }
        usleep_range(100, 200);

        data = deipce_read_switch_reg(dp, FRS_REG_GEN);
    } while (data & FRS_GEN_RESET);

    dev_printk(KERN_DEBUG, dp->this_dev, "SW reset done\n");

    return 0;
}

/**
 * Initialize switch registers.
 * @param dp Switch privates.
 */
static void kairos_init_switch_registers(struct deipce_dev_priv *dp)
{
    uint16_t data = 0;

    // Write general switch config, including management trailer settings.
    if (dp->trailer_len > 1) {
        data |= FRS_GEN_MGMT_TRAILER_LEN;
        if (dp->trailer_offset > 0)
            data |= FRS_GEN_MGMT_TRAILER_OFFSET;
    }
    deipce_write_switch_reg(dp, FRS_REG_GEN, data);

    // Init timestampers.
    if (!dp->use_port_ts) {
        dp->tx_stamper = 0;
        dp->rx_stamper.next = 0;
        dp->rx_stamper.next_skb = 0;

        deipce_write_switch_reg(dp, FRS_REG_TS_CTRL_RX,
                                (1u << FRS_TS_NUMBER_TIMESTAMPS) - 1);
        deipce_write_switch_reg(dp, FRS_REG_TS_CTRL_TX,
                                (1u << FRS_TS_NUMBER_TIMESTAMPS) - 1);
    }

    // Make sure to use driver defaults for VLAN configuration.
    deipce_drop_all_vlans(dp);

    return;
}

/**
 * Initialize port registers.
 * @param dp Switch privates.
 * @param pp Port privates.
 */
static void deipce_init_port_registers(struct deipce_dev_priv *dp,
                                       struct deipce_port_priv *pp)
{
    uint16_t data;

    /// Enable management trailer on CPU port.
    if (pp->flags & DEIPCE_PORT_CPU) {
        data = deipce_read_port_reg(pp, PORT_REG_STATE);
        data |= PORT_STATE_MANAGEMENT;
        deipce_write_port_reg(pp, PORT_REG_STATE, data);
    }

    // Init timestampers.
    if (dp->use_port_ts && (dp->features.ts_ports & (1u << pp->port_num))) {
        pp->rx_stamper = 0;
        pp->tx_stamper.next = 0;
        pp->tx_stamper.next_skb = 0;

        deipce_write_port_reg(pp, PORT_REG_TS_CTRL_RX,
                              (1u << PORT_TS_NUMBER_TIMESTAMPS) - 1);
        deipce_write_port_reg(pp, PORT_REG_TS_CTRL_TX,
                              (1u << PORT_TS_NUMBER_TIMESTAMPS) - 1);
    }

    deipce_reset_port_vlan_config(pp, true);

    /*
     * Use 1:1 mapping from VLAN PCP to output priority queue by default
     * when there are enough queues.
     */
    if (dp->features.prio_queues == DEIPCE_MAX_PRIO_QUEUES) {
        deipce_write_port_reg(pp, PORT_REG_VLAN_PRIO,
                              PORT_VLAN_PRIO(0, 0) |
                              PORT_VLAN_PRIO(1, 1) |
                              PORT_VLAN_PRIO(2, 2) |
                              PORT_VLAN_PRIO(3, 3) |
                              PORT_VLAN_PRIO(4, 4) |
                              PORT_VLAN_PRIO(5, 5) |
                              PORT_VLAN_PRIO(6, 6) |
                              PORT_VLAN_PRIO(7, 7));
        deipce_write_port_reg(pp, PORT_REG_VLAN_PRIO_HI,
                              PORT_VLAN_PRIO_HI(0, 0) |
                              PORT_VLAN_PRIO_HI(1, 1) |
                              PORT_VLAN_PRIO_HI(2, 2) |
                              PORT_VLAN_PRIO_HI(3, 3) |
                              PORT_VLAN_PRIO_HI(4, 4) |
                              PORT_VLAN_PRIO_HI(5, 5) |
                              PORT_VLAN_PRIO_HI(6, 6) |
                              PORT_VLAN_PRIO_HI(7, 7));
    }

    return;
}

/**
 * Read additional switch configuration from device tree.
 * @param dp Switch privates.
 * @param config Place for intermediate configuration time only information
 * from device tree.
 */
static int kairos_config_switch_dt(struct kairos_dev_priv *dp,
                                   struct kairos_switch_config *config)
{
    struct device_node *switch_node = dp->this_dev->of_node;
    struct device_node *phc_node;

    // Underlying Ethernet MAC name
    if (of_property_read_string(switch_node, "mac_name", &config->mac_name)) {
        dev_printk(KERN_DEBUG, dp->this_dev, "unable to get MAC name\n");
        return -ENODEV;
    }

    // Default PTP hardware clock for all ports and endpoint
    phc_node = of_parse_phandle(switch_node, "ptp-clock", 0);
    if (phc_node) {
        dp->time =
            kairos_time_get_by_clock(deipce_clock_of_get_clock(phc_node));
        of_node_put(phc_node);
    }

    // Time interface is mandatory.
    if (!dp->time) {
        dev_err(dp->this_dev, "No time interface\n");
        return -ENODEV;
    }

    // Interface name for switch
    if (of_property_read_string(switch_node, "if_name", &config->ep_name)) {
        dev_dbg(dp->this_dev, "%s() Using default endpoint name\n", __func__);
    }

    return 0;
}

/**
 * Read additional switch port configuration from device tree.
 * @param dp Switch privates.
 * @param pp Port privates.
 * @param port_node Port device tree node.
 * @param config Temporary storage to write port configuration to.
 */
static int kairos_config_port_dt(struct kairos_dev_priv *dp,
                                 struct kairos_port_priv *pp,
                                 struct kairos_node *port_node,
                                 struct kairos_port_config *config)
{
    phy_interface_t phy_mode;
    struct of_phandle_args args = { .np = NULL };
    struct resource res_adapter = {
        .start = 0,
        .end = 0,
    };
    const __be32 *reg = NULL;
    int length = 0;
    int ret;

    dev_dbg(dp->this_dev, "%s() Configure port %u from device tree\n",
            __func__, pp->port_num);

    if (of_property_read_string(port_node, "if_name", &config->name) == 0) {
        dev_dbg(dp->this_dev, "%s() Port %u name %s\n",
                __func__, pp->port_num, config->name);
    }

    if (of_property_read_bool(port_node, "auto-speed-select")) {
        pp->flags |= DEIPCE_PORT_SPEED_EXT;
    }

    // Ext PHY
    pp->ext_phy.node = of_parse_phandle(port_node, "phy-handle", 0);
    pp->ext_phy.interface = PHY_INTERFACE_MODE_NA;
    if (pp->ext_phy.node) {
        pp->medium_type = KAIROS_MEDIUM_PHY;
        pp->flags |= KAIROS_HAS_PHY;

        phy_mode = of_get_phy_mode(port_node);
        if (phy_mode >= 0)
            pp->ext_phy.interface = phy_mode;
    }

    // SFP EEPROM for SFP type detection
    pp->sfp.eeprom_node = of_parse_phandle(port_node, "sfp-eeprom", 0);
    if (pp->sfp.eeprom_node) {
        pp->medium_type = KAIROS_MEDIUM_SFP;
        pp->flags |= KAIROS_SFP_EEPROM;
    }

    // SFP PHY, always use SGMII with SFP PHY.
    pp->sfp.phy.interface = PHY_INTERFACE_MODE_SGMII;
    pp->sfp.phy.node = of_parse_phandle(port_node, "sfp-phy-handle", 0);
    if (pp->sfp.phy.node) {
        pp->medium_type = KAIROS_MEDIUM_SFP;
        pp->flags |= KAIROS_HAS_SFP_PHY;
    }

    // SGMII mode for SGMII adapter
    if (of_property_read_bool(port_node, "sgmii-phy-mode")) {
        pp->flags |= KAIROS_ADAPTER_SGMII_PHY_MODE;
        dev_printk(KERN_DEBUG, dp->this_dev, "port %u SGMII %s mode\n",
                   pp->port_num, "PHY");
    }

    if ((pp->flags & KAIROS_HAS_PHY) &&
        (pp->flags & KAIROS_HAS_SFP_PHY))
        pp->flags |= KAIROS_HAS_SEPARATE_SFP;

    if (dp->features.sched_ports & (1u << pp->port_num)) {
        ret = of_parse_phandle_with_fixed_args(port_node, "scheduler", 1,
                                               0, &args);
        if (ret >= 0) {
            struct device_node *node = args.np;

            pp->sched.num = args.args[0];
            pp->sched.fsc = kairos_fsc_of_get_device_by_node(node);
            of_node_put(node);
        }
        else {
            dev_info(dp->this_dev,
                     "Disabling scheduled traffic support from port %u\n",
                     pp->port_num);
            dp->features.sched_ports &= ~(1u << pp->port_num);
        }
    }

    // Adapters are optional.
    reg = of_get_property(port_node, "adapter-reg", &length);
    if (reg && length == 2*sizeof(uint32_t)) {
        res_adapter.start = be32_to_cpu(reg[0]);
        res_adapter.end = res_adapter.start + be32_to_cpu(reg[1]) - 1u;
        pp->adapter.ioaddr = ioremap_nocache(res_adapter.start,
                                             resource_size(&res_adapter));
        if (!pp->adapter.ioaddr) {
            dev_err(dp->this_dev,
                    "ioremap failed for port %u adapter at 0x%llx/0x%llx\n",
                    pp->port_num,
                    (unsigned long long int)res_adapter.start,
                    (unsigned long long int)resource_size(&res_adapter));
            goto err_ioremap_adapter;
        }

        dev_printk(KERN_DEBUG, dp->this_dev,
                   "Port %u adapter registers at 0x%llx/0x%llx\n",
                   pp->port_num,
                   (unsigned long long int)res_adapter.start,
                   (unsigned long long int)resource_size(&res_adapter));
    }

    return 0;

err_ioremap_adapter:
    return -ENOMEM;
}

/**
 * Create switch port.
 * @param dp Switch privates.
 * @param port_num Port number.
 */
static int kairos_create_port(struct kairos_dev_priv *dp,
                              unsigned int port_num)
{
    struct kairos_port_priv *pp;
    int ret = -ENOMEM;

    pp = kmalloc(sizeof(*pp), GFP_KERNEL);
    if (!pp) {
        dev_err(dp->this_dev, "kmalloc failed\n");
        goto err_alloc;
    }

    *pp = (struct deipce_port_priv){
        .dp = dp,
        .port_num = port_num,
        .medium_type = DEIPCE_MEDIUM_NOPHY,
        .sfp.supported = DEIPCE_ETHTOOL_SUPPORTED,
        .mirror_port = -1,

        // Management trailer for sending.
        .trailer = 1u << (dp->trailer_offset + port_num),
    };

    mutex_init(&pp->stats_lock);
    mutex_init(&pp->port_reg_lock);
    spin_lock_init(&pp->tx_stamper.lock);
    dp->port[port_num] = pp;

    if (!dp->cpu_port_mask && (dp->features.mgmt_ports & (1u << port_num))) {
        dp->cpu_port_mask |= pp->trailer;
        pp->flags |= DEIPCE_PORT_CPU;
    }

    if (dp->features.macsec_ports & (1u << pp->port_num))
        pp->trailer |= deipce_get_macsec_trailer(dp);

    return 0;

err_ioremap:
    mutex_destroy(&pp->stats_lock);
    mutex_destroy(&pp->port_reg_lock);
    dp->port[pp->port_num] = NULL;
    pp->dp = NULL;
    kfree(pp);

err_alloc:
    return ret;
}

/**
 * Initialize switch port for use.
 * @param dp Switch privates.
 * @param pp Port privates.
 * @param port_node Port device tree node, or NULL.
 */
static int kairos_init_port(struct kairos_dev_priv *dp,
                            struct kairos_port_priv *pp,
                            struct kairos_node *port_node)
{
    struct kairos_port_config config = { .name = NULL };
    int ret = -ENOMEM;

    if (port_node) {
        ret = kairos_config_port_dt(dp, pp, port_node, &config);
        if (ret)
            goto err_config;
    }

    kairos_init_port_registers(dp, pp);

    ret = kairos_mstp_init_port(dp, pp);
    if (ret)
        goto err_mstp;

    ret = kairos_netdev_init_port(dp, pp, &config);
    if (ret)
        goto err_netdev;

    ret = kairos_switchdev_init_port(dp, pp);
    if (ret)
        goto err_switchdev;

    return 0;

err_switchdev:
    kairos_netdev_cleanup_port(dp, pp);

err_netdev:
    kairos_mstp_cleanup_port(dp, pp);

err_mstp:
err_config:
    return ret;
}

/**
 * Uninitialize switch port.
 * @param dp Switch privates.
 * @param pp Port privates.
 */
static void kairos_cleanup_port(struct kairos_dev_priv *dp,
                                struct kairos_port_priv *pp)
{
    kairos_netdev_cleanup_port(dp, pp);
    kairos_mstp_cleanup_port(dp, pp);

    return;
}

/**
 * Destroy uninitialized switch port.
 * @param dp Switch privates.
 * @param pp Port privates.
 */
static void kairos_destroy_port(struct kairos_dev_priv *dp,
                                struct kairos_port_priv *pp)
{
    if (pp->adapter.ioaddr) {
        iounmap(pp->adapter.ioaddr);
        pp->adapter.ioaddr = NULL;
    }

    iounmap(pp->ioaddr);
    pp->ioaddr = NULL;

    mutex_destroy(&pp->stats_lock);
    mutex_destroy(&pp->port_reg_lock);

    dp->port[pp->port_num] = NULL;
    pp->dp = NULL;
    kfree(pp);

    return;
}

/**
 * Create switch.
 * @param pdev Switch platform device.
 */
static int kairos_create_switch(struct platform_device *pdev)
{
    int ret = -ENXIO;
    struct kairos_drv_priv *drv = deipce_get_drv_priv();
    struct kairos_dev_priv *dp;
    struct resource *res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    struct resource *res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    unsigned int smac_usage_bits;
    size_t smac_usage_size;
    unsigned int port_num;
    struct deipce_port_priv *pp;
    unsigned int dev_num = 0;

    dev_dbg(&pdev->dev, "%s() Create switch\n", __func__);

    if (!res_mem) {
        dev_err(&pdev->dev, "No I/O memory defined\n");
        goto err_iomem;
    }

    // use pdev->id if provided, if only one, pdev->id == -1
    if (pdev->id >= 0) {
        dev_num = pdev->id;
    } else {
        // With device tree pdev->id may always be -1.
        while (dev_num < ARRAY_SIZE(drv->dev_priv)) {
            if (!drv->dev_priv[dev_num])
                break;
            dev_num++;
        }
    }
    if (dev_num >= ARRAY_SIZE(drv->dev_priv)) {
        dev_err(&pdev->dev, "Too many FRS devices\n");
        ret = -ENODEV;
        goto err_too_many;
    }

    dp = kmalloc(sizeof(*dp), GFP_KERNEL);
    if (!dp) {
        dev_err(&pdev->dev, "kmalloc failed\n");
        goto err_alloc;
    }

    *dp = (struct deipce_dev_priv) {
        .this_dev = &pdev->dev,
        .dev_num = dev_num,
        .mgmt_tc = 1,
        .tstamp_cfg = {
            .tx_type = HWTSTAMP_TX_OFF,
            .rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT,
        },
        .base_addr = DEIPCE_SWITCH_MGMT_ADDR(res_mem->start),
        .irq = res_irq ? res_irq->start : 0,
    };

    spin_lock_init(&dp->rx_stamper.lock);
    mutex_init(&dp->common_reg_lock);
    mutex_init(&dp->smac_table_lock);

    drv->dev_priv[dev_num] = dp;

    dp->ioaddr = ioremap_nocache(res_mem->start, resource_size(res_mem));
    if (!dp->ioaddr) {
        dev_err(dp->this_dev,
                "ioremap failed for switch address 0x%llx/0x%llx\n",
                (unsigned long long int)res_mem->start,
                (unsigned long long int)resource_size(res_mem));
        goto err_ioremap;
    }

    dev_printk(KERN_DEBUG, dp->this_dev,
               "Switch registers at 0x%llx/0x%llx\n",
               (unsigned long long int)res_mem->start,
               (unsigned long long int)resource_size(res_mem));

    ret = kairos_sw_reset(dp);
    if (ret)
        goto err_sw_reset;

    kairos_read_switch_parameters(dp);

    ret = kairos_irq_init(dp);
    if (ret)
        goto err_irq;

    /*
     * Determine management trailer length.
     * MACsec needs 1 bit, preemption needs 2 bits (just before MACsec bit,
     * regardless of whether or not MACsec is enabled).
     */
    dp->trailer_len = dp->num_of_ports;
    if (dp->features.preempt_ports)
        dp->trailer_len +=
            DEIPCE_TRAILER_MACSEC_BITS + DEIPCE_TRAILER_PREEMPT_BITS;
    else if (dp->features.macsec_ports)
        dp->trailer_len += DEIPCE_TRAILER_MACSEC_BITS;
    dp->trailer_len = (dp->trailer_len + 7) / 8;
    if (dp->trailer_len > sizeof(uint16_t)) {
        dp->trailer_len = sizeof(uint16_t);
        dev_warn(dp->this_dev, "Too many ports\n");
    }

    smac_usage_bits = dp->features.smac_rows * FRS_SMAC_TABLE_COLS;
    smac_usage_size = BITS_TO_LONGS(smac_usage_bits) * sizeof(*dp->smac.used);
    dp->smac.used = kzalloc(smac_usage_size, GFP_KERNEL);
    if (!dp->smac.used) {
        dev_err(dp->this_dev, "Failed to allocate SMAC usage bitmap\n");
        ret = -ENOMEM;
        goto err_smac;
    }

    // Allow use of port timestampers only when they are available.
    if (dp->features.ts_ports == 0)
        port_ts &= ~(1u << dp->dev_num);
    if (port_ts & (1u << dp->dev_num))
        dp->use_port_ts = true;
    else
        dp->use_port_ts = false;

    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        ret = kairos_create_port(dp, port_num);
        if (ret)
            goto err_port;
    }

    return 0;

err_port:
    while (port_num-- > 0) {
        pp = dp->port[port_num];
        if (pp)
            kairos_destroy_port(dp, pp);
    }
    if (dp->smac.used) {
        kfree(dp->smac.used);
        dp->smac.used = NULL;
    }

err_smac:
    kairos_irq_cleanup(dp);

err_irq:
err_sw_reset:
    iounmap(dp->ioaddr);
    dp->ioaddr = NULL;

err_ioremap:
    drv->dev_priv[dev_num] = NULL;
    kfree(dp);

err_alloc:
err_too_many:
err_iomem:
    return ret;
}

/**
 * Platform driver probe function for DE-IP Core Edge.
 * @param pdev Switch platform device.
 */
static int kairos_probe(struct platform_device *pdev)
{
    int ret;

    ret = kairos_create_switch(pdev);

    return ret;
}

/**
 * Initialize switch for use.
 * @param dp Switch privates.
 */
static int kairos_init_switch(struct kairos_dev_priv *dp)
{
    int ret = -ENOMEM;
    struct kairos_switch_config config = { .mac_name = NULL };
    unsigned int port_num = 0;
    // "port%u"
    char port_node_name[8];
    struct device_node *port_node;
    struct kairos_port_priv *pp;
    int wrk_phc = -1;

    dev_dbg(dp->this_dev, "%s() Init switch\n", __func__);

    ret = kairos_config_switch_dt(dp, &config);
    if (ret)
        goto err_config_switch_dt;

    // Allow use of port timestampers only when they are available.
    if (dp->features.ts_ports == 0)
        port_ts &= ~(1u << dp->dev_num);
    if (port_ts & (1u << dp->dev_num)) {
        dp->use_port_ts = true;
        // Use also 2nd PHC for worker clock by default if possible.
        wrk_phc = deipce_time_get_avail_phc(dp->time, 1);
    }
    else {
        dp->use_port_ts = false;
    }
    if (wrk_phc < 0)
        wrk_phc = deipce_time_get_avail_phc(dp->time, 0);
    deipce_time_set_phc(dp->time, DEIPCE_TIME_SEL_WRK, wrk_phc);

    deipce_init_switch_registers(dp);

    ret = deipce_init_vlan_fids(dp);
    if (ret)
        goto err_fid;

    ret = deipce_mstp_init_switch(dp);
    if (ret)
        goto err_mstp;

    ret = deipce_switchdev_init_switch(dp);
    if (ret)
        goto err_switchdev;

    ret = deipce_netdev_init_switch(dp, &config);
    if (ret)
        goto err_netdev;

    for (port_num = 0; port_num < dp->num_of_ports; port_num++) {
        pp = dp->port[port_num];
        if (!pp)
            continue;

        sprintf(port_node_name, "port%u", port_num);
        port_node = of_get_child_by_name(dp->this_dev->of_node,
                                         port_node_name);
        ret = deipce_init_port(dp, pp, port_node);

        if (port_node)
            of_node_put(port_node);

        if (ret)
            goto err_port;

        if (pp->sched.fsc && pp->sched.fsc != last_fsc) {
            deipce_fsc_set_time(pp->sched.fsc, dp->time);
            last_fsc = pp->sched.fsc;
        }
    }

    deipce_irq_enable(dp);

    return 0;

err_netdev:
err_port:
    while (port_num-- > 0) {
        pp = dp->port[port_num];
        if (!pp)
            continue;

        deipce_cleanup_port(dp, pp);
    }
    deipce_switchdev_cleanup_switch(dp);

err_switchdev:
    deipce_mstp_cleanup_switch(dp);

err_mstp:
    deipce_cleanup_vlan_fids(dp);

err_fid:
err_config_switch_dt:
    return ret;
}

/**
 * Uninitialize switch.
 * @param dp Switch privates.
 */
static void kairos_cleanup_switch(struct kairos_dev_priv *dp)
{
    struct kairos_port_priv *pp;
    unsigned int port_num;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    for (port_num = 0; port_num < ARRAY_SIZE(dp->port); port_num++) {
        pp = dp->port[port_num];
        if (pp)
            kairos_cleanup_port(dp, pp);
    }

    kairos_netdev_cleanup_switch(dp);
    kairos_switchdev_cleanup_switch(dp);
    kairos_mstp_cleanup_switch(dp);

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    return;
}

/**
 * Release all resources used by switch.
 * @param dp Switch privates.
 */
static void kairos_destroy_switch(struct kairos_dev_priv *dp)
{
    struct kairos_drv_priv *drv = kairos_get_drv_priv();
    struct kairos_port_priv *pp;
    unsigned int port_num;

    dev_dbg(dp->this_dev, "%s()\n", __func__);

    for (port_num = 0; port_num < ARRAY_SIZE(dp->port); port_num++) {
        pp = dp->port[port_num];
        if (pp)
            kairos_destroy_port(dp, pp);
    }

    if (dp->smac.used) {
        kfree(dp->smac.used);
        dp->smac.used = NULL;
    }

    kairos_cleanup_vlan_fids(dp);

    drv->dev_priv[dp->dev_num] = NULL;

    mutex_destroy(&dp->common_reg_lock);
    mutex_destroy(&dp->smac_table_lock);

    dev_dbg(dp->this_dev, "%s() done\n", __func__);

    dp->this_dev = NULL;
    kfree(dp);

    return;
}

/**
 * Platform driver remove function for DE-IP Core Edge.
 * @param pdev Switch platform device
 */
static int kairos_remove(struct platform_device *pdev)
{
    struct kairos_drv_priv *drv = kairos_get_drv_priv();
    struct kairos_dev_priv *dp = NULL;
    unsigned int i;

    dev_printk(KERN_DEBUG, &pdev->dev, "Remove device\n");

    for (i = 0; i < ARRAY_SIZE(drv->dev_priv); i++) {
        dp = drv->dev_priv[i];
        if (!dp)
            continue;

        if (dp->this_dev == &pdev->dev) {
            kairos_cleanup_switch(dp);
            kairos_destroy_switch(dp);
            return 0;
        }
    }

    pr_err(DRV_NAME ": Device %s not found\n", dev_name(&pdev->dev));

    return -ENODEV;
}

/**
 * Initialize all detected DE-IP Core Edge switches for use.
 */
static int kairos_init_switches(void)
{
    int ret = -ENODEV;
    struct kairos_drv_priv *drv = kairos_get_drv_priv();
    struct kairos_dev_priv *dp;
    unsigned int dev_num;

    pr_debug(DRV_NAME ": %s() Init switches\n", __func__);

    for (dev_num = 0; dev_num < ARRAY_SIZE(drv->dev_priv); dev_num++) {
        dp = drv->dev_priv[dev_num];
        if (!dp)
            continue;

        ret = kairos_init_switch(dp);
        if (ret) {
            dev_warn(dp->this_dev, "Failed to initialize switch\n");
            // Leave only fully initialized switches.
            kairos_destroy_switch(dp);
        }
    }

    return 0;
}

/**
 * Cleanup and destroy all DE-IP Core Edge switches.
 */
static void kairos_destroy_switches(void)
{
    struct kairos_drv_priv *drv = kairos_get_drv_priv();
    struct kairos_dev_priv *dp;
    unsigned int dev_num;

    for (dev_num = 0; dev_num < ARRAY_SIZE(drv->dev_priv); dev_num++) {
        dp = drv->dev_priv[dev_num];
        if (!dp)
            continue;

        kairos_cleanup_switch(dp);
        kairos_destroy_switch(dp);
    }

    return;
}

/*
 * Platform device driver match table.
 */
static const struct of_device_id kairos_match[] = {
    { .compatible = "exor,kairos", },
    { },
};

/**
 * Platform Driver definition for Linux core.
 */
static struct platform_driver kairos_dev_driver = {
    .driver = {
        .name = "kairos",
        .owner = THIS_MODULE,
        .of_match_table = kairos_match,
    },
    .probe = &kairos_probe,
    .remove = &kairos_remove,
};

/**
 * Module init.
 * Initialize everything in correct order.
 */
static int __init kairos_init(void)
{
    struct kairos_drv_priv *drv = kairos_get_drv_priv();
    int ret = 0;

    /*
     * Ordering dependencies:
     * - switch must be reset first
     * - clock depends on FPTS
     * - FSC depends on clock
     * - IBC depends on clocks
     */

    ret = kairos_init_crc40(drv);
    if (ret)
        goto err_init_crc40;

    drv->wq_low = alloc_workqueue(DRV_NAME, WQ_MEM_RECLAIM, 0);
    if (!drv->wq_low) {
        pr_err(DRV_NAME ": Failed to create work queue\n");
        ret = -ENOMEM;
        goto err_workqueue_lowpri;
    }

    drv->wq_high = alloc_workqueue(DRV_NAME, WQ_HIGHPRI | WQ_MEM_RECLAIM, 0);
    if (!drv->wq_high) {
        pr_err(DRV_NAME ": Failed to create work queue\n");
        ret = -ENOMEM;
        goto err_workqueue_highpri;
    }

    ret = platform_driver_register(&kairos_dev_driver);
    if (ret)
        goto err_reg_driver;

    ret = kairos_init_switches();
    if (ret)
        goto err_init_switches;

    ret = kairos_switchdev_init_driver();
    if (ret)
        goto err_init_switchdev;

    return 0;

err_init_switchdev:
    kairos_destroy_switches();

err_reg_driver:
    destroy_workqueue(drv->wq_high);
    drv->wq_high = NULL;

err_workqueue_highpri:
    destroy_workqueue(drv->wq_low);
    drv->wq_low = NULL;

err_workqueue_lowpri:
    kairos_cleanup_crc40(drv);

err_init_crc40:
    return ret;
}

/**
 * Module exit function.
 * Cleanup everything in correct order.
 */
static void __exit kairos_cleanup(void)
{
    struct kairos_drv_priv *drv = kairos_get_drv_priv();

    printk(KERN_DEBUG DRV_NAME ": Module cleanup\n");

    kairos_switchdev_cleanup_driver();

    platform_driver_unregister(&kairos_dev_driver);

    destroy_workqueue(drv->wq_high);
    drv->wq_high = NULL;
    destroy_workqueue(drv->wq_low);
    drv->wq_low = NULL;

    kairos_cleanup_crc40(drv);

    pr_debug(DRV_NAME ": %s() done\n", __func__);

    return;
}

// Module init and exit function
module_init(deipce_init);
module_exit(deipce_cleanup);

