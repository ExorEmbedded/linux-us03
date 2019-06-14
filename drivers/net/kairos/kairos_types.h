/** @file
 */

/*

   Kairos Linux driver

*/

#ifndef KAIROS_TYPES_H
#define KAIROS_TYPES_H

#include <linux/if.h>
#include <linux/netdevice.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/net_tstamp.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

#include "kairos_if.h"
#include "kairos_hw_type.h"
#include "kairos_switchdev.h"
#include "kairos_preempt.h"
#include "kairos_mstp.h"

#define KAIROS_MAX_DEVICES              1       ///< limit for switch devices
#define KAIROS_MAX_PORTS                2       ///< limit for number of ports

#define KAIROS_MAX_PRIO_QUEUES          8       ///< limit for priority queues
#define KAIROS_MIN_POLICERS             128     ///< limit for policers
#define KAIROS_MAX_POLICERS             4096    ///< limit for policers
#define KAIROS_MIN_SMAC_ROWS            128     ///< limit for SMAC rows
#define KAIROS_MAX_SMAC_ROWS            4096    ///< limit for SMAC rows
#define KAIROS_MAX_FIDS                 64      ///< limit for FIDs

// Port flags
#define KAIROS_PORT_CPU                 BIT(0)  ///< CPU port
#define KAIROS_PORT_SPEED_EXT           BIT(1)  ///< Auto/ext speed sel
#define KAIROS_ADAPTER_SGMII_PHY_MODE   BIT(2)  ///< SGMII in PHY mode
#define KAIROS_SFP_EEPROM               BIT(3)  ///< Use SFP EEPROM
#define KAIROS_HAS_PHY                  BIT(4)  ///< PHY configured
#define KAIROS_HAS_SFP_PHY              BIT(5)  ///< SFP PHY configured
#define KAIROS_HAS_SEPARATE_SFP         BIT(6)  ///< Separate SFP port
#define KAIROS_HAS_MASTER               BIT(7)  ///< Attached to master
#define KAIROS_ADAPTER_DELAY_OVERRIDE   BIT(8)  ///< Adapter delays overridden

/// Modes supported by FES
#define KAIROS_ETHTOOL_SUPPORTED \
    (SUPPORTED_10baseT_Full | \
     SUPPORTED_100baseT_Full | \
     SUPPORTED_1000baseT_Full | \
     SUPPORTED_Autoneg | \
     SUPPORTED_TP | \
     SUPPORTED_MII | \
     SUPPORTED_FIBRE | \
     SUPPORTED_Autoneg)

struct kairos_dev_priv;
struct kairos_port_priv;
struct kairos_ibc_dev_priv;
struct kairos_fsc_dev_priv;
struct kairos_time;

/**
 * Medium types.
 */
enum kairos_medium_type {
    KAIROS_MEDIUM_NONE = 0,     ///< Indicates unused port
    KAIROS_MEDIUM_SFP = 1,      ///< SFP module
    KAIROS_MEDIUM_PHY = 2,      ///< Normal PHY
    KAIROS_MEDIUM_NOPHY = 5,    ///< External port, no PHY
};

/**
 * Traffic delays for both directions.
 */
struct kairos_delay {
    unsigned long int tx;       ///< TX delay in ns
    unsigned long int rx;       ///< RX delay in ns
};

/**
 * Traffic delay components.
 */
enum kairos_delay_type {
    KAIROS_DELAY_PHY,           ///< PHY delay
    KAIROS_DELAY_ADAPTER,       ///< adapter delay
};

/**
 * Type for specifying direction.
 */
enum kairos_dir {
    KAIROS_DIR_TX,              ///< transmit
    KAIROS_DIR_RX,              ///< receive
};

/**
 * Recognized SFP types.
 */
enum kairos_sfp_type {
    KAIROS_SFP_NONE,           ///< None

    // Fiber
    KAIROS_SFP_1000BASEX,      ///< 1000Base-X or any other 1000 Mb/s fiber
    KAIROS_SFP_100BASEFX,      ///< 100Base-FX or any other 100 Mb/s fiber

    // Copper
    KAIROS_SFP_1000BASET,      ///< 1000Base-T
    KAIROS_SFP_100BASET,       ///< 100Base-T

    KAIROS_SFP_UNSUPPORTED,    ///< Unsupported or unrecognized SFP module
};

/**
 * FRS statistics directly available from FRS counters.
 * References to RMON refer to RMON statistics group etherStatsXxxx.
 */
enum {
    // Available from FRS registers
    FRS_CNT_RX_GOOD_OCTETS,
    FRS_CNT_RX_BAD_OCTETS,
    FRS_CNT_RX_UNICAST,         ///< RMON UnicastPkts
    FRS_CNT_RX_BROADCAST,       ///< RMON BroadcastPkts
    FRS_CNT_RX_MULTICAST,       ///< RMON MulticastPkts
    FRS_CNT_RX_UNDERSIZE,       ///< RMON UndersizePkts
    FRS_CNT_RX_FRAGMENT,        ///< RMON Fragments
    FRS_CNT_RX_OVERSIZE,        ///< RMON OverSizePkts
    FRS_CNT_RX_JABBER,          ///< RMON Jabbers
    FRS_CNT_RX_ERR,
    FRS_CNT_RX_CRC,             ///< RMON CRCAlignErrors
    FRS_CNT_RX_64,              ///< RMON Pkts64Octets
    FRS_CNT_RX_65_127,          ///< RMON Pkts65to127Octets
    FRS_CNT_RX_128_255,         ///< RMON Pkts128to255Octets
    FRS_CNT_RX_256_511,         ///< RMON Pkts256to511ctets
    FRS_CNT_RX_512_1023,        ///< RMON Pkts512to1023ctets
    FRS_CNT_RX_1024_1536,       ///< RMON Pkts1024to1536Octets
    FRS_CNT_RX_HSRPRP,
    FRS_CNT_RX_WRONGLAN,
    FRS_CNT_RX_DUPLICATE,
    FRS_CNT_RX_MEM_FULL_DROP,
    FRS_CNT_RX_POLICED,
    FRS_CNT_RX_MACSEC_UNTAGGED,
    FRS_CNT_RX_MACSEC_NOTSUPP,
    FRS_CNT_RX_MACSEC_UNKNOWN_SCI,
    FRS_CNT_RX_MACSEC_NOTVALID,
    FRS_CNT_RX_MACSEC_LATE,
    FRS_CNT_RX_PREEMPT_SMD_ERR,
    FRS_CNT_RX_PREEMPT_ASS_ERR,
    FRS_CNT_RX_PREEMPT_ASS_OK,
    FRS_CNT_RX_PREEMPT_FRAG,
    FRS_CNT_TX_OCTETS,
    FRS_CNT_TX_UNICAST,
    FRS_CNT_TX_BROADCAST,
    FRS_CNT_TX_MULTICAST,
    FRS_CNT_TX_HSRPRP,
    FRS_CNT_TX_PRIQ_DROP,
    FRS_CNT_TX_EARLY_DROP,
    FRS_CNT_TX_PREEMPT_FRAG,
    FRS_CNT_TX_OVERRUN,

    /// Number of statistics available from FRS counters
    FRS_CNT_REG_COUNT,
};

/**
 * Type for mapping statistics counters to name and register address.
 */
struct kairos_stat {
    char name[ETH_GSTRING_LEN];         ///< counter name
    uint32_t reg;                       ///< register address (number)
};

/**
 * Table of counter names and addresses.
 * Index is FRS_CNT_xxx
 */
extern const struct deipce_stat deipce_stat_info[FRS_CNT_REG_COUNT];

/**
 * FRS netdevice privates structure.
 */
struct kairos_netdev_priv {
    struct kairos_dev_priv *dp;         ///< switch privates
    struct kairos_port_priv *pp;        ///< switch port privates,
                                        ///< or NULL for endpoint
    uint32_t msg_enable;                ///< NETIF message level
    struct net_device_stats stats;      ///< Statistics

    // Rest are meaningful for switch ports only.
    enum link_mode link_mode;           ///< Current link mode
    enum link_mode force_link_mode;     ///< Forced link mode
    struct mutex link_mode_lock;        ///< Synchronize changes
                                        ///< Cannot use spinlock
    unsigned int stp_state;             ///< port STP state, BR_STATE_xxx
    struct kairos_switchdev_port switchdev;     ///< switchdev support
};

/**
 * PHY device context.
 */
struct kairos_phy {
    struct device_node *node;           ///< PHY device tree node
    phy_interface_t interface;          ///< PHY interface mode to use
    struct phy_device *phydev;          ///< PHY device itself
    uint32_t orig_supported;            ///< supported PHY ETHTOOL features
    /// PHY delays for each link mode
    struct kairos_delay delay[DEIPCE_LINK_MODE_COUNT];
};

/**
 * SFP module information.
 */
struct kairos_sfp {
    struct device_node *eeprom_node;    ///< SFP EEPROM node
    struct i2c_client *eeprom;          ///< SFP EEPROM I2C client
    enum kairos_sfp_type type;          ///< SFP module type
    uint32_t supported;                 ///< supported ETHTOOL features/speeds,
                                        ///< set also when SFP not in use
    struct kairos_phy phy;              ///< SFP PHY context
};

/**
 * FRS port adapter operations.
 */
struct kairos_port_adapter_ops {
    /**
     * Function to setup adapter and PHY for detected SFP type.
     */
    void (*setup) (struct kairos_port_priv *port);

    /**
     * Function to check link status from adapter, or NULL.
     * This is used when there is no PHY and adapter provides link status.
     */
    enum link_mode (*check_link) (struct kairos_port_priv *port);

    /**
     * Function to update link status to adapter, or NULL.
     * This is used when adapter needs to be informed
     * about link status changes.
     */
    void (*update_link) (struct kairos_port_priv *port);
};

/**
 * FRS port adapter structure.
 */
struct kairos_port_adapter {
    //@todo
    void __iomem *ioaddr;               ///< port adapter registers
    uint32_t supported;                 ///< supported ETHTOOL features/speeds,
                                        ///< set also when adapter is not
                                        ///< recognized
    uint8_t port;                       ///< current ETHTOOL port
    /// adapter delays for each link mode
    struct kairos_delay delay[DEIPCE_LINK_MODE_COUNT];
    struct kairos_port_adapter_ops ops; ///< port adapter operations
};

/**
 * FRS frame timestamper context for detecting timestamps of sent frames.
 */
struct kairos_tx_stamper {
    spinlock_t lock;                    ///< synchronize timestamper
    unsigned int next;                  ///< next FRS timestamper index
    unsigned int next_skb;              ///< next skb index
    /// original sk_buff in progress
    struct sk_buff *orig_skb[FRS_TS_NUMBER_TIMESTAMPS];
};                                      ///< PTP RX frame timestamper

/**
 * FRS port private information structure.
 */
struct kairos_port_priv {
    struct deipce_dev_priv *dp;         ///< back reference
    uint8_t port_num;                   ///< port number
    uint16_t trailer;                   ///< management trailer for sending

    struct delayed_work check_link;     ///< check link state periodically
    struct delayed_work capture_stats;  ///< capture statistics counters

    //@todo
    void __iomem *ioaddr;               ///< port registers
    enum kairos_medium_type medium_type; ///< medium type
    uint32_t flags;                     ///< port flags
    struct net_device *netdev;          ///< net_device for port

    struct kairos_port_adapter adapter; ///< adapter handling state
    struct kairos_phy ext_phy;          ///< external PHY device context
    struct kairos_sfp sfp;              ///< SFP module information

    struct mutex stats_lock;            ///< synchronize statistics access
    uint64_t stats[FRS_CNT_REG_COUNT];  ///< statistics

    struct mutex port_reg_lock;         ///< synchronize port register access

    int mirror_port;                    ///< mirror port or -1 to disable
    struct kairos_delay cur_delay;      ///< current total delays
    struct kairos_tx_stamper tx_stamper;       ///< PTP TX frame timestamper
    unsigned int rx_stamper;            ///< PTP RX frame timestamper index

    struct kairos_preempt preempt;      ///< preemption state
    struct {
        struct kairos_fsc_dev_priv *fsc;        ///< FSC instance
        unsigned int num;               ///< scheduler number
    } sched;                            ///< associated scheduler info
    struct {
        spinlock_t lock;                ///< synchronize access
        uint64_t bitrate[KAIROS_MAX_PRIO_QUEUES];       ///< rate in bps
    } shaper;
    struct kairos_port_mstp mstp;       ///< port MSTP support
};

/**
 * Device statistics. Other that netdev stats.
 */
struct kairos_stats {
    uint32_t rx_stamp;          ///< PTP timestamper RX event
    uint32_t tx_stamp;          ///< PTP timestamper TX event
    uint32_t rx_error;          ///< RX error event
    uint32_t congested;         ///< Congested event
    uint32_t tx_stamp_lost;     ///< PTP TX timestamps lost
};

/**
 * SMAC table row selection algorithms.
 */
enum frs_smac_row_sel {
    FLX_FRS_SMAC_ROW_SEL_NO_VLAN,       ///< XOR with VLAN ID disabled
    FLX_FRS_SMAC_ROW_SEL_VLAN,          ///< XOR with VLAN ID enabled
};

/**
 * SMAC table configuration.
 */
struct frs_smac_table_config {
    /// row selection algorithm for each column
    enum frs_smac_row_sel row_sel[FRS_SMAC_TABLE_COLS];
};

/**
 * FRS static MAC address table entry.
 */
struct frs_smac_table_entry {
    uint8_t mac_address[ETH_ALEN];      ///< MAC address
    uint16_t column;                    ///< static MAC address table column
    uint16_t config;                    ///< static MAC config register
                                        ///< (SMAC_TABLE0)
    uint16_t fwd_mask;                  ///< forward port mask
    uint16_t policed_mask;              ///< policed port mask
    uint16_t policer;                   ///< policer number
    uint16_t vlan;                      ///< VLAN number
};

// feature flags
#define FLX_FRS_FEAT_GIGABIT    BIT(0)  ///< gigabit speed
#define FLX_FRS_FEAT_STATS      BIT(1)  ///< statistics counters
#define FLX_FRS_FEAT_SHAPER     BIT(2)  ///< traffic shaper

/**
 * features.
 */
struct deipce_features {
    uint32_t flags;             ///< feature flags
    uint32_t clock_freq;        ///< clock frequency in Hz
    uint16_t smac_rows;         ///< number of static MAC address table rows
    uint16_t policers;          ///< number of policers
    uint16_t prio_queues;       ///< number of priority queues
    uint16_t mgmt_ports;        ///< bitmask of management trailer capable ports
    uint16_t hsr_ports;         ///< bitmask of HSR-capable ports
    uint16_t prp_ports;         ///< bitmask of PRP-capable ports
    uint16_t macsec_ports;      ///< bitmask of MACsec-capable ports
    uint16_t sched_ports;       ///< bitmask of scheduled ports
    uint16_t ts_ports;          ///< bitmask of ports with time stamper,
                                ///< zero means port 0 only
    uint16_t ct_ports;          ///< bitmask of ports with cut-through support
    uint16_t preempt_ports;     ///< bitmask of ports with preemption support
    uint16_t fids;              ///< number of FIDs
};

/**
 * device private information structure.
 */
struct kairos_dev_priv {
    struct device *this_dev;            ///< pointer to (platform) device
    unsigned int dev_num;               ///< number of this device
    resource_size_t base_addr;          ///< base address
    unsigned int irq;                   ///< IRQ number

    unsigned int trailer_len;           ///< management trailer length
    unsigned int trailer_offset;        ///< management trailer offset

    struct {
        unsigned int use_port_ts : 1;   ///< use port 0 timestamper only (false),
                                        ///< or port 0 (TX) and port N (RX)
                                        ///< time stampers (true)
        unsigned int irq_work_time_valid : 1;   ////< irq_work_time is set
    };
    struct work_struct irq_work;        ///< interrupt work
    struct timespec64 irq_work_time;    ///< timestamper clock time
    //@todo
    void __iomem *ioaddr;               ///< switch registers

    uint16_t cpu_port_mask;             ///< port mask of CPU port, 1 bit only
    uint16_t mgmt_tc;                   ///< management traffic class
    struct hwtstamp_config tstamp_cfg;  ///< hardware timestamper config,
                                        ///< synchronized via common_reg_lock
    /// PTP (port 0) RX frame timestamper
    struct kairos_tx_stamper rx_stamper;
    unsigned int tx_stamper;            ///< PTP TX frame timestamper index

    struct net_device *real_netdev;     ///< underlying MAC netdevice
    struct net_device *netdev;          ///< switch (endpoint) net device
    unsigned int num_of_ports;          ///< number of FRS ports
    /// Port privates (some may be NULL)
    struct kairos_port_priv *port[DEIPCE_MAX_PORTS];

    struct kairos_switchdev switchdev;  ///< switchdev support
    struct {
        unsigned int count;             ///< number of VLANs in use
        /// bitmap of new to-be-added VIDs
        unsigned long int new_vids[BITS_TO_LONGS(VLAN_N_VID)];
        uint16_t *fid_vlan_count;       ///< number of VLANs in each FID
        struct kairos_mstp mstp;        ///< MSTP support
    } vlan;                             ///< VLAN configuration information
    struct kairos_stats stats;          ///< IP statistics
    struct mutex common_reg_lock;       ///< synchronize switch register access
    struct mutex smac_table_lock;       ///< synchronize SMAC table access
    struct {
        struct frs_smac_table_config cfg; ///< table configuration
        unsigned long int *used;        ///< bitmap of used entries
        struct {
            unsigned int no_vlan;       ///< number of rows used without VLAN
            unsigned int vlan;          ///< number of rows used with VLAN
        } col_count[FRS_SMAC_TABLE_COLS]; ///< column usage counts
    } smac;                             ///< SMAC configuration information

    struct kairos_features features;    ///< switch features

    struct kairos_time *time;           ///< time interface
};

/**
 * FRS driver private information structure.
 */
struct kairos_drv_priv {
    struct workqueue_struct *wq_low;    ///< low priority work queue
    struct workqueue_struct *wq_high;   ///< high priority work queue

    unsigned int dev_count;             ///< FRS device count
    /// Privates of found devices
    struct kairos_dev_priv *dev_priv[KAIROS_MAX_DEVICES];

    uint64_t *crc40_table;              ///< small CRC40 table (4-bit index)
};

/**
 * Get pointer to given register for use with memory mapped I/O.
 * @param ioaddr Pointer to start of mapped register area.
 * @param regnum Register number.
 * @return Memory mapped IO address of register regnum.
 */
static inline void __iomem *deipce_reg_addr(void __iomem *ioaddr,
                                            unsigned int regnum)
{
    // Byte addressing: x2 is needed
    return ioaddr + (regnum << 1);
}

/**
 * Helper to read 32-bit port counter values.
 * @param pp FRS port privates.
 * @param low_reg_num Register number of the low word of 32-bit counter.
 * @return 32-bit counter value or 0xffffffff on error.
 */
static inline uint32_t deipce_read_port_counter(struct deipce_port_priv *pp,
                                                unsigned int low_reg_num)
{
    // They are properly aligned.
    return ioread32(deipce_reg_addr(pp->ioaddr, low_reg_num));
}

static inline uint16_t deipce_read_port_reg(struct deipce_port_priv *pp,
                                            unsigned int regnum)
{
    return ioread16(deipce_reg_addr(pp->ioaddr, regnum));
}

static inline void deipce_write_port_reg(struct deipce_port_priv *pp,
                                         unsigned int regnum,
                                         uint16_t value)
{
    iowrite16(value, deipce_reg_addr(pp->ioaddr, regnum));
}

/**
 * Read adapter register. Adapters are optional, this must not be called
 * if port has no adapter defined.
 */
static inline uint16_t deipce_read_adapter_reg(struct deipce_port_priv *pp,
                                               unsigned int regnum)
{
    return ioread16(deipce_reg_addr(pp->adapter.ioaddr, regnum));
}

/**
 * Write adapter register. Adapters are optional, this must not be called
 * if port has no adapter defined.
 */
static inline void deipce_write_adapter_reg(struct deipce_port_priv *pp,
                                            unsigned int regnum,
                                            uint16_t value)
{
    iowrite16(value, deipce_reg_addr(pp->adapter.ioaddr, regnum));
}

static inline uint16_t deipce_read_switch_reg(struct deipce_dev_priv *dp,
                                              unsigned int regnum)
{
    return ioread16(deipce_reg_addr(dp->ioaddr, regnum));
}

static inline void deipce_write_switch_reg(struct deipce_dev_priv *dp,
                                           unsigned int regnum,
                                           uint16_t value)
{
    iowrite16(value, deipce_reg_addr(dp->ioaddr, regnum));
}

static inline uint32_t deipce_read_switch_uint32(struct deipce_dev_priv *dp,
                                                 unsigned int regnum)
{
    return ioread32(deipce_reg_addr(dp->ioaddr, regnum));
}

/**
 * Read switch generic register value.
 * @param dp Switch privates.
 * @param reg Generic register to read.
 * @param mask Bitmask for value.
 * @return Masked generic value.
 */
static inline uint16_t deipce_read_generic(struct deipce_dev_priv *dp,
                                           unsigned int reg,
                                           uint16_t mask)
{
    // Generics addresses are already byte addresses.
    return deipce_read_switch_reg(dp, reg >> 1) & mask;
}

#endif
