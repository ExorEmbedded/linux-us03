
#include <linux/net_tstamp.h>
#include <net/switchdev.h>

#include "kairos.h"
#include "kairos-regs.h"


extern struct kairos_data* kairos_global;
extern struct ptp_clock_info kairos_pch_caps;

extern int kairos_phy_add(unsigned int irq, int phy_addr,
		  struct fixed_phy_status *status,
		  struct dsa_switch* ds);
extern struct phy_device *kairos_phy_register(unsigned int irq,
	      struct fixed_phy_status *status,
	      struct dsa_switch* ds,
	      struct device_node *np);
extern void kairos_phy_unregister(struct phy_device* phy);

/*-------------------------------------------------------------------------*/
/* 
 * Switch implementation 
 */                                   
/*-------------------------------------------------------------------------*/
static struct fixed_phy_status kairos_phy_status __initdata = {
	.link		= 1,
	.speed		= 100,
	.duplex		= 1,
};

static struct net_device* kairos_get_netdev(struct dsa_switch* ds)
{
	struct net_device* netdev = ds->dst->master_netdev;
	if (!netdev)
		netdev = ds->master_netdev;

	return netdev;
}

static int kairos_switch_setup_global(struct dsa_switch* ds, bool enable)
{
    u16 data;
    u16 wdata;
    int port, prio;
	struct kairos_data *kairos = (struct kairos_data*)ds->priv;
	if (!kairos)
		return -EINVAL;

	dev_info(&kairos->spi->dev, "setting up switch...");

    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_CTRL, 0x0001);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_DAT, 0x0101);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_CMD, 0x8104);

    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_DAT, 0x3300);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_CMD, 0x8100);

    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_DAT, 0x0101);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_CMD, 0x8204);

    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_DAT, 0x3300);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_SMI_CMD, 0x8200);

    /* set port limits */
    for (prio=0; prio<=TSN_PER_XSEL_PRIO_MAX_VALUE; prio++)
    {
        kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_PSEL, 0x0030);
        kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_PRCFG, 0x8000 | prio);
    }

    /* limit background collective queue */
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_PSEL, 0x0300);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_LIMITS0, 0x0000);

    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_PSEL, 0x0320);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_LIMITS0, 0x0000);

    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_PSEL, 0x0330);
    kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_HR_LIMITS0, 0x0000);

    /* value for Control Status register */
    if (enable)
        data = 0x00000001;  /* Enable  */
    else
        data = 0x00000000;  /* Disable */

    /* save register value */
    wdata = data;

    kairos->shadow_regs[TSN_SHADOW_CNTRL_STATUS] = data;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_TR_CTRL, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (1)");
        return -1;
    }

    /* check status */
    if (true == enable)
    {
        if (-1 == kairos_check_enabled(kairos))
        {
			dev_err(&kairos->spi->dev, "switch is not enabled");
			return -1;
        }
    }

    /* Reset Packet Logger, prepare shadows */
    data = 0x00000000;
    kairos->shadow_regs[TSN_SHADOW_PLCTRL] = data;
    data |= 0x8000;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_TR_PLCTRL, data)) // clear contents
    {
		dev_err(&kairos->spi->dev, "error writing register (2)");
        return -1;
    }

    data = 0x00000000;
    kairos->shadow_regs[TSN_SHADOW_PLMASKS] = data;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_TR_PLMASKS, data)) // clear contents
    {
		dev_err(&kairos->spi->dev, "error writing register (3)");
        return -1;
    }

    /* Enable OBT */
#define KAIROS_ALWAYS_OBT    
#ifdef KAIROS_ALWAYS_OBT     
    data = TSN_SWITCH_CFG_ALWAYS_OBT | TSN_SWITCH_CFG_FDB_AGE_EN | TSN_SWITCH_CFG_FDB_LRN_EN;
#else    
    data = TSN_SWITCH_CFG_FDB_AGE_EN | TSN_SWITCH_CFG_FDB_LRN_EN;
#endif    
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_SWCFG, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (4)");
        return -1;
    }

    /* VLAN membership : all untagged members in vlan 1 */
    data = 0x0001;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_VIDCFG, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (5)");
        return -1;
    }

    data = 0x0055;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_VIDMBRCFG, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (6)");
        return -1;
    }

    /* unblock all ports */
    data = 0x0000;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PSEL, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (7)");
        return -1;
    }

    data = 0x0003;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PTCFG, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (8)");
        return -1;
    }
    data = 0x0010;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PSEL, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (9)");
        return -1;
    }
    data = 0x0003;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PTCFG, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (10)");
        return -1;
    }
    data = 0x0020;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PSEL, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (11)");
        return -1;
    }
    data = 0x0003;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PTCFG, data))
    {
        dev_err(&kairos->spi->dev, "error writing register (12)");
        return -1;
    }
    data = 0x0030;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PSEL, data))
    {
        dev_err(&kairos->spi->dev, "error writing register (13)");
        return -1;
    }
    data = 0x0003;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PTCFG, data))
    {
        dev_err(&kairos->spi->dev, "error writing register (14)");
        return -1;
    }

    data = 0x0022;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PSEL, data))
    {
        dev_err(&kairos->spi->dev, "error writing register (15)");
        return -1;
    }

    /*
    data = 0x0000;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_PTPRCFG, data))
    {
		dev_err(&kairos->spi->dev, "error writing register (15)");
        return -1;
    }
    */

    for (port = 0; port <= TSN_PER_XSEL_PORT_MAX_VALUE; port++) {
        for (prio = 0; prio <= TSN_PER_XSEL_PRIO_MAX_VALUE; prio++) {
            kairos->shadow_max_sdu[port][prio] = TSN_MAXSDU_DEFVAL;
        }
    }

    /* Set Qmon to something useful for demo purposes, like port 2 tc 2*/
    //tsnSelectQmonPortTc(2, 2);

#ifdef TSN_TGD_WORKAROUND
    /* Select front TGD (2), workaround for two port IP < 14.17 for single port only */
    data = 0x00000002;
    if (KAIROS_ERR_OK != kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_TGDSEL, data))
    {
        return -1;
    }
#endif

	dev_info(&kairos->spi->dev, "switch setup completed");

    return 1;
}

static int kairos_switch_reset(struct dsa_switch *ds)
{
	struct kairos_data *kairos = (struct kairos_data*)ds->priv;
	if (!kairos)
		return -EINVAL;

	dev_info(&kairos->spi->dev, "resetting switch...");

	// disable ports
	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_PORTEN, 0x0000);

	// set reset signal to 0
	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_GPIOA_OUT, 0x0000);
	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_GPIOB_OUT, 0x0000);

	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_GPIOA_TRIS, 0x0001);
	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_GPIOB_TRIS, 0x0001);

	// minimum reset pulse width
	usleep_range(20000, 30000);

	// set reset signal to 1
	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_GPIOA_OUT, 0x0001);
	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_GPIOB_OUT, 0x0001);

	// wait for stable clock
	usleep_range(200000, 300000);

	// enable ports
	kairos_reg_write(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_PORTEN, 0x0003);

	dev_info(&kairos->spi->dev, "switch reset completed");
	return 0;
}

int kairos_link_status(struct dsa_switch* ds, u8 port)
{
	u16 data;
	u16 mask = 0x0001 << port;
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;

	data = 0x0000;
	kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_GR_LINK, &data);

	return (data & mask)? 1 : 0;
}

int kairos_enable_hwtstamp(struct dsa_switch* ds, struct ifreq *ifr)
{
	u16 data;
	struct hwtstamp_config config;
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		kairos->hw_tx_tstamp_on = 0;
		break;
	case HWTSTAMP_TX_ON:
		kairos->hw_tx_tstamp_on = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		kairos->hw_rx_tstamp_on = 0;
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;

	default:
		kairos->hw_rx_tstamp_on = 1;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	}

	if (copy_to_user(ifr->ifr_data, &config, sizeof(config)))
	    return -EFAULT;

	data = 0x0000;
	if (kairos->hw_tx_tstamp_on)
		data |= 0x4000;
	if (kairos->hw_rx_tstamp_on)
		data |= 0x8000;

	dev_dbg(&kairos->spi->dev, "enabling timestamping (value: 0x%04X)", data);
	kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_TR_ITSCTL, data);
	return 0;
}

static enum dsa_tag_protocol kairos_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_KAIROS;
}

static const char *kairos_drv_probe(struct device *dsa_dev,
				       struct device *host_dev, int sw_addr,
				       void **_priv)
{
	int i; 
	struct dsa_switch* ds = (struct dsa_switch*)dsa_dev; //container_of(&dsa_dev, struct dsa_switch, dev);
	struct kairos_data *kairos = kairos_global;
	if (!kairos)
		return NULL;

	// add PHC devices
	kairos->caps = kairos_pch_caps;
	kairos->ptp_clocks[0] = ptp_clock_register(&kairos->caps, dsa_dev);

	for (i=0; i<KAIROS_MAX_PORTS; i++)
	{
		// add simulated phys
		kairos->phys[i] = kairos_phy_register(-1, &kairos_phy_status, ds, NULL);

		kairos->ptp_clocks[i] = kairos->ptp_clocks[0];
		if (IS_ERR(kairos->ptp_clocks[i])) 
		{
			dev_err(&kairos->spi->dev, "error creating pch device: %ld\n", PTR_ERR(kairos->ptp_clocks[i]));
		}
		else
		{
			dev_info(&kairos->spi->dev, "pch device ptp%d created\n", ptp_clock_index(kairos->ptp_clocks[i]));
		}
	}

	kairos->ds = ds;
	kairos->sw_addr = sw_addr;

	ds->priv = kairos;
	*_priv = kairos;

	kairos_switch_reset(ds);
	kairos_switch_setup_global(ds, true);

	dev_info(&kairos->spi->dev, "switch probe completed");

	return "KAIROS";
}

static int kairos_setup(struct dsa_switch *ds)
{
	struct net_device* netdev;
	struct kairos_data *kairos;

	netdev = kairos_get_netdev(ds);
	if (!netdev)
		return -EINVAL;

	kairos = (struct kairos_data*)ds->priv;
	if (!kairos)
		return -EINVAL;

	kairos_build_date(kairos);
	kairos_fw_release(kairos);
	kairos_status(kairos);

	dev_info(&kairos->spi->dev, "kairos switch setup");
	return 0;
}

static int kairos_set_addr(struct dsa_switch *ds, u8 *addr)
{
	//@todo
	struct net_device* netdev = kairos_get_netdev(ds);
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;
	if (!kairos)
		return -EINVAL;

	(void)netdev;

	dev_info(&kairos->spi->dev, "kairos switch set addr");
	return 0;
}

static int kairos_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	u16 link_status;
	int result;
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;

	struct net_device* netdev = kairos_get_netdev(ds);
	(void)netdev;

	dev_info(&kairos->spi->dev, "kairos switch phy read port %d, reg %d", port, regnum);

	// only read of register 0x01 (status register) is supported
	if (regnum != 0x01)
		return 0xffff;

	//@todo
	// read link status
	link_status = 0x0000;
	kairos_reg_read(kairos, KAIROS_MODULE_GENERAL, KAIROS_REG_PTP_CLK_RD, &link_status);

	result = link_status;

	return result;
}

static int kairos_phy_write(struct dsa_switch *ds, int port, int regnum, u16 val)
{
	// write not supported
	struct net_device* netdev = kairos_get_netdev(ds);
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;
	if (!kairos)
		return -EINVAL;

	(void)netdev;

	dev_info(&kairos->spi->dev, "kairos switch phy write");
	return 0xffff;
}

extern int _kairos_pch_gettime(struct kairos_data* kairos, struct timespec64 *ts);
void kairos_netdev_tx_start(struct sk_buff* skb, void* user_data)
{
#if 0
	struct kairos_data* kairos = (struct kairos_data*)user_data;

	if (!kairos)
		return;

printk(KERN_INFO "%s (1) len %d flags %X\n", __func__, skb->len, skb_shinfo(skb)->tx_flags);
	if ((unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) && kairos->hw_tx_tstamp_on)
	{
		struct timespec64 ts;
		struct skb_shared_hwtstamps hwtstamps;

		memset(&hwtstamps, 0, sizeof(hwtstamps));
		_kairos_pch_gettime(kairos, &ts);

		hwtstamps.hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);
printk(KERN_INFO "%s (2) ts %llu\n", __func__, hwtstamps.hwtstamp.tv64);

		skb_tx_timestamp(skb);
		skb_tstamp_tx(skb, &hwtstamps);
		skb_shinfo(skb)->tx_flags &= (~SKBTX_IN_PROGRESS);

		kairos->tx_skb = skb;
	}
#endif	
}

void kairos_netdev_tx_complete(struct sk_buff* skb, void* user_data)
{
#if 0
	struct kairos_data* kairos = (struct kairos_data*)user_data;

	if (!kairos)
		return;

printk(KERN_INFO "%s (1) len %d flags %X\n", __func__, skb->len, skb_shinfo(skb)->tx_flags);
	//if ((unlikely(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)) && kairos->hw_tx_tstamp_on)
	if (kairos->hw_tx_tstamp_on)
	{
		struct skb_shared_hwtstamps hwtstamps;

		memset(&hwtstamps, 0, sizeof(hwtstamps));
		kairos_read_timestamp(kairos, 0, 0, &hwtstamps.hwtstamp);
		if (hwtstamps.hwtstamp.tv64 == 0)
		{
			struct timespec64 ts;
			_kairos_pch_gettime(kairos, &ts);

			hwtstamps.hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);
printk(KERN_INFO "%s (2) ts %llu\n", __func__, hwtstamps.hwtstamp.tv64);
		}
else printk(KERN_INFO "%s (3) ts %llu\n", __func__, hwtstamps.hwtstamp.tv64);

		skb_tstamp_tx(skb, &hwtstamps);
		skb_shinfo(skb)->tx_flags &= (~SKBTX_IN_PROGRESS);
		
		//dev_kfree_skb_any(skb);
		kairos->tx_skb = NULL;
	}
#endif
}

void kairos_netdev_rx_complete(struct sk_buff* skb, void* user_data)
{
#if 0
	struct skb_shared_hwtstamps *shhwtstamps;
	struct kairos_data* kairos = (struct kairos_data*)user_data;

	if (!kairos)
		return;

	if (kairos->hw_rx_tstamp_on)
	{
		/*
//printk(KERN_INFO "%s ts: %d\n", __func__, kairos->hw_rx_tstamp_on);
		shhwtstamps = skb_hwtstamps(skb);

		memset(shhwtstamps, 0, sizeof(*shhwtstamps));
		kairos_read_timestamp(kairos, 0, 1, &shhwtstamps->hwtstamp);

		if (shhwtstamps->hwtstamp.tv64 == 0)
		*/
		{
			struct timespec64 ts;
			_kairos_pch_gettime(kairos, &ts);

			shhwtstamps->hwtstamp.tv64 = timespec64_to_ns(&ts);
		}

//printk(KERN_INFO "%s len: %d ts: %d (2) %llu\n", __func__, skb->len, kairos->hw_rx_tstamp_on, shhwtstamps->hwtstamp.tv64);
	}
#endif	
}

static void kairos_adjust_link(struct dsa_switch *ds, int port,
				struct phy_device *phydev)
{
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;
	struct net_device* netdev = kairos_get_netdev(ds);

printk(KERN_INFO "kairos switch adjust link port %d %p %p\n", port, kairos, netdev);
	if (netdev)
	{
		dev_info(&netdev->dev, "kairos switch: registering PTP support");
		kairos->fec_cbs.tx_start = kairos_netdev_tx_start;
		kairos->fec_cbs.tx_complete = kairos_netdev_tx_complete;
		kairos->fec_cbs.rx_complete = kairos_netdev_rx_complete;
		kairos->fec_cbs.user_data = kairos;

		fec_ext_callbacks(netdev, &kairos->fec_cbs);
	}
}

static void	kairos_fixed_link_update(struct dsa_switch *ds, int port,
				struct fixed_phy_status *st)
{
	struct net_device* netdev = kairos_get_netdev(ds);
	dev_info(&netdev->dev, "kairos switch fixed link update");
}

struct COUNTER_INFO
{
    const char* name;
    uint8_t mask;
};

const struct COUNTER_INFO COUNTER_INFOS[KAIROS_NUM_COUNTERS] =
{
    { "Rx filtered", 0x0F },
    { "Rx octets", 0x0C },
    { "Rx tagged", 0x0C },
    { "Rx errors", 0x0C },
    { "Rx overload", 0x0F },
    { "Rx unicast", 0x0C },
    { "Rx multicast", 0x0C },
    { "Rx broadcast", 0x0C },
    { "Rx < 64 bytes", 0x0C },
    { "Rx 64 bytes", 0x0C },
    { "Rx < 128 bytes", 0x0C },
    { "Rx < 256 bytes", 0x0C },
    { "Rx < 512 bytes", 0x0C },
    { "Rx < 1024 bytes", 0x0C },
    { "Rx <=1518 bytes", 0x0C },
    { "Rx > 1518 bytes", 0x0C },
    { "Tx 0 dropped", 0x0F },
    { "Tx 1 dropped", 0x0F },
    { "Tx 2 dropped", 0x0F },
    { "Tx 3 dropped", 0x0F },
    { "Tx 4 dropped", 0x0F },
    { "Tx 5 dropped", 0x0F },
    { "Tx 6 dropped", 0x0F },
    { "Tx 7 dropped", 0x0F },
    { "Rx 0 frames", 0x0F },
    { "Rx 1 frames", 0x0F },
    { "Rx 2 frames", 0x0F },
    { "Rx 3 frames", 0x0F },
    { "Rx 4 frames", 0x0F },
    { "Rx 5 frames", 0x0F },
    { "Rx 6 frames", 0x0F },
    { "Rx 7 frames", 0x0F },
    { "Tx octets", 0x0C },
    { "Tx tagged", 0x0C },
    { "Tx errors", 0x0C },
    { "Tx unicast", 0x0C },
    { "Tx multicast", 0x0C },
    { "Tx broadcast", 0x0C },
    { "Tx < 64 bytes", 0x0C },
    { "Tx 64 bytes", 0x0C },
    { "Tx < 128 bytes", 0x0C },
    { "Tx < 256 bytes", 0x0C },
    { "Tx < 512 bytes", 0x0C },
    { "Tx < 1024 bytes", 0x0C },
    { "Tx <=1518 bytes", 0x0C },
    { "Tx > 1518 bytes", 0x0C },
    { "Tx 0 overrun", 0x0E },
    { "Tx 1 overrun", 0x0E },
    { "Tx 2 overrun", 0x0E },
    { "Tx 3 overrun", 0x0E },
    { "Tx 4 overrun", 0x0E },
    { "Tx 5 overrun", 0x0E },
    { "Tx 6 overrun", 0x0E },
    { "Tx 7 frames", 0x0F },
    { "Tx 0 frames", 0x0F },
    { "Tx 1 frames", 0x0F },
    { "Tx 2 frames", 0x0F },
    { "Tx 3 frames", 0x0F },
    { "Tx 4 frames", 0x0F },
    { "Tx 5 frames", 0x0F },
    { "Tx 6 frames", 0x0F },
    { "Tx 7 frames", 0x0F }
}; 

static void	kairos_get_strings(struct dsa_switch *ds, int stringset, uint8_t *data)
{
	int i;

	for (i = 0; i < KAIROS_NUM_COUNTERS; i++) {
		memcpy(data, COUNTER_INFOS[i].name, ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}
}
	
static void kairos_get_ethtool_stats(struct dsa_switch *ds,
				     int port, u64 *data)
{
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;
	u32 counter;
	int i;
	int mask = 0x01 << port;

	for (i = 0; i < KAIROS_NUM_COUNTERS; i++) {
		counter = 0;
		if (COUNTER_INFOS[i].mask & mask)
			kairos_reg_counter(kairos, port, i, &counter);	

		*data = (u64)counter;
		data ++;
	}
}

static int kairos_get_sset_count(struct dsa_switch *ds)
{
	return KAIROS_NUM_COUNTERS;
}

int	kairos_port_fdb_prepare(struct dsa_switch *ds, int port,
				    const struct switchdev_obj_port_fdb *fdb,
				    struct switchdev_trans *trans)
{
	return 0;
}

void kairos_port_fdb_add(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_fdb *fdb,
				struct switchdev_trans *trans)
{
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;
	uint16_t tmp;
	uint16_t macL;
	uint16_t macM;
	uint16_t macH;
	uint16_t metadata;
	uint16_t cmd;

	if (!kairos)
		return;

	dev_info(&kairos->spi->dev, "adding FDB entry %02X:%02X:%02X:%02X:%02X:%02X, vid: %d, nmd: %d",
		fdb->addr[0], fdb->addr[1], fdb->addr[2], fdb->addr[3], fdb->addr[4], fdb->addr[5],
		fdb->vid,
		fdb->ndm_state);

	tmp = fdb->addr[4]; macL = (tmp <<= 8) | fdb->addr[5];
	tmp = fdb->addr[2]; macM = (tmp <<= 8) | fdb->addr[3];
	tmp = fdb->addr[0]; macH = (tmp <<= 8) | fdb->addr[1];

	metadata = TSN_FDB_META_NO_FLAGS;
    if (fdb->vid != 0)
    	metadata = TSN_FDB_META_REPRIO | ((fdb->vid << 12) & TSN_FDB_META_REPRIO_MASK);

    metadata |= (TSN_FDB_META_PASSBLOCKED | TSN_FDB_META_OBT | TSN_FDB_META_PORT_CASCADING);

    cmd = 0x0000;

    kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBWRL, macL);
    kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBWRM, macM);
    kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBWRH, macH);

    kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBWRM0, metadata);
    kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBWRCMD, cmd);
}

int	kairos_port_fdb_del(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_fdb *fdb)
{
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;
	int idx = 0;
	uint16_t tmp;
	uint16_t macL, emacL;
	uint16_t macM, emacM;
	uint16_t macH, emacH;
	uint16_t metadata;
	uint16_t cmd;
	uint16_t fdbmax;
	uint16_t feabits;
	int num_entries;

	if (!kairos)
		return -EINVAL;

	dev_info(&kairos->spi->dev, "deleting FDB entry %02X:%02X:%02X:%02X:%02X:%02X",
		fdb->addr[0], fdb->addr[1], fdb->addr[2], fdb->addr[3], fdb->addr[4], fdb->addr[5]);

	tmp = fdb->addr[4]; emacL = (tmp <<= 8) | fdb->addr[5];
	tmp = fdb->addr[2]; emacM = (tmp <<= 8) | fdb->addr[3];
	tmp = fdb->addr[0]; emacH = (tmp <<= 8) | fdb->addr[1];

	// reset read pointer
    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBMAX, &fdbmax);

    // get number of entries
    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FEABITS0, &feabits);
	num_entries = (feabits & TSN_FEATUREBITS_FDBBINS_MASK) >> 4;

	dev_dbg(&kairos->spi->dev, "looking for %d FDB entries", num_entries);

	while (num_entries > 0)
	{
		// read until MAC address is null
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBRDL, &macL);
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBRDM, &macM);
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBMDRD, &metadata);
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBRDH, &macH);

	    if ((macL == emacL) && (macM == emacM) && (macH == emacH))
	    {
	    	// entry found: delete
	    	cmd = TSN_FDB_CMD_DEL | (idx & TSN_FDB_CMD_IDX_MASK);
		    kairos_reg_write(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBWRCMD, cmd);

		    return 0;
	    }

	    idx ++;
	    num_entries --;

	} while ((macL != 0) || (macM != 0) || (macH != 0));

	return -EINVAL;
}

int	kairos_port_fdb_dump(struct dsa_switch *ds, int port,
				 struct switchdev_obj_port_fdb *fdb,
				 int (*cb)(struct switchdev_obj *obj))
{
	struct kairos_data* kairos = (struct kairos_data*)ds->priv;
	uint16_t macL;
	uint16_t macM;
	uint16_t macH;
	uint16_t metadata;
	uint16_t fdbmax;
	uint16_t feabits;
	int num_entries;

	if (!kairos)
		return -EINVAL;

	// reset read pointer
    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBMAX, &fdbmax);

    // get number of entries
    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FEABITS0, &feabits);
	num_entries = (feabits & TSN_FEATUREBITS_FDBBINS_MASK) >> 4;

	dev_info(&kairos->spi->dev, "dumping %d FDB entries", num_entries);

	while (num_entries > 0)
	{
		// read until MAC address is null
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBRDL, &macL);
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBRDM, &macM);
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBMDRD, &metadata);
	    kairos_reg_read(kairos, KAIROS_MODULE_TSN, KAIROS_REG_HR_FDBRDH, &macH);

	    if ((macL != 0) || (macM != 0) || (macH != 0))
	    {
		    fdb->addr[1] = (unsigned char)(macH & 0x00FF); macH >>= 8;
		    fdb->addr[0] = (unsigned char)macH;

		    fdb->addr[3] = (unsigned char)(macM & 0x00FF); macM >>= 8;
		    fdb->addr[2] = (unsigned char)macM;

		    fdb->addr[5] = (unsigned char)(macL & 0x00FF); macL >>= 8;
		    fdb->addr[4] = (unsigned char)macL;

		    fdb->vid = (metadata >> 8) & 0x0F;

		    // invoke callback
		    cb(&fdb->obj);
		}

		num_entries --;
	} 

	return 0;
}

struct dsa_switch_ops kairos_switch_ops = {
	.get_tag_protocol = kairos_get_tag_protocol,
	.probe		= kairos_drv_probe,
	.setup		= kairos_setup,
	.set_addr	= kairos_set_addr,
	.phy_read	= kairos_phy_read,
	.phy_write	= kairos_phy_write,
	.adjust_link = kairos_adjust_link,
	.fixed_link_update = kairos_fixed_link_update,
	.get_strings = kairos_get_strings,
	.get_ethtool_stats = kairos_get_ethtool_stats,
	.get_sset_count = kairos_get_sset_count,

	.port_fdb_prepare = kairos_port_fdb_prepare,
	.port_fdb_add = kairos_port_fdb_add,
	.port_fdb_del = kairos_port_fdb_del,
	.port_fdb_dump = kairos_port_fdb_dump,

};

int kairos_switch_init(struct kairos_data* kairos)
{
	// register switch device
	register_switch_driver(&kairos_switch_ops);

	return 0;
}