/*
 * Kairos MDIO bus (MDIO bus emulation with fixed PHYs)
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/seqlock.h>
#include <linux/idr.h>
#include <linux/net_tstamp.h>

#include "swphy.h"


struct kairos_mdio_bus {
	struct mii_bus *mii_bus;
	struct list_head phys;
};

struct kairos_phy {
	int addr;
	struct phy_device *phydev;
	seqcount_t seqcount;
	struct fixed_phy_status status;
	int (*link_update)(struct net_device *, struct fixed_phy_status *);
	struct list_head node;
	struct dsa_switch* ds;
};

static struct platform_device *pdev;
static struct kairos_mdio_bus platform_fmb = {
	.phys = LIST_HEAD_INIT(platform_fmb.phys),
};

extern int kairos_link_status(struct dsa_switch* ds, u8 port);
int kairos_enable_hwtstamp(struct dsa_switch* ds, struct ifreq *ifr);

static void kairos_phy_update(struct kairos_phy *fp)
{
	struct phy_device* phydev = fp->phydev;
	if (!phydev)
		return;

	// TODO: read SPI register
	fp->status.link = kairos_link_status(fp->ds, phydev->mdio.addr);
	fp->status.link = 1;
}

static int kairos_mdio_read(struct mii_bus *bus, int phy_addr, int reg_num)
{
	struct kairos_mdio_bus *fmb = bus->priv;
	struct kairos_phy *fp;

	list_for_each_entry(fp, &fmb->phys, node) {
		if (fp->addr == phy_addr) {
			struct fixed_phy_status state;
			int s;

			do {
				s = read_seqcount_begin(&fp->seqcount);
				/* Issue callback if user registered it. */
				if (fp->link_update) {
					fp->link_update(fp->phydev->attached_dev,
							&fp->status);
					kairos_phy_update(fp);
				}
				state = fp->status;
			} while (read_seqcount_retry(&fp->seqcount, s));

			if ((reg_num == 2) || (reg_num == 3))
				return 0x1234;

			return swphy_read_reg(reg_num, &state);
		}
	}

	return 0xFFFF;
}

static int kairos_mdio_write(struct mii_bus *bus, int phy_addr, int reg_num,
			    u16 val)
{
	return 0;
}

/*
 * If something weird is required to be done with link/speed,
 * network driver is able to assign a function to implement this.
 * May be useful for PHY's that need to be software-driven.
 */
int kairos_phy_set_link_update(struct phy_device *phydev,
			      int (*link_update)(struct net_device *,
						 struct fixed_phy_status *))
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct kairos_phy *fp;

	if (!phydev || !phydev->mdio.bus)
		return -EINVAL;

	list_for_each_entry(fp, &fmb->phys, node) {
		if (fp->addr == phydev->mdio.addr) {
			fp->link_update = link_update;
			fp->phydev = phydev;
			return 0;
		}
	}

	return -ENOENT;
}
EXPORT_SYMBOL_GPL(kairos_phy_set_link_update);

int kairos_phy_update_state(struct phy_device *phydev,
			   const struct fixed_phy_status *status,
			   const struct fixed_phy_status *changed)
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct kairos_phy *fp;

	if (!phydev || phydev->mdio.bus != fmb->mii_bus)
		return -EINVAL;

	list_for_each_entry(fp, &fmb->phys, node) {
		if (fp->addr == phydev->mdio.addr) {
			write_seqcount_begin(&fp->seqcount);
#define _UPD(x) if (changed->x) \
	fp->status.x = status->x
			_UPD(link);
			_UPD(speed);
			_UPD(duplex);
			_UPD(pause);
			_UPD(asym_pause);
#undef _UPD
			kairos_phy_update(fp);
			write_seqcount_end(&fp->seqcount);
			return 0;
		}
	}

	return -ENOENT;
}
EXPORT_SYMBOL(kairos_phy_update_state);

int kairos_phy_add(unsigned int irq, int phy_addr,
		  struct fixed_phy_status *status,
		  struct dsa_switch* ds,
		  	struct kairos_phy** newfp)
{
	int ret;
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct kairos_phy *fp;

	ret = swphy_validate_state(status);
	if (ret < 0)
		return ret;

	fp = kzalloc(sizeof(*fp), GFP_KERNEL);
	if (!fp)
		return -ENOMEM;

	seqcount_init(&fp->seqcount);

	if (irq != PHY_POLL)
		fmb->mii_bus->irq[phy_addr] = irq;

	fp->addr = phy_addr;
	fp->status = *status;
	fp->ds = ds;

	kairos_phy_update(fp);

	list_add_tail(&fp->node, &fmb->phys);

	*newfp = fp;

	return 0;
}
EXPORT_SYMBOL_GPL(kairos_phy_add);

static DEFINE_IDA(phy_fixed_ida);

static void kairos_phy_del(int phy_addr)
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct kairos_phy *fp, *tmp;

	list_for_each_entry_safe(fp, tmp, &fmb->phys, node) {
		if (fp->addr == phy_addr) {
			list_del(&fp->node);
			kfree(fp);
			ida_simple_remove(&phy_fixed_ida, phy_addr);
			return;
		}
	}
}

struct phy_device *kairos_phy_register(unsigned int irq,
				      struct fixed_phy_status *status,
				      struct dsa_switch* ds,
				      struct device_node *np)
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct phy_device *phy;
	struct kairos_phy *fp;
	int phy_addr;
	int ret;

	if (!fmb->mii_bus || fmb->mii_bus->state != MDIOBUS_REGISTERED)
		return ERR_PTR(-EPROBE_DEFER);

	/* Get the next available PHY address, up to PHY_MAX_ADDR */
	phy_addr = ida_simple_get(&phy_fixed_ida, 0, PHY_MAX_ADDR, GFP_KERNEL);
	if (phy_addr < 0)
		return ERR_PTR(phy_addr);

	ret = kairos_phy_add(irq, phy_addr, status, ds, &fp);
	if (ret < 0) {
		ida_simple_remove(&phy_fixed_ida, phy_addr);
		return ERR_PTR(ret);
	}

	phy = get_phy_device(fmb->mii_bus, phy_addr, false);
	if (IS_ERR(phy)) {
		kairos_phy_del(phy_addr);
		return ERR_PTR(-EINVAL);
	}

	fp->phydev = phy;

	/* propagate the fixed link values to struct phy_device */
	phy->link = status->link;
	if (status->link) {
		phy->speed = status->speed;
		phy->duplex = status->duplex;
		phy->pause = status->pause;
		phy->asym_pause = status->asym_pause;
	}

	of_node_get(np);
	phy->mdio.dev.of_node = np;
	phy->is_pseudo_fixed_link = true;

	switch (status->speed) {
	case SPEED_1000:
		phy->supported = PHY_1000BT_FEATURES;
		break;
	case SPEED_100:
		phy->supported = PHY_100BT_FEATURES;
		break;
	case SPEED_10:
	default:
		phy->supported = PHY_10BT_FEATURES;
	}

	ret = phy_device_register(phy);
	if (ret) {
		phy_device_free(phy);
		of_node_put(np);
		kairos_phy_del(phy_addr);
		return ERR_PTR(ret);
	}

	return phy;
}
EXPORT_SYMBOL_GPL(kairos_phy_register);

void kairos_phy_unregister(struct phy_device *phy)
{
	phy_device_remove(phy);
	of_node_put(phy->mdio.dev.of_node);
	kairos_phy_del(phy->mdio.addr);
}
EXPORT_SYMBOL_GPL(kairos_phy_unregister);

struct mii_bus* kairos_phy_bus(void)
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	return fmb->mii_bus;
}
EXPORT_SYMBOL_GPL(kairos_phy_bus);

static int kairos_phy_probe(struct phy_device *phydev)
{
	return 0;
}

static int kairos_phy_aneg_done(struct phy_device *phydev)
{
	int result = genphy_config_aneg(phydev);
	return 1;
}

static int kairos_phy_config_init(struct phy_device *phydev)
{
	int result = genphy_config_init(phydev);
	return result;
}

static int kairos_phy_read_status(struct phy_device *phydev)
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct kairos_phy *fp;

	if (!phydev || !phydev->mdio.bus)
		return -EINVAL;

	list_for_each_entry(fp, &fmb->phys, node) {
		if (fp->addr == phydev->mdio.addr) {
			kairos_phy_update(fp);
			return fp->status.link;
		}
	}

	return 0;
}

int kairos_phy_ts_info(struct phy_device *phydev, struct ethtool_ts_info *ti)
{
	ti->so_timestamping =
				SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_TX_SOFTWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RX_SOFTWARE |
				SOF_TIMESTAMPING_SOFTWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;
	ti->phc_index = 2; //AG FIXME: how can I get the PHC clock id?
	
	return 0;
}

int kairos_phy_hwtstamp(struct phy_device *phydev, struct ifreq *ifr)
{
	int result = 0;
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct kairos_phy *fp;

	if (!phydev || !phydev->mdio.bus)
		return -EINVAL;

	list_for_each_entry(fp, &fmb->phys, node) {
		if (fp->addr == phydev->mdio.addr) {
			result = kairos_enable_hwtstamp(fp->ds, ifr);
			break;
		}
	}

	return result;
}

bool kairos_phy_rxtstamp(struct phy_device *dev, struct sk_buff *skb, int type)
{
	return true;
}

void kairos_phy_txtstamp(struct phy_device *dev, struct sk_buff *skb, int type)
{
}


static struct phy_driver kairos_drivers[] = {
	{
		.phy_id = 0x12341234,
		.phy_id_mask = 0xFFFFFFFF,
		.name = "Kairos PHY",
		.features = PHY_BASIC_FEATURES,
		.flags = 0,
		.probe = kairos_phy_probe,
		.config_aneg = &genphy_config_aneg,
		.config_init = &kairos_phy_config_init,
		.aneg_done = &kairos_phy_aneg_done,
		.read_status = &kairos_phy_read_status,
		.ts_info = kairos_phy_ts_info,
		.hwtstamp = kairos_phy_hwtstamp,
		.rxtstamp = kairos_phy_rxtstamp,
		.txtstamp = kairos_phy_txtstamp,
	}
};

static struct mdio_device_id __maybe_unused kairos_tbl[] = {
	{ 0x12341234, 0xFFFFFFFF },
	{ }
};

MODULE_DEVICE_TABLE(mdio, kairos_tbl);

static int __init kairos_mdio_bus_init(void)
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	int ret;

	// register phy driver
	phy_drivers_register(kairos_drivers, 1, THIS_MODULE);

	pdev = platform_device_register_simple("Kairos MDIO bus", 0, NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		goto err_pdev;
	}

	fmb->mii_bus = mdiobus_alloc();
	if (fmb->mii_bus == NULL) {
		ret = -ENOMEM;
		goto err_mdiobus_reg;
	}

	snprintf(fmb->mii_bus->id, MII_BUS_ID_SIZE, "kairos-bus");
	fmb->mii_bus->name = "Kairos MDIO Bus";
	fmb->mii_bus->priv = fmb;
	fmb->mii_bus->parent = &pdev->dev;
	fmb->mii_bus->read = &kairos_mdio_read;
	fmb->mii_bus->write = &kairos_mdio_write;

	ret = mdiobus_register(fmb->mii_bus);
	if (ret)
		goto err_mdiobus_alloc;

	return 0;

err_mdiobus_alloc:
	mdiobus_free(fmb->mii_bus);
err_mdiobus_reg:
	platform_device_unregister(pdev);
err_pdev:
	return ret;
}
module_init(kairos_mdio_bus_init);

static void __exit kairos_mdio_bus_exit(void)
{
	struct kairos_mdio_bus *fmb = &platform_fmb;
	struct kairos_phy *fp, *tmp;

	mdiobus_unregister(fmb->mii_bus);
	mdiobus_free(fmb->mii_bus);
	platform_device_unregister(pdev);

	list_for_each_entry_safe(fp, tmp, &fmb->phys, node) {
		list_del(&fp->node);
		kfree(fp);
	}
	ida_destroy(&phy_fixed_ida);
}
module_exit(kairos_mdio_bus_exit);

MODULE_DESCRIPTION("Kairos MDIO bus (MDIO bus emulation with fixed PHYs)");
MODULE_AUTHOR("Ambrogio Galbusera");
MODULE_LICENSE("GPL");

