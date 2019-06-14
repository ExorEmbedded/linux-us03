/*
 * net/dsa/kairos.c - Driver for Kairos switch chips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <net/dsa.h>
#include "kairos.h"

static int reg_read(struct dsa_switch *ds, int addr, int reg)
{
	struct kairos_priv *priv = ds->priv;

	// read using SPI 
	return mdiobus_read_nested(priv->bus, priv->sw_addr + addr, reg);
}

#define REG_READ(addr, reg)					\
	({							\
		int __ret;					\
								\
		__ret = reg_read(ds, addr, reg);		\
		if (__ret < 0)					\
			return __ret;				\
		__ret;						\
	})


static int reg_write(struct dsa_switch *ds, int addr, int reg, u16 val)
{
	struct mv88e6060_priv *priv = ds->priv;

	return mdiobus_write_nested(priv->bus, priv->sw_addr + addr, reg, val);
}

#define REG_WRITE(addr, reg, val)				\
	({							\
		int __ret;					\
								\
		__ret = reg_write(ds, addr, reg, val);		\
		if (__ret < 0)					\
			return __ret;				\
	})

static const char *kairos_get_name(struct mii_bus *bus, int sw_addr)
{
	int ret;

	ret = mdiobus_read(bus, sw_addr + REG_PORT(0), PORT_SWITCH_ID);
	if (ret >= 0) {
		if (ret == PORT_SWITCH_ID_6060)
			return "Marvell 88E6060 (A0)";
		if (ret == PORT_SWITCH_ID_6060_R1 ||
		    ret == PORT_SWITCH_ID_6060_R2)
			return "Marvell 88E6060 (B0)";
		if ((ret & PORT_SWITCH_ID_6060_MASK) == PORT_SWITCH_ID_6060)
			return "Marvell 88E6060";
	}

	return NULL;
}

static enum dsa_tag_protocol mv88e6060_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_TRAILER;
}

static const char *kairos_drv_probe(struct device *dsa_dev,
				       struct device *host_dev, int sw_addr,
				       void **_priv)
{
	struct mii_bus *bus = dsa_host_dev_to_mii_bus(host_dev);
	struct mv88e6060_priv *priv;
	const char *name;

	name = mv88e6060_get_name(bus, sw_addr);
	if (name) {
		priv = devm_kzalloc(dsa_dev, sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return NULL;
		*_priv = priv;
		priv->bus = bus;
		priv->sw_addr = sw_addr;
	}

	return name;
}

static int kairos_switch_reset(struct dsa_switch *ds)
{
	//@todo
	return 0;
}

static int kairos_setup_global(struct dsa_switch *ds)
{
	//@todo
	return 0;
}

static int kairos_setup_port(struct dsa_switch *ds, int p)
{
	//@todo
	return 0;
}

static int kairos_setup(struct dsa_switch *ds)
{
	int ret;
	int i;

	ret = kairos_switch_reset(ds);
	if (ret < 0)
		return ret;

	/* @@@ initialise atu */
	ret = kairos_setup_global(ds);
	if (ret < 0)
		return ret;

	for (i = 0; i < MV88E6060_PORTS; i++) {
		ret = kairos_setup_port(ds, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int mv88e6060_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	// only read register 1 (status register)
	if (regnum != 0x01)
		return 0xffff;

	//@todo
}

static int
kairos_phy_write(struct dsa_switch *ds, int port, int regnum, u16 val)
{
	// not supported
	return 0xffff;
}

static struct dsa_switch_ops kairos_switch_ops = {
	.get_tag_protocol = kairos_get_tag_protocol,
	.probe		= kairos_drv_probe,
	.setup		= kairos_setup,
	.phy_read	= kairos_phy_read,
	.phy_write	= kairos_phy_write,
};

int kairos_init(struct spi_device* spi)
{
	kairos_spi = spi;

	register_switch_driver(&kairos_switch_ops);
	return 0;
}

void kairos_cleanup(void)
{
	unregister_switch_driver(&kairos_switch_ops);
}

MODULE_AUTHOR("Ambrogio Galbusera");
MODULE_DESCRIPTION("Driver for Kairos TSN switch chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("");
