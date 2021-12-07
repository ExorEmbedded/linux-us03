/*
 * drivers/net/phy/realtek.c
 *
 * Driver for Realtek PHYs
 *
 * Author: Johnson Leung <r58129@freescale.com>
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/phy.h>
#include <linux/module.h>

#define RTL821x_PHYSR		0x11
#define RTL821x_PHYSR_DUPLEX	0x2000
#define RTL821x_PHYSR_SPEED	0xc000
#define RTL821x_INER		0x12
#define RTL821x_INER_INIT	0x6400
#define RTL821x_INSR		0x13
#define RTL8211E_INER_LINK_STATUS 0x400

#define RTL8211F_INER_LINK_STATUS 0x0010
#define RTL8211F_INSR		0x1d
#define RTL8211F_PAGE_SELECT	0x1f
#define RTL8211F_TX_DELAY	0x100
#define RTL8211F_RX_DELAY	0x08

MODULE_DESCRIPTION("Realtek PHY driver");
MODULE_AUTHOR("Johnson Leung");
MODULE_LICENSE("GPL");

static int rtl821x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL821x_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8211b_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL821x_INER_INIT);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int rtl8211e_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL821x_INER,
				RTL8211E_INER_LINK_STATUS);
	else
		err = phy_write(phydev, RTL821x_INER, 0);

	return err;
}

static int generic_config_init(struct phy_device *phydev)
{
	int val;
	u32 features;

	/* For now, I'll claim that the generic driver supports
	 * all possible port types
	 */
	features = (SUPPORTED_TP | SUPPORTED_MII
			| SUPPORTED_AUI | SUPPORTED_FIBRE |
			SUPPORTED_BNC);

	/* Do we support autonegotiation? */
	val = phy_read(phydev, MII_BMSR);
	if (val < 0)
		return val;

	if (val & BMSR_ANEGCAPABLE)
		features |= SUPPORTED_Autoneg;

	if (val & BMSR_100FULL)
		features |= SUPPORTED_100baseT_Full;
	if (val & BMSR_100HALF)
		features |= SUPPORTED_100baseT_Half;
	if (val & BMSR_10FULL)
		features |= SUPPORTED_10baseT_Full;
	if (val & BMSR_10HALF)
		features |= SUPPORTED_10baseT_Half;

	if (val & BMSR_ESTATEN) {
		val = phy_read(phydev, MII_ESTATUS);
		if (val < 0)
			return val;

		if (val & ESTATUS_1000_TFULL)
			features |= SUPPORTED_1000baseT_Full;
		if (val & ESTATUS_1000_THALF)
			features |= SUPPORTED_1000baseT_Half;
	}

	phydev->supported &= features;
	phydev->advertising &= features;

	return 0;
}

static int rtl8211f_config_init(struct phy_device *phydev)
{
	int ret;
	u16 reg;

	ret = generic_config_init(phydev);
	if (ret < 0)
		return ret;
	
	/* Enable RGMII TX delay */
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xd08);
	reg = phy_read(phydev, 0x11);
	reg |= RTL8211F_TX_DELAY;
	phy_write(phydev, 0x11, reg);
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);

	/* Enable RGMII RX delay */
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0xd08);
	reg = phy_read(phydev, 0x15);
	reg |= RTL8211F_RX_DELAY;
	phy_write(phydev, 0x15, reg);
	phy_write(phydev, RTL8211F_PAGE_SELECT, 0x0);
	
	return 0;
}

static struct phy_driver rtl8201cp_driver = {
	.phy_id         = 0x00008201,
	.name           = "RTL8201CP Ethernet",
	.phy_id_mask    = 0x0000ffff,
	.features       = PHY_BASIC_FEATURES,
	.flags          = PHY_HAS_INTERRUPT,
	.config_aneg    = &genphy_config_aneg,
	.read_status    = &genphy_read_status,
	.driver         = { .owner = THIS_MODULE,},
};

/* RTL8211B */
static struct phy_driver rtl8211b_driver = {
	.phy_id		= 0x001cc912,
	.name		= "RTL8211B Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.ack_interrupt	= &rtl821x_ack_interrupt,
	.config_intr	= &rtl8211b_config_intr,
	.driver		= { .owner = THIS_MODULE,},
};

/* RTL8211E */
static struct phy_driver rtl8211e_driver = {
	.phy_id		= 0x001cc915,
	.name		= "RTL8211E Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.ack_interrupt	= &rtl821x_ack_interrupt,
	.config_intr	= &rtl8211e_config_intr,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
	.driver		= { .owner = THIS_MODULE,},
};

static struct phy_driver rtl8211f_driver = {
	.phy_id		= 0x001cc916,
	.name		= "RTL8211F Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= 0,
	.config_aneg	= &genphy_config_aneg,
	.config_init	= &rtl8211f_config_init,
	.read_status	= &genphy_read_status,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
	.driver		= { .owner = THIS_MODULE,},
};
	
static int __init realtek_init(void)
{
	int ret;

	ret = phy_driver_register(&rtl8201cp_driver);
	if (ret < 0)
		return -ENODEV;
	ret = phy_driver_register(&rtl8211b_driver);
	if (ret < 0)
		return -ENODEV;
	ret = phy_driver_register(&rtl8211e_driver);
	if (ret < 0)
		return -ENODEV;
	return phy_driver_register(&rtl8211f_driver);
}

static void __exit realtek_exit(void)
{
	phy_driver_unregister(&rtl8211b_driver);
	phy_driver_unregister(&rtl8211e_driver);
	phy_driver_unregister(&rtl8211f_driver);
}

module_init(realtek_init);
module_exit(realtek_exit);

static struct mdio_device_id __maybe_unused realtek_tbl[] = {
	{ 0x001cc912, 0x001fffff },
	{ 0x001cc914, 0x001fffff },
	{ 0x001cc915, 0x001fffff },
	{ 0x001cc916, 0x001fffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, realtek_tbl);
