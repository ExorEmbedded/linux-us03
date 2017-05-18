#ifndef __PHY_FIXED_H
#define __PHY_FIXED_H

struct fixed_phy_status {
	int link;
	int speed;
	int duplex;
	int pause;
	int asym_pause;
};

#ifdef CONFIG_FIXED_PHY
extern int fixed_phy_add(unsigned int irq, int phy_id,
			 struct fixed_phy_status *status);

//Just to fix for build errors...
static inline struct phy_device *fixed_phy_register(unsigned int irq,
						struct fixed_phy_status *status,
						struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}


#else
static inline int fixed_phy_add(unsigned int irq, int phy_id,
				struct fixed_phy_status *status)
{
	return -ENODEV;
}
#endif /* CONFIG_FIXED_PHY */

/*
 * This function issued only by fixed_phy-aware drivers, no need
 * protect it with #ifdef
 */
extern int fixed_phy_set_link_update(struct phy_device *phydev,
			int (*link_update)(struct net_device *,
					   struct fixed_phy_status *));

#endif /* __PHY_FIXED_H */
