#ifndef __LINUX_I2C_TSC2004_H
#define __LINUX_I2C_TSC2004_H

/* linux/i2c/tsc2004.h */

struct tsc2004_platform_data {
	u16	model;				/* 2004. */
	u16	x_plate_ohms;

	unsigned gpio;
	unsigned reset_gpio;

	int	(*get_pendown_state)(struct device *);
	void	(*clear_penirq)(void);		/* If needed, clear 2nd level
						   interrupt source */
	int	(*init_platform_hw)(void);
	void	(*exit_platform_hw)(void);
	unsigned int irq_flags;
};

#endif
