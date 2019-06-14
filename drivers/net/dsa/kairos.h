#ifndef __KAIROS_H
#define __KAIROS_H

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

struct kairos_priv {
	/* MDIO bus and address on bus to use. When in single chip
	 * mode, address is 0, and the switch uses multiple addresses
	 * on the bus.  When in multi-chip mode, the switch uses a
	 * single address which contains two registers used for
	 * indirect access to more registers.
	 */
	struct spi_device *spi;
};

#endif