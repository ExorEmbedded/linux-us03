/*
 *  tja1145.h - Linux kernel module for uSxx working hours counters
 *
 *  Written by: Luigi Scagnet, Exor S.p.a.
 *  Copyright (c) 2017 Exor S.p.a.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef SPI_TJA1145_H
#define SPI_TJA1145_H

#include <linux/types.h>
#include <linux/memory.h>
#include <linux/err.h>
#include <linux/spi/spi.h>

#define     MAJOR_DRIVER_VERS   1
#define     MINOR_DRIVER_VERS   0
/***********************************/
/****** TJA1145 SPI Registers ******/
/***********************************/
/* SPI REG ACCESS */
#define READ_ONLY 			        (0x01)
#define WRITE_ONLY			        (0x00)

/* REG ADDR */
#define REG_MODE_CONTROL	      (0x01)
#define REG_IDENTIFICATION      (0x7E)

/* REG VALUE            */
/*  REG_MODE_CONTROL    */
#define SLEEP_MODE			        (0x01)
#define NORMAL_MODE			        (0x07)
/*  REG_IDENTIFICATION  */
#define WHOIAM_VALUE            (0x70)
#define WHOIAM_VALUE_FD		      (0x74)


/* interface commands */
#define SPI_TJA1145_GET_MEMORY_ACCESSOR	1

/*
 * Guard function, used to create a dependency btw flexcan and tja
 */
void tja1145_driver_version(void);

/*
 * The method called in the client is
 *
 * int (*command)(struct spi_device *client, unsigned int cmd, void *arg);
 *
 * A single command is supported, which returns a pointer to the memory
 * accessor already available, but which was only accessible via platform
 * callbacks. We can't use platform callbacks anymore for device tree
 * platforms, hence this interface.
 *
 */
static inline struct memory_accessor *
spi_tja1145_get_memory_accessor(struct spi_device *client)
{
	int ret;
	struct memory_accessor *macc;
	struct spi_driver *driver;

	/* verify that the i2c client's driver has a command method */
	if (!client || !client->dev.driver)
		return ERR_PTR(-ENOTSUPP);

	driver = to_spi_driver(client->dev.driver);
	if(!driver->command)
		return ERR_PTR(-ENOTSUPP);

	macc = NULL;
	ret = driver->command(client, SPI_TJA1145_GET_MEMORY_ACCESSOR, &macc);
	if (ret != 0)
		return ERR_PTR(ret);

	if (macc == NULL)
		return ERR_PTR(-ENOTSUPP);

	return macc;
}

#endif
