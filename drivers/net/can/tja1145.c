/*
 *  tja1145.c - Linux kernel module for uSxx working hours counters
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/tja1145.h>
#include <linux/memory.h>


struct tja1145_data {
	struct memory_accessor macc;
	struct spi_device	*spi;
};

static const struct spi_device_id tja1145_id[] = {
	{ "tja1145x", 0 },
	{ "tja1145xFD", 1 },
	{ }
};

/*
 * tja1145_driver_version --> guard function
 */
void tja1145_driver_version(void)
{
    printk(KERN_INFO "Tja1145x driver version %d.%d \n",  MAJOR_DRIVER_VERS ,  MINOR_DRIVER_VERS );
}
EXPORT_SYMBOL(tja1145_driver_version);

/*
 * tja1145_read_single_reg
 */
static int tja1145_read_single_reg(struct spi_device *spi, u8 reg)
{
    u8  read_reg = ((reg << 1) | READ_ONLY);
    return spi_w8r8(spi, read_reg);
}

/*
 * tja1145_write_single_reg
 */
static int tja1145_write_single_reg(struct spi_device *spi, u8 reg, u8 data)
{
    int     status;
    u16  write_reg_data = ((data << 8) | (reg << 1));
    status = spi_write(spi, &write_reg_data, 2 );
    return status;
}

/*
 * tja1145_macc_read
 */
static ssize_t tja1145_macc_read(struct memory_accessor *macc,        char *buf, off_t offset, size_t count)
{
    struct tja1145_data* data = container_of(macc, struct tja1145_data, macc);
    struct spi_device*   spi  = data->spi;
    dev_info(&spi->dev, "%s Function not implemented\n", __func__ );
    return 0;
}

/*
 * tja1145_macc_write
 */
static ssize_t tja1145_macc_write(struct memory_accessor *macc, const char *buf, off_t offset, size_t count)
{
    struct tja1145_data* data = container_of(macc, struct tja1145_data, macc);
    struct spi_device*   spi  = data->spi;
    tja1145_write_single_reg(spi, (u8)offset, (u8)buf[0]);
    return 0;
}

/*
 * tja1145_get_chip_id
 */
static int tja1145_get_chip_id(struct spi_device *spi)
{
    int chip_id = tja1145_read_single_reg(spi, REG_IDENTIFICATION);

    switch(chip_id)
    {
        case WHOIAM_VALUE:
            dev_info(&spi->dev, "found TJA1145x\n" );
            break;

        case WHOIAM_VALUE_FD:
            dev_info(&spi->dev, "found TJA1145xFD\n" );
            break;
        default:
            dev_err(&spi->dev, "chip id %x not know\n", chip_id );
            return -ENODEV;
    }

    return chip_id;
}

/*
 * tja1145_set_normal_mode

static int tja1145_set_normal_mode(struct spi_device *spi)
{
    return tja1145_write_single_reg(spi, REG_MODE_CONTROL, NORMAL_MODE );
}
 */
/*
 * tja1145_set_normal_mode

static int tja1145_set_sleep_mode(struct spi_device *spi)
{
    return tja1145_write_single_reg(spi, REG_MODE_CONTROL, SLEEP_MODE );
}
 */

/*
 * tja1145_probe
 */
static int tja1145_probe(struct spi_device *spi)
{
	int status = 0;
	int tmp;
    struct device_node	*np     = spi->dev.of_node;
    struct tja1145_data	*data   = NULL;

    dev_info(&spi->dev, "Start probing \n");

    if (!spi->dev.platform_data)
    {
        if (!np){
			dev_err(&spi->dev, "Probing Error: no chip in DT\n");
			return -ENODEV;
		}
	} else {
		dev_err(&spi->dev, "Probing Error: not implemented platform_data\n");
		return -ENODEV;
    }

    tmp = tja1145_get_chip_id(spi);
    if(tmp < 0)
    {
		dev_err(&spi->dev, "Probing Error: chip not compatible\n");
		return -ENODEV;
    }

    data = devm_kzalloc(&spi->dev, sizeof(struct tja1145_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

    data->spi = spi_dev_get(spi);
	spi_set_drvdata(spi, data);
    data->macc.write = tja1145_macc_write;
    data->macc.read  = tja1145_macc_read;

    dev_info(&spi->dev, "Probed\n");
	return status;
}

/*
 * tja1145_remove
 */
static int tja1145_remove(struct spi_device *spi)
{
    dev_info(&spi->dev, "%s \n", __func__ );
	return 0;
}

static int tja1145_command(struct spi_device *client, unsigned int cmd, void *arg)
{
    struct tja1145_data	*data;
    const struct memory_accessor **maccp;

    /* only supporting a single command */
    if (cmd != SPI_TJA1145_GET_MEMORY_ACCESSOR)
        return -ENOTSUPP;

    /* rudimentary check */
    if (arg == NULL)
        return -EINVAL;

    data = spi_get_drvdata(client);

    maccp = arg;
    *maccp = &data->macc;

	return 0;
}

/*
 * tja1145_dt_ids
 */
static const struct of_device_id tja1145_dt_ids[] = {
	{ .compatible = "nxp,tja1145" },
	{},
};
MODULE_DEVICE_TABLE(of, tja1145_dt_ids);

/*
 * tja1145_spi_driver
 */
static struct spi_driver tja1145_spi_driver = {
	.driver = {
		.name =		"tja1145",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(tja1145_dt_ids),
	},
	.probe      = tja1145_probe,
	.remove     = tja1145_remove,
	.id_table	= tja1145_id,
	.command    = tja1145_command,
};

module_spi_driver(tja1145_spi_driver);


MODULE_AUTHOR("Luigi Scagnet <luigi.scagnet@exorint.it>, ");
MODULE_DESCRIPTION("NXP TJA1145 CAN transceiver driver");
MODULE_LICENSE("GPL");
