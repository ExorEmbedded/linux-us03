/*
 *  disp-pa20 - Driver for SPI init of the pa20 LCD monochrome display
 *
 *  Copyright (c) 2019 Exor Int.
 *  Written by: Giovanni Pavoni, Exor Int.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 or
 *  later as publishhed by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of.h>

#define DEVICE_NAME "disp-pa20"

/* Data structure holding the dvi plugin instance params
 */
struct disp_pa20_data {
	struct device       *dev;
	struct spi_device   *spi;               //Pointer to spi device
};

/* Do a SPI W access
 */
static void spi_w(struct spi_device *spi, u8* buf, u8 len)
{
  struct spi_transfer t;
  struct spi_message  m;
  unsigned char rx_buf[64];
  
  spi_message_init(&m);
  memset(&t, 0, sizeof t);
  t.tx_buf = buf;
  t.rx_buf = rx_buf;
  t.len = len;
  spi_message_add_tail(&t, &m);
  spi_sync(spi, &m);
}

/*
 * Do a SPI write access to the specified register
 */
static void disp_pa20_reg(struct spi_device *spi, unsigned short reg, unsigned short dat)
{
  unsigned char buf[3];
  
  //Register access
  buf[0] = 0x70;
  buf[1] = (unsigned char)((reg >> 8) & 0xff);
  buf[2] = (unsigned char)((reg >> 0) & 0xff);
  spi_w(spi, buf, 3);
  
  //Data access
  buf[0] = 0x72;
  buf[1] = (unsigned char)((dat >> 8) & 0xff);
  buf[2] = (unsigned char)((dat >> 0) & 0xff);
  spi_w(spi, buf, 3);
}

/*
 * Initialize the pa20 display
 */
static void disp_pa20_init(struct spi_device *spi)
{
  disp_pa20_reg(spi, 0x01, 0x7300);
  disp_pa20_reg(spi, 0x02, 0x0200);
  disp_pa20_reg(spi, 0x03, 0x6464);
  disp_pa20_reg(spi, 0x04, 0x04c7);
  disp_pa20_reg(spi, 0x05, 0xf844);
  disp_pa20_reg(spi, 0x08, 0x06ff);
  disp_pa20_reg(spi, 0x0a, 0x4008);
  disp_pa20_reg(spi, 0x0b, 0xd400);
  disp_pa20_reg(spi, 0x0d, 0x422c);
  disp_pa20_reg(spi, 0x0e, 0x2d00);
  disp_pa20_reg(spi, 0x0f, 0x0000);
  disp_pa20_reg(spi, 0x16, 0x9f80);
  disp_pa20_reg(spi, 0x17, 0x2212);
  disp_pa20_reg(spi, 0x1e, 0x0078);
  disp_pa20_reg(spi, 0x30, 0x0103);
  disp_pa20_reg(spi, 0x31, 0x0407);
  disp_pa20_reg(spi, 0x32, 0x0102);
  disp_pa20_reg(spi, 0x33, 0x0102);
  disp_pa20_reg(spi, 0x34, 0x0506);
  disp_pa20_reg(spi, 0x35, 0x0003);
  disp_pa20_reg(spi, 0x36, 0x0406);
  disp_pa20_reg(spi, 0x37, 0x0201);
  disp_pa20_reg(spi, 0x3a, 0x1800);
  disp_pa20_reg(spi, 0x3b, 0x1800);
}

/*
 * Driver probe function
 */
static int disp_pa20_probe(struct spi_device *spi)
{
	struct disp_pa20_data *pdata;
	
	/* Configure the SPI bus */
	spi->mode = (SPI_MODE_0);
	spi->bits_per_word = 8;
	spi_setup(spi);
	
	pdata = devm_kzalloc(&spi->dev, sizeof(struct disp_pa20_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	
	pdata->spi = spi;
	spi_set_drvdata(spi, pdata);
	
	disp_pa20_init(spi);
	
	printk("%s : SUCCESS\n", __func__);
	return 0;
}

static int disp_pa20_remove(struct spi_device *spi)
{
	return 0;
}

/*-------------------------------------------------------------*/
static const struct of_device_id disp_pa20_of_match[] = {
	{ .compatible = "exor,disp-pa20", },
	{ }
};
MODULE_DEVICE_TABLE(of, disp_pa20_of_match);

static struct spi_driver disp_pa20_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = disp_pa20_of_match,
		
	},

	.probe = disp_pa20_probe,
	.remove = disp_pa20_remove,
};

module_spi_driver(disp_pa20_driver);

MODULE_AUTHOR("Giovanni Pavoni , Exor Int.");
MODULE_DESCRIPTION("Display PA20 spi driver");
MODULE_LICENSE("GPL");
