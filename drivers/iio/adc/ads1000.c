/*
 * Copyright (C) 2018 Exor International S.p.a.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>

#include <linux/iio/iio.h>

struct ads1000 {
	struct i2c_client *i2c;
	int mux_gpio;
	struct mutex lock;
};

#define ADS1000_CFG_REG 0x02
#define ADS1000_OUT_REG 0x00

#define ADS1000_DRIVER_NAME			"ads1000"

static int ads1000_read_channel(struct ads1000 *adc, struct iio_chan_spec const *channel, int *value)
{
    int ret;
    u8 req_channel;

    mutex_lock(&adc->lock);
    req_channel = channel->channel;

    //Select the input line by using the mux_gpio
    if(req_channel == 0)
        gpio_set_value(adc->mux_gpio, 0);
    else
        gpio_set_value(adc->mux_gpio, 1);

    //Wait for the free running ADC to acquire a valid sample
    msleep(15);

    // Get the measured data
    ret = i2c_smbus_read_word_swapped(adc->i2c, ADS1000_OUT_REG);
    if (ret < 0)
        ret = i2c_smbus_read_word_swapped(adc->i2c, ADS1000_OUT_REG);

    if (ret < 0)
        goto err_read_channel;

    if(ret>2047)
        ret=0;

    *value = ret>>3;

err_read_channel:
    mutex_unlock(&adc->lock);
    return ret;
}

static int ads1000_read_raw(struct iio_dev *iio, struct iio_chan_spec const *channel, int *value, int *shift, long mask)
{
    struct ads1000 *adc = iio_priv(iio);
    int err;

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        err = ads1000_read_channel(adc, channel, value);
        if (err < 0)
            return -EINVAL;
        return IIO_VAL_INT;
    default:
        break;
    }
    return -EINVAL;
}

#define ADS1000_CHANNEL(chan) {                     \
    .type = IIO_VOLTAGE,                            \
    .indexed = 1,                                   \
    .channel = (chan),                              \
    .scan_index = (chan),                           \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),   \
}

static const struct iio_chan_spec ads1000_channel[] = {
	ADS1000_CHANNEL(0),
	ADS1000_CHANNEL(1),
};

static const struct iio_info ads1000_info = {
	.read_raw = ads1000_read_raw,
	.driver_module = THIS_MODULE,
};

static int ads1000_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *iio;
	struct ads1000 *adc;
	struct device_node *np = client->dev.of_node;
	int err;
	int ret;

    dev_info( &client->dev, "ADC ads1000 Start probing!\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	iio = devm_iio_device_alloc(&client->dev, sizeof(*adc));
	if (!iio)
		return -ENOMEM;

	adc = iio_priv(iio);
	adc->i2c = client;

	mutex_init(&adc->lock);

	adc->mux_gpio = -EINVAL;
    ret = of_get_named_gpio(np, "mux-gpio", 0);
	if (ret >= 0 && gpio_is_valid(ret))
	{
	  adc->mux_gpio = ret;
	  
	  ret = gpio_request(adc->mux_gpio, "mux-gpio");
	  if(ret < 0)
        return ret;
	  
	  ret = gpio_direction_output(adc->mux_gpio, 0);
	  if(ret < 0)
	    return ret;
    } else {
        return -EPROBE_DEFER;
    }

	iio->dev.parent = &client->dev;
	iio->name =  ADS1000_DRIVER_NAME; //dev_name(&client->dev);
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &ads1000_info;

	iio->channels = ads1000_channel;
	iio->num_channels = 2;

	err = iio_device_register(iio);
	if (err < 0)
	  return err;

	i2c_set_clientdata(client, iio);
	
	i2c_smbus_write_byte_data(adc->i2c, ADS1000_CFG_REG, 0x00);

    dev_info( &client->dev, "ADC ads1000 Probed!\n");

	return 0;
}

static int ads1000_remove(struct i2c_client *client)
{
	struct iio_dev *iio = i2c_get_clientdata(client);

	iio_device_unregister(iio);

	return 0;
}

static const struct i2c_device_id ads1000_id[] = {
	{ "ads1000", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads1000_id);

#ifdef CONFIG_OF
static const struct of_device_id ads1000_of_match[] = {
	{ .compatible = "exor,ads1000" },
	{ }
};
MODULE_DEVICE_TABLE(of, ads1000_of_match);
#endif

static struct i2c_driver ads1000_driver = {
	.driver = {
		.name = ADS1000_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ads1000_of_match),
	},
	.probe = ads1000_probe,
	.remove = ads1000_remove,
	.id_table = ads1000_id,
};
module_i2c_driver(ads1000_driver);

MODULE_AUTHOR("Giovanni Pavoni, Exor International S.p.a.");
MODULE_DESCRIPTION("Driver for ads1000 ADC, used for analog pots on X5 board");
MODULE_LICENSE("GPL v2");
