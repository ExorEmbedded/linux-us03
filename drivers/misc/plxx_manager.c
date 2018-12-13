/*
 *  plxx_manager.c - Linux plugins manager driver
 *
 *  Written by: Giovanni Pavoni, Exor S.p.a.
 *  Copyright (c) 2016 Exor S.p.a.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/kthread.h>
#include <linux/i2c/I2CSeeprom.h>

#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/plxx_manager.h>
#include <linux/nvmem-consumer.h>

#define SEE_FUNCAREA_NBITS (SEE_FUNCAREALEN * 8)
#define FULLEEPROMSIZE (256)

static DEFINE_MUTEX(plxx_lock);

struct plxx_data 
{
    struct nvmem_device *    seeprom_device;
    struct nvmem_device *    ioexp_device;
    u32                      index;                   // Plugin index
    u32                      sel_gpio;                // Gpio index to select the plugin
    u32                      installed;               // Indicates if the plugin is physically installed
    u8                       eeprom[FULLEEPROMSIZE];  // Image of the I2C SEEPROM contents of the plugin
    bool                     f_updated;               // Flag indicating if datas for the current plugin were still taken
};

static int UpdatePluginData(struct plxx_data *data);

/* External function to sen commands to the plxx_manager driver from other drivers.
 * Commands are actually implemented only for setting FullDuplex or HalfDuplex mode
 * of the RS422/485 plugin modules
 */
int plxx_manager_sendcmd(struct platform_device *pdev, unsigned int cmd)
{
    struct plxx_data *data = dev_get_drvdata(&pdev->dev);

    if(data == NULL)
        return -EPROBE_DEFER;

    /* only supporting the 2 following commands */
    if ((cmd != RS422_485_IF_SETFD) &&  (cmd != RS422_485_IF_SETHD))
        return -ENOTSUPP;

    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }

    gpio_set_value(data->sel_gpio, 1);                      //Select the plugin I2C bus
    msleep(1);

    // If we have a RS485/422 plugin module (bit #3 in func. area) perform the command
    if(data->eeprom[SEE_FUNCT_AREA_OFF] & (0x01 << RS422_485_IF_FLAG))
    {
	u8 tmp;
	tmp=0x0e;
	nvmem_device_write(data->ioexp_device, 3, sizeof(u8), (void*)&tmp);
	msleep(1);
	if(cmd == RS422_485_IF_SETFD)
	    tmp = 0x00;
	else
	    tmp = 0x01;

	printk("plxx_manager_sendcmd cmd=0x%x\n",cmd);
	nvmem_device_write(data->ioexp_device, 1, sizeof(u8), (void*)&tmp);
    }

    gpio_set_value(data->sel_gpio, 0);                      //Deselect the plugin I2C bus
    msleep(1);
    mutex_unlock(&plxx_lock);
    return 0;
}
EXPORT_SYMBOL(plxx_manager_sendcmd);

/*
 * static helper function for parsing the DTB tree
 */
#ifdef CONFIG_OF
static int plxx_parse_dt(struct device *dev, struct plxx_data *data)
{
    struct device_node* node = dev->of_node;
    int                 ret;
    struct nvmem_device*  nvmem;

    /* Parse the DT to find the I2C SEEPROM bindings*/
    nvmem = nvmem_device_get(dev, "eeprom");
    if (IS_ERR(nvmem))
    {
      dev_err(dev, "nvmem_device_get:eeprom err=%ld\n", PTR_ERR(nvmem));
      return PTR_ERR(nvmem);
    }
    data->seeprom_device = nvmem;

    /* Parse the DT to find the I2C SEEPROM bindings*/
    nvmem = nvmem_device_get(dev, "ioexp");
    if (IS_ERR(nvmem))
    {
      dev_err(dev, "nvmem_device_get:ioexp err=%ld\n", PTR_ERR(nvmem));
      return PTR_ERR(nvmem);
    }
    data->ioexp_device = nvmem;
    
    /* Get the plugin index */
    ret = of_property_read_u32(node, "index", &data->index);
    if (ret)
        return ret;

    /* Get the sel_gpio */
    data->sel_gpio = of_get_named_gpio(node, "sel-gpio", 0);
    if (gpio_is_valid(data->sel_gpio))
    {
        ret = gpio_request(data->sel_gpio, "plxx_sel-gpio");
        if (ret < 0)
            return -EPROBE_DEFER;
        ret = gpio_direction_output(data->sel_gpio,0);
        if (ret < 0)
            return ret;

        gpio_set_value(data->sel_gpio, 0);
    }
    else
        return -EPROBE_DEFER;

    return 0;
}
#else
static int hrs_parse_dt(struct device *dev, struct hrs_data *data)
{
    return -ENODEV;
}
#endif

/*
 * sysfs interface
 */
static ssize_t show_installed(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 tmp;
    struct plxx_data *data = dev_get_drvdata(dev);
    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }
    tmp = data->installed;
    mutex_unlock(&plxx_lock);
    return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_hwcode(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 tmp;
    struct plxx_data *data = dev_get_drvdata(dev);
    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }
    tmp = data->eeprom[SEE_CODE_OFF];
    mutex_unlock(&plxx_lock);
    return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_hwsubcode(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 tmp;
    struct plxx_data *data = dev_get_drvdata(dev);
    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }
    tmp = data->eeprom[SEE_SUBCODE_OFF];
    mutex_unlock(&plxx_lock);
    return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_fpgacode(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 tmp;
    struct plxx_data *data = dev_get_drvdata(dev);
    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }
    tmp = data->eeprom[SEE_XILCODE_OFF];
    mutex_unlock(&plxx_lock);
    return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_fpgasubcode(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 tmp;
    struct plxx_data *data = dev_get_drvdata(dev);
    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }
    tmp = data->eeprom[SEE_XILSUBCODE_OFF];
    mutex_unlock(&plxx_lock);
    return sprintf(buf, "%d\n",(u32)tmp);
}

static ssize_t show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
    u8 tmp[SEE_MODULENAMELEN+1];
    struct plxx_data *data = dev_get_drvdata(dev);
    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }
    memcpy(tmp, &data->eeprom[SEE_NAME_OFF], SEE_MODULENAMELEN);
    tmp[SEE_MODULENAMELEN]=0;
    mutex_unlock(&plxx_lock);
    return sprintf(buf, "%s\n",tmp);
}

/* The function bit area of the plugin is seen as a RO binary file, where the i-th byte represents the binary status (0|1) of the i-th bit of the 
 * function bit area
 */
static ssize_t func_bit_area_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
    int bitIndex;
    int i;

    if(count == 0)
        return count;

    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }

    if(count > (SEE_FUNCAREA_NBITS - off))
        count = SEE_FUNCAREA_NBITS - off;

    for(i=0; i < count; i++)
    {
        bitIndex = off + i;
        buf[i] = (data->eeprom[SEE_FUNCT_AREA_OFF + bitIndex/8] & (1 << (bitIndex & 7)))? 1 : 0;
    }

    mutex_unlock(&plxx_lock);
    return count;
}

/* Show the eeprom contents of the plugin as a raw file
 */
static ssize_t eeprom_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
    int i;

    if(count == 0)
        return count;

    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }

    if(count > (FULLEEPROMSIZE - off))
        count = FULLEEPROMSIZE - off;

    for(i=0; i < count; i++)
    {
        buf[i] = data->eeprom[i+off];
    }

    mutex_unlock(&plxx_lock);
    return count;
}

/* Write the eeprom contents of the plugin as a raw file
 */
static ssize_t eeprom_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
    int i;

    if(count == 0)
        return count;

    if(off >= FULLEEPROMSIZE)
        return 0;

    mutex_lock(&plxx_lock);
    if(!data->f_updated)
    {
        UpdatePluginData(data);
        data->f_updated = true;
    }

    if(count > (FULLEEPROMSIZE - off))
        count = FULLEEPROMSIZE - off;

    gpio_set_value(data->sel_gpio, 1);                      //Select the plugin I2C bus
    msleep(1);

    i = nvmem_device_write(data->seeprom_device, off, count, (void*)buf);
    msleep(1);

    nvmem_device_read(data->seeprom_device, off, count, (void*)&data->eeprom[off]);
    msleep(1);

    gpio_set_value(data->sel_gpio, 0);                      //Select the plugin I2C bus
    mutex_unlock(&plxx_lock);
    return i;
}

/* Show the registers of the i2cexpander of the plugin as a raw file
 */
static ssize_t i2cexpander_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));

    if(count == 0)
        return count;

    mutex_lock(&plxx_lock);

    if(count > (FULLEEPROMSIZE - off))
        count = FULLEEPROMSIZE - off;

    gpio_set_value(data->sel_gpio, 1);                      //Select the plugin I2C bus
    usleep_range(1, 2);

    nvmem_device_read(data->ioexp_device, off, count, (void*)buf);
    usleep_range(1, 2);

    gpio_set_value(data->sel_gpio, 0);                      //Select the plugin I2C bus
    mutex_unlock(&plxx_lock);
    return count;
}

/* Write the registers of the i2cexpander of the plugin as a raw file
 */
static ssize_t i2cexpander_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
    struct plxx_data *data = dev_get_drvdata(container_of(kobj, struct device, kobj));
    int i;

    if(count == 0)
        return count;

    if(off >= FULLEEPROMSIZE)
        return 0;

    mutex_lock(&plxx_lock);

    if(count > (FULLEEPROMSIZE - off))
        count = FULLEEPROMSIZE - off;

    gpio_set_value(data->sel_gpio, 1);                      //Select the plugin I2C bus
    usleep_range(1, 2);

    i = nvmem_device_write(data->ioexp_device, off, count, (void*)buf);
    usleep_range(1, 2);

    gpio_set_value(data->sel_gpio, 0);                      //Select the plugin I2C bus
    mutex_unlock(&plxx_lock);
    return i;
}

static DEVICE_ATTR(installed, S_IRUGO, show_installed, NULL);
static DEVICE_ATTR(hwcode, S_IRUGO, show_hwcode, NULL);
static DEVICE_ATTR(hwsubcode, S_IRUGO, show_hwsubcode, NULL);
static DEVICE_ATTR(fpgacode, S_IRUGO, show_fpgacode, NULL);
static DEVICE_ATTR(fpgasubcode, S_IRUGO, show_fpgasubcode, NULL);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static BIN_ATTR_RO(func_bit_area, SEE_FUNCAREA_NBITS);
static BIN_ATTR_RW(eeprom, FULLEEPROMSIZE);
static BIN_ATTR_RW(i2cexpander, FULLEEPROMSIZE);

static struct attribute *plxx_sysfs_attributes[] = {
    &dev_attr_installed.attr,
    &dev_attr_name.attr,
    &dev_attr_hwcode.attr,
    &dev_attr_hwsubcode.attr,
    &dev_attr_fpgacode.attr,
    &dev_attr_fpgasubcode.attr,
    NULL
};

static struct bin_attribute *plxx_sysfs_bin_attrs[] = {
    &bin_attr_func_bit_area,
    &bin_attr_eeprom,
    &bin_attr_i2cexpander,
    NULL,
};

static const struct attribute_group plxx_attr_group = {
    .attrs = plxx_sysfs_attributes,    //sysfs single entries
    .bin_attrs = plxx_sysfs_bin_attrs, //sysfs binary file for the function bit area
};

/*
 * Read and validate the whole plugin SEEPROM and update the plugin data structure accordingly
 */
static int UpdatePluginData(struct plxx_data *data)
{
    int ret = 0;
    int n;

    gpio_set_value(data->sel_gpio, 1);                      //Select the plugin I2C bus
    msleep(1);
    n = nvmem_device_read(data->seeprom_device, 0, FULLEEPROMSIZE, (void*)data->eeprom);

    if(n < FULLEEPROMSIZE)
    {
        ret = -1;                                        //Plugin not found
        memset(data->eeprom, 0, FULLEEPROMSIZE);
        data->installed = 0;
    }
    else
        data->installed = 1;

    //If plugin found, now check if the seeprom contents are valid
    if(ret==0)
    {
        u8 chksum = 1;
        int i;
        bool eeprom_isvalid = true;

        for(i = SEE_CHKSM_START; i < SEE_FACTORYSIZE; i++)
            chksum += data->eeprom[i];
        chksum -= 0xaa;

        if(chksum != data->eeprom[SEE_CHKSM_START - 1])       //Checksum check
            eeprom_isvalid = false;

        if(data->eeprom[0] != 0xaa)                           //Header check
            eeprom_isvalid = false;

        if(data->eeprom[1] != 0x55)                           //Header check
            eeprom_isvalid = false;

        if(data->eeprom[2] != 0x03)                           //Header check
            eeprom_isvalid = false;

        if(eeprom_isvalid == false)
        {                                                     //The plugin was detected bus has invalid SEEPROM contents ...
            memset(data->eeprom, 0, SEE_FACTORYSIZE);
        }

    }
    gpio_set_value(data->sel_gpio, 0);                      //Deselect the plugin I2C bus
    return ret;
}

/*
 * Probe and remove functions
 */
static int plxx_probe(struct platform_device *pdev)
{
    int res = 0;
    struct plxx_data *data;

    data = kzalloc(sizeof(struct plxx_data), GFP_KERNEL);
    if (data == NULL)
    {
        dev_err(&pdev->dev, "Memory allocation failed\n");
        return -ENOMEM;
    }
    memset(data, 0, sizeof(struct plxx_data));
    dev_set_drvdata(&pdev->dev, data);

    res = plxx_parse_dt(&pdev->dev, data);
    if (res)
    {
        dev_err(&pdev->dev, "Could not find valid DT data.\n");
        goto plxx_error1;
    }

    data->f_updated = false;

    // Create sysfs entry
    res = sysfs_create_group(&pdev->dev.kobj, &plxx_attr_group);
    if (res)
    {
        dev_err(&pdev->dev, "device create file failed\n");
        goto plxx_error1;
    }

    return res;
plxx_error1:
    kfree(data);
    return res;
}

static int plxx_remove(struct platform_device *pdev)
{
    struct plxx_data *data = dev_get_drvdata(&pdev->dev);

    sysfs_remove_group(&pdev->dev.kobj, &plxx_attr_group);
    nvmem_device_put(data->seeprom_device);
    nvmem_device_put(data->ioexp_device);
    kfree(data);
    return 0;
}

/*
 * Driver instantiation
 */
#ifdef CONFIG_OF
static struct of_device_id plxx_of_match[] = {
{ .compatible = "exor,plxx_manager" },
{ }
};

MODULE_DEVICE_TABLE(of, plxx_of_match);
#endif

static struct platform_driver plxx_driver = {
    .driver		= {
        .name		= "plxx_manager",
        .owner		= THIS_MODULE,
        .of_match_table	= of_match_ptr(plxx_of_match),
    },
    .probe		= plxx_probe,
    .remove		= plxx_remove,
};

module_platform_driver(plxx_driver);

MODULE_DESCRIPTION("plxx_manager");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:plxx_manager");
