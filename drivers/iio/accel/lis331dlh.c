/******************** (C) COPYRIGHT 2016 Exor International ********************
*
* File Name		: lis331dlh.c
*			: Luigi Scagnet (luigi.scagnet@exorint.it)
* Version		: V 2.0.0
* Date			: 2016/10/27
* Description	: LIS331DLH 3D accelerometer sensor API
*
* Based on: lis331dlh_acc.c and lsp331ap.c written by
*	    Carmine Iascone (carmine.iascone@st.com)
*	    Matteo Dameno (matteo.dameno@st.com)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* TODO:
* - Improve interrupts management
******************************************************************************/
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/bitops.h>

#define LIS331DLH_VENDOR                "STM"
#define LIS331DLH_CHIP_ID               "LIS331"

#define G_MAX                           8000

#define WHOAMI_LIS331DLH_ACC            0x32    /*  Expctd content for WAI  */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I                        0x0F    /*  WhoAmI register         */
#define CTRL_REG1                       0x20    /*                          */
#define CTRL_REG2                       0x21    /*                          */
#define CTRL_REG3                       0x22    /*                          */
#define CTRL_REG4                       0x23    /*                          */
#define CTRL_REG5                       0x24    /*                          */
#define INT_CFG1                        0x30    /*  interrupt 1 config      */
#define INT_SRC1                        0x31    /*  interrupt 1 source      */
#define INT_THS1                        0x32    /*  interrupt 1 threshold   */
#define INT_DUR1                        0x33    /*  interrupt 1 duration    */
#define INT_CFG2                        0x34    /*  interrupt 2 config      */
#define INT_SRC2                        0x35    /*  interrupt 2 source      */
#define INT_THS2                        0x36    /*  interrupt 2 threshold   */
#define INT_DUR2                        0x37    /*  interrupt 2 duration    */

#define SENSITIVITY_2G                  1       /*  mg/LSB                  */
#define SENSITIVITY_4G                  2       /*  mg/LSB                  */
#define SENSITIVITY_8G                  4       /*  mg/LSB                  */

#define AXISDATA_REG                    0x28

#define LIS331DLH_ACC_ENABLE_ALL_AXES   0x07
#define LIS331DLH_SELFTEST_EN           0x02
#define LIS331DLH_SELFTEST_DIS          0x00
#define LIS331DLH_SELFTEST_POS          0x00
#define LIS331DLH_SELFTEST_NEG          0x08

/* Accelerometer output data rate  */
#define LIS331DLH_ODR_DELAY_MINIMUM     1       /* 1 mS --> 1Khz minimun deley btw samples  */
#define LIS331DLH_ACC_ODRHALF           0x40    /* 0.5Hz output data rate                   */
#define LIS331DLH_ACC_ODR1              0x60    /* 1Hz output data rate                     */
#define LIS331DLH_ACC_ODR2              0x80    /* 2Hz output data rate                     */
#define LIS331DLH_ACC_ODR5              0xA0    /* 5Hz output data rate                     */
#define LIS331DLH_ACC_ODR10             0xC0    /* 10Hz output data rate                    */
#define LIS331DLH_ACC_ODR50             0x00    /* 50Hz output data rate                    */
#define LIS331DLH_ACC_ODR100            0x08    /* 100Hz output data rate                   */
#define LIS331DLH_ACC_ODR400            0x10    /* 400Hz output data rate                   */
#define LIS331DLH_ACC_ODR1000           0x18    /* 1000Hz output data rate                  */

#define I2C_AUTO_INCREMENT              0x80

/* RESUME STATE INDICES */
#define RES_CTRL_REG1                   0
#define RES_CTRL_REG2                   1
#define RES_CTRL_REG3                   2
#define RES_CTRL_REG4                   3
#define RES_CTRL_REG5                   4
#define RES_REFERENCE                   5

#define RES_INT_CFG1                    6
#define RES_INT_THS1                    7
#define RES_INT_DUR1                    8
#define RES_INT_CFG2                    9
#define RES_INT_THS2                    10
#define RES_INT_DUR2                    11

#define RESUME_ENTRIES                  12
#define REG_ENTRIES                     12
/* end RESUME STATE INDICES */


#define LIS331DLH_DEV_NAME              "lis331dlh"
#define LIS331DLH_PM_NORMAL             0x20
/* Accelerometer Sensor Full Scale */
#define LIS331DLH_ACC_FS_MASK           0x30
#define LIS331DLH_ACC_G_2G              0x00
#define LIS331DLH_ACC_G_4G              0x10
#define LIS331DLH_ACC_G_8G              0x30

#define LIS331DLH_DRDY_INT1             0x02    /* En INT1 'data ready' */
#define LIS331DLH_DRDY_INT2             0x10    /* En INT2 'data ready' */

enum {
    LIS331DLH_LIS331DLH_VENDOR,
    LIS331DLH_NAME,
    LIS331DLH_ODR,
    LIS331DLH_G_RANGE,
    LIS331DLH_DISABLE,
    LIS331DLH_SELFTEST,
};

enum {
    LIS331DLH_INTERRUPT_SRC_NONE,
    LIS331DLH_INTERRUPT_SRC_INT1,
    LIS331DLH_INTERRUPT_SRC_INT2
};

enum acc_state {
    FL_HW_ENABLED,
    FL_HW_INITIALIZED,
    FL_HW_SELF_TEST_ENABLED,
};

enum acc_channel {
    CHANNEL_INDEX_X,
    CHANNEL_INDEX_Y,
    CHANNEL_INDEX_Z,
    CHANNEL_MAX,
};

static struct {
    unsigned int cutoff_ms;
    unsigned int mask;
} lis331dlh_odr_table[] = {
{1,  LIS331DLH_PM_NORMAL    | LIS331DLH_ACC_ODR1000 },  /*  1ms --> ODR 1000 Hz     */
{3,  LIS331DLH_PM_NORMAL 	| LIS331DLH_ACC_ODR400  },  /*  3ms --> ODR  400 Hz     */
{10, LIS331DLH_PM_NORMAL 	| LIS331DLH_ACC_ODR100  },  /* 10ms --> ODR  100 Hz     */
{20, LIS331DLH_PM_NORMAL 	| LIS331DLH_ACC_ODR50   },  /* 20ms --> ODR   50 Hz     */
/* low power settings, max low pass filter cut-off freq */
{100,  LIS331DLH_ACC_ODR10  | LIS331DLH_ACC_ODR1000 },  /* LowPw --> ODR   10 Hz    */
{200,  LIS331DLH_ACC_ODR5   | LIS331DLH_ACC_ODR1000 },  /* LowPw --> ODR    5 Hz    */
{500,  LIS331DLH_ACC_ODR2   | LIS331DLH_ACC_ODR1000 },  /* LowPw --> ODR    2 Hz    */
{1000, LIS331DLH_ACC_ODR1   | LIS331DLH_ACC_ODR1000 },  /* LowPw --> ODR    1 Hz    */
{2000, LIS331DLH_ACC_ODRHALF| LIS331DLH_ACC_ODR1000 },  /* LowPw --> ODR  0.5 Hz    */
};

struct lis331dlh_data {
    struct i2c_client *client;
    struct mutex lock;
    u8 sensitivity;

    unsigned int output_data_rate;
    unsigned long flags;
    u8 reg_cache[REG_ENTRIES];

    int lis_irq;
    int lis_irq_src;
};


static int lis331dlh_i2c_read(struct lis331dlh_data *acc, u8 *buf, int len)
{
    int err;
    struct i2c_msg msgs[] = {
    {
        .addr = acc->client->addr,
                .flags = acc->client->flags & I2C_M_TEN,
                .len = 1,
                .buf = buf,
    },
    {
        .addr = acc->client->addr,
                .flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
                .len = len,
                .buf = buf,
    },
};
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    err = i2c_transfer(acc->client->adapter, msgs, 2);
    if (err != 2) {
        dev_err(&acc->client->dev, "read transfer error = %d\n", err);
        err = -EIO;
    }
    return err;
}

static int lis331dlh_i2c_write(struct lis331dlh_data *acc, u8 *buf, int len)
{
    int err;
    struct i2c_msg msgs[] = {
    {
        .addr = acc->client->addr,
                .flags = acc->client->flags & I2C_M_TEN,
                .len = len + 1,
                .buf = buf,
    },
};

    dev_dbg(&acc->client->dev, "%s \n", __func__);
    err = i2c_transfer(acc->client->adapter, msgs, 1);
    if (err != 1) {
        dev_err(&acc->client->dev, "write transfer error\n");
        err = -EIO;
    }
    return err;
}

static int lis331dlh_hw_init(struct lis331dlh_data *acc)
{
    int err = -1;
    u8 buf[6];
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    buf[0] = WHO_AM_I;
    err = lis331dlh_i2c_read(acc, buf, 1);
    if (err < 0) {
        dev_err(&acc->client->dev,
                "error reading WHO_AM_I: is device available/working?\n");
        goto err_hw_init;
    }

    if (buf[0] != WHOAMI_LIS331DLH_ACC) {
        err = -ENODEV;
        dev_err(&acc->client->dev,
                "device unknown. Expected: 0x%x, Replies: 0x%x\n",
                WHOAMI_LIS331DLH_ACC, buf[0]);
        goto err_hw_init;
    }

    buf[0] = CTRL_REG1;
    buf[1] = acc->reg_cache[RES_CTRL_REG1];
    err = lis331dlh_i2c_write(acc, buf, 1);
    if (err < 0)
        goto err_hw_init;

    buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
    buf[1] = acc->reg_cache[RES_INT_THS1];
    buf[2] = acc->reg_cache[RES_INT_DUR1];
    err = lis331dlh_i2c_write(acc, buf, 2);
    if (err < 0)
        goto err_hw_init;

    buf[0] = INT_CFG1;
    buf[1] = acc->reg_cache[RES_INT_CFG1];
    err = lis331dlh_i2c_write(acc, buf, 1);
    if (err < 0)
        goto err_hw_init;

    buf[0] = (I2C_AUTO_INCREMENT | INT_THS2);
    buf[1] = acc->reg_cache[RES_INT_THS2];
    buf[2] = acc->reg_cache[RES_INT_DUR2];
    err = lis331dlh_i2c_write(acc, buf, 2);
    if (err < 0)
        goto err_hw_init;

    buf[0] = INT_CFG2;
    buf[1] = acc->reg_cache[RES_INT_CFG2];
    err = lis331dlh_i2c_write(acc, buf, 1);
    if (err < 0)
        goto err_hw_init;

    acc->reg_cache[RES_CTRL_REG3] |= LIS331DLH_DRDY_INT1;
    buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
    buf[1] = acc->reg_cache[RES_CTRL_REG2];
    buf[2] = acc->reg_cache[RES_CTRL_REG3];
    buf[3] = acc->reg_cache[RES_CTRL_REG4];
    buf[4] = acc->reg_cache[RES_CTRL_REG5];
    err = lis331dlh_i2c_write(acc, buf, 4);
    if (err < 0)
        goto err_hw_init;

    set_bit(FL_HW_INITIALIZED, &acc->flags);

    return 0;

err_hw_init:
    clear_bit(FL_HW_INITIALIZED, &acc->flags);
    dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0], buf[1], err);
    return err;
}

static int lis331dlh_device_power_off(struct lis331dlh_data *acc)
{
    int err;
    u8 buf[2];
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    acc->reg_cache[RES_CTRL_REG1] &= ~LIS331DLH_PM_NORMAL;

    buf[0] = CTRL_REG1;
    buf[1] = acc->reg_cache[RES_CTRL_REG1];
    err = lis331dlh_i2c_write(acc, buf, 1);

    if (err < 0)
    {
        dev_err(&acc->client->dev, "soft power off failed: %d\n", err);
        return err;
    }

    clear_bit(FL_HW_INITIALIZED, &acc->flags);

    return 0;
}

static int lis331dlh_device_power_on(struct lis331dlh_data *acc)
{
    u8 buf[2];
    int err = -1;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    acc->reg_cache[RES_CTRL_REG1] |= LIS331DLH_PM_NORMAL;

    if (!test_bit(FL_HW_INITIALIZED, &acc->flags))
        err = lis331dlh_hw_init(acc);

    buf[0] = CTRL_REG1;
    buf[1] = acc->reg_cache[RES_CTRL_REG1];
    err = lis331dlh_i2c_write(acc, buf, 1);

    return err;
}

int lis331dlh_update_g_range(struct lis331dlh_data *acc, u8 new_g_range)
{
    int err = -1;

    u8 sensitivity;
    u8 buf[2];
    u8 updated_val;
    u8 init_val;
    u8 new_val;
    u8 mask = LIS331DLH_ACC_FS_MASK;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    switch (new_g_range) {
    case LIS331DLH_ACC_G_2G:
        sensitivity = SENSITIVITY_2G;
        break;
    case LIS331DLH_ACC_G_4G:
        sensitivity = SENSITIVITY_4G;
        break;
    case LIS331DLH_ACC_G_8G:
        sensitivity = SENSITIVITY_8G;
        break;
    default:
        dev_err(&acc->client->dev, "invalid g range requested: %u\n", new_g_range);
        return -EINVAL;
    }

    if ( test_bit(FL_HW_ENABLED, &acc->flags) )
    {
        /* Set configuration register 4, which contains g range setting
    *  NOTE: this is a straight overwrite because this driver does
    *  not use any of the other configuration bits in this
    *  register.  Should this become untrue, we will have to read
    *  out the value and only change the relevant bits --XX----
    *  (marked by X) */
        buf[0] = CTRL_REG4;
        err = lis331dlh_i2c_read(acc, buf, 1);
        if (err < 0)
            goto error;

        init_val = buf[0];
        acc->reg_cache[RES_CTRL_REG4] = init_val;
        new_val = new_g_range;
        updated_val = ((mask & new_val) | ((~mask) & init_val));

        buf[1] = updated_val;
        buf[0] = CTRL_REG4;
        err = lis331dlh_i2c_write(acc, buf, 1);
        if (err < 0)
            goto error;
        acc->reg_cache[RES_CTRL_REG4] = updated_val;
        acc->sensitivity = sensitivity;
    }

    return err;

error:
    dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n", buf[0], buf[1], err);
    return err;
}

int lis331dlh_update_odr(struct lis331dlh_data *acc, int poll_interval_ms)
{
    int err = -1;
    int i;
    u8 buf[2];
    dev_dbg(&acc->client->dev, "%s \n", __func__);

/* Following, looks for the longest possible odr interval scrolling the
 * odr_table vector from the end (shortest interval) backward (longest
 * interval), to support the poll_interval requested by the system.
 * It must be the longest interval lower then the poll interval.
 */
    for (i = ARRAY_SIZE(lis331dlh_odr_table) - 1; i >= 0; i--) {
        if ((lis331dlh_odr_table[i].cutoff_ms <= poll_interval_ms) || (i == 0))
            break;
    }

    acc->output_data_rate = lis331dlh_odr_table[i].cutoff_ms;

    buf[0]  = CTRL_REG1;
    buf[1]  = lis331dlh_odr_table[i].mask;
    buf[1] |= LIS331DLH_ACC_ENABLE_ALL_AXES;

    /* If device is currently enabled, we need to write new configuration out to it */
    if ( test_bit(FL_HW_ENABLED, &acc->flags) )
    {
        err = lis331dlh_i2c_write(acc, buf, 1);
        if (err < 0)
            goto error;
        acc->reg_cache[RES_CTRL_REG1] = buf[1];
    }

    return err;

error:
    dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n", buf[0], buf[1], err);
    return err;
}

static int lis331dlh_enable(struct lis331dlh_data *acc)
{
    int err;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    if ( !test_bit(FL_HW_ENABLED, &acc->flags) ) {
        err = lis331dlh_device_power_on(acc);
        if (err < 0) {
            clear_bit(FL_HW_ENABLED, &acc->flags);
            return err;
        }
        set_bit(FL_HW_ENABLED, &acc->flags);
    }

    return 0;
}

static int lis331dlh_disable(struct lis331dlh_data *acc)
{
    int err;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    if ( !test_bit(FL_HW_ENABLED, &acc->flags) )
        return 0;

    err = lis331dlh_device_power_off(acc);
    clear_bit(FL_HW_ENABLED, &acc->flags);

    return err;
}

static int lis331dlh_acc_register_write(struct lis331dlh_data *acc, u8 *buf, u8 reg_address, u8 new_value)
{
    int err = -1;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    /* Sets configuration register at reg_address
     *  NOTE: this is a straight overwrite  */
    buf[0] = reg_address;
    buf[1] = new_value;
    err = lis331dlh_i2c_write(acc, buf, 1);

    return err;
}

static int lis331dlh_acc_register_read(struct lis331dlh_data *acc, u8 *buf, u8 reg_address)
{
    int err = -1;
    dev_dbg(&acc->client->dev, "%s \n", __func__);
    buf[0] = (reg_address);
    err = lis331dlh_i2c_read(acc, buf, 1);
    return err;
}

static int lis331dlh_acc_register_update(struct lis331dlh_data *acc, u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
    int err = -1;
    u8 init_val;
    u8 updated_val;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    err = lis331dlh_acc_register_read(acc, buf, reg_address);
    if (!(err < 0)) {
        init_val = buf[1];
        updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
        err = lis331dlh_acc_register_write(acc, buf, reg_address, updated_val);
    }
    return err;
}

static int lis331dlh_selftest(struct lis331dlh_data *acc, u8 enable)
{
    int err = -1;
    u8 buf[2]={0x00,0x00};
    char reg_address, mask, bit_values;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    reg_address = CTRL_REG4;
    mask = 0x0A;
    if (enable > 0)
        bit_values = LIS331DLH_SELFTEST_EN | LIS331DLH_SELFTEST_POS;
    else
        bit_values = LIS331DLH_SELFTEST_DIS | LIS331DLH_SELFTEST_POS;

    if ( test_bit(FL_HW_ENABLED, &acc->flags) )
    {
        mutex_lock(&acc->lock);
        err = lis331dlh_acc_register_update(acc, buf, reg_address, mask, bit_values);

        set_bit(FL_HW_SELF_TEST_ENABLED, &acc->flags);

        mutex_unlock(&acc->lock);
        if (err < 0)
            return err;
        acc->reg_cache[RES_CTRL_REG4] = ((mask & bit_values) | ( ~mask & acc->reg_cache[RES_CTRL_REG4]));
    }
    return err;
}

static int lis331dlh_acc_get_acceleration_data(struct lis331dlh_data *acc, int *xyz)
{
    int err = -1;
    /* Data bytes from hardware xL, xH, yL, yH, zL, zH */
    u8 acc_data[6];
    /* x,y,z hardware data */
    s16 hw_d[3] = { 0 };
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
    err = lis331dlh_i2c_read(acc, acc_data, sizeof(acc_data) );
    if (err < 0)
        return err;

    hw_d[CHANNEL_INDEX_X] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
    hw_d[CHANNEL_INDEX_Y] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
    hw_d[CHANNEL_INDEX_Z] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

    dev_dbg(&acc->client->dev, "hw_d[%d] = %d \n", CHANNEL_INDEX_X, hw_d[CHANNEL_INDEX_X] );
    dev_dbg(&acc->client->dev, "hw_d[%d] = %d \n", CHANNEL_INDEX_Y, hw_d[CHANNEL_INDEX_Y] );
    dev_dbg(&acc->client->dev, "hw_d[%d] = %d \n", CHANNEL_INDEX_Z, hw_d[CHANNEL_INDEX_Z] );

    xyz[CHANNEL_INDEX_X] = hw_d[CHANNEL_INDEX_X] * acc->sensitivity;
    xyz[CHANNEL_INDEX_Y] = hw_d[CHANNEL_INDEX_Y] * acc->sensitivity;
    xyz[CHANNEL_INDEX_Z] = hw_d[CHANNEL_INDEX_Z] * acc->sensitivity;

    return err;
}

static int lis331dlh_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{

    struct lis331dlh_data *acc = iio_priv(indio_dev);
    int ret_type;
    int err;
    int xyz[3] = { 0 };
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        dev_dbg(&acc->client->dev, "Read accelleration %d \n",chan->scan_index );
        err = lis331dlh_acc_get_acceleration_data(acc, xyz );
        if (err < 0)
            dev_err(&acc->client->dev, "get_acceleration_data failed\n");

        *val = xyz[chan->scan_index];
        ret_type = IIO_VAL_INT;
        break;

    default:
        *val = 0;
        ret_type = -EINVAL;
        break;
    }
    return ret_type;
}

static ssize_t lis331dlh_get_odr(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val;
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    mutex_lock(&acc->lock);
    val = acc->output_data_rate;
    mutex_unlock(&acc->lock);

    return sprintf(buf, "%d\n", val);
}

static ssize_t lis331dlh_set_odr(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    unsigned long delay_ms = 0;
    unsigned int delay_min = LIS331DLH_ODR_DELAY_MINIMUM;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    if (kstrtoul(buf, 10, &delay_ms))
        return -EINVAL;
    if (!delay_ms)
        return -EINVAL;

    dev_dbg(&acc->client->dev, "delay_ms passed = %ld\n", delay_ms);
    delay_ms = max_t(unsigned int, (unsigned int)delay_ms, delay_min);

    mutex_lock(&acc->lock);
    acc->output_data_rate = delay_ms;
    lis331dlh_update_odr(acc, delay_ms);

    if (delay_ms == LIS331DLH_ODR_DELAY_MINIMUM )
        dev_dbg(&acc->client->dev, "delay limited to 40ms\n");

    mutex_unlock(&acc->lock);

    return size;
}

static ssize_t lis331dlh_get_g_range(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val;
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    mutex_lock(&acc->lock);
    val = acc->sensitivity * 2; // *2 così restituisco i g
    mutex_unlock(&acc->lock);

    return sprintf(buf, "%d\n", val);
}

static ssize_t lis331dlh_set_g_range(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    u8 g_range = 0;
    u8 new_g_range = 0;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    if (kstrtou8(buf, 10, &g_range))
        return -EINVAL;
    if (!g_range)
        return -EINVAL;

    mutex_lock(&acc->lock);
    if( g_range <= 2 )
        new_g_range = LIS331DLH_ACC_G_2G;
    else if( g_range <= 4 )
        new_g_range = LIS331DLH_ACC_G_4G;
    else
        new_g_range = LIS331DLH_ACC_G_8G;

    lis331dlh_update_g_range( acc, new_g_range );
    mutex_unlock(&acc->lock);

    return size;
}

static ssize_t lis331dlh_get_disable(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val;
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    mutex_lock(&acc->lock);
    val = (! test_bit(FL_HW_ENABLED, &acc->flags) );
    mutex_unlock(&acc->lock);
    return sprintf(buf, "%d\n", val);
}

static ssize_t lis331dlh_set_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    u8 disable = 0;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    if (kstrtou8(buf, 10, &disable))
        return -EINVAL;

    if( disable )
        lis331dlh_disable(acc);
    else
        lis331dlh_enable(acc);

    return size;
}

static ssize_t lis331dlh_get_selftest(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val;
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    mutex_lock(&acc->lock);
    val = ( test_bit(FL_HW_SELF_TEST_ENABLED, &acc->flags) );
    mutex_unlock(&acc->lock);
    return sprintf(buf, "%d\n", val);
}

static ssize_t lis331dlh_set_selftest(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    u8 selftest = 0;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    if (kstrtou8(buf, 10, &selftest))
        return -EINVAL;
    lis331dlh_selftest(acc, selftest);

    return size;
}

static ssize_t lis331dlh_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    dev_dbg(&acc->client->dev, "%s \n", __func__);
    return sprintf(buf, "%s\n", LIS331DLH_VENDOR);
}

static ssize_t lis331dlh_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct iio_dev *dev_info = dev_to_iio_dev(dev);
    struct lis331dlh_data *acc = iio_priv(dev_info);
    dev_dbg(&acc->client->dev, "%s \n", __func__);
    return sprintf(buf, "%s\n", LIS331DLH_CHIP_ID);
}

static int lis331dlh_write_event_config(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir,
					   int state)
{
    /* TO BE DONE */
    return 0;
}

static int lis331dlh_read_event_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir)
{
    /* TO BE DONE */
    return 0;
}

static IIO_DEVICE_ATTR(odr,	    0664, lis331dlh_get_odr,	    lis331dlh_set_odr,      LIS331DLH_ODR		);
static IIO_DEVICE_ATTR(vendor,      0644, lis331dlh_vendor_show,    NULL,		    LIS331DLH_LIS331DLH_VENDOR	);
static IIO_DEVICE_ATTR(name,	    0644, lis331dlh_name_show,      NULL,		    LIS331DLH_NAME		);
static IIO_DEVICE_ATTR(g_range,     0644, lis331dlh_get_g_range,    lis331dlh_set_g_range,  LIS331DLH_G_RANGE		);
static IIO_DEVICE_ATTR(disable,     0644, lis331dlh_get_disable,    lis331dlh_set_disable,  LIS331DLH_DISABLE		);
static IIO_DEVICE_ATTR(self_test,   0644, lis331dlh_get_selftest,   lis331dlh_set_selftest, LIS331DLH_SELFTEST		);

static struct attribute *lis331dlh_attributes[] = {
    &iio_dev_attr_vendor.dev_attr.attr,
    &iio_dev_attr_name.dev_attr.attr,
    &iio_dev_attr_odr.dev_attr.attr,
    &iio_dev_attr_g_range.dev_attr.attr,
    &iio_dev_attr_disable.dev_attr.attr,
    &iio_dev_attr_self_test.dev_attr.attr,
    NULL
};

static struct attribute_group lis331dlh_attribute_group = {
    .attrs = lis331dlh_attributes,
};

static const struct iio_chan_spec lis331dlh_channels[] = {
{
    .type = IIO_ACCEL,
    .modified = 1,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    .scan_index = CHANNEL_INDEX_X,
    .channel2 = IIO_MOD_X,
}, {
    .type = IIO_ACCEL,
    .modified = 1,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    .scan_index = CHANNEL_INDEX_Y,
    .channel2 = IIO_MOD_Y,
}, {
    .type = IIO_ACCEL,
    .modified = 1,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    .scan_index = CHANNEL_INDEX_Z,
    .channel2 = IIO_MOD_Z,
}
};

static const struct iio_info lis331dlh_info = {
    .attrs = &lis331dlh_attribute_group,
    .driver_module = THIS_MODULE,
    .read_raw = &lis331dlh_read_raw,
    .read_event_config = &lis331dlh_read_event_config,
    .write_event_config = &lis331dlh_write_event_config,
};

static int lis331dlh_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct lis331dlh_data *acc;
    struct iio_dev *indio_dev;
    int err = -EINVAL;
    int int1_src, int2_src;
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    indio_dev = iio_device_alloc(sizeof(struct lis331dlh_data));
    if (indio_dev == NULL)
        return -ENOMEM;

    acc = iio_priv(indio_dev);

    if (client->dev.of_node) {
        int1_src = irq_of_parse_and_map(client->dev.of_node, 0);
        int2_src = irq_of_parse_and_map(client->dev.of_node, 1);

        if (int1_src) {
            acc->lis_irq = int1_src;
            acc->lis_irq_src = LIS331DLH_INTERRUPT_SRC_INT1;
        } else if (int2_src) {
            acc->lis_irq = int2_src;
            acc->lis_irq_src = LIS331DLH_INTERRUPT_SRC_INT2;
        } else {
            acc->lis_irq_src = LIS331DLH_INTERRUPT_SRC_NONE;
        }
    } else {
        acc->lis_irq = -EINVAL;
    }

    acc->output_data_rate = LIS331DLH_ACC_ODR100;
    acc->client = client;
    mutex_init(&acc->lock);
    indio_dev->dev.parent = &client->dev;
    indio_dev->channels = lis331dlh_channels;
    indio_dev->num_channels = ARRAY_SIZE(lis331dlh_channels);
    indio_dev->info = &lis331dlh_info;
    indio_dev->modes = INDIO_DIRECT_MODE;

    i2c_set_clientdata(client, indio_dev);

    memset(acc->reg_cache, 0, ARRAY_SIZE(acc->reg_cache));

    acc->reg_cache[RES_CTRL_REG1] = LIS331DLH_ACC_ENABLE_ALL_AXES;
    acc->reg_cache[RES_CTRL_REG2] = 0x00;
    acc->reg_cache[RES_CTRL_REG3] = 0x00;
    acc->reg_cache[RES_CTRL_REG4] = 0x00;
    acc->reg_cache[RES_CTRL_REG5] = 0x00;
    acc->reg_cache[RES_REFERENCE] = 0x00;

    acc->reg_cache[RES_INT_CFG1] = 0x00;
    acc->reg_cache[RES_INT_THS1] = 0x00;
    acc->reg_cache[RES_INT_DUR1] = 0x00;
    acc->reg_cache[RES_INT_CFG2] = 0x00;
    acc->reg_cache[RES_INT_THS2] = 0x00;
    acc->reg_cache[RES_INT_DUR2] = 0x00;

    err = lis331dlh_device_power_on(acc);
    if (err < 0) {
        dev_err(&client->dev, "power on failed: %d\n", err);
        goto err_free_data;
    }

    set_bit(FL_HW_ENABLED, &acc->flags);

    err = lis331dlh_update_g_range(acc, LIS331DLH_ACC_G_2G);
    if (err < 0) {
        dev_err(&client->dev, "update_g_range failed\n");
        goto err_power_off;
    }

    err = lis331dlh_update_odr(acc, acc->output_data_rate);
    if (err < 0) {
        dev_err(&client->dev, "update_odr failed\n");
        goto err_power_off;
    }

    //lis331dlh_device_power_off(acc);
    //clear_bit(FL_HW_ENABLED, &acc->flags);

    err = iio_device_register(indio_dev);
    if (err < 0)
        goto err_free_data;

    dev_info(&client->dev, "%s: probed\n", LIS331DLH_DEV_NAME);

    return 0;

err_power_off:
    lis331dlh_device_power_off(acc);
err_free_data:
    iio_device_free(indio_dev);

    return err;
}

static int lis331dlh_remove(struct i2c_client *client)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct lis331dlh_data *acc = iio_priv(indio_dev);
    dev_dbg(&acc->client->dev, "%s \n", __func__);

    //if (acc->lis_irq)
    //	free_irq(acc->lis_irq, indio_dev);

    lis331dlh_device_power_off(acc);

    iio_device_unregister(indio_dev);
    iio_device_free(indio_dev);

    return 0;
}

static const struct i2c_device_id lis331dlh_acc_id[] = {
{ LIS331DLH_DEV_NAME, 0},
{ },
};

MODULE_DEVICE_TABLE(i2c, lis331dlh_acc_id);

#ifdef CONFIG_OF
static const struct of_device_id lis331dlh_of_match[] = {
{ .compatible = "st,lis331dlh" },
{ .compatible = "lis331dlh" },
{ }
};
#endif

static struct i2c_driver lis331dlh_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = LIS331DLH_DEV_NAME,
        .of_match_table = of_match_ptr(lis331dlh_of_match),
    },
    .probe = lis331dlh_probe,
    .remove = lis331dlh_remove,
    .id_table = lis331dlh_acc_id,
};

module_i2c_driver(lis331dlh_driver);

MODULE_DESCRIPTION("Exor Int LIS331DLH accelerometer IIO driver");
MODULE_AUTHOR("Luigi Scagnet, Exorint <luigi.scagnet@exorint.it>");
MODULE_LICENSE("GPL");
