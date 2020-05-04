/*
* dual_dual_rotary_encoder.c
*
*  Written by: Luigi Scagnet, Exor S.p.a.
*  Copyright (c) 2020 Exor S.p.a.
*  Based on rotary_encoder
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
*
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/property.h>

#define DRV_NAME "dual-rotary-encoder"

enum dual_rotary_encoder_encoding {
	ROTENC_GRAY,
	ROTENC_BINARY,
};

struct dual_rotary_encoder {
	struct input_dev *input_abs;
	struct input_dev *input_rel;

	struct mutex access_mutex;

	u32 steps;
	u32 axis;
	bool relative_axis;
	bool rollover;
	enum dual_rotary_encoder_encoding encoding;

	uint64_t pos;

	struct gpio_descs *gpios;

	unsigned int *irq;

	bool armed;
	signed char dir;	/* 1 - clockwise, -1 - CCW */

	unsigned int last_stable;
};


static ssize_t dual_rotary_encoder_read(struct file *filp, struct kobject *kobj,
										struct bin_attribute *bin_attr,
										char *buf, loff_t off, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct platform_device *pdev = to_platform_device(dev);
	struct dual_rotary_encoder *encoder = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s %llX  count: %d offset %lld\n", __func__, encoder->pos, count, off);
	memcpy(buf, &encoder->pos, sizeof(encoder->pos));
	return count;
}

static struct bin_attribute abs_enc_file_attr = {
	.attr = {
		.name = "raw_absdata",
		.mode = S_IRUGO,
	},
	.size = sizeof(uint64_t),	//Size of encoder.raw_absdata var
	.read = dual_rotary_encoder_read,
};

static unsigned int dual_rotary_encoder_get_state(struct dual_rotary_encoder *encoder)
{
	int i;
	unsigned int ret = 0;

	for (i = 0; i < encoder->gpios->ndescs; ++i) {
		int val = gpiod_get_value_cansleep(encoder->gpios->desc[i]);

		/* convert from gray encoding to normal */
		if (encoder->encoding == ROTENC_GRAY && ret & 1)
			val = !val;

		ret = ret << 1 | val;
	}

	return ret & 3;
}

static void dual_rotary_encoder_report_event(struct dual_rotary_encoder *encoder)
{

	unsigned int pos = encoder->pos;

	if (encoder->dir < 0) {
		/* turning counter-clockwise */
		if (encoder->rollover)
			pos += encoder->steps;
		if (pos)
			pos--;
	} else {
		/* turning clockwise */
		if (encoder->rollover || pos < encoder->steps)
			pos++;
	}

	if (encoder->rollover)
		pos %= encoder->steps;

	encoder->pos = pos;
	input_report_abs(encoder->input_abs, encoder->axis, encoder->pos);
	input_sync(encoder->input_abs);

	input_report_rel(encoder->input_rel, encoder->axis, encoder->dir);
	input_sync(encoder->input_rel);
}

static irqreturn_t dual_rotary_encoder_irq(int irq, void *dev_id)
{
	struct dual_rotary_encoder *encoder = dev_id;
	unsigned int state;

	mutex_lock(&encoder->access_mutex);

	state = dual_rotary_encoder_get_state(encoder);

	switch (state) {
	case 0x0:
		if (encoder->armed) {
			dual_rotary_encoder_report_event(encoder);
			encoder->armed = false;
		}
		break;

	case 0x1:
	case 0x3:
		if (encoder->armed)
			encoder->dir = 2 - state;
		break;

	case 0x2:
		encoder->armed = true;
		break;
	}

	mutex_unlock(&encoder->access_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t dual_rotary_encoder_half_period_irq(int irq, void *dev_id)
{
	struct dual_rotary_encoder *encoder = dev_id;
	unsigned int state;

	mutex_lock(&encoder->access_mutex);

	state = dual_rotary_encoder_get_state(encoder);

	if (state & 1) {
		encoder->dir = ((encoder->last_stable - state + 1) % 4) - 1;
	} else {
		if (state != encoder->last_stable) {
			dual_rotary_encoder_report_event(encoder);
			encoder->last_stable = state;
		}
	}

	mutex_unlock(&encoder->access_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t dual_rotary_encoder_quarter_period_irq(int irq, void *dev_id)
{
	struct dual_rotary_encoder *encoder = dev_id;
	unsigned int state;

	mutex_lock(&encoder->access_mutex);

	state = dual_rotary_encoder_get_state(encoder);

	if ((encoder->last_stable + 1) % 4 == state)
		encoder->dir = 1;
	else if (encoder->last_stable == (state + 1) % 4)
		encoder->dir = -1;
	else
		goto out;

	dual_rotary_encoder_report_event(encoder);

out:
	encoder->last_stable = state;
	mutex_unlock(&encoder->access_mutex);

	return IRQ_HANDLED;
}

static int dual_rotary_encoder_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dual_rotary_encoder *encoder;
	struct input_dev *input_abs;
	struct input_dev *input_rel;
	irq_handler_t handler;
	u32 steps_per_period;
	unsigned int i;
	int err;

	dev_info(dev, "%s start probing \n", __func__);
	encoder = devm_kzalloc(dev, sizeof(struct dual_rotary_encoder), GFP_KERNEL);
	if (!encoder)
		return -ENOMEM;

	mutex_init(&encoder->access_mutex);

	device_property_read_u32(dev, "rotary-encoder,steps", &encoder->steps);

	err = device_property_read_u32(dev, "rotary-encoder,steps-per-period",
				       &steps_per_period);
	if (err) {
		/*
		 * The 'half-period' property has been deprecated, you must
		 * use 'steps-per-period' and set an appropriate value, but
		 * we still need to parse it to maintain compatibility. If
		 * neither property is present we fall back to the one step
		 * per period behavior.
		 */
		steps_per_period = device_property_read_bool(dev,
					"rotary-encoder,half-period") ? 2 : 1;
	}

	encoder->rollover =
		device_property_read_bool(dev, "rotary-encoder,rollover");

	if (!device_property_present(dev, "rotary-encoder,encoding") ||
	    !device_property_match_string(dev, "rotary-encoder,encoding",
					  "gray")) {
		dev_info(dev, "gray");
		encoder->encoding = ROTENC_GRAY;
	} else if (!device_property_match_string(dev, "rotary-encoder,encoding",
						 "binary")) {
		dev_info(dev, "binary");
		encoder->encoding = ROTENC_BINARY;
	} else {
		dev_err(dev, "unknown encoding setting\n");
		return -EINVAL;
	}

	device_property_read_u32(dev, "linux,axis", &encoder->axis);
	encoder->relative_axis =
		device_property_read_bool(dev, "rotary-encoder,relative-axis");

	encoder->gpios = devm_gpiod_get_array(dev, NULL, GPIOD_IN);
	if (IS_ERR(encoder->gpios)) {
		dev_err(dev, "unable to get gpios\n");
		return PTR_ERR(encoder->gpios);
	}
	if (encoder->gpios->ndescs < 2) {
		dev_err(dev, "not enough gpios found\n");
		return -EINVAL;
	}

	input_abs = devm_input_allocate_device(dev);
	input_rel = devm_input_allocate_device(dev);
	if (!input_abs || !input_rel)
		return -ENOMEM;

	encoder->input_abs = input_abs;
	input_abs->name = "input_abs";
	input_abs->id.bustype = BUS_HOST;
	input_abs->dev.parent = dev;
	input_set_abs_params(input_abs, encoder->axis, 0, encoder->steps, 0, 1);

	encoder->input_rel = input_rel;
	input_rel->name = "input_rel";
	input_rel->id.bustype = BUS_HOST;
	input_rel->dev.parent = dev;
	input_set_capability(input_rel, EV_REL, encoder->axis);

	switch (steps_per_period >> (encoder->gpios->ndescs - 2)) {
	case 4:
		handler = &dual_rotary_encoder_quarter_period_irq;
		encoder->last_stable = dual_rotary_encoder_get_state(encoder);
		break;
	case 2:
		handler = &dual_rotary_encoder_half_period_irq;
		encoder->last_stable = dual_rotary_encoder_get_state(encoder);
		break;
	case 1:
		handler = &dual_rotary_encoder_irq;
		break;
	default:
		dev_err(dev, "'%d' is not a valid steps-per-period value\n",
			steps_per_period);
		return -EINVAL;
	}

	encoder->irq =
		devm_kzalloc(dev,
			     sizeof(*encoder->irq) * encoder->gpios->ndescs,
			     GFP_KERNEL);
	if (!encoder->irq)
		return -ENOMEM;

	for (i = 0; i < encoder->gpios->ndescs; ++i) {
		encoder->irq[i] = gpiod_to_irq(encoder->gpios->desc[i]);

		err = devm_request_threaded_irq(dev, encoder->irq[i],
				NULL, handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
				DRV_NAME, encoder);
		if (err) {
			dev_err(dev, "unable to request IRQ %d (gpio#%d)\n",
				encoder->irq[i], i);
			return err;
		}
	}

	err = input_register_device(input_abs);
	if (err) {
		dev_err(dev, "failed to register input_abs device\n");
		return err;
	}


	err = input_register_device(input_rel);
	if (err) {
		dev_err(dev, "failed to register input_rel device\n");
		return err;
	}

	device_init_wakeup(dev,
			   device_property_read_bool(dev, "wakeup-source"));

	platform_set_drvdata(pdev, encoder);

	/* create the sysfs key file */
	return sysfs_create_bin_file(&pdev->dev.kobj, &abs_enc_file_attr);
}

static int dual_rotary_encoder_remove(struct platform_device *pdev)
{
	sysfs_remove_bin_file(&pdev->dev.kobj, &abs_enc_file_attr);
	return 0;
}

static int __maybe_unused dual_rotary_encoder_suspend(struct device *dev)
{
	struct dual_rotary_encoder *encoder = dev_get_drvdata(dev);
	unsigned int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < encoder->gpios->ndescs; ++i)
			enable_irq_wake(encoder->irq[i]);
	}

	return 0;
}

static int __maybe_unused dual_rotary_encoder_resume(struct device *dev)
{
	struct dual_rotary_encoder *encoder = dev_get_drvdata(dev);
	unsigned int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < encoder->gpios->ndescs; ++i)
			disable_irq_wake(encoder->irq[i]);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(dual_rotary_encoder_pm_ops,
			 dual_rotary_encoder_suspend, dual_rotary_encoder_resume);

#ifdef CONFIG_OF
static const struct of_device_id dual_rotary_encoder_of_match[] = {
	{ .compatible = "dual-rotary-encoder", },
	{ },
};
MODULE_DEVICE_TABLE(of, dual_rotary_encoder_of_match);
#endif

static struct platform_driver dual_rotary_encoder_driver = {
	.probe		= dual_rotary_encoder_probe,
	.remove		= dual_rotary_encoder_remove,
	.driver		= {
		.name	= DRV_NAME,
		.pm	= &dual_rotary_encoder_pm_ops,
		.of_match_table = of_match_ptr(dual_rotary_encoder_of_match),
	}
};
module_platform_driver(dual_rotary_encoder_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DESCRIPTION("Dual Event GPIO rotary encoder driver");
MODULE_AUTHOR("Luigi Scagnet, Exor S.p.a. <luigi.scagnet@exorint.com>");
MODULE_AUTHOR("Daniel Mack <daniel@caiaq.de>, Johan Hovold");
MODULE_LICENSE("GPL v2");
