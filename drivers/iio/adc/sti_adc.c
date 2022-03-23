/*
 * STi ADC driver
 * Author: Ajit Pal Singh <ajitpal.singh@st.com>
 *
 * Copyright (C) 2015 STMicroelectronics (R&D) Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>

#define STI_ADC_MAX_CHANNELS	8
#define STI_ADC_MAX_VCHAN	16
#define STI_ADC_VCHAN_MAX_GPIO	4

#define VREF_HIGH_MV_DEFAULT	1300
#define VREF_LOW_MV_DEFAULT	40
/* Some chips have bad calibration data.  ~800 corresponds to about 1100mV */
#define VREF_HIGH_MIN_VALID	800
#define VREF_LOW_MAX_VALID	120

#define STI_ADC_IS_VCHAN(idx)	(idx >= STI_ADC_MAX_CHANNELS)
#define STI_ADC_VCHAN_IDX(idx)	(idx - STI_ADC_MAX_CHANNELS)

/* Internal reference is always 1.4 V */
#define STI_ADC_INTERNAL_VREF	1400

struct sti_adc_info {
	void __iomem *mmio;
	struct completion completion;
	struct regmap *regmap;
	unsigned int vref_external;
	unsigned int channel;
	unsigned int value;
	u32 use_external_ref[STI_ADC_MAX_CHANNELS];

	struct {
		u8 input_chan;
		u8 gpio_count;
		struct gpio_desc *gpios[STI_ADC_VCHAN_MAX_GPIO];
		unsigned long gpio_values;
	} vchan[STI_ADC_MAX_VCHAN];

	struct {
		int scale_val1;
		int scale_val2;
		int offset_val1;
		int offset_val2;
	} cal;
};

#define DRIVER "sti-adc"

/* ADC Registers */
#define STI_ADCCR	0x4 /* Channel Configuration Register */
#define STI_ADCLR	0x0 /* AD Control Logic Register */
#define STI_ADCDRx(x)	(0x20 + (x) * 4) /* ADC Data Register for channel x */

#define CONT		BIT(4)
/* End of Conversion Interrupt */
#define ECVI		BIT(10)
#define SET_NO_WDOG(ch) (0x3 << ((ch) * 2))
#define SET_CHANNEL(ch)	((ch) << 6)
#define ANALOG_CELL_EN	BIT(3)
#define VREF_INTERNAL	BIT(2)
#define ADC_START	BIT(0)
#define DATA_BITS	10

#define STI_ADC_CHAN(idx) {				\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
				BIT(IIO_CHAN_INFO_SCALE) | \
				BIT(IIO_CHAN_INFO_OFFSET), \
	.channel = idx,					\
	.scan_type = {					\
		.sign = 'u',				\
		.storagebits = 16,			\
	},						\
}

#define STI_ADC_VCHAN(vidx)	STI_ADC_CHAN((STI_ADC_MAX_CHANNELS + vidx))

static const struct iio_chan_spec sti_adc_iio_channels[] = {
	STI_ADC_CHAN(0),
	STI_ADC_CHAN(1),
	STI_ADC_CHAN(2),
	STI_ADC_CHAN(3),
	STI_ADC_CHAN(4),
	STI_ADC_CHAN(5),
	STI_ADC_CHAN(6),
	STI_ADC_CHAN(7),

	STI_ADC_VCHAN(0),
	STI_ADC_VCHAN(1),
	STI_ADC_VCHAN(2),
	STI_ADC_VCHAN(3),
	STI_ADC_VCHAN(4),
	STI_ADC_VCHAN(5),
	STI_ADC_VCHAN(6),
	STI_ADC_VCHAN(7),
	STI_ADC_VCHAN(8),
	STI_ADC_VCHAN(9),
	STI_ADC_VCHAN(10),
	STI_ADC_VCHAN(11),
	STI_ADC_VCHAN(12),
	STI_ADC_VCHAN(13),
	STI_ADC_VCHAN(14),
	STI_ADC_VCHAN(15),
};

static irqreturn_t sti_adc_isr(int irq, void *dev_id)
{
	struct sti_adc_info *info = (struct sti_adc_info *)dev_id;

	/* PLAT-3567: Delay reading the channel data to avoid stale data */

	/* Without this delay next ADC data readings are random */
	usleep_range(10, 15);
	/* Clear ECV interrupt */
	regmap_update_bits(info->regmap, STI_ADCLR, BIT(15), BIT(15));
	complete(&info->completion);

	return IRQ_HANDLED;
}

static void sti_set_mux(struct sti_adc_info *info, int base_channel,
			int channel)
{
	int vidx = STI_ADC_VCHAN_IDX(channel);

	gpiod_set_array_value(info->vchan[vidx].gpio_count,
			      info->vchan[vidx].gpios,
			      NULL,
			      &info->vchan[vidx].gpio_values);
}

static int sti_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask)
{
	struct sti_adc_info *info = iio_priv(indio_dev);
	u32 status = 0;
	u32 scale_mv = 0;
	int ret;
	int channel;

	if (STI_ADC_IS_VCHAN(chan->channel)) {
		channel = info->vchan[STI_ADC_VCHAN_IDX(chan->channel)].input_chan;
	} else {
		channel = chan->channel;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (STI_ADC_IS_VCHAN(chan->channel)) {
			sti_set_mux(info, channel, chan->channel);
		}

		reinit_completion(&info->completion);

		info->channel = channel;

		ret = regmap_write(info->regmap, STI_ADCLR, 0);
		if (ret) {
			mutex_unlock(&indio_dev->mlock);
			return -EINVAL;
		}

		/* Enable channel conversion without threshold matching */
		ret = regmap_write(info->regmap, STI_ADCCR,
				   SET_NO_WDOG(channel));
		if (ret) {
			mutex_unlock(&indio_dev->mlock);
			return -EINVAL;
		}

		/* Single mode */
		status &= ~CONT;

		/*
		 * Set the following
		 * 1) End of Conversion Interrupt
		 * 2) Enable Analog Cell
		 * 3) Select channel
		 * 4) Internal/External Reference voltage
		 * 5) ADC start conversion bit
		 */
		status |= ECVI | ANALOG_CELL_EN | SET_CHANNEL(channel);
		if (info->use_external_ref[channel] == 0)
			status |= VREF_INTERNAL;

		ret = regmap_write(info->regmap, STI_ADCLR, status);
		if (ret) {
			mutex_unlock(&indio_dev->mlock);
			return -EINVAL;
		}
		/* Delay needed for the previous write to take effect */
		usleep_range(10, 15);
		/* Start ADC conversion */
		status |= ADC_START;
		ret = regmap_write(info->regmap, STI_ADCLR, status);
		if (ret) {
			mutex_unlock(&indio_dev->mlock);
			return -EINVAL;
		}

		ret = wait_for_completion_timeout(&info->completion,
						  msecs_to_jiffies(1000));
		if (!ret) {
			mutex_unlock(&indio_dev->mlock);
			return -ETIMEDOUT;
		}

		usleep_range(50, 65);
		ret = regmap_read(info->regmap, STI_ADCDRx(info->channel),
				  &info->value);
		if (ret)
			WARN_ON_ONCE(1);

		*val = info->value;

		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (!info->use_external_ref[channel] &&
		    (info->cal.scale_val1 || info->cal.scale_val2)) {
			*val = info->cal.scale_val1;
			*val2 = info->cal.scale_val2;
		} else {
			scale_mv = info->use_external_ref[channel] ?
					info->vref_external : STI_ADC_INTERNAL_VREF;
			scale_mv = (scale_mv * 1000) >> DATA_BITS;
			*val =  scale_mv / 1000000;
			*val2 = (scale_mv % 1000000);
		}

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		if (!info->use_external_ref[channel] &&
		    (info->cal.offset_val1 || info->cal.offset_val2)) {
			*val = info->cal.offset_val1;
			*val2 = info->cal.offset_val2;
		} else {
			*val = 0;
			*val2 = 0;
		}

		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static const struct iio_info sti_adc_iio_info = {
	.read_raw = sti_read_raw,
};

static const struct regmap_config sti_adc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int sti_adc_probe_cal(struct sti_adc_info *info,
			     struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct regmap *cal_regmap;
	struct regmap_field *vref_high, *vref_low;
	int vref_high_val, vref_low_val;
	struct reg_field vref_low_rfield = REG_FIELD(0x30, 0, 9);
	struct reg_field vref_high_rfield = REG_FIELD(0x30, 10, 19);
	s64 uv_tmp;
	s32 vref_fused_low_mv, vref_fused_high_mv;

	if (of_property_read_s32(np, "vref-low-mv", &vref_fused_low_mv) < 0) {
		vref_fused_low_mv = VREF_LOW_MV_DEFAULT;
	}

	if (of_property_read_s32(np, "vref-high-mv", &vref_fused_high_mv) < 0) {
		vref_fused_high_mv = VREF_HIGH_MV_DEFAULT;
	}

	/* See GLLCFF_GP-ADC_AN_V1.0.pdf section 4.3 */

	/* Get Calibration data regmap entry */
	cal_regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg-calib");
	if (IS_ERR(cal_regmap)) {
		dev_err(dev, "no st,syscfg-calib found, fallback to default\n");
		return 0;
	}

	vref_high = devm_regmap_field_alloc(dev, cal_regmap, vref_high_rfield);
	if (IS_ERR(vref_high)) {
		dev_err(dev, "Failed to alloc vref_high regfield\n");
		return PTR_ERR(vref_high);
	}

	ret = regmap_field_read(vref_high, &vref_high_val);
	devm_regmap_field_free(dev, vref_high);
	if (ret) {
		dev_err(dev, "Failed to read vref_high regfield\n");
		return ret;
	}

	vref_low = devm_regmap_field_alloc(dev, cal_regmap, vref_low_rfield);
	if (IS_ERR(vref_low)) {
		dev_err(dev, "Failed to alloc vref_low regfield\n");

		return PTR_ERR(vref_low);
	}

	ret = regmap_field_read(vref_low, &vref_low_val);
	devm_regmap_field_free(dev, vref_low);
	if (ret) {
		dev_err(dev, "Failed to read vref_low regfield\n");
		return ret;
	}

	if (vref_low_val > VREF_LOW_MAX_VALID ||
	    vref_high_val < VREF_HIGH_MIN_VALID) {
		dev_warn(dev, "Invalid calibration values %d-%d\n",
			 vref_low_val, vref_high_val);
		return 0;
	}

	uv_tmp = ((vref_fused_high_mv - vref_fused_low_mv) * 1000LL) /
		 (vref_high_val - vref_low_val);
	info->cal.scale_val1 = uv_tmp / 1000000;
	info->cal.scale_val2 = (uv_tmp % 1000000);

	uv_tmp = ((vref_fused_low_mv * vref_high_val -
			vref_fused_high_mv * vref_low_val) * 1000LL) /
		 (vref_high_val - vref_low_val);
	info->cal.offset_val1 = uv_tmp / 1000000;
	info->cal.offset_val2 = (uv_tmp % 1000000);

	return 0;
}

static int sti_adc_probe_mux(struct sti_adc_info *info,
			     struct platform_device *pdev)
{
	int vchan_count = 0;
	int i, j, ret;
	u32 input_channel, mux_chan_count;
	struct gpio_descs *gpio_descs;
	struct device_node *mux = NULL;

	gpio_descs = devm_gpiod_get_array(&pdev->dev, "mux", GPIOD_OUT_LOW);
	if (IS_ERR_OR_NULL(gpio_descs) || gpio_descs->ndescs == 0)
		return 0;

	if (gpio_descs->ndescs > STI_ADC_VCHAN_MAX_GPIO) {
		dev_err(&pdev->dev, "Invalid MUX GPIO configuration\n");
		return -EINVAL;
	}

	for_each_child_of_node(pdev->dev.of_node, mux) {
		if (of_node_cmp(mux->name, "mux")) {
			continue;
		}

		ret = of_property_read_u32(mux, "input-channel",
					   &input_channel);
		if (ret != 0 || input_channel > STI_ADC_MAX_CHANNELS) {
			dev_err(&pdev->dev,
				"ADC mux missing input-channel property\n");
			return -EINVAL;
		}

		ret = of_property_read_u32(mux, "vchan-count", &mux_chan_count);
		if (ret != 0 ||
		    vchan_count + mux_chan_count > STI_ADC_MAX_VCHAN) {
			dev_err(&pdev->dev,
				"ADC mux missing vchan-count property\n");
			return -EINVAL;
		}

		if (mux_chan_count > (1 << gpio_descs->ndescs)) {
			dev_err(&pdev->dev,
				"MUX contains too many channels (%d) for GPIOs.\n",
				mux_chan_count);
			return -EINVAL;
		}

		for (i = 0; i < mux_chan_count; i++) {
			info->vchan[vchan_count].input_chan = input_channel;
			info->vchan[vchan_count].gpio_count = gpio_descs->ndescs;
			info->vchan[vchan_count].gpio_values = i;

			for (j = 0; j < gpio_descs->ndescs; j++) {
				info->vchan[vchan_count].gpios[j] = gpio_descs->desc[j];
			}

			vchan_count++;
		}

		dev_info(&pdev->dev, "Added %d input mux on channel %d\n",
			 mux_chan_count, input_channel);
	}

	return vchan_count;
}

static int sti_adc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct sti_adc_info *info;
	struct iio_dev *iodev = NULL;
	struct resource *res;
	int ret = 0;
	int irq;
	int i;
	int vchan_count = 0;

	iodev = iio_device_alloc(dev, sizeof(struct sti_adc_info));
	if (!iodev) {
		dev_err(dev, "failed allocating iio device\n");
		return -ENOMEM;
	}

	info = iio_priv(iodev);
	memset(info, 0, sizeof(*info));

	platform_set_drvdata(pdev, iodev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -EINVAL;
		goto out;
	}

	info->mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(info->mmio)) {
		ret = PTR_ERR(info->mmio);
		goto out;
	}

	info->regmap = devm_regmap_init_mmio(dev, info->mmio,
					     &sti_adc_regmap_config);
	if (IS_ERR(info->regmap)) {
		ret = PTR_ERR(info->regmap);
		goto out;
	}

	ret = sti_adc_probe_cal(info, pdev);
	if (ret < 0) {
		goto out;
	}

	/*
	 * Optional vref_external defaults to 0, resulting in internal vref
	 * selection
	 */
	of_property_read_u32(np, "vref-external", &info->vref_external);
	if (info->vref_external) {
		/* Default to using external */
		for (i = 0; i < ARRAY_SIZE(info->use_external_ref); i++) {
			info->use_external_ref[i] = 1;
		}

		ret = of_property_read_u32_array(np, "use-external",
						 info->use_external_ref,
						 ARRAY_SIZE(info->use_external_ref));
		if (ret < 0 && ret != -EINVAL) {
			dev_warn(dev, "Failed to parse use-external property\n");
			ret = -EINVAL;
			goto out;
		}
	}

	vchan_count = sti_adc_probe_mux(info, pdev);
	if (vchan_count < 0) {
		ret = vchan_count;
		goto out;
	}

	init_completion(&info->completion);

	iodev->name = DRIVER;
	iodev->dev.parent = dev;
	iodev->info = &sti_adc_iio_info;
	iodev->modes = INDIO_DIRECT_MODE;
	iodev->channels = sti_adc_iio_channels;
	iodev->num_channels = STI_ADC_MAX_CHANNELS + vchan_count;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed getting interrupt resource\n");
		ret = -EINVAL;
		goto out;
	}

	ret = devm_request_threaded_irq(dev, irq, NULL, sti_adc_isr,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					DRIVER, info);
	if (ret < 0) {
		dev_err(dev, "failed requesting interrupt:%d\n", ret);
		goto out;
	}

	ret = iio_device_register(iodev);
	if (ret)
		goto out;

	dev_info(dev, "STi ADC driver loaded, IRQ %d\n", irq);

	return 0;

out:
	iio_device_free(iodev);
	return ret;
}

static int sti_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *iodev = platform_get_drvdata(pdev);

	iio_device_unregister(iodev);
	platform_set_drvdata(pdev, NULL);
	iio_device_free(iodev);

	return 0;
}

static const struct of_device_id sti_adc_dt_ids[] = {
	{ .compatible = "st,sti-adc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sti_adc_dt_ids);

static struct platform_driver sti_adc_driver = {
	.probe = sti_adc_probe,
	.remove = sti_adc_remove,
	.driver = {
		.name = DRIVER,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sti_adc_dt_ids),
	},
};

module_platform_driver(sti_adc_driver);

MODULE_AUTHOR("Ajit Pal Singh <ajitpal.singh@st.com>");
MODULE_DESCRIPTION("STi ADC driver");
MODULE_LICENSE("GPL");
