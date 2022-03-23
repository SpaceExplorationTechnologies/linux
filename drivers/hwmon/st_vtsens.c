/*
 * ST VTSens thermal sensor driver
 *
 * Copyright (C) STMicroelectronics 2016
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/st_vtsens_kernel.h>
#include <linux/delay.h>

static struct st_vtsens *g_vtsens;

/*
 * 3ms was tested to be the longest timeout in regular operation.  PLAT-3614
 * tracks followup investigation.
 */
#define VTSENS_TIMEOUT_USEC		10000

/*
 * Helper to set the reg_field - id_ version not available
 * within the standard regmap.h so macro defined here instead
 */
#define REG_FIELD_ID(_reg, _lsb, _msb, _id_size, _id_offset) {	\
			.reg = _reg,				\
			.lsb = _lsb,				\
			.msb = _msb,				\
			.id_size = _id_size,			\
			.id_offset = _id_offset,		\
}

enum st_vtsens_sensors_regfield_ids {
	S_DATA = 0,
	S_THR0_VAL,
	S_THR0_POL,
	S_THR0_ENB,
	S_THR1_VAL,
	S_THR1_POL,
	S_THR1_ENB,
	S_IRQ_CLEAR,
	S_IRQ_MASK,
	S_IRQ_UNMASK,

	S_MAX_REGFIELDS
};

#define DCONF_ENABLE BIT(0)
#define DCONF_MODE_MASK      0x06
#define DCONF_MODE_SHIFT     1
#define DCONF_MODE_VOLTAGE   2
#define DCONF_CLUSTER_MASK   0x70
#define DCONF_CLUSTER_SHIFT  4

#define VTSENS_SENSOR_COUNT	2

/**
 * Description of thermal vtsens driver compatible data.
 *
 * @reg_fields:		Pointer to the regfields array for a sensor.
 */
#define CALIB_DATA_NB	4
#define CALIB_CURVATURE	0
#define CALIB_DCORRECT	1
#define CALIB_PIVOT	2
#define CALIB_TRIMVBE	3

#define MAX_R_SENSOR	7
#define SENSOR_OFF	0
#define SENSOR_THER	1
#define SENSOR_VOLT	2
struct st_vtsens_compat_data {
	unsigned int dconf_reg;
	unsigned int sconf_reg;
	const struct reg_field calib_reg[CALIB_DATA_NB];
	const struct reg_field calib_data[CALIB_DATA_NB];
	unsigned int calib_defs[CALIB_DATA_NB];
	const struct reg_field clockdiv;
	const struct reg_field sensor_reg_fields[S_MAX_REGFIELDS];
	const struct attribute_group **attr_groups;
	const unsigned int r_sensors_mode[MAX_R_SENSOR];
};

#define MODE_POLLING_ONLY	0
#define MODE_POLLING_AND_IRQ	1

struct st_vtsens {
	struct platform_device *pdev;
	struct device *hwmon_dev;
	const struct st_vtsens_compat_data *cdata;
	struct clk *clk;
	struct regmap *regmap;
	struct regmap *regmap_ca;
	struct regmap_field *fld[S_MAX_REGFIELDS];
	int irq[MAX_R_SENSOR + 1];
	unsigned int mode;
	struct mutex lock;
};

/*
 * Temperature value is calculated as follow:
 * T = (DATA[13:0] * A) / 2^14 - B
 * A = 731, B = 273.4
 */
#define FACTOR_A	7310
#define FACTOR_B	2734

static ssize_t show_mode(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct st_vtsens *vtsens = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 (vtsens->mode == MODE_POLLING_ONLY) ?
					"Polling only" : "Polling & IRQ");
}

/* Temperature value can be calculated as follow:
 * T = (DATA[13:0] * A) / 2^14 - B
 * A = 731, B = 273.4
 */
#define TEMP_FACTOR_A	7310
#define TEMP_FACTOR_B	2734

/* Voltage value can be computed as follow:
 * V = (DATA[13:0] * A) / 2^12
 * A = 1.226
 */
#define VOLT_FACTOR_A	1226

/* Convert the data into:
 * temperature: (degree C) multiplied by 10
 * voltage: (milli V)
 */
static inline int data_to_value(unsigned int data, unsigned int mode)
{
	if (mode == SENSOR_THER)
		return (((data * TEMP_FACTOR_A) >> 14) - TEMP_FACTOR_B);
	else
		return ((data * VOLT_FACTOR_A) >> 12);
}

/* Convert temperature (multiply by 10) into register value */
static inline unsigned int temp_to_data(int temp)
{
	/* Produce a register value from a temperature * 10 */
	return ((temp + TEMP_FACTOR_B) << 14) / TEMP_FACTOR_A;
}

#define DATAOVERF BIT(18)
#define DATAREADY BIT(17)
#define MODE_VOLT BIT(16)

#define DATA_MASK (0x3fff)
static int read_sensor(struct st_vtsens *vtsens, unsigned int nr,
		       unsigned int mode, int *value)
{
	struct device *dev = &vtsens->pdev->dev;
	struct regmap *regmap = vtsens->regmap;
	int ret = 0;
	int ret2 = 0;
	unsigned int data;
	unsigned int conf;
	unsigned long timeout;

	mutex_lock(&vtsens->lock);

	/* Read voltage sensors in manual mode */
	if (mode == SENSOR_VOLT) {
		ret = regmap_read(regmap, vtsens->cdata->dconf_reg, &conf);
		if (ret) {
			dev_err(dev, "failed to read dynamic configuration\n");
			goto out;
		}

		conf &= ~(DCONF_MODE_MASK | DCONF_CLUSTER_MASK);
		conf |= (nr << DCONF_CLUSTER_SHIFT);
		conf |= (DCONF_MODE_VOLTAGE << DCONF_MODE_SHIFT);
		conf |= DCONF_ENABLE;

		ret = regmap_write(regmap, vtsens->cdata->dconf_reg, conf);
		if (ret) {
			dev_err(dev, "failed to write dynamic configuration\n");
			goto out;
		}
	}

	timeout = jiffies + usecs_to_jiffies(VTSENS_TIMEOUT_USEC);

	/* Wait for the DATAREADY bit and jump to out after timeout */
	while (true) {
		unsigned long start_time = jiffies;

		ret = regmap_fields_read(vtsens->fld[S_DATA], nr, &data);
		if (ret) {
			dev_err(dev, "Failed to read data\n");
			goto out;
		}

		/*
		 * Check that vtsens is in the right mode.  If in voltage mode
		 * verify command has completed.
		 */
		if ((mode == SENSOR_THER) && !(data & MODE_VOLT)) {
			break;
		} else if (mode == SENSOR_VOLT && (data & MODE_VOLT)) {
			if (data & DATAREADY)
				break;
		}

		/* Check data is available */
		if (time_after(start_time, timeout)) {
			dev_err(dev, "Timed out waiting for data\n");
			ret = -EIO;
			/*
			 * Need to set back to continuous mode or subsequent
			 * temp readings will fail.
			 */
			break;
		}

		usleep_range(100, 200);
	};

	if (data & DATAOVERF) {
		dev_err(dev, "Overflowed data received\n");
		ret = -EIO;
	}

	if (!ret) {
		/* Convert the data into value */
		*value = data_to_value(data & DATA_MASK, mode);
	}

	/* Disable manual mode in case of voltage sensor */
	if (mode == SENSOR_VOLT) {
		ret2 = regmap_read(regmap, vtsens->cdata->dconf_reg, &conf);
		if (ret2) {
			dev_err(dev, "failed to read dynamic configuration\n");
			goto out;
		}

		conf &= ~(DCONF_ENABLE | DCONF_MODE_MASK | DCONF_CLUSTER_MASK);

		ret2 = regmap_write(regmap, vtsens->cdata->dconf_reg, conf);
		if (ret2) {
			dev_err(dev, "failed to write dynamic configuration\n");
			goto out;
		}
	}

out:
	mutex_unlock(&vtsens->lock);
	return ret ? ret : ret2;
}

int vtsens_read_temperature(unsigned int sensor, int *temperature)
{
	int ret, temp;

	if (!g_vtsens)
		return -EPROBE_DEFER;

	if (sensor > VTSENS_SENSOR_COUNT)
		return -EINVAL;

	ret = read_sensor(g_vtsens, sensor, SENSOR_THER, &temp);
	if (ret)
		return ret;

	*temperature = temp / 10;

	return 0;
}
EXPORT_SYMBOL(vtsens_read_temperature);

static ssize_t show_temp(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct st_vtsens *vtsens = dev_get_drvdata(dev);
	unsigned int nr = to_sensor_dev_attr_2(attr)->nr;
	int ret, temp;

	ret = read_sensor(vtsens, nr, SENSOR_THER, &temp);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%d\n", temp * 10);
}

static ssize_t show_temp_thr(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct st_vtsens *vtsens = dev_get_drvdata(dev);
	unsigned int nr = to_sensor_dev_attr_2(attr)->nr;
	unsigned int index = to_sensor_dev_attr_2(attr)->index;
	int ret;
	unsigned int data, enabled;
	int temp;

	if (vtsens->mode == MODE_POLLING_ONLY)
		return -EINVAL;

	mutex_lock(&vtsens->lock);

	ret = regmap_fields_read(vtsens->fld[index == 1 ?
					     S_THR0_VAL : S_THR1_VAL],
				 nr, &data);
	if (ret)
		goto out;

	ret = regmap_fields_read(vtsens->fld[index == 1 ?
					     S_THR0_ENB : S_THR1_ENB],
				 nr, &enabled);
	if (ret)
		goto out;

	/* Convert the data into temperature (degree C * 10) */
	temp = data_to_value(data, SENSOR_THER);

	if (enabled) {
		ret = scnprintf(buf, PAGE_SIZE, "%d (Enabled)\n", temp * 10);
	} else {
		ret = scnprintf(buf, PAGE_SIZE, "(Disabled)\n");
	}

out:
	mutex_unlock(&vtsens->lock);
	return ret;
}

static ssize_t set_temp_thr(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct st_vtsens *vtsens = dev_get_drvdata(dev);
	int nr = to_sensor_dev_attr_2(attr)->nr;
	int index = to_sensor_dev_attr_2(attr)->index;
	int ret, temp;
	unsigned int data;

	if (vtsens->mode == MODE_POLLING_ONLY)
		return -EINVAL;

	ret = kstrtoint(buf, 10, &temp);
	if (ret < 0)
		return ret;

	/* Compute the register value */
	data = temp_to_data(temp / 10);

	mutex_lock(&vtsens->lock);

	/* Disable the threshold first */
	ret = regmap_fields_write(vtsens->fld[index == 1 ?
					      S_THR0_ENB : S_THR1_ENB],
				  nr, 0);
	if (ret)
		goto out;

	ret = regmap_fields_write(vtsens->fld[index == 1 ?
					      S_THR0_VAL : S_THR1_VAL],
				  nr, data);
	if (ret)
		goto out;

	/* Enable the threshold */
	ret = regmap_fields_write(vtsens->fld[index == 1 ?
					      S_THR0_ENB : S_THR1_ENB],
				 nr, 1);
	if (ret)
		goto out;

	/* Clear the interrupt of the corresponding sensor */
	ret = regmap_write(vtsens->regmap,
		vtsens->cdata->sensor_reg_fields[S_IRQ_CLEAR].reg +
		(vtsens->cdata->sensor_reg_fields[S_IRQ_CLEAR].id_offset * nr),
		1);
	if (ret) {
		dev_err(dev, "Failed to clear IRQ of sensor %d\n", nr);
		goto out;
	}

	/* Unmask the interrupt of the corresponding sensor */
	ret = regmap_write(vtsens->regmap,
		vtsens->cdata->sensor_reg_fields[S_IRQ_UNMASK].reg +
		(vtsens->cdata->sensor_reg_fields[S_IRQ_UNMASK].id_offset * nr),
		1);
	if (ret) {
		dev_err(dev, "Failed to unmask IRQ of sensor %d\n", nr);
		goto out;
	}
out:
	mutex_unlock(&vtsens->lock);
	if (ret)
		return ret;
	return count;
}

static ssize_t show_volt(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct st_vtsens *vtsens = dev_get_drvdata(dev);
	unsigned int nr = to_sensor_dev_attr_2(attr)->nr;
	int ret, volt;

	ret = read_sensor(vtsens, nr, SENSOR_VOLT, &volt);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", volt);
}

/* Compatible device data GLLCFF */
static struct sensor_device_attribute_2 st_gllcff_sensors_attrs[] = {
	SENSOR_ATTR(mode, 0444, show_mode, NULL, 0),
	SENSOR_ATTR_2(sensor_temp0, 0444, show_temp, NULL, 0, 0),
	SENSOR_ATTR_2(sensor_temp0_min, 0644, show_temp_thr,
		      set_temp_thr, 0, 1),
	SENSOR_ATTR_2(sensor_temp0_max, 0644, show_temp_thr,
		      set_temp_thr, 0, 2),
	SENSOR_ATTR_2(sensor_temp1, 0444, show_temp, NULL, 1, 0),
	SENSOR_ATTR_2(sensor_temp1_min, 0644, show_temp_thr,
		      set_temp_thr, 1, 1),
	SENSOR_ATTR_2(sensor_temp1_max, 0644, show_temp_thr,
		      set_temp_thr, 1, 2),
	SENSOR_ATTR_2(sensor_temp2, 0444, show_temp, NULL, 2, 0),
	SENSOR_ATTR_2(sensor_temp2_min, 0644, show_temp_thr,
		      set_temp_thr, 2, 1),
	SENSOR_ATTR_2(sensor_temp2_max, 0644, show_temp_thr,
		      set_temp_thr, 2, 2),
	SENSOR_ATTR_2(sensor_volt0, 0444, show_volt, NULL, 0, 0),
	SENSOR_ATTR_2(sensor_volt1, 0444, show_volt, NULL, 3, 0),
	SENSOR_ATTR_2(sensor_volt2, 0444, show_volt, NULL, 4, 0),
	SENSOR_ATTR_2(sensor_volt3, 0444, show_volt, NULL, 5, 0),
	SENSOR_ATTR_2(sensor_volt4, 0444, show_volt, NULL, 6, 0),
};

static struct attribute *st_gllcff_attrs[] = {
	&st_gllcff_sensors_attrs[0].dev_attr.attr,
	&st_gllcff_sensors_attrs[1].dev_attr.attr,
	&st_gllcff_sensors_attrs[2].dev_attr.attr,
	&st_gllcff_sensors_attrs[3].dev_attr.attr,
	&st_gllcff_sensors_attrs[4].dev_attr.attr,
	&st_gllcff_sensors_attrs[5].dev_attr.attr,
	&st_gllcff_sensors_attrs[6].dev_attr.attr,
	&st_gllcff_sensors_attrs[7].dev_attr.attr,
	&st_gllcff_sensors_attrs[8].dev_attr.attr,
	&st_gllcff_sensors_attrs[9].dev_attr.attr,
	&st_gllcff_sensors_attrs[10].dev_attr.attr,
	&st_gllcff_sensors_attrs[11].dev_attr.attr,
	&st_gllcff_sensors_attrs[12].dev_attr.attr,
	&st_gllcff_sensors_attrs[13].dev_attr.attr,
	&st_gllcff_sensors_attrs[14].dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(st_gllcff);

static const struct st_vtsens_compat_data st_gllcff_cdata = {
	.dconf_reg	= 0x0,
	.sconf_reg	= 0x4,
	.calib_reg = {
		[CALIB_CURVATURE]	= REG_FIELD(0x8,	0,	2),
		[CALIB_DCORRECT]	= REG_FIELD(0x8,	8,	13),
		[CALIB_PIVOT]		= REG_FIELD(0x8,	16,	21),
		[CALIB_TRIMVBE]		= REG_FIELD(0xC,	0,	8),
	},
	.calib_data = {
		[CALIB_CURVATURE]	= REG_FIELD(0x6C,	21,	23),
		[CALIB_DCORRECT]	= REG_FIELD(0x6C,	9,	14),
		[CALIB_PIVOT]		= REG_FIELD(0x6C,	14,	20),
		[CALIB_TRIMVBE]		= REG_FIELD(0x6C,	0,	8),
	},
	.calib_defs = {
		[CALIB_TRIMVBE] = 0x00,
		[CALIB_DCORRECT] = 0x20,
		[CALIB_PIVOT] = 0x20,
		[CALIB_CURVATURE] = 0x00,
	},
	.clockdiv	= REG_FIELD(0x14,	0,	1),
	.sensor_reg_fields = {
		[S_DATA]	= REG_FIELD_ID(0x20, 0, 18, 7, 0x4),
		[S_THR0_VAL]	= REG_FIELD_ID(0x40, 0, 13, 3, 0x4),
		[S_THR0_POL]	= REG_FIELD_ID(0x40, 14, 14, 3, 0x4),
		[S_THR0_ENB]	= REG_FIELD_ID(0x40, 15, 15, 3, 0x4),
		[S_THR1_VAL]	= REG_FIELD_ID(0x40, 16, 29, 3, 0x4),
		[S_THR1_POL]	= REG_FIELD_ID(0x40, 30, 30, 3, 0x4),
		[S_THR1_ENB]	= REG_FIELD_ID(0x40, 31, 31, 3, 0x4),
		[S_IRQ_CLEAR]	= REG_FIELD_ID(0x74, 0, 0, 3, 0x20),
		[S_IRQ_MASK]	= REG_FIELD_ID(0x84, 0, 0, 3, 0x20),
		[S_IRQ_UNMASK]	= REG_FIELD_ID(0x80, 0, 0, 3, 0x20),
	},
	.attr_groups = st_gllcff_groups,
	.r_sensors_mode = {
		SENSOR_THER,	SENSOR_THER,	SENSOR_VOLT,
		SENSOR_VOLT,	SENSOR_VOLT,	SENSOR_VOLT,
		SENSOR_OFF,
	},
};

static const struct of_device_id st_vtsens_of_match[] = {
	{ .compatible = "st,vtsens-gllcff", .data = &st_gllcff_cdata },
	{ }
};
MODULE_DEVICE_TABLE(of, st_vtsens_of_match);

static int vtsens_static_calib_conf(struct st_vtsens *vtsens)
{
	struct device *dev = &vtsens->pdev->dev;
	struct regmap *regmap = vtsens->regmap;
	struct regmap *regmap_ca = vtsens->regmap_ca;
	struct regmap_field *fld, *fld2;
	int ret;
	unsigned int i;
	unsigned int calib;

	/* Copy of calibration data from SAFEMEM */
	for (i = 0; i < CALIB_DATA_NB; i++) {
		fld = devm_regmap_field_alloc(dev, regmap,
					      vtsens->cdata->calib_reg[i]);
		if (IS_ERR(fld)) {
			dev_err(dev, "failed to alloc calib regfield\n");
			return PTR_ERR(fld);
		}

		fld2 = devm_regmap_field_alloc(dev, regmap_ca,
					       vtsens->cdata->calib_data[i]);
		if (IS_ERR(fld2)) {
			dev_err(dev, "failed to alloc calib data regfield\n");
			devm_regmap_field_free(dev, fld);
			return PTR_ERR(fld2);
		}

		ret = regmap_field_read(fld2, &calib);
		if (ret) {
			dev_err(dev, "failed to read calib data %d (0x%x)\n",
				i, ret);
			devm_regmap_field_free(dev, fld);
			devm_regmap_field_free(dev, fld2);
			return ret;
		}

		/* Use the default value if the calibration data is 0 */
		if (calib == 0)
			calib = vtsens->cdata->calib_defs[i];

		ret = regmap_field_write(fld, calib);
		if (ret) {
			dev_err(dev, "failed to write calib reg %d (0x%x)\n",
				i, ret);
			devm_regmap_field_free(dev, fld);
			devm_regmap_field_free(dev, fld2);
			return ret;
		}

		devm_regmap_field_free(dev, fld);
		devm_regmap_field_free(dev, fld2);
	}

	return 0;
}

static int vtsens_static_conf(struct st_vtsens *vtsens)
{
	struct device *dev = &vtsens->pdev->dev;
	struct regmap *regmap = vtsens->regmap;
	struct regmap_field *clkdiv_fld;
	int ret;
	unsigned int i;
	unsigned int conf = 0;

	/* Copy the calibration data, if available */
	if (vtsens->regmap_ca) {
		ret = vtsens_static_calib_conf(vtsens);
		if (ret) {
			dev_err(dev, "failed to set calibration data\n");
			return ret;
		}
	}

	/* Set the clock divider based on the source clock rate */
	clkdiv_fld = devm_regmap_field_alloc(dev, regmap,
					     vtsens->cdata->clockdiv);
	if (IS_ERR(clkdiv_fld)) {
		dev_err(dev, "failed to alloc regfield\n");
		return PTR_ERR(clkdiv_fld);
	}

	ret = regmap_field_write(clkdiv_fld,
				(clk_get_rate(vtsens->clk) < 32000000) ? 0 : 1);
	if (ret) {
		dev_err(dev, "failed to write clock divider\n");
		devm_regmap_field_free(dev, clkdiv_fld);
		return ret;
	}

	devm_regmap_field_free(dev, clkdiv_fld);

	/* Compute the static configuration register */
	for (i = 0; i < MAX_R_SENSOR; i++) {
		if (vtsens->cdata->r_sensors_mode[i] == SENSOR_OFF)
			continue;

		if (vtsens->cdata->r_sensors_mode[i] == SENSOR_THER) {
			/* Detection of edge done BELOW threshold 0 */
			ret = regmap_fields_write(vtsens->fld[S_THR0_POL],
						  i, 1);
			if (ret) {
				dev_err(dev, "failed to write polarity setting\n");
				return ret;
			}

			/* Detection of edge done ABOVE threshold 1 */
			ret = regmap_fields_write(vtsens->fld[S_THR1_POL],
						  i, 0);
			if (ret) {
				dev_err(dev, "failed to write polarity setting\n");
				return ret;
			}
		}

		/* Enable the remote sensor */
		conf |= (1 << i);
		conf |= ((vtsens->cdata->r_sensors_mode[i] == SENSOR_THER) ?
				(256 << i) : 0);
	}

	ret = regmap_write(regmap, vtsens->cdata->sconf_reg, conf);
	if (ret) {
		dev_err(dev, "failed to write static configuration\n");
		return ret;
	}

	/* Enable thermal sensor scan if at least one remote is defined. */
	for (i = 0; i < MAX_R_SENSOR; i++) {
		if (vtsens->cdata->r_sensors_mode[i] != SENSOR_THER)
			continue;

		/* Temperature sensor found.  Set scan mode and break. */
		ret = regmap_read(regmap, vtsens->cdata->dconf_reg, &conf);
		if (ret) {
			dev_err(dev, "failed to read dynamic configuration\n");
			return ret;
		}

		conf |= (1 << 16);

		ret = regmap_write(regmap, vtsens->cdata->dconf_reg, conf);
		if (ret) {
			dev_err(dev, "failed to write dynamic configuration\n");
			return ret;
		}

		break;
	}

	return 0;
}

static irqreturn_t vtsens_irq_thread(int irq, void *dev_id)
{
	struct st_vtsens *vtsens = (struct st_vtsens *)dev_id;
	struct device *dev = &vtsens->pdev->dev;
	int nr = -1;
	int i, ret;
	char sensor_node[30];

	/* Retrieve the sensor id based on IRQ value */
	for (i = 0; i < MAX_R_SENSOR + 1; i++) {
		if (irq == vtsens->irq[i]) {
			nr = i;
			break;
		}
	}

	if (nr < 0) {
		dev_err(dev, "Got an unexpected interrupt %d\n", irq);
		return IRQ_NONE;
	}

	/* Mask the interrupt of the corresponding sensor */
	ret = regmap_write(vtsens->regmap,
		vtsens->cdata->sensor_reg_fields[S_IRQ_MASK].reg +
		(vtsens->cdata->sensor_reg_fields[S_IRQ_MASK].id_offset * nr),
		1);
	if (ret) {
		dev_err(dev, "Failed to mask IRQ of sensor %d\n", nr);
		return IRQ_NONE;
	}

	/* Generate an event to the application */
	snprintf(sensor_node, sizeof(sensor_node), "sensor_temp%d", nr);
	sysfs_notify(&vtsens->hwmon_dev->kobj, NULL, sensor_node);

	return IRQ_HANDLED;
}

static int request_sensor_irq(struct st_vtsens *vtsens)
{
	int i, ret;
	struct device *dev = &vtsens->pdev->dev;

	for (i = 0; i < MAX_R_SENSOR + 1; i++) {
		/*
		 * Sensors consist of one core and up to MAR_R_SENSOR remote.
		 * Sensor 0 is the core.  Subtract one to translate overall
		 * to remote config.
		 */
		if ((i > 0) &&
		    (vtsens->cdata->r_sensors_mode[i - 1] != SENSOR_THER))
			continue;

		vtsens->irq[i] = platform_get_irq(vtsens->pdev, i);
		if (vtsens->irq[i] < 0) {
			dev_err(dev, "Failed to get THERM %d irq: %d\n", i,
				vtsens->irq[0]);
			return vtsens->irq[i];
		}

		ret = devm_request_threaded_irq(dev, vtsens->irq[i], NULL,
						vtsens_irq_thread,
						IRQF_TRIGGER_RISING |
							IRQF_ONESHOT,
						dev_name(dev), vtsens);
		if (ret) {
			dev_err(dev, "Failed to request irq %d: %d\n",
				vtsens->irq[i], ret);
			return ret;
		}
	}

	return 0;
}

static int alloc_sensor_regfields(struct st_vtsens *vtsens)
{
	struct device *dev = &vtsens->pdev->dev;
	struct regmap *regmap = vtsens->regmap;
	const struct reg_field *reg_fields = vtsens->cdata->sensor_reg_fields;
	int i;

	for (i = 0; i < S_MAX_REGFIELDS; i++) {
		vtsens->fld[i] = devm_regmap_field_alloc(dev, regmap,
						reg_fields[i]);
		if (IS_ERR(vtsens->fld[i])) {
			dev_err(dev, "failed to alloc sensor regfields\n");
			return PTR_ERR(vtsens->fld[i]);
		}
	}

	return 0;
}

static const struct regmap_config regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int st_vtsens_probe(struct platform_device *pdev)
{
	struct st_vtsens *vtsens;
	struct device *dev = &pdev->dev;
	struct device *hwmon_dev;
	const struct of_device_id *match;
	struct resource *res;
	void __iomem *mmio_base;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	vtsens = devm_kzalloc(dev, sizeof(*vtsens), GFP_KERNEL);
	if (!vtsens)
		return -ENOMEM;

	vtsens->pdev = pdev;
	platform_set_drvdata(pdev, vtsens);

	match = of_match_device(st_vtsens_of_match, dev);
	if (!match || !match->data)
		return -EINVAL;

	vtsens->cdata = match->data;

	mutex_init(&vtsens->lock);

	/* Get VTSens regmap entry */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmio_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(mmio_base)) {
		dev_err(dev, "failed to remap IO\n");
		return PTR_ERR(mmio_base);
	}

	vtsens->regmap = devm_regmap_init_mmio(dev, mmio_base, &regmap_config);
	if (IS_ERR(vtsens->regmap)) {
		dev_err(dev, "failed to initialise regmap\n");
		return PTR_ERR(vtsens->regmap);
	}

	/* Get Calibration data regmap entry */
	vtsens->regmap_ca = syscon_regmap_lookup_by_phandle(np,
							    "st,syscfg-calib");
	if (IS_ERR(vtsens->regmap_ca)) {
		dev_err(dev, "no st,syscfg-calib found, fallback to default\n");
		vtsens->regmap_ca = NULL;
	}

	vtsens->clk = devm_clk_get(dev, "thermal");
	if (IS_ERR(vtsens->clk)) {
		dev_err(dev, "failed to fetch clock\n");
		ret = PTR_ERR(vtsens->clk);
		goto out;
	}

	ret = request_sensor_irq(vtsens);
	if (ret) {
		dev_warn(dev, "Failed get necessary IRQ. Polling only mode\n");
		vtsens->mode = MODE_POLLING_ONLY;
	} else {
		vtsens->mode = MODE_POLLING_AND_IRQ;
	}

	ret = alloc_sensor_regfields(vtsens);
	if (ret)
		goto out;

	/* Static calibration of the VTSens */
	ret = vtsens_static_conf(vtsens);
	if (ret)
		goto out;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, dev_name(dev),
							   vtsens,
						   vtsens->cdata->attr_groups);
	if (IS_ERR(hwmon_dev)) {
		ret = PTR_ERR(hwmon_dev);
		goto out;
	}

	vtsens->hwmon_dev = hwmon_dev;

	/* Driver only currently supports one vtsens block */
	WARN_ON(g_vtsens);

	g_vtsens = vtsens;

	return 0;

out:
	of_node_put(np);

	return ret;
}

static int st_vtsens_remove(struct platform_device *pdev)
{
	g_vtsens = NULL;

	return 0;
}

static struct platform_driver st_vtsens_thermal_driver = {
	.driver = {
		.name	= "st_vtsens",
		.of_match_table = st_vtsens_of_match,
	},
	.probe		= st_vtsens_probe,
	.remove		= st_vtsens_remove,
};

module_platform_driver(st_vtsens_thermal_driver);

MODULE_AUTHOR("STMicroelectronics <alain.volmat@st.com>");
MODULE_DESCRIPTION("STMicroelectronics VTsens Sensor Driver");
MODULE_LICENSE("GPL v2");
