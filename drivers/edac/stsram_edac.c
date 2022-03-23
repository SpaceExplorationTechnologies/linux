/*
 * ST SRAM EDAC (error detection and correction)
 *
 * Copyright (C) 2017 STMicroelectronics
 * Author: Nicolas Toromanoff <nicolas.toromanoff@st.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/edac.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "edac_device.h"
#include "edac_mc.h"

static int scrub_freq_seconds = 1;
static long stsram_ce_sw_inject_count, stsram_ue_sw_inject_count;

static const struct regmap_config stsramss_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

#define SYSTEM_CONFIG1 0x4
#define SYSTEM_CONFIG2 0x8
#define SYSTEM_STATUS3 0xC
#define SYSTEM_CONFIG4 0x10
#define SYSTEM_CONFIG5 0x14
#define SYSTEM_CONFIG6 0x18
#define SYSTEM_STATUS7 0x1C
#define SYSTEM_STATUS8 0x20
#define SYSTEM_STATUS9 0x24
#define SYSTEM_STATUS10 0x28
#define SYSTEM_CONFIG11 0x2C
#define SYSTEM_CONFIG12 0x30
#define SYSTEM_STATUS13 0x34
#define SYSTEM_CONFIG14 0x38
#define SYSTEM_CONFIG15 0x3C
#define SYSTEM_CONFIG16 0x40

#define IT_STATUS       SYSTEM_STATUS10
#define IT_CLEAR        SYSTEM_CONFIG11
#define IT_SET          SYSTEM_CONFIG12
#define IT_MASKSTATUS   SYSTEM_STATUS13
#define IT_MASKCLEAR    SYSTEM_CONFIG14
#define IT_MASKSET      SYSTEM_CONFIG15

#define IT_END_OF_SCRUBBING    BIT(3)
#define IT_END_OF_SCAN_PERIOD  BIT(2)
#define IT_DOUBLE_ERROR        BIT(1)
#define IT_SINGLE_ERROR        BIT(0)

#define SINGLE_ERROR_LOG       SYSTEM_STATUS7
#define DOUBLE_ERROR_LOG       SYSTEM_STATUS8

#define ERROR_CORRUPTED_ADDR_MASK  0x1fffff
#define ERROR_IDENTIFIER_MASK      BIT(31)

/**
 * enum stsram_regmap_fields_id - id for each regmap field
 */
enum stsram_regmap_fields_id {
	MANUAL_SCRUB_MODE,
	OPERATING_MODE_STATUS,
	SINGLE_ERROR_DEBUG_MODE,
	SCAN_SCRUBBING_PERIOD,
	DOUBLE_BIT_ERRORS_NUM,
	SINGLE_BIT_ERRORS_NUM,
	CLK_TIMER_SRC
};

/**
 * struct stsram_regfields - reg_field definition for each
 * stsram_regmap_fields_id
 */
static const struct reg_field stsram_regfields[] = {
	/* SYSTEM_CONFIG1 Scrubbing_Mode */
	[MANUAL_SCRUB_MODE] = REG_FIELD(SYSTEM_CONFIG1, 0, 0),
	/* SYSTEM_CONFIG2 Debug_mode */
	[SINGLE_ERROR_DEBUG_MODE] = REG_FIELD(SYSTEM_CONFIG2, 0, 0),
	/* SYSTEM_STATUS3 Operating_Mode_Status */
	[OPERATING_MODE_STATUS] = REG_FIELD(SYSTEM_STATUS3, 0, 1),
	/* SYSTEM_CONFIG4 Scrubbing_period */
	[SCAN_SCRUBBING_PERIOD] = REG_FIELD(SYSTEM_CONFIG4, 0, 12),
	/* SYSTEM_STATUS9 SINGLE_BIT_ERRORS_NUMBER */
	[SINGLE_BIT_ERRORS_NUM] = REG_FIELD(SYSTEM_STATUS9, 0, 15),
	/* SYSTEM_STATUS9 DOUBLE_BIT_ERRORS_NUMBER */
	[DOUBLE_BIT_ERRORS_NUM] = REG_FIELD(SYSTEM_STATUS9, 16, 31),
	/* SYSTEM_CONFIG16  CLK_TIMER_SET */
	[CLK_TIMER_SRC] = REG_FIELD(SYSTEM_CONFIG16, 0, 0)
};

/**
 * struct stsram_edac_priv - ST SRAM memory controller private instance data
 * @mmio:	Base address of the DDR controller
 * @rmf:	Arrray that store each regmap fields
 * @irq:	irq in case of error detection, end of scrubbing, and end of
 *              scan period
 */
struct stsram_edac_priv {
	void __iomem *mmio;
	struct regmap *regmap;
	struct regmap_field *rmf[ARRAY_SIZE(stsram_regfields)];
	int irq;
	u32 ce_errors_detected;
	u32 ue_errors_detected;
};

static int stsram_edac_configure_and_start(struct platform_device *pdev)
{
	struct stsram_edac_priv *priv;
	struct edac_device_ctl_info *edac_dev;
	struct clk *clk_timer;
	u32 mode;
	unsigned long scrubbing_period;

	edac_dev = platform_get_drvdata(pdev);
	priv = edac_dev->pvt_info;

	clk_timer = clk_get(&pdev->dev, "timer");
	if (IS_ERR(clk_timer)) {
		dev_err(&pdev->dev, "failed to get the timer clock\n");
		return PTR_ERR(clk_timer);
	}

	/* select input frequency 0 for 60MHz / 1 for 50MHz */
	if (clk_get_rate(clk_timer) == 60000000) {
		regmap_field_write(priv->rmf[CLK_TIMER_SRC], 0);
	} else if (clk_get_rate(clk_timer) == 50000000) {
		regmap_field_write(priv->rmf[CLK_TIMER_SRC], 1);
	} else {
		dev_err(&pdev->dev, "timer speed %ld not supported\n",
			clk_get_rate(clk_timer));
	}
	clk_put(clk_timer);

	/*
	 * HW allows a period of 1 second to 4096 seconds, but only by
	 * a power of two (1, 2, 4, 8, ... 4096).  Round down to the closest
	 * match.
	 */
	scrubbing_period = scrub_freq_seconds ? fls(scrub_freq_seconds) : 1;
	if (scrubbing_period > 4096)
		scrubbing_period = 4096;
	regmap_field_write(priv->rmf[SCAN_SCRUBBING_PERIOD], scrubbing_period);

	if (edac_op_state == EDAC_OPSTATE_INT) {
		/* Enable correctable debug mode. */
		regmap_field_write(priv->rmf[SINGLE_ERROR_DEBUG_MODE], 1);

		/* Enable interrupts */
		writel(~(IT_DOUBLE_ERROR | IT_SINGLE_ERROR),
		       priv->mmio + IT_MASKSET);
		writel(IT_DOUBLE_ERROR | IT_SINGLE_ERROR,
		       priv->mmio + IT_MASKCLEAR);
	}

	/* Start scrubbing if controller is in idle mode */
	regmap_field_read(priv->rmf[OPERATING_MODE_STATUS], &mode);
	if (!mode)
		regmap_field_write(priv->rmf[MANUAL_SCRUB_MODE], 1);
	else
		dev_warn(&pdev->dev,
			 "didn't start scrubbing, sram not ready (mode =%d)\n",
			 mode);

	return 0;
}

static void stsram_edac_stop_and_unconfigure(struct edac_device_ctl_info
								*edac_dev)
{
	struct stsram_edac_priv *priv;

	priv = edac_dev->pvt_info;

	/* Stop scrubbing */
	regmap_field_write(priv->rmf[MANUAL_SCRUB_MODE], 0);

	/* Reset interrupt mask as init */
	if (edac_op_state == EDAC_OPSTATE_INT) {
		writel(IT_END_OF_SCRUBBING | IT_END_OF_SCAN_PERIOD |
		       IT_DOUBLE_ERROR | IT_SINGLE_ERROR,
		       priv->mmio + IT_MASKCLEAR);

		/* Remove the single error Interrupt */
		regmap_field_write(priv->rmf[SINGLE_ERROR_DEBUG_MODE], 0);
	}
}

static int stsram_edac_alloc_regfields(struct device *dev,
				       struct stsram_edac_priv *priv)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(stsram_regfields); i++) {
		priv->rmf[i] = devm_regmap_field_alloc(dev, priv->regmap,
						       stsram_regfields[i]);
		if (IS_ERR(priv->rmf[i])) {
			dev_err(dev, "failed to alloc regfields[%d]\n", i);
			return PTR_ERR(priv->rmf[i]);
		}
	}

	return 0;
}

static int stsram_edac_alloc_regmap(struct device *dev,
				    struct stsram_edac_priv *priv)
{
	priv->regmap = devm_regmap_init_mmio(dev, priv->mmio,
					   &stsramss_regmap_cfg);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	return stsram_edac_alloc_regfields(dev, priv);
}

static void stsram_edac_check(struct edac_device_ctl_info *edac_dev)
{
	struct stsram_edac_priv *priv = edac_dev->pvt_info;
	u32 hw_ce_errors, hw_ue_errors;
	u32 error_log;
	u32 status;

	regmap_field_read(priv->rmf[SINGLE_BIT_ERRORS_NUM], &hw_ce_errors);
	regmap_field_read(priv->rmf[DOUBLE_BIT_ERRORS_NUM], &hw_ue_errors);

	status = ioread32(priv->mmio + IT_STATUS);
	if (status & IT_END_OF_SCAN_PERIOD) {
		writel(status & IT_END_OF_SCAN_PERIOD, priv->mmio + IT_CLEAR);
		/* The counters values are loaded in the
		 * SINGLE_BIT_ERRORS_NUM and DOUBLE_BIT_ERRORS_NUM registers at
		 * the end of each scan period (end_of _scan_period IRQ raised)
		 *
		 * So if an IT_END_OF_SCAN_PERIOD arrived between previous
		 * check and now, we have to reset saved counters.
		 */
		priv->ue_errors_detected = 0;
		priv->ce_errors_detected = 0;
	}

	if (stsram_ce_sw_inject_count > 0) {
		status |= IT_SINGLE_ERROR;
		stsram_ce_sw_inject_count--;
		dev_info(edac_dev->dev, "Injecting correctable error\n");
		priv->ce_errors_detected--;
		if (edac_op_state == EDAC_OPSTATE_INT) {
			writel(IT_SINGLE_ERROR, priv->mmio + IT_SET);
		}
	}
	if (stsram_ue_sw_inject_count > 0) {
		status |= IT_DOUBLE_ERROR;
		stsram_ue_sw_inject_count--;
		dev_warn(edac_dev->dev, "Injecting uncorrectable error\n");
		priv->ue_errors_detected--;
		if (edac_op_state == EDAC_OPSTATE_INT) {
			writel(IT_DOUBLE_ERROR, priv->mmio + IT_SET);
		}
	}

	if ((status & IT_DOUBLE_ERROR) &&
	    (hw_ue_errors != priv->ue_errors_detected)) {
		writel(IT_DOUBLE_ERROR, priv->mmio + IT_CLEAR);
		error_log = ioread32(priv->mmio + DOUBLE_ERROR_LOG);
		edac_device_printk(edac_dev, KERN_ERR,
				   "Double error found in sram @%x by %s access",
				   error_log & ERROR_CORRUPTED_ADDR_MASK,
				   error_log & ERROR_IDENTIFIER_MASK ?
				   "scrubbing" : "system");
		edac_device_handle_ue(edac_dev, 0, 0, "stsram double");
		priv->ue_errors_detected = hw_ue_errors;
	}

	if ((status & IT_SINGLE_ERROR) &&
	    (hw_ce_errors != priv->ce_errors_detected)) {
		writel(IT_SINGLE_ERROR, priv->mmio + IT_CLEAR);
		error_log = ioread32(priv->mmio + SINGLE_ERROR_LOG);
		edac_device_printk(edac_dev, KERN_ERR,
				   "Single error found in sram @%x by %s access",
				   error_log & ERROR_CORRUPTED_ADDR_MASK,
				   error_log & ERROR_IDENTIFIER_MASK ?
				   "scrubbing" : "system");
		edac_device_handle_ce(edac_dev, 0, 0, "stsram single");
		priv->ce_errors_detected = hw_ce_errors;
	}
}

static irqreturn_t stsram_edac_isr(int irq, void *pdata)
{
	struct edac_device_ctl_info *edac_dev = pdata;
	struct stsram_edac_priv *priv = edac_dev->pvt_info;
	u32 status;
	irqreturn_t irqreturn = IRQ_NONE;

	status = ioread32(priv->mmio + IT_STATUS);
	writel_relaxed(status, priv->mmio + IT_CLEAR);

	if (status & (IT_END_OF_SCRUBBING | IT_END_OF_SCAN_PERIOD)) {
		/* Nothing to do */
		irqreturn = IRQ_HANDLED;
	}

	if (status & (IT_SINGLE_ERROR | IT_DOUBLE_ERROR)) {
		stsram_edac_check(edac_dev);
		irqreturn = IRQ_HANDLED;
	}

	return irqreturn;
}

static ssize_t stsram_ce_reset_counter_store(struct edac_device_ctl_info *edac_dev,
					     const char *data, size_t count)
{
	int i, j;

	/* reset all instances */
	for (i = 0 ; i < edac_dev->nr_instances ; i++) {
		for (j = 0 ; j < edac_dev->instances[i].nr_blocks ; j++)
			edac_dev->instances[i].blocks[j].counters.ce_count = 0;
		edac_dev->instances[i].counters.ce_count = 0;
	}

	/* reset the 'total' tree */
	edac_dev->counters.ce_count = 0;

	return count;
}

static ssize_t stsram_ue_reset_counter_store(struct edac_device_ctl_info *edac_dev,
					     const char *data, size_t count)
{
	int i;

	/* reset all instances (there is no blocks)*/
	for (i = 0 ; i < edac_dev->nr_instances ; i++) {
		edac_dev->instances[i].counters.ue_count = 0;
		/* no need to reset
		 * edac_dev->instances[i].blocks[j].counters.ue_count
		 * no sub block in this device
		 */
	}

	/*reset the 'total' tree */
	edac_dev->counters.ue_count = 0;

	return count;
}

static ssize_t stsram_ue_sw_inject_show(struct edac_device_ctl_info *edac_info,
					char *data)
{
	return sprintf(data, "%ld\t Number of errors to be injected\n",
		       stsram_ue_sw_inject_count);
}

static ssize_t stsram_ue_sw_inject_store(struct edac_device_ctl_info *edac_info,
					 const char *data, size_t count)
{
	int ret;
	struct stsram_edac_priv *priv = edac_info->pvt_info;

	ret = kstrtol(data, 0, &stsram_ue_sw_inject_count);
	if (edac_op_state == EDAC_OPSTATE_INT)
		writel(IT_DOUBLE_ERROR, priv->mmio + IT_SET); /* simulate it*/

	if (ret)
		return ret;
	return count;
}

static ssize_t stsram_ce_sw_inject_show(struct edac_device_ctl_info *edac_info,
					char *data)
{
	return sprintf(data, "%ld\t Number of errors to be injected\n",
		       stsram_ce_sw_inject_count);
}

static ssize_t stsram_ce_sw_inject_store(struct edac_device_ctl_info *edac_info,
					 const char *data, size_t count)
{
	int ret;
	struct stsram_edac_priv *priv = edac_info->pvt_info;

	ret = kstrtol(data, 0, &stsram_ce_sw_inject_count);
	if (edac_op_state == EDAC_OPSTATE_INT)
		writel(IT_SINGLE_ERROR, priv->mmio + IT_SET); /* simulate it*/

	if (ret)
		return ret;
	return count;
}

#define SYSFS_ATTR(_name, _mode, _show, _store)                 \
(struct edac_dev_sysfs_attribute){                              \
	.attr = {.name = __stringify(_name), .mode = _mode },   \
	.show   = _show,                                        \
	.store  = _store,                                       \
}

static struct edac_dev_sysfs_attribute stsram_sysfs_attrs[] = {
	SYSFS_ATTR(ce_reset_counter, 0200, NULL,
		   stsram_ce_reset_counter_store),
	SYSFS_ATTR(ue_reset_counter, 0200, NULL,
		   stsram_ue_reset_counter_store),
	SYSFS_ATTR(ce_sw_inject, 0644, stsram_ce_sw_inject_show,
		   stsram_ce_sw_inject_store),
	SYSFS_ATTR(ue_sw_inject, 0644, stsram_ue_sw_inject_show,
		   stsram_ue_sw_inject_store),
	{},
};

/**
 * stsram_edac_mc_probe - Check controller and bind driver
 * @pdev:	Pointer to the platform_device struct
 *
 * Probes a specific controller instance for binding with the driver.
 *
 * Return: 0 if the controller instance was successfully bound to the
 * driver; otherwise, < 0 on error.
 */
static int stsram_edac_mc_probe(struct platform_device *pdev)
{
	int rc;
	struct stsram_edac_priv *priv;
	struct edac_device_ctl_info *edac_dev;
	struct resource *res;
	struct device *dev = &pdev->dev;

	edac_dev = edac_device_alloc_ctl_info(sizeof(*priv),
					      "sram", 1, "sram", 1, 0,
					      NULL, 0,
					      edac_device_alloc_index());
	if (!edac_dev)
		return -ENOMEM;

	/* make sure error reporting method is sane */
	switch (edac_op_state) {
	case EDAC_OPSTATE_POLL:
	case EDAC_OPSTATE_INT:
		break;
	default:
		edac_op_state = EDAC_OPSTATE_POLL;
		break;
	}

	priv = edac_dev->pvt_info;

#ifdef CONFIG_SPACEX
	edac_dev->panic_on_ue = 1;
#endif

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sysconf");
	priv->mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->mmio)) {
		rc = PTR_ERR(priv->mmio);
		goto free_edac_ctl;
	}

	rc = stsram_edac_alloc_regmap(dev, priv);
	if (rc)
		goto free_edac_ctl;

	edac_dev->dev = dev;
	edac_dev->mod_name = dev_name(dev);
	edac_dev->dev_name = dev_name(dev);
	edac_dev->ctl_name = "sram_err";

	if (edac_op_state == EDAC_OPSTATE_POLL)
		edac_dev->edac_check = stsram_edac_check;
	else
		edac_dev->edac_check = NULL;

	/* Add our specific sysfs entries */
	edac_dev->sysfs_attributes = stsram_sysfs_attrs;

	rc = edac_device_add_device(edac_dev);
	if (rc)
		goto free_edac_ctl;

	if (edac_op_state == EDAC_OPSTATE_INT) {
		priv->irq = platform_get_irq(pdev, 0);
		rc = devm_request_irq(dev, priv->irq, stsram_edac_isr, 0,
				      "[EDAC] SRAM err", edac_dev);
		if (rc < 0) {
			dev_err(dev, "Unable to alloc irq %d for SRAM ECC\n",
				priv->irq);
			rc = -ENODEV;
			goto del_device;
		}
		dev_info(dev, "Irq %d allocated for SRAM ECC Err\n", priv->irq);
	}

	platform_set_drvdata(pdev, edac_dev);

	/* Start capturing correctable and uncorrectable errors. */
	return stsram_edac_configure_and_start(pdev);

del_device:
	edac_device_del_device(dev);
free_edac_ctl:
	edac_device_free_ctl_info(edac_dev);
	return rc;
}

/**
 * stsram_edac_mc_remove - Unbind driver from controller
 * @pdev:	Pointer to the platform_device struct
 *
 * Return: Unconditionally 0
 */
static int stsram_edac_mc_remove(struct platform_device *pdev)
{
	struct edac_device_ctl_info *edac_dev;

	edac_dev = platform_get_drvdata(pdev);
	stsram_edac_stop_and_unconfigure(edac_dev);

	edac_device_del_device(edac_dev->dev);
	edac_device_free_ctl_info(edac_dev);

	return 0;
}

static const struct of_device_id stsram_edac_match[] = {
	{ .compatible = "st,sram-ecc", },
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, stsram_edac_match);

static struct platform_driver stsram_edac_mc_driver = {
	.driver = {
		   .name = "stsram-edac",
		   .of_match_table = stsram_edac_match,
		   },
	.probe = stsram_edac_mc_probe,
	.remove = stsram_edac_mc_remove,
};

module_platform_driver(stsram_edac_mc_driver);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("ST SRAM ECC driver");
MODULE_LICENSE("GPL v2");
module_param(scrub_freq_seconds, int, 0444);
MODULE_PARM_DESC(scrub_freq_seconds, "ST SRAM scrubbing interval in seconds");
module_param(edac_op_state, int, 0444);
MODULE_PARM_DESC(edac_op_state,
		 "EDAC error reporting state: 0=Poll, 2=Interrupt");
