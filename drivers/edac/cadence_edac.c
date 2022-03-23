/*
 * Cadence DDR EDAC (error detection and correction)
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
#include <linux/edac.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include "edac_module.h"

static int idle_scrubbing;
static unsigned int scrub_interval = 1000;
static long cadence_ce_sw_inject_count, cadence_ue_sw_inject_count;

static const struct regmap_config cadence_edac_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

/* Granularity of reported error in bytes */
#define CADENCE_EDAC_ERR_GRAIN	16

/* registers IP name and OFFSET */
/* ECC registers */
#define DENALI_CTL_33 0x84
#define DENALI_CTL_34 0x88
#define DENALI_CTL_35 0x8c
#define DENALI_CTL_36 0x90
#define DENALI_CTL_37 0x94
#define DENALI_CTL_38 0x98
#define DENALI_CTL_39 0x9c
#define DENALI_CTL_40 0xa0
#define DENALI_CTL_41 0xa4
#define DENALI_CTL_42 0xa8
#define DENALI_CTL_43 0xac
#define DENALI_CTL_44 0xb0
#define DENALI_CTL_45 0xb4
#define DENALI_CTL_46 0xb8
/* interrupt registers */
#define DENALI_CTL_60 0xf0
#define DENALI_CTL_61 0xf4
#define DENALI_CTL_62 0xf8

/*register meaningful name */
#define INT_STATUS  DENALI_CTL_60
#define INT_ACK     DENALI_CTL_61
#define INT_MASK    DENALI_CTL_62

/* bit for IT_STATUS, IT_ACK and IT_MASK */
#define IT_ERROR_RECOVERABLE  BIT(3)
#define IT_SECOND_ERROR_RECOVERABLE  BIT(4)
#define IT_ERROR_NON_RECOVERABLE  BIT(5)
#define IT_SECOND_ERROR_NON_RECOVERABLE  BIT(6)
#define IT_END_OF_SCRUB  BIT(8)
#define IT_ONE_ORE_MORE_WRITEBACK_NOT_EXEC  BIT(7)
#define IT_ERROR_RECOVERABLE_SCRUBBING  BIT(9)
#define IT_MASK_ALL BIT(29)

/**
 * enum cadence_edac_regmap_fields_id - id for each regmap field
 */
enum cadence_edac_regmap_fields_id {
	FWC,
	ECC_EN,
	XOR_CHECK_BITS,
	ECC_WRITEBACK_EN,
	ECC_U_ADDR,
	ECC_U_SYND,
	ECC_U_DATA,
	ECC_C_ADDR,
	ECC_C_SYND,
	ECC_C_DATA,
	ECC_SCRUB_IN_PROGRESS,
	ECC_SCRUB_START,
	ECC_C_ID,
	ECC_U_ID,
	ECC_SCRUB_INTERVAL,
	ECC_SCRUB_MODE,
	ECC_SCRUB_LEN,
	ECC_SCRUB_IDLE_CNT,
	ECC_SCRUB_START_ADDR,
	ECC_SCRUB_END_ADDR,
};

/**
 * struct cadence_regfields - reg_field definition for each
 * cadence_regmap_fields_id
 */
static const struct reg_field cadence_regfields[] = {
	[FWC] = REG_FIELD(DENALI_CTL_33, 8, 8),
	[ECC_EN] = REG_FIELD(DENALI_CTL_33, 0, 0),
	[XOR_CHECK_BITS] = REG_FIELD(DENALI_CTL_34, 0, 27),
	[ECC_WRITEBACK_EN] = REG_FIELD(DENALI_CTL_35, 0, 0),
	[ECC_U_ADDR] = REG_FIELD(DENALI_CTL_36, 0, 30),
	[ECC_U_SYND] = REG_FIELD(DENALI_CTL_37, 0, 6),
	[ECC_U_DATA] = REG_FIELD(DENALI_CTL_38, 0, 31),
	[ECC_C_ADDR] = REG_FIELD(DENALI_CTL_39, 0, 30),
	[ECC_C_SYND] = REG_FIELD(DENALI_CTL_40, 0, 6),
	[ECC_C_DATA] = REG_FIELD(DENALI_CTL_41, 0, 31),
	[ECC_SCRUB_IN_PROGRESS] = REG_FIELD(DENALI_CTL_42, 24, 24),
	[ECC_SCRUB_START] = REG_FIELD(DENALI_CTL_42, 16, 16),
	[ECC_C_ID] = REG_FIELD(DENALI_CTL_42, 8, 8),
	[ECC_U_ID] = REG_FIELD(DENALI_CTL_42, 0, 0),
	[ECC_SCRUB_INTERVAL] = REG_FIELD(DENALI_CTL_43, 16, 31),
	[ECC_SCRUB_MODE] = REG_FIELD(DENALI_CTL_43, 8, 8),
	[ECC_SCRUB_LEN] = REG_FIELD(DENALI_CTL_43, 0, 7),
	[ECC_SCRUB_IDLE_CNT] = REG_FIELD(DENALI_CTL_44, 0, 15),
	[ECC_SCRUB_START_ADDR] = REG_FIELD(DENALI_CTL_45, 0, 30),
	[ECC_SCRUB_END_ADDR] = REG_FIELD(DENALI_CTL_46, 0, 30),
};

/**
 * struct cadence_edac_priv - DDR memory controller private instance data
 * @mmio:	Base address of the DDR controller
 * @rmf:	Array that store each regmap fields
 * @irq:	irq in case of error detection, end of scrubbing, and end of
 *              scan period
 */
struct cadence_edac_priv {
	/* HW info */
	void __iomem *mmio;
	struct regmap *regmap;
	struct regmap_field *rmf[ARRAY_SIZE(cadence_regfields)];
	struct resource mem_res;
	int irq;
#ifdef CONFIG_SPACEX
	unsigned int scrub_count;
#endif
};

static struct mem_ctl_info *cadence_mci;

static void start_scrubbing(struct mem_ctl_info *mci)
{
	u32 scrub_in_progress;
	struct cadence_edac_priv *priv = mci->pvt_info;

	regmap_field_read(priv->rmf[ECC_SCRUB_IN_PROGRESS],
			  &scrub_in_progress);

	if (scrub_in_progress) {
		/* nothing to do */
		return;
	}

	/* configure scrubbing */
	regmap_field_write(priv->rmf[ECC_SCRUB_START_ADDR], 0);
	regmap_field_write(priv->rmf[ECC_SCRUB_END_ADDR],
			   resource_size(&priv->mem_res));
	/* ECC_SCRUB_LEN must be a multiple of the memory burst length (8) */
	regmap_field_write(priv->rmf[ECC_SCRUB_LEN], 64);

	/* start scrubbing */
	regmap_field_write(priv->rmf[ECC_SCRUB_START], 1);

#ifdef CONFIG_SPACEX
	priv->scrub_count++;
#endif
}

static void report_error(struct mem_ctl_info *mci,
			 struct cadence_edac_priv *priv,
			 const int data_id, const int addr_id,
			 const int source_id, const int syndrome_id,
			 const enum hw_event_mc_err_type err_type,
			 const char *msg,
			 const char *other_detail)
{
	u32 data, addr, source, syndrome, offset_in_page;
	unsigned long pfn;

	/*
	 * Cadence DDR Controller User’s Manual
	 * 10.10 Clearing a Reported ECC Event
	 */
	regmap_read(priv->regmap, data_id, &data);
	regmap_field_read(priv->rmf[addr_id], &addr);
	regmap_field_read(priv->rmf[source_id], &source);
	regmap_field_read(priv->rmf[syndrome_id], &syndrome);

	pfn = (priv->mem_res.start + addr) >> PAGE_SHIFT;
	offset_in_page = (priv->mem_res.start + addr) & ~PAGE_MASK;

	edac_mc_handle_error(err_type, mci, 1,
			     pfn, offset_in_page, syndrome,
			     0 /*top_layer */,
			     0 /*mid_layer */,
			     -1/*low_layer*/,
			     msg, other_detail);
}

static void cadence_edac_mc_check(struct mem_ctl_info *mci)
{
	struct cadence_edac_priv *priv = mci->pvt_info;
	u32 status, ack = 0;
	bool start_scrub = false;

	status = ioread32(priv->mmio + INT_STATUS);

	/* Inject sw/tests errors, only one per check call*/
	if (cadence_ce_sw_inject_count > 0) {
		cadence_ce_sw_inject_count--;
		dev_dbg(mci->pdev, "Inject correctable error\n");
		status |= IT_ERROR_RECOVERABLE;
	}
	if (cadence_ue_sw_inject_count > 0) {
		cadence_ue_sw_inject_count--;
		dev_dbg(mci->pdev, "Inject uncorrectable error\n");
		status |= IT_ERROR_NON_RECOVERABLE;
	}

	if (status & IT_END_OF_SCRUB) {
		/* restart the scrubbing */
		start_scrub = true;
		ack |= IT_END_OF_SCRUB;
	}

	if (status & IT_ERROR_NON_RECOVERABLE) {
		report_error(mci, priv, ECC_U_DATA, ECC_U_ADDR, ECC_U_ID,
			     ECC_U_SYND, HW_EVENT_ERR_FATAL,
			     "uncorrectable error detected", "");
		ack |= IT_ERROR_NON_RECOVERABLE;
	}

	if (status & IT_SECOND_ERROR_NON_RECOVERABLE) {
		edac_mc_handle_error(HW_EVENT_ERR_FATAL, mci, 1,
				     0, 0, 0, 0, 0, -1,
				     "new uncorrectable error detected while first",
				     "");
		ack |= IT_SECOND_ERROR_NON_RECOVERABLE;
	}

	if (status & IT_ERROR_RECOVERABLE) {
		report_error(mci, priv, ECC_C_DATA, ECC_C_ADDR, ECC_C_ID,
			     ECC_C_SYND, HW_EVENT_ERR_CORRECTED,
			     "correctable error corrected", "");
		ack |= IT_ERROR_RECOVERABLE;
	}

	if (status & IT_SECOND_ERROR_RECOVERABLE) {
		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci, 1,
				     0, 0, 0, 0, 0, -1,
				     "new correctable error detected while first",
				     "");
		ack |= IT_SECOND_ERROR_RECOVERABLE;
	}

	/* ACK all managed IT*/
	writel(ack & status, priv->mmio + INT_ACK);

	/* Do not start scrubbing until after ack to prevent race condition */
	if (start_scrub) {
		start_scrubbing(mci);
	}
}

static irqreturn_t cadence_edac_isr(int irq, void *dev_id)
{
	struct mem_ctl_info *mci = dev_id;
	irqreturn_t irqreturn = IRQ_HANDLED;

	cadence_edac_mc_check(mci);

	return irqreturn;
}

static ssize_t cadence_scrubbing_show(struct device *dev,
				      struct device_attribute *mattr,
				      char *data)
{
	struct mem_ctl_info *mci = to_mci(dev);
	struct cadence_edac_priv *priv = mci->pvt_info;
	u32 scrub_in_progress;

	regmap_field_read(priv->rmf[ECC_SCRUB_IN_PROGRESS],
			  &scrub_in_progress);
	return scnprintf(data, PAGE_SIZE, "%d\n", scrub_in_progress);
}

static ssize_t cadence_scrubbing_store(struct device *dev,
				       struct device_attribute *mattr,
				       const char *data, size_t count)
{
	struct mem_ctl_info *mci = to_mci(dev);

	start_scrubbing(mci);

	return count;
}

static ssize_t cadence_ue_sw_inject_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *data)
{
	return scnprintf(data, PAGE_SIZE,
			 "%ld\t Number of errors to be injected\n",
			 cadence_ue_sw_inject_count);
}

static ssize_t cadence_ue_sw_inject_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret;

	ret = kstrtol(data, 0, &cadence_ue_sw_inject_count);
	if (ret)
		return ret;
	return count;
}

static ssize_t cadence_ce_sw_inject_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *data)
{
	return scnprintf(data, PAGE_SIZE,
			 "%ld\t Number of errors to be injected\n",
			 cadence_ce_sw_inject_count);
}

static ssize_t cadence_ce_sw_inject_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	int ret;

	ret = kstrtol(data, 0, &cadence_ce_sw_inject_count);
	if (ret)
		return ret;
	return count;
}

#ifdef CONFIG_SPACEX
static ssize_t cadence_scrub_count_show(struct device *dev,
					struct device_attribute *mattr,
					char *data)
{
	struct mem_ctl_info *mci = to_mci(dev);
	struct cadence_edac_priv *priv = mci->pvt_info;
	return scnprintf(data, PAGE_SIZE, "%u\n", priv->scrub_count);
}

static DEVICE_ATTR(scrub_count, 0444, cadence_scrub_count_show, NULL);
#endif

static DEVICE_ATTR(scrubbing, 0644,
		   cadence_scrubbing_show, cadence_scrubbing_store);

static DEVICE_ATTR(ue_sw_inject, 0644,
		   cadence_ue_sw_inject_show, cadence_ue_sw_inject_store);

static DEVICE_ATTR(ce_sw_inject, 0644,
		   cadence_ce_sw_inject_show, cadence_ce_sw_inject_store);

static struct attribute *cadence_dev_attrs[] = {
	&dev_attr_scrubbing.attr,
	&dev_attr_ce_sw_inject.attr,
	&dev_attr_ue_sw_inject.attr,
#ifdef CONFIG_SPACEX
	&dev_attr_scrub_count.attr,
#endif
	NULL
};

ATTRIBUTE_GROUPS(cadence_dev);

static int cadence_edac_alloc_regfields(struct device *dev,
					struct cadence_edac_priv *priv)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cadence_regfields); i++) {
		priv->rmf[i] = devm_regmap_field_alloc(dev, priv->regmap,
						cadence_regfields[i]);
		if (IS_ERR(priv->rmf[i])) {
			dev_err(dev, "failed to alloc regfields[%d]\n", i);
			return PTR_ERR(priv->rmf[i]);
		}
	}

	return 0;
}

static int cadence_edac_alloc_regmap(struct device *dev,
				     struct cadence_edac_priv *priv)
{
	priv->regmap = devm_regmap_init_mmio(dev, priv->mmio,
					     &cadence_edac_regmap_cfg);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	return cadence_edac_alloc_regfields(dev, priv);
}

/**
 * cadence_edac_init_csrows - Initialize the cs row data
 * @mci:	Pointer to the edac memory controller instance
 *
 * Initializes the chip select rows associated with the EDAC memory
 * controller instance
 *
 * Return: void.
 */
static void cadence_edac_init_csrows(struct mem_ctl_info *mci)
{
	struct cadence_edac_priv *priv = mci->pvt_info;
	struct csrow_info *csi;
	struct dimm_info *dimm;
	u32 size;
	int row, j;

	size = resource_size(&priv->mem_res) / mci->nr_csrows;

	for (row = 0; row < mci->nr_csrows; row++) {
		csi = mci->csrows[row];

		for (j = 0; j < csi->nr_channels; j++) {
			dimm = csi->channels[j]->dimm;
			dimm->edac_mode = EDAC_FLAG_SECDED;
			dimm->mtype = MEM_UNKNOWN;
			dimm->nr_pages = (size >> PAGE_SHIFT) /
				csi->nr_channels;
			dimm->grain = CADENCE_EDAC_ERR_GRAIN;
			dimm->dtype = DEV_UNKNOWN;
		}
	}
}

/**
 * select_scrubbing_mode - Set the scrubbing mode
 * @priv:	pointer to the cadence ecc priv structure
 *
 * in idle mode only when the DDR control is idle a dummy read is done
 * to check one address after the other
 *
 * in non idle mode, the SCRUB_INTERVAL time is waited between two
 * address dummy reads
 */
static void select_scrubbing_mode(struct mem_ctl_info *mci)
{
	struct cadence_edac_priv *priv = mci->pvt_info;

	dev_dbg(mci->pdev, "Select scrubbing mode (idle_scrubbing = %d)\n",
		idle_scrubbing);
	if (idle_scrubbing) {
		dev_dbg(mci->pdev, "Scrub while ddr controller is idle\n");
		regmap_field_write(priv->rmf[ECC_SCRUB_MODE], 1);
	} else {
		dev_dbg(mci->pdev, "Scrub at interval %d clocks\n",
			scrub_interval);
		regmap_field_write(priv->rmf[ECC_SCRUB_INTERVAL],
				   scrub_interval);
		regmap_field_write(priv->rmf[ECC_SCRUB_MODE],  0);
	}
}

/**
 * cadence_edac_mc_init - Initialize driver instance
 * @mci:	Pointer to the edac memory controller instance
 * @pdev:	Pointer to the platform_device struct
 *
 * Performs initialization of the EDAC memory controller instance and
 * related driver-private data associated with the memory controller the
 * instance is bound to.
 *
 * Return: void.
 */
static void cadence_edac_mc_init(struct mem_ctl_info *mci,
				 struct platform_device *pdev)
{
	struct cadence_edac_priv *priv;

	dev_dbg(&pdev->dev, "cadence_edac_mc_init\n");

	mci->pdev = &pdev->dev;
	priv = mci->pvt_info;
	platform_set_drvdata(pdev, mci);

	mci->mtype_cap = MEM_FLAG_DDR3 | MEM_FLAG_DDR2 | MEM_FLAG_DDR;
	mci->edac_ctl_cap = EDAC_FLAG_NONE | EDAC_FLAG_SECDED;
	mci->scrub_cap = SCRUB_HW_TUNABLE;
	mci->scrub_mode = SCRUB_HW_TUNABLE;

	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->ctl_name = "cadence_ddr_controller";
	mci->dev_name = "cadence_edac";
	mci->mod_name = "-";
	if (edac_op_state == EDAC_OPSTATE_POLL)
		mci->edac_check = cadence_edac_mc_check;

	cadence_edac_init_csrows(mci);

	/* Enable auto correction of error while a CPU read out of scrubbing*/
	regmap_field_write(priv->rmf[ECC_WRITEBACK_EN], 1);

	select_scrubbing_mode(mci);
}

/**
 * cadence_edac_mc_probe - Check controller and bind driver
 * @pdev:	Pointer to the platform_device struct
 *
 * Probes a specific controller instance for binding with the driver.
 *
 * Return: 0 if the controller instance was successfully bound to the
 * driver; otherwise, < 0 on error.
 */
static int cadence_edac_mc_probe(struct platform_device *pdev)
{
	struct mem_ctl_info *mci;
	struct edac_mc_layer layers[1];
	struct cadence_edac_priv *priv;
	int rc;
	u32 ecc_en, status;
	struct device_node *mem_node;
	struct resource *res;

	layers[0].type = EDAC_MC_LAYER_ALL_MEM;
	layers[0].size = 1;
	layers[0].is_virt_csrow = true;

	if (WARN_ON(cadence_mci))
		return -EINVAL;

	mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers,
			    sizeof(struct cadence_edac_priv));
	if (!mci) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed memory allocation for mc instance\n");
		return -ENOMEM;
	}
	priv = mci->pvt_info;

	/* make sure error reporting method is sane */
	switch (edac_op_state) {
	case EDAC_OPSTATE_POLL:
	case EDAC_OPSTATE_INT:
		break;
	default:
		edac_op_state = EDAC_OPSTATE_POLL;
		break;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbahn");
	priv->mmio = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->mmio)) {
		rc = PTR_ERR(priv->mmio);
		goto free_edac_mc;
	}

	mem_node = of_find_node_by_type(NULL, "memory");
	if (!mem_node) {
		rc = -ENXIO;
		goto free_edac_mc;
	}
	rc = of_address_to_resource(mem_node, 0, &priv->mem_res);
	if (rc)
		goto free_edac_mc;
	of_node_put(mem_node);

	rc = cadence_edac_alloc_regmap(&pdev->dev, priv);
	if (rc)
		goto free_edac_mc;

	/*
	 * Check that ECC has been enabled by bootloader (TP)
	 */
	regmap_field_read(priv->rmf[ECC_EN], &ecc_en);
	if (ecc_en != 1) {
		dev_err(&pdev->dev, "ECC wasn't previously enabled\n");
		rc = -ENOENT;
		goto free_edac_mc;
	}

	cadence_edac_mc_init(mci, pdev);

	rc = edac_mc_add_mc_with_groups(mci, cadence_dev_groups);
	if (rc) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed to register with EDAC core\n");
		goto free_edac_mc;
	}

	cadence_mci = mci;

	/*
	 * unmask relevant irq
	 */
	writel_relaxed(~(IT_MASK_ALL |
			 IT_ERROR_RECOVERABLE |
			 IT_SECOND_ERROR_RECOVERABLE |
			 IT_ERROR_NON_RECOVERABLE |
			 IT_SECOND_ERROR_NON_RECOVERABLE |
			 IT_END_OF_SCRUB),
			priv->mmio + INT_MASK);

	/* Ack previously existing IT */
	status = ioread32(priv->mmio + INT_STATUS);
	writel(status, priv->mmio + INT_ACK);

	if (edac_op_state == EDAC_OPSTATE_INT) {
		priv->irq = platform_get_irq(pdev, 0);
		rc = devm_request_irq(&pdev->dev, priv->irq, cadence_edac_isr,
				      0, "[EDAC] DDR err", mci);
		if (rc < 0) {
			edac_mc_printk(mci, KERN_ERR,
				       "Unable to alloc irq %d for DDR ECC Err\n",
				       priv->irq);
			rc = -ENODEV;
			goto del_edac_mc;
		}
	}

	start_scrubbing(mci);

	return rc;

del_edac_mc:
	edac_mc_del_mc(&pdev->dev);
free_edac_mc:
	edac_mc_free(mci);

	return rc;
}

/**
 * cadence_edac_mc_remove - Unbind driver from controller
 * @pdev:	Pointer to the platform_device struct
 *
 * Return: Unconditionally 0
 */
static int cadence_edac_mc_remove(struct platform_device *pdev)
{
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);

	cadence_mci = NULL;

	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id cadence_edac_match[] = {
	{ .compatible = "st,cadence-ddr-edac", },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, cadence_edac_match);

static struct platform_driver cadence_edac_mc_driver = {
	.driver = {
		   .name = "cadence-edac",
		   .of_match_table = cadence_edac_match,
		   },
	.probe = cadence_edac_mc_probe,
	.remove = cadence_edac_mc_remove,
};
module_platform_driver(cadence_edac_mc_driver);

int param_set_scrub_interval(const char *val, const struct kernel_param *kp)
{
	int rc;
	rc = param_set_uint(val, kp);
	if (cadence_mci)
		select_scrubbing_mode(cadence_mci);
	return rc;
}

static int param_set_idle_scrub(const char *val, const struct kernel_param *kp)
{
	int rc;

	rc = param_set_bool(val, kp);
	if (cadence_mci)
		select_scrubbing_mode(cadence_mci);
	return rc;
}

const struct kernel_param_ops param_ops_idle_scrub = {
	.set = param_set_idle_scrub,
	.get = param_get_uint,
};

const struct kernel_param_ops param_ops_scrub_inter = {
	.set = param_set_scrub_interval,
	.get = param_get_uint,
};

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("Cadence DDR ECC driver");
MODULE_LICENSE("GPL v2");
module_param_cb(idle_scrubbing, &param_ops_idle_scrub, &idle_scrubbing, 0644);
MODULE_PARM_DESC(idle_scrubbing, "Enable the scrubbing while idle");
module_param_cb(scrub_interval, &param_ops_scrub_inter, &scrub_interval, 0644);
MODULE_PARM_DESC(scrub_interval,
		 "Define interval (in nb clock) between 2 scrubbing dummy read in non idle mode");
module_param(edac_op_state, int, 0444);
MODULE_PARM_DESC(edac_op_state,
		 "EDAC error reporting state: 0=Poll, 2=Interrupt");
