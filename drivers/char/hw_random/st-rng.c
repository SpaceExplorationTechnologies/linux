// SPDX-License-Identifier: GPL-2.0-only
/*
 * ST Random Number Generator Driver ST's Platforms
 *
 * Author: Pankaj Dev: <pankaj.dev@st.com>
 *         Lee Jones <lee.jones@linaro.org>
 *
 * Copyright (C) 2015 STMicroelectronics (R&D) Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Registers */
#define ST_RNG_STATUS_REG		0x20
#define ST_RNG_DATA_REG			0x24

#define SX_ZYNQMP_RNG_DATA_REG		0xC0

/* Registers fields */
#define ST_RNG_STATUS_BAD_SEQUENCE	BIT(0)
#define ST_RNG_STATUS_BAD_ALTERNANCE	BIT(1)
#define ST_RNG_STATUS_FIFO_FULL		BIT(5)

#define ST_RNG_SAMPLE_SIZE		2 /* 2 Byte (16bit) samples */
#define ST_RNG_FIFO_DEPTH		4
#define ST_RNG_FIFO_SIZE		(ST_RNG_FIFO_DEPTH * ST_RNG_SAMPLE_SIZE)

/* ZynqMP SMCs */
#define GET_TRNG_STATUS_SVC	0xC3001007

#define SX_TRNG_LENGTH		0x1000

/*
 * Samples are documented to be available every 0.667us, so in theory
 * the 4 sample deep FIFO should take 2.668us to fill.  However, during
 * thorough testing, it became apparent that filling the FIFO actually
 * takes closer to 12us.  We then multiply by 2 in order to account for
 * the lack of udelay()'s reliability, suggested by Russell King.
 */
#define ST_RNG_FILL_FIFO_TIMEOUT	(12 * 2)

struct st_rng_data {
	void __iomem	*base;
	struct clk	*clk;
	struct hwrng	ops;
	u64		data_reg_offset;
};

struct rng_params_t {
	int (*read_function)(struct hwrng *rng, void *data,
			     size_t max, bool wait);
	u64 data_reg_offset;
};

static int st_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct st_rng_data *ddata = (struct st_rng_data *)rng->priv;
	u32 status;
	int i;

	if (max < sizeof(u16))
		return -EINVAL;

	/* Wait until FIFO is full - max 4uS*/
	for (i = 0; i < ST_RNG_FILL_FIFO_TIMEOUT; i++) {
		status = readl_relaxed(ddata->base + ST_RNG_STATUS_REG);
		if (status & ST_RNG_STATUS_FIFO_FULL)
			break;
		udelay(1);
	}

	if (i == ST_RNG_FILL_FIFO_TIMEOUT)
		return 0;

	for (i = 0; i < ST_RNG_FIFO_SIZE && i < max; i += 2)
		*(u16 *)(data + i) =
			readl_relaxed(ddata->base + ST_RNG_DATA_REG);

	return i;	/* No of bytes read */
}

/*
 * It takes 0.667 us to deliver a new random value. It is possible that
 * several clients are reading the same register so waiting 0.667 might
 * not be enough to get a new valid value since it might already have
 * been taken from another client. In case of the requestor accept to wait
 * we do a retry of up to 10 times before giving up and returning the
 * retrieved number of bytes
 */
#define RNG_RETRY 10
#define ST_RNG_GLLCFF_FIFO_EMPTY	BIT(16)
#define ST_RNG_GLLCFF_BAD_ENTROPY	BIT(17)

static int st_rng_gllcff_read(struct hwrng *rng, void *data, size_t max,
				bool wait)
{
	struct st_rng_data *ddata = (struct st_rng_data *)rng->priv;
	int bytes_read = 0;
	u32 rd;
	int i;
	u8 *buffer = (u8 *)data;

	if (max < sizeof(u16))
		return -EINVAL;

	/*
	 * If you read back the wrong offset and it comes back as all zeros,
	 * that will be considered valid. As a sanity check, we confirm that an
	 * offset was configured.
	 */
	if (!ddata->data_reg_offset)
		return -ENODEV;

	/*
	 * The RNG provides 16 bit values,
	 * so we read back as many u16s as will fit in data.
	 */
	while (bytes_read < (max-1)) {
		rd = readl_relaxed(ddata->base + ddata->data_reg_offset);
		if ((rd & ST_RNG_GLLCFF_FIFO_EMPTY) && wait) {
			for (i = 0; i < RNG_RETRY; i++) {
				udelay(1);
				cpu_relax();
				rd = readl_relaxed(ddata->base +
							ddata->data_reg_offset);
				if (!(rd & ST_RNG_GLLCFF_FIFO_EMPTY))
					break;
			}
		}

		/* Consider the BAD_ENTROPY as an error */
		if (rd & ST_RNG_GLLCFF_BAD_ENTROPY) {
			return -EIO;
		}

		/* If fifo is empty */
		if (rd & ST_RNG_GLLCFF_FIFO_EMPTY)
			break;

		/*
		 * The random value is the lower 16 bits
		 * of the the data register
		 */
		buffer[bytes_read]   = (u8)((rd >> 0) & 0xFF);
		buffer[bytes_read+1] = (u8)((rd >> 8) & 0xFF);

		bytes_read += sizeof(u16);
	}

	return bytes_read;
}

/**
 * smc_read_trng_status - helper function that makes an uncached SMC
 * to fetch the TRNG status.
 *
 * @param dev pointer to the platform device.
 * @param status pointer to a u64 where the trng status will be saved
 * @param base_addr pointer to a u64 where the trng base addr will be saved
 *
 * Returns 0 on success, or -EIO on failure.
 */
static int smc_read_trng_status(struct device *dev, u64 *status, u64 *base_addr)
{
	struct arm_smccc_res res = {};

	arm_smccc_smc(GET_TRNG_STATUS_SVC, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != 0) {
		dev_err(dev, "Failed to read TRNG status with error %lx\n",
			res.a0);
		return -EIO;
	}

	*status = res.a1;
	*base_addr = res.a2;
	return 0;
}

static const struct rng_params_t st_rng_params = {
	.read_function = st_rng_read,
};

static const struct rng_params_t gllcff_rng_params = {
	.read_function = st_rng_gllcff_read,
	.data_reg_offset = ST_RNG_DATA_REG
};

static const struct rng_params_t zynqmp_rng_params = {
	.read_function = st_rng_gllcff_read,
	.data_reg_offset = SX_ZYNQMP_RNG_DATA_REG
};

static const struct of_device_id st_rng_match[] = {
	{ .compatible = "st,rng", .data = &st_rng_params },
	{ .compatible = "st,rng-gllcff", .data = &gllcff_rng_params },
	{ .compatible = "sx,zynqmp_trng-1.00.a", .data = &zynqmp_rng_params },
	{},
};
MODULE_DEVICE_TABLE(of, st_rng_match);

static int st_rng_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	struct st_rng_data *ddata;
	struct clk *clk;
	void __iomem *base;
	u16 tmp[1];
	int ret;
	u64 tz_base, tz_trng_status;
	struct rng_params_t *params;
	struct resource *res;

	match = of_match_node(st_rng_match, np);
	if (!match)
		return -ENODEV;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	clk = NULL;

#ifndef CONFIG_SPACEX
	base = devm_platform_ioremap_resource(pdev, 0);
#else
	if (strcmp(match->compatible, "sx,zynqmp_trng-1.00.a") == 0) {
		/*
		 * ZynqMP doesn't define the TRNG base in the device tree,
		 * instead we get it from the ATF. It also requires no clock
		 * config, since the ATF handles all init.
		 *
		 * This SMC also returns the RNG status from the ATF,
		 * which is saved for use later.
		 */
		if (smc_read_trng_status(&pdev->dev, &tz_trng_status, &tz_base))
			return -ENODEV;

		if (tz_trng_status) {
			dev_info(&pdev->dev, "TRNG returned code %llu.\n",
				 tz_trng_status);
			return -ENXIO;
		}

		base = devm_ioremap(&pdev->dev, tz_base, SX_TRNG_LENGTH);
	} else {
		/*
		 * Catson stores its RNG address in the device tree,
		 * and requires clock config.
		 */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		base = devm_ioremap_resource(&pdev->dev, res);
	}
#endif
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);
	if (clk) {
		ret = clk_prepare_enable(clk);
		if (ret)
			return ret;
	}

	params = (struct rng_params_t *)match->data;

	ddata->ops.priv	= (unsigned long)ddata;
	ddata->ops.read	= params->read_function;
	ddata->ops.name	= pdev->name;
	ddata->base	= base;
	ddata->clk	= clk;
	ddata->data_reg_offset = params->data_reg_offset;

	if (strcmp(match->compatible, "st,rng-gllcff") == 0) {
		/*
		 * While not documented, testing revealed that the first value
		 * in the Catson TRNG FIFO is always zeros, so we discard it.
		 *
		 * Note that it's important to call this before invoking
		 * devm_hwrng_register, since that will invoke
		 * add_early_randomness.
		 */
		st_rng_gllcff_read(&ddata->ops, tmp, sizeof(u16), true);
	}

	dev_set_drvdata(&pdev->dev, ddata);

	ret = devm_hwrng_register(&pdev->dev, &ddata->ops);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register HW RNG\n");
		if (clk)
			clk_disable_unprepare(clk);
		return ret;
	}

	dev_info(&pdev->dev, "Successfully registered HW RNG\n");

	return 0;
}

static int st_rng_remove(struct platform_device *pdev)
{
	struct st_rng_data *ddata = dev_get_drvdata(&pdev->dev);

	if (ddata->clk)
		clk_disable_unprepare(ddata->clk);

	return 0;
}

static struct platform_driver st_rng_driver = {
	.driver = {
		.name = "st-hwrandom",
		.of_match_table = of_match_ptr(st_rng_match),
	},
	.probe = st_rng_probe,
	.remove = st_rng_remove
};

module_platform_driver(st_rng_driver);

MODULE_AUTHOR("Pankaj Dev <pankaj.dev@st.com>");
MODULE_LICENSE("GPL v2");
