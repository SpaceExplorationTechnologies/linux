/**
 * irq-sx-catson-2.00 is a SpaceX standardized interrupt controller to expand
 * the number of interrupts from SpaceX IP to the GIC in Catson 2.0.
 *
 * Interrupts take a single cell which specify the bit position of the interrupt
 * in the expander block.
 *
 * The expander block consists of two registers.
 *   - A status register containing active interrupts, and which is w1c
 *   - A control register.  Interrupts are generated when (status & control).
 *
 * Documentation can be found here:
 * https://stash/projects/SAT_RTL/repos/rtl_essentials/browse/interrupt/irq_expander.docx
 */

#include <linux/bug.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>

#define CATSON_IRQ_REG_STATUS			0x00	/* IFG */
#define CATSON_IRQ_REG_TEST			0x04	/* IFG_TEST */
#define CATSON_IRQ_REG_IEN			0x08
#define CATSON_IRQ_REG_IEN_SET			0x0C
#define CATSON_IRQ_REG_IEN_CLEAR		0x10

struct catson_irq_chip {
	/*
	 * OF based address that registers are located at.
	 */
	void __iomem        *base;

	/*
	 * The domain that all interrupts in the expander belong to.
	 */
	struct irq_domain   *root_domain;

	/*
	 * Which interrupts are currently enabled.  Saves the overhead of having
	 * to read for a RMW.
	 */
	unsigned long		intr_mask;

	/*
	 * The OF based number of interrupts in this control block.
	 */
	u32         num_intrs;
};

/**
 * catson_irq_write() - Helper function to write a catson register.
 *
 * @priv:       Device to write to.
 * @offset:     Register to write.
 * @data:       Data to write.
 */
static inline void catson_irq_write(struct catson_irq_chip *priv,
				    u32 offset, u32 data)
{
	iowrite32(data, priv->base + offset);
}

/**
 * catson_irq_read() - Helper function to read a catson register.
 *
 * @offset:     Register to write.
 *
 * This function returns data read from register.
 */
static inline u32 catson_irq_read(struct catson_irq_chip *priv,
				  u32 offset)
{
	return ioread32(priv->base + offset);
}

/**
 * catson_irq_unmask() - Linux callback to unmask or enable an interrupt.
 *
 * @d:           irq_data to act upon.
 */
static void catson_irq_unmask(struct irq_data *d)
{
	struct catson_irq_chip *priv = irq_data_get_irq_chip_data(d);

	/*
	 * Because of the sticky nature of the ack, the ack performed in
	 * catson_irq_mask_ack has been lost, because the client had not yet
	 * handled the interrupt. Go ahead and clear the ack here, because if
	 * the level is still high, the sticky ack bit will be immediately set
	 * again.
	 *
	 * Note that currently only level-based interrupts are supported, so
	 * this doesn't actually do anything.
	 */
	if (irqd_is_level_type(d))
		catson_irq_write(priv, CATSON_IRQ_REG_STATUS, 1 << d->hwirq);

	set_bit(d->hwirq, &priv->intr_mask);
	catson_irq_write(priv, CATSON_IRQ_REG_IEN_SET, 1 << d->hwirq);
}

/**
 * catson_irq_mask() - Linux callback function to mask or disable an interrupt.
 *
 * @d:        irq_data to act upon.
 */
static void catson_irq_mask(struct irq_data *d)
{
	struct catson_irq_chip *priv = irq_data_get_irq_chip_data(d);

	catson_irq_write(priv, CATSON_IRQ_REG_IEN_CLEAR, 1 << d->hwirq);
	clear_bit(d->hwirq, &priv->intr_mask);
}

/**
 * catson_irq_ack() - Linux callback function to acknowledge an interrupt.
 *
 * @d:        irq_data to act upon.
 */
static void catson_irq_ack(struct irq_data *d)
{
	struct catson_irq_chip *priv = irq_data_get_irq_chip_data(d);

	catson_irq_write(priv, CATSON_IRQ_REG_STATUS, 1 << d->hwirq);
}

/**
 * catson_irq_mask_ack() - Linux callback to mask then ack an interrupt.
 *
 * @d:      irq_data to act upon.
 */
static void catson_irq_mask_ack(struct irq_data *d)
{
	catson_irq_mask(d);
	catson_irq_ack(d);
}

static struct irq_chip catson_ictlr_dev = {
	.name = "catson_ictlr",
	.irq_unmask = catson_irq_unmask,
	.irq_mask = catson_irq_mask,
	.irq_ack = catson_irq_ack,
	.irq_mask_ack = catson_irq_mask_ack,
};

/**
 * catson_map - Linux callback function for registering a virtual to
 *      physical interrupt.
 * @d:      The IRQ domain to register to.
 * @irq:    The Linux IRQ number.
 * @hw:     The phyiscal hardware number.
 *
 * This function returns 0 on success.
 */
static int catson_map(struct irq_domain *d, unsigned int irq,
		      irq_hw_number_t hw)
{
	struct catson_irq_chip *priv = d->host_data;

	irq_set_chip_and_handler(irq,
				 &catson_ictlr_dev,
				 handle_level_irq);

	irq_set_status_flags(irq, IRQ_LEVEL);

	irq_set_chip_data(irq, priv);

	return 0;
}

static const struct irq_domain_ops catson_irq_domain_ops = {
	.xlate = irq_domain_xlate_onecell,
	.map = catson_map,
};

/**
 * catson_irq_handler - Linux callback for handling an expander interrupt.
 * @desc:   The descriptor for the interrupt that occurred.
 */
static void catson_irq_handler(struct irq_desc *desc)
{
	unsigned int hwirq, irq, lirq;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct catson_irq_chip *priv =
			irq_data_get_irq_handler_data(&desc->irq_data);

	chained_irq_enter(chip, desc);
	hwirq = catson_irq_read(priv, CATSON_IRQ_REG_STATUS) & priv->intr_mask;

	for (irq = 0; irq < priv->num_intrs; irq++) {
		if ((1 << irq) & hwirq) {
			lirq = irq_find_mapping(priv->root_domain, irq);

			generic_handle_irq(lirq);
		}
	}

	chained_irq_exit(chip, desc);
}

/**
 * catson_ictlr_of_common_init - Probe function for Catson interrupt expander.
 * @intc:   The device node associated with the expander.
 * @parent: The parent of the expander.
 *
 * This function returns 0 on success.
 *
 */
static int __init catson_ictlr_of_init(struct device_node *intc,
				       struct device_node *parent)
{
	struct catson_irq_chip *priv;
	int ret, irq;

	/*
	 * This driver is by definition an expander.  Therefore, it expects
	 * to have a parent.
	 */
	if (!parent)
		return -EINVAL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_iomap(intc, 0);
	if (WARN_ON(!priv->base)) {
		ret = -EINVAL;
		goto err_alloc;
	}

	ret = of_property_read_u32(intc, "sx,intr-count", &priv->num_intrs);
	if (ret < 0) {
		pr_err("irq-catson: unable to read sx,intr-count\n");
		goto err_map;
	}

	if (priv->num_intrs > 32) {
		pr_err("irq-catson: invalid interrupt count %d\n",
		       priv->num_intrs);
		goto err_map;
	}

	/*
	 * Disable all interrupts by default.
	 */
	priv->intr_mask = 0;
	catson_irq_write(priv, CATSON_IRQ_REG_IEN, 0);
	catson_irq_write(priv, CATSON_IRQ_REG_STATUS, ~0);

	priv->root_domain = irq_domain_add_linear(intc,
						  priv->num_intrs,
						  &catson_irq_domain_ops,
						  priv);
	if (!priv->root_domain) {
		pr_err("irq-catson: unable to allocate linear irq\n");
		goto err_map;
	}

	irq = irq_of_parse_and_map(intc, 0);
	if (!irq) {
		pr_err("Invalid device tree entry does not contain interrupts\n");
		ret = -EINVAL;
		goto err_domain;
	}

	irq_set_chained_handler_and_data(irq,
					 catson_irq_handler,
					 priv);

	return 0;

err_domain:
	irq_domain_remove(priv->root_domain);
err_map:
	iounmap(priv->base);
err_alloc:
	kfree(priv);

	return ret;
}

IRQCHIP_DECLARE(catson_ictlr,
		"sx,catson-intc-2.00",
		catson_ictlr_of_init);
