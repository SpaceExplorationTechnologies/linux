/**
 * irq-catson-modem is a SpaceX standardized interrupt controller to expand
 * the number of interrupts from SpaceX Modems to the GIC in Catson 2.0.
 *
 * Interrupts take a single cell which specify the bit position of the interrupt
 * in the expander block.
 *
 * The expander block consists of an arbitrary number of register pairs.
 *   - A status register containing active interrupts, and which is w1c
 *   - A control register.  Interrupts are generated when (status & control).
 *
 * The interrupt is calculated as the register pair number * 32 as all registers
 * are 32 bits even if they do not contain that many valid interrupts.
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
#include <linux/debugfs.h>

#define CATSON_MIRQ_MAX_REGS	8
#define CATSON_MIRQ_PER_REG	32

struct catson_mirq_chip {
	/*
	 * OF based address that registers are located at.
	 */
	void __iomem		*base;

	/*
	 * The domain that all interrupts in the expander belong to.
	 */
	struct irq_domain	*root_domain;

	/*
	 *	The irq of the parent interrupt
	 */
	unsigned int parent_irq;

	/*
	 * A spinlock is necessary because there is only a single control
	 * rather than separate enable and disable registers.  It must be a
	 * raw spinlock due to RT.
	 */
	raw_spinlock_t		lock;

	/*
	 * The size of the reg array.
	 */
	u32			reg_count;

	/*
	 * Status and control offset into memory mapped region.
	 */
	struct {
		u32 enabled;
		void *status;
		void *control;
	} reg[CATSON_MIRQ_MAX_REGS];
};

/*
 * A static global used to store the dentry for the debugfs directory in which the kernel
 * can provide the parent irq number to user userspace so that userspce can control the
 * core affinity of the top half of the catson-modem interrupts
 */
static struct dentry *catson_mirq_debugfs_base_dir;

/**
 * catson_mirq_unmask() - Linux callback to unmask or enable an interrupt.
 *
 * @d:		     irq_data to act upon.
 */
static void catson_mirq_unmask(struct irq_data *d)
{
	struct catson_mirq_chip *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	unsigned long reg_offset = d->hwirq / 32;
	unsigned long bit_offset = 1 << (d->hwirq % 32);

	raw_spin_lock_irqsave(&priv->lock, flags);

	/*
	 * Because of the sticky nature of the ack, the ack performed in
	 * catson_mirq_mask_ack has been lost, because the client had not yet
	 * handled the interrupt. Go ahead and clear the ack here, because if
	 * the level is still high, the sticky ack bit will be immediately set
	 * again.
	 *
	 * Note that currently only level-based interrupts are supported, so
	 * this doesn't actually do anything.
	 */
	if (irqd_is_level_type(d))
		iowrite32(bit_offset, priv->reg[reg_offset].status);

	priv->reg[reg_offset].enabled |= bit_offset;
	iowrite32(priv->reg[reg_offset].enabled, priv->reg[reg_offset].control);

	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

/**
 * catson_mirq_mask() - Linux callback function to mask or disable an interrupt.
 *
 * @d:		  irq_data to act upon.
 */
static void catson_mirq_mask(struct irq_data *d)
{
	struct catson_mirq_chip *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	unsigned long reg_offset = d->hwirq / 32;
	unsigned long bit_offset = 1 << (d->hwirq % 32);

	raw_spin_lock_irqsave(&priv->lock, flags);
	priv->reg[reg_offset].enabled &= ~bit_offset;
	iowrite32(priv->reg[reg_offset].enabled, priv->reg[reg_offset].control);
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

/**
 * catson_mirq_ack() - Linux callback function to acknowledge an interrupt.
 *
 * @d:		  irq_data to act upon.
 */
static void catson_mirq_ack(struct irq_data *d)
{
	struct catson_mirq_chip *priv = irq_data_get_irq_chip_data(d);
	unsigned long reg_offset = d->hwirq / 32;
	unsigned long bit_offset = 1 << (d->hwirq % 32);

	iowrite32(bit_offset, priv->reg[reg_offset].status);
}

/**
 * catson_mirq_mask_ack() - Linux callback to mask then ack an interrupt.
 *
 * @d:		irq_data to act upon.
 */
static void catson_mirq_mask_ack(struct irq_data *d)
{
	struct catson_mirq_chip *priv = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	unsigned long reg_offset = d->hwirq / 32;
	unsigned long bit_offset = 1 << (d->hwirq % 32);

	raw_spin_lock_irqsave(&priv->lock, flags);

	priv->reg[reg_offset].enabled &= ~bit_offset;
	iowrite32(priv->reg[reg_offset].enabled, priv->reg[reg_offset].control);
	iowrite32(bit_offset, priv->reg[reg_offset].status);

	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static struct irq_chip catson_ictlr_dev = {
	.name = "catson_mictlr",
	.irq_unmask = catson_mirq_unmask,
	.irq_mask = catson_mirq_mask,
	.irq_ack = catson_mirq_ack,
	.irq_mask_ack = catson_mirq_mask_ack,
};

/**
 * irq_domain_catson_xlate - Linux callback function translate dev tree to domain info
 * @d:			The IRQ domain.
 * @ctrlr:		The device node.
 * @intspec:	The interrupt spec which contains the phyiscal hardware irq.
 * @intsize:	The size of the intspec.
 * @out_hwirq:	The hardware irq being returned.
 * @out_type:	The interrupt type being returned.
 *
 * This function returns 0 on success.
 */
static int irq_domain_catson_xlate(struct irq_domain *d, struct device_node *ctrlr,
			const u32 *intspec, unsigned int intsize,
			unsigned long *out_hwirq, unsigned int *out_type)
{
	if (WARN_ON(intsize < 1))
		return -EINVAL;
	*out_hwirq = intspec[0];
	*out_type = IRQ_TYPE_LEVEL_HIGH;
	return 0;
}

/**
 * catson_map - Linux callback function for registering a virtual to
 *		physical interrupt.
 * @d:		The IRQ domain to register to.
 * @irq:	The Linux IRQ number.
 * @hw:		The phyiscal hardware number.
 *
 * This function returns 0 on success.
 */
static int catson_map(struct irq_domain *d,
		      unsigned int irq,
		      irq_hw_number_t hw)
{
	struct catson_mirq_chip *priv = d->host_data;
	struct dentry *dbg_irq_dir;
	char name[8];

	irq_set_chip_and_handler(irq,
				 &catson_ictlr_dev,
				 handle_level_irq);

	irq_set_chip_data(irq, priv);

	/* if needed create /sys/kernel/debug/catson_mirq/ subdir */
	if (catson_mirq_debugfs_base_dir == NULL) {
		catson_mirq_debugfs_base_dir = debugfs_create_dir("catson_mirq", NULL);
		if (IS_ERR_OR_NULL(catson_mirq_debugfs_base_dir)) {
			pr_err("error creating irq base dir debugfs entry\n");
			catson_mirq_debugfs_base_dir = NULL;
			return 0;
		}
	}

	/* now create /sys/kernel/debug/catson_mirq/<irq>/ subdir */
	sprintf(name, "%d", irq);
	dbg_irq_dir = debugfs_create_dir(name, catson_mirq_debugfs_base_dir);
	if (IS_ERR_OR_NULL(dbg_irq_dir)) {
		pr_err("error creating irq directory\n");
		return 0;
	}

	/* now create /sys/kernel/debug/catson_mirq/<irq>/parent_irq file */
	debugfs_create_u32("parent_irq", 0400, dbg_irq_dir, &priv->parent_irq);

	return 0;
}

static const struct irq_domain_ops catson_mirq_domain_ops = {
	.xlate = irq_domain_catson_xlate,
	.map = catson_map,
};

/**
 * catson_mirq_handler - Linux callback for handling an expander interrupt.
 * @desc:	The descriptor for the interrupt that occurred.
 */
static void catson_mirq_handler(struct irq_desc *desc)
{
	unsigned int hwirq, hwirq_mask, lirq, irqreg;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct catson_mirq_chip *priv =
			irq_data_get_irq_handler_data(&desc->irq_data);

	chained_irq_enter(chip, desc);

	for (irqreg = 0; irqreg < priv->reg_count; irqreg++) {
		hwirq_mask = priv->reg[irqreg].enabled &
			 ioread32(priv->reg[irqreg].status);

		while ((hwirq = fls(hwirq_mask))) {
			hwirq--; /* FLS to bit position */
			hwirq_mask &= ~(1 << hwirq);
			lirq = irq_find_mapping(priv->root_domain,
						hwirq + irqreg * CATSON_MIRQ_PER_REG);
			generic_handle_irq(lirq);
		}
	}

	chained_irq_exit(chip, desc);
}

/**
 * catson_mictlr_of_init - Probe function for Catson modem interrupt expander.
 * @intc:	The device node associated with the expander.
 * @parent:	The parent of the expander.
 *
 * This function returns 0 on success.
 *
 */
static int __init catson_modem_ictlr_of_init(struct device_node *intc,
					     struct device_node *parent)
{
	struct catson_mirq_chip *priv;
	int ret, irq;
	u32 registers[CATSON_MIRQ_MAX_REGS];
	int i;
	struct resource res;

	/*
	 * This driver is by definition an expander.  Therefore, it expects
	 * to have a parent.
	 */
	if (!parent)
		return -EINVAL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	raw_spin_lock_init(&priv->lock);

	priv->base = of_iomap(intc, 0);
	if (WARN_ON(!priv->base)) {
		ret = -EINVAL;
		goto err_alloc;
	}

	if (of_address_to_resource(intc, 0, &res)) {
		pr_err("Failed to get io resource\n");
		ret = -EINVAL;
		goto err_alloc;
	}

	priv->reg_count = of_property_read_variable_u32_array(intc, "irq-regs",
							registers, 1,
							CATSON_MIRQ_MAX_REGS);
	if (priv->reg_count <= 0) {
		pr_err("Invalid device tree does not contain registers\n");
		ret = -EINVAL;
		goto err_alloc;
	}

	for (i = 0; i < priv->reg_count; i++) {
		if (registers[i] + sizeof(u32) * 2 >= resource_size(&res)) {
			pr_err("Invalid register offset\n");
			ret = -EINVAL;
			goto err_alloc;
		}

		priv->reg[i].status = priv->base + registers[i];
		priv->reg[i].control = priv->reg[i].status + sizeof(u32);
		writel(0, priv->reg[i].control);
		writel(~0, priv->reg[i].status);
	}

	priv->root_domain = irq_domain_add_linear(intc,
						  priv->reg_count *
							CATSON_MIRQ_PER_REG,
						  &catson_mirq_domain_ops,
						  priv);
	if (!priv->root_domain) {
		pr_err("irq-catson: unable to allocate linear irq\n");
		goto err_alloc;
	}

	irq = irq_of_parse_and_map(intc, 0);
	if (!irq) {
		pr_err("Invalid device tree entry does not contain interrupts\n");
		ret = -EINVAL;
		goto err_alloc;
	}

	/* save the irq number for this interrupt's parent */
	priv->parent_irq = irq;

	irq_set_chained_handler_and_data(irq, catson_mirq_handler, priv);

	return 0;

err_alloc:
	kfree(priv);

	return ret;
}

IRQCHIP_DECLARE(catson_ictlr,
		"sx,catson-modem-intc-2.00",
		catson_modem_ictlr_of_init);
