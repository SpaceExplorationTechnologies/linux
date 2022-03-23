/*
 * Realtek RTL8366 SMI interface driver
 *
 * Copyright (C) 2009-2010 Gabor Juhos <juhosg@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/rtl8366.h>
#include <linux/of_mdio.h>

#ifdef CONFIG_RTL8366_SMI_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include "rtl8366_smi.h"

#define RTL8366_SMI_ACK_RETRY_COUNT         5

#define RTL8366_SMI_HW_STOP_DELAY		25	/* msecs */
#define RTL8366_SMI_HW_START_DELAY		100	/* msecs */

/***********************/
/*  MDC/MDIO porting   */
/***********************/
/* define the PHY ID currently used */
#define MDC_MDIO_PHY_ID     0  /* PHY ID 0 or 29 */

#define MDC_MDIO_CTRL0_REG          31
#define MDC_MDIO_START_REG          29
#define MDC_MDIO_CTRL1_REG          21
#define MDC_MDIO_ADDRESS_REG        23
#define MDC_MDIO_DATA_WRITE_REG     24
#define MDC_MDIO_DATA_READ_REG      25
#define MDC_MDIO_PREAMBLE_LEN       32

#define MDC_MDIO_START_OP          0xFFFF
#define MDC_MDIO_ADDR_OP           0x000E
#define MDC_MDIO_READ_OP           0x0001
#define MDC_MDIO_WRITE_OP          0x0003

/* MDC/MDIO, redefine/implement the following Macro */
#define MDC_MDIO_WRITE(mii, preamble_length, phy_id, reg_id, data) \
		((mii)->write((mii), phy_id, reg_id, data))

#define MDC_MDIO_READ(mii, preamble_length, phy_id, reg_id, pdata) \
		(*(pdata) = (mii)->read((mii), phy_id, reg_id))

static inline void rtl8366_smi_clk_delay(struct rtl8366_smi *smi)
{
	ndelay(smi->clk_delay);
}

static void rtl8366_smi_start(struct rtl8366_smi *smi)
{
	unsigned int sda = smi->gpio_sda;
	unsigned int sck = smi->gpio_sck;

	/* Set GPIO pins to output mode, with initial state:
	 * SCK = 0, SDA = 1
	 */
	gpio_direction_output(sck, 0);
	gpio_direction_output(sda, 1);
	rtl8366_smi_clk_delay(smi);

	/* CLK 1: 0 -> 1, 1 -> 0 */
	gpio_set_value(sck, 1);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sck, 0);
	rtl8366_smi_clk_delay(smi);

	/* CLK 2: */
	gpio_set_value(sck, 1);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sda, 0);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sck, 0);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sda, 1);
}

static void rtl8366_smi_stop(struct rtl8366_smi *smi)
{
	unsigned int sda = smi->gpio_sda;
	unsigned int sck = smi->gpio_sck;

	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sda, 0);
	gpio_set_value(sck, 1);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sda, 1);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sck, 1);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sck, 0);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sck, 1);

	/* add a click */
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sck, 0);
	rtl8366_smi_clk_delay(smi);
	gpio_set_value(sck, 1);

	/* set GPIO pins to input mode */
	gpio_direction_input(sda);
	gpio_direction_input(sck);
}

static void rtl8366_smi_write_bits(struct rtl8366_smi *smi, u32 data, u32 len)
{
	unsigned int sda = smi->gpio_sda;
	unsigned int sck = smi->gpio_sck;

	for (; len > 0; len--) {
		rtl8366_smi_clk_delay(smi);

		/* prepare data */
		gpio_set_value(sda, !!(data & (1 << (len - 1))));
		rtl8366_smi_clk_delay(smi);

		/* clocking */
		gpio_set_value(sck, 1);
		rtl8366_smi_clk_delay(smi);
		gpio_set_value(sck, 0);
	}
}

static void rtl8366_smi_read_bits(struct rtl8366_smi *smi, u32 len, u32 *data)
{
	unsigned int sda = smi->gpio_sda;
	unsigned int sck = smi->gpio_sck;

	gpio_direction_input(sda);

	for (*data = 0; len > 0; len--) {
		u32 u;

		rtl8366_smi_clk_delay(smi);

		/* clocking */
		gpio_set_value(sck, 1);
		rtl8366_smi_clk_delay(smi);
		u = !!gpio_get_value(sda);
		gpio_set_value(sck, 0);

		*data |= (u << (len - 1));
	}

	gpio_direction_output(sda, 0);
}

static int rtl8366_smi_wait_for_ack(struct rtl8366_smi *smi)
{
	int retry_cnt;

	retry_cnt = 0;
	do {
		u32 ack;

		rtl8366_smi_read_bits(smi, 1, &ack);
		if (ack == 0)
			break;

		if (++retry_cnt > RTL8366_SMI_ACK_RETRY_COUNT) {
			dev_err(smi->parent, "ACK timeout\n");
			return -ETIMEDOUT;
		}
	} while (1);

	return 0;
}

static int rtl8366_smi_write_byte(struct rtl8366_smi *smi, u8 data)
{
	rtl8366_smi_write_bits(smi, data, 8);
	return rtl8366_smi_wait_for_ack(smi);
}

static int rtl8366_smi_write_byte_noack(struct rtl8366_smi *smi, u8 data)
{
	rtl8366_smi_write_bits(smi, data, 8);
	return 0;
}

static int rtl8366_smi_read_byte0(struct rtl8366_smi *smi, u8 *data)
{
	u32 t;

	/* read data */
	rtl8366_smi_read_bits(smi, 8, &t);
	*data = (t & 0xff);

	/* send an ACK */
	rtl8366_smi_write_bits(smi, 0x00, 1);

	return 0;
}

static int rtl8366_smi_read_byte1(struct rtl8366_smi *smi, u8 *data)
{
	u32 t;

	/* read data */
	rtl8366_smi_read_bits(smi, 8, &t);
	*data = (t & 0xff);

	/* send an ACK */
	rtl8366_smi_write_bits(smi, 0x01, 1);

	return 0;
}

int rtl8366_smi_read_reg(struct rtl8366_smi *smi, u32 addr, u32 *data)
{
	unsigned long flags;
	u8 lo = 0;
	u8 hi = 0;
	int ret = 0;

	spin_lock_irqsave(&smi->lock, flags);

	if (smi->mdio_bus) {
		/* Write address control code to register 31 */
		MDC_MDIO_WRITE(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			       MDC_MDIO_PHY_ID, MDC_MDIO_CTRL0_REG,
			       MDC_MDIO_ADDR_OP);

		/* Write address to register 23 */
		MDC_MDIO_WRITE(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			       MDC_MDIO_PHY_ID, MDC_MDIO_ADDRESS_REG, addr);

		/* Write read control code to register 21 */
		MDC_MDIO_WRITE(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			       MDC_MDIO_PHY_ID, MDC_MDIO_CTRL1_REG,
			       MDC_MDIO_READ_OP);

		/* Read data from register 25 */
		MDC_MDIO_READ(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			      MDC_MDIO_PHY_ID, MDC_MDIO_DATA_READ_REG, data);
	} else {
		rtl8366_smi_start(smi);

		/* send READ command */
		ret = rtl8366_smi_write_byte(smi, smi->cmd_read);
		if (ret)
			goto out;

		/* set ADDR[7:0] */
		ret = rtl8366_smi_write_byte(smi, addr & 0xff);
		if (ret)
			goto out;

		/* set ADDR[15:8] */
		ret = rtl8366_smi_write_byte(smi, addr >> 8);
		if (ret)
			goto out;

		/* read DATA[7:0] */
		rtl8366_smi_read_byte0(smi, &lo);
		/* read DATA[15:8] */
		rtl8366_smi_read_byte1(smi, &hi);

		*data = ((u32)lo) | (((u32)hi) << 8);

		ret = 0;

out:
		rtl8366_smi_stop(smi);
	}

	spin_unlock_irqrestore(&smi->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_read_reg);

static int __rtl8366_smi_write_reg(struct rtl8366_smi *smi,
				   u32 addr, u32 data, bool ack)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&smi->lock, flags);

	if (smi->mdio_bus) {
		/* Write address control code to register 31 */
		MDC_MDIO_WRITE(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			       MDC_MDIO_PHY_ID, MDC_MDIO_CTRL0_REG,
			       MDC_MDIO_ADDR_OP);

		/* Write address to register 23 */
		MDC_MDIO_WRITE(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			       MDC_MDIO_PHY_ID, MDC_MDIO_ADDRESS_REG, addr);

		/* Write data to register 24 */
		MDC_MDIO_WRITE(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			       MDC_MDIO_PHY_ID, MDC_MDIO_DATA_WRITE_REG, data);

		/* Write data control code to register 21 */
		MDC_MDIO_WRITE(smi->mdio_bus, MDC_MDIO_PREAMBLE_LEN,
			       MDC_MDIO_PHY_ID, MDC_MDIO_CTRL1_REG,
			       MDC_MDIO_WRITE_OP);
	} else {
		rtl8366_smi_start(smi);

		/* send WRITE command */
		ret = rtl8366_smi_write_byte(smi, smi->cmd_write);
		if (ret)
			goto out;

		/* set ADDR[7:0] */
		ret = rtl8366_smi_write_byte(smi, addr & 0xff);
		if (ret)
			goto out;

		/* set ADDR[15:8] */
		ret = rtl8366_smi_write_byte(smi, addr >> 8);
		if (ret)
			goto out;

		/* write DATA[7:0] */
		ret = rtl8366_smi_write_byte(smi, data & 0xff);
		if (ret)
			goto out;

		/* write DATA[15:8] */
		if (ack)
			ret = rtl8366_smi_write_byte(smi, data >> 8);
		else
			ret = rtl8366_smi_write_byte_noack(smi, data >> 8);
		if (ret)
			goto out;

		ret = 0;

out:
		rtl8366_smi_stop(smi);
	}

	spin_unlock_irqrestore(&smi->lock, flags);

	return ret;
}

int rtl8366_smi_write_reg(struct rtl8366_smi *smi, u32 addr, u32 data)
{
	return __rtl8366_smi_write_reg(smi, addr, data, true);
}
EXPORT_SYMBOL_GPL(rtl8366_smi_write_reg);

int rtl8366_smi_write_reg_noack(struct rtl8366_smi *smi, u32 addr, u32 data)
{
	return __rtl8366_smi_write_reg(smi, addr, data, false);
}
EXPORT_SYMBOL_GPL(rtl8366_smi_write_reg_noack);

int rtl8366_smi_rmwr(struct rtl8366_smi *smi, u32 addr, u32 mask, u32 data)
{
	u32 t;
	int err;

	if (WARN_ON_ONCE(data & ~mask)) {
		return -EINVAL;
	}

	err = rtl8366_smi_read_reg(smi, addr, &t);
	if (err)
		return err;

	err = rtl8366_smi_write_reg(smi, addr, (t & ~mask) | (data & mask));
	return err;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_rmwr);

static int rtl8366_reset(struct rtl8366_smi *smi)
{
	if (smi->hw_reset) {
		smi->hw_reset(true);
		msleep(RTL8366_SMI_HW_STOP_DELAY);
		smi->hw_reset(false);
		msleep(RTL8366_SMI_HW_START_DELAY);
		return 0;
	}

	if (smi->ops->reset_chip) {
		return smi->ops->reset_chip(smi);
	} else {
		return -EINVAL;
	}
}

static int rtl8366_mc_is_used(struct rtl8366_smi *smi, int mc_index, int *used)
{
	int err;
	int i;

	*used = 0;
	for (i = 0; i < smi->num_ports; i++) {
		int index = 0;

		err = smi->ops->get_mc_index(smi, i, &index);
		if (err)
			return err;

		if (mc_index == index) {
			*used = 1;
			break;
		}
	}

	return 0;
}

static int rtl8366_set_vlan(struct rtl8366_smi *smi, int vid, u32 member,
			    u32 untag, u32 fid)
{
	struct rtl8366_vlan_4k vlan4k;
	int err;
	int i;
	int match = 0;

	/* Update the 4K table */
	err = smi->ops->get_vlan_4k(smi, vid, &vlan4k);
	if (err)
		return err;

	vlan4k.member = member;
	vlan4k.untag = untag;
	vlan4k.fid = fid;
	err = smi->ops->set_vlan_4k(smi, &vlan4k);
	if (err)
		return err;

	/* Try to find an existing MC entry for this VID */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		struct rtl8366_vlan_mc vlanmc;

		err = smi->ops->get_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;

		if (vid == vlanmc.vid) {
			/* update the MC entry */
			vlanmc.member = member;
			vlanmc.untag = untag;
			vlanmc.fid = fid;

			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			match = 1;
			break;
		}
	}

	if (!match) {
		dev_err(smi->parent, "Failed to configure vlan\n");
		return -ENODEV;
	}

	return err;
}

static int rtl8366_get_pvid(struct rtl8366_smi *smi, int port, int *val)
{
	struct rtl8366_vlan_mc vlanmc;
	int err;
	int index;

	err = smi->ops->get_mc_index(smi, port, &index);
	if (err)
		return err;

	err = smi->ops->get_vlan_mc(smi, index, &vlanmc);
	if (err)
		return err;

	*val = vlanmc.vid;

	return 0;
}

static int rtl8366_set_pvid(struct rtl8366_smi *smi, unsigned int port,
			    unsigned int vid)
{
	struct rtl8366_vlan_mc vlanmc;
	struct rtl8366_vlan_4k vlan4k;
	int err;
	int i;

	/* Try to find an existing MC entry for this VID */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		err = smi->ops->get_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;

		if (vid == vlanmc.vid) {
			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			if (err)
				return err;

			err = smi->ops->set_mc_index(smi, port, i);
			return err;
		}
	}

	/* We have no MC entry for this VID, try to find an empty one */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		err = smi->ops->get_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;

		if (vlanmc.vid == 0 && vlanmc.member == 0) {
			/* Update the entry from the 4K table */
			err = smi->ops->get_vlan_4k(smi, vid, &vlan4k);
			if (err)
				return err;

			vlanmc.vid = vid;
			vlanmc.member = vlan4k.member;
			vlanmc.untag = vlan4k.untag;
			vlanmc.fid = vlan4k.fid;
			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			if (err)
				return err;

			err = smi->ops->set_mc_index(smi, port, i);
			return err;
		}
	}

	/* MC table is full, try to find an unused entry and replace it */
	for (i = 0; i < smi->num_vlan_mc; i++) {
		int used;

		err = rtl8366_mc_is_used(smi, i, &used);
		if (err)
			return err;

		if (!used) {
			/* Update the entry from the 4K table */
			err = smi->ops->get_vlan_4k(smi, vid, &vlan4k);
			if (err)
				return err;

			vlanmc.vid = vid;
			vlanmc.member = vlan4k.member;
			vlanmc.untag = vlan4k.untag;
			vlanmc.fid = vlan4k.fid;
			err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
			if (err)
				return err;

			err = smi->ops->set_mc_index(smi, port, i);
			return err;
		}
	}

	dev_err(smi->parent,
		"all VLAN member configurations are in use\n");

	return -ENOSPC;
}

int rtl8366_enable_vlan(struct rtl8366_smi *smi, int enable)
{
	int err = -ENXIO;

	if (smi->ops->enable_vlan)
		err = smi->ops->enable_vlan(smi, enable);
	if (err)
		return err;

	smi->vlan_enabled = enable;

	if (!enable) {
		smi->vlan4k_enabled = 0;
		err = -ENXIO;
		if (smi->ops->enable_vlan4k)
			err = smi->ops->enable_vlan4k(smi, enable);
	}

	return err;
}
EXPORT_SYMBOL_GPL(rtl8366_enable_vlan);

static int rtl8366_enable_vlan4k(struct rtl8366_smi *smi, int enable)
{
	int err = -ENXIO;

	if (enable) {
		if (smi->ops->enable_vlan)
			err = smi->ops->enable_vlan(smi, enable);
		if (err)
			return err;

		smi->vlan_enabled = enable;
	}

	err = -ENXIO;
	if (smi->ops->enable_vlan4k)
		err = smi->ops->enable_vlan4k(smi, enable);
	if (err)
		return err;

	smi->vlan4k_enabled = enable;
	return 0;
}

int rtl8366_enable_all_ports(struct rtl8366_smi *smi, int enable)
{
	int port;
	int err;

	for (port = 0; port < smi->num_ports; port++) {
		err = smi->ops->enable_port(smi, port, enable);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_enable_all_ports);

int rtl8366_reset_vlan(struct rtl8366_smi *smi)
{
	struct rtl8366_vlan_mc vlanmc;
	int err;
	int i;

	rtl8366_enable_vlan(smi, 0);
	rtl8366_enable_vlan4k(smi, 0);

	/* clear VLAN member configurations */
	vlanmc.vid = 0;
	vlanmc.priority = 0;
	vlanmc.member = 0;
	vlanmc.untag = 0;
	vlanmc.fid = 0;
	for (i = 0; i < smi->num_vlan_mc; i++) {
		err = smi->ops->set_vlan_mc(smi, i, &vlanmc);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rtl8366_reset_vlan);

static int rtl8366_init_vlan(struct rtl8366_smi *smi)
{
	int port;
	int err;

	err = rtl8366_reset_vlan(smi);
	if (err)
		return err;

	for (port = 0; port < smi->num_ports; port++) {
		u32 mask;

		if (port == smi->cpu_port)
			mask = (1 << smi->num_ports) - 1;
		else
			mask = (1 << port) | (1 << smi->cpu_port);

		err = rtl8366_set_vlan(smi, (port + 1), mask, mask, 0);
		if (err)
			return err;

		err = rtl8366_set_pvid(smi, port, (port + 1));
		if (err)
			return err;
	}

	return rtl8366_enable_vlan(smi, 1);
}

#ifdef CONFIG_RTL8366_SMI_DEBUG_FS
static ssize_t rtl8366_read_debugfs_vlan_mc(struct file *file,
					    char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct rtl8366_smi *smi = (struct rtl8366_smi *)file->private_data;
	int i, len = 0;
	char *buf = smi->buf;
	ssize_t ret;

	mutex_lock(&smi->buf_lock);

	len += scnprintf(buf + len, sizeof(smi->buf) - len,
			 "%2s %6s %4s %6s %6s %3s\n",
			 "id", "vid", "prio", "member", "untag", "fid");

	for (i = 0; i < smi->num_vlan_mc; ++i) {
		struct rtl8366_vlan_mc vlanmc;

		smi->ops->get_vlan_mc(smi, i, &vlanmc);

		len += scnprintf(buf + len, sizeof(smi->buf) - len,
				 "%2d %6d %4d 0x%04x 0x%04x %3d\n",
				 i, vlanmc.vid, vlanmc.priority,
				 vlanmc.member, vlanmc.untag, vlanmc.fid);
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&smi->buf_lock);

	return ret;
}

#define RTL8366_VLAN4K_PAGE_SIZE	64
#define RTL8366_VLAN4K_NUM_PAGES	(4096 / RTL8366_VLAN4K_PAGE_SIZE)

static ssize_t rtl8366_read_debugfs_vlan_4k(struct file *file,
					    char __user *user_buf,
					    size_t count, loff_t *ppos)
{
	struct rtl8366_smi *smi = (struct rtl8366_smi *)file->private_data;
	int i, len = 0;
	int offset;
	char *buf = smi->buf;
	ssize_t ret;

	mutex_lock(&smi->buf_lock);

	if (smi->dbg_vlan_4k_page >= RTL8366_VLAN4K_NUM_PAGES) {
		len += scnprintf(buf + len, sizeof(smi->buf) - len,
				 "invalid page: %u\n", smi->dbg_vlan_4k_page);
		goto out;
	}

	len += scnprintf(buf + len, sizeof(smi->buf) - len,
			 "%4s %6s %6s %3s\n",
			 "vid", "member", "untag", "fid");

	offset = RTL8366_VLAN4K_PAGE_SIZE * smi->dbg_vlan_4k_page;
	for (i = 0; i < RTL8366_VLAN4K_PAGE_SIZE; i++) {
		struct rtl8366_vlan_4k vlan4k;

		smi->ops->get_vlan_4k(smi, offset + i, &vlan4k);

		len += scnprintf(buf + len, sizeof(smi->buf) - len,
				 "%4d 0x%04x 0x%04x %3d\n",
				 vlan4k.vid, vlan4k.member,
				 vlan4k.untag, vlan4k.fid);
	}

out:
	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&smi->buf_lock);

	return ret;
}

static ssize_t rtl8366_read_debugfs_pvid(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct rtl8366_smi *smi = (struct rtl8366_smi *)file->private_data;
	char *buf = smi->buf;
	int len = 0;
	int i;
	ssize_t ret;

	mutex_lock(&smi->buf_lock);

	len += scnprintf(buf + len, sizeof(smi->buf) - len, "%4s %4s\n",
			"port", "pvid");

	for (i = 0; i < smi->num_ports; i++) {
		int pvid;
		int err;

		err = rtl8366_get_pvid(smi, i, &pvid);
		if (err)
			len += scnprintf(buf + len, sizeof(smi->buf) - len,
					 "%4d error\n", i);
		else
			len += scnprintf(buf + len, sizeof(smi->buf) - len,
					 "%4d %4d\n", i, pvid);
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&smi->buf_lock);

	return ret;
}

static ssize_t rtl8366_read_debugfs_reg(struct file *file,
					char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct rtl8366_smi *smi = (struct rtl8366_smi *)file->private_data;
	u32 t, reg = smi->dbg_reg;
	int err, len = 0;
	char *buf = smi->buf;
	ssize_t ret;

	mutex_lock(&smi->buf_lock);

	memset(buf, '\0', sizeof(smi->buf));

	err = rtl8366_smi_read_reg(smi, reg, &t);
	if (err) {
		len += scnprintf(buf, sizeof(smi->buf),
				 "Read failed (reg: 0x%04x)\n", reg);
		goto out;
	}

	len += scnprintf(buf, sizeof(smi->buf), "reg = 0x%04x, val = 0x%04x\n",
			 reg, t);

out:
	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&smi->buf_lock);

	return ret;
}

static ssize_t rtl8366_write_debugfs_reg(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct rtl8366_smi *smi = (struct rtl8366_smi *)file->private_data;
	unsigned long data;
	u32 reg = smi->dbg_reg;
	int err;
	size_t len;
	char *buf = smi->buf;

	mutex_lock(&smi->buf_lock);

	len = min(count, sizeof(smi->buf) - 1);
	if (copy_from_user(buf, user_buf, len)) {
		dev_err(smi->parent, "copy from user failed\n");
		mutex_unlock(&smi->buf_lock);
		return -EFAULT;
	}

	buf[len] = '\0';
	if (len > 0 && buf[len - 1] == '\n')
		buf[len - 1] = '\0';

	if (kstrtoul(buf, 16, &data)) {
		dev_err(smi->parent, "Invalid reg value %s\n", buf);
	} else {
		err = rtl8366_smi_write_reg(smi, reg, data);
		if (err) {
			dev_err(smi->parent,
				"writing reg 0x%04x val 0x%04lx failed\n",
				reg, data);
		}
	}

	mutex_unlock(&smi->buf_lock);

	return count;
}

static ssize_t rtl8366_read_debugfs_mibs(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct rtl8366_smi *smi = file->private_data;
	int i, j, len = 0;
	char *buf = smi->buf;
	ssize_t ret;

	mutex_lock(&smi->buf_lock);

	len += scnprintf(buf + len, sizeof(smi->buf) - len, "%-36s",
			 "Counter");

	for (i = 0; i < smi->num_ports; i++) {
		char port_buf[10];

		scnprintf(port_buf, sizeof(port_buf), "Port %d", i);
		len += scnprintf(buf + len, sizeof(smi->buf) - len, " %12s",
				 port_buf);
	}
	len += scnprintf(buf + len, sizeof(smi->buf) - len, "\n");

	for (i = 0; i < smi->num_mib_counters; i++) {
		len += scnprintf(buf + len, sizeof(smi->buf) - len, "%-36s ",
				smi->mib_counters[i].name);
		for (j = 0; j < smi->num_ports; j++) {
			unsigned long long counter = 0;

			if (!smi->ops->get_mib_counter(smi, i, j, &counter))
				len += scnprintf(buf + len,
						 sizeof(smi->buf) - len,
						 "%12llu ", counter);
			else
				len += scnprintf(buf + len,
						 sizeof(smi->buf) - len,
						 "%12s ", "error");
		}
		len += scnprintf(buf + len, sizeof(smi->buf) - len, "\n");
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	mutex_unlock(&smi->buf_lock);

	return ret;
}

static const struct file_operations fops_rtl8366_regs = {
	.read	= rtl8366_read_debugfs_reg,
	.write	= rtl8366_write_debugfs_reg,
	.open	= simple_open,
	.owner	= THIS_MODULE
};

static const struct file_operations fops_rtl8366_vlan_mc = {
	.read	= rtl8366_read_debugfs_vlan_mc,
	.open	= simple_open,
	.owner	= THIS_MODULE
};

static const struct file_operations fops_rtl8366_vlan_4k = {
	.read	= rtl8366_read_debugfs_vlan_4k,
	.open	= simple_open,
	.owner	= THIS_MODULE
};

static const struct file_operations fops_rtl8366_pvid = {
	.read	= rtl8366_read_debugfs_pvid,
	.open	= simple_open,
	.owner	= THIS_MODULE
};

static const struct file_operations fops_rtl8366_mibs = {
	.read	= rtl8366_read_debugfs_mibs,
	.open	= simple_open,
	.owner	= THIS_MODULE
};

static void rtl8366_debugfs_init(struct rtl8366_smi *smi)
{
	struct dentry *node;
	struct dentry *root;

	mutex_init(&smi->buf_lock);

	if (!smi->debugfs_root)
		smi->debugfs_root = debugfs_create_dir(dev_name(smi->parent),
						       NULL);

	if (!smi->debugfs_root) {
		dev_err(smi->parent, "Unable to create debugfs dir\n");
		return;
	}
	root = smi->debugfs_root;

	debugfs_create_x16("reg", 0644, root, &smi->dbg_reg);

	debugfs_create_file("val", 0644, root, smi, &fops_rtl8366_regs);

	debugfs_create_file("vlan_mc", 0400, root, smi,
				   &fops_rtl8366_vlan_mc);

	debugfs_create_u8("vlan_4k_page", 0644, root,
				 &smi->dbg_vlan_4k_page);

	debugfs_create_file("vlan_4k", 0400, root, smi,
				   &fops_rtl8366_vlan_4k);

	debugfs_create_file("pvid", 0400, root, smi, &fops_rtl8366_pvid);

	debugfs_create_file("mibs", 0400, smi->debugfs_root, smi,
				   &fops_rtl8366_mibs);
}

static void rtl8366_debugfs_remove(struct rtl8366_smi *smi)
{
	debugfs_remove_recursive(smi->debugfs_root);
	smi->debugfs_root = NULL;
}
#else
static inline void rtl8366_debugfs_init(struct rtl8366_smi *smi) {}
static inline void rtl8366_debugfs_remove(struct rtl8366_smi *smi) {}
#endif /* CONFIG_RTL8366_SMI_DEBUG_FS */

static int rtl8366_smi_mii_init(struct rtl8366_smi *smi)
{
	int ret;
	int i;

	smi->mii_bus = mdiobus_alloc();
	if (smi->mii_bus == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	smi->mii_bus->priv = (void *)smi;
	smi->mii_bus->name = dev_name(smi->parent);
	smi->mii_bus->read = smi->ops->mii_read;
	smi->mii_bus->write = smi->ops->mii_write;
	snprintf(smi->mii_bus->id, MII_BUS_ID_SIZE, "%s",
		 dev_name(smi->parent));
	smi->mii_bus->parent = smi->parent;
	smi->mii_bus->phy_mask = ~(0x1f);
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		smi->mii_irq[i] = PHY_POLL;
		smi->mii_bus->irq[i] = PHY_POLL;
	}

	ret = mdiobus_register(smi->mii_bus);
	if (ret)
		goto err_free;

	return 0;

 err_free:
	mdiobus_free(smi->mii_bus);
 err:
	return ret;
}

static void rtl8366_smi_mii_cleanup(struct rtl8366_smi *smi)
{
	mdiobus_unregister(smi->mii_bus);
	mdiobus_free(smi->mii_bus);
}

struct rtl8366_smi *rtl8366_smi_alloc(struct device *parent)
{
	struct rtl8366_smi *smi;

	if (WARN_ON(!parent))
		return NULL;

	smi = kzalloc(sizeof(*smi), GFP_KERNEL);
	if (!smi)
		return NULL;

	smi->parent = parent;
	return smi;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_alloc);

static int __rtl8366_smi_init(struct rtl8366_smi *smi, const char *name)
{
	int err;

	if (!smi->mdio_bus) {
		err = gpio_request(smi->gpio_sda, name);
		if (err) {
			dev_err(smi->parent,
				"rtl8366_smi: gpio_request failed for %u, err=%d\n",
				smi->gpio_sda, err);
			return err;
		}

		err = gpio_request(smi->gpio_sck, name);
		if (err) {
			gpio_free(smi->gpio_sda);
			dev_err(smi->parent,
				"rtl8366_smi: gpio_request failed for %u, err=%d\n",
				smi->gpio_sck, err);
			return err;
		}
	}

	spin_lock_init(&smi->lock);

	/* start the switch */
	if (smi->hw_reset) {
		smi->hw_reset(false);
		msleep(RTL8366_SMI_HW_START_DELAY);
	}

	return 0;
}

static void __rtl8366_smi_cleanup(struct rtl8366_smi *smi)
{
	if (smi->hw_reset)
		smi->hw_reset(true);

	if (!smi->mdio_bus) {
		gpio_free(smi->gpio_sck);
		gpio_free(smi->gpio_sda);
	}
}

enum rtl8366_type rtl8366_smi_detect(struct rtl8366_platform_data *pdata)
{
	static struct rtl8366_smi smi;
	enum rtl8366_type type = RTL8366_TYPE_UNKNOWN;
	u32 reg = 0;

	memset(&smi, 0, sizeof(smi));
	if (!smi.mdio_bus) {
		smi.gpio_sda = pdata->gpio_sda;
		smi.gpio_sck = pdata->gpio_sck;
	}
	smi.clk_delay = 10;
	smi.cmd_read  = 0xa9;
	smi.cmd_write = 0xa8;

	if (__rtl8366_smi_init(&smi, "rtl8366"))
		goto out;

	if (rtl8366_smi_read_reg(&smi, 0x5c, &reg))
		goto cleanup;

	switch (reg) {
	case 0x6027:
		pr_info("Found an RTL8366S switch\n");
		type = RTL8366_TYPE_S;
		break;
	case 0x5937:
		pr_info("Found an RTL8366RB switch\n");
		type = RTL8366_TYPE_RB;
		break;
	default:
		pr_info("Found an Unknown RTL8366 switch (id=0x%04x)\n", reg);
		break;
	}

cleanup:
	__rtl8366_smi_cleanup(&smi);
out:
	return type;
}

int rtl8366_smi_init(struct rtl8366_smi *smi)
{
	int err;

	if (!smi->ops)
		return -EINVAL;

	err = __rtl8366_smi_init(smi, dev_name(smi->parent));
	if (err)
		goto err_out;

	if (!smi->mdio_bus) {
		dev_info(smi->parent, "using GPIO pins %u (SDA) and %u (SCK)\n",
			 smi->gpio_sda, smi->gpio_sck);
	}

	/* Some HW does not provide the detection helper. */
	if (smi->ops->detect) {
		err = smi->ops->detect(smi);
		if (err) {
			dev_err(smi->parent, "chip detection failed, err=%d\n",
				err);
			goto err_free_sck;
		}
	}

	err = rtl8366_reset(smi);
	if (err)
		goto err_free_sck;

	/* To provide the root_debugfs parent in case of it is needed when setup
	 * a plugin.
	 */
	rtl8366_debugfs_init(smi);

	err = smi->ops->setup(smi);
	if (err) {
		dev_err(smi->parent, "chip setup failed, err=%d\n", err);
		goto err_free_sck;
	}

	/* Init VLAN only if the support is available. */
	if (smi->ops->enable_vlan) {
		err = rtl8366_init_vlan(smi);
		if (err) {
			dev_err(smi->parent, "VLAN init failed, err=%d\n", err);
			goto err_free_sck;
		}
	}

	err = rtl8366_enable_all_ports(smi, 1);
	if (err)
		goto err_free_sck;

	err = rtl8366_smi_mii_init(smi);
	if (err)
		goto err_free_sck;

	return 0;

 err_free_sck:
	__rtl8366_smi_cleanup(smi);
 err_out:
	return err;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_init);

void rtl8366_smi_cleanup(struct rtl8366_smi *smi)
{
	rtl8366_debugfs_remove(smi);
	rtl8366_smi_mii_cleanup(smi);
	__rtl8366_smi_cleanup(smi);
}
EXPORT_SYMBOL_GPL(rtl8366_smi_cleanup);

#ifdef CONFIG_OF
int rtl8366_smi_probe_of(struct platform_device *pdev, struct rtl8366_smi *smi)
{
	struct device_node *mdio;

	mdio = of_parse_phandle(pdev->dev.of_node, "mdio-bus", 0);
	if (mdio) {
		smi->mdio_bus = of_mdio_find_bus(mdio);
		if (!smi->mdio_bus) {
			dev_err(&pdev->dev, "invalid mdio-bus handle in devicetree\n");
			return -EINVAL;
		}
	} else {
		int sck = of_get_named_gpio(pdev->dev.of_node, "gpio-sck", 0);
		int sda = of_get_named_gpio(pdev->dev.of_node, "gpio-sda", 0);

		if (!gpio_is_valid(sck) || !gpio_is_valid(sda)) {
			dev_err(&pdev->dev, "gpios and mdio-bus both are missing in devicetree\n");
			return -EINVAL;
		}
		smi->gpio_sda = sda;
		smi->gpio_sck = sck;
	}

	return 0;
}
#else
static inline int rtl8366_smi_probe_of(struct platform_device *pdev,
				       struct rtl8366_smi *smi)
{
	return -ENODEV;
}
#endif

int rtl8366_smi_probe_plat(struct platform_device *pdev,
			   struct rtl8366_smi *smi)
{
	struct rtl8366_platform_data *pdata = pdev->dev.platform_data;

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "no platform data specified\n");
		return -EINVAL;
	}

	if (!smi->mdio_bus) {
		smi->gpio_sda = pdata->gpio_sda;
		smi->gpio_sck = pdata->gpio_sck;
	}

	smi->hw_reset = pdata->hw_reset;

	return 0;
}

struct rtl8366_smi *rtl8366_smi_probe(struct platform_device *pdev)
{
	struct rtl8366_smi *smi;
	int err;

	smi = rtl8366_smi_alloc(&pdev->dev);
	if (!smi)
		return NULL;

	if (pdev->dev.of_node)
		err = rtl8366_smi_probe_of(pdev, smi);
	else
		err = rtl8366_smi_probe_plat(pdev, smi);

	if (err)
		goto free_smi;

	return smi;

free_smi:
	kfree(smi);
	return NULL;
}
EXPORT_SYMBOL_GPL(rtl8366_smi_probe);

MODULE_DESCRIPTION("Realtek RTL8366 SMI interface driver");
MODULE_AUTHOR("Gabor Juhos <juhosg@openwrt.org>");
MODULE_LICENSE("GPL v2");
