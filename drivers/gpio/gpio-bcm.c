/*
 * Copyright (C) 2014 Institut de Recherche Technologique SystemX and OpenWide.
 * All rights reserved.
 *
 * MXC GPIO support. (c) 2008 Daniel Mack <daniel@caiaq.de>
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 *
 * Based on code from Freescale,
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * @file gpio-mxc.c
 * @author Jimmy Durand Wesolowski (jimmy.durand-wesolowski@openwide.fr)
 * @brief MXC GPIO support
 */

/* FIXME WIP */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <asm-generic/bug.h>

#include "bcm2836.h"

#define DEBUG

#ifdef DEBUG
#define DPRINTF(fmt, ...)  vmm_printf("\t[%s] " fmt, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#else
#define DPRINTF(fmt, ...)
#endif

#define NOT_IMPLEMENTED\
    do{\
        vmm_printf("\t\t\t/!\\/!\\/!\\/!\\/!\\ ERROR : %s Not Implemented /!\\/!\\/!\\/!\\/!\\\n", __PRETTY_FUNCTION__);\
        BUG();\
    } while(false)


#define BCM2836_PERIPH_BASE	(0x3F000000)
#define PERIPH_BASE         BCM2836_PERIPH_BASE

#define GPIO_OFFSET (0x00200000)
#define GPIO_BASE	 (PERIPH_BASE + GPIO_OFFSET)

#define GPSET0            0x0000001C
#define GPSET1            0x00000020
#define GPCLR0            0x00000028
#define GPCLR1            0x0000002C
#define GPLEV0            0x00000034
#define GPLEV1            0x00000038

#define GPFSEL0           0x00000000
#define GPFSEL1           0x00000004
#define GPFSEL2           0x00000008
#define GPFSEL3           0x0000000C
#define GPFSEL4           0x00000010
#define GPFSEL5           0x00000014

#define GPPUD             0x00000094
#define GPPUDCLK0         0x00000098
#define GPPUDCLK1         0x0000009C


#define BCM_GPIO_PASSWD				0x00a5a501
#define GPIO_PER_BANK				32
/*#define GPIO_MAX_BANK_NUM			8*/

#define GPIO_BANK(gpio)				((gpio) >> 5)
#define GPIO_BIT(gpio)				((gpio) & (GPIO_PER_BANK - 1))

/* There is a GPIO control register for each GPIO */
#define GPIO_CONTROL(gpio)			(0x00000100 + ((gpio) << 2))

/* The remaining registers are per GPIO bank */
#define GPIO_OUT_STATUS(bank)			(0x00000000 + ((bank) << 2))
#define GPIO_IN_STATUS(bank)			(0x00000020 + ((bank) << 2))
#define GPIO_OUT_SET(bank)			(0x00000040 + ((bank) << 2))
#define GPIO_OUT_CLEAR(bank)			(0x00000060 + ((bank) << 2))
#define GPIO_INT_STATUS(bank)			(0x00000080 + ((bank) << 2))
#define GPIO_INT_MASK(bank)			(0x000000a0 + ((bank) << 2))
#define GPIO_INT_MSKCLR(bank)			(0x000000c0 + ((bank) << 2))
#define GPIO_PWD_STATUS(bank)			(0x00000500 + ((bank) << 2))

#define GPIO_GPPWR_OFFSET			0x00000520

#define GPIO_GPCTR0_DBR_SHIFT			5
#define GPIO_GPCTR0_DBR_MASK			0x000001e0

#define GPIO_GPCTR0_ITR_SHIFT			3
#define GPIO_GPCTR0_ITR_MASK			0x00000018
#define GPIO_GPCTR0_ITR_CMD_RISING_EDGE		0x00000001
#define GPIO_GPCTR0_ITR_CMD_FALLING_EDGE	0x00000002
#define GPIO_GPCTR0_ITR_CMD_BOTH_EDGE		0x00000003

#define GPIO_GPCTR0_IOTR_MASK			0x00000001
#define GPIO_GPCTR0_IOTR_CMD_0UTPUT		0x00000000
#define GPIO_GPCTR0_IOTR_CMD_INPUT		0x00000001

#define GPIO_GPCTR0_DB_ENABLE_MASK		0x00000100

#define LOCK_CODE				0xffffffff
#define UNLOCK_CODE				0x00000000

enum mxc_gpio_hwtype {
	IMX1_GPIO,	/* runs on i.mx1 */
	IMX21_GPIO,	/* runs on i.mx21 and i.mx27 */
	IMX31_GPIO,	/* runs on i.mx31 */
	IMX35_GPIO,	/* runs on all other i.mx */
};

/* device type dependent stuff */
struct mxc_gpio_hwdata {
	unsigned dr_reg;
	unsigned gdir_reg;
	unsigned psr_reg;
	unsigned icr1_reg;
	unsigned icr2_reg;
	unsigned imr_reg;
	unsigned isr_reg;
	int edge_sel_reg;
	unsigned low_level;
	unsigned high_level;
	unsigned rise_edge;
	unsigned fall_edge;
};

// FIXME change content of the struct ?
struct mxc_gpio_port {
	struct list_head node;
	void __iomem* base;
	u32 irq;
	u32 irq_high;
	struct vmm_host_irqdomain *domain;
	struct gpio_chip gc;
	u32 both_edges;
};
const static unsigned GPIO_MAX_BANK_NUM = 8;
const static unsigned GPIO_PER_BANKS = 32;

struct bcm_gpio {
	void __iomem*               base;
        int                         num_banks;
	spinlock_t                  lock;
	struct gpio_chip            gc;
	struct vmm_host_irqdomain*  domain;
        struct bcm_gpio_bank*       banks;
	struct platform_device*     pdev;
};

struct bcm_gpio_bank {
        int                   id;
        int                   irq;
        struct bcm_kona_gpio* kona_gpio;

    struct bcm_gpio * gpio;
};

static struct mxc_gpio_hwdata imx35_gpio_hwdata = {
	.dr_reg		= 0x00,
	.gdir_reg	= 0x04,
	.psr_reg	= 0x08,
	.icr1_reg	= 0x0c,
	.icr2_reg	= 0x10,
	.imr_reg	= 0x14,
	.isr_reg	= 0x18,
	.edge_sel_reg	= 0x1c,
	.low_level	= 0x00,
	.high_level	= 0x01,
	.rise_edge	= 0x02,
	.fall_edge	= 0x03,
};

static enum mxc_gpio_hwtype mxc_gpio_hwtype;
static struct mxc_gpio_hwdata *mxc_gpio_hwdata;

#define GPIO_DR			(mxc_gpio_hwdata->dr_reg)
#define GPIO_GDIR		(mxc_gpio_hwdata->gdir_reg)
#define GPIO_PSR		(mxc_gpio_hwdata->psr_reg)
#define GPIO_ICR1		(mxc_gpio_hwdata->icr1_reg)
#define GPIO_ICR2		(mxc_gpio_hwdata->icr2_reg)
#define GPIO_IMR		(mxc_gpio_hwdata->imr_reg)
#define GPIO_ISR		(mxc_gpio_hwdata->isr_reg)
#define GPIO_EDGE_SEL		(mxc_gpio_hwdata->edge_sel_reg)

#define GPIO_INT_LOW_LEV	(mxc_gpio_hwdata->low_level)
#define GPIO_INT_HIGH_LEV	(mxc_gpio_hwdata->high_level)
#define GPIO_INT_RISE_EDGE	(mxc_gpio_hwdata->rise_edge)
#define GPIO_INT_FALL_EDGE	(mxc_gpio_hwdata->fall_edge)
#define GPIO_INT_BOTH_EDGES	0x4

static struct platform_device_id mxc_gpio_devtype[] = {
	{
		.name = "imx1-gpio",
		.driver_data = IMX1_GPIO,
	}, {
		.name = "imx21-gpio",
		.driver_data = IMX21_GPIO,
	}, {
		.name = "imx31-gpio",
		.driver_data = IMX31_GPIO,
	}, {
		.name = "imx35-gpio",
		.driver_data = IMX35_GPIO,
	}, {
		/* sentinel */
	}
};

static const struct vmm_devtree_nodeid mxc_gpio_dt_ids[] = {
	{ .compatible = "brcm,bcm2835-gpio", .data = &mxc_gpio_devtype[IMX35_GPIO], },
	{ /* sentinel */ }
};

/*
 * MX2 has one interrupt *for all* gpio ports. The list is used
 * to save the references to all ports, so that mx2_gpio_irq_handler
 * can walk through all interrupt status registers.
 */
static LIST_HEAD(mxc_gpio_ports);

/* Note: This driver assumes 32 GPIOs are handled in one register */

static int gpio_set_irq_type(struct vmm_host_irq *d, u32 type)
{
        NOT_IMPLEMENTED;
	struct mxc_gpio_port *port = vmm_host_irq_get_chip_data(d);
	u32 bit, val;
	u32 gpio_idx = vmm_host_irqdomain_to_hwirq(port->domain, d->num);
	u32 gpio = port->gc.base + gpio_idx;
	int edge;
	void __iomem *reg = port->base;

	port->both_edges &= ~(1 << gpio_idx);
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge = GPIO_INT_RISE_EDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge = GPIO_INT_FALL_EDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		if (GPIO_EDGE_SEL >= 0) {
			edge = GPIO_INT_BOTH_EDGES;
		} else {
			val = __gpio_get_value(gpio);
			if (val) {
				edge = GPIO_INT_LOW_LEV;
				pr_debug("mxc: set GPIO %d to low trigger\n", gpio);
			} else {
				edge = GPIO_INT_HIGH_LEV;
				pr_debug("mxc: set GPIO %d to high trigger\n", gpio);
			}
			port->both_edges |= 1 << gpio_idx;
		}
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge = GPIO_INT_LOW_LEV;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge = GPIO_INT_HIGH_LEV;
		break;
	default:
		return -EINVAL;
	}

	if (GPIO_EDGE_SEL >= 0) {
		val = readl(port->base + GPIO_EDGE_SEL);
		if (edge == GPIO_INT_BOTH_EDGES)
			writel(val | (1 << gpio_idx),
				port->base + GPIO_EDGE_SEL);
		else
			writel(val & ~(1 << gpio_idx),
				port->base + GPIO_EDGE_SEL);
	}

	if (edge != GPIO_INT_BOTH_EDGES) {
		reg += GPIO_ICR1 + ((gpio_idx & 0x10) >> 2); /* lower or upper register */
		bit = gpio_idx & 0xf;
		val = readl(reg) & ~(0x3 << (bit << 1));
		writel(val | (edge << (bit << 1)), reg);
	}

	writel(1 << gpio_idx, port->base + GPIO_ISR);

	return 0;
}

void __noinline mxc_flip_edge(struct mxc_gpio_port *port, u32 gpio)
{
        NOT_IMPLEMENTED;
	void __iomem *reg = port->base;
	u32 bit, val;
	int edge;

	reg += GPIO_ICR1 + ((gpio & 0x10) >> 2); /* lower or upper register */
	bit = gpio & 0xf;
	val = readl(reg);
	edge = (val >> (bit << 1)) & 3;
	val &= ~(0x3 << (bit << 1));
	if (edge == GPIO_INT_HIGH_LEV) {
		edge = GPIO_INT_LOW_LEV;
		pr_debug("mxc: switch GPIO %d to low trigger\n", gpio);
	} else if (edge == GPIO_INT_LOW_LEV) {
		edge = GPIO_INT_HIGH_LEV;
		pr_debug("mxc: switch GPIO %d to high trigger\n", gpio);
	} else {
		pr_err("mxc: invalid configuration for GPIO %d: %x\n",
		       gpio, edge);
		return;
	}
	writel(val | (edge << (bit << 1)), reg);
}

/* handle 32 interrupts in one status register */
static void mxc_gpio_irq_handler(struct mxc_gpio_port *port, u32 irq_stat)
{
        NOT_IMPLEMENTED;
	u32 irq_num = 0;
	u32 cpu = vmm_smp_processor_id();
	struct vmm_host_irq *irq;

	while (irq_stat != 0) {
		int irqoffset = fls(irq_stat) - 1;

		if (port->both_edges & (1 << irqoffset))
			mxc_flip_edge(port, irqoffset);

		irq_num = vmm_host_irqdomain_find_mapping(port->domain,
						       irqoffset);
		irq = vmm_host_irq_get(irq_num);
		vmm_handle_level_irq(irq, cpu, port);
		irq_stat &= ~(1 << irqoffset);
	}
}

/* MX1 and MX3 has one interrupt *per* gpio port */
static vmm_irq_return_t mx3_gpio_irq_handler(int irq, void *data)
{
        NOT_IMPLEMENTED;
	u32 irq_stat;
	struct vmm_host_irq* desc = NULL;
	struct mxc_gpio_port *port = data;
	struct vmm_host_irq_chip *chip = NULL;

	desc = vmm_host_irq_get(irq);
	chip = vmm_host_irq_get_chip(desc);

	vmm_chained_irq_enter(chip, desc);

	irq_stat = readl(port->base + GPIO_ISR) & readl(port->base + GPIO_IMR);
	mxc_gpio_irq_handler(port, irq_stat);

	vmm_chained_irq_exit(chip, desc);
	return VMM_IRQ_HANDLED;
}

/* MX2 has one interrupt *for all* gpio ports */
static vmm_irq_return_t mx2_gpio_irq_handler(int irq, void *data)
{
        NOT_IMPLEMENTED;
	u32 irq_msk, irq_stat;
	struct vmm_host_irq* desc = NULL;
	struct mxc_gpio_port *port = NULL;
	struct vmm_host_irq_chip *chip = NULL;

	desc = vmm_host_irq_get(irq);
	chip = vmm_host_irq_get_chip(desc);
	port = vmm_host_irq_get_chip_data(desc);
	vmm_chained_irq_enter(chip, desc);

	/* walk through all interrupt status registers */
	list_for_each_entry(port, &mxc_gpio_ports, node) {
		irq_msk = readl(port->base + GPIO_IMR);
		if (!irq_msk)
			continue;

		irq_stat = readl(port->base + GPIO_ISR) & irq_msk;
		if (irq_stat)
			mxc_gpio_irq_handler(port, irq_stat);
	}
	vmm_chained_irq_exit(chip, desc);
	return VMM_IRQ_HANDLED;
}

/* FIXME: Temporary */
static void irq_gc_lock(struct vmm_host_irq_chip *gc)
{
        NOT_IMPLEMENTED;
	gc = gc;
}

/* FIXME: Temporary */
static void irq_gc_unlock(struct vmm_host_irq_chip *gc)
{
        NOT_IMPLEMENTED;
	gc = gc;
}

static void irq_gc_init_lock(struct vmm_host_irq_chip *gc)
{
        NOT_IMPLEMENTED;
	gc = gc;
}

/**
 * irq_gc_ack_set_bit - Ack pending interrupt via setting bit
 * @d: irq_data
 */
void irq_gc_ack_set_bit(struct vmm_host_irq *d)
{
        NOT_IMPLEMENTED;
	struct vmm_host_irq_chip *gc = vmm_host_irq_get_chip(d);
	struct mxc_gpio_port *port = vmm_host_irq_get_chip_data(d);
	int irqoffset = vmm_host_irqdomain_to_hwirq(port->domain, d->num);

	irq_gc_lock(gc);
	writel(1 << irqoffset, port->base + GPIO_ISR);
	irq_gc_unlock(gc);
}

/**
 * irq_gc_mask_clr_bit - Mask chip via clearing bit in mask register
 * @d: irq_data
 *
 * Chip has a single mask register. Values of this register are cached
 * and protected by gc->lock
 */
void irq_gc_mask_clr_bit(struct vmm_host_irq *d)
{
        NOT_IMPLEMENTED;
	struct vmm_host_irq_chip *gc = vmm_host_irq_get_chip(d);
	struct mxc_gpio_port *port = vmm_host_irq_get_chip_data(d);
	int irqoffset = vmm_host_irqdomain_to_hwirq(port->domain, d->num);
	u32 mask = 0;

	irq_gc_lock(gc);
	mask = readl(port->base + GPIO_IMR) & ~(1 << irqoffset);
	writel(mask, port->base + GPIO_IMR);
	irq_gc_unlock(gc);
}

/**
 * irq_gc_mask_set_bit - Mask chip via setting bit in mask register
 * @d: irq_data
 *
 * Chip has a single mask register. Values of this register are cached
 * and protected by gc->lock
 */
void irq_gc_mask_set_bit(struct irq_data *d)
{
        NOT_IMPLEMENTED;
	struct vmm_host_irq_chip *gc = vmm_host_irq_get_chip(d);
	struct mxc_gpio_port *port = vmm_host_irq_get_chip_data(d);
	int irqoffset = vmm_host_irqdomain_to_hwirq(port->domain, d->num);
	u32 mask = 0;

	irq_gc_lock(gc);
	mask = readl(port->base + GPIO_IMR) | (1 << irqoffset);
	writel(mask, port->base + GPIO_IMR);
	irq_gc_unlock(gc);
}

static int __init mxc_gpio_init_gc(struct mxc_gpio_port *port,
				   const char *name, int sz,
				   struct vmm_device *dev)
{
        NOT_IMPLEMENTED;
	struct vmm_host_irq_chip *gc;
	int irq = 0;
	int i = 0;

	if (NULL == (gc = vmm_zalloc(sizeof(struct vmm_host_irq_chip))))
	{
		pr_err("mxc: Failed to allocate IRQ chip\n");
		return -ENOMEM;
	}
	irq_gc_init_lock(gc);

	gc->irq_ack = irq_gc_ack_set_bit;
	gc->irq_mask = irq_gc_mask_clr_bit;
	gc->irq_unmask = irq_gc_mask_set_bit;
	gc->irq_set_type = gpio_set_irq_type;
	gc->name = name;

	port->domain = vmm_host_irqdomain_add(dev->of_node, -1, sz,
					  &irqdomain_simple_ops, port);
	if (!port->domain)
		return VMM_ENOTAVAIL;

	for (i = 0; i < sz; ++i) {
		irq = vmm_host_irqdomain_create_mapping(port->domain, i);
		if (irq < 0) {
			pr_err("mxc: Failed to map extended IRQs\n");
			vmm_free(gc);
			return -ENODEV;
		}
		vmm_host_irq_set_chip(irq, gc);
		vmm_host_irq_set_chip_data(irq, port);
	}

	return VMM_OK;
}

static void mxc_gpio_get_hw(const struct vmm_devtree_nodeid *dev)
{
#if 0
	const struct vmm_devtree_nodeid *nodeid = 
		of_match_device(mxc_gpio_dt_ids, &pdev->dev);
#endif
        NOT_IMPLEMENTED;
	const struct platform_device_id *pdev = dev->data;
	enum mxc_gpio_hwtype hwtype;

	hwtype = pdev->driver_data;

	if (mxc_gpio_hwtype) {
		/*
		 * The driver works with a reasonable presupposition,
		 * that is all gpio ports must be the same type when
		 * running on one soc.
		 */
		BUG_ON(mxc_gpio_hwtype != hwtype);
		return;
	}

#if 0
	if (hwtype == IMX35_GPIO)
		mxc_gpio_hwdata = &imx35_gpio_hwdata;
	else if (hwtype == IMX31_GPIO)
		mxc_gpio_hwdata = &imx31_gpio_hwdata;
	else
		mxc_gpio_hwdata = &imx1_imx21_gpio_hwdata;
#endif
        mxc_gpio_hwdata = &imx35_gpio_hwdata;

	mxc_gpio_hwtype = hwtype;
}

static int mxc_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
        NOT_IMPLEMENTED;
	struct mxc_gpio_port *port =
			container_of(gc, struct mxc_gpio_port, gc);

	return vmm_host_irqdomain_find_mapping(port->domain, offset);
}

static void bcm_kona_gpio_lock_gpio(struct bcm_kona_gpio *kona_gpio,
					unsigned gpio)
{
        NOT_IMPLEMENTED;
	/*u32 val;*/
	/*unsigned long flags;*/
	/*int bank_id = GPIO_BANK(gpio);*/

	/*spin_lock_irqsave(&kona_gpio->lock, flags);*/

	/*val = readl(kona_gpio->reg_base + GPIO_PWD_STATUS(bank_id));*/
	/*val |= BIT(gpio);*/
	/*bcm_kona_gpio_write_lock_regs(kona_gpio->reg_base, bank_id, val);*/

	/*spin_unlock_irqrestore(&kona_gpio->lock, flags);*/
}

static int bcm_gpio_request(struct gpio_chip *chip, unsigned gpio)
{
	/*struct bcm_gpio *_gpio = chip->gpiodev->data;*/

	/*bcm_kona_gpio_unlock_gpio(_gpio, gpio);*/
	return 0;
}

#define PORT_NAME_LEN	12
static const struct gpio_chip template_chip = {
	.label            = "bcm-gpio",
	.owner            = THIS_MODULE,
	.request          = NULL,//bcm_gpio_request,
	.free             = NULL,
	.get_direction    = NULL,
	.direction_input  = NULL,
	.get              = NULL,
	.direction_output = NULL,
	.set              = NULL,
	.set_debounce     = NULL,
	.to_irq           = NULL,
	.base             = 0,
};

static int bcm_get_num_gpio(struct device_node * np, struct vmm_device * dev) {
        int ret = vmm_devtree_irq_count(np);
        if (ret == 0) {
            dev_err(dev, "Couldn't determine # GPIO banks\n");
            return -ENOENT;
        }
        if (ret > GPIO_MAX_BANK_NUM) {
            dev_err(dev, "Too many GPIO banks configured (max=%d)\n", GPIO_MAX_BANK_NUM);
            return -ENXIO;
        }
        return ret;
}
static void *bgpio_map(struct vmm_device *dev,
		       const char *name)
{
	int rc;
	virtual_addr_t ret;

	rc = vmm_devtree_regname_to_regset(dev->of_node, name);
	if (rc < 0)
		return NULL;

	rc = vmm_devtree_regmap_byname(dev->of_node, &ret, name);
	if (rc)
		return VMM_ERR_PTR(rc);

	return (void *)ret;
}

static int mxc_gpio_probe(struct vmm_device *dev,
			  const struct vmm_devtree_nodeid *devid)
{
        DPRINTF("dev->name = %s\n", dev->name);
        DPRINTF("dev->type->name = %s\n", dev->type ? dev->type->name : "None");
        DPRINTF("dev->driver->name = %s\n", dev->driver ? dev->driver->name : "None");
	struct device_node *np = dev->of_node;
        struct bcm_gpio *gpio = NULL;
        physical_size_t sz;
        int flags=0;
	int err = VMM_OK;

        DPRINTF("Initialisation of struct bcm_gpio gpio\n");
        gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
        if (!gpio)
                return -ENOMEM;

        // Init gpio register
        DPRINTF("Setting up registers\n");
	err = vmm_devtree_request_regmap(np, (virtual_addr_t *)&gpio->base, 0,
					 "BCM GPIO");
	if (VMM_OK != err) {
		dev_err(dev, "fail to map registers from the device tree\n");
		goto out_regmap;
	}
        // Get the number of bank
        DPRINTF("Counting number of GPIO bank\n");
        gpio->num_banks = bcm_get_num_gpio(np, dev);
        if(gpio->num_banks < 0) { // less than 0 return value means error
            return gpio->num_banks;
        }
        DPRINTF("%d banks\n", gpio->num_banks);

        // Init the banks
        DPRINTF("Initialisation of GPIO bank\n");
        gpio->banks = devm_kzalloc(dev,
                                   sizeof(*gpio->banks) * gpio->num_banks,
                                   GFP_KERNEL);
        if (!gpio->banks) {
            DPRINTF("No GPIO bank !\n");
            return -ENOMEM;
        }

        // Init the gpio_chip
        /*gpio->gc         = template_chip;*/
        /*FIXME find the right way to get the register*/
        err = vmm_devtree_regsize(dev->of_node, &sz, err);
        if (err) {
            DPRINTF("devtree_regsize failed with %d\n", err);
            return err;
        }
        DPRINTF("sz=%lu\n", sz);
        /*err = vmm_devtree_regname_to_regset(dev->of_node, "dat");*/
        /*if (err < 0) {*/
            /*DPRINTF("devtree_regname_to_regset failed with %d\n", err);*/
            /*return err;*/
        /*}*/

        void* dat = bgpio_map(dev, "dat");
        if (VMM_IS_ERR(dat)) {
            DPRINTF("error in bgpio_map(dev, dat)\n");
            return VMM_PTR_ERR(dat);
        }

        void* set = bgpio_map(dev, "set");
        if (VMM_IS_ERR(set)) {
            DPRINTF("error in bgpio_map(dev, set)\n");
            return VMM_PTR_ERR(set);
        }

        void* clr = bgpio_map(dev, "clr");
        if (VMM_IS_ERR(clr)) {
            DPRINTF("error in bgpio_map(dev, clr)\n");
            return VMM_PTR_ERR(clr);
        }

        void* dirout = bgpio_map(dev, "dirout");
        if (VMM_IS_ERR(dirout)) {
            DPRINTF("error in bgpio_map(dev, dirout)\n");
            return VMM_PTR_ERR(dirout);
        }

        void* dirin = bgpio_map(dev, "dirin");
        if (VMM_IS_ERR(dirin)) {
            DPRINTF("error in bgpio_map(dev, dirin)\n");
            return VMM_PTR_ERR(dirin);
        }
        DPRINTF("Calling bgpio_init\n");
        err = bgpio_init(&gpio->gc,            //gpio_chip
                         dev,                  //device
                         4,                    //size
                         gpio->base + GPLEV0,  //dat register
                         gpio->base + GPSET0,  //set register
                         gpio->base + GPCLR1,  //clr register
                         gpio->base + GPFSEL0, //dirout register
                         NULL,                 //dirin register
                         flags);               // flags
        if (err) {
            DPRINTF("bgpio_init exit with error %d\n", err);
            return err;
        }
        gpio->gc.of_node = dev->of_node;
        gpio->gc.ngpio   = gpio->num_banks * GPIO_PER_BANKS;
        DPRINTF("Adding the GPIO chip : %s\n", gpio->gc.label);
        DPRINTF("base=%p, sizeof(*base)=%lu\n",gpio->base, sizeof(*gpio->base));
	err = gpiochip_add(&gpio->gc);
	if (err) {
            DPRINTF("gpiochip_add returns %d", err);
            goto out_bgpio;
        }

        DPRINTF("Everything went well !\n");
        return VMM_OK;

/*out_gpiochip_remove:*/
	/*gpiochip_remove(&gpio->gc);*/
out_bgpio:
	vmm_devtree_regunmap_release(np, (virtual_addr_t)gpio->base, 0);
out_regmap:
	devm_kfree(dev, gpio);
	dev_info(dev, "%s failed with errno %d\n", __func__, err);
	return err;
}

static struct vmm_driver mxc_gpio_driver = {
	.name		= "gpio-bcm",
	.match_table	= mxc_gpio_dt_ids,
	.probe		= mxc_gpio_probe,
};

static int __init gpio_mxc_init(void)
{
        DPRINTF("RPI3 GPIO driver\n");
	return vmm_devdrv_register_driver(&mxc_gpio_driver);
}
#if 0
postcore_initcall(gpio_mxc_init);
#endif

VMM_DECLARE_MODULE("Rasberry Pi 3 GPIO driver",
		   "<>",
		   "GPL",
		   1,
		   gpio_mxc_init,
		   NULL);

#if 0
MODULE_AUTHOR("Freescale Semiconductor, "
	      "Daniel Mack <danielncaiaq.de>, "
	      "Juergen Beisert <kernel@pengutronix.de>");
MODULE_DESCRIPTION("Freescale MXC GPIO");
MODULE_LICENSE("GPL");
#endif
