/*
 * Mac80211 SPI driver for ST-Ericsson CW1200 device
 *
 * Copyright (c) 2011, Sagrad Inc.
 * Author:  Solomon Peachy <speachy@sagrad.com>
 * 
 * Based on cw1200_gpio.c
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <net/mac80211.h>

#include <linux/spi/spi.h>
#include <linux/device.h>

#include "cw1200.h"
#include "sbus.h"
#include "cw1200_plat.h"
#include "hwio.h"

MODULE_AUTHOR("Solomon Peachy <speachy@sagrad.com>");
MODULE_DESCRIPTION("mac80211 ST-Ericsson CW1200 SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cw1200_wlan_spi");

//#define USE_IRQ_WAKE
//#define USE_INTERNAL_RESOURCE_LIST
// XXX The intent is that this info is part of the board platform data in arch/mach-xxxx/mach-yyyy.c

#ifdef USE_INTERNAL_RESOURCE_LIST
static struct resource cw1200_href_resources[] = {
        {
                .start = 215,  // fix me as appropriate
                .end = 215,    //
                .flags = IORESOURCE_IO,
                .name = "cw1200_wlan_reset",
        },
        {
                .start = 161, // fix me as appropriate
                .end = 161,   // 
                .flags = IORESOURCE_IRQ,
                .name = "cw1200_wlan_irq",
        },
};

static int cw1200_power_ctrl(const struct cw1200_platform_data *pdata,
                             bool enable) 
{
	/* Turn PWR_EN off and on as appropriate. */
	/* Note this is not needed when there's a hardware reset circuit */
	return 0;
}

static int cw1200_clk_ctrl(const struct cw1200_platform_data *pdata,
			   bool enable) 
{
	/* Turn CLK_32K off and on as appropriate. */
	/* Note this is not needed if it's always on */
	return 0;
}

static struct cw1200_platform_data cw1200_platform_data = {
	.pll_init_val = DPLL_INIT_VAL_CW1200_38_4MHZ,
	.reset = &cw1200_href_resources[0],
	.irq = &cw1200_href_resources[1],
	.power_ctrl = cw1200_power_ctrl,
};

const struct cw1200_platform_data *cw1200_get_platform_data(void)
{
        return &cw1200_platform_data;
}
EXPORT_SYMBOL_GPL(cw1200_get_platform_data);
#endif

struct sbus_priv {
	struct spi_device	*func;
	struct cw1200_common	*core;
	const struct cw1200_platform_data *pdata;
	spinlock_t		lock;
	sbus_irq_handler	irq_handler;
	void			*irq_priv;
};

/* sbus_ops implemetation */

#define SDIO_TO_SPI_ADDR(addr) ((addr & 0x1f)>>2)
#define SET_WRITE 0x7FFF //usage: and operation
#define SET_READ 0x8000  //usage: or operation

static int cw1200_spi_memcpy_fromio(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count)
{
	int ret;
        uint16_t regaddr;
        struct spi_message      m;

#ifndef SPI_SINGLETRANS
        struct spi_transfer     t_addr = {
                    .tx_buf         = &regaddr,
                    .len            = sizeof(regaddr),
            };
#endif
        struct spi_transfer     t_msg = {
                    .rx_buf         = dst,
#ifdef SPI_SINGLETRANS
                    .tx_buf         = dst,
#endif
                    .len            = count,
            };

        addr = SDIO_TO_SPI_ADDR(addr);

	regaddr=(addr)<<12;
	regaddr|=SET_READ;
	regaddr|=(count>>1);
	regaddr=cpu_to_le16(regaddr);

        /* We have to byteswap if the SPI bus is limited to 8b operation */
	if (self->func->bits_per_word == 8)
                regaddr=cpu_to_be16(regaddr);

#ifdef SPI_SINGLETRANS
        memcpy(dst, &regaddr, sizeof(regaddr));
#endif
        spi_message_init(&m);

#ifndef SPI_SINGLETRANS
        spi_message_add_tail(&t_addr, &m);
#endif
        spi_message_add_tail(&t_msg, &m);
        ret = spi_sync(self->func, &m);

        /* We have to byteswap if the SPI bus is limited to 8b operation */
        if (self->func->bits_per_word == 8) {
                int i;
                uint16_t *buf = (uint16_t*)dst;

                for (i = 0 ; i < (count>>1) ; i++) {
                        buf[i] = be16_to_cpu(buf[i]);
                }
        }

#if 0
	{
		int i = 0;
		char *ptr = dst;
		printk(KERN_ERR "READ  %08x (%04d): ", addr, count);
		for (i = 0 ; i < count ; i++) {
			printk("%02x ", ptr[i]);
		}
		printk("\n");
	}
#endif

	return ret;
}

static int cw1200_spi_memcpy_toio(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count)
{
        int rval;
        uint16_t regaddr;
        struct spi_transfer     t_addr = {
                    .tx_buf         = &regaddr,
                    .len            = sizeof(regaddr),
            };
        struct spi_transfer     t_msg = {
                    .tx_buf         = src,
                    .len            = count,
            };
        struct spi_message      m;

        addr = SDIO_TO_SPI_ADDR(addr);

#if 0
	{
		int i = 0;
		char *ptr = src;
		printk(KERN_ERR "WRITE %08x (%04d): ", addr, count);
		for (i = 0 ; i < count ; i++) {
			printk("%02x ", ptr[i]);
		}
		printk("\n");
	}
#endif

        regaddr=(addr)<<12;
        regaddr&=SET_WRITE;
        regaddr|=(count>>1);
        regaddr=cpu_to_le16(regaddr);

        /* We have to byteswap if the SPI bus is limited to 8b operation */
        if (self->func->bits_per_word == 8)
                regaddr=cpu_to_be16(regaddr);

        /* We have to byteswap if the SPI bus is limited to 8b operation */
        if (self->func->bits_per_word == 8) {
                int i;
                uint16_t *buf = (uint16_t*)src;

                for (i = 0 ; i < (count>>1) ; i++) {
                        buf[i] = cpu_to_be16(buf[i]);
                }
        }
        spi_message_init(&m);
        spi_message_add_tail(&t_addr, &m);
        spi_message_add_tail(&t_msg, &m);
        rval = spi_sync(self->func, &m);

        /* We have to byteswap if the SPI bus is limited to 8b operation */
        if (self->func->bits_per_word == 8) {
                int i;
                uint16_t *buf = (uint16_t*)src;

                for (i = 0 ; i < (count>>1) ; i++) {
                        buf[i] = be16_to_cpu(buf[i]);
                }
        }

	return rval;
}

static void cw1200_spi_lock(struct sbus_priv *self)
{
	return;
}

static void cw1200_spi_unlock(struct sbus_priv *self)
{
	return;
}

static irqreturn_t cw1200_gpio_irq_handler(int irq, void *dev_id)
{
	struct sbus_priv *self = dev_id;

	BUG_ON(!self);
	if (self->irq_handler)
		self->irq_handler(self->irq_priv);
	return IRQ_HANDLED;
}

static int cw1200_spi_request_irq(struct sbus_priv *self,
				  irq_handler_t handler)
{
	int ret;
	const struct resource *irq = self->pdata->irq;

	ret = request_any_context_irq(irq->start, handler,
			IRQF_TRIGGER_RISING, irq->name, self);
	if (WARN_ON(ret < 0))
		goto exit;

#ifdef USE_IRQ_WAKE
	ret = enable_irq_wake(irq->start);
	if (WARN_ON(ret))
		goto free_irq;
#endif

	return 0;

#ifdef USE_IRQ_WAKE
free_irq:
	free_irq(irq->start, self);
#endif
exit:
	return ret;
}

static void cw1200_spi_irq_enable(struct sbus_priv *self, int enable)
{
	return;
}

static int cw1200_spi_irq_subscribe(struct sbus_priv *self,
				    sbus_irq_handler handler,
				    void *priv)
{
	int ret;
	unsigned long flags;

	if (!handler)
		return -EINVAL;

	spin_lock_irqsave(&self->lock, flags);
	self->irq_priv = priv;
	self->irq_handler = handler;
	spin_unlock_irqrestore(&self->lock, flags);

	printk(KERN_DEBUG "SW IRQ subscribe\n");
	ret = cw1200_spi_request_irq(self, cw1200_gpio_irq_handler);
	return ret;
}

static int cw1200_spi_irq_unsubscribe(struct sbus_priv *self)
{
	int ret = 0;
	unsigned long flags;
	const struct resource *irq = self->pdata->irq;

	WARN_ON(!self->irq_handler);
	if (!self->irq_handler)
		return 0;

	printk(KERN_DEBUG "SW IRQ unsubscribe\n");
#ifdef USE_IRQ_WAKE
	disable_irq_wake(irq->start);
#endif
	free_irq(irq->start, self);

	spin_lock_irqsave(&self->lock, flags);
	self->irq_priv = NULL;
	self->irq_handler = NULL;
	spin_unlock_irqrestore(&self->lock, flags);

	return ret;
}

static int cw1200_spi_off(const struct cw1200_platform_data *pdata)
{
	const struct resource *reset = pdata->reset;
	if (!reset) return 0;
	gpio_set_value(reset->start, 0);
	gpio_free(reset->start);
	return 0;
}

static int cw1200_spi_on(const struct cw1200_platform_data *pdata)
{
	const struct resource *reset = pdata->reset;
	if (!reset) return 0;
	gpio_request(reset->start, reset->name);
	gpio_direction_output(reset->start, 1);
	/* It is not stated in the datasheet, but at least some of devices
	 * have problems with reset if this stage is omited. */
	msleep(50);
	gpio_set_value(reset->start, 0);
	/* A valid reset shall be obtained by maintaining WRESETN
	 * active (low) for at least two cycles of LP_CLK after VDDIO
	 * is stable within it operating range. */
	msleep(50);
	gpio_set_value(reset->start, 1);
	/* The host should wait 30 ms after the WRESETN release
	 * for the on-chip LDO to stabilize */
	msleep(30);
	return 0;
}

static int cw1200_spi_reset(struct sbus_priv *self)
{
	cw1200_spi_off(self->pdata);
	msleep(1000);
	cw1200_spi_on(self->pdata);
	return 0;
}

static size_t cw1200_spi_align_size(struct sbus_priv *self, size_t size)
{
	return size & 1 ? size + 1 : size;
}

static struct sbus_ops cw1200_spi_sbus_ops = {
	.sbus_memcpy_fromio	= cw1200_spi_memcpy_fromio,
	.sbus_memcpy_toio	= cw1200_spi_memcpy_toio,
	.lock			= cw1200_spi_lock,
	.unlock			= cw1200_spi_unlock,
	.irq_subscribe		= cw1200_spi_irq_subscribe,
	.irq_unsubscribe	= cw1200_spi_irq_unsubscribe,
	.reset			= cw1200_spi_reset,
	.align_size		= cw1200_spi_align_size,
#ifdef CONFIG_CW1200_PM
	.power_mgmt		= cw1200_spi_pm, // XXX not implemented.
#endif
	.irq_enable             = cw1200_spi_irq_enable,
};

/* Probe Function to be called by SPI stack when device is discovered */
static int __devinit cw1200_spi_probe(struct spi_device *func) 
{
	const struct cw1200_platform_data *plat_data;
	struct sbus_priv *self;
	int status;

	plat_data = cw1200_get_platform_data();

	/* Sanity check speed */
	if (func->max_speed_hz > 52000000)
                func->max_speed_hz = 52000000;
	if (func->max_speed_hz < 1000000)
		func->max_speed_hz = 1000000;

	/* Fix up transfer size */
	if (plat_data->spi_bits_per_word)
		func->bits_per_word = plat_data->spi_bits_per_word;
        if (!func->bits_per_word)
                func->bits_per_word = 16;

	/* And finally.. */
        func->mode = SPI_MODE_0;

        cw1200_dbg(CW1200_DBG_INIT, "SPI Probe called (CS %d M %d BPW %d CLK %d)\n", 
		   func->chip_select, func->mode, func->bits_per_word, func->max_speed_hz);

        if (spi_setup(func)) {
                printk(KERN_ERR "spi_setup() failed!\n");
                return -1;
        }

	self = kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		cw1200_dbg(CW1200_DBG_ERROR, "Can't allocate SPI sbus_priv.");
		return -ENOMEM;
	}

	spin_lock_init(&self->lock);
	self->pdata = plat_data;
	self->func = func;

	status = cw1200_core_probe(&cw1200_spi_sbus_ops,
				   self, &func->dev, &self->core, 
				   self->pdata->pll_init_val,
				   self->pdata->macaddr);

	return status;
}

/* Disconnect Function to be called by SPI stack when device is disconnected */
static int __devexit cw1200_spi_disconnect(struct spi_device *func)
{
	struct sbus_priv *self = spi_get_drvdata(func);

	if (self) {
		if (self->core) {
			cw1200_core_release(self->core);
			self->core = NULL;
		}
		kfree(self);
	}
	return 0;
}

static struct spi_driver spi_driver = {
	.probe		= cw1200_spi_probe,
	.remove		= __devexit_p(cw1200_spi_disconnect),
	.driver = {
		.name		= "cw1200_wlan_spi",
		.bus            = &spi_bus_type,
		.owner          = THIS_MODULE,
	},
};

/* Init Module function -> Called by insmod */
static int __init cw1200_spi_init(void)
{
	const struct cw1200_platform_data *pdata;
	int ret;

	pdata = cw1200_get_platform_data();

	if (pdata->clk_ctrl) {
		ret = pdata->clk_ctrl(pdata, true);
		if (ret)
			goto err_clk;
		msleep(20); // XXX may be lowered?
	}

	if (pdata->power_ctrl) {
		ret = pdata->power_ctrl(pdata, true);
		if (ret)
			goto err_power;
		msleep(250);  // XXX can be lowered probably
	}

	ret = cw1200_spi_on(pdata);
	if (ret)
		goto err_on;

	ret = spi_register_driver(&spi_driver);
	if (ret)
		goto err_reg;

	return 0;

err_reg:
	cw1200_spi_off(pdata);
err_on:
	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
err_power:
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
err_clk:
	return ret;
}

/* Called at Driver Unloading */
static void __exit cw1200_spi_exit(void)
{
	const struct cw1200_platform_data *pdata;
	pdata = cw1200_get_platform_data();
	spi_unregister_driver(&spi_driver);
	cw1200_spi_off(pdata);

	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);
}

module_init(cw1200_spi_init);
module_exit(cw1200_spi_exit);
