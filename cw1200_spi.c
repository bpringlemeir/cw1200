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
#include <linux/platform_device.h>
#include <linux/netdevice.h>


static struct workqueue_struct *cw1200_fwio_workqueue = NULL;
static struct spi_device *cw1200_spi_dev;
static int cw1200_fw_reset_cnt = 0;

MODULE_AUTHOR("Solomon Peachy <speachy@sagrad.com>");
MODULE_DESCRIPTION("mac80211 ST-Ericsson CW1200 SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cw1200_wlan_spi");


struct sbus_priv {
	struct spi_device	*func;
	struct cw1200_common	*core;
	const struct cw1200_platform_data *pdata;
	spinlock_t		lock;
	sbus_irq_handler	irq_handler;
	void			*irq_priv;
// VLAD:
	spinlock_t      fw_reset_lock;
};


static struct platform_device *cw1200_fwio_dev;
static void cw1200_fw_failure_job(struct work_struct *work);
static int cw1200_fwio_prepare(struct device *dev)
{
 struct sbus_priv *self = NULL;

 self = spi_get_drvdata(cw1200_spi_dev);
 if(self) {
  self->core->cw1200_fw_error_status = CW1200_FW_ERR_DOTERMINATE;
  wake_up_interruptible(&self->core->cw1200_fw_wq);
  wait_event_interruptible(self->core->cw1200_fw_wq,CW1200_FW_ERR_TERMINATED == self->core->cw1200_fw_error_status);
 }

 return 0;
}


static void cw1200_fwio_complete(struct device *dev)
{
 struct sbus_priv *self = NULL;

 self = spi_get_drvdata(cw1200_spi_dev);
 if(self) {
  self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
  init_waitqueue_head(&self->core->cw1200_fw_wq);
  INIT_WORK(&self->core->cw1200_fw_failure_work, cw1200_fw_failure_job);
  queue_work(cw1200_fwio_workqueue,&self->core->cw1200_fw_failure_work);
 }

}

static const struct dev_pm_ops cw1200_fwio_ops = {
  .prepare =  cw1200_fwio_prepare,
  .complete =  cw1200_fwio_complete,
};

static struct class *cw1200_class;

static int cw1200_fwio_probe(struct platform_device *pdev)
{

	cw1200_class = class_create(THIS_MODULE, "cw1200_fw_io");
	  if (IS_ERR(cw1200_class)) {
		  printk(KERN_ERR"========[  failed to create cw1200_fw_io class ]=========\n");
	  }
    device_create(cw1200_class,&pdev->dev,0,NULL,"cw1200_fw_io");

	return 0;
}

static int cw1200_fwio_remove(struct platform_device *pdev)
{
	device_destroy(cw1200_class,0);
    class_destroy(cw1200_class);
	return 0;
}


static struct platform_driver cw1200_fwio_driver = {
	.probe = cw1200_fwio_probe,
	.remove = cw1200_fwio_remove,
	.driver = {
		.name = "cw1200_fw_io",
		.pm = &cw1200_fwio_ops,
	},
};



#ifdef CONFIG_CW1200_PM
#define USE_IRQ_WAKE
#endif
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
	usleep_range(1000, 20000);
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

#ifdef CONFIG_CW1200_PM
static int cw1200_spi_pm(struct sbus_priv *self, bool suspend)
{
	int ret = 0;
	const struct resource *irq = self->pdata->irq;

#ifdef USE_IRQ_WAKE
	if (irq)
		ret = set_irq_wake(irq->start, suspend);
#endif

	return ret;
}
#endif

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
	.power_mgmt		= cw1200_spi_pm,
#endif
	.irq_enable             = cw1200_spi_irq_enable,
};

static int cw1200_spi_suspend(struct spi_device *pdev, pm_message_t state)
{

 const struct cw1200_platform_data *pdata;
 struct sbus_priv *self = NULL;
 const struct resource *reset;

 self = spi_get_drvdata(pdev);
 pdata = cw1200_get_platform_data();
 reset = pdata->reset;

 if(self) {
   if(self->core) {
    cw1200_core_release(self->core);
    self->core = NULL;
   }
 }
/**/


 if (reset) {
  gpio_set_value(reset->start, 0);
 }

 if (pdata->power_ctrl)
	pdata->power_ctrl(pdata, false);

 return 0;
}

static int cw1200_spi_resume(struct spi_device *pdev)
{
 const struct resource *reset;
 const struct cw1200_platform_data *pdata = cw1200_get_platform_data();
 struct sbus_priv *self = spi_get_drvdata(pdev);



 if (pdata->power_ctrl) {
 	pdata->power_ctrl(pdata, true);
	msleep(50);
 }

 reset = pdata->reset;
 if (reset) {
		gpio_direction_output(reset->start, 1);
		/* It is not stated in the datasheet, but at least some of devices
		 * have problems with reset if this stage is omited. */
		msleep(50);
		gpio_set_value(reset->start, 0);
		/* A valid reset shall be obtained by maintaining WRESETN
		 * active (low) for at least two cycles of LP_CLK after VDDIO
		 * is stable within it operating range. */
		usleep_range(1000, 20000);
		gpio_set_value(reset->start, 1);
		/* The host should wait 30 ms after the WRESETN release
		 * for the on-chip LDO to stabilize */
		msleep(30);

 }


/************************/
 cw1200_core_probe(&cw1200_spi_sbus_ops,
 				   self, &pdev->dev, &self->core,
 				   self->pdata->pll_init_val,
 				   self->pdata->macaddr);

 return 0;
}

static void cw1200_fw_failure_job(struct work_struct *work)
{
 int status;
 pm_message_t evt;
 struct sbus_priv *self;

 struct cw1200_common *priv =
		container_of(work, struct cw1200_common, cw1200_fw_failure_work);

 evt.event = 0;

 status = wait_event_interruptible(priv->cw1200_fw_wq,CW1200_FW_ERR_IDLE != priv->cw1200_fw_error_status);
 if(status < 0 ) {

  dev_err(&cw1200_spi_dev->dev,"%s failed to wait for fw failure %d",__func__,status);

 } else if (CW1200_FW_ERR_DOALARM == priv->cw1200_fw_error_status) {
  if(cw1200_fwio_dev) { /* sending mdev event to initiate user-space driven wifi reset sequence */
    dev_info(&cw1200_spi_dev->dev,"cw1200 firmware exception detected, preparing to reset\n");
    platform_driver_unregister(&cw1200_fwio_driver);
	cw1200_fwio_dev->dev.platform_data = NULL;
	platform_device_unregister(cw1200_fwio_dev);
	cw1200_fwio_dev = NULL;

    status = wait_event_interruptible_timeout(priv->cw1200_fw_wq,priv->cw1200_fw_error_status > CW1200_FW_ERR_DOALARM,HZ*10);
    if(status < 0 ) {
     dev_err(&cw1200_spi_dev->dev,"%s failed to wait for fw reset command %d",__func__,status);
    } else if(0 == status) { /* timeout*/
     dev_err(&cw1200_spi_dev->dev,"cw1200 reset fw command timeout\n");
    } else if( CW1200_FW_ERR_DORESET == priv->cw1200_fw_error_status) {
     dev_info(&cw1200_spi_dev->dev,"executing cw1200 firmware reset\n");
     cw1200_fw_reset_cnt++;
     cw1200_spi_suspend(cw1200_spi_dev,evt);
     msleep_interruptible(200);
     cw1200_spi_resume(cw1200_spi_dev);
    } else if (CW1200_FW_ERR_DOTERMINATE ==  priv->cw1200_fw_error_status) {
    	   goto terminate;
    } else goto oops;
  }
 } else if (CW1200_FW_ERR_DORESET ==  priv->cw1200_fw_error_status) {
     dev_info(&cw1200_spi_dev->dev,"external request to reset cw1200 firmware received\n");
     cw1200_spi_suspend(cw1200_spi_dev,evt);
     msleep_interruptible(200);
     cw1200_spi_resume(cw1200_spi_dev);
 } else if (CW1200_FW_ERR_DOTERMINATE ==  priv->cw1200_fw_error_status) {
   goto terminate;
 } else goto oops;
 self = spi_get_drvdata(cw1200_spi_dev);


 self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
 init_waitqueue_head(&self->core->cw1200_fw_wq);
 INIT_WORK(&self->core->cw1200_fw_failure_work, cw1200_fw_failure_job);


 if(!cw1200_fwio_dev) {
   int ret;
   ret = platform_driver_register(&cw1200_fwio_driver);
   cw1200_fwio_dev = platform_device_alloc("cw1200_fw_io", 0);
   ret = platform_device_add(cw1200_fwio_dev);
	 if (ret) {
		kfree(cw1200_fwio_dev);
	 }
 }
 queue_work(cw1200_fwio_workqueue,&self->core->cw1200_fw_failure_work);
 return;
oops:
 dev_err(&cw1200_spi_dev->dev,"%s() unexpected event: %d\n",__func__,priv->cw1200_fw_error_status);
terminate:
 dev_info(&cw1200_spi_dev->dev,"%s() termination \n",__func__);
 priv->cw1200_fw_error_status = CW1200_FW_ERR_TERMINATED;
 wake_up_interruptible(&priv->cw1200_fw_wq);
}


static ssize_t dev_type_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return sprintf(buf,"%d\n",cw1200_fw_reset_cnt);
}

static ssize_t cw1200_do_reset(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
 if(!strcmp(buf,"RESET\n")) {

  struct sbus_priv *self = NULL;
  pm_message_t evt;

  evt.event = 0;
  self = spi_get_drvdata(cw1200_spi_dev);
  if(self) {
   self->core->cw1200_fw_error_status = CW1200_FW_ERR_DORESET;
   wake_up_interruptible(&self->core->cw1200_fw_wq);
  } else {
   dev_err(&cw1200_spi_dev->dev,"%s self == NULL\n",__func__);
  }
 } else {
	 return -EACCES;
 }
 return size;
}

DEVICE_ATTR(cw1200_fw_ok, S_IRUGO, dev_type_show, NULL);
DEVICE_ATTR(cw1200_fw_reset, S_IWUGO, NULL, cw1200_do_reset);


/* Probe Function to be called by SPI stack when device is discovered */
static int __devinit cw1200_spi_probe(struct spi_device *func) 
{
	const struct cw1200_platform_data *plat_data;
	struct sbus_priv *self;
	int status;

	cw1200_fwio_workqueue = create_workqueue("cw1200_fwio_q");
	if( NULL == cw1200_fwio_workqueue) {
		dev_err(&func->dev,"cw1200_fwio_workqueue == NULL\n");
		return -EFAULT;
	}

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

	spi_set_drvdata(func,self);

	spin_lock_init(&self->lock);
	self->pdata = plat_data;
	self->func = func;

	spin_lock_init(&self->fw_reset_lock);

	status = cw1200_core_probe(&cw1200_spi_sbus_ops,
				   self, &func->dev, &self->core, 
				   self->pdata->pll_init_val,
				   self->pdata->macaddr);

    if( 0 == status) {
     self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
	 init_waitqueue_head(&self->core->cw1200_fw_wq);
	 INIT_WORK(&self->core->cw1200_fw_failure_work, cw1200_fw_failure_job);
	 queue_work(cw1200_fwio_workqueue,&self->core->cw1200_fw_failure_work);

    }

	cw1200_spi_dev = func;

	status = device_create_file(&func->dev,&dev_attr_cw1200_fw_ok);
	if(status) dev_err(&func->dev, "dev_attr_dev_type %d", status);

	status = device_create_file(&func->dev,&dev_attr_cw1200_fw_reset);
	if(status) dev_err(&func->dev, "dev_attr_dev_type %d", status);



#if defined(CONFIG_DEBUG_FS)
	{
	 void cw1200_create_debugfs(void);
     cw1200_create_debugfs();
	}
#endif

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

	destroy_workqueue(cw1200_fwio_workqueue);
	return 0;
}




static struct spi_driver spi_driver = {
	.probe		= cw1200_spi_probe,
	.remove		= __devexit_p(cw1200_spi_disconnect),
#if CONFIG_CW1200_PM
    .suspend = cw1200_spi_suspend,
    .resume  = cw1200_spi_resume,
#endif
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
{
	ret = platform_driver_register(&cw1200_fwio_driver);
	if (ret)
		return ret;
	cw1200_fwio_dev = platform_device_alloc("cw1200_fw_io", 0);
	if (!cw1200_fwio_dev) {
		platform_driver_unregister(&cw1200_fwio_driver);
		return -ENOMEM;
	}
	ret = platform_device_add(cw1200_fwio_dev);
	if (ret) {
		kfree(cw1200_fwio_dev);
	}

}
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

#ifdef CONFIG_DEBUG_FS
static int debugfs_cmd;
static int cw1200_spi_suspend(struct spi_device *pdev, pm_message_t state);
static int cw1200_spi_resume(struct spi_device *pdev);
static int cw1200_debugfs_set_cmd(void *data, u64 val)
{
 debugfs_cmd = (int)val;
 printk(KERN_CRIT"VLAD: %s(%d)\n",__func__,debugfs_cmd);
 switch(debugfs_cmd) {
 case 1: {
  pm_message_t evt;
  evt.event = 0;


  cw1200_spi_suspend(cw1200_spi_dev,evt);
  msleep_interruptible(100);
  cw1200_spi_resume(cw1200_spi_dev);

 } break;

 case 2: {
   if(cw1200_fwio_dev) {
	platform_driver_unregister(&cw1200_fwio_driver);
	cw1200_fwio_dev->dev.platform_data = NULL;
	platform_device_unregister(cw1200_fwio_dev);
	cw1200_fwio_dev = NULL;
   }
 } break;
 case 3: {
   if(!cw1200_fwio_dev) {
     int ret;
     ret = platform_driver_register(&cw1200_fwio_driver);
     cw1200_fwio_dev = platform_device_alloc("cw1200_fw_io", 0);
     ret = platform_device_add(cw1200_fwio_dev);
	 if (ret) {
		kfree(cw1200_fwio_dev);
	 }
   }
 } break;
 case 4: {
  struct sbus_priv *self = NULL;
  self = spi_get_drvdata(cw1200_spi_dev);
  self->core->cw1200_fw_error_status = CW1200_FW_ERR_DOALARM;
  wake_up_interruptible(&self->core->cw1200_fw_wq);

 } break;

 }

 return 0;
}
static int cw1200_debugfs_get_cmd(void *data, u64* val)
{
 *val = (u64)debugfs_cmd;
 return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_cw1200_debugfs, cw1200_debugfs_get_cmd,cw1200_debugfs_set_cmd, "%08lld\n");
void cw1200_create_debugfs(void)
{
 struct dentry *regs;
 static const mode_t dm = S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH;

 regs = debugfs_create_dir("cw1200_spi", NULL);
 if(regs) {
  debugfs_create_file("debug_cmd", dm, regs,(void*)&debugfs_cmd, &fops_cw1200_debugfs);
 }

}

#endif


module_init(cw1200_spi_init);
module_exit(cw1200_spi_exit);
