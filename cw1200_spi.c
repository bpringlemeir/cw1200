/*
 * Mac80211 SPI driver for ST-Ericsson CW1200 device
 *
 * Copyright (c) 2011, Sagrad Inc.
 * Author:  Solomon Peachy <speachy@sagrad.com>
 *
 * Based on cw1200_sdio.c
 * Copyright (c) 2010, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@lockless.no>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <net/mac80211.h>

#include <linux/spi/spi.h>
#include <linux/device.h>

#include "cw1200.h"
#include "sbus.h"
#include <linux/cw1200_platform.h>

#include "hwio.h"

MODULE_AUTHOR("Solomon Peachy <speachy@sagrad.com>");
MODULE_DESCRIPTION("mac80211 ST-Ericsson CW1200 SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cw1200_wlan_spi");


static struct workqueue_struct *cw1200_fwio_workqueue = NULL;
static int cw1200_fw_reset_cnt = 0;
/* #define SPI_DEBUG */
static struct spi_device *cw1200_spi_dev;
struct sbus_priv {
	struct spi_device	*func;
	struct cw1200_common	*core;
	const struct cw1200_platform_data_spi *pdata;
	spinlock_t		lock; /* Serialize all bus operations */
	wait_queue_head_t       wq;
	int claimed;
	// VLAD:
		spinlock_t      fw_reset_lock;
		int manually_suspended;

};

static struct platform_device *cw1200_fwio_dev;
static void cw1200_fw_failure_job(struct work_struct *work);
static int cw1200_fwio_prepare(struct device *dev)
{
  struct sbus_priv *self = spi_get_drvdata(cw1200_spi_dev);

  if(self->manually_suspended) {
	  dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);

	  return 0;
  }
  dev_dbg(&cw1200_spi_dev->dev,"%s() \n",__func__);
  self->core->cw1200_fw_error_status = CW1200_FW_ERR_DOTERMINATE;
  wake_up_interruptible(&self->core->cw1200_fw_wq);
  wait_event_interruptible(self->core->cw1200_fw_wq,CW1200_FW_ERR_TERMINATED == self->core->cw1200_fw_error_status);

 return 0;
}


static void cw1200_fwio_complete(struct device *dev)
{
	struct sbus_priv *self = spi_get_drvdata(cw1200_spi_dev);

  if(self->manually_suspended) {
	  dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);
	  return;
  }
  dev_dbg(&cw1200_spi_dev->dev,"%s() \n",__func__);
  self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
  init_waitqueue_head(&self->core->cw1200_fw_wq);
  INIT_WORK(&self->core->cw1200_fw_failure_work,cw1200_fw_failure_job);
  queue_work(cw1200_fwio_workqueue,&self->core->cw1200_fw_failure_work);


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



#define SDIO_TO_SPI_ADDR(addr) ((addr & 0x1f)>>2)
#define SET_WRITE 0x7FFF /* usage: and operation */
#define SET_READ 0x8000  /* usage: or operation */

/*
   Notes on byte ordering:
   LE:  B0 B1 B2 B3
   BE:  B3 B2 B1 B0

   Hardware expects 32-bit data to be written as 16-bit BE words:

   B1 B0 B3 B2

*/

static int cw1200_spi_memcpy_fromio(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count)
{
	int ret, i;
	uint16_t regaddr;
	struct spi_message      m;

	struct spi_transfer     t_addr = {
		.tx_buf         = &regaddr,
		.len            = sizeof(regaddr),
	};
	struct spi_transfer     t_msg = {
		.rx_buf         = dst,
		.len            = count,
	};

	regaddr = (SDIO_TO_SPI_ADDR(addr))<<12;
	regaddr |= SET_READ;
	regaddr |= (count>>1);
	regaddr = cpu_to_le16(regaddr);

#ifdef SPI_DEBUG
	pr_info("READ : %04d from 0x%02x (%04x)\n", count, addr,
		le16_to_cpu(regaddr));
#endif

#if defined(__LITTLE_ENDIAN)
	/* We have to byteswap if the SPI bus is limited to 8b operation */
	if (self->func->bits_per_word == 8)
#endif
		regaddr = swab16(regaddr);

	spi_message_init(&m);
	spi_message_add_tail(&t_addr, &m);
	spi_message_add_tail(&t_msg, &m);
	ret = spi_sync(self->func, &m);

#ifdef SPI_DEBUG
	pr_info("READ : ");
	for (i = 0; i < t_addr.len; i++)
		printk("%02x ", ((u8 *)t_addr.tx_buf)[i]);
	printk(" : ");
	for (i = 0; i < t_msg.len; i++)
		printk("%02x ", ((u8 *)t_msg.rx_buf)[i]);
	printk("\n");
#endif

#if defined(__LITTLE_ENDIAN)
	/* We have to byteswap if the SPI bus is limited to 8b operation */
	if (self->func->bits_per_word == 8)
#endif
	{
		uint16_t *buf = (uint16_t *)dst;
		for (i = 0; i < ((count + 1) >> 1); i++)
			buf[i] = swab16(buf[i]);
	}

	return ret;
}

static int cw1200_spi_memcpy_toio(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count)
{
	int rval, i;
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

	regaddr = (SDIO_TO_SPI_ADDR(addr))<<12;
	regaddr &= SET_WRITE;
	regaddr |= (count>>1);
	regaddr = cpu_to_le16(regaddr);

#ifdef SPI_DEBUG
	pr_info("WRITE: %04d  to  0x%02x (%04x)\n", count, addr,
		le16_to_cpu(regaddr));
#endif

#if defined(__LITTLE_ENDIAN)
	/* We have to byteswap if the SPI bus is limited to 8b operation */
	if (self->func->bits_per_word == 8)
#endif
	{
		uint16_t *buf = (uint16_t *)src;
		regaddr = swab16(regaddr);
		for (i = 0; i < ((count + 1) >> 1); i++)
			buf[i] = swab16(buf[i]);
	}

#ifdef SPI_DEBUG
	pr_info("WRITE: ");
	for (i = 0; i < t_addr.len; i++)
		printk("%02x ", ((u8 *)t_addr.tx_buf)[i]);
	printk(" : ");
	for (i = 0; i < t_msg.len; i++)
		printk("%02x ", ((u8 *)t_msg.tx_buf)[i]);
	printk("\n");
#endif

	spi_message_init(&m);
	spi_message_add_tail(&t_addr, &m);
	spi_message_add_tail(&t_msg, &m);
	rval = spi_sync(self->func, &m);

#ifdef SPI_DEBUG
	pr_info("WROTE: %d\n", m.actual_length);
#endif

#if defined(__LITTLE_ENDIAN)
	/* We have to byteswap if the SPI bus is limited to 8b operation */
	if (self->func->bits_per_word == 8)
#endif
	{
		uint16_t *buf = (uint16_t *)src;
		for (i = 0; i < ((count + 1) >> 1); i++)
			buf[i] = swab16(buf[i]);
	}
	return rval;
}

static void cw1200_spi_lock(struct sbus_priv *self)
{
	unsigned long flags;

	DECLARE_WAITQUEUE(wait, current);

	might_sleep();

	add_wait_queue(&self->wq, &wait);
	spin_lock_irqsave(&self->lock, flags);
	while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (!self->claimed)
			break;
		spin_unlock_irqrestore(&self->lock, flags);
		schedule();
		spin_lock_irqsave(&self->lock, flags);
	}
	set_current_state(TASK_RUNNING);
	self->claimed = 1;
	spin_unlock_irqrestore(&self->lock, flags);
	remove_wait_queue(&self->wq, &wait);

	return;
}

static void cw1200_spi_unlock(struct sbus_priv *self)
{
	unsigned long flags;

	spin_lock_irqsave(&self->lock, flags);
	self->claimed = 0;
	spin_unlock_irqrestore(&self->lock, flags);
	wake_up(&self->wq);

	return;
}

static irqreturn_t cw1200_spi_irq_handler(int irq, void *dev_id)
{
	struct sbus_priv *self = dev_id;

	if (self->core) {
		cw1200_spi_lock(self);
		cw1200_irq_handler(self->core);
		cw1200_spi_unlock(self);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int cw1200_spi_irq_subscribe(struct sbus_priv *self)
{
	int ret;

	pr_debug("SW IRQ subscribe\n");

	ret = request_threaded_irq(self->func->irq, NULL,
				   cw1200_spi_irq_handler,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   "cw1200_wlan_irq", self);
	if (WARN_ON(ret < 0))
		goto exit;

	ret = enable_irq_wake(self->func->irq);
	if (WARN_ON(ret))
		goto free_irq;

	return 0;

free_irq:
	free_irq(self->func->irq, self);
exit:
	return ret;
}

static int cw1200_spi_irq_unsubscribe(struct sbus_priv *self)
{
	int ret = 0;

	pr_debug("SW IRQ unsubscribe\n");
	disable_irq_wake(self->func->irq);
	free_irq(self->func->irq, self);

	return ret;
}

static int cw1200_spi_off(const struct cw1200_platform_data_spi *pdata)
{
	const struct resource *reset = pdata->reset;
	const struct resource *powerup = pdata->powerup;

	if (reset) {
		gpio_set_value(reset->start, 0);
		msleep(30); /* Min is 2 * CLK32K cycles */
		gpio_free(reset->start);
	}

	if (powerup) {
		gpio_direction_output(powerup->start, 0);
		gpio_free(powerup->start);

	}


	if (pdata->power_ctrl)
		pdata->power_ctrl(pdata, false);
	if (pdata->clk_ctrl)
		pdata->clk_ctrl(pdata, false);

	return 0;
}

static int cw1200_spi_on(const struct cw1200_platform_data_spi *pdata)
{
	const struct resource *reset = pdata->reset;
// VLAD: ???
//	const struct resource *powerup = pdata->reset;
	const struct resource *powerup = pdata->powerup;

	/* Ensure I/Os are pulled low */
	if (reset) {
		int res;
		if( (res = gpio_request(reset->start, reset->name) < 0) )
			printk(KERN_CRIT"VLAD: %s request gpio %s failed: %d \n",__func__,reset->name,res);
		gpio_direction_output(reset->start, 0);
	}
	if (powerup) {
		int res;
		if( ( res = gpio_request(powerup->start, powerup->name)) < 0)
			printk(KERN_CRIT"VLAD: %s request gpio %s failed: %d\n",__func__,powerup->name,res);
		gpio_direction_output(powerup->start, 0);
	}
	if (reset || powerup)
		msleep(10); /* Settle time? */

	/* Enable 3v3 and 1v8 to hardware */
	if (pdata->power_ctrl) {
		if (pdata->power_ctrl(pdata, true)) {
			pr_err("power_ctrl() failed!\n");
			return -1;
		}
	}

	/* Enable CLK32K */
	if (pdata->clk_ctrl) {
		if (pdata->clk_ctrl(pdata, true)) {
			pr_err("clk_ctrl() failed!\n");
			return -1;
		}
		msleep(10); /* Delay until clock is stable for 2 cycles */
	}

	/* Enable POWERUP signal */
	if (powerup) {
		gpio_set_value(powerup->start, 1);
		msleep(250); /* or more..? */
	}
	/* Enable RSTn signal */
	if (reset) {
		gpio_set_value(reset->start, 1);
		msleep(50); /* Or more..? */
	}
	return 0;
}

static size_t cw1200_spi_align_size(struct sbus_priv *self, size_t size)
{
	return size & 1 ? size + 1 : size;
}

static int cw1200_spi_pm(struct sbus_priv *self, bool suspend)
{
	return irq_set_irq_wake(self->func->irq, suspend);
}

static void cw1200_spi_irq_enable(struct sbus_priv *self,int enable)
{


  if(enable) {
	  enable_irq(self->func->irq);
  } else {
	  disable_irq(self->func->irq);
  }

}

static struct sbus_ops cw1200_spi_sbus_ops = {
	.sbus_memcpy_fromio	= cw1200_spi_memcpy_fromio,
	.sbus_memcpy_toio	= cw1200_spi_memcpy_toio,
	.lock			= cw1200_spi_lock,
	.unlock			= cw1200_spi_unlock,
	.align_size		= cw1200_spi_align_size,
	.power_mgmt		= cw1200_spi_pm,
	.irq_enable     = cw1200_spi_irq_enable,
};


static int cw1200_spi_suspend(struct spi_device *spi_dev, pm_message_t state)
{
	struct sbus_priv *self = spi_get_drvdata(spi_dev);
	const struct cw1200_platform_data_spi *pdata = cw1200_get_platform_data();
	const struct resource *reset = pdata->reset;
	const struct resource *powerup = pdata->powerup;

	if(self->manually_suspended) {
     dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);
     return 0;
	}


	dev_info(&cw1200_spi_dev->dev,"%s() \n",__func__);

	 if(self) {
       cw1200_spi_irq_unsubscribe(self);
	   if(self->core) {

	    cw1200_core_release(self->core);
	    self->core = NULL;
	   }
	 }

	 if (reset) {
	  gpio_set_value(reset->start, 0);
	 }


	 if (powerup) {
	  gpio_set_value(powerup->start, 0);
	 }

	return 0;
}

static int cw1200_spi_resume(struct spi_device *spi_dev)
{
	struct sbus_priv *self = spi_get_drvdata(spi_dev);
	const struct cw1200_platform_data_spi *pdata = cw1200_get_platform_data();
	const struct resource *reset = pdata->reset;
	const struct resource *powerup = pdata->powerup;

    if(self->manually_suspended) {
        dev_dbg(&cw1200_spi_dev->dev,"%s() skipped resume \n",__func__);
        return 0;

    }

	if (reset || powerup)
		msleep(10); /* Settle time? */


	/* Enable POWERUP signal */
	if (powerup) {
		gpio_set_value(powerup->start, 1);
		msleep(250); /* or more..? */
	}
	/* Enable RSTn signal */
	if (reset) {
		gpio_set_value(reset->start, 1);
		msleep(50); /* Or more..? */
	}
	 dev_info(&cw1200_spi_dev->dev,"%s() \n",__func__);

	cw1200_spi_irq_subscribe(self);
	cw1200_core_probe(&cw1200_spi_sbus_ops,
					   self, &self->func->dev, &self->core,
					   self->pdata->ref_clk,
					   self->pdata->macaddr,
					   self->pdata->sdd_file,
					   self->pdata->have_5ghz);
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

    status = wait_event_interruptible_timeout(priv->cw1200_fw_wq,priv->cw1200_fw_error_status > CW1200_FW_ERR_DOALARM,HZ*60*5);
    if(status < 0 ) {
     dev_err(&cw1200_spi_dev->dev,"%s failed to wait for fw reset command %d",__func__,status);
     goto terminate;
    } else if(0 == status) { /* timeout*/
     dev_err(&cw1200_spi_dev->dev,"cw1200 reset fw command timeout\n");
     goto terminate;
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
 pm_message_t evt;
 struct sbus_priv *self = NULL;
 self = spi_get_drvdata(cw1200_spi_dev);

 evt.event = 0;

 if(!strcmp(buf,"RESET\n")) {
  dev_info(&cw1200_spi_dev->dev,"%s() RESET received \n",__func__);
  if(self) {
   dev_info(&cw1200_spi_dev->dev,"%s() waking up the terminator \n",__func__);
   self->core->cw1200_fw_error_status = CW1200_FW_ERR_DORESET;
   wake_up_interruptible(&self->core->cw1200_fw_wq);
  } else {
   dev_err(&cw1200_spi_dev->dev,"%s self == NULL\n",__func__);
  }
 } else if(!strcmp(buf,"SUSPEND\n")) {
	 dev_dbg(&cw1200_spi_dev->dev,"%s() SUSPEND received \n",__func__);
	 if( 0 == self->manually_suspended ) {
	  cw1200_fwio_prepare(NULL);
	  cw1200_spi_suspend(cw1200_spi_dev,evt);
      self->manually_suspended = 1;
	 }
 } else if(!strcmp(buf,"RESUME\n")) {
	  dev_dbg(&cw1200_spi_dev->dev,"%s() RESUME received \n",__func__);
	  if(self->manually_suspended) {
	   self->manually_suspended = 0;
	   cw1200_spi_resume(cw1200_spi_dev);
	   cw1200_fwio_complete(NULL);
	  }
 } else {
	 return -EACCES;
 }
 return size;
}

DEVICE_ATTR(cw1200_fw_ok, S_IRUGO, dev_type_show, NULL);
DEVICE_ATTR(cw1200_fw_reset, S_IWUGO, NULL, cw1200_do_reset);


/* Probe Function to be called by SPI stack when device is discovered */
static int cw1200_spi_probe(struct spi_device *func)
{
	const struct cw1200_platform_data_spi *plat_data = cw1200_get_platform_data();
//		func->dev.platform_data;
	struct sbus_priv *self;
	int status;

	cw1200_fwio_workqueue = create_workqueue("cw1200_fwio_q");
	if( NULL == cw1200_fwio_workqueue) {
		dev_err(&func->dev,"cw1200_fwio_workqueue == NULL\n");
		return -EFAULT;
	}
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

	pr_info("cw1200_wlan_spi: Probe called (CS %d M %d BPW %d CLK %d)\n",
		func->chip_select, func->mode, func->bits_per_word,
		func->max_speed_hz);

	if (cw1200_spi_on(plat_data)) {
		pr_err("spi_on() failed!\n");
		return -1;
	}

	if (spi_setup(func)) {
		pr_err("spi_setup() failed!\n");
		return -1;
	}

	self = kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		pr_err("Can't allocate SPI sbus_priv.");
		return -ENOMEM;
	}
    self->manually_suspended = 0;
	self->pdata = plat_data;
	self->func = func;
	spin_lock_init(&self->lock);
	spin_lock_init(&self->fw_reset_lock);
	spi_set_drvdata(func, self);

	init_waitqueue_head(&self->wq);

	status = cw1200_spi_irq_subscribe(self);

	status = cw1200_core_probe(&cw1200_spi_sbus_ops,
				   self, &func->dev, &self->core,
				   self->pdata->ref_clk,
				   self->pdata->macaddr,
				   self->pdata->sdd_file,
				   self->pdata->have_5ghz);

    if( 0 == status) {
     self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
	 init_waitqueue_head(&self->core->cw1200_fw_wq);
	 INIT_WORK(&self->core->cw1200_fw_failure_work, cw1200_fw_failure_job);
	 queue_work(cw1200_fwio_workqueue,&self->core->cw1200_fw_failure_work);

    }


	if (status) {
		cw1200_spi_irq_unsubscribe(self);
		cw1200_spi_off(plat_data);
		kfree(self);
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
static int cw1200_spi_disconnect(struct spi_device *func)
{
	struct sbus_priv *self = spi_get_drvdata(func);

	if (self) {
		cw1200_spi_irq_unsubscribe(self);
		if (self->core) {
			cw1200_core_release(self->core);
			self->core = NULL;
		}
		kfree(self);
	}

	cw1200_spi_off(func->dev.platform_data);
	destroy_workqueue(cw1200_fwio_workqueue);
	return 0;
}

static struct spi_driver spi_driver = {
	.probe		= cw1200_spi_probe,
	.remove		= cw1200_spi_disconnect,
	.suspend        = cw1200_spi_suspend,
	.resume         = cw1200_spi_resume,
	.driver = {
		.name		= "cw1200_wlan_spi",
		.bus            = &spi_bus_type,
		.owner          = THIS_MODULE,
	},
};

/* Init Module function -> Called by insmod */
static int __init cw1200_spi_init(void)
{
	int ret;
	ret = spi_register_driver(&spi_driver);

	if (!ret) {
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
	return ret;
}

/* Called at Driver Unloading */
static void __exit cw1200_spi_exit(void)
{
	spi_unregister_driver(&spi_driver);
}


#ifdef CONFIG_DEBUG_FS
static int debugfs_cmd;
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
