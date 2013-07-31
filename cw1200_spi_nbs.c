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
MODULE_ALIAS("spi:cw1200_wlan_spi_nbs");

/* #define SPI_DEBUG */
static struct spi_device *cw1200_spi_dev;
struct sbus_priv {
	struct spi_device	*func;
	struct cw1200_common	*core;
	const struct cw1200_platform_data_spi *pdata;
	spinlock_t		lock; /* Serialize all bus operations */
	int claimed;
	u8 spi_io_buf[2048];
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
	struct spi_transfer     t_addr;

	memset(&t_addr,0,sizeof(t_addr));

	regaddr = (SDIO_TO_SPI_ADDR(addr))<<12;
	regaddr |= SET_READ;
	regaddr |= (count>>1);
	regaddr = cpu_to_le16(regaddr);

	if(unlikely(count < 0)) {  /* single SPI transaction crashes firmware, disabled*/
     u8 spi_io_buf[32];
     spi_io_buf[0] = ((u8*)&regaddr)[1];
     spi_io_buf[1] = ((u8*)&regaddr)[0];
     t_addr.len = count + 2;
     t_addr.tx_buf = spi_io_buf;
     t_addr.rx_buf = spi_io_buf;
     t_addr.bits_per_word = 0;
	 spi_message_init(&m);
	 spi_message_add_tail(&t_addr, &m);
	 ret = spi_async_locked(self->func, &m);
     if( 0 == ret) {
       if( 4 == count) {
         ((u8*)dst)[0] = spi_io_buf[3];
         ((u8*)dst)[1] = spi_io_buf[2];
         ((u8*)dst)[2] = spi_io_buf[5];
         ((u8*)dst)[3] = spi_io_buf[4];
       } else if( 2 == count) {
         ((u8*)dst)[0] = spi_io_buf[3];
         ((u8*)dst)[1] = spi_io_buf[2];
       }  else if( 0 == (count&1) ) { // data must be presented in 16-bit words
           for( i = 0; i < (count>>1); i += 2) {
        	   ((u8*)dst)[ 2 + i ] = spi_io_buf[i+1];
        	   ((u8*)dst)[ 3 + i ] = spi_io_buf[i];
           }
         } else {
        	 printk(KERN_CRIT"VLAD: %s(%d)!!!\n",__func__,count);
        	 return -ENOTSUPP;
         }
     }

	} else {

     int n_trans;
     int msg_ofs;
     struct spi_transfer     t_msg[4];

     if(count > sizeof(self->spi_io_buf))
    	 return -ENOTSUPP;

     memset(t_msg,0,sizeof(t_msg));

   	 regaddr = swab16(regaddr);
     t_addr.tx_buf = &regaddr;
     t_addr.len = sizeof(regaddr);

	 spi_message_init(&m);
	 spi_message_add_tail(&t_addr, &m);

	 n_trans = (count >> 9) + (count&511?1:0);

	 if(n_trans > sizeof(t_msg)/sizeof(t_msg[0]))
		 return -ENOTSUPP;

	 for( i = 0 , msg_ofs = 0; i < n_trans;i++,msg_ofs+=512) {
       t_msg[i].rx_buf = &self->spi_io_buf[msg_ofs];
       t_msg[i].len = ( (count - (i<<9) ) >> 9 ) ? 512 : (count&511) ;
       spi_message_add_tail(&t_msg[i], &m);
	 }

	 ret = spi_async_locked(self->func, &m);

     for( i = 0; i < (count>>1);i++)
      ((u16*)dst)[i] = swab16(((u16*)self->spi_io_buf)[i]);

	} /* if(count > 4)*/
	return ret;
}

static int cw1200_spi_memcpy_toio(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count)
{
	int rval, i;
	uint16_t regaddr;
	struct spi_transfer     t_addr;
	struct spi_message      m;

	memset(&t_addr,0,sizeof(t_addr));

	regaddr = (SDIO_TO_SPI_ADDR(addr))<<12;
	regaddr &= SET_WRITE;
	regaddr |= (count>>1);
	regaddr = cpu_to_le16(regaddr);

	if(unlikely(count <  0)) { /* single SPI transaction crashes firmware, disabled*/
	     u8 spi_io_buf[32];
	     spi_io_buf[0] = ((u8*)&regaddr)[1];
	     spi_io_buf[1] = ((u8*)&regaddr)[0];
         if( 4 == count) {
             spi_io_buf[2] = ((u8*)src)[1];
        	 spi_io_buf[3] = ((u8*)src)[0];
        	 spi_io_buf[4] = ((u8*)src)[3];
        	 spi_io_buf[5] = ((u8*)src)[2];
         } else if( 2 == count) {
             spi_io_buf[2] = ((u8*)src)[1];
        	 spi_io_buf[3] = ((u8*)src)[0];
         } else if( 0 == (count&1) ) { // data must be presented in 16-bit words
           for( i = 0; i < (count>>1); i += 2) {
             spi_io_buf[ 2 + i ] = ((u8*)src)[i+1];
          	 spi_io_buf[ 3 + i ] = ((u8*)src)[i];
           }
         } else {
        	 printk(KERN_CRIT"VLAD: %s(%d)!!!\n",__func__,count);
        	 return -ENOTSUPP;
         }

	     t_addr.len = count + 2;
	     t_addr.tx_buf = spi_io_buf;
	     t_addr.rx_buf = spi_io_buf;
	     t_addr.bits_per_word = 0;

		 spi_message_init(&m);
		 spi_message_add_tail(&t_addr, &m);
		 rval = spi_async_locked(self->func, &m);
	} else {

      int n_trans;
      int msg_ofs;
      struct spi_transfer     t_msg[4];

      if(count > sizeof(self->spi_io_buf))
    	 return -ENOTSUPP;

      memset(t_msg,0,sizeof(t_msg));

      regaddr = swab16(regaddr);
      t_addr.tx_buf = &regaddr;
      t_addr.len = sizeof(regaddr);

 	  spi_message_init(&m);
 	  spi_message_add_tail(&t_addr, &m);

 	  n_trans = (count >> 9) + (count&511?1:0);

 	  if(n_trans > sizeof(t_msg)/sizeof(t_msg[0]))
 		 return -ENOTSUPP;

 	  for( i = 0; i < (count>>1);i++) {
        ((u16*)self->spi_io_buf)[i] = swab16( ((u16*)src)[i]);
 	  }

   	  for( i = 0 , msg_ofs = 0; i < n_trans;i++,msg_ofs+=512) {
        t_msg[i].tx_buf = &self->spi_io_buf[msg_ofs];
        t_msg[i].len = ( (count - (i<<9) ) >> 9 ) ? 512 : (count&511) ;
        spi_message_add_tail(&t_msg[i], &m);
 	  }

 	  rval = spi_async_locked(self->func, &m);


	}
	return rval;
}

static void cw1200_spi_lock(struct sbus_priv *self)
{
	unsigned long flags;

#if 0

#else
	might_sleep();

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
#endif
	return;
}

static void cw1200_spi_unlock(struct sbus_priv *self)
{
	unsigned long flags;
#if 0

#else
	spin_lock_irqsave(&self->lock, flags);
	self->claimed = 0;
	spin_unlock_irqrestore(&self->lock, flags);
#endif
	return;
}

static irqreturn_t cw1200_spi_irq_handler(int irq, void *dev_id)
{
	struct sbus_priv *self = dev_id;

	if (self->core) {
		cw1200_irq_handler(self->core);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int cw1200_spi_irq_subscribe(struct sbus_priv *self)
{
	int ret;

	pr_debug("SW IRQ subscribe\n");

	ret = request_any_context_irq(self->func->irq, cw1200_spi_irq_handler,
// VLAD:
//				      IRQF_TRIGGER_HIGH,
			          IRQF_TRIGGER_RISING,
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

/* Probe Function to be called by SPI stack when device is discovered */
static int cw1200_spi_probe(struct spi_device *func)
{
	const struct cw1200_platform_data_spi *plat_data = cw1200_get_platform_data();
//		func->dev.platform_data;
	struct sbus_priv *self;
	int status;

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

	self->pdata = plat_data;
	self->func = func;
	spin_lock_init(&self->lock);

	spi_set_drvdata(func, self);

	status = cw1200_spi_irq_subscribe(self);

	status = cw1200_core_probe(&cw1200_spi_sbus_ops,
				   self, &func->dev, &self->core,
				   self->pdata->ref_clk,
				   self->pdata->macaddr,
				   self->pdata->sdd_file,
				   self->pdata->have_5ghz);

	if (status) {
		cw1200_spi_irq_unsubscribe(self);
		cw1200_spi_off(plat_data);
		kfree(self);
	}
	cw1200_spi_dev = func;

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

	return 0;
}

static int cw1200_spi_suspend(struct device *dev, pm_message_t state)
{
	struct sbus_priv *self = spi_get_drvdata(to_spi_device(dev));
	const struct cw1200_platform_data_spi *pdata = cw1200_get_platform_data();
	const struct resource *reset = pdata->reset;
	const struct resource *powerup = pdata->powerup;



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

static int cw1200_spi_resume(struct device *dev)
{
	struct sbus_priv *self = spi_get_drvdata(to_spi_device(dev));
	const struct cw1200_platform_data_spi *pdata = cw1200_get_platform_data();
	const struct resource *reset = pdata->reset;
	const struct resource *powerup = pdata->powerup;


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

	cw1200_spi_irq_subscribe(self);
	cw1200_core_probe(&cw1200_spi_sbus_ops,
					   self, &self->func->dev, &self->core,
					   self->pdata->ref_clk,
					   self->pdata->macaddr,
					   self->pdata->sdd_file,
					   self->pdata->have_5ghz);
	return 0;
}

static struct spi_driver spi_driver = {
	.probe		= cw1200_spi_probe,
	.remove		= cw1200_spi_disconnect,
	.driver = {
		.name		= "cw1200_wlan_spi",
		.bus            = &spi_bus_type,
		.owner          = THIS_MODULE,
		.suspend        = cw1200_spi_suspend,
		.resume         = cw1200_spi_resume,
	},
};

/* Init Module function -> Called by insmod */
static int __init cw1200_spi_init(void)
{
	return spi_register_driver(&spi_driver);
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


  cw1200_spi_suspend(&cw1200_spi_dev->dev,evt);
  msleep_interruptible(100);
  cw1200_spi_resume(&cw1200_spi_dev->dev);

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
