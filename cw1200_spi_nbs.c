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
#include <linux/suspend.h>
#include <net/mac80211.h>

#include <linux/spi/spi.h>
#include <linux/device.h>

#include "cw1200.h"
#include "hwbus.h"
#include <linux/cw1200_platform.h>

#include "hwio.h"

MODULE_AUTHOR("Solomon Peachy <speachy@sagrad.com>");
MODULE_DESCRIPTION("mac80211 ST-Ericsson CW1200 SPI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:cw1200_wlan_spi");

#define SPI_NBS_MAX_BUF_SIZE 4096

static struct workqueue_struct *cw1200_fwio_workqueue = NULL;
static int cw1200_fw_reset_cnt = 0;
/* #define SPI_DEBUG */
static struct spi_device *cw1200_spi_dev;
struct hwbus_priv {
	struct spi_device	*func;
	struct cw1200_common	*core;
	const struct cw1200_platform_data_spi *pdata;
	spinlock_t		lock; /* Serialize all bus operations */
	wait_queue_head_t       wq;
	int claimed;
	// VLAD:
		int manually_suspended;
		atomic_t suspended;
		struct work_struct cw1200_fw_failure_work;
		struct notifier_block	pm_notify;
		struct mutex			spi_pm_mutex;
		unsigned long           irqsave; /* used by cw1200_spi_lock/unlock() */


};

static struct platform_device *cw1200_fwio_dev;
static void cw1200_fw_failure_job(struct work_struct *work);

int cw1200_pm_notify(struct notifier_block *notify_block,
					unsigned long mode, void *unused)
{
  struct hwbus_priv *self = container_of(notify_block, struct hwbus_priv, pm_notify);
  switch (mode) {
  case PM_SUSPEND_PREPARE:
  { int ret;
    mutex_lock(&self->spi_pm_mutex);
    if(self->manually_suspended || atomic_read(&self->suspended) ) {
  	  dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);
  	  mutex_unlock(&self->spi_pm_mutex);
  	  return 0;
    }
    self->core->cw1200_fw_error_status = CW1200_FW_ERR_DOTERMINATE;
    wake_up_interruptible(&self->core->cw1200_fw_wq);
    ret = wait_event_interruptible(self->core->cw1200_fw_wq,CW1200_FW_ERR_TERMINATED == self->core->cw1200_fw_error_status);
    self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
    mutex_unlock(&self->spi_pm_mutex);

  } break;
  case PM_POST_SUSPEND:
  {
    if(self->manually_suspended) {
  	  dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);
  	  return 0;
    }



    if(NULL == self->core) {
  	dev_err(&cw1200_spi_dev->dev,"%s(): self->core is NULL.\n",__func__);
  	return 0;

    }

    self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;

    init_waitqueue_head(&self->core->cw1200_fw_wq);
    INIT_WORK(&self->cw1200_fw_failure_work,cw1200_fw_failure_job);
    queue_work(cw1200_fwio_workqueue,&self->cw1200_fw_failure_work);
  } break;

  }
  return 0;
}


static int cw1200_fwio_prepare(struct device *dev)
{
	  struct hwbus_priv *self = spi_get_drvdata(cw1200_spi_dev);

	  if(self->manually_suspended || atomic_read(&self->suspended) ) {
		  dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);
		  return 0;
	  }
	  dev_dbg(&cw1200_spi_dev->dev,"%s() \n",__func__);
	  self->core->cw1200_fw_error_status = CW1200_FW_ERR_DOTERMINATE;
	  wake_up_interruptible(&self->core->cw1200_fw_wq);
	  wait_event_interruptible(self->core->cw1200_fw_wq,CW1200_FW_ERR_TERMINATED == self->core->cw1200_fw_error_status);
	  self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
	  return 0;
}


static void cw1200_fwio_complete(struct device *dev)
{
	struct hwbus_priv *self = spi_get_drvdata(cw1200_spi_dev);

  if(self->manually_suspended) {
	  dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);
	  return;
  }

  dev_dbg(&cw1200_spi_dev->dev,"%s() \n",__func__);

  if(NULL == self->core) {
	dev_err(&cw1200_spi_dev->dev,"%s(): self->core is NULL.\n",__func__);
	return;

  }

  self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;

  init_waitqueue_head(&self->core->cw1200_fw_wq);
  queue_work(cw1200_fwio_workqueue,&self->cw1200_fw_failure_work);
}

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
   VLAD:
   The 16 bit byte swap is handled by spi_imx_nbs bus master driver when the device has SPI_LSB_FIRST flag selected.

*/
/* Mark wrapper data as free to re-use. */


#define SPI_MAX_CHUNK_SHIFT 9
#define SPI_MAX_CHUNK  (1<<SPI_MAX_CHUNK_SHIFT)

static __maybe_unused void spi_complete(void *context)
{
   udelay(10);
}

static int cw1200_spi_memcpy_fromio(struct hwbus_priv *self,
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

	if(count < 5) {
     u8 spi_io_buf[32];
     *((u16*)spi_io_buf) = regaddr;
     t_addr.len = count + 2;
     t_addr.tx_buf = spi_io_buf;
     t_addr.rx_buf = spi_io_buf;
     t_addr.bits_per_word = 0;
	 spi_message_init(&m);
	 spi_message_add_tail(&t_addr, &m);
	 ret = spi_async_locked(self->func, &m);
	 udelay(5);
     if( 0 == ret) {
       if( 4 == count) {
    	 *((u32*)dst) = *((u32*)(&spi_io_buf[2]));
       } else if( 2 == count) {
      	 *((u16*)dst) = *((u16*)(&spi_io_buf[2]));
       }  else if( 0 == (count&1) ) { // data must be presented in 16-bit words
           for( i = 0; i < (count>>1); i += 2) {
        	   ((u8*)dst)[ 2 + i ] = spi_io_buf[i];
        	   ((u8*)dst)[ 3 + i ] = spi_io_buf[i+1];
           }
         } else {
        	 printk(KERN_CRIT"VLAD: %s(%d)!!!\n",__func__,count);
        	 return -ENOTSUPP;
         }
     }

	} else {
     int n_trans;
     int msg_ofs;
     struct spi_transfer     t_msg[SPI_NBS_MAX_BUF_SIZE/SPI_MAX_CHUNK];

     if(count > SPI_NBS_MAX_BUF_SIZE)
    	 return -ENOTSUPP;

     memset(t_msg,0,sizeof(t_msg));

     t_addr.tx_buf = &regaddr;
     t_addr.len = sizeof(regaddr);

	 spi_message_init(&m);
	// VLAD:
	 spi_message_add_tail(&t_addr, &m);

	 n_trans = (count >> SPI_MAX_CHUNK_SHIFT) + (count&(SPI_MAX_CHUNK-1)?1:0);

	 if(n_trans > sizeof(t_msg)/sizeof(t_msg[0]))
		 return -ENOTSUPP;

	 for( i = 0 , msg_ofs = 0; i < n_trans;i++,msg_ofs+=SPI_MAX_CHUNK) {
       t_msg[i].rx_buf = &(((u8*)dst)[msg_ofs]);
       t_msg[i].len = ( (count - (i<<SPI_MAX_CHUNK_SHIFT) ) >> SPI_MAX_CHUNK_SHIFT ) ? SPI_MAX_CHUNK : (count&(SPI_MAX_CHUNK-1)) ;
       spi_message_add_tail(&t_msg[i], &m);
	 }
	 ret = spi_async_locked(self->func, &m);
	 udelay(5);
	} /* if(count > 4)*/
	return ret;
}

static int cw1200_spi_memcpy_toio(struct hwbus_priv *self,
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

	if(count <  5) {
	     u8 spi_io_buf[32];
         *((u16*)spi_io_buf) = regaddr;
         if( 4 == count) {
        	 *((u32*)(&spi_io_buf[2])) = *((u32*)src);
         } else if( 2 == count) {
        	 *((u16*)(&spi_io_buf[2])) = *((u16*)src);
         } else if( 0 == (count&1) ) { // data must be presented in 16-bit words
           for( i = 0; i < (count>>1); i += 2) {
             spi_io_buf[ 2 + i ] = ((u8*)src)[i];
          	 spi_io_buf[ 3 + i ] = ((u8*)src)[i+1];
           }
         } else {
        	 printk(KERN_CRIT"VLAD: %s(%d)!!!\n",__func__,count);
        	 return -ENOTSUPP;
         }

	     t_addr.len = count + 2;
	     t_addr.tx_buf = spi_io_buf;
	     t_addr.bits_per_word = 0;

		 spi_message_init(&m);
		 spi_message_add_tail(&t_addr, &m);
		 rval = spi_async_locked(self->func, &m);
		 udelay(5);
	} else {
      int n_trans;
      int msg_ofs;
      struct spi_transfer     t_msg[SPI_NBS_MAX_BUF_SIZE/SPI_MAX_CHUNK];

      if(count > SPI_NBS_MAX_BUF_SIZE)
    	 return -ENOTSUPP;

      memset(t_msg,0,sizeof(t_msg));

      t_addr.tx_buf = &regaddr;
      t_addr.len = sizeof(regaddr);

 	  spi_message_init(&m);
 	  // VLAD:

 	  spi_message_add_tail(&t_addr, &m);

 	  n_trans = (count >> SPI_MAX_CHUNK_SHIFT) + (count&(SPI_MAX_CHUNK-1)?1:0);

 	  if(n_trans > sizeof(t_msg)/sizeof(t_msg[0]))
 		 return -ENOTSUPP;
   	  for( i = 0 , msg_ofs = 0; i < n_trans;i++,msg_ofs+=SPI_MAX_CHUNK) {
        t_msg[i].tx_buf = &(((u8*)src)[msg_ofs]);
        t_msg[i].len = ( (count - (i<<SPI_MAX_CHUNK_SHIFT) ) >> SPI_MAX_CHUNK_SHIFT ) ? SPI_MAX_CHUNK : (count&(SPI_MAX_CHUNK-1)) ;
        spi_message_add_tail(&t_msg[i], &m);
 	  }
 	  rval = spi_async_locked(self->func, &m);
 	  udelay(5);
	}
	return rval;
}

static void cw1200_spi_lock(struct hwbus_priv *self)
{
	spin_lock_irqsave(&self->lock, self->irqsave);
	return;
}

static void cw1200_spi_unlock(struct hwbus_priv *self)
{
	spin_unlock_irqrestore(&self->lock, self->irqsave);
	return;
}

static irqreturn_t cw1200_spi_irq_handler(int irq, void *dev_id)
{
	struct hwbus_priv *self = dev_id;

	if (self->core) {
		cw1200_spi_lock(self);
		cw1200_irq_handler(self->core);
		cw1200_spi_unlock(self);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int cw1200_spi_irq_subscribe(struct hwbus_priv *self)
{
	int ret;

	pr_debug("SW IRQ subscribe\n");

	ret = request_irq(self->func->irq,cw1200_spi_irq_handler,
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

static int cw1200_spi_irq_unsubscribe(struct hwbus_priv *self)
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

static size_t cw1200_spi_align_size(struct hwbus_priv *self, size_t size)
{
	return size & 1 ? size + 1 : size;
}

static int cw1200_spi_pm(struct hwbus_priv *self, bool suspend)
{
	return irq_set_irq_wake(self->func->irq, suspend);
}

static struct hwbus_ops cw1200_spi_hwbus_ops = {
	.hwbus_memcpy_fromio	= cw1200_spi_memcpy_fromio,
	.hwbus_memcpy_toio	= cw1200_spi_memcpy_toio,
	.lock			= cw1200_spi_lock,
	.unlock			= cw1200_spi_unlock,
	.align_size		= cw1200_spi_align_size,
	.power_mgmt		= cw1200_spi_pm,
};


static int cw1200_spi_suspend(struct spi_device *spi_dev, pm_message_t state)
{
	struct hwbus_priv *self = spi_get_drvdata(spi_dev);
	const struct cw1200_platform_data_spi *pdata = cw1200_get_platform_data();
	const struct resource *reset = pdata->reset;
	const struct resource *powerup = pdata->powerup;

	if(self->manually_suspended || atomic_cmpxchg(&self->suspended,0,1)) {
     dev_dbg(&cw1200_spi_dev->dev,"%s() already suspended \n",__func__);
     return 0;
	}


	dev_dbg(&cw1200_spi_dev->dev,"%s() \n",__func__);

	 if(self) {
	   if(self->core) {
        cw1200_spi_irq_unsubscribe(self);
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
	struct hwbus_priv *self = spi_get_drvdata(spi_dev);
	const struct cw1200_platform_data_spi *pdata = cw1200_get_platform_data();
	const struct resource *reset = pdata->reset;
	const struct resource *powerup = pdata->powerup;

    if(self->manually_suspended || !atomic_cmpxchg(&self->suspended,1,0)) {
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
	 dev_dbg(&cw1200_spi_dev->dev,"%s() \n",__func__);

	cw1200_spi_irq_subscribe(self);
	cw1200_core_probe(&cw1200_spi_hwbus_ops,
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
 struct hwbus_priv *self = container_of(work, struct hwbus_priv, cw1200_fw_failure_work);
 struct cw1200_common *priv;

 priv = self->core;
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
        mutex_lock(&self->spi_pm_mutex);
        cw1200_spi_suspend(cw1200_spi_dev,evt);
        msleep_interruptible(200);
        cw1200_spi_resume(cw1200_spi_dev);
        self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
        init_waitqueue_head(&self->core->cw1200_fw_wq);
        mutex_unlock(&self->spi_pm_mutex);
    } else if (CW1200_FW_ERR_DOTERMINATE ==  priv->cw1200_fw_error_status) {
    	   goto terminate;
    } else goto oops;
  }
 } else if (CW1200_FW_ERR_DORESET ==  priv->cw1200_fw_error_status) {
     dev_err(&cw1200_spi_dev->dev,"RESET not allowed\n");
     self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
 } else if (CW1200_FW_ERR_DOTERMINATE ==  priv->cw1200_fw_error_status) {
   goto terminate;
 } else goto oops;

 if(!cw1200_fwio_dev) {
   int ret;
   ret = platform_driver_register(&cw1200_fwio_driver);
   cw1200_fwio_dev = platform_device_alloc("cw1200_fw_io", 0);
   ret = platform_device_add(cw1200_fwio_dev);
	 if (ret) {
		kfree(cw1200_fwio_dev);
	 }
 }
 queue_work(cw1200_fwio_workqueue,&self->cw1200_fw_failure_work);
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
 struct hwbus_priv *self = NULL;
 self = spi_get_drvdata(cw1200_spi_dev);

 evt.event = 0;

 if(!strcmp(buf,"RESET\n")) {
  dev_info(&cw1200_spi_dev->dev,"%s() RESET received \n",__func__);
  if(self && self->core) {

   if(CW1200_FW_ERR_DOALARM == self->core->cw1200_fw_error_status  ) {
	   dev_info(&cw1200_spi_dev->dev,"%s() waking up the terminator \n",__func__);
	   self->core->cw1200_fw_error_status = CW1200_FW_ERR_DORESET;
	   wake_up_interruptible(&self->core->cw1200_fw_wq);
   } else {
	   dev_info(&cw1200_spi_dev->dev,"%s() RESET not permitted at state: %d \n",__func__,self->core->cw1200_fw_error_status );

   }
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
	struct hwbus_priv *self;
	int status;

	cw1200_fwio_workqueue = alloc_workqueue("cw1200_fwio_q",WQ_RESCUER , 1);
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
	func->mode = SPI_MODE_0 | SPI_LSB_FIRST; // SPI_LSB_FIRST option tells nbs_spi driver to swap order in 16 bit words

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
		pr_err("Can't allocate SPI hwbus_priv.");
		return -ENOMEM;
	}
    self->manually_suspended = 0;
	self->pdata = plat_data;
	self->func = func;
	spin_lock_init(&self->lock);
	spi_set_drvdata(func, self);

	init_waitqueue_head(&self->wq);
	mutex_init(&self->spi_pm_mutex);

	status = cw1200_spi_irq_subscribe(self);

	status = cw1200_core_probe(&cw1200_spi_hwbus_ops,
				   self, &func->dev, &self->core,
				   self->pdata->ref_clk,
				   self->pdata->macaddr,
				   self->pdata->sdd_file,
				   self->pdata->have_5ghz);

    if( 0 == status) {
     self->core->cw1200_fw_error_status = CW1200_FW_ERR_IDLE;
	 init_waitqueue_head(&self->core->cw1200_fw_wq);
	 INIT_WORK(&self->cw1200_fw_failure_work, cw1200_fw_failure_job);
	 queue_work(cw1200_fwio_workqueue,&self->cw1200_fw_failure_work);

    }


	if (status) {
		cw1200_spi_irq_unsubscribe(self);
		cw1200_spi_off(plat_data);
		kfree(self);
	}
	cw1200_spi_dev = func;
	self->pm_notify.notifier_call = cw1200_pm_notify;
	register_pm_notifier(&self->pm_notify);

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
	struct hwbus_priv *self = spi_get_drvdata(func);
	unregister_pm_notifier(&self->pm_notify);

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
	mutex_destroy(&self->spi_pm_mutex);
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
  struct hwbus_priv *self = NULL;
  self = spi_get_drvdata(cw1200_spi_dev);
  self->core->cw1200_fw_error_status = CW1200_FW_ERR_DOALARM;
  wake_up_interruptible(&self->core->cw1200_fw_wq);

 } break;
 case 5: {
  struct hwbus_priv *self = NULL;
  self = spi_get_drvdata(cw1200_spi_dev);

  self->core->bh_error = 1;
  barrier();
  wake_up(&self->core->bh_wq);

 }


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
