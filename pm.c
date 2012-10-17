/*
 * Mac80211 power management API for ST-Ericsson CW1200 drivers
 *
 * Copyright (c) 2011, ST-Ericsson
 * Author: Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/if_ether.h>
#include "cw1200.h"
#include "pm.h"
#include "sta.h"
#include "bh.h"
#include "sbus.h"
#include "cw1200_plat.h"
#include "hwio.h"
#include "fwio.h"
#include <linux/gpio.h>


#define CW1200_BEACON_SKIPPING_MULTIPLIER 3

struct cw1200_udp_port_filter {
	struct wsm_udp_port_filter_hdr hdr;
	struct wsm_udp_port_filter dhcp;
	struct wsm_udp_port_filter upnp;
} __packed;

struct cw1200_ether_type_filter {
	struct wsm_ether_type_filter_hdr hdr;
	struct wsm_ether_type_filter ip;
	struct wsm_ether_type_filter pae;
	struct wsm_ether_type_filter wapi;
} __packed;

static struct cw1200_udp_port_filter cw1200_udp_port_filter_on = {
	.hdr.nrFilters = 2,
	.dhcp = {
		.filterAction = WSM_FILTER_ACTION_FILTER_OUT,
		.portType = WSM_FILTER_PORT_TYPE_DST,
		.udpPort = __cpu_to_le16(67),
	},
	.upnp = {
		.filterAction = WSM_FILTER_ACTION_FILTER_OUT,
		.portType = WSM_FILTER_PORT_TYPE_DST,
		.udpPort = __cpu_to_le16(1900),
	},
	/* Please add other known ports to be filtered out here and
	 * update nrFilters field in the header.
	 * Up to 4 filters are allowed. */
};

static struct wsm_udp_port_filter_hdr cw1200_udp_port_filter_off = {
	.nrFilters = 0,
};

#ifndef ETH_P_WAPI
#define ETH_P_WAPI     0x88B4
#endif

static struct cw1200_ether_type_filter cw1200_ether_type_filter_on = {
	.hdr.nrFilters = 3,
	.ip = {
		.filterAction = WSM_FILTER_ACTION_FILTER_IN,
		.etherType = __cpu_to_le16(ETH_P_IP),
	},
	.pae = {
		.filterAction = WSM_FILTER_ACTION_FILTER_IN,
		.etherType = __cpu_to_le16(ETH_P_PAE),
	},
	.wapi = {
		.filterAction = WSM_FILTER_ACTION_FILTER_IN,
		.etherType = __cpu_to_le16(ETH_P_WAPI),
	},
	/* Please add other known ether types to be filtered out here and
	 * update nrFilters field in the header.
	 * Up to 4 filters are allowed. */
};

static struct wsm_ether_type_filter_hdr cw1200_ether_type_filter_off = {
	.nrFilters = 0,
};

static int cw1200_suspend_late(struct device *dev);
static int cw1200_pm_suspend(struct device *dev);
static int cw1200_pm_resume(struct device *dev);
static int cw1200_pm_resume_early(struct device *dev);

static void cw1200_pm_release(struct device *dev);
static int cw1200_pm_probe(struct platform_device *pdev);


/* private */
struct cw1200_suspend_state {
	unsigned long bss_loss_tmo;
	unsigned long connection_loss_tmo;
	unsigned long join_tmo;
	unsigned long direct_probe;
	unsigned long link_id_gc;
	bool beacon_skipping;
};

static const struct dev_pm_ops cw1200_pm_ops = {
    .suspend = cw1200_pm_suspend,
    .resume  = cw1200_pm_resume,
	.suspend_noirq = cw1200_suspend_late,
	.resume_noirq = cw1200_pm_resume_early,
};

static struct platform_driver cw1200_power_driver = {
	.probe = cw1200_pm_probe,
	.driver = {
		.name = "cw1200_power",
		.pm = &cw1200_pm_ops,
	},
};

static int cw1200_pm_init_common(struct cw1200_pm_state *pm,
				  struct cw1200_common *priv)
{
	int ret;

	spin_lock_init(&pm->lock);
	ret = platform_driver_register(&cw1200_power_driver);
	if (ret)
		return ret;
	pm->pm_dev = platform_device_alloc("cw1200_power", 0);
	if (!pm->pm_dev) {
		platform_driver_unregister(&cw1200_power_driver);
		return -ENOMEM;
	}

	pm->pm_dev->dev.platform_data = priv;
	ret = platform_device_add(pm->pm_dev);
	if (ret) {
		kfree(pm->pm_dev);
		pm->pm_dev = NULL;
	}

	return ret;
}

static void cw1200_pm_deinit_common(struct cw1200_pm_state *pm)
{
	platform_driver_unregister(&cw1200_power_driver);
	if (pm->pm_dev) {
		pm->pm_dev->dev.platform_data = NULL;
		platform_device_unregister(pm->pm_dev);
		pm->pm_dev = NULL;
	}
}

#ifdef CONFIG_WAKELOCK

int cw1200_pm_init(struct cw1200_pm_state *pm,
		   struct cw1200_common *priv)
{
	int ret = cw1200_pm_init_common(pm, priv);
	if (!ret)
		wake_lock_init(&pm->wakelock,
			WAKE_LOCK_SUSPEND, "cw1200_wlan");
	return ret;
}

void cw1200_pm_deinit(struct cw1200_pm_state *pm)
{
	if (wake_lock_active(&pm->wakelock))
		wake_unlock(&pm->wakelock);
	wake_lock_destroy(&pm->wakelock);
	cw1200_pm_deinit_common(pm);
}

void cw1200_pm_stay_awake(struct cw1200_pm_state *pm,
			  unsigned long tmo)
{
	long cur_tmo;
	spin_lock_bh(&pm->lock);
	cur_tmo = pm->wakelock.ws.timer_expires - jiffies;
	if (!wake_lock_active(&pm->wakelock) ||
			cur_tmo < (long)tmo)
		wake_lock_timeout(&pm->wakelock, tmo);
	spin_unlock_bh(&pm->lock);
}

#else /* CONFIG_WAKELOCK */

static void cw1200_pm_stay_awake_tmo(unsigned long arg)
{
}

int cw1200_pm_init(struct cw1200_pm_state *pm,
		   struct cw1200_common *priv)
{
// VLAD:
	return 0;
//
	int ret = cw1200_pm_init_common(pm, priv);
	if (!ret) {
		init_timer(&pm->stay_awake);
		pm->stay_awake.data = (unsigned long)pm;
		pm->stay_awake.function = cw1200_pm_stay_awake_tmo;
	}
	return ret;
}

void cw1200_pm_deinit(struct cw1200_pm_state *pm)
{
// VLAD:
	return;
//
	del_timer_sync(&pm->stay_awake);
	cw1200_pm_deinit_common(pm);
}

void cw1200_pm_stay_awake(struct cw1200_pm_state *pm,
			  unsigned long tmo)
{

// VLAD:
  return;
//
	long cur_tmo;
	spin_lock_bh(&pm->lock);
	cur_tmo = pm->stay_awake.expires - jiffies;
	if (!timer_pending(&pm->stay_awake) ||
			cur_tmo < (long)tmo)
		mod_timer(&pm->stay_awake, jiffies + tmo);
	spin_unlock_bh(&pm->lock);
}

#endif /* CONFIG_WAKELOCK */

static long cw1200_suspend_work(struct delayed_work *work)
{
	int ret = cancel_delayed_work(work);
	long tmo;
	if (ret > 0) {
		/* Timer is pending */
		tmo = work->timer.expires - jiffies;
		if (tmo < 0)
			tmo = 0;
	} else {
		tmo = -1;
	}
	return tmo;
}

static int cw1200_resume_work(struct cw1200_common *priv,
			       struct delayed_work *work,
			       unsigned long tmo)
{
	if ((long)tmo < 0)
		return 1;

	return queue_delayed_work(priv->workqueue, work, tmo);
}

static int cw1200_suspend_late(struct device *dev)
{
	struct cw1200_common *priv = dev->platform_data;
	const struct cw1200_platform_data *pdata;
	printk(KERN_CRIT"VLAD:%s",__func__);
	if (atomic_read(&priv->bh_rx)) {
		// VLAD:
		wiphy_err/*dbg*/(priv->hw->wiphy,
			"%s: Suspend interrupted.\n",
			__func__);
		return -EAGAIN;
	}
	pdata = cw1200_get_platform_data();
	{
	 const struct resource *reset = pdata->reset;
	 if (reset) {
	  gpio_set_value(reset->start, 0);
	  printk(KERN_CRIT"VLAD:%s reset",__func__);
	 }
    }
	if(pdata->power_ctrl) {
	  pdata->power_ctrl(pdata,false);
	  printk(KERN_CRIT"VLAD:%s power_ctrl",__func__);
	}

	priv->wsm_caps.firmwareReady = 0;
	priv->wsm_rx_seq = 0;
	priv->wsm_tx_seq = 0;
	atomic_set(&priv->bh_rx, 0);
	atomic_set(&priv->bh_tx, 0);
	atomic_set(&priv->bh_term, 0);
	priv->buf_id_tx = 0;
	priv->buf_id_rx = 0;

	printk(KERN_CRIT"VLAD:%s OK",__func__);
	return 0;
}

static int cw1200_pm_resume_early(struct device *dev)
{
	const struct cw1200_platform_data *pdata;
	const struct resource *reset;
	int ret;

	pdata = cw1200_get_platform_data();

	if (pdata->power_ctrl) {
		ret = pdata->power_ctrl(pdata, true);
		if (ret)
         return 0;
		msleep(250);  // XXX can be lowered probably
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


	printk(KERN_CRIT"VLAD:%s",__func__);

	return 0;
}


static int cw1200_pm_resume(struct device *dev)
{
	struct cw1200_common *priv;
	struct ieee80211_hw *hw;
	int err = 0;

	printk(KERN_CRIT"VLAD:%s",__func__);

    priv = dev->platform_data;
    hw = priv->hw;

	/* Resume BH thread */
	WARN_ON(cw1200_bh_resume(priv));

	err = cw1200_load_firmware(priv); /* does priv->sbus_ops->irq_subscribe there as well */
    if(err) {
    	printk(KERN_CRIT"VLAD:cw1200_load_firmware:%d\n",err);
    }

	return cw1200_wow_resume(hw);
}


static int cw1200_pm_suspend(struct device *dev)
{
	struct cw1200_common *priv;
	struct ieee80211_hw *hw;
	int ret;

    priv = dev->platform_data;
    hw = priv->hw;

    ret = priv->sbus_ops->irq_unsubscribe(priv->sbus_priv);

	printk(KERN_CRIT"VLAD:%s (%s) \n",__func__,dev->driver->name);

	return cw1200_wow_suspend(hw,NULL);
}


static void cw1200_pm_release(struct device *dev)
{
}

static int cw1200_pm_probe(struct platform_device *pdev)
{
	pdev->dev.release = cw1200_pm_release;
	return 0;
}
#if 1
int cw1200_wow_suspend(struct ieee80211_hw *hw, struct cfg80211_wowlan *wowlan)
{
	struct cw1200_common *priv = hw->priv;
	struct cw1200_pm_state *pm_state = &priv->pm_state;
	struct cw1200_suspend_state *state;
	int ret;

	printk(KERN_CRIT"VLAD:%s",__func__);

#ifndef CONFIG_WAKELOCK
	spin_lock_bh(&pm_state->lock);
	ret = timer_pending(&pm_state->stay_awake);
	spin_unlock_bh(&pm_state->lock);
	if (ret) {
		printk(KERN_CRIT"VLAD:%s 1",__func__);
		return -EAGAIN;
	}
#endif

	/* Do not suspend when datapath is not idle */
	if (priv->tx_queue_stats.num_queued) {
		printk(KERN_CRIT"VLAD:%s 2",__func__);
		return -EBUSY;
	}

	/* Make sure there is no configuration requests in progress. */
	if (!mutex_trylock(&priv->conf_mutex)) {
		printk(KERN_CRIT"VLAD:%s 3",__func__);
		return -EBUSY;
	}

	/* Ensure pending operations are done.
	 * Note also that wow_suspend must return in ~2.5sec, before
	 * watchdog is triggered. */
	if (priv->channel_switch_in_progress) {
		printk(KERN_CRIT"VLAD:%s 4",__func__);
		goto revert1;
	}
	/* Do not suspend when join work is scheduled */
	if (work_pending(&priv->join_work)) {
		printk(KERN_CRIT"VLAD:%s 5",__func__);
		goto revert1;
	}

	/* Do not suspend when scanning */
	if (down_trylock(&priv->scan.lock)) {
		printk(KERN_CRIT"VLAD:%s 6",__func__);
		goto revert1;
	}

	/* Lock TX. */
	wsm_lock_tx_async(priv);

	/* Wait to avoid possible race with bh code.
	 * But do not wait too long... */
	if (wait_event_timeout(priv->bh_evt_wq,
			!priv->hw_bufs_used, HZ / 10) <= 0) {
		printk(KERN_CRIT"VLAD:%s 7",__func__);
		goto revert2;
	}

	/* Set UDP filter */
	wsm_set_udp_port_filter(priv, &cw1200_udp_port_filter_on.hdr);

	/* Set ethernet frame type filter */
	wsm_set_ether_type_filter(priv, &cw1200_ether_type_filter_on.hdr);

	/* Allocate state */
	state = kzalloc(sizeof(struct cw1200_suspend_state), GFP_KERNEL);
	if (!state) {
		printk(KERN_CRIT"VLAD:%s 8",__func__);
		goto revert3;
	}

	/* Store delayed work states. */
	state->bss_loss_tmo =
		cw1200_suspend_work(&priv->bss_loss_work);
	state->connection_loss_tmo =
		cw1200_suspend_work(&priv->connection_loss_work);
	state->join_tmo =
		cw1200_suspend_work(&priv->join_timeout);
	state->direct_probe =
		cw1200_suspend_work(&priv->scan.probe_work);
	state->link_id_gc =
		cw1200_suspend_work(&priv->link_id_gc_work);

	/* Enable beacon skipping */
	if (priv->join_status == CW1200_JOIN_STATUS_STA
			&& priv->join_dtim_period
			&& !priv->has_multicast_subscription) {
		state->beacon_skipping = true;
		wsm_set_beacon_wakeup_period(priv,
				priv->join_dtim_period,
				CW1200_BEACON_SKIPPING_MULTIPLIER *
				 priv->join_dtim_period);
	}

	/* Stop serving thread */
	if (cw1200_bh_suspend(priv)) {
		printk(KERN_CRIT"VLAD:%s 9",__func__);
		goto revert4;
	}

	ret = timer_pending(&priv->mcast_timeout);
	if (ret) {
		printk(KERN_CRIT"VLAD:%s 10",__func__);
		goto revert5;
	}

#ifndef CONFIG_CW1200_DISABLE_BLOCKACK
	/* Cancel block ack stat timer */
	del_timer_sync(&priv->ba_timer);
#endif

	/* Store suspend state */
	pm_state->suspend_state = state;

	/* Enable IRQ wake */
	ret = priv->sbus_ops->power_mgmt(priv->sbus_priv, true);
	if (ret) {
		wiphy_err(priv->hw->wiphy,
			"%s: PM request failed: %d. WoW is disabled.\n",
			__func__, ret);
		cw1200_wow_resume(hw);
		return -EBUSY;
	}

	/* Force resume if event is coming from the device. */
	if (atomic_read(&priv->bh_rx)) {
		cw1200_wow_resume(hw);
		printk(KERN_CRIT"VLAD:%s 11",__func__);
		return -EAGAIN;
	}
	printk(KERN_CRIT"VLAD:%s OK",__func__);
	return 0;

revert5:
	WARN_ON(cw1200_bh_resume(priv));
revert4:
	cw1200_resume_work(priv, &priv->bss_loss_work,
			state->bss_loss_tmo);
	cw1200_resume_work(priv, &priv->connection_loss_work,
			state->connection_loss_tmo);
	cw1200_resume_work(priv, &priv->join_timeout,
			state->join_tmo);
	cw1200_resume_work(priv, &priv->scan.probe_work,
			state->direct_probe);
	cw1200_resume_work(priv, &priv->link_id_gc_work,
			state->link_id_gc);
	kfree(state);
revert3:
	wsm_set_udp_port_filter(priv, &cw1200_udp_port_filter_off);
	wsm_set_ether_type_filter(priv, &cw1200_ether_type_filter_off);
revert2:
	wsm_unlock_tx(priv);
	up(&priv->scan.lock);
revert1:
	mutex_unlock(&priv->conf_mutex);
	return -EBUSY;
}

int cw1200_wow_resume(struct ieee80211_hw *hw)
{
	struct cw1200_common *priv = hw->priv;
	struct cw1200_pm_state *pm_state = &priv->pm_state;
	struct cw1200_suspend_state *state;
	struct wsm_operational_mode mode = {
		.power_mode = cw1200_power_mode,
		.disableMoreFlagUsage = true,
	};

	printk(KERN_CRIT"VLAD:%s",__func__);

	state = pm_state->suspend_state;
	pm_state->suspend_state = NULL;

	/* Disable IRQ wake */
	priv->sbus_ops->power_mgmt(priv->sbus_priv, false);

	/* Resume BH thread */
//	WARN_ON(cw1200_bh_resume(priv));

	if (wait_event_interruptible_timeout(priv->wsm_startup_done,
				priv->wsm_caps.firmwareReady, 3*HZ) <= 0) {
    	printk(KERN_CRIT"VLAD:cw1200 firmware ready: waiting timeout\n");
	}

	/* Set low-power mode. */
	WARN_ON(wsm_set_operational_mode(priv, &mode));

	/* Enable multi-TX confirmation */
	WARN_ON(wsm_use_multi_tx_conf(priv, true));


	if (state->beacon_skipping) {
		printk(KERN_CRIT"VLAD: %s state->beacon_skipping true\n",__func__);
		wsm_set_beacon_wakeup_period(priv, priv->beacon_int *
				priv->join_dtim_period >
				MAX_BEACON_SKIP_TIME_MS ? 1 :
				priv->join_dtim_period, 0);
		state->beacon_skipping = false;
	}

	/* Resume delayed work */
	cw1200_resume_work(priv, &priv->bss_loss_work,
			state->bss_loss_tmo);
	cw1200_resume_work(priv, &priv->connection_loss_work,
			state->connection_loss_tmo);
	cw1200_resume_work(priv, &priv->join_timeout,
			state->join_tmo);
	cw1200_resume_work(priv, &priv->scan.probe_work,
			state->direct_probe);
	cw1200_resume_work(priv, &priv->link_id_gc_work,
			state->link_id_gc);

#ifndef CONFIG_CW1200_DISABLE_BLOCKACK
	/* Restart block ack stat */
	spin_lock_bh(&priv->ba_lock);
	if (priv->ba_cnt)
		mod_timer(&priv->ba_timer,
			jiffies + CW1200_BLOCK_ACK_INTERVAL);
	spin_unlock_bh(&priv->ba_lock);
#endif



	/* Remove UDP port filter */
	wsm_set_udp_port_filter(priv, &cw1200_udp_port_filter_off);

	/* Remove ethernet frame type filter */
	wsm_set_ether_type_filter(priv, &cw1200_ether_type_filter_off);

	/* Unlock datapath */
	wsm_unlock_tx(priv);

	/* Unlock scan */
	up(&priv->scan.lock);

	/* Unlock configuration mutex */
	mutex_unlock(&priv->conf_mutex);

	/* Free memory */
	kfree(state);

	return 0;
}
#else
int cw1200_wow_suspend(struct ieee80211_hw *hw, struct cfg80211_wowlan *wowlan)
{
	struct cw1200_common *priv = hw->priv;
	struct cw1200_pm_state *pm_state = &priv->pm_state;
	struct cw1200_suspend_state *state;
	int ret;

	printk(KERN_CRIT"VLAD:%s",__func__);

#ifndef CONFIG_WAKELOCK
	spin_lock_bh(&pm_state->lock);
	ret = timer_pending(&pm_state->stay_awake);
	spin_unlock_bh(&pm_state->lock);
	if (ret) {
		printk(KERN_CRIT"VLAD:%s 1",__func__);
		return -EAGAIN;
	}
#endif

	/* Do not suspend when datapath is not idle */
	if (priv->tx_queue_stats.num_queued) {
		printk(KERN_CRIT"VLAD:%s 2",__func__);
		return -EBUSY;
	}

	/* Make sure there is no configuration requests in progress. */
	if (!mutex_trylock(&priv->conf_mutex)) {
		printk(KERN_CRIT"VLAD:%s 3",__func__);
		return -EBUSY;
	}

	/* Ensure pending operations are done.
	 * Note also that wow_suspend must return in ~2.5sec, before
	 * watchdog is triggered. */
	if (priv->channel_switch_in_progress) {
		printk(KERN_CRIT"VLAD:%s 4",__func__);
		goto revert1;
	}
	/* Do not suspend when join work is scheduled */
	if (work_pending(&priv->join_work)) {
		printk(KERN_CRIT"VLAD:%s 5",__func__);
		goto revert1;
	}

	/* Do not suspend when scanning */
	if (down_trylock(&priv->scan.lock)) {
		printk(KERN_CRIT"VLAD:%s 6",__func__);
		goto revert1;
	}

	/* Lock TX. */
	wsm_lock_tx_async(priv);

	/* Wait to avoid possible race with bh code.
	 * But do not wait too long... */
	if (wait_event_timeout(priv->bh_evt_wq,
			!priv->hw_bufs_used, HZ / 10) <= 0) {
		printk(KERN_CRIT"VLAD:%s 7",__func__);
		goto revert2;
	}



	/* Stop serving thread */
	if (cw1200_bh_suspend(priv)) {
		printk(KERN_CRIT"VLAD:%s 9",__func__);
		goto revert4;
	}

	ret = timer_pending(&priv->mcast_timeout);
	if (ret) {
		printk(KERN_CRIT"VLAD:%s 10",__func__);
		goto revert5;
	}


	/* Force resume if event is coming from the device. */
	if (atomic_read(&priv->bh_rx)) {
		cw1200_wow_resume(hw);
		printk(KERN_CRIT"VLAD:%s 11",__func__);
		return -EAGAIN;
	}


	return 0;

revert5:
	WARN_ON(cw1200_bh_resume(priv));
revert4:
revert3:
	wsm_set_udp_port_filter(priv, &cw1200_udp_port_filter_off);
	wsm_set_ether_type_filter(priv, &cw1200_ether_type_filter_off);
revert2:
	wsm_unlock_tx(priv);
	up(&priv->scan.lock);
revert1:
	mutex_unlock(&priv->conf_mutex);
	return -EBUSY;
}

int cw1200_wow_resume(struct ieee80211_hw *hw)
{
	struct cw1200_common *priv = hw->priv;
	struct cw1200_pm_state *pm_state = &priv->pm_state;
	struct cw1200_suspend_state *state;
	struct wsm_operational_mode mode = {
		.power_mode = cw1200_power_mode,
		.disableMoreFlagUsage = true,
	};

	printk(KERN_CRIT"VLAD:%s",__func__);


	/* Resume BH thread */
	WARN_ON(cw1200_bh_resume(priv));

	if (wait_event_interruptible_timeout(priv->wsm_startup_done,
				priv->wsm_caps.firmwareReady, 3*HZ) <= 0) {
    	printk(KERN_CRIT"VLAD:cw1200 firmware ready: waiting timeout\n");
	}

	/* Set low-power mode. */
	WARN_ON(wsm_set_operational_mode(priv, &mode));

	/* Enable multi-TX confirmation */
	WARN_ON(wsm_use_multi_tx_conf(priv, true));


	/* Unlock datapath */
	wsm_unlock_tx(priv);

	/* Unlock scan */
	up(&priv->scan.lock);

	/* Unlock configuration mutex */
	mutex_unlock(&priv->conf_mutex);

	return 0;
}

#endif
