/*
 * battery-charger-gauge-comm.c -- Communication between battery charger and
 *	battery gauge driver.
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/thermal.h>
#include <linux/list.h>
#include <linux/power/battery-charger-gauge-comm.h>
#include <linux/wakelock.h>

#define JETI_TEMP_COLD		-13   //0
#define JETI_TEMP_COOL		10
#define JETI_TEMP_WARM		45
#define JETI_TEMP_HOT		60

DEFINE_MUTEX(charger_gauge_list_mutex);
static LIST_HEAD(charger_list);
static LIST_HEAD(gauge_list);

struct battery_charger_dev {
	int				cell_id;
	int				charging_restart_interval;
	char				tz_name[THERMAL_NAME_LENGTH];
	struct device			*parent_dev;
	struct battery_charging_ops	*ops;
	struct list_head		list;
	void				*drv_data;
	struct delayed_work		restart_charging_wq;
	struct delayed_work		poll_temp_monitor_wq;
	int				polling_time_sec;
	struct thermal_zone_device	*battery_tz;
	bool				start_monitoring;
	struct wake_lock		charger_wake_lock;
};

struct battery_gauge_dev {
	int				cell_id;
	char				tz_name[THERMAL_NAME_LENGTH];
	struct device			*parent_dev;
	struct battery_gauge_ops	*ops;
	struct list_head		list;
	void				*drv_data;
	struct thermal_zone_device	*battery_tz;
};

static int battery_charger_soc_get_value(struct battery_charger_dev *bc_dev)
{
	struct battery_gauge_dev *bg_dev;
	int ret = -EINVAL;

	if (!bc_dev) {
		dev_err(bc_dev->parent_dev, "Invalid parameters\n");
		return -EINVAL;
	}

	mutex_lock(&charger_gauge_list_mutex);

	list_for_each_entry(bg_dev, &gauge_list, list) {
		if (bg_dev->cell_id != bc_dev->cell_id)
			continue;
		if (bg_dev->ops && bg_dev->ops->get_soc_value)
			ret = bg_dev->ops->get_soc_value(bg_dev);
	}

	mutex_unlock(&charger_gauge_list_mutex);
	return ret;
}

static void battery_charger_restart_charging_wq(struct work_struct *work)
{
	struct battery_charger_dev *bc_dev;
	int battery_soc;

	bc_dev = container_of(work, struct battery_charger_dev,
					restart_charging_wq.work);

	battery_soc = battery_charger_soc_get_value(bc_dev);

	if (battery_soc >= 100) {
		dev_info(bc_dev->parent_dev,
			"Deferring charging restart as battery is full..\n");
		schedule_delayed_work(&bc_dev->restart_charging_wq,
				msecs_to_jiffies
				(bc_dev->charging_restart_interval * HZ));
		return;
	}

	if (!bc_dev->ops->restart_charging) {
		dev_err(bc_dev->parent_dev,
				"No callback for restart charging\n");
		return;
	}
	bc_dev->ops->restart_charging(bc_dev);
}

static void battery_charger_thermal_monitor_wq(struct work_struct *work)
{
	struct battery_charger_dev *bc_dev;
	struct device *dev;
	long temperature;
	bool charger_enable_state;
	bool charger_current_half;
	int battery_thersold_voltage;
	int ret;

	bc_dev = container_of(work, struct battery_charger_dev,
					poll_temp_monitor_wq.work);
	dev = bc_dev->parent_dev;

	if (!bc_dev->battery_tz)
		bc_dev->battery_tz =
			thermal_zone_device_find_by_name(bc_dev->tz_name);

	if (!bc_dev->battery_tz) {
		dev_info(dev, "Battery thermal zone %s is not registered yet\n",
					bc_dev->tz_name);
		schedule_delayed_work(&bc_dev->poll_temp_monitor_wq,
			msecs_to_jiffies(bc_dev->polling_time_sec * HZ));
		return;
	}

	ret = thermal_zone_get_temp(bc_dev->battery_tz, &temperature);
	if (ret < 0) {
		dev_err(dev, "Temperature read failed: %d\n ", ret);
		goto exit;
	}

	temperature = temperature / 1000;
	charger_enable_state = true;
	charger_current_half = false;
#if (CONFIG_MACH_S9321 == 1)
	battery_thersold_voltage = 4400;	
#else	
	battery_thersold_voltage = 4200;
#endif
	if (temperature <= JETI_TEMP_COLD || temperature >= JETI_TEMP_HOT) {
		charger_enable_state = false;
	//} else if (temperature <= JETI_TEMP_COOL ||
	//			temperature >= JETI_TEMP_WARM) {
		//charger_current_half = true;   //ljs remove, bc test requirements
		//battery_thersold_voltage = 4100;   //ljs remove, bc 4100mV not charge full
	}

	if (bc_dev->ops->thermal_configure)
		bc_dev->ops->thermal_configure(bc_dev, temperature,
			charger_enable_state, charger_current_half,
			battery_thersold_voltage);

exit:
	if (bc_dev->start_monitoring)
		schedule_delayed_work(&bc_dev->poll_temp_monitor_wq,
			msecs_to_jiffies(bc_dev->polling_time_sec * HZ));
	return;
}

int battery_charger_thermal_start_monitoring(
	struct battery_charger_dev *bc_dev)
{
	if (!bc_dev || !bc_dev->polling_time_sec)
		return -EINVAL;

	bc_dev->start_monitoring = true;
	schedule_delayed_work(&bc_dev->poll_temp_monitor_wq,
			msecs_to_jiffies(bc_dev->polling_time_sec * HZ));
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charger_thermal_start_monitoring);

int battery_charger_thermal_stop_monitoring(
	struct battery_charger_dev *bc_dev)
{
	if (!bc_dev || !bc_dev->polling_time_sec)
		return -EINVAL;

	bc_dev->start_monitoring = false;
	cancel_delayed_work_sync(&bc_dev->poll_temp_monitor_wq);
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charger_thermal_stop_monitoring);

int battery_charger_acquire_wake_lock(struct battery_charger_dev *bc_dev)
{
	wake_lock(&bc_dev->charger_wake_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charger_acquire_wake_lock);

int battery_charger_release_wake_lock(struct battery_charger_dev *bc_dev)
{
	wake_unlock(&bc_dev->charger_wake_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charger_release_wake_lock);

int battery_charging_restart(struct battery_charger_dev *bc_dev, int after_sec)
{
	if (!bc_dev->ops->restart_charging) {
		dev_err(bc_dev->parent_dev,
			"No callback for restart charging\n");
		return -EINVAL;
	}

	bc_dev->charging_restart_interval = after_sec;

	schedule_delayed_work(&bc_dev->restart_charging_wq,
			msecs_to_jiffies(after_sec * HZ));
	return 0;
}
EXPORT_SYMBOL_GPL(battery_charging_restart);

struct battery_charger_dev *battery_charger_register(struct device *dev,
	struct battery_charger_info *bci, void *drv_data)
{
	struct battery_charger_dev *bc_dev;

	dev_info(dev, "Registering battery charger driver\n");

	if (!dev || !bci) {
		dev_err(dev, "Invalid parameters\n");
		return ERR_PTR(-EINVAL);
	}

	bc_dev = kzalloc(sizeof(*bc_dev), GFP_KERNEL);
	if (!bc_dev) {
		dev_err(dev, "Memory alloc for bc_dev failed\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&charger_gauge_list_mutex);

	INIT_LIST_HEAD(&bc_dev->list);
	bc_dev->cell_id = bci->cell_id;
	bc_dev->ops = bci->bc_ops;
	bc_dev->parent_dev = dev;
	bc_dev->drv_data = drv_data;

	/* Thermal monitoring */
	bc_dev->polling_time_sec = bci->polling_time_sec;
	strcpy(bc_dev->tz_name, bci->tz_name ? : "");
	bc_dev->battery_tz = thermal_zone_device_find_by_name(bc_dev->tz_name);
	if (!bc_dev->battery_tz)
		dev_info(dev, "Battery thermal zone %s is not registered yet\n",
			bc_dev->tz_name);
	INIT_DELAYED_WORK(&bc_dev->poll_temp_monitor_wq,
			battery_charger_thermal_monitor_wq);

	INIT_DELAYED_WORK(&bc_dev->restart_charging_wq,
			battery_charger_restart_charging_wq);

	wake_lock_init(&bc_dev->charger_wake_lock, WAKE_LOCK_SUSPEND,
						"charger-suspend-lock");
	list_add(&bc_dev->list, &charger_list);
	mutex_unlock(&charger_gauge_list_mutex);
	return bc_dev;
}
EXPORT_SYMBOL_GPL(battery_charger_register);

void battery_charger_unregister(struct battery_charger_dev *bc_dev)
{
	mutex_lock(&charger_gauge_list_mutex);
	list_del(&bc_dev->list);
	cancel_delayed_work(&bc_dev->poll_temp_monitor_wq);
	cancel_delayed_work(&bc_dev->restart_charging_wq);
	wake_lock_destroy(&bc_dev->charger_wake_lock);
	mutex_unlock(&charger_gauge_list_mutex);
	kfree(bc_dev);
}
EXPORT_SYMBOL_GPL(battery_charger_unregister);

int battery_gauge_get_battery_temperature(struct battery_gauge_dev *bg_dev,
	int *temp)
{
	int ret;
	long temperature;

	if (!bg_dev)
		return -EINVAL;

	if (!bg_dev->battery_tz)
		bg_dev->battery_tz =
			thermal_zone_device_find_by_name(bg_dev->tz_name);

	if (!bg_dev->battery_tz) {
		dev_info(bg_dev->parent_dev,
			"Battery thermal zone %s is not registered yet\n",
			bg_dev->tz_name);
		return -ENODEV;
	}

	ret = thermal_zone_get_temp(bg_dev->battery_tz, &temperature);
	if (ret < 0)
		return ret;

	*temp = temperature / 1000;
	return 0;
}
EXPORT_SYMBOL_GPL(battery_gauge_get_battery_temperature);

struct battery_gauge_dev *battery_gauge_register(struct device *dev,
	struct battery_gauge_info *bgi, void *drv_data)
{
	struct battery_gauge_dev *bg_dev;

	dev_info(dev, "Registering battery gauge driver\n");

	if (!dev || !bgi) {
		dev_err(dev, "Invalid parameters\n");
		return ERR_PTR(-EINVAL);
	}

	bg_dev = kzalloc(sizeof(*bg_dev), GFP_KERNEL);
	if (!bg_dev) {
		dev_err(dev, "Memory alloc for bg_dev failed\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_lock(&charger_gauge_list_mutex);

	INIT_LIST_HEAD(&bg_dev->list);
	bg_dev->cell_id = bgi->cell_id;
	bg_dev->ops = bgi->bg_ops;
	bg_dev->parent_dev = dev;
	bg_dev->drv_data = drv_data;
	strcpy(bg_dev->tz_name, bgi->tz_name ? : "");
	bg_dev->battery_tz = thermal_zone_device_find_by_name(bg_dev->tz_name);
	if (!bg_dev->battery_tz)
		dev_info(dev, "Battery thermal zone %s is not registered yet\n",
			bg_dev->tz_name);
	list_add(&bg_dev->list, &gauge_list);
	mutex_unlock(&charger_gauge_list_mutex);
	return bg_dev;
}
EXPORT_SYMBOL_GPL(battery_gauge_register);

void battery_gauge_unregister(struct battery_gauge_dev *bg_dev)
{
	mutex_lock(&charger_gauge_list_mutex);
	list_del(&bg_dev->list);
	mutex_unlock(&charger_gauge_list_mutex);
	kfree(bg_dev);
}
EXPORT_SYMBOL_GPL(battery_gauge_unregister);

int battery_charging_status_update(struct battery_charger_dev *bc_dev,
	enum battery_charger_status status)
{
	struct battery_gauge_dev *node;
	int ret = -EINVAL;

	if (!bc_dev) {
		dev_err(bc_dev->parent_dev, "Invalid parameters\n");
		return -EINVAL;
	}

	mutex_lock(&charger_gauge_list_mutex);

	list_for_each_entry(node, &gauge_list, list) {
		if (node->cell_id != bc_dev->cell_id)
			continue;
		if (node->ops && node->ops->update_battery_status)
			ret = node->ops->update_battery_status(node, status);
	}

	mutex_unlock(&charger_gauge_list_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(battery_charging_status_update);

void *battery_charger_get_drvdata(struct battery_charger_dev *bc_dev)
{
	if (bc_dev)
		return bc_dev->drv_data;
	return NULL;
}
EXPORT_SYMBOL_GPL(battery_charger_get_drvdata);

void battery_charger_set_drvdata(struct battery_charger_dev *bc_dev, void *data)
{
	if (bc_dev)
		bc_dev->drv_data = data;
}
EXPORT_SYMBOL_GPL(battery_charger_set_drvdata);

void *battery_gauge_get_drvdata(struct battery_gauge_dev *bg_dev)
{
	if (bg_dev)
		return bg_dev->drv_data;
	return NULL;
}
EXPORT_SYMBOL_GPL(battery_gauge_get_drvdata);

void battery_gauge_set_drvdata(struct battery_gauge_dev *bg_dev, void *data)
{
	if (bg_dev)
		bg_dev->drv_data = data;
}
EXPORT_SYMBOL_GPL(battery_gauge_set_drvdata);

int battery_set_charging(struct battery_gauge_dev *bg_dev,
	bool enable)
{
	struct battery_charger_dev *node;
	int ret = -EINVAL;

	if (!bg_dev) {
		dev_err(bg_dev->parent_dev, "Invalid parameters\n");
		return -EINVAL;
	}

	list_for_each_entry(node, &charger_list, list) {
		if (node->cell_id != bg_dev->cell_id)
			continue;
		if (node->ops && node->ops->set_charging)
			ret = node->ops->set_charging(node, enable);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(battery_set_charging);
