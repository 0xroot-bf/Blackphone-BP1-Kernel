/*
 * drivers/video/tegra/host/nvhost_syncpt.c
 *
 * Tegra Graphics Host Syncpoints
 *
 * Copyright (c) 2010-2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/nvhost_ioctl.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/export.h>
#include <trace/events/nvhost.h>
#include "nvhost_syncpt.h"
#include "nvhost_sync.h"
#include "nvhost_acm.h"
#include "dev.h"
#include "chip_support.h"
#include "nvhost_channel.h"

#define MAX_SYNCPT_LENGTH	5

/* Name of sysfs node for min and max value */
static const char *min_name = "min";
static const char *max_name = "max";

static const char *num_syncpts_name = "num_pts";
static const char *num_mutexes_name = "num_mlocks";
static const char *num_waitbases_name = "num_bases";

/**
 * Resets syncpoint and waitbase values to sw shadows
 */
void nvhost_syncpt_reset(struct nvhost_syncpt *sp)
{
	u32 i;

	for (i = 0; i < nvhost_syncpt_nb_pts(sp); i++)
		syncpt_op().reset(sp, i);
	for (i = 0; i < nvhost_syncpt_nb_bases(sp); i++)
		syncpt_op().reset_wait_base(sp, i);
	wmb();
}

int nvhost_syncpt_get_waitbase(struct nvhost_channel *ch, int id)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	int i;
	bool ret = false;
	for (i = 0; i < NVHOST_MODULE_MAX_SYNCPTS && pdata->syncpts[i]; ++i)
		ret |= (pdata->syncpts[i] == id);

	if (!ret || (id == NVSYNCPT_2D_0))
		return NVSYNCPT_INVALID;

	return pdata->waitbases[0];
}

/**
 * Resets syncpoint and waitbase values of a
 * single client to sw shadows
 */
void nvhost_syncpt_reset_client(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_master *nvhost_master = nvhost_get_host(pdev);
	u32 id;

	BUG_ON(!(syncpt_op().reset && syncpt_op().reset_wait_base));

	for (id = 0; pdata->syncpts[id] &&
		(id < NVHOST_MODULE_MAX_SYNCPTS); ++id)
		syncpt_op().reset(&nvhost_master->syncpt, pdata->syncpts[id]);

	for (id = 0; pdata->waitbases[id] &&
		(id < NVHOST_MODULE_MAX_WAITBASES); ++id)
		syncpt_op().reset_wait_base(&nvhost_master->syncpt,
			pdata->waitbases[id]);
	wmb();
}


/**
 * Updates sw shadow state for client managed registers
 */
void nvhost_syncpt_save(struct nvhost_syncpt *sp)
{
	u32 i;

	for (i = 0; i < nvhost_syncpt_nb_pts(sp); i++) {
		if (nvhost_syncpt_client_managed(sp, i))
			syncpt_op().update_min(sp, i);
		else
			WARN_ON(!nvhost_syncpt_min_eq_max(sp, i));
	}

	for (i = 0; i < nvhost_syncpt_nb_bases(sp); i++)
		syncpt_op().read_wait_base(sp, i);
}

/**
 * Updates the last value read from hardware.
 */
u32 nvhost_syncpt_update_min(struct nvhost_syncpt *sp, u32 id)
{
	u32 val;

	val = syncpt_op().update_min(sp, id);
	trace_nvhost_syncpt_update_min(id, val);

	return val;
}

/**
 * Get the current syncpoint value
 */
u32 nvhost_syncpt_read(struct nvhost_syncpt *sp, u32 id)
{
	u32 val;
	nvhost_module_busy(syncpt_to_dev(sp)->dev);
	val = syncpt_op().update_min(sp, id);
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	return val;
}

/**
 * Get the current syncpoint base
 */
u32 nvhost_syncpt_read_wait_base(struct nvhost_syncpt *sp, u32 id)
{
	u32 val;
	nvhost_module_busy(syncpt_to_dev(sp)->dev);
	syncpt_op().read_wait_base(sp, id);
	val = sp->base_val[id];
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	return val;
}

/**
 * Write a cpu syncpoint increment to the hardware, without touching
 * the cache. Caller is responsible for host being powered.
 */
void nvhost_syncpt_cpu_incr(struct nvhost_syncpt *sp, u32 id)
{
	syncpt_op().cpu_incr(sp, id);
}

/**
 * Increment syncpoint value from cpu, updating cache
 */
void nvhost_syncpt_incr(struct nvhost_syncpt *sp, u32 id)
{
	if (nvhost_syncpt_client_managed(sp, id))
		nvhost_syncpt_incr_max(sp, id, 1);
	nvhost_module_busy(syncpt_to_dev(sp)->dev);
	nvhost_syncpt_cpu_incr(sp, id);
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
}

/**
 * Updated sync point form hardware, and returns true if syncpoint is expired,
 * false if we may need to wait
 */
static bool syncpt_update_min_is_expired(
	struct nvhost_syncpt *sp,
	u32 id,
	u32 thresh)
{
	syncpt_op().update_min(sp, id);
	return nvhost_syncpt_is_expired(sp, id, thresh);
}

/**
 * Main entrypoint for syncpoint value waits.
 */
int nvhost_syncpt_wait_timeout(struct nvhost_syncpt *sp, u32 id,
			u32 thresh, u32 timeout, u32 *value,
			struct timespec *ts)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	void *ref;
	void *waiter;
	int err = 0, check_count = 0, low_timeout = 0;
	u32 val;

	if (value)
		*value = 0;

	/* first check cache */
	if (nvhost_syncpt_is_expired(sp, id, thresh)) {
		if (value)
			*value = nvhost_syncpt_read_min(sp, id);
		if (ts)
			ktime_get_ts(ts);
		return 0;
	}

	/* keep host alive */
	nvhost_module_busy(syncpt_to_dev(sp)->dev);

	/* try to read from register */
	val = syncpt_op().update_min(sp, id);
	if (nvhost_syncpt_is_expired(sp, id, thresh)) {
		if (value)
			*value = val;
		if (ts)
			ktime_get_ts(ts);
		goto done;
	}

	if (!timeout) {
		err = -EAGAIN;
		goto done;
	}

	/* schedule a wakeup when the syncpoint value is reached */
	waiter = nvhost_intr_alloc_waiter();
	if (!waiter) {
		err = -ENOMEM;
		goto done;
	}

	err = nvhost_intr_add_action(&(syncpt_to_dev(sp)->intr), id, thresh,
				NVHOST_INTR_ACTION_WAKEUP_INTERRUPTIBLE, &wq,
				waiter,
				&ref);
	if (err)
		goto done;

	err = -EAGAIN;
	/* Caller-specified timeout may be impractically low */
	if (timeout < SYNCPT_CHECK_PERIOD)
		low_timeout = timeout;

	/* wait for the syncpoint, or timeout, or signal */
	while (timeout) {
		u32 check = min_t(u32, SYNCPT_CHECK_PERIOD, timeout);
		int remain = wait_event_interruptible_timeout(wq,
				syncpt_update_min_is_expired(sp, id, thresh),
				check);
		if (remain > 0 || nvhost_syncpt_is_expired(sp, id, thresh)) {
			if (value)
				*value = nvhost_syncpt_read_min(sp, id);
			if (ts) {
				err = nvhost_intr_release_time(ref, ts);
				if (err)
					ktime_get_ts(ts);
			}

			err = 0;
			break;
		}
		if (remain < 0) {
			err = remain;
			break;
		}
		if (timeout != NVHOST_NO_TIMEOUT)
			timeout -= check;
		if (timeout && check_count <= MAX_STUCK_CHECK_COUNT) {
			dev_warn(&syncpt_to_dev(sp)->dev->dev,
				"%s: syncpoint id %d (%s) stuck waiting %d, timeout=%d\n",
				 current->comm, id, syncpt_op().name(sp, id),
				 thresh, timeout);
			syncpt_op().debug(sp);
			if (check_count == MAX_STUCK_CHECK_COUNT) {
				if (low_timeout) {
					dev_warn(&syncpt_to_dev(sp)->dev->dev,
						"is timeout %d too low?\n",
						low_timeout);
				}
				nvhost_debug_dump(syncpt_to_dev(sp));
			}
			check_count++;
		}
	}
	nvhost_intr_put_ref(&(syncpt_to_dev(sp)->intr), id, ref);

done:
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	return err;
}

/**
 * Returns true if syncpoint is expired, false if we may need to wait
 */
static bool _nvhost_syncpt_is_expired(
	u32 current_val,
	u32 future_val,
	bool has_future_val,
	u32 thresh)
{
	/* Note the use of unsigned arithmetic here (mod 1<<32).
	 *
	 * c = current_val = min_val	= the current value of the syncpoint.
	 * t = thresh			= the value we are checking
	 * f = future_val  = max_val	= the value c will reach when all
	 *			   	  outstanding increments have completed.
	 *
	 * Note that c always chases f until it reaches f.
	 *
	 * Dtf = (f - t)
	 * Dtc = (c - t)
	 *
	 *  Consider all cases:
	 *
	 *	A) .....c..t..f.....	Dtf < Dtc	need to wait
	 *	B) .....c.....f..t..	Dtf > Dtc	expired
	 *	C) ..t..c.....f.....	Dtf > Dtc	expired	   (Dct very large)
	 *
	 *  Any case where f==c: always expired (for any t).  	Dtf == Dcf
	 *  Any case where t==c: always expired (for any f).  	Dtf >= Dtc (because Dtc==0)
	 *  Any case where t==f!=c: always wait.	 	Dtf <  Dtc (because Dtf==0,
	 *							Dtc!=0)
	 *
	 *  Other cases:
	 *
	 *	A) .....t..f..c.....	Dtf < Dtc	need to wait
	 *	A) .....f..c..t.....	Dtf < Dtc	need to wait
	 *	A) .....f..t..c.....	Dtf > Dtc	expired
	 *
	 *   So:
	 *	   Dtf >= Dtc implies EXPIRED	(return true)
	 *	   Dtf <  Dtc implies WAIT	(return false)
	 *
	 * Note: If t is expired then we *cannot* wait on it. We would wait
	 * forever (hang the system).
	 *
	 * Note: do NOT get clever and remove the -thresh from both sides. It
	 * is NOT the same.
	 *
	 * If future valueis zero, we have a client managed sync point. In that
	 * case we do a direct comparison.
	 */
	if (has_future_val)
		return future_val - thresh >= current_val - thresh;
	else
		return (s32)(current_val - thresh) >= 0;
}

/**
 * Compares syncpoint values a and b, both of which will trigger either before
 * or after ref (i.e. a and b trigger before ref, or a and b trigger after
 * ref). Supplying ref allows us to handle wrapping correctly.
 *
 * Returns -1 if a < b (a triggers before b)
 *	    0 if a = b (a and b trigger at the same time)
 *	    1 if a > b (b triggers before a)
 */
static int _nvhost_syncpt_compare_ref(
	u32 ref,
	u32 a,
	u32 b)
{
	/*
	 * We normalize both a and b by subtracting ref from them.
	 * Denote the normalized values by a_n and b_n. Note that because
	 * of wrapping, a_n and/or b_n may be negative.
	 *
	 * The normalized values a_n and b_n satisfy:
	 * - a positive value triggers before a negative value
	 * - a smaller positive value triggers before a greater positive value
	 * - a smaller negative value (greater in absolute value) triggers
	 *   before a greater negative value (smaller in absolute value).
	 *
	 * Thus we can just stick to unsigned arithmetic and compare
	 * (u32)a_n to (u32)b_n.
	 *
	 * Just to reiterate the possible cases:
	 *
	 *	1A) ...ref..a....b....
	 *	1B) ...ref..b....a....
	 *	2A) ...b....ref..a....              b_n < 0
	 *	2B) ...a....ref..b....     a_n > 0
	 *	3A) ...a....b....ref..     a_n < 0, b_n < 0
	 *	3A) ...b....a....ref..     a_n < 0, b_n < 0
	 */
	u32 a_n = a - ref;
	u32 b_n = b - ref;
	if (a_n < b_n)
		return -1;
	else if (a_n > b_n)
		return 1;
	else
		return 0;
}

/**
 * Returns -1 if a < b (a triggers before b)
 *	    0 if a = b (a and b trigger at the same time)
 *	    1 if a > b (b triggers before a)
 */
static int _nvhost_syncpt_compare(
	u32 current_val,
	u32 future_val,
	bool has_future_val,
	u32 a,
	u32 b)
{
	bool a_expired;
	bool b_expired;

	/* Early out */
	if (a == b)
		return 0;

	a_expired = _nvhost_syncpt_is_expired(current_val, future_val,
					      has_future_val, a);
	b_expired = _nvhost_syncpt_is_expired(current_val, future_val,
					      has_future_val, b);
	if (a_expired && !b_expired) {
		/* Easy, a was earlier */
		return -1;
	} else if (!a_expired && b_expired) {
		/* Easy, b was earlier */
		return 1;
	}

	/* Both a and b are expired (trigger before current_val) or not
	 * expired (trigger after current_val), so we can use current_val
	 * as a reference value for _nvhost_syncpt_compare_ref.
	 */
	return _nvhost_syncpt_compare_ref(current_val, a, b);
}

/**
 * Returns true if syncpoint is expired, false if we may need to wait
 */
bool nvhost_syncpt_is_expired(
	struct nvhost_syncpt *sp,
	u32 id,
	u32 thresh)
{
	u32 current_val = (u32)atomic_read(&sp->min_val[id]);
	u32 future_val = (u32)atomic_read(&sp->max_val[id]);
	bool has_future_val = !nvhost_syncpt_client_managed(sp, id);
	return _nvhost_syncpt_is_expired(current_val, future_val,
					 has_future_val, thresh);
}

/**
 * Returns -1 if a < b (a triggers before b)
 *	    0 if a = b (a and b trigger at the same time)
 *	    1 if a > b (b triggers before a)
 */
int nvhost_syncpt_compare(
	struct nvhost_syncpt *sp,
	u32 id,
	u32 thresh_a,
	u32 thresh_b)
{
	u32 current_val;
	u32 future_val;
	bool has_future_val = !nvhost_syncpt_client_managed(sp, id);

	current_val = (u32)atomic_read(&sp->min_val[id]);
	future_val = (u32)atomic_read(&sp->max_val[id]);
	return _nvhost_syncpt_compare(current_val, future_val,
				      has_future_val, thresh_a, thresh_b);
}

void nvhost_syncpt_debug(struct nvhost_syncpt *sp)
{
	syncpt_op().debug(sp);
}

int nvhost_mutex_try_lock(struct nvhost_syncpt *sp, int idx)
{
	struct nvhost_master *host = syncpt_to_dev(sp);
	u32 reg;

	nvhost_module_busy(host->dev);
	reg = syncpt_op().mutex_try_lock(sp, idx);
	if (reg) {
		nvhost_module_idle(host->dev);
		return -EBUSY;
	}
	atomic_inc(&sp->lock_counts[idx]);
	return 0;
}

void nvhost_mutex_unlock(struct nvhost_syncpt *sp, int idx)
{
	syncpt_op().mutex_unlock(sp, idx);
	nvhost_module_idle(syncpt_to_dev(sp)->dev);
	atomic_dec(&sp->lock_counts[idx]);
}

/* remove a wait pointed to by patch_addr */
int nvhost_syncpt_patch_wait(struct nvhost_syncpt *sp, void *patch_addr)
{
	return syncpt_op().patch_wait(sp, patch_addr);
}

#if CONFIG_TEGRA_GRHOST_SYNC
struct nvhost_sync_timeline *nvhost_syncpt_timeline(struct nvhost_syncpt *sp,
		int idx)
{
	if (idx != NVSYNCPT_INVALID)
		return sp->timeline[idx];
	else
		return sp->timeline_invalid;
}
#endif

/* Displays the current value of the sync point via sysfs */
static ssize_t syncpt_min_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct nvhost_syncpt_attr *syncpt_attr =
		container_of(attr, struct nvhost_syncpt_attr, attr);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			nvhost_syncpt_read(&syncpt_attr->host->syncpt,
				syncpt_attr->id));
}

static ssize_t syncpt_max_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct nvhost_syncpt_attr *syncpt_attr =
		container_of(attr, struct nvhost_syncpt_attr, attr);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			nvhost_syncpt_read_max(&syncpt_attr->host->syncpt,
				syncpt_attr->id));
}

static ssize_t nvhost_capability_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct nvhost_capability_node *node =
		container_of(attr, struct nvhost_capability_node, attr);

	return snprintf(buf, PAGE_SIZE, "%u\n", node->func(node->sp));
}

static inline int nvhost_syncpt_set_sysfs_capability_node(
				struct nvhost_syncpt *sp, const char *name,
				struct nvhost_capability_node *node,
				int (*func)(struct nvhost_syncpt *sp))
{
	node->attr.attr.name = name;
	node->attr.attr.mode = S_IRUGO;
	node->attr.show = nvhost_capability_show;
	node->func = func;
	node->sp = sp;

	return sysfs_create_file(sp->caps_kobj, &node->attr.attr);
}

static int nvhost_syncpt_timeline_attr(struct nvhost_master *host,
				       struct nvhost_syncpt *sp,
				       struct nvhost_syncpt_attr *min,
				       struct nvhost_syncpt_attr *max,
				       int i)
{
	char name[MAX_SYNCPT_LENGTH];
	struct kobject *kobj;

	/* Create one directory per sync point */
	snprintf(name, sizeof(name), "%d", i);
	kobj = kobject_create_and_add(name, sp->kobj);
	if (!kobj)
		return -EIO;

	min->id = i;
	min->host = host;
	min->attr.attr.name = min_name;
	min->attr.attr.mode = S_IRUGO;
	min->attr.show = syncpt_min_show;
	sysfs_attr_init(&min->attr.attr);
	if (sysfs_create_file(kobj, &min->attr.attr))
		return -EIO;

	max->id = i;
	max->host = host;
	max->attr.attr.name = max_name;
	max->attr.attr.mode = S_IRUGO;
	max->attr.show = syncpt_max_show;
	sysfs_attr_init(&max->attr.attr);
	if (sysfs_create_file(kobj, &max->attr.attr))
		return -EIO;

	return 0;
}

int nvhost_syncpt_init(struct platform_device *dev,
		struct nvhost_syncpt *sp)
{
	int i;
	struct nvhost_master *host = syncpt_to_dev(sp);
	int err = 0;

	/* Allocate structs for min, max and base values */
	sp->min_val = kzalloc(sizeof(atomic_t) * nvhost_syncpt_nb_pts(sp),
			GFP_KERNEL);
	sp->max_val = kzalloc(sizeof(atomic_t) * nvhost_syncpt_nb_pts(sp),
			GFP_KERNEL);
	sp->base_val = kzalloc(sizeof(u32) * nvhost_syncpt_nb_bases(sp),
			GFP_KERNEL);
	sp->lock_counts =
		kzalloc(sizeof(atomic_t) * nvhost_syncpt_nb_mlocks(sp),
			GFP_KERNEL);
	sp->caps_nodes = kzalloc(sizeof(struct nvhost_capability_node) * 3,
			GFP_KERNEL);
#if CONFIG_TEGRA_GRHOST_SYNC
	sp->timeline = kzalloc(sizeof(struct nvhost_sync_timeline *) *
			nvhost_syncpt_nb_pts(sp), GFP_KERNEL);
	if (!sp->timeline) {
		err = -ENOMEM;
		goto fail;
	}
#endif

	if (!(sp->min_val && sp->max_val && sp->base_val && sp->lock_counts &&
		sp->caps_nodes)) {
		/* frees happen in the deinit */
		err = -ENOMEM;
		goto fail;
	}

	sp->kobj = kobject_create_and_add("syncpt", &dev->dev.kobj);
	if (!sp->kobj) {
		err = -EIO;
		goto fail;
	}

	sp->caps_kobj = kobject_create_and_add("capabilities", &dev->dev.kobj);
	if (!sp->caps_kobj) {
		err = -EIO;
		goto fail;
	}

	if (nvhost_syncpt_set_sysfs_capability_node(sp, num_syncpts_name,
		sp->caps_nodes, &nvhost_syncpt_nb_pts)) {
		err = -EIO;
		goto fail;
	}

	if (nvhost_syncpt_set_sysfs_capability_node(sp, num_waitbases_name,
		sp->caps_nodes + 1, &nvhost_syncpt_nb_bases)) {
		err = -EIO;
		goto fail;
	}

	if (nvhost_syncpt_set_sysfs_capability_node(sp, num_mutexes_name,
		sp->caps_nodes + 2, &nvhost_syncpt_nb_mlocks)) {
		err = -EIO;
		goto fail;
	}

	/* Allocate two attributes for each sync point: min and max */
	sp->syncpt_attrs = kzalloc(sizeof(*sp->syncpt_attrs)
			* nvhost_syncpt_nb_pts(sp) * 2, GFP_KERNEL);
	if (!sp->syncpt_attrs) {
		err = -ENOMEM;
		goto fail;
	}

	/* Fill in the attributes */
	for (i = 0; i < nvhost_syncpt_nb_pts(sp); i++) {
		struct nvhost_syncpt_attr *min = &sp->syncpt_attrs[i*2];
		struct nvhost_syncpt_attr *max = &sp->syncpt_attrs[i*2+1];

		err = nvhost_syncpt_timeline_attr(host, sp, min, max, i);
		if (err)
			goto fail;

#if CONFIG_TEGRA_GRHOST_SYNC
		sp->timeline[i] = nvhost_sync_timeline_create(sp, i);
		if (!sp->timeline[i]) {
			err = -ENOMEM;
			goto fail;
		}
#endif
	}

	err = nvhost_syncpt_timeline_attr(host, sp, &sp->invalid_min_attr,
					  &sp->invalid_max_attr,
					  NVSYNCPT_INVALID);
	if (err)
		goto fail;

	sp->timeline_invalid = nvhost_sync_timeline_create(sp,
							   NVSYNCPT_INVALID);
	if (!sp->timeline_invalid) {
		err = -ENOMEM;
		goto fail;
	}

	return err;

fail:
	nvhost_syncpt_deinit(sp);
	return err;
}

static void nvhost_syncpt_deinit_timeline(struct nvhost_syncpt *sp)
{
#if CONFIG_TEGRA_GRHOST_SYNC
	int i;
	for (i = 0; i < nvhost_syncpt_nb_pts(sp); i++) {
		if (sp->timeline && sp->timeline[i]) {
			sync_timeline_destroy(
				(struct sync_timeline *)sp->timeline[i]);
		}
	}
	kfree(sp->timeline);
	sp->timeline = NULL;
	if (sp->timeline_invalid)
		sync_timeline_destroy((struct sync_timeline *)sp->timeline_invalid);
#endif
}

void nvhost_syncpt_deinit(struct nvhost_syncpt *sp)
{
	kobject_put(sp->kobj);
	kobject_put(sp->caps_kobj);

	kfree(sp->min_val);
	sp->min_val = NULL;

	kfree(sp->max_val);
	sp->max_val = NULL;

	kfree(sp->base_val);
	sp->base_val = NULL;

	kfree(sp->lock_counts);
	sp->lock_counts = 0;

	kfree(sp->syncpt_attrs);
	sp->syncpt_attrs = NULL;

	kfree(sp->caps_nodes);
	sp->caps_nodes = NULL;

	nvhost_syncpt_deinit_timeline(sp);
}

int nvhost_syncpt_client_managed(struct nvhost_syncpt *sp, u32 id)
{
	u64 mask = 1ULL << id;
	return !!(syncpt_to_dev(sp)->info.client_managed & mask);
}

int nvhost_syncpt_nb_pts(struct nvhost_syncpt *sp)
{
	return syncpt_to_dev(sp)->info.nb_pts;
}

int nvhost_syncpt_nb_bases(struct nvhost_syncpt *sp)
{
	return syncpt_to_dev(sp)->info.nb_bases;
}

int nvhost_syncpt_nb_mlocks(struct nvhost_syncpt *sp)
{
	return syncpt_to_dev(sp)->info.nb_mlocks;
}

void nvhost_syncpt_set_manager(struct nvhost_syncpt *sp, int id, bool client)
{
	u64 mask = 1ULL << id;
	syncpt_to_dev(sp)->info.client_managed &= ~mask;
	syncpt_to_dev(sp)->info.client_managed |= client ? mask : 0;
}

/* public sync point API */
u32 nvhost_syncpt_incr_max_ext(struct platform_device *dev, u32 id, u32 incrs)
{
	struct platform_device *pdev;
	struct nvhost_syncpt *sp;

	if (!nvhost_get_parent(dev)) {
		dev_err(&dev->dev, "Incr max called with wrong dev\n");
		return 0;
	}

	/* get the parent */
	pdev = to_platform_device(dev->dev.parent);
	sp = &(nvhost_get_host(pdev)->syncpt);

	return nvhost_syncpt_incr_max(sp, id, incrs);
}
EXPORT_SYMBOL(nvhost_syncpt_incr_max_ext);

void nvhost_syncpt_cpu_incr_ext(struct platform_device *dev, u32 id)
{
	struct platform_device *pdev;
	struct nvhost_syncpt *sp;

	if (!nvhost_get_parent(dev)) {
		dev_err(&dev->dev, "Incr called with wrong dev\n");
		return;
	}

	/* get the parent */
	pdev = to_platform_device(dev->dev.parent);
	sp = &(nvhost_get_host(pdev)->syncpt);

	nvhost_syncpt_cpu_incr(sp, id);
}
EXPORT_SYMBOL(nvhost_syncpt_cpu_incr_ext);

void nvhost_syncpt_cpu_set_wait_base(struct platform_device *pdev, u32 id,
					u32 val)
{
	struct nvhost_syncpt *sp = &(nvhost_get_host(pdev)->syncpt);

	sp->base_val[id] = val;
	syncpt_op().reset_wait_base(sp, id);
	wmb();
}

u32 nvhost_syncpt_read_ext(struct platform_device *dev, u32 id)
{
	struct platform_device *pdev;
	struct nvhost_syncpt *sp;

	if (!nvhost_get_parent(dev)) {
		dev_err(&dev->dev, "Read called with wrong dev\n");
		return 0;
	}

	/* get the parent */
	pdev = to_platform_device(dev->dev.parent);
	sp = &(nvhost_get_host(pdev)->syncpt);

	return nvhost_syncpt_read(sp, id);
}
EXPORT_SYMBOL(nvhost_syncpt_read_ext);

int nvhost_syncpt_wait_timeout_ext(struct platform_device *dev, u32 id,
	u32 thresh, u32 timeout, u32 *value, struct timespec *ts)
{
	struct platform_device *pdev;
	struct nvhost_syncpt *sp;

	if (!nvhost_get_parent(dev)) {
		dev_err(&dev->dev, "Wait called with wrong dev\n");
		return -EINVAL;
	}

	/* get the parent */
	pdev = to_platform_device(dev->dev.parent);
	sp = &(nvhost_get_host(pdev)->syncpt);

	return nvhost_syncpt_wait_timeout(sp, id, thresh, timeout, value, ts);
}
EXPORT_SYMBOL(nvhost_syncpt_wait_timeout_ext);
