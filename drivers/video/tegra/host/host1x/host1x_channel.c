/*
 * drivers/video/tegra/host/host1x/channel_host1x.c
 *
 * Tegra Graphics Host Channel
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

#include "nvhost_channel.h"
#include "dev.h"
#include "nvhost_acm.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"
#include <trace/events/nvhost.h>
#include <linux/slab.h>

#include "nvhost_hwctx.h"
#include "nvhost_intr.h"
#include "class_ids.h"

#define NV_FIFO_READ_TIMEOUT 200000

static int host1x_drain_read_fifo(struct nvhost_channel *ch,
	u32 *ptr, unsigned int count, unsigned int *pending);

static void sync_waitbases(struct nvhost_channel *ch, u32 syncpt_val)
{
	unsigned long waitbase;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	if (pdata->waitbasesync) {
		waitbase = pdata->waitbases[0];
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_load_syncpt_base_r(),
				1),
				nvhost_class_host_load_syncpt_base(waitbase,
						syncpt_val));
	}
}

static void serialize(struct nvhost_job *job)
{
	struct nvhost_channel *ch = job->ch;
	struct nvhost_syncpt *sp = &nvhost_get_host(ch->dev)->syncpt;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	int i;

	if (!job->serialize && !pdata->serialize)
		return;

	/*
	 * Force serialization by inserting a host wait for the
	 * previous job to finish before this one can commence.
	 *
	 * NOTE! This cannot be packed because otherwise we might
	 * overwrite the RESTART opcode at the end of the push
	 * buffer.
	 */

	for (i = 0; i < job->num_syncpts; ++i) {
		u32 id = job->sp[i].id;
		u32 max = nvhost_syncpt_read_max(sp, id);

		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
				host1x_uclass_wait_syncpt_r(), 1),
			nvhost_class_host_wait_syncpt(id, max));
	}
}


static void *pre_submit_ctxsave(struct nvhost_job *job,
		struct nvhost_hwctx *cur_ctx)
{
	struct nvhost_channel *ch = job->ch;
	void *ctxsave_waiter = NULL;

	/* Is a save needed? */
	if (!cur_ctx || ch->cur_ctx == job->hwctx)
		return NULL;

	if (cur_ctx->has_timedout) {
		dev_dbg(&ch->dev->dev,
			"%s: skip save of timed out context (0x%p)\n",
			__func__, ch->cur_ctx);

		return NULL;
	}

	/* Allocate save waiter if needed */
	if (cur_ctx->h->save_service) {
		ctxsave_waiter = nvhost_intr_alloc_waiter();
		if (!ctxsave_waiter)
			return ERR_PTR(-ENOMEM);
	}

	return ctxsave_waiter;
}

static void submit_ctxsave(struct nvhost_job *job, void *ctxsave_waiter,
		struct nvhost_hwctx *cur_ctx)
{
	struct nvhost_master *host = nvhost_get_host(job->ch->dev);
	struct nvhost_channel *ch = job->ch;
	u32 syncval;
	int err;
	u32 save_thresh = 0;

	/* Is a save needed? */
	if (!cur_ctx || cur_ctx == job->hwctx || cur_ctx->has_timedout)
		return;

	/* Retrieve save threshold if we have a waiter */
	if (ctxsave_waiter)
		save_thresh =
			nvhost_syncpt_read_max(&host->syncpt,
			job->sp[job->hwctx_syncpt_idx].id)
			+ cur_ctx->save_thresh;

	/* Adjust the syncpoint max */
	job->sp[job->hwctx_syncpt_idx].incrs +=
		cur_ctx->save_incrs;
	syncval = nvhost_syncpt_incr_max(&host->syncpt,
			job->sp[job->hwctx_syncpt_idx].id,
			cur_ctx->save_incrs);

	/* Send the save to channel */
	cur_ctx->valid = true;
	cur_ctx->h->save_push(cur_ctx, &ch->cdma);
	nvhost_job_get_hwctx(job, cur_ctx);

	/* Notify save service */
	if (ctxsave_waiter) {
		err = nvhost_intr_add_action(&host->intr,
			job->sp[job->hwctx_syncpt_idx].id,
			save_thresh,
			NVHOST_INTR_ACTION_CTXSAVE, cur_ctx,
			ctxsave_waiter,
			NULL);
		ctxsave_waiter = NULL;
		WARN(err, "Failed to set ctx save interrupt");
	}

	trace_nvhost_channel_context_save(ch->dev->name, cur_ctx);
}

static void submit_ctxrestore(struct nvhost_job *job)
{
	struct nvhost_master *host = nvhost_get_host(job->ch->dev);
	struct nvhost_channel *ch = job->ch;
	u32 syncval;
	struct nvhost_hwctx *ctx = job->hwctx;

	/* First check if we have a valid context to restore */
	if(ch->cur_ctx == job->hwctx || !job->hwctx || !job->hwctx->valid)
		return;

	/* Increment syncpt max */
	job->sp[job->hwctx_syncpt_idx].incrs += ctx->restore_incrs;
	syncval = nvhost_syncpt_incr_max(&host->syncpt,
			job->sp[job->hwctx_syncpt_idx].id,
			ctx->restore_incrs);

	/* Send restore buffer to channel */
	ctx->h->restore_push(ctx, &ch->cdma);

	trace_nvhost_channel_context_restore(ch->dev->name, ctx);
}

static void submit_nullkickoff(struct nvhost_job *job, u32 user_syncpt_incrs)
{
	struct nvhost_channel *ch = job->ch;
	int incr, i;
	u32 op_incr;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);

	/* push increments that correspond to nulled out commands */
	for (i = 0; i < job->num_syncpts; ++i) {
		u32 incrs = (i == job->hwctx_syncpt_idx) ?
			user_syncpt_incrs : job->sp[i].incrs;
		op_incr = nvhost_opcode_imm_incr_syncpt(
			host1x_uclass_incr_syncpt_cond_op_done_v(),
			job->sp[i].id);
		for (incr = 0; incr < (incrs >> 1); incr++)
			nvhost_cdma_push(&ch->cdma, op_incr, op_incr);
		if (incrs & 1)
			nvhost_cdma_push(&ch->cdma, op_incr,
				NVHOST_OPCODE_NOOP);
	}

	/* for 3d, waitbase needs to be incremented after each submit */
	if (pdata->class == NV_GRAPHICS_3D_CLASS_ID) {
		u32 waitbase = job->hwctx->h->waitbase;
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(
				NV_HOST1X_CLASS_ID,
				host1x_uclass_incr_syncpt_base_r(),
				1),
			nvhost_class_host_incr_syncpt_base(
				waitbase,
				job->sp[job->hwctx_syncpt_idx].incrs));
	}
}

static inline u32 gather_regnum(u32 word)
{
	return (word >> 16) & 0xfff;
}

static inline  u32 gather_type(u32 word)
{
	return (word >> 28) & 1;
}

static inline u32 gather_count(u32 word)
{
	return word & 0x3fff;
}

static void submit_gathers(struct nvhost_job *job)
{
	/* push user gathers */
	int i;
	for (i = 0 ; i < job->num_gathers; i++) {
		struct nvhost_job_gather *g = &job->gathers[i];
		u32 op1;
		u32 op2;

		/* If register is specified, add a gather with incr/nonincr.
		 * This allows writing large amounts of data directly from
		 * memory to a register. */
		if (gather_regnum(g->words))
			op1 = nvhost_opcode_gather_insert(
					gather_regnum(g->words),
					gather_type(g->words),
					gather_count(g->words));
		else
			op1 = nvhost_opcode_gather(g->words);
		op2 = job->gathers[i].mem_base + g->offset;
		nvhost_cdma_push_gather(&job->ch->cdma,
				job->memmgr,
				g->ref,
				g->offset,
				op1, op2);
	}
}

static int host1x_channel_submit(struct nvhost_job *job)
{
	struct nvhost_channel *ch = job->ch;
	struct nvhost_syncpt *sp = &nvhost_get_host(job->ch->dev)->syncpt;
	u32 user_syncpt_incrs;
	u32 prev_max = 0;
	int err, i;
	void *completed_waiters[job->num_syncpts], *ctxsave_waiter = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	struct nvhost_job_syncpt *hwctx_sp = job->sp + job->hwctx_syncpt_idx;

	memset(completed_waiters, 0, sizeof(void *) * job->num_syncpts);

	/* Bail out on timed out contexts */
	if (job->hwctx && job->hwctx->has_timedout)
		return -ETIMEDOUT;

	/* Turn on the client module and host1x */
	for (i = 0; i < job->num_syncpts; ++i)
		nvhost_module_busy(ch->dev);

	/* before error checks, return current max */
	prev_max = hwctx_sp->fence = nvhost_syncpt_read_max(sp, hwctx_sp->id);

	/* get submit lock */
	err = mutex_lock_interruptible(&ch->submitlock);
	if (err) {
		nvhost_module_idle_mult(ch->dev, job->num_syncpts);
		goto error;
	}

	/* Do the needed allocations */
	ctxsave_waiter = pre_submit_ctxsave(job, ch->cur_ctx);
	if (IS_ERR(ctxsave_waiter)) {
		err = PTR_ERR(ctxsave_waiter);
		nvhost_module_idle_mult(ch->dev, job->num_syncpts);
		mutex_unlock(&ch->submitlock);
		goto error;
	}

	for (i = 0; i < job->num_syncpts; ++i) {
		completed_waiters[i] = nvhost_intr_alloc_waiter();
		if (!completed_waiters[i]) {
			nvhost_module_idle_mult(ch->dev, job->num_syncpts);
			mutex_unlock(&ch->submitlock);
			err = -ENOMEM;
			goto error;
		}
		if (nvhost_intr_has_pending_jobs(
			&nvhost_get_host(ch->dev)->intr, job->sp[i].id, ch))
			dev_warn(&ch->dev->dev,
				"%s: cross-channel dependencies on syncpt %d\n",
				__func__, job->sp[i].id);
	}

	/* begin a CDMA submit */
	err = nvhost_cdma_begin(&ch->cdma, job);
	if (err) {
		mutex_unlock(&ch->submitlock);
		nvhost_module_idle_mult(ch->dev, job->num_syncpts);
		goto error;
	}

	serialize(job);

	/* submit_ctxsave() and submit_ctxrestore() use the channel syncpt */
	user_syncpt_incrs = hwctx_sp->incrs;

	submit_ctxsave(job, ctxsave_waiter, ch->cur_ctx);
	submit_ctxrestore(job);
	ch->cur_ctx = job->hwctx;

	/* determine fences for all syncpoints */
	for (i = 0; i < job->num_syncpts; ++i) {
		u32 incrs = (i == job->hwctx_syncpt_idx) ?
			user_syncpt_incrs :
			job->sp[i].incrs;

		/* create a valid max for client managed syncpoints */
		if (nvhost_syncpt_client_managed(sp, job->sp[i].id)) {
			u32 min = nvhost_syncpt_read(sp, job->sp[i].id);
			if (min)
				dev_warn(&job->ch->dev->dev,
					"converting an active unmanaged syncpoint %d to managed\n",
					job->sp[i].id);
			nvhost_syncpt_set_max(sp, job->sp[i].id, min);
			nvhost_syncpt_set_manager(sp, job->sp[i].id, false);
		}

		job->sp[i].fence =
			nvhost_syncpt_incr_max(sp, job->sp[i].id, incrs);
	}

	/* add a setclass for modules that require it */
	if (pdata->class)
		nvhost_cdma_push(&ch->cdma,
			nvhost_opcode_setclass(pdata->class, 0, 0),
			NVHOST_OPCODE_NOOP);

	if (job->null_kickoff)
		submit_nullkickoff(job, user_syncpt_incrs);
	else
		submit_gathers(job);

	sync_waitbases(ch, hwctx_sp->fence);

	/* end CDMA submit & stash pinned hMems into sync queue */
	nvhost_cdma_end(&ch->cdma, job);

	trace_nvhost_channel_submitted(ch->dev->name, prev_max,
		hwctx_sp->fence);

	for (i = 0; i < job->num_syncpts; ++i) {
		/* schedule a submit complete interrupt */
		err = nvhost_intr_add_action(&nvhost_get_host(ch->dev)->intr,
			job->sp[i].id, job->sp[i].fence,
			NVHOST_INTR_ACTION_SUBMIT_COMPLETE, ch,
			completed_waiters[i],
			NULL);
		WARN(err, "Failed to set submit complete interrupt");
	}

	mutex_unlock(&ch->submitlock);

	return 0;

error:
	for (i = 0; i < job->num_syncpts; ++i)
		kfree(completed_waiters[i]);

	kfree(ctxsave_waiter);
	return err;
}

static int host1x_drain_read_fifo(struct nvhost_channel *ch,
	u32 *ptr, unsigned int count, unsigned int *pending)
{
	unsigned int entries = *pending;
	unsigned long timeout = jiffies + NV_FIFO_READ_TIMEOUT;
	void __iomem *chan_regs = ch->aperture;
	while (count) {
		unsigned int num;

		while (!entries && time_before(jiffies, timeout)) {
			/* query host for number of entries in fifo */
			entries = host1x_channel_fifostat_outfentries_v(
				readl(chan_regs + host1x_channel_fifostat_r()));
			if (!entries)
				cpu_relax();
		}

		/*  timeout -> return error */
		if (!entries)
			return -EIO;

		num = min(entries, count);
		entries -= num;
		count -= num;

		while (num & ~0x3) {
			u32 arr[4];
			arr[0] = readl(chan_regs + host1x_channel_inddata_r());
			arr[1] = readl(chan_regs + host1x_channel_inddata_r());
			arr[2] = readl(chan_regs + host1x_channel_inddata_r());
			arr[3] = readl(chan_regs + host1x_channel_inddata_r());
			memcpy(ptr, arr, 4*sizeof(u32));
			ptr += 4;
			num -= 4;
		}
		while (num--)
			*ptr++ = readl(chan_regs + host1x_channel_inddata_r());
	}
	*pending = entries;

	return 0;
}

static int host1x_save_context(struct nvhost_channel *ch)
{
	struct nvhost_hwctx *hwctx_to_save;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wq);
	u32 syncpt_incrs, syncpt_val;
	int err = 0;
	void *ref;
	void *ctx_waiter = NULL, *wakeup_waiter = NULL;
	struct nvhost_job *job;
	u32 syncpt_id, waitbase;

	ctx_waiter = nvhost_intr_alloc_waiter();
	wakeup_waiter = nvhost_intr_alloc_waiter();
	if (!ctx_waiter || !wakeup_waiter) {
		err = -ENOMEM;
		goto done;
	}

	nvhost_module_busy(nvhost_get_parent(ch->dev));

	mutex_lock(&ch->submitlock);
	hwctx_to_save = ch->cur_ctx;
	if (!hwctx_to_save) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	job = nvhost_job_alloc(ch, hwctx_to_save, 0, 0, 0, 1,
			nvhost_get_host(ch->dev)->memmgr);
	if (!job) {
		err = -ENOMEM;
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	hwctx_to_save->valid = true;
	ch->cur_ctx = NULL;
	syncpt_id = hwctx_to_save->h->syncpt;
	waitbase = hwctx_to_save->h->waitbase;

	syncpt_incrs = hwctx_to_save->save_incrs;
	syncpt_val = nvhost_syncpt_incr_max(&nvhost_get_host(ch->dev)->syncpt,
					syncpt_id, syncpt_incrs);

	job->hwctx_syncpt_idx = 0;
	job->sp->id = syncpt_id;
	job->sp->waitbase = waitbase;
	job->sp->incrs = syncpt_incrs;
	job->sp->fence = syncpt_val;
	job->num_syncpts = 1;

	err = nvhost_cdma_begin(&ch->cdma, job);
	if (err) {
		mutex_unlock(&ch->submitlock);
		goto done;
	}

	hwctx_to_save->h->save_push(hwctx_to_save, &ch->cdma);
	nvhost_cdma_end(&ch->cdma, job);
	nvhost_job_put(job);
	job = NULL;

	err = nvhost_intr_add_action(&nvhost_get_host(ch->dev)->intr, syncpt_id,
			syncpt_val - syncpt_incrs + hwctx_to_save->save_thresh,
			NVHOST_INTR_ACTION_CTXSAVE, hwctx_to_save,
			ctx_waiter,
			NULL);
	ctx_waiter = NULL;
	WARN(err, "Failed to set context save interrupt");

	err = nvhost_intr_add_action(&nvhost_get_host(ch->dev)->intr,
			syncpt_id, syncpt_val,
			NVHOST_INTR_ACTION_WAKEUP, &wq,
			wakeup_waiter,
			&ref);
	wakeup_waiter = NULL;
	WARN(err, "Failed to set wakeup interrupt");
	wait_event(wq,
		nvhost_syncpt_is_expired(&nvhost_get_host(ch->dev)->syncpt,
				syncpt_id, syncpt_val));

	nvhost_intr_put_ref(&nvhost_get_host(ch->dev)->intr, syncpt_id, ref);

	nvhost_cdma_update(&ch->cdma);

	mutex_unlock(&ch->submitlock);
	nvhost_module_idle(nvhost_get_parent(ch->dev));

done:
	kfree(ctx_waiter);
	kfree(wakeup_waiter);
	return err;
}

static inline void __iomem *host1x_channel_aperture(void __iomem *p, int ndx)
{
	p += ndx * NV_HOST1X_CHANNEL_MAP_SIZE_BYTES;
	return p;
}

static inline int hwctx_handler_init(struct nvhost_channel *ch)
{
	int err = 0;

	struct nvhost_device_data *pdata = platform_get_drvdata(ch->dev);
	u32 syncpt = pdata->syncpts[0];
	u32 waitbase = pdata->waitbases[0];

	if (pdata->alloc_hwctx_handler) {
		ch->ctxhandler = pdata->alloc_hwctx_handler(syncpt,
				waitbase, ch);
		if (!ch->ctxhandler)
			err = -ENOMEM;
	}

	return err;
}

static int host1x_channel_init(struct nvhost_channel *ch,
	struct nvhost_master *dev, int index)
{
	ch->chid = index;
	mutex_init(&ch->reflock);
	mutex_init(&ch->submitlock);

	ch->aperture = host1x_channel_aperture(dev->aperture, index);

	return hwctx_handler_init(ch);
}

static const struct nvhost_channel_ops host1x_channel_ops = {
	.init = host1x_channel_init,
	.submit = host1x_channel_submit,
	.save_context = host1x_save_context,
	.drain_read_fifo = host1x_drain_read_fifo,
};
