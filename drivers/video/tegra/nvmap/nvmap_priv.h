/*
 * drivers/video/tegra/nvmap/nvmap.h
 *
 * GPU memory management driver for Tegra
 *
 * Copyright (c) 2010-2013, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *'
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __VIDEO_TEGRA_NVMAP_NVMAP_H
#define __VIDEO_TEGRA_NVMAP_NVMAP_H

#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <linux/dma-buf.h>
#include <linux/syscalls.h>
#include <linux/nvmap.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>

#include <asm/tlbflush.h>
#include <asm/dma-iommu.h>

#include <mach/iomap.h>

#include "nvmap_heap.h"

struct nvmap_device;
struct page;
struct tegra_iommu_area;

extern const struct file_operations nvmap_fd_fops;
void _nvmap_handle_free(struct nvmap_handle *h);

#if defined(CONFIG_TEGRA_NVMAP)
#define nvmap_err(_client, _fmt, ...)				\
	dev_err(nvmap_client_to_device(_client),		\
		"%s: "_fmt, __func__, ##__VA_ARGS__)

#define nvmap_warn(_client, _fmt, ...)				\
	dev_warn(nvmap_client_to_device(_client),		\
		 "%s: "_fmt, __func__, ##__VA_ARGS__)

#define nvmap_debug(_client, _fmt, ...)				\
	dev_dbg(nvmap_client_to_device(_client),		\
		"%s: "_fmt, __func__, ##__VA_ARGS__)

#define nvmap_ref_to_id(_ref)		((unsigned long)(_ref)->handle)

/*
 *
 */
struct nvmap_deferred_ops {
	struct list_head ops_list;
	spinlock_t deferred_ops_lock;
	bool enable_deferred_cache_maintenance;
	u64 deferred_maint_inner_requested;
	u64 deferred_maint_inner_flushed;
	u64 deferred_maint_outer_requested;
	u64 deferred_maint_outer_flushed;
};

/* handles allocated using shared system memory (either IOVMM- or high-order
 * page allocations */
struct nvmap_pgalloc {
	struct page **pages;
	struct tegra_iommu_area *area;
	struct list_head mru_list;	/* MRU entry for IOVMM reclamation */
	bool contig;			/* contiguous system memory */
	bool dirty;			/* area is invalid and needs mapping */
	u32 iovm_addr;	/* is non-zero, if client need specific iova mapping */
};

struct nvmap_handle {
	struct rb_node node;	/* entry on global handle tree */
	atomic_t ref;		/* reference count (i.e., # of duplications) */
	atomic_t pin;		/* pin count */
	unsigned long flags;
	size_t size;		/* padded (as-allocated) size */
	size_t orig_size;	/* original (as-requested) size */
	size_t align;
	struct nvmap_client *owner;
	struct nvmap_handle_ref *owner_ref; /* use this ref to avoid spending
			time on validation in some cases.
			if handle was duplicated by other client and
			original client destroy ref, this field
			has to be set to zero. In this case ref should be
			obtained through validation */
	struct nvmap_device *dev;
	union {
		struct nvmap_pgalloc pgalloc;
		struct nvmap_heap_block *carveout;
	};
	bool global;		/* handle may be duplicated by other clients */
	bool secure;		/* zap IOVMM area on unpin */
	bool heap_pgalloc;	/* handle is page allocated (sysmem / iovmm) */
	bool alloc;		/* handle has memory allocated */
	unsigned int userflags;	/* flags passed from userspace */
	struct mutex lock;
	void *nvhost_priv;	/* nvhost private data */
	void (*nvhost_priv_delete)(void *priv);
};

/* handle_ref objects are client-local references to an nvmap_handle;
 * they are distinct objects so that handles can be unpinned and
 * unreferenced the correct number of times when a client abnormally
 * terminates */
struct nvmap_handle_ref {
	struct nvmap_handle *handle;
	struct rb_node	node;
	atomic_t	dupes;	/* number of times to free on file close */
	atomic_t	pin;	/* number of times to unpin on free */
};

#ifdef CONFIG_NVMAP_PAGE_POOLS
#define NVMAP_UC_POOL NVMAP_HANDLE_UNCACHEABLE
#define NVMAP_WC_POOL NVMAP_HANDLE_WRITE_COMBINE
#define NVMAP_IWB_POOL NVMAP_HANDLE_INNER_CACHEABLE
#define NVMAP_WB_POOL NVMAP_HANDLE_CACHEABLE
#define NVMAP_NUM_POOLS (NVMAP_HANDLE_CACHEABLE + 1)

struct nvmap_page_pool {
	struct mutex lock;
	int npages;
	struct page **page_array;
	struct page **shrink_array;
	int max_pages;
	int flags;
};

int nvmap_page_pool_init(struct nvmap_page_pool *pool, int flags);
#endif

struct nvmap_share {
	struct tegra_iommu_client *iovmm;
	wait_queue_head_t pin_wait;
	struct mutex pin_lock;
#ifdef CONFIG_NVMAP_PAGE_POOLS
	union {
		struct nvmap_page_pool pools[NVMAP_NUM_POOLS];
		struct {
			struct nvmap_page_pool uc_pool;
			struct nvmap_page_pool wc_pool;
			struct nvmap_page_pool iwb_pool;
			struct nvmap_page_pool wb_pool;
		};
	};
#endif
#ifdef CONFIG_NVMAP_RECLAIM_UNPINNED_VM
	struct mutex mru_lock;
	struct list_head *mru_lists;
	int nr_mru;
#endif
};

struct nvmap_carveout_commit {
	size_t commit;
	struct list_head list;
};

struct nvmap_client {
	const char			*name;
	struct nvmap_device		*dev;
	struct nvmap_share		*share;
	struct rb_root			handle_refs;
	atomic_t			iovm_commit;
	size_t				iovm_limit;
	struct mutex			ref_lock;
	bool				super;
	atomic_t			count;
	struct task_struct		*task;
	struct list_head		list;
	struct nvmap_carveout_commit	carveout_commit[0];
};

struct nvmap_vma_priv {
	struct nvmap_handle *handle;
	size_t		offs;
	atomic_t	count;	/* number of processes cloning the VMA */
};

static inline void nvmap_ref_lock(struct nvmap_client *priv)
{
	mutex_lock(&priv->ref_lock);
}

static inline void nvmap_ref_unlock(struct nvmap_client *priv)
{
	mutex_unlock(&priv->ref_lock);
}

static inline struct nvmap_handle *nvmap_handle_get(struct nvmap_handle *h)
{
	if (unlikely(atomic_inc_return(&h->ref) <= 1)) {
		pr_err("%s: %s getting a freed handle\n",
			__func__, current->group_leader->comm);
		if (atomic_read(&h->ref) <= 0)
			return NULL;
	}
	return h;
}

static inline pgprot_t nvmap_pgprot(struct nvmap_handle *h, pgprot_t prot)
{
	if (h->flags == NVMAP_HANDLE_UNCACHEABLE)
		return pgprot_noncached(prot);
	else if (h->flags == NVMAP_HANDLE_WRITE_COMBINE)
		return pgprot_writecombine(prot);
#ifndef CONFIG_ARM_LPAE /* !!!FIXME!!! BUG 892578 */
	else if (h->flags == NVMAP_HANDLE_INNER_CACHEABLE)
		return pgprot_inner_writeback(prot);
#endif
	return prot;
}

#else /* CONFIG_TEGRA_NVMAP */
struct nvmap_handle *nvmap_handle_get(struct nvmap_handle *h);
void nvmap_handle_put(struct nvmap_handle *h);
pgprot_t nvmap_pgprot(struct nvmap_handle *h, pgprot_t prot);

#endif /* !CONFIG_TEGRA_NVMAP */

struct device *nvmap_client_to_device(struct nvmap_client *client);

pte_t **nvmap_alloc_pte(struct nvmap_device *dev, void **vaddr);

pte_t **nvmap_alloc_pte_irq(struct nvmap_device *dev, void **vaddr);

void nvmap_free_pte(struct nvmap_device *dev, pte_t **pte);

pte_t **nvmap_vaddr_to_pte(struct nvmap_device *dev, unsigned long vaddr);

struct nvmap_heap_block *nvmap_carveout_alloc(struct nvmap_client *dev,
					      struct nvmap_handle *handle,
					      unsigned long type);

unsigned long nvmap_carveout_usage(struct nvmap_client *c,
				   struct nvmap_heap_block *b);

struct nvmap_carveout_node;
void nvmap_carveout_commit_add(struct nvmap_client *client,
			       struct nvmap_carveout_node *node, size_t len);

void nvmap_carveout_commit_subtract(struct nvmap_client *client,
				    struct nvmap_carveout_node *node,
				    size_t len);

struct nvmap_share *nvmap_get_share_from_dev(struct nvmap_device *dev);

void nvmap_cache_maint_ops_flush(struct nvmap_device *dev,
		struct nvmap_handle *h);

struct nvmap_deferred_ops *nvmap_get_deferred_ops_from_dev(
		struct nvmap_device *dev);

int nvmap_find_cache_maint_op(struct nvmap_device *dev,
		struct nvmap_handle *h);

struct nvmap_handle *nvmap_validate_get(struct nvmap_client *client,
					unsigned long handle, bool skip_val);

struct nvmap_handle *nvmap_get_handle_id(struct nvmap_client *client,
					 unsigned long id);

void nvmap_handle_put(struct nvmap_handle *h);

struct nvmap_handle_ref *_nvmap_validate_id_locked(struct nvmap_client *priv,
						   unsigned long id);

struct nvmap_handle_ref *nvmap_create_handle(struct nvmap_client *client,
					     size_t size);

struct nvmap_handle_ref *nvmap_duplicate_handle_id(struct nvmap_client *client,
					unsigned long id, bool skip_val);

struct nvmap_handle_ref *nvmap_create_handle_from_fd(
			struct nvmap_client *client, int fd);

int nvmap_alloc_handle_id(struct nvmap_client *client,
			  unsigned long id, unsigned int heap_mask,
			  size_t align, unsigned int flags);

void nvmap_free_handle_id(struct nvmap_client *c, unsigned long id);

void nvmap_free_handle_user_id(struct nvmap_client *c, unsigned long user_id);

int nvmap_pin_ids(struct nvmap_client *client,
		  unsigned int nr, const unsigned long *ids);

void nvmap_unpin_ids(struct nvmap_client *priv,
		     unsigned int nr, const unsigned long *ids);

int nvmap_handle_remove(struct nvmap_device *dev, struct nvmap_handle *h);

void nvmap_handle_add(struct nvmap_device *dev, struct nvmap_handle *h);

int is_nvmap_vma(struct vm_area_struct *vma);

phys_addr_t _nvmap_pin(struct nvmap_client *c, struct nvmap_handle_ref *r);

void nvmap_unpin_handles(struct nvmap_client *client,
			 struct nvmap_handle **h, int nr);

int nvmap_get_dmabuf_fd(struct nvmap_client *client, ulong id);
ulong nvmap_get_id_from_dmabuf_fd(struct nvmap_client *client, int fd);

#ifdef CONFIG_COMPAT
ulong unmarshal_user_handle(__u32 handle);
__u32 marshal_kernel_handle(ulong handle);
ulong unmarshal_user_id(u32 id);
#else
ulong unmarshal_user_handle(struct nvmap_handle *handle);
struct nvmap_handle *marshal_kernel_handle(ulong handle);
ulong unmarshal_user_id(ulong id);
#endif

static inline void nvmap_flush_tlb_kernel_page(unsigned long kaddr)
{
#ifdef CONFIG_ARM_ERRATA_798181
	flush_tlb_kernel_page_skip_errata_798181(kaddr);
#else
	flush_tlb_kernel_page(kaddr);
#endif
}

extern void v7_flush_kern_cache_all(void *);
extern void v7_clean_kern_cache_all(void *);
extern void __flush_dcache_page(struct address_space *, struct page *);

extern size_t cache_maint_outer_threshold;

static inline void inner_flush_cache_all(void)
{
#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
	v7_flush_kern_cache_all(NULL);
#else
	on_each_cpu(v7_flush_kern_cache_all, NULL, 1);
#endif
}

static inline void inner_clean_cache_all(void)
{
#if defined(CONFIG_ARCH_TEGRA_11x_SOC)
	v7_clean_kern_cache_all(NULL);
#else
	on_each_cpu(v7_clean_kern_cache_all, NULL, 1);
#endif
}

struct tegra_iommu_client {
	struct device *dev;
};

struct tegra_iommu_area {
	dma_addr_t		iovm_start;
	size_t			iovm_length;
	pgprot_t		pgprot;
	struct device		*dev;
	u32			flags; /* for internal consistency */
};

#define TEGRA_IOMMU_IOVA	BIT(0) /* only IOVA allocted */
#define TEGRA_IOMMU_MAP		BIT(1) /* create map between IOVA and pages */

#define tegra_iommu_vm_insert_pfn(area, handle, pfn)			\
	({								\
		dma_addr_t da;						\
		struct device *dev = area->dev;				\
		const struct dma_map_ops *ops = get_dma_ops(dev);	\
		DEFINE_DMA_ATTRS(attrs);				\
									\
		if (WARN_ON(!(area->flags & TEGRA_IOMMU_IOVA) ||	\
			    (area->flags & TEGRA_IOMMU_MAP))) {		\
			pr_err("%s(): wrong area flags: %08x\n",	\
			       __func__, area->flags);			\
			return -EINVAL;					\
		}							\
									\
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);		\
		da = ops->map_page_at(dev, pfn_to_page(pfn), handle,	\
				 PAGE_SIZE, 0, 0, &attrs);		\
		if (dma_mapping_error(dev, da))				\
			return -ENOMEM;					\
									\
		area->flags |= TEGRA_IOMMU_MAP;				\
		return 0;						\
	})

static inline int tegra_iommu_vm_insert_pages(struct tegra_iommu_area *area,
					      dma_addr_t va,
					      struct page **pages, size_t count)
{
	dma_addr_t da;
	struct device *dev = area->dev;
	const struct dma_map_ops *ops = get_dma_ops(dev);
	DEFINE_DMA_ATTRS(attrs);

	if (WARN_ON(!(area->flags & TEGRA_IOMMU_IOVA) ||
		    (area->flags & TEGRA_IOMMU_MAP))) {
		pr_err("%s(): area flags: %08x\n", __func__, area->flags);
		return -EINVAL;
	}

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	da = ops->map_pages(dev, pages, va, count, 0, &attrs);
	if (dma_mapping_error(dev, da))
		return -ENOMEM;

	area->flags |= TEGRA_IOMMU_MAP;
	return 0;
}

struct tegra_iommu_area *tegra_iommu_create_vm(struct device *dev,
		       dma_addr_t req, size_t size, pgprot_t prot);

void tegra_iommu_free_vm(struct tegra_iommu_area *area);

void tegra_iommu_zap_vm(struct tegra_iommu_area *area);

struct tegra_iommu_client *tegra_iommu_alloc_client(struct device *dev);

void tegra_iommu_free_client(struct tegra_iommu_client *client);

static inline ulong tegra_iommu_get_vm_size(struct tegra_iommu_client *client)
{
	return TEGRA_IOMMU_SIZE;
}

#endif /* __VIDEO_TEGRA_NVMAP_NVMAP_H */
