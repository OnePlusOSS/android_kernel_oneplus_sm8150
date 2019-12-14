/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SCHED_MM_H
#define _LINUX_SCHED_MM_H

#include <linux/kernel.h>
#include <linux/atomic.h>
#include <linux/sched.h>
#include <linux/mm_types.h>
#include <linux/gfp.h>

/*
 * Routines for handling mm_structs
 */
extern struct mm_struct * mm_alloc(void);

/**
 * mmgrab() - Pin a &struct mm_struct.
 * @mm: The &struct mm_struct to pin.
 *
 * Make sure that @mm will not get freed even after the owning task
 * exits. This doesn't guarantee that the associated address space
 * will still exist later on and mmget_not_zero() has to be used before
 * accessing it.
 *
 * This is a preferred way to to pin @mm for a longer/unbounded amount
 * of time.
 *
 * Use mmdrop() to release the reference acquired by mmgrab().
 *
 * See also <Documentation/vm/active_mm.txt> for an in-depth explanation
 * of &mm_struct.mm_count vs &mm_struct.mm_users.
 */
static inline void mmgrab(struct mm_struct *mm)
{
	atomic_inc(&mm->mm_count);
}

/* mmdrop drops the mm and the page tables */
extern void __mmdrop(struct mm_struct *);
static inline void mmdrop(struct mm_struct *mm)
{
	if (unlikely(atomic_dec_and_test(&mm->mm_count)))
		__mmdrop(mm);
}

static inline void mmdrop_async_fn(struct work_struct *work)
{
	struct mm_struct *mm = container_of(work, struct mm_struct, async_put_work);
	__mmdrop(mm);
}

static inline void mmdrop_async(struct mm_struct *mm)
{
	if (unlikely(atomic_dec_and_test(&mm->mm_count))) {
		INIT_WORK(&mm->async_put_work, mmdrop_async_fn);
		schedule_work(&mm->async_put_work);
	}
}

/*
 * This has to be called after a get_task_mm()/mmget_not_zero()
 * followed by taking the mmap_sem for writing before modifying the
 * vmas or anything the coredump pretends not to change from under it.
 *
 * It also has to be called when mmgrab() is used in the context of
 * the process, but then the mm_count refcount is transferred outside
 * the context of the process to run down_write() on that pinned mm.
 *
 * NOTE: find_extend_vma() called from GUP context is the only place
 * that can modify the "mm" (notably the vm_start/end) under mmap_sem
 * for reading and outside the context of the process, so it is also
 * the only case that holds the mmap_sem for reading that must call
 * this function. Generally if the mmap_sem is hold for reading
 * there's no need of this check after get_task_mm()/mmget_not_zero().
 *
 * This function can be obsoleted and the check can be removed, after
 * the coredump code will hold the mmap_sem for writing before
 * invoking the ->core_dump methods.
 */
static inline bool mmget_still_valid(struct mm_struct *mm)
{
	return likely(!mm->core_state);
}

/**
 * mmget() - Pin the address space associated with a &struct mm_struct.
 * @mm: The address space to pin.
 *
 * Make sure that the address space of the given &struct mm_struct doesn't
 * go away. This does not protect against parts of the address space being
 * modified or freed, however.
 *
 * Never use this function to pin this address space for an
 * unbounded/indefinite amount of time.
 *
 * Use mmput() to release the reference acquired by mmget().
 *
 * See also <Documentation/vm/active_mm.txt> for an in-depth explanation
 * of &mm_struct.mm_count vs &mm_struct.mm_users.
 */
static inline void mmget(struct mm_struct *mm)
{
	atomic_inc(&mm->mm_users);
}

static inline bool mmget_not_zero(struct mm_struct *mm)
{
	return atomic_inc_not_zero(&mm->mm_users);
}

/* mmput gets rid of the mappings and all user-space */
extern int mmput(struct mm_struct *mm);
#ifdef CONFIG_MMU
/* same as above but performs the slow path from the async context. Can
 * be called from the atomic context as well
 */
void mmput_async(struct mm_struct *);
#endif

/* Grab a reference to a task's mm, if it is not already going away */
extern struct mm_struct *get_task_mm(struct task_struct *task);
/*
 * Grab a reference to a task's mm, if it is not already going away
 * and ptrace_may_access with the mode parameter passed to it
 * succeeds.
 */
extern struct mm_struct *mm_access(struct task_struct *task, unsigned int mode);
/* Remove the current tasks stale references to the old mm_struct on exit() */
extern void exit_mm_release(struct task_struct *, struct mm_struct *);
/* Remove the current tasks stale references to the old mm_struct on exec() */
extern void exec_mm_release(struct task_struct *, struct mm_struct *);

#ifdef CONFIG_MEMCG
extern void mm_update_next_owner(struct mm_struct *mm);
#else
static inline void mm_update_next_owner(struct mm_struct *mm)
{
}
#endif /* CONFIG_MEMCG */

#ifdef CONFIG_MMU
extern void arch_pick_mmap_layout(struct mm_struct *mm);
extern unsigned long
arch_get_unmapped_area(struct file *, unsigned long, unsigned long,
		       unsigned long, unsigned long);
extern unsigned long
arch_get_unmapped_area_topdown(struct file *filp, unsigned long addr,
			  unsigned long len, unsigned long pgoff,
			  unsigned long flags);
#else
static inline void arch_pick_mmap_layout(struct mm_struct *mm) {}
#endif

static inline bool in_vfork(struct task_struct *tsk)
{
	bool ret;

	/*
	 * need RCU to access ->real_parent if CLONE_VM was used along with
	 * CLONE_PARENT.
	 *
	 * We check real_parent->mm == tsk->mm because CLONE_VFORK does not
	 * imply CLONE_VM
	 *
	 * CLONE_VFORK can be used with CLONE_PARENT/CLONE_THREAD and thus
	 * ->real_parent is not necessarily the task doing vfork(), so in
	 * theory we can't rely on task_lock() if we want to dereference it.
	 *
	 * And in this case we can't trust the real_parent->mm == tsk->mm
	 * check, it can be false negative. But we do not care, if init or
	 * another oom-unkillable task does this it should blame itself.
	 */
	rcu_read_lock();
	ret = tsk->vfork_done && tsk->real_parent->mm == tsk->mm;
	rcu_read_unlock();

	return ret;
}

/*
 * Applies per-task gfp context to the given allocation flags.
 * PF_MEMALLOC_NOIO implies GFP_NOIO
 * PF_MEMALLOC_NOFS implies GFP_NOFS
 */
static inline gfp_t current_gfp_context(gfp_t flags)
{
	/*
	 * NOIO implies both NOIO and NOFS and it is a weaker context
	 * so always make sure it makes precendence
	 */
	if (unlikely(current->flags & PF_MEMALLOC_NOIO))
		flags &= ~(__GFP_IO | __GFP_FS);
	else if (unlikely(current->flags & PF_MEMALLOC_NOFS))
		flags &= ~__GFP_FS;
	return flags;
}

#ifdef CONFIG_LOCKDEP
extern void fs_reclaim_acquire(gfp_t gfp_mask);
extern void fs_reclaim_release(gfp_t gfp_mask);
#else
static inline void fs_reclaim_acquire(gfp_t gfp_mask) { }
static inline void fs_reclaim_release(gfp_t gfp_mask) { }
#endif

static inline unsigned int memalloc_noio_save(void)
{
	unsigned int flags = current->flags & PF_MEMALLOC_NOIO;
	current->flags |= PF_MEMALLOC_NOIO;
	return flags;
}

static inline void memalloc_noio_restore(unsigned int flags)
{
	current->flags = (current->flags & ~PF_MEMALLOC_NOIO) | flags;
}

static inline unsigned int memalloc_nofs_save(void)
{
	unsigned int flags = current->flags & PF_MEMALLOC_NOFS;
	current->flags |= PF_MEMALLOC_NOFS;
	return flags;
}

static inline void memalloc_nofs_restore(unsigned int flags)
{
	current->flags = (current->flags & ~PF_MEMALLOC_NOFS) | flags;
}

static inline unsigned int memalloc_noreclaim_save(void)
{
	unsigned int flags = current->flags & PF_MEMALLOC;
	current->flags |= PF_MEMALLOC;
	return flags;
}

static inline void memalloc_noreclaim_restore(unsigned int flags)
{
	current->flags = (current->flags & ~PF_MEMALLOC) | flags;
}

#ifdef CONFIG_MEMBARRIER
enum {
	MEMBARRIER_STATE_PRIVATE_EXPEDITED_READY	= (1U << 0),
	MEMBARRIER_STATE_SWITCH_MM			= (1U << 1),
};

static inline void membarrier_execve(struct task_struct *t)
{
	atomic_set(&t->mm->membarrier_state, 0);
}
#else
static inline void membarrier_execve(struct task_struct *t)
{
}
#endif

#endif /* _LINUX_SCHED_MM_H */
