/*
 * Copyright (C) 2010 op, Inc.
 * Author: Andy
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <asm/ioctls.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/stacktrace.h>
#include <linux/hugetlb.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
/*#include <asm/atomic.h>*/
#include <linux/atomic.h>

#include <asm/page.h>
#include <asm/pgtable.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_CMA
#include <linux/cma.h>
#endif

#include "oem_trace.h"


#define IOCTL_OTRACER_TEST			(1<<0)
#define IOCTL_OTRACER_STACK			(1<<1)
#define IOCTL_OTRACER_MEMINFO		(1<<2)
#define IOCTL_OTRACER_TASKINFO		(1<<3)
#define IOCTL_OTRACER_ALLINFO		(1<<4)
#define IOCTL_OTRACER_TOLCD			(1<<5)

#define IOCTL_OTRACER_PANIC		(1<<12)

struct vmalloc_info {
	unsigned long	used;
	unsigned long	largest_chunk;
};


void backtrace_test_saved(void)
{
	struct stack_trace trace;
	unsigned long entries[8];

	pr_info("\nThe following trace is a kernel self test and not a bug!\n");

	trace.nr_entries = 0;
	trace.max_entries = ARRAY_SIZE(entries);
	trace.entries = entries;
	trace.skip = 0;

	pr_info("Testing a dump_stack.\n");
	dump_stack();
	pr_info("Testing a print_stack_trace.\n");
	print_stack_trace(&trace, 0);
}

void tasks_mem_get(
struct mm_struct *mm, unsigned long *vsize, unsigned long *vrss)
{
	unsigned long hiwater_vm, total_vm, hiwater_rss, total_rss;

	hiwater_vm = total_vm = mm->total_vm;
	if (hiwater_vm < mm->hiwater_vm)
		hiwater_vm = mm->hiwater_vm;
	hiwater_rss = total_rss = get_mm_rss(mm);
	if (hiwater_rss < mm->hiwater_rss)
		hiwater_rss = mm->hiwater_rss;

	*vsize = total_vm << (PAGE_SHIFT-10);
	*vrss = total_rss << (PAGE_SHIFT-10);
}

static char *task_state_array[] = {
	"R-0",		/*	0 (running) */
	"S-1",		/*	1 (sleeping) */
	"D-2",	/*	2 (disk sleep) */
	"T-4",		/*	4 (stopped) */
	"T-8",	/*	8 (tracing stop) */
	"Z-F",		/* 16 (zombie) */
	"X-"		/* 32 (dead) */
};

static inline const char *get_task_state(struct task_struct *tsk)
{
	unsigned int state = (tsk->state & TASK_REPORT) | tsk->exit_state;
	char **p = &task_state_array[0];

	while (state) {
		p++;
		state >>= 1;
	}
	return *p;
}

void tasks_test_saved(void)
{
	struct task_struct *p;
	struct cred *cred = NULL;
	struct mm_struct *mm;
	unsigned long vsize = 0, vrss = 0;

	pr_info("\nThe following trace is a kernel tasks test and not a bug!\n");

	pr_info("USER\tPID\tVSIZE\tRSS\tSTATE\tNAME\n");
	write_lock_irq(&tasklist_lock);
	for_each_process(p) {

		cred = (struct cred *)get_cred((struct cred *) __task_cred(p));

		vsize = 0;
		vrss = 0;
		mm = get_task_mm(p);

		if (mm)
			tasks_mem_get(mm, &vsize, &vrss);

		pr_info("%u\t%d\t%ld\t%ld\t%s\t%s\n",
			(cred->uid).val,
			task_pid_nr(p),
			vsize,
			vrss,
			get_task_state(p),
			p->comm);
	}
	write_unlock_irq(&tasklist_lock);
}

void meminfo_test_saved(void)
{
struct sysinfo i;
	unsigned long committed;
	unsigned long allowed;
	long cached;
	unsigned long pages[NR_LRU_LISTS];
	int lru;

/*
 * display in kilobytes.
 */
#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);
	allowed = ((totalram_pages - hugetlb_total_pages())
		* sysctl_overcommit_ratio / 100) + total_swap_pages;

	cached = global_page_state(NR_FILE_PAGES) -
			total_swapcache_pages() - i.bufferram;
	if (cached < 0)
		cached = 0;


	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_page_state(NR_LRU_BASE + lru);

	/*
	 * Tagged format, for easy grepping and expansion.
	 */
	pr_info("Meminfo:\n"
		"MemTotal:		 %8lu kB\n"
		"MemFree:		 %8lu kB\n"
		"Buffers:		 %8lu kB\n"
		"Cached:		 %8lu kB\n"
		"SwapCached:	 %8lu kB\n"
		"Active:		 %8lu kB\n"
		"Inactive:		 %8lu kB\n"
		"Active(anon):	 %8lu kB\n"
		"Inactive(anon): %8lu kB\n"
		"Active(file):	 %8lu kB\n"
		"Inactive(file): %8lu kB\n"
		"Unevictable:	 %8lu kB\n"
		"Mlocked:		 %8lu kB\n"
#ifdef CONFIG_HIGHMEM
		"HighTotal:      %8lu kB\n"
		"HighFree:		 %8lu kB\n"
		"LowTotal:		 %8lu kB\n"
		"LowFree:		 %8lu kB\n"
#endif
#ifndef CONFIG_MMU
		"MmapCopy:		 %8lu kB\n"
#endif
		"SwapTotal:      %8lu kB\n"
		"SwapFree:       %8lu kB\n"
		"Dirty:          %8lu kB\n"
		"Writeback:      %8lu kB\n"
		"AnonPages:      %8lu kB\n"
		"Mapped:		 %8lu kB\n"
		"Shmem:          %8lu kB\n"
		"Slab:			 %8lu kB\n"
		"SReclaimable:	 %8lu kB\n"
		"SUnreclaim:	 %8lu kB\n"
		"KernelStack:	 %8lu kB\n"
		"PageTables:	 %8lu kB\n"
#ifdef CONFIG_QUICKLIST
		"Quicklists:	 %8lu kB\n"
#endif
		"NFS_Unstable:	 %8lu kB\n"
		"Bounce:		 %8lu kB\n"
		"WritebackTmp:	 %8lu kB\n"
		"CommitLimit:	 %8lu kB\n"
		"Committed_AS:	 %8lu kB\n"
		"VmallocTotal:	 %8lu kB\n"
		"VmallocUsed:	 %8lu kB\n"
		"VmallocChunk:	 %8lu kB\n"
#ifdef CONFIG_MEMORY_FAILURE
		"HardwareCorrupted: %5lu kB\n"
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		"AnonHugePages:  %8lu kB\n"
#endif
#ifdef CONFIG_CMA
		"CmaTotal:		 %8lu kB\n"
		"CmaFree:		 %8lu kB\n"
#endif
		,
		K(i.totalram),
		K(i.freeram),
		K(i.bufferram),
		K(cached),
		K(total_swapcache_pages()),
		K(pages[LRU_ACTIVE_ANON]   + pages[LRU_ACTIVE_FILE]),
		K(pages[LRU_INACTIVE_ANON] + pages[LRU_INACTIVE_FILE]),
		K(pages[LRU_ACTIVE_ANON]),
		K(pages[LRU_INACTIVE_ANON]),
		K(pages[LRU_ACTIVE_FILE]),
		K(pages[LRU_INACTIVE_FILE]),
		K(pages[LRU_UNEVICTABLE]),
		K(global_page_state(NR_MLOCK)),
#ifdef CONFIG_HIGHMEM
		K(i.totalhigh),
		K(i.freehigh),
		K(i.totalram-i.totalhigh),
		K(i.freeram-i.freehigh),
#endif
#ifndef CONFIG_MMU
		K((unsigned long) atomic_long_read(&mmap_pages_allocated)),
#endif
		K(i.totalswap),
		K(i.freeswap),
		K(global_page_state(NR_FILE_DIRTY)),
		K(global_page_state(NR_WRITEBACK)),
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		K(global_page_state(NR_PAGETABLE)
		  + global_page_state(NR_ANON_TRANSPARENT_HUGEPAGES) *
		  HPAGE_PMD_NR),
#else
		K(global_page_state(NR_PAGETABLE)),
#endif
		K(global_page_state(NR_FILE_MAPPED)),
		K(global_page_state(NR_SHMEM)),
		K(global_page_state(NR_SLAB_RECLAIMABLE) +
				global_page_state(NR_SLAB_UNRECLAIMABLE)),
		K(global_page_state(NR_SLAB_RECLAIMABLE)),
		K(global_page_state(NR_SLAB_UNRECLAIMABLE)),
		global_page_state(NR_KERNEL_STACK_KB) * THREAD_SIZE / 1024,
		K(global_page_state(NR_PAGETABLE)),
#ifdef CONFIG_QUICKLIST
		K(quicklist_total_size()),
#endif
		K(global_page_state(NR_UNSTABLE_NFS)),
		K(global_page_state(NR_BOUNCE)),
		K(global_page_state(NR_WRITEBACK_TEMP)),
		K(allowed),
		K(committed),
		(unsigned long)VMALLOC_TOTAL >> 10,
	   0ul, /* used to be vmalloc 'used'*/
	   0ul	/* used to be vmalloc 'largest_chunk'*/
#ifdef CONFIG_MEMORY_FAILURE
		, atomic_long_read(&num_poisoned_pages) << (PAGE_SHIFT - 10)
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		, K(global_page_state(NR_ANON_TRANSPARENT_HUGEPAGES) *
		   HPAGE_PMD_NR)
#endif
#ifdef CONFIG_CMA
		, K(totalcma_pages)
		, K(global_page_state(NR_FREE_CMA_PAGES))
#endif
		);
#undef K
}

int oem_con_write(const unsigned char *buf, int count)
{
	return 0;
}

void console_activate(void)
{
}


static ssize_t otracer_read(struct file *filp, char __user *buf,
				size_t size, loff_t *offp)
{
	pr_info("otracer_read: initialized\n");
	return 0;
}


static int otrace_on = -1;


bool is_otrace_on(void)
{
	return !!otrace_on;
}
static ssize_t otracer_write(struct file *filp, const char __user *buf,
						size_t count, loff_t *offp)
{
	char *kbuf = NULL;

	if (!is_otrace_on())
		return count;

	pr_info("otracer_write\n");
	kbuf = kzalloc(PAGE_SIZE, GFP_KERNEL);

	if (kbuf == NULL)
		goto end;

	if (!count)
		goto free_buf;

	if (copy_from_user(kbuf, buf, ((count > PAGE_SIZE)?PAGE_SIZE:count)))
		goto free_buf;

	oem_con_write(kbuf, ((count > PAGE_SIZE)?PAGE_SIZE:count));
	kfree(kbuf);

	return count;
free_buf:
	kfree(kbuf);
end:
	pr_info("otracer_write fail!\n");
	return -EFAULT;
}

static long otracer_ioctl(struct file *filp,
		   unsigned int cmd, unsigned long arg)
{
	unsigned int cmdv = cmd;
	int i;

	cmdv = cmd;

	pr_info("otracer_ioctl: initialized. cmd=0x%x\n", cmd);

	/*
	* mwalker give a chance to change reboot
	* result to android for android framework.
	*/
	if (cmd == IOCTL_TRACE_UPDATE_REBOOTFLAG) {
		pr_info("android update reboot flag\n");
		goto end;
	}

	for (i = 0; i < 16; i++) {
		if (cmdv == 0)
			break;

		if (cmdv & IOCTL_OTRACER_TOLCD)
			cmdv &= (~IOCTL_OTRACER_TOLCD);

		if (cmdv & IOCTL_OTRACER_STACK) {
			backtrace_test_saved();
			cmdv &= (~IOCTL_OTRACER_STACK);
		}
		if (cmdv & IOCTL_OTRACER_MEMINFO) {
			meminfo_test_saved();
			cmdv &= (~IOCTL_OTRACER_MEMINFO);
		}
		if (cmdv & IOCTL_OTRACER_TASKINFO) {
			tasks_test_saved();
			cmdv &= (~IOCTL_OTRACER_TASKINFO);
		}
		if (cmdv & IOCTL_OTRACER_ALLINFO) {
			backtrace_test_saved();
			meminfo_test_saved();
			tasks_test_saved();
			cmdv &= (~IOCTL_OTRACER_ALLINFO);
		}
		if (cmdv & IOCTL_OTRACER_PANIC) {
			pr_info("ioctl panic reboot\n");
			panic("android");
			cmdv &= (~IOCTL_OTRACER_PANIC);
		}
	}
end:
	return 0;
}
static int otracer_open(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	return nonseekable_open(inode, file);
}
static int otracer_close(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	return 0;
}
static const struct file_operations otracer_fops = {
	.owner = THIS_MODULE,
	.open = otracer_open,
	.release = otracer_close,
	.read = otracer_read,
	.write = otracer_write,
	.unlocked_ioctl = otracer_ioctl,
};

static struct miscdevice otracer_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "otracer",
	.fops = &otracer_fops,
};
static int otrace_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "\notrace_on:%d\n", is_otrace_on());
	return 0;
}
static int otrace_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, otrace_proc_show, NULL);
}
static ssize_t otrace_proc_write(struct file *file, const char __user *buffer,
				 size_t count, loff_t *pos)
{
	unsigned char *lbuf;
	size_t local_count;
	unsigned long val;
	ssize_t ret;

	if (count <= 0)
		return 0;

#define LBUFSIZE 1200UL
	lbuf = kmalloc(LBUFSIZE, GFP_KERNEL);
	if (!lbuf)
		return 0;

	local_count = (LBUFSIZE - 1) > count?count:(LBUFSIZE - 1);
	if (copy_from_user(lbuf, buffer, local_count) != 0) {
		kfree(lbuf);
		return -EFAULT;
	}

	ret = kstrtoul(lbuf, 10, &val);
	if (ret)
		return ret;

	if (val == 7978)
		otrace_on = 1;
	else
		otrace_on = 0;

	pr_info("val:%ld, otrace_on:%d\n", val, otrace_on);

	kfree(lbuf);
	return count;
}



static const struct file_operations otrace_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= otrace_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= otrace_proc_write,
};

static struct proc_dir_entry *otrace_entry;
static int __init otracer_init(void)
{
	int ret;

	ret = misc_register(&otracer_misc);
	if (unlikely(ret)) {
		pr_err("otracer: failed to register misc device!\n");
		return ret;
	}

	/* Set up the proc file system */
	otrace_entry = proc_create("otrace_on", 0644, NULL, &otrace_proc_fops);
	if (!otrace_entry) {
		ret = -ENOMEM;
		goto out_misc;
	}

	pr_info("otracer: initialized\n");

	return 0;
out_misc:
	misc_deregister(&otracer_misc);
	return ret;
}

static void __exit otracer_exit(void)
{

	remove_proc_entry("otrace_on", NULL);

	misc_deregister(&otracer_misc);

	pr_info("otracer: exit\n");
}

module_init(otracer_init);
module_exit(otracer_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("oem tracer");
MODULE_AUTHOR("Andy");


