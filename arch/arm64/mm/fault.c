/*
 * Based on arch/arm/mm/fault.c
 *
 * Copyright (C) 1995  Linus Torvalds
 * Copyright (C) 1995-2004 Russell King
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/extable.h>
#include <linux/signal.h>
#include <linux/mm.h>
#include <linux/hardirq.h>
#include <linux/init.h>
#include <linux/kprobes.h>
#include <linux/uaccess.h>
#include <linux/page-flags.h>
#include <linux/sched/signal.h>
#include <linux/sched/debug.h>
#include <linux/highmem.h>
#include <linux/perf_event.h>
#include <linux/preempt.h>
#include <linux/hugetlb.h>

#include <asm/bug.h>
#include <asm/cmpxchg.h>
#include <asm/cpufeature.h>
#include <asm/exception.h>
#include <asm/debug-monitors.h>
#include <asm/esr.h>
#include <asm/sysreg.h>
#include <asm/system_misc.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <soc/qcom/scm.h>

#include <acpi/ghes.h>
#include <soc/qcom/scm.h>
#include <trace/events/exception.h>

struct fault_info {
	int	(*fn)(unsigned long addr, unsigned int esr,
		      struct pt_regs *regs);
	int	sig;
	int	code;
	const char *name;
};

static const struct fault_info fault_info[];

static inline const struct fault_info *esr_to_fault_info(unsigned int esr)
{
	return fault_info + (esr & 63);
}

#ifdef CONFIG_KPROBES
static inline int notify_page_fault(struct pt_regs *regs, unsigned int esr)
{
	int ret = 0;

	/* kprobe_running() needs smp_processor_id() */
	if (!user_mode(regs)) {
		preempt_disable();
		if (kprobe_running() && kprobe_fault_handler(regs, esr))
			ret = 1;
		preempt_enable();
	}

	return ret;
}
#else
static inline int notify_page_fault(struct pt_regs *regs, unsigned int esr)
{
	return 0;
}
#endif

static void data_abort_decode(unsigned int esr)
{
	pr_alert("Data abort info:\n");

	if (esr & ESR_ELx_ISV) {
		pr_alert("  Access size = %u byte(s)\n",
			 1U << ((esr & ESR_ELx_SAS) >> ESR_ELx_SAS_SHIFT));
		pr_alert("  SSE = %lu, SRT = %lu\n",
			 (esr & ESR_ELx_SSE) >> ESR_ELx_SSE_SHIFT,
			 (esr & ESR_ELx_SRT_MASK) >> ESR_ELx_SRT_SHIFT);
		pr_alert("  SF = %lu, AR = %lu\n",
			 (esr & ESR_ELx_SF) >> ESR_ELx_SF_SHIFT,
			 (esr & ESR_ELx_AR) >> ESR_ELx_AR_SHIFT);
	} else {
		pr_alert("  ISV = 0, ISS = 0x%08lx\n", esr & ESR_ELx_ISS_MASK);
	}

	pr_alert("  CM = %lu, WnR = %lu\n",
		 (esr & ESR_ELx_CM) >> ESR_ELx_CM_SHIFT,
		 (esr & ESR_ELx_WNR) >> ESR_ELx_WNR_SHIFT);
}

/*
 * Decode mem abort information
 */
static void mem_abort_decode(unsigned int esr)
{
	pr_alert("Mem abort info:\n");

	pr_alert("  Exception class = %s, IL = %u bits\n",
		 esr_get_class_string(esr),
		 (esr & ESR_ELx_IL) ? 32 : 16);
	pr_alert("  SET = %lu, FnV = %lu\n",
		 (esr & ESR_ELx_SET_MASK) >> ESR_ELx_SET_SHIFT,
		 (esr & ESR_ELx_FnV) >> ESR_ELx_FnV_SHIFT);
	pr_alert("  EA = %lu, S1PTW = %lu\n",
		 (esr & ESR_ELx_EA) >> ESR_ELx_EA_SHIFT,
		 (esr & ESR_ELx_S1PTW) >> ESR_ELx_S1PTW_SHIFT);
	pr_alert("  FSC = %u\n", (esr & ESR_ELx_FSC));

	if (esr_is_data_abort(esr))
		data_abort_decode(esr);
}

/*
 * Dump out the page tables associated with 'addr' in the currently active mm.
 */
void show_pte(unsigned long addr)
{
	struct mm_struct *mm;
	pgd_t *pgd;

	if (addr < TASK_SIZE) {
		/* TTBR0 */
		mm = current->active_mm;
		if (mm == &init_mm) {
			pr_alert("[%016lx] user address but active_mm is swapper\n",
				 addr);
			return;
		}
	} else if (addr >= VA_START) {
		/* TTBR1 */
		mm = &init_mm;
	} else {
		pr_alert("[%016lx] address between user and kernel address ranges\n",
			 addr);
		return;
	}

	pr_alert("%s pgtable: %luk pages, %u-bit VAs, pgd = %p\n",
		 mm == &init_mm ? "swapper" : "user", PAGE_SIZE / SZ_1K,
		 VA_BITS, mm->pgd);
	pgd = pgd_offset(mm, addr);
	pr_alert("[%016lx] *pgd=%016llx", addr, pgd_val(*pgd));

	do {
		pud_t *pud;
		pmd_t *pmd;
		pte_t *pte;

		if (pgd_none(*pgd) || pgd_bad(*pgd))
			break;

		pud = pud_offset(pgd, addr);
		pr_cont(", *pud=%016llx", pud_val(*pud));
		if (pud_none(*pud) || pud_bad(*pud))
			break;

		pmd = pmd_offset(pud, addr);
		pr_cont(", *pmd=%016llx", pmd_val(*pmd));
		if (pmd_none(*pmd) || pmd_bad(*pmd))
			break;

		pte = pte_offset_map(pmd, addr);
		pr_cont(", *pte=%016llx", pte_val(*pte));
		pte_unmap(pte);
	} while(0);

	pr_cont("\n");
}

/*
 * This function sets the access flags (dirty, accessed), as well as write
 * permission, and only to a more permissive setting.
 *
 * It needs to cope with hardware update of the accessed/dirty state by other
 * agents in the system and can safely skip the __sync_icache_dcache() call as,
 * like set_pte_at(), the PTE is never changed from no-exec to exec here.
 *
 * Returns whether or not the PTE actually changed.
 */
int ptep_set_access_flags(struct vm_area_struct *vma,
			  unsigned long address, pte_t *ptep,
			  pte_t entry, int dirty)
{
	pteval_t old_pteval, pteval;

	if (pte_same(*ptep, entry))
		return 0;

	/* only preserve the access flags and write permission */
	pte_val(entry) &= PTE_RDONLY | PTE_AF | PTE_WRITE | PTE_DIRTY;

	/*
	 * Setting the flags must be done atomically to avoid racing with the
	 * hardware update of the access/dirty state. The PTE_RDONLY bit must
	 * be set to the most permissive (lowest value) of *ptep and entry
	 * (calculated as: a & b == ~(~a | ~b)).
	 */
	pte_val(entry) ^= PTE_RDONLY;
	pteval = READ_ONCE(pte_val(*ptep));
	do {
		old_pteval = pteval;
		pteval ^= PTE_RDONLY;
		pteval |= pte_val(entry);
		pteval ^= PTE_RDONLY;
		pteval = cmpxchg_relaxed(&pte_val(*ptep), old_pteval, pteval);
	} while (pteval != old_pteval);

	flush_tlb_fix_spurious_fault(vma, address);
	return 1;
}

static bool is_el1_instruction_abort(unsigned int esr)
{
	return ESR_ELx_EC(esr) == ESR_ELx_EC_IABT_CUR;
}

static inline bool is_permission_fault(unsigned int esr, struct pt_regs *regs,
				       unsigned long addr)
{
	unsigned int ec       = ESR_ELx_EC(esr);
	unsigned int fsc_type = esr & ESR_ELx_FSC_TYPE;

	if (ec != ESR_ELx_EC_DABT_CUR && ec != ESR_ELx_EC_IABT_CUR)
		return false;

	if (fsc_type == ESR_ELx_FSC_PERM)
		return true;

	if (addr < TASK_SIZE && system_uses_ttbr0_pan())
		return fsc_type == ESR_ELx_FSC_FAULT &&
			(regs->pstate & PSR_PAN_BIT);

	return false;
}

/*
 * The kernel tried to access some page that wasn't present.
 */
static void __do_kernel_fault(unsigned long addr, unsigned int esr,
			      struct pt_regs *regs)
{
	const char *msg;

	/*
	 * Are we prepared to handle this kernel fault?
	 * We are almost certainly not prepared to handle instruction faults.
	 */
	if (!is_el1_instruction_abort(esr) && fixup_exception(regs))
		return;

	/*
	 * No handler, we'll have to terminate things with extreme prejudice.
	 */
	bust_spinlocks(1);

	if (is_permission_fault(esr, regs, addr)) {
		if (esr & ESR_ELx_WNR)
			msg = "write to read-only memory";
		else
			msg = "read from unreadable memory";
	} else if (addr < PAGE_SIZE) {
		msg = "NULL pointer dereference";
	} else {
		msg = "paging request";
	}

	pr_alert("Unable to handle kernel %s at virtual address %08lx\n", msg,
		 addr);

	mem_abort_decode(esr);

	show_pte(addr);
	die("Oops", regs, esr);
	bust_spinlocks(0);
	do_exit(SIGKILL);
}

/*
 * Something tried to access memory that isn't in our memory map. User mode
 * accesses just cause a SIGSEGV
 */
static void __do_user_fault(struct task_struct *tsk, unsigned long addr,
			    unsigned int esr, unsigned int sig, int code,
			    struct pt_regs *regs, int fault)
{
	struct siginfo si;
	const struct fault_info *inf;
	unsigned int lsb = 0;

	trace_user_fault(tsk, addr, esr);

	if (unhandled_signal(tsk, sig) && show_unhandled_signals_ratelimited()) {
		inf = esr_to_fault_info(esr);
		pr_info("%s[%d]: unhandled %s (%d) at 0x%08lx, esr 0x%03x",
			tsk->comm, task_pid_nr(tsk), inf->name, sig,
			addr, esr);
		print_vma_addr(KERN_CONT ", in ", regs->pc);
		pr_cont("\n");
		__show_regs(regs);
	}

	tsk->thread.fault_address = addr;
	tsk->thread.fault_code = esr;
	si.si_signo = sig;
	si.si_errno = 0;
	si.si_code = code;
	si.si_addr = (void __user *)addr;
	/*
	 * Either small page or large page may be poisoned.
	 * In other words, VM_FAULT_HWPOISON_LARGE and
	 * VM_FAULT_HWPOISON are mutually exclusive.
	 */
	if (fault & VM_FAULT_HWPOISON_LARGE)
		lsb = hstate_index_to_shift(VM_FAULT_GET_HINDEX(fault));
	else if (fault & VM_FAULT_HWPOISON)
		lsb = PAGE_SHIFT;
	si.si_addr_lsb = lsb;

	force_sig_info(sig, &si, tsk);
}

static void do_bad_area(unsigned long addr, unsigned int esr, struct pt_regs *regs)
{
	struct task_struct *tsk = current;
	const struct fault_info *inf;

	/*
	 * If we are in kernel mode at this point, we have no context to
	 * handle this fault with.
	 */
	if (user_mode(regs)) {
		inf = esr_to_fault_info(esr);
		__do_user_fault(tsk, addr, esr, inf->sig, inf->code, regs, 0);
	} else
		__do_kernel_fault(addr, esr, regs);
}

#define VM_FAULT_BADMAP		0x010000
#define VM_FAULT_BADACCESS	0x020000

static int __do_page_fault(struct vm_area_struct *vma, unsigned long addr,
			   unsigned int mm_flags, unsigned long vm_flags,
			   struct task_struct *tsk)
{
	int fault;

	fault = VM_FAULT_BADMAP;
	if (unlikely(!vma))
		goto out;
	if (unlikely(vma->vm_start > addr))
		goto check_stack;

	/*
	 * Ok, we have a good vm_area for this memory access, so we can handle
	 * it.
	 */
good_area:
	/*
	 * Check that the permissions on the VMA allow for the fault which
	 * occurred.
	 */
	if (!(vma->vm_flags & vm_flags)) {
		fault = VM_FAULT_BADACCESS;
		goto out;
	}

	return handle_mm_fault(vma, addr & PAGE_MASK, mm_flags);

check_stack:
	if (vma->vm_flags & VM_GROWSDOWN && !expand_stack(vma, addr))
		goto good_area;
out:
	return fault;
}

static bool is_el0_instruction_abort(unsigned int esr)
{
	return ESR_ELx_EC(esr) == ESR_ELx_EC_IABT_LOW;
}

static int __kprobes do_page_fault(unsigned long addr, unsigned int esr,
				   struct pt_regs *regs)
{
	struct task_struct *tsk;
	struct mm_struct *mm;
	int fault, sig, code, major = 0;
	unsigned long vm_flags = VM_READ | VM_WRITE;
	unsigned int mm_flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_KILLABLE;
	struct vm_area_struct *vma = NULL;

	if (notify_page_fault(regs, esr))
		return 0;

	tsk = current;
	mm  = tsk->mm;

	/*
	 * If we're in an interrupt or have no user context, we must not take
	 * the fault.
	 */
	if (faulthandler_disabled() || !mm)
		goto no_context;

	if (user_mode(regs))
		mm_flags |= FAULT_FLAG_USER;

	if (is_el0_instruction_abort(esr)) {
		vm_flags = VM_EXEC;
	} else if ((esr & ESR_ELx_WNR) && !(esr & ESR_ELx_CM)) {
		vm_flags = VM_WRITE;
		mm_flags |= FAULT_FLAG_WRITE;
	}

	if (addr < TASK_SIZE && is_permission_fault(esr, regs, addr)) {
		/* regs->orig_addr_limit may be 0 if we entered from EL0 */
		if (regs->orig_addr_limit == KERNEL_DS)
			die("Accessing user space memory with fs=KERNEL_DS", regs, esr);

		if (is_el1_instruction_abort(esr))
			die("Attempting to execute userspace memory", regs, esr);

		if (!search_exception_tables(regs->pc))
			die("Accessing user space memory outside uaccess.h routines", regs, esr);
	}

	perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS, 1, regs, addr);

	/*
	 * let's try a speculative page fault without grabbing the
	 * mmap_sem.
	 */
	fault = handle_speculative_fault(mm, addr, mm_flags, &vma);
	if (fault != VM_FAULT_RETRY)
		goto done;

	/*
	 * As per x86, we may deadlock here. However, since the kernel only
	 * validly references user space from well defined areas of the code,
	 * we can bug out early if this is from code which shouldn't.
	 */
	if (!down_read_trylock(&mm->mmap_sem)) {
		if (!user_mode(regs) && !search_exception_tables(regs->pc))
			goto no_context;
retry:
		down_read(&mm->mmap_sem);
	} else {
		/*
		 * The above down_read_trylock() might have succeeded in which
		 * case, we'll have missed the might_sleep() from down_read().
		 */
		might_sleep();
#ifdef CONFIG_DEBUG_VM
		if (!user_mode(regs) && !search_exception_tables(regs->pc))
			goto no_context;
#endif
	}

	if (!vma || !can_reuse_spf_vma(vma, addr))
		vma = find_vma(mm, addr);

	fault = __do_page_fault(vma, addr, mm_flags, vm_flags, tsk);
	major |= fault & VM_FAULT_MAJOR;

	if (fault & VM_FAULT_RETRY) {
		/*
		 * If we need to retry but a fatal signal is pending,
		 * handle the signal first. We do not need to release
		 * the mmap_sem because it would already be released
		 * in __lock_page_or_retry in mm/filemap.c.
		 */
#ifdef CONFIG_MEMPLUS
		count_vm_event(RETRYPAGE);
#endif
		if (fatal_signal_pending(current)) {
			if (!user_mode(regs))
				goto no_context;
			return 0;
		}

		/*
		 * Clear FAULT_FLAG_ALLOW_RETRY to avoid any risk of
		 * starvation.
		 */
		if (mm_flags & FAULT_FLAG_ALLOW_RETRY) {
			mm_flags &= ~FAULT_FLAG_ALLOW_RETRY;
			mm_flags |= FAULT_FLAG_TRIED;

			/*
			 * Do not try to reuse this vma and fetch it
			 * again since we will release the mmap_sem.
			 */
			vma = NULL;

			goto retry;
		}
	}
	up_read(&mm->mmap_sem);

done:

	/*
	 * Handle the "normal" (no error) case first.
	 */
	if (likely(!(fault & (VM_FAULT_ERROR | VM_FAULT_BADMAP |
			      VM_FAULT_BADACCESS)))) {
		/*
		 * Major/minor page fault accounting is only done
		 * once. If we go through a retry, it is extremely
		 * likely that the page will be found in page cache at
		 * that point.
		 */
		if (major) {
			tsk->maj_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MAJ, 1, regs,
				      addr);
		} else {
			tsk->min_flt++;
			perf_sw_event(PERF_COUNT_SW_PAGE_FAULTS_MIN, 1, regs,
				      addr);
		}

		return 0;
	}

	/*
	 * If we are in kernel mode at this point, we have no context to
	 * handle this fault with.
	 */
	if (!user_mode(regs))
		goto no_context;

	if (fault & VM_FAULT_OOM) {
		/*
		 * We ran out of memory, call the OOM killer, and return to
		 * userspace (which will retry the fault, or kill us if we got
		 * oom-killed).
		 */
		pagefault_out_of_memory();
		return 0;
	}

	if (fault & VM_FAULT_SIGBUS) {
		/*
		 * We had some memory, but were unable to successfully fix up
		 * this page fault.
		 */
		sig = SIGBUS;
		code = BUS_ADRERR;
	} else if (fault & (VM_FAULT_HWPOISON | VM_FAULT_HWPOISON_LARGE)) {
		sig = SIGBUS;
		code = BUS_MCEERR_AR;
	} else {
		/*
		 * Something tried to access memory that isn't in our memory
		 * map.
		 */
		sig = SIGSEGV;
		code = fault == VM_FAULT_BADACCESS ?
			SEGV_ACCERR : SEGV_MAPERR;
	}

	__do_user_fault(tsk, addr, esr, sig, code, regs, fault);
	return 0;

no_context:
	__do_kernel_fault(addr, esr, regs);
	return 0;
}

static int do_tlb_conf_fault(unsigned long addr,
				unsigned int esr,
				struct pt_regs *regs)
{
#define SCM_TLB_CONFLICT_CMD	0x1F
	struct scm_desc desc = {
		.args[0] = addr,
		.arginfo = SCM_ARGS(1),
	};

	if (scm_call2_atomic(SCM_SIP_FNID(SCM_SVC_MP, SCM_TLB_CONFLICT_CMD),
						&desc))
		return 1;

	return 0;
}

/*
 * First Level Translation Fault Handler
 *
 * We enter here because the first level page table doesn't contain a valid
 * entry for the address.
 *
 * If the address is in kernel space (>= TASK_SIZE), then we are probably
 * faulting in the vmalloc() area.
 *
 * If the init_task's first level page tables contains the relevant entry, we
 * copy the it to this task.  If not, we send the process a signal, fixup the
 * exception, or oops the kernel.
 *
 * NOTE! We MUST NOT take any locks for this case. We may be in an interrupt
 * or a critical region, and should only copy the information from the master
 * page table, nothing more.
 */
static int __kprobes do_translation_fault(unsigned long addr,
					  unsigned int esr,
					  struct pt_regs *regs)
{
	if (addr < TASK_SIZE)
		return do_page_fault(addr, esr, regs);

	do_bad_area(addr, esr, regs);
	return 0;
}

static int do_alignment_fault(unsigned long addr, unsigned int esr,
			      struct pt_regs *regs)
{
	do_bad_area(addr, esr, regs);
	return 0;
}

/*
 * This abort handler always returns "fault".
 */
static int do_bad(unsigned long addr, unsigned int esr, struct pt_regs *regs)
{
	return 1;
}

/*
 * This abort handler deals with Synchronous External Abort.
 * It calls notifiers, and then returns "fault".
 */
static int do_sea(unsigned long addr, unsigned int esr, struct pt_regs *regs)
{
	struct siginfo info;
	const struct fault_info *inf;
	int ret = 0;

	inf = esr_to_fault_info(esr);
	pr_err("Synchronous External Abort: %s (0x%08x) at 0x%016lx\n",
		inf->name, esr, addr);

	/*
	 * Synchronous aborts may interrupt code which had interrupts masked.
	 * Before calling out into the wider kernel tell the interested
	 * subsystems.
	 */
	if (IS_ENABLED(CONFIG_ACPI_APEI_SEA)) {
		if (interrupts_enabled(regs))
			nmi_enter();

		ret = ghes_notify_sea();

		if (interrupts_enabled(regs))
			nmi_exit();
	}

	info.si_signo = SIGBUS;
	info.si_errno = 0;
	info.si_code  = 0;
	if (esr & ESR_ELx_FnV)
		info.si_addr = NULL;
	else
		info.si_addr  = (void __user *)addr;
	arm64_notify_die("", regs, &info, esr);

	return ret;
}

static const struct fault_info fault_info[] = {
	{ do_bad,		SIGBUS,  0,		"ttbr address size fault"	},
	{ do_bad,		SIGBUS,  0,		"level 1 address size fault"	},
	{ do_bad,		SIGBUS,  0,		"level 2 address size fault"	},
	{ do_bad,		SIGBUS,  0,		"level 3 address size fault"	},
	{ do_translation_fault,	SIGSEGV, SEGV_MAPERR,	"level 0 translation fault"	},
	{ do_translation_fault,	SIGSEGV, SEGV_MAPERR,	"level 1 translation fault"	},
	{ do_translation_fault,	SIGSEGV, SEGV_MAPERR,	"level 2 translation fault"	},
	{ do_translation_fault,	SIGSEGV, SEGV_MAPERR,	"level 3 translation fault"	},
	{ do_bad,		SIGBUS,  0,		"unknown 8"			},
	{ do_page_fault,	SIGSEGV, SEGV_ACCERR,	"level 1 access flag fault"	},
	{ do_page_fault,	SIGSEGV, SEGV_ACCERR,	"level 2 access flag fault"	},
	{ do_page_fault,	SIGSEGV, SEGV_ACCERR,	"level 3 access flag fault"	},
	{ do_bad,		SIGBUS,  0,		"unknown 12"			},
	{ do_page_fault,	SIGSEGV, SEGV_ACCERR,	"level 1 permission fault"	},
	{ do_page_fault,	SIGSEGV, SEGV_ACCERR,	"level 2 permission fault"	},
	{ do_page_fault,	SIGSEGV, SEGV_ACCERR,	"level 3 permission fault"	},
	{ do_sea,		SIGBUS,  0,		"synchronous external abort"	},
	{ do_bad,		SIGBUS,  0,		"unknown 17"			},
	{ do_bad,		SIGBUS,  0,		"unknown 18"			},
	{ do_bad,		SIGBUS,  0,		"unknown 19"			},
	{ do_sea,		SIGBUS,  0,		"level 0 (translation table walk)"	},
	{ do_sea,		SIGBUS,  0,		"level 1 (translation table walk)"	},
	{ do_sea,		SIGBUS,  0,		"level 2 (translation table walk)"	},
	{ do_sea,		SIGBUS,  0,		"level 3 (translation table walk)"	},
	{ do_sea,		SIGBUS,  0,		"synchronous parity or ECC error" },
	{ do_bad,		SIGBUS,  0,		"unknown 25"			},
	{ do_bad,		SIGBUS,  0,		"unknown 26"			},
	{ do_bad,		SIGBUS,  0,		"unknown 27"			},
	{ do_sea,		SIGBUS,  0,		"level 0 synchronous parity error (translation table walk)"	},
	{ do_sea,		SIGBUS,  0,		"level 1 synchronous parity error (translation table walk)"	},
	{ do_sea,		SIGBUS,  0,		"level 2 synchronous parity error (translation table walk)"	},
	{ do_sea,		SIGBUS,  0,		"level 3 synchronous parity error (translation table walk)"	},
	{ do_bad,		SIGBUS,  0,		"unknown 32"			},
	{ do_alignment_fault,	SIGBUS,  BUS_ADRALN,	"alignment fault"		},
	{ do_bad,		SIGBUS,  0,		"unknown 34"			},
	{ do_bad,		SIGBUS,  0,		"unknown 35"			},
	{ do_bad,		SIGBUS,  0,		"unknown 36"			},
	{ do_bad,		SIGBUS,  0,		"unknown 37"			},
	{ do_bad,		SIGBUS,  0,		"unknown 38"			},
	{ do_bad,		SIGBUS,  0,		"unknown 39"			},
	{ do_bad,		SIGBUS,  0,		"unknown 40"			},
	{ do_bad,		SIGBUS,  0,		"unknown 41"			},
	{ do_bad,		SIGBUS,  0,		"unknown 42"			},
	{ do_bad,		SIGBUS,  0,		"unknown 43"			},
	{ do_bad,		SIGBUS,  0,		"unknown 44"			},
	{ do_bad,		SIGBUS,  0,		"unknown 45"			},
	{ do_bad,		SIGBUS,  0,		"unknown 46"			},
	{ do_bad,		SIGBUS,  0,		"unknown 47"			},
	{ do_tlb_conf_fault,	SIGBUS,  0,		"TLB conflict abort"		},
	{ do_bad,		SIGBUS,  0,		"unknown 49"			},
	{ do_bad,		SIGBUS,  0,		"unknown 50"			},
	{ do_bad,		SIGBUS,  0,		"unknown 51"			},
	{ do_bad,		SIGBUS,  0,		"implementation fault (lockdown abort)" },
	{ do_bad,		SIGBUS,  0,		"implementation fault (unsupported exclusive)" },
	{ do_bad,		SIGBUS,  0,		"unknown 54"			},
	{ do_bad,		SIGBUS,  0,		"unknown 55"			},
	{ do_bad,		SIGBUS,  0,		"unknown 56"			},
	{ do_bad,		SIGBUS,  0,		"unknown 57"			},
	{ do_bad,		SIGBUS,  0,		"unknown 58" 			},
	{ do_bad,		SIGBUS,  0,		"unknown 59"			},
	{ do_bad,		SIGBUS,  0,		"unknown 60"			},
	{ do_bad,		SIGBUS,  0,		"section domain fault"		},
	{ do_bad,		SIGBUS,  0,		"page domain fault"		},
	{ do_bad,		SIGBUS,  0,		"unknown 63"			},
};

/*
 * Handle Synchronous External Aborts that occur in a guest kernel.
 *
 * The return value will be zero if the SEA was successfully handled
 * and non-zero if there was an error processing the error or there was
 * no error to process.
 */
int handle_guest_sea(phys_addr_t addr, unsigned int esr)
{
	int ret = -ENOENT;

	if (IS_ENABLED(CONFIG_ACPI_APEI_SEA))
		ret = ghes_notify_sea();

	return ret;
}

/*
 * Dispatch a data abort to the relevant handler.
 */
asmlinkage void __exception do_mem_abort(unsigned long addr, unsigned int esr,
					 struct pt_regs *regs)
{
	const struct fault_info *inf = esr_to_fault_info(esr);
	struct siginfo info;

	if (!inf->fn(addr, esr, regs))
		return;

	pr_alert("Unhandled fault: %s (0x%08x) at 0x%016lx\n",
		 inf->name, esr, addr);

	mem_abort_decode(esr);

	info.si_signo = inf->sig;
	info.si_errno = 0;
	info.si_code  = inf->code;
	info.si_addr  = (void __user *)addr;
	arm64_notify_die("", regs, &info, esr);
}

asmlinkage void __exception do_el0_irq_bp_hardening(void)
{
	/* PC has already been checked in entry.S */
	arm64_apply_bp_hardening();
}

asmlinkage void __exception do_el0_ia_bp_hardening(unsigned long addr,
						   unsigned int esr,
						   struct pt_regs *regs)
{
	/*
	 * We've taken an instruction abort from userspace and not yet
	 * re-enabled IRQs. If the address is a kernel address, apply
	 * BP hardening prior to enabling IRQs and pre-emption.
	 */
	if (addr > TASK_SIZE)
		arm64_apply_bp_hardening();

	local_irq_enable();
	do_mem_abort(addr, esr, regs);
}


/*
 * Handle stack alignment exceptions.
 */
asmlinkage void __exception do_sp_pc_abort(unsigned long addr,
					   unsigned int esr,
					   struct pt_regs *regs)
{
	struct siginfo info;
	struct task_struct *tsk = current;

	if (user_mode(regs)) {
		if (instruction_pointer(regs) > TASK_SIZE)
			arm64_apply_bp_hardening();
		local_irq_enable();
	}

	if (show_unhandled_signals && unhandled_signal(tsk, SIGBUS))
		pr_info_ratelimited("%s[%d]: %s exception: pc=%p sp=%p\n",
				    tsk->comm, task_pid_nr(tsk),
				    esr_get_class_string(esr), (void *)regs->pc,
				    (void *)regs->sp);

	info.si_signo = SIGBUS;
	info.si_errno = 0;
	info.si_code  = BUS_ADRALN;
	info.si_addr  = (void __user *)addr;
	arm64_notify_die("Oops - SP/PC alignment exception", regs, &info, esr);
}

int __init early_brk64(unsigned long addr, unsigned int esr,
		       struct pt_regs *regs);

/*
 * __refdata because early_brk64 is __init, but the reference to it is
 * clobbered at arch_initcall time.
 * See traps.c and debug-monitors.c:debug_traps_init().
 */
static struct fault_info __refdata debug_fault_info[] = {
	{ do_bad,	SIGTRAP,	TRAP_HWBKPT,	"hardware breakpoint"	},
	{ do_bad,	SIGTRAP,	TRAP_HWBKPT,	"hardware single-step"	},
	{ do_bad,	SIGTRAP,	TRAP_HWBKPT,	"hardware watchpoint"	},
	{ do_bad,	SIGBUS,		0,		"unknown 3"		},
	{ do_bad,	SIGTRAP,	TRAP_BRKPT,	"aarch32 BKPT"		},
	{ do_bad,	SIGTRAP,	0,		"aarch32 vector catch"	},
	{ early_brk64,	SIGTRAP,	TRAP_BRKPT,	"aarch64 BRK"		},
	{ do_bad,	SIGBUS,		0,		"unknown 7"		},
};

void __init hook_debug_fault_code(int nr,
				  int (*fn)(unsigned long, unsigned int, struct pt_regs *),
				  int sig, int code, const char *name)
{
	BUG_ON(nr < 0 || nr >= ARRAY_SIZE(debug_fault_info));

	debug_fault_info[nr].fn		= fn;
	debug_fault_info[nr].sig	= sig;
	debug_fault_info[nr].code	= code;
	debug_fault_info[nr].name	= name;
}

asmlinkage int __exception do_debug_exception(unsigned long addr_if_watchpoint,
					      unsigned int esr,
					      struct pt_regs *regs)
{
	const struct fault_info *inf = debug_fault_info + DBG_ESR_EVT(esr);
	unsigned long pc = instruction_pointer(regs);
	struct siginfo info;
	int rv;

	/*
	 * Tell lockdep we disabled irqs in entry.S. Do nothing if they were
	 * already disabled to preserve the last enabled/disabled addresses.
	 */
	if (interrupts_enabled(regs))
		trace_hardirqs_off();

	if (user_mode(regs) && pc > TASK_SIZE)
		arm64_apply_bp_hardening();

	if (!inf->fn(addr_if_watchpoint, esr, regs)) {
		rv = 1;
	} else {
		pr_alert("Unhandled debug exception: %s (0x%08x) at 0x%016lx\n",
			 inf->name, esr, pc);

		info.si_signo = inf->sig;
		info.si_errno = 0;
		info.si_code  = inf->code;
		info.si_addr  = (void __user *)pc;
		arm64_notify_die("", regs, &info, 0);
		rv = 0;
	}

	if (interrupts_enabled(regs))
		trace_hardirqs_on();

	return rv;
}
NOKPROBE_SYMBOL(do_debug_exception);

#ifdef CONFIG_ARM64_PAN
int cpu_enable_pan(void *__unused)
{
	/*
	 * We modify PSTATE. This won't work from irq context as the PSTATE
	 * is discarded once we return from the exception.
	 */
	WARN_ON_ONCE(in_interrupt());

	config_sctlr_el1(SCTLR_EL1_SPAN, 0);
	asm(SET_PSTATE_PAN(1));
	return 0;
}
#endif /* CONFIG_ARM64_PAN */
