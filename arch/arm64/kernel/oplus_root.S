/* SPDX-License-Identifier: GPL-2.0-only */
/**************************************************************
* Copyright (c)  2008- 2030  OPLUS Mobile communication Corp.ltd, All rights reserved.
* File       : oplus_root.S
* Description: OPLUS Block root code,block root action and record. This program is
* distributed in the hope that it will block process to elevate privilege of root.
 * 
* Version    : 1.0
* Date       : 
* Author     : 
* TAG        : 
****************************************************************/

#include <linux/init.h>
#include <linux/linkage.h>
#include <generated/asm-offsets.h>

#include <asm/assembler.h>
#include <asm/errno.h>
#include <asm/esr.h>
#include <asm/thread_info.h>
#include <asm/unistd.h>
#include "rootguard/oplus_root.h"
/*
 X26 : syscall number
 x27 : syscall table pointer
 x28 : current thread_info
 X25 : number of system calls
 X19,X20,X29: store uid\gid\euid\egid\fsuid\fsgid
 */

.extern oplus_root_reboot

#ifdef CONFIG_OPLUS_KEVENT_UPLOAD
.extern oplus_root_check_succ
.extern is_unlocked
#endif

ENTRY(oplus_root_check)
	mov	x21, x30
#ifdef CONFIG_THREAD_INFO_IN_TASK
	mrs	x16, sp_el0
#else
	ldr	x16, [x28, #PROOT_THREAD_TSK]
#endif
	ldr	x15, [x16, #PROOT_TSK_CRED]
	ldr	x19, [x15, #PROOT_CRED_UID]
	ldr	x20, [x15, #PROOT_CRED_EUID]
	ldr	x29, [x15, #PROOT_CRED_FSUID]

	ldr	x16, [x27, x26, lsl #3]				// address in the syscall table
	blr	x16

	mov x22, x0

	cmp	x19, #0						//if phone adb have root permission,we donnot check
	beq	2f

	bl	is_unlocked
	cmp	x0, #0
	bne	2f

#ifdef CONFIG_THREAD_INFO_IN_TASK
	mrs	x16, sp_el0
	ldr	x15, [x16, #PROOT_THREAD_ADDR_LIMIT]
#else
	ldr	x15, [x28, #PROOT_THREAD_ADDR_LIMIT]
#endif
	mov	x17, #0x0000008000000000
	cmp	x15, x17
	b.hi	5f

#ifdef CONFIG_THREAD_INFO_IN_TASK
	mrs	x15, sp_el0					//if uid not change,we return
#else
	ldr	x15, [x28, #PROOT_THREAD_TSK]			//if uid not change,we return
#endif
	ldr	x17, [x15, #PROOT_TSK_CRED]
	ldr	x16, [x17, #PROOT_CRED_UID]
	cmp	x16, x19
	bne	3f
	ldr	x16, [x17, #PROOT_CRED_EUID]			//if euid not change,we return
	cmp	x16, x20
	bne	3f
	ldr	x16, [x17, #PROOT_CRED_FSUID]			//if fsuid not change,we return
	cmp	x16, x29
	bne	3f

2:
	mov	x30, x21
	mov x0, x22
	ret

3:
/*
//In Android R K 4.14, there both have 32 and 64 bit syscalls
#ifdef CONFIG_COMPAT
	cmp	X25, #__NR_compat_syscalls
	beq	4f
#endif
*/
	cmp	x26, #__NR_setresgid
	beq	2b
	sub	x15, x26, #__NR_setregid
	cmp	x15, #0x4
	b.ls	2b
//	b	5f

#ifdef CONFIG_COMPAT
4:
	cmp	x26, #__NR_setreuid32
	beq	2b
	cmp	x26, #__NR_setregid32
	beq	2b
	cmp	x26, #__NR_setresuid32
	beq	2b
	cmp	x26, #__NR_setresgid32
	beq	2b
	cmp	x26, #__NR_setuid32
	beq	2b
	cmp	x26, #__NR_setgid32
	beq	2b
#endif

5:
#ifdef CONFIG_OPLUS_KEVENT_UPLOAD
	mov	x0, x19
	mov	x1, x20
	mov	x2, x29
	mov	x3, x26
	bl	oplus_root_check_succ
#endif

	mov	x0, #0
	bl	oplus_root_reboot

END(oplus_root_check)
