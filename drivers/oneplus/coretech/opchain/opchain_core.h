#ifndef _LINUX_OPCHAIN_CORE_H
#define _LINUX_OPCHAIN_CORE_H

#if UX_DEBUG
extern int opchain_status_show_core(char *buf, const struct kernel_param *kp);
#endif
extern void __exit opc_exit_module(void);
#endif
