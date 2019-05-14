#ifndef _LINUX_OPCHAIN_BINDER_H
#define _LINUX_OPCHAIN_BINDER_H

extern void opc_binder_pass(size_t data_size, uint32_t *data, int send);
extern void opc_binder_pass_cb(void (*cb)(size_t dsize, uint32_t *data, int send));
#endif
