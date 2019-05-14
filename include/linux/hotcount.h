#ifndef _HOTCOUNT_H_
#define _HOTCOUNT_H_
extern void insert_prio_node(unsigned int new_hotcount, uid_t uid);
extern void delete_prio_node(uid_t uid);
extern void print_prio_chain(struct seq_file *m);
#endif
