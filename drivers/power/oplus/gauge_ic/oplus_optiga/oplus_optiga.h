#ifndef _OPTIGA_TYPE_H_
#define _OPTIGA_TYPE_H_
#include <linux/ctype.h>
//#include <stdio.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/types.h>
//typedef signed char BYTE;
//typedef unsigned char UBYTE;
//typedef unsigned char uint8_t;

//typedef short WORD;
//typedef unsigned short UWORD;
//typedef unsigned short uint16_t;

//typedef long LONG;
//typedef unsigned long ULONG;
//typedef unsigned long uint32_t;

typedef unsigned char BOOL;

struct optiga_test_result {
	int test_count_total;
	int test_count_now;
	int test_fail_count;
	int real_test_count_now;
	int real_test_fail_count;
};

struct optiga_hmac_status {
	int authenticate_result;
	int fail_count;
	int total_count;
	int real_fail_count;
	int real_total_count;
};

struct oplus_optiga_chip {
	struct device *dev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *optiga_active;
	int cpu_id;
	int key_id;
	int data_gpio;
	int try_count;
	spinlock_t slock;
	struct optiga_test_result test_result;
	struct optiga_hmac_status hmac_status;
	struct completion	is_complete;
	struct delayed_work auth_work;
	struct delayed_work test_work;
};
int get_optiga_pin(void);
void set_optiga_pin_dir(uint8_t dir);
void set_optiga_pin(uint8_t level);
#define MAX_DEGREE (163)
#define ARRAY_LEN(A) (((A)+31)/32)

typedef uint32_t dwordvec_t[ARRAY_LEN(MAX_DEGREE)];

#ifndef TRUE
#define TRUE  1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#endif
