#ifndef _BOARD_H_
#define _BOARD_H_

#define GPIO_INPUT	0
#define GPIO_OUTPUT	1
#include <linux/types.h>

void timing_init(void);

uint8_t get_pin(void);

void set_pin(uint8_t level);

void set_pin_dir(uint8_t dir);

void ic_udelay(volatile uint32_t ul_Loops);

#endif
