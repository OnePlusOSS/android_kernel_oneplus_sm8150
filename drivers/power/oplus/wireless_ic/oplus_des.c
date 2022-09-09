/************************************************************************************
** File:  oplus_des.c
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2019-2024, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      for wireless charge
** 
** Version: 1.0
** Date created: 21:03:46,04/11/2019
** Author: Lin Shangbo
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2019-11-04    Lin Shangbo   		Created for wireless charge
************************************************************************************************************/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>

#include <linux/random.h>

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
	
#include <linux/alarmtimer.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/iio/consumer.h>
#include <uapi/linux/qg.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"

#include <soc/oplus/device_info.h>

#include "oplus_des.h"

#define ENCODE 0,16,1       
#define DECODE 15,-1,-1  

#define KEY1 {0x21, 0x43, 0x88, 0x2A, 0xF3, 0xE1, 0x77, 0xCC}
#define KEY2 {0x24, 0xA2, 0x76, 0x08, 0x10, 0xB3, 0x48, 0x00}
  
char msg_ch[64] = {  
	58, 50, 42, 34, 26, 18, 10, 2, 60, 52, 44, 36, 28, 20, 12, 4,  
	62, 54, 46, 38, 30, 22, 14, 6, 64, 56, 48, 40, 32, 24, 16, 8,  
	57, 49, 41, 33, 25, 17,  9, 1, 59, 51, 43, 35, 27, 19, 11, 3,  
	61, 53, 45, 37, 29, 21, 13, 5, 63, 55, 47, 39, 31, 23, 15, 7  
};  
   
char key_ch[56] = {  
	57, 49, 41, 33, 25, 17,  9,  1, 58, 50, 42, 34, 26, 18,  
	10,  2, 59, 51, 43, 35, 27, 19, 11,  3, 60, 52, 44, 36,  
	63, 55, 47, 39, 31, 23, 15,  7, 62, 54, 46, 38, 30, 22,  
	14,  6, 61, 53, 45, 37, 29, 21, 13,  5, 28, 20, 12,  4  
};  
    
char msg_ex[48] = {  
	32,  1,  2,  3,  4,  5,  4,  5,  6,  7,  8,  9,  
	 8,  9, 10, 11, 12, 13, 12, 13, 14, 15, 16, 17,  
	16, 17, 18, 19, 20, 21, 20, 21, 22, 23, 24, 25,  
	24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32,  1  
};  
   
char key_mov[16] = {  
	1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1  
};  
  
char key_cmprs[48] = {  
	14, 17, 11, 24,  1,  5,  3, 28, 15,  6, 21, 10,  
	23, 19, 12,  4, 26,  8, 16,  7, 27, 20, 13,  2,  
	41, 52, 31, 37, 47, 55, 30, 40, 51, 45, 33, 48,  
	44, 49, 39, 56, 34, 53, 46, 42, 50, 36, 29, 32  
};  
   
char s_box[8][4][16] = {    
	// S1  
	{{14,  4, 13,  1,  2, 15, 11,  8,  3, 10,  6, 12,  5,  9,  0,  7},  
	{ 0, 15,  7,  4, 14,  2, 13,  1, 10,  6, 12, 11,  9,  5,  3,  8},  
	{ 4,  1, 14,  8, 13,  6,  2, 11, 15, 12,  9,  7,  3, 10,  5,  0},  
	{15, 12,  8,  2,  4,  9,  1,  7,  5, 11,  3, 14, 10,  0,  6, 13}},  
	// S2  
	{{15,  1,  8, 14,  6, 11,  3,  4,  9,  7,  2, 13, 12,  0,  5, 10},  
	{ 3, 13,  4,  7, 15,  2,  8, 14, 12,  0,  1, 10,  6,  9, 11,  5},  
	{ 0, 14,  7, 11, 10,  4, 13,  1,  5,  8, 12,  6,  9,  3,  2, 15},  
	{13,  8, 10,  1,  3, 15,  4,  2, 11,  6,  7, 12,  0,  5, 14,  9}},  
	// S3  
	{{10,  0,  9, 14,  6,  3, 15,  5,  1, 13, 12,  7, 11,  4,  2,  8},  
	{13,  7,  0,  9,  3,  4,  6, 10,  2,  8,  5, 14, 12, 11, 15,  1},  
	{13,  6,  4,  9,  8, 15,  3,  0, 11,  1,  2, 12,  5, 10, 14,  7},  
	{ 1, 10, 13,  0,  6,  9,  8,  7,  4, 15, 14,  3, 11,  5,  2, 12}},  
	// S4  
	{{ 7, 13, 14,  3,  0,  6,  9, 10,  1,  2,  8,  5, 11, 12,  4, 15},  
	{13,  8, 11,  5,  6, 15,  0,  3,  4,  7,  2, 12,  1, 10, 14,  9},  
	{10,  6,  9,  0, 12, 11,  7, 13, 15,  1,  3, 14,  5,  2,  8,  4},  
	{ 3, 15,  0,  6, 10,  1, 13,  8,  9,  4,  5, 11, 12,  7,  2, 14}},  
	// S5  
	{{ 2, 12,  4,  1,  7, 10, 11,  6,  8,  5,  3, 15, 13,  0, 14,  9},  
	{14, 11,  2, 12,  4,  7, 13,  1,  5,  0, 15, 10,  3,  9,  8,  6},  
	{ 4,  2,  1, 11, 10, 13,  7,  8, 15,  9, 12,  5,  6,  3,  0, 14},  
	{11,  8, 12,  7,  1, 14,  2, 13,  6, 15,  0,  9, 10,  4,  5,  3}},  
	// S6  
	{{12,  1, 10, 15,  9,  2,  6,  8,  0, 13,  3,  4, 14,  7,  5, 11},  
	{10, 15,  4,  2,  7, 12,  9,  5,  6,  1, 13, 14,  0, 11,  3,  8},  
	{ 9, 14, 15,  5,  2,  8, 12,  3,  7,  0,  4, 10,  1, 13, 11,  6},  
	{ 4,  3,  2, 12,  9,  5, 15, 10, 11, 14,  1,  7,  6,  0,  8, 13}},  
	// S7  
	{{ 4, 11,  2, 14, 15,  0,  8, 13,  3, 12,  9,  7,  5, 10,  6,  1},  
	{13,  0, 11,  7,  4,  9,  1, 10, 14,  3,  5, 12,  2, 15,  8,  6},  
	{ 1,  4, 11, 13, 12,  3,  7, 14, 10, 15,  6,  8,  0,  5,  9,  2},  
 	{ 6, 11, 13,  8,  1,  4, 10,  7,  9,  5,  0, 15, 14,  2,  3, 12}},  
	// S8  
	{{13,  2,  8,  4,  6, 15, 11,  1, 10,  9,  3, 14,  5,  0, 12,  7},  
	{ 1, 15, 13,  8, 10,  3,  7,  4, 12,  5,  6, 11,  0, 14,  9,  2},  
	{ 7, 11,  4,  1,  9, 12, 14,  2,  0,  6, 10, 13, 15,  3,  5,  8},  
	{ 2,  1, 14,  7,  4, 10,  8, 13, 15, 12,  9,  0,  3,  5,  6, 11}}  
};  
   
char p_box[32] = {  
	16,  7, 20, 21, 29, 12, 28, 17,  1, 15, 23, 26,  5, 18, 31, 10,  
	 2,  8, 24, 14, 32, 27,  3,  9, 19, 13, 30,  6, 22, 11,  4, 25  
};  
   
char last_ch[64] = {  
	40, 8, 48, 16, 56, 24, 64, 32, 39, 7, 47, 15, 55, 23, 63, 31,  
	38, 6, 46, 14, 54, 22, 62, 30, 37, 5, 45, 13, 53, 21, 61, 29,  
	36, 4, 44, 12, 52, 20, 60, 28, 35, 3, 43, 11, 51, 19, 59, 27,  
	34, 2, 42, 10, 50, 18, 58, 26, 33, 1, 41,  9, 49, 17, 57, 25  
};  
  
char msgb[72], msgbt[72], keyb[18][72];  

void char_to_bit(char* dest, char* src, int length) 
{  
	int i, j;
	char t;
    
	for (i = 0; i < length; i++) { 
        for (j = 8, t = src[i]; j > 0; j--) {  
            dest[(i << 3) + j] = t & 1;    
            t >>= 1;  
        } 
    }
}  
  
void bit_to_char(char* dest, char* src, int length) 
{  
	int i;
    
	for (i = 0; i < length << 3; i++) {  
		dest[i >> 3] <<= 1;  
		dest[i >> 3] |= src[i + 1]; 
	}	  
}  
   
void batch_set(char* dest, char* src, char* offset, int count) 
{  
	int i;
	
	for (i = 0; i < count; i++) {
        dest[i + 1] = src[(unsigned char)offset[i]];  
	}
}  
  
void generate_key_arrays(char* key) 
{  
	char tk[128], bk[65];  
	char* ptk = tk;  
	int i, j;
		  
    char_to_bit(bk, key, 8);  
    batch_set(tk, bk, key_ch, 56); 
	 
    for (i = 0; i < 16; i++) {  
        for (j = 0; j < key_mov[i]; j++, ptk++) {  
            ptk[57] = ptk[28];  
            ptk[28] = ptk[1];  
        }
		  
        batch_set(keyb[i], ptk, key_cmprs, 48);  
    }  
}  
  
void DES_function(char* tmsg, char* pmsg, int st, int cl, int step) 
{  
	int i, row, col;  
	char r[64], rt[48], s[8];  
    
	char_to_bit(msgbt, pmsg, 8);  
	batch_set(msgb, msgbt, msg_ch, 64); 
	 
	for (; st != cl; st += step) {  
		memcpy(rt, msgb + 33, 32);  
		batch_set(r, msgb + 32, msg_ex, 48);  
        
		for (i = 1; i <= 48; i++) {
            r[i] ^= keyb[st][i]; 
		}
		
		for (i = 0; i < 48; i += 6) {  
			row = col = 0;  
			row = r[i + 1] << 1 | r[i + 6];  
			col = (r[i + 2] << 3) | (r[i + 3] << 2) | (r[i + 4] << 1) | r[i + 5];  
			s[i / 12] = (s[i / 12] <<= 4) | s_box[i / 6][row][col];  
		}
		  
		char_to_bit(r, s, 4);  
		batch_set(msgb + 32, r, p_box, 32);  
        
		for (i = 1; i <= 32; i++) {
			msgb[i + 32] ^= msgb[i]; 
		}
		
		memcpy(msgb + 1, rt, 32);  
	}
	  
	memcpy(msgbt + 33, msgb + 1, 32);  
	memcpy(msgbt + 1, msgb + 33, 32);  
	batch_set(msgb, msgbt, last_ch, 64); 
 
	bit_to_char(tmsg, msgb, 8);
}  

void get_verify_numbers(char random_num[8], char encode_num[8], char* encode_mark)
{
    char key1[8] = KEY1; 
    char key2[8] = KEY2; 
    char en_msg[8];

	get_random_bytes(random_num, 8);
	*encode_mark = 3;
	
	generate_key_arrays(key1); 
	DES_function(en_msg, random_num, ENCODE); 
	
	generate_key_arrays(key2); 
	DES_function(encode_num, en_msg, ENCODE); 
}

