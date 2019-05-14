/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : ftsTool.c
 * Description: Source file for ST fst1ba90a driver lib
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/

/*!
  * \file ftsTool.c
  * \brief Contains all the functions to support common operation inside the
  *driver
  */

#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTime.h"
#include "ftsTool.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ctype.h>

#include "../../../touchpanel_common.h"

/*******Part0:LOG TAG Declear************************/
#define TPD_PRINT_POINT_NUM 150
#define TPD_DEVICE "fts-lib"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (LEVEL_DEBUG == tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DETAIL(a, arg...)\
    do{\
        if (LEVEL_BASIC != tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_SPECIFIC_PRINT(count, a, arg...)\
    do{\
        if (count++ == TPD_PRINT_POINT_NUM || LEVEL_DEBUG == tp_debug) {\
            TPD_INFO(TPD_DEVICE ": " a, ##arg);\
            count = 0;\
        }\
    }while(0)

/**
  * Print an array of byte in a HEX string and attach at the beginning a label.
  * The function allocate memory that should be free outside the function itself
  * @param label string to attach at the beginning
  * @param buff pointer to the byte array that should be printed as HEX string
  * @param count size of buff
  * @param result pointer to the array of characters that compose the HEX final
  * string
  * @return pointer to the array of characters that compose the HEX string,
  * (same address of result)
  * @warning result MUST be allocated outside the function and should be
  * big enough to contain the data converted as HEX!
  */
char *printHex(char *label, u8 *buff, int count, u8 *result)
{
    int i, offset;

    offset = strlen(label);
    strlcpy(result, label, offset+1); /* +1 for terminator char */

    for (i = 0; i < count; i++) {
        snprintf(&result[offset], 4, "%02X ", buff[i]);
        /* this append automatically a null terminator char */
        offset += 3;
    }
    return result;
}

/**
  * Clear the FIFO from any event
  * @return OK if success or an error code which specify the type of error
  */
int flushFIFO(void)
{
    int ret;
    u8 sett = SPECIAL_FIFO_FLUSH;

    ret = writeSysCmd(SYS_CMD_SPECIAL, &sett, 1);    /* flush the FIFO */
    if (ret < OK) {
        TPD_INFO("flushFIFO: ERROR %08X\n", ret);
        return ret;
    }

    TPD_DEBUG("FIFO flushed!\n");
    return OK;
}

/**
  * Convert an array of bytes to an array of u16 taking two bytes at time,
  * src has LSB first.
  * @param src pointer to the source byte array
  * @param src_length size of src
  * @param dst pointer to the destination array.
  * @return the final size of dst (half of the source) or ERROR_OP_NOT_ALLOW
  * if the size of src is not multiple of 2.
  */
int u8ToU16n(u8 *src, int src_length, u16 *dst)
{
    int i, j;

    if (src_length % 2 != 0)
        return ERROR_OP_NOT_ALLOW;
    else {
        j = 0;
        dst = (u16 *)kzalloc((src_length / 2) * sizeof(u16), GFP_KERNEL);
        for (i = 0; i < src_length; i += 2) {
            dst[j] = ((src[i + 1] & 0x00FF) << 8) + (src[i] & 0x00FF);
            j++;
        }
    }
	kfree(dst);
    return src_length / 2;
}

/**
  * Convert an array of 2 bytes to a u16, src has LSB first (little endian).
  * @param src pointer to the source byte array
  * @param dst pointer to the destination u16.
  * @return OK
  */
int u8ToU16(u8 *src, u16 *dst)
{
    *dst = (u16)(((src[1] & 0x00FF) << 8) + (src[0] & 0x00FF));
    return OK;
}

/**
  * Convert an array of 2 bytes to a u16, src has MSB first (big endian).
  * @param src pointer to the source byte array
  * @param dst pointer to the destination u16.
  * @return OK
  */
int u8ToU16_be(u8 *src, u16 *dst)
{
    *dst = (u16)(((src[0] & 0x00FF) << 8) + (src[1] & 0x00FF));
    return OK;
}

/**
  * Convert an array of u16 to an array of u8, dst has MSB first (big endian).
  * @param src pointer to the source array of u16
  * @param src_length size of src
  * @param dst pointer to the destination array of u8. This array should be free
  * when no need anymore
  * @return size of dst (src size multiply by 2)
  */
int u16ToU8n_be(u16 *src, int src_length, u8 *dst)
{
    int i, j;

    dst = (u8 *)kzalloc((2 * src_length) * sizeof(u8), GFP_KERNEL);
    j = 0;
    for (i = 0; i < src_length; i++) {
        dst[j] = (u8)(src[i] & 0xFF00) >> 8;
        dst[j + 1] = (u8)(src[i] & 0x00FF);
        j += 2;
    }
	kfree(dst);
    return src_length * 2;
}

/**
  * Convert a u16 to an array of 2 u8, dst has MSB first (big endian).
  * @param src u16 to convert
  * @param dst pointer to the destination array of 2 u8.
  * @return OK
  */
int u16ToU8_be(u16 src, u8 *dst)
{
    dst[0] = (u8)((src & 0xFF00) >> 8);
    dst[1] = (u8)(src & 0x00FF);
    return OK;
}

/**
  * Convert a u16 to an array of 2 u8, dst has LSB first (little endian).
  * @param src u16 to convert
  * @param dst pointer to the destination array of 2 u8.
  * @return OK
  */
int u16ToU8(u16 src, u8 *dst)
{
    dst[1] = (u8)((src & 0xFF00) >> 8);
    dst[0] = (u8)(src & 0x00FF);
    return OK;
}

/**
  * Convert an array of bytes to a u32, src has LSB first (little endian).
  * @param src array of bytes to convert
  * @param dst pointer to the destination u32 variable.
  * @return OK
  */
int u8ToU32(u8 *src, u32 *dst)
{
    *dst = (u32)(((src[3] & 0xFF) << 24) + ((src[2] & 0xFF) << 16) + ((src[1] & 0xFF) << 8) + (src[0] & 0xFF));
    return OK;
}

/**
  * Convert an array of bytes to a u32, src has MSB first (big endian).
  * @param src array of bytes to convert
  * @param dst pointer to the destination u32 variable.
  * @return OK
  */
int u8ToU32_be(u8 *src, u32 *dst)
{
    *dst = (u32)(((src[0] & 0xFF) << 24) + ((src[1] & 0xFF) << 16) + ((src[2] & 0xFF) << 8) + (src[3] & 0xFF));
    return OK;
}


/**
  * Convert a u32 to an array of 4 bytes, dst has LSB first (little endian).
  * @param src u32 value to convert
  * @param dst pointer to the destination array of 4 bytes.
  * @return OK
  */
int u32ToU8(u32 src, u8 *dst)
{
    dst[3] = (u8)((src & 0xFF000000) >> 24);
    dst[2] = (u8)((src & 0x00FF0000) >> 16);
    dst[1] = (u8)((src & 0x0000FF00) >> 8);
    dst[0] = (u8)(src & 0x000000FF);
    return OK;
}

/**
  * Convert a u32 to an array of 4 bytes, dst has MSB first (big endian).
  * @param src u32 value to convert
  * @param dst pointer to the destination array of 4 bytes.
  * @return OK
  */
int u32ToU8_be(u32 src, u8 *dst)
{
    dst[0] = (u8)((src & 0xFF000000) >> 24);
    dst[1] = (u8)((src & 0x00FF0000) >> 16);
    dst[2] = (u8)((src & 0x0000FF00) >> 8);
    dst[3] = (u8)(src & 0x000000FF);
    return OK;
}

/**
  * Execute a function passed as argment and retry it defined number of times if
  *not successfull
  * @param code pointer to a function which return an int and doesn't have any
  *parameters
  * @param wait_before_retry interval of time in ms to wait between one trial
  *and another one
  * @param retry_count max number of retry to attemp
  * @return last return value obtained from the last execution of the code
  *function
  */
int attempt_function(int (*code)(void), unsigned long wait_before_retry, int
             retry_count)
{
    int result;
    int count = 0;

    do {
        result = code();
        count++;
        msleep(wait_before_retry);
    } while (count < retry_count && result < 0);


    if (count == retry_count)
        return result | ERROR_TIMEOUT;
    else
        return result;
}

/**
  * Enable all the possible sensing mode supported by the FW
  * @return OK if success or an error code which specify the type of error
  */
int senseOn(void)
{
    int ret;

    ret = setScanMode(SCAN_MODE_ACTIVE, 0xFF);    /* enable all */
    if (ret < OK) {
        TPD_INFO("senseOn: ERROR %08X\n", ret);
        return ret;
    }

    TPD_DEBUG("senseOn: SENSE ON\n");
    return OK;
}

/**
  * Disable  all the sensing mode
  * @return  OK if success or an error code which specify the type of error
  */
int senseOff(void)
{
    int ret;

    ret = setScanMode(SCAN_MODE_ACTIVE, 0x00);
    if (ret < OK) {
        TPD_INFO("senseOff: ERROR %08X\n", ret);
        return ret;
    }

    TPD_DEBUG("senseOff: SENSE OFF\n");
    return OK;
}

/**
  * Clean up the IC status executing a system reset and giving
  * the possibility to re-enabling the sensing
  * @param enableTouch if 1, re-enable the sensing and the interrupt of the IC
  * @return OK if success or an error code which specify the type of error
  */
int cleanUp(int enableTouch)
{
    int res;

    TPD_DEBUG("cleanUp: system reset...\n");
    res = fts_system_reset();
    if (res < OK)
        return res;
    if (enableTouch) {
        TPD_DEBUG("cleanUp: enabling touches...\n");
        res = senseOn();    /* already enable everything */
        if (res < OK)
            return res;
        TPD_DEBUG("cleanUp: enabling interrupts...\n");
        res = fts_enableInterrupt();
        if (res < OK)
            return res;
    }
    return OK;
}

/**
  * Transform an array of short in a matrix of short with a defined number of
  * columns and the resulting number of rows
  * @param data array of bytes to convert
  * @param size size of data
  * @param columns number of columns that the resulting matrix should have.
  * @return a reference to a matrix of short where for each row there are
  * columns elements
  * @warning If size = 0 it will be allocated a matrix 1*1 wich still should
  * be free
  */
short **array1dTo2d_short(short *data, int size, int columns)
{
    int i;
    short **matrix = NULL;
    if (size == 0) {
        matrix = (short **)kzalloc(1 * sizeof(short *), GFP_KERNEL);
        matrix[0] = (short *)kzalloc(0 * sizeof(short), GFP_KERNEL);
    } else {
        matrix = (short **)kzalloc(((int)(size / columns)) * sizeof(short *), GFP_KERNEL);

        if (matrix != NULL) {
            for (i = 0; i < (int)(size / columns); i++)
                matrix[i] = (short *)kzalloc(columns * sizeof(short), GFP_KERNEL);

            for (i = 0; i < size; i++)
                matrix[i / columns][i % columns] = data[i];
        }
    }

    return matrix;
}

/**
  * Transform an array of u16 in a matrix of u16 with a defined number of
  * columns and the resulting number of rows
  * @param data array of bytes to convert
  * @param size size of data
  * @param columns number of columns that the resulting matrix should have.
  * @return a reference to a matrix of u16 where for each row there are columns
  * elements
  * @warning If size = 0 it will be allocated a matrix 1*1 wich still should
  * be free
  */
u16 **array1dTo2d_u16(u16 *data, int size, int columns)
{
    int i;
    u16 **matrix = NULL;


    if (size == 0) {
        matrix = (u16 **)kzalloc(1 * sizeof(u16 *), GFP_KERNEL);
        matrix[0] = (u16 *)kzalloc(0 * sizeof(u16), GFP_KERNEL);
    } else {
        matrix = (u16 **)kzalloc(((int)(size / columns)) * sizeof(u16 *), GFP_KERNEL);

        if (matrix != NULL) {
            for (i = 0; i < (int)(size / columns); i++)
                matrix[i] = (u16 *)kzalloc(columns * sizeof(u16), GFP_KERNEL);

            for (i = 0; i < size; i++)
                matrix[i / columns][i % columns] = data[i];
        }
    }

    return matrix;
}

/**
  * Transform an array of u8 in a matrix of u8 with a defined number of
  * columns and the resulting number of rows
  * @param data array of bytes to convert
  * @param size size of data
  * @param columns number of columns that the resulting matrix should have.
  * @return a reference to a matrix of short where for each row there are
  * columns elements
  * @warning If size = 0 it will be allocated a matrix 1*1 wich still should
  * be free
  */
u8 **array1dTo2d_u8(u8 *data, int size, int columns)
{
    int i;
    u8 **matrix = NULL;


    if (size == 0) {
        matrix = (u8 **)kzalloc(1 * sizeof(u8 *), GFP_KERNEL);
        matrix[0] = (u8 *)kzalloc(0 * sizeof(u8), GFP_KERNEL);
    } else {
        matrix = (u8 **)kzalloc(((int)(size / columns)) * sizeof(u8 *), GFP_KERNEL);
        if (matrix != NULL) {
            for (i = 0; i < (int)(size / columns); i++)
                matrix[i] = (u8 *)kzalloc(columns * sizeof(u8), GFP_KERNEL);

            for (i = 0; i < size; i++)
                matrix[i / columns][i % columns] = data[i];
        }
    }

    return matrix;
}

/**
  * Transform an array of i8 in a matrix of i8 with a defined number of
  * columns and the resulting number of rows
  * @param data array of bytes to convert
  * @param size size of data
  * @param columns number of columns that the resulting matrix should have.
  * @return a reference to a matrix of short where for each row there are
  * columns elements
  * @warning If size = 0 it will be allocated a matrix 1*1 wich still should
  * be free
  */
i8 **array1dTo2d_i8(i8 *data, int size, int columns)
{
    int i;
    i8 **matrix = NULL;

    if (size == 0) {
        matrix = (i8 **)kzalloc(1 * sizeof(i8 *), GFP_KERNEL);
        matrix[0] = (i8 *)kzalloc(0 * sizeof(i8), GFP_KERNEL);
    } else {
        matrix = (i8 **)kzalloc(((int)(size / columns)) * sizeof(i8 *), GFP_KERNEL);

        if (matrix != NULL) {
            for (i = 0; i < (int)(size / columns); i++)
                matrix[i] = (i8 *)kzalloc(columns * sizeof(i8), GFP_KERNEL);

            for (i = 0; i < size; i++)
                matrix[i / columns][i % columns] = data[i];
        }
    }

    return matrix;
}

/**
  * Print in the kernel log a label followed by a matrix of short row x columns
  * and free its memory
  * @param label pointer to the string to print before the actual matrix
  * @param matrix reference to the matrix of short which contain the actual data
  * @param row number of rows on which the matrix should be print
  * @param column number of columns for each row
  */
void print_frame_short(char *label, short **matrix, int row, int column)
{
    int i, j;

    TPD_DEBUG("%s\n", label);
    for (i = 0; i < row; i++) {
        TPD_DEBUG("");
        for (j = 0; j < column; j++)
            printk("%d ", matrix[i][j]);
        TPD_DEBUG( "\n");
        kfree(matrix[i]);
    }
    kfree(matrix);
}

/**
  * Print in the kernel log a label followed by a matrix of u16 row x columns
  * and free its memory
  * @param label pointer to the string to print before the actual matrix
  * @param matrix reference to the matrix of u16 which contain the actual data
  * @param row number of rows on which the matrix should be print
  * @param column number of columns for each row
  */
void print_frame_u16(char *label, u16 **matrix, int row, int column)
{
    int i, j;

    TPD_DEBUG("%s\n", label);
    for (i = 0; i < row; i++) {
        TPD_DEBUG("");
        for (j = 0; j < column; j++)
            printk("%d ", matrix[i][j]);
        TPD_DEBUG( "\n");
        kfree(matrix[i]);
    }
    kfree(matrix);
}

/**
  * Print in the kernel log a label followed by a matrix of u8 row x columns and
  * free its memory
  * @param label pointer to the string to print before the actual matrix
  * @param matrix reference to the matrix of u8 which contain the actual data
  * @param row number of rows on which the matrix should be print
  * @param column number of columns for each row
  */
void print_frame_u8(char *label, u8 **matrix, int row, int column)
{
    int i, j;

    TPD_DEBUG("%s\n", label);
    for (i = 0; i < row; i++) {
        TPD_DEBUG("");
        for (j = 0; j < column; j++)
            printk("%d ", matrix[i][j]);
        TPD_DEBUG( "\n");
        kfree(matrix[i]);
    }
    kfree(matrix);
}

/**
  * Print in the kernel log a label followed by a matrix of i8 row x columns and
  * free its memory
  * @param label pointer to the string to print before the actual matrix
  * @param matrix reference to the matrix of u8 which contain the actual data
  * @param row number of rows on which the matrix should be print
  * @param column number of columns for each row
  */
void print_frame_i8(char *label, i8 **matrix, int row, int column)
{
    int i, j;

    TPD_DEBUG("%s\n", label);
    for (i = 0; i < row; i++) {
        TPD_DEBUG("");
        for (j = 0; j < column; j++)
            printk("%d ", matrix[i][j]);
        TPD_DEBUG( "\n");
        kfree(matrix[i]);
    }
    kfree(matrix);
}

/**
  * Print in the kernel log a label followed by a matrix of u32 row x columns
  * and free its memory
  * @param label pointer to the string to print before the actual matrix
  * @param matrix reference to the matrix of u32 which contain the actual data
  * @param row number of rows on which the matrix should be print
  * @param column number of columns for each row
  */
void print_frame_u32(char *label, u32 **matrix, int row, int column)
{
    int i, j;

    TPD_DEBUG("%s\n", label);
    for (i = 0; i < row; i++) {
        TPD_DEBUG("");
        for (j = 0; j < column; j++)
            printk("%d ", matrix[i][j]);
        TPD_DEBUG( "\n");
        kfree(matrix[i]);
    }
    kfree(matrix);
}

/**
  * Print in the kernel log a label followed by a matrix of int row x columns
  * and free its memory
  * @param label pointer to the string to print before the actual matrix
  * @param matrix reference to the matrix of int which contain the actual data
  * @param row number of rows on which the matrix should be print
  * @param column number of columns for each row
  */
void print_frame_int(char *label, int **matrix, int row, int column)
{
    int i, j;

    TPD_DEBUG("%s\n", label);
    for (i = 0; i < row; i++) {
        TPD_DEBUG("");
        for (j = 0; j < column; j++)
            printk("%d ", matrix[i][j]);
        TPD_DEBUG( "\n");
        kfree(matrix[i]);
    }
    kfree(matrix);
}

/**
  * Convert an array of bytes to an u64, src has MSB first (big endian).
  * @param src array of bytes
  * @param dest pointer to the destination u64.
  * @param size size of src (can be <= 8)
  * @return OK if success or ERROR_OP_NOT_ALLOW if size exceed 8
  */
int u8ToU64_be(u8 *src, u64 *dest, int size)
{
    int i = 0;

    /* u64 temp =0; */
    if (size > sizeof(u64))
        return ERROR_OP_NOT_ALLOW;
    else {
        *dest = 0;

        for (i = 0; i < size; i++)
            *dest |= (u64)(src[i]) << ((size - 1 - i) * 8);

        return OK;
    }
}

/**
  * Convert an u64 to an array of bytes, dest has MSB first (big endian).
  * @param src value of u64
  * @param dest pointer to the destination array of bytes.
  * @param size size of src (can be <= 8)
  * @return OK if success or ERROR_OP_NOT_ALLOW if size exceed 8
  */
int u64ToU8_be(u64 src, u8 *dest, int size)
{
    int i = 0;

    if (size > sizeof(u64))
        return ERROR_OP_NOT_ALLOW;
    else
        for (i = 0; i < size; i++)
            dest[i] = (u8)((src >> ((size - 1 - i) * 8)) & 0xFF);

    return OK;
}

/**
  * Convert a value of an id in a bitmask with a 1 in the position of the value
  * of the id
  * @param id Value of the ID to convert
  * @param mask pointer to the bitmask that will be updated with the value of id
  * @param size dimension in bytes of mask
  * @return OK if success or ERROR_OP_NOT_ALLOW if size of mask is not enough to
  * contain ID
  */
int fromIDtoMask(u8 id, u8 *mask, int size)
{
    if (((int)((id) / 8)) < size) {
        TPD_DEBUG("%s: ID = %d Index = %d Position = %d !\n", __func__, id, ((int)((id) / 8)), (id % 8));
        mask[((int)((id) / 8))] |= 0x01 << (id % 8);
        return OK;
    } else {
        TPD_INFO("%s: Bitmask too small! Impossible contain ID = %d %d>=%d! ERROR %08X\n",
             __func__, id, ((int)((id) / 8)), size, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }
}
