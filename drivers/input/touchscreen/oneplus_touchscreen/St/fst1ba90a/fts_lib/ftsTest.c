/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : ftsTest.c
 * Description: Source file for ST fst1ba90a driver lib
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/

/*!
  * \file ftsTest.c
  * \brief Contains all the functions related to the Mass Production Test
  */

#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsFrame.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/firmware.h>
#include "../fts.h"
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

TestToDo tests;    /* /< global variable that specify the tests to perform during
                    * the Mass Production Test */
static LimitFile limit_file;    /* /< variable which contains the limit file
                                 * during test */
//extern ErrorList errors;

/**
  * Initialize the testToDo variable with the default tests to perform during
  * the Mass Production Test
  * @return OK
  */
int initTestToDo(void)
{
    /*** Initialize Limit File ***/
    limit_file.size = 0;
    limit_file.data = NULL;
    strlcpy(limit_file.name, " ", MAX_LIMIT_FILE_NAME);


	tests.MutualRawAdjITO = 1;

	tests.MutualRaw = 1;  /* in case of YOCTA please use Map */
    tests.MutualRawMap = 0;
    tests.MutualRawGap = 0;
    tests.MutualRawAdj = 0;
	tests.MutualRawAdjGap = 0;
    tests.MutualRawAdjPeak = 0;

    tests.MutualRawLP = 0; /* in case of YOCTA please use Map */
    tests.MutualRawGapLP = 0;
    tests.MutualRawMapLP = 0;
    tests.MutualRawAdjLP = 0;

    tests.MutualCx1 = 0;
    tests.MutualCx2 = 0;
    tests.MutualCx2Adj = 0;
	tests.MutualCxTotal = 0;
	tests.MutualCxTotalAdj = 0;

    tests.MutualCx1LP = 0;
	tests.MutualCx2LP = 0;
	tests.MutualCx2AdjLP = 0;
	tests.MutualCxTotalLP = 1;
    tests.MutualCxTotalAdjLP = 0;

#ifdef PHONE_KEY
	tests.MutualKeyRaw = 1;
#else
    tests.MutualKeyRaw = 0;
#endif
    tests.MutualKeyCx1 = 0;
    tests.MutualKeyCx2 = 0;
#ifdef PHONE_KEY
	tests.MutualKeyCxTotal = 1;
#else
    tests.MutualKeyCxTotal = 0;
#endif

	tests.SelfForceRaw = 1; /* in case of YOCTA please use Map */
    tests.SelfForceRawGap = 0;
    tests.SelfForceRawMap = 0;

	tests.SelfForceRawLP = 1; /* in case of YOCTA please use Map */
    tests.SelfForceRawGapLP = 0;
    tests.SelfForceRawMapLP = 0;

    tests.SelfForceIx1 = 0;
    tests.SelfForceIx2 = 0;
    tests.SelfForceIx2Adj = 0;
	tests.SelfForceIxTotal = 1;
    tests.SelfForceIxTotalAdj = 0;
    tests.SelfForceCx1 = 0;
    tests.SelfForceCx2 = 0;
    tests.SelfForceCx2Adj = 0;
    tests.SelfForceCxTotal = 0;
    tests.SelfForceCxTotalAdj = 0;
    tests.SelfForceIx1LP = 0;
    tests.SelfForceIx2LP = 0;
    tests.SelfForceIx2AdjLP = 0;
    tests.SelfForceIxTotalLP = 0;
    tests.SelfForceIxTotalAdjLP = 0;
    tests.SelfForceCx1LP = 0;
    tests.SelfForceCx2LP = 0;
    tests.SelfForceCx2AdjLP = 0;
    tests.SelfForceCxTotalLP = 0;
    tests.SelfForceCxTotalAdjLP = 0;

	tests.SelfSenseRaw = 1;
    tests.SelfSenseRawGap = 0;
    tests.SelfSenseRawMap = 0;

	tests.SelfSenseRawLP = 1;
    tests.SelfSenseRawGapLP = 0;
    tests.SelfSenseRawMapLP = 0;

    tests.SelfSenseIx1 = 0;
    tests.SelfSenseIx2 = 0;
    tests.SelfSenseIx2Adj = 0;
	tests.SelfSenseIxTotal = 1;
    tests.SelfSenseIxTotalAdj = 0;
    tests.SelfSenseCx1 = 0;
    tests.SelfSenseCx2 = 0;
    tests.SelfSenseCx2Adj = 0;
    tests.SelfSenseCxTotal = 0;
    tests.SelfSenseCxTotalAdj = 0;
    tests.SelfSenseIx1LP = 0;
    tests.SelfSenseIx2LP = 0;
    tests.SelfSenseIx2AdjLP = 0;
    tests.SelfSenseIxTotalLP = 0;
    tests.SelfSenseIxTotalAdjLP = 0;
    tests.SelfSenseCx1LP = 0;
    tests.SelfSenseCx2LP = 0;
    tests.SelfSenseCx2AdjLP = 0;
    tests.SelfSenseCxTotalLP = 0;
    tests.SelfSenseCxTotalAdjLP = 0;

    return OK;
}

/**
  * Compute the Horizontal adjacent matrix doing the abs of the difference
  * between the column i with the i-1 one. \n
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHoriz(i8 *data, int row, int column, u8 **result)
{
    int i, j;
    int size = row * (column - 1);

    if (column < 2) {
        TPD_INFO("computeAdjHoriz: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u8 *)kzalloc(size * sizeof(u8), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjHoriz: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 0; i < row; i++)
        for (j = 1; j < column; j++)
            *(*result + (i * (column - 1) + (j - 1))) = abs(data[i * column + j] - data[i * column + (j - 1)]);

    return OK;
}

/**
  * Compute the Horizontal adjacent matrix of short values doing the abs of
  * the difference between the column i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  *  array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which
  * will contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizTotal(short *data, int row, int column, u16 **result)
{
    int i, j;
    int size = row * (column - 1);

    if (column < 2) {
        TPD_INFO("computeAdjHorizTotal: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u16 *)kzalloc(size * sizeof(u16), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjHorizTotal: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 0; i < row; i++)
        for (j = 1; j < column; j++)
            *(*result + (i * (column - 1) + (j - 1))) = abs(data[i * column + j] - data[i * column + (j - 1)]);

    return OK;
}

/**
  * Compute the Vertical adjacent matrix doing the abs of the difference between
  * the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVert(i8 *data, int row, int column, u8 **result)
{
    int i, j;
    int size = (row - 1) * (column);

    if (row < 2) {
        TPD_INFO("computeAdjVert: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u8 *)kzalloc(size * sizeof(u8), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjVert: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 1; i < row; i++)
        for (j = 0; j < column; j++)
            *(*result + ((i - 1) * column + j)) = abs(data[i * column + j] - data[(i - 1) * column + j]);

    return OK;
}

/**
  * Compute the Vertical adjacent matrix of short values doing the abs of
  * the difference between the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of signed bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertTotal(short *data, int row, int column, u16 **result)
{
    int i, j;
    int size = (row - 1) * (column);

    if (row < 2) {
        TPD_INFO("computeAdjVertTotal: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u16 *)kzalloc(size * sizeof(u16), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjVertTotal: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 1; i < row; i++)
        for (j = 0; j < column; j++) *(*result + ((i - 1) * column + j)) = abs(data[i * column + j] - data[(i - 1) * column + j]);

    return OK;
}

/**
  * Compute the Horizontal adjacent matrix doing the abs of the difference
  * between the column i with the i-1 one. \n
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  *  contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizFromU(u8 *data, int row, int column, u8 **result)
{
    int i, j;
    int size = row * (column - 1);

    if (column < 2) {
        TPD_INFO("computeAdjHoriz: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u8 *)kzalloc(size * sizeof(u8), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjHoriz: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 0; i < row; i++)
        for (j = 1; j < column; j++)
            *(*result + (i * (column - 1) + (j - 1))) = abs(data[i * column + j] - data[i * column + (j - 1)]);

    return OK;
}

/**
  * Compute the Horizontal adjacent matrix of u16 values doing the abs of
  * the difference between the column i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  *  array one row after the other \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjHorizTotalFromU(u16 *data, int row, int column, u16 **result)
{
    int i, j;
    int size = row * (column - 1);

    if (column < 2) {
        TPD_INFO("computeAdjHorizTotal: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u16 *)kzalloc(size * sizeof(u16), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjHorizTotal: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 0; i < row; i++)
        for (j = 1; j < column; j++)
            *(*result + (i * (column - 1) + (j - 1))) = abs(data[i * column + j] - data[i * column + (j - 1)]);

    return OK;
}

/**
  * Compute the Vertical adjacent matrix doing the abs of the difference between
  *  the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  *  contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertFromU(u8 *data, int row, int column, u8 **result)
{
    int i, j;
    int size = (row - 1) * (column);

    if (row < 2) {
        TPD_INFO("computeAdjVert: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u8 *)kzalloc(size * sizeof(u8), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjVert: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 1; i < row; i++)
        for (j = 0; j < column; j++)
            *(*result + ((i - 1) * column + j)) = abs(data[i * column + j] - data[(i - 1) * column + j]);

    return OK;
}

/**
  * Compute the Vertical adjacent matrix of u16 values doing the abs of
  * the difference between the row i with the i-1 one.
  * Both the original data matrix and the adj matrix are disposed as 1 dimension
  * array one row after the other. \n
  * The resulting matrix has one column less than the starting original one \n
  * @param data pointer to the array of unsigned bytes containing the original
  * data
  * @param row number of rows of the original data
  * @param column number of columns of the original data
  * @param result pointer of a pointer to an array of unsigned bytes which will
  * contain the adj matrix
  * @return OK if success or an error code which specify the type of error
  */
int computeAdjVertTotalFromU(u16 *data, int row, int column, u16 **result)
{
    int i, j;
    int size = (row - 1) * (column);

    if (row < 2) {
        TPD_INFO("computeAdjVertTotal: ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    *result = (u16 *)kzalloc(size * sizeof(u16), GFP_KERNEL);
    if (*result == NULL) {
        TPD_INFO("computeAdjVertTotal: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    for (i = 1; i < row; i++)
        for (j = 0; j < column; j++)
            *(*result + ((i - 1) * column + j)) = abs(data[i * column + j] - data[(i - 1) * column + j]);

    return OK;
}



/**
  * Check that each value of a matrix of short doesn't exceed a min and a Max
  * value (these values are included in the interval). \n
  * The matrix is stored as 1 dimension array one row after the other. \n
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min minimum value allowed
  * @param max Maximum value allowed
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMinMax(short *data, int row, int column, int min, int max)
{
    int i, j;
    int count = 0;

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] < min || data[i * column + j] > max) {
                TPD_INFO("checkLimitsMinMax: Node[%d,%d] = %d exceed limit [%d, %d]\n", i, j, data[i * column + j], min, max);
                count++;
            }
        }
    }

    return count;    /* if count is 0 = OK, test completed successfully */
}

/**
  * Check that the difference between the max and min of a matrix of short is
  * less or equal to a threshold.\n
  * The matrix is stored as 1 dimension array one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param threshold threshold value allowed
  * @return OK if the difference is <= to threshold otherwise
  * ERROR_TEST_CHECK_FAIL
  */
int checkLimitsGap(short *data, int row, int column, int threshold)
{
    int i, j;
    int min_node;
    int max_node;

    if (row == 0 || column == 0) {
        TPD_INFO("checkLimitsGap: invalid number of rows = %d or columns = %d  ERROR %08X\n",row, column, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    min_node = data[0];
    max_node = data[0];

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] < min_node)
                min_node = data[i * column + j];
            else if (data[i * column + j] > max_node)
                max_node = data[i * column + j];
        }
    }

    if (max_node - min_node > threshold) {
        TPD_INFO("checkLimitsGap: GAP = %d exceed limit  %d\n", max_node - min_node, threshold);
        return ERROR_TEST_CHECK_FAIL;
    } else
        return OK;
}

/**
  * Check that the difference between the max and min of a matrix of short is
  * less or equal to a threshold.\n
  * The matrix is stored as 1 dimension array one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param threshold threshold value allowed
  * @param row_start index of the starting column which should be considered
  * @param column_start index of the starting column which should be considered
  * @param row_end number of index to subtract to row to identify last
  *        valid row to check
  * @param column_end number of index to subtract to column to identify last
  *        valid column to check
  * @return OK if the difference is <= to threshold otherwise
  * ERROR_TEST_CHECK_FAIL
  */
int checkLimitsGapOffsets(short *data, int row, int column, int threshold,
    int row_start, int column_start, int row_end, int column_end)
{
    int i, j;
    int min_node;
    int max_node;

    if (row == 0 || column == 0) {
        TPD_INFO("checkLimitsGap: invalid number of rows = %d or columns = %d  ERROR %08X\n", row, column, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    min_node = data[row_start*column+column_start];
    max_node = data[row_start*column+column_start];

    for (i = row_start; i < row-row_end; i++) {
        for (j = column_start; j < column-column_end; j++) {
            if (data[i * column + j] < min_node)
                min_node = data[i * column + j];
            else if (data[i * column + j] > max_node)
                max_node = data[i * column + j];
        }
    }

    if (max_node - min_node > threshold) {
        TPD_INFO("checkLimitsGap: GAP = %d exceed limit  %d\n", max_node - min_node, threshold);
        return ERROR_TEST_CHECK_FAIL;
    } else
        return OK;
}

/**
  * Check that each value of a matrix of i8 doesn't exceed a specific min and
  *Max value  set for each node (these values are included in the interval). \n
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  *one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  *each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  *each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMap(i8 *data, int row, int column, int *min, int *max)
{
    int i, j;
    int count = 0;

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] < min[i * column + j] || data[i * column + j] > max[i * column + j]) {
                TPD_INFO("checkLimitsMap: Node[%d,%d] = %d exceed limit [%d, %d]\n", i, j, data[i * column + j], min[i * column + j], max[i * column + j]);
                count++;
            }
        }
    }

    return count;    /* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of short doesn't exceed a specific min and
  *  Max value  set for each node (these values are included in the interval).
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapTotal(short *data, int row, int column, int *min, int *max)
{
    int i, j;
    int count = 0;

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] < min[i * column + j] || data[i * column + j] > max[i * column + j]) {
                TPD_INFO("checkLimitsMapTotal: Node[%d,%d] = %d exceed limit [%d, %d]\n", i, j, data[i * column + j], min[i * column + j], max[i * column + j]);
                count++;
            }
        }
    }

    return count;    /* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u8 doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval). \n
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapFromU(u8 *data, int row, int column, int *min, int *max)
{
    int i, j;
    int count = 0;

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] < min[i * column + j] || data[i * column + j] > max[i * column + j]) {
                TPD_INFO("checkLimitsMap: Node[%d,%d] = %d exceed limit [%d, %d]\n", i, j, data[i * column + j], min[i * column + j], max[i * column + j]);
                count++;
            }
        }
    }

    return count;    /* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u16 doesn't exceed a specific min and
  * Max value  set for each node (these values are included in the interval).
  * The matrixes of data, min and max values are stored as 1 dimension arrays
  * one row after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param min pointer to a matrix which specify the minimum value allowed for
  * each node
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapTotalFromU(u16 *data, int row, int column, int *min, int *max)
{
    int i, j;
    int count = 0;

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] < min[i * column + j] || data[i * column + j] > max[i * column + j]) {
                TPD_INFO("checkLimitsMapTotal: Node[%d,%d] = %d exceed limit [%d, %d]\n", i, j, data[i * column + j], min[i * column + j], max[i * column + j]);
                count++;
            }
        }
    }

    return count;    /* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u8 doesn't exceed a specific Max value
  * set for each node (max value is included in the interval).
  * The matrixes of data and max values are stored as 1 dimension arrays one row
  * after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param max pointer to a matrix which specify the Maximum value allowed for
  *each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapAdj(u8 *data, int row, int column, int *max)
{
    int i, j;
    int count = 0;

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] > max[i * column + j]) {
                TPD_INFO("checkLimitsMapAdj: Node[%d,%d] = %d exceed limit > %d\n", i, j, data[i * column + j], max[i * column + j]);
                count++;
            }
        }
    }

    return count;    /* if count is 0 = OK, test completed successfully */
}

/**
  * Check that each value of a matrix of u16 doesn't exceed a specific Max value
  * set for each node (max value is included in the interval).
  * The matrixes of data and max values are stored as 1 dimension arrays one row
  * after the other.
  * @param data pointer to the array of short containing the data to check
  * @param row number of rows of data
  * @param column number of columns of data
  * @param max pointer to a matrix which specify the Maximum value allowed for
  * each node
  * @return the number of elements that overcome the specified interval (0 = OK)
  */
int checkLimitsMapAdjTotal(u16 *data, int row, int column, int *max)
{
    int i, j;
    int count = 0;

    for (i = 0; i < row; i++) {
        for (j = 0; j < column; j++) {
            if (data[i * column + j] > max[i * column + j]) {
                TPD_INFO("checkLimitsMapAdjTotal: Node[%d,%d] = %d exceed limit > %d\n", i, j, data[i * column + j], max[i * column + j]);
                count++;
            }
        }
    }

    return count;    /* if count is 0 = OK, test completed successfully */
}

/**
  * Perform an ITO test setting all the possible options
  * (see @link ito_opt ITO Options @endlink) and checking MS Raw ADJ if enabled
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ito(char *path_limits, TestToDo *todo)
{
    int res = OK;
    u8 sett[2] = { 0x00, 0x00 };
    MutualSenseFrame msRawFrame;
    int *thresholds = NULL;
    u16 *adj = NULL;
    int trows, tcolumns;

    msRawFrame.node_data = NULL;

    TPD_DEBUG("ITO Production test is starting...\n");

    res = fts_system_reset();
    if (res < 0) {
        TPD_INFO("%s: ERROR %08X\n", __func__, ERROR_PROD_TEST_ITO);
        return res | ERROR_PROD_TEST_ITO;
    }

    sett[0] = SPECIAL_TUNING_IOFF;
    TPD_DEBUG("Trimming Ioff...\n");
    res = writeSysCmd(SYS_CMD_SPECIAL_TUNING, sett, 2);
    if (res < OK) {
        TPD_INFO("production_test_ito: Trimm Ioff ERROR %08X\n", (res | ERROR_PROD_TEST_ITO));
        return res | ERROR_PROD_TEST_ITO;
    }

    sett[0] = 0xFF;
    sett[1] = 0xFF;
    TPD_DEBUG("ITO Check command sent...\n");
    res = writeSysCmd(SYS_CMD_ITO, sett, 2);
    if (res < OK) {
        TPD_INFO("production_test_ito: ERROR %08X\n", (res | ERROR_PROD_TEST_ITO));
        return res | ERROR_PROD_TEST_ITO;
    }

    TPD_DEBUG("ITO Command = OK!\n");

    TPD_DEBUG("MS RAW ITO ADJ TEST:\n");
    if (todo->MutualRawAdjITO == 1) {
        TPD_DEBUG("Collecting MS Raw data...\n");
        res |= getMSFrame3(MS_RAW, &msRawFrame);
        if (res < OK) {
            TPD_INFO("%s: getMSFrame failed... ERROR %08X\n", __func__, ERROR_PROD_TEST_ITO);
            goto ERROR;
        }
        TPD_DEBUG("MS RAW ITO ADJ HORIZONTAL TEST:\n");
        res = computeAdjHorizTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, &adj);
        if (res < OK) {
            TPD_INFO("%s: computeAdjHoriz failed... ERROR %08X\n", __func__, ERROR_PROD_TEST_ITO);
            goto ERROR;
        }

        res = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_ITO_ADJH, &thresholds, &trows, &tcolumns);
        if (res < OK || (trows != msRawFrame.header.force_node || tcolumns != msRawFrame.header.sense_node - 1)) {
            TPD_INFO("%s: parseProductionTestLimits MS_RAW_ITO_ADJH failed... ERROR %08X\n", __func__, ERROR_PROD_TEST_DATA);
            goto ERROR;
        }

        res = checkLimitsMapAdjTotal(adj, msRawFrame.header.force_node, msRawFrame.header.sense_node - 1, thresholds);
        if (res != OK) {
            TPD_INFO("production_test_data: checkLimitsAdj MS RAW ITO ADJH failed... ERROR COUNT = %d\n", res);
            TPD_DEBUG("MS RAW ITO ADJ HORIZONTAL TEST:.................FAIL\n\n");
            print_frame_short("MS Raw ITO frame =",
                      array1dTo2d_short( msRawFrame.node_data, msRawFrame. node_data_size, msRawFrame.header. sense_node),
                      msRawFrame.header.force_node, msRawFrame.header.sense_node);
            res = ERROR_PROD_TEST_ITO;
            goto ERROR;
        } else
            TPD_DEBUG("MS RAW ITO ADJ HORIZONTAL TEST:.................OK\n");

        kfree(thresholds);
        thresholds = NULL;

        kfree(adj);
        adj = NULL;

        TPD_DEBUG("MS RAW ITO ADJ VERTICAL TEST:\n");
        res = computeAdjVertTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, &adj);
        if (res < OK) {
            TPD_INFO("%s: computeAdjVert failed... ERROR %08X\n", __func__, ERROR_PROD_TEST_ITO);
            goto ERROR;
        }

        res = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_ITO_ADJV, &thresholds, &trows, &tcolumns);
        if (res < OK || (trows != msRawFrame.header.force_node - 1 || tcolumns != msRawFrame.header.sense_node)) {
            TPD_INFO("%s: parseProductionTestLimits MS_RAW_ITO_ADJV failed... ERROR %08X\n", __func__, ERROR_PROD_TEST_ITO);
            goto ERROR;
        }

        res = checkLimitsMapAdjTotal(adj, msRawFrame.header.force_node - 1, msRawFrame.header.sense_node, thresholds);
        if (res != OK) {
            TPD_INFO("%s: checkLimitsAdj MS RAW ITO ADJV failed... ERROR COUNT = %d\n", __func__, res);
            TPD_DEBUG("MS RAW ITO ADJ VERTICAL TEST:.................FAIL\n\n");
            print_frame_short("MS Raw ITO frame =", array1dTo2d_short(msRawFrame.node_data, msRawFrame. node_data_size, msRawFrame.header. sense_node),
                      msRawFrame.header.force_node, msRawFrame.header.sense_node);
            res = ERROR_PROD_TEST_ITO;
            goto ERROR;
        } else
            TPD_DEBUG("MS RAW ITO ADJ VERTICAL TEST:.................OK\n");

        kfree(thresholds);
        thresholds = NULL;

        kfree(adj);
        adj = NULL;
    } else
        TPD_DEBUG("MS RAW ITO ADJ TEST:.................SKIPPED\n");

ERROR:
    if (thresholds != NULL)
        kfree(thresholds);
    if (adj != NULL)
        kfree(adj);
    if (msRawFrame.node_data != NULL)
        kfree(msRawFrame.node_data);
    freeLimitsFile(&limit_file);
    res |= fts_system_reset();
    if (res < OK) {
        TPD_INFO("production_test_ito: ERROR %08X\n",
             ERROR_PROD_TEST_ITO);
        res = (res | ERROR_PROD_TEST_ITO);
    }
    return res;
}

/**
  * Perform the Initialization of the IC
  * @param type type of initialization to do
  * (see @link sys_special_opt Initialization Options (Full or Panel) @endlink)
  * @return OK if success or an error code which specify the type of error
  */
int production_test_initialization(u8 type)
{
    int res = 0;
	
    TPD_DEBUG("INITIALIZATION Production test is starting...\n");
    if (type != SPECIAL_PANEL_INIT && type != SPECIAL_FULL_PANEL_INIT) {
        TPD_INFO("production_test_initialization: Type incompatible! Type = %02X ERROR %08X\n", type, ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_INITIALIZATION);
        return ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_INITIALIZATION;
    }

    res = fts_system_reset();
    if (res < 0) {
        TPD_INFO("production_test_initialization: ERROR %08X\n",ERROR_PROD_TEST_INITIALIZATION);
        return res | ERROR_PROD_TEST_INITIALIZATION;
    }

    TPD_DEBUG("INITIALIZATION command sent... %02X\n", type);

    res = writeSysCmd(SYS_CMD_SPECIAL, &type, 1);
    if (res < OK) {
        TPD_INFO("production_test_initialization: ERROR %08X\n", (res | ERROR_PROD_TEST_INITIALIZATION));
        return res | ERROR_PROD_TEST_INITIALIZATION;
    }
    TPD_DEBUG("Refresh Sys Info...\n");
    res |= readSysInfo(1);    /* need to update the chipInfo in order
                  * to refresh several versions */

    if (res < 0) {
        TPD_INFO("production_test_initialization: read sys info ERROR %08X\n", ERROR_PROD_TEST_INITIALIZATION);
        res = (res | ERROR_PROD_TEST_INITIALIZATION);
    }

    return res;
}


/**
  * Perform a FULL (ITO + INIT + DATA CHECK) Mass Production Test of the IC
  * @param pathThresholds name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param saveInit if >0 (possible values: NO_INIT, SPECIAL_PANEL_INIT or
  * SPECIAL_FULL_PANEL_INIT),
  * the Initialization of the IC is executed otherwise it is skipped
  * @param todo pointer to a TestToDo variable which select the test to do
  * @param mpflag MP flag value to write in case of successful test
  * @return OK if success or an error code which specify the type of error
  */
int production_test_main(char *pathThresholds, int stop_on_fail, int saveInit,
             TestToDo *todo, u8 mpflag)
{
    int res, ret;

    TPD_DEBUG("MAIN Production test is starting...\n");

    TPD_DEBUG("ITO TEST:\n");
    res = production_test_ito(pathThresholds, todo);
    if (res < 0) {
        TPD_DEBUG("Error during ITO TEST! ERROR %08X\n", res);
		if (stop_on_fail)
        goto END;/* in case of ITO TEST failure is no sense keep going
             * */
    } else
        TPD_DEBUG("ITO TEST OK!\n");

    TPD_DEBUG("INITIALIZATION TEST :\n");
    if (saveInit != NO_INIT) {
        res = production_test_initialization((u8)saveInit);
        if (res < 0) {
            TPD_DEBUG("Error during  INITIALIZATION TEST! ERROR %08X\n", res);
            if (stop_on_fail)
                goto END;
        } else
            TPD_DEBUG("INITIALIZATION TEST OK!\n");
    } else
        TPD_DEBUG("INITIALIZATION TEST :................. SKIPPED\n");

    if (saveInit != NO_INIT) {
        TPD_DEBUG("Cleaning up...\n");
        ret = fts_system_reset();
        if (ret < 0) {
            TPD_INFO("production_test_main: system reset ERROR %08X\n", ret);
            res |= ret;
            if (stop_on_fail)
                goto END;
        }
    }

    TPD_DEBUG("PRODUCTION DATA TEST:\n");
    ret = production_test_data(pathThresholds, stop_on_fail, todo);
    if (ret < OK)
        TPD_DEBUG("Error during PRODUCTION DATA TEST! ERROR %08X\n", ret);
    else {
        TPD_DEBUG("PRODUCTION DATA TEST OK!\n");

#ifdef COMPUTE_INIT_METHOD
        if (saveInit != NO_INIT) {
            /* save the mp flag to desired value
             * because data check OK */
            ret = saveMpFlag(mpflag);
            if (ret < OK)
                TPD_DEBUG("Error while saving MP FLAG! ERROR %08X\n", ret);
            else
                TPD_DEBUG("MP FLAG saving OK!\n");
        }
#endif

    }

    res |= ret;
    /* the OR is important because if the data test is OK but
      * the init test fail, the main production test result should = FAIL */


END:
    if (res < OK) {
        TPD_DEBUG("MAIN Production test finished.................FAILED\n");
        return res;
    } else {
        TPD_DEBUG("MAIN Production test finished.................OK\n");
        return OK;
    }
}

/**
  * Perform all the test selected in a TestTodo variable related to MS raw data
  * (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_raw(char *path_limits, int stop_on_fail, TestToDo *todo)
{
    int ret, count_fail = 0;
    MutualSenseFrame msRawFrame;


    int *thresholds = NULL;
    int trows, tcolumns;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;
    u16 *adj = NULL;

    int maxAdjH = 0, maxAdjV = 0;
    int i, z;

    msRawFrame.node_data = NULL;

    /************** Mutual Sense Test *************/
    TPD_DEBUG("MS RAW DATA TEST is starting...\n");
    if (todo->MutualRaw == 1 || todo->MutualRawGap == 1 ||
        todo->MutualRawAdj == 1 || todo->MutualRawMap == 1 ||
        todo->MutualRawAdjGap == 1 || todo->MutualRawAdjPeak == 1) {
        ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
        msleep(WAIT_FOR_FRESH_FRAMES);
        ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
        msleep(WAIT_AFTER_SENSEOFF);
        ret |= getMSFrame3(MS_RAW, &msRawFrame);
        if (ret < OK) {
            TPD_INFO("production_test_data: getMSFrame failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            return ret | ERROR_PROD_TEST_DATA;
        }

        TPD_DEBUG("MS RAW MIN MAX TEST:\n");
        if (todo->MutualRaw == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_MIN_MAX, &thresholds, &trows, &tcolumns);
            if (ret < OK || (trows != 1 || tcolumns != 2)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMinMax(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node,
                        thresholds[0], thresholds[1]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax MS RAW failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS RAW MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else
                TPD_DEBUG("MS RAW MIN MAX TEST:.................OK\n");
            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("MS RAW MIN MAX TEST:.................SKIPPED\n");
        TPD_DEBUG("MS RAW MAP MIN MAX TEST:\n");
        if (todo->MutualRawMap == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_EACH_NODE_MIN, &thresholds_min, &trows, &tcolumns);
            if (ret < OK || (trows != msRawFrame.header.force_node || tcolumns != msRawFrame.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_EACH_NODE_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_EACH_NODE_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < OK || (trows != msRawFrame.header.force_node || tcolumns != msRawFrame.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_EACH_NODE_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = checkLimitsMapTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, thresholds_min, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMaxEachNodeData failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS RAW MAP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else{
                TPD_DEBUG("MS RAW MAP MIN MAX TEST:.................OK\n");
            }
            if (thresholds_min != NULL) {
                kfree(thresholds_min);
                thresholds_min = NULL;
            }
            if (thresholds_max != NULL) {
                kfree(thresholds_max);
                thresholds_max = NULL;
            }
        } else {
            TPD_DEBUG("MS RAW MAP MIN MAX TEST:.................SKIPPED\n");
        }
        TPD_DEBUG("MS RAW GAP TEST:\n");
        if (todo->MutualRawGap == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_GAP, &thresholds, &trows, &tcolumns);
            if (ret < OK || (trows != 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_GAP failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsGap(msRawFrame.node_data,
                         msRawFrame.header.force_node,
                         msRawFrame.header.sense_node,
                         thresholds[0]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsGap MS RAW failed... ERROR = %08X\n", ret);
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else
                TPD_DEBUG("MS RAW GAP TEST:.................OK\n\n");
            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("MS RAW GAP TEST:.................SKIPPED\n");

        TPD_DEBUG("MS RAW ADJ TEST:\n");
        if ((todo->MutualRawAdj == 1) || (todo->MutualRawAdjGap == 1) || (todo->MutualRawAdjPeak == 1)) {
            TPD_DEBUG("MS RAW ADJ HORIZONTAL TESTs:\n");
            ret = computeAdjHorizTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, &adj);
            if (ret < OK) {
                TPD_INFO("production_test_data: computeAdjHoriz failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            if (todo->MutualRawAdj) {
                TPD_DEBUG("MS RAW ADJ HORIZONTAL min/Max:\n");

                ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_ADJH, &thresholds, &trows, &tcolumns);
                if (ret < OK || (trows != msRawFrame.header.force_node || tcolumns != msRawFrame.header.sense_node - 1)) {
                    TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                    ret |= ERROR_PROD_TEST_DATA;
                    goto ERROR_LIMITS;
                }

                ret = checkLimitsMapAdjTotal(adj, msRawFrame.header. force_node, msRawFrame.header. sense_node - 1, thresholds);
                if (ret != OK) {
                    TPD_INFO("production_test_data: checkLimitsAdj MS RAW ADJH failed... ERROR COUNT = %d\n", ret);
                    TPD_DEBUG("MS RAW ADJ HORIZONTAL min/Max:.................FAIL\n\n");
                    count_fail += 1;
                    if (stop_on_fail == 1)
                        goto ERROR;
                } else
                    TPD_DEBUG("MS RAW ADJ HORIZONTAL min/Max:.................OK\n");

                if (thresholds != NULL) {
                    kfree(thresholds);
                    thresholds = NULL;
                }
            }


            if (todo->MutualRawAdjGap) {
                TPD_DEBUG("MS RAW ADJ HORIZONTAL GAP:\n");

                ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_ADJH_GAP, &thresholds, &trows, &tcolumns);
                if (ret < OK || (trows != 1 || tcolumns != 1)) {
                    TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                    ret |= ERROR_PROD_TEST_DATA;
                    goto ERROR_LIMITS;
                }

                ret = checkLimitsGapOffsets(adj, msRawFrame.header.force_node, msRawFrame.header.sense_node - 1, thresholds[0], 1, 1, 1, 1);
                if (ret != OK) {
                    TPD_INFO("production_test_data: checkLimitsAdj MS RAW ADJH GAP failed...\n");
                    TPD_DEBUG("MS RAW ADJ HORIZONTAL GAP:.................FAIL\n\n");
                    count_fail += 1;
                    if (stop_on_fail == 1)
                        goto ERROR;
                } else
                    TPD_DEBUG("MS RAW ADJ HORIZONTAL GAP:.................OK\n");

                if (thresholds != NULL) {
                    kfree(thresholds);
                    thresholds = NULL;
                }
            }


            if (todo->MutualRawAdjPeak) {
                TPD_DEBUG("MS RAW ADJ Peak: Getting max ADJH\n");
                maxAdjH = abs(adj[msRawFrame.header.force_node + 1]);
                /* skip nodes on the edges */
                for (i = 1; i < msRawFrame.header.force_node - 1; i++) {
                    for (z = 1; z < (msRawFrame.header.sense_node - 2); z++){
                        if (maxAdjH < abs(adj[(i * msRawFrame.header.force_node) + z]))
                        maxAdjH = abs(adj[(i * msRawFrame.header.force_node) + z]);
                    }

                }
            }

            kfree(adj);
            adj = NULL;

            TPD_DEBUG("MS RAW ADJ VERTICAL TESTs:\n");
            ret = computeAdjVertTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, &adj);
            if (ret < OK) {
                TPD_INFO("production_test_data: computeAdjVert failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            if (todo->MutualRawAdj) {
                TPD_DEBUG("MS RAW ADJ VERTICAL min/Max:\n");
                ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_ADJV, &thresholds, &trows, &tcolumns);
                if (ret < OK || (trows != msRawFrame.header.force_node - 1 || tcolumns != msRawFrame.header.sense_node)) {
                    TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_ADJV failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                    ret |= ERROR_PROD_TEST_DATA;
                    goto ERROR_LIMITS;
                }

                ret = checkLimitsMapAdjTotal(adj,msRawFrame.header.force_node - 1, msRawFrame.header.sense_node, thresholds);
                if (ret != OK) {
                    TPD_INFO("production_test_data: checkLimitsAdj MS RAW ADJV failed... ERROR COUNT = %d\n", ret);
                    TPD_DEBUG("MS RAW ADJ VERTICAL min/Max:.................FAIL\n\n");
                    count_fail += 1;
                    if (stop_on_fail == 1)
                        goto ERROR;
                } else
                    TPD_DEBUG("MS RAW ADJ VERTICAL min/Max:.................OK\n");

                if (thresholds != NULL) {
                    kfree(thresholds);
                    thresholds = NULL;
                }
            }

            if (todo->MutualRawAdjGap) {
                TPD_DEBUG("MS RAW ADJ VERTICAL GAP:\n");
                ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_ADJV_GAP, &thresholds, &trows, &tcolumns);
                if (ret < OK || (trows != 1 || tcolumns != 1)) {
                    TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_ADJV_GAP failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                    ret |= ERROR_PROD_TEST_DATA;
                    goto ERROR_LIMITS;
                }

                ret = checkLimitsGapOffsets(adj, msRawFrame.header.force_node - 1, msRawFrame.header.sense_node, thresholds[0], 1, 1, 1, 1);
                if (ret != OK) {
                    TPD_INFO("production_test_data: checkLimitsAdj MS RAW ADJV GAP failed... ERROR COUNT = %d\n", ret);
                    TPD_DEBUG("MS RAW ADJ VERTICAL GAP:.................FAIL\n\n");
                    count_fail += 1;
                    if (stop_on_fail == 1)
                        goto ERROR;
                } else
                    TPD_DEBUG("MS RAW ADJ VERTICAL GAP:.................OK\n");

                if (thresholds != NULL) {
                    kfree(thresholds);
                    thresholds = NULL;
                }
            }


            if (todo->MutualRawAdjPeak) {
                TPD_DEBUG("MS RAW ADJ Peak: Getting max ADJV\n");
                maxAdjV = abs(adj[(msRawFrame.header.force_node) + 1]);

                /* skip nodes on the edges */
                for (i = 1; i < (msRawFrame.header.force_node - 2); i++) {
                    for (z = 1; z < msRawFrame.header.sense_node - 1; z++) {
                        if (maxAdjH < abs(adj[(i * msRawFrame.header.force_node) + z]))
                            maxAdjH = abs(adj[(i * msRawFrame.header.force_node) + z]);

                    }
                }

                ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_ADJ_PEAK, &thresholds, &trows, &tcolumns);
                if (ret < OK || (trows != 1 || tcolumns != 1)) {
                    TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_ADJV_PEAK failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                    ret |= ERROR_PROD_TEST_DATA;
                    goto ERROR_LIMITS;
                }

                TPD_INFO("maxAdjH = %d  maxAdjV = %d  threshold = %d\n",maxAdjH, maxAdjV, thresholds[0]);

                ret = OK;
                if (maxAdjH > maxAdjV) {
                    if (maxAdjH > thresholds[0])
                        ret = ERROR_PROD_TEST_DATA;
                } else {
                    if (maxAdjV > thresholds[0])
                        ret = ERROR_PROD_TEST_DATA;
                }

                if (ret != OK) {
                    TPD_INFO("production_test_data: checkLimitsAdj MS RAW ADJV GAP failed... ERROR COUNT = %d\n", ret);
                    TPD_DEBUG("MS RAW ADJ PEAK:.................FAIL\n\n");
                    count_fail += 1;
                    if (stop_on_fail == 1)
                        goto ERROR;
                } else
                    TPD_DEBUG("MS RAW ADJ PEAK:.................OK\n");

                if (thresholds != NULL) {
                    kfree(thresholds);
                    thresholds = NULL;
                }

            }


            kfree(adj);
            adj = NULL;
        } else
            TPD_DEBUG("MS RAW ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("MS RAW DATA TEST:.................SKIPPED\n");

    TPD_DEBUG("MS KEY RAW TEST:\n");
    if (todo->MutualKeyRaw == 1) {
        ret = production_test_ms_key_raw(path_limits);
        if (ret < 0) {
            TPD_INFO("production_test_data: production_test_ms_key_raw failed... ERROR = %08X\n", ret);
            count_fail += 1;
            if (count_fail == 1) {
                TPD_DEBUG("MS RAW DATA TEST:.................FAIL fails_count = %d\n\n", count_fail);
                goto ERROR_LIMITS;
            }
        }
    } else
        TPD_DEBUG("MS KEY RAW TEST:.................SKIPPED\n");

    ret = production_test_ms_raw_lp(path_limits, stop_on_fail, todo);
    if (ret < 0) {
        TPD_INFO("production_test_data: production_test_ms_raw_lp failed... ERROR = %08X\n",ret);
        count_fail += 1;
        if (count_fail == 1) {
            TPD_DEBUG("MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",count_fail);
            goto ERROR_LIMITS;
        }
    }

ERROR:
    if (count_fail == 0) {
        if (msRawFrame.node_data != NULL) {
            kfree(msRawFrame.node_data);
            msRawFrame.node_data = NULL;
        }
        TPD_DEBUG("MS RAW DATA TEST finished!.................OK\n");
        return OK;
    } else {
        print_frame_short("MS Raw frame =", array1dTo2d_short(msRawFrame.node_data, msRawFrame.node_data_size, msRawFrame.header.sense_node),
                  msRawFrame.header.force_node, msRawFrame.header.sense_node);

        if (msRawFrame.node_data != NULL)
            kfree(msRawFrame.node_data);
        if (thresholds != NULL)
            kfree(thresholds);
        if (adj != NULL)
            kfree(adj);
        if (thresholds_min != NULL) {
            kfree(thresholds_min);
            thresholds_min = NULL;
        }
        if (thresholds_max != NULL) {
            kfree(thresholds_max);
            thresholds_max = NULL;
        }
        TPD_DEBUG("MS RAW DATA TEST:.................FAIL fails_count = %d\n\n",count_fail);
        return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
    }


ERROR_LIMITS:
    if (msRawFrame.node_data != NULL)
        kfree(msRawFrame.node_data);
    if (thresholds != NULL)
        kfree(thresholds);
    return ret;
}


/**
  * Perform all the test selected in a TestTodo variable related to MS low power
  * raw data
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_raw_lp(char *path_limits, int stop_on_fail,
                  TestToDo *todo)
{
    int ret, count_fail = 0;
    MutualSenseFrame msRawFrame;


    int *thresholds = NULL;
    int trows, tcolumns;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;

    u16 *adj = NULL;

    msRawFrame.node_data = NULL;

    /************** Mutual Sense Test **************/
    TPD_DEBUG("MS RAW LP DATA TEST:\n");
    if (todo->MutualRawLP == 1 || todo->MutualRawGapLP == 1 ||
        todo->MutualRawAdjLP == 1) {
        ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_LP_ACTIVE);
        msleep(WAIT_FOR_FRESH_FRAMES);
        ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
        msleep(WAIT_AFTER_SENSEOFF);
        ret |= getMSFrame3(MS_RAW, &msRawFrame);
        if (ret < 0) {
            TPD_INFO("production_test_data: getMSFrame failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            return ret | ERROR_PROD_TEST_DATA;
        }

        TPD_DEBUG("MS RAW LP MIN MAX TEST:\n");
        if (todo->MutualRawLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_LP_MIN_MAX, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != 1 || tcolumns != 2)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_LP_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMinMax(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node,
                        thresholds[0], thresholds[1]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax MS RAW LP failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS RAW LP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else
                TPD_DEBUG("MS RAW LP MIN MAX TEST:.................OK\n");
            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("MS RAW LP MIN MAX TEST:.................SKIPPED\n");

        TPD_DEBUG("MS RAW LP MAP MIN MAX TEST:\n");
        if (todo->MutualRawMapLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_LP_EACH_NODE_MIN, &thresholds_min, &trows, &tcolumns);
            if (ret < OK || (trows != msRawFrame.header.force_node || tcolumns != msRawFrame.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_LP_EACH_NODE_MIN failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_LP_EACH_NODE_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < OK || (trows != msRawFrame.header.force_node || tcolumns != msRawFrame.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_LP_EACH_NODE_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = checkLimitsMapTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, thresholds_min, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMaxEachNodeData failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS RAW LP MAP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else{
                TPD_DEBUG("MS RAW LP MAP MIN MAX TEST:.................OK\n");
            }
            if (thresholds_min != NULL) {
                kfree(thresholds_min);
                thresholds_min = NULL;
            }
            if (thresholds_max != NULL) {
                kfree(thresholds_max);
                thresholds_max = NULL;
            }
        } else {
            TPD_DEBUG("MS RAW LP MAP MIN MAX TEST:.................SKIPPED\n");
        }

        TPD_DEBUG("MS RAW LP GAP TEST:\n");
        if (todo->MutualRawGapLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_LP_GAP, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_LP_GAP failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsGap(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, thresholds[0]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsGap MS RAW LP failed... ERROR = %08X\n", ret);
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else
                TPD_DEBUG("MS RAW LP GAP TEST:.................OK\n\n");
            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("MS RAW LP GAP TEST:.................SKIPPED\n");

        TPD_DEBUG("MS RAW LP ADJ TEST:\n");
        if (todo->MutualRawAdjLP == 1) {
            TPD_DEBUG("MS RAW LP ADJ HORIZONTAL TEST:\n");
            ret = computeAdjHorizTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, &adj);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjHoriz failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_LP_ADJH, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != msRawFrame.header.force_node || tcolumns != msRawFrame.header.sense_node - 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_LP_ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(adj, msRawFrame.header.force_node, msRawFrame.header.sense_node - 1, thresholds);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsAdj MS RAW LP ADJH failed... ERROR COUNT = %d\n",ret);
                TPD_DEBUG("MS RAW LP ADJ HORIZONTAL TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else
                TPD_DEBUG("MS RAW LP ADJ HORIZONTAL TEST:.................OK\n");

            kfree(thresholds);
            thresholds = NULL;

            kfree(adj);
            adj = NULL;

            TPD_DEBUG("MS RAW LP ADJ VERTICAL TEST:\n");
            ret = computeAdjVertTotal(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node, &adj);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjVert failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_RAW_LP_ADJV, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != msRawFrame.header.force_node - 1 || tcolumns != msRawFrame.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_RAW_ADJV failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(adj, msRawFrame.header.force_node - 1, msRawFrame.header.sense_node, thresholds);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsAdj MS RAW ADJV failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS RAW LP ADJ VERTICAL TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail == 1)
                    goto ERROR;
            } else
                TPD_DEBUG("MS RAW LP ADJ VERTICAL TEST:.................OK\n");
            kfree(thresholds);
            thresholds = NULL;

            kfree(adj);
            adj = NULL;
        } else
            TPD_DEBUG("MS RAW LP ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("MS RAW LP DATA TEST:.................SKIPPED\n");

ERROR:
    if (count_fail == 0) {
        if (msRawFrame.node_data != NULL) {
            kfree(msRawFrame.node_data);
            msRawFrame.node_data = NULL;
        }
        TPD_DEBUG("MS RAW DATA TEST finished!.................OK\n");
        return OK;
    } else {
        if (msRawFrame.node_data != NULL) {
            print_frame_short("MS Raw LP frame =",
                      array1dTo2d_short(msRawFrame.node_data, msRawFrame.node_data_size, msRawFrame.header.sense_node),
                      msRawFrame.header.force_node, msRawFrame.header.sense_node);
            kfree(msRawFrame.node_data);
        }
        if (thresholds != NULL)
            kfree(thresholds);
        if (adj != NULL)
            kfree(adj);
        if (thresholds_min != NULL) {
            kfree(thresholds_min);
            thresholds_min = NULL;
        }
        if (thresholds_max != NULL) {
            kfree(thresholds_max);
            thresholds_max = NULL;
        }
        TPD_DEBUG("MS RAW LP DATA TEST:.................FAIL fails_count = %d\n\n", count_fail);
        return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
    }


ERROR_LIMITS:
    if (msRawFrame.node_data != NULL)
        kfree(msRawFrame.node_data);
    if (thresholds != NULL)
        kfree(thresholds);
    return ret;
}

/**
  * Perform MS raw test for keys data
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_key_raw(char *path_limits)
{
    int ret;
    MutualSenseFrame msRawFrame;

    int *thresholds = NULL;
    int trows, tcolumns;

    msRawFrame.node_data = NULL;

    /************** Mutual Sense Test **************/
    TPD_DEBUG("MS KEY RAW DATA TEST is starting...\n");
    ret = setScanMode(SCAN_MODE_ACTIVE, 0xFF);
    msleep(WAIT_FOR_FRESH_FRAMES);
    ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    ret |= getMSFrame3(MS_KEY_RAW, &msRawFrame);
    if (ret < 0) {
        TPD_INFO("production_test_data: getMSKeyFrame failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    ret = parseProductionTestLimits(path_limits, &limit_file,
                    MS_KEY_RAW_MIN_MAX, &thresholds, &trows, &tcolumns);
    if (ret < 0 || (trows != 1 || tcolumns != 2)) {
        TPD_INFO("production_test_data: parseProductionTestLimits MS_KEY_RAW_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        ret |= ERROR_PROD_TEST_DATA;
        goto ERROR_LIMITS;
    }

    ret = checkLimitsMinMax(msRawFrame.node_data, msRawFrame.header.force_node, msRawFrame.header.sense_node,
                thresholds[0], thresholds[1]);
    if (ret != OK) {
        TPD_INFO("production_test_data: checkLimitsMinMax MS KEY RAW failed... ERROR COUNT = %d\n", ret);
        goto ERROR;
    } else
        TPD_DEBUG("MS KEY RAW TEST:.................OK\n\n");

    kfree(thresholds);
    thresholds = NULL;

    kfree(msRawFrame.node_data);
    msRawFrame.node_data = NULL;
    return OK;

ERROR:
    print_frame_short("MS Key Raw frame =", array1dTo2d_short(msRawFrame.node_data, msRawFrame.node_data_size, msRawFrame.header.sense_node),
              msRawFrame.header.force_node, msRawFrame.header.sense_node);
    if (msRawFrame.node_data != NULL)
        kfree(msRawFrame.node_data);
    if (thresholds != NULL)
        kfree(thresholds);
    TPD_DEBUG("MS KEY RAW TEST:.................FAIL\n\n");
    return ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;

ERROR_LIMITS:
    if (msRawFrame.node_data != NULL)
        kfree(msRawFrame.node_data);
    if (thresholds != NULL)
        kfree(thresholds);
    return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to MS Init
  * data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_cx(char *path_limits, int stop_on_fail, TestToDo *todo)
{
    int ret;
    int count_fail = 0;

    int *thresholds = NULL;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;
    int trows, tcolumns;

    MutualSenseData msCompData;
    TotMutualSenseData totCompData;

    u8 *adjhor = NULL;

    u8 *adjvert = NULL;

    u16 container;
    /* u16 *total_cx = NULL; */
    u16 *total_adjhor = NULL;
    u16 *total_adjvert = NULL;

    msCompData.node_data = NULL;
    totCompData.node_data = NULL;

    /* MS CX TEST */
    TPD_DEBUG("MS CX Testes are starting...\n");

    ret = readMutualSenseCompensationData(LOAD_CX_MS_TOUCH, &msCompData);
    /* read MS compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readMutualSenseCompensationData failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    ret = readTotMutualSenseCompensationData(LOAD_PANEL_CX_TOT_MS_TOUCH, &totCompData);
    /* read  TOT MS compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readTotMutualSenseCompensationData failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        kfree(msCompData.node_data);
        msCompData.node_data = NULL;
        return ret | ERROR_PROD_TEST_DATA;
    }

    TPD_DEBUG("MS CX1 TEST:\n");
    if (todo->MutualCx1 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX1_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX1_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (u16)msCompData.cx1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);
        /* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax MS CX1 failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS CX1 TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS CX1 TEST:.................OK\n\n");
    } else
        TPD_DEBUG("MS CX1 TEST:.................SKIPPED\n\n");

    kfree(thresholds);
    thresholds = NULL;

    TPD_DEBUG("MS CX2 MIN MAX TEST:\n");
    if (todo->MutualCx2 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_MAP_MIN, &thresholds_min, &trows, &tcolumns);
                        /* load min thresholds */
        if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load max thresholds */
        if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMap(msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, thresholds_min, thresholds_max);
                     /* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap MS CX2 MIN MAX failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS CX2 MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG(" MS CX2 MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("MS CX2 MIN MAX TEST:.................SKIPPED\n\n");

    TPD_DEBUG("MS CX2 ADJ TEST:\n");
    if (todo->MutualCx2Adj == 1) {
        /* MS CX2 ADJ HORIZ */
        TPD_DEBUG("MS CX2 ADJ HORIZ TEST:\n");

        ret = computeAdjHoriz(msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, &adjhor);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjHoriz failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("MS CX2 ADJ HORIZ computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns);
        if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node - 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_ADJH_MAP_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjhor, msCompData.header.force_node, msCompData.header.sense_node - 1, thresholds_max);
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj CX2 ADJH failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS CX2 ADJ HORIZ TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS CX2 ADJ HORIZ TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjhor);
        adjhor = NULL;

        /* MS CX2 ADJ VERT */
        TPD_DEBUG("MS CX2 ADJ VERT TEST:\n");

        ret = computeAdjVert(msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, &adjvert);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjVert failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("MS CX2 ADJ VERT computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);
        if (ret < 0 || (trows != msCompData.header.force_node - 1 || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_ADJV_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjvert, msCompData.header.force_node - 1, msCompData.header.sense_node - 1, thresholds_max);
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj CX2 ADJV failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS CX2 ADJ HORIZ TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS CX2 ADJ VERT TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjvert);
        adjvert = NULL;
    } else
        TPD_DEBUG("MS CX2 ADJ TEST:.................SKIPPED\n\n");

    /* START OF TOTAL CHECK */
    TPD_DEBUG("MS TOTAL CX TEST:\n");

    if (todo->MutualCxTotal == 1 || todo->MutualCxTotalAdj == 1) {
        TPD_DEBUG("MS TOTAL CX MIN MAX TEST:\n");
        if (todo->MutualCxTotal == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_MAP_MIN, &thresholds_min, &trows, &tcolumns);
            /* load min thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != totCompData.header.sense_node)) {
				TPD_INFO("ret = %d, trows = %d, force_node =%d, tcolumns =%d, sense_node =%d\n", ret, trows, totCompData.header.force_node, tcolumns, totCompData.header.sense_node);
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_MAP_MAX, &thresholds_max, &trows, &tcolumns);
            /* load max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != totCompData.header.sense_node)) {
				TPD_INFO("ret = %d, trows = %d, force_node =%d, tcolumns =%d, sense_node =%d\n", ret, trows, totCompData.header.force_node, tcolumns, totCompData.header.sense_node);
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(totCompData.node_data, totCompData.header.force_node, totCompData.header.sense_node, thresholds_min, thresholds_max);
            /* check the limits */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap  MS TOTAL CX TEST failed... ERROR COUNT = %d\n",ret);
                TPD_DEBUG(" MS TOTAL CX MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("MS TOTAL CX MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("MS TOTAL CX MIN MAX TEST:.................SKIPPED\n\n");


        TPD_DEBUG("MS TOTAL CX ADJ TEST:\n");
        if (todo->MutualCxTotalAdj == 1) {
            /* MS TOTAL CX ADJ HORIZ */
            TPD_DEBUG("MS TOTAL CX ADJ HORIZ TEST:\n");

            ret = computeAdjHorizTotal(totCompData.node_data, totCompData.header.force_node, totCompData.header.sense_node, &total_adjhor);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjHoriz failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("MS TOTAL CX ADJ HORIZ computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != totCompData.header.sense_node - 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_ADJH_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjhor, totCompData.header.force_node, totCompData.header.sense_node - 1, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj MS TOTAL CX ADJH failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS TOTAL CX ADJ HORIZ TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("MS TOTAL CX ADJ HORIZ TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjhor);
            total_adjhor = NULL;

            /* MS TOTAL CX ADJ VERT */
            TPD_DEBUG("MS TOTAL CX ADJ VERT TEST:\n");

            ret = computeAdjVertTotal(totCompData.node_data, totCompData.header.force_node, totCompData.header.sense_node, &total_adjvert);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjVert failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("MS TOTAL CX ADJ VERT computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < 0 || (trows != totCompData.header.force_node - 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_ADJV_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjvert, totCompData.header.force_node - 1, totCompData.header.sense_node - 1, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj MS TOTAL CX ADJV failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS TOTAL CX ADJ HORIZ TEST:.................FAIL\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("MS TOTAL CX ADJ VERT TEST:.................OK\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjvert);
            total_adjvert = NULL;
        } else
            TPD_DEBUG("MS TOTAL CX ADJ TEST:.................SKIPPED\n");

        kfree(totCompData.node_data);
        totCompData.node_data = NULL;
    } else
        TPD_DEBUG("MS TOTAL CX TEST:.................SKIPPED\n");



    if ((todo->MutualCx1LP | todo->MutualCx2LP | todo->MutualCx2AdjLP | todo->MutualCxTotalLP | todo->MutualCxTotalAdjLP) == 1) {
        ret = production_test_ms_cx_lp(path_limits, stop_on_fail, todo);
        if (ret < OK) {
            count_fail += 1;
            TPD_INFO("production_test_data: production_test_cx_lp failed... ERROR = %08X\n", ret);
            TPD_DEBUG("MS CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
            return ret;
        }
    } else
        TPD_DEBUG("MS CX LP TEST:.................SKIPPED\n");


    if ((todo->MutualKeyCx1 | todo->MutualKeyCx2 | todo->MutualKeyCxTotal) == 1) {
        ret = production_test_ms_key_cx(path_limits, stop_on_fail, todo);
        if (ret < 0) {
            count_fail += 1;
            TPD_INFO("production_test_data: production_test_ms_key_cx failed... ERROR = %08X\n", ret);
            TPD_DEBUG("MS CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
            return ret;
        }
    } else
        TPD_DEBUG("MS KEY CX TEST:.................SKIPPED\n");



ERROR:
    if (count_fail == 0) {
        TPD_DEBUG("MS CX testes finished!.................OK\n");
        kfree(msCompData.node_data);
        msCompData.node_data = NULL;
        return OK;
    } else {
        print_frame_i8("MS Init Data (Cx2) =", array1dTo2d_i8(msCompData.node_data, msCompData.node_data_size, msCompData.header.sense_node),
                   msCompData.header.force_node, msCompData.header.sense_node);
        print_frame_short(" TOT MS Init Data (Cx) =", array1dTo2d_short(totCompData.node_data, totCompData.node_data_size, totCompData.header.sense_node),
                  totCompData.header.force_node, totCompData.header.sense_node);
        TPD_DEBUG("MS CX testes finished!.................FAILED  fails_count = %d\n\n",count_fail);
        if (thresholds != NULL)
            kfree(thresholds);
        if (thresholds_min != NULL)
            kfree(thresholds_min);
        if (thresholds_max != NULL)
            kfree(thresholds_max);
        if (adjhor != NULL)
            kfree(adjhor);
        if (adjvert != NULL)
            kfree(adjvert);
        if (totCompData.node_data != NULL)
            kfree(totCompData.node_data);
        if (total_adjhor != NULL)
            kfree(total_adjhor);
        if (total_adjvert != NULL)
            kfree(total_adjvert);
        if (msCompData.node_data != NULL)
            kfree(msCompData.node_data);
        return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
    }

ERROR_LIMITS:
    if (thresholds != NULL)
        kfree(thresholds);
    if (thresholds_min != NULL)
        kfree(thresholds_min);
    if (thresholds_max != NULL)
        kfree(thresholds_max);
    if (adjhor != NULL)
        kfree(adjhor);
    if (adjvert != NULL)
        kfree(adjvert);
    if (totCompData.node_data != NULL)
        kfree(totCompData.node_data);
    if (total_adjhor != NULL)
        kfree(total_adjhor);
    if (total_adjvert != NULL)
        kfree(total_adjvert);
    if (msCompData.node_data != NULL)
        kfree(msCompData.node_data);
    return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to MS Init
  * data of the keys
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  *  otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_key_cx(char *path_limits, int stop_on_fail,
                  TestToDo *todo)
{
    int ret;
    int count_fail = 0;
    int num_keys = 0;

    int *thresholds = NULL;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;
    int trows, tcolumns;

    MutualSenseData msCompData;
    TotMutualSenseData totCompData;


    short container;

    msCompData.node_data = NULL;
    totCompData.node_data = NULL;

    /* MS CX TEST */
    TPD_DEBUG("MS KEY CX Testes are starting...\n");

    ret = readMutualSenseCompensationData(LOAD_CX_MS_KEY, &msCompData);
    /* read MS compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readMutualSenseCompensationData failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    if (msCompData.header.force_node > msCompData.header.sense_node)
        /* the meaningful data are only in the first row,
          * the other rows are only a copy of the first one */
        num_keys = msCompData.header.force_node;
    else
        num_keys = msCompData.header.sense_node;

    TPD_DEBUG("MS KEY CX1 TEST:\n");
    if (todo->MutualKeyCx1 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, MS_KEY_CX1_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_KEY_CX1_MIN_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (short)msCompData.cx1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);    /* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax MS CX1 failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS KEY CX1 TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS KEY CX1 TEST:.................OK\n\n");
    } else
        TPD_DEBUG("MS KEY CX1 TEST:.................SKIPPED\n\n");

    kfree(thresholds);
    thresholds = NULL;

    TPD_DEBUG("MS KEY CX2 TEST:\n");
    if (todo->MutualKeyCx2 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, MS_KEY_CX2_MAP_MIN, &thresholds_min, &trows, &tcolumns);
        /* load min thresholds */
        if (ret < 0 || (trows != msCompData.header.force_node  || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_KEY_CX2_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_KEY_CX2_MAP_MAX, &thresholds_max, &trows, &tcolumns);
        /* load max thresholds */
        if (ret < 0 || (trows != msCompData.header.force_node  || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_KEY_CX2_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMap(msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, thresholds_min, thresholds_max);
        /* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap MS KEY CX2 failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS KEY CX2 TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS KEY CX2 TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("MS CX2 TEST:.................SKIPPED\n\n");

    /* START OF TOTAL CHECK */
    TPD_DEBUG("MS KEY TOTAL CX TEST:\n");

    if (todo->MutualKeyCxTotal == 1) {
        ret = readTotMutualSenseCompensationData(LOAD_PANEL_CX_TOT_MS_KEY, &totCompData);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeTotalCx failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_KEY_TOTAL_CX_MAP_MIN, &thresholds_min, &trows, &tcolumns);
        /* load min thresholds */
        if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != totCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_KEY_TOTAL_CX_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_KEY_TOTAL_CX_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load max thresholds */
        if (ret < 0 || (trows != totCompData.header.force_node  || tcolumns != totCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_KEY_TOTAL_CX_MAP_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapTotal(totCompData.node_data, totCompData.header.force_node, totCompData.header.sense_node, thresholds_min, thresholds_max); /* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap  MS TOTAL KEY CX TEST failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS KEY TOTAL CX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS KEY TOTAL CX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;

        kfree(totCompData.node_data);
        totCompData.node_data = NULL;
    } else
        TPD_DEBUG("MS KEY TOTAL CX TEST:.................SKIPPED\n");


ERROR:
    if (count_fail == 0) {
        TPD_DEBUG("MS KEY CX testes finished!.................OK\n");
        kfree(msCompData.node_data);
        msCompData.node_data = NULL;
        return OK;
    } else {
        print_frame_i8("MS Key Init Data (Cx2) =", array1dTo2d_i8(msCompData.node_data, msCompData.node_data_size, msCompData.header.sense_node),
                   msCompData.header.force_node, msCompData.header.sense_node);
        TPD_DEBUG("MS Key CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
        if (thresholds != NULL)
            kfree(thresholds);
        if (thresholds_min != NULL)
            kfree(thresholds_min);
        if (thresholds_max != NULL)
            kfree(thresholds_max);
        if (msCompData.node_data != NULL)
            kfree(msCompData.node_data);
        if (totCompData.node_data != NULL)
            kfree(totCompData.node_data);
        return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
    }

ERROR_LIMITS:
    if (thresholds != NULL)
        kfree(thresholds);
    if (thresholds_min != NULL)
        kfree(thresholds_min);
    if (thresholds_max != NULL)
        kfree(thresholds_max);
    if (msCompData.node_data != NULL)
        kfree(msCompData.node_data);
    if (totCompData.node_data != NULL)
        kfree(totCompData.node_data);
    return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to MS LowPower
  * Init data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ms_cx_lp(char *path_limits, int stop_on_fail, TestToDo *todo)
{
    int ret;
    int count_fail = 0;

    int *thresholds = NULL;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;
    int trows, tcolumns;

    MutualSenseData msCompData;
    TotMutualSenseData totCompData;

    u8 *adjhor = NULL;

    u8 *adjvert = NULL;

    u16 container;
    /* u16 *total_cx = NULL; */
    u16 *total_adjhor = NULL;
    u16 *total_adjvert = NULL;

    msCompData.node_data = NULL;
    totCompData.node_data = NULL;

    /* MS CX TEST */
    TPD_DEBUG("MS LP CX Testes are starting...\n");

    ret = readMutualSenseCompensationData(LOAD_CX_MS_LOW_POWER, &msCompData);/* read MS compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readMutualSenseCompensationData failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    ret = readTotMutualSenseCompensationData(LOAD_PANEL_CX_TOT_MS_LOW_POWER, &totCompData);/* read  TOT MS compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readTotMutualSenseCompensationData failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
        kfree(msCompData.node_data);
        msCompData.node_data = NULL;
        return ret | ERROR_PROD_TEST_DATA;
    }

    TPD_DEBUG("MS LP CX1 TEST:\n");
    if (todo->MutualCx1LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX1_LP_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX1_LP_MIN_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (u16)msCompData.cx1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax MS LP CX1 failed... ERROR COUNT = %d\n",ret);
            TPD_DEBUG("MS LP CX1 TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS LP CX1 TEST:.................OK\n\n");
    } else
        TPD_DEBUG("MS LP CX1 TEST:.................SKIPPED\n\n");

    kfree(thresholds);
    thresholds = NULL;

    TPD_DEBUG("MS LP CX2 MIN MAX TEST:\n");
    if (todo->MutualCx2LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_LP_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load min thresholds */
        if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_LP_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_LP_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load max thresholds */
        if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_LP_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMap(msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, thresholds_min, thresholds_max);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap MS LP CX2 MIN MAX failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG(" MS LP CX2 MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS LP CX2 MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("MS LP CX2 MIN MAX TEST:.................SKIPPED\n\n");

    TPD_DEBUG("MS LP CX2 ADJ TEST:\n");
    if (todo->MutualCx2AdjLP == 1) {
        /* MS CX2 ADJ HORIZ */
        TPD_DEBUG("MS LP CX2 ADJ HORIZ TEST:\n");

        ret = computeAdjHoriz(msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, &adjhor);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjHoriz failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("MS LP CX2 ADJ HORIZ computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_ADJH_LP_MAP_MAX, &thresholds_max, &trows, &tcolumns);
        if (ret < 0 || (trows != msCompData.header.force_node || tcolumns != msCompData.header.sense_node - 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_ADJH_LP_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjhor, msCompData.header.force_node, msCompData.header.sense_node - 1, thresholds_max);
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj CX2 ADJH LP failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("MS LP CX2 ADJ HORIZ TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS LP CX2 ADJ HORIZ TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjhor);
        adjhor = NULL;

        /* MS CX2 ADJ VERT */
        TPD_DEBUG("MS LP CX2 ADJ VERT TEST:\n");

        ret = computeAdjVert(msCompData.node_data, msCompData.header.force_node, msCompData.header.sense_node, &adjvert);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjVert failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("MS LP CX2 ADJ VERT computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, MS_CX2_ADJV_LP_MAP_MAX, &thresholds_max, &trows, &tcolumns);
        if (ret < 0 || (trows != msCompData.header.force_node - 1 || tcolumns != msCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits MS_CX2_ADJV_LP_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjvert, msCompData.header.force_node - 1, msCompData.header.sense_node - 1, thresholds_max);
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj CX2 ADJV LP failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG(" MS LP CX2 ADJ HORIZ TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("MS LP CX2 ADJ VERT TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjvert);
        adjvert = NULL;
    } else
        TPD_DEBUG("MS LP CX2 ADJ TEST:.................SKIPPED\n\n");

    /* START OF TOTAL CHECK */
    TPD_DEBUG("MS TOTAL LP CX TEST:\n");

    if (todo->MutualCxTotalLP == 1 || todo->MutualCxTotalAdjLP == 1) {
        TPD_DEBUG("MS TOTAL LP CX MIN MAX TEST:\n");
        if (todo->MutualCxTotalLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_LP_MAP_MIN, &thresholds_min, &trows, &tcolumns);
            /* load min thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_LP_MAP_MIN failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_LP_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_LP_MAP_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(totCompData.node_data, totCompData.header.force_node, totCompData.header.sense_node, thresholds_min, thresholds_max);/* check the limits */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap  MS TOTAL CX LP TEST failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS TOTAL CX LP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("MS TOTAL CX LP MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("MS TOTAL CX LP MIN MAX TEST:.................SKIPPED\n\n");


        TPD_DEBUG("MS TOTAL CX ADJ LP TEST:\n");
        if (todo->MutualCxTotalAdjLP == 1) {
            /* MS TOTAL CX ADJ HORIZ */
            TPD_DEBUG("MS TOTAL CX ADJ HORIZ LP TEST:\n");

            ret = computeAdjHorizTotal(totCompData.node_data, totCompData.header.force_node, totCompData.header.sense_node, &total_adjhor);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjHoriz failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("MS TOTAL CX ADJ HORIZ LP computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_ADJH_LP_MAP_MAX,
                            &thresholds_max, &trows, &tcolumns);
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != totCompData.header.sense_node - 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_ADJH_LP_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjhor, totCompData.header.force_node, totCompData.header.sense_node - 1, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj MS TOTAL CX ADJH LP failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS TOTAL CX ADJ HORIZ LP TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("MS TOTAL CX ADJ HORIZ LP TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjhor);
            total_adjhor = NULL;

            /* MS TOTAL CX ADJ VERT */
            TPD_DEBUG("MS TOTAL CX ADJ VERT LP TEST:\n");

            ret = computeAdjVertTotal(totCompData.node_data, totCompData.header.force_node, totCompData.header.sense_node, &total_adjvert);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjVert failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("MS TOTAL CX ADJ VERT LP computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, MS_TOTAL_CX_ADJV_LP_MAP_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < 0 || (trows != totCompData.header.force_node - 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits MS_TOTAL_CX_ADJV_LP_MAP_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjvert, totCompData.header.force_node - 1, totCompData.header.sense_node - 1, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj MS TOTAL CX ADJV failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("MS TOTAL CX ADJ HORIZ LP TEST:.................FAIL\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("MS TOTAL CX ADJ VERT LP TEST:.................OK\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjvert);
            total_adjvert = NULL;
        } else
            TPD_DEBUG("MS TOTAL CX ADJ LP TEST:.................SKIPPED\n");

        kfree(totCompData.node_data);
        totCompData.node_data = NULL;
    } else
        TPD_DEBUG("MS TOTAL CX LP TEST:.................SKIPPED\n");



ERROR:

    if (count_fail == 0) {
        TPD_DEBUG("MS LP CX testes finished!.................OK\n");
        kfree(msCompData.node_data);
        msCompData.node_data = NULL;
        return OK;
    } else {
        print_frame_i8("MS LP Init Data (Cx2) =", array1dTo2d_i8(msCompData.node_data, msCompData.node_data_size, msCompData.header.sense_node),
                   msCompData.header.force_node, msCompData.header.sense_node);
        print_frame_short(" TOT MS LP Init Data (Cx) =", array1dTo2d_short(totCompData.node_data,totCompData.node_data_size,totCompData.header.sense_node),
                  totCompData.header.force_node, totCompData.header.sense_node);
        TPD_DEBUG("MS LP CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
        if (thresholds != NULL)
            kfree(thresholds);
        if (thresholds_min != NULL)
            kfree(thresholds_min);
        if (thresholds_max != NULL)
            kfree(thresholds_max);
        if (adjhor != NULL)
            kfree(adjhor);
        if (adjvert != NULL)
            kfree(adjvert);
        if (totCompData.node_data != NULL)
            kfree(totCompData.node_data);
        if (total_adjhor != NULL)
            kfree(total_adjhor);
        if (total_adjvert != NULL)
            kfree(total_adjvert);
        if (msCompData.node_data != NULL)
            kfree(msCompData.node_data);
        return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
    }

ERROR_LIMITS:
    if (thresholds != NULL)
        kfree(thresholds);
    if (thresholds_min != NULL)
        kfree(thresholds_min);
    if (thresholds_max != NULL)
        kfree(thresholds_max);
    if (adjhor != NULL)
        kfree(adjhor);
    if (adjvert != NULL)
        kfree(adjvert);
    if (totCompData.node_data != NULL)
        kfree(totCompData.node_data);
    if (total_adjhor != NULL)
        kfree(total_adjhor);
    if (total_adjvert != NULL)
        kfree(total_adjvert);
    if (msCompData.node_data != NULL)
        kfree(msCompData.node_data);
    return ret;
}

/**
  * Perform all the test selected in a TestTodo variable related to SS raw data
  * (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_raw(char *path_limits, int stop_on_fail, TestToDo *todo)
{
    int ret;
    int count_fail = 0;
    int rows, columns;

    SelfSenseFrame ssRawFrame;

    int *thresholds = NULL;
    int trows, tcolumns;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;

    ssRawFrame.sense_data = NULL;
    ssRawFrame.force_data = NULL;

    /* SS TEST */

    TPD_DEBUG("SS RAW Testes are starting...\n");

    /************** Self Sense Test **************/

    TPD_DEBUG("Getting SS Frame...\n");
    ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
    msleep(WAIT_FOR_FRESH_FRAMES);
    ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    ret |= getSSFrame3(SS_RAW, &ssRawFrame);
    if (ret < 0) {
        TPD_INFO("production_test_data: getSSFrame failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    /* SS RAW (PROXIMITY) FORCE TEST */
    TPD_DEBUG("SS RAW FORCE TEST:\n");



    if (todo->SelfForceRaw == 1 || todo->SelfForceRawGap == 1 || todo->SelfForceRawMap == 1) {
        columns = 1;    /* there are no data for the sense channels
                  * because is a force frame */
        rows = ssRawFrame.header.force_node;

        TPD_DEBUG("SS RAW FORCE MIN MAX TEST:\n");
        if (todo->SelfForceRaw == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != 1 || tcolumns != 2)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_FORCE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMinMax(ssRawFrame.force_data, rows, columns, thresholds[0], thresholds[1]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax SS RAW FORCE failed... ERROR COUNT = %d\n",ret);
                TPD_DEBUG("SS RAW (PROXIMITY) FORCE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw force frame =", array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows,columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW FORCE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW FORCE MIN MAX TEST:.................SKIPPED\n\n");

        TPD_DEBUG("SS RAW FORCE MAP MIN MAX TEST:\n");
        if (todo->SelfForceRawMap == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_FORCE_EACH_NODE_MIN, &thresholds_min, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_FORCE_EACH_NODE_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_FORCE_EACH_NODE_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_FORCE_EACH_NODE_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(ssRawFrame.force_data, rows, columns, thresholds_min, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax SS RAW FORCE MAP failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS RAW FORCE MAP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw force frame =", array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW FORCE MAP MIN MAX TEST:.................OK\n\n");

            if (thresholds_min != NULL) {
                kfree(thresholds_min);
                thresholds_min = NULL;
            }
            if (thresholds_max != NULL) {
                kfree(thresholds_max);
                thresholds_max = NULL;
            }
        } else
            TPD_DEBUG("SS RAW FORCE MAP MIN MAX TEST:.................SKIPPED\n\n");


        TPD_DEBUG("SS RAW FORCE GAP TEST:\n");
        if (todo->SelfForceRawGap == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_FORCE_GAP, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_FORCE_GAP failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsGap(ssRawFrame.force_data, rows, columns, thresholds[0]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsGap SS RAW FORCE GAP failed... ERROR = %08X\n", ret);
                TPD_DEBUG("SS RAW FORCE GAP TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw force frame =", array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW FORCE GAP TEST:.................OK\n\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW FORCE GAP TEST:.................SKIPPED\n\n");

        kfree(ssRawFrame.force_data);
        ssRawFrame.force_data = NULL;
    } else
        TPD_DEBUG("SS RAW FORCE TEST:.................SKIPPED\n\n");


    /* SS RAW (PROXIMITY) SENSE TEST */
    TPD_DEBUG("SS RAW SENSE TEST:\n");

    if (todo->SelfSenseRaw == 1 || todo->SelfSenseRawGap == 1 || todo->SelfSenseRawMap == 1) {
        columns = ssRawFrame.header.sense_node;
        rows = 1; /* there are no data for the force channels
              *  because is a sense frame */

        TPD_DEBUG("SS RAW SENSE MIN MAX TEST:\n");
        if (todo->SelfSenseRaw == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != 1 || tcolumns != 2)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_SENSE_MIN_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMinMax(ssRawFrame.sense_data, rows, columns, thresholds[0], thresholds[1]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax SS RAW SENSE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS RAW SENSE MIN MAX TEST:.................FAIL\n");
                count_fail += 1;
                print_frame_short("SS Raw sense frame =", array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW SENSE MIN MAX TEST:.................OK\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW SENSE MIN MAX TEST:.................SKIPPED\n");

        TPD_DEBUG("SS RAW SENSE MAP MIN MAX TEST:\n");
        if (todo->SelfSenseRawMap == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_SENSE_EACH_NODE_MIN, &thresholds_min, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_SENSE_EACH_NODE_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_SENSE_EACH_NODE_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_SENSE_EACH_NODE_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(ssRawFrame.sense_data, rows, columns, thresholds_min, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax SS RAW SENSE MAP failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS RAW SENSE MAP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw sense frame =", array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW SENSE MAP MIN MAX TEST:.................OK\n\n");

            if (thresholds_min != NULL) {
                kfree(thresholds_min);
                thresholds_min = NULL;
            }
            if (thresholds_max != NULL) {
                kfree(thresholds_max);
                thresholds_max = NULL;
            }
        } else
            TPD_DEBUG("SS RAW SENSE MAP MIN MAX TEST:.................SKIPPED\n\n");


        TPD_DEBUG("SS RAW SENSE GAP TEST:\n");
        if (todo->SelfSenseRawGap == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_SENSE_GAP, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_SENSE_GAP failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsGap(ssRawFrame.sense_data, rows, columns, thresholds[0]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsGap SS RAW SENSE GAP failed... ERROR = %08X\n", ret);
                TPD_DEBUG("SS RAW SENSE GAP TEST:.................FAIL\n");
                count_fail += 1;
                print_frame_short("SS Raw sense frame =", array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW SENSE GAP TEST:.................OK\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW SENSE GAP TEST:.................SKIPPED\n");

        kfree(ssRawFrame.sense_data);
        ssRawFrame.sense_data = NULL;
    } else
        TPD_DEBUG("SS RAW SENSE TEST:.................SKIPPED\n\n");

    ret = production_test_ss_raw_lp(path_limits, stop_on_fail, todo);
    if (ret < OK) {
        TPD_INFO("production_test_data: production_test_ss_raw_lp failed... ERROR = %08X\n", ret);
        count_fail += 1;
    }

    if (count_fail == 0) {
        TPD_DEBUG("SS RAW testes finished!.................OK\n\n");
        return OK;
    } else {
        TPD_DEBUG("SS RAW testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
        return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
    }

ERROR_LIMITS:
    if (ssRawFrame.force_data != NULL)
        kfree(ssRawFrame.force_data);
    if (ssRawFrame.sense_data != NULL)
        kfree(ssRawFrame.sense_data);
    if (thresholds != NULL)
        kfree(thresholds);
    if (thresholds_min != NULL)
        kfree(thresholds_min);
    if (thresholds_max != NULL)
        kfree(thresholds_max);

    return ret;
}


/**
  * Perform all the test selected in a TestTodo variable related to SS raw data
  * low power
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  *  otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_raw_lp(char *path_limits, int stop_on_fail,
                  TestToDo *todo)
{
    int ret;
    int count_fail = 0;
    int rows, columns;

    SelfSenseFrame ssRawFrame;

    int *thresholds = NULL;
    int trows, tcolumns;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;

    ssRawFrame.sense_data = NULL;
    ssRawFrame.force_data = NULL;
    /* SS TEST */

    TPD_DEBUG("SS RAW LP Testes are starting...\n");

    /************** Self Sense Test **************/

    TPD_DEBUG("Getting SS LP Frame...\n");
    ret = setScanMode(SCAN_MODE_LOCKED, LOCKED_LP_DETECT);
    msleep(WAIT_FOR_FRESH_FRAMES);
    ret |= setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    ret |= getSSFrame3(SS_DETECT_RAW, &ssRawFrame);
    if (ret < 0) {
        TPD_INFO("production_test_data: getSSFrame failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    /* SS RAW (PROXIMITY) FORCE TEST */
    TPD_DEBUG("SS RAW LP FORCE TEST:\n");



    if ((todo->SelfForceRawLP == 1 || todo->SelfForceRawGapLP == 1 || todo->SelfForceRawMapLP == 1) &&
        ssRawFrame.header.force_node != 0) {
        columns = 1;    /* there are no data for the sense channels
                  *  because is a force frame */
        rows = ssRawFrame.header.force_node;

        TPD_DEBUG("SS RAW LP FORCE MIN MAX TEST:\n");
        if (todo->SelfForceRawLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
            if (ret < 0 || (trows != 1 || tcolumns != 2)) {
                TPD_INFO(" production_test_data: parseProductionTestLimits SS_RAW_FORCE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMinMax(ssRawFrame.force_data, rows,columns, thresholds[0], thresholds[1]);
            if (ret != OK) {
                TPD_INFO(" production_test_data: checkLimitsMinMax SS RAW FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS RAW LP FORCE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw LP force frame =", array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW LP FORCE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW LP FORCE MIN MAX TEST:.................SKIPPED\n\n");

        TPD_DEBUG("SS RAW LP FORCE MAP MIN MAX TEST:\n");
        if (todo->SelfForceRawMapLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_FORCE_EACH_NODE_MIN, &thresholds_min, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_LP_FORCE_EACH_NODE_MIN failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_FORCE_EACH_NODE_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_LP_FORCE_EACH_NODE_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(ssRawFrame.force_data, rows, columns, thresholds_min, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax SS RAW LP FORCE MAP failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS RAW LP FORCE MAP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw LP force frame =",
                          array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW FORCE LP MAP MIN MAX TEST:.................OK\n\n");

            if (thresholds_min != NULL) {
                kfree(thresholds_min);
                thresholds_min = NULL;
            }
            if (thresholds_max != NULL) {
                kfree(thresholds_max);
                thresholds_max = NULL;
            }
        } else
            TPD_DEBUG("SS RAW FORCE LP MAP MIN MAX TEST:.................SKIPPED\n\n");


        TPD_DEBUG("SS RAW LP FORCE GAP TEST:\n");
        if (todo->SelfForceRawGapLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_FORCE_GAP, &thresholds, &trows, &tcolumns);
            if (ret < OK || (trows != 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_FORCE_GAP failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsGap(ssRawFrame.force_data, rows, columns, thresholds[0]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsGap SS RAW FORCE GAP failed... ERROR = %08X\n", ret);
                TPD_DEBUG("SS RAW LP FORCE GAP TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw LP force frame =", array1dTo2d_short(ssRawFrame.force_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW LP FORCE GAP TEST:.................OK\n\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW LP FORCE GAP TEST:.................SKIPPED\n\n");

        kfree(ssRawFrame.force_data);
        ssRawFrame.force_data = NULL;
    } else
        TPD_DEBUG("SS RAW LP FORCE TEST:.................SKIPPED\n\n");


    /* SS RAW (PROXIMITY) SENSE TEST */
    TPD_DEBUG("SS RAW LP SENSE TEST:\n");

    if ((todo->SelfSenseRawLP == 1 || todo->SelfSenseRawGapLP == 1 || todo->SelfSenseRawMapLP == 1) && ssRawFrame.header.sense_node != 0) {
        columns = ssRawFrame.header.sense_node;
        rows = 1; /* there are no data for the force channels
              * because is a sense frame */

        TPD_DEBUG("SS RAW LP SENSE MIN MAX TEST:\n");
        if (todo->SelfSenseRawLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
            if (ret < OK || (trows != 1 || tcolumns != 2)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_LP_SENSE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMinMax(ssRawFrame.sense_data, rows, columns, thresholds[0], thresholds[1]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax SS RAW LP SENSE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS RAW LP SENSE MIN MAX TEST:.................FAIL\n");
                count_fail += 1;
                print_frame_short("SS Raw LP sense frame =", array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW SENSE MIN MAX TEST:.................OK\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW LP SENSE MIN MAX TEST:.................SKIPPED\n");

        TPD_DEBUG("SS RAW LP SENSE MAP MIN MAX TEST:\n");
        if (todo->SelfSenseRawMapLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_SENSE_EACH_NODE_MIN, &thresholds_min, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_LP_SENSE_EACH_NODE_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_SENSE_EACH_NODE_MAX, &thresholds_max, &trows, &tcolumns);
            if (ret < OK || (trows != rows || tcolumns != columns)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_LP_SENSE_EACH_NODE_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(ssRawFrame.sense_data, rows, columns, thresholds_min, thresholds_max);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMinMax SS RAW LP SENSE MAP failed... ERROR COUNT = %d\n",ret);
                TPD_DEBUG("SS RAW LP SENSE MAP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                print_frame_short("SS Raw LP sense frame =", array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW LP SENSE MAP MIN MAX TEST:.................OK\n\n");

            if (thresholds_min != NULL) {
                kfree(thresholds_min);
                thresholds_min = NULL;
            }
            if (thresholds_max != NULL) {
                kfree(thresholds_max);
                thresholds_max = NULL;
            }
        } else
            TPD_DEBUG("SS RAW LP SENSE MAP MIN MAX TEST:.................SKIPPED\n\n");


        TPD_DEBUG("SS RAW LP SENSE GAP TEST:\n");
        if (todo->SelfSenseRawGapLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_RAW_LP_SENSE_GAP, &thresholds, &trows, &tcolumns);
            if (ret < OK || (trows != 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_RAW_LP_SENSE_GAP failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsGap(ssRawFrame.sense_data, rows, columns, thresholds[0]);
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsGap SS RAW LP SENSE GAP failed... ERROR = %08X\n", ret);
                TPD_DEBUG("SS RAW LP SENSE GAP TEST:.................FAIL\n");
                count_fail += 1;
                print_frame_short("SS Raw LP sense frame =", array1dTo2d_short(ssRawFrame.sense_data, rows * columns, columns), rows, columns);
                if (stop_on_fail) {
                    ret = ERROR_PROD_TEST_DATA | ERROR_TEST_CHECK_FAIL;
                    goto ERROR_LIMITS;
                }
            } else
                TPD_DEBUG("SS RAW LP SENSE GAP TEST:.................OK\n");

            kfree(thresholds);
            thresholds = NULL;
        } else
            TPD_DEBUG("SS RAW LP SENSE GAP TEST:.................SKIPPED\n");

        kfree(ssRawFrame.sense_data);
        ssRawFrame.sense_data = NULL;
    } else
        TPD_DEBUG("SS RAW LP SENSE TEST:.................SKIPPED\n\n");


    if (count_fail == 0) {
        TPD_DEBUG("SS RAW LP testes finished!.................OK\n\n");
        return OK;
    } else {
        TPD_DEBUG("SS RAW LP testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
        return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
    }

ERROR_LIMITS:
    if (ssRawFrame.force_data != NULL)
        kfree(ssRawFrame.force_data);
    if (ssRawFrame.sense_data != NULL)
        kfree(ssRawFrame.sense_data);
    if (thresholds != NULL)
        kfree(thresholds);
    if (thresholds_min != NULL)
        kfree(thresholds_min);
    if (thresholds_max != NULL)
        kfree(thresholds_max);
    return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to SS Init
  * data (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_ix_cx(char *path_limits, int stop_on_fail,
                 TestToDo *todo)
{
    int ret;
    int count_fail = 0;

    int *thresholds = NULL;
    int trows, tcolumns;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;

    SelfSenseData ssCompData;
    TotSelfSenseData totCompData;

    u8 *adjhor = NULL;
    u8 *adjvert = NULL;

    short container;

    u16 *total_adjhor = NULL;
    u16 *total_adjvert = NULL;

    ssCompData.ix2_fm = NULL;
    ssCompData.ix2_sn = NULL;
    ssCompData.cx2_fm = NULL;
    ssCompData.cx2_sn = NULL;

    totCompData.ix_fm = NULL;
    totCompData.ix_sn = NULL;
    totCompData.cx_fm = NULL;
    totCompData.cx_sn = NULL;

    TPD_DEBUG("SS IX CX testes are starting...\n");
    ret = readSelfSenseCompensationData(LOAD_CX_SS_TOUCH, &ssCompData);
    /* read the SS compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readSelfSenseCompensationData failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    ret = readTotSelfSenseCompensationData(LOAD_PANEL_CX_TOT_SS_TOUCH, &totCompData);
    /* read the TOT SS compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readTotSelfSenseCompensationData failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        kfree(ssCompData.ix2_fm);
        kfree(ssCompData.ix2_sn);
        kfree(ssCompData.cx2_fm);
        kfree(ssCompData.cx2_sn);
        return ret | ERROR_PROD_TEST_DATA;
    }

    /************* SS FORCE IX **************/
    /* SS IX1 FORCE TEST */
    TPD_DEBUG("SS IX1 FORCE TEST:\n");
    if (todo->SelfForceIx1 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX1_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO(" production_test_data: parseProductionTestLimits SS_IX1_FORCE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        container = (short)ssCompData.f_ix1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);
        /* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS IX1 FORCE TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX1 FORCE TEST:.................OK\n\n");
    } else
        TPD_DEBUG(" SS IX1 FORCE TEST:.................SKIPPED\n\n");

    kfree(thresholds);
    thresholds = NULL;
    /* SS IX2 FORCE TEST */
    TPD_DEBUG("SS IX2 FORCE MIN MAX TEST:\n");
    if (todo->SelfForceIx2 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node ||
                tcolumns != 1)) {
            TPD_INFO(
                 "production_test_data: parseProductionTestLimits SS_IX2_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_FORCE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapFromU(ssCompData.ix2_fm, ssCompData.header.force_node, 1, thresholds_min,thresholds_max);    /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS IX2 FORCE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 FORCE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS IX2 FORCE MIN MAX TEST:.................SKIPPED\n\n");

    TPD_DEBUG("SS IX2 FORCE ADJ TEST:\n");
    if (todo->SelfForceIx2Adj == 1) {
        /* SS IX2 FORCE ADJV TEST */
        TPD_DEBUG("SS IX2 FORCE ADJVERT TEST:\n");
        ret = computeAdjVertFromU(ssCompData.ix2_fm, ssCompData.header.force_node, 1, &adjvert);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjVert SS IX2 FORCE ADJV failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS IX2 FORCE ADJV computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);    /* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjvert, ssCompData.header.force_node - 1, 1, thresholds_max);    /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS IX2 FORCE ADJV TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 FORCE ADJV TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjvert);
        adjvert = NULL;
    } else
        TPD_DEBUG("SS IX2 FORCE ADJ TEST:.................SKIPPED\n\n");

    /* SS TOTAL FORCE IX */
    TPD_DEBUG("SS TOTAL IX FORCE TEST:\n");
    if (todo->SelfForceIxTotal == 1 || todo->SelfForceIxTotalAdj == 1) {
        TPD_DEBUG("SS TOTAL IX FORCE MIN MAX TEST:\n");
        if (todo->SelfForceIxTotal == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_FORCE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotalFromU(totCompData.ix_fm, totCompData.header.force_node, 1, thresholds_min, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap  SS TOTAL IX FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX FORCE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX FORCE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX FORCE MIN MAX TEST:.................SKIPPED\n");

        TPD_DEBUG("SS TOTAL IX FORCE ADJ TEST:\n");
        if (todo->SelfForceIxTotalAdj == 1) {
            /* SS TOTAL IX FORCE ADJV TEST */
            TPD_DEBUG("SS TOTAL IX FORCE ADJVERT TEST:\n");
            ret = computeAdjVertTotalFromU(totCompData.ix_fm, totCompData.header.force_node, 1, &total_adjvert);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjVert SS TOTAL IX FORCE ADJV failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL IX FORCE ADJV computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node - 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_FORCE_ADJV_MAP_MAX... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjvert, totCompData.header.force_node - 1, 1, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL IX FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX FORCE ADJV TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX FORCE ADJV TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjvert);
            total_adjvert = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX FORCE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL IX FORCE TEST:.................SKIPPED\n\n");


    /************** SS SENSE IX **************/
    /* SS IX1 SENSE TEST */
    TPD_DEBUG("SS IX1 SENSE TEST:\n");
    if (todo->SelfSenseIx1 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX1_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX1_SENSE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (short)ssCompData.s_ix1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS IX1 SENSE TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX1 SENSE TEST:.................OK\n\n");
    } else
        TPD_DEBUG("SS IX1 SENSE TEST:.................SKIPPED\n\n");

    kfree(thresholds);
    thresholds = NULL;
    /* SS IX2 SENSE TEST */
    TPD_DEBUG("SS IX2 SENSE MIN MAX TEST:\n");
    if (todo->SelfSenseIx2 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapFromU(ssCompData.ix2_sn, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 SENSE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS IX2 SENSE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 SENSE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS IX2 SENSE MIN MAX TEST:.................SKIPPED\n\n");

    TPD_DEBUG("SS IX2 SENSE ADJ TEST:\n");
    if (todo->SelfSenseIx2Adj == 1) {
        /* SS IX2 SENSE ADJH TEST */
        TPD_DEBUG("SS IX2 SENSE ADJHORIZ TEST:\n");
        ret = computeAdjHorizFromU(ssCompData.ix2_sn, 1, ssCompData.header.sense_node, &adjhor);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjHoriz SS IX2 SENSE ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS IX2 SENSE ADJ HORIZ computed!\n");


        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node - 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj SS IX2 SENSE ADJH failed... ERROR COUNT = %d\n",ret);
            TPD_DEBUG("SS IX2 SENSE ADJH TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 SENSE ADJH TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjhor);
        adjhor = NULL;
    } else
        TPD_DEBUG("SS IX2 SENSE ADJ TEST:.................SKIPPED\n");

    /* SS TOTAL IX SENSE */
    TPD_DEBUG("SS TOTAL IX SENSE TEST:\n");
    if (todo->SelfSenseIxTotal == 1 || todo->SelfSenseIxTotalAdj == 1) {
        TPD_DEBUG("SS TOTAL IX SENSE MIN MAX TEST:\n");
        if (todo->SelfSenseIxTotal == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotalFromU(totCompData.ix_sn, 1, totCompData.header.sense_node, thresholds_min, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL IX SENSE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX SENSE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX SENSE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX SENSE MIN MAX TEST:.................SKIPPED\n");


        TPD_DEBUG("SS TOTAL IX SENSE ADJ TEST:\n");
        if (todo->SelfSenseIxTotalAdj == 1) {
            /* SS TOTAL IX SENSE ADJH TEST */
            TPD_DEBUG("SS TOTAL IX SENSE ADJHORIZ TEST:\n");
            ret = computeAdjHorizTotalFromU(totCompData.ix_sn, 1, totCompData.header.sense_node, &total_adjhor);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjHoriz SS TOTAL IX SENSE ADJH failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL IX SENSE ADJ HORIZ computed!\n");


            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node - 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjhor, 1, totCompData.header.sense_node - 1, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj SS TOTAL IX SENSE ADJH failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX SENSE ADJH TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX SENSE ADJH TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjhor);
            total_adjhor = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX SENSE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL IX SENSE TEST:.................SKIPPED\n");

    /************* SS SENSE CX **************/
    /* SS CX1 FORCE TEST */
    TPD_DEBUG("SS CX1 FORCE TEST:\n");
    if (todo->SelfForceCx1 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX1_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX1_FORCE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (short)ssCompData.f_cx1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS CX1 FORCE TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX1 FORCE TEST:.................OK\n\n");
        kfree(thresholds);
        thresholds = NULL;
    } else
        TPD_DEBUG("SS CX1 FORCE TEST:.................SKIPPED\n\n");

    /* SS CX2 FORCE TEST */
    TPD_DEBUG("SS CX2 FORCE MIN MAX TEST:\n");
    if (todo->SelfForceCx2 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_FORCE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMap(ssCompData.cx2_fm, ssCompData.header.force_node, 1, thresholds_min, thresholds_max); /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS CX2 FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 FORCE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 FORCE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS CX2 FORCE MIN MAX TEST:.................SKIPPED\n");

    TPD_DEBUG("SS CX2 FORCE ADJ TEST:\n");
    if (todo->SelfForceCx2Adj == 1) {
        /* SS CX2 FORCE ADJV TEST */
        TPD_DEBUG("SS CX2 FORCE ADJVERT TEST:\n");
        ret = computeAdjVert(ssCompData.cx2_fm, ssCompData.header.force_node, 1, &adjvert);/* compute the ADJV for CX2  FORCE */
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjVert SS CX2 FORCE ADJV failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS CX2 FORCE ADJV computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjvert, ssCompData.header.force_node - 1, 1, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 FORCE ADJV TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 FORCE ADJV TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjvert);
        adjvert = NULL;
    } else
        TPD_DEBUG("SS CX2 FORCE ADJ TEST:.................SKIPPED\n\n");

    /* SS TOTAL CX FORCE */
    TPD_DEBUG("SS TOTAL CX FORCE TEST:\n");
    if (todo->SelfForceCxTotal == 1 || todo->SelfForceCxTotalAdj == 1) {
        TPD_DEBUG("SS TOTAL CX FORCE MIN MAX TEST:\n");
        if (todo->SelfForceCxTotal == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_FORCE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(totCompData.cx_fm, totCompData.header.force_node, 1, thresholds_min, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL FORCE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL FORCE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX FORCE MIN MAX TEST:.................SKIPPED\n");

        /* SS TOTAL CX FORCE ADJV TEST */
        TPD_DEBUG("SS TOTAL CX FORCE ADJ TEST:\n");
        if (todo->SelfForceCxTotalAdj == 1) {
            TPD_DEBUG("SS TOTAL CX FORCE ADJVERT TEST:\n");
            ret = computeAdjVertTotal(totCompData.cx_fm, totCompData.header.force_node, 1, &total_adjvert);/* compute the ADJV for CX2  FORCE */
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjVert SS TOTAL CX FORCE ADJV failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL CX FORCE ADJV computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node - 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjvert, totCompData.header.force_node - 1, 1,thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL CX FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL CX FORCE ADJV TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL CX FORCE ADJV TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjvert);
            total_adjvert = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX FORCE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL CX FORCE TEST:.................SKIPPED\n\n");



    /************* SS SENSE CX *************/
    /* SS CX1 SENSE TEST */
    TPD_DEBUG("SS CX1 SENSE TEST:\n");
    if (todo->SelfSenseCx1 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX1_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX1_SENSE_MIN_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (short)ssCompData.s_cx1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS CX1 SENSE TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX1 SENSE TEST:.................OK\n\n");
        kfree(thresholds);
        thresholds = NULL;
    } else
        TPD_DEBUG("SS CX1 SENSE TEST:.................SKIPPED\n\n");

    /* SS CX2 SENSE TEST */
    TPD_DEBUG("SS CX2 SENSE MIN MAX TEST:\n");
    if (todo->SelfSenseCx2 == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMap(ssCompData.cx2_sn, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max); /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS CX2 SENSE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 SENSE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 SENSE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS CX2 SENSE MIN MAX TEST:.................SKIPPED\n");

    TPD_DEBUG("SS CX2 SENSE ADJ TEST:\n");
    if (todo->SelfSenseCx2Adj == 1) {
        /* SS CX2 SENSE ADJH TEST */
        TPD_DEBUG("SS CX2 SENSE ADJHORIZ TEST:\n");
        ret = computeAdjHoriz(ssCompData.cx2_sn, 1, ssCompData.header.sense_node, &adjhor);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjHoriz SS CX2 SENSE ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS CX2 SENSE ADJH computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node - 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max); /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj SS CX2 SENSE ADJH failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 SENSE ADJH TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 SENSE ADJH TEST:.................OK\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjhor);
        adjhor = NULL;
    } else
        TPD_DEBUG("SS CX2 SENSE ADJ TEST:.................SKIPPED\n\n");

    /* SS TOTAL CX SENSE */
    TPD_DEBUG("SS TOTAL CX SENSE TEST:\n");
    if (todo->SelfSenseCxTotal == 1 || todo->SelfSenseCxTotalAdj == 1) {
        TPD_DEBUG("SS TOTAL CX SENSE MIN MAX TEST:\n");
        if (todo->SelfSenseCxTotal == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /* load the min thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(totCompData.cx_sn, 1, totCompData.header.sense_node, thresholds_min, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL CX SENSE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL CX SENSE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL CX SENSE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX SENSE MIN MAX TEST:.................SKIPPED\n");

        /* SS TOTAL IX SENSE ADJH TEST */
        TPD_DEBUG("SS TOTAL CX SENSE ADJ TEST:\n");
        if (todo->SelfSenseCxTotalAdj == 1) {
            TPD_DEBUG("SS TOTAL CX SENSE ADJHORIZ TEST:\n");
            ret = computeAdjHorizTotal(totCompData.cx_sn, 1, totCompData.header.sense_node, &total_adjhor);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjHoriz SS TOTAL CX SENSE ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL CX SENSE ADJ HORIZ computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node - 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjhor, 1, totCompData.header.sense_node - 1,thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj SS TOTAL CX SENSE ADJH failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL CX SENSE ADJH TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL CX SENSE ADJH TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjhor);
            total_adjhor = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX SENSE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL CX SENSE TEST:.................SKIPPED\n");


    if ((todo->SelfSenseCx1LP | todo->SelfSenseCx2LP |
        todo->SelfSenseCx2AdjLP | todo->SelfSenseCxTotalLP |
        todo->SelfSenseCxTotalAdjLP | todo->SelfSenseIx1LP |
        todo->SelfSenseIx2LP | todo->SelfSenseIx2AdjLP |
        todo->SelfSenseIxTotalLP | todo->SelfSenseIxTotalAdjLP) == 1) {
        ret = production_test_ss_ix_cx_lp(path_limits, stop_on_fail, todo);
        if (ret < OK) {
            count_fail += 1;
            TPD_INFO("production_test_data: production_test_ss_ix_cx_lp failed... ERROR = %08X\n", ret);
            goto ERROR;
        }
    } else
        TPD_DEBUG("SS IX CX LP TEST:.................SKIPPED\n");

ERROR:

    if (count_fail == 0) {
        kfree(ssCompData.ix2_fm);
        ssCompData.ix2_fm = NULL;
        kfree(ssCompData.ix2_sn);
        ssCompData.ix2_sn = NULL;
        kfree(ssCompData.cx2_fm);
        ssCompData.cx2_fm = NULL;
        kfree(ssCompData.cx2_sn);
        ssCompData.cx2_sn = NULL;
        kfree(totCompData.ix_fm);
        totCompData.ix_fm = NULL;
        kfree(totCompData.ix_sn);
        totCompData.ix_sn = NULL;
        kfree(totCompData.cx_fm);
        totCompData.cx_fm = NULL;
        kfree(totCompData.cx_sn);
        totCompData.cx_sn = NULL;
        TPD_DEBUG("SS IX CX testes finished!.................OK\n\n");
        return OK;
    } else {
    /* print all kind of data in just one row for readability reason */
        print_frame_u8("SS Init Data Ix2_fm = ", array1dTo2d_u8(ssCompData.ix2_fm, ssCompData.header.force_node, 1), ssCompData.header.force_node, 1);
        print_frame_i8("SS Init Data Cx2_fm = ", array1dTo2d_i8(ssCompData.cx2_fm, ssCompData.header.force_node, 1), ssCompData.header.force_node, 1);
        print_frame_u8("SS Init Data Ix2_sn = ", array1dTo2d_u8(ssCompData.ix2_sn, ssCompData.header.sense_node, ssCompData.header.sense_node), 1, ssCompData.header.sense_node);
        print_frame_i8("SS Init Data Cx2_sn = ", array1dTo2d_i8(ssCompData.cx2_sn, ssCompData.header.sense_node, ssCompData.header.sense_node), 1, ssCompData.header.sense_node);
        print_frame_u16("TOT SS Init Data Ix_fm = ", array1dTo2d_u16(totCompData.ix_fm, totCompData.header.force_node, 1), totCompData.header.force_node, 1);
        print_frame_short("TOT SS Init Data Cx_fm = ", array1dTo2d_short(totCompData.cx_fm, totCompData.header.force_node, 1), totCompData.header.force_node, 1);
        print_frame_u16("TOT SS Init Data Ix_sn = ", array1dTo2d_u16(totCompData.ix_sn, totCompData.header.sense_node, totCompData.header.sense_node), 1, totCompData.header.sense_node);
        print_frame_short("TOT SS Init Data Cx_sn = ", array1dTo2d_short(totCompData.cx_sn, totCompData.header.sense_node, totCompData.header.sense_node), 1, totCompData.header.sense_node);
        TPD_DEBUG("SS IX CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
        if (thresholds != NULL)
            kfree(thresholds);
        if (thresholds_min != NULL)
            kfree(thresholds_min);
        if (thresholds_max != NULL)
            kfree(thresholds_max);
        if (adjhor != NULL)
            kfree(adjhor);
        if (adjvert != NULL)
            kfree(adjvert);
        if (total_adjhor != NULL)
            kfree(total_adjhor);
        if (total_adjvert != NULL)
            kfree(total_adjvert);
        if (ssCompData.ix2_fm != NULL)
            kfree(ssCompData.ix2_fm);
        if (ssCompData.ix2_sn != NULL)
            kfree(ssCompData.ix2_sn);
        if (ssCompData.cx2_fm != NULL)
            kfree(ssCompData.cx2_fm);
        if (ssCompData.cx2_sn != NULL)
            kfree(ssCompData.cx2_sn);
        if (totCompData.ix_fm != NULL)
            kfree(totCompData.ix_fm);
        if (totCompData.ix_sn != NULL)
            kfree(totCompData.ix_sn);
        if (totCompData.cx_fm != NULL)
            kfree(totCompData.cx_fm);
        if (totCompData.cx_sn != NULL)
            kfree(totCompData.cx_sn);
        return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
    }

ERROR_LIMITS:
    if (thresholds != NULL)
        kfree(thresholds);
    if (thresholds_min != NULL)
        kfree(thresholds_min);
    if (thresholds_max != NULL)
        kfree(thresholds_max);
    if (adjhor != NULL)
        kfree(adjhor);
    if (adjvert != NULL)
        kfree(adjvert);
    if (total_adjhor != NULL)
        kfree(total_adjhor);
    if (total_adjvert != NULL)
        kfree(total_adjvert);
    if (ssCompData.ix2_fm != NULL)
        kfree(ssCompData.ix2_fm);
    if (ssCompData.ix2_sn != NULL)
        kfree(ssCompData.ix2_sn);
    if (ssCompData.cx2_fm != NULL)
        kfree(ssCompData.cx2_fm);
    if (ssCompData.cx2_sn != NULL)
        kfree(ssCompData.cx2_sn);
    if (totCompData.ix_fm != NULL)
        kfree(totCompData.ix_fm);
    if (totCompData.ix_sn != NULL)
        kfree(totCompData.ix_sn);
    if (totCompData.cx_fm != NULL)
        kfree(totCompData.cx_fm);
    if (totCompData.cx_sn != NULL)
        kfree(totCompData.cx_sn);
    return ret;
}

/**
  * Perform all the tests selected in a TestTodo variable related to SS Init
  * data for LP mode (touch, keys etc..)
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure
  * otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_ss_ix_cx_lp(char *path_limits, int stop_on_fail,
                 TestToDo *todo)
{
    int ret;
    int count_fail = 0;

    int *thresholds = NULL;
    int trows, tcolumns;
    int *thresholds_min = NULL;
    int *thresholds_max = NULL;

    SelfSenseData ssCompData;
    TotSelfSenseData totCompData;

    u8 *adjhor = NULL;
    u8 *adjvert = NULL;

    short container;

    u16 *total_adjhor = NULL;
    u16 *total_adjvert = NULL;

    ssCompData.ix2_fm = NULL;
    ssCompData.ix2_sn = NULL;
    ssCompData.cx2_fm = NULL;
    ssCompData.cx2_sn = NULL;

    totCompData.ix_fm = NULL;
    totCompData.ix_sn = NULL;
    totCompData.cx_fm = NULL;
    totCompData.cx_sn = NULL;

    TPD_DEBUG("SS LP IX CX testes are starting...\n");
    ret = readSelfSenseCompensationData(LOAD_CX_SS_TOUCH_IDLE, &ssCompData);/* read the SS LP compensation data */
    if (ret < 0) {
        TPD_INFO("production_test_data: readSelfSenseCompensationData failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        return ret | ERROR_PROD_TEST_DATA;
    }

    ret = readTotSelfSenseCompensationData(LOAD_PANEL_CX_TOT_SS_TOUCH_IDLE, &totCompData);/* read the TOT SS LP compensation data */
    if (ret < 0) {
        TPD_INFO(" production_test_data: readTotSelfSenseCompensationData failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
        kfree(ssCompData.ix2_fm);
        kfree(ssCompData.ix2_sn);
        kfree(ssCompData.cx2_fm);
        kfree(ssCompData.cx2_sn);
        return ret | ERROR_PROD_TEST_DATA;
    }

    /************* SS FORCE IX LP **************/
    /* SS IX1 LP FORCE TEST */
    TPD_DEBUG("SS IX1 LP FORCE TEST:\n");
    if (todo->SelfForceIx1LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX1_LP_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX1_LP_FORCE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        container = (short)ssCompData.f_ix1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS IX1 LP FORCE TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX1 LP FORCE TEST:.................OK\n\n");
    } else
        TPD_DEBUG("SS IX1 LP FORCE TEST:.................SKIPPED\n\n");

    kfree(thresholds);
    thresholds = NULL;
    /* SS IX2 LP FORCE TEST */
    TPD_DEBUG("SS IX2 LP FORCE MIN MAX TEST:\n");
    if (todo->SelfForceIx2LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_LP_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /* load the min thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_LP_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_LP_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_LP_FORCE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapFromU(ssCompData.ix2_fm, ssCompData.header.force_node, 1, thresholds_min, thresholds_max);    /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 LP FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS IX2 LP FORCE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 LP FORCE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS IX2 LP FORCE MIN MAX TEST:.................SKIPPED\n\n");

    TPD_DEBUG("SS IX2 LP FORCE ADJ TEST:\n");
    if (todo->SelfForceIx2AdjLP == 1) {
        /* SS IX2 FORCE ADJV TEST */
        TPD_DEBUG("SS IX2 LP FORCE ADJVERT TEST:\n");
        ret = computeAdjVertFromU(ssCompData.ix2_fm, ssCompData.header.force_node, 1, &adjvert);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjVert SS IX2 LP FORCE ADJV failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS IX2 LP FORCE ADJV computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_LP_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);    /* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_LP_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjvert, ssCompData.header.force_node - 1, 1, thresholds_max); /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 LP FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS IX2 LP FORCE ADJV TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 LP FORCE ADJV TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjvert);
        adjvert = NULL;
    } else
        TPD_DEBUG("SS IX2 LP FORCE ADJ TEST:.................SKIPPED\n\n");

    /* SS TOTAL FORCE IX */
    TPD_DEBUG("SS TOTAL IX LP FORCE TEST:\n");
    if (todo->SelfForceIxTotalLP == 1 || todo->SelfForceIxTotalAdjLP == 1) {
        TPD_DEBUG("SS TOTAL IX LP FORCE MIN MAX TEST:\n");
        if (todo->SelfForceIxTotalLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_LP_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /* load the min thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_LP_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_LP_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_LP_FORCE_MAP_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotalFromU(totCompData.ix_fm, totCompData.header.force_node, 1, thresholds_min, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap  SS TOTAL IX LP FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX LP FORCE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX LP FORCE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX LP FORCE MIN MAX TEST:.................SKIPPED\n");

        TPD_DEBUG("SS TOTAL IX LP FORCE ADJ TEST:\n");
        if (todo->SelfForceIxTotalAdjLP == 1) {
            /* SS TOTAL IX FORCE ADJV TEST */
            TPD_DEBUG("SS TOTAL IX LP FORCE ADJVERT TEST:\n");
            ret = computeAdjVertTotalFromU(totCompData.ix_fm, totCompData.header.force_node, 1, &total_adjvert);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjVert SS TOTAL IX LP FORCE ADJV failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL IX LP FORCE ADJV computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_LP_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node - 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_LP_FORCE_ADJV_MAP_MAX... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjvert, totCompData.header.force_node - 1, 1, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL IX LP FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX LP FORCE ADJV TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX LP FORCE ADJV TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjvert);
            total_adjvert = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX LP FORCE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL IX LP FORCE TEST:.................SKIPPED\n\n");


    /************** SS SENSE IX LP **************/
    /* SS IX1 LP SENSE TEST */
    TPD_DEBUG("SS IX1 LP SENSE TEST:\n");
    if (todo->SelfSenseIx1LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX1_LP_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX1_LP_SENSE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (short)ssCompData.s_ix1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS IX1 SENSE LP TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX1 LP SENSE TEST:.................OK\n\n");
    } else
        TPD_DEBUG("SS IX1 LP SENSE TEST:.................SKIPPED\n\n");

    kfree(thresholds);
    thresholds = NULL;
    /* SS IX2 SENSE TEST */
    TPD_DEBUG("SS IX2 LP SENSE MIN MAX TEST:\n");
    if (todo->SelfSenseIx2LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_LP_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_LP_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_LP_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_LP_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapFromU(ssCompData.ix2_sn, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 LP SENSE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS IX2 LP SENSE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 LP SENSE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS IX2 LP SENSE MIN MAX TEST:.................SKIPPED\n\n");

    TPD_DEBUG("SS IX2 LP SENSE ADJ TEST:\n");
    if (todo->SelfSenseIx2AdjLP == 1) {
        /* SS IX2 SENSE ADJH TEST */
        TPD_DEBUG("SS IX2 SENSE ADJHORIZ TEST:\n");
        ret = computeAdjHorizFromU(ssCompData.ix2_sn, 1, ssCompData.header.sense_node,&adjhor);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjHoriz SS IX2 SENSE ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS IX2 SENSE ADJ HORIZ computed!\n");


        ret = parseProductionTestLimits(path_limits, &limit_file, SS_IX2_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node - 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_LP_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max); /* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj SS IX2 LP SENSE ADJH failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS IX2 LP SENSE ADJH TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS IX2 LP SENSE ADJH TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjhor);
        adjhor = NULL;
    } else
        TPD_DEBUG("SS IX2 LP SENSE ADJ TEST:.................SKIPPED\n");

    /* SS TOTAL IX SENSE */
    TPD_DEBUG("SS TOTAL IX LP SENSE TEST:\n");
    if (todo->SelfSenseIxTotalLP == 1 || todo->SelfSenseIxTotalAdjLP == 1) {
        TPD_DEBUG("SS TOTAL IX LP SENSE MIN MAX TEST:\n");
        if (todo->SelfSenseIxTotalLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_LP_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_LP_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_LP_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_LP_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotalFromU(totCompData.ix_sn, 1, totCompData.header.sense_node, thresholds_min, thresholds_max); /* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL IX LP SENSE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX LP SENSE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX LP SENSE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX LP SENSE MIN MAX TEST:.................SKIPPED\n");


        TPD_DEBUG("SS TOTAL IX LP SENSE ADJ TEST:\n");
        if (todo->SelfSenseIxTotalAdjLP == 1) {
            /* SS TOTAL IX SENSE ADJH TEST */
            TPD_DEBUG("SS TOTAL IX LP SENSE ADJHORIZ TEST:\n");
            ret = computeAdjHorizTotalFromU(totCompData.ix_sn, 1, totCompData.header.sense_node, &total_adjhor);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjHoriz SS TOTAL IX LP SENSE ADJH failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL IX LP SENSE ADJ HORIZ computed!\n");


            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_IX_LP_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node - 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_IX_LP_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjhor, 1, totCompData.header.sense_node - 1, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj SS TOTAL IX LP SENSE ADJH failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL IX LP SENSE ADJH TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL IX LP SENSE ADJH TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjhor);
            total_adjhor = NULL;
        } else
            TPD_DEBUG("SS TOTAL IX LP SENSE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL IX LP SENSE TEST:.................SKIPPED\n");

    /************* SS SENSE CX LP **************/
    /* SS CX1 LP FORCE TEST */
    TPD_DEBUG("SS CX1 LP FORCE TEST:\n");
    if (todo->SelfForceCx1LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX1_LP_FORCE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX1_LP_FORCE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (short)ssCompData.f_cx1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS CX1 LP FORCE TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX1 LP FORCE TEST:.................OK\n\n");
        kfree(thresholds);
        thresholds = NULL;
    } else
        TPD_DEBUG("SS CX1 LP FORCE TEST:.................SKIPPED\n\n");

    /* SS CX2 LP FORCE TEST */
    TPD_DEBUG("SS CX2 LP FORCE MIN MAX TEST:\n");
    if (todo->SelfForceCx2LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_LP_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_LP_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_LP_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_LP_FORCE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMap(ssCompData.cx2_fm, ssCompData.header.force_node, 1, thresholds_min, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS CX2 LP FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 LP FORCE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 LP FORCE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS CX2 LP FORCE MIN MAX TEST:.................SKIPPED\n");

    TPD_DEBUG("SS CX2 LP FORCE ADJ TEST:\n");
    if (todo->SelfForceCx2AdjLP == 1) {
        /* SS CX2 FORCE ADJV TEST */
        TPD_DEBUG("SS CX2 LP FORCE ADJVERT TEST:\n");
        ret = computeAdjVert(ssCompData.cx2_fm, ssCompData.header.force_node, 1, &adjvert);/* compute the ADJV for CX2  FORCE */
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjVert SS CX2 LP FORCE ADJV failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS CX2 LP FORCE ADJV computed!\n");

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_LP_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
        if (ret < 0 || (trows != ssCompData.header.force_node - 1 || tcolumns != 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_LP_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjvert, ssCompData.header.force_node - 1, 1, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS IX2 LP FORCE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 LP FORCE ADJV TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 LP FORCE ADJV TEST:.................OK\n\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjvert);
        adjvert = NULL;
    } else
        TPD_DEBUG("SS CX2 LP FORCE ADJ TEST:.................SKIPPED\n\n");

    /* SS TOTAL CX LP FORCE */
    TPD_DEBUG("SS TOTAL CX LP FORCE TEST:\n");
    if (todo->SelfForceCxTotalLP == 1 || todo->SelfForceCxTotalAdjLP == 1) {
        TPD_DEBUG("SS TOTAL CX LP FORCE MIN MAX TEST:\n");
        if (todo->SelfForceCxTotalLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_LP_FORCE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /* load the min thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_LP_FORCE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_LP_FORCE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_LP_FORCE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(totCompData.cx_fm, totCompData.header.force_node, 1, thresholds_min, thresholds_max); /* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL LP FORCE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL FORCE LP MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL FORCE LP MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX LP FORCE MIN MAX TEST:.................SKIPPED\n");

        /* SS TOTAL CX LP FORCE ADJV TEST */
        TPD_DEBUG("SS TOTAL CX LP FORCE ADJ TEST:\n");
        if (todo->SelfForceCxTotalAdjLP == 1) {
            TPD_DEBUG("SS TOTAL CX LP FORCE ADJVERT TEST:\n");
            ret = computeAdjVertTotal(totCompData.cx_fm, totCompData.header.force_node, 1, &total_adjvert);/* compute the ADJV for CX2  FORCE */
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjVert SS TOTAL CX LP FORCE ADJV failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL CX LP FORCE ADJV computed!\n");

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_LP_FORCE_ADJV_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != totCompData.header.force_node - 1 || tcolumns != 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_LP_FORCE_ADJV_MAP_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjvert, totCompData.header.force_node - 1, 1, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL CX LP FORCE failed... ERROR COUNT = %d\n",ret);
                TPD_DEBUG("SS TOTAL CX LP FORCE ADJV TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL CX LP FORCE ADJV TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjvert);
            total_adjvert = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX LP FORCE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL CX LP FORCE TEST:.................SKIPPED\n\n");



    /************* SS SENSE CX *************/
    /* SS CX1 SENSE TEST */
    TPD_DEBUG("SS CX1 LP SENSE TEST:\n");
    if (todo->SelfSenseCx1LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX1_LP_SENSE_MIN_MAX, &thresholds, &trows, &tcolumns);
        if (ret < 0 || (trows != 1 || tcolumns != 2)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX1_LP_SENSE_MIN_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        container = (short)ssCompData.s_cx1;
        ret = checkLimitsMinMax(&container, 1, 1, thresholds[0], thresholds[1]);/* check the limits */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMinMax SS CX1 LP SENSE TEST failed... ERROR COUNT = %d\n", ret);
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX1 LP SENSE TEST:.................OK\n\n");
        kfree(thresholds);
        thresholds = NULL;
    } else
        TPD_DEBUG("SS CX1 LP SENSE TEST:.................SKIPPED\n\n");

    /* SS CX2 LP SENSE TEST */
    TPD_DEBUG("SS CX2 LP SENSE MIN MAX TEST:\n");
    if (todo->SelfSenseCx2LP == 1) {
        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_LP_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns);/* load the min thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_LP_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_LP_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns != ssCompData.header.sense_node)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_CX2_LP_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMap(ssCompData.cx2_sn, 1, ssCompData.header.sense_node, thresholds_min, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMap SS CX2 LP SENSE failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 LP SENSE MIN MAX TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 LP SENSE MIN MAX TEST:.................OK\n\n");

        kfree(thresholds_min);
        thresholds_min = NULL;
        kfree(thresholds_max);
        thresholds_max = NULL;
    } else
        TPD_DEBUG("SS CX2 LP SENSE MIN MAX TEST:.................SKIPPED\n");

    TPD_DEBUG("SS CX2 LP SENSE ADJ TEST:\n");
    if (todo->SelfSenseCx2AdjLP == 1) {
        /* SS CX2 SENSE ADJH TEST */
        TPD_DEBUG("SS CX2 LP SENSE ADJHORIZ TEST:\n");
        ret = computeAdjHoriz(ssCompData.cx2_sn, 1, ssCompData.header.sense_node, &adjhor);
        if (ret < 0) {
            TPD_INFO("production_test_data: computeAdjHoriz SS CX2 LP SENSE ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }
        TPD_DEBUG("SS CX2 LP SENSE ADJH computed!\n");


        ret = parseProductionTestLimits(path_limits, &limit_file, SS_CX2_LP_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
        if (ret < 0 || (trows != 1 || tcolumns !=
                ssCompData.header.sense_node - 1)) {
            TPD_INFO("production_test_data: parseProductionTestLimits SS_IX2_SENSE_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
            ret |= ERROR_PROD_TEST_DATA;
            goto ERROR_LIMITS;
        }

        ret = checkLimitsMapAdj(adjhor, 1, ssCompData.header.sense_node - 1, thresholds_max);/* check the values with thresholds */
        if (ret != OK) {
            TPD_INFO("production_test_data: checkLimitsMapAdj SS CX2 LP SENSE ADJH failed... ERROR COUNT = %d\n", ret);
            TPD_DEBUG("SS CX2 LP SENSE ADJH TEST:.................FAIL\n\n");
            count_fail += 1;
            if (stop_on_fail)
                goto ERROR;
        } else
            TPD_DEBUG("SS CX2 LP SENSE ADJH TEST:.................OK\n");

        kfree(thresholds_max);
        thresholds_max = NULL;
        kfree(adjhor);
        adjhor = NULL;
    } else
        TPD_DEBUG("SS CX2 LP SENSE ADJ TEST:.................SKIPPED\n\n");

    /* SS TOTAL CX SENSE */
    TPD_DEBUG("SS TOTAL CX LP SENSE TEST:\n");
    if (todo->SelfSenseCxTotalLP == 1 || todo->SelfSenseCxTotalAdjLP == 1) {
        TPD_DEBUG("SS TOTAL CX LP SENSE MIN MAX TEST:\n");
        if (todo->SelfSenseCxTotalLP == 1) {
            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_LP_SENSE_MAP_MIN, &thresholds_min, &trows, &tcolumns); /* load the min thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_LP_SENSE_MAP_MIN failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_LP_SENSE_MAP_MAX, &thresholds_max, &trows, &tcolumns);/* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_LP_SENSE_MAP_MAX failed... ERROR %08X\n",ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapTotal(totCompData.cx_sn, 1, totCompData.header.sense_node, thresholds_min, thresholds_max); /* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMap SS TOTAL CX LP SENSE failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG("SS TOTAL CX LP SENSE MIN MAX TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL CX LP SENSE MIN MAX TEST:.................OK\n\n");

            kfree(thresholds_min);
            thresholds_min = NULL;
            kfree(thresholds_max);
            thresholds_max = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX LP SENSE MIN MAX TEST:.................SKIPPED\n");


        /* SS TOTAL IX SENSE ADJH TEST */
        TPD_DEBUG("SS TOTAL CX LP SENSE ADJ TEST:\n");
        if (todo->SelfSenseCxTotalAdjLP == 1) {
            TPD_DEBUG("SS TOTAL CX LP SENSE ADJHORIZ TEST:\n");
            ret = computeAdjHorizTotal(totCompData.cx_sn, 1, totCompData.header.sense_node, &total_adjhor);
            if (ret < 0) {
                TPD_INFO("production_test_data: computeAdjHoriz SS TOTAL CX LP SENSE ADJH failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }
            TPD_DEBUG("SS TOTAL CX LP SENSE ADJ HORIZ computed!\n");


            ret = parseProductionTestLimits(path_limits, &limit_file, SS_TOTAL_CX_LP_SENSE_ADJH_MAP_MAX, &thresholds_max, &trows, &tcolumns); /* load the max thresholds */
            if (ret < 0 || (trows != 1 || tcolumns != totCompData.header.sense_node - 1)) {
                TPD_INFO("production_test_data: parseProductionTestLimits SS_TOTAL_CX_LP_SENSE_ADJH_MAP_MAX failed... ERROR %08X\n", ERROR_PROD_TEST_DATA);
                ret |= ERROR_PROD_TEST_DATA;
                goto ERROR_LIMITS;
            }

            ret = checkLimitsMapAdjTotal(total_adjhor, 1, totCompData.header.sense_node - 1, thresholds_max);/* check the values with thresholds */
            if (ret != OK) {
                TPD_INFO("production_test_data: checkLimitsMapAdj SS TOTAL CX LP SENSE ADJH failed... ERROR COUNT = %d\n", ret);
                TPD_DEBUG(" SS TOTAL CX LP SENSE ADJH TEST:.................FAIL\n\n");
                count_fail += 1;
                if (stop_on_fail)
                    goto ERROR;
            } else
                TPD_DEBUG("SS TOTAL CX LP SENSE ADJH TEST:.................OK\n\n");

            kfree(thresholds_max);
            thresholds_max = NULL;
            kfree(total_adjhor);
            total_adjhor = NULL;
        } else
            TPD_DEBUG("SS TOTAL CX LP SENSE ADJ TEST:.................SKIPPED\n");
    } else
        TPD_DEBUG("SS TOTAL CX LP SENSE TEST:.................SKIPPED\n");



ERROR:

    if (count_fail == 0) {
        kfree(ssCompData.ix2_fm);
        ssCompData.ix2_fm = NULL;
        kfree(ssCompData.ix2_sn);
        ssCompData.ix2_sn = NULL;
        kfree(ssCompData.cx2_fm);
        ssCompData.cx2_fm = NULL;
        kfree(ssCompData.cx2_sn);
        ssCompData.cx2_sn = NULL;
        kfree(totCompData.ix_fm);
        totCompData.ix_fm = NULL;
        kfree(totCompData.ix_sn);
        totCompData.ix_sn = NULL;
        kfree(totCompData.cx_fm);
        totCompData.cx_fm = NULL;
        kfree(totCompData.cx_sn);
        totCompData.cx_sn = NULL;
        TPD_DEBUG("SS LP IX CX  testes finished!.................OK\n\n");
        return OK;
    } else {
    /* print all kind of data in just one row for readability reason */
        print_frame_u8("SS LP Init Data Ix2_fm = ", array1dTo2d_u8(ssCompData.ix2_fm, ssCompData.header.force_node, 1), ssCompData.header.force_node, 1);
        print_frame_i8("SS LP Init Data Cx2_fm = ", array1dTo2d_i8(ssCompData.cx2_fm, ssCompData.header.force_node, 1), ssCompData.header.force_node, 1);
        print_frame_u8("SS LP Init Data Ix2_sn = ", array1dTo2d_u8(ssCompData.ix2_sn, ssCompData.header.sense_node, ssCompData.header.sense_node), 1, ssCompData.header.sense_node);
        print_frame_i8("SS LP Init Data Cx2_sn = ", array1dTo2d_i8(ssCompData.cx2_sn, ssCompData.header.sense_node, ssCompData.header.sense_node), 1, ssCompData.header.sense_node);
        print_frame_u16("TOT SS LP Init Data Ix_fm = ", array1dTo2d_u16(totCompData.ix_fm, totCompData.header.force_node, 1), totCompData.header.force_node, 1);
        print_frame_short("TOT SS LP Init Data Cx_fm = ", array1dTo2d_short(totCompData.cx_fm,totCompData.header.force_node, 1), totCompData.header.force_node, 1);
        print_frame_u16("TOT SS LP Init Data Ix_sn = ", array1dTo2d_u16(totCompData.ix_sn, totCompData.header.sense_node, totCompData.header.sense_node), 1, totCompData.header.sense_node);
        print_frame_short("TOT SS LP Init Data Cx_sn = ", array1dTo2d_short(totCompData.cx_sn, totCompData.header.sense_node, totCompData.header.sense_node), 1, totCompData.header.sense_node);
        TPD_DEBUG("SS LP IX CX testes finished!.................FAILED  fails_count = %d\n\n", count_fail);
        if (thresholds != NULL)
            kfree(thresholds);
        if (thresholds_min != NULL)
            kfree(thresholds_min);
        if (thresholds_max != NULL)
            kfree(thresholds_max);
        if (adjhor != NULL)
            kfree(adjhor);
        if (adjvert != NULL)
            kfree(adjvert);
        if (total_adjhor != NULL)
            kfree(total_adjhor);
        if (total_adjvert != NULL)
            kfree(total_adjvert);
        if (ssCompData.ix2_fm != NULL)
            kfree(ssCompData.ix2_fm);
        if (ssCompData.ix2_sn != NULL)
            kfree(ssCompData.ix2_sn);
        if (ssCompData.cx2_fm != NULL)
            kfree(ssCompData.cx2_fm);
        if (ssCompData.cx2_sn != NULL)
            kfree(ssCompData.cx2_sn);
        if (totCompData.ix_fm != NULL)
            kfree(totCompData.ix_fm);
        if (totCompData.ix_sn != NULL)
            kfree(totCompData.ix_sn);
        if (totCompData.cx_fm != NULL)
            kfree(totCompData.cx_fm);
        if (totCompData.cx_sn != NULL)
            kfree(totCompData.cx_sn);
        return ERROR_TEST_CHECK_FAIL | ERROR_PROD_TEST_DATA;
    }

ERROR_LIMITS:
    if (thresholds != NULL)
        kfree(thresholds);
    if (thresholds_min != NULL)
        kfree(thresholds_min);
    if (thresholds_max != NULL)
        kfree(thresholds_max);
    if (adjhor != NULL)
        kfree(adjhor);
    if (adjvert != NULL)
        kfree(adjvert);
    if (total_adjhor != NULL)
        kfree(total_adjhor);
    if (total_adjvert != NULL)
        kfree(total_adjvert);
    if (ssCompData.ix2_fm != NULL)
        kfree(ssCompData.ix2_fm);
    if (ssCompData.ix2_sn != NULL)
        kfree(ssCompData.ix2_sn);
    if (ssCompData.cx2_fm != NULL)
        kfree(ssCompData.cx2_fm);
    if (ssCompData.cx2_sn != NULL)
        kfree(ssCompData.cx2_sn);
    if (totCompData.ix_fm != NULL)
        kfree(totCompData.ix_fm);
    if (totCompData.ix_sn != NULL)
        kfree(totCompData.ix_sn);
    if (totCompData.cx_fm != NULL)
        kfree(totCompData.cx_fm);
    if (totCompData.cx_sn != NULL)
        kfree(totCompData.cx_sn);
    return ret;
}

/**
  * Perform a complete Data Test check of the IC
  * @param path_limits name of Production Limit file to load or
  * "NULL" if the limits data should be loaded by a .h file
  * @param stop_on_fail if 1, the test flow stops at the first data check
  * failure otherwise it keeps going performing all the selected test
  * @param todo pointer to a TestToDo variable which select the test to do
  * @return OK if success or an error code which specify the type of error
  */
int production_test_data(char *path_limits, int stop_on_fail, TestToDo *todo)
{
    int res = OK, ret;

    if (todo == NULL) {
        TPD_INFO("production_test_data: No TestToDo specified!! ERROR = %08X\n", (ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA));
        return ERROR_OP_NOT_ALLOW | ERROR_PROD_TEST_DATA;
    }

    TPD_DEBUG("DATA Production test is starting...\n");

    ret = production_test_ms_raw(path_limits, stop_on_fail, todo);
    res |= ret;
    if (ret < 0) {
        TPD_INFO("production_test_data: production_test_ms_raw failed... ERROR = %08X\n", ret);
        if (stop_on_fail == 1)
            goto END;
    }

    ret = production_test_ms_cx(path_limits, stop_on_fail, todo);
    res |= ret;
    if (ret < 0) {
        TPD_INFO("production_test_data: production_test_ms_cx failed... ERROR = %08X\n", ret);
        if (stop_on_fail == 1)
            goto END;
    }

    ret = production_test_ss_raw(path_limits, stop_on_fail, todo);
    res |= ret;
    if (ret < 0) {
        TPD_INFO("production_test_data: production_test_ss_raw failed... ERROR = %08X\n", ret);
        if (stop_on_fail == 1)
            goto END;
    }

    ret = production_test_ss_ix_cx(path_limits, stop_on_fail, todo);
    res |= ret;
    if (ret < 0) {
        TPD_INFO("production_test_data: production_test_ss_ix_cx failed... ERROR = %08X\n", ret);
        if (stop_on_fail == 1)
            goto END;
    }

END:
    freeLimitsFile(&limit_file);    /* /< release the limit file loaded during the test */
    if (res < OK)
        TPD_DEBUG("DATA Production test failed!\n");
    else
        TPD_DEBUG("DATA Production test finished!\n");
    return res;
}


/**
  * Retrieve the actual Test Limit data from the system (bin file or header
  *file)
  * @param path name of Production Test Limit file to load or "NULL" if the
  *limits data should be loaded by a .h file
  * @param file pointer to the LimitFile struct which will contains the limits
  *data
  * @return OK if success or an error code which specify the type of error
  *encountered
  */
int getLimitsFile(char *path, LimitFile *file)
{
    const struct firmware *fw = NULL;
    struct device *dev = NULL;
    int fd = -1;

    TPD_DEBUG("Get Limits File starting... %s\n", path);

    if (file->data != NULL) {/* to avoid memory leak on consecutive call of the function with the same pointer */
        TPD_DEBUG(
             "Pointer to Limits Data already contains something... freeing its content!\n");
        kfree(file->data);
        file->data = NULL;
        file->size = 0;
    }

    strlcpy(file->name, path, MAX_LIMIT_FILE_NAME);
    if (strncmp(path, "NULL", 4) == 0) {
        return ERROR_FILE_NOT_FOUND;
    } else {
        dev = getDev();
        if (dev != NULL) {
            TPD_DEBUG("Loading Limits File from .csv!\n");
            fd = request_firmware(&fw, path, dev);
            if (fd == 0) {
                TPD_DEBUG("Start to copy %s...\n",
                     path);
                file->size = fw->size;
                file->data = (char *)kzalloc((file->size) * sizeof(char), GFP_KERNEL);
                if (file->data != NULL) {
                    memcpy(file->data, (char *)fw->data, file->size);
                    TPD_DEBUG("Limit file Size = %d\n", file->size);
                    release_firmware(fw);
                    return OK;
                } else {
                    TPD_INFO("Error while allocating data... ERROR %08X\n", ERROR_ALLOC);
                    release_firmware(fw);
                    return ERROR_ALLOC;
                }
            } else {
                TPD_INFO("Request the file %s failed... ERROR %08X\n", path, ERROR_FILE_NOT_FOUND);
                return ERROR_FILE_NOT_FOUND;
            }
        } else {
            TPD_INFO("Error while getting the device ERROR %08X\n",
                 ERROR_FILE_READ);
            return ERROR_FILE_READ;
        }
    }
}

/**
  * Reset and release the memory which store a Production Limit File previously
  *loaded
  * @param file pointer to the LimitFile struct to free
  * @return OK if success or an error code which specify the type of error
  *encountered
  */

int freeLimitsFile(LimitFile *file)
{
    TPD_DEBUG("Freeing Limit File ...\n");
    if (file != NULL) {
        if (file->data != NULL) {
            kfree(file->data);
            file->data = NULL;
        } else
            TPD_DEBUG("Limit File was already freed!\n");
        file->size = 0;
        strlcpy(file->name, " ", MAX_LIMIT_FILE_NAME);
        return OK;
    } else {
        TPD_INFO("Passed a NULL argument! ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }
}

/**
  * Reset and release the memory which store the current Limit File previously
  *loaded
  * @return OK if success or an error code which specify the type of error
  *encountered
  */

int freeCurrentLimitsFile(void)
{
    return freeLimitsFile(&limit_file);
}


/**
  * Parse the raw data read from a Production test limit file in order to find
  *the specified information
  * If no limits file data are passed, the function loads and stores the limit
  *file from the system
  * @param path name of Production Test Limit file to load or "NULL" if the
  *limits data should be loaded by a .h file
  * @param file pointer to LimitFile struct that should be parsed or NULL if the
  *limit file in the system should be loaded and then parsed
  * @param label string which identify a particular set of data in the file that
  *want to be loaded
  * @param data pointer to the pointer which will contains the specified limits
  *data as 1 dimension matrix with data arranged row after row
  * @param row pointer to a int variable which will contain the number of row of
  *data
  * @param column pointer to a int variable which will contain the number of
  *column of data
  * @return OK if success or an error code which specify the type of error
  */
int parseProductionTestLimits(char *path, LimitFile *file, char *label, int **data, int *row, int *column)
{
    int find = 0;
    char *token = NULL;
    int i = 0;
    int j = 0;
    int z = 0;

    char *line2 = NULL;
    char line[800];
    char *buf = NULL;
    int n, size, pointer = 0, ret = OK;
    char *data_file = NULL;


    if (file == NULL || strcmp(path, file->name) != 0 || file->size == 0) {
        TPD_DEBUG("No limit File data passed... try to get them from the system!\n");
        ret = getLimitsFile(path, &limit_file);
        if (ret < OK) {
            TPD_INFO("parseProductionTestLimits: ERROR %08X\n", ERROR_FILE_NOT_FOUND);
            return ERROR_FILE_NOT_FOUND;
        }
        size = limit_file.size;
        data_file = limit_file.data;
    } else {
        TPD_DEBUG("Limit File data passed as arguments!\n");
        size = file->size;
        data_file = file->data;
    }

    TPD_DEBUG("The size of the limits file is %d bytes...\n", size);

    while (find == 0) {
        /* start to look for the wanted label */
        if (readLine(&data_file[pointer], line, size - pointer, &n) < 0) {
            find = -1;
            break;
        }
        pointer += n;
        if (line[0] == '*') {    /* each header row start with *  ex.* *label,n_row,n_colum */
            line2 = kstrdup(line, GFP_KERNEL);
            if (line2 == NULL) {
                TPD_INFO("parseProductionTestLimits: kstrdup ERROR %08X\n", ERROR_ALLOC);
                ret = ERROR_ALLOC;
                goto END;
            }
            buf = line2;
            line2 += 1;
            token = strsep(&line2, ",");
            if (strcmp(token, label) == 0) {/* if the row is the wanted one i retrieve rows and columns info */
                find = 1;
                token = strsep(&line2, ",");
                if (token != NULL) {
                    if (sscanf(token, "%d", row) == 1)
                        TPD_DEBUG("Row = %d\n", *row);
                    else {
                        TPD_DEBUG("ERROR while reading the row value! ERROR %08X\n", ERROR_FILE_PARSE);
                        ret = ERROR_FILE_PARSE;
                        goto END;
                    }

                } else {
                    TPD_INFO("parseProductionTestLimits 1: ERROR %08X\n", ERROR_FILE_PARSE);
                    ret = ERROR_FILE_PARSE;
                    goto END;
                }
                token = strsep(&line2, ",");
                if (token != NULL) {
                    if (sscanf(token, "%d", column) == 1)
                        TPD_DEBUG("Column = %d\n", *column);
                    else {
                        TPD_DEBUG("ERROR while reading the column value! ERROR %08X\n",ERROR_FILE_PARSE);
                        ret = ERROR_FILE_PARSE;
                        goto END;
                    }
                } else {
                    TPD_INFO("parseProductionTestLimits 2: ERROR %08X\n", ERROR_FILE_PARSE);
                    ret = ERROR_FILE_PARSE;
                    goto END;
                }

                kfree(buf);
                buf = NULL;
                *data = (int *)kzalloc(((*row) * (*column)) * sizeof(int), GFP_KERNEL);/* allocate the memory for containing the data */
                j = 0;
                if (*data == NULL) {
                    TPD_INFO("parseProductionTestLimits: ERROR %08X\n", ERROR_ALLOC);
                    ret = ERROR_ALLOC;
                    goto END;
                }

                /* start to read the data */
                for (i = 0; i < *row; i++) {
                    if (readLine(&data_file[pointer], line, size - pointer, &n) < 0) {
                        TPD_INFO("parseProductionTestLimits : ERROR %08X\n", ERROR_FILE_READ);
                        ret = ERROR_FILE_READ;
                        goto END;
                    }
                    pointer += n;
                    line2 = kstrdup(line, GFP_KERNEL);
                    if (line2 == NULL) {
                        TPD_INFO("parseProductionTestLimits: kstrdup ERROR %08X\n", ERROR_ALLOC);
                        ret = ERROR_ALLOC;
                        goto END;
                    }
                    buf = line2;
                    token = strsep(&line2, ",");
                    for (z = 0; (z < *column) && (token != NULL); z++) {
                        if (sscanf(token, "%d", ((*data) + j)) == 1) {
                            j++;
                            token = strsep(&line2, ",");
                        }
                    }
                    kfree(buf);
                    buf = NULL;
                }
                if (j == ((*row) * (*column))) {/* check that all the data are read */
                    TPD_DEBUG("READ DONE!\n");
                    ret = OK;
                    goto END;
                }
                TPD_INFO("parseProductionTestLimits 3: ERROR %08X\n", ERROR_FILE_PARSE);
                ret = ERROR_FILE_PARSE;
                goto END;
            }
            kfree(buf);
            buf = NULL;
        }
    }
    TPD_INFO("parseProductionTestLimits: ERROR %08X\n", ERROR_LABEL_NOT_FOUND);
    ret = ERROR_LABEL_NOT_FOUND;
END:
    if (buf != NULL)
        kfree(buf);
    return ret;
}


/**
  * Read one line of a text file passed as array of byte and terminate it with a
  *termination character '\0'
  * @param data text file as array of bytes
  * @param line pointer to an array of char that will contain the line read
  * @param size size of data
  * @param n pointer to a int variable which will contain the number of
  *characters of the line
  * @return OK if success or an error code which specify the type of error
  */
int readLine(char *data, char *line, int size, int *n)
{
    int i = 0;

    if (size < 1)
        return ERROR_OP_NOT_ALLOW;

    while (data[i] != '\n' && i < size) {
        line[i] = data[i];
        i++;
    }
    *n = i + 1;
    line[i] = '\0';

    return OK;
}
