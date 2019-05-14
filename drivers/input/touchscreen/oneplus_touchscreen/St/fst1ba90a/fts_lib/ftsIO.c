/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : ftsIO.c
 * Description: Source file for ST fst1ba90a driver lib
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/

/*!
  * \file ftsIO.c
  * \brief Contains all the functions which handle with the I2C/SPI
  *communication
  */


#include "ftsSoftware.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/of_gpio.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

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


static void *client;    /* /< bus client retrived by the OS and used to execute the bus transfers */

#include "ftsCore.h"
#include "ftsError.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "../../../util_interface/touch_interfaces.h"


/**
  * Initialize the static client variable of the fts_lib library in order
  * to allow any i2c/spi transaction in the driver (Must be called in the probe)
  * @param clt pointer to i2c_client or spi_device struct which identify the bus
  * slave device
  * @return OK
  */
int openChannel(void *clt)
{
    client = clt;

    return OK;
}

/**
  * Retrieve the pointer to the device struct of the IC
  * @return a the device struct pointer if client was previously set
  * or NULL in all the other cases
  */
struct device *getDev(void)
{
    if (client != NULL)
        return &(getClient()->dev);
    else
        return NULL;
}

/**
  * Retrieve the pointer of the i2c_client struct representing the IC as i2c
  *slave
  * @return client if it was previously set or NULL in all the other cases
  */
struct i2c_client *getClient()
{
    if (client != NULL)
        return (struct i2c_client *)client;
    else
        return NULL;
}

/****************** New I2C API *********************/

/**
  * Perform a direct bus read
  * @param outBuf pointer of a byte array which should contain the byte read
  *from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int fts_read(u8 *outBuf, int byteToRead)
{
    int ret = 0;

    if (client == NULL)
        return ERROR_BUS_O;

    ret = touch_i2c_read(client, NULL, 0, (__u8 *)outBuf, byteToRead);
    if(ret < 0) {
        return ERROR_BUS_WR;
    }

    return OK;
}


/**
  * Perform a bus write followed by a bus read without a stop condition
  * @param cmd byte array containing the command to write
  * @param cmdLength size of cmd
  * @param outBuf pointer of a byte array which should contain the bytes read
  *from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int fts_writeRead(u8 *cmd, int cmdLength, u8 *outBuf, int byteToRead)
{
    int ret = 0;

    if (client == NULL)
        return ERROR_BUS_O;

    ret = touch_i2c_read(client, (__u8 *)cmd, (__u16)cmdLength, (__u8 *)outBuf, byteToRead);
    if(ret < 0) {
        return ERROR_BUS_WR;
    }

    return OK;
}

/**
  * Perform a bus write
  * @param cmd byte array containing the command to write
  * @param cmdLength size of cmd
  * @return OK if success or an error code which specify the type of error
  */
int fts_write(u8 *cmd, int cmdLength)
{
    int ret = 0;

    if (client == NULL)
        return ERROR_BUS_O;

    ret = touch_i2c_write(client, (__u8 *)cmd, (__u16)cmdLength);
    if(ret < 0) {
        return ERROR_BUS_WR;
    }

    return OK;
}

/**
  * Write a FW command to the IC and check automatically the echo event
  * @param cmd byte array containing the command to send
  * @param cmdLength size of cmd
  * @return OK if success, or an error code which specify the type of error
  */
int fts_writeFwCmd(u8 *cmd, int cmdLength)
{
    int ret = -1;
    int ret2 = -1;
    int retry = 0;

    if (client == NULL)
        return ERROR_BUS_O;

    resetErrorList();

    fts_disableInterruptNoSync();

    while (retry < I2C_RETRY && (ret < OK || ret2 < OK)) {

        ret = touch_i2c_write(client, (__u8 *)cmd, (__u16)cmdLength);

        retry++;
        if (ret >= 0)
            ret2 = checkEcho(cmd, cmdLength);
        if (ret < OK || ret2 < OK)
            msleep(I2C_WAIT_BEFORE_RETRY);
        /* TPD_INFO("fts_writeCmd: attempt %d\n", retry); */
    }

    fts_enableInterrupt();

    if (ret < 0) {
        TPD_INFO("fts_writeFwCmd: ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }
    if (ret2 < OK) {
        TPD_INFO("fts_writeFwCmd: check echo ERROR %08X\n", ret2);
        return ret2;
    }
    return OK;
}


/**
  * Perform two bus write and one bus read without any stop condition
  * In case of FTI this function is not supported and the same sequence
  * can be achieved calling fts_write followed by an fts_writeRead.
  * @param writeCmd1 byte array containing the first command to write
  * @param writeCmdLength size of writeCmd1
  * @param readCmd1 byte array containing the second command to write
  * @param readCmdLength size of readCmd1
  * @param outBuf pointer of a byte array which should contain the bytes read
  * from the IC
  * @param byteToRead number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int fts_writeThenWriteRead(u8 *writeCmd1, int writeCmdLength, u8 *readCmd1, int
               readCmdLength, u8 *outBuf, int byteToRead)
{
    int ret = -1;

    if (client == NULL)
        return ERROR_BUS_O;

    ret = touch_i2c_write(client, (__u8 *)writeCmd1, (__u16)writeCmdLength);
    if(ret < 0) {
        return ERROR_BUS_WR;
    }
    ret = touch_i2c_read(client, (__u8 *)readCmd1, (__u16)readCmdLength, (__u8 *)outBuf, byteToRead);
    if(ret < 0) {
        return ERROR_BUS_WR;
    }

    return OK;
}

/**
  * Perform a chunked write with one byte op code and 1 to 8 bytes address
  * @param cmd byte containing the op code to write
  * @param addrSize address size in byte
  * @param address the starting address
  * @param data pointer of a byte array which contain the bytes to write
  * @param dataSize size of data
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the address is max 8 bytes */
int fts_writeU8UX(u8 cmd, AddrSize addrSize, u64 address, u8 *data, int
          dataSize)
{
    u8 finalCmd[1 + addrSize + WRITE_CHUNK];
    int remaining = dataSize;
    int toWrite = 0, i = 0;

    if (addrSize <= sizeof(u64)) {
        while (remaining > 0) {
            if (remaining >= WRITE_CHUNK) {
                toWrite = WRITE_CHUNK;
                remaining -= WRITE_CHUNK;
            } else {
                toWrite = remaining;
                remaining = 0;
            }

            finalCmd[0] = cmd;
            TPD_DEBUG("%s: addrSize = %d\n", __func__,
                 addrSize);
            for (i = 0; i < addrSize; i++) {
                finalCmd[i + 1] = (u8)((address >> ((addrSize - 1 - i) * 8)) & 0xFF);
                TPD_INFO("%s: cmd[%d] = %02X\n", __func__, i + 1, finalCmd[i + 1]);
            }

            memcpy(&finalCmd[addrSize + 1], data, toWrite);

            if (fts_write(finalCmd, 1 + addrSize + toWrite) < OK) {
                TPD_INFO("%s: ERROR %08X\n", __func__, ERROR_BUS_W);
                return ERROR_BUS_W;
            }

            address += toWrite;

            data += toWrite;
        }
    } else
        TPD_INFO("%s: address size bigger than max allowed %ld... ERROR %08X\n",__func__, sizeof(u64), ERROR_OP_NOT_ALLOW);

    return OK;
}

/**
  * Perform a chunked write read with one byte op code and 1 to 8 bytes address
  * and dummy byte support.
  * @param cmd byte containing the op code to write
  * @param addrSize address size in byte
  * @param address the starting address
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param byteToRead number of bytes to read
  * @param hasDummyByte  if the first byte of each reading is dummy (must be
  * skipped)
  * set to 1, otherwise if it is valid set to 0 (or any other value)
  * @return OK if success or an error code which specify the type of error
  */
int fts_writeReadU8UX(u8 cmd, AddrSize addrSize, u64 address, u8 *outBuf, int
              byteToRead, int hasDummyByte)
{
    u8 finalCmd[1 + addrSize];
    u8 *buff = NULL;/* worst case has dummy byte */
    int remaining = byteToRead;
    int toRead = 0, i = 0;
	buff = kzalloc(sizeof(u8) *(READ_CHUNK + 1), GFP_KERNEL);
	if(buff == NULL) {
		TPD_INFO("kzalloc buff failed\n");
		return ERROR_BUS_WR;
	}

    while (remaining > 0) {
        if (remaining >= READ_CHUNK) {
            toRead = READ_CHUNK;
            remaining -= READ_CHUNK;
        } else {
            toRead = remaining;
            remaining = 0;
        }

        finalCmd[0] = cmd;
        for (i = 0; i < addrSize; i++)
            finalCmd[i + 1] = (u8)((address >> ((addrSize - 1 - i) * 8)) & 0xFF);

        if (hasDummyByte == 1) {
            if (fts_writeRead(finalCmd, 1 + addrSize, buff, toRead + 1) < OK) {
                TPD_INFO("%s: read error... ERROR %08X\n", __func__, ERROR_BUS_WR);
                return ERROR_BUS_WR;
            }
            memcpy(outBuf, buff + 1, toRead);
        } else {
            if (fts_writeRead(finalCmd, 1 + addrSize, buff, toRead) < OK) {
                TPD_INFO("%s: read error... ERROR %08X\n", __func__, ERROR_BUS_WR);
                return ERROR_BUS_WR;
            }
            memcpy(outBuf, buff, toRead);
        }

        address += toRead;

        outBuf += toRead;
    }
	kfree(buff);

    return OK;
}

/**
  * Perform a chunked write followed by a second write with one byte op code
  * for each write and 1 to 8 bytes address (the sum of the 2 address size of
  * the two writes can not exceed 8 bytes)
  * @param cmd1 byte containing the op code of first write
  * @param addrSize1 address size in byte of first write
  * @param cmd2 byte containing the op code of second write
  * @param addrSize2 address size in byte of second write
  * @param address the starting address
  * @param data pointer of a byte array which contain the bytes to write
  * @param dataSize size of data
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the sum of two addresses in the two commands is
 * max 8 bytes */
int fts_writeU8UXthenWriteU8UX(u8 cmd1, AddrSize addrSize1, u8 cmd2, AddrSize
                   addrSize2, u64 address, u8 *data, int dataSize)
{
    u8 finalCmd1[1 + addrSize1];
    u8 finalCmd2[1 + addrSize2 + WRITE_CHUNK];
    int remaining = dataSize;
    int toWrite = 0, i = 0;

    while (remaining > 0) {
        if (remaining >= WRITE_CHUNK) {
            toWrite = WRITE_CHUNK;
            remaining -= WRITE_CHUNK;
        } else {
            toWrite = remaining;
            remaining = 0;
        }

        finalCmd1[0] = cmd1;
        for (i = 0; i < addrSize1; i++)
            finalCmd1[i + 1] = (u8)((address >> ((addrSize1 + addrSize2 - 1 - i) * 8)) & 0xFF);

        finalCmd2[0] = cmd2;
        for (i = addrSize1; i < addrSize1 + addrSize2; i++)
            finalCmd2[i - addrSize1 + 1] = (u8)((address >> ((addrSize1 + addrSize2 - 1 - i) * 8)) & 0xFF);

        memcpy(&finalCmd2[addrSize2 + 1], data, toWrite);

        if (fts_write(finalCmd1, 1 + addrSize1) < OK) {
            TPD_INFO("%s: first write error... ERROR %08X\n",__func__, ERROR_BUS_W);
            return ERROR_BUS_W;
        }

        if (fts_write(finalCmd2, 1 + addrSize2 + toWrite) < OK) {
            TPD_INFO("%s: second write error... ERROR %08X\n", __func__, ERROR_BUS_W);
            return ERROR_BUS_W;
        }

        address += toWrite;

        data += toWrite;
    }

    return OK;
}

/**
  * Perform a chunked write  followed by a write read with one byte op code
  * and 1 to 8 bytes address for each write and dummy byte support.
  * @param cmd1 byte containing the op code of first write
  * @param addrSize1 address size in byte of first write
  * @param cmd2 byte containing the op code of second write read
  * @param addrSize2 address size in byte of second write    read
  * @param address the starting address
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param byteToRead number of bytes to read
  * @param hasDummyByte  if the first byte of each reading is dummy (must be
  * skipped) set to 1,
  *  otherwise if it is valid set to 0 (or any other value)
  * @return OK if success or an error code which specify the type of error
  */
/* this function works only if the sum of two addresses in the two commands is
 * max 8 bytes */
int fts_writeU8UXthenWriteReadU8UX(u8 cmd1, AddrSize addrSize1, u8 cmd2,
                   AddrSize addrSize2, u64 address, u8 *outBuf,
                   int byteToRead, int hasDummyByte)
{
    u8 finalCmd1[1 + addrSize1];
    u8 finalCmd2[1 + addrSize2];
    u8 *buff=NULL;//[READ_CHUNK + 1];/* worst case has dummy byte */
    int remaining = byteToRead;
    int toRead = 0, i = 0;

	buff = kzalloc(sizeof(u8) *(READ_CHUNK + 1), GFP_KERNEL);
	if(buff == NULL) {
		TPD_INFO("kzalloc buff failed\n");
		return ERROR_BUS_WR;
	}
    while (remaining > 0) {
        if (remaining >= READ_CHUNK) {
            toRead = READ_CHUNK;
            remaining -= READ_CHUNK;
        } else {
            toRead = remaining;
            remaining = 0;
        }


        finalCmd1[0] = cmd1;
        for (i = 0; i < addrSize1; i++)
            finalCmd1[i + 1] = (u8)((address >> ((addrSize1 + addrSize2 - 1 - i) * 8)) & 0xFF);
        /* TPD_INFO("%s: finalCmd1[%d] =  %02X\n",
          *     __func__, i+1, finalCmd1[i + 1]); */

        finalCmd2[0] = cmd2;
        for (i = addrSize1; i < addrSize1 + addrSize2; i++)
            finalCmd2[i - addrSize1 + 1] = (u8)((address >> ((addrSize1 + addrSize2 - 1 - i) * 8)) & 0xFF);

        if (fts_write(finalCmd1, 1 + addrSize1) < OK) {
            TPD_INFO("%s: first write error... ERROR %08X\n",__func__, ERROR_BUS_W);
			kfree(buff);
            return ERROR_BUS_W;
        }

        if (hasDummyByte == 1) {
            if (fts_writeRead(finalCmd2, 1 + addrSize2, buff, toRead + 1) < OK) {
                TPD_INFO("%s: read error... ERROR %08X\n", __func__, ERROR_BUS_WR);
				kfree(buff);
                return ERROR_BUS_WR;
            }
            memcpy(outBuf, buff + 1, toRead);
        } else {
            if (fts_writeRead(finalCmd2, 1 + addrSize2, buff, toRead) < OK) {
                TPD_INFO("%s: read error... ERROR %08X\n", __func__, ERROR_BUS_WR);
				kfree(buff);
                return ERROR_BUS_WR;
            }
            memcpy(outBuf, buff, toRead);
        }

        address += toRead;

        outBuf += toRead;
    }
	kfree(buff);
    return OK;
}
