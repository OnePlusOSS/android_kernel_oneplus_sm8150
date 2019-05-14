/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : ftsIO.h
 * Description: Source file for ST fst1ba90a driver lib
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/
/*!
  * \file ftsIO.h
  * \brief Contains all the definitions and prototypes used and implemented in
  * ftsIO.c
  */

#ifndef FTS_IO_H
#define FTS_IO_H

#include "ftsSoftware.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define I2C_RETRY                   3    /* /< number of retry in case of i2c failure */
#define I2C_WAIT_BEFORE_RETRY       2    /* /< wait in ms before retry an i2c transaction */

struct i2c_client *getClient(void);
int openChannel(void *clt);
struct device *getDev(void);

/*************** NEW I2C API ****************/
int changeSAD(u8 sad);
int fts_read(u8 *outBuf, int byteToRead);
int fts_writeRead(u8 *cmd, int cmdLength, u8 *outBuf, int byteToRead);
int fts_write(u8 *cmd, int cmdLength);
int fts_writeFwCmd(u8 *cmd, int cmdLenght);
int fts_writeThenWriteRead(u8 *writeCmd1, int writeCmdLength, u8 *readCmd1, int readCmdLength, u8 *outBuf, int byteToRead);
int fts_writeU8UX(u8 cmd, AddrSize addrSize, u64 address, u8 *data, int dataSize);
int fts_writeReadU8UX(u8 cmd, AddrSize addrSize, u64 address, u8 *outBuf, int byteToRead, int hasDummyByte);
int fts_writeU8UXthenWriteU8UX(u8 cmd1, AddrSize addrSize1, u8 cmd2, AddrSize addrSize2, u64 address, u8 *data, int dataSize);
int fts_writeU8UXthenWriteReadU8UX(u8 cmd1, AddrSize addrSize1, u8 cmd2,
                   AddrSize addrSize2, u64 address, u8 *outBuf, int count, int hasDummyByte);
#endif
