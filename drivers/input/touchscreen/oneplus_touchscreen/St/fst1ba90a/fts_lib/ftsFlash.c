/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : ftsFlash.c
 * Description: Source file for ST fst1ba90a driver lib
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/


/*!
  * \file ftsFlash.c
  * \brief Contains all the functions to handle the FW update process
  */

#include "ftsCore.h"
#include "ftsCompensation.h"
#include "ftsError.h"
#include "ftsFlash.h"
#include "ftsFrame.h"
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
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
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


extern SysInfo systemInfo;    /* /< forward declaration of the global variable
                               *  of containing System Info Data */


/**
  * Retrieve the actual FW data from the system (bin file or header file)
  * @param pathToFile name of FW file to load or "NULL" if the FW data should be
  * loaded by a .h file
  * @param data pointer to the pointer which will contains the FW data
  * @param size pointer to a variable which will contain the size of the loaded
  * data
  * @return OK if success or an error code which specify the type of error
  */
int getFWdata(const char *pathToFile, u8 **data, int *size)
{
    const struct firmware *fw = NULL;
    struct device *dev = NULL;
    int res = 0;
    char *path = (char *)pathToFile;

    TPD_INFO("getFWdata starting ...\n");
    /* keep the switch case because if the argument passed is null but
     * the option from .h is not set we still try to load from bin */
    TPD_INFO("Read FW from BIN file %s !\n", path);
    dev = getDev();

    if (dev != NULL) {
        res = request_firmware(&fw, path, dev);
        if (res == 0) {
            *size = fw->size;
            *data = (u8 *)kzalloc((*size) * sizeof(u8), GFP_KERNEL);
            if (*data == NULL) {
                TPD_INFO("getFWdata: Impossible to allocate memory! ERROR %08X\n",ERROR_ALLOC);
                release_firmware(fw);
                return ERROR_ALLOC;
            }
            memcpy(*data, (u8 *)fw->data, (*size));
            release_firmware(fw);
        } else {
            TPD_INFO("getFWdata: No File found! ERROR %08X\n", ERROR_FILE_NOT_FOUND);
            return ERROR_FILE_NOT_FOUND;
        }
    } else {
        TPD_INFO("getFWdata: No device found! ERROR %08X\n", ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    TPD_INFO("getFWdata Finished!\n");
    return OK;
}


/**
  * Perform all the steps to read the FW that should be burnt in the IC from
  * the system and parse it in order to fill a Firmware struct with the relevant
  * info
  * @param path name of FW file to load or "NULL" if the FW data should be
  * loaded by a .h file
  * @param fw pointer to a Firmware variable which will contains the FW data and
  * info
  * @param keep_cx if 1, the CX area will be loaded otherwise will be skipped
  * @return OK if success or an error code which specify the type of error
  */
int readFwFile(const char *path, Firmware *fw, int keep_cx)
{
    int res;
    int orig_size;
    u8 *orig_data = NULL;


    res = getFWdata(path, &orig_data, &orig_size);
    if (res < OK) {
        TPD_INFO("readFwFile: impossible retrieve FW... ERROR %08X\n", ERROR_MEMH_READ);
        return res | ERROR_MEMH_READ;
    }
    res = parseBinFile(orig_data, orig_size, fw, keep_cx);
    if (res < OK) {
        TPD_INFO("readFwFile: impossible parse ERROR %08X\n", ERROR_MEMH_READ);
        return res | ERROR_MEMH_READ;
    }

    return OK;
}

/**
  * Perform all the steps necessary to burn the FW into the IC
  * @param path name of FW file to load or "NULL" if the FW data should be
  * loaded by a .h file
  * @param force if 1, the flashing procedure will be forced and executed
  * regardless the additional info, otherwise the FW in the file will be burnt
  * only if it is newer than the one running in the IC
  * @param keep_cx if 1, the CX area will be loaded and burnt otherwise will be
  * skipped and the area will be untouched
  * @return OK if success or an error code which specify the type of error
  */
int flashProcedure(const char *path, int force, int keep_cx)
{
    Firmware fw;
    int res;

    fw.data = NULL;

    TPD_DEBUG("Reading Fw file...\n");
    res = readFwFile(path, &fw, keep_cx);
    if (res < OK) {
        TPD_INFO("flashProcedure: ERROR %08X\n", (res | ERROR_FLASH_PROCEDURE));
        if(fw.data != NULL)
            kfree(fw.data);
        return res | ERROR_FLASH_PROCEDURE;
    }
    TPD_DEBUG("Fw file read COMPLETED!\n");

    TPD_DEBUG("Starting flashing procedure...\n");
    res = flash_burn(fw, force, keep_cx);
    if (res < OK && res != (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)) {
        TPD_INFO("flashProcedure: ERROR %08X\n", ERROR_FLASH_PROCEDURE);
        if(fw.data != NULL)
            kfree(fw.data);
        return res | ERROR_FLASH_PROCEDURE;
    }
    TPD_DEBUG("flashing procedure Finished!\n");

    if(fw.data != NULL)
        kfree(fw.data);

    return res;
}

/**
  * Poll the Flash Status Registers after the execution of a command to check
  * if the Flash becomes ready within a timeout
  * @param type register to check according to the previous command sent
  * @return OK if success or an error code which specify the type of error
  */
int wait_for_flash_ready(u8 type)
{
    u8 cmd[5] = { FTS_CMD_HW_REG_R, 0x20, 0x00, 0x00, type };

    u8 readData[2] = { 0 };
    int i, res = -1;

    TPD_DEBUG("Waiting for flash ready ...\n");
    for (i = 0; i < FLASH_RETRY_COUNT && res != 0; i++) {
        res = fts_writeRead(cmd, ARRAY_SIZE(cmd), readData, 2);
        if (res < OK)
            TPD_INFO("wait_for_flash_ready: ERROR %08X\n", ERROR_BUS_W);
        else {
            res = readData[0] & 0x80;
            TPD_DEBUG("flash status = %d\n", res);
        }
        msleep(FLASH_WAIT_BEFORE_RETRY);
    }

    if (i == FLASH_RETRY_COUNT && res != 0) {
        TPD_INFO("Wait for flash TIMEOUT! ERROR %08X\n", ERROR_TIMEOUT);
        return ERROR_TIMEOUT;
    }

    TPD_DEBUG("Flash READY!\n");
    return OK;
}

/**
  * Put the M3 in hold
  * @return OK if success or an error code which specify the type of error
  */
int hold_m3(void)
{
    int ret;
    u8 cmd[1] = { 0x01 };

    TPD_DEBUG("Command m3 hold...\n");
    ret = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_SYSTEM_RESET, cmd, 1);
    if (ret < OK) {
        TPD_INFO("hold_m3: ERROR %08X\n", ret);
        return ret;
    }
    TPD_DEBUG("Hold M3 DONE!\n");

    return OK;
}

/**
  * Parse the raw data read from a FW file in order to fill properly the fields
  * of a Firmware variable
  * @param fw_data raw FW data loaded from system
  * @param fw_size size of fw_data
  * @param fwData pointer to a Firmware variable which will contain the
  * processed data
  * @param keep_cx if 1, the CX area will be loaded and burnt otherwise will be
  * skipped and the area will be untouched
  * @return OK if success or an error code which specify the type of error
  */
int parseBinFile(u8 *fw_data, int fw_size, Firmware *fwData, int keep_cx)
{
    int dimension, index = 0;
    u32 temp;
    int res, i;

    /* the file should contain at least the header plus the content_crc */
    if (fw_size < FW_HEADER_SIZE + FW_BYTES_ALLIGN || fw_data == NULL) {
        TPD_INFO("parseBinFile: Read only %d instead of %d... ERROR %08X\n", fw_size, FW_HEADER_SIZE + FW_BYTES_ALLIGN, ERROR_FILE_PARSE);
        res = ERROR_FILE_PARSE;
        goto END;
    } else {
        /* start parsing of bytes */
        u8ToU32(&fw_data[index], &temp);
        if (temp != FW_HEADER_SIGNATURE) {
            TPD_INFO("parseBinFile: Wrong Signature %08X ... ERROR %08X\n", temp, ERROR_FILE_PARSE);
            res = ERROR_FILE_PARSE;
            goto END;
        }
        TPD_DEBUG("parseBinFile: Fw Signature OK!\n");
        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        if (temp != FW_FTB_VER) {
            TPD_INFO("parseBinFile: Wrong ftb_version %08X ... ERROR %08X\n", temp, ERROR_FILE_PARSE);
            res = ERROR_FILE_PARSE;
            goto END;
        }
        TPD_DEBUG("parseBinFile: ftb_version OK!\n");
        index += FW_BYTES_ALLIGN;
        if (fw_data[index] != DCHIP_ID_0 || fw_data[index + 1] != DCHIP_ID_1) {
            TPD_INFO("parseBinFile: Wrong target %02X != %02X  %02X != %02X ... ERROR %08X\n", fw_data[index], DCHIP_ID_0, fw_data[index + 1], DCHIP_ID_1, ERROR_FILE_PARSE);
            res = ERROR_FILE_PARSE;
            goto END;
        }
		if((fw_data[FW_ORG_INFO_OFFSET] == 0x00) && (fw_data[FW_ORG_INFO_OFFSET+1] == 0x00) &&
			(fw_data[FW_ORG_INFO_OFFSET+2] == 0x00) && (fw_data[FW_ORG_INFO_OFFSET+3] == 0x00)){
			fwData->flash_org_info[0]= FLASH_FW_CODE_COUNT;
			fwData->flash_org_info[1]= FLASH_PANEL_CFG_COUNT;
			fwData->flash_org_info[2]= FLASH_CX_CFG_COUNT;
			fwData->flash_org_info[3]= FLASH_FW_CFG_COUNT;

		}else{
			for (i = 0; i < 4; i++) {
				fwData->flash_org_info[i] = fw_data[FW_ORG_INFO_OFFSET+i];
			}

		}
		 TPD_INFO("parseBinFile: Flash Organization Info fw_code:%02X,panel_cfg:%02X,cx_cfg:%02X,fw_cfg:%02X\n",
		 	fwData->flash_org_info[0],fwData->flash_org_info[1],fwData->flash_org_info[2],fwData->flash_org_info[3]);
        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        TPD_INFO("parseBinFile: FILE SVN REV = %08X\n", temp);

        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        fwData->fw_ver = temp;
        TPD_INFO("parseBinFile: FILE Fw Version = %04X\n", fwData->fw_ver);

        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        fwData->config_id = temp;
        TPD_INFO("parseBinFile: FILE Config Project ID = %08X\n", temp);

        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        TPD_INFO("parseBinFile: FILE Config Version = %08X\n", temp);

        index += FW_BYTES_ALLIGN * 2;    /* skip reserved data */

        index += FW_BYTES_ALLIGN;
        TPD_INFO("parseBinFile: File External Release =  ");
        for (i = 0; i < EXTERNAL_RELEASE_INFO_SIZE; i++) {
            fwData->externalRelease[i] = fw_data[index++];
            TPD_INFO( "%02X ", fwData->externalRelease[i]);
        }
        TPD_INFO( "\n");

        /* index+=FW_BYTES_ALLIGN; */
        u8ToU32(&fw_data[index], &temp);
        fwData->sec0_size = temp;
        TPD_INFO("parseBinFile:  sec0_size = %08X (%d bytes)\n", fwData->sec0_size, fwData->sec0_size);

        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        fwData->sec1_size = temp;
        TPD_INFO("parseBinFile:  sec1_size = %08X (%d bytes)\n", fwData->sec1_size, fwData->sec1_size);

        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        fwData->sec2_size = temp;
        TPD_INFO("parseBinFile:  sec2_size = %08X (%d bytes)\n", fwData->sec2_size, fwData->sec2_size);

        index += FW_BYTES_ALLIGN;
        u8ToU32(&fw_data[index], &temp);
        fwData->sec3_size = temp;
        TPD_INFO("parseBinFile:  sec3_size = %08X (%d bytes)\n", fwData->sec3_size, fwData->sec3_size);

        index += FW_BYTES_ALLIGN;/* skip header crc */

        dimension = fwData->sec0_size + fwData->sec1_size + fwData->sec2_size + fwData->sec3_size;
        temp = fw_size;

        if (dimension + FW_HEADER_SIZE + FW_BYTES_ALLIGN != temp) {
            TPD_INFO("parseBinFile: Read only %d instead of %d... ERROR %08X\n", fw_size, dimension + FW_HEADER_SIZE +
                 FW_BYTES_ALLIGN, ERROR_FILE_PARSE);
            res = ERROR_FILE_PARSE;
            goto END;
        }

        fwData->data = (u8 *)kzalloc(dimension * sizeof(u8), GFP_KERNEL);
        if (fwData->data == NULL) {
            TPD_INFO("parseBinFile: ERROR %08X\n", ERROR_ALLOC);
            res = ERROR_ALLOC;
            goto END;
        }

        index += FW_BYTES_ALLIGN;
        memcpy(fwData->data, &fw_data[index], dimension);
        if (fwData->sec2_size != 0)
            u8ToU16(&fwData->data[fwData->sec0_size + fwData->sec1_size + FW_CX_VERSION], &fwData->cx_ver);
        else {
            TPD_INFO("parseBinFile: Initialize cx_ver to default value!\n");
            fwData->cx_ver = systemInfo.u16_cxVer;
        }

        TPD_INFO("parseBinFile: CX Version = %04X\n", fwData->cx_ver);

        fwData->data_size = dimension;

        TPD_DEBUG("READ FW DONE %d bytes!\n", fwData->data_size);
        res = OK;
        goto END;
    }

END:
    kfree(fw_data);
    return res;
}

/**
  * Enable UVLO and Auto Power Down Mode
  * @return OK if success or an error code which specify the type of error
  */
int flash_enable_uvlo_autopowerdown(void)
{
	u8 cmd[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_UVLO_ENABLE_CODE0,
			FLASH_UVLO_ENABLE_CODE1 };
	u8 cmd1[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_AUTOPOWERDOWN_ENABLE_CODE0,
			FLASH_AUTOPOWERDOWN_ENABLE_CODE1 };
	TPD_INFO("%s Command enable uvlo ...\n", __func__);
	if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
		TPD_INFO("%s flash_enable_uvlo_autopowerdown: ERROR %08X\n", __func__, ERROR_BUS_W);
		return ERROR_BUS_W;
	}
	if (fts_write(cmd1, ARRAY_SIZE(cmd1)) < OK) {
		TPD_INFO("%s flash_enable_uvlo_autopowerdown: ERROR %08X\n", __func__, ERROR_BUS_W);
		return ERROR_BUS_W;
	}
	TPD_INFO("%s Enable uvlo and flash auto power down  DONE!\n", __func__);
	return OK;
}
/**
  * Unlock the flash to be programmed
  * @return OK if success or an error code which specify the type of error
  */
int flash_unlock(void)
{
    u8 cmd[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_UNLOCK_CODE0,
              FLASH_UNLOCK_CODE1 };

	u8 cmd1[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_UNLOCK_CODE2,
		      FLASH_UNLOCK_CODE3 };
    TPD_DEBUG("Command unlock ...\n");
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
        TPD_INFO("flash_unlock: ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }

	if (fts_write(cmd1, ARRAY_SIZE(cmd1)) < OK) {
		TPD_DEBUG("%s Command unlock: ERROR %08X\n", __func__, ERROR_BUS_W);
		return ERROR_BUS_W;
	}
    TPD_DEBUG("Unlock flash DONE!\n");

    return OK;
}

/**
  * Unlock the flash to be erased
  * @return OK if success or an error code which specify the type of error
  */
int flash_erase_unlock(void)
{
    u8 cmd[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00,0x00, FLASH_ERASE_UNLOCK_CODE0, FLASH_ERASE_UNLOCK_CODE1 };

    TPD_DEBUG("Try to erase unlock flash...\n");

    TPD_DEBUG("Command erase unlock ...\n");
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < 0) {
        TPD_INFO("flash_erase_unlock: ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    TPD_DEBUG("Erase Unlock flash DONE!\n");

    return OK;
}

/**
  * Erase the full flash
  * @return OK if success or an error code which specify the type of error
  */
int flash_full_erase(void)
{
    int status;

    u8 cmd1[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0 + 1, 0x00 };
    u8 cmd[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0, FLASH_ERASE_CODE1 };

    if (fts_write(cmd1, ARRAY_SIZE(cmd1)) < OK) {
        TPD_INFO("flash_erase_page_by_page: ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }


    TPD_DEBUG("Command full erase sent ...\n");
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
        TPD_INFO("flash_full_erase: ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    status = wait_for_flash_ready(FLASH_ERASE_CODE0);

    if (status != OK) {
        TPD_INFO("flash_full_erase: ERROR %08X\n", ERROR_FLASH_NOT_READY);
        return status | ERROR_FLASH_NOT_READY;
        /* Flash not ready within the chosen time, better exit! */
    }

    TPD_DEBUG("Full Erase flash DONE!\n");

    return OK;
}

/**
  * Erase the flash page by page, giving the possibility to skip the CX area and
  * maintain therefore its value
  * @param keep_cx if SKIP_PANEL_INIT the Panel Init pages will be skipped,
  * if > SKIP_PANEL_CX_INIT Cx and Panel Init pages otherwise all the pages will
  * be deleted
  * @return OK if success or an error code which specify the type of error
  */
int flash_erase_page_by_page(Firmware fw,ErasePage keep_cx)
{
    u8 status, i = 0;
    u8 cmd1[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0 + 1, 0x00 };
    u8 cmd[6] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x00, FLASH_ERASE_CODE0, 0xA0 };
    u8 cmd2[9] = { FTS_CMD_HW_REG_W, 0x20, 0x00, 0x01, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    u8 mask[4] = { 0 };
	int start=0,end=0;
	
	start = (int)(fw.flash_org_info[0]+fw.flash_org_info[1]);
	end = (int)(fw.flash_org_info[0]+fw.flash_org_info[1] + fw.flash_org_info[2]) ;

	 TPD_INFO("%s Cx page start:%d end:%d\n",__func__,start,end-1);
    for (i = start; i < end && keep_cx >= SKIP_PANEL_CX_INIT; i++) {
        TPD_DEBUG("Skipping erase CX page %d!\n", i);
        fromIDtoMask(i, mask, 4);
    }
	start = (int)(fw.flash_org_info[0]);
	end = (int)(fw.flash_org_info[0]+fw.flash_org_info[1]);
	TPD_INFO("%s Panel Configuration page start:%d end:%d\n",__func__,start,end-1);
    for (i = start; i < end && keep_cx >= SKIP_PANEL_INIT; i++) {
        TPD_DEBUG("Skipping erase Panel Init page %d!\n", i);
        fromIDtoMask(i, mask, 4);
    }

    TPD_DEBUG("Setting the page mask = ");
    for (i = 0; i < 4; i++) {
        cmd2[5 + i] = cmd2[5 + i] & (~mask[i]);
        TPD_DEBUG( "%02X ", cmd2[5 + i]);
    }

    TPD_DEBUG( "\nWriting page mask...\n");
    if (fts_write(cmd2, ARRAY_SIZE(cmd2)) < OK) {
        TPD_INFO("flash_erase_page_by_page: Page mask ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    if (fts_write(cmd1, ARRAY_SIZE(cmd1)) < OK) {
        TPD_INFO("flash_erase_page_by_page: Disable info ERROR %08X\n",ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    TPD_DEBUG("Command erase pages sent ...\n");
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
        TPD_INFO("flash_erase_page_by_page: Erase ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    status = wait_for_flash_ready(FLASH_ERASE_CODE0);

    if (status != OK) {
        TPD_INFO("flash_erase_page_by_page: ERROR %08X\n", ERROR_FLASH_NOT_READY);
        return status | ERROR_FLASH_NOT_READY;
        /* Flash not ready within the chosen time, better exit! */
    }

    TPD_DEBUG("Erase flash page by page DONE!\n");

    return OK;
}


/**
  * Start the DMA procedure which actually transfer and burn the data loaded
  * from memory into the Flash
  * @return OK if success or an error code which specify the type of error
  */
int start_flash_dma(void)
{
    int status;
	u8 cmd[12] = { FLASH_CMD_WRITE_REGISTER, 0x20, 0x00, 0x00,
		      0x6B, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00,	FLASH_DMA_CODE1 };

    /* write the command to erase the flash */
    TPD_DEBUG("Command flash DMA ...\n");
    if (fts_write(cmd, ARRAY_SIZE(cmd)) < OK) {
        TPD_INFO("start_flash_dma: ERROR %08X\n", ERROR_BUS_W);
        return ERROR_BUS_W;
    }

    status = wait_for_flash_ready(FLASH_DMA_CODE0);

    if (status != OK) {
        TPD_INFO("start_flash_dma: ERROR %08X\n", ERROR_FLASH_NOT_READY);
        return status | ERROR_FLASH_NOT_READY;
        /* Flash not ready within the chosen time, better exit! */
    }

    TPD_DEBUG("flash DMA DONE!\n");

    return OK;
}

/**
  * Copy the FW data that should be burn in the Flash into the memory and then
  * the DMA will take care about burning it into the Flash
  * @param address address in memory where to copy the data, possible values
  * are FLASH_ADDR_CODE, FLASH_ADDR_CONFIG, FLASH_ADDR_CX
  * @param data pointer to an array of byte which contain the data that should
  * be copied into the memory
  * @param size size of data
  * @return OK if success or an error code which specify the type of error
  */
int fillFlash(u32 address, u8 *data, int size)
{
    int remaining = size, index = 0;
    int toWrite = 0;
    int byteBlock = 0;
    int wheel = 0;
    u32 addr = 0;
    int res;
    int delta;
    u8 *buff = NULL;
    u8 buff2[12] = { 0 };

    buff = (u8 *)kzalloc((DMA_CHUNK + 5) * sizeof(u8), GFP_KERNEL);
    if (buff == NULL) {
        TPD_INFO("fillFlash: ERROR %08X\n", ERROR_ALLOC);
        return ERROR_ALLOC;
    }

    while (remaining > 0) {
        byteBlock = 0;

        addr = 0x00100000;

        while (byteBlock < FLASH_CHUNK && remaining > 0) {
            index = 0;
            if (remaining >= DMA_CHUNK) {
                if ((byteBlock + DMA_CHUNK) <= FLASH_CHUNK) {
                    toWrite = DMA_CHUNK;
                    remaining -= DMA_CHUNK;
                    byteBlock += DMA_CHUNK;
                } else {
                    delta = FLASH_CHUNK - byteBlock;
                    toWrite = delta;
                    remaining -= delta;
                    byteBlock += delta;
                }
            } else {
                if ((byteBlock + remaining) <= FLASH_CHUNK) {
                    toWrite = remaining;
                    byteBlock += remaining;
                    remaining = 0;
                } else {
                    delta = FLASH_CHUNK - byteBlock;
                    toWrite = delta;
                    remaining -= delta;
                    byteBlock += delta;
                }
            }


            buff[index++] = FTS_CMD_HW_REG_W;
            buff[index++] = (u8)((addr & 0xFF000000) >> 24);
            buff[index++] = (u8)((addr & 0x00FF0000) >> 16);
            buff[index++] = (u8)((addr & 0x0000FF00) >> 8);
            buff[index++] = (u8)(addr & 0x000000FF);

            memcpy(&buff[index], data, toWrite);
            /* TPD_DEBUG("Command = %02X , address = %02X %02X
             * , bytes = %d, data =  %02X %02X, %02X %02X\n",
             * buff[0], buff[1], buff[2], toWrite, buff[3],
             * buff[4], buff[3 + toWrite-2],
             * buff[3 + toWrite-1]); */
            if (fts_write(buff, index + toWrite) < OK) {
                TPD_INFO("fillFlash: ERROR %08X\n", ERROR_BUS_W);
                kfree(buff);
                return ERROR_BUS_W;
            }

            /* msleep(10); */
            addr += toWrite;
            data += toWrite;
        }

        /* configuring the DMA */
        byteBlock = byteBlock / 4 - 1;
        index = 0;

        buff2[index++] = FLASH_CMD_WRITE_REGISTER;
        buff2[index++] = 0x20;
        buff2[index++] = 0x00;
        buff2[index++] = 0x00;
        buff2[index++] = FLASH_DMA_CONFIG;
        buff2[index++] = 0x00;
        buff2[index++] = 0x00;

        addr = address + ((wheel * FLASH_CHUNK) / 4);
        buff2[index++] = (u8)((addr & 0x000000FF));
        buff2[index++] = (u8)((addr & 0x0000FF00) >> 8);
        buff2[index++] = (u8)(byteBlock & 0x000000FF);
        buff2[index++] = (u8)((byteBlock & 0x0000FF00) >> 8);
        buff2[index++] = 0x00;

        TPD_DEBUG("DMA Command = %02X , address = %02X %02X, words =  %02X %02X\n",buff2[0], buff2[8], buff2[7], buff2[10], buff2[9]);

        if (fts_write(buff2, index) < OK) {
            TPD_INFO(" Error during filling Flash! ERROR %08X\n", ERROR_BUS_W);
            kfree(buff);
            return ERROR_BUS_W;
        }

        res = start_flash_dma();
        if (res < OK) {
            TPD_INFO("Error during flashing DMA! ERROR %08X\n", res);
            kfree(buff);
            return res;
        }
        wheel++;
    }
    kfree(buff);
    return OK;
}


/**
  * Execute the procedure to burn a FW in FTM4/FTI IC
  * @param fw structure which contain the FW to be burnt
  * @param force_burn if >0, the flashing procedure will be forced and executed
  * regardless the additional info, otherwise the FW in the file will be burnt
  * only if it is newer than the one running in the IC
  * @param keep_cx if 1, the function preserve the CX/Panel Init area otherwise
  * will be cleared
  * @return OK if success or an error code which specify the type of error
  */
int flash_burn(Firmware fw, int force_burn, int keep_cx)
{
    int res;
	u32 addr_cx =0x00;
	u32 addr_config =0x00;

    if (!force_burn) {
        for (res = EXTERNAL_RELEASE_INFO_SIZE - 1; res >= 0; res--)
            if (fw.externalRelease[res] > systemInfo.u8_releaseInfo[res])
                goto start;
        TPD_INFO("flash_burn: Firmware in the chip matches(or later) the firmware to flash! NO UPDATE ERROR %08X\n",ERROR_FW_NO_UPDATE);
        return ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED;
    } else {
        /* burn procedure to update the CX memory, if not present just
         * skip it if there isn't a new fw release. */
        if (force_burn == CRC_CX && fw.sec2_size == 0) {
            for (res = EXTERNAL_RELEASE_INFO_SIZE - 1; res >= 0; res--) {
                if (fw.externalRelease[res] > systemInfo.u8_releaseInfo[res]) {
                    force_burn = 0;
                /* Avoid loading the CX because it is missing
                  * in the bin file, it just need to update
                  * to last fw+cfg because a new release */
                    goto start;
                }
            }
        TPD_INFO("flash_burn: CRC in CX but fw does not contain CX data! NO UPDATE ERROR %08X\n",ERROR_FW_NO_UPDATE);
        return ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED;
        }
    }

    /* programming procedure start */
start:
    TPD_DEBUG("Programming Procedure for flashing started:\n\n");
	addr_config = (u32)((fw.flash_org_info[0] + fw.flash_org_info[1] +fw.flash_org_info[2])*0x400);
	addr_cx = (u32)((fw.flash_org_info[0] + fw.flash_org_info[1])*0x400);
	TPD_INFO("FTS The addr config:0X%X,CX:0X%X\n",addr_config,addr_cx);

    TPD_DEBUG("1) SYSTEM RESET:\n");
    res = fts_system_reset();
    if (res < 0) {
        TPD_INFO("system reset FAILED!\n");
        /* If there is no firmware, there is no controller ready event
          * and there will be a timeout, we can keep going. But if
          * there is an I2C error, we must exit.
          */
        if (res != (ERROR_SYSTEM_RESET_FAIL | ERROR_TIMEOUT))
            return res | ERROR_FLASH_BURN_FAILED;
    } else
        TPD_DEBUG("system reset COMPLETED!\n\n");

    msleep(100); /* required by hw during flash procedure */

    TPD_DEBUG("2) HOLD M3 :\n");
    res = hold_m3();
    if (res < OK) {
        TPD_INFO("hold_m3 FAILED!\n");
        return res | ERROR_FLASH_BURN_FAILED;
    } else
        TPD_DEBUG("hold_m3 COMPLETED!\n\n");


	TPD_INFO("%s 3) ENABLE UVLO AND AUTO POWER DOWN MODE :\n", __func__);
	res = flash_enable_uvlo_autopowerdown();
	if (res < OK) {
		TPD_DEBUG("%s    flash_enable_uvlo_autopowerdown FAILED!\n", __func__);
		return res | ERROR_FLASH_BURN_FAILED;
	} else
		TPD_DEBUG("%s    flash_enable_uvlo_autopowerdown COMPLETED!\n\n", __func__);

	TPD_DEBUG("%s 4) FLASH UNLOCK:\n", __func__);
    res = flash_unlock();
    if (res < OK) {
        TPD_INFO("flash unlock FAILED! ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
        return res | ERROR_FLASH_BURN_FAILED;
    } else
        TPD_DEBUG("flash unlock COMPLETED!\n\n");


	TPD_DEBUG("%s 5) FLASH ERASE UNLOCK:\n", __func__);
    res = flash_erase_unlock();
    if (res < 0) {
        TPD_INFO("  flash unlock FAILED! ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
        return res | ERROR_FLASH_BURN_FAILED;
    } else
        TPD_DEBUG("flash unlock COMPLETED!\n\n");

	TPD_DEBUG("%s 6) FLASH ERASE:\n", __func__);
    if (keep_cx > 0) {
        if (fw.sec2_size != 0 && force_burn == CRC_CX)
            res = flash_erase_page_by_page(fw,SKIP_PANEL_INIT);
        else
            res = flash_erase_page_by_page(fw,SKIP_PANEL_CX_INIT);
    } else {
        /* res = flash_full_erase(); */
        res = flash_erase_page_by_page(fw,SKIP_PANEL_INIT);
        if (fw.sec2_size == 0)
            TPD_INFO("WARNING!!! Erasing CX memory but no CX in fw file! touch will not work right after fw update!\n");
    }

    if (res < OK) {
        TPD_INFO("flash erase FAILED! ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
        return res | ERROR_FLASH_BURN_FAILED;
    } else
        TPD_DEBUG("flash erase COMPLETED!\n\n");

	TPD_DEBUG("%s 7) LOAD PROGRAM:\n", __func__);
	
    res = fillFlash(FLASH_ADDR_CODE, &fw.data[0], fw.sec0_size);
    if (res < OK) {
        TPD_INFO("load program ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
        return res | ERROR_FLASH_BURN_FAILED;
    }
    TPD_INFO("load program DONE!\n");

	TPD_DEBUG("%s 8) LOAD CONFIG:\n", __func__);
    res = fillFlash(addr_config, &(fw.data[fw.sec0_size]), fw.sec1_size);
    if (res < OK) {
        TPD_INFO("load config ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
        return res | ERROR_FLASH_BURN_FAILED;
    }
    TPD_INFO("load config DONE!\n");

    if (fw.sec2_size != 0 && (force_burn == CRC_CX || keep_cx <= 0)) {
		TPD_DEBUG("%s 8.1) LOAD CX:\n", __func__);
        res = fillFlash(addr_cx, &(fw.data[fw.sec0_size + fw.sec1_size]), fw.sec2_size);
        if (res < OK) {
            TPD_INFO("load cx ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
            return res | ERROR_FLASH_BURN_FAILED;
        }
        TPD_INFO("load cx DONE!\n");
    }

    TPD_DEBUG("Flash burn COMPLETED!\n\n");

	TPD_DEBUG("%s 9) SYSTEM RESET:\n", __func__);
    res = fts_system_reset();
    if (res < 0) {
        TPD_INFO("system reset FAILED! ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
        return res | ERROR_FLASH_BURN_FAILED;
    }
    TPD_DEBUG("system reset COMPLETED!\n\n");


	TPD_DEBUG("%s 10) FINAL CHECK:\n", __func__);
    res = readSysInfo(0);
    if (res < 0) {
        TPD_INFO("flash_burn: Unable to retrieve Chip INFO! ERROR %08X\n", ERROR_FLASH_BURN_FAILED);
        return res | ERROR_FLASH_BURN_FAILED;
    }

    for (res = 0; res < EXTERNAL_RELEASE_INFO_SIZE; res++) {
        if (fw.externalRelease[res] != systemInfo.u8_releaseInfo[res]) {
            /* External release is printed during readSysInfo */
            TPD_INFO("Firmware in the chip different from the one that was burn!\n");
            return ERROR_FLASH_BURN_FAILED;
        }
    }

    TPD_DEBUG("Final check OK!\n");

    return OK;
}
