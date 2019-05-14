/***************************************************
 * File:novatek_drivers_nt36672.h
 * Copyright (c)  2017 - 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             novatek nt36672 driver
 * Version:1.0:
 * Date created:2017-09-15
 * Author: Cong.Dai@Bsp.Driver
 * TAG: BSP.TP.Init
 * *
 * -------------- Revision History: -----------------
 *  <author >  <data>  <version>  <desc>
 ***************************************************/
#ifndef NVT_H_NT36672_NOFLASH
#define NVT_H_NT36672_NOFLASH

/*********PART1:Head files**********************/
#include <linux/spi/spi.h>
#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif
#ifdef CONFIG_SPI_MT65XX
#include <linux/platform_data/spi-mt65xx.h>
#endif

#include "../novatek_common.h"
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#include "mtk_spi.h"
#else
#include "driver_spi.h"
#endif
#include "nvt_firmware.h"

#define NVT_ID_BYTE_MAX 6
#define POINT_DATA_LEN 65
#define FW_BIN_SIZE_116KB               (118784)
#define FW_BIN_SIZE FW_BIN_SIZE_116KB
#define FW_BIN_VER_OFFSET               (0x1A000)
#define FW_BIN_VER_BAR_OFFSET   (0x1A001)
#define SPI_READ_FAST   (0x1F654)
#define SWRST_N8_ADDR   (0x03F0FE)      /*672A*/

#define DTAP_DETECT                     15
#define UP_VEE_DETECT                   14
#define DOWN_VEE_DETECT                 33
#define LEFT_VEE_DETECT                 31
#define RIGHT_VEE_DETECT                32
#define CIRCLE_DETECT                   18
#define DOUSWIP_DETECT                  34
#define RIGHT_SLIDE_DETECT              24
#define LEFT_SLIDE_DETECT               23
#define DOWN_SLIDE_DETECT               22
#define UP_SLIDE_DETECT                 21
#define M_DETECT                        17
#define W_DETECT                        13

#define EVENTBUFFER_EDGE_LIMIT_ON  0x79
#define EVENTBUFFER_EDGE_LIMIT_OFF 0x7A
#define EVENTBUFFER_PWR_PLUG_IN    0x53
#define EVENTBUFFER_PWR_PLUG_OUT   0x51

#define NVT_DUMP_SRAM   (0)

#define SPI_TANSFER_LENGTH 256

#define XDATA_SECTOR_SIZE       256
#define NORMAL_MODE             0x00
#define TEST_MODE_1             0x21
#define TEST_MODE_2             0x22
#define MP_MODE_CC              0x41
#define FREQ_HOP_DISABLE        0x66
#define FREQ_HOP_ENABLE         0x65
#define HANDSHAKING_HOST_READY  0xBB

typedef enum {
    NVT_RAWDATA,    //raw data
    NVT_DIFFDATA,   //diff data
    NVT_BASEDATA,   //baseline data
}DEBUG_READ_TYPE;

typedef enum
{
    PALM = 0,
    GLOVE,
    PWR_FLAG,
    HOVER,
    EDGE_REGECT
}CMD_OFFSET;

typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

typedef enum {
        RESET_STATE_INIT = 0xA0,// IC reset
        RESET_STATE_REK,                // ReK baseline
        RESET_STATE_REK_FINISH, // baseline is ready
        RESET_STATE_NORMAL_RUN, // normal run
        RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

struct nvt_ts_mem_map {
        uint32_t EVENT_BUF_ADDR;
        uint32_t RAW_PIPE0_ADDR;
        uint32_t RAW_PIPE0_Q_ADDR;
        uint32_t RAW_PIPE1_ADDR;
        uint32_t RAW_PIPE1_Q_ADDR;
        uint32_t BASELINE_ADDR;
        uint32_t BASELINE_Q_ADDR;
        uint32_t BASELINE_BTN_ADDR;
        uint32_t BASELINE_BTN_Q_ADDR;
        uint32_t DIFF_PIPE0_ADDR;
        uint32_t DIFF_PIPE0_Q_ADDR;
        uint32_t DIFF_PIPE1_ADDR;
        uint32_t DIFF_PIPE1_Q_ADDR;
        uint32_t RAW_BTN_PIPE0_ADDR;
        uint32_t RAW_BTN_PIPE0_Q_ADDR;
        uint32_t RAW_BTN_PIPE1_ADDR;
        uint32_t RAW_BTN_PIPE1_Q_ADDR;
        uint32_t DIFF_BTN_PIPE0_ADDR;
        uint32_t DIFF_BTN_PIPE0_Q_ADDR;
        uint32_t DIFF_BTN_PIPE1_ADDR;
        uint32_t DIFF_BTN_PIPE1_Q_ADDR;
        uint32_t READ_FLASH_CHECKSUM_ADDR;
        uint32_t RW_FLASH_DATA_ADDR;
        /* Phase 2 Host Download */
        uint32_t BOOT_RDY_ADDR;
        uint32_t POR_CD_ADDR;
        /* BLD CRC */
        uint32_t BLD_LENGTH_ADDR;
        uint32_t ILM_LENGTH_ADDR;
        uint32_t DLM_LENGTH_ADDR;
        uint32_t BLD_DES_ADDR;
        uint32_t ILM_DES_ADDR;
        uint32_t DLM_DES_ADDR;
        uint32_t G_ILM_CHECKSUM_ADDR;
        uint32_t G_DLM_CHECKSUM_ADDR;
        uint32_t R_ILM_CHECKSUM_ADDR;
        uint32_t R_DLM_CHECKSUM_ADDR;
		uint32_t BLD_CRC_EN_ADDR;
        uint32_t DMA_CRC_EN_ADDR;
        uint32_t BLD_ILM_DLM_CRC_ADDR;
        uint32_t DMA_CRC_FLAG_ADDR;
        uint32_t DOZE_GM_S1D_SCAN_RAW_ADDR;
        uint32_t DOZE_GM_BTN_SCAN_RAW_ADDR;
};

struct nvt_ts_bin_map {
        char name[12];
        uint32_t BIN_addr;
        uint32_t SRAM_addr;
        uint32_t size;
        uint32_t crc;
};

struct nvt_ts_trim_id_table {
        uint8_t id[NVT_ID_BYTE_MAX];
        uint8_t mask[NVT_ID_BYTE_MAX];
        const struct nvt_ts_mem_map *mmap;
        uint8_t carrier_system;
        uint8_t support_hw_crc;
};

struct nvt_ts_firmware {
        size_t size;
        const u8 *data;
};

struct chip_data_nt36672 {
    bool                            is_sleep_writed;
    char                            *fw_name;
    char                            *test_limit_name;
    tp_dev                          tp_type;
    uint8_t                         fw_ver;
    uint8_t                         fw_sub_ver;
    uint8_t                         recovery_cnt;
    uint8_t                         ilm_dlm_num;
    uint8_t                         *fwbuf;
    uint16_t                        nvt_pid;
    uint32_t                        ENG_RST_ADDR;
    uint32_t                        partition;
    struct spi_device               *s_client;
    struct hw_resource              *hw_res;
    struct nvt_ts_trim_id_table     trim_id_table;
    struct nvt_ts_bin_map           *bin_map;
    struct device                   *dev;
#ifdef CONFIG_SPI_MT65XX
    struct mtk_chip_config          spi_ctrl;
#else
    struct mt_chip_conf             spi_ctrl;
#endif
};
#endif
