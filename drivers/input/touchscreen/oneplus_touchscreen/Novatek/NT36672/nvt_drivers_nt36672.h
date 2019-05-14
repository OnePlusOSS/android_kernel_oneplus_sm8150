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

#ifndef NVT_H_NT36672
#define NVT_H_NT36672

/*********PART1:Head files**********************/
#include <linux/i2c.h>
#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include "../novatek_common.h"

/*********PART2:Define Area**********************/
#define I2C_HW_Address          0x62
#define I2C_BLDR_Address        0x01
#define I2C_FW_Address          0x01

#define POINT_DATA_LEN          65
#define NVT_ID_BYTE_MAX         6
#define XDATA_SECTOR_SIZE       256
#define I2C_TANSFER_LENGTH      64
#define FW_BIN_SIZE_116KB       118784
#define FW_BIN_SIZE             FW_BIN_SIZE_116KB
#define FW_BIN_VER_OFFSET       0x1A000
#define FW_BIN_VER_BAR_OFFSET   0x1A001
#define BLOCK_64KB_NUM          4
#define SIZE_64KB               65536
#define FLASH_SECTOR_SIZE       4096

#define NORMAL_MODE             0x00
#define TEST_MODE_1             0x21
#define TEST_MODE_2             0x22
#define MP_MODE_CC              0x41
#define FREQ_HOP_DISABLE        0x66
#define FREQ_HOP_ENABLE         0x65
#define HANDSHAKING_HOST_READY  0xBB

#define EVENTBUFFER_EDGE_LIMIT_ON  0x79
#define EVENTBUFFER_EDGE_LIMIT_OFF 0x7A
#define EVENTBUFFER_PWR_PLUG_IN    0x53
#define EVENTBUFFER_PWR_PLUG_OUT   0x51
#define EVENTBUFFER_JITTER_ON    0x7D
#define EVENTBUFFER_JITTER_OFF   0x7E

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

/*********PART3:Struct Area**********************/
typedef enum
{
    PALM = 0,
    GLOVE,
    PWR_FLAG,
    HOVER,
    EDGE_REGECT,
    JITTER_FLAG = 6
}CMD_OFFSET;

typedef enum {
    RESET_STATE_INIT = 0xA0,    // IC reset
    RESET_STATE_REK,            // ReK baseline
    RESET_STATE_REK_FINISH,     // baseline is ready
    RESET_STATE_NORMAL_RUN,     // normal run
    RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
    NVT_RAWDATA,    //raw data
    NVT_DIFFDATA,   //diff data
    NVT_BASEDATA,   //baseline data
}DEBUG_READ_TYPE;


typedef enum {
    EVENT_MAP_HOST_CMD                      = 0x50,
    EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
    EVENT_MAP_RESET_COMPLETE                = 0x60,
    EVENT_MAP_FWINFO                        = 0x78,
    EVENT_MAP_PROJECTID                     = 0x9A,
} I2C_EVENT_MAP;

struct nvt_mem_map {
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
    uint32_t DOZE_GM_S1D_SCAN_RAW_ADDR;
    uint32_t DOZE_GM_BTN_SCAN_RAW_ADDR;
};

struct nvt_trim_id_table {
    uint8_t id[NVT_ID_BYTE_MAX];
    uint8_t mask[NVT_ID_BYTE_MAX];
    const struct nvt_mem_map *mmap;
    uint8_t carrier_system;
};

struct chip_data_nt36672 {
    tp_dev                          tp_type;
    char                            *test_limit_name;
    struct i2c_client               *client;
    const struct nvt_mem_map       *mmap;
    uint8_t                        id[NVT_ID_BYTE_MAX];
    uint8_t                         carrier_system;
    uint8_t                         fw_ver;
    uint16_t                        nvt_pid;
    struct hw_resource              *hw_res;
    bool                            is_sleep_writed;
};

#endif
