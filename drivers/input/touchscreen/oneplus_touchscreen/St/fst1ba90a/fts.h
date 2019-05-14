/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : fts.h
 * Description:  ST fst1ba90a driver
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/

#ifndef _LINUX_FTS_I2C_H_
#define _LINUX_FTS_I2C_H_

#include <linux/device.h>
#include "fts_lib/ftsSoftware.h"
#include "fts_lib/ftsHardware.h"
#include "../../touchpanel_common.h"


/****************** CONFIGURATION SECTION ******************/
/** @defgroup conf_section     Driver Configuration Section
  * Settings of the driver code in order to suit the HW set up and the
  * application behavior
  */
/* **** CODE CONFIGURATION **** */
#define FTS_TS_DRV_NAME        "fts"    /*  driver name */

/* If both COMPUTE_INIT_METHOD and PRE_SAVED_METHOD are not defined, driver will be automatically configured as GOLDEN_VALUE_METHOD */
#define COMPUTE_INIT_METHOD     /* Allow to compute init data on phone during production */
#ifndef COMPUTE_INIT_METHOD
#define PRE_SAVED_METHOD        /* Pre-Saved Method used during production */
#endif


/* **** FEATURES USED IN THE IC **** */
//#define PHONE_KEY               /* Enable the support of keys */

#define GESTURE_MODE            /* enable the support of the gestures */
#ifdef GESTURE_MODE
#define USE_GESTURE_MASK        /* the gestures to select are referred using a gesture bitmask instead of their gesture IDs */
#endif


#define CHARGER_MODE    /* enable the support to charger mode feature (comment to disable) */

#define GLOVE_MODE      /* enable the support to glove mode feature (comment to disable) */

//#define COVER_MODE    /* enable the support to cover mode feature (comment to disable) */

//#define STYLUS_MODE   /* enable the support to stylus mode feature (comment to disable) */

//#define GRIP_MODE     /* enable the support to grip mode feature (comment to disable) */
/***** END **** */


/* **** PANEL SPECIFICATION **** */
#define PRESSURE_MIN    0       /* min value of pressure reported */
#define PRESSURE_MAX    127     /* Max value of pressure reported */

#define DISTANCE_MIN    0       /* min distance between the tool and the display */
#define DISTANCE_MAX    127     /* Max distance between the tool and the display */
/* **** END **** */

struct fts_ts_info {
    struct device               *dev;                                   /* Pointer to the structure device */

    struct i2c_client           *client;                                /* I2C client structure */

    struct hw_resource          *hw_res;                                /* hw res parse from device trees */

    unsigned char               data[FIFO_DEPTH][FIFO_EVENT_SIZE];      /* event data from FIFO */

    struct st_proc_operations   *st_ops;                                /* st common ops */

    char                        *fw_name;                               /* FW path name */

    char                        *test_limit_name;                       /* test limit item name */

    tp_dev                      tp_type;                                /* tp type */

    u8                          key_mask;                               /* key mask of key event */
	int							proximity_status;
};


int fts_chip_powercycle(struct fts_ts_info *info);

/* export declaration of functions in fts_proc.c */
extern int fts_proc_init(struct touchpanel_data *ts);
extern void fts_proc_remove(struct touchpanel_data *ts);

#endif
