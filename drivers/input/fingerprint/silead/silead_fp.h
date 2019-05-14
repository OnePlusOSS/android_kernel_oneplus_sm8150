/*
 * @file   silead_fp.h
 * @brief  Contains silead_fp device head file.
 *
 *
 * Copyright 2016-2018 Slead Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * ------------------- Revision History ------------------------------
 * <author>    <date>   <version>     <desc>
 * Bill Yu    2018/5/2    0.1.0      Init version
 * Bill Yu    2018/5/28   0.1.1      Disable netlink if netlink id = 0
 * Bill Yu    2018/6/1    0.1.2      Support wakelock
 * Bill Yu    2018/6/5    0.1.3      Support chip enter power down
 * Bill Yu    2018/6/7    0.1.4      Support create proc node
 *
 */

#ifndef __SILEAD_FP_H__
#define __SILEAD_FP_H__

#ifndef _LINUX_WAKELOCK_H
enum {
    WAKE_LOCK_SUSPEND, /* Prevent suspend */
    WAKE_LOCK_TYPE_COUNT
};

struct wake_lock {
    struct wakeup_source ws;
};

static inline void wake_lock_init(struct wake_lock *lock, int type,
                                  const char *name)
{
    wakeup_source_init(&lock->ws, name);
}

static inline void wake_lock_destroy(struct wake_lock *lock)
{
    wakeup_source_trash(&lock->ws);
}

static inline void wake_lock(struct wake_lock *lock)
{
    __pm_stay_awake(&lock->ws);
}

static inline void wake_lock_timeout(struct wake_lock *lock, long timeout)
{
    __pm_wakeup_event(&lock->ws, jiffies_to_msecs(timeout));
}

static inline void wake_unlock(struct wake_lock *lock)
{
    __pm_relax(&lock->ws);
}
#endif /* _LINUX_WAKELOCK_H */

enum _pwdn_mode_t {
	SIFP_PWDN_NONE = 0,
	SIFP_PWDN_POWEROFF = 1, /* shutdown the avdd power supply */
	SIFP_PWDN_FLASH = 2, /* shutdown avdd 200ms for H/W full reset */
	SIFP_PWDN_MAX,
};
enum _netlink_cmd_t {
	SIFP_NETLINK_START = 0,
	SIFP_NETLINK_IRQ = 1,
	SIFP_NETLINK_SCR_OFF,
	SIFP_NETLINK_SCR_ON,
	SIFP_NETLINK_CONNECT,
	SIFP_NETLINK_DISCONNECT,
	SIFP_NETLINK_UI_READY,
	SIFP_NETLINK_TP_TOUCHDOWN,
	SIFP_NETLINK_TP_TOUCHUP,
	SIFP_NETLINK_MAX,
};

enum _fp_nav_key_v_t {
    NAV_KEY_UNKNOWN = 0,
    NAV_KEY_START = 1,
    NAV_KEY_UP = NAV_KEY_START,
    NAV_KEY_DOWN,
    NAV_KEY_RIGHT,
    NAV_KEY_LEFT,
    NAV_KEY_CLICK,
    NAV_KEY_DCLICK,
    NAV_KEY_LONGPRESS,
    NAV_KEY_CLICK_DOWN,
    NAV_KEY_CLICK_UP,
    NAV_KEY_MAX,
    NAV_KEY_WAITMORE = 1000,
};

#define IS_KEY_VALID(k) ((k) > NAV_KEY_UNKNOWN && (k) < NAV_KEY_MAX)

enum _fp_nav_key_f_t {
	NAV_KEY_FLAG_UP = 0,
	NAV_KEY_FLAG_DOWN,
	NAV_KEY_FLAG_CLICK,
};

struct fp_dev_key_t {
	int value;
	uint32_t flag;   /* key down = 1, key up = 0, key down+up = 2 */
};

#define DEVNAME_LEN  16
struct fp_dev_init_t {
	uint8_t mode;
	uint8_t bits;
	uint16_t delay;
	uint32_t speed;
	char dev[DEVNAME_LEN];
	uint8_t nl_id;
	uint8_t dev_id;
	uint16_t reserve;
	uint32_t reg;
	char ta[DEVNAME_LEN];
};

struct fp_underscreen_info {
    uint8_t touch_state;
    uint8_t area_rate;
    uint16_t x;
    uint16_t y;
};

struct fp_dev_debug_t {
	uint8_t cmd[4];
};

struct fp_dev_kmap_t {
	uint16_t k[NAV_KEY_MAX-NAV_KEY_START];  /* Up/Down/Right/Left/Click/Double Click/Longpress */
};

#define PROC_VND_ID_LEN   32

#define SIFP_IOC_MAGIC	's'

#define SIFP_IOC_RESET        _IOW(SIFP_IOC_MAGIC, 10, u8)

#define SIFP_IOC_ENABLE_IRQ   _IO(SIFP_IOC_MAGIC,  11)
#define SIFP_IOC_DISABLE_IRQ  _IO(SIFP_IOC_MAGIC,  12)
#define SIFP_IOC_WAIT_IRQ     _IOR(SIFP_IOC_MAGIC, 13, u8)
#define SIFP_IOC_CLR_IRQ      _IO(SIFP_IOC_MAGIC,  14)
//#define SPFP_IOC_EXIT       _IOR(SIFP_IOC_MAGIC, 1, u8)

#define SIFP_IOC_KEY_EVENT    _IOW(SIFP_IOC_MAGIC, 15, struct fp_dev_key_t)
#define SIFP_IOC_INIT         _IOR(SIFP_IOC_MAGIC, 16, struct fp_dev_init_t)
#define SIFP_IOC_DEINIT       _IO(SIFP_IOC_MAGIC,  17)
#define SIFP_IOC_IRQ_STATUS   _IOR(SIFP_IOC_MAGIC, 18, u8)
#define SIFP_IOC_DEBUG        _IOR(SIFP_IOC_MAGIC, 19, struct fp_dev_debug_t)
#define SIFP_IOC_SCR_STATUS   _IOR(SIFP_IOC_MAGIC, 20, u8)
#define SIFP_IOC_GET_VER      _IOR(SIFP_IOC_MAGIC, 21, char[10])
#define SIFP_IOC_SET_KMAP     _IOW(SIFP_IOC_MAGIC, 22, uint16_t[7])
#define SIFP_IOC_ACQ_SPI      _IO(SIFP_IOC_MAGIC,  23)
#define SIFP_IOC_RLS_SPI      _IO(SIFP_IOC_MAGIC,  24)
#define SIFP_IOC_PKG_SIZE     _IOR(SIFP_IOC_MAGIC, 25, u8)
#define SIFP_IOC_DBG_LEVEL    _IOWR(SIFP_IOC_MAGIC,26, u8)
#define SIFP_IOC_WAKELOCK     _IOW(SIFP_IOC_MAGIC, 27, u8)
#define SIFP_IOC_PWDN         _IOW(SIFP_IOC_MAGIC,  28, u8)
#define SIFP_IOC_PROC_NODE    _IOW(SIFP_IOC_MAGIC, 29, char[PROC_VND_ID_LEN])
#define SIFP_IOC_GET_TP_TOUCH_INFO          _IOR(SIFP_IOC_MAGIC, 30, struct fp_underscreen_info)
#define SIFP_IOC_SET_TP_MSG_REPORT_MODE          _IOW(SIFP_IOC_MAGIC, 31, u8)

#define SIFP_IOC_REPORT_KEY          _IOW(SIFP_IOC_MAGIC, 32, uint8_t)

#define RESET_TIME            1	/* Default chip reset wait time(ms) */
#define RESET_TIME_MULTIPLE   1 /* Multiple for reset time multiple*wait_time */
#define SIFP_NETLINK_ROUTE    30
#define NL_MSG_LEN            16

//#define PROC_DIR		"fp"      /* if defined, create node under /proc/fp/xxx */
//#define PROC_NODE		"fp_id"   /* proc node name */ //remove by chenran

#if (SIFP_NETLINK_ROUTE > 0)
    #define BSP_SIL_NETLINK
#endif

#if !defined(BSP_SIL_PLAT_MTK) && !defined(BSP_SIL_PLAT_QCOM)
  #define BSP_SIL_PLAT_COMM
#endif /* ! BSP_SIL_PLAT_MTK & ! BSP_SIL_PLAT_QCOM */

/* Todo: enable correct power supply mode */
#if !defined(BSP_SIL_POWER_SUPPLY_REGULATOR)
//#define BSP_SIL_POWER_SUPPLY_REGULATOR
//#define BSP_SIL_POWER_SUPPLY_PINCTRL
#define BSP_SIL_POWER_SUPPLY_GPIO
#endif

/* AVDD voltage range 2.8v ~ 3.3v */
#define AVDD_MAX  3000000
#define AVDD_MIN  3000000

/* VDDIO voltage range 1.8v ~ AVDD */
#define VDDIO_MAX 1800000
#define VDDIO_MIN 1800000

#define CURRENT 50000

#if defined(BSP_SIL_POWER_SUPPLY_REGULATOR) && defined(BSP_SIL_POWER_SUPPLY_PINCTRL) || defined(BSP_SIL_POWER_SUPPLY_REGULATOR) && defined(BSP_SIL_POWER_SUPPLY_GPIO) || defined(BSP_SIL_POWER_SUPPLY_GPIO) && defined(BSP_SIL_POWER_SUPPLY_PINCTRL)
  #error "Don't define multiple power supply mode!"
#endif

#ifdef BSP_SIL_PLAT_MTK
  #include "silead_fp_mtk.h"
  #define PLAT_H "silead_fp_mtk.c"

  #define DEVICE "/dev/spidev1.0"
  //#define BSP_SIL_IRQ_CONFIRM
  #define PKG_SIZE 1
#elif defined(BSP_SIL_PLAT_QCOM)
  #define QSEE_V4  /* Enable it if QSEE v4 or higher */
  #include "silead_fp_qcom.h"
  #define PLAT_H "silead_fp_qcom.c"

  #define DEVICE "/dev/spidev0.0"
  #define BSP_SIL_IRQ_CONFIRM
  #define PKG_SIZE 4
  #define TANAME "sileadta"
#else
  #include "silead_fp_comm.h"
  #define PLAT_H "silead_fp_comm.c"

  #define DEVICE "/dev/spidev0.0"
  #define BSP_SIL_IRQ_CONFIRM
  #define PKG_SIZE 4
  #define TANAME ""
#endif /* BSP_SIL_PLAT_XXX */

extern int opticalfp_irq_handler(struct fp_underscreen_info* tp_info);
#endif /* __SILEAD_FP_H__ */

/* End of file silead_fp.h */
