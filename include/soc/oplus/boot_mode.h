/************************************************************************************
** File: - android\kernel\arch\arm\mach-msm\include\mach\oplus_boot.h
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description:  
**     change define of boot_mode here for other place to use it
** Version: 1.0 
** --------------------------- Revision History: --------------------------------
** 	           <author>	           <data>			    <desc>
************************************************************************************/
#ifndef _OPLUS_BOOT_H
#define _OPLUS_BOOT_H
enum{
        MSM_BOOT_MODE__NORMAL,
        MSM_BOOT_MODE__FASTBOOT,
        MSM_BOOT_MODE__RECOVERY,
        MSM_BOOT_MODE__FACTORY,
        MSM_BOOT_MODE__RF,
        MSM_BOOT_MODE__WLAN,
        MSM_BOOT_MODE__MOS,
        MSM_BOOT_MODE__CHARGE,
        MSM_BOOT_MODE__SILENCE,
        MSM_BOOT_MODE__SAU,
        MSM_BOOT_MODE__AGING    = 998,
        MSM_BOOT_MODE__SAFE     = 999,
};

extern int get_boot_mode(void);
#ifdef VENDOR_EDIT
extern bool qpnp_is_power_off_charging(void);
#endif
#ifdef VENDOR_EDIT
extern bool qpnp_is_charger_reboot(void);
#endif /*VENDOR_EDIT*/

#endif  /*_OPLUS_BOOT_H*/

#ifdef VENDOR_EDIT
#ifdef PHOENIX_PROJECT
extern bool op_is_monitorable_boot(void);
#endif
#endif

