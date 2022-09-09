#ifndef __OPLUS_DEBUG_INFO__H
#define __OPLUS_DEBUG_INFO__H

#include "oplus_charger.h"

enum GAUGE_SEAL_UNSEAL_ERROR{
	OPLUS_GAUGE_SEAL_FAIL,
	OPLUS_GAUGE_UNSEAL_FIAL,
};

extern int oplus_chg_debug_info_init(void);
extern int oplus_chg_debug_chg_monitor(struct oplus_chg_chip *chip);
extern int oplus_chg_debug_set_cool_down_by_user(int is_cool_down);
extern int oplus_chg_debug_get_cooldown_current(int chg_current_by_tbatt, int chg_current_by_cooldown);
extern int oplus_chg_debug_set_soc_info(struct oplus_chg_chip *chip);
extern void oplus_chg_gauge_seal_unseal_fail(int type);
extern void oplus_chg_vooc_mcu_error( int error );
extern void oplus_chg_set_fast_chg_type(int value);

#endif
