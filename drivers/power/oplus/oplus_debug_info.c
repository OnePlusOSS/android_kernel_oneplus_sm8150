#include <linux/power_supply.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/module.h>
#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
#include <soc/oplus/system/kernel_fb.h>
#endif
#include "oplus_vooc.h"
#include "oplus_charger.h"
#include "oplus_gauge.h"

#include "oplus_debug_info.h"

extern int charger_abnormal_log;

static int chg_check_point_debug = 0;

module_param(chg_check_point_debug, int, 0644);
MODULE_PARM_DESC(chg_check_point_debug, "debug charger check point");

#define OPEN_LOG_BIT BIT(0)

static int fake_soc = -1;
static int fake_ui_soc = -1;
static int aging_cap_test = -1;
static int break_flag = -1;
static int mcu_update_flag = -1;
static int gauge_seal_flag = -1;

#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
#define OPLUS_CHG_DEBUG_LOG_TAG      "OplusCharger"
#define OPLUS_CHG_DEBUG_EVENT_ID     "charge_monitor"
#endif
struct oplus_chg_debug_info oplus_chg_debug_info;
struct oplus_chg_chip *g_debug_oplus_chip = NULL;
struct vooc_charger_type {
	int chg_type;
	int vol;
	int cur;
	int output_power_type;
};
struct vooc_adapter_name {
	char name[32];
};
struct norm_adapter_name {
	char name[32];
};
struct charging_state_name {
	char name[32];
};
struct vooc_fw_type {
	int fw_type;
	int vol;
	int cur;
};
struct capacity_range {
	int low;
	int high;
};
struct temp_range {
	int low;
	int high;
};
struct input_current_setting {
	int cur;
	int vol;
	int time;
};
#define OPLUS_CHG_VOOC_INPUT_CURRENT_MAX_CNT 10
struct vooc_charge_strategy {
	struct capacity_range capacity_range;
	struct temp_range temp_range;
	struct input_current_setting input_current[OPLUS_CHG_VOOC_INPUT_CURRENT_MAX_CNT];
};
enum {
	OPLUS_FASTCHG_OUTPUT_POWER_20W,
	OPLUS_FASTCHG_OUTPUT_POWER_30W,
	OPLUS_FASTCHG_OUTPUT_POWER_33W,
	OPLUS_FASTCHG_OUTPUT_POWER_50W,
	OPLUS_FASTCHG_OUTPUT_POWER_65W,
	OPLUS_FASTCHG_OUTPUT_POWER_MAX,
};
#define OPLUS_CHG_PLUG_OUT 1
#define OPLUS_CHG_DISABLE 2
#define OPLUS_CHG_START_STABLE_TIME (30)
#define OPLUS_CHG_FEEDBACK_TIME (30)//debug tmp //300
#define OPLUS_CHG_DEBUG_FASTCHG_STOP_CNT 6
#define OPLUS_CHG_DEBUG_MAX_SOC  85
#define OPLUS_CHG_DEBUG_VOOC_MAX_VOLT  4400
#define LED_ON_HIGH_POWER_CONSUPTION   (4000 * 500)
#define LED_OFF_HIGH_POWER_CONSUPTION   (4000 * 300)
#define LED_OFF_STABLE_TIME (30)
#define INPUT_POWER_RATIO   60
#define INPUT_CURRENT_DECREASE  (300000)
#define OPLUS_CHARGERID_VOLT_LOW_THLD        650
#define OPLUS_CHARGERID_VOLT_HIGH_THLD        1250
#define HIGH_POWER_CHG_LOW_TEMP     160
#define HIGH_POWER_CHG_HIGH_TEMP    400
#define OPLUS_CHG_BATT_AGING_CAP_DECREASE 300
#define OPLUS_CHG_BATT_INVALID_CAPACITY  -0x1010101
#define OPLUS_CHG_BATT_S0C_CAPACITY_LOAD_JUMP_NUM      5
#define OPLUS_CHG_BATT_UI_S0C_CAPACITY_LOAD_JUMP_NUM   5
#define OPLUS_CHG_BATT_S0C_CAPACITY_JUMP_NUM           3
#define OPLUS_CHG_BATT_UI_S0C_CAPACITY_JUMP_NUM        5
#define OPLUS_CHG_BATT_UI_TO_S0C_CAPACITY_JUMP_NUM     3
#define OPLUS_CHG_MONITOR_FILE   "/data/oplus_charge/oplus_chg_debug_monitor.txt"
#define OPLUS_CHG_BATT_AGING_CHECK_CNT   360
#define OPLUS_CHG_BATT_AGING_CHECK_TIME  (7 * 24 * 60 * 60)
#define OPLUS_CHG_DEBUG_MSG_LEN 1024*4
char oplus_chg_debug_msg[OPLUS_CHG_DEBUG_MSG_LEN] = "";
static int send_info_flag = 0;
#define SEND_INFO_FLAG_WORK 1
#define SEND_INFO_FLAG_IRQ 2
static int oplus_chg_debug_notify_type_is_set(int type);
static int oplus_chg_get_vooc_adapter_type_index(struct oplus_chg_chip *chip);
enum oplus_chg_debug_info_notify_type {
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_BATT_FCC,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_SLOW,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_BREAK,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR,
	OPLUS_CHG_DEBUG_NOTIFY_TYPE_MAX,
};
enum oplus_chg_debug_info_notify_flag {
	OPLUS_NOTIFY_BATT_SOC_CAPCITY_LOAD_JUMP,
	OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_LOAD_JUMP,
	OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP,
	OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP,
	OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP,
	OPLUS_NOTIFY_CHG_BATT_FULL_NON_100_CAP,//soc end
	OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP,//slow start
	OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP,
	OPLUS_NOTIFY_CHG_SLOW_LOW_CHARGE_CURRENT_LONG_TIME,
	OPLUS_NOTIFY_CHG_SLOW_CHG_TYPE_SDP,
	OPLUS_NOTIFY_CHG_SLOW_VOOC_NON_START,
	OPLUS_NOTIFY_CHG_SLOW_VOOC_ADAPTER_NON_MAX_POWER,
	OPLUS_NOTIFY_CHG_SLOW_COOLDOWN_LONG_TIME,
	OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH,
	OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME,//slow end
	OPLUS_NOTIFY_CHG_SLOW_BATT_NON_AUTH,//ic start
	OPLUS_NOTIFY_CHG_SLOW_CHARGER_OV,
	OPLUS_NOTIFY_GAUGE_SEAL_FAIL,
	OPLUS_NOTIFY_GAUGE_UNSEAL_FAIL,
	OPLUS_NOTIFY_MCU_UPDATE_FAIL, //ic end
	OPLUS_NOTIFY_BATT_AGING_CAP,
	OPLUS_NOTIFY_CHG_VOOC_BREAK,
	OPLUS_NOTIFY_CHG_GENERAL_BREAK,
	OPLUS_NOTIFY_CHARGER_INFO,
	OPLUS_NOTIFY_FAST_CHARGER_TIME,
	OPLUS_NOTIFY_MCU_ERROR_CODE,
	OPLUS_NOTIFY_CHG_MAX_CNT,
};
enum {
	OPLUS_CHG_NOTIFY_UNKNOW,
	OPLUS_CHG_NOTIFY_ONCE,
	OPLUS_CHG_NOTIFY_REPEAT,
	OPLUS_CHG_NOTIFY_TOTAL,
};
struct oplus_chg_debug_info {
	int initialized;
	int pre_soc;
	int cur_soc;
	int pre_ui_soc;
	int cur_ui_soc;
	int soc_load_flag;
	unsigned long sleep_tm_sec;
	int soc_notified_flag;
#define SOC_LOAD_DELAY (60 * 1000)
	struct delayed_work soc_load_dwork;
	int fast_chg_type;
	int real_charger_type;
	int pre_prop_status;
	int chg_start_ui_soc;
	int chg_start_temp;
	int chg_start_time;
	int chg_start_batt_volt;
	int chg_end_soc;
	int chg_end_temp;
	int chg_end_time;
	int chg_end_batt_volt;
	int chg_total_time;
	int total_time;
	int total_time_count;
	int pre_led_state;
	int led_off_start_time;
	int chg_cnt[OPLUS_NOTIFY_CHG_MAX_CNT];
	int notify_type;
	unsigned long long notify_flag;
	struct mutex nflag_lock; 
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	int fastchg_stop_cnt;
	int cool_down_by_user;
	int chg_current_by_tbatt;
	int chg_current_by_cooldown;
	int fastchg_input_current;
	struct vooc_charge_strategy *vooc_charge_strategy;
	int vooc_charge_input_current_index;
	int vooc_charge_cur_state_chg_time;
	int vooc_max_input_volt;
	int vooc_max_input_current;
	int fcc_design;
	int chg_full_notified_flag;
	struct workqueue_struct *oplus_chg_debug_wq;
#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
	struct kernel_packet_info *dcs_info;
	struct mutex dcs_info_lock;
#define SEND_INFO_DELAY 3000
	struct delayed_work send_info_dwork;
#define SEND_INFO_MAX_CNT 5
	int retry_cnt;
#endif
	char flag_reason[32];
	char type_reason[32];
	int vooc_mcu_error;
	bool report_soh;
};
struct vooc_charge_strategy *g_vooc_charge_strategy_p[OPLUS_FASTCHG_OUTPUT_POWER_MAX] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};
struct oplus_chg_debug_notify_policy {
	int policy;
	int fast_chg_time;
	int normal_chg_time;
	int percent;
	char reason[32];
};
static struct oplus_chg_debug_notify_policy oplus_chg_debug_notify_policy[] = {
	[OPLUS_NOTIFY_BATT_SOC_CAPCITY_LOAD_JUMP]             = {OPLUS_CHG_NOTIFY_ONCE,          1,    1,      -1,"Soc-LoadSocJump"},
	[OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_LOAD_JUMP]          = {OPLUS_CHG_NOTIFY_ONCE,          1,    1,      -1,"UiSoc_LoadSocJump"},
	[OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP]                  = {OPLUS_CHG_NOTIFY_REPEAT,        1,    1,      -1,"SocJump"},
	[OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP]               = {OPLUS_CHG_NOTIFY_REPEAT,        1,    1,      -1,"UiSocJump"},
	[OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP]            = {OPLUS_CHG_NOTIFY_REPEAT,        1,    1,      -1,"UiSoc-SocJump"},
	[OPLUS_NOTIFY_CHG_BATT_FULL_NON_100_CAP]             = {OPLUS_CHG_NOTIFY_ONCE,          3,    3,      -1,"BattFull-SocNot100"},
	[OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP]               = {OPLUS_CHG_NOTIFY_ONCE,         1,  1,   -1,"BattTempWarm"},
	[OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP]               = {OPLUS_CHG_NOTIFY_ONCE,         1,  1,   -1,"BattTempCold"},
	[OPLUS_NOTIFY_CHG_SLOW_LOW_CHARGE_CURRENT_LONG_TIME] = {OPLUS_CHG_NOTIFY_TOTAL,         720,  1440,   70,"SmallCurrentLongTime"},
	[OPLUS_NOTIFY_CHG_SLOW_CHG_TYPE_SDP]				 = {OPLUS_CHG_NOTIFY_ONCE,		   1,  1,	-1,"SDP-CHG"},
	[OPLUS_NOTIFY_CHG_SLOW_VOOC_NON_START]               = {OPLUS_CHG_NOTIFY_ONCE,         1,  1,   -1,"FastChgNotStart"},
	[OPLUS_NOTIFY_CHG_SLOW_VOOC_ADAPTER_NON_MAX_POWER]   = {OPLUS_CHG_NOTIFY_ONCE,         1,  1,   -1,"AdatpterNotMaxPower"},
	[OPLUS_NOTIFY_CHG_SLOW_COOLDOWN_LONG_TIME]           = {OPLUS_CHG_NOTIFY_TOTAL,         720,  1440,   70,"CoolDownCtlLongTime"},
	[OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH]       = {OPLUS_CHG_NOTIFY_TOTAL,         720,  1440,   40,"SysConsumeHigh"},
	[OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME]             = {OPLUS_CHG_NOTIFY_TOTAL,         720,  1440,   70,"LedOnLongTime"},
	[OPLUS_NOTIFY_CHG_SLOW_BATT_NON_AUTH]				 = {OPLUS_CHG_NOTIFY_ONCE,			1,	  1,	  -1,"BattNotAuth"},
	[OPLUS_NOTIFY_CHG_SLOW_CHARGER_OV]					 = {OPLUS_CHG_NOTIFY_REPEAT,		  1,	1,		-1,"ChargerOV"},
	[OPLUS_NOTIFY_GAUGE_SEAL_FAIL]                      = {OPLUS_CHG_NOTIFY_REPEAT,         1,    1,      -1,"GaugeSealFail"},
	[OPLUS_NOTIFY_GAUGE_UNSEAL_FAIL]                    = {OPLUS_CHG_NOTIFY_REPEAT,         1,    1,      -1,"GaugeUnSealFail"},
	[OPLUS_NOTIFY_MCU_UPDATE_FAIL]					= {OPLUS_CHG_NOTIFY_REPEAT, 		1,	  1,	  -1,"McuUpdateFail"},
	[OPLUS_NOTIFY_BATT_AGING_CAP]                    = {OPLUS_CHG_NOTIFY_REPEAT,        1,    1,      -1,"BattAgeStat"},
	[OPLUS_NOTIFY_CHG_VOOC_BREAK]             		= {OPLUS_CHG_NOTIFY_ONCE,          1,    1,      -1,"FastChgBreak"},
	[OPLUS_NOTIFY_CHG_GENERAL_BREAK]             	= {OPLUS_CHG_NOTIFY_ONCE,          1,    1,      -1,"GeneralChgBreak"},
	[OPLUS_NOTIFY_CHARGER_INFO]                    = {OPLUS_CHG_NOTIFY_REPEAT,         1,    1,      -1,"ChargerInfo"},
	[OPLUS_NOTIFY_FAST_CHARGER_TIME]                    = {OPLUS_CHG_NOTIFY_TOTAL,         720,  1440,   70,"FastChgTime-s"},
	[OPLUS_NOTIFY_MCU_ERROR_CODE]                    = {OPLUS_CHG_NOTIFY_REPEAT,         1,    1,      -1,"ErroFromMcu"},
};
struct oplus_chg_debug_type_policy {
	int upload;
	char reason[32];
};
static struct oplus_chg_debug_type_policy oplus_chg_debug_type_policy[] = {
	[OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP]		= {-1,"soc_error"},
	[OPLUS_CHG_DEBUG_NOTIFY_TYPE_BATT_FCC]		= {-1,"batt_aging"},
	[OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_SLOW]		= {-1,"charge_slow"},
	[OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_BREAK]		= {-1,"charge_break"},
	[OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR]	= {-1,"ic_error"},
};
static struct vooc_adapter_name vooc_adapter_name[] = {
	[OPLUS_FASTCHG_OUTPUT_POWER_20W]		= {"VOOC_20W"},
	[OPLUS_FASTCHG_OUTPUT_POWER_30W]		= {"VOOC_30W"},
	[OPLUS_FASTCHG_OUTPUT_POWER_33W]		= {"VOOC_33W"},
	[OPLUS_FASTCHG_OUTPUT_POWER_50W]		= {"SVOOC_50W"},
	[OPLUS_FASTCHG_OUTPUT_POWER_65W]		= {"SVOOC_65W"},
};
struct vooc_charger_type vooc_charger_type_list[] = {
	{0x01, 5000,  4000, OPLUS_FASTCHG_OUTPUT_POWER_20W},
	{0x13, 5000,  4000, OPLUS_FASTCHG_OUTPUT_POWER_20W},
	{0x34, 5000,  4000, OPLUS_FASTCHG_OUTPUT_POWER_20W},
	{0x19, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x29, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x41, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x42, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x43, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x44, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x45, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x46, 5000,  6000, OPLUS_FASTCHG_OUTPUT_POWER_30W},
	{0x61, 11000,  3000, OPLUS_FASTCHG_OUTPUT_POWER_33W},
	{0x49, 11000,  3000, OPLUS_FASTCHG_OUTPUT_POWER_33W},
	{0x4A, 11000,  3000, OPLUS_FASTCHG_OUTPUT_POWER_33W},
	{0x4B, 11000,  3000, OPLUS_FASTCHG_OUTPUT_POWER_33W},
	{0x4C, 11000,  3000, OPLUS_FASTCHG_OUTPUT_POWER_33W},
	{0x4D, 11000,  3000, OPLUS_FASTCHG_OUTPUT_POWER_33W},
	{0x4E, 11000,  3000, OPLUS_FASTCHG_OUTPUT_POWER_33W},
	{0x11, 10000, 5000, OPLUS_FASTCHG_OUTPUT_POWER_50W},
	{0x12, 10000, 5000, OPLUS_FASTCHG_OUTPUT_POWER_50W},
	{0x21, 10000, 5000, OPLUS_FASTCHG_OUTPUT_POWER_50W},
	{0x31, 10000, 5000, OPLUS_FASTCHG_OUTPUT_POWER_50W},
	{0x33, 10000, 5000, OPLUS_FASTCHG_OUTPUT_POWER_50W},
	{0x62, 10000, 5000, OPLUS_FASTCHG_OUTPUT_POWER_50W},
	{0x14, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x32, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x35, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x36, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x63, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x64, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x65, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x66, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x69, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x6A, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x6B, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x6C, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x6D, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
	{0x6E, 10000, 6500, OPLUS_FASTCHG_OUTPUT_POWER_65W},
};
static struct norm_adapter_name norm_adapter_name[] = {
	[POWER_SUPPLY_TYPE_UNKNOWN]		= {"UNKNOWN"},
	[POWER_SUPPLY_TYPE_BATTERY]		= {"BATTERY"},
	[POWER_SUPPLY_TYPE_UPS]		= {"UPS"},
	[POWER_SUPPLY_TYPE_MAINS]		= {"MAINS"},
	[POWER_SUPPLY_TYPE_USB]		= {"USB"},
	[POWER_SUPPLY_TYPE_USB_DCP]		= {"DCP"},
	[POWER_SUPPLY_TYPE_USB_CDP]		= {"CDP"},
	[POWER_SUPPLY_TYPE_USB_ACA]		= {"ACA"},
	[POWER_SUPPLY_TYPE_USB_TYPE_C]		= {"TYPE_C"},
	[POWER_SUPPLY_TYPE_USB_PD]		= {"PD"},
	[POWER_SUPPLY_TYPE_USB_PD_DRP]		= {"PD_DRP"},
	[POWER_SUPPLY_TYPE_APPLE_BRICK_ID]		= {"BRICK_ID"},//>4.9
#if defined(CONFIG_OPLUS_HVDCP_SUPPORT) || \
	defined(CONFIG_OPLUS_CHARGER_MTK6785) || \
	!defined(CONFIG_OPLUS_CHARGER_MTK)
	[POWER_SUPPLY_TYPE_USB_HVDCP]		= {"HVDCP"},//mtk hvdcp
	[POWER_SUPPLY_TYPE_USB_HVDCP_3]		= {"HVDCP_3"},
#endif
#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)) && \
		(!defined(CONFIG_OPLUS_CHARGER_MTK)))|| \
	((LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)) && \
		defined(CONFIG_OPLUS_CHARGER_MTK))
	[POWER_SUPPLY_TYPE_USB_HVDCP_3P5]		= {"HVDCP_3P5"},//>4.14
	[POWER_SUPPLY_TYPE_USB_FLOAT]		= {"FLOAT"},//>4.14
	[POWER_SUPPLY_TYPE_BMS]		= {"BMS"},
	[POWER_SUPPLY_TYPE_PARALLEL]		= {"PARALLEL"},
	[POWER_SUPPLY_TYPE_MAIN]		= {"MAIN"},
	[POWER_SUPPLY_TYPE_UFP]		= {"UFP"},
	[POWER_SUPPLY_TYPE_DFP]		= {"DFP"},
	[POWER_SUPPLY_TYPE_CHARGE_PUMP]		= {"CHARGE_PUMP"},
#endif
	[POWER_SUPPLY_TYPE_WIRELESS]		= {"WIRELESS"},
#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)) && \
	(!defined(CONFIG_OPLUS_CHARGER_MTK)))
	[POWER_SUPPLY_TYPE_WIPOWER]		= {"WIPOWER"},//qcom
#endif
};
static struct charging_state_name charging_state_name[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN]		= {"UNKNOWN"},
	[POWER_SUPPLY_STATUS_CHARGING]		= {"CHARGING"},
	[POWER_SUPPLY_STATUS_DISCHARGING]	= {"DISCHARGING"},
	[POWER_SUPPLY_STATUS_NOT_CHARGING]	= {"NOT_CHARGING"},
	[POWER_SUPPLY_STATUS_FULL]		= {"FULL"},
};
static int oplus_chg_debug_info_reset(void);
static int oplus_chg_chg_is_normal_path(struct oplus_chg_chip *chip);
static int oplus_chg_debug_notify_flag_is_set(int flag);
static int oplus_chg_debug_reset_notify_flag(void);
static int oplus_chg_reset_chg_notify_type(void);
static int oplus_chg_chg_batt_capacity_jump_check(struct oplus_chg_chip *chip);
static int oplus_chg_mcu_update_check(struct oplus_chg_chip *chip);
#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
static int oplus_chg_pack_debug_info(struct oplus_chg_chip *chip)
{
	char log_tag[] = OPLUS_CHG_DEBUG_LOG_TAG;
	char event_id[] = OPLUS_CHG_DEBUG_EVENT_ID;
	int len;
	len = strlen(&oplus_chg_debug_msg[sizeof(struct kernel_packet_info)]);
	if (len) {
		mutex_lock(&oplus_chg_debug_info.dcs_info_lock);
		memset(oplus_chg_debug_info.dcs_info, 0x0, sizeof(struct kernel_packet_info));
		oplus_chg_debug_info.dcs_info->type = 1;
		memcpy(oplus_chg_debug_info.dcs_info->log_tag, log_tag, strlen(log_tag));
		memcpy(oplus_chg_debug_info.dcs_info->event_id, event_id, strlen(event_id));
		oplus_chg_debug_info.dcs_info->payload_length = len + 1;

		if (chg_check_point_debug&OPEN_LOG_BIT) {
			chg_err("%s\n", oplus_chg_debug_info.dcs_info->payload);
		}

		mutex_unlock(&oplus_chg_debug_info.dcs_info_lock);
		return 0;
	}
	return -1;
}
static int oplus_chg_debug_mask_notify_flag(int low, int high)
{
	unsigned long long mask = -1;
	int bits = sizeof(mask) * 8 - 1;
	mask = (mask >> low) << low;
	mask = (mask << (bits - high)) >> (bits - high);
	mutex_lock(&oplus_chg_debug_info.nflag_lock);
	oplus_chg_debug_info.notify_flag &= ~mask;
	mutex_unlock(&oplus_chg_debug_info.nflag_lock);
	return 0;
}
static void oplus_chg_send_info_dwork(struct work_struct *work)
{
	int ret;
	mutex_lock(&oplus_chg_debug_info.dcs_info_lock);
	ret = fb_kevent_send_to_user(oplus_chg_debug_info.dcs_info);
	mutex_unlock(&oplus_chg_debug_info.dcs_info_lock);
	if ((ret > 0) && (oplus_chg_debug_info.retry_cnt > 0)) {
		queue_delayed_work(oplus_chg_debug_info.oplus_chg_debug_wq,
				&oplus_chg_debug_info.send_info_dwork, msecs_to_jiffies(SEND_INFO_DELAY));
	}
	else {
		//soc jump
		oplus_chg_debug_mask_notify_flag(0, OPLUS_NOTIFY_CHG_BATT_FULL_NON_100_CAP);
		//slow check
		if(send_info_flag != SEND_INFO_FLAG_IRQ) {//plug out with mcu error 0x54  do not clear charger slow flag
			oplus_chg_debug_mask_notify_flag(OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP,
				OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME);
		}
		//ic error
		oplus_chg_debug_mask_notify_flag(OPLUS_NOTIFY_CHG_SLOW_BATT_NON_AUTH, OPLUS_NOTIFY_GAUGE_UNSEAL_FAIL);
		oplus_chg_reset_chg_notify_type();
		memset(oplus_chg_debug_info.flag_reason,0,sizeof(oplus_chg_debug_info.flag_reason));
		memset(oplus_chg_debug_info.type_reason,0,sizeof(oplus_chg_debug_info.type_reason));
		oplus_chg_debug_info.vooc_mcu_error = 0;
		if(send_info_flag == SEND_INFO_FLAG_IRQ) {
			send_info_flag = 0;
		}
		break_flag = -1;
		mcu_update_flag = -1;
		gauge_seal_flag = -1;
	}
	chg_err("retry_cnt: %d\n", oplus_chg_debug_info.retry_cnt);
	oplus_chg_debug_info.retry_cnt--;
}
#endif
static int oplus_chg_read_filedata(struct timespec *ts)
{
	struct file *filp = NULL;
	mm_segment_t old_fs;
	ssize_t size;
	loff_t offsize = 0;
	char buf[32] = {0};
	filp = filp_open(OPLUS_CHG_MONITOR_FILE, O_RDONLY, 0644);
	if (IS_ERR(filp)) {
		chg_err("open file %s failed[%d].\n", OPLUS_CHG_MONITOR_FILE, PTR_ERR(filp));
		return -1;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	memset(buf, 0x0, sizeof(buf));
	size = vfs_read(filp, buf, sizeof(buf), &offsize);
	set_fs(old_fs);
	filp_close(filp, NULL);
	if (size <= 0) {
		chg_err("read file size is zero.\n");
		return -1;
	}
	sscanf(buf, "%ld", &ts->tv_sec);
	return 0;
}
static int oplus_chg_write_filedata(struct timespec *ts)
{
	struct file *filp = NULL;
	mm_segment_t old_fs;
	ssize_t size;
	loff_t offsize = 0;
	char buf[32] = {0};
	filp = filp_open(OPLUS_CHG_MONITOR_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(filp)) {
		chg_err("open file %s failed[%d].\n", OPLUS_CHG_MONITOR_FILE, PTR_ERR(filp));
		return -1;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	snprintf(buf, sizeof(buf), "%lu", ts->tv_sec);
	size = vfs_write(filp, buf, sizeof(buf), &offsize);
	set_fs(old_fs);
	filp_close(filp, NULL);
	if (size <= 0) {
		chg_err("read file size is zero.\n");
		return -1;
	}
	return 0;
}
static int oplus_chg_percent_cnt(int index) {
	int pct = 0;
	if(oplus_chg_debug_info.total_time_count <= 0) {
		return -1;
	}
	pct = oplus_chg_debug_info.chg_cnt[index]
			*5*100/oplus_chg_debug_info.total_time_count;
	if(pct > 100) {
		pct = 100;
	}
	return pct;
}
static int oplus_chg_get_vooc_adapter_name_index(struct oplus_chg_chip *chip) {
	int index = -1;
	int index_adapter = -1;
	if(oplus_chg_debug_info.fast_chg_type == 0) {
		return -1;
	}
	index = oplus_chg_get_vooc_adapter_type_index(chip);
	index_adapter = vooc_charger_type_list[index].output_power_type;
	return index_adapter;
}
static int oplus_chg_get_real_charger_type(struct oplus_chg_chip *chip) {
	if( chip == NULL) {
		return POWER_SUPPLY_TYPE_UNKNOWN;
	}
	if(chip->real_charger_type != 0) {
		return chip->real_charger_type;
	}
	if(chip->chg_ops->oplus_chg_get_pd_type) {
		if(chip->chg_ops->oplus_chg_get_pd_type()) {
			return POWER_SUPPLY_TYPE_USB_PD;
		}
	}
	if(chip->chg_ops->get_charger_subtype()) {
		if (chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_PD) {
			return POWER_SUPPLY_TYPE_USB_PD;
		} else if (chip->chg_ops->get_charger_subtype() == CHARGER_SUBTYPE_QC) {
			return POWER_SUPPLY_TYPE_USB_HVDCP;
		}
	}
	return POWER_SUPPLY_TYPE_UNKNOWN;
}
static void oplus_chg_print_debug_info(struct oplus_chg_chip *chip)
{
	int ret = 0;
	struct oplus_vooc_chip *vooc_chip = NULL;
	struct timespec ts;
	struct rtc_time tm;
	if( chip == NULL )
	{
		return;
	}

	if (oplus_chg_debug_info.notify_type ||
		oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHARGER_INFO))
	{
#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
		if (delayed_work_pending(&oplus_chg_debug_info.send_info_dwork))
			cancel_delayed_work_sync(&oplus_chg_debug_info.send_info_dwork);
		mutex_lock(&oplus_chg_debug_info.dcs_info_lock);
#endif
		memset(oplus_chg_debug_msg, 0x0, sizeof(oplus_chg_debug_msg));
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
		ret += sizeof(struct kernel_packet_info);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				OPLUS_CHG_DEBUG_EVENT_ID"$$");
#endif
		//add for common
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"type@@0x%x", oplus_chg_debug_info.notify_type);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$flag@@0x%llx", oplus_chg_debug_info.notify_flag);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
						"$$type_reason@@%s", oplus_chg_debug_info.type_reason);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
						"$$flag_reason@@%s", oplus_chg_debug_info.flag_reason);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$charging_state@@%s", charging_state_name[chip->prop_status]);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$adapter_type@@0x%x", (oplus_chg_debug_info.fast_chg_type << 8) | oplus_chg_debug_info.real_charger_type);
		if(oplus_chg_get_vooc_adapter_name_index(chip) >= 0) {
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$vooc_adapter_name@@%s", vooc_adapter_name[oplus_chg_get_vooc_adapter_name_index(chip)]);
		}
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$norm_adapter_name@@%s", norm_adapter_name[oplus_chg_debug_info.real_charger_type]);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$chg_total_time@@%d", oplus_chg_debug_info.chg_total_time);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$chg_start_temp@@%d", oplus_chg_debug_info.chg_start_temp);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$chg_start_soc@@%d", oplus_chg_debug_info.chg_start_ui_soc);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$chg_cur_temp@@%d", chip->temperature);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$cur_ui_soc@@%d", oplus_chg_debug_info.cur_ui_soc);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$cur_soc@@%d", oplus_chg_debug_info.cur_soc);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$fast_chg_percent@@%d",
				oplus_chg_percent_cnt(OPLUS_NOTIFY_FAST_CHARGER_TIME));
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$low_current_long_time_percent@@%d",
				oplus_chg_percent_cnt(OPLUS_NOTIFY_CHG_SLOW_LOW_CHARGE_CURRENT_LONG_TIME));
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$cool_down_time_percent@@%d",
				oplus_chg_percent_cnt(OPLUS_NOTIFY_CHG_SLOW_COOLDOWN_LONG_TIME));
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$batt_warm_time_percent@@%d",
				oplus_chg_percent_cnt(OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP));
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$batt_cold_time_percent@@%d",
				oplus_chg_percent_cnt(OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP));
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$sys_consume_high_time_percent@@%d",
				oplus_chg_percent_cnt(OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH));
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$led_on_percent@@%d",
				oplus_chg_percent_cnt(OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME));
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$sleep_time@@%lu", oplus_chg_debug_info.sleep_tm_sec);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$batt_fcc@@%d", chip->batt_fcc);
		if(oplus_chg_debug_info.report_soh) {
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$batt_soh@@%d", chip->batt_soh);
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$batt_cc@@%d", chip->batt_cc);
			oplus_chg_debug_info.report_soh = false;
		}
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$charge_status@@%d", chip->prop_status);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$boot_mode@@%d", chip->boot_mode);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$icharging@@%d", chip->icharging);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$vbat@@%d", chip->batt_volt);
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$cool_down@@%d", chip->cool_down);
		//add for soc jump
		if(oplus_chg_debug_notify_type_is_set(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP)) {
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$pre_soc@@%d", oplus_chg_debug_info.pre_soc);
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$pre_ui_soc@@%d", oplus_chg_debug_info.pre_ui_soc);
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$soc_load@@%d", chip->soc_load);
		}
		//add for vooc_mcu_error
		if(oplus_chg_debug_info.vooc_mcu_error != 0)
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$vooc_mcu_error@@0x%x", oplus_chg_debug_info.vooc_mcu_error);
		if (oplus_chg_chg_is_normal_path(chip))
		{
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$ibus@@%d", chip->ibus);
		}else {
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"$$ibus@@%d", oplus_chg_debug_info.fastchg_input_current);
		}
		ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
				"$$other@@time[%d-%d-%d %d:%d:%d], "
				"CHGR[ %d %d %d %d %d %d %d], "
				"BAT[ %d %d %d %d %d %4d ], "
				"GAUGE[ %3d %d %d %4d %7d %3d %3d %3d %3d %4d], "
				"STATUS[ %d %4d %d %d %d 0x%-4x %d %d %d], "
				"OTHER[ %d %d %d %d %d %d ], ",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec,
				chip->charger_exist, chip->charger_type, chip->charger_volt, chip->prop_status, chip->boot_mode,oplus_chg_debug_info.chg_current_by_tbatt, oplus_chg_debug_info.chg_current_by_cooldown,
				chip->batt_exist, chip->batt_full, chip->chging_on, chip->in_rechging, chip->charging_state, oplus_chg_debug_info.total_time,
				chip->temperature, chip->batt_volt, chip->batt_volt_min, chip->icharging, chip->ibus, chip->soc, chip->ui_soc, chip->soc_load, chip->batt_rm, chip->batt_fcc,
				chip->vbatt_over, chip->chging_over_time, chip->vchg_status, chip->tbatt_status, chip->stop_voter, chip->notify_code, chip->sw_full, chip->hw_full_by_sw, chip->hw_full,
				chip->otg_switch, chip->mmi_chg, chip->boot_reason, chip->boot_mode, chip->chargerid_volt, chip->chargerid_volt_got);
		oplus_vooc_get_vooc_chip_handle(&vooc_chip);
		if (vooc_chip) {
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"VOOC[ %d / %d / %d / %d / %d / %d]",
					vooc_chip->fastchg_allow, vooc_chip->fastchg_started, vooc_chip->fastchg_dummy_started,
					vooc_chip->fastchg_to_normal, vooc_chip->fastchg_to_warm, vooc_chip->btb_temp_over);
		}
		if (chg_check_point_debug&OPEN_LOG_BIT) {
			int i;
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					", CHG_CNT[");
			for(i = 0; i < OPLUS_NOTIFY_CHG_MAX_CNT; i++)
			{
				ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
						"%4d ", oplus_chg_debug_info.chg_cnt[i]);
			}
			ret += snprintf(&oplus_chg_debug_msg[ret], OPLUS_CHG_DEBUG_MSG_LEN - ret,
					"]");
		}
#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
		mutex_unlock(&oplus_chg_debug_info.dcs_info_lock);
		chg_err("[feedback] %s\n", &oplus_chg_debug_msg[sizeof(struct kernel_packet_info)]);
		if (oplus_chg_debug_info.notify_type ||
			oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHARGER_INFO)) {
			ret = oplus_chg_pack_debug_info(chip);
			if (!ret) {
				oplus_chg_debug_info.retry_cnt = SEND_INFO_MAX_CNT;
				queue_delayed_work(oplus_chg_debug_info.oplus_chg_debug_wq, &oplus_chg_debug_info.send_info_dwork, 0);
			}
		}
#else
		if (chg_check_point_debug&OPEN_LOG_BIT) {
			chg_err("[debug test]%s\n", oplus_chg_debug_msg);
		}
#endif
	oplus_chg_debug_info.real_charger_type = oplus_chg_get_real_charger_type(chip);
	}
}
static int oplus_chg_get_input_power(struct oplus_chg_chip *chip)
{
	int input_power;
	input_power = chip->ibus / 1000 * chip->charger_volt * 8 / 10;
	if (chg_check_point_debug&OPEN_LOG_BIT) {
		chg_err("input_power: %d\n", input_power);
	}
	return input_power;
}
static int oplus_chg_get_vooc_adapter_type_index(struct oplus_chg_chip *chip)
{
	int i;
	int arr_size = sizeof(vooc_charger_type_list) / sizeof(vooc_charger_type_list[0]);
	for(i = 0; i < arr_size; i++) {
		if (oplus_chg_debug_info.fast_chg_type == vooc_charger_type_list[i].chg_type) {
			break;
		}
	}
	if (i == arr_size)
		return 0;
	return i;
}
static int oplus_chg_get_vooc_adapter_is_low_input_power(struct oplus_chg_chip *chip)
{
	int adapter_input_power = 0;
	int vooc_adapter_type_index = -1;
	vooc_adapter_type_index = oplus_chg_get_vooc_adapter_type_index(chip);
	adapter_input_power = vooc_charger_type_list[vooc_adapter_type_index].vol * vooc_charger_type_list[vooc_adapter_type_index].cur;
	if (adapter_input_power < (oplus_chg_debug_info.vooc_max_input_volt * oplus_chg_debug_info.vooc_max_input_current))
		return 1;
	return 0;
}
static int oplus_chg_get_vooc_chg_max_input_power(struct oplus_chg_chip *chip)
{
	int vooc_adapter_type_index = -1;
	int adapter_input_power = 0;
	vooc_adapter_type_index = oplus_chg_get_vooc_adapter_type_index(chip);
	adapter_input_power = vooc_charger_type_list[vooc_adapter_type_index].vol * vooc_charger_type_list[vooc_adapter_type_index].cur;
	if (adapter_input_power < (oplus_chg_debug_info.vooc_max_input_volt * oplus_chg_debug_info.vooc_max_input_current))
		return adapter_input_power * 8 / 10;
	return (oplus_chg_debug_info.vooc_max_input_volt * oplus_chg_debug_info.vooc_max_input_current * 8 / 10);
}
static int oplus_chg_get_vooc_input_power(struct oplus_chg_chip *chip)
{
	int i;
	int input_power = 0;
	int vol,cur;
	struct oplus_vooc_chip *vooc_chip = NULL;
	int mcu_chg_current = 0;
	oplus_vooc_get_vooc_chip_handle(&vooc_chip);
	if (vooc_chip == NULL)
		return 0;
	i = oplus_chg_get_vooc_adapter_type_index(chip);
	vol = oplus_chg_debug_info.vooc_max_input_volt < vooc_charger_type_list[i].vol ?
		oplus_chg_debug_info.vooc_max_input_volt : vooc_charger_type_list[i].vol;
	cur = oplus_chg_debug_info.vooc_max_input_current < vooc_charger_type_list[i].cur ?
		oplus_chg_debug_info.vooc_max_input_current : vooc_charger_type_list[i].cur;
	if (vooc_chip->vooc_chg_current_now > 0)
		cur = cur < vooc_chip->vooc_chg_current_now ? cur : vooc_chip->vooc_chg_current_now;
	if (oplus_chg_debug_info.vooc_charge_strategy) {
		chg_err("vooc strategy input_current_index: %d\n", oplus_chg_debug_info.vooc_charge_input_current_index);
		mcu_chg_current = oplus_chg_debug_info.vooc_charge_strategy->input_current[oplus_chg_debug_info.vooc_charge_input_current_index].cur;
		cur = cur < mcu_chg_current ? cur : mcu_chg_current;
	}
	oplus_chg_debug_info.fastchg_input_current = cur;
	input_power = vol * cur * 8 / 10;
	if (chg_check_point_debug&OPEN_LOG_BIT) {
		chg_err("input_power: %d\n", input_power);
	}
	return input_power;
}
static int oplus_chg_get_charge_power(struct oplus_chg_chip *chip)
{
	int charge_power;
	charge_power = chip->icharging * chip->batt_volt * chip->vbatt_num;
	if (chg_check_point_debug&OPEN_LOG_BIT) {
		chg_err("charge_pwoer: %d icharging = %d batt_volt = %d\n", charge_power, chip->icharging, chip->batt_volt);//debug tmp
	}
	return charge_power;
}
static int oplus_chg_set_notify_type(int index)
{
	if ((index < 0) || (index >= OPLUS_CHG_DEBUG_NOTIFY_TYPE_MAX))
		return -1;
	mutex_lock(&oplus_chg_debug_info.nflag_lock);
	oplus_chg_debug_info.notify_type |= (1 << index);
	strcpy(oplus_chg_debug_info.type_reason,
			oplus_chg_debug_type_policy[index].reason);
	mutex_unlock(&oplus_chg_debug_info.nflag_lock);
	return 0;
}
static int oplus_chg_debug_notify_type_is_set(int type)
{
	int ret = 0;
	mutex_lock(&oplus_chg_debug_info.nflag_lock);
	if (oplus_chg_debug_info.notify_type & (1 << type)) {
		ret = 1;
	}
	mutex_unlock(&oplus_chg_debug_info.nflag_lock);
	return ret;
}
static int oplus_chg_reset_chg_notify_type(void)
{
	mutex_lock(&oplus_chg_debug_info.nflag_lock);
	oplus_chg_debug_info.notify_type = 0;
	mutex_unlock(&oplus_chg_debug_info.nflag_lock);
	return 0;
}
static int oplus_chg_set_chg_flag(int index)
{
	int rc = 0;
	int policy_time = 1;
	if ((index < 0) || (index >= OPLUS_NOTIFY_CHG_MAX_CNT))
		return -1;
	mutex_lock(&oplus_chg_debug_info.nflag_lock);
	oplus_chg_debug_info.chg_cnt[index]++;
	if ((oplus_chg_debug_info.fast_chg_type == FASTCHG_CHARGER_TYPE_UNKOWN)
			|| (oplus_chg_debug_info.fast_chg_type == CHARGER_SUBTYPE_FASTCHG_VOOC)) {
		policy_time = oplus_chg_debug_notify_policy[index].normal_chg_time;
	}
	else {
		policy_time = oplus_chg_debug_notify_policy[index].fast_chg_time;
	}
	switch(oplus_chg_debug_notify_policy[index].policy)
	{
		case OPLUS_CHG_NOTIFY_ONCE:
			if (oplus_chg_debug_info.chg_cnt[index] == policy_time)
			{
				oplus_chg_debug_info.notify_flag |= (1L << index);
			}
			rc = 0;
			break;
		case OPLUS_CHG_NOTIFY_REPEAT:
			if ((oplus_chg_debug_info.chg_cnt[index] % policy_time) == 0)
			{
				oplus_chg_debug_info.notify_flag |= (1L << index);
			}
			rc = 0;
			break;
		case OPLUS_CHG_NOTIFY_TOTAL:
			if (((oplus_chg_debug_info.chg_cnt[index] % (policy_time * oplus_chg_debug_notify_policy[index].percent / 100)) == 0))
			{
				oplus_chg_debug_info.notify_flag |= (1L << index);
			}
			rc = 0;
			break;
		default:
			rc = -1;
	}
	if (chg_check_point_debug&OPEN_LOG_BIT) {
		chg_err("cnt: %d, index: %d, flag: 0x%x\n",
				oplus_chg_debug_info.chg_cnt[index], index, oplus_chg_debug_info.notify_flag);
	}
	mutex_unlock(&oplus_chg_debug_info.nflag_lock);
	return rc;
}
static int oplus_chg_vooc_charge_strategy_init(struct oplus_chg_chip *chip)
{
	int i;
	int index;
	int output_power_type = -1;
	int start_soc, start_temp;
	struct vooc_charge_strategy *vooc_charge_strategy_p;
	if (oplus_chg_debug_info.vooc_charge_strategy != NULL)
	{
		for(i = oplus_chg_debug_info.vooc_charge_input_current_index; i < OPLUS_CHG_VOOC_INPUT_CURRENT_MAX_CNT; i++) {
			if (chip->batt_volt <= oplus_chg_debug_info.vooc_charge_strategy->input_current[i].vol) {
				if (i == oplus_chg_debug_info.vooc_charge_input_current_index)
					oplus_chg_debug_info.vooc_charge_cur_state_chg_time +=5;
				if (oplus_chg_debug_info.vooc_charge_strategy->input_current[i].time > 0) {
					if (oplus_chg_debug_info.vooc_charge_cur_state_chg_time >= oplus_chg_debug_info.vooc_charge_strategy->input_current[i].time) {
						if (i < (OPLUS_CHG_VOOC_INPUT_CURRENT_MAX_CNT - 1)) {
							i++;
						}
					}
				}
				if (i != oplus_chg_debug_info.vooc_charge_input_current_index) {
					oplus_chg_debug_info.vooc_charge_cur_state_chg_time = 0;
					oplus_chg_debug_info.vooc_charge_input_current_index = i;
				}
				break;
			}
		}
		return 0;
	}
	if (!oplus_chg_chg_is_normal_path(chip)) {
		if (oplus_vooc_get_fastchg_started()) {
			index = oplus_chg_get_vooc_adapter_type_index(chip);
			output_power_type = vooc_charger_type_list[index].output_power_type;
		}
	}
	if (output_power_type >= 0) {
		start_soc = chip->soc;
		start_temp = chip->temperature;
		vooc_charge_strategy_p = g_vooc_charge_strategy_p[output_power_type];
		if (vooc_charge_strategy_p) {
			for(i = 0; vooc_charge_strategy_p[i].capacity_range.low != -1; i++) {
				if ((vooc_charge_strategy_p[i].capacity_range.low < start_soc)
						&& (vooc_charge_strategy_p[i].capacity_range.high >= start_soc)
						&& (vooc_charge_strategy_p[i].temp_range.low < start_temp)
						&& (vooc_charge_strategy_p[i].temp_range.high >= start_temp)) {
					oplus_chg_debug_info.vooc_charge_strategy = &vooc_charge_strategy_p[i];
					chg_err("output_power_type: %d, capacity_range [%d, %d], temp_range [%d, %d]\n",
							output_power_type,
							vooc_charge_strategy_p[i].capacity_range.low,
							vooc_charge_strategy_p[i].capacity_range.high,
							vooc_charge_strategy_p[i].temp_range.low,
							vooc_charge_strategy_p[i].temp_range.high);
					break;
				}
			}
		}
	}
	if (oplus_chg_debug_info.vooc_charge_strategy) {
		for(i = oplus_chg_debug_info.vooc_charge_input_current_index; i < OPLUS_CHG_VOOC_INPUT_CURRENT_MAX_CNT; i++) {
			if (chip->batt_volt <= oplus_chg_debug_info.vooc_charge_strategy->input_current[i].vol) {
				oplus_chg_debug_info.vooc_charge_input_current_index = i;
				oplus_chg_debug_info.vooc_charge_cur_state_chg_time = 5;
				break;
			}
		}
	}
	return 0;
}
#if 0
static int oplus_chg_vooc_charge_strategy_post(struct oplus_chg_chip *chip)
{
	if (oplus_chg_debug_info.vooc_charge_strategy != NULL)
		return 0;
	if (oplus_chg_chg_is_normal_path(chip))
		return 0;
	if (oplus_vooc_get_fastchg_started())
	{
		oplus_chg_debug_info.vooc_charge_cur_state_chg_time += 5;
	}
	else
	{
		oplus_chg_debug_info.vooc_charge_strategy = NULL;
		oplus_chg_debug_info.vooc_charge_input_current_index = 0;
		oplus_chg_debug_info.vooc_charge_cur_state_chg_time = 0;
	}
	return 0;
}
#endif
int oplus_chg_debug_set_cool_down_by_user(int is_cool_down)
{
	oplus_chg_debug_info.cool_down_by_user = is_cool_down;
	return 0;
}
int oplus_chg_debug_get_cooldown_current(int chg_current_by_tbatt, int chg_current_by_cooldown)
{
	oplus_chg_debug_info.chg_current_by_tbatt = chg_current_by_tbatt;
	oplus_chg_debug_info.chg_current_by_cooldown = chg_current_by_cooldown;
	return 0;
}
static int oplus_chg_chg_is_normal_path(struct oplus_chg_chip *chip)
{
	struct oplus_vooc_chip *vooc_chip = NULL;
	if (!oplus_vooc_get_fastchg_started())
		return 1;
	oplus_vooc_get_vooc_chip_handle(&vooc_chip);
	if (vooc_chip == NULL)
		return 1;
	if (vooc_chip->support_vooc_by_normal_charger_path
			&& (oplus_vooc_get_fast_chg_type() == CHARGER_SUBTYPE_FASTCHG_VOOC))
		return 1;
	return 0;
}
static int oplus_chg_normal_chg_consume_power_check(struct oplus_chg_chip *chip)
{
	int input_power;
	int charge_power;
	input_power = oplus_chg_get_input_power(chip);
	charge_power = oplus_chg_get_charge_power(chip);
	if (!chip->led_on) {
		if ((chip->total_time - oplus_chg_debug_info.led_off_start_time) > LED_OFF_STABLE_TIME) {
			if ((input_power + charge_power) > LED_OFF_HIGH_POWER_CONSUPTION) {
				oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH);
			}
		}
	}else {
		if ((input_power + charge_power) > LED_ON_HIGH_POWER_CONSUPTION) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH);
		}
	}
	return 0;
}
static int oplus_chg_break_check(struct oplus_chg_chip *chip)
{
	if (chip == NULL)
		return 0;
	if(chip->detect_detach_unexpeactly == 1 ||
		(break_flag == 1 && (chg_check_point_debug&OPEN_LOG_BIT))){
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_VOOC_BREAK);
		strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHG_VOOC_BREAK].reason);
	}else if(chip->detect_detach_unexpeactly == 2 ||
		(break_flag == 2 && (chg_check_point_debug&OPEN_LOG_BIT))) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_GENERAL_BREAK);
		strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHG_GENERAL_BREAK].reason);
	}else {
		return 0;
	}
	if (!oplus_chg_debug_notify_type_is_set(OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_BREAK)) {
			oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_BREAK);
	}
	oplus_chg_print_debug_info(chip);
	send_info_flag = SEND_INFO_FLAG_IRQ;
	chip->detect_detach_unexpeactly = 0;
	return 0;
}
static int oplus_chg_fastchg_consume_power_check(struct oplus_chg_chip *chip)
{
	int input_power;
	int charge_power;
	input_power = oplus_chg_get_vooc_input_power(chip);
	charge_power = oplus_chg_get_charge_power(chip);
	if (!chip->led_on) {
		if ((chip->total_time - oplus_chg_debug_info.led_off_start_time) > LED_OFF_STABLE_TIME) {
			if ((input_power + charge_power) > LED_OFF_HIGH_POWER_CONSUPTION) {
				oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH);
			}
		}
	}else {
		if ((input_power + charge_power) > LED_ON_HIGH_POWER_CONSUPTION) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_SYS_POWER_CONSUME_HIGH);
		}
	}
	return 0;
}
static int oplus_chg_fastchg_power_limit_check(struct oplus_chg_chip *chip)
{
	int vooc_chg_max_input_power;
	int charge_power;
	vooc_chg_max_input_power = oplus_chg_get_vooc_chg_max_input_power(chip);
	charge_power = oplus_chg_get_charge_power(chip);
	if (chg_check_point_debug&OPEN_LOG_BIT) {
		chg_err("vooc_chg_max_input_power = %d, charge_power = %d\n",
				vooc_chg_max_input_power, charge_power);
	}

	if ((-1 * charge_power * 10) < (vooc_chg_max_input_power)) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_LOW_CHARGE_CURRENT_LONG_TIME);
	}
	return 0;
}
static int oplus_chg_input_power_limit_check(struct oplus_chg_chip *chip)
{
	int float_voltage;
	union power_supply_propval pval = {0, };
	int default_input_current = -1;
	int max_charger_voltage = -1;
	int charge_power = oplus_chg_get_charge_power(chip);
	struct oplus_vooc_chip *vooc_chip = NULL;
	oplus_vooc_get_vooc_chip_handle(&vooc_chip);
	if (vooc_chip) {
		if (chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP) {
			if ((chip->soc < vooc_chip->vooc_high_soc) && (chip->soc > vooc_chip->vooc_low_soc)) {
				if (chip->temperature >= vooc_chip->vooc_high_temp) {
					oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP);
				}
				if (chip->temperature <= vooc_chip->vooc_low_temp) {
					oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP);
				}
			}
		}
	}
	if (oplus_chg_chg_is_normal_path(chip)) {
		float_voltage = chip->limits.vfloat_sw_set;
		pval.intval = 0;
		power_supply_get_property(oplus_chg_debug_info.usb_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
		max_charger_voltage = pval.intval;
		if (chip->batt_volt < float_voltage) {
			switch(oplus_chg_get_real_charger_type(chip))
			{
				case POWER_SUPPLY_TYPE_USB_DCP:
					if (oplus_vooc_get_fast_chg_type() == CHARGER_SUBTYPE_FASTCHG_VOOC) {
						default_input_current = chip->limits.default_input_current_vooc_ma_normal;
					}
					else {
						default_input_current = chip->limits.default_input_current_charger_ma;
					}
					break;
				case POWER_SUPPLY_TYPE_USB_CDP:
					default_input_current = chip->limits.input_current_cdp_ma;
					break;
				case POWER_SUPPLY_TYPE_USB_PD:
					default_input_current = chip->limits.pd_input_current_charger_ma;
					break;
#ifndef CONFIG_OPLUS_CHARGER_MTK
				case POWER_SUPPLY_TYPE_USB_HVDCP:
				case POWER_SUPPLY_TYPE_USB_HVDCP_3:
					//case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
					default_input_current = chip->limits.qc_input_current_charger_ma;
					break;
#endif
				default:
					default_input_current = -1;
					max_charger_voltage = -1;
					break;
			}
			if ((default_input_current > 0) && (max_charger_voltage > 0)) {
				if ((-1 * charge_power * 10) < (default_input_current * max_charger_voltage / 1000)) {
					oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_LOW_CHARGE_CURRENT_LONG_TIME);
				}
			}
		}
	}
	else {
		if (oplus_vooc_get_fastchg_started() == true)
			oplus_chg_fastchg_power_limit_check(chip);
	}
	return 0;
}
#define soc_high_speed 51
#define soc_mid_speed 76
#define soc_low_speed 91
#define soc_loweset_speed 101
static int oplus_get_time_percent(struct oplus_chg_chip *chip, int param) {
	int rst = -1, dec = -1;
	rst = (chip->batt_capacity_mah * 36 / chip->vbatt_num) / param;
	dec = (chip->batt_capacity_mah * 36 / chip->vbatt_num) % param;
	if((param % 2) == 0) {
		dec = (dec >= (param / 2));
	}else {
		dec = (dec >= ((param / 2) + 1));
	}
	rst += dec;
	return rst;
}
static int oplus_get_chg_time_ref(struct oplus_chg_chip *chip) {
	int soc_start = -1,soc_end = -1,speed_high = -1,speed_mid = -1,
		speed_low = -1,speed_lowest = -1;
	int current_high = -1, current_mid = -1, current_low = -1,
		current_lowest = -1;
	int time_ref = 0;
	soc_start = oplus_chg_debug_info.chg_start_ui_soc;
	soc_end =oplus_chg_debug_info.cur_ui_soc;
	if(chip->vooc_project == 1) {// 5V4A and 5V6A
		current_high = 5000;
		current_mid = 3500;
		current_low = 2500;
		current_lowest = 1000;
	}else if(chip->vooc_project == 2) {// 10V5A
		current_high = 3200;
		current_mid = 2250;
		current_low = 1800;
		current_lowest = 1000;
	}else if(chip->vooc_project == 3) {// 10V6.5A
		current_high = 3200;//second/1*soc
		current_mid = 2250;
		current_low = 1800;
		current_lowest = 1000;
	}else if(chip->vooc_project == 4) {//10V5A one batt
		current_high = 4500;
		current_mid = 3500;
		current_low = 2500;
		current_lowest = 1500;
	}else if(chip->vooc_project == 5) {//11V3A
		current_high = 5000;
		current_mid = 3500;
		current_low = 2500;
		current_lowest = 1000;
	}else {//non vooc
		speed_high = 140;
		speed_mid = 200;
		speed_low = 240;
		speed_lowest = 240;
	}
	speed_high = oplus_get_time_percent(chip, current_high);
	speed_mid = oplus_get_time_percent(chip, current_mid);
	speed_low = oplus_get_time_percent(chip, current_low);
	speed_lowest = oplus_get_time_percent(chip, current_lowest);
	chg_err("speed_high:%d speed_mid:%d speed_low:%d speed_lowest:%d\n",speed_high,speed_mid, speed_low,speed_lowest);//debug temp
	if(soc_start < soc_high_speed) {//start < 51
		if(soc_end < soc_high_speed) {//start<51 end<51
			time_ref = speed_high*
				(soc_end - soc_start);
		}else if(soc_end < soc_mid_speed) {//start<51 end<76
			time_ref = speed_high*
				(soc_high_speed - soc_start) + speed_mid*
				(soc_end - soc_start);
		}else if(soc_end < soc_low_speed) {//start<51 end<91
			time_ref = speed_high*
				(soc_high_speed - soc_start) + speed_mid*
				(soc_mid_speed - soc_high_speed) + speed_low*
				(soc_end - soc_start);
		}else if(soc_end < soc_loweset_speed) {//start<51 end<101
			time_ref = speed_high*
				(soc_high_speed - soc_start) + speed_mid*
				(soc_mid_speed - soc_high_speed) + speed_low*
				(soc_low_speed - soc_mid_speed) + speed_lowest*
				(soc_end - soc_start);
		}
	}else if(soc_start < soc_mid_speed) {
		if(soc_end < soc_mid_speed) {
			time_ref = speed_mid*
				(soc_end - soc_start);
		}else if(soc_end < soc_low_speed) {
			time_ref = speed_mid*
				(soc_mid_speed - soc_start) + speed_low*
				(soc_end - soc_start);
		}else if(soc_end < soc_loweset_speed) {
			time_ref = speed_mid*
				(soc_mid_speed - soc_start) + speed_low*
				(soc_low_speed - soc_mid_speed) + speed_lowest*
				(soc_end - soc_start);
		}
	}else if(soc_start < soc_low_speed) {
		if(soc_end < soc_low_speed) {
			time_ref = speed_low*
				(soc_end - soc_start);
		}else if(soc_end < soc_loweset_speed) {
			time_ref = speed_low*
				(soc_low_speed - soc_start) + speed_lowest*
				(soc_end - soc_start);
		}
	}else if(soc_start < soc_loweset_speed) {
		time_ref = speed_lowest*
				(soc_end - soc_start);
	}
	return time_ref;
}
static int oplus_get_chg_slow_reason(struct oplus_chg_chip *chip) {
	int time_ref = 0;
	int time = 0;
	int i = 0;
	if(oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP) == 1) {
		oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_SLOW);
		strcpy(oplus_chg_debug_info.flag_reason,
			oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP].reason);
		return 0;
	}
	if(oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP) == 1) {
		oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_SLOW);
		strcpy(oplus_chg_debug_info.flag_reason,
			oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP].reason);
		return 0;
	}
	time_ref = oplus_get_chg_time_ref(chip);
	if(oplus_chg_debug_info.cur_ui_soc <= oplus_chg_debug_info.chg_start_ui_soc) {
		time = time_ref + 1;
	}else {
		time = oplus_chg_debug_info.total_time_count;
	}
	chg_err("time %d;time_ref %d\n",time,time_ref);//debug tmp
	if (time > time_ref){
		oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_CHG_SLOW);
	}else {
		if(oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHARGER_INFO)) {
			strcpy(oplus_chg_debug_info.flag_reason,
				oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHARGER_INFO].reason);
		}
		return 0;
	}
	for(i = OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP; i <= OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME; i++){
		if(oplus_chg_debug_notify_policy[i].policy == OPLUS_CHG_NOTIFY_ONCE) {
			if(oplus_chg_debug_notify_flag_is_set(i)) {
				strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[i].reason);
				break;
			}
		}
		if(oplus_chg_debug_notify_policy[i].policy == OPLUS_CHG_NOTIFY_TOTAL) {
			if(oplus_chg_debug_info.chg_cnt[i]
				*5*100/oplus_chg_debug_info.total_time_count > 
				oplus_chg_debug_notify_policy[i].percent) {
				strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[i].reason);
				break;
			}
		}
		if(i == OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME) {
			strcpy(oplus_chg_debug_info.flag_reason,
				"Others");
			break;
		}
	}
	return 0;
}

static void oplus_chg_stop_action(struct oplus_chg_chip *chip) {
	oplus_get_chg_slow_reason(chip);
	oplus_chg_print_debug_info(chip);
	send_info_flag = SEND_INFO_FLAG_WORK;
	return ;
}

#define SOH_START_LOW_TEMP 300
#define SOH_START_HIGH_TEMP 400
static void oplus_chg_update_soh(struct oplus_chg_chip *chip) {
	if(oplus_chg_debug_info.chg_start_temp >=  SOH_START_LOW_TEMP &&
		oplus_chg_debug_info.chg_start_temp <=  SOH_START_HIGH_TEMP) {
		chip->batt_cc = oplus_gauge_get_batt_cc();
		chip->batt_soh = oplus_gauge_get_batt_soh();
		oplus_chg_debug_info.report_soh = true;
	}
	return ;
}

static int oplus_chg_chg_slow_check(struct oplus_chg_chip *chip)
{
	struct timespec ts = {0, 0};
	static struct timespec charge_start_ts = {0, 0};
	static bool stop_chg_flag =  false;
	if (chip->charger_exist && (chip->prop_status != POWER_SUPPLY_STATUS_FULL)) {
		if ((chip->tbatt_status == BATTERY_STATUS__WARM_TEMP)
				|| (chip->tbatt_status == BATTERY_STATUS__HIGH_TEMP)) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP);//batt temp high slow chg
		}
		if ((chip->tbatt_status == BATTERY_STATUS__COLD_TEMP)
				|| (chip->tbatt_status == BATTERY_STATUS__LOW_TEMP)
				|| (chip->tbatt_status == BATTERY_STATUS__REMOVED)) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP);//batt temp low slow chg
		}
	}
	if((oplus_chg_debug_info.pre_prop_status != chip->prop_status)
			&& (chip->prop_status == POWER_SUPPLY_STATUS_CHARGING)
			&& stop_chg_flag && chip->charger_exist) {
			stop_chg_flag = false;
	}
	if ((oplus_chg_debug_info.pre_prop_status != chip->prop_status)
			&& (chip->prop_status != POWER_SUPPLY_STATUS_CHARGING)) {
		oplus_chg_debug_info.pre_prop_status = chip->prop_status;
		if (!chip->charger_exist) {
			if(stop_chg_flag) {//plug after stop
				stop_chg_flag = false;
				return 0;
			}else {//plug after charge
				if (oplus_chg_debug_info.total_time_count > OPLUS_CHG_FEEDBACK_TIME) {
					oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHARGER_INFO);
					oplus_chg_update_soh(chip);
					oplus_chg_stop_action(chip);
				}
				oplus_chg_debug_info_reset();//???
			}
		}else if(!stop_chg_flag){//stop charge or full before plug out
			stop_chg_flag = true;
			if (oplus_chg_debug_info.total_time_count > OPLUS_CHG_FEEDBACK_TIME) {
				oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHARGER_INFO);
				oplus_chg_update_soh(chip);
				oplus_chg_stop_action(chip);
			}
			oplus_chg_debug_info_reset();//???
		}
		return 0;
	}
	if (chip->charger_exist) {
		if (!oplus_chg_debug_info.total_time) {
			oplus_chg_debug_info.total_time = 5;
			getnstimeofday(&charge_start_ts);
		}else {
			getnstimeofday(&ts);
			oplus_chg_debug_info.total_time = 5 + ts.tv_sec - charge_start_ts.tv_sec;
		}
	}else {
		if((oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP) == 1 ||
			oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHG_SLOW_BATT_COLD_TEMP) == 1) &&
			 stop_chg_flag == false) {//tbatt is warm or cold before charge start
			oplus_chg_stop_action(chip);
			oplus_chg_debug_info_reset();
			return 0;
		}
		if(stop_chg_flag)
			stop_chg_flag = false;
	}
	oplus_chg_debug_info.total_time_count = chip->total_time;
	if (chip->charger_type == POWER_SUPPLY_TYPE_USB) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_CHG_TYPE_SDP);
	}
	if (oplus_vooc_get_fastchg_started()) {
		if (oplus_chg_debug_info.fast_chg_type == FASTCHG_CHARGER_TYPE_UNKOWN) {
			oplus_chg_debug_info.fast_chg_type = oplus_vooc_get_fast_chg_type();
			chg_err("fast_chg_type = %d\n", oplus_chg_debug_info.fast_chg_type);
		}
	}
	oplus_chg_debug_info.real_charger_type = oplus_chg_get_real_charger_type(chip);
	if ((oplus_chg_debug_info.pre_prop_status != chip->prop_status)
			&& (oplus_chg_debug_info.pre_prop_status != POWER_SUPPLY_STATUS_CHARGING)) {
		oplus_chg_debug_info.chg_start_ui_soc = chip->ui_soc;
		oplus_chg_debug_info.chg_start_temp = chip->temperature;
		oplus_chg_debug_info.chg_start_time = oplus_chg_debug_info.total_time;
		oplus_chg_debug_info.chg_start_batt_volt = chip->batt_volt;
	}
	oplus_chg_debug_info.pre_prop_status = chip->prop_status;
	oplus_chg_debug_info.chg_total_time = oplus_chg_debug_info.total_time - oplus_chg_debug_info.chg_start_time;
	if ((oplus_chg_debug_info.pre_led_state != chip->led_on)
			&& (!chip->led_on)) {
		oplus_chg_debug_info.led_off_start_time = chip->total_time;
	}
	oplus_chg_debug_info.pre_led_state = chip->led_on;
	oplus_chg_vooc_charge_strategy_init(chip);
	if (chip->vooc_project && (oplus_vooc_get_fastchg_started() == true)) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_FAST_CHARGER_TIME);
		if (oplus_chg_get_vooc_adapter_is_low_input_power(chip)) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_VOOC_ADAPTER_NON_MAX_POWER);
		}
	}else if(chip->vooc_project && (oplus_vooc_get_fastchg_started() == false) &&
		oplus_chg_debug_info.chg_cnt[OPLUS_NOTIFY_FAST_CHARGER_TIME] < 3 &&
		chip->total_time > 30) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_VOOC_NON_START);
	}
	if (oplus_chg_chg_is_normal_path(chip)) {
		oplus_chg_normal_chg_consume_power_check(chip);
	}else {
		oplus_chg_fastchg_consume_power_check(chip);
	}
	if (!oplus_chg_chg_is_normal_path(chip)) {
		if (oplus_chg_debug_info.cool_down_by_user) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_COOLDOWN_LONG_TIME);
		}
	}
	oplus_chg_input_power_limit_check(chip);
	if (chip->led_on) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME);
	}
	return 0;
}
static int oplus_chg_chg_ic_check(struct oplus_chg_chip *chip) {
	static int count_hmac = 0;
	if (!chip->hmac && count_hmac < 37) {
		count_hmac++;
		if(count_hmac == 36) {
			count_hmac = 37;
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_BATT_NON_AUTH);
			strcpy(oplus_chg_debug_info.flag_reason,
						oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHG_SLOW_BATT_NON_AUTH].reason);
			goto end;
		}
	}
	if (chip->notify_code & (1 << NOTIFY_CHARGER_OVER_VOL)) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_SLOW_CHARGER_OV);
		strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHG_SLOW_CHARGER_OV].reason);
		goto end;
	}
	return 0;
end:
	if (!oplus_chg_debug_notify_type_is_set(OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR)) {
		oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR);
		oplus_chg_print_debug_info(chip);
		send_info_flag = SEND_INFO_FLAG_WORK;
	}
	return 0;
}
//TODO
static int oplus_chg_chg_batt_aging_check(struct oplus_chg_chip *chip)
{
	union power_supply_propval pval = {0, };
	int ret;
	struct timespec ts;
	static struct timespec last_ts = {0, 0};
	struct rtc_time tm;
	static int count = 0;
	if ((aging_cap_test > 0) ||
		((count++ > OPLUS_CHG_BATT_AGING_CHECK_CNT) && (chg_check_point_debug&OPEN_LOG_BIT)))
		{
			getnstimeofday(&ts);
			if (last_ts.tv_sec == 0) {
				ret = oplus_chg_read_filedata(&last_ts);
				if (ret < 0) {
					ret = oplus_chg_write_filedata(&ts);
					if (!ret) {
						last_ts.tv_sec = ts.tv_sec;
						count = 0;
					}
				}
				return 0;
			}
			rtc_time_to_tm(last_ts.tv_sec, &tm);
			chg_err("last date: %d-%d-%d %d:%d:%d\n",
					tm.tm_year + 1900,
					tm.tm_mon,
					tm.tm_mday,
					tm.tm_hour,
					tm.tm_min,
					tm.tm_sec
			       );
			rtc_time_to_tm(ts.tv_sec, &tm);
			chg_err("current date: %d-%d-%d %d:%d:%d\n",
					tm.tm_year + 1900,
					tm.tm_mon,
					tm.tm_mday,
					tm.tm_hour,
					tm.tm_min,
					tm.tm_sec
			       );
			if (((aging_cap_test > 0) && (chg_check_point_debug&OPEN_LOG_BIT)) ||
				(ts.tv_sec - last_ts.tv_sec > OPLUS_CHG_BATT_AGING_CHECK_TIME)) {
					//ret = power_supply_get_property(oplus_chg_debug_info.batt_psy,
							//POWER_SUPPLY_PROP_BATTERY_FCC, &pval);
					pval.intval = chip->batt_fcc;
					if (ret < 0)
						return -1;
					ret = oplus_chg_write_filedata(&ts);
					if (!ret) {
						last_ts.tv_sec = ts.tv_sec;
						count = 0;
						oplus_chg_set_chg_flag(OPLUS_NOTIFY_BATT_AGING_CAP);
						if (oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_BATT_AGING_CAP)) {
							oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_BATT_FCC);
							aging_cap_test = -1;
						}
					}
				}
				else {
					count = 0;
				}
		}
	return 0;
}
int oplus_chg_debug_set_soc_info(struct oplus_chg_chip *chip)
{
	if (chip->sleep_tm_sec > 0) {
		//suspend
		oplus_chg_debug_info.sleep_tm_sec = chip->sleep_tm_sec;
		oplus_chg_debug_info.cur_soc = chip->soc;
		oplus_chg_debug_info.cur_ui_soc = chip->ui_soc;
	}
	else {
		//resume
		oplus_chg_chg_batt_capacity_jump_check(chip);
	}
	return 0;
}
static void oplus_chg_soc_load_dwork(struct work_struct *work)
{
	if (oplus_chg_debug_info.soc_load_flag & (1 << OPLUS_NOTIFY_BATT_SOC_CAPCITY_LOAD_JUMP)) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_BATT_SOC_CAPCITY_LOAD_JUMP);
		if (oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_BATT_SOC_CAPCITY_LOAD_JUMP)) {
			oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP);
		}
	}
	if (oplus_chg_debug_info.soc_load_flag & (1 << OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_LOAD_JUMP)) {
		oplus_chg_set_chg_flag(OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_LOAD_JUMP);
		if (oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_LOAD_JUMP)) {
			oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP);
		}
	}
	oplus_chg_debug_info.soc_load_flag = 0;
}
static int oplus_chg_set_soc_notified_flag(int flag)
{
	oplus_chg_debug_info.soc_notified_flag |= (1 << flag);
	return 0;
}
static int oplus_chg_unset_soc_notified_flag(int flag)
{
	oplus_chg_debug_info.soc_notified_flag &= ~(1 << flag);
	return 0;
}
static int oplus_chg_soc_notified_flag_is_set(int flag)
{
	if (oplus_chg_debug_info.soc_notified_flag & (1 << flag))
		return 1;
	return 0;
}
static int oplus_chg_chg_batt_capacity_jump_check(struct oplus_chg_chip *chip)
{
	static ui_to_soc_jump_flag = false;
	union power_supply_propval pval = {0, };
	int status;
	power_supply_get_property(oplus_chg_debug_info.batt_psy,
			POWER_SUPPLY_PROP_STATUS, &pval);
	status = pval.intval;
	if (oplus_chg_debug_info.cur_soc == OPLUS_CHG_BATT_INVALID_CAPACITY) {
		oplus_chg_debug_info.pre_soc = chip->soc;
		oplus_chg_debug_info.cur_soc = chip->soc;
		oplus_chg_debug_info.cur_ui_soc = chip->ui_soc;
		oplus_chg_debug_info.pre_ui_soc = chip->ui_soc;
		if (abs(oplus_chg_debug_info.cur_soc - chip->soc_load) > OPLUS_CHG_BATT_S0C_CAPACITY_LOAD_JUMP_NUM) {
			oplus_chg_debug_info.soc_load_flag |= 1 << OPLUS_NOTIFY_BATT_SOC_CAPCITY_LOAD_JUMP;
		}
		if (abs(oplus_chg_debug_info.cur_ui_soc - chip->soc_load) > OPLUS_CHG_BATT_UI_S0C_CAPACITY_LOAD_JUMP_NUM) {
			oplus_chg_debug_info.soc_load_flag |= 1 << OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_LOAD_JUMP;
		}
		if (oplus_chg_debug_info.soc_load_flag) {
			//delay upload soc_load jump to wait for oplus_kevent deamon
			queue_delayed_work(oplus_chg_debug_info.oplus_chg_debug_wq,
					&oplus_chg_debug_info.soc_load_dwork, msecs_to_jiffies(SOC_LOAD_DELAY));
		}
	} else {
		oplus_chg_debug_info.pre_soc = oplus_chg_debug_info.cur_soc;
		oplus_chg_debug_info.pre_ui_soc = oplus_chg_debug_info.cur_ui_soc;
		if (chg_check_point_debug&OPEN_LOG_BIT) {
			if (fake_soc >= 0) {
				oplus_chg_debug_info.cur_soc = fake_soc;
			}else {
				oplus_chg_debug_info.cur_soc = chip->soc;
			}

			if (fake_ui_soc >= 0) {
				oplus_chg_debug_info.cur_ui_soc = fake_ui_soc;
			}else {
				oplus_chg_debug_info.cur_ui_soc = chip->ui_soc;
			}
		} else {
			oplus_chg_debug_info.cur_soc = chip->soc;
			oplus_chg_debug_info.cur_ui_soc = chip->ui_soc;
		}
		if ((abs(oplus_chg_debug_info.cur_soc - oplus_chg_debug_info.pre_soc) > OPLUS_CHG_BATT_S0C_CAPACITY_JUMP_NUM)
				&& (!oplus_chg_soc_notified_flag_is_set(OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP))) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP);
			if (oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP)) {
				oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP);
				oplus_chg_set_soc_notified_flag(OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP);
				strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP].reason);
			}
		}else {
			if (oplus_chg_debug_info.cur_soc == oplus_chg_debug_info.pre_soc) {
				oplus_chg_unset_soc_notified_flag(OPLUS_NOTIFY_BATT_SOC_CAPCITY_JUMP);
			}
			if ((abs(oplus_chg_debug_info.cur_ui_soc - oplus_chg_debug_info.pre_ui_soc) > OPLUS_CHG_BATT_UI_S0C_CAPACITY_JUMP_NUM)
					&& (!oplus_chg_soc_notified_flag_is_set(OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP))) {
				oplus_chg_set_chg_flag(OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP);
				if (oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP)) {
					oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP);
					oplus_chg_set_soc_notified_flag(OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP);
					strcpy(oplus_chg_debug_info.flag_reason,
						oplus_chg_debug_notify_policy[OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP].reason);
				}
			}else {
				if (oplus_chg_debug_info.cur_ui_soc == oplus_chg_debug_info.pre_ui_soc) {
					oplus_chg_unset_soc_notified_flag(OPLUS_NOTIFY_BATT_UI_SOC_CAPCITY_JUMP);
				}
				if ((abs(oplus_chg_debug_info.cur_ui_soc - oplus_chg_debug_info.cur_soc) > OPLUS_CHG_BATT_UI_TO_S0C_CAPACITY_JUMP_NUM)
						&& (!oplus_chg_soc_notified_flag_is_set(OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP))
						&& ui_to_soc_jump_flag == false) {
					oplus_chg_set_chg_flag(OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP);
					if (oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP)) {
						oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP);
						oplus_chg_set_soc_notified_flag(OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP);
						strcpy(oplus_chg_debug_info.flag_reason,
							oplus_chg_debug_notify_policy[OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP].reason);
						ui_to_soc_jump_flag = true;
					}
				}else {
					if (oplus_chg_debug_info.cur_ui_soc == oplus_chg_debug_info.cur_soc) {
						ui_to_soc_jump_flag = false;
						oplus_chg_unset_soc_notified_flag(OPLUS_NOTIFY_BATT_UI_TO_SOC_CAPCITY_JUMP);
					}
					if ((status == POWER_SUPPLY_STATUS_FULL)&&(oplus_chg_debug_info.cur_soc != 100)&&(!(oplus_chg_debug_info.chg_full_notified_flag & (1 << 0)))) {
						oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHG_BATT_FULL_NON_100_CAP);
						if (oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHG_BATT_FULL_NON_100_CAP)) {
							oplus_chg_debug_info.chg_full_notified_flag |= (1 << 0);
							oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP);
							strcpy(oplus_chg_debug_info.flag_reason,
								oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHG_BATT_FULL_NON_100_CAP].reason);
						}
					}else {
						oplus_chg_debug_info.chg_full_notified_flag = 0;
					}
				}
			}
		}
		if (!oplus_chg_debug_notify_type_is_set(OPLUS_CHG_DEBUG_NOTIFY_TYPE_SOC_JUMP)) {
			oplus_chg_debug_info.sleep_tm_sec = 0;
		}else {
			oplus_chg_print_debug_info(chip);
			send_info_flag = SEND_INFO_FLAG_WORK;
		}

	}
	return 0;
}
static int oplus_chg_mcu_update_check(struct oplus_chg_chip *chip) {
	static flag = false;
	if((charger_abnormal_log == CRITICAL_LOG_VOOC_FW_UPDATE_ERR && flag == false)
		|| (mcu_update_flag == 1 && (chg_check_point_debug&OPEN_LOG_BIT))){
		flag = true;
		if (!oplus_chg_debug_notify_type_is_set(OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR)) {
			oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR);
			if (!oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_MCU_UPDATE_FAIL)) {
				oplus_chg_set_chg_flag(OPLUS_NOTIFY_MCU_UPDATE_FAIL);
				strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[OPLUS_NOTIFY_MCU_UPDATE_FAIL].reason);
				oplus_chg_print_debug_info(g_debug_oplus_chip);
			}
		}
	}
	return 0;
}
void oplus_chg_gauge_seal_unseal_fail( int type ) {
	if (!oplus_chg_debug_notify_type_is_set(OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR)) {
		oplus_chg_set_notify_type(OPLUS_CHG_DEBUG_NOTIFY_TYPE_GAUGE_ERROR);
		if(type == OPLUS_GAUGE_SEAL_FAIL) {
			if (!oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_GAUGE_SEAL_FAIL)) {
				oplus_chg_set_chg_flag(OPLUS_NOTIFY_GAUGE_SEAL_FAIL);
				strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[OPLUS_NOTIFY_GAUGE_SEAL_FAIL].reason);
				oplus_chg_print_debug_info(g_debug_oplus_chip);
				send_info_flag = SEND_INFO_FLAG_IRQ;
			}
		}
		if(type == OPLUS_GAUGE_UNSEAL_FIAL) {
			if (!oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_GAUGE_UNSEAL_FAIL)) {
				oplus_chg_set_chg_flag(OPLUS_NOTIFY_GAUGE_UNSEAL_FAIL);
				strcpy(oplus_chg_debug_info.flag_reason,
					oplus_chg_debug_notify_policy[OPLUS_NOTIFY_GAUGE_UNSEAL_FAIL].reason);
				oplus_chg_print_debug_info(g_debug_oplus_chip);
				send_info_flag = SEND_INFO_FLAG_IRQ;
			}
		}
	}
}
void oplus_chg_vooc_mcu_error( int error ) {
	if(error){
		if (!oplus_chg_debug_notify_flag_is_set(OPLUS_NOTIFY_CHARGER_INFO)) {
			oplus_chg_set_chg_flag(OPLUS_NOTIFY_CHARGER_INFO);
			strcpy(oplus_chg_debug_info.flag_reason,
				oplus_chg_debug_notify_policy[OPLUS_NOTIFY_CHARGER_INFO].reason);
			oplus_chg_debug_info.vooc_mcu_error = error;
			oplus_chg_print_debug_info(g_debug_oplus_chip);
			send_info_flag = SEND_INFO_FLAG_IRQ;
		}
	}
	return ;
}
int oplus_chg_debug_chg_monitor(struct oplus_chg_chip *chip)
{
	int rc = 0;
	if (!oplus_chg_debug_info.initialized)
		return -1;
	if(g_debug_oplus_chip == NULL)
		g_debug_oplus_chip = chip;
	if(send_info_flag)
		goto end;
	if (chip->authenticate) {
		oplus_chg_chg_batt_aging_check(chip);
		if(send_info_flag)
			goto end;
		oplus_chg_chg_batt_capacity_jump_check(chip);
		if(send_info_flag)
			goto end;
		oplus_chg_mcu_update_check(chip);
		if(send_info_flag)
			goto end;
	}
	oplus_chg_break_check(chip);
	if(send_info_flag)
				goto end;
	oplus_chg_chg_slow_check(chip);
	if(send_info_flag)
			goto end;
	oplus_chg_chg_ic_check(chip);
end:
	send_info_flag = false;
	return rc;
}
void oplus_chg_set_fast_chg_type(int value) {
	oplus_chg_debug_info.fast_chg_type = value;
	return ;
}
static int oplus_chg_debug_notify_flag_is_set(int flag)
{
	int ret = 0;
	mutex_lock(&oplus_chg_debug_info.nflag_lock);
	if (oplus_chg_debug_info.notify_flag & (1L << flag)) {
		ret = 1;
	}
	mutex_unlock(&oplus_chg_debug_info.nflag_lock);
	return ret;
}
static int oplus_chg_debug_reset_notify_flag(void)
{
	mutex_lock(&oplus_chg_debug_info.nflag_lock);
	oplus_chg_debug_info.notify_flag = 0;
	mutex_unlock(&oplus_chg_debug_info.nflag_lock);
	return 0;
}
static int oplus_chg_debug_info_parse_dt(void)
{
	struct oplus_vooc_chip *vooc_chip = NULL;
	struct device_node *vooc_node = NULL;
	struct device_node *node, *child;
	int ret;
	int i, j, k;
	int count;
	int size;
	u32 val[2];
	char *vooc_adpater_name[] = {
		"vooc_charge_strategy_20w",
		"vooc_charge_strategy_30w",
		"vooc_charge_strategy_33w",
		"vooc_charge_strategy_50w",
		"vooc_charge_strategy_65w"
	};
	oplus_vooc_get_vooc_chip_handle(&vooc_chip);
	if (vooc_chip == NULL)
		return 0;
	vooc_node = vooc_chip->dev->of_node;
	if (!vooc_node)
		return 0;
	ret = of_property_read_u32(vooc_node, "qcom,vooc-max-input-volt-support", &oplus_chg_debug_info.vooc_max_input_volt);
	if (ret) {
		oplus_chg_debug_info.vooc_max_input_volt = 5000;
	}
	ret = of_property_read_u32(vooc_node, "qcom,vooc-max-input-current-support", &oplus_chg_debug_info.vooc_max_input_current);
	if (ret) {
		oplus_chg_debug_info.vooc_max_input_current = 4000;
	}
	for(i = 0; i < OPLUS_FASTCHG_OUTPUT_POWER_MAX; i++) {
		node = of_find_node_by_name(vooc_node, vooc_adpater_name[i]);
		if (!node)
			continue;
		count = 0;
		for_each_child_of_node(node, child)
			count++;
		if (!count)
			continue;
		g_vooc_charge_strategy_p[i] = kzalloc((count + 1) * sizeof(struct vooc_charge_strategy), GFP_KERNEL);
		j = 0;
		for_each_child_of_node(node, child)
		{
			ret = of_property_read_u32_array(child, "capacity_range", val, 2);
			if (!ret) {
				g_vooc_charge_strategy_p[i][j].capacity_range.low = val[0];
				g_vooc_charge_strategy_p[i][j].capacity_range.high = val[1];
			}
			ret = of_property_read_u32_array(child, "temp_range", val, 2);
			if (!ret) {
				g_vooc_charge_strategy_p[i][j].temp_range.low = val[0];
				g_vooc_charge_strategy_p[i][j].temp_range.high = val[1];
			}
			size = of_property_count_elems_of_size(child, "input_current", sizeof(u32));
			if (size >= 0) {
				for(k = 0; (k < (size / 3)) && (k < OPLUS_CHG_VOOC_INPUT_CURRENT_MAX_CNT); k++)
				{
					ret = of_property_read_u32_index(child, "input_current", 3 * k, &val[0]);
					if (!ret) {
						g_vooc_charge_strategy_p[i][j].input_current[k].cur = val[0];
					}
					ret = of_property_read_u32_index(child, "input_current", 3 * k + 1, &val[0]);
					if (!ret) {
						g_vooc_charge_strategy_p[i][j].input_current[k].vol = val[0];
					}
					ret = of_property_read_u32_index(child, "input_current", 3 * k + 2, &val[0]);
					if (!ret) {
						g_vooc_charge_strategy_p[i][j].input_current[k].time = val[0];
					}
				}
			}
			j++;
		}
		g_vooc_charge_strategy_p[i][count].capacity_range.low = -1;
		g_vooc_charge_strategy_p[i][count].capacity_range.high = -1;
	}
	return 0;
}
static ssize_t charge_monitor_test_write(struct file *filp,
		const char __user *buf, size_t len, loff_t *data)
{
	char tmp_buf[32] = {0};
	int str_len;
	str_len = len;
	if (str_len >= sizeof(tmp_buf))
		str_len = sizeof(tmp_buf) - 1;
	if (copy_from_user(tmp_buf, buf, str_len))
	{
		chg_err("copy_from_user failed.\n");
		return -EFAULT;
	}
	chg_err("%s\n", tmp_buf);
	if (!strncmp(tmp_buf, "soc:", strlen("soc:"))) {
		sscanf(tmp_buf, "soc:%d", &fake_soc);
	}else if (!strncmp(tmp_buf, "ui_soc:", strlen("ui_soc:"))) {
		sscanf(tmp_buf, "ui_soc:%d", &fake_ui_soc);
	}else if (!strncmp(tmp_buf, "aging_cap:", strlen("aging_cap:"))) {
		sscanf(tmp_buf, "aging_cap:%d", &aging_cap_test);
	}else if (!strncmp(tmp_buf, "break:", strlen("break:"))) {
		sscanf(tmp_buf, "break:%d", &break_flag);
	}else if (!strncmp(tmp_buf, "mcu_update:", strlen("mcu_update:"))) {
		sscanf(tmp_buf, "mcu_update:%d", &mcu_update_flag);
	}else if(!strncmp(tmp_buf, "gauge_seal:", strlen("gauge_seal:"))) {
		sscanf(tmp_buf, "gauge_seal:%d", &gauge_seal_flag);
		if(gauge_seal_flag == OPLUS_GAUGE_SEAL_FAIL
			|| gauge_seal_flag == gauge_seal_flag == OPLUS_GAUGE_UNSEAL_FIAL){
			oplus_chg_gauge_seal_unseal_fail(gauge_seal_flag);
		}
	}
	return len;
}
static ssize_t charge_monitor_test_read(struct file *filp,
		char __user *buff, size_t count, loff_t *off)
{
	int len = 0;
	char buf[512] = {0};
	sprintf(buf, "fake_soc:%d, fake_ui_soc:%d, aging_cap_test:%d\n",
			fake_soc, fake_ui_soc, aging_cap_test);
	len = strlen(buf);
	if (len > *off)
		len -= *off;
	else
		len = 0;
	if (copy_to_user(buff, buf, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}
static const struct file_operations charge_monitor_test_fops = {
	.write = charge_monitor_test_write,
	.read = charge_monitor_test_read,
};
static int charge_monitor_test_init(void)
{
	struct proc_dir_entry *proc_dentry = NULL;
	proc_dentry = proc_create("charge_monitor_test", 0664, NULL,
			&charge_monitor_test_fops);
	if (!proc_dentry) {
		pr_err("proc_create charge_monitor_test fops fail!\n");
		return -1;
	}
	return 0;
}
static int oplus_chg_debug_info_reset(void)
{
	int i;
	chg_err("enter\n");

	for(i = OPLUS_NOTIFY_CHG_SLOW_BATT_WARM_TEMP; i <= OPLUS_NOTIFY_CHG_SLOW_LED_ON_LONG_TIME; i++)
	{
		oplus_chg_debug_info.chg_cnt[i] = 0;
	}

	oplus_chg_debug_info.fast_chg_type = FASTCHG_CHARGER_TYPE_UNKOWN;
	oplus_chg_debug_info.fastchg_stop_cnt = 0;
	oplus_chg_debug_info.cool_down_by_user = 0;
	oplus_chg_debug_info.chg_current_by_cooldown = 0;
	oplus_chg_debug_info.chg_current_by_tbatt = 0;
	oplus_chg_debug_info.fastchg_input_current = -1;
	oplus_chg_debug_info.vooc_charge_strategy = NULL;
	oplus_chg_debug_info.vooc_charge_input_current_index = 0;
	oplus_chg_debug_info.vooc_charge_cur_state_chg_time = 0;
	oplus_chg_debug_info.chg_full_notified_flag = 0;
	oplus_chg_debug_info.total_time = 0;
	oplus_chg_debug_info.total_time_count = 0;
	oplus_chg_debug_reset_notify_flag();
	oplus_chg_reset_chg_notify_type();
	return 0;
}
int oplus_chg_debug_info_init(void)
{
	int ret = 0;
	memset(&oplus_chg_debug_info, 0x0, sizeof(oplus_chg_debug_info));
	memset(&oplus_chg_debug_msg[0], 0x0, sizeof(oplus_chg_debug_msg));
	mutex_init(&oplus_chg_debug_info.nflag_lock);
	oplus_chg_debug_info_reset();
	oplus_chg_debug_info.pre_led_state = 1;
	oplus_chg_debug_info.pre_soc = OPLUS_CHG_BATT_INVALID_CAPACITY;
	oplus_chg_debug_info.cur_soc = OPLUS_CHG_BATT_INVALID_CAPACITY;
	oplus_chg_debug_info.pre_ui_soc = OPLUS_CHG_BATT_INVALID_CAPACITY;
	oplus_chg_debug_info.cur_ui_soc = OPLUS_CHG_BATT_INVALID_CAPACITY;
	oplus_chg_debug_info.vooc_mcu_error = 0;
	oplus_chg_debug_info.usb_psy = power_supply_get_by_name("usb");
	oplus_chg_debug_info.batt_psy = power_supply_get_by_name("battery");
	oplus_chg_debug_info.oplus_chg_debug_wq = create_workqueue("oplus_chg_debug_wq");
#ifdef CONFIG_OPLUS_FEATURE_FEEDBACK
	oplus_chg_debug_info.dcs_info = (struct kernel_packet_info *)&oplus_chg_debug_msg[0];
	INIT_DELAYED_WORK(&oplus_chg_debug_info.send_info_dwork, oplus_chg_send_info_dwork);
	mutex_init(&oplus_chg_debug_info.dcs_info_lock);
#endif
	INIT_DELAYED_WORK(&oplus_chg_debug_info.soc_load_dwork, oplus_chg_soc_load_dwork);

	oplus_chg_debug_info_parse_dt();
	ret = charge_monitor_test_init();
	if (ret < 0)
		return ret;

	oplus_chg_debug_info.initialized = 1;
	return 0;
}

