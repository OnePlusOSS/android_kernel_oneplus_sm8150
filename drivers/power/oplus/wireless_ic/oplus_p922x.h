/************************************************************************************
** File:  oplus_p922x.h
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      for wireless charge
** 
** Version: 1.0
** Date created: 21:03:46,06/11/2018
** Author: Lin Shangbo
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2018-11-06    Lin Shangbo   		Created for wireless charge
************************************************************************************************************/
#ifndef __OPLUS_P922X_H__
#define __OPLUS_P922X_H__

#include "../oplus_wireless.h"

#define P922X_REG_RTX_STATUS                                 0x78
#define P922X_RTX_READY                                     BIT(0)
#define P922X_RTX_DIGITALPING                               BIT(1)
#define P922X_RTX_ANALOGPING                                BIT(2)
#define P922X_RTX_TRANSFER                                  BIT(3)

#define P922X_REG_RTX_ERR_STATUS                            0x79
#define P922X_REG_RTX_ERR_TX_RXAC                           BIT(0)
#define P922X_REG_RTX_ERR_TX_OCP                            BIT(1)
#define P922X_REG_RTX_ERR_TX_OVP                            BIT(2)
#define P922X_REG_RTX_ERR_TX_LVP                            BIT(3)
#define P922X_REG_RTX_ERR_TX_FOD                            BIT(4)
#define P922X_REG_RTX_ERR_TX_OTP                            BIT(5)
#define P922X_REG_RTX_ERR_TX_CEPTIMEOUT                     BIT(6)
#define P922X_REG_RTX_ERR_TX_RXEPT                          BIT(7)


#define P922X_PRE2CC_CHG_THD_LO                             3400
#define P922X_PRE2CC_CHG_THD_HI                             3420

#define P922X_CC2CV_CHG_THD_LO                              4350
#define P922X_CC2CV_CHG_THD_HI                              4370

/*The message to WPC DOCK */
#define P9221_CMD_INDENTIFY_ADAPTER							0xA1 
#define P9221_CMD_INTO_FASTCHAGE							0xA2
#define P9221_CMD_INTO_USB_CHARGE							0xA3
#define P9221_CMD_INTO_NORMAL_CHARGE						0xA4
#define P9221_CMD_SET_FAN_WORK								0xA5
#define P9221_CMD_SET_FAN_SILENT							0xA6
#define P9221_CMD_RESET_SYSTEM								0xA9
#define P9221_CMD_ENTER_TEST_MODE							0xAA
#define P9221_CMD_GET_FW_VERSION							0xAB
#define P9221_CMD_SET_LED_BRIGHTNESS						0xAC
#define P9221_CMD_SET_CEP_TIMEOUT							0xAD
#define P9221_CMD_SET_PWM_PULSE								0xAE
#define P9221_CMD_SEND_1ST_RANDOM_DATA						0xB1
#define P9221_CMD_SEND_2ND_RANDOM_DATA						0xB2
#define P9221_CMD_SEND_3RD_RANDOM_DATA						0xB3
#define P9221_CMD_GET_1ST_ENCODE_DATA						0xB4
#define P9221_CMD_GET_2ND_ENCODE_DATA						0xB5
#define P9221_CMD_GET_3RD_ENCODE_DATA						0xB6
#define P9221_CMD_NULL										0x00

/*The message from WPC DOCK */
#define P9237_RESPONE_ADAPTER_TYPE							0xF1
#define P9237_RESPONE_INTO_FASTCHAGE						0xF2
#define P9237_RESPONE_INTO_USB_CHARGE						0xF3
#define P9237_RESPONE_INTO_NORMAL_CHARGER					0xF4
#define P9237_RESPONE_FAN_WORK								0xF5
#define P9237_RESPONE_FAN_SILENT							0xF6
#define P9237_RESPONE_WORKING_IN_EPP						0xF8
#define P9237_RESPONE_RESET_SYSTEM							0xF9
#define P9237_RESPONE_ENTER_TEST_MODE						0xFA
#define P9237_RESPONE_FW_VERSION							0xFB
#define P9237_RESPONE_LED_BRIGHTNESS						0xFC
#define P9237_RESPONE_CEP_TIMEOUT							0xFD
#define P9237_RESPONE_PWM_PULSE								0xFE
#define P9237_RESPONE_RX_1ST_RANDOM_DATA					0xE1
#define P9237_RESPONE_RX_2ND_RANDOM_DATA					0xE2
#define P9237_RESPONE_RX_3RD_RANDOM_DATA					0xE3
#define P9237_RESPONE_SEND_1ST_DECODE_DATA					0xE4
#define P9237_RESPONE_SEND_2ND_DECODE_DATA					0xE5
#define P9237_RESPONE_SEND_3RD_DECODE_DATA					0xE6
#define P9237_RESPONE_NULL									0x00


#define P922X_TASK_INTERVAL							round_jiffies_relative(msecs_to_jiffies(500))	
#define P922X_CEP_INTERVAL							round_jiffies_relative(msecs_to_jiffies(200))
#define P922X_UPDATE_INTERVAL							round_jiffies_relative(msecs_to_jiffies(3000))
#define P922X_UPDATE_RETRY_INTERVAL						round_jiffies_relative(msecs_to_jiffies(500))

//extern struct oplus_chg_chip *p922x_chip;
struct oplus_p922x_ic{
	struct i2c_client				 *client;
	struct device					 *dev;

	struct power_supply *wireless_psy;
	enum power_supply_type wireless_type;
	enum wireless_mode wireless_mode;
	bool disable_charge;
	int quiet_mode_need;


	int				idt_en_gpio;		//for WPC
	int				idt_con_gpio;	//for WPC
	int				idt_con_irq;	//for WPC
	int				idt_int_gpio;	//for WPC
	int				idt_int_irq;	//for WPC
	int				vbat_en_gpio;	//for WPC
	int				booster_en_gpio;	//for WPC
	int             ext1_wired_otg_en_gpio;
	int             ext2_wireless_otg_en_gpio;
	int             cp_ldo_5v_gpio;
    struct pinctrl                       *pinctrl;
	struct pinctrl_state 		*idt_con_active;	//for WPC
	struct pinctrl_state 		*idt_con_sleep;		//for WPC
	struct pinctrl_state 		*idt_con_default;	//for WPC
	struct pinctrl_state 		*idt_int_active;	//for WPC
	struct pinctrl_state 		*idt_int_sleep;		//for WPC
	struct pinctrl_state 		*idt_int_default;	//for WPC
	struct pinctrl_state 		*vbat_en_active;	//for WPC
	struct pinctrl_state 		*vbat_en_sleep;		//for WPC
	struct pinctrl_state 		*vbat_en_default;	//for WPC
	struct pinctrl_state 		*booster_en_active;	//for WPC
	struct pinctrl_state 		*booster_en_sleep;		//for WPC
	struct pinctrl_state 		*booster_en_default;	//for WPC
	struct pinctrl_state 		*idt_con_out_active;	//for WPC
	struct pinctrl_state 		*idt_con_out_sleep;		//for WPC
	struct pinctrl_state 		*idt_con_out_default;	//for WPC

	struct pinctrl_state 		*ext1_wired_otg_en_active;	//for WPC
	struct pinctrl_state 		*ext1_wired_otg_en_sleep;		//for WPC
	struct pinctrl_state 		*ext1_wired_otg_en_default;	//for WPC
	struct pinctrl_state 		*ext2_wireless_otg_en_active;	//for WPC
	struct pinctrl_state 		*ext2_wireless_otg_en_sleep;		//for WPC
	struct pinctrl_state 		*ext2_wireless_otg_en_default;	//for WPC
	struct pinctrl_state 		*cp_ldo_5v_active;	//for WPC
	struct pinctrl_state 		*cp_ldo_5v_sleep;		//for WPC
	struct pinctrl_state 		*cp_ldo_5v_default;	//for WPC

    struct delayed_work idt_con_work;   //for WPC
    struct delayed_work p922x_task_work;    //for WPC
    struct delayed_work p922x_CEP_work; //for WPC
    struct delayed_work p922x_update_work;  //for WPC
    struct delayed_work p922x_test_work;    //for WPC
    struct delayed_work idt_event_int_work; //for WPC
    struct delayed_work idt_connect_int_work;   //for WPC
    struct delayed_work idt_dischg_work;   //for WPC
    struct delayed_work p922x_self_reset_work;  //for WPC
    struct wpc_data p922x_chg_status;   //for WPC
    //int         batt_volt_2cell_max;    //for WPC
    //int         batt_volt_2cell_min;    //for WPC
    atomic_t                         suspended;
};


bool p922x_wpc_get_fast_charging(void);
bool p922x_wpc_get_ffc_charging(void);
bool p922x_wpc_get_normal_charging(void);
bool p922x_wpc_get_otg_charging(void);
void p922x_set_rtx_function_prepare(void);
void p922x_set_rtx_function(bool is_on);
void p922x_wpc_print_log(void);
void p922x_set_vbat_en_val(int value);
int p922x_get_vbat_en_val(void);
void p922x_set_booster_en_val(int value);
int p922x_get_booster_en_val(void);
bool p922x_wireless_charge_start(void);
void p922x_set_wireless_charge_stop(void);
bool p922x_firmware_is_updating(void);
void p922x_set_ext1_wired_otg_en_val(int value);
int p922x_get_ext1_wired_otg_en_val(void);
void p922x_set_ext2_wireless_otg_en_val(int value);
int p922x_get_ext2_wireless_otg_en_val(void);
void p922x_set_cp_ldo_5v_val(int value);
int p922x_get_cp_ldo_5v_val(void);
int p922x_wpc_get_adapter_type(void);
bool p922x_check_chip_is_null(void);
int p922x_set_tx_cep_timeout_1500ms(void);
int p922x_get_CEP_flag(struct oplus_p922x_ic * chip);
//int p922x_set_dock_fan_pwm_pulse(int pwm_pulse);
#endif

