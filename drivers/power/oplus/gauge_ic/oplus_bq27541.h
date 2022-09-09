/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oplus77_12015\kernel\battery\battery
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**          for dc-dc sn111008 charg
**
** Version: 1.0
** Date created: 21:03:46, 05/04/2012
** Author: Fanhong.Kong@ProDrv.CHG
**
** --------------------------- Revision History: ------------------------------------------------------------
* <version>           <date>                <author>                           <desc>
* Revision 1.0        2015-06-22        Fanhong.Kong@ProDrv.CHG          Created for new architecture
************************************************************************************************************/

#ifndef __OPLUS_BQ27541_H__
#define __OPLUS_BQ27541_H__


#define OPLUS_USE_FAST_CHARGER
#define DRIVER_VERSION			"1.1.0"

/* Bq28Z610 standard data commands */
#define Bq28Z610_REG_TI			0x0c
#define Bq28Z610_REG_AI			0x14

/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_INTTEMP		0x28
#define BQ27541_REG_CC			0x2a
#define BQ27541_REG_SOH			0x2E
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34
#define BQ27541_REG_DOD0		0x36
#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)
#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS			BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS		0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE		0x0001
#define BQ27541_SUBCMD_FW_VER			0x0002
#define BQ27541_SUBCMD_HW_VER			0x0003
#define BQ27541_SUBCMD_DF_CSUM			0x0004
#define BQ27541_SUBCMD_PREV_MACW		0x0007
#define BQ27541_SUBCMD_CHEM_ID			0x0008
#define BQ27541_SUBCMD_BD_OFFSET		0x0009
#define BQ27541_SUBCMD_INT_OFFSET		0x000a
#define BQ27541_SUBCMD_CC_VER			0x000b
#define BQ27541_SUBCMD_OCV			0x000c
#define BQ27541_SUBCMD_BAT_INS			0x000d
#define BQ27541_SUBCMD_BAT_REM			0x000e
#define BQ27541_SUBCMD_SET_HIB			0x0011
#define BQ27541_SUBCMD_CLR_HIB			0x0012
#define BQ27541_SUBCMD_SET_SLP			0x0013
#define BQ27541_SUBCMD_CLR_SLP			0x0014
#define BQ27541_SUBCMD_FCT_RES			0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG		0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG		0x0019
#define BQ27541_SUBCMD_SEALED			0x0020
#define BQ27541_SUBCMD_ENABLE_IT		0x0021
#define BQ27541_SUBCMD_DISABLE_IT		0x0023
#define BQ27541_SUBCMD_CAL_MODE			0x0040
#define BQ27541_SUBCMD_RESET			0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN		(-2731)
#define BQ27541_INIT_DELAY		((HZ)*1)


/*----------------------- Bq27411 standard data commands----------------------------------------- */

#define BQ27411_REG_CNTL			0x00
#define BQ27411_REG_TEMP			0x02
#define BQ27411_REG_VOLT			0x04
#define BQ27411_REG_FLAGS			0x06
#define BQ27411_REG_NAC				0x08
#define BQ27411_REG_FAC				0x0a
#define BQ27411_REG_RM				0x0c
#define BQ27411_REG_FCC				0x2c
#define BQ27411_REG_AI				0x10
#define BQ27411_REG_SI				0x12
#define BQ27411_REG_MLI				0x14
#define BQ27411_REG_AP				0x18
#define BQ27411_REG_SOC				0x1c
#define BQ27411_REG_INTTEMP			0x1e
#define BQ27411_REG_SOH				0x20
#define BQ27411_REG_FC			0x0e //add gauge reg print log start
#define BQ27411_REG_QM			0x16
#define BQ27411_REG_PD			0x1a
#define BQ27411_REG_RCU			0x28
#define BQ27411_REG_RCF			0x2a
#define BQ27411_REG_FCU			0x2c
#define BQ27411_REG_FCF			0x2e
#define BQ27411_REG_SOU			0x30
#define BQ27411_REG_DO0			0x66
#define BQ27411_REG_DOE			0x68
#define BQ27411_REG_TRM			0x6a
#define BQ27411_REG_PC			0x6c
#define BQ27411_REG_QS			0x6e //add gauge reg print log end 
#define BQ27411_FLAG_DSC			BIT(0)
#define BQ27411_FLAG_FC				BIT(9)
#define BQ27411_CS_DLOGEN			BIT(15)
#define BQ27411_CS_SS				BIT(13)

/* Bq27411 sub commands */
#define BQ27411_SUBCMD_CNTL_STATUS		0x0000
#define BQ27411_SUBCMD_DEVICE_TYPE		0x0001
#define BQ27411_SUBCMD_FW_VER			0x0002
#define BQ27411_SUBCMD_DM_CODE			0x0004
#define BQ27411_SUBCMD_CONFIG_MODE		0x0006
#define BQ27411_SUBCMD_PREV_MACW		0x0007
#define BQ27411_SUBCMD_CHEM_ID			0x0008
#define BQ27411_SUBCMD_SET_HIB			0x0011
#define BQ27411_SUBCMD_CLR_HIB			0x0012
#define BQ27411_SUBCMD_SET_CFG			0x0013
#define BQ27411_SUBCMD_SEALED			0x0020
#define BQ27411_SUBCMD_RESET			0x0041
#define BQ27411_SUBCMD_SOFTRESET		0x0042
#define BQ27411_SUBCMD_EXIT_CFG			0x0043
#define BQ27411_SUBCMD_ENABLE_DLOG		0x0018
#define BQ27411_SUBCMD_DISABLE_DLOG		0x0019
#define BQ27411_SUBCMD_ENABLE_IT		0x0021
#define BQ27411_SUBCMD_DISABLE_IT		0x0023
#define BQ27541_BQ27411_CMD_INVALID		0xFF

/*----------------------- Bq27541 standard data commands-----------------------------------------*/
#define BQ27541_BQ27411_REG_CNTL				0
#define BQ27541_BQ27411_CS_DLOGEN				BIT(15)
#define BQ27541_BQ27411_CS_SS					BIT(13)
#define BQ27541_BQ27411_SUBCMD_CTNL_STATUS		0x0000
#define BQ27541_BQ27411_SUBCMD_ENABLE_IT		0x0021
#define BQ27541_BQ27411_SUBCMD_ENABLE_DLOG		0x0018
#define BQ27541_BQ27411_SUBCMD_DEVICE_TYPE		0x0001
#define BQ27541_BQ27411_SUBCMD_FW_VER			0x0002
#define BQ27541_BQ27411_SUBCMD_DISABLE_DLOG		0x0019
#define BQ27541_BQ27411_SUBCMD_DISABLE_IT		0x0023

#define CAPACITY_SALTATE_COUNTER				4
#define CAPACITY_SALTATE_COUNTER_NOT_CHARGING	20
#define CAPACITY_SALTATE_COUNTER_80				30
#define CAPACITY_SALTATE_COUNTER_90				40
#define CAPACITY_SALTATE_COUNTER_95				60
#define CAPACITY_SALTATE_COUNTER_FULL			120

#define BATTERY_2700MA				0
#define BATTERY_3000MA				1
#define TYPE_INFO_LEN				8

#define DEVICE_TYPE_BQ27541			0x0541
#define DEVICE_TYPE_BQ27411			0x0421
#define DEVICE_TYPE_BQ28Z610		0xFFA5
#define DEVICE_TYPE_ZY0602			0x0602
#define DEVICE_TYPE_ZY0603			0xA5FF

#define DEVICE_BQ27541				0
#define DEVICE_BQ27411				1
#define DEVICE_BQ28Z610				2
#define DEVICE_ZY0602				3
#define DEVICE_ZY0603				4

#define DEVICE_TYPE_FOR_VOOC_BQ27541		0
#define DEVICE_TYPE_FOR_VOOC_BQ27411		1


#define CONTROL_CMD					0x00
#define CONTROL_STATUS				0x00
#define SEAL_POLLING_RETRY_LIMIT	100
/*#define BQ27541_UNSEAL_KEY			11151986   */
#define BQ27541_UNSEAL_KEY			0x11151986
#define BQ27411_UNSEAL_KEY			0x80008000

#define BQ27541_RESET_SUBCMD		0x0041
#define BQ27411_RESET_SUBCMD		0x0042
#define SEAL_SUBCMD					0x0020

#define BQ27411_CONFIG_MODE_POLLING_LIMIT	100
#define BQ27411_CONFIG_MODE_BIT				BIT(4)
#define BQ27411_BLOCK_DATA_CONTROL			0x61
#define BQ27411_DATA_CLASS_ACCESS			0x003e
#define BQ27411_CC_DEAD_BAND_ID				0x006b
#define BQ27411_CC_DEAD_BAND_ADDR			0x42
#define BQ27411_CHECKSUM_ADDR				0x60
#define BQ27411_CC_DEAD_BAND_POWERUP_VALUE	0x11
#define BQ27411_CC_DEAD_BAND_SHUTDOWN_VALUE	0x71

#define BQ27411_OPCONFIGB_ID				0x0040
#define BQ27411_OPCONFIGB_ADDR				0x42
#define BQ27411_OPCONFIGB_POWERUP_VALUE		0x07
#define BQ27411_OPCONFIGB_SHUTDOWN_VALUE	0x0f

#define BQ27411_DODATEOC_ID					0x0024
#define BQ27411_DODATEOC_ADDR				0x48
#define BQ27411_DODATEOC_POWERUP_VALUE		0x32
#define BQ27411_DODATEOC_SHUTDOWN_VALUE		0x32

/*----------------------- Bq27541 standard data commands-----------------------------------------*/
#define BQ28Z610_REG_CNTL1					0x3e
#define BQ28Z610_REG_CNTL2					0x60
#define BQ28Z610_SEAL_POLLING_RETRY_LIMIT	10

#define BQ28Z610_SEAL_STATUS				0x0054
#define BQ28Z610_SEAL_SUBCMD				0x0030
#define BQ28Z610_UNSEAL_SUBCMD1				0x0414
#define BQ28Z610_UNSEAL_SUBCMD2				0x3672
//#define BQ28Z610_SEAL_BIT		     (BIT(8) | BIT(9))
#define BQ28Z610_SEAL_BIT				(BIT(0) | BIT(1))

#define BQ28Z610_SEAL_SHIFT					8
#define BQ28Z610_SEAL_VALUE					3
#define BQ28Z610_MAC_CELL_VOLTAGE_ADDR		0x40
#define BQ28Z610_REG_CNTL1_SIZE				4

#define BQ28Z610_REG_I2C_SIZE				3

#define BQ28Z610_REG_GAUGE_EN				0x0057
#define BQ28Z610_GAUGE_EN_BIT				BIT(3)

#define BQ28Z610_MAC_CELL_VOLTAGE_EN_ADDR		0x3E
#define BQ28Z610_MAC_CELL_VOLTAGE_CMD			0x0071
#define BQ28Z610_MAC_CELL_VOLTAGE_ADDR			0x40
#define BQ28Z610_MAC_CELL_VOLTAGE_SIZE			4//total 34byte,only read 4byte(aaAA bbBB)

#define BQ28Z610_MAC_CELL_BALANCE_TIME_EN_ADDR	0x3E
#define BQ28Z610_MAC_CELL_BALANCE_TIME_CMD		0x0076
#define BQ28Z610_MAC_CELL_BALANCE_TIME_ADDR		0x40
#define BQ28Z610_MAC_CELL_BALANCE_TIME_SIZE		4//total 10byte,only read 4byte(aaAA bbBB)

#define BQ28Z610_OPERATION_STATUS_EN_ADDR		0x3E
#define BQ28Z610_OPERATION_STATUS_CMD			0x0054
#define BQ28Z610_OPERATION_STATUS_ADDR			0x40
#define BQ28Z610_OPERATION_STATUS_SIZE			4
#define BQ28Z610_BALANCING_CONFIG_BIT			BIT(28)

#define BQ28Z610_DEVICE_CHEMISTRY_EN_ADDR		0x3E
#define BQ28Z610_DEVICE_CHEMISTRY_CMD			0x4B
#define BQ28Z610_DEVICE_CHEMISTRY_ADDR			0x40
#define BQ28Z610_DEVICE_CHEMISTRY_SIZE			4

struct cmd_address {
/*      bq27411 standard cmds     */
	u8	reg_cntl;
	u8	reg_temp;
	u8	 reg_volt;
	u8	reg_flags;
	u8	reg_nac;
	u8	 reg_fac;
	u8	reg_rm;
	u8	reg_fcc;
	u8	reg_ai;
	u8	reg_si;
	u8	reg_mli;
	u8	reg_ap;
	u8	reg_soc;
	u8	reg_inttemp;
	u8	reg_soh;
	u8	reg_fc; //add gauge reg print log start
	u8	reg_qm;
	u8	reg_pd;
	u8	reg_rcu;
	u8	reg_rcf;
	u8	reg_fcu;
	u8	reg_fcf;
	u8	reg_sou;
	u8	reg_do0;
	u8	reg_doe;
	u8	reg_trm;
	u8	reg_pc;
	u8	reg_qs; //add gauge reg print log end
	u16	flag_dsc;
	u16	flag_fc;
	u16	cs_dlogen;
	u16 cs_ss;

/*     bq27541 external standard cmds      */
	u8	reg_ar;
	u8	reg_artte;
	u8	reg_tte;
	u8	reg_ttf;
	u8	reg_stte;
	u8	reg_mltte;
	u8	reg_ae;
	u8	reg_ttecp;
	u8	reg_cc;
	u8	reg_nic;
	u8	reg_icr;
	u8	reg_logidx;
	u8	reg_logbuf;
	u8	reg_dod0;


/*      bq27411 sub cmds       */
	u16 subcmd_cntl_status;
	u16 subcmd_device_type;
	u16 subcmd_fw_ver;
	u16 subcmd_dm_code;
	u16 subcmd_prev_macw;
	u16 subcmd_chem_id;
	u16 subcmd_set_hib;
	u16 subcmd_clr_hib;
	u16 subcmd_set_cfg;
	u16 subcmd_sealed;
	u16 subcmd_reset;
	u16 subcmd_softreset;
	u16 subcmd_exit_cfg;
	u16 subcmd_enable_dlog;
	u16 subcmd_disable_dlog;
	u16 subcmd_enable_it;
	u16 subcmd_disable_it;

/*      bq27541 external sub cmds     */
	u16 subcmd_hw_ver;
	u16 subcmd_df_csum;
	u16 subcmd_bd_offset;
	u16 subcmd_int_offset;
	u16 subcmd_cc_ver;
	u16 subcmd_ocv;
	u16 subcmd_bat_ins;
	u16 subcmd_bat_rem;
	u16 subcmd_set_slp;
	u16 subcmd_clr_slp;
	u16 subcmd_fct_res;
	u16 subcmd_cal_mode;
};

#define BQ27541_AUTHENTICATE_OK						0x56
#define AUTHEN_MESSAGE_MAX_COUNT					30
struct bq27541_authenticate_data {
	uint8_t result;
	uint8_t message_offset;
	uint8_t message_len;
	uint8_t message[AUTHEN_MESSAGE_MAX_COUNT];		// 25, larger than 20 bytes
} ;

#define BQ27541_AUTHENTICATE_DATA_COUNT			sizeof(struct bq27541_authenticate_data)

//#define SMEM_CHARGER_BATTERY_INFO	81
#define SMEM_RESERVED_BOOT_INFO_FOR_APPS       418
#define GAUGE_AUTH_MSG_LEN 20
typedef struct {
	int result;
	unsigned char msg[GAUGE_AUTH_MSG_LEN];
	unsigned char rcv_msg[GAUGE_AUTH_MSG_LEN];
} oplus_gauge_auth_result;

typedef struct {
	oplus_gauge_auth_result rst_k0;
	oplus_gauge_auth_result rst_k1;
} oplus_gauge_auth_info_type;

struct chip_bq27541 {
	struct i2c_client *client;
	struct device *dev;

	int soc_pre;
	int temp_pre;
	int batt_vol_pre;
	int current_pre;
	int cc_pre;
	int soh_pre;
	int fcc_pre;
	int rm_pre;
	int fc_pre; //add gauge reg print log start
	int qm_pre;
	int pd_pre;
	int rcu_pre;
	int rcf_pre;
	int fcu_pre;
	int fcf_pre;
	int sou_pre;
	int do0_pre;
	int doe_pre;
	int trm_pre;
	int pc_pre;
	int qs_pre; //add gauge reg print log end
	int device_type;
	int device_type_for_vooc;
	struct cmd_address cmd_addr;
	atomic_t suspended;
	int batt_cell_1_vol;
	int batt_cell_2_vol;
	int batt_cell_max_vol;
	int batt_cell_min_vol;
	int max_vol_pre;
	int min_vol_pre;
	/*struct  delayed_work		hw_config;*/

	int opchg_swtich1_gpio;
	int opchg_swtich2_gpio;
	int opchg_reset_gpio;
	int opchg_clock_gpio;
	int opchg_data_gpio;

	struct pinctrl  *pinctrl;
	struct pinctrl_state *gpio_switch1_act_switch2_act;
	struct pinctrl_state *gpio_switch1_sleep_switch2_sleep;
	struct pinctrl_state *gpio_switch1_act_switch2_sleep;
	struct pinctrl_state *gpio_switch1_sleep_switch2_act;
	struct pinctrl_state *gpio_clock_active;
	struct pinctrl_state *gpio_clock_sleep;
	struct pinctrl_state *gpio_data_active;
	struct pinctrl_state *gpio_data_sleep;
	struct pinctrl_state *gpio_reset_active;
	struct pinctrl_state *gpio_reset_sleep;

	bool modify_soc_smooth;
	bool modify_soc_calibration;
	
	bool battery_full_param;//only for wite battery full param in guage dirver probe on 7250 platform
	int sha1_key_index;
	struct delayed_work afi_update;
	bool afi_update_done;
	bool protect_check_done;
	bool disabled;
	bool error_occured;
	bool need_check;
	unsigned int afi_count;
	unsigned int zy_dts_qmax_min;
	unsigned int zy_dts_qmax_max;
	const u8 *static_df_checksum_3e;
	const u8 *static_df_checksum_60;
	const u8 **afi_buf;
	unsigned int *afi_buf_len;

	bool batt_bq28z610;
	bool batt_zy0603;
	bool bq28z610_need_balancing;
	int bq28z610_device_chem;
	struct bq27541_authenticate_data *authenticate_data;
	struct file_operations *authenticate_ops;
};

extern bool oplus_gauge_ic_chip_is_null(void);

#endif  /* __OPLUS_BQ27541_H__ */
