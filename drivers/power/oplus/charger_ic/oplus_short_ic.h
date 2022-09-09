/**********************************************************************************
* Copyright (c)  2017-2019  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: For short circuit battery check
* Version   : 1.0
* Date      : 2018-05-24
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        	<author>              		<desc>
***********************************************************************************/

#ifndef _OPLUS_SHORT_IC_H_
#define _OPLUS_SHORT_IC_H_


#define OPLUS_SHORT_IC_CHIP_ID_REG					0x00
#define OPLUS_SHORT_IC_TEMP_VOLT_DROP_THRESH_REG		0x02
#define OPLUS_SHORT_IC_WORK_MODE_REG					0x03
#define OPLUS_SHORT_IC_OTP_REG						0x08

#define OPLUS_SHORT_IC_TEMP_VOLT_DROP_THRESH_VAL		0x44
struct oplus_short_ic{
	struct i2c_client				 *client;
	struct device					 *dev;
	struct delayed_work 			 oplus_short_ic_init_work;

	bool 							 b_oplus_short_ic_exist;
	bool							 b_factory_id_get;
	int								 volt_drop_threshold;
	bool							 b_volt_drop_set;
	bool							 b_work_mode_set;
	int								 otp_error_cnt;
	int								 otp_error_value;
    atomic_t                         suspended;
};

bool oplus_short_ic_otp_check(void);
int oplus_short_ic_set_volt_threshold(struct oplus_chg_chip *chip);
bool oplus_short_ic_is_exist(struct oplus_chg_chip *chip);
int oplus_short_ic_get_otp_error_value(struct oplus_chg_chip *chip);

#endif

