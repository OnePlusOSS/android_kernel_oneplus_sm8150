/************************************************************************************
** File:  oplus_p922x.c
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      for wireless charge
** 
** Version: 1.0
** Date created: 21:03:46,06/11/2018
** Author: Lin Shangbo, Li Jiada
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2018-11-06    Lin Shangbo,Li Jiada   		Created for wireless charge
************************************************************************************************************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/rtc.h>
#include <soc/oplus/device_info.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include "../charger_ic/oplus_mp2650.h"
#include "../charger_ic/oplus_chargepump.h"
#include <oplus_sp6001.h>

int send_msg_timer = 0;
int send_step_timer = 0;

struct oplus_sp6001_ic *sp6001_chip = NULL;

static DEFINE_MUTEX(sp6001_i2c_access);

#define SP6001_ADD_COUNT      2
static int __sp6001_read_reg(struct oplus_sp6001_ic *chip, int reg, char *returnData, int count)
{
	/* We have 16-bit i2c addresses - care for endianness */
	char cmd_buf[2]={ reg >> 8, reg & 0xff };
	int ret = 0;
	int i;
	char val_buf[count];
	
	for (i = 0; i < count; i++) {
		val_buf[i] = 0;
	}

	ret = i2c_master_send(chip->client, cmd_buf, P22X_ADD_COUNT);
	if (ret < P22X_ADD_COUNT) {
		chg_err("%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(chip->client, val_buf, count);
	if (ret < count) {
		chg_err("%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	for (i = 0; i < count; i++) {
		*(returnData + i) = val_buf[i];
	}
	
	return 0;
}

static int __sp6001_write_reg(struct oplus_sp6001_ic *chip, int reg, int val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(chip->client, data, 3);
	if (ret < 3) {
		chg_err("%s: i2c write error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int sp6001_read_reg(struct oplus_sp6001_ic *chip, int reg, char *returnData, int count)
{
	int ret = 0;

	mutex_lock(&sp6001_i2c_access);
	ret = __sp6001_read_reg(chip, reg, returnData, count);
	mutex_unlock(&sp6001_i2c_access);
	return ret;
}

static int sp6001_write_reg(struct oplus_sp6001_ic *chip, int reg, int val)
{
	int ret = 0;

	mutex_lock(&sp6001_i2c_access);
	ret = __sp6001_write_reg(chip, reg, val);
	mutex_unlock(&sp6001_i2c_access);
	return ret;
}

static int sp6001_config_interface (struct oplus_sp6001_ic *chip, int RegNum, int val, int MASK)
{
	char sp6001_reg = 0;
	int ret = 0;

	mutex_lock(&sp6001_i2c_access);
	ret = __sp6001_read_reg(chip, RegNum, &sp6001_reg, 1);

	sp6001_reg &= ~MASK;
	sp6001_reg |= val;

	ret = __sp6001_write_reg(chip, RegNum, sp6001_reg);

	mutex_unlock(&sp6001_i2c_access);
	
	return ret;
}

void sp600_send_msg_ask_idt_pack1(struct oplus_sp6001_ic *chip)
{
	chg_err("<~WPC~>sp600_send_msg_ask_idt_pack1\n");
	
    p922x_write_reg(chip, WRITE_REG_00, 0x90);
    p922x_write_reg(chip, WRITE_REG_01, 0x38);
    p922x_write_reg(chip, WRITE_REG_02, 0x48);
    p922x_write_reg(chip, WRITE_REG_03, 0x00);
    p922x_write_reg(chip, WRITE_REG_04, 0x24);
    p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

void sp600_send_msg_ask_idt_pack2(struct oplus_sp6001_ic *chip)
{
	chg_err("<~WPC~>sp600_send_msg_ask_idt_pack2\n");

    p922x_write_reg(chip, WRITE_REG_00, 0x90);
    p922x_write_reg(chip, WRITE_REG_01, 0x38);
    p922x_write_reg(chip, WRITE_REG_02, 0x3B);
    p922x_write_reg(chip, WRITE_REG_03, 0x04);
    p922x_write_reg(chip, WRITE_REG_04, 0x44);

    p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

void sp600_send_msg_ask_idt_pack3(struct oplus_sp6001_ic *chip)
{
	chg_err("<~WPC~>sp600_send_msg_ask_idt_pack3\n");

    p922x_write_reg(chip, WRITE_REG_00, 0x90);
    p922x_write_reg(chip, WRITE_REG_01, 0x19);
    p922x_write_reg(chip, WRITE_REG_02, 0xFF);

    p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

void sp600_send_msg_ask_adapter_type(struct oplus_sp6001_ic *chip)
{
	chg_err("<~WPC~>sp600_send_msg_ask_adapter_type\n");

    p922x_write_reg(chip, WRITE_REG_00, 0x90);
    p922x_write_reg(chip, WRITE_REG_01, 0x48);
    p922x_write_reg(chip, WRITE_REG_02, 0xA1);
    p922x_write_reg(chip, WRITE_REG_03, 0x5E);
    p922x_write_reg(chip, WRITE_REG_04, 0xFF);
    p922x_write_reg(chip, WRITE_REG_05, 0x00);  

    p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

void sp600_send_msg_enter_fastchg_mode(struct oplus_sp6001_ic *chip)
{
	chg_err("<~WPC~>sp600_send_msg_enter_fastchg_mode\n");

    p922x_write_reg(chip, WRITE_REG_00, 0x90);
    p922x_write_reg(chip, WRITE_REG_01, 0x48);
    p922x_write_reg(chip, WRITE_REG_02, 0xA2);
    p922x_write_reg(chip, WRITE_REG_03, 0x5D);
    p922x_write_reg(chip, WRITE_REG_04, 0xFF);
    p922x_write_reg(chip, WRITE_REG_05, 0x00); 

    p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

void sp600_send_msg_enter_usb_mode(struct oplus_sp6001_ic *chip)
{  
	chg_err("<~WPC~>sp600_send_msg_enter_usb_mode\n");

    p922x_write_reg(chip, WRITE_REG_00, 0x90);
    p922x_write_reg(chip, WRITE_REG_01, 0x48);
    p922x_write_reg(chip, WRITE_REG_02, 0xA3);
    p922x_write_reg(chip, WRITE_REG_03, 0x5C);
    p922x_write_reg(chip, WRITE_REG_04, 0xFF);
    p922x_write_reg(chip, WRITE_REG_05, 0x00);

    p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

void sp600_send_msg_enter_normal_mode(struct oplus_sp6001_ic *chip)
{ 
	chg_err("<~WPC~>sp600_send_msg_enter_normal_mode\n");

    p922x_write_reg(chip, WRITE_REG_00, 0x90);
    p922x_write_reg(chip, WRITE_REG_01, 0x48);
    p922x_write_reg(chip, WRITE_REG_02, 0xA4);
    p922x_write_reg(chip, WRITE_REG_03, 0x5B);
    p922x_write_reg(chip, WRITE_REG_04, 0xFF);
    p922x_write_reg(chip, WRITE_REG_05, 0x00);

    p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

static void p922x_set_rx_charge_voltage(struct oplus_sp6001_ic *chip, int vol)
{
	char value_h, value_l;	
	int vout_set,vrect_set;

	if(vol < WPC_CHARGE_VOLTAGE_CHGPUMP_MIN) {
		vol = WPC_CHARGE_VOLTAGE_CHGPUMP_MIN;
		return;
	}
	else if(vol > WPC_CHARGE_VOLTAGE_CHGPUMP_MAX) {
		vol = WPC_CHARGE_VOLTAGE_CHGPUMP_MAX;
	}
	
	chip->p922x_chg_status.charge_voltage = vol;
	chg_err("<~WPC~> p922x_set_rx_charge_voltage: %d\n", vol);

	vout_set = (vol * 3352) >> 16;	
	vrect_set = ((vol + 200) * 2438) >> 16;	

	value_h = (char)(vout_set >> 8); 	
	value_l = (char)(vout_set & 0xFF);	
	p922x_write_reg(chip, WRITE_REG_01, value_h);
	p922x_write_reg(chip, WRITE_REG_02, value_l);

	value_h = (char)(vrect_set >> 8);	
	value_l = (char)(vrect_set & 0xFF);	
	p922x_write_reg(chip, WRITE_REG_03, value_h);
	p922x_write_reg(chip, WRITE_REG_04, value_l);
		
	p922x_write_reg(chip, WRITE_REG_00, 0x80);
	p922x_write_reg(chip, WRITE_REG_TX, 0x55); 
}

int sp6001_get_vrect_iout(struct oplus_sp6001_ic * chip)
{
	char vrect_value = 0;
	char vout_value = 0;
	char iout_value = 0;
	int i;
	int temp = 0;
  
	p922x_read_reg(chip, READ_REG_VRECT, &vrect_value, 1);
	chip->p922x_chg_status.vrect = (vrect_value * 27500) >> 8;
  	
	p922x_read_reg(chip, READ_REG_VOUT, &vout_value, 1);
	chip->p922x_chg_status.vout = (vout_value * 22500) >> 8;

	for (i = 1; i < 10; i++) {
		chip->p922x_chg_status.iout_array[i] = chip->p922x_chg_status.iout_array[i - 1];
	}
  
	p922x_read_reg(chip, READ_REG_IOUT, &iout_value, 1);
	iout_value = iout_value - 15;
	chip->p922x_chg_status.iout_array[0] = (iout_value * 25 * 128) >> 8;

	temp = 0;
	for (i = 0; i < 10; i++) {
		temp = temp + chip->p922x_chg_status.iout_array[i];
	}
  
	chip->p922x_chg_status.iout = temp / 10;
  
	chg_err("<~WPC~> Vout:%d,  Vrect:%d,	Iout:%d\n", chip->p922x_chg_status.vout, chip->p922x_chg_status.vrect, chip->p922x_chg_status.iout);  //just debug
	
	return 0;
}

static int p922x_detect_CEP(struct oplus_sp6001_ic * chip)
{
	int rc = -1;
	char temp = 0;
	//char freq[2] = {0, 0};//just debug

#ifdef WPC_USE_SP6001
	rc = p922x_read_reg(chip, READ_REG_CEP, &temp, 1);
#else
	rc = p922x_read_reg(chip, 0x0033, &temp, 1);
#endif
	if (rc) {
		chg_err("Couldn't read CEP rc = %x\n", rc);
		return rc;
	}

	if ((temp == 0) || (temp == 1)  || (temp == 0xFF)  || (temp == 0xFE)) {
		chg_err("<~WPC~> CEP value = %d. *OK*\n", temp);
		chip->p922x_chg_status.CEP_is_OK = true;
		chip->p922x_chg_status.CEP_now_OK = true;
		return 0;
	} else {	
		chg_err("<~WPC~> CEP value = %d\n", temp);
		chip->p922x_chg_status.CEP_now_OK = false;
	}
	
	return -1;
}

static int p922x_RXTX_message_process(struct oplus_sp6001_ic *chip)
{
#ifdef WPC_USE_SP6001
	if (send_msg_timer < 5) {
		send_msg_timer++;
		send_step_timer = 0;
	} else {
		send_step_timer++;
		if (chip->p922x_chg_status.adapter_type == ADAPTER_TYPE_UNKNOW) {
			if (send_step_timer == 2) {
				sp600_send_msg_ask_idt_pack1(chip);
			} else if (send_step_timer == 4) {
				sp600_send_msg_ask_idt_pack2(chip);
			} else if (send_step_timer == 6) {
				sp600_send_msg_ask_adapter_type(chip);
			} else if (send_step_timer > 6) {
				send_msg_timer = 0;
			}
		} else if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_FASTCHAGE) {
			if (send_step_timer == 2) {
				sp600_send_msg_ask_idt_pack3(chip);
			} else if (send_step_timer == 4) {
				sp600_send_msg_enter_fastchg_mode(chip);
			} else if (send_step_timer > 4) {
				send_msg_timer = 0;
			}
		} else if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_USB_CHARGE) {
			if (send_step_timer == 2) {
				sp600_send_msg_ask_idt_pack3(chip);
			} else if (send_step_timer == 4) {
				sp600_send_msg_enter_usb_mode(chip);
			} else if (send_step_timer > 4) {
				send_msg_timer = 0;
			}
		} else if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_NORMAL_CHARGE) {
			if (send_step_timer == 2) {
				sp600_send_msg_ask_idt_pack3(chip);
			} else if (send_step_timer == 4) {
				sp600_send_msg_enter_normal_mode(chip);
			} else if (send_step_timer > 4) {
				send_msg_timer = 0;
			}
		} else {
			send_msg_timer = 0;
		}
	}
#else
	send_msg_timer++;
	if (send_msg_timer > 5) {
		send_msg_timer = 0;

		if (chip->p922x_chg_status.adapter_type == ADAPTER_TYPE_UNKNOW) {
			p922x_set_tx_charger_dect(chip);
		} else if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_FASTCHAGE) {
			p922x_set_tx_charger_fastcharge(chip);
			mp2650_set_reg_test();
		} else if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_USB_CHARGE) {
			p922x_set_tx_charger_usb(chip);
		} else if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_NORMAL_CHARGE) {
			p922x_set_tx_charger_normal(chip);
		}
	}
#endif

	switch (chip->p922x_chg_status.tx_command) { 
	case P9237_RESPONE_ADAPTER_TYPE:
		chip->p922x_chg_status.adapter_type = chip->p922x_chg_status.tx_data;
		chg_err("<~WPC~> get adapter type = 0x%02X\n", chip->p922x_chg_status.adapter_type);
		if ((chip->p922x_chg_status.adapter_type == ADAPTER_TYPE_FASTCHAGE_VOOC)
			|| (chip->p922x_chg_status.adapter_type == ADAPTER_TYPE_FASTCHAGE_SVOOC)){
			chip->p922x_chg_status.fastchg_ing = true;
		}
		break;
		
	case P9237_RESPONE_INTO_FASTCHAGE:
		chip->p922x_chg_status.charge_type = WPC_CHARGE_TYPE_FAST;
		chg_err("<~WPC~> enter charge type = WPC_CHARGE_TYPE_FAST\n");
		if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_FASTCHAGE) {
			chip->p922x_chg_status.send_message = P9221_CMD_NULL;
		}
		break;
		
	case P9237_RESPONE_INTO_USB_CHARGE:
		chip->p922x_chg_status.charge_type = WPC_CHARGE_TYPE_USB;
		chg_err("<~WPC~> enter charge type = WPC_CHARGE_TYPE_USB\n");
		if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_USB_CHARGE) {
			chip->p922x_chg_status.send_message = P9221_CMD_NULL;
		}
		break;
		
	case P9237_RESPONE_INTO_NORMAL_CHARGER:
		chip->p922x_chg_status.charge_type = WPC_CHARGE_TYPE_NORMAL;
		chg_err("<~WPC~> enter charge type = WPC_CHARGE_TYPE_NORMAL\n");
		if (chip->p922x_chg_status.send_message == P9221_CMD_INTO_NORMAL_CHARGE) {
			chip->p922x_chg_status.send_message = P9221_CMD_NULL;
		}
		break;

	default:
		break;
	}

	chip->p922x_chg_status.tx_command = P9237_RESPONE_NULL;
	return 0;
}

int sp6001_init_registers(void)
{
	if (!sp6001_chip) {
		chg_err("<~WPC~> sp6001_chip is NULL\n");
		return -1;
	}

	return 0;
}


void sp6001_commu_data_process(char *tx_command, char *tx_data)
{
	char temp[2] = {0, 0};
	char val_buf[5] = {0,0,0,0,0};

	chg_err("<linshangbo> RX message int@@@@@@@@@@@@@@@\n");

	if (!sp6001_chip) {
		*tx_command = 0;
		*tx_data = 0;
		chg_err("<~WPC~> sp6001_chip is NULL\n");
		return;
	}
	
	p922x_read_reg(sp6001_chip, READ_REG_00, &val_buf[0], 1);
	p922x_read_reg(sp6001_chip, READ_REG_01, &val_buf[1], 1);
	p922x_read_reg(sp6001_chip, READ_REG_02, &val_buf[2], 1);
	p922x_read_reg(sp6001_chip, READ_REG_03, &val_buf[3], 1);
	p922x_read_reg(sp6001_chip, READ_REG_04, &val_buf[4], 1);
	
	p922x_write_reg(sp6001_chip, WRITE_REG_TX, 0xAA); 

	chg_err("<linshangbo> 1st RX Data: 0x%02X  0x%02X  0x%02X  0x%02X  0x%02X \n",
			val_buf[0], val_buf[1], val_buf[2], val_buf[3], val_buf[4]);
	
	temp[0] = ~val_buf[2];
	temp[1] = ~val_buf[4];
	if ((val_buf[0] == 0x4F) && (val_buf[1] == temp[0]) && (val_buf[3] == temp[1])) {
		*tx_command = val_buf[1];
		*tx_data = val_buf[3];
		chg_err("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT\n");
		chg_err("<~WPC~> Received TX command: 0x%02X, data: 0x%02X\n", *tx_command, *tx_data);
		chg_err("TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT\n");
	} else {
		*tx_command = 0;
		*tx_data = 0;
	}
}

bool sp6001_device_reconize(void)
{
	char read_value;
	int ret;
	
	if (!sp6001_chip) {
		chg_err("<~WPC~> sp6001_chip is NULL\n");
		return false;
	}

	ret = sp6001_read_reg(sp6001_chip, READ_REG_00, &read_value, 1);
	if (ret == 0) {
		chg_err("<~WPC~> sp6001_chip has been found!\n");
		return true;
	} else {
		chg_err("<~WPC~> sp6001_chip doesn't appear!\n");
		return false;
	}
}

static int sp6001_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
	struct oplus_sp6001_ic	*chip;

	chg_debug( " call \n");

	if (oplus_chg_check_chip_is_null() == true) {
		chg_debug( " g_oplus_chg chip is null, probe again \n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev,
		sizeof(struct oplus_sp6001_ic), GFP_KERNEL);
	if (!chip) {
		chg_err(" kzalloc() failed\n");
		return -ENOMEM;
	}
	
	chip->client = client;
	chip->dev = &client->dev;
	
	sp6001_chip = chip;

	chg_debug( " call end\n");

	return 0;                                                                                       

}


static struct i2c_driver sp6001_i2c_driver;

static int sp6001_driver_remove(struct i2c_client *client)
{    
	return 0;
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
static int sp6001_pm_resume(struct device *dev)
{
	return 0;
}

static int sp6001_pm_suspend(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops sp6001_pm_ops = {
	.resume		= sp6001_pm_resume,
	.suspend		= sp6001_pm_suspend,
};
#else
static int sp6001_resume(struct i2c_client *client)
{	
	return 0;
}

static int sp6001_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
#endif

static void sp6001_reset(struct i2c_client *client)
{
	return;
}


/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/

static const struct of_device_id sp6001_match[] = {
	{ .compatible = "oplus,sp6001-charger"},
	{ },
};

static const struct i2c_device_id sp6001_id[] = {
	{"sp6001-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, sp6001_id);


static struct i2c_driver sp6001_i2c_driver = {
	.driver		= {
		.name = "sp6001-charger",
		.owner	= THIS_MODULE,
		.of_match_table = sp6001_match,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
					.pm 	= &sp6001_pm_ops,
#endif

	},
	.probe		= sp6001_driver_probe,
	.remove		= sp6001_driver_remove,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
	.resume		= sp6001_resume,
	.suspend	= sp6001_suspend,
#endif
	.shutdown	= sp6001_reset,
	.id_table	= sp6001_id,
};


module_i2c_driver(p922x_i2c_driver);
MODULE_DESCRIPTION("Driver for sp6001 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:sp6001-charger");
