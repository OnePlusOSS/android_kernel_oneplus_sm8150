/**********************************************************************************
* Copyright (c)  2017-2019  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: For Nuvoton N76E003 MCU
* Version   : 1.0
* Date      : 2018-03-12
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        	<author>              		<desc>
***********************************************************************************/

#define VOOC_MCU_N76E

#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/xlog.h>
//#include <upmu_common.h>
//#include <mt-plat/mtk_gpio.h>
#include <linux/dma-mapping.h>

//#include <mt-plat/battery_meter.h>
#include <linux/module.h>
#include <soc/oplus/device_info.h>

#else
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <soc/oplus/device_info.h>
#endif
#include "oplus_vooc_fw.h"

#ifdef CONFIG_OPLUS_CHARGER_MTK
#define I2C_MASK_FLAG	(0x00ff)
#define I2C_ENEXT_FLAG	(0x0200)
#define I2C_DMA_FLAG	(0xdead2000)
#endif

#define ERASE_COUNT			959/*0x0000-0x3BFF*/

#define BYTE_OFFSET				2
#define BYTES_TO_WRITE			16
#define FW_CHECK_FAIL			0
#define FW_CHECK_SUCCESS		1
static struct oplus_vooc_chip *the_chip = NULL;

#ifdef CONFIG_OPLUS_CHARGER_MTK
#define GTP_SUPPORT_I2C_DMA   			0
#define I2C_MASTER_CLOCK				300
#define GTP_DMA_MAX_TRANSACTION_LENGTH	255		/* for DMA mode */

DEFINE_MUTEX(dma_wr_access_n76e);

#if GTP_SUPPORT_I2C_DMA
static int i2c_dma_write(struct i2c_client *client, u8 addr, s32 len, u8 *txbuf);
static int i2c_dma_read(struct i2c_client *client, u8 addr, s32 len, u8 *txbuf);
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#endif

#if GTP_SUPPORT_I2C_DMA
static int i2c_dma_read(struct i2c_client *client, u8 addr, s32 len, u8 *rxbuf)
{
	int ret;
	s32 retry = 0;
	u8 buffer[1];

	struct i2c_msg msg[2] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buffer,
			.len = 1,
			.timing = I2C_MASTER_CLOCK
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (__u8 *)gpDMABuf_pa,   /*modified by PengNan*/
			.len = len,
			.timing = I2C_MASTER_CLOCK
		},
	};
	mutex_lock(&dma_wr_access_n76e);
	/*buffer[0] = (addr >> 8) & 0xFF;*/
	buffer[0] = addr & 0xFF;
	if (rxbuf == NULL) {
		mutex_unlock(&dma_wr_access_n76e);
		return -1;
	}
	/*chg_err("vooc dma i2c read: 0x%x, %d bytes(s)\n", addr, len);*/
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			continue;
		}
		memcpy(rxbuf, gpDMABuf_va, len);
		mutex_unlock(&dma_wr_access_n76e);
		return 0;
	}
	/*chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);*/
	mutex_unlock(&dma_wr_access_n76e);
	return ret;
}

static int i2c_dma_write(struct i2c_client *client, u8 addr, s32 len, u8 const *txbuf)
{
	int ret = 0;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_va;
	struct i2c_msg msg =
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = (__u8 *)gpDMABuf_pa,	/*modified by PengNan*/
		.len = 1 + len,
		.timing = I2C_MASTER_CLOCK
	};
	mutex_lock(&dma_wr_access_n76e);
	wr_buf[0] = (u8)(addr & 0xFF);
	if (txbuf == NULL) {
		mutex_unlock(&dma_wr_access_n76e);
		return -1;
	}
	memcpy(wr_buf+1, txbuf, len);
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			continue;
		}
		mutex_unlock(&dma_wr_access_n76e);
		return 0;
	}
	/*chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);*/
	mutex_unlock(&dma_wr_access_n76e);
	return ret;
}
#endif /*GTP_SUPPORT_I2C_DMA*/
#endif

static int oplus_vooc_i2c_read(struct i2c_client *client, u8 addr, s32 len, u8 *rxbuf)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if GTP_SUPPORT_I2C_DMA
	return i2c_dma_read(client, addr, len, rxbuf);
#else
	return i2c_smbus_read_i2c_block_data(client, addr, len, rxbuf);
#endif
#else
	return i2c_smbus_read_i2c_block_data(client, addr, len, rxbuf);
#endif
}

static int oplus_vooc_i2c_write(struct i2c_client *client, u8 addr, s32 len, u8 const *txbuf)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
#if GTP_SUPPORT_I2C_DMA
	return i2c_dma_write(client, addr, len, txbuf);
#else
	return i2c_smbus_write_i2c_block_data(client, addr, len, txbuf);
#endif
#else
	return i2c_smbus_write_i2c_block_data(client, addr, len, txbuf);
#endif
}

static bool n76e_fw_check_frontline(struct oplus_vooc_chip *chip)
{
	unsigned char addr_buf[2] = {0x00, 0x00};
	unsigned char data_buf[16] = {0x0};
	int rc = 0;
	int j = 0;
	int fw_line = 0;
	int total_line = 0;

	if (!chip->firmware_data) {
		chg_err("n76e_fw_data Null, Return\n");
		return FW_CHECK_FAIL;
	}

	total_line = chip->fw_data_count / 18;

	for (fw_line = 0; fw_line < (total_line - 1); fw_line++) {
		addr_buf[0] = chip->firmware_data[fw_line * 18 + 1];
		addr_buf[1] = chip->firmware_data[fw_line * 18];
		rc = oplus_vooc_i2c_write(chip->client, 0x01, 2, &addr_buf[0]);
		if (rc < 0) {
			chg_err("i2c_write 0x01 error\n");
			return FW_CHECK_FAIL;
		}

		data_buf[0] = 0;
		oplus_vooc_i2c_write(chip->client, 0x03, 1, &data_buf[0]);
		usleep_range(2000, 2100);
		oplus_vooc_i2c_read(chip->client, 0x03, 16, &data_buf[0]);

		for (j = 0; j < 16; j++) {
			if (data_buf[j] != chip->firmware_data[fw_line * 18 + 2 + j]) {
				chg_err("fail, data_buf[%d]:0x%x != n76e_firmware_data[%d]:0x%x\n",
						j, data_buf[j], (fw_line * 18 + 2 + j),
						chip->firmware_data[fw_line * 18 + 2 + j]);
				chg_err("addr = 0x%x, %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
						((addr_buf[0] << 8) | addr_buf[1]),
						data_buf[0], data_buf[1], data_buf[2], data_buf[3],
						data_buf[4], data_buf[5], data_buf[6], data_buf[7],
						data_buf[8], data_buf[9], data_buf[10],data_buf[11],
						data_buf[12], data_buf[13], data_buf[14], data_buf[15]);
				return FW_CHECK_FAIL;
			}
		}
	}
	return FW_CHECK_SUCCESS;
}

static bool n76e_fw_check_lastline(struct oplus_vooc_chip *chip)
{
	unsigned char addr_buf[2] = {0x3B, 0xF0};
	unsigned char data_buf[16] = {0x0};
	int i = 0;
	int rc = 0;
	int total_line = 0;

	if (!chip->firmware_data) {
		chg_err("n76e_fw_data Null, Return\n");
		return FW_CHECK_FAIL;
	}

	total_line = chip->fw_data_count / 18;
	addr_buf[0] = chip->firmware_data[(total_line-1) * 18 + 1];
	addr_buf[1] = chip->firmware_data[(total_line-1) * 18];
	rc = oplus_vooc_i2c_write(chip->client, 0x01, 2, &addr_buf[0]);
	if (rc < 0) {
		chg_err("i2c_write 0x01 error\n");
		return FW_CHECK_FAIL;
	}

	/* need to write 0x03 before read 0x03 */
	data_buf[0] = 0;
	oplus_vooc_i2c_write(chip->client, 0x03, 1, &data_buf[0]);
	usleep_range(2000, 2100);
	oplus_vooc_i2c_read(chip->client, 0x03, 16, &data_buf[0]);
	chip->fw_mcu_version = data_buf[12];
	
	for (i = 0; i < 16; i++) {
		if (data_buf[i] != chip->firmware_data[chip->fw_data_count - 16 + i]) {
			chg_err("fail, data_buf[%d]:0x%x != n76e_firmware_data[]:0x%x\n" ,
					i, data_buf[i], chip->firmware_data[chip->fw_data_count - 16 + i]);
			chg_err("data %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n" ,
					data_buf[0], data_buf[1], data_buf[2], data_buf[3], data_buf[4], data_buf[5],
					data_buf[6], data_buf[7], data_buf[8], data_buf[9], data_buf[10], data_buf[11],
					data_buf[12], data_buf[13], data_buf[14], data_buf[15]);
			return FW_CHECK_FAIL;
		}
	}
	return FW_CHECK_SUCCESS;
}

static int n76e_fw_write(struct oplus_vooc_chip *chip,
	const unsigned char *data_buf, unsigned int offset, unsigned int length)
{
	unsigned int count = 0;
	unsigned char zero_buf[1] = {0};
	unsigned char temp_buf[1] = {0};
	unsigned char addr_buf[2] = {0x00, 0x00};
	int rc = 0;

	if (!chip->firmware_data) {
		chg_err("n76e_fw_data Null, Return\n");
		return -1;
	}

	count = offset;
	/*write data begin*/
	while (count < (offset + length)) {
		addr_buf[0] = data_buf[count + 1];
		addr_buf[1] = data_buf[count];
		/*chg_err(" write data addr_buf[0]:0x%x, addr_buf[1]:0x%x\n", addr_buf[0], addr_buf[1]);*/
		rc = oplus_vooc_i2c_write(chip->client, 0x01, 2, &addr_buf[0]);
		if (rc < 0) {
			chg_err("i2c_write 0x01 error\n");
			return -1;
		}

		/*swap low byte and high byte begin*/
		/*swap low byte and high byte end*/
		/*write 16 bytes data to n76e*/
		oplus_vooc_i2c_write(chip->client, 0x02, BYTES_TO_WRITE,
			&data_buf[count+BYTE_OFFSET]);
		oplus_vooc_i2c_write(chip->client, 0x05, 1, &zero_buf[0]);
		oplus_vooc_i2c_read(chip->client, 0x05, 1, &temp_buf[0]);
		/*chg_err("lfc read 0x05, temp_buf[0]:0x%x\n", temp_buf[0]);*/

		count = count + BYTE_OFFSET + BYTES_TO_WRITE;

		usleep_range(2000, 2100);
		/*chg_err("  count:%d, offset:%d, length:%d\n" , count, offset, length);*/
		if (count > (offset + length - 1)) {
			break;
		}
	}
	return 0;
}

static int n76e_fw_update(struct oplus_vooc_chip *chip)
{
	unsigned char zero_buf[1] = {0};
	unsigned char addr_buf[2] = {0x00, 0x00};
	unsigned char temp_buf[1] = {0};
	int i = 0;
	int rc = 0;
	unsigned int addr = 0x0000;
	int download_again = 0;

	if (!chip->firmware_data) {
		chg_err("n76e_fw_data Null, Return\n");
		return -1;
	} else {
		chg_debug("start, erase data ing.......\n");
	}

update_fw:
	/*erase address 0x00-0x3BFF*/
	for (i = 0; i < ERASE_COUNT; i++) {
		/*first:set address*/
		/*rc = i2c_smbus_write_i2c_block_data(chip->client, 0x01, 2, &addr_buf[0]);*/
		rc = oplus_vooc_i2c_write(chip->client, 0x01, 2, &addr_buf[0]);
		if (rc < 0) {
			chg_err("n76e_update_fw, i2c_write 0x01 error\n");
			goto update_fw_err;
		}

		/*erase data:0x10 words once*/
		//oplus_vooc_i2c_write(chip->client, 0x04, 1, &zero_buf[0]);
		//msleep(1);
		oplus_vooc_i2c_read(chip->client, 0x04, 1, &temp_buf[0]);
		/*chg_err("lfc read 0x04, temp_buf[0]:0x%x\n", temp_buf[0]);*/
		/*delay 4ms is not OK and 5 is OK*/
		usleep_range(6000, 6100);
		/*erase data:0x10 words once*/
		addr = addr + 0x10;
		addr_buf[0] = addr >> 8;
		addr_buf[1] = addr & 0xFF;
		/*chg_err("lfc addr_buf[0]:0x%x, addr_buf[1]:0x%x\n", addr_buf[0], addr_buf[1]);*/
	}
	msleep(10);

	n76e_fw_write(chip, chip->firmware_data, 0, chip->fw_data_count - 18);

	/*fw check begin:read data from stm1503/1508, and compare it with n76e_firmware_data[]*/
	rc = n76e_fw_check_frontline(chip);
	if (rc == FW_CHECK_FAIL) {
		download_again++;
		if (download_again > 3) {
			goto update_fw_err;
		}
		chip->mcu_update_ing = false;
		opchg_set_reset_active(chip);
		chip->mcu_update_ing = true;
		msleep(1000);
		chg_err("fw check fail, download fw again\n");
		goto update_fw;
	}
	/*fw check end*/

	rc = n76e_fw_write(chip, chip->firmware_data, chip->fw_data_count - 18, 18);

	if (rc < 0) {
		goto update_fw_err;
	}
	/*write 0x7F0~0x7FF end*/

	msleep(2);
	/*jump to app code begin*/
	oplus_vooc_i2c_write(chip->client, 0x06, 1, &zero_buf[0]);
	oplus_vooc_i2c_read(chip->client, 0x06, 1, &temp_buf[0]);
	/*jump to app code end*/
	chip->have_updated = 1;
	chip->mcu_update_ing = false;
	/*pull down GPIO96 to power off MCU1503/1508*/
	chip->vooc_fw_check = true;
	chg_debug("success\n");
	return 0;

update_fw_err:
	chip->mcu_update_ing = false;
	chg_err("fail\n");
	return 1;
}

static int n76e_get_fw_verion_from_ic(struct oplus_vooc_chip *chip)
{
	unsigned char addr_buf[2] = {0x3B, 0xF0};
	unsigned char data_buf[4] = {0};
	int rc = 0;
	int update_result = 0;

//	if (!chip->firmware_data) {
//		chg_err("Stm8s_fw_data Null, Return\n");
//		return FW_CHECK_FAIL;
//	}
	if(oplus_is_power_off_charging(chip) == true || oplus_is_charger_reboot(chip)== true) {
		chip->mcu_update_ing = true;
		update_result = n76e_fw_update(chip);
		chip->mcu_update_ing = false;
		if(update_result) {
			msleep(30);
			opchg_set_clock_sleep(chip);
			opchg_set_reset_active(chip);
		}
	} else {
		opchg_set_clock_active(chip);
		chip->mcu_boot_by_gpio = true;
		msleep(10);
		opchg_set_reset_active_force(chip);
		chip->mcu_update_ing = true;
		msleep(2500);
		chip->mcu_boot_by_gpio = false;
		opchg_set_clock_sleep(chip);

	//first:set address
	rc = oplus_vooc_i2c_write(chip->client,0x01,2,&addr_buf[0]);
	if (rc < 0) {
		chg_err(" i2c_write 0x01 error\n" );
		return FW_CHECK_FAIL;
	}
	msleep(2);
	oplus_vooc_i2c_read(chip->client,0x03,4,data_buf);
	//	strcpy(ver,&data_buf[0]);
	chg_err("data:%x %x %x %x, fw_ver:%x\n",data_buf[0], data_buf[1], data_buf[2], data_buf[3],data_buf[0]);

		chip->mcu_update_ing = false;
		msleep(5);
		opchg_set_reset_active(chip);
	}

	return data_buf[0];
}
static int n76e_fw_check_then_recover(struct oplus_vooc_chip *chip)
{
	int update_result = 0;
	int ret = 0;

	if (!chip->firmware_data) {
		chg_err("n76e_fw_data Null, Return\n");
		return FW_ERROR_DATA_MODE;
	} else {
		chg_debug("begin\n");
	}

	if (oplus_is_power_off_charging(chip) == true || oplus_is_charger_reboot(chip)== true) {
		chip->mcu_update_ing = true;
		update_result = n76e_fw_update(chip);
		chip->mcu_update_ing = false;
		ret = FW_NO_CHECK_MODE;
	} else {
		opchg_set_clock_active(chip);
		chip->mcu_boot_by_gpio = true;
		msleep(10);
		opchg_set_reset_active_force(chip);
		chip->mcu_update_ing = true;
		msleep(2500);
		chip->mcu_boot_by_gpio = false;
		opchg_set_clock_sleep(chip);
		if (n76e_fw_check_frontline(chip) == FW_CHECK_FAIL || n76e_fw_check_lastline(chip) == FW_CHECK_FAIL) {
			n76e_fw_update(chip);
		} else {
			chip->vooc_fw_check = true;
			chg_debug("fw check ok\n");
		}
		chip->mcu_update_ing = false;
		msleep(5);
		opchg_set_reset_active_force(chip);
		ret = FW_CHECK_MODE;
	}

	return ret;
}

struct oplus_vooc_operations oplus_n76e_ops = {
		.fw_update = n76e_fw_update,
		.fw_check_then_recover = n76e_fw_check_then_recover,
		.set_switch_mode = opchg_set_switch_mode,
		.eint_regist = oplus_vooc_eint_register,
		.eint_unregist = oplus_vooc_eint_unregister,
		.set_data_active = opchg_set_data_active,
		.set_data_sleep = opchg_set_data_sleep,
		.set_clock_active = opchg_set_clock_active,
		.set_clock_sleep = opchg_set_clock_sleep,
		.get_gpio_ap_data = opchg_get_gpio_ap_data,
		.read_ap_data = opchg_read_ap_data,
		.reply_mcu_data = opchg_reply_mcu_data,
		.reply_mcu_data_4bits = opchg_reply_mcu_data_4bits,
		.reset_fastchg_after_usbout = reset_fastchg_after_usbout,
		.switch_fast_chg = switch_fast_chg,
		.reset_mcu = opchg_set_reset_active,
		.is_power_off_charging = oplus_is_power_off_charging,
		.get_reset_gpio_val = oplus_vooc_get_reset_gpio_val,
		.get_switch_gpio_val = oplus_vooc_get_switch_gpio_val,
		.get_ap_clk_gpio_val = oplus_vooc_get_ap_clk_gpio_val,
		.get_fw_version = n76e_get_fw_verion_from_ic,
		.get_clk_gpio_num = opchg_get_clk_gpio_num,
		.get_data_gpio_num = opchg_get_data_gpio_num,
};

static void register_vooc_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;

	version = "n76e";
	manufacture = "NUVOTON";

	ret = register_device_proc("vooc", version, manufacture);
	if (ret) {
		chg_err(" fail\n");
	}
}

static void n76e_shutdown(struct i2c_client *client)
{
	if (!the_chip) {
		return;
	}
	opchg_set_switch_mode(the_chip, NORMAL_CHARGER_MODE);
	msleep(10);
	if (oplus_vooc_get_fastchg_started() == true) {
		opchg_set_clock_sleep(the_chip);
		msleep(10);
		opchg_set_reset_active(the_chip);
	}
	msleep(80);
	return;
}

static ssize_t vooc_fw_check_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	char page[256] = {0};
	char read_data[32] = {0};
	int len = 0;

	if (the_chip && the_chip->vooc_fw_check == true) {
		read_data[0] = '1';
	} else {
		read_data[0] = '0';
	}
	len = sprintf(page, "%s", read_data);
	if (len > *off) {
		len -= *off;
	} else {
		len = 0;
	}
	if (copy_to_user(buff, page, (len < count ? len : count))) {
		return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);
}

static const struct file_operations vooc_fw_check_proc_fops = {
	.read = vooc_fw_check_read,
	.llseek = noop_llseek,
};

static int init_proc_vooc_fw_check(void)
{
	struct proc_dir_entry *p = NULL;

	p = proc_create("vooc_fw_check", 0444, NULL, &vooc_fw_check_proc_fops);
	if (!p) {
		chg_err("proc_create vooc_fw_check_proc_fops fail!\n");
	}
	return 0;
}

static int n76e_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct oplus_vooc_chip *chip;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

#ifdef CONFIG_OPLUS_CHARGER_MTK
#if GTP_SUPPORT_I2C_DMA
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		chg_err("[Error] Allocate DMA I2C Buffer failed!\n");
	} else {
		chg_debug(" ppp dma_alloc_coherent success\n");
	}
	memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif
#endif
	if(get_vooc_mcu_type(chip) != OPLUS_VOOC_MCU_HWID_N76E){
		chg_err("It is not n76e\n");
		return -ENOMEM;
	}

	chip->pcb_version = g_hw_version;
	chip->vooc_fw_check = false;
		mutex_init(&chip->pinctrl_mutex);

	oplus_vooc_fw_type_dt(chip);
	if (chip->batt_type_4400mv) {
		chip->firmware_data = n76e_fw_data_4400;
		chip->fw_data_count = sizeof(n76e_fw_data_4400);
		chip->fw_data_version = n76e_fw_data_4400[chip->fw_data_count - 4];
	} else {//default
		chip->firmware_data = n76e_fw_data_4400;
		chip->fw_data_count = sizeof(n76e_fw_data_4400);
		chip->fw_data_version = n76e_fw_data_4400[chip->fw_data_count - 4];
	}

	if (chip->vooc_fw_type == VOOC_FW_TYPE_N76E_4400_AVOID_OVER_TEMP_NTC61C) {
		chip->firmware_data = n76e_fw_data_4400_ntc61c;
		chip->fw_data_count = sizeof(n76e_fw_data_4400_ntc61c);
		chip->fw_data_version = n76e_fw_data_4400_ntc61c[chip->fw_data_count - 4];
	} else if (chip->vooc_fw_type == VOOC_FW_TYPE_N76E_4400_VOOC_FFC_15C) {
		chip->firmware_data = n76e_fw_data_4400_vooc_ffc_15c;
		chip->fw_data_count = sizeof(n76e_fw_data_4400_vooc_ffc_15c);
		chip->fw_data_version = n76e_fw_data_4400_vooc_ffc_15c[chip->fw_data_count - 4];
	} else if (chip->vooc_fw_type == VOOC_FW_TYPE_N76E_4400_VOOC_FFC_15C_FV4450) {
		chip->firmware_data = n76e_fw_data_4400_vooc_ffc_15c_fv4450;
		chip->fw_data_count = sizeof(n76e_fw_data_4400_vooc_ffc_15c_fv4450);
		chip->fw_data_version = n76e_fw_data_4400_vooc_ffc_15c_fv4450[chip->fw_data_count - 4];
	}

	chip->vops = &oplus_n76e_ops;
	chip->fw_mcu_version = 0;

	oplus_vooc_gpio_dt_init(chip);
	opchg_set_clock_sleep(chip);
	oplus_vooc_delay_reset_mcu_init(chip);
	if(chip->vooc_fw_update_newmethod) {
		if(oplus_is_rf_ftm_mode()){
			oplus_vooc_fw_update_work_init(chip);
		}
	} else {
		oplus_vooc_fw_update_work_init(chip);
	}

	oplus_vooc_init(chip);
	register_vooc_devinfo();
	init_proc_vooc_fw_check();
	the_chip = chip;
	chg_debug("n76e success\n");
	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
static const struct of_device_id n76e_match[] = {
	{ .compatible = "oplus,n76e-fastcg"},
	{ },
};

static const struct i2c_device_id n76e_id[] = {
	{ "n76e-fastcg", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, n76e_id);

struct i2c_driver n76e_i2c_driver = {
	.driver = {
		.name = "n76e-fastcg",
		.owner	= THIS_MODULE,
		.of_match_table = n76e_match,
	},
	.probe = n76e_driver_probe,
	.shutdown = n76e_shutdown,
	.id_table = n76e_id,
};

static int __init n76e_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");
	init_hw_version();

	if (i2c_add_driver(&n76e_i2c_driver) != 0) {
		chg_err(" failed to register n76e i2c driver.\n");
	} else {
		chg_debug(" Success to register n76e i2c driver.\n");
	}
	return ret;
}

/*
static void  pic16f_exit(void)
{
	i2c_del_driver(&pic16f_i2c_driver);
}
*/
subsys_initcall(n76e_subsys_init);
MODULE_DESCRIPTION("Driver for oplus vooc n76e fast mcu");
MODULE_LICENSE("GPL v2");

