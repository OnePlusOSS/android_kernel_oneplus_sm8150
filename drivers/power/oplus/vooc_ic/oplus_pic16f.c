/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oplus77_12015\kernel\battery\battery
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      for dc-dc sn111008 charg
**
** Version: 1.0
** Date created: 21:03:46,05/04/2012
** Author: Fanhong.Kong@ProDrv.CHG
**
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
************************************************************************************************************/


#define VOOC_MCU_PIC16F

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/xlog.h>
#include <upmu_common.h>
#include <mt-plat/mtk_gpio.h>
#include <linux/dma-mapping.h>

#include <mt-plat/battery_meter.h>
#include <linux/module.h>
#include <linux/of_device.h>

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

#define ERASE_COUNT			224	//0x200-0xFFF
#define READ_COUNT			223	//448

#define BYTE_OFFSET 					2
#define BYTES_TO_WRITE 					16
#define FW_CHECK_FAIL					0
#define FW_CHECK_SUCCESS				1

#ifdef CONFIG_OPLUS_CHARGER_MTK

#define GTP_SUPPORT_I2C_DMA				1
#define I2C_MASTER_CLOCK				300
#define GTP_DMA_MAX_TRANSACTION_LENGTH	255			// for DMA mode

DEFINE_MUTEX(dma_wr_access_pic);

#if GTP_SUPPORT_I2C_DMA
static int i2c_dma_write(struct i2c_client *client, u8 addr, s32 len, u8 *txbuf);
static int i2c_dma_read(struct i2c_client *client, u8 addr, s32 len, u8 *txbuf);
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#endif

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
			.buf = ((__u8 *)gpDMABuf_pa),   //modified by PengNan
			.len = len,
			.timing = I2C_MASTER_CLOCK
		},
	};

	mutex_lock(&dma_wr_access_pic);
	//buffer[0] = (addr >> 8) & 0xFF;
	buffer[0] = addr & 0xFF;

	if (rxbuf == NULL){
		mutex_unlock(&dma_wr_access_pic);
		return -1;
		}
	//chg_err(" : 0x%x, %d bytes(s)", addr, len);
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
		{
			continue;
		}
		memcpy(rxbuf, gpDMABuf_va, len);
		mutex_unlock(&dma_wr_access_pic);
		return 0;
	}
	//chg_err("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	mutex_unlock(&dma_wr_access_pic);
	return ret;
}


static int i2c_dma_write(struct i2c_client *client, u8 addr, s32 len, u8 *txbuf)
{
	int ret;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_va;
	struct i2c_msg msg =
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = ((__u8 *)gpDMABuf_pa),	//modified by PengNan
		.len = 1 + len,
		.timing = I2C_MASTER_CLOCK
	};
	mutex_lock(&dma_wr_access_pic);
	wr_buf[0] = (u8)(addr & 0xFF);
	if (txbuf == NULL){
			mutex_unlock(&dma_wr_access_pic);
			return -1;
		}
	memcpy(wr_buf+1, txbuf, len);
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
		{
			continue;
		}
		mutex_unlock(&dma_wr_access_pic);
		return 0;
	}
	//chg_err("Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	mutex_unlock(&dma_wr_access_pic);
	return ret;
}
#endif

static int oplus_vooc_i2c_read(struct i2c_client *client, u8 addr, s32 len, u8 *rxbuf)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK
	return i2c_dma_read(client, addr, len, rxbuf);
#else
	return i2c_smbus_read_i2c_block_data(client, addr, len, rxbuf);
#endif
}

static int oplus_vooc_i2c_write(struct i2c_client *client, u8 addr, s32 len, u8 *txbuf)
{
#ifdef CONFIG_OPLUS_CHARGER_MTK

	return i2c_dma_write(client, addr, len, txbuf);
#else
	return i2c_smbus_write_i2c_block_data(client, addr, len, txbuf);
#endif
}


static bool pic16f_fw_check(struct oplus_vooc_chip *chip)
{
	unsigned char addr_buf[2] = {0x02,0x00};
	unsigned char data_buf[32] = {0x0};
	int rc,i,j,addr;
	int fw_line = 0;

	//first:set address
	rc = oplus_vooc_i2c_write(chip->client,0x01,2,&addr_buf[0]);
	if(rc < 0){
		chg_err(" i2c_write 0x01 error\n");
		goto i2c_err;
	}
	msleep(10);

	for(i = 0;i < READ_COUNT;i++){	//1508:448,1503:192
		oplus_vooc_i2c_read(chip->client,0x03,16,&data_buf[0]);
		msleep(2);
		oplus_vooc_i2c_read(chip->client,0x03,16,&data_buf[16]);

		addr = 0x200 + i * 16;
/*
		chg_err(" addr = 0x%x,%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",addr,
			data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5],data_buf[6],data_buf[7],
			data_buf[8],data_buf[9],data_buf[10],data_buf[11],data_buf[12],data_buf[13],data_buf[14],
			data_buf[15],data_buf[16],data_buf[17],data_buf[18],data_buf[19],data_buf[20],data_buf[21],data_buf[22],
			data_buf[23],data_buf[24],data_buf[25],data_buf[26],data_buf[27],data_buf[28],data_buf[29],data_buf[30],
			data_buf[31]);
		*/
//compare recv_buf with Pic16F_firmware_data[] begin

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if(addr == ((Pic16F_firmware_data[fw_line * 34 + 1] << 8) | Pic16F_firmware_data[fw_line * 34])){
		for(j = 0; j < 32; j++){
			if(data_buf[j] != Pic16F_firmware_data[fw_line * 34 + 2 + j]){
				chg_err(" fail,data_buf[%d]:0x%x != Pic16F_fimware_data[%d]:0x%x\n",
					j,data_buf[j],(fw_line * 34 + 2 + j),Pic16F_firmware_data[fw_line * 34 + 2 + j]);
				chg_err(" addr = 0x%x,%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",addr,
					data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5],data_buf[6],data_buf[7],
					data_buf[8],data_buf[9],data_buf[10],data_buf[11],data_buf[12],data_buf[13],data_buf[14],
					data_buf[15],data_buf[16],data_buf[17],data_buf[18],data_buf[19],data_buf[20],data_buf[21],data_buf[22],
					data_buf[23],data_buf[24],data_buf[25],data_buf[26],data_buf[27],data_buf[28],data_buf[29],data_buf[30],
					data_buf[31]);
				return FW_CHECK_FAIL;
			}
		}
		fw_line++;
	} else {
		//chg_err(" addr dismatch,addr:0x%x,pic_data:0x%x\n",
			//addr,(Pic16F_firmware_data[fw_line * 34 + 1] << 8) | Pic16F_firmware_data[fw_line * 34]);
	}
#else
		j = 0;
		if(addr == ((Pic16F_firmware_data[fw_line * 34 + 1] << 8) | Pic16F_firmware_data[fw_line * 34])){
			if(data_buf[0] != Pic16F_firmware_data[fw_line * 34 + 2]){
				chg_err(" fail,data_buf[%d]:0x%x != Pic16F_fimware_data[%d]:0x%x\n",
					j,data_buf[j],(fw_line * 34 + 2 + j),Pic16F_firmware_data[fw_line * 34 + 2 + j]);
				chg_err(" addr = 0x%x,%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",addr,
						data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5],data_buf[6],data_buf[7],
						data_buf[8],data_buf[9],data_buf[10],data_buf[11],data_buf[12],data_buf[13],data_buf[14],
						data_buf[15],data_buf[16],data_buf[17],data_buf[18],data_buf[19],data_buf[20],data_buf[21],data_buf[22],
						data_buf[23],data_buf[24],data_buf[25],data_buf[26],data_buf[27],data_buf[28],data_buf[29],data_buf[30],
						data_buf[31]);
				return FW_CHECK_FAIL;
			}
			fw_line++;
		} else {
			//chg_err(" addr dismatch,addr:0x%x,pic_data:0x%x\n",
				//addr,(Pic16F_firmware_data[fw_line * 34 + 1] << 8) | Pic16F_firmware_data[fw_line * 34]);
		}
#endif
	//compare recv_buf with Pic16F_firmware_data[] end
	}
	chg_debug( " success\n");
	return FW_CHECK_SUCCESS;

i2c_err:
	chg_err(" failed\n");
	return FW_CHECK_FAIL;
}

static void pic16f_fw_data_recover(struct oplus_vooc_chip *chip,unsigned char *data_buf,
		unsigned int offset,unsigned int length){
	unsigned int count = 0;
	unsigned char temp;
	int i;
	count = offset;

	while(count < (offset + length)) {
		for(i = 0;i < 2 * BYTES_TO_WRITE;i = (i+2)){
			temp = data_buf[count+BYTE_OFFSET+i];
			data_buf[count+BYTE_OFFSET+i] = data_buf[count+BYTE_OFFSET+i+1];
			data_buf[count+BYTE_OFFSET+i+1] = temp;
		}
		count = count + BYTE_OFFSET + 2 * BYTES_TO_WRITE;
		if(count > (offset + length - 1)){
			break;
		}
	}
}

static int pic16f_fw_write(struct oplus_vooc_chip *chip,unsigned char *data_buf,unsigned int offset,unsigned int length)
{
	unsigned int count = 0;
	unsigned char zero_buf[1] = {0};
	unsigned char temp_buf[1] = {0};
	unsigned char addr_buf[2] = {0x00,0x00};
	unsigned char temp;
	int i,rc;

	count = offset;
	//write data begin
	while(count < (offset + length)) {
		addr_buf[0] = data_buf[count + 1];
		addr_buf[1] = data_buf[count];
		//chg_err(" write data addr_buf[0]:0x%x,addr_buf[1]:0x%x\n",addr_buf[0],addr_buf[1]);
			rc = oplus_vooc_i2c_write(chip->client,0x01,2,&addr_buf[0]);
		if(rc < 0){
			chg_err(" i2c_write 0x01 error\n");
			return -1;
		}

		//swap low byte and high byte begin
		//because LSB is before MSB in buf,but pic16F receive MSB first
		for(i = 0;i < 2 * BYTES_TO_WRITE;i = (i+2)){
			temp = data_buf[count+BYTE_OFFSET+i];
			data_buf[count+BYTE_OFFSET+i] = data_buf[count+BYTE_OFFSET+i+1];
			data_buf[count+BYTE_OFFSET+i+1] = temp;
		}
		//swap low byte and high byte end
		//write 16 bytes data to pic16F

		oplus_vooc_i2c_write(chip->client,0x02,BYTES_TO_WRITE,&data_buf[count+BYTE_OFFSET]);
		oplus_vooc_i2c_write(chip->client,0x05,1,&zero_buf[0]);
		oplus_vooc_i2c_read(chip->client,0x05,1,&temp_buf[0]);
		//chg_err("lfc read 0x05,temp_buf[0]:0x%x\n",temp_buf[0]);

		//write 16 bytes data to pic16F again
		oplus_vooc_i2c_write(chip->client,0x02,BYTES_TO_WRITE,&data_buf[count+BYTE_OFFSET+BYTES_TO_WRITE]);
		oplus_vooc_i2c_write(chip->client,0x05,1,&zero_buf[0]);
		oplus_vooc_i2c_read(chip->client,0x05,1,&temp_buf[0]);
		//chg_err("lfc read again 0x05,temp_buf[0]:0x%x\n",temp_buf[0]);

		count = count + BYTE_OFFSET + 2 * BYTES_TO_WRITE;

		msleep(2);
		//chg_err(" count:%d,offset:%d,length:%d\n",count,offset,length);
		if(count > (offset + length - 1)){
			break;
		}
	}
	return 0;
}

static int pic16f_fw_update(struct oplus_vooc_chip *chip)
{
	unsigned char zero_buf[1] = {0};
	unsigned char addr_buf[2] = {0x02,0x00};
	unsigned char temp_buf[1]={0};
	int i,rc=0;
	unsigned int addr = 0x200;
	int download_again = 0;

	chg_debug( " is start,erase data ing.......\n");

update_fw:
	//erase address 0x200-0x7FF
	for(i = 0; i < ERASE_COUNT; i++){
		//first:set address
		rc = oplus_vooc_i2c_write(chip->client,0x01,2,&addr_buf[0]);
		if(rc < 0){
			chg_err(" i2c_write 0x01 error\n");
			goto update_fw_err;
		}

		//erase data:0x10 words once
		oplus_vooc_i2c_write(chip->client,0x04,1,&zero_buf[0]);
		msleep(1);
		oplus_vooc_i2c_read(chip->client,0x04,1,&temp_buf[0]);
		//chg_err("lfc read 0x04,temp_buf[0]:0x%x\n",temp_buf[0]);

		//erase data:0x10 words once
		addr = addr + 0x10;
		addr_buf[0] = addr >> 8;
		addr_buf[1] = addr & 0xFF;
		//chg_err("lfc addr_buf[0]:0x%x,addr_buf[1]:0x%x\n",addr_buf[0],addr_buf[1]);
	}
	msleep(10);

	pic16f_fw_write(chip,Pic16F_firmware_data,0,sizeof(Pic16F_firmware_data) - 34);

	//fw check begin:read data from pic1503/1508,and compare it with Pic16F_firmware_data[]
	rc = pic16f_fw_check(chip);
	pic16f_fw_data_recover(chip,Pic16F_firmware_data,0,sizeof(Pic16F_firmware_data) - 34);
	msleep(10);
	if(rc == FW_CHECK_FAIL){
		download_again++;
		if(download_again > 3){
			goto update_fw_err;
		}
		chip->mcu_update_ing = false;
		opchg_set_reset_active(chip);
		chip->mcu_update_ing = true;
		msleep(1000);
		chg_err(" fw check fail,download fw again\n");
		goto update_fw;
	}
	//fw check end

	chg_debug( " is start55555\n");
	//write 0x7F0~0x7FF(0x7FF = 0x3455)
	rc = pic16f_fw_write(chip,Pic16F_firmware_data,sizeof(Pic16F_firmware_data) - 34,34);
	if(rc < 0){
		goto update_fw_err;
	}
	//write 0x7F0~0x7FF end

	msleep(2);
	//jump to app code begin
	oplus_vooc_i2c_write(chip->client,0x06,1,&zero_buf[0]);
	oplus_vooc_i2c_read(chip->client,0x06,1,&temp_buf[0]);
	//jump to app code end
	chip->have_updated = 1;
	chip->mcu_update_ing = false;
	chg_debug( " success\n");
	return 0;

update_fw_err:
	chip->mcu_update_ing = false;
	chg_err(" fail\n");
	return 1;
}

static int pic16f_fw_check_then_recover(struct oplus_vooc_chip *chip)
{
	int update_result = 0;
	int ret = 0;

	chg_debug( " begin\n");
	if(oplus_is_power_off_charging(chip) == true) {
		chip->mcu_update_ing = true;
		update_result = pic16f_fw_update(chip);
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
		if(pic16f_fw_check(chip) == FW_CHECK_FAIL)
			pic16f_fw_update(chip);
		else
			chg_debug( " fw check ok\n");
		chip->mcu_update_ing = false;
		msleep(5);
		opchg_set_reset_active_force(chip);
		ret = FW_CHECK_MODE;
	}

	return ret;
}

struct oplus_vooc_operations oplus_pic16f_ops = {
	.fw_update = pic16f_fw_update,
	.fw_check_then_recover = pic16f_fw_check_then_recover,
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
};

static void register_vooc_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;
	version = "pic16f";
	manufacture = "MICROCHIP";

	ret = register_device_proc("vooc", version, manufacture);
	if (ret) {
		chg_err("register_vooc_devinfo fail\n");
	}
}

static int pic16f_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct oplus_vooc_chip *chip = NULL;

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
		chg_debug( " ppp dma_alloc_coherent success\n");
	}
	memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif
#endif

	chip->pcb_version = g_hw_version;
	chip->firmware_data = Pic16F_firmware_data;
	chip->fw_data_count = sizeof(Pic16F_firmware_data);
	chip->fw_data_version = Pic16F_firmware_data[chip->fw_data_count - 4];
	chip->vops = &oplus_pic16f_ops;

	oplus_vooc_gpio_dt_init(chip);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	chip->vooc_gpio.data_irq = CUST_EINT_MCU_AP_DATA;
#else
	chip->vooc_gpio.data_irq = gpio_to_irq(chip->vooc_gpio.data_gpio);
#endif

	opchg_set_clock_sleep(chip);
	oplus_vooc_delay_reset_mcu_init(chip);
	oplus_vooc_fw_update_work_init(chip);
	oplus_vooc_init(chip);
	register_vooc_devinfo();
	chg_debug( " success\n");

	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/

static const struct of_device_id pic16f_match[] = {
	{ .compatible = "oplus,pic16f-fastcg"},
	{ },
};

static const struct i2c_device_id pic16f_id[] = {
	{ "pic16f-fastcg", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, pic16f_id);

struct i2c_driver pic16f_i2c_driver = {
	.driver	= {
	.name = "pic16f-fastcg",
	.owner	= THIS_MODULE,
	.of_match_table = pic16f_match,
	},
	.probe = pic16f_driver_probe,
//	.shutdown	= NULL,
	.id_table = pic16f_id,
};

static int __init pic16f_subsys_init(void)
{
	int ret=0;

	chg_debug( "[pic16f_init] init start\n");
	init_hw_version();
	if(1) {
		chg_err("pic16f_subsys_init err,MCU stm8s \n");
		return ret;
	}
	if (i2c_add_driver(&pic16f_i2c_driver) != 0) {
		chg_err("[pic16f_init] failed to register pic16f i2c driver.\n");
	} else {
		chg_debug( "[pic16f_init] Success to register pic16f i2c driver.\n");
	}
	return ret;
}

/*
static void  pic16f_exit(void)
{
	i2c_del_driver(&pic16f_i2c_driver);
}
*/

subsys_initcall(pic16f_subsys_init);
MODULE_DESCRIPTION("Driver for oplus vooc pic16f fast mcu");
MODULE_LICENSE("GPL v2");

