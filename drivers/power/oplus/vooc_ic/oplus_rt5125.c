/**********************************************************************************
* Copyright (c)  2017-2019  Guangdong OPLUS Mobile Comm Corp., Ltd
* VENDOR_EDIT
* Description: For Rockchip RT5125 ASIC
* Version   : 1.0
* Date      : 2019-08-15
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        	<author>              		<desc>
***********************************************************************************/

#define VOOC_ASIC_RT5125

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/xlog.h>
#include <linux/dma-mapping.h>

#include <linux/module.h>
#include <soc/oplus/device_info.h>

#else
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
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
#include "../oplus_gauge.h"
#include "../oplus_charger.h"
#include "../oplus_vooc.h"
extern int charger_abnormal_log;

#define I2C_MASK_FLAG	(0x00ff)
#define I2C_ENEXT_FLAG	(0x0200)
#define I2C_DMA_FLAG	(0xdead2000)

#define GTP_DMA_MAX_TRANSACTION_LENGTH	255	/* for DMA mode */

#define ERASE_COUNT			959	/*0x0000-0x3BFF */

#define BYTE_OFFSET			2
#define BYTES_TO_WRITE			16
#define FW_CHECK_FAIL			0
#define FW_CHECK_SUCCESS		1

#define PAGE_UNIT			128
#define TRANSFER_LIMIT			72
#define I2C_ADDR			0x14
#define REG_RESET			0x5140
#define REG_SYS0			0x52C0
#define REG_HOST			0x52C8
#define	REG_SLAVE			0x52CC
#define REG_STATE			0x52C4
#define REG_MTP_SELECT			0x4308
#define REG_MTP_ADDR			0x4300
#define REG_MTP_DATA			0x4304
#define REG_SRAM_BEGIN			0x2000
#define SYNC_FLAG			0x53594E43
#define NOT_SYNC_FLAG			(~SYNC_FLAG)
#define REC_01_FLAG			0x52454301
#define REC_0O_FLAG			0x52454300
#define RESTART_FLAG			0x52455354
#define MTP_SELECT_FLAG			0x000f0001
#define MTP_ADDR_FLAG			0xffff8000
#define SLAVE_IDLE			0x49444C45
#define SLAVE_BUSY			0x42555359
#define SLAVE_ACK			0x41434B00
#define SLAVE_ACK_01			0x41434B01
#define FORCE_UPDATE_FLAG		0xaf1c0b76
#define SW_RESET_FLAG			0X0000fdb9

#define STATE_READY			0x0
#define STATE_SYNC			0x1
#define STATE_REQUEST			0x2
#define STATE_FIRMWARE			0x3
#define STATE_FINISH			0x4

typedef struct {
	u32 tag;
	u32 length;
	u32 timeout;
	u32 ram_offset;
	u32 fw_crc;
	u32 header_crc;
} struct_req, *pstruct_req;

struct rt5125_bat {
	int uv_bat;
	int current_bat;
	int temp_bat;
	int soc_bat;
	int reset_status;
	int i2c_err_count;
};

static struct rt5125_bat the_bat;
static struct oplus_vooc_chip *the_chip = NULL;
struct wakeup_source *rt5125_update_wake_lock = NULL;
struct delayed_work rt5125_update_temp_soc;

#ifdef CONFIG_OPLUS_CHARGER_MTK
#define GTP_SUPPORT_I2C_DMA		0
#define I2C_MASTER_CLOCK			300

DEFINE_MUTEX(dma_wr_access_rt5125);

static char gpDMABuf_pa[GTP_DMA_MAX_TRANSACTION_LENGTH] = { 0 };

#if GTP_SUPPORT_I2C_DMA
static int i2c_dma_write(struct i2c_client *client, u8 addr, s32 len, u8 * txbuf);
static int i2c_dma_read(struct i2c_client *client, u8 addr, s32 len, u8 * txbuf);
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#endif

#if GTP_SUPPORT_I2C_DMA
static int i2c_dma_read(struct i2c_client *client, u8 addr, s32 len, u8 * rxbuf)
{
	int ret;
	s32 retry = 0;
	u8 buffer[1];
	struct i2c_msg msg[2] = {
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .flags = 0,
		 .buf = buffer,
		 .len = 1,
		 .timing = I2C_MASTER_CLOCK},
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		 .flags = I2C_M_RD,
		 .buf = (__u8 *) gpDMABuf_pa,	/*modified by PengNan */
		 .len = len,
		 .timing = I2C_MASTER_CLOCK},
	};
	mutex_lock(&dma_wr_access_rt5125);
	/*buffer[0] = (addr >> 8) & 0xFF; */
	buffer[0] = addr & 0xFF;
	if (rxbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}
	/*chg_err("vooc dma i2c read: 0x%x, %d bytes(s)\n", addr, len); */
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
			continue;
		memcpy(rxbuf, gpDMABuf_va, len);
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	/*chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret); */
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}

static int i2c_dma_write(struct i2c_client *client, u8 addr, s32 len, u8 const *txbuf)
{
	int ret = 0;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_va;
	struct i2c_msg msg = {
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = (__u8 *) gpDMABuf_pa,	/*modified by PengNan */
		.len = 1 + len,
		.timing = I2C_MASTER_CLOCK
	};
	mutex_lock(&dma_wr_access_rt5125);
	wr_buf[0] = (u8) (addr & 0xFF);
	if (txbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}
	memcpy(wr_buf + 1, txbuf, len);
	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			continue;
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	/*chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret); */
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}
#endif /*GTP_SUPPORT_I2C_DMA */

static int oplus_i2c_dma_read(struct i2c_client *client, u16 addr, s32 len, u8 * rxbuf)
{
	int ret;
	s32 retry = 0;
	u8 buffer[1] = { 0 };
	struct i2c_msg msg[2] = {
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .flags = 0,
		 .buf = buffer,
		 .len = 1,
		 },
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .flags = I2C_M_RD,
		 .buf = (__u8 *) gpDMABuf_pa,	/*modified by PengNan */
		 .len = len,
		 },
	};
	mutex_lock(&dma_wr_access_rt5125);
	buffer[0] = (u8) (addr & 0xFF);
	if (rxbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}

	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
			continue;
		memcpy(rxbuf, gpDMABuf_pa, len);
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}

static int oplus_i2c_dma_write(struct i2c_client *client, u16 addr, s32 len, u8 const *txbuf)
{
	int ret = 0;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_pa;
	struct i2c_msg msg = {
		.addr = (client->addr & I2C_MASK_FLAG),
		.flags = 0,
		.buf = (__u8 *) gpDMABuf_pa,	/*modified by PengNan */
		.len = 1 + len,
	};
	mutex_lock(&dma_wr_access_rt5125);
	wr_buf[0] = (u8) (addr & 0xFF);
	if (txbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}
	memcpy(wr_buf + 1, txbuf, len);

	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			continue;
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}

#else /*CONFIG_OPLUS_CHARGER_MTK */
DEFINE_MUTEX(dma_wr_access_rt5125);
static char * gpDMABuf_pa;

static int oplus_i2c_dma_read(struct i2c_client *client, u8 addr, s32 len, u8 * rxbuf)
{
	int ret;
	s32 retry = 0;
	u8 buffer[1] = { 0 };
	struct i2c_msg msg[2] = {
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .flags = 0,
		 .buf = buffer,
		 .len = 1,
		 },
		{
		 .addr = (client->addr & I2C_MASK_FLAG),
		 .flags = I2C_M_RD,
		 .buf = (__u8 *) gpDMABuf_pa,	/*modified by PengNan */
		 .len = len,
		 },
	};
	mutex_lock(&dma_wr_access_rt5125);
	buffer[0] = (u8) (addr & 0xFF);
	if (rxbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}

	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0)
			continue;
		memcpy(rxbuf, gpDMABuf_pa, len);
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}

static int oplus_i2c_dma_write(struct i2c_client *client, u8 addr, s32 len, u8 const *txbuf)
{
	int ret = 0;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_pa;
	struct i2c_msg msg = {
		.addr = (client->addr & I2C_MASK_FLAG),
		.flags = 0,
		.buf = (__u8 *) gpDMABuf_pa,	/*modified by PengNan */
		.len = 1 + len,
	};
	mutex_lock(&dma_wr_access_rt5125);
	wr_buf[0] = (u8) (addr & 0xFF);
	wr_buf[1] = (u8) ((addr >> 8) & 0xFF);
	if (txbuf == NULL) {
		mutex_unlock(&dma_wr_access_rt5125);
		return -1;
	}
	memcpy(wr_buf + 1, txbuf, len);

	for (retry = 0; retry < 5; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0)
			continue;
		mutex_unlock(&dma_wr_access_rt5125);
		return 0;
	}
	chg_err(" Error: 0x%04X, %d byte(s), err-code: %d\n", addr, len, ret);
	mutex_unlock(&dma_wr_access_rt5125);
	return ret;
}
#endif /*CONFIG_OPLUS_CHARGER_MTK */

static int oplus_vooc_i2c_read(struct i2c_client *client, u8 addr, s32 len, u8 * rxbuf)
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

/*****************************************************************/
#define DEFAULT_MAX_BINSIZE	(16 * 1024)
#define DEFAULT_MAX_DATALEN	(128)
#define DEFAULT_MAX_PAGELEN	(128)
#define DEFAULT_MAX_PAGEIDX	(DEFAULT_MAX_BINSIZE / DEFAULT_MAX_PAGELEN)
#define DEFAULT_VERINFO_LEN	(11)
#define DEFAULT_PAGEWR_RETRY	(110)
#define DEFAULT_I2C_RETRY	(5)
/* cmd 1 + data 128 + crc8 */
#define DEFAULT_MAX_BUFFLEN	(1 + DEFAULT_MAX_DATALEN + 1)
#define RT5125_CRC16_INIT	(0xffff)
#define RT5125_CID		(0x5125)

#define RT5125_CHIP_ID		(0x00)
#define RT5125_MTP_INFO_0	(0x01)
#define RT5125_PAGE_IDX		(0x10)
#define RT5125_ACCESS_CTL	(0x12)
#define RT5125_STATUS		(0x13)
#define RT5125_DATA_BUF		(0x80)
#define RT5125_FW_CRC16_INFO	(0x90)
#define RT5125_CMD_CRC8_INFO	(0x91)

#define RT5125_MTP_MODE		BIT(0)
#define RT5125_FW_CRC16_RSLT	BIT(3)
#define RT5125_OPSTATUS_MASK	(0x7)
#define RT5125_OPSTATUS_DFAIL	(0x7)
#define RT5125_OPSTATUS_CFAIL	(0x6)
#define RT5125_OPSTATUS_PFAIL	(0x5)
#define RT5125_OPSTATUS_FAIL	(0x4)
#define RT5125_OPSTATUS_SUCCESS	(0x2)
#define RT5125_OPSTATUS_ONGOING	(0x1)
#define RT5125_OPSTATUS_IDLE	(0x0)
#define RT5125_OPSTATUS_FAILMSK	BIT(2)
#define RT5125_WT_PAGE		BIT(7)
#define RT5125_RD_PAGE		BIT(6)
#define RT5125_WT_FW_CRC16	BIT(3)
#define RT5125_FW_CRC16_VRFY	BIT(2)
#define RT5125_WT_KEY		BIT(1)
#define CRC8_TABLE_SIZE	256

static u8 crc8_table[CRC8_TABLE_SIZE];
static DEFINE_MUTEX(data_lock);
static u8 data_buff[DEFAULT_MAX_BUFFLEN];

static u16 const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static u8 crc8(const u8 table[CRC8_TABLE_SIZE], u8 * pdata, size_t nbytes, u8 crc)
{
	/* loop over the buffer data */
	while (nbytes-- > 0)
		crc = table[(crc ^ *pdata++) & 0xff];
	return crc;
}

static void crc8_populate_msb(u8 table[CRC8_TABLE_SIZE], u8 polynomial)
{
	int i, j;
	const u8 msbit = 0x80;
	u8 t = msbit;
	table[0] = 0;
	for (i = 1; i < CRC8_TABLE_SIZE; i *= 2) {
		t = (t << 1) ^ (t & msbit ? polynomial : 0);
		for (j = 0; j < i; j++)
			table[i + j] = table[j] ^ t;
	}
}

static inline u16 crc16_byte(u16 crc, const u8 data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

static u16 crc16(u16 crc, u8 const *buffer, size_t len)
{
	while (len--)
		crc = crc16_byte(crc, *buffer++);
	return crc;
}

static int rt5125_i2c_block_read(struct oplus_vooc_chip *chip, u8 cmd, u8 * data, s32 len)
{
	u8 crc;
	int retry = 0, ret;

	if (len > DEFAULT_MAX_DATALEN || len <= 0)
		return -EINVAL;
	mutex_lock(&data_lock);
retry_read:
	if (retry++ >= DEFAULT_I2C_RETRY) {
		ret = -EIO;
		goto out_read;
	}
	ret = oplus_i2c_dma_read(chip->client, cmd, len + 1, data_buff + 1);
	if (ret < 0)
		goto out_read;
	data_buff[0] = cmd;
	/* verify crc 8 : cmd + data */
	crc = crc8(crc8_table, data_buff, len + 1, 0);
	if (crc != data_buff[len + 1])
		goto retry_read;
	memcpy(data, data_buff + 1, len);
out_read:
	mutex_unlock(&data_lock);
	return ret;
}

static int rt5125_i2c_block_write(struct oplus_vooc_chip *chip, u8 cmd, const u8 * data, s32 len)
{
	int retry = 0, ret;

	if (len > DEFAULT_MAX_DATALEN || len <= 0)
		return -EINVAL;
	mutex_lock(&data_lock);
retry_write:
	if (retry++ >= DEFAULT_I2C_RETRY) {
		ret = -EIO;
		goto out_write;
	}
	data_buff[0] = cmd;
	memcpy(data_buff + 1, data, len);
	data_buff[len + 1] = crc8(crc8_table, data_buff, len + 1, 0);
	ret = oplus_i2c_dma_write(chip->client, cmd, len + 1, data_buff + 1);
	if (ret < 0)
		goto out_write;
	ret = oplus_i2c_dma_read(chip->client, RT5125_CMD_CRC8_INFO, 1, data_buff);
	if (ret < 0)
		goto out_write;
	if (data_buff[0] != (~data_buff[len + 1] & 0xff))
		goto retry_write;
out_write:
	mutex_unlock(&data_lock);
	return ret;
}

static int rt5125_fw_update(struct oplus_vooc_chip *chip)
{
	const u8 *data = chip->firmware_data;
	s32 len = chip->fw_data_count;
	u8 rwdata = 0, status, fwdata[DEFAULT_MAX_PAGELEN];
	u8 last_page[DEFAULT_MAX_PAGELEN];
	s32 elapsed, wr_len;
	u32 fw_info;
	int i, idx, retry, ret;
	u32 keyword = 0;
	/* poly = x^8 + x^2 + x^1 + 1 */
	crc8_populate_msb(crc8_table, 0x7);
	/* wirte every single page */
	for (i = 0; i < len; i += DEFAULT_MAX_PAGELEN) {
		idx = i / DEFAULT_MAX_PAGELEN;
		elapsed = len - i;
		wr_len = (elapsed > DEFAULT_MAX_PAGELEN)
		    ? DEFAULT_MAX_PAGELEN : elapsed;
		memset(fwdata, 0xff, DEFAULT_MAX_PAGELEN);
		memcpy(fwdata, data + i, wr_len);
		/* page idx */
		rwdata = (u8) idx;
		ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX, &rwdata, sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] wrpage idx fail\n", idx);
			goto update_fw_err;
		}
		/* page data */
		ret = rt5125_i2c_block_write(chip, RT5125_DATA_BUF, fwdata, DEFAULT_MAX_PAGELEN);
		if (ret < 0) {
			chg_err("[%d] wrpage data fail\n", idx);
			goto update_fw_err;
		}
		/* access to write page from buffer to mtp */
		rwdata = (u8) RT5125_WT_PAGE;
		ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata, sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] wrpage access fail\n", idx);
			goto update_fw_err;
		}
		/* wait 128ms for mtp write */
		msleep(128);
		retry = 0;
busy_check:
		if (retry++ > DEFAULT_PAGEWR_RETRY) {
			chg_err("[%d] wrpage over retrycnt\n", idx);
			goto update_fw_err;
		}
		msleep(5);
		/* check page wr status */
		ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata, sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] read status fail\n", idx);
			goto update_fw_err;
		}
		status = rwdata & RT5125_OPSTATUS_MASK;
		if (status & RT5125_OPSTATUS_FAILMSK) {
			chg_err("[%d] wrpage fail 0x%02x\n", idx, status);
			goto update_fw_err;
		} else if (status == RT5125_OPSTATUS_ONGOING) {
			chg_err("[%d] wrpage ongoing\n", idx);
			goto busy_check;
		} else if (status == RT5125_OPSTATUS_SUCCESS)
			chg_err("[%d] wrpage success\n", idx);

		else {
			chg_err("[%d] wrpage unknown 0x%02x\n", idx, status);
			goto update_fw_err;
		}
	}
	/* FWINFO[31:16] = CRC16_H:CRC16_L */
	fw_info = crc16(RT5125_CRC16_INIT, data, len) << 16;
	/* FWINFO[15:0] = FWSIZE_H:FWSIZE_L */
	fw_info |= (u16) len;
	ret = rt5125_i2c_block_write(chip, RT5125_FW_CRC16_INFO, (void *)&fw_info, sizeof(fw_info));
	if (ret < 0) {
		chg_err("write fw info fail\n");
		goto update_fw_err;
	}
	/* write access for crc16 write */
	rwdata = (u8) RT5125_WT_FW_CRC16;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("wr_fw_crc access fail\n");
		goto update_fw_err;
	}
	/* wait 20ms for CRC write */
	msleep(20);
	/* check wr_fw_crc status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("wr_fw_crc read status fail\n");
		goto update_fw_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("wr_fw_crc status fail\n");
		goto update_fw_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS)
		chg_err("wr_fw_crc success\n");
	else {
		chg_err("wr_fw_crc unknown 0x%02x\n", status);
		goto update_fw_err;
	}
	retry = 0;
	/* write access for crc16 verify */
	rwdata = (u8) RT5125_FW_CRC16_VRFY;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy access fail\n");
		goto update_fw_err;
	}
	/* wait 200ms for CRC verify */
crc_busy_retry:
	msleep(200);
	/* check fw_crc_vrfy status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy read status fail\n");
		goto update_fw_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_ONGOING) {
			dev_err(chip->dev, "fw_crc_vrfy ongoing\n");
			retry++;
			if (retry < 10)
				goto crc_busy_retry;
			else{
				dev_err(chip->dev, "fw_crc_vrfy busy retry fail\n");
				goto update_fw_err;
			}
	} else if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("fw_crc_vrfy status fail\n");
		goto update_fw_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS)
		chg_err("fw_crc_vrfy success\n");

	else {
		chg_err("fw_crc_vrfy unknown 0x%02x\n", status);
		goto update_fw_err;
	}
	if (!(rwdata & RT5125_FW_CRC16_RSLT)) {
		chg_err("fw_crc_vrfy crc result fail\n");
		goto update_fw_err;
	}
	chg_err("fw_crc_vrfy OK\n");
	/* write access for keyword */
	retry = 0;
keyword_retry:
	rwdata = (u8) RT5125_WT_KEY;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("key_write access fail\n");
		goto update_fw_err;
	}
	/* wait keyword write success */
	msleep(10);
	/* check keyword write status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("key_write read status fail\n");
		goto update_fw_err;
	}
	retry++;
	if (retry > 10 ) {
		dev_err(chip->dev, "Key_write fail by retry over %d\n", retry);
		return -EFAULT;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("key_write status fail retry 0x%02x\n", status);
		goto keyword_retry;
	} else if (status == RT5125_OPSTATUS_SUCCESS)
		chg_err("key_write success\n");

	else {
		chg_err("key_write unknown status: 0x%02x, retry\n", status);
		goto keyword_retry;
	}

	/* check keyword correctly */
	idx = DEFAULT_MAX_PAGEIDX - 1;
	rwdata = (u8)idx;
	ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX,
				     &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] rdpage idx fail\n", idx);
		goto update_fw_err;
	}

	/* access to read page from mtp to buffer */
	rwdata = (u8)RT5125_RD_PAGE;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL,
				     &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d]rdpage access fail\n", idx);
		goto update_fw_err;
	}

	/* wait 5ms for mtp read */
	msleep(5);

	/* check page rd status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS,
				    &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] read status fail\n", idx);
		goto update_fw_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("[%d] rdpage fail 0x%02x\n", idx, status);
		goto keyword_retry;
	} else if (status == RT5125_OPSTATUS_SUCCESS) {
		chg_err("[%d] rdpage success\n", idx);
	} else {
		chg_err("[%d]rdpage unknown 0x%02x\n", idx, status);
		goto keyword_retry;
	}

	/* page data */
	ret = rt5125_i2c_block_read(chip, RT5125_DATA_BUF,
				    last_page, DEFAULT_MAX_PAGELEN);
	if (ret < 0) {
		chg_err("[%d] rdpage data fail\n", idx);
		goto update_fw_err;
	}

	/* get keyword */
	keyword = (last_page[127] << 8) + last_page[126];
	if (keyword != 0x5125) {
		dev_err(chip->dev, "key_write [%d] check fail\n", keyword);
		goto keyword_retry;
	} else {
		chg_err("key_write success\n");
	}

	chip->fw_mcu_version = chip->fw_data_version;
	chg_debug("success\n");
	return 0;
update_fw_err:
	charger_abnormal_log = CRITICAL_LOG_VOOC_FW_UPDATE_ERR;
	chg_err("fail\n");
	return 1;
}

static bool rt5125_fw_update_check(struct oplus_vooc_chip *chip)
{
	const u8 *data = chip->firmware_data;
	s32 len = chip->fw_data_count;
	u8 rwdata = 0, status, fwdata[DEFAULT_MAX_PAGELEN * 2];
	u8 last_page[DEFAULT_MAX_PAGELEN];
	u8 verinfo[DEFAULT_VERINFO_LEN];
	u32 fw_info = 0;
	u32 keyword = 0;
	int i, idx, ret, retry = 0;

	if (!chip->firmware_data) {
		chg_err("rt5125_fw_data Null, Return\n");
		return FW_CHECK_FAIL;
	}
	/* poly = x^8 + x^2 + x^1 + 1 */
	crc8_populate_msb(crc8_table, 0x7);
	/* write access for crc16 verify */
	rwdata = (u8) RT5125_FW_CRC16_VRFY;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy access fail\n");
		goto fw_update_check_err;
	}
	/* wait 200ms for CRC verify */
crc_busy_retry:
	msleep(200);
	/* check fw_crc_vrfy status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("fw_crc_vrfy read status fail\n");
		goto fw_update_check_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_ONGOING) {
		dev_err(chip->dev, "fw_crc_vrfy ongoing\n");
		retry++;
		if (retry < 10)
			goto crc_busy_retry;
		else{
			dev_err(chip->dev, "fw_crc_vrfy busy retry fail\n");
			goto fw_update_check_err;
		}
	} else if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("fw_crc_vrfy status fail\n");
		goto fw_update_check_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS)
		chg_err("fw_crc_vrfy success\n");

	else {
		chg_err("fw_crc_vrfy unknown 0x%02x\n", status);
		goto fw_update_check_err;
	}
	if (!(rwdata & RT5125_FW_CRC16_RSLT)) {
		chg_err("fw_crc_vrfy crc result fail\n");
		goto fw_update_check_err;
	}
	chg_err("fw_crc_vrfy OK\n");
	/* read orig fw info */
	idx = DEFAULT_MAX_PAGEIDX - 1;
	rwdata = (u8) idx;
	ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] rdpage idx fail\n", idx);
		goto fw_update_check_err;
	}
	/* access to read page from mtp to buffer */
	rwdata = (u8) RT5125_RD_PAGE;
	ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d]rdpage access fail\n", idx);
		goto fw_update_check_err;
	}
	/* wait 5ms for mtp read */
	msleep(5);
	/* check page rd status */
	ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata, sizeof(rwdata));
	if (ret < 0) {
		chg_err("[%d] read status fail\n", idx);
		goto fw_update_check_err;
	}
	status = rwdata & RT5125_OPSTATUS_MASK;
	if (status & RT5125_OPSTATUS_FAILMSK) {
		chg_err("[%d] rdpage fail 0x%02x\n", idx, status);
		goto fw_update_check_err;
	} else if (status == RT5125_OPSTATUS_SUCCESS)
		chg_err("[%d] rdpage success\n", idx);

	else {
		chg_err("[%d]rdpage unknown 0x%02x\n", idx, status);
		goto fw_update_check_err;
	}
	/* page data */
	ret = rt5125_i2c_block_read(chip, RT5125_DATA_BUF, last_page, DEFAULT_MAX_PAGELEN);
	if (ret < 0) {
		chg_err("[%d] rdpage data fail\n", idx);
		goto fw_update_check_err;
	}
	/* get fw size */
	fw_info = (last_page[123] << 8) + last_page[122];
	if (fw_info < DEFAULT_VERINFO_LEN) {
		chg_err("size [%d] smaller than verinfo\n", fw_info);
		goto fw_update_check_err;
	}

	/* get keyword */
	keyword = (last_page[127] << 8) + last_page[126];
	if (keyword != 0x5125) {
		chg_err("keyword [%d] check fail\n", keyword);
		goto fw_update_check_err;
	}

	/* always read last two single page */
	idx = fw_info / DEFAULT_MAX_PAGELEN - 1;
	for (i = 0; i < 2; i++) {
		/* page idx */
		if ((idx + i) < 0 || (idx + i) >= DEFAULT_MAX_PAGEIDX)
			continue;
		rwdata = (u8) (idx + i);
		ret = rt5125_i2c_block_write(chip, RT5125_PAGE_IDX, &rwdata, sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] rdpage idx fail\n", idx + i);
			goto fw_update_check_err;
		}
		/* access to read page from mtp to buffer */
		rwdata = (u8) RT5125_RD_PAGE;
		ret = rt5125_i2c_block_write(chip, RT5125_ACCESS_CTL, &rwdata, sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d]rdpage access fail\n", idx + i);
			goto fw_update_check_err;
		}
		/* wait 5ms for mtp read */
		msleep(5);
		/* check page rd status */
		ret = rt5125_i2c_block_read(chip, RT5125_STATUS, &rwdata, sizeof(rwdata));
		if (ret < 0) {
			chg_err("[%d] read status fail\n", idx + i);
			goto fw_update_check_err;
		}
		status = rwdata & RT5125_OPSTATUS_MASK;
		if (status & RT5125_OPSTATUS_FAILMSK) {
			chg_err("[%d] rdpage fail 0x%02x\n", idx + i, status);
			goto fw_update_check_err;
		} else if (status == RT5125_OPSTATUS_SUCCESS)
			chg_err("[%d] rdpage success\n", idx + i);

		else {
			chg_err("[%d]rdpage unknown 0x%02x\n", idx + i, status);
			goto fw_update_check_err;
		}
		/* page data */
		ret = rt5125_i2c_block_read(chip, RT5125_DATA_BUF, fwdata + i * DEFAULT_MAX_PAGELEN, DEFAULT_MAX_PAGELEN);
		if (ret < 0) {
			chg_err("[%d] rdpage data fail\n", idx + i);
			goto fw_update_check_err;
		}
	}
	idx = DEFAULT_MAX_PAGELEN + fw_info % DEFAULT_MAX_PAGELEN - DEFAULT_VERINFO_LEN;
	memcpy(verinfo, fwdata + idx, DEFAULT_VERINFO_LEN);
	chip->fw_mcu_version = verinfo[DEFAULT_VERINFO_LEN - 4];
	chg_err("update before: fw_mcu_version=%x,%x\n", chip->fw_mcu_version, verinfo[7]);
	idx = len - DEFAULT_VERINFO_LEN;
	ret = memcmp(verinfo, data + idx, DEFAULT_VERINFO_LEN);
	if (ret != 0) {
		chg_err("verinfo not equal\n");
		goto fw_update_check_err;
	}
	return FW_CHECK_SUCCESS;
fw_update_check_err:
	chg_err("rt5125_fw_data check fail\n");
	return FW_CHECK_FAIL;
}

/******************************************************************/
static int rt5125_get_fw_verion_from_ic(struct oplus_vooc_chip *chip)
{
	unsigned char addr_buf[2] = { 0x3B, 0xF0 };
	unsigned char data_buf[4] = { 0 };
	int rc = 0;
	int update_result = 0;
	chg_debug("kilody in\n");

	if (oplus_is_power_off_charging(chip) == true || oplus_is_charger_reboot(chip) == true) {
		chip->mcu_update_ing = true;
		update_result = rt5125_fw_update(chip);
		chip->mcu_update_ing = false;
		if (update_result) {
			msleep(30);
			opchg_set_clock_sleep(chip);
			opchg_set_reset_active_force(chip);
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
/*		first:set address*/
		rc = oplus_vooc_i2c_write(chip->client, 0x01, 2, &addr_buf[0]);
		if (rc < 0) {
			chg_err(" i2c_write 0x01 error\n");
			return FW_CHECK_FAIL;
		}
		msleep(2);
		oplus_vooc_i2c_read(chip->client, 0x03, 4, data_buf);

		chg_err("data:%x %x %x %x, fw_ver:%x\n", data_buf[0], data_buf[1], data_buf[2], data_buf[3], data_buf[0]);
		chip->mcu_update_ing = false;
		msleep(5);
		opchg_set_reset_active_force(chip);
	}
	return data_buf[0];
}

static int rt5125_fw_check_then_recover(struct oplus_vooc_chip *chip)
{
	int update_result = 0;
	int try_count = 5;
	int ret = 0;

	if (!chip->firmware_data) {
		chg_err("rt5125_fw_data Null, Return\n");
		return FW_ERROR_DATA_MODE;
	} else
		chg_debug("begin\n");
	if (oplus_is_power_off_charging(chip) == true || oplus_is_charger_reboot(chip) == true) {
		chip->mcu_update_ing = false;
		opchg_set_clock_sleep(chip);
		opchg_set_reset_sleep(chip);
		the_bat.reset_status = 0;
		ret = FW_NO_CHECK_MODE;
	} else {
		chip->mcu_boot_by_gpio = false;
		opchg_set_clock_active(chip);
		chip->mcu_boot_by_gpio = true;
		msleep(10);
		opchg_set_reset_active_force(chip);
		chip->mcu_update_ing = true;
		msleep(2500);
		chip->mcu_boot_by_gpio = false;
		opchg_set_clock_sleep(chip);
		__pm_stay_awake(rt5125_update_wake_lock);
		if (rt5125_fw_update_check(chip) == FW_CHECK_FAIL) {
			chg_debug("firmware update start\n");
			do {
				opchg_set_clock_active(chip);
				chip->mcu_boot_by_gpio = true;
				msleep(10);
				chip->mcu_update_ing = false;
				opchg_set_reset_active(chip);
				chip->mcu_update_ing = true;
				msleep(2500);
				chip->mcu_boot_by_gpio = false;
				opchg_set_clock_sleep(chip);
				update_result = rt5125_fw_update(chip);
				if (!update_result)
					break;
			} while ((update_result) && (--try_count > 0));
			chg_debug("firmware update end, retry times %d\n", 5 - try_count);
		} else {
			chip->vooc_fw_check = true;
			chg_debug("fw check ok\n");
		}
		__pm_relax(rt5125_update_wake_lock);
		chip->mcu_update_ing = false;
		msleep(5);
		opchg_set_reset_active(chip);
		ret = FW_CHECK_MODE;
	}
	opchg_set_reset_sleep(chip);
	the_bat.reset_status = 0;
	return ret;
}

#if 1
int rt5125_get_battery_mvolts_current(void)
{
	int ret = 0;
	int uv_bat;
	int current_bat;
	int debug_addr1 = 0;
	int debug_addr2 = 0;
	static int asic_err = 0;
	u8 read_buf[4] = { 0 };
	ret = oplus_i2c_dma_read(the_chip->client, 0x00, 2, read_buf);
	if (ret < 0) {
		chg_err("rt5125 read slave ack fail, reset status:%d\n", oplus_vooc_get_reset_gpio_status());
		the_bat.uv_bat = 0;
		if (oplus_vooc_get_fastchg_started() != true)
		the_bat.i2c_err_count++;
		return -1;
	}
	the_bat.i2c_err_count = 0;
	uv_bat = (read_buf[1] << 8) | read_buf[0];
	ret = oplus_i2c_dma_read(the_chip->client, 0x02, 4, read_buf);
	if (ret < 0) {
		chg_err("rt5125 read slave ack fail, reset status:%d\n", oplus_vooc_get_reset_gpio_status());
		the_bat.current_bat = 0;
		return -1;
	}
	current_bat = (read_buf[3] << 24) | (read_buf[2] << 16) | (read_buf[1] << 8) | read_buf[0];
	ret = oplus_i2c_dma_read(the_chip->client, 0x20, 2, read_buf);
	if (ret < 0) {
		chg_err("rt5125 read slave ack fail, reset status:%d\n", oplus_vooc_get_reset_gpio_status());
		return -1;
	}
	debug_addr1 = (read_buf[1] << 8) | read_buf[0];
	ret = oplus_i2c_dma_read(the_chip->client, 0x22, 2, read_buf);
	if (ret < 0) {
		chg_err("rt5125 read slave ack fail, reset status:%d\n", oplus_vooc_get_reset_gpio_status());
		return -1;
	}
	debug_addr2 = (read_buf[1] << 8) | read_buf[0];
	if ((uv_bat != 0) && (uv_bat != 0xffff)) {
		if ((the_bat.uv_bat == 0) || ((abs(uv_bat - the_bat.uv_bat)) < 500)) {
			the_bat.uv_bat = uv_bat;
			the_bat.current_bat = current_bat;
			asic_err = 0;
		} else {
			asic_err++;
			chg_err("rt5125 read uvbat err,uv_bat=%d,the_bat.uv_bat=%d\n", uv_bat, the_bat.uv_bat);
			if (asic_err > 5) {
				the_bat.uv_bat = uv_bat;
				asic_err = 0;
				chg_err("rt5125 read uvbat err 5 times,uv_bat=%d,the_bat.uv_bat=%d\n", uv_bat, the_bat.uv_bat);
				return -1;
			}
		}
	}
	chg_err("kilody:rt5125:uv_bat=%d mv,current_bat=%d ma\n", uv_bat, current_bat);
	chg_err("kilody:rt5125:debug_addr1=%d ,debug_addr2=%d \n", debug_addr1, debug_addr2);
	return 0;
}

int rt5125_get_prev_battery_mvolts(void)
{
	if (the_bat.reset_status == 0) {
		chg_debug("rt5125 reset sleep, cannot read volts\n");
		return 0;
	}
	if (the_chip->mcu_update_ing) {
		chg_debug("mcu_update_ing:%d,return\n", the_chip->mcu_update_ing);
		return 0;
	}
	if (the_bat.i2c_err_count > 10) {
		chg_debug("i2c_err_count:%d,return\n", the_bat.i2c_err_count);
		return 0;
	}
	if (oplus_vooc_get_fastchg_started() != true) {
		rt5125_get_battery_mvolts_current();
	}
	return the_bat.uv_bat;
}

int rt5125_set_battery_temperature_soc(int temp_bat, int soc_bat)
{
	int ret = 0;
	u8 read_buf[4] = { 0 };
	chg_err("kilody write rt5125:temp_bat=%d,soc_bat=%d\n", temp_bat, soc_bat);

	the_bat.temp_bat = temp_bat;
	the_bat.soc_bat = soc_bat;
	read_buf[0] = temp_bat & 0xff;
	read_buf[1] = (temp_bat >> 8) & 0xff;

	ret = oplus_i2c_dma_write(the_chip->client, 0x06, 2, read_buf);
	if (ret < 0) {
		chg_err("rt5125 write slave ack fail");
		return -1;
	}
	read_buf[0] = soc_bat & 0xff;
	read_buf[1] = (soc_bat >> 8) & 0xff;
	ret = oplus_i2c_dma_write(the_chip->client, 0x08, 2, read_buf);
	if (ret < 0) {
		chg_err("rt5125 write slave ack fail");
		return -1;
	}
	return 0;
}

int rt5125_get_prev_battery_current(void)
{
	if (the_bat.reset_status == 0) {
		chg_debug("rt5125 reset sleep, cannot read volts\n");
		return 0;
	}
	if (the_chip->mcu_update_ing) {
		chg_debug("mcu_update_ing:%d,return\n", the_chip->mcu_update_ing);
		return 0;
	}
	if (the_bat.i2c_err_count > 10) {
		chg_debug("i2c_err_count:%d,return\n", the_bat.i2c_err_count);
		return 0;
	}
	return -the_bat.current_bat/1000;
}

extern bool oplus_chg_get_chging_status(void);

static void rt5125_update_battery_temperature_soc(struct work_struct *work)
{
	int temp = 0;
	int soc = 0;

	if (oplus_chg_get_chging_status()) {
		soc = oplus_gauge_get_batt_soc();
		temp = oplus_chg_match_temp_for_chging();
		rt5125_get_battery_mvolts_current();
		if (oplus_vooc_get_fastchg_ing())
			rt5125_set_battery_temperature_soc(temp, soc);
	} else {
		the_bat.uv_bat = 0;
		the_bat.current_bat = 0;
	}
	schedule_delayed_work(&rt5125_update_temp_soc, round_jiffies_relative(msecs_to_jiffies(400)));
}

void rt5125_update_temperature_soc(void)
{
	int temp = 0;
	int soc = 0;
	if (oplus_chg_get_chging_status()) {
		soc = oplus_gauge_get_batt_soc();
		temp = oplus_chg_match_temp_for_chging();
		rt5125_get_battery_mvolts_current();
		if (1)
			rt5125_set_battery_temperature_soc(temp, soc);
	} else {
		the_bat.uv_bat = 0;
		the_bat.current_bat = 0;
	}
	chg_err("kilody in! soc = %d,temp = %d,uv_bat = %d,current_bat = %d,chging = %d\n", soc, temp, the_bat.uv_bat, the_bat.current_bat,
		oplus_vooc_get_fastchg_ing());
}

void rt5125_update_work_init(void)
{
	INIT_DELAYED_WORK(&rt5125_update_temp_soc, rt5125_update_battery_temperature_soc);
	/*      schedule_delayed_work(&rt5125_update_temp_soc, round_jiffies_relative(msecs_to_jiffies(1000))); */
}

void rt5125_set_reset_active(struct oplus_vooc_chip *chip)
{
	the_bat.uv_bat = 0;
	the_bat.current_bat = 0;
	opchg_set_reset_active(chip);
	the_bat.reset_status = 1;
	the_bat.i2c_err_count = 0;
}

void rt5125_set_reset_sleep(struct oplus_vooc_chip *chip)
{
	the_bat.uv_bat = 0;
	the_bat.current_bat = 0;
	opchg_set_reset_sleep(chip);
	the_bat.reset_status = 0;
}
#endif

struct oplus_vooc_operations oplus_rt5125_ops = {
	.fw_update = rt5125_fw_update,
	.fw_check_then_recover = rt5125_fw_check_then_recover,
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
	.reset_mcu = rt5125_set_reset_active,
	.set_mcu_sleep = rt5125_set_reset_sleep,
	.set_vooc_chargerid_switch_val = opchg_set_vooc_chargerid_switch_val,
	.is_power_off_charging = oplus_is_power_off_charging,
	.get_reset_gpio_val = oplus_vooc_get_reset_gpio_val,
	.get_switch_gpio_val = oplus_vooc_get_switch_gpio_val,
	.get_ap_clk_gpio_val = oplus_vooc_get_ap_clk_gpio_val,
	.get_fw_version = rt5125_get_fw_verion_from_ic,
	.get_clk_gpio_num = opchg_get_clk_gpio_num,
	.get_data_gpio_num = opchg_get_data_gpio_num,
	.update_temperature_soc = rt5125_update_temperature_soc,
};

struct oplus_plat_gauge_operations oplus_rt5125_plat_ops = {
	.get_plat_battery_mvolts = rt5125_get_prev_battery_mvolts,
	.get_plat_battery_current = rt5125_get_prev_battery_current,
};

static void register_vooc_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;
	version = "rt5125";
	manufacture = "ROCKCHIP";
	ret = register_device_proc("vooc", version, manufacture);
	if (ret)
		chg_err(" fail\n");
}

static void rt5125_shutdown(struct i2c_client *client)
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
}

static int rt5125_parse_fw_from_dt(struct oplus_vooc_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	const char *data;
	int len = 0;

	if (!node) {
		pr_err("device tree info. missing\n");
		return -ENOMEM;
	}

	data = of_get_property(node, "vooc,firmware_data", &len);
	if (!data) {
		pr_err("%s: parse vooc fw failed\n", __func__);
		return -ENOMEM;
	}

	chip->firmware_data = data;
	chip->fw_data_count = len;
	chip->fw_data_version = data[len - 4];
	pr_err("%s: version: 0x%x, count: %d\n", __func__, chip->fw_data_version, chip->fw_data_count);

	return 0;
}

static int rt5125_parse_fw_from_array(struct oplus_vooc_chip *chip)
{
	if (chip->batt_type_4400mv) {
		chip->firmware_data = rt5125_fw;
		chip->fw_data_count = sizeof(rt5125_fw);
		chip->fw_data_version = rt5125_fw[chip->fw_data_count - 4];
	} else {
		chip->firmware_data = rt5125_fw;
		chip->fw_data_count = sizeof(rt5125_fw);
		chip->fw_data_version = rt5125_fw[chip->fw_data_count - 4];
	}

	if (chip->vooc_fw_type == VOOC_FW_TYPE_RT5125_4400_VOOC_FFC_15C) {
		chip->firmware_data = rt5125_fw;
		chip->fw_data_count = sizeof(rt5125_fw);
		chip->fw_data_version = rt5125_fw[chip->fw_data_count - 4];
	}

	return 0;
}

static int rt5125_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
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
	gpDMABuf_va = (u8 *) dma_alloc_coherent(&client->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va)
		chg_err("[Error] Allocate DMA I2C Buffer failed!\n");

	else
		chg_debug(" ppp dma_alloc_coherent success\n");
	memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif
#else
	gpDMABuf_pa = kmalloc(GTP_DMA_MAX_TRANSACTION_LENGTH, GFP_DMA);
#endif
	/*      if (get_vooc_mcu_type(chip) != OPLUS_VOOC_ASIC_HWID_RT5125) {
	   chg_err("It is not rt5125\n");
	   return -ENOMEM;
	   } */
	chip->pcb_version = g_hw_version;
	chip->vooc_fw_check = false;
	mutex_init(&chip->pinctrl_mutex);
#if 1
	oplus_vooc_fw_type_dt(chip);

	if (chip->parse_fw_from_dt)
		rt5125_parse_fw_from_dt(chip);
	else
		rt5125_parse_fw_from_array(chip);

	chip->vops = &oplus_rt5125_ops;
	chip->fw_mcu_version = 0;
	oplus_vooc_gpio_dt_init(chip);
	opchg_set_clock_sleep(chip);
	oplus_vooc_delay_reset_mcu_init(chip);
	rt5125_update_wake_lock = wakeup_source_register(NULL, "rt5125_update_wake_lock");
	if (chip->vooc_fw_update_newmethod) {
		if (oplus_is_rf_ftm_mode())
			oplus_vooc_fw_update_work_init(chip);
	} else
		oplus_vooc_fw_update_work_init(chip);
	rt5125_update_work_init();
	oplus_plat_gauge_init(&oplus_rt5125_plat_ops);
	the_bat.reset_status = 0;
	oplus_vooc_init(chip);
#endif
	register_vooc_devinfo();
	the_chip = chip;
	chg_debug("rt5125 success\n");
	return 0;
}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
static const struct of_device_id rt5125_match[] = {
	{.compatible = "oplus,rt5125-fastcg"},
	{},
};

static const struct i2c_device_id rt5125_id[] = {
	{"rt5125-fastcg", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, rt5125_id);

struct i2c_driver rt5125_i2c_driver = {
	.driver = {
		   .name = "rt5125-fastcg",
		   .owner = THIS_MODULE,
		   .of_match_table = rt5125_match,
		   },
	.probe = rt5125_driver_probe,
	.shutdown = rt5125_shutdown,
	.id_table = rt5125_id,
};

static int __init rt5125_subsys_init(void)
{
	int ret = 0;
	chg_debug(" init start\n");
	init_hw_version();
	if (i2c_add_driver(&rt5125_i2c_driver) != 0)
		chg_err(" failed to register rt5125 i2c driver.\n");

	else
		chg_debug(" Success to register rt5125 i2c driver.\n");
	return ret;
}

subsys_initcall(rt5125_subsys_init);
MODULE_DESCRIPTION("Driver for oplus vooc rt5125 fast mcu");
MODULE_LICENSE("GPL v2");
