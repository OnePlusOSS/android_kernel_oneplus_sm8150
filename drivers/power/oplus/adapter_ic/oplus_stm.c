/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: Charger IC management module for charger system framework.
*                          Manage all charger IC and define abstarct function flow.
**
** Version: 1.0
** Date created: 21:03:46, 05/04/2012
**
** --------------------------- Revision History: ------------------------------------------------------------
* <version>           <date>                <author>                            <desc>
************************************************************************************************************/

#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/xlog.h>
#include <linux/gpio.h>
#include <linux/module.h>
#else
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#endif

#include "oplus_stm.h"
#include "../oplus_charger.h"
#include "../oplus_adapter.h"

static struct chip_stm *the_chip = NULL;

static void vooc_uart_gpio_set_value(unsigned long pin, bool value)
{
	gpio_set_value(pin, value);
}

static int vooc_uart_gpio_get_value(unsigned long pin)
{
        return gpio_get_value(pin);
}

static void vooc_uart_tx_bit(unsigned char tx_data)
{
        static unsigned char tx_bit = BIT_START;
		struct chip_stm *chip = the_chip;
		
        switch (tx_bit) {
        case BIT_START:
                chip->tx_byte_over = false;
                vooc_uart_gpio_set_value(chip->uart_tx_gpio, 0);
                tx_bit = BIT_0;
                break;
        case BIT_0:
        case BIT_1:
        case BIT_2:
        case BIT_3:
        case BIT_4:
        case BIT_5:
        case BIT_6:
        case BIT_7:
                if (tx_data & (1 << tx_bit)) {
                        vooc_uart_gpio_set_value(chip->uart_tx_gpio, 1);
                } else {
                        vooc_uart_gpio_set_value(chip->uart_tx_gpio, 0);
                }
                tx_bit++;
                break;
        case BIT_STOP:
        case BIT_IDLE:
                vooc_uart_gpio_set_value(chip->uart_tx_gpio, 1);
                tx_bit = BIT_START;
                chip->tx_byte_over = true;
                break;
        default:
                break;
        }
}

static int vooc_uart_rx_bit(void)
{
        static unsigned char rx_bit = BIT_IDLE, rx_val = 0;
		struct chip_stm *chip = the_chip;
		
        switch (rx_bit) {
        case BIT_IDLE:
                chip->rx_byte_over = false;
                if (!vooc_uart_gpio_get_value(chip->uart_rx_gpio)) {
                        rx_bit = BIT_0;
                        chip->timer_delay = 75;        /*1.5 cycle*/
                } else {
                        chip->timer_delay = 2;        /*0.02 cycle*/
                }
                break;
        case BIT_0:
        case BIT_1:
        case BIT_2:
        case BIT_3:
        case BIT_4:
        case BIT_5:
        case BIT_6:
        case BIT_7:
                chip->timer_delay = 50;        /* 1 cycle*/
                if (vooc_uart_gpio_get_value(chip->uart_rx_gpio)) {
                        rx_val |= (1 << rx_bit);
                } else {
                        rx_val &= ~(1 << rx_bit);
                }
                rx_bit++;
                break;
        case BIT_STOP:
                rx_bit = BIT_IDLE;
                chip->rx_byte_over = true;
                break;
        default:
                break;
        }
        return rx_val;
}

static void vooc_uart_tx_byte(unsigned char tx_data)
{
		struct chip_stm *chip = the_chip;

		chip->timer_delay = 51;
        while (1) {
                vooc_uart_tx_bit(tx_data);
                udelay(chip->timer_delay);
                if (chip->tx_byte_over) {
                        chip->timer_delay = 25;
                        break;
                }
        }
}

static unsigned char vooc_uart_rx_byte(unsigned int cmd)
{
        unsigned char rx_val = 0;
        unsigned int count = 0;
        unsigned int max_count = 0;
		struct chip_stm *chip = the_chip;
		
        if (cmd == Read_Addr_Line_Cmd) {
                max_count = Read_Addr_Line_Cmd_Count;
        } else if (cmd == Write_Addr_Line_Cmd) {
                max_count = Write_Addr_Line_Cmd_Count;
        } else if (cmd == Erase_Addr_Line_Cmd) {
                max_count = Erase_Addr_Line_Cmd_Count;
        } else if (cmd == Read_All_Cmd) {
                max_count = Read_All_Cmd_Count;
        } else if (cmd == Erase_All_Cmd) {
                max_count = Erase_All_Cmd_Count;
        } else if (cmd == Boot_Over_Cmd) {
                max_count = Boot_Over_Cmd_Count;
        } else {
                max_count = Other_Cmd_count;
        }
        chip->rx_timeout = false;
        chip->timer_delay = 25;
        while (1) {
                rx_val = vooc_uart_rx_bit();
                udelay(chip->timer_delay);
                if (chip->rx_byte_over) {
                        return rx_val;
                }
                if (count > max_count) {
                        chip->rx_timeout = true;
                        return 0x00;
                }
                count++;
        }
}

static void vooc_uart_irq_fiq_enable(bool enable)
{
#if 0
        if (enable) {
                preempt_enable();
                local_fiq_enable();
                local_irq_enable();
        } else {
                local_irq_disable();
                local_fiq_disable();
                preempt_disable();
        }
        #endif
}

static int vooc_uart_write_some_addr(u8 *fw_buf, int length)
{
        unsigned int write_addr = 0, i = 0, fw_count = 0;
        unsigned char rx_val = 0;
		struct chip_stm *chip = the_chip;
		
        while (1) {
                /*cmd(2 bytes) + count(1 byte) + addr(2 bytes) + data(16 bytes)*/
                /*tx: 0xF5*/
                vooc_uart_irq_fiq_enable(false);
                vooc_uart_tx_byte((Write_Addr_Line_Cmd >> 8) & 0xff);
                vooc_uart_irq_fiq_enable(true);

                /*tx: 0x02*/
                vooc_uart_irq_fiq_enable(false);
                vooc_uart_tx_byte(Write_Addr_Line_Cmd & 0xff);
                vooc_uart_irq_fiq_enable(true);

                /*count:16 bytes*/
                vooc_uart_irq_fiq_enable(false);
                vooc_uart_tx_byte(16);
                vooc_uart_irq_fiq_enable(true);

                /*addr: 2 byte*/
                if (write_addr == 0) {
                        write_addr = (fw_buf[fw_count + 1] << 8) | fw_buf[fw_count];
                }
                vooc_uart_irq_fiq_enable(false);
                vooc_uart_tx_byte((write_addr >> 8) & 0xff);
                vooc_uart_irq_fiq_enable(true);

                vooc_uart_irq_fiq_enable(false);
                vooc_uart_tx_byte(write_addr & 0xff);
                vooc_uart_irq_fiq_enable(true);

                if (!(write_addr % 0x20)) {
                        fw_count += 2;
                }
                /*data: 16 bytes*/
                for (i = 0;i < 16;i++) {
                        vooc_uart_irq_fiq_enable(false);
                        vooc_uart_tx_byte(fw_buf[fw_count]);
                        fw_count++;
                        if (i == 15) {
                                rx_val = vooc_uart_rx_byte(Write_Addr_Line_Cmd);
                        }
                        vooc_uart_irq_fiq_enable(true);
                }
                write_addr += 16;
                if (rx_val != UART_ACK || chip->rx_timeout) {
                        chg_err(" err, write_addr:0x%x, chip->rx_timeout:%d\n", write_addr, chip->rx_timeout);
                        return -1;
                }
                if (fw_count >= length) {
                        return 0;
                }
        }
}

#define STM8S_ADAPTER_FIRST_ADDR                0x8C00
#define STM8S_ADAPTER_LAST_ADDR                 0x9FEF
#define HALF_ONE_LINE                           16

static bool vooc_uart_read_addr_line_and_check(unsigned int addr)
{
        unsigned char fw_check_buf[20] = {0x00};
        int i = 0;
        static int fw_line = 0;
        bool check_result = false;
        int addr_check_err = 0;
		struct chip_stm *chip = the_chip;
		
        if (addr == STM8S_ADAPTER_FIRST_ADDR) {
                fw_line = 0;
        }
        /*Tx_Read_Addr_Line */
        /*tx:0xF5*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((Read_Addr_Line_Cmd >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x01*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(Read_Addr_Line_Cmd & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x9F*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((addr >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0xF0*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(addr & 0xff);

        /*addr(2 bytes) + data(16 bytes)*/
        fw_check_buf[0] = vooc_uart_rx_byte(Read_Addr_Line_Cmd);
        if (chip->rx_timeout) {
                goto  read_addr_line_err;
        }
        fw_check_buf[1] = vooc_uart_rx_byte(Read_Addr_Line_Cmd);
        if (chip->rx_timeout) {
                goto  read_addr_line_err;
        }
        if (addr != ((fw_check_buf[0] << 8) | fw_check_buf[1])) {
                addr_check_err = 1;
                goto read_addr_line_err;
        }
        for (i = 0; i < 16;i++) {
                fw_check_buf[i + 2] = vooc_uart_rx_byte(Read_Addr_Line_Cmd);
                if (chip->rx_timeout) {
                        goto  read_addr_line_err;
                }
        }
        if (!(addr % 0x20)) {
                if (addr == ((adapter_stm8s_firmware_data[fw_line * 34 + 1] << 8) | (adapter_stm8s_firmware_data[fw_line * 34]))) {
                        for (i = 0;i < 16;i++) {
                                if (fw_check_buf[i + 2] != adapter_stm8s_firmware_data[fw_line * 34 + 2 + i]) {
                                        goto read_addr_line_err;
                                }
                        }
                }
        } else {
                if ((addr - 16) ==
                        ((adapter_stm8s_firmware_data[fw_line * 34 + 1] << 8) | (adapter_stm8s_firmware_data[fw_line * 34]))) {
                        for (i = 0;i < 16;i++) {
                                if (fw_check_buf[i + 2] != adapter_stm8s_firmware_data[fw_line * 34 + 2 + HALF_ONE_LINE + i]) {
                                        goto read_addr_line_err;
                                }
                        }
                }
                fw_line++;
        }
        check_result = true;
read_addr_line_err:
        vooc_uart_irq_fiq_enable(true);
        if (addr_check_err) {
                chg_debug(" addr:0x%x, buf[0]:0x%x, buf[1]:0x%x\n", addr, fw_check_buf[0], fw_check_buf[1]);
        }
        if (!check_result) {
                chg_err(" fw_check err, addr:0x%x, check_buf[%d]:0x%x != fw_data[%d]:0x%x\n",
                         addr, i + 2, fw_check_buf[i + 2], (fw_line * 34 + 2 + i),
                        adapter_stm8s_firmware_data[fw_line * 34 + 2 + i]);
        }
        return check_result;
}

static int vooc_uart_read_front_addr_and_check(void)
{
        unsigned int read_addr = STM8S_ADAPTER_FIRST_ADDR;
        bool result = false;
		struct chip_stm *chip = the_chip;

        while (read_addr < STM8S_ADAPTER_LAST_ADDR) {
                result = vooc_uart_read_addr_line_and_check(read_addr);
                read_addr = read_addr + 16;
                if ((!result) || chip->rx_timeout) {
                        chg_err(" result:%d, chip->rx_timeout:%d\n", result, chip->rx_timeout);
                        return -1;
                }
        }
        return 0;
}

static bool vooc_adapter_update_handle(unsigned long tx_pin, unsigned long rx_pin)
{
        unsigned char rx_val = 0;
        int rx_last_line_count = 0;
        unsigned char rx_last_line[18] = {0x0};
        int rc = 0;
		struct chip_stm *chip = the_chip;
		
        chg_debug(" begin\n");
        chip->uart_tx_gpio = tx_pin;
        chip->uart_rx_gpio = rx_pin;
	
        chip->adapter_update_ing = true;
        chip->rx_timeout = false;
/*step1: Tx_Erase_Addr_Line*/
        /*tx:0xF5*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((Erase_Addr_Line_Cmd >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x03*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(Erase_Addr_Line_Cmd & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x9F*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((Last_Line_Addr >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0xF0*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(Last_Line_Addr & 0xff);
        rx_val = vooc_uart_rx_byte(Erase_Addr_Line_Cmd);
        vooc_uart_irq_fiq_enable(true);
        if (rx_val != UART_ACK || chip->rx_timeout) {
                chg_err(" Tx_Erase_Addr_Line err, chip->rx_timeout:%d\n", chip->rx_timeout);
                goto update_err;
        }

/*Step2: Tx_Read_Addr_Line */
        /*tx:0xF5*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((Read_Addr_Line_Cmd >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x01*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(Read_Addr_Line_Cmd & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x9F*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((Last_Line_Addr >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0xF0*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(Last_Line_Addr & 0xff);
        for (rx_last_line_count = 0; rx_last_line_count < 18;rx_last_line_count++) {
                rx_last_line[rx_last_line_count] = vooc_uart_rx_byte(Read_Addr_Line_Cmd);
                if (chip->rx_timeout) {
                        break;
                }
        }
        vooc_uart_irq_fiq_enable(true);
        if ((rx_last_line[FW_EXIST_LOW] == 0x55 && rx_last_line[FW_EXIST_HIGH] == 0x34) || chip->rx_timeout) {
                chg_err(" Tx_Read_Addr_Line err, chip->rx_timeout:%d\n",  chip->rx_timeout);
                goto update_err;
        }

/*Step3: Tx_Erase_All */
        /*tx:0xF5*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((Erase_All_Cmd >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x05*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(Erase_All_Cmd & 0xff);
        rx_val = vooc_uart_rx_byte(Erase_All_Cmd);
        vooc_uart_irq_fiq_enable(true);
        if (rx_val != UART_ACK || chip->rx_timeout) {
                chg_err(" Tx_Erase_All err, chip->rx_timeout:%d\n", chip->rx_timeout);
                goto update_err;
        }

/* Step4: Tx_Write_Addr_Line */
        rc = vooc_uart_write_some_addr(&adapter_stm8s_firmware_data[0],
                (sizeof(adapter_stm8s_firmware_data) - 34));
        if (rc) {
                chg_err(" Tx_Write_Addr_Line err\n");
                goto update_err;
        }

/* Step5: Tx_Read_All */
        rc = vooc_uart_read_front_addr_and_check();
        if (rc) {
                chg_err(" Tx_Read_All err\n");
                goto update_err;
        }

/* Step6: write the last line */
        rc = vooc_uart_write_some_addr(&adapter_stm8s_firmware_data[sizeof(adapter_stm8s_firmware_data) - 34], 34);
        if (rc) {
                chg_err(" write the last line err\n");
                goto update_err;
        }

/* Step7: Tx_Boot_Over */
        /*tx:0xF5*/
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte((Boot_Over_Cmd >> 8) & 0xff);
        vooc_uart_irq_fiq_enable(true);

        /*tx:0x06 */
        vooc_uart_irq_fiq_enable(false);
        vooc_uart_tx_byte(Boot_Over_Cmd & 0xff);
        rx_val = vooc_uart_rx_byte(Boot_Over_Cmd);
        vooc_uart_irq_fiq_enable(true);
        if (rx_val != UART_ACK || chip->rx_timeout) {
                chg_err("  Tx_Boot_Over err, chip->rx_timeout:%d\n" , chip->rx_timeout);
                goto update_err;
        }
        chip->rx_timeout = false;
        chip->adapter_update_ing = false;
        chg_debug(" success\n");
        return true;

update_err:
        chip->rx_timeout = false;
        chip->adapter_update_ing = false;
        chg_err(" err\n");
        return false;
}

#ifndef CONFIG_OPLUS_CHARGER_MTK
bool oplus_vooc_adapter_update_is_tx_gpio(unsigned long gpio_num)
{
        if (!the_chip) {
                return false;
        }
        if (the_chip->adapter_update_ing && gpio_num == the_chip->uart_tx_gpio) {
                return true;
        } else {
                return false;
        }
}

bool oplus_vooc_adapter_update_is_rx_gpio(unsigned long gpio_num)
{
        if (!the_chip) {
                return false;
        }
        if (the_chip->adapter_update_ing && gpio_num == the_chip->uart_rx_gpio) {
                return true;
        } else {
                return false;
        }
}
#endif

static void register_adapter_devinfo(void)
{
	int ret = 0;
	char *version;
	char *manufacture;
	
	version = "adapter";
	manufacture = "stm8s";

	ret = register_device_proc("adapter", version, manufacture);
	if (ret)
		chg_err("register_adapter_devinfo fail\n");
}

struct oplus_adapter_operations oplus_adapter_ops = {
        .adapter_update = vooc_adapter_update_handle,
};

static int __init adapter_ic_init(void)
{
        struct oplus_adapter_chip *chip = NULL;
		struct chip_stm *adapter_ic = NULL;

        adapter_ic = kzalloc(sizeof(struct chip_stm), GFP_KERNEL);
        if (!adapter_ic) {
                chg_err(" chip_stm alloc failed\n");
                return -1;
        }
        adapter_ic->timer_delay = 0;
        adapter_ic->tx_byte_over = false;
        adapter_ic->rx_byte_over = false;
        adapter_ic->rx_timeout = false;
        adapter_ic->uart_tx_gpio = 0;
        adapter_ic->uart_rx_gpio = 0;
        adapter_ic->adapter_update_ing = false;
        adapter_ic->adapter_firmware_data = adapter_stm8s_firmware_data;
        adapter_ic->adapter_fw_data_count = sizeof(adapter_stm8s_firmware_data);

		the_chip = adapter_ic;


        chip = kzalloc(sizeof(struct oplus_adapter_chip), GFP_KERNEL);
        if (!chip) {
                chg_err(" vooc_adapter alloc fail\n");
                return -1;
        }

 //       chip->client = client;
 //       chip->dev = &client->dev;
        chip->vops = &oplus_adapter_ops;
	
        oplus_adapter_init(chip);
		
		register_adapter_devinfo();

        chg_debug(" success\n");
        return 0;
}

/*
static void __init adapter_ic_exit(void)
{
        return;
}
*/

subsys_initcall(adapter_ic_init);
//module_exit(adapter_ic_exit);

MODULE_DESCRIPTION("Driver for oplus adapter ic stm8s");
MODULE_LICENSE("GPL v2");

