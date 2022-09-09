#ifndef _MAX20328_H_
#define _MAX20328_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#define MAX20328_I2C_RETRY_DELAY	20
#define MAX20328_I2C_MAX_RETRIES	5

/* Register Map */
#define MAX20328_REG_DEVICE_ID	(0x00)
#define MAX20328_ADC_VAL        (0x01)
#define MAX20328_STATUS1        (0x02)
#define MAX20328_STATUS2        (0x03)
#define MAX20328_REG_INTERRUPT  (0x04)
#define MAX20328_REG_MASK		(0x05)

#define MAX20328_CTL1           (0x06)
#define MAX20328_CTL2           (0x07)
#define MAX20328_CTL3           (0x08)
#define MAX20328_ADC_CTL1           (0x09)
#define MAX20328_ADC_CTL2           (0x0A)
#define MAX20328_HIHS_VAL           (0x0B)
#define MAX20328_OMTP_VAL           (0x0C)
#define MAX20328_DEFAULT1           (0x0D)
#define MAX20328_DEFAULT2           (0x0E)

#define MAX20328_SW_MODE_MASK       (0x7)


struct max20328_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct mutex lock;
	struct mutex i2clock;
	struct mutex activelock;
	struct power_supply *usb_psy;
	struct notifier_block psy_nb;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head max20328_notifier;
	atomic_t usbc_mode;
#ifdef OPLUS_ARCH_EXTENDS
	bool hs_det_ready;    //hs detection module ready flag
#endif /*OPLUS_ARCH_EXTENDS*/
	#ifdef OPLUS_ARCH_EXTENDS
	int mode;    //switch mode
	#endif /*OPLUS_ARCH_EXTENDS*/
	s32 int_pin;
	s32 hs_det_pin;
	s32 max_irq;
	u8 irq_state;
	u8 reg_read_buf;
};

#define  MAX20328_USB_MODE  (0)
#define  MAX20328_AUDIO_MODE		   (1)
#define  MAX20328_AUDIO_AUTO_DET_MODE  (2)
#define  MAX20328_AUDIO_MAN_DET_MODE   (3)
#define MAX20328_LOWPOWER_MODE		(4)                     // working in low power mode, minize power consumption
#define MAX20328_DISP_UART_MODE		(5)                     // working in UART mode

enum max20328_function {
	MAX20328_USBC_ORIENTATION_CC1,
	MAX20328_USBC_ORIENTATION_CC2,
	MAX20328_USBC_DISPLAYPORT_DISCONNECTED,
	MAX20328_EVENT_MAX,
};

int max20328_set_switch_mode(int mode);
int max20328_swap_mic_gnd(void);
int max20328_switch_event(enum max20328_function event);
#ifdef OPLUS_ARCH_EXTENDS
void max20328_set_det_ready(void);
#endif /*VENDOR_EDIT*/
#ifdef OPLUS_ARCH_EXTENDS
int max20328_set_LR_cnt(bool state);
#endif /*OPLUS_ARCH_EXTENDS*/
int max20328_reg_notifier(struct notifier_block *nb, struct device_node *node);
int max20328_unreg_notifier(struct notifier_block *nb, struct device_node *node);
#endif
