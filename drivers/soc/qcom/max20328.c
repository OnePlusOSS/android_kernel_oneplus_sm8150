/*
* Copyright (c) 2018 JY Kim, jy.kim@maximintegrated.com
* Copyright (c) 2017 Maxim Integrated Products, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/soc/qcom/max20328.h>
#include <linux/power_supply.h>

#define REG_CFG_CTL1_SW_EN			(0x13)  			// switch operation is on, default switch mode seletcion
#define REG_CFG_CTL1_SW_AUDIO 		(0x16)  			// switch operation is on, default switch mode seletcion
#define REG_CFG_CTL1_SW_DIS			(0x03)  			// switch operation is on, default switch mode seletcion
#define REG_CFG_ADC_CTL1_ADC_OFF   	(0x30)  			// ADC is always off, detection is not used in mux switch.
#define REG_CFG_ADC_CTL1_ADC_ON   	(0x33)  			// manual ADC detection, detection is not used in mux switch.
#define REG_CFG_ADC_CTL2_ADC_DET   	(0x51)  			// manual ADC detection, detection is not used in mux switch.
#define REG_CFG_CTL2_FORCE_MODE		(0xFB)              // force mode for mode switch
#define REG_CFG_CTL2_FORCE_MODE_2  	(0xC9)              // force mode for mode switch
#define REG_CFG_CTL2_FORCE_MODE_3  	(0xC8)              // force mode for mode switch
#define REG_CFG_CTL3_USB_MODE		(0x46)              // usb data connection
#define REG_CFG_CTL3_AUDIO_MODE     (0xA6)              // audio accessory connection
#define REG_CFG_CTL3_UART_MODE      (0x03)              // UART connection
#define SBU1_MIC_SBU2_AGND			(0x06)              // MIC to SBU1 AGND to SBU2
#define SBU1_AGND_SBU2_MIC			(0x09)              // MIC to SBU2 AGND to SBU1

#define STA1_EBO_BITS_MASK  		(0x08)
#define CTL3_MIC_AGND_BITS_MASK		(0x0F)


// need to be fine tuned
#define MAX20328_INTERVAL_MS	   	(5)
#define MAX20328_CHK_TIMES			(50)
#define MAX20328_HIHS_REF			(0x20)
#define MAX20328_OMTP_REF			(0xDF)

#ifdef OPLUS_ARCH_EXTENDS
#define MAX20328_SWITCH_DELAY_TIME			(10)  //ms
#endif /* OPLUS_ARCH_EXTENDS */

static const int delay_msec[] = {5, 10, 15, 20};        // for chip status checking loop
static struct max20328_data *s_max20328 = NULL;         // static variable for copy of driver data

static int audio_mic_gnd = 0;
module_param_named(mic_gnd, audio_mic_gnd, int, S_IRUGO);

#define DEBUG 1
/* I2C function */
static int max20328_write_reg(struct max20328_data *device,
	u8 reg_addr, u8 data)
{
	int err;
	int tries = 0;
	u8 buffer[2] = { reg_addr, data };
	struct i2c_msg msgs[] = {
		{
			.addr = device->client->addr,
			.flags = device->client->flags & I2C_M_TEN,
			.len = 2,
			.buf = buffer,
		},
	};

	do {
		mutex_lock(&device->i2clock);
		err = i2c_transfer(device->client->adapter, msgs, 1);
		mutex_unlock(&device->i2clock);
		if (err != 1)
			msleep_interruptible(MAX20328_I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < MAX20328_I2C_MAX_RETRIES));

	if (err != 1) {
		pr_err("%s -write transfer error\n", __func__);
		err = -EIO;
		return err;
	}
	return 0;
}

static int max20328_read_reg(struct max20328_data *data,
	u8 *buffer, int length)
{
	int err = -1;
	int tries = 0; /* # of attempts to read the device */
	int addr = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = data->client->addr,
			.flags = data->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buffer,
		},
		{
			.addr = data->client->addr,
			.flags = (data->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = length,
			.buf = buffer,
		},
	};
	addr = *buffer;
	do {
		mutex_lock(&data->i2clock);
		err = i2c_transfer(data->client->adapter, msgs, 2);
		mutex_unlock(&data->i2clock);
		if (err != 2)
			msleep_interruptible(MAX20328_I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < MAX20328_I2C_MAX_RETRIES));

	if (err != 2) {
		pr_err("%s -read transfer error, at %x\n", __func__, addr);
		err = -EIO;
	} else
		err = 0;

	return err;
}

#ifndef OPLUS_ARCH_EXTENDS
static inline int check_bit_valid(u8 reg, u8 bit_mask, int delay_ms, int try_num, bool invert)
{
	int i= 0;
	u8 val = 0;

	if (invert)
		bit_mask = ~bit_mask;

	/* val = reg; */
	/* max20328_read_reg(s_max20328, &val, 1); */

	for (i = 0; i < try_num; i++) {
		mdelay(delay_ms);
		val = reg;
		max20328_read_reg(s_max20328, &val, 1);
		if (invert){
			if( (val | bit_mask) == bit_mask ){
				pr_err("max20328 -check valid for reg(%x), bit_mask %x try times %d, invert\n", reg, bit_mask, i);
				return 1;
			}
		} else {
			if( (val & bit_mask) == bit_mask ){
				pr_err("max20328 -check valid for reg(%x), bit_mask %x try times %d\n", reg, bit_mask, i);
				return 1;
			}
		}
	}

	pr_err("max20328 %s -read status %x, bit_mask %x times %d\n", __func__, val, bit_mask, i);
	return 0;
}
#endif /* OPLUS_ARCH_EXTENDS */

int max20328_enable_FM(int enable_flag)
{
	int rc = -1;
    u8 val = 0;
    struct max20328_data * data = NULL;

	pr_err("%s\n", __func__);

    if (s_max20328){
		data = s_max20328;
        val = MAX20328_CTL3;
        rc = max20328_read_reg(data, &val, 1);

		if (enable_flag){
			val = ((val & ~0x3) | 0x3);             // force mgs
		}else{
			val = ((val & ~0x3) | ((~(val >> 2)) & 0x3));             // recovery status.
		}
    }
	return rc;
}
EXPORT_SYMBOL(max20328_enable_FM);

#ifndef OPLUS_ARCH_EXTENDS
int max20328_Device_RDY(void)
{
	return check_bit_valid(0x02, 0x02, 1, 20, false);
}
EXPORT_SYMBOL(max20328_Device_RDY);
#endif /* OPLUS_ARCH_EXTENDS */

int max20328_ADC_val(long *adc_val)
{
	int rc;
	long idet;
	u8 get = 0x02;
	rc = max20328_read_reg(s_max20328, &get, 1);
	if (get & 0x1){                         // EOC checking
		switch (get & 0xC0){
			case 0xC0:
				idet = 55;
				break;
			case 0x80:
				idet = 11;
				break;
			default:
				idet = 1;
				break;
		}
		get = 0x01;
		rc = max20328_read_reg(s_max20328, &get, 1);
		*adc_val = 47460;
        *adc_val = (*adc_val * get)/idet;

		return 0;
	}
	return -1;
}
EXPORT_SYMBOL(max20328_ADC_val);

int max20328_STATUS1_VAL(u8 *val)
{
	*val = 0x02;
	return max20328_read_reg(s_max20328, val, 1);
}
EXPORT_SYMBOL(max20328_STATUS1_VAL);

int max20328_STATUS2_VAL(u8 *val)
{
	*val = 0x03;
	return max20328_read_reg(s_max20328, val, 1);
}
EXPORT_SYMBOL(max20328_STATUS2_VAL);

int max20328_swap_mic_gnd(void)
{
	int rc = -1;
	struct max20328_data * data = NULL;
	u8 val = 0;

	pr_err("%s\n", __func__);

    if (s_max20328){
		data = s_max20328;
        #ifdef OPLUS_ARCH_EXTENDS
        if (MAX20328_AUDIO_MODE != data->mode) {
            pr_err("%s: mode(%d) not audio mode, return\n", __func__, data->mode);
            return -1;
        }
        #endif /* OPLUS_ARCH_EXTENDS */
        val = 0x0D;
        max20328_read_reg(s_max20328, &val, 1);
        #ifndef OPLUS_ARCH_EXTENDS
         if (val == 0x87){
            rc = max20328_write_reg(data, 0x0E, 0x10);
            rc = max20328_write_reg(data, 0x0D, 0x83);
            rc = max20328_write_reg(data, 0x06, 0x13);
        }else{
            rc = max20328_write_reg(data, 0x0E, 0x40);
            rc = max20328_write_reg(data, 0x0D, 0x87);
            rc = max20328_write_reg(data, 0x06, 0x13);
        }
        #else
        if ((val == 0x87) || (val == 0x07)){
            rc = max20328_write_reg(data, 0x0E, 0x10);
            if(val == 0x87) {
                rc = max20328_write_reg(data, 0x0D, 0x83);
            } else {
                rc = max20328_write_reg(data, 0x0D, 0x03);
            }
            rc = max20328_write_reg(data, 0x06, 0x13);
        }else if ((val == 0x83) || (val == 0x03)){
            rc = max20328_write_reg(data, 0x0E, 0x40);
            if(val == 0x83) {
                rc = max20328_write_reg(data, 0x0D, 0x87);
            } else {
                rc = max20328_write_reg(data, 0x0D, 0x07);
            }
            rc = max20328_write_reg(data, 0x06, 0x13);
        } else {
            pr_err("%s: reg(0x0d) = 0x%x error, set default audio\n", __func__, val);
            rc = max20328_write_reg(data, 0x0E, 0x10);
            rc = max20328_write_reg(data, 0x0D, 0x83);
            rc = max20328_write_reg(data, 0x06, 0x13);
        }
        #endif /* OPLUS_ARCH_EXTENDS */

        #ifndef OPLUS_ARCH_EXTENDS
        if ( check_bit_valid(0x02, 0x02, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // switching finalized checking to max 5ms ~ 100ms delay
        }
        #else
        msleep(MAX20328_SWITCH_DELAY_TIME);
        #endif /* OPLUS_ARCH_EXTENDS */

		audio_mic_gnd = 1;
    }
	return rc;
}
EXPORT_SYMBOL(max20328_swap_mic_gnd);

#ifdef OPLUS_ARCH_EXTENDS
int max20328_set_LR_cnt(bool state)
{
	int rc = -1;
	struct max20328_data * data = NULL;
	u8 val = 0;

	pr_err("%s: L/R connect state = %d\n", __func__, state);
    if (s_max20328){
        data = s_max20328;
        if(data->mode != MAX20328_AUDIO_MODE) {
            pr_err("%s: mode(%d) not audio mode, return\n", __func__, data->mode);
            return -1;
        }

        val = 0x0D;
        max20328_read_reg(s_max20328, &val, 1);
        if ((val != 0x87) && (val != 0x07)
            && (val != 0x83) && (val != 0x03)){
            pr_err("%s: reg(0x0d) = 0x%x error, just return\n", __func__, val);
            return -1;
        }

        if(true == state) {
            val |= 0x80;
        } else {
            val &= (~0x80);
        }
        rc = max20328_write_reg(data, 0x0D, val);
        pr_err("%s: set val = 0x%x ret = %d\n", __func__, val, rc);
    }
	return rc;
}
EXPORT_SYMBOL(max20328_set_LR_cnt);
#endif /* OPLUS_ARCH_EXTENDS */

int max20328_set_switch_mode(int mode)
{
	int rc = -1;
	struct max20328_data * data = NULL;

	pr_err("%s -mode(%d)\n", __func__, mode);

	if (s_max20328){
		data = s_max20328;
		switch(mode){
			case MAX20328_USB_MODE:                 //USB mode
				rc = max20328_write_reg(data, 0x0E, 0x00);         //DEF register2 set 00
				rc = max20328_write_reg(data, 0x0D, 0x40);         //DEF register1 set TOP side closed in data connection, bottom side is open
				#ifdef OPLUS_ARCH_EXTENDS
				data->mode = MAX20328_USB_MODE;
				#endif /* OPLUS_ARCH_EXTENDS */
				rc = max20328_write_reg(data, 0x07, 0x00);         //CONTROL2 register, switch state NOT Force mode nor follow MODE[0:2]
				rc = max20328_write_reg(data, 0x08, 0x00);         //CONTROL3 register, force value is not use, anyway default it.
				#ifndef OPLUS_ARCH_EXTENDS
				rc = max20328_write_reg(data, 0x09, 0x30);         //ADC CONTROL1, ADC is always off on USB MODE
				#else
				rc = max20328_write_reg(data, 0x09, 0x40);         //ADC CONTROL1, ADC is always off on USB MODE
				#endif /* OPLUS_ARCH_EXTENDS */
				rc = max20328_write_reg(data, 0x06, 0x13);         //CONTROL1 register, switch enable, default programmable with registers 0x0D and 0x0E
				#ifndef OPLUS_ARCH_EXTENDS
				if ( check_bit_valid(0x02, 0x02, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // switching finalized checking to max 5ms ~ 100ms delay
				}
				#else
				msleep(MAX20328_SWITCH_DELAY_TIME);
				#endif /* OPLUS_ARCH_EXTENDS */

				gpio_direction_output(data->hs_det_pin, 1);               // reset msm codec INT pin to high level

				audio_mic_gnd = 0;
				break;
			case MAX20328_AUDIO_MODE:
				rc = max20328_write_reg(data, 0x0E, 0x10);         //DEF register2
				rc = max20328_write_reg(data, 0x0D, 0x83);         //DEF register
				#ifdef OPLUS_ARCH_EXTENDS
				data->mode = MAX20328_AUDIO_MODE;
				#endif /* OPLUS_ARCH_EXTENDS */
				/* rc = max20328_write_reg(data, 0x0A, 0xF0);         //ADC CONTROL2 */
				rc = max20328_write_reg(data, 0x07, 0x02);         //CONTROL2 register
				rc = max20328_write_reg(data, 0x08, 0x00);         //CONTROL3 register
				/* rc = max20328_write_reg(data, 0x0B, MAX20328_HIHS_REF);         // High Impedance Threashold */
				/* rc = max20328_write_reg(data, 0x0C, MAX20328_OMTP_REF);         // OMTP headset Detection Threshold */
				#ifndef OPLUS_ARCH_EXTENDS
				rc = max20328_write_reg(data, 0x09, 0x00);         //ADC CONTROL1, ADC is always off on USB MODE
				#else
				rc = max20328_write_reg(data, 0x09, 0x40);         //ADC CONTROL1, ADC is always off on USB MODE
				#endif /* OPLUS_ARCH_EXTENDS */
				rc = max20328_write_reg(data, 0x06, 0x13);         // CONTROL1 register, switch enable, single Audio accessory
				#ifndef OPLUS_ARCH_EXTENDS
				if ( check_bit_valid(0x02, 0x02, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // switching finalized checking to max 5ms ~ 100ms delay
				}
				#else
				msleep(MAX20328_SWITCH_DELAY_TIME);
				#endif /* OPLUS_ARCH_EXTENDS */

				gpio_direction_output(data->hs_det_pin, 0);               // reset msm codec INT pin to high level
				break;
			case MAX20328_DISP_UART_MODE:
				rc = max20328_write_reg(data, 0x06, 0x14);         //CONTROL1 register,  UART MODE Top side USB switchs connected.
				#ifdef OPLUS_ARCH_EXTENDS
				data->mode = MAX20328_DISP_UART_MODE;
				#endif /* OPLUS_ARCH_EXTENDS */
				break;
			default:
				break;
		}
	}

	return rc;
}
EXPORT_SYMBOL(max20328_set_switch_mode);

int max20328_switch_event(enum max20328_function event)
{
	int rc = -1;
	struct max20328_data * data = NULL;
	if (s_max20328){
		data = s_max20328;
		switch(event){
			case MAX20328_USBC_ORIENTATION_CC1:
				rc = max20328_write_reg(data, 0x06, 0x33);
				rc = max20328_write_reg(data, 0x07, 0x08);
				break;
			case MAX20328_USBC_ORIENTATION_CC2:
				rc = max20328_write_reg(data, 0x06, 0x13);
				rc = max20328_write_reg(data, 0x07, 0x08);
				break;
			case MAX20328_USBC_DISPLAYPORT_DISCONNECTED:
				rc = max20328_set_switch_mode(MAX20328_USB_MODE);
				break;
			default:
				break;
		}
	}
	return rc;
}
EXPORT_SYMBOL(max20328_switch_event);

static int max20328_usbc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	int ret;
	union power_supply_propval mode;
	struct max20328_data *data =
			container_of(nb, struct max20328_data, psy_nb);

	if (!data)
		return -EINVAL;

	if(!data->usb_psy){
		data->usb_psy = power_supply_get_by_name("usb");
		if (!data->usb_psy) {
			pr_err("%s: could not get USB psy info\n",__func__);
			return -EINVAL;
		}
	}

	if ((struct power_supply *)ptr != data->usb_psy ||
				evt != PSY_EVENT_PROP_CHANGED)
		return 0;

	ret = power_supply_get_property(data->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &mode);
	if (ret) {
		pr_err("%s: Unable to read USB TYPEC_MODE: %d\n",__func__, ret);
		return ret;
	}

	switch (mode.intval) {
	case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
		#ifdef OPLUS_ARCH_EXTENDS
		if (false == data->hs_det_ready) {
			pr_err("%s: hs connected but detection module not ready, just return\n",__func__);
			return ret;
		}
		//do not need "break;"
		#endif /*OPLUS_ARCH_EXTENDS*/
	case POWER_SUPPLY_TYPEC_NONE:
		if (atomic_read(&(data->usbc_mode)) == mode.intval)
			return ret; /* filter notifications received before */

		atomic_set(&(data->usbc_mode), mode.intval);

		pr_err("%s: queueing usbc_analog_work, mode: %d\n",__func__, mode.intval);
		schedule_work(&data->usbc_analog_work);

		break;
	default:
		break;
	}
	return ret;
}

#ifdef OPLUS_ARCH_EXTENDS
void max20328_set_det_ready(void)
{
	if (!s_max20328)
		return;

	s_max20328->hs_det_ready = true;
	pr_err("%s: set hs_det_ready: %d\n",__func__, s_max20328->hs_det_ready);
}
EXPORT_SYMBOL(max20328_set_det_ready);
#endif /*OPLUS_ARCH_EXTENDS*/

int max20328_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_data *data;

	if (!s_max20328)
		return -EINVAL;

	if (!client)
		return -EINVAL;

	data = (struct max20328_data *)i2c_get_clientdata(client);
	if (!data)
		return -EINVAL;
	data = s_max20328;

	rc = blocking_notifier_chain_register
				(&data->max20328_notifier, nb);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	pr_err("%s: verify if USB adapter is already inserted\n",__func__);
	rc = max20328_usbc_event_changed(&data->psy_nb,
					     PSY_EVENT_PROP_CHANGED,
					     data->usb_psy);
	return rc;
}
EXPORT_SYMBOL(max20328_reg_notifier);

int max20328_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct max20328_data *data;

	if (!client)
		return -EINVAL;

	data = (struct max20328_data *)i2c_get_clientdata(client);
	if (!data)
		return -EINVAL;

	data = s_max20328;
	return blocking_notifier_chain_unregister
					(&data->max20328_notifier, nb);
}
EXPORT_SYMBOL(max20328_unreg_notifier);

static void max20328_usbc_analog_work_fn(struct work_struct *work)
{
	struct max20328_data *data =
		container_of(work, struct max20328_data, usbc_analog_work);

	if (!data) {
		pr_err("%s: max container invalid\n", __func__);
		return;
	}

	if(atomic_read(&(data->usbc_mode)) == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER){
		#ifndef OPLUS_ARCH_EXTENDS
		max20328_set_switch_mode(MAX20328_AUDIO_MODE);
		blocking_notifier_call_chain(&data->max20328_notifier,POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER, NULL);
		#else
		if(true == data->hs_det_ready) {
			max20328_set_switch_mode(MAX20328_AUDIO_MODE);
			blocking_notifier_call_chain(&data->max20328_notifier,POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER, NULL);
		} else {
			pr_err("%s: hs is connected but detection module has not ready\n", __func__);
		}
		#endif /*OPLUS_ARCH_EXTENDS*/
	} else {
		max20328_set_switch_mode(MAX20328_USB_MODE);
		blocking_notifier_call_chain(&data->max20328_notifier,POWER_SUPPLY_TYPEC_NONE, NULL);
	}
}

irqreturn_t max20328_irq_handler(int max_irq, void *device)
{
	int err;
	u8 interupt_status;
	struct max20328_data *data = device;

	interupt_status = MAX20328_REG_INTERRUPT;
	err = max20328_read_reg(data, &interupt_status, 1);
	if (err == 0) {
		pr_err("%s INTTERUPT : %02x\n", __func__, interupt_status);
		input_report_rel(data->input_dev, REL_X, (s32)interupt_status);
		input_sync(data->input_dev);
	} else {
		pr_err("%s INTERRUPT read fail:%d\n", __func__, err);
	}

	return IRQ_HANDLED;
}

static int max20328_parse_dt(struct max20328_data *data,
	struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	int ret;

	if (dNode == NULL)
		return -ENODEV;

	data->hs_det_pin = of_get_named_gpio(dNode,
		"max20328,hs-det-gpio", 0);
	if (data->hs_det_pin < 0) {
		pr_err("%s - get int error\n", __func__);
		return -ENODEV;
	}
	ret = gpio_request(data->hs_det_pin, "max20328_hs_det");
	gpio_direction_output(data->hs_det_pin, 1);
	return 0;
}

static int max20328_setup_irq(struct max20328_data *data)
{
#ifdef NEED_IRQ
	int errorno = -EIO;

	errorno = request_threaded_irq(data->max_irq, NULL,
		max20328_irq_handler, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
		"max20328_irq", data);

	if (errorno < 0) {
		pr_err("%s - failed for setup max_irq errono= %d\n",
			   __func__, errorno);
		errorno = -ENODEV;
		return errorno;
	}
	data->irq_state = 0;
	disable_irq(data->max_irq);

	return errorno;
#else
	return 0;
#endif
}

static void max20328_irq_set_state(struct max20328_data *data, int irq_enable)
{
	pr_err("%s - irq_enable : %d, irq_state : %d\n",
		__func__, irq_enable, data->irq_state);

	if (irq_enable) {
		if (data->irq_state == 0) {
			data->irq_state = 1;
			enable_irq(data->max_irq);
		}
	} else {
		if (data->irq_state == 0)
			return;
		if (data->irq_state == 1) {
			disable_irq(data->max_irq);
			data->irq_state = 0;
		}
	}
}

/* Register Read Access */
static ssize_t max20328_read_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct max20328_data *data = dev_get_drvdata(dev);

	pr_err("%s - val=0x%02x\n", __func__, data->reg_read_buf);

	return snprintf(buf, PAGE_SIZE, "%02x\n", (u32)data->reg_read_buf);
}

static ssize_t max20328_read_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int cmd = 0;
	u8 recvData;

	struct max20328_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->activelock);
	err = sscanf(buf, "%2x", &cmd);
	if (err == 0) {
		pr_err("%s - sscanf fail\n", __func__);
		mutex_unlock(&data->activelock);
		return size;
	}

	recvData = (u8)cmd;
	err = max20328_read_reg(data, &recvData, 1);
	if (err != 0) {
		pr_err("%s err=%d, val=0x%02x\n",
			__func__, err, recvData);
		mutex_unlock(&data->activelock);
		return size;
	}
	data->reg_read_buf = recvData;
	mutex_unlock(&data->activelock);

	return size;
}

static ssize_t max20328_write_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int cmd = 0;
	unsigned int val = 0;

	struct max20328_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->activelock);
	err = sscanf(buf, "%2x %2x", &cmd, &val);

	if (err == 0) {
		pr_err("%s - sscanf fail\n", __func__);
		mutex_unlock(&data->activelock);
		return size;
	}

	pr_err("%s - addr:0x%02x, val:0x%02x\n", __func__, cmd, val);

	/* Enable/Disable IRQ */
	if (cmd == MAX20328_REG_MASK) {
		if ( val == 0x00 )
			max20328_irq_set_state(data, 0);
		else
			max20328_irq_set_state(data, 1);
	}

	err = max20328_write_reg(data, (u8)cmd, (u8)val);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, cmd, val);
		mutex_unlock(&data->activelock);
		return size;
	}

	mutex_unlock(&data->activelock);

	return size;
}

static DEVICE_ATTR(read_reg, S_IRUGO|S_IWUSR|S_IWGRP,
	max20328_read_reg_show, max20328_read_reg_store);

static DEVICE_ATTR(write_reg, S_IRUGO|S_IWUSR|S_IWGRP,
	NULL, max20328_write_reg_store);

static struct attribute *max20328_sysfs_attrs[] = {
	&dev_attr_read_reg.attr,
	&dev_attr_write_reg.attr,
	NULL
};
static struct attribute_group hrm_attribute_group = {
	.attrs = max20328_sysfs_attrs,
};

static void max20328_initialize(struct max20328_data *data)
{
	data->reg_read_buf = 0;
	return;
}

static int max20328_usb_c_analog_init(struct max20328_data *data)
{
	int err = 0;

	data->psy_nb.notifier_call = max20328_usbc_event_changed;
	data->psy_nb.priority = 0;
	err = power_supply_reg_notifier(&data->psy_nb);
	if (err) {
		pr_err("%s: power supply reg failed: %d\n",__func__, err);
		return err;
	}

	INIT_WORK(&data->usbc_analog_work,max20328_usbc_analog_work_fn);
	pr_err("%s - end\n", __func__);
	return err;
}

static int max20328_probe(struct i2c_client *client,
			       const struct i2c_device_id *devid)
{
	int i, err = 0;
	struct max20328_data *data;
	struct input_dev *input_dev;
	u8 recvData;

	pr_err("%s - called\n", __func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}

	data->client = client;

	err = max20328_parse_dt(data, &client->dev);
	if (err < 0) {
		pr_err("%s - of_node error\n", __func__);
		err = -ENODEV;
		goto err_of_node;
	}

	err = max20328_setup_irq(data);
	if (err) {
		pr_err("%s - could not setup irq\n", __func__);
		goto err_setup_irq;
	}

	mutex_init(&data->i2clock);
	mutex_init(&data->lock);
	mutex_init(&data->activelock);

	recvData = MAX20328_REG_DEVICE_ID;
	err = max20328_read_reg(data, &recvData, 1);
	if (err) {
		pr_err("%s WHOAMI read fail\n", __func__);
		err = -ENODEV;
		goto err_read_reg;
	} else {
		pr_err("MAX20328 is detected. ID: %X\n", recvData);
	}

	recvData = MAX20328_STATUS2;
	for (i = 0; i < ARRAY_SIZE(delay_msec); i++){
		recvData = MAX20328_STATUS2;
		err = max20328_read_reg(data, &recvData, 1);
		if (!err && (recvData & STA1_EBO_BITS_MASK)){
			break;										// chip is ready now , goto initial setting.
		}
		msleep(delay_msec[i]);
	}
	if (i >= ARRAY_SIZE(delay_msec)){
 		pr_err("%s chip bootup failed\n", __func__);
		err = -ENODEV;
		goto err_read_reg;
	}

#ifdef OPLUS_ARCH_EXTENDS
#define MAX20328_ADC_CTL2_5V_MASK (0xF0)
	recvData = MAX20328_ADC_CTL2;
	err = max20328_read_reg(data, &recvData, 1);
	if (err) {
		pr_err("%s read MAX20328_ADC_CTL2 fail\n", __func__);
		err = -ENODEV;
		goto err_read_reg;
	} else {
		if ((recvData & MAX20328_ADC_CTL2_5V_MASK) != MAX20328_ADC_CTL2_5V_MASK) {
			pr_err("%s Need to update OVP to 5V.\n", __func__);
			recvData = MAX20328_ADC_CTL2_5V_MASK;
			err = max20328_write_reg(data, MAX20328_ADC_CTL2, (u8)recvData);
			if (err != 0) {
				pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
				__func__, err, MAX20328_ADC_CTL2, recvData);
				goto err_read_reg;
			}
			msleep(10);
		}
	}
#endif /* OPLUS_ARCH_EXTENDS */
    recvData = 0x40;				//default value
 	err = max20328_write_reg(data, 0x0D, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x0D, recvData);
		goto err_read_reg;
	}

    recvData = 0x0;				//default value
 	err = max20328_write_reg(data, 0x0E, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x0E, recvData);
		goto err_read_reg;
	}

    recvData = 0xF0;				//default value
 	err = max20328_write_reg(data, 0x0A, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x0A, recvData);
		goto err_read_reg;
	}

    recvData = 0x0;				//default value
 	err = max20328_write_reg(data, 0x07, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x07, recvData);
		goto err_read_reg;
	}

    recvData = 0x0;				//default value
 	err = max20328_write_reg(data, 0x08, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x08, recvData);
		goto err_read_reg;
	}

    recvData = 0x30;				//ADC is always off and disable all
 	err = max20328_write_reg(data, 0x09, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x09, recvData);
		goto err_read_reg;
	}

    recvData = 0x13;					//default mode
 	err = max20328_write_reg(data, 0x06, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x06, recvData);
		goto err_read_reg;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device;
	}

	data->input_dev = input_dev;
	input_dev->name = "max20328";
	input_set_drvdata(input_dev, data);
	input_set_capability(input_dev, EV_REL, REL_X);

	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(data->input_dev);
		goto err_input_register_device;
	}

	err = sysfs_create_group(&data->input_dev->dev.kobj,
				 &hrm_attribute_group);


	i2c_set_clientdata(client, data);
	dev_set_drvdata(&input_dev->dev, data);

	/* INIT VAR */
	max20328_initialize(data);
	#ifdef OPLUS_ARCH_EXTENDS
	data->hs_det_ready = false;
	#endif /*OPLUS_ARCH_EXTENDS*/
	#ifdef OPLUS_ARCH_EXTENDS
	data->mode = MAX20328_USB_MODE;
	#endif /*OPLUS_ARCH_EXTENDS*/
	max20328_usb_c_analog_init(data);

	data->max20328_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((data->max20328_notifier).rwsem);
	data->max20328_notifier.head = NULL;

	s_max20328 = data;

	return 0;

err_input_register_device:
err_input_allocate_device:
err_read_reg:
	mutex_destroy(&data->lock);
	mutex_destroy(&data->i2clock);
	mutex_destroy(&data->activelock);
err_setup_irq:
err_of_node:
	kfree(data);
exit:
	return err;
}

static int max20328_remove(struct i2c_client *client)
{
	int err = 0;
	struct max20328_data *max20328_data = i2c_get_clientdata(client);

	input_unregister_device(max20328_data->input_dev);
	mutex_destroy(&max20328_data->lock);
	kfree(max20328_data);
	s_max20328 = NULL;
		pr_err("%s: \n", __func__);
	return err;
}

//static int max20328_suspend(struct device *dev)
//{
	//struct max20328_data *data = dev_get_drvdata(dev);
	//u8 val = REG_CFG_CTL1_SW_AUDIO;
	//int err = 0;

//	err = max20328_write_reg(data, MAX20328_CTL1, val);         // make sure enable switch operation
//	return err;
//}

//static int max20328_resume(struct device *dev)
//{
//	struct max20328_data *data = dev_get_drvdata(dev);
//	u8 val = REG_CFG_CTL1_SW_AUDIO;
//	int err = 0;

//	err = max20328_write_reg(data, MAX20328_CTL1, val);         // make sure enable switch operation
//	return err;
//}

static const struct dev_pm_ops max20328_pm_ops = {
	.suspend = NULL,
	.resume = NULL
};

static const struct i2c_device_id max20328_id[] = {
	{ "max20328", 0 },
	{ }
};

static struct of_device_id max20328_match_table[] = {
	{ .compatible = "max20328",},
	{ },
};

MODULE_DEVICE_TABLE(i2c, max20328_id);

static struct i2c_driver max20328_driver = {
	.driver = {
		.name		= "max20328",
		.of_match_table = max20328_match_table,
	},
	.probe = max20328_probe,
	.remove = max20328_remove,
	.id_table = max20328_id,
};

static int __init max20328_init(void)
{
	return i2c_add_driver(&max20328_driver);
}

static void __exit max20328_exit(void)
{
	i2c_del_driver(&max20328_driver);
}

module_init(max20328_init);
module_exit(max20328_exit);

MODULE_AUTHOR("JY Kim <jy.kim@maximintegrated.com>");
MODULE_DESCRIPTION("max20328 Module driver");
MODULE_LICENSE("GPL");
