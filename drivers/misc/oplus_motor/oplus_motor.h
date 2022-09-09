/************************************************************************************
** Copyright (C), 2008-2018, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_motor.h
**
** Description:
**	Definitions for motor control layer.
**
** Version: 1.0
** Date created: 2018/01/14,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>		<date>		<author>		<desc>
**************************************************************************************/
#ifndef __OPLUS_MOTOR__H
#define __OPLUS_MOTOR__H

#include <linux/alarmtimer.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
#include <linux/wakelock.h>
#endif

#define MOTOR_TAG		   "[oplus_motor] "
#define MOTOR_ERR(fmt, args...)	   printk(KERN_ERR MOTOR_TAG" %s : "fmt, __FUNCTION__, ##args)
#define MOTOR_LOG(fmt, args...)	   printk(KERN_INFO MOTOR_TAG" %s : "fmt, __FUNCTION__, ##args)
#define MOTOR_DBG(fmt, args...)	   printk(KERN_DEBUG MOTOR_TAG" %s : "fmt, __FUNCTION__, ##args)

#define MOTOR_EVENT_TYPE		EV_KEY
#define MOTOR_EVENT_MANUAL_TO_UP	KEY_F13
#define MOTOR_EVENT_MANUAL_TO_DOWN	KEY_F14
#define MOTOR_EVENT_UP			KEY_F15
#define MOTOR_EVENT_UP_ABNORMAL		KEY_F16
#define MOTOR_EVENT_UP_NORMAL		KEY_F17
#define MOTOR_EVENT_DOWN		KEY_F18
#define MOTOR_EVENT_DOWN_ABNORMAL	KEY_F19
#define MOTOR_EVENT_DOWN_NORMAL		KEY_F20
#define DHALL_DETECT_RANGE_HIGH		(170)
#define DHALL_DETECT_RANGE_LOW		(-512)
#define MOTOR_RESET_TIMER		(500)/*500ms*/
#define MOTOR_STOP_DEFAULT_POS_VALUE		(3)
#define MOTOR_STOP_DEFAULT_NEG_VALUE		(-3)
#define MOTOR_STOP_COMPENSATE_VALUE		(30)
#define MOTOR_STOP_RETARD_VALUE		(430)
#define MOTOR_IRQ_MONITOR_TIME		(20)/*20ms*/
#define MOTOR_IRQ_MONITOR_COUNT		(8)
#define HALL_THRESH_HIGH			(511)
#define HALL_THRESH_LOW				(-512)
#define HALL_FULL_RANGE				(510)
#define HALL_STOP_NEG_DL			(-1)
#define HALL_STOP_POS_DL			(1)
#define STOP_IN_ADVANCE_RANG (50)
#define STOP_DELTA  (-20)
#define POSITIVE 1
#define NEGATIVE (-1)
#define RETRY_TIMES	(3)
#define MAX_BUF_LEN					(2048)

#define MOTOR_SHUTDOWN_TIMEOUT	(2*HZ)  /*2s*/

enum motor_speed {/*ns*/
	SPEED_100KHZ	= 10000, /*High*/
	SPEED_80KHZ		= 12000, /*High*/
	SPEED_64KHZ		= 15500,
	SPEED_60_8KHZ	= 16447,
	SPEED_57_6KHZ	= 17361,
	SPEED_54_4KHZ	= 18382,
	SPEED_51_2KHZ	= 19531,
	SPEED_38_4KHZ	= 26041,
	SPEED_32KHZ		= 31250,
	SPEED_25_6KHZ	= 39062,
	SPEED_22_4KHZ	= 44642,
	SPEED_19_2KHZ	= 52083,
	SPEED_16_6KHZ	= 60240,
	SPEED_12_8KHZ	= 78125,
	SPEED_9_6KHZ	= 104166,
	SPEED_6_4KHZ	= 156250,
	SPEED_3_2KHZ	= 312500,
	SPEED_1_6KHZ	= 625000/*LOW*/
};

enum motor_driver_id {
	UNKNOWN = 0,
	DRV8834,
	STSPIN220,
	LT3572
};

enum dhall_id {
	DHALL_0 = 0,
	DHALL_1,
	HALL_MAX
};

enum wakelock_id {
	MOTOR_RUN_LOCK = 0,
	HALL_DATA_LOCK,
	POSITION_DETECT_LOCK,
	MAX_LOCK
};

enum dhall_detection_mode {
	DETECTION_MODE_POLLING = 0,
	DETECTION_MODE_INTERRUPT,
	DETECTION_MODE_INVALID,
};

enum {
	MOTOR_POWER_OFF = 0,
	MOTOR_POWER_ON,
	MOTOR_POWER_IGNORE,
};

enum motor_direction {
	MOTOR_DOWN = 0,
	MOTOR_UPWARD,
};

enum motor_pull_push {
	MOTOR_PULL,
	MOTOR_PUSH,
};

enum motor_type {
	MOTOR_UNKNOWN = 0,
	MOTOR_FI5,
	MOTOR_FI6,
};

enum motor_move_state {
	MOTOR_STOP,/*never move after boot*/
	MOTOR_UPWARD_ING,
	MOTOR_DOWNWARD_ING,
	MOTOR_UPWARD_STOP,
	MOTOR_DOWNWARD_STOP
};

enum motor_position {
	UP_STATE,
	DOWN_STATE,
	MID_STATE,
};

enum motor_driver_mode {
	MOTOR_MODE_FULL = 0,
	MOTOR_MODE_1_2,
	MOTOR_MODE_1_4,
	MOTOR_MODE_1_8,
	MOTOR_MODE_1_16,
	MOTOR_MODE_1_32
};

enum motor_state {
	STATE_INVALID,
	STAY_ACTIVE,
	WAKE_ACTIVE,
	DEEP_SLEEP,
	STATE_MAX,
};

enum start_mode {
	STOP = 0,
	USER_START,
	ABNORMAL_START,
	IRQ_START,
};

enum boot_up_mode {
	MOTOR_NORMAL_MODE = 0,
	MOTOR_OTHER_MODE,
};

typedef struct {
	short irq_pos_hall0;
	short irq_pos_hall1;
	short irq_pos_hall0_bak;
	short irq_pos_hall1_bak;
	short irq_makeup_hall0;
	short irq_makeup_hall1;
	short up_retard_hall0;
	short up_retard_hall1;
	short down_retard_hall0;
	short down_retard_hall1;
} cali_data_t;

typedef struct {
	short data0;
	short data1;
} dhall_data_t;

struct dhall_max_data {
	short data0;
	short data1;
};

typedef struct {
	int l;
	int pwm;
	unsigned int speed;
} L_param_t;

struct noise_range {
	short high;
	short low;
};

struct noise_thresh {
	bool support;
	short pre_hall0;
	short pre_hall1;
	short irq_max;
	short irq_min;
	short normal_max;
	short normal_min;
	short hall0_weak;
	short hall1_weak;
	short average;
};

/*0:up 1:down*/
struct stop_range {
	short neg[2];
	short pos[2];
	short offset[2];
	short pos_change_thrd[2];
	short cali_retard_offset[2];    /*shows the diff of max data and retard position, should match with the engineermode config*/
	short hall_max_thrd[2];         /*diff threshold of the  start hall value and calibration max value*/
};

struct speed_param {
	L_param_t full;
	L_param_t speed_up;
	L_param_t nd;
	L_param_t speed_down;
	unsigned int delay;
	unsigned int up_brake_delay;
	unsigned int down_brake_delay;
};

struct pwm_param {
	enum motor_driver_mode md_mode;
	struct speed_param run;
	struct speed_param normal;
	struct speed_param slow;
	unsigned long	pwm_duty;
	unsigned long	pwm_period;
};

struct motor_info {
	const char *d_name;
	const char *m_name;
	enum motor_type type;
	enum motor_driver_id motor_ic;
};

#define MOTOR_IOCTL_BASE			(0x89)
#define MOTOR_IOCTL_START_MOTOR			_IOW(MOTOR_IOCTL_BASE, 0x00, int)
#define MOTOR_IOCTL_STOP_MOTOR			_IOW(MOTOR_IOCTL_BASE, 0x01, int)
#define MOTOR_IOCTL_MOTOR_UPWARD		_IOW(MOTOR_IOCTL_BASE, 0x02, int)
#define MOTOR_IOCTL_MOTOR_DOWNWARD		_IOW(MOTOR_IOCTL_BASE, 0x03, int)
#define MOTOR_IOCTL_GET_POSITION		_IOR(MOTOR_IOCTL_BASE, 0x04, int)
#define MOTOR_IOCTL_SET_DIRECTION		_IOW(MOTOR_IOCTL_BASE, 0x05, int)
#define MOTOR_IOCTL_SET_SPEED			_IOW(MOTOR_IOCTL_BASE, 0x06, int)
#define MOTOR_IOCTL_SET_DELAY			_IOW(MOTOR_IOCTL_BASE, 0x07, int)
#define MOTOR_IOCTL_GET_DHALL_DATA		_IOR(MOTOR_IOCTL_BASE, 0x08, dhall_data_t)
#define MOTOR_IOCTL_SET_CALIBRATION		_IOW(MOTOR_IOCTL_BASE, 0x09, cali_data_t)
#define MOTOR_IOCTL_GET_INTERRUPT		_IOR(MOTOR_IOCTL_BASE, 0x0A, int)

#define MOTOR_STOP_TIMEOUT (12000) /*80k*/


struct oplus_dhall_operations {
	int (*get_data)(unsigned int id, short *data);
	int (*set_detection_mode)(unsigned int id, u8 mode);
	int (*enable_irq)(unsigned int id, bool enable);
	int (*clear_irq)(unsigned int id);
	int (*get_irq_state)(unsigned int id);
	bool (*update_threshold)(unsigned int id, int position, short lowthd,
				 short highthd);
	void (*dump_regs)(unsigned int id, u8 *buf);
	int (*set_reg)(unsigned int id, int reg, int val);
	bool (*is_power_on)(unsigned int id);
	void (*set_sensitivity)(unsigned int id, char *value);
};

struct oplus_motor_operations {
	int (*set_power)(int mode);
	int (*set_direction)(int dir);
	int (*set_working_mode)(int mode);
	int (*get_all_config)(int *config , int count);
	int (*calculate_pwm_count)(int angle, int mode);
	int (*pwm_config)(int duty_ns, int period_ns, uint32_t wave_num);
	int (*pwm_enable)(void);
	int (*pwm_disable)(void);
	int (*pctrl_config)(bool active);
};

struct oplus_motor_chip {
	struct device *dev;
	struct pinctrl *pctrl;
	struct pinctrl_state *free_fall_state;
	struct input_dev *i_dev;
	struct workqueue_struct *manual2auto_wq;
	struct delayed_work	detect_work;
	struct work_struct	motor_work;
	struct work_struct	force_work;
	struct work_struct	manual_position_work;
	struct delayed_work	up_work;
	struct delayed_work	down_work;
	struct hrtimer stop_timer;
	struct hrtimer speed_up_timer;
	struct hrtimer speed_down_timer;
	struct hrtimer force_move_timer;
	struct alarm reset_timer;
	struct notifier_block fb_notify;
	struct oplus_dhall_operations *dhall_ops;
	struct oplus_motor_operations *motor_ops;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	struct wake_lock	suspend_lock;
#else
	struct wakeup_source	*suspend_ws;
#endif
	atomic_t	in_suspend;
	enum motor_position		position;
	struct motor_info info;
	struct stop_range stop;
	struct pwm_param pwm_param;
	struct noise_thresh noise;
	struct dhall_max_data hall_max;
	struct mutex power_mutex;
	cali_data_t cali_data;
	bool		shutting;
	bool		stop_timer_trigger;
	bool		hall_detect_switch;
	bool		is_motor_test;
	bool		is_skip_pos_check;
	bool	    irq_monitor_started;
	bool	    is_irq_abnormal;
	bool	    is_support_ffd;/*free fall detcet*/
	bool		is_in_calibration;
	bool		force_moving;
	bool		have_mode;
	bool		strong_mag;
	bool		extre_stop_support;
	bool		retard_sign;
	bool		stop_in_advance;
	bool		motor_stop_val_support;
	bool		motor_down_stop_support;
	bool		cal_fix_stop_retry_support;
	bool		motor_retard_offset_support;
	bool		motor_modify_stopcount_support;
	bool		motor_small_irq_later_speed_down_support;
	bool		down_state_high_thresh_support;
	int		motor_down_stop_val;
	int		stop_rang_a;
	int		stop_rang_b;
	int		sign;
	int		dir_sign;
	int		motor_started;
	int		motor_switch;
	int		md_dir;
	int		motor_enable;
	int		irq_count[HALL_MAX];
	int		move_state;
	int		manual2auto_up_switch;
	int		manual2auto_down_switch;
	int		irq;
	int		boot_mode;
	int		mono_offset;
	int		stop_retry;
	int		cal_stop_pos_val;
	int		cal_stop_neg_val;
	int		full_stop_pos_val;
	int		full_stop_neg_val;
	int		cal_fix_stop_retry;
	int		offset_stop_retry;
	int     small_irq;
	int     later_speed_down;
	int     later_speed_down_stop_offset;
	int		retard_range;
	int		retard_range_end;
	unsigned int	free_fall_gpio;
	uint8_t		*up_data_buf;
	uint8_t		*down_data_buf;
	short	run_check_hall0;
	short	run_check_hall1;
};

extern int oplus_register_dhall(const char *name,
			       struct oplus_dhall_operations *ops);
extern int oplus_unregister_dhall(const char *name);
extern int oplus_register_motor(const char *name,
			       struct oplus_motor_operations *ops);
extern int oplus_unregister_motor(const char *name);
/*dhall control api*/
extern int oplus_dhall_get_data(unsigned int id, short *hall_val, bool raw);
extern int oplus_dhall_set_detection_mode(unsigned int id, u8 mode);
extern int oplus_dhall_enable_irq(unsigned int id, bool enable);
extern int oplus_dhall_clear_irq(unsigned int id);
extern int oplus_dhall_irq_handler(unsigned int id);
extern int oplus_dhall_get_irq_state(unsigned int id);
extern void oplus_dhall_dump_regs(unsigned int id, u8 *buf);
extern int oplus_dhall_set_reg(unsigned int id, int reg, int val);
extern bool oplus_dhall_update_threshold(unsigned int id, int position,
					short thresh);
extern bool oplus_dhall_is_power_on(void);
/*motor control api*/
extern int oplus_motor_set_power(enum motor_state state, int mode);
extern int oplus_motor_set_direction(int dir);
extern int oplus_motor_set_working_mode(int mode);
extern int oplus_motor_calculate_pwm_count(int L, int mode);
extern int oplus_motor_pwm_config(int duty_ns, int period_ns, uint32_t wave_num);
extern int oplus_motor_pwm_enable(void);
extern int oplus_motor_pwm_disable(void);
extern int oplus_motor_get_all_config(int *config , int count);
extern int oplus_get_motor_type(void);
extern int oplus_get_driver_ic_id(void);
extern int oplus_get_dir_sign(void);
extern bool oplus_is_support_mode(void);
#endif /*__OPLUS_MOTOR__H*/
