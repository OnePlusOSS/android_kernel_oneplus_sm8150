/************************************************************************************
 ** File: - /android/kernel/drivers/input/touchscreen/synaptic_s3320.c
 ** Copyright (C), 2008-2012, OEM Mobile Comm Corp., Ltd
 **
 ** Description:
 **      touch panel driver for synaptics
 **      can change MAX_POINT_NUM value to support multipoint
 ** Version: 1.0
 ** Date created: 10:49:46,18/01/2012
 ** Author: Yixue.Ge@BasicDrv.TP
 **
 ** --------------------------- Revision History: --------------------------------
 ** 	<author>	<data>			<desc>
 **  morgan.gu@BSP.TP modified for oem 2017-10-30 s3706 tp_driver
 ************************************************************************************/
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>

#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

/*modify by morgan.gu for sdm845 */
#define CONFIG_MSM_RDM_NOTIFY
#undef CONFIG_FB

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#elif defined(CONFIG_MSM_RDM_NOTIFY)
#include <linux/msm_drm_notify.h>
#include <linux/notifier.h>
#endif

#include <linux/input/mt.h>

#include "synaptics_redremote.h"
#include <linux/project_info.h>
#include "synaptics_baseline.h"
#include "synaptics_dsx_core.h"
#include <linux/oneplus/boot_mode.h>
/*------------------------------------------------Global Define--------------------------------------------*/

#define TP_UNKNOWN 0
#define TP_G2Y 1
#define TP_TPK 2
#define TP_TRULY 3
#define TP_OFILM 4
#define TP_JDI_TPK 6
#define TP_TEST_ENABLE 1

#define DiagonalUpperLimit  1100
#define DiagonalLowerLimit  900

#define PAGESIZE 512
#define TPD_USE_EINT

#define TPD_DEVICE "synaptics,s3320"

//#define SUPPORT_SLEEP_POWEROFF
#define SUPPORT_GESTURE
#define RESET_ONESECOND
//#define SUPPORT_GLOVES_MODE
//#define REPORT_2D_PRESSURE
//#define SUPPORT_VIRTUAL_KEY


#define SUPPORT_TP_SLEEP_MODE
#define TYPE_B_PROTOCOL      //Multi-finger operation
#define TP_FW_NAME_MAX_LEN 128
#define SUPPORT_TP_TOUCHKEY

#define TEST_MAGIC1 0x494D494C
#define TEST_MAGIC2 0x474D4954

struct test_header {
	unsigned int magic1;
	unsigned int magic2;
	unsigned int withCBC;
	unsigned int array_limit_offset;
	unsigned int array_limit_size;
	unsigned int array_limitcbc_offset;
	unsigned int array_limitcbc_size;
};

struct fp_underscreen_info {
    uint8_t touch_state;
    uint8_t area_rate;
    uint16_t x;
    uint16_t y;
};

/******************for Red function*****************/
#define CONFIG_SYNAPTIC_RED

/*********************for gesture*******************/
#ifdef SUPPORT_GESTURE
#define ENABLE_UNICODE  0x40
#define ENABLE_VEE      0x20
#define ENABLE_CIRCLE   0x08
#define ENABLE_SWIPE    0x02
#define ENABLE_DTAP     0x01

#define UNICODE_DETECT  0x0b
#define VEE_DETECT      0x0a
#define CIRCLE_DETECT   0x08
#define SWIPE_DETECT    0x07
#define DTAP_DETECT     0x03

#define FINGER_DOWN     0x0f
#define FINGER_UP       0x1f
#define SINGLE_TAP      0x10

#define UnkownGestrue       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W
#define Sgestrue            14  // S
#define SingleTap           15  // single tap


//ruanbanmao@BSP add for tp gesture 2015-05-06, begin
#define BIT0 (0x1 << 0)
#define BIT1 (0x1 << 1)
#define BIT2 (0x1 << 2)
#define BIT3 (0x1 << 3)
#define BIT4 (0x1 << 4)
#define BIT5 (0x1 << 5)
#define BIT6 (0x1 << 6)
#define BIT7 (0x1 << 7)

int LeftVee_gesture = 0; //">"
int RightVee_gesture = 0; //"<"
int DouSwip_gesture = 0; // "||"
int Circle_gesture = 0; // "O"
int UpVee_gesture = 0; //"V"
int DownVee_gesture = 0; //"^"
int DouTap_gesture = 0; //"double tap"

int Left2RightSwip_gesture=0;//"(-->)"
int Right2LeftSwip_gesture=0;//"(<--)"
int Up2DownSwip_gesture =0;//"up to down |"
int Down2UpSwip_gesture =0;//"down to up |"

int Wgestrue_gesture =0;//"(W)"
int Mgestrue_gesture =0;//"(M)"
int Sgestrue_gesture =0;//"(S)"
int Single_gesture = 0; //"(SingleTap)"
int Enable_gesture =0;
static int gesture_switch = 0;
//ruanbanmao@BSP add for tp gesture 2015-05-06, end
#endif

/*********************for Debug LOG switch*******************/
#define TPD_ERR(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)
#define TPDTM_DMESG(a, arg...)  printk(TPD_DEVICE ": " a, ##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
		pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)

/*---------------------------------------------Global Variable----------------------------------------------*/
static int baseline_ret = 0;
static long int TP_FW;
static int tp_dev = 6;
unsigned int tp_debug;
static int button_map[3];
static int tx_rx_num[2];
static int16_t Rxdata[33][33];/*s3706 tx rx 16 33 s3508 tx rx 15 30*/
static int16_t delta_baseline[33][33];
static int16_t baseline[33][33];
static int16_t delta[33][33];
static int TX_NUM;
static int RX_NUM;
static int report_key_point_y = 0;
static int force_update = 0;
static int LCD_WIDTH ;
static int LCD_HEIGHT ;
static int get_tp_base = 0;
#define ENABLE_TPEDGE_LIMIT
#ifdef ENABLE_TPEDGE_LIMIT
static int F51_GRIP_CONFIGURATION;
static int limit_enable=1;
static void synaptics_tpedge_limitfunc(void);
#endif
//static int ch_getbase_status = 0;
struct timeval tpstart, tpend;
int not_getbase;
int need_reset;
int singer_touch_mun;
int recored_pointx[2] = {0, 0};
int recored_pointy[2] = {0, 0};

#ifdef SUPPORT_TP_SLEEP_MODE
static int sleep_enable;
#endif
#ifdef SUPPORT_TP_TOUCHKEY
static int key_switch = 0;
static bool key_back_disable=false,key_appselect_disable=false;
#endif
static struct synaptics_ts_data *ts_g = NULL;
static struct workqueue_struct *synaptics_wq = NULL;
static struct workqueue_struct *synaptics_report = NULL;
static struct workqueue_struct *get_base_report = NULL;
static struct proc_dir_entry *prEntry_tp = NULL;


#ifdef SUPPORT_GESTURE
static uint32_t clockwise;
static uint32_t gesture;

/****point position*****/
struct Coordinate {
	uint32_t x;
	uint32_t y;
};
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
#endif

/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F12_2D_QUERY_BASE;
static int F12_2D_CMD_BASE;
static int F12_2D_CTRL_BASE;
static int F12_2D_DATA_BASE;
static int F12_2D_DATA15;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_DATA01;
static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;
static int F01_RMI_CTRL02;

static int F12_2D_CTRL08;
static int F12_2D_CTRL32;
static int F12_2D_DATA04;
static int F12_2D_DATA38;
static int F12_2D_DATA39;
static int F12_2D_CMD00;
static int F12_2D_CTRL20;
static int F12_2D_CTRL27;

static int F34_FLASH_CTRL00;

static int F51_CUSTOM_CTRL00;
static int F51_CUSTOM_DATA04;
static int F51_CUSTOM_DATA11;
static int version_is_s3508=0;
#if TP_TEST_ENABLE
static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00
#endif

/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_i2c_suspend(struct device *dev);
static int synaptics_i2c_resume(struct device *dev);
/**************I2C resume && suspend end*********/
static void speedup_synaptics_resume(struct work_struct *work);
static int synaptics_ts_resume(struct device *dev);
static int synaptics_ts_suspend(struct device *dev);
static int synaptics_ts_remove(struct i2c_client *client);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static ssize_t synaptics_rmi4_baseline_show(struct device *dev, char *buf, bool savefile)	;
static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev, struct device_attribute *attr, char *buf);
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force);

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr);

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data);

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr);

static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
		unsigned char addr,unsigned short data);
static int synaptics_mode_change(int mode);
int tp_single_tap_en(struct synaptics_ts_data *ts, bool enable);
int opticalfp_irq_handler(struct fp_underscreen_info* tp_info);

#ifdef TPD_USE_EINT
static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id);
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_MSM_RDM_NOTIFY)
static int msm_drm_notifier_callback(
	struct notifier_block *self, unsigned long event, void *data);
#endif
static int synaptics_soft_reset(struct synaptics_ts_data *ts);
static void synaptics_hard_reset(struct synaptics_ts_data *ts);
static int set_changer_bit(struct synaptics_ts_data *ts);
static int tp_baseline_get(struct synaptics_ts_data *ts,bool flag);
static void set_doze_time(int doze_time);
static int synaptics_soft_reset(struct synaptics_ts_data *ts);
static int touch_hold_en(struct synaptics_ts_data *ts, bool enable);


/*-------------------------------Using Struct----------------------------------*/
struct point_info {
	unsigned char status;
	int x;
	int raw_x;
	int y;
	int raw_y;
	int z;
#ifdef REPORT_2D_PRESSURE
    unsigned char pressure;
#endif
};

static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct of_device_id synaptics_match_table[] = {
	{ .compatible = TPD_DEVICE,},
	{ },
};

static const struct dev_pm_ops synaptic_pm_ops = {
#ifdef CONFIG_PM
	.suspend = synaptics_i2c_suspend,
	.resume = synaptics_i2c_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
};

//add by jiachenghui for boot time optimize 2015-5-13
static int probe_ret;
struct synaptics_optimize_data{
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct i2c_client *client;
	const struct i2c_device_id *dev_id;
};
static struct synaptics_optimize_data optimize_data;
static void synaptics_ts_probe_func(struct work_struct *w)
{
	struct i2c_client *client_optimize = optimize_data.client;
	const struct i2c_device_id *dev_id = optimize_data.dev_id;
	TPD_ERR("after on cpu [%d]\n",smp_processor_id());
	probe_ret = synaptics_ts_probe(client_optimize,dev_id);
}

static int oem_synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	optimize_data.client = client;
	optimize_data.dev_id = id;
	optimize_data.workqueue = create_workqueue("tpd_probe_optimize");
	INIT_DELAYED_WORK(&(optimize_data.work), synaptics_ts_probe_func);
	TPD_ERR("before on cpu [%d]\n",smp_processor_id());

	//add by lifeng@bsp 2015-12-10 for only one cpu on line
	for (i = 0; i < NR_CPUS; i++){
         TPD_ERR("check CPU[%d] is [%s]\n",i,cpu_is_offline(i)?"offline":"online");
		 if (cpu_online(i) && (i != smp_processor_id()))
            break;
    }
    queue_delayed_work_on(i != NR_CPUS?i:0,optimize_data.workqueue,&(optimize_data.work),msecs_to_jiffies(300));
    //end add by lifeng@bsp 2015-12-10 for only one cpu on line

	return probe_ret;
}
//end add by jiachenghui for boot time optimize 2015-5-13

static struct i2c_driver tpd_i2c_driver = {
//add by jiachenghui for boot time optimize 2015-5-13
	.probe		= oem_synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
	.id_table	= synaptics_ts_id,
	.driver = {
		//		.owner  = THIS_MODULE,
		.name	= TPD_DEVICE,
		.of_match_table =  synaptics_match_table,
		.pm = &synaptic_pm_ops,
	},
};

struct synaptics_ts_data {
	struct i2c_client *client;
	struct mutex mutex;
	struct mutex mutexreport;
	int irq;
	int irq_gpio;
	atomic_t irq_enable;
	int id1_gpio;
	int id2_gpio;
	int id3_gpio;
	int reset_gpio;
	int v1p8_gpio;
	int support_hw_poweroff;
	int support_1080x2160_tp;
	int support_1080x2340_tp;
	int enable2v8_gpio;
	int max_num;
	int enable_remote;
	int regulator_vdd_vmin;
	int regulator_vdd_vmax;
	int regulator_vdd_current;
	int regulator_avdd_vmin;
	int regulator_avdd_vmax;
	int regulator_avdd_current;

	uint32_t irq_flags;
	uint32_t max_x;
	uint32_t max_y;
	uint32_t max_y_real;
	uint32_t btn_state;
	uint32_t pre_finger_state;
	uint32_t pre_btn_state;
	struct delayed_work  base_work;
	struct work_struct  report_work;
	struct delayed_work speed_up_work;
	struct input_dev *input_dev;
	struct hrtimer timer;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_MSM_RDM_NOTIFY)
	struct notifier_block msm_drm_notif;
#endif
	/******gesture*******/
	int gesture_enable;
	int in_gesture_mode;
	int glove_enable;
    int changer_connet;
	int is_suspended;
    atomic_t is_stop;
    spinlock_t lock;

	/********test*******/
	int i2c_device_test;

	/******power*******/
	struct regulator *vdd_2v8;
	struct regulator *vcc_i2c_1v8;

	/*pinctrl******/
	struct device						*dev;
	struct pinctrl 						*pinctrl;
	struct pinctrl_state 				*pinctrl_state_active;
	struct pinctrl_state 				*pinctrl_state_suspend;

	/*******for FW update*******/
	bool loading_fw;
	bool support_ft;/*support force touch*/
	char fw_name[TP_FW_NAME_MAX_LEN];
	char test_limit_name[TP_FW_NAME_MAX_LEN];
	char fw_id[20];
	char manu_name[30];
#ifdef SUPPORT_VIRTUAL_KEY
	struct kobject *properties_kobj;
#endif
	uint8_t fp_up_down;
	unsigned int en_up_down;
	unsigned int fp_aod_cnt;
	unsigned int unlock_succes;
	int project_version;
};

static struct device_attribute attrs_oem[] = {
	//	__ATTR(baseline_test, 0664, synaptics_rmi4_baseline_show, NULL),
	__ATTR(vendor_id, 0664, synaptics_rmi4_vendor_id_show, NULL),
};

static struct synaptics_rmi4_data *rmi4_data_s3706;

static ssize_t fp_irq_get(struct device *device,
			     struct device_attribute *attribute,
			     char *buf)
{
	struct synaptics_ts_data *ts = ts_g;

	return scnprintf(buf, PAGE_SIZE, "%i\n", ts->fp_up_down);
}
static DEVICE_ATTR(fp_irq, 0400, fp_irq_get, NULL);


static void touch_enable (struct synaptics_ts_data *ts)
{
    spin_lock(&ts->lock);
    if(0 == atomic_read(&ts->irq_enable))
    {
        if(ts->irq)
            enable_irq(ts->irq);
        atomic_set(&ts->irq_enable,1);
        //TPD_ERR("test %%%% enable irq\n");
    }
    spin_unlock(&ts->lock);
}

static void touch_disable(struct synaptics_ts_data *ts)
{
    spin_lock(&ts->lock);
    if(1 == atomic_read(&ts->irq_enable))
    {
        if(ts->irq)
            disable_irq_nosync(ts->irq);
        atomic_set(&ts->irq_enable,0);
        //TPD_ERR("test ****************** disable irq\n");
    }
    spin_unlock(&ts->lock);
}

static int tpd_hw_pwron(struct synaptics_ts_data *ts)
{
	int rc = 0;

	/***enable the 2v8 power*****/
	if (!IS_ERR(ts->vdd_2v8)) {
		//regulator_set_optimum_mode(ts->vdd_2v8,100000);
		rc = regulator_enable(ts->vdd_2v8);
		if(rc){
			dev_err(&ts->client->dev,
					"Regulator vdd enable failed rc=%d\n", rc);
			//return rc;
		}
	}
	/*add for mrogan for 3V delay 10ms ,1.8v delay 10ms*/
	usleep_range(10*1000, 10*1000);
	if( ts->v1p8_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the v1p8_gpio\n");
		gpio_direction_output(ts->v1p8_gpio, 1);
	}
	//msleep(100);

	if( ts->enable2v8_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 1);
	}
	/*usleep_range(10*1000, 10*1000);*/
	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		//regulator_set_optimum_mode(ts->vcc_i2c_1v8,100000);
		rc = regulator_enable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			//return rc;
		}
	}
	usleep_range(10*1000, 10*1000);
	if( ts->reset_gpio > 0 ) {
		gpio_direction_output(ts->reset_gpio, 1);
        usleep_range(10*1000, 10*1000);
		gpio_direction_output(ts->reset_gpio, 0);
        usleep_range(10*1000, 10*1000);
		gpio_direction_output(ts->reset_gpio, 1);
		TPD_DEBUG("synaptics:enable the reset_gpio\n");
	}
	return rc;
}

static int tpd_hw_pwroff(struct synaptics_ts_data *ts)
{
	int rc = 0;
	if( ts->reset_gpio > 0 ) {
		TPD_DEBUG("%s set reset gpio low\n",__func__);
		gpio_direction_output(ts->reset_gpio, 0);
	}

	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		rc = regulator_disable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}
	if( ts->v1p8_gpio > 0 ) {
		TPD_DEBUG("synaptics:disable the v1p8_gpio\n");
		gpio_direction_output(ts->v1p8_gpio, 0);
	}
	if (!IS_ERR(ts->vdd_2v8)) {
		rc = regulator_disable(ts->vdd_2v8);
		if (rc) {
			dev_err(&ts->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	}
	if( ts->enable2v8_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 0);
	}
	return rc;
}

static int tpd_power(struct synaptics_ts_data *ts, unsigned int on)
{
	int ret;
	if(on)
		ret = tpd_hw_pwron(ts);
	else
		ret = tpd_hw_pwroff(ts);

	return ret;
}

static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
	uint8_t buf[4];
	int ret;
	memset(buf, 0, sizeof(buf));
	ret = synaptics_rmi4_i2c_write_byte( ts->client, 0xff, 0x0 );
	if( ret < 0 ){
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xDD, 4, &(buf[0x0]));
	if( ret < 0 ){
		TPD_ERR("failed for page select!\n");
		return -1;
	}

	F12_2D_QUERY_BASE = buf[0];
	F12_2D_CMD_BASE = buf[1];
	F12_2D_CTRL_BASE = buf[2];
	F12_2D_DATA_BASE = buf[3];

	TPD_ERR("F12_2D_QUERY_BASE = %x \n \
			F12_2D_CMD_BASE  = %x \n\
			F12_2D_CTRL_BASE	= %x \n\
			F12_2D_DATA_BASE	= %x \n\
			",F12_2D_QUERY_BASE,F12_2D_CMD_BASE,F12_2D_CTRL_BASE,F12_2D_DATA_BASE);


	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE3, 4, &(buf[0x0]));
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2];
	F01_RMI_DATA_BASE = buf[3];
	TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
			F01_RMI_CMD_BASE  = %x \n\
			F01_RMI_CTRL_BASE	= %x \n\
			F01_RMI_DATA_BASE	= %x \n\
			", F01_RMI_QUERY_BASE, F01_RMI_CMD_BASE, F01_RMI_CTRL_BASE, F01_RMI_DATA_BASE);

	ret = synaptics_rmi4_i2c_read_block( ts->client, 0xE9, 4, &(buf[0x0]) );
	F34_FLASH_QUERY_BASE = buf[0];
	F34_FLASH_CMD_BASE = buf[1];
	F34_FLASH_CTRL_BASE = buf[2];
	F34_FLASH_DATA_BASE = buf[3];
	TPD_ERR("F34_FLASH_QUERY_BASE = %x \n\
			F34_FLASH_CMD_BASE	= %x \n\
			F34_FLASH_CTRL_BASE	= %x \n\
			F34_FLASH_DATA_BASE	= %x \n\
			", F34_FLASH_QUERY_BASE, F34_FLASH_CMD_BASE, F34_FLASH_CTRL_BASE, F34_FLASH_DATA_BASE);

	F01_RMI_QUERY11 = F01_RMI_QUERY_BASE+11;
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
	F01_RMI_CTRL02 = F01_RMI_CTRL_BASE + 2;
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_RMI_DATA01 = F01_RMI_DATA_BASE + 1;

	F12_2D_CTRL08 = F12_2D_CTRL_BASE;
	F12_2D_CTRL32 = F12_2D_CTRL_BASE + 15;
	F12_2D_DATA38 = F12_2D_DATA_BASE + 54;
	F12_2D_DATA39 = F12_2D_DATA_BASE + 55;
	F12_2D_CMD00 = F12_2D_CMD_BASE;
	if (version_is_s3508 == 2)/*s3706*/
		F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x06;
	else
		F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x07;
	F12_2D_CTRL27 = F12_2D_CTRL_BASE + 0x0c;


	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	if( ret < 0 ){
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2];
	F51_CUSTOM_DATA_BASE = buf[3];
	F51_CUSTOM_CTRL00 = F51_CUSTOM_CTRL_BASE;
	F51_CUSTOM_DATA04 = F51_CUSTOM_DATA_BASE;
	F51_CUSTOM_DATA11 = F51_CUSTOM_DATA_BASE;

	TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
			F51_CUSTOM_CMD_BASE  = %x \n\
			F51_CUSTOM_CTRL_BASE    = %x \n\
			F51_CUSTOM_DATA_BASE    = %x \n\
			", F51_CUSTOM_QUERY_BASE, F51_CUSTOM_CMD_BASE, F51_CUSTOM_CTRL_BASE, F51_CUSTOM_DATA_BASE);

#if TP_TEST_ENABLE
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x01);
	if(ret < 0) {
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	TPD_ERR("F54_QUERY_BASE = %x \n\
			F54_CMD_BASE  = %x \n\
			F54_CTRL_BASE	= %x \n\
			F54_DATA_BASE	= %x \n\
			", F54_ANALOG_QUERY_BASE, F54_ANALOG_COMMAND_BASE , F54_ANALOG_CONTROL_BASE, F54_ANALOG_DATA_BASE);
#endif
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	return 0;
}

#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	unsigned char reportbuf[4];
	//chenggang.li@BSP.TP modified for gesture
	TPD_DEBUG("%s is called\n", __func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ) {
		TPD_DEBUG("read reg F12_2D_CTRL20[0x%x] failed\n",F12_2D_CTRL20);
		return -1;
	}

	if( enable ) {
		ts->in_gesture_mode = 1;
		reportbuf[2] |= 0x02 ;
	} else {
		ts->in_gesture_mode = 0;
		reportbuf[2] &= 0xfd ;
	}
	TPD_DEBUG("F12_2D_CTRL20:0x%x=[2]:0x%x\n", F12_2D_CTRL20, reportbuf[2]);
	ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ){
		TPD_ERR("%s :Failed to write report buffer\n", __func__);
		return -1;
	}
	gesture = UnkownGestrue;
	return 0;
}
#endif

#ifdef SUPPORT_GLOVES_MODE
#define GLOVES_ADDR 0x001f //0x001D 0x001f
static int synaptics_glove_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	TPD_DEBUG("glove mode enable\n");
	/* page select = 0x4 */
	if( 1 == ts->glove_enable)  {
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data(ts->client, GLOVES_ADDR);
		//TPDTM_DMESG("enable glove  ret is %x ret|0x20 is %x\n", ret, ret|0x20);
		ret = i2c_smbus_write_byte_data(ts->client, GLOVES_ADDR, ret | 0x01);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	}else{
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data(ts->client, GLOVES_ADDR);
		ret = i2c_smbus_write_byte_data(ts->client, GLOVES_ADDR, ret & 0xFE);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	if( ret < 0 ){
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		goto GLOVE_ENABLE_END;
	}

GLOVE_ENABLE_END:
	return ret;
}
#endif

#ifdef SUPPORT_TP_SLEEP_MODE
static int synaptics_sleep_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	if( 1 == sleep_enable ){
		/*0x00:enable glove mode,0x02:disable glove mode,*/
		TPDTM_DMESG("sleep mode enable\n");
        ret = synaptics_mode_change(0x01);
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}else{
		TPDTM_DMESG("sleep mode disable\n");
        ret = synaptics_mode_change(0x84);
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}

SLEEP_ENABLE_END:
	return ret;
}
#endif

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;

	memset(buf1, 0 , sizeof(buf1));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY11, 8, &(buf1[0x0]));
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));
	if( ret < 0 ){
		TPD_ERR("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;

	TPD_DEBUG("%s is called!\n",__func__);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPD_ERR("init_panel failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/

    ret = synaptics_mode_change(0x80);//change tp to doze mode
	if( ret < 0 ){
		msleep(150);
        ret = synaptics_mode_change(0x80);
		if( ret < 0 ){
			TPD_ERR("%s failed for mode select\n",__func__);
		}
	}

	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t abs_status_int;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}
	if( enable ) {
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE+1);
		if( ret < 0 ) {
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL00+1, abs_status_int);
	if( ret < 0 ) {
		TPDTM_DMESG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n", __func__, abs_status_int);
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_CTRL00+1);
	return 0;
}

static void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;
	for(i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++) {
			udelay(1);
		}
	}
}
/*
static void int_state(struct synaptics_ts_data *ts)
{
	int ret = -1;
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
	if(ret){
		TPD_ERR("%s:error cannot reset touch panel!\n",__func__);
		return;
	}
	//delay_qt_ms(170);
	delay_qt_ms(100);
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	ret = synaptics_init_panel(ts);
	if( ret < 0 ){
		TPD_DEBUG("%s:error cannot change mode!\n",__func__);
		return;
	}
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret){
		TPD_DEBUG("%s:error cannot enable interrupt!\n",__func__);
		return;
	}
}
*/
/*Added for larger than 32 length read!*/

int synaptics_rmi4_i2c_read_block(
		struct i2c_client *client,
		unsigned char addr,
		unsigned short length,
		unsigned char *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};
	buf = addr & 0xFF;
	for( retry = 0; retry < 2; retry++ ) {
		if( i2c_transfer(client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if( retry == 2 ) {
		dev_err(&client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		//rst_flag_counter = 1;//reset tp
		retval = -5;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

int synaptics_rmi4_i2c_write_block(
		struct i2c_client *client,
		unsigned char addr,
		unsigned short length,
		unsigned char const *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = addr & 0xff;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < 2; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if (retry == 2) {
		//rst_flag_counter = 1;//rest tp
		retval = -EIO;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr)
{
	int retval = 0;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,1,buf);
	if(retval >= 0)
		retval = buf[0]&0xff;
	return retval;
}

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data)
{
	int retval;
	unsigned char data_send = data;
	retval = synaptics_rmi4_i2c_write_block(client,addr,1,&data_send);
	return retval;
}

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr)
{
	int retval;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,2,buf);
	if(retval >= 0)
		retval = buf[1]<<8|buf[0];
	return retval;
}

static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
		unsigned char addr,unsigned short data)
{
	int retval;
	unsigned char buf[2] = {data&0xff,(data>>8)&0xff};
	retval = synaptics_rmi4_i2c_write_block(client,addr,2,buf);
	if(retval >= 0)
		retval = buf[1]<<8|buf[0];
	return retval;
}

//chenggang.li@BSP.TP modified for oem 2014-08-05 gesture_judge
/***************start****************/
#ifdef SUPPORT_GESTURE
static void synaptics_get_coordinate_point(struct synaptics_ts_data *ts)
{
	int ret,i;
	uint8_t coordinate_buf[25] = {0};
	uint16_t trspoint = 0;
/* add by lifeng 2016/1/19 workarounds for the gestrue two interrupts begin*/
	static uint8_t coordinate_buf_last[25]= {0};
/* add by lifeng 2016/1/19 workarounds for the gestrue two interrupts end*/

	TPD_DEBUG("%s is called!\n",__func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	if (version_is_s3508 == 2) {
		ret = i2c_smbus_read_i2c_block_data(ts->client,
			F51_CUSTOM_DATA_BASE,
			sizeof(coordinate_buf),
			&(coordinate_buf[0]));
	} else if (version_is_s3508 == 1 || version_is_s3508 == 0) {
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11, 8, &(coordinate_buf[0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 8, 8, &(coordinate_buf[8]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 16, 8, &(coordinate_buf[16]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 24, 1, &(coordinate_buf[24]));
	}
/* add by lifeng 2016/1/19 workarounds for the gestrue two interrupts begin*/
	if (!memcmp(coordinate_buf_last, coordinate_buf,
		sizeof(coordinate_buf))) {
		TPD_ERR("%s reject the same gestrue[%d]\n", __func__, gesture);
		gesture = UnkownGestrue;
	}
	memcpy(coordinate_buf_last,coordinate_buf,sizeof(coordinate_buf));
/* strcpy(coordinate_buf_last, coordinate_buf, sizeof(coordinate_buf));
 * add by lifeng 2016/1/19 workarounds for the gestrue two interrupts end
 */

	for(i = 0; i< 23; i += 2) {
		trspoint = coordinate_buf[i]|coordinate_buf[i+1] << 8;
		TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n",i,trspoint);
	}

	TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n",coordinate_buf[24]);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8)) * LCD_WIDTH/ (ts->max_x);
	Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8)) * LCD_HEIGHT/ (ts->max_y);
	Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8)) * LCD_HEIGHT / (ts->max_y);
	clockwise     = (coordinate_buf[24] & 0x10) ? 1 :
		(coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
}

static int set_tp_info(struct synaptics_ts_data *ts, uint8_t up_down)
{
	int ret = 0;
	uint8_t tp_infor[8] = {0};
	struct fp_underscreen_info tp_info;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	ret = i2c_smbus_read_i2c_block_data(ts->client,
			F51_CUSTOM_DATA_BASE,
			sizeof(tp_infor),
			&(tp_infor[0]));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	TPD_DEBUG("%s:tp info %d %d %d %d\n", __func__,
			tp_infor[0]|tp_infor[1]<<8,tp_infor[2]|tp_infor[3]<<8,
			tp_infor[4]|tp_infor[5]<<8,tp_infor[6]|tp_infor[7]<<8);
	tp_info.x = tp_infor[1]<<8|tp_infor[0];
	tp_info.y = tp_infor[3]<<8|tp_infor[2];
	tp_info.area_rate = tp_infor[5]<<8|tp_infor[4];
	tp_info.touch_state = up_down;
	opticalfp_irq_handler(&tp_info);

	return ret;
}

static int recored_xy(struct synaptics_ts_data *ts, int x)
{
	int ret = 0;
	uint8_t tp_info[8] = {0};

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	ret = i2c_smbus_read_i2c_block_data(ts->client,
			F51_CUSTOM_DATA_BASE,
			sizeof(tp_info),
			&(tp_info[0]));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	TPD_DEBUG("%s:recored_xy %d %d %d %d\n", __func__,
			tp_info[0]|tp_info[1]<<8, tp_info[2]|tp_info[3]<<8,
			tp_info[4]|tp_info[5]<<8, tp_info[6]|tp_info[7]<<8);
	recored_pointx[x] = tp_info[1]<<8|tp_info[0];
	recored_pointy[x] = tp_info[3]<<8|tp_info[2];
	TPD_DEBUG("%s:recored_xy[%d] x= %d y= %d\n", __func__,
		x, recored_pointx[x], recored_pointy[x]);

	return ret;
}

static void fp_detect(struct synaptics_ts_data *ts)
{
	int ret = 0, gesture_sign;
	uint8_t gesture_buffer[10];

	ret = i2c_smbus_write_byte_data(
		ts->client, 0xff, 0x00);
	ret = i2c_smbus_read_i2c_block_data(
		ts->client,
		0x000A, 5,
		&(gesture_buffer[0]));

	gesture_sign = gesture_buffer[0];
	switch (gesture_sign) {
	case FINGER_DOWN:
		ts->fp_up_down = 1;
		sysfs_notify(&ts->dev->kobj, NULL,
			dev_attr_fp_irq.attr.name);
		TPD_DEBUG("%s:FINGER_DOWN %d\n", __func__,
			ts->fp_up_down);
		/*update tp info*/
		set_tp_info(ts, 1);
		need_reset = 0;
			break;
	case FINGER_UP:
		ts->fp_up_down = 0;
		sysfs_notify(&ts->dev->kobj, NULL,
			dev_attr_fp_irq.attr.name);
		TPD_DEBUG("%s:FINGER_UP %d\n", __func__,
			ts->fp_up_down);
		/*update tp info*/
		set_tp_info(ts, 0);
		if (ts->fp_aod_cnt > 0)
			need_reset = 1;
		ts->fp_aod_cnt = 0;
		break;
	}
	ret = i2c_smbus_write_byte_data(
		ts->client, 0xff, 0x00);
}

static int time_recored(bool start_time)
{
	int timeuse = 0;

	if (start_time) {
		do_gettimeofday(&tpstart);
	} else {
		do_gettimeofday(&tpend);
		timeuse = 1000000 * (tpend.tv_sec-tpstart.tv_sec)
				+ tpend.tv_usec-tpstart.tv_usec;
		TPD_DEBUG("Use time: %d uSeconds!!", timeuse);
	}
	if ((timeuse >= 800000) || (timeuse <= 0))
		return 0;

	return 1;
}

static int double_tap(struct synaptics_ts_data *ts)
{
	int time_use = 0;
	int ret = 0;

	if (singer_touch_mun == 0) {
		time_recored(true);
		recored_xy(ts, 0);
		singer_touch_mun = 1;
	} else {
		time_use = time_recored(false);
		recored_xy(ts, 1);
		singer_touch_mun = 0;
		if ((abs(recored_pointx[0] - recored_pointx[1]) < 100)
			&& (abs(recored_pointy[0] - recored_pointy[1]) < 120)
				&& (time_use)) {
			ret = 1;
		} else {
			time_recored(true);
			recored_xy(ts, 0);
			singer_touch_mun = 1;
			ret = -1;
		}
	}

	return ret;
}


static void gesture_judge(struct synaptics_ts_data *ts)
{
	unsigned int keyCode = KEY_F4;
	int ret = 0, regswipe = 0;
	uint8_t gesture_buffer[10];
	uint8_t gesture_sign = 0;
	unsigned char reportbuf[3];
	uint8_t regswipe_s3706[25];
	int is_double_tap = 0;

	if (version_is_s3508 == 1)
		F12_2D_DATA04 = 0x0008;
	else if (version_is_s3508 == 2) /*s3706*/
 		F12_2D_DATA04 = 0x000A;
	else
		F12_2D_DATA04 = 0x000A;
	TPD_DEBUG("%s start!\n",__func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0) {
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	}
	ret = i2c_smbus_write_byte_data(
		ts->client, 0xff, 0x00);
	ret = i2c_smbus_read_i2c_block_data(
		ts->client,
		F12_2D_DATA04, 5,
		&(gesture_buffer[0]));
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4);
	if (version_is_s3508 == 2) {
		ret = i2c_smbus_read_i2c_block_data(
		ts->client,  F51_CUSTOM_DATA_BASE,
		sizeof(regswipe_s3706), &(regswipe_s3706[0]));
		if (ret < 0)
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	} else if (version_is_s3508 == 1)
		regswipe = i2c_smbus_read_byte_data(ts->client,
				F51_CUSTOM_DATA04+0x18);
	else
		regswipe = i2c_smbus_read_byte_data(ts->client,
				F51_CUSTOM_DATA04+0x18);
	if (version_is_s3508 == 2) {
		TPD_DEBUG("GestureType[0x%x]=[0x%x]\n",
		F12_2D_DATA04, gesture_buffer[0]);
		TPD_DEBUG("lpwgSwipeID[0x4%x] = [0x%x]\n",
		F51_CUSTOM_DATA_BASE, regswipe_s3706[24]);
	} else if (version_is_s3508 == 1) {
		TPD_DEBUG("GestureType[0x%x]=[0x%x]\n",
		F12_2D_DATA04, gesture_buffer[0]);
		TPD_DEBUG("lpwgSwipeID[0x4%x] = [0x%x]\n",
		(F51_CUSTOM_DATA04+0x18), regswipe);
	} else
		TPD_DEBUG("Gesture Type[0x%x]=[0x%x],lpwg Swipe ID[0x4%x] = [0x%x]\n",\
		F12_2D_DATA04,gesture_buffer[0],(F51_CUSTOM_DATA04+0x18),regswipe);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	gesture_sign = gesture_buffer[0];

	if (ts->project_version == 0x03) {
		if (DouTap_gesture) {
			if (gesture_sign == SINGLE_TAP) {
				is_double_tap = double_tap(ts);
				if (is_double_tap == 1) {
					gesture_sign = DTAP_DETECT;
				}
			}
		}
	}
	//detect the gesture mode
	switch (gesture_sign) {
		case DTAP_DETECT:
			    gesture = DouTap;
			break;
		case SWIPE_DETECT:
			if (version_is_s3508 == 2) {
			gesture =
			(regswipe_s3706[24] == 0x41) ? Left2RightSwip :
			(regswipe_s3706[24] == 0x42) ? Right2LeftSwip :
			(regswipe_s3706[24] == 0x44) ? Up2DownSwip :
			(regswipe_s3706[24] == 0x48) ? Down2UpSwip :
			(regswipe_s3706[24] == 0x80) ? DouSwip  :
			UnkownGestrue;
				break;
			} else if (version_is_s3508 == 1) {
				gesture =   (regswipe == 0x41) ? Left2RightSwip   :
					(regswipe == 0x42) ? Right2LeftSwip   :
					(regswipe == 0x44) ? Up2DownSwip      :
					(regswipe == 0x48) ? Down2UpSwip      :
					(regswipe == 0x80) ? DouSwip          :
					UnkownGestrue;
				break;
			}else{
				gesture = (regswipe == 0x41) ? Left2RightSwip   :
					(regswipe == 0x42) ? Right2LeftSwip   :
					(regswipe == 0x44) ? Up2DownSwip      :
					(regswipe == 0x48) ? Down2UpSwip      :
					(regswipe == 0x84) ? DouSwip          :
					UnkownGestrue;
				break;
			}
		case CIRCLE_DETECT:
			    gesture = Circle;
			break;
		case VEE_DETECT:
			gesture = (gesture_buffer[2] == 0x01) ? DownVee  :
				(gesture_buffer[2] == 0x02) ? UpVee    :
				(gesture_buffer[2] == 0x04) ? RightVee :
				(gesture_buffer[2] == 0x08) ? LeftVee  :
				UnkownGestrue;
			break;
		case FINGER_DOWN:
			if (ts->project_version == 0x03) {
				ts->fp_up_down = 1;
				ts->fp_aod_cnt = 1;
				sysfs_notify(&ts->dev->kobj, NULL,
					dev_attr_fp_irq.attr.name);
				TPD_DEBUG("%s:FINGER_DOWN %d\n", __func__,
					ts->fp_up_down);
				/*update tp info*/
				set_tp_info(ts, 1);
				not_getbase = 1;
				need_reset = 0;
			}
			gesture = UnkownGestrue;
			break;
		case FINGER_UP:
			if (ts->project_version == 0x03) {
				ts->fp_up_down = 0;
				ts->fp_aod_cnt = 0;
				sysfs_notify(&ts->dev->kobj, NULL,
					dev_attr_fp_irq.attr.name);
				TPD_DEBUG("%s:FINGER_UP %d\n", __func__,
					ts->fp_up_down);
				/*update tp info*/
				set_tp_info(ts, 0);
			}
			gesture = UnkownGestrue;
			break;
		case SINGLE_TAP:
			if (ts->project_version == 0x03) {
				TPD_DEBUG("%s:SINGLE TAP\n", __func__);
				gesture = SingleTap;
			}
			break;
		case UNICODE_DETECT:
			gesture = (gesture_buffer[2] == 0x77) ? Wgestrue :
				(gesture_buffer[2] == 0x6d) ? Mgestrue :
				(gesture_buffer[2] == 0x73) ? Sgestrue :
				UnkownGestrue;
			break;
		default:
			gesture = UnkownGestrue;
			break;
		}

	TPD_ERR("detect %s gesture\n", gesture == DouTap ? "(double tap)" :
			gesture == UpVee ? "(V)" :
			gesture == DownVee ? "(^)" :
			gesture == LeftVee ? "(>)" :
			gesture == RightVee ? "(<)" :
			gesture == Circle ? "(O)" :
			gesture == DouSwip ? "(||)" :
			gesture == Left2RightSwip ? "(-->)" :
			gesture == Right2LeftSwip ? "(<--)" :
			gesture == Up2DownSwip ? "(up to down |)" :
			gesture == Down2UpSwip ? "(down to up |)" :
			gesture == Mgestrue ? "(M)" :
			gesture == Sgestrue ? "(S)" :
			gesture == Wgestrue ? "(W)" :
			gesture == SingleTap ? "(single tap)" : "[unknown]");
	if ((gesture != SingleTap) && (gesture != DouTap))
		synaptics_get_coordinate_point(ts);

    TPD_DEBUG("gesture suport LeftVee:%d RightVee:%d DouSwip:%d Circle:%d UpVee:%d DouTap:%d\n",\
        LeftVee_gesture,RightVee_gesture,DouSwip_gesture,Circle_gesture,UpVee_gesture,DouTap_gesture);
	if((gesture == DouTap && DouTap_gesture)||(gesture == RightVee && RightVee_gesture)\
		||(gesture == LeftVee && LeftVee_gesture)||(gesture == UpVee && UpVee_gesture)\
		||(gesture == Circle && Circle_gesture)||(gesture == DouSwip && DouSwip_gesture)\
		||(gesture == Sgestrue && Sgestrue_gesture)||(gesture == Wgestrue && Wgestrue_gesture)\
		||(gesture == Mgestrue && Mgestrue_gesture)||(gesture == SingleTap && Single_gesture)) {
		input_report_key(ts->input_dev, keyCode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keyCode, 0);
		input_sync(ts->input_dev);
	}else{
		mutex_lock(&ts->mutex);
		ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
		ret = reportbuf[2] & 0x20;
		if(ret == 0)
			reportbuf[2] |= 0x02 ;
		ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) ); //enable gesture
		if (ret < 0)
			TPD_ERR("%s :Failed to write report buffer\n", __func__);
		mutex_unlock(&ts->mutex);
	}
	TPD_DEBUG("%s end!\n", __func__);
}
#endif
/***************end****************/
static char prlog_count = 0;
#ifdef REPORT_2D_PRESSURE
static unsigned char pres_value = 1;
#endif
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
//extern struct completion key_cm;
bool key_back_pressed = 0;
bool key_appselect_pressed = 0;
bool key_home_pressed =0;
extern bool virtual_key_enable;
#endif

void int_touch(void)
{
	int ret = -1,i = 0;
	uint8_t buf[90];
	uint8_t count_data = 0;
	uint8_t object_attention[2];
	uint16_t total_status = 0;
	uint8_t finger_num = 0;
	uint8_t finger_status = 0;
	struct point_info points;
	uint32_t finger_info = 0;
	static uint8_t current_status = 0;
	uint8_t last_status = 0 ;
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
	bool key_appselect_check = false;
	bool key_back_check = false;
	bool key_home_check = false;
	bool key_pressed = key_appselect_pressed || key_back_pressed;// || key_home_pressed;
#endif
	struct synaptics_ts_data *ts = ts_g;

	memset(buf, 0, sizeof(buf));
	points.x = 0;
	points.y = 0;
	points.z = 0;
	points.status = 0;

	mutex_lock(&ts->mutexreport);
#ifdef REPORT_2D_PRESSURE
    if (ts->support_ft){
        ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4);
        ret = synaptics_rmi4_i2c_read_block(ts->client, 0x19,\
            sizeof(points.pressure), &points.pressure);
        if (ret < 0) {
            TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
            goto INT_TOUCH_END;
        }
        if (0 == points.pressure)//workaround for have no pressure value input reader into hover mode
        {
            pres_value++;
            if (255 == pres_value)
                pres_value = 1;
        }
        else
        {
            pres_value = points.pressure;
        }
    }
#endif
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if (version_is_s3508 == 1)
		F12_2D_DATA15 = 0x0009;
	else if (version_is_s3508 == 0)
		F12_2D_DATA15 = 0x000C;
	else if (version_is_s3508 == 2)
		F12_2D_DATA15 = 0x000B;/*s3706 for 17819*/
	ret = synaptics_rmi4_i2c_read_block(ts->client, F12_2D_DATA15, 2, object_attention);
    if (ret < 0) {
        TPD_ERR("synaptics_int_touch F12_2D_DATA15: i2c_transfer failed\n");
        goto INT_TOUCH_END;
    }
	total_status = (object_attention[1] << 8) | object_attention[0];

	if(total_status){
		while(total_status){
			count_data++;
			total_status >>= 1;
		}
	}else{
		count_data = 0;
	}
        if(count_data > 10){
            TPD_ERR("count_data is: %d\n", count_data);
            goto INT_TOUCH_END;
        }
	ret = synaptics_rmi4_i2c_read_block(ts->client, F12_2D_DATA_BASE, count_data*8+1, buf);
	if (ret < 0) {
		TPD_ERR("synaptics_int_touch F12_2D_DATA_BASE: i2c_transfer failed\n");
		goto INT_TOUCH_END;
	}
	for( i = 0; i < count_data; i++ ) {
		points.status = buf[i*8];
		points.x = ((buf[i*8+2]&0x0f)<<8) | (buf[i*8+1] & 0xff);
		points.raw_x = buf[i*8+6] & 0x0f;
		points.y = ((buf[i*8+4]&0x0f)<<8) | (buf[i*8+3] & 0xff);
		points.raw_y = buf[i*8+7] & 0x0f;
		points.z = buf[i*8+5];
		finger_info <<= 1;
		finger_status =  points.status & 0x03;
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
            if(virtual_key_enable){
                if(points.y > 0x780 && key_pressed){
                        TPD_DEBUG("Drop TP event due to key pressed\n");
                        finger_status = 0;
                }else{
                    finger_status =  points.status & 0x03;
                }
            }else{
                finger_status =  points.status & 0x03;
            }
            if(virtual_key_enable){
                    if (!finger_status){
                        if (key_appselect_pressed && !key_appselect_check){
                            points.x = 0xb4;
                            points.y = 0x7e2;
                            points.z = 0x33;
                            points.raw_x = 4;
                            points.raw_y = 6;
                            key_appselect_check = true;
                            points.status = 1;
                            finger_status =  points.status & 0x03;
                        }else if (key_back_pressed && !key_back_check){
                            points.x = 0x384;
                            points.y = 0x7e2;
                            points.z = 0x33;
                            points.raw_x = 4;
                            points.raw_y = 6;
                            key_back_check = true;
                            points.status = 1;
                            finger_status =  points.status & 0x03;
                        }else if(key_home_pressed && !key_home_check){
                            points.x = 0x21c;
                            points.y = 0x7e2;
                            points.z = 0x33;
                            points.raw_x = 4;
                            points.raw_y = 6;
                            key_home_check = true;
                            points.status = 1;
                            finger_status =  points.status & 0x03;
                    }else{
                            //TPD_DEBUG(" finger %d with !finger_statue and no key match\n",i);
                        }
                    }
            }
#endif
		if (version_is_s3508 == 0){//for 15811 panel
			points.x = 1079 - points.x;
			points.y = 1919 - points.y;
		}
		if (finger_status) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, finger_status);
			input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, points.x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, points.y);
			//#ifdef REPORT_2D_W
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, max(points.raw_x, points.raw_y));
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, min(points.raw_x, points.raw_y));
			//#endif
#ifdef REPORT_2D_PRESSURE
            if (ts->support_ft){
                input_report_abs(ts->input_dev,ABS_MT_PRESSURE,pres_value);
                TPD_DEBUG("%s: pressure%d[%d]\n",__func__,i,pres_value);
            }
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(ts->input_dev);
#endif
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
            if(virtual_key_enable){
              //  complete(&key_cm);
            }
#endif
			finger_num++;
			finger_info |= 1 ;
			//TPD_DEBUG("%s: Finger %d: status = 0x%02x "
					//"x = %4d, y = %4d, wx = %2d, wy = %2d\n",
					//__func__, i, points.status, points.x, points.y, points.raw_x, points.raw_y);

		}
	}

	finger_info <<= (ts->max_num - count_data);

	for ( i = 0; i < ts->max_num; i++ )
	{
		finger_status = (finger_info>>(ts->max_num-i-1)) & 1 ;
		if(!finger_status)
		{
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, finger_status);
		}

	}

	last_status = current_status & 0x02;

	if (ts->project_version == 0x03) {
		if (ts->en_up_down && ts->in_gesture_mode == 0)
			fp_detect(ts);
	}

	if (finger_num == 0/* && last_status && (check_key <= 1)*/) {
		if (3 == (++prlog_count % 6))
			TPD_ERR("all finger up\n");
		if (ts->project_version == 0x03) {
			if ((ts->unlock_succes == 1) && (need_reset ==1) && (ts->is_suspended == 0)) {
				TPD_DEBUG("touch hold reset %d\n", need_reset);
				need_reset = 0;
				ts->unlock_succes = 0;
				mutex_lock(&ts->mutex);
				ret = synaptics_rmi4_i2c_write_byte
					(ts->client, 0xff, 0x0);
				ret = i2c_smbus_write_byte_data(ts->client,
					F01_RMI_CMD_BASE, 0x01);//soft reset
				mutex_unlock(&ts->mutex);
				msleep(2);
			#ifdef ENABLE_TPEDGE_LIMIT
				synaptics_tpedge_limitfunc();
			#endif
			}
		}
		input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(ts->input_dev);
#endif
	}
	input_sync(ts->input_dev);

	if ((finger_num == 0) && (get_tp_base == 0)){//all finger up do get base once
		get_tp_base = 1;
		TPD_ERR("start get base data:%d\n",get_tp_base);
		if (!ts->en_up_down)
			tp_baseline_get(ts, false);
	}

#ifdef SUPPORT_GESTURE
	if (ts->in_gesture_mode == 1 && ts->is_suspended == 1) {
		gesture_judge(ts);
	}
#endif

INT_TOUCH_END:
	mutex_unlock(&ts->mutexreport);
}
static char log_count = 0;
#ifdef SUPPORT_TP_TOUCHKEY
#define OEM_KEY_BACK (key_switch?KEY_APPSELECT:KEY_BACK)
#define OEM_KEY_APPSELECT (key_switch?KEY_BACK:KEY_APPSELECT)
#else
#define OEM_KEY_BACK KEY_BACK
#define OEM_KEY_APPSELECT KEY_APPSELECT
#endif
static void int_key_report_s3508(struct synaptics_ts_data *ts)
{
    	int ret= 0;
	int F1A_0D_DATA00=0x00;
	int button_key;

	if (ts->is_suspended == 1)
		return;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x02 );
	if (ret < 0) {
		TPD_ERR("%s: line[%d]Failed to change page!!\n",__func__,__LINE__);
		return;
	 }
	button_key = synaptics_rmi4_i2c_read_byte(ts->client,F1A_0D_DATA00);
	if (1 == (++log_count % 4))
		TPD_ERR("touch_key[0x%x],touchkey_state[0x%x]\n",button_key,ts->pre_btn_state);
	if((button_key & 0x01) && !(ts->pre_btn_state & 0x01) && !key_back_disable)//back
	{
		input_report_key(ts->input_dev, OEM_KEY_BACK, 1);
		input_sync(ts->input_dev);
	}else if(!(button_key & 0x01) && (ts->pre_btn_state & 0x01) && !key_back_disable){
		input_report_key(ts->input_dev, OEM_KEY_BACK, 0);
		input_sync(ts->input_dev);
	}

	if((button_key & 0x02) && !(ts->pre_btn_state & 0x02) && !key_appselect_disable)//menu
	{
		input_report_key(ts->input_dev, OEM_KEY_APPSELECT, 1);
		input_sync(ts->input_dev);
	}else if(!(button_key & 0x02) && (ts->pre_btn_state & 0x02) && !key_appselect_disable){
		input_report_key(ts->input_dev, OEM_KEY_APPSELECT, 0);
		input_sync(ts->input_dev);
	}

	ts->pre_btn_state = button_key & 0x07;
	//input_sync(ts->input_dev);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0) {
		TPD_ERR("%s: line[%d]Failed to change page!!\n",__func__,__LINE__);
		return;
	}
	return;
}

static int synaptics_rmi4_free_fingers(struct synaptics_ts_data *ts)
{
	unsigned char i;

#ifdef TYPE_B_PROTOCOL
	for (i = 0; i < ts->max_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	return 0;
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret,status_check;
	uint8_t status = 0;
	uint8_t inte = 0;

    	struct synaptics_ts_data *ts = ts_g;

	if (atomic_read(&ts->is_stop) == 1)
	{
		touch_disable(ts);
		return;
	}

	if( ts->enable_remote) {
		goto END;
	}
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00 );
	ret = synaptics_rmi4_i2c_read_word(ts->client, F01_RMI_DATA_BASE);

	if( ret < 0 ) {
		TPDTM_DMESG("Synaptic:ret = %d\n", ret);
		synaptics_hard_reset(ts);
		if (ts->is_suspended == 1 && ts->gesture_enable == 0) {
			touch_disable(ts);
			goto EXIT;
		}
		goto END;
	}
	status = ret & 0xff;
	inte = (ret & 0x7f00)>>8;
	//TPD_ERR("%s status[0x%x],inte[0x%x]\n",__func__,status,inte);
        if(status & 0x80){
		TPD_DEBUG("enter reset tp status,and ts->in_gesture_mode is:%d\n",ts->in_gesture_mode);
		status_check = synaptics_init_panel(ts);
		if (status_check < 0) {
			TPD_ERR("synaptics_init_panel failed\n");
		}
		if ((ts->is_suspended == 1) && (ts->gesture_enable == 1)){
			synaptics_enable_interrupt_for_gesture(ts, 1);
			if (ts->project_version == 0x03) {
				mutex_lock(&ts->mutex);
				tp_single_tap_en(ts, true);
				if (ts->en_up_down)
					touch_hold_en(ts, true);
				mutex_unlock(&ts->mutex);
			}
		}
	}
/*
	if(0 != status && 1 != status) {//0:no error;1: after hard reset;the two state don't need soft reset
        TPD_ERR("%s status[0x%x],inte[0x%x]\n",__func__,status,inte);
		int_state(ts);
		goto END;
	}
*/
	if (inte == 1) {
		TPD_ERR("%s: spontaneous reset detected\n", __func__);
		ret = synaptics_rmi4_free_fingers(ts);
		if (ret < 0)
			TPD_ERR("%s: Failed to reinit device\n", __func__);
	}

	if( inte & 0x04 ) {

		int_touch();
	}
	if( inte & 0x10 ){
		int_key_report_s3508(ts);
	}


END:
	//ret = set_changer_bit(ts);
	touch_enable(ts);
EXIT:
	return;
}

#ifndef TPD_USE_EINT
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	mutex_lock(&ts->mutex);
	synaptics_ts_work_func(ts);
	mutex_unlock(&ts->mutex);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)dev_id;
    touch_disable(ts);
	synaptics_ts_work_func(&ts->report_work);
	return IRQ_HANDLED;
}
#endif

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static ssize_t tp_baseline_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return baseline_ret;
	if(baseline_ret == 0){
		count = synaptics_rmi4_baseline_show(ts->dev,page,1);
		baseline_ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	}else{
		baseline_ret = 0;
	}
	return baseline_ret;
}
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

static ssize_t i2c_device_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts_g)
		return ret;
	TPD_DEBUG("gesture enable is: %d\n", ts->gesture_enable);
	ret = sprintf(page, "%d\n", ts->i2c_device_test);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

#ifdef SUPPORT_GESTURE
static ssize_t tp_gesture_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_DEBUG("gesture enable is: %d\n", ts->gesture_enable);
	ret = sprintf(page, "%d\n", ts->gesture_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_gesture_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[10];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;
	if( count > 3 || ts->is_suspended)
		return count;
	if( copy_from_user(buf, buffer, count) ){
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	//ruanbanmao@BSP add for tp gesture 2015-05-06, begin
	TPD_ERR("%s write argc1[0x%x],argc2[0x%x]\n",__func__,buf[0],buf[1]);

	UpVee_gesture = (buf[0] & BIT0)?1:0; //"V"
	DouSwip_gesture = (buf[0] & BIT1)?1:0;//"||"
	LeftVee_gesture = (buf[0] & BIT3)?1:0; //">"
	RightVee_gesture = (buf[0] & BIT4)?1:0;//"<"
	Circle_gesture = (buf[0] & BIT6)?1:0; //"O"
	DouTap_gesture = (buf[0] & BIT7)?1:0; //double tap

	Sgestrue_gesture = (buf[1] & BIT0)?1:0;//"S"
	Mgestrue_gesture = (buf[1] & BIT1)?1:0; //"M"
	Wgestrue_gesture = (buf[1] & BIT2)?1:0; //"W"
	Single_gesture = (buf[1] & BIT3)?1:0;   //"Single_gesture"
	//enable gesture
	Enable_gesture = (buf[1] & BIT7)?1:0;

	if (DouTap_gesture || Circle_gesture || UpVee_gesture
		|| LeftVee_gesture || RightVee_gesture || DouSwip_gesture
		|| Sgestrue_gesture || Mgestrue_gesture || Wgestrue_gesture
		|| Enable_gesture || Single_gesture) {
		ts->gesture_enable = 1;
	}
	else
	{
		ts->gesture_enable = 0;
	}
    //ruanbanmao@BSP add for tp gesture 2015-05-06, end
	return count;
}
static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_ERR("%s:gesture_upload = %d \n", __func__, gesture);
	ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n",
		gesture, Point_start.x, Point_start.y, Point_end.x, Point_end.y,
		Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
		Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
		clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t gesture_switch_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	ret = sprintf(page, "gesture_switch:%d\n", gesture_switch);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t gesture_switch_write_func(struct file *file, const char __user *page, size_t count, loff_t *ppos)
{
	int ret,write_flag=0;
	char buf[10]={0};
	struct synaptics_ts_data *ts = ts_g;

	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!!\n",__func__);
		return count;
	}
	if( copy_from_user(buf, page, count) ){
		TPD_ERR("%s: read proc input error.\n", __func__);
		return count;
	}
	mutex_lock(&ts->mutex);
	ret = sscanf(buf,"%d",&write_flag);
	gesture_switch = write_flag;
	TPD_ERR("gesture_switch:%d,suspend:%d,gesture:%d\n",gesture_switch,ts->is_suspended,ts->gesture_enable);
	if (1 == gesture_switch){
		if ((ts->is_suspended == 1) && (ts->gesture_enable == 1)){
			i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
			synaptics_mode_change(0x80);
			//synaptics_enable_interrupt_for_gesture(ts, 1);
			//change active mode no need to write gesture mode.
			touch_enable(ts);
		}
	}else if(2 == gesture_switch){
		if ((ts->is_suspended == 1) && (ts->gesture_enable == 1)){
			i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
			synaptics_mode_change(0x81);
			touch_disable(ts);
			//synaptics_enable_interrupt_for_gesture(ts, 0);
			//change slepp mode no need to write gesture mode.
		}
	}
	mutex_unlock(&ts->mutex);

	return count;
}

// chenggang.li@BSP.TP modified for oem 2014-08-08 create node
/******************************start****************************/
static const struct file_operations tp_gesture_proc_fops = {
	.write = tp_gesture_write_func,
	.read =  tp_gesture_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations gesture_switch_proc_fops = {
	.write = gesture_switch_write_func,
	.read =  gesture_switch_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif
static int page ,address,block;
static ssize_t synap_read_address(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret;
	char buffer[PAGESIZE];
	char buf[128];
	int i;
	int cnt = 0;

	struct synaptics_ts_data *ts = ts_g;
	TPD_DEBUG("%s page=0x%x,address=0x%x,block=0x%x\n",__func__,page,address,block);
	cnt += sprintf(&(buffer[cnt]), "page=0x%x,address=0x%x,block=0x%x\n",page,address,block);
	ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,page);
	ret = synaptics_rmi4_i2c_read_block(ts->client,address,block,buf);
	for (i=0;i < block;i++)
	{
		cnt += sprintf(&(buffer[cnt]), "buf[%d]=0x%x\n",i,buf[i]);
		TPD_DEBUG("buffer[%d]=0x%x\n",i,buffer[i]);
	}
	ret = simple_read_from_buffer(user_buf, count, ppos, buffer, strlen(buffer));
	return ret;
}

static ssize_t synap_write_address(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int buf[128];
	char buffer_local[128];
	int ret, i;
	struct synaptics_ts_data *ts = ts_g;
	int temp_block, wbyte;
	char reg[30];

	if (count > 128)
		return count;
	if (copy_from_user(buffer_local, buffer, count)) {
		TPD_ERR("%s: write proc error.\n", __func__);
		return count;
	}

	ret = sscanf(buffer_local,
	"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
	&buf[0], &buf[1], &buf[2], &buf[3], &buf[4],
	&buf[5], &buf[6], &buf[7], &buf[8], &buf[9],
	&buf[10], &buf[11], &buf[12], &buf[13], &buf[14],
	&buf[15], &buf[16], &buf[17]);
    for (i = 0;i < ret;i++)
    {
        TPD_DEBUG("buf[i]=0x%x,",buf[i]);
    }
    TPD_DEBUG("\n");
    page= buf[0];
    address = buf[1];
    temp_block = buf[2];
    wbyte = buf[3];
    if (0xFF == temp_block)//the  mark is to write register else read register
    {
        for (i=0;i < wbyte;i++)
        {
            reg[i] = (char)buf[4+i];
        }
        ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,page);
        ret = synaptics_rmi4_i2c_write_block(ts->client,(char)address,wbyte,reg);
        TPD_DEBUG("%s write page=0x%x,address=0x%x\n",__func__,page,address);
        for (i=0;i < wbyte;i++)
        {
            TPD_DEBUG("reg=0x%x\n",reg[i]);
        }
    }
    else
        block = temp_block;
	return count;
}

#ifdef SUPPORT_GLOVES_MODE
static ssize_t tp_glove_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_DEBUG("glove mode enable is: %d\n", ts->glove_enable);
	ret = sprintf(page, "%d\n", ts->glove_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_glove_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts= ts_g;
	int ret = 0 ;
	char buf[10]={0};

	if( count > 10 )
		goto GLOVE_ENABLE_END;
	if( copy_from_user( buf, buffer, count) ){
		TPD_ERR("%s: read proc input error.\n", __func__);
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf, "%d", &ret);
	if(!ts)
		return count;
	TPDTM_DMESG("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){
		ts->glove_enable = ret;
		synaptics_glove_mode_enable(ts);
	}
	switch(ret){
		case 0:
			TPDTM_DMESG("tp_glove_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_glove_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
	return count;
}
#endif


#ifdef SUPPORT_TP_SLEEP_MODE
static ssize_t tp_sleep_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_DEBUG("sleep mode enable is: %d\n", sleep_enable);
	ret = sprintf(page, "%d\n", sleep_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_sleep_write_func(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	char buf[10]={0};
	struct synaptics_ts_data *ts = ts_g;
	int ret = 0 ;
	if( count > 10 )
		return count;
	if(!ts)
		return count;
	if( copy_from_user( buf, buffer, count) ) {
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &ret);
	TPDTM_DMESG("tp_sleep_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ) {
		sleep_enable = ret;
		synaptics_sleep_mode_enable(ts);
	}
	switch(ret) {
		case 0:
			TPDTM_DMESG("tp_sleep_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_sleep_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}
#endif

static ssize_t tp_show(struct device_driver *ddri, char *buf)
{
	// uint8_t ret = 0;
	struct synaptics_ts_data *ts = ts_g;
	int a ;
	int b,c;
	if(!ts)
		return 0;
	a = synaptics_rmi4_i2c_read_word(ts->client, F01_RMI_DATA_BASE);
	if( a < 0 )
		TPD_ERR("tp_show read i2c err\n");
	b = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA01);
	if( b < 0 )
		TPD_ERR("tp_show read i2c err\n");
	c = synaptics_rmi4_i2c_read_byte(ts->client, F12_2D_DATA_BASE);
	if( c < 0 )
		TPD_ERR("tp_show read i2c err\n");

	return sprintf(buf, "F01_RMI_DATA_BASE[0x%x]=0x%x;F01_RMI_DATA01[0x%x]=0x%x;F12_2D_DATA_BASE[0x%x]=0x%x;\n", \
			F01_RMI_DATA_BASE,a,F01_RMI_DATA01,b,F12_2D_DATA_BASE,c);
}

static ssize_t store_tp(struct device_driver *ddri, const char *buf, size_t count)
{
	int tmp = 0;
	if( 1 == sscanf(buf, "%d", &tmp) ){
		tp_debug = tmp;
	} else {
		TPDTM_DMESG("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}
static ssize_t vendor_id_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4];
	ret = sprintf(page, "%d\n",7);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

#if TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{
	unsigned char buf[4];
	int ret;
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if( ret < 0 ) {
		TPD_ERR("synaptics_rmi4_i2c_write_byte failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];

	TPD_ERR("F54_ANALOG_QUERY_BASE   = 0x%x \n \
			F54_ANALOG_COMMAND_BASE  = 0x%x \n\
			F54_ANALOG_CONTROL_BASE	 = 0x%x \n\
			F54_ANALOG_DATA_BASE	 = 0x%x \n\
			",F54_ANALOG_QUERY_BASE,F54_ANALOG_COMMAND_BASE,F54_ANALOG_CONTROL_BASE,F54_ANALOG_DATA_BASE);
	return 0;
}

static void checkCMD(int delay_time)
{
	int ret;
	int flag_err = 0;
	struct synaptics_ts_data *ts = ts_g;
	do {
		delay_qt_ms(delay_time); /*wait delay_time ms*/
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_COMMAND_BASE);
		flag_err++;
	} while ((ret > 0x00) && (flag_err < 60));
	if (ret > 0x00 || flag_err >= 60)
		TPD_ERR("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);
}

static void checkCMD_RT133(void)
{
	int ret = 0;
	int err_count = 0;
	struct synaptics_ts_data *ts = ts_g;
	do {
		if (version_is_s3508 == 2)
			delay_qt_ms(80);
		else
			delay_qt_ms(10);
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_COMMAND_BASE);
		err_count++;
	}while( (ret & 0x01) && (err_count < 30) );
	if( ret & 0x01 || err_count >= 30)
		TPD_ERR("%s line%d %x count %d\n", __func__, __LINE__, ret, err_count);
}

#endif

static ssize_t tp_baseline_show(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint16_t tmp_old = 0;
	uint16_t tmp_new = 0;
	uint16_t count = 0;
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
	/*disable irq when read data from IC*/
	touch_disable(ts);
	mutex_lock(&ts->mutex);
	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);

	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");
	//msm_cpuidle_set_sleep_disable(true);
	/*step 1:check raw capacitance*/
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	if( ret < 0 ){
		TPD_ERR("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+20, 0x01);

	if (version_is_s3508 != 2) {
	ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+23);
	tmp_old = ret & 0xff;
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+23, (tmp_old & 0xef));
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
	tmp_new = ret & 0xdf;
	i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_CONTROL_BASE+27, tmp_new);
	} else if (version_is_s3508 == 2) {
	/* s3706 no nedd 0D CBC Settings */
	ret = i2c_smbus_write_word_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0x04);
	ret = i2c_smbus_read_byte_data(ts->client,
		F54_ANALOG_CONTROL_BASE+23);/* Analog Control 1 */
	tmp_new = ret & 0xdf;
	i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_CONTROL_BASE+23, tmp_new);
	}

	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation

	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD(10);
	//TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD(10);
	//TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD(10);
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
		//printk("\n[%d]", x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h << 8)|tmp_l;
			//printk("%d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%5d", delta_baseline[x][y]);
		}
	}
	num_read_chars += sprintf(&(buf[num_read_chars]), "\n");
	TPD_DEBUG("\nread all is oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);
	delay_qt_ms(60);
	//msm_cpuidle_set_sleep_disable(false);
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);
//modify by zhouwenping for solve cat tp_baseline_image node cause touch disable 20160225 start
	synaptics_enable_interrupt(ts,1);
	ret = synaptics_soft_reset(ts);
	if (ret < 0){
           TPD_ERR("%s faile to reset device\n",__func__);
        }
	mutex_unlock(&ts->mutex);
//modify by zhouwenping 20160225 end
	return num_read_chars;

}

static ssize_t tp_rawdata_show(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0, tmp_h = 0;
	uint16_t count = 0;
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return 0;
	memset(delta_baseline, 0, sizeof(delta_baseline));
	/*disable irq when read data from IC*/
	touch_disable(ts);
	mutex_lock(&ts->mutex);
	synaptics_read_register_map_page1(ts);

	//TPD_DEBUG("\nstep 2:report type2 delta image\n");
	memset(delta_baseline, 0, sizeof(delta_baseline));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_DATA_BASE, 0x02);//select report type 0x02
	ret = synaptics_rmi4_i2c_write_word(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_COMMAND_BASE, 0X01);//get report
	checkCMD(10);
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
		//printk("\n[%d]", x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			//printk("%3d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%3d ", delta_baseline[x][y]);
		}
	}
	num_read_chars += sprintf(&(buf[num_read_chars]), "\n");
	ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_COMMAND_BASE,0X02);
	delay_qt_ms(60);
	synaptics_enable_interrupt(ts, 1);
	mutex_unlock(&ts->mutex);
	touch_enable(ts);
	return num_read_chars;
}

static ssize_t tp_delta_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	TPDTM_DMESG("tp_test_store is not support\n");
	return count;
}

static ssize_t synaptics_rmi4_baseline_show_s3508(struct device *dev, char *buf, bool savefile)
{

	ssize_t num_read_chars = 0;
#if TP_TEST_ENABLE
	int ret = 0;
	uint8_t x,y;
	int tx_datal;
	int16_t err_RT251 = 0, err_RT251_self = 0, err_RT253 = 0;
	int16_t baseline_data = 0;
	uint16_t unsigned_baseline_data = 0;
	uint8_t tmp_old = 0;
	uint8_t	tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint16_t count = 0;
	int error_count = 0;
	uint8_t buffer[9]={0};
	int16_t *baseline_data_test;
	int enable_cbc = 0;
	int readdata_fail=0,first_check=0;
	int16_t left_ramdata=0,right_ramdata=0;
	int fd = -1;
	struct timespec   now_time;
	struct rtc_time   rtc_now_time;
	uint8_t  data_buf[64];
	mm_segment_t old_fs;
    uint32_t CURRENT_FIRMWARE_ID = 0 ;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	/*
	   const struct firmware *fw = NULL;
	   struct test_header *ph = NULL;
	   ret = request_firmware(&fw, ts->test_limit_name, dev);
	   if (ret < 0) {
	   TPD_ERR("Request firmware failed - %s (%d)\n",ts->test_limit_name, ret);
	   error_count++;
	   num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);
	   num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
	   return num_read_chars;
	   }

	//ph = (struct test_header *)(fw->data);
	 */

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
     synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL00, 4, buf);
     CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
     TPD_ERR("[sk]CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);
	sprintf(ts->fw_id,"0x%x",CURRENT_FIRMWARE_ID);
	//push_component_info(TP, ts->fw_id, ts->manu_name);
READDATA_AGAIN:
	msleep(30);
	mutex_lock(&ts->mutex);
	touch_disable(ts);

	memset(Rxdata, 0, sizeof(Rxdata));
	synaptics_read_register_map_page1(ts);
	//TPDTM_DMESG("step 1:select report type 0x03\n");

	if(savefile) {
		getnstimeofday(&now_time);
		rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
		sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv",
				(rtc_now_time.tm_year+1900)%100, rtc_now_time.tm_mon+1, rtc_now_time.tm_mday,
				rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
		if (fd < 0) {
			TPD_ERR("Open log file '%s' failed.\n", data_buf);
			set_fs(old_fs);
		}
	}

	//step 1:check raw capacitance.
TEST_WITH_CBC_s3508:
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	if( ret < 0 ){
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed \n");
		goto END;
	}
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+20, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23);
	tmp_old = ret&0xff;

	if(enable_cbc){
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23,(tmp_old | 0x10));
		ret = i2c_smbus_write_word_data(ts->client,F54_ANALOG_COMMAND_BASE,0x04);
		checkCMD(30);
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
		tmp_new = ret | 0x20;
		i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
		TPD_DEBUG("Test open cbc\n");
		if(CURRENT_FIRMWARE_ID == 0xAB056006)
			baseline_data_test = (int16_t *)baseline_cap_data_old[0];
		else {
			if (ts->support_1080x2160_tp)
				baseline_data_test = (int16_t *)baseline_cap_17801_data[0];
			else
				baseline_data_test = (int16_t *)baseline_cap_data[0];
		}
	}else{
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23,(tmp_old & 0xef));
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
		tmp_new = ret & 0xdf;
		i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
		if(CURRENT_FIRMWARE_ID == 0xAB056006)
			baseline_data_test = (int16_t *)baseline_cap_data_old[1];
		else {
			if (ts->support_1080x2160_tp)
				baseline_data_test = (int16_t *)baseline_cap_17801_data[1];
			else
				baseline_data_test = (int16_t *)baseline_cap_data[1];
		}
	}
	/******write No Relax to 1******/
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD(30);
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD(30);
	TPD_DEBUG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD(30);

	count = 0;
	for( x = 0; x < TX_NUM; x++ ){

		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			if (fd >= 0){
				sprintf(data_buf, "%d,", baseline_data);
				sys_write(fd, data_buf, strlen(data_buf));
			}
			if( (y < RX_NUM ) && (x < TX_NUM) ){
				//printk("%4d ,",baseline_data);
				if (x == (TX_NUM-1) && y == (RX_NUM-1))
					left_ramdata = baseline_data;
				else if (x == (TX_NUM-1) && y == (RX_NUM-2))
					right_ramdata = baseline_data;
				if(((baseline_data+60) < *(baseline_data_test+count*2)) || ((baseline_data-60) > *(baseline_data_test+count*2+1))){
					if((x == (TX_NUM-1) && (y != RX_NUM-1 || y != RX_NUM-2))||\
						(x != (TX_NUM-1) && (y == RX_NUM-1 || y == RX_NUM-2)))//the last tx and rx last two line for touchkey,others no need take care
					{
						count++;
						continue;
					}
					TPD_ERR("touchpanel failed,RX_NUM:%d,TX_NUM:%d,baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",y,x,baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
					if((baseline_data <= 0) && (first_check == 0)){
						first_check = 1;
						readdata_fail = 1;
					}
					num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
					error_count++;
					goto END;
				}
			}
			/*
			//test virtual key
			if( (y==(RX_NUM-1)) && (x>= TX_NUM-3) ){
			TPD_DEBUG("synaptics:test virtual key,y= %d ,x = %d\n",y,x);
			TPD_DEBUG("Synaptic:test virtual key;baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
			if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
			TPD_ERR("Synaptic:test virtual key failed------------;baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
			num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
			error_count++;
			goto END;
			}
			}
			 */
			count++;
		}
		//printk("\n synaptics:s3320 TX_NUM:%d\n",x);
		if (fd >= 0){
			sys_write(fd, "\n", 1);
		}
	}

	if(!enable_cbc){
		enable_cbc = 1;
		if (fd >= 0){
			sys_write(fd, "\n", 1);
		}
		TPD_ERR("enable cbc baseline test again\n");
		goto TEST_WITH_CBC_s3508;
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if( ret < 0 ) {
		TPD_ERR("%s line%d failed\n",__func__,__LINE__);
		error_count++;
		goto END;
	}

	//Step2 : Check trx-to-ground
	TPD_ERR("step 2:Check trx-to-ground\n" );
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x19);//select report type 25
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD(10);
	tx_datal = i2c_smbus_read_i2c_block_data(ts->client, F54_ANALOG_DATA_BASE+3, 7, buffer);
	if (ts->support_1080x2160_tp) {
		buffer[0] |= 0x20;/*no care 5 31 32 34 36 37 40 52 53chanel*/
		buffer[3] |= 0x80;
		buffer[4] |= 0x35;
		buffer[5] |= 0x01;
		buffer[6] |= 0xc0;
	} else {
		buffer[0] |= 0x10;/*no care 4 31 32 40 50 51 52chanel*/
		buffer[3] |= 0x80;
		buffer[5] |= 0x01;
		buffer[6] |= 0xc0;
	}
	for(x = 0;x < 7; x++)
	{
		if(0xff != buffer[x]){
			error_count++;
            TPD_ERR("step 2:error_count[%d] buff%d[0x%x] ERROR!\n",error_count,x,buffer[x]);
			goto END;
		}
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPD_ERR("%s line%d failed\n",__func__,__LINE__);
		error_count++;
		goto END;
	}

	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	msleep(50);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if( ret < 0 ) {
		TPD_ERR("%s line%d failed\n",__func__,__LINE__);
		error_count++;
		goto END;
	}

	//step 3 :check tx-to-tx and tx-to-vdd
	TPD_ERR("step 3:check TRx-TRx & TRx-Vdd short\n" );
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x1A);//select report type 26
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD(10);
	tx_datal = i2c_smbus_read_i2c_block_data(ts->client, F54_ANALOG_DATA_BASE+3, 7, buffer);
	buffer[0] &= 0xef;/*no care 4 31 32 40 50 51 52chanel*/
	buffer[3] &= 0x7f;
	buffer[5] &= 0xfe;
	buffer[6] &= 0x3f;
	for(x = 0;x < 7; x++)
	{
		if(buffer[x]){
			error_count++;
	        TPD_ERR("step 3:error_count[%d] buff%d[0x%x] ERROR!\n",error_count,x,buffer[x]);
			goto END;
		}
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPD_ERR("%s line%d failed\n",__func__,__LINE__);
		error_count++;
		goto END;
	}

	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	msleep(50);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if( ret < 0 ) {
		TPD_ERR("%s line%d failed\n",__func__,__LINE__);
		error_count++;
		goto END;
	}
	//Step4 : Check RT133
	TPD_ERR("step 4:Check RT133\n" );
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x85);//select report type 133
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x0);//set fifo 0
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD_RT133();
	for (y = 0; y < RX_NUM; y++) {
		ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
		tmp_l = ret&0xff;
		ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
		tmp_h = ret&0xff;
		baseline_data = (tmp_h<<8)|tmp_l;
		if(baseline_data > 100){
			error_count++;
            TPD_ERR("step 4:error_count[%d] baseline_data%d[0x%x] ERROR!\n",error_count,y,baseline_data);
			goto END;
		}
	}
	/*Step 5 : Check RT251 for random touch event*/
	TPD_ERR("Step 5 : Check RT251 for random touch event\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0xFB);/*select report type 0xFB*/
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);/*set fifo 00*/
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);/*get report*/
	checkCMD(100);

	for (x = 0; x < TX_NUM; x++) {

		for (y = 0; y < RX_NUM; y++) {
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret;
			baseline_data = (tmp_h << 8) | tmp_l;
			if ((x < TX_NUM-1) && (y < RX_NUM-2) && (baseline_data > 20)) {        /*red zone, test capacitance,if half num large than 20, fail*/
				if (++err_RT251 > ((TX_NUM - 1) * (RX_NUM - 2) / 2)) {
				    error_count++;
				    TPD_ERR("Step 5 : Check RT251 for random touch event error, capacitance, err_RT251 is %d\n", err_RT251);
				    goto END;
				}
			}

			if ((x != TX_NUM - 1) && (y == RX_NUM - 1) && baseline_data > 500) {      /*blue zone, test self capaccitance, if half num large than 500, fail*/
				if (++err_RT251_self > (TX_NUM - 1) / 2) {
				    error_count++;
				    TPD_ERR("Step 5 : Check RT251 for random touch event error, self capacitance, err_RT251_self is %d\n", err_RT251_self);
				    goto END;
				}
			}
		}
	}
	TPD_ERR("ROLAND----> err_RT251 is %d err_RT251_self is %d\n", err_RT251, err_RT251_self);
	/*Step 6 : Check RT252 for random touch event*/
	TPD_ERR("Step 6 : Check RT252 for random touch event\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0xFC);/*select report type 0xFC*/
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE + 1, 0x00);/*set fifo 00*/
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);/*get report*/
	checkCMD(70);
	for (y = 0; y < RX_NUM + TX_NUM - 3; y++) {
		ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE + 3);
		tmp_l = ret & 0xff;
		ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE + 3);
		tmp_h = ret & 0xff;
		unsigned_baseline_data = (tmp_h << 8) | tmp_l;
		if (unsigned_baseline_data < 10000) {
			error_count++;
			TPD_ERR("Step 6 : Check RT252 for random touch event, error_line is y =%d,data = %hu\n", y, unsigned_baseline_data);
			goto END;
		}
	}
    /*Step 7 : Check RT253 for random touch event*/
	TPD_ERR("Step 7 : Check RT253 for random touch event\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0xFD);/*select report type 0xFD*/
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE + 1, 0x00);/*set fifo 00*/
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);/*get report*/
	checkCMD(70);

	for (x = 0; x < TX_NUM; x++) {
		for (y = 0; y < RX_NUM; y++) {
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret;
			baseline_data = (tmp_h << 8) | tmp_l;
			if (baseline_data  > 20) {
				if (++err_RT253 > (TX_NUM * RX_NUM) / 2) {
				    error_count++;
				    TPD_ERR("Step 7 : Check RT253 for random touch event, self capacitance, err_RT253 is %d\n", err_RT253);
				    goto END;
				}
			}
		}
	}
	TPD_ERR("ROLAND----> err_RT253 is %d\n", err_RT253);
END:
	if (fd >= 0) {
		sys_close(fd);
		set_fs(old_fs);
	}
	//release_firmware(fw);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);
	delay_qt_ms(60);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
	msleep(150);

#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts, 1);
	touch_enable(ts);
	TPD_ERR("\n\nstep5 reset and open irq complete\n");
	mutex_unlock(&ts->mutex);
#endif
	if(readdata_fail == 1){
		TPD_ERR("readdata_fail...try again:%d\n",first_check);
		readdata_fail =0;
		goto READDATA_AGAIN;
	}
#ifdef ENABLE_TPEDGE_LIMIT
	synaptics_tpedge_limitfunc();
#endif
	TPD_ERR("status...first_check:%d:readdata_fail:%d\n",
			first_check, readdata_fail);
	num_read_chars += snprintf(&(buf[num_read_chars]), 128,
		"imageid=0x%lx,deviceid=0x%lx\n", TP_FW, TP_FW);
	num_read_chars += snprintf(&(buf[num_read_chars]), 128,
		"left:=%d,right:%d\n", left_ramdata, right_ramdata);
	num_read_chars += snprintf(&(buf[num_read_chars]), 128,
		"%d error(s). %s\n", error_count,
		error_count?"":"All test passed.");
	return num_read_chars;
}

static ssize_t synaptics_rmi4_baseline_show_s3706(
	struct device *dev, char *buf, bool savefile)
{
	ssize_t num_read_chars = 0;
#if TP_TEST_ENABLE
	int ret = 0;
	uint8_t x, y;
	int tx_datal;
	int16_t baseline_data = 0;
	uint32_t unsigned_baseline_data = 0;
	uint8_t tmp_old = 0;
	uint8_t tmp_new = 0;
	uint8_t tmp_l = 0, tmp_h = 0;
	uint16_t count = 0;
	int error_count = 0;
	uint8_t buffer[9] = {0};
	int16_t *baseline_data_test;
	int enable_cbc = 0;
	int readdata_fail = 0, first_check = 0;
	int16_t left_ramdata = 0, right_ramdata = 0;
	int fd = -1;
	struct timespec   now_time;
	struct rtc_time   rtc_now_time;
	uint8_t  data_buf[64];
	mm_segment_t old_fs;
	unsigned long int  CURRENT_FIRMWARE_ID = 0;

	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	//msm_cpuidle_set_sleep_disable(true);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	synaptics_rmi4_i2c_read_block(ts->client,
		F34_FLASH_CTRL00, 8, buf);
	CURRENT_FIRMWARE_ID = (buf[0] << 24) |
			(buf[1] << 16) | (buf[2] << 8) | buf[3];
	CURRENT_FIRMWARE_ID = CURRENT_FIRMWARE_ID << 32 |
		((buf[4]<<24) | (buf[5]<<16) | (buf[6]<<8) | buf[7]);
	TPD_ERR("[sk]CURRENT_FIRMWARE_ID = 0x%lx\n",
			CURRENT_FIRMWARE_ID);
	snprintf(ts->fw_id, 20, "0x%lx", CURRENT_FIRMWARE_ID);
	//push_component_info(TP, ts->fw_id, ts->manu_name);
READDATA_AGAIN:
	msleep(30);
	mutex_lock(&ts->mutex);
	touch_disable(ts);

	memset(Rxdata, 0, sizeof(Rxdata));
	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);

	if (savefile) {
		getnstimeofday(&now_time);
		rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
		snprintf(data_buf, 40,
		"/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv",
				(rtc_now_time.tm_year+1900)%100,
				rtc_now_time.tm_mon+1,
				rtc_now_time.tm_mday,
				rtc_now_time.tm_hour,
				rtc_now_time.tm_min,
				rtc_now_time.tm_sec);

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
		if (fd < 0) {
			TPD_ERR("Open log file '%s' failed.\n", data_buf);
			set_fs(old_fs);
		}
	}

	/*step 1:check raw capacitance.*/
TEST_WITH_CBC_s3508:
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_DATA_BASE, 0x03);/*select report type 0x03*/
	if (ret < 0) {
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed\n");
		goto END;
	}
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_CONTROL_BASE+20, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client,
		F54_ANALOG_CONTROL_BASE+23);
	tmp_old = ret & 0xff;

	if (enable_cbc) {
		synaptics_hard_reset(ts);
		synaptics_read_register_map_page1(ts);

		ret = i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_DATA_BASE, 0x03);/*select report type 0x03*/
		if (ret < 0) {
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed\n");
			goto END;
		}
		ret = i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+20, 0x01);
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23);

		ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04);
		checkCMD(30);
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23);
		tmp_new = ret  | 0x20;/*CBC Xmtr carrier select bit 5*/
		i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23, tmp_new);
		ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04);
		checkCMD(30);
		TPD_DEBUG("Test open cbc\n");
		baseline_data_test = (int16_t *)baseline_cap_17819_data[0];
	} else {
		ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04);
		checkCMD(30);
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23);
		tmp_new = ret & 0xdf;/*CBC Xmtr carrier select bit 5*/
		i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23, tmp_new);
		ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04);
		/* force update*/
		checkCMD(30);
		ret = i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+7, 0x01);
		/* Forbid NoiseMitigation*/
		baseline_data_test = (int16_t *)baseline_cap_17819_data[1];
	}
	/******write No Relax to 1******/
	ret = i2c_smbus_write_word_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0x04); /* force update*/
	checkCMD(30);
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0X02);/*force Cal*/
	checkCMD(30);
	TPD_DEBUG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts->client,
		F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD(30);

	count = 0;
	for (x = 0; x < TX_NUM; x++) {
	for (y = 0; y < RX_NUM; y++) {
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_DATA_BASE+3);
		tmp_l = ret & 0xff;
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_DATA_BASE+3);
		tmp_h = ret & 0xff;
		usleep_range(1000, 1001);
		baseline_data = (tmp_h << 8)  | tmp_l;
		if (fd >= 0) {
			snprintf(data_buf, 20, "%d,", baseline_data);
			sys_write(fd, data_buf, strlen(data_buf));
		}
		if ((y < RX_NUM) && (x < TX_NUM)) {
			/*printk("%4d ,",baseline_data);*/
			if (x == (TX_NUM-1) &&
				y == (RX_NUM-1))
				left_ramdata = baseline_data;
			else if (x == (TX_NUM-1) &&
				y == (RX_NUM-2))
				right_ramdata = baseline_data;
			if (((baseline_data+60) <
				*(baseline_data_test+count*2)) ||
				((baseline_data-60) >
				*(baseline_data_test+count*2+1))) {
				if (ts->support_1080x2340_tp) {
					if ((y == 0) && (x >= 7) && (x <= 8)) {
						count++;
					TPD_ERR("RX_NU:%dTX_NU:%dbaslin_da%d\n",
					y, x, baseline_data);
						continue;
					}
				} else {
					if ((y == 0) && (x >= 6) && (x <= 9)) {
						count++;
					TPD_ERR("RX_NU:%dTX_NU:%dbaslin_da%d\n",
					y, x, baseline_data);
						continue;
					}
				}
	TPD_ERR("failed,RX_NU:%d,TX_NU:%d,baslin_da is%d\n",
				y, x, baseline_data);
	TPD_ERR("failed,TPK_arrlimit[%d*2]=%d,TPK_arrlimit[%d*2+1]=%d\n",
				count, *(baseline_data_test+count*2),
				count, *(baseline_data_test+count*2+1));
				if ((baseline_data <= 0) &&
					(first_check == 0)) {
					first_check = 1;
					readdata_fail = 1;
				}
				num_read_chars +=
				snprintf(&(buf[num_read_chars]), 60,
				"0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",
				x, y, baseline_data,
				*(baseline_data_test+count*2),
				*(baseline_data_test+count*2+1));
				error_count++;
				goto END;
			}
			}
			count++;
		}
		/*printk("\n synaptics:s3320 TX_NUM:%d\n",x);*/
		if (fd >= 0)
			sys_write(fd, "\n", 1);
	}

	if (!enable_cbc) {
		enable_cbc = 1;
		if (fd >= 0)
			sys_write(fd, "\n", 1);
		TPD_ERR("enable cbc baseline test again\n");
		goto TEST_WITH_CBC_s3508;
	}
	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if (ret < 0) {
		TPD_ERR("%s line%d failed\n", __func__, __LINE__);
		error_count++;
		goto END;
	}

	/*Step2 : Check trx-to-ground*/
	TPD_ERR("step 2:Check trx-to-ground\n");
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_DATA_BASE, 0x19);/*select report type 25*/
	ret = i2c_smbus_write_word_data(ts->client,
		F54_ANALOG_DATA_BASE+1, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0x01);/*get report*/
	checkCMD(10);
	tx_datal = i2c_smbus_read_i2c_block_data(ts->client,
		F54_ANALOG_DATA_BASE+3, 7, buffer);

	for (x = 0; x < 7; x++) {
		if (buffer[x] != 0xff) {
			error_count++;
		TPD_ERR("step 2:error_count[%d] buff%d[0x%x] ERROR!\n",
				error_count, x, buffer[x]);
			goto END;
		}
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if (ret < 0) {
		TPD_ERR("%s line%d failed\n", __func__, __LINE__);
		error_count++;
		goto END;
	}

	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if (ret < 0) {
		TPD_ERR("%s line%d failed\n", __func__, __LINE__);
		error_count++;
		goto END;
	}

	/*step 3 :check tx-to-tx and tx-to-vdd*/
	TPD_ERR("step 3:check TRx-TRx & TRx-Vdd short\n");
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_DATA_BASE, 0x1A);/*select report type 26*/
	ret = i2c_smbus_write_word_data(ts->client,
		F54_ANALOG_DATA_BASE+1, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0x01);/*get report*/
	checkCMD(80);
	tx_datal = i2c_smbus_read_i2c_block_data(ts->client,
			F54_ANALOG_DATA_BASE+3, 7, buffer);
	buffer[0] &= 0xFC;/*buf = 03*/
	buffer[4] &= 0xFC;/*buf = 03*/
	for (x = 0; x < 7; x++) {
		if (buffer[x]) {
			error_count++;
		TPD_ERR("step 3:error_count[%d] buff%d[0x%x] ERROR!\n",
				error_count, x, buffer[x]);
			goto END;
		}
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if (ret < 0) {
		TPD_ERR("%s line%d failed\n", __func__, __LINE__);
		error_count++;
		goto END;
	}

	/*hard reset TP*/
	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if (ret < 0) {
		TPD_ERR("%s line%d failed\n", __func__, __LINE__);
		error_count++;
		goto END;
	}
	/*Step4 : Check RT133*/
	TPD_ERR("step 4:Check RT133\n");
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_DATA_BASE, 0x85);/*select report type 133*/
	ret = i2c_smbus_write_word_data(ts->client,
		F54_ANALOG_DATA_BASE+1, 0x0);/*set fifo 0*/
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0x01);/*get report*/
	checkCMD_RT133();
	for (y = 0; y < RX_NUM; y++) {
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_DATA_BASE+3);
		tmp_l = ret & 0xff;
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_DATA_BASE+3);
		tmp_h = ret & 0xff;
		baseline_data = (tmp_h << 8) | tmp_l;
		if (baseline_data > 100) {
			error_count++;
		TPD_ERR("step 4:error_count[%d] baseline_data%d[0x%x] ERROR!\n",
				error_count, y, baseline_data);
			goto END;
		}
	}
	/*must add reset for RT150 */
	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);
	/*Step 5 : Check RT150 for random touch event*/
	TPD_ERR("Step 5 : Check RT150 for Hybrid Absolute\n");
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_DATA_BASE, 0x96);/*select report type 0x96*/
	ret = i2c_smbus_write_word_data(ts->client,
		F54_ANALOG_DATA_BASE + 1, 0x00);/*set fifo 00*/
	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0x01);/*get report*/
	checkCMD(10);
	/*all channel 16+33 */
	for (y = 0; y < RX_NUM + TX_NUM ; y++) {
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_DATA_BASE + 3);
		tmp_l = ret & 0xff;
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_DATA_BASE + 3);
		tmp_h = ret & 0xff;
		unsigned_baseline_data = (tmp_h << 8) | tmp_l;

		if (unsigned_baseline_data < 10000) {
			error_count++;
		TPD_ERR("Step5 RT150 ,error_line is y =%d,data = %hu\n",
				y, unsigned_baseline_data);
			goto END;
		}
	}
END:
	if (fd >= 0) {
		sys_close(fd);
		set_fs(old_fs);
	}

	ret = i2c_smbus_write_byte_data(ts->client,
		F54_ANALOG_COMMAND_BASE, 0X02);
	delay_qt_ms(60);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
	/*hard reset TP*/
	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);

#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts, 1);
	touch_enable(ts);
	TPD_ERR("\n\nstep5 reset and open irq complete\n");
	mutex_unlock(&ts->mutex);
#endif
	if (readdata_fail == 1) {
		TPD_ERR("readdata_fail...try again:%d\n", first_check);
		readdata_fail = 0;
		goto READDATA_AGAIN;
	}
#ifdef ENABLE_TPEDGE_LIMIT
	synaptics_tpedge_limitfunc();
#endif
	TPD_ERR("status...first_check:%d:readdata_fail:%d\n",
					first_check, readdata_fail);
	//msm_cpuidle_set_sleep_disable(false);
	num_read_chars += snprintf(&(buf[num_read_chars]), 128,
					"imageid=0x%lx,deviceid=0x%lx\n",
					TP_FW, TP_FW);
	num_read_chars += snprintf(&(buf[num_read_chars]), 128,
					"left:=%d,right:%d\n", left_ramdata,
					right_ramdata);
	num_read_chars += snprintf(&(buf[num_read_chars]), 128,
					"%d error(s). %s\n", error_count,
					error_count?"":"All test passed.");
	return num_read_chars;
}


static ssize_t tp_baseline_show_with_cbc(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint8_t tmp_old, tmp_new;
	uint16_t count = 0;
	struct synaptics_ts_data *ts = ts_g;
	if (ts->is_suspended == 1)
		return count;
	memset(delta_baseline, 0, sizeof(delta_baseline));
	if (!ts)
		return 0;
	/*disable irq when read data from IC*/
	touch_disable(ts);
	mutex_lock(&ts->mutex);
	synaptics_hard_reset(ts);
	synaptics_read_register_map_page1(ts);
	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");
	//msm_cpuidle_set_sleep_disable(true);
	//step 1:check raw capacitance.
	//select report type 0x03/* Report Type */
	ret = synaptics_rmi4_i2c_write_byte(ts->client,
		F54_ANALOG_DATA_BASE, 0x03);
	if (ret < 0) {
		TPDTM_DMESG("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}
	if (version_is_s3508 == 2) {
		/* Multi Metric Noise Mitigation Control */
		ret = i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+20, 0x01);
		/* Analog Command 0 *//*force update*/
		ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04);
		checkCMD(10);
		TPD_DEBUG("open CBC oK\n");
		/* Analog Control 1  s3508 is 27 s3706 is 27*/
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23);
		tmp_new = ret | 0x20;/*CBC Xmtr carrier select*/
		i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23, tmp_new);
	} else {
		/* Multi Metric Noise Mitigation Control */
		ret = i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+20, 0x01);
		/* 0D CBC Settings */
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23);
		tmp_old = ret&0xff;
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",
					ret, tmp_old, (tmp_old | 0x10));
		ret = i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+23,
			(tmp_old | 0x10));
		/* Analog Command 0 *//*force update*/
		ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04);
		checkCMD(10);
		TPD_DEBUG("open CBC oK\n");
		/* Analog Control 1  s3508 is 27 s3706 is 27*/
		ret = i2c_smbus_read_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+27);
		tmp_new = ret | 0x20;/*CBC Xmtr carrier select*/
		ret = i2c_smbus_write_byte_data(ts->client,
			F54_ANALOG_CONTROL_BASE+27, tmp_new);
	}
	ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04);

	/*force F54_ANALOG_CMD00*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client,
			F54_ANALOG_COMMAND_BASE, 0X04);
	checkCMD(10);
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	/*Force Cal, F54_ANALOG_CMD00*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client,
			F54_ANALOG_COMMAND_BASE, 0X02);
	checkCMD(10);
	TPDTM_DMESG("Force Cal oK\n");
	/* Report Index LSB //set fifo 00 */
	ret = synaptics_rmi4_i2c_write_word(ts->client,
			F54_ANALOG_DATA_BASE+1, 0x00);
	/*get report*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x01);
	checkCMD(10);
	count = 0;
	for(x = 0;x < TX_NUM; x++) {
		TPD_DEBUG("\n[%d]",x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
		for(y = 0; y < RX_NUM; y++){
			ret = synaptics_rmi4_i2c_read_byte(ts->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(ts->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			TPD_DEBUG("%d,",delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",delta_baseline[x][y]);
		}
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0X02);
	delay_qt_ms(60);
	//msm_cpuidle_set_sleep_disable(false);
	synaptics_enable_interrupt(ts,1);
	mutex_unlock(&ts->mutex);
	touch_enable(ts);
	return num_read_chars;
}

static ssize_t synaptics_rmi4_baseline_show(struct device *dev,	char *buf, bool savefile)
{
	if (version_is_s3508 == 2) {
		return synaptics_rmi4_baseline_show_s3706(dev, buf, savefile);
	} else {
		return synaptics_rmi4_baseline_show_s3508(dev, buf, savefile);
	}
}

static ssize_t tp_test_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	TPDTM_DMESG("tp_test_store is not support\n");
	return count;
}

static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if( (tp_dev == TP_G2Y) || (tp_dev == TP_TPK) )
		return sprintf(buf, "%d\n", TP_TPK);
	if(tp_dev == TP_TRULY)
		return sprintf(buf, "%d\n", TP_TRULY);
	if(tp_dev == TP_OFILM)
		return sprintf(buf, "%d\n", TP_OFILM);
	return sprintf(buf, "%d\n", tp_dev);
}


static int	synaptics_input_init(struct synaptics_ts_data *ts)
{
	int attr_count = 0;
	int ret = 0;

	TPD_DEBUG("%s is called\n",__func__);
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("synaptics_ts_probe: Failed to allocate input device\n");
		return ret;
	}
	ts->input_dev->name = TPD_DEVICE;;
	ts->input_dev->dev.parent = &ts->client->dev;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	set_bit(BTN_TOOL_FINGER, ts->input_dev->keybit);
#ifdef SUPPORT_GESTURE
	set_bit(KEY_F4 , ts->input_dev->keybit);//doulbe-tap resume
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
#endif
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MINOR, 0,255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, (ts->max_x-1), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, (ts->max_y-1), 0, 0);
#ifdef REPORT_2D_PRESSURE
    if (ts->support_ft){
        input_set_abs_params(ts->input_dev,ABS_MT_PRESSURE,0,255, 0, 0);
    }
#endif
#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(ts->input_dev, ts->max_num, 0);
#endif
	input_set_drvdata(ts->input_dev, ts);

	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);
		return -1;
	}
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs_oem); attr_count++) {
		ret = sysfs_create_file(&ts->input_dev->dev.kobj,
				&attrs_oem[attr_count].attr);
		if (ret < 0) {
			dev_err(&ts->client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			for (attr_count--; attr_count >= 0; attr_count--) {
				sysfs_remove_file(&ts->input_dev->dev.kobj,
						&attrs_oem[attr_count].attr);
			}
			return -1;
		}
	}
	return 0;
}

#include "fw_update_v7.if"
static int check_hardware_version(struct device *dev)
{
        int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
        const struct firmware *fw = NULL;
	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");
		return 0;
	}
	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
		return ret;
	}
        ret = fwu_start_reflash_check(fw->data,ts->client);
	release_firmware(fw);
        if (ret < 0)
		return -1;
        else
            	return ret;
}
static int check_version = 0;

/*FW Update Func for s3706*/
static int synaptics_rmi4_sw_reset(struct i2c_client *client)
{
	int retval;
	unsigned char command = 0x01;

	retval = synaptics_rmi4_i2c_write_block(client,
		F01_RMI_CTRL_BASE,
		sizeof(command),
		&command);

	return retval;
}

static int synaptics_rmi4_reset_device(struct i2c_client *client,
		bool rebuild)
{
	int retval;

	rebuild = 1;
	/*mutex_lock(&(rmi4_data->rmi4_reset_mutex));*/

	retval = synaptics_rmi4_sw_reset(client);
	if (retval < 0) {
		TPD_ERR("%s: Failed to issue reset command\n",
				__func__);
		return retval;
	}
	/*mutex_unlock(&(rmi4_data->rmi4_reset_mutex));*/
	return retval;
}

/*FW Update Func**/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int ret;
	char fw_id_temp[20];
	uint8_t buf[8];
	unsigned long int CURRENT_FIRMWARE_ID = 0;

        static bool check_onetime = true;

        TPD_DEBUG("%s is called\n",__func__);
	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");
		return 0;
	}
	if (!strncmp(ts->manu_name, "S3706", 5)) {
		TPD_ERR("enter version 17819 update mode\n");
			ret = request_firmware(&fw, ts->fw_name, dev);
			if (ret < 0) {
				TPD_ERR("Request firmware failed - %s (%d)\n",
				ts->fw_name, ret);
				return ret;
			}
	} else if (!strncmp(ts->manu_name, "S3718", 5)) {
		if(check_onetime){
			check_onetime = false;
			check_version = check_hardware_version(dev);
			TPD_ERR("%s:first check hardware version %d\n",__func__,check_version);
			if(check_version < 0){
				TPD_ERR("checkversion fail....\n");
				return -1;
			}
		}

		if(1 == check_version ) {
			TPD_DEBUG("enter version 15801 update mode\n");
			strcpy(ts->fw_name,"tp/fw_synaptics_15801.img");
			//push_component_info(TP, ts->fw_id, "S3718_vA");
			ret = request_firmware(&fw, ts->fw_name, dev);
			if (ret < 0) {
				TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
				return ret;
		       }

		 }else{
		        TPD_DEBUG("enter version 15801 vb update mode\n");
			//push_component_info(TP, ts->fw_id, "S3718_vB");
			ret = request_firmware(&fw, ts->fw_name, dev);
			if (ret < 0) {
				TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
				return ret;
		       }
		}

	}else if(!strncmp(ts->manu_name,"s3508",5) || !strncmp(ts->manu_name,"15811",5)){
		        TPD_ERR("enter version 16859 update mode\n");
			//push_component_info(TP, ts->fw_id, "s3508");
			ret = request_firmware(&fw, ts->fw_name, dev);
			if (ret < 0) {
				TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
				return ret;
		       }
	}else{
		TPD_ERR("firmware name not match\n");
		return -1;
	}

	ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
	if(ret < 0){
		TPD_ERR("FW update not success try again\n");
		ret = synapitcs_ts_update(ts->client, fw->data, fw->size, true);
		if(ret < 0){
			TPD_ERR("FW update failed twice, quit updating process!\n");
			return ret;
		}
	}
	release_firmware(fw);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	ret = synaptics_rmi4_i2c_read_block(ts->client,
		F34_FLASH_CTRL00, 8, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) |
		(buf[2]<<8) | buf[3];
	if (version_is_s3508 == 2)
		CURRENT_FIRMWARE_ID = CURRENT_FIRMWARE_ID << 32 |
			((buf[4]<<24) | (buf[5]<<16) | (buf[6]<<8) | buf[7]);
	TPD_ERR("CURRENT_FIRMWARE_ID 0x%lx!\n", CURRENT_FIRMWARE_ID);
	snprintf(fw_id_temp, 20, "0x%lx", CURRENT_FIRMWARE_ID);
	strcpy(ts->fw_id,fw_id_temp);
	TP_FW = CURRENT_FIRMWARE_ID;
	report_key_point_y = ts->max_y*button_map[2]/LCD_HEIGHT;
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts,1);
	return 0;
}

static ssize_t synaptics_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t synaptics_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	unsigned long val;
	int rc;
/*
	int bootmode;

	bootmode = get_boot_mode();
	TPD_ERR("synaptics bootmode %d  !\n", bootmode);
	if ((bootmode == MSM_BOOT_MODE__FACTORY)
		|| (bootmode == MSM_BOOT_MODE__RF)
		|| (bootmode == MSM_BOOT_MODE__WLAN)) {
		TPD_ERR("synaptics disable tp update firmware update\n");
		return size;
	}
*/
	if (ts->is_suspended && ts->support_hw_poweroff){
		TPD_ERR("power off firmware abort!\n");
		return size;
	}
	if (version_is_s3508 == 1) {
		if (strncmp(ts->manu_name,"s3508",5) && strncmp(ts->manu_name,"15811",5)){
        		TPD_ERR("product name[%s] do not update!\n",ts->manu_name);
        		return size;
   		 }
	} else if (version_is_s3508 == 2) {
		if (strncmp(ts->manu_name, "S3706", 5)) {
			TPD_ERR("product name[%s] do not update!\n",
			ts->manu_name);
			return size;
		}
	}else{
    		if (strncmp(ts->manu_name,"S3718",5)){
        		TPD_ERR("product name[%s] do not update!\n",ts->manu_name);
        		return size;
   		 }
	}
	TPD_ERR("start update ******* fw_name:%s,ts->manu_name:%s\n",ts->fw_name,ts->manu_name);

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if(!val)
		val = force_update;

	touch_disable(ts);
	mutex_lock(&ts->mutex);
	ts->loading_fw = true;
	synatpitcs_fw_update(dev, val);
	ts->loading_fw = false;
	mutex_unlock(&ts->mutex);
	touch_enable(ts);
	force_update = 0;
	return size;
}
/*********************FW Update Func End*************************************/


static ssize_t synaptics_test_limit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	int ret = 0;
	uint16_t *prow = NULL;
	uint16_t *prowcbc = NULL;
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;
	int i = 0;
	int temp = 0;
	static int cat_cbc_change = 0;
	ret = request_firmware(&fw, ts->test_limit_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
				ts->test_limit_name, ret);
		temp = temp + sprintf(&buf[temp],"Request failed,Check the path %d",temp);
		return temp;
	}

	ph = (struct test_header *)(fw->data);
	prow = (uint16_t *)(fw->data + ph->array_limit_offset);

	prowcbc = (uint16_t *)(fw->data + ph->array_limitcbc_offset);

	TPD_DEBUG("synaptics_test_limit_show:array_limit_offset = %x array_limitcbc_offset = %x\n",
			ph->array_limit_offset,ph->array_limitcbc_offset);

	TPD_DEBUG("test begin:\n");
	if(cat_cbc_change == 0 || ph->withCBC == 0) {
		temp += sprintf(buf, "Without cbc:");
		for(i = 0 ;i < (ph->array_limit_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
				temp += sprintf(&(buf[temp]), "\n[%d] ",(i/RX_NUM)/2);
			temp += sprintf(&buf[temp],"%d,",prow[i]);
			printk("%d,",prow[i]);
		}
		cat_cbc_change = 1;
	}else{
		temp += sprintf(buf, "With cbc:");
		cat_cbc_change = 0;
		if( ph->withCBC == 0){
			return temp;
		}
		for(i = 0 ;i < (ph->array_limitcbc_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
				temp += sprintf(&(buf[temp]), "\n[%d] ",(i/RX_NUM)/2);
			temp += sprintf(&buf[temp],"%d,",prowcbc[i]);
			printk("%d,",prowcbc[i]);
		}
	}
	release_firmware(fw);
	return temp;
}

static ssize_t synaptics_test_limit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t tp_doze_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int doze_time = 0;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0)
		return snprintf(buf, 16, "switch page err\n");

	doze_time = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL02);
	return snprintf(buf, 2, "%d\n", doze_time);
}

static ssize_t tp_doze_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static int touch_hold_en(struct synaptics_ts_data *ts, bool enable)
{
	int ret = 0;
	int touch_hold_enable = 0;

	/* SYNA_F51_CUSTOM_CTRL20_00 0x0428*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x04);
	if (ret < 0)
		TPD_ERR("%s: set page 0x04 fail!\n", __func__);
	touch_hold_enable = i2c_smbus_read_byte_data(ts->client, 0x2c);

	if (enable == 1)
		touch_hold_enable = touch_hold_enable | 0x01;
	else if (enable == 0)
		touch_hold_enable = touch_hold_enable & 0xfe;

	ret = synaptics_rmi4_i2c_write_byte(ts->client,
		0x2c, touch_hold_enable);
	if (ret < 0)
		TPD_ERR("%s: set reg fail!\n", __func__);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0)
		TPD_ERR("%s: set page 00 fail!\n", __func__);

	return ret;

}

int tp_single_tap_en(struct synaptics_ts_data *ts, bool enable)
{
	uint8_t ret = 0;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x04);
	if (ret < 0)
		return ret;
	if (enable) {
		ret = synaptics_rmi4_i2c_write_byte(ts->client,
			0x22, 0x01);
		ret = synaptics_rmi4_i2c_write_byte(ts->client,
			0x23, 0);
		ret = synaptics_rmi4_i2c_write_byte(ts->client,
			0x24, 0);
		ret = synaptics_rmi4_i2c_write_byte(ts->client,
			0x25, 0x64);
		ret = i2c_smbus_write_word_data(ts->client,
			F54_ANALOG_COMMAND_BASE, 0x04); // force update
		TPD_DEBUG("%s: force update %x\n",
			__func__, F54_ANALOG_COMMAND_BASE);
	} else
		ret = synaptics_rmi4_i2c_write_byte(ts->client,
			0x22, 0x00);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0)
		TPD_ERR("%s: set page 00 fail!\n", __func__);

	return ret;
}

static ssize_t tp_gesture_touch_hold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int touch_hold_enable = 0;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	/* SYNA_F51_CUSTOM_CTRL20_00 0x0428*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x04);
	if (ret < 0)
		return snprintf(buf, 16, "switch page err\n");
	touch_hold_enable = i2c_smbus_read_byte_data(ts->client, 0x2c);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0)
		TPD_ERR("%s: set page 00 fail!\n", __func__);
	return snprintf(buf, 6, "0x%x\n", touch_hold_enable);
}

static ssize_t tp_gesture_touch_hold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int tmp = 0;
	int touch_hold_enable = 0;
	int ret = 0;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &tmp);
	if (ret < 0) {
		TPDTM_DMESG("invalid content: '%s', length = %zd\n", buf, size);
		return size;
	}

	TPD_ERR("%s: set %d\n", __func__, tmp);
	if (tmp == 2 && ts->fp_aod_cnt > 0) {
		ts->fp_up_down = 0;
		ts->unlock_succes = 1;
		return size;
	}

	if (tmp == 3) {
		ts->unlock_succes = 0;
		return size;
	}

	mutex_lock(&ts->mutex);
	/* SYNA_F51_CUSTOM_CTRL20_00 0x0428*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x04);
	if (ret < 0)
		TPD_ERR("%s: set page 0x04 fail!\n", __func__);
	touch_hold_enable = i2c_smbus_read_byte_data(ts->client, 0x2c);

	if (tmp == 1) {
		touch_hold_enable = touch_hold_enable | 0x01;
		ts->en_up_down = 1;
	} else if (tmp == 0) {
		touch_hold_enable = touch_hold_enable & 0xfe;
		ts->en_up_down = 0;
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client,
		0x2c, touch_hold_enable);
	if (ret < 0)
		TPD_ERR("%s: set reg fail!\n", __func__);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0)
		TPD_ERR("%s: set page 00 fail!\n", __func__);
	mutex_unlock(&ts->mutex);

	return size;
}

//static DRIVER_ATTR(tp_baseline_image_with_cbc, 0664, tp_baseline_show_with_cbc, tp_test_store);
static DEVICE_ATTR(test_limit, 0664, synaptics_test_limit_show, synaptics_test_limit_store);
static DRIVER_ATTR(tp_baseline_image, 0664, tp_baseline_show, tp_delta_store);
static DRIVER_ATTR(tp_baseline_image_with_cbc, 0664, tp_baseline_show_with_cbc, tp_test_store);
static DRIVER_ATTR(tp_delta_image, 0664, tp_rawdata_show, NULL);
static DRIVER_ATTR(tp_debug_log, 0664, tp_show, store_tp);
static DEVICE_ATTR(tp_fw_update, 0664, synaptics_update_fw_show, synaptics_update_fw_store);
static DEVICE_ATTR(tp_doze_time, 0664, tp_doze_time_show, tp_doze_time_store);
static DEVICE_ATTR(tp_gesture_touch_hold, 0664,
	tp_gesture_touch_hold_show, tp_gesture_touch_hold_store);
static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts);

static ssize_t tp_debug_log_write_func(
	struct file *file, const char *buffer,
	size_t count, loff_t *ppos)
{
	int ret, tmp = 0;
	char buf[4] = {0};

	if (count > 4)
		return count;
	if (copy_from_user(buf, buffer, count)) {
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	ret = kstrtoint(buf, 10, &tmp);
	if (ret >= 0) {
		tp_debug = tmp;
	} else {
	TPDTM_DMESG("invalid content: '%s', length = %zd\n",
	buf, count);
	}
	return count;
}

static ssize_t tp_debug_log_read_func(
	struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4];

	ret = snprintf(page, 4, "%d\n", tp_debug);
	ret = simple_read_from_buffer(user_buf, count,
		ppos, page, strlen(page));
	return ret;
}

static const struct file_operations tp_debug_log_proc_fops = {
	.write = tp_debug_log_write_func,
	.read =  tp_debug_log_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static ssize_t synaptics_main_reg_read_func(
	struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	int device_contrl = 0;
	int doze_interval = 0;
	unsigned char reportbuf[4] = {0};
	struct synaptics_ts_data *ts = ts_g;

	if (!ts)
		return ret;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0)
		TPD_ERR("%s: chage page failed:%d\n", __func__, ret);

	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL00);
	if (ret < 0)
		TPD_ERR("%s: read failed:%d\n", __func__, ret);

	TPD_ERR("%s Device Control 0x%x=0x%x\n", __func__, F01_RMI_CTRL00, ret);
	device_contrl = ret;

	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL02);
	if (ret < 0)
		TPD_ERR("%s: read failed:%d\n", __func__, ret);

	TPD_ERR("Doze Interval 0x%x=0x%x\n", F01_RMI_CTRL02, ret);
	doze_interval = ret;

	ret = i2c_smbus_read_i2c_block_data(ts->client, F12_2D_CTRL20,
		3, &(reportbuf[0x0]));
	if (ret < 0)
		TPD_ERR("read reg F12_2D_CTRL20[0x%x] failed\n", F12_2D_CTRL20);

	TPD_ERR("Gesture Report Flag 0x%x=[2]:0x%x\n",
		F12_2D_CTRL20, reportbuf[2]);

	ret = snprintf(page, 128, "0x%x:0x%x:0x%x\n",
		device_contrl, doze_interval, reportbuf[2]);
	ret = simple_read_from_buffer(user_buf, count,
		ppos, page, strlen(page));
	return ret;
}

static const struct file_operations tp_main_reg = {
	.read =  synaptics_main_reg_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};


static ssize_t tp_reset_write_func (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int ret, write_flag, i;
	char buf[10] = {0};
	struct synaptics_ts_data *ts = ts_g;

	if (count > 10)
		return count;
	if (!ts)
		return count;
	if (ts->loading_fw) {
		TPD_ERR("%s FW is updating break!!\n", __func__);
		return count;
	}
	if (copy_from_user(buf, buffer, count)) {
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	ret = sscanf(buf, "%d", &write_flag);

	TPD_ERR("%s write [%d]\n",__func__,write_flag);
	if (1 == write_flag)
	{
		ret = synaptics_soft_reset(ts);
	}
	else if(2 == write_flag)
	{
		synaptics_hard_reset(ts);
	}
	else if(3 == write_flag)
	{
		disable_irq_nosync(ts->irq);
	}
	else if(4 == write_flag)
	{
		enable_irq(ts->irq);
	}
	else if(8 == write_flag)
	{
		touch_enable(ts);
	}
	else if(9 == write_flag)
	{
		touch_disable(ts);
	}
	else if(5 == write_flag)
	{
		synaptics_read_register_map(ts);
	}
	else if(6 == write_flag)
	{
		for (i = 0; i < ts->max_num; i++)
		{
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
		#ifndef TYPE_B_PROTOCOL
		input_mt_sync(ts->input_dev);
		#endif
		input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
		input_sync(ts->input_dev);
	}
	return count;
}

//chenggang.li@bsp add for 14045
static const struct file_operations base_register_address= {
	.write = synap_write_address,
	.read =  synap_read_address,
	.open = simple_open,
	.owner = THIS_MODULE,
};


//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations i2c_device_test_fops = {
	.read =  i2c_device_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations tp_baseline_test_proc_fops = {
	.read =  tp_baseline_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

#ifdef SUPPORT_GLOVES_MODE
static const struct file_operations glove_mode_enable_proc_fops = {
	.write = tp_glove_write_func,
	.read =  tp_glove_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

static const struct file_operations sleep_mode_enable_proc_fops = {
	.write = tp_sleep_write_func,
	.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations tp_reset_proc_fops = {
	.write = tp_reset_write_func,
	//.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static const struct file_operations vendor_id_proc_fops = {
	.read =  vendor_id_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static int set_changer_bit(struct synaptics_ts_data *ts)
{
    int mode;
    int ret;
    mode = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_CTRL00);
    if (ts->changer_connet)
        mode = mode | 0x20;
    else
        mode = mode & 0xDF;
    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, mode);
    return ret;
}
static ssize_t changer_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	ret = sprintf(page, "the changer is %s!\n", ts->changer_connet?("conneted"):("disconneted"));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t changer_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts= ts_g;
	int ret = 0 ;
	char buf[4] = {0};

	if (count > 2)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		TPD_ERR(KERN_INFO "%s: write proc input error.\n", __func__);
		return count;
	}

	if (-1 == sscanf(buf, "%d", &ret)) {
		TPD_ERR("%s sscanf error\n", __func__);
		return count;
	}
	if(!ts)
		return count;
	if( (ret == 0 ) || (ret == 1) ){
		ts->changer_connet = ret;
        ret = set_changer_bit(ts);
	}
	TPDTM_DMESG("%s:ts->changer_connet = %d\n",__func__,ts->changer_connet);
	return count;
}
static const struct file_operations changer_ops = {
	.write = changer_write_func,
	.read =  changer_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static void set_doze_time(int doze_time)
{
	static int pre_doze_time;
	int ret = 0;
	struct synaptics_ts_data *ts = ts_g;

	/* change to page 0 */
	if (ts == NULL) {
		TPD_ERR("ts crash!\n");
		return;
	}
	if (pre_doze_time == doze_time) {
		TPD_ERR("set time have already been set\n");
		return;
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0) {
		TPD_ERR("%s: chage page failed:%d\n", __func__, ret);
		return;
	}

	TPD_ERR("%s: set doze time: %d\n", __func__, doze_time);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL02, doze_time);
	if (ret < 0) {
		TPD_ERR("%s: set doze time err:%d\n", __func__, ret);
		return;
	}
	pre_doze_time = doze_time;

	/* use the read out circle to delay */
	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL02);
	if (ret < 0)
		return;
	if (ret != doze_time) {
		TPD_ERR("reset doze time\n");
		ret = i2c_smbus_write_byte_data(ts->client,
				F01_RMI_CTRL02, doze_time);
		if (ret < 0) {
			TPD_ERR("%s: reset doze time err:%d\n", __func__, ret);
			return;
		}
	}
}

#define SUBABS(x,y) ((x)-(y))
static int tp_baseline_get(struct synaptics_ts_data *ts, bool flag)
{
	int ret = 0;
	int x, y;
	uint8_t *value;
	int k = 0;
	int touch_hold_enable = 0;
	int touch_hold_retry = 0;

	if(!ts)
		return -1;

	atomic_set(&ts->is_stop,1);
	touch_disable(ts);
	TPD_DEBUG("%s start!\n",__func__);
	value = kzalloc(TX_NUM*RX_NUM*2, GFP_KERNEL);
	memset(delta_baseline,0,sizeof(delta_baseline));

	mutex_lock(&ts->mutex);
	if (ts->gesture_enable) {
		synaptics_enable_interrupt_for_gesture(ts,false);
		synaptics_mode_change(0x00);
		//change to active later getbase data
	} else {
		synaptics_mode_change(0x00);
		//change to active later getbase data
	}
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD(10);

	ret = synaptics_rmi4_i2c_read_block(ts->client,F54_ANALOG_DATA_BASE+3,2*TX_NUM*RX_NUM,value);
	for( x = 0; x < TX_NUM; x++ ){
		for( y = 0; y < RX_NUM; y++ ){
			delta_baseline[x][y] =  (int16_t)(((uint16_t)( value [k])) | ((uint16_t)( value [k+1] << 8)));
			k = k + 2;

			if (flag)
				delta[x][y] = SUBABS(delta_baseline[x][y],baseline[x][y]);
			else
				baseline[x][y] = delta_baseline[x][y];
		}
	}
	//ret = i2c_smbus_write_byte_data(ts->client,
	//F54_ANALOG_COMMAND_BASE, 0X02);

	if (ts->project_version != 0x03) {
		ret = synaptics_rmi4_i2c_write_byte(ts->client,
				0xff, 0x0);
		ret = i2c_smbus_write_byte_data(ts->client,
			F01_RMI_CMD_BASE, 0x01);//soft reset
	} else {
		if (!not_getbase && ts->unlock_succes == 0) {
			not_getbase = 0;
			TPD_DEBUG("touch hold not getbase reset");
			ret = synaptics_rmi4_i2c_write_byte(ts->client,
				0xff, 0x0);
			ret = i2c_smbus_write_byte_data(ts->client,
				F01_RMI_CMD_BASE, 0x01);//soft reset
		}
	}
	mutex_unlock(&ts->mutex);
	atomic_set(&ts->is_stop, 0);
	msleep(2);
	if (ts->gesture_enable)
		set_doze_time(1);
	touch_enable(ts);
#ifdef ENABLE_TPEDGE_LIMIT
	synaptics_tpedge_limitfunc();
#endif
	if (ts->project_version == 0x03) {
		ts->fp_up_down = 0;
		TPD_DEBUG("%s:fp_down_up %d gesture %d, en_down_up %d\n",
			__func__, ts->fp_up_down,
			ts->in_gesture_mode, ts->en_up_down);
		while (ts->en_up_down) {
			touch_hold_enable = 0;
			touch_hold_retry++;
			ret = synaptics_rmi4_i2c_write_byte(ts->client,
				0xff, 0x04);
			if (ret < 0)
				TPD_ERR("set page first fail!\n");
			touch_hold_enable =
				i2c_smbus_read_byte_data(ts->client, 0x2c);
			TPDTM_DMESG("%s:read reg 0x%x\n",
				__func__, touch_hold_enable);
			touch_hold_enable = touch_hold_enable | 0x01;
			ret = synaptics_rmi4_i2c_write_byte(ts->client,
				0x2c, touch_hold_enable);
			if (ret < 0)
				TPD_ERR("set first fail!\n");
			ret = i2c_smbus_read_byte_data(ts->client, 0x2c);
			TPDTM_DMESG("%s:read reg again 0x%x,0x%x\n", __func__,
				touch_hold_enable, ret);
			ret = synaptics_rmi4_i2c_write_byte(ts->client,
				0xff, 0x00);
			if (ret < 0)
				TPD_ERR("set page 00 fail!\n");
			if (touch_hold_enable == ret)
				break;
			if (touch_hold_retry == 3)
				break;
		}
	}
	TPD_DEBUG("%s end! \n",__func__);
	kfree(value);
	return 0;
}
static void tp_baseline_get_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts= ts_g;

	tp_baseline_get(ts, true);//get the delta data
}

static ssize_t touch_press_status_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int x,y;
	int press_points = 0;
	int points_misspresee =0;
	int str_n = 0;

	char *page = kzalloc(1024*2,GFP_KERNEL);
	if (!page){
		TPD_ERR("%s malloc memery error!",__func__);
		return -ENOMEM;
	}
	TPD_ERR("%s",__func__);

	for (x = 0; x < TX_NUM; x++) {
		for (y = 0; y < RX_NUM; y++) {
			if (ts_g->support_1080x2160_tp) {
				if (x > (TX_NUM-1) || y > (RX_NUM-15))
					continue;
			} else {
				if (x > (TX_NUM-1) || y < (RX_NUM-12))
					continue;
			}
			if ((delta[x][y] < -30) && (delta[x][y] > -250))
			{
				//str_n += sprintf(&page[str_n],"x%d,y%d = %4d\n", x, y, delta[x][y]);
				press_points++;
			}
			if((delta[x][y] > 30) && (delta[x][y] < 200))
				points_misspresee ++;
		}

	}

	if(points_misspresee > 4)
		get_tp_base = 0;
	TPD_ERR("points_mispressee num:%d,get_tp_base:%d\n",points_misspresee,get_tp_base);
	str_n += sprintf(&page[str_n], "\n%s %d points delta > [25]\n",(press_points>4)?"near":"away", press_points);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	kfree(page);
	return ret;
}

static ssize_t touch_press_status_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts= ts_g;
	int ret = 0 ;
	char buf[4] = {0};

	if (count > 2)
		return count;

	if (copy_from_user(buf, buffer, count)) {
		TPD_ERR("%s write error\n", __func__);
		return count;
	}
	if (-1 == sscanf(buf, "%d", &ret)) {
		TPD_ERR("%s sscanf error\n", __func__);
		return count;
	}
	if(!ts)
		return count;

	TPD_ERR("%s write %d\n",__func__,ret);
	if (ret == 0){
		tp_baseline_get(ts,false);
	}
	else if(ret == 1) {
		if (0 == ts->gesture_enable)
			queue_delayed_work(get_base_report, &ts->base_work,msecs_to_jiffies(120));
		else
			queue_delayed_work(get_base_report, &ts->base_work,msecs_to_jiffies(1));
	}
	return count;
}
static const struct file_operations touch_press_status = {
	.write = touch_press_status_write,
	.read =  touch_press_status_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

#ifdef ENABLE_TPEDGE_LIMIT
static ssize_t limit_enable_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	ssize_t ret =0;
	char page[PAGESIZE];

	TPD_DEBUG("the limit_enable is: %d\n", limit_enable);
	ret = sprintf(page, "%d\n", limit_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t limit_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
        int ret;
	char buf[8]={0};
        int limit_mode = 0;

	if (version_is_s3508 == 2) {
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x0a;
	} else if (version_is_s3508 == 0) {
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x34;
	} else if (version_is_s3508 == 1) {
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x1b;
	} else
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x0a;

	if( count > 2)
		count = 2;
	if(ts_g == NULL)
	{
		TPD_ERR("ts_g is NULL!\n");
		return -1;
	}
	if(copy_from_user(buf, buffer, count))
	{
		TPD_DEBUG("%s: read proc input error.\n", __func__);
		return count;
	}

	if('0' == buf[0]){
		limit_enable = 0;
	}else if('1' == buf[0]){
		limit_enable = 1;
	}
	msleep(30);
	mutex_lock(&ts_g->mutex);
	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x4);
	limit_mode = i2c_smbus_read_byte_data(ts_g->client,
						F51_GRIP_CONFIGURATION);
	TPD_ERR("%s_proc limit_enable =%d,mode:0x%x,grip:0x%x!\n",
					__func__, limit_enable, limit_mode,
					F51_GRIP_CONFIGURATION);
	if(limit_mode){
		i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x4);
		if(0 == limit_enable)
		{
			limit_mode = limit_mode & 0xFE;
			ret = i2c_smbus_write_byte_data(ts_g->client,
				F51_GRIP_CONFIGURATION, limit_mode);
		}
		else if(1 == limit_enable)
		{
			limit_mode = limit_mode | 0x1;
			ret = i2c_smbus_write_byte_data(ts_g->client,
				F51_GRIP_CONFIGURATION, limit_mode);
		}
	}
	i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0);
	mutex_unlock(&ts_g->mutex);
	return count;
}

static const struct file_operations proc_limit_enable =
{
	.read = limit_enable_read,
	.write = limit_enable_write,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif
#ifdef SUPPORT_TP_TOUCHKEY
static ssize_t key_switch_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_ERR("%s left:%s right:%s\n", __func__,
		    key_switch?"key_back":"key_appselect",
		    key_switch?"key_appselect":"key_back");
	ret = snprintf(page, PAGE_SIZE, "key_switch left:%s right:%s\n",
		    key_switch?"key_back":"key_appselect",
		    key_switch?"key_appselect":"key_back");
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t key_switch_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[4] = {0};
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;
	if(count > 2)
		return count;
	if(copy_from_user(buf, buffer, count))
	{
		TPD_ERR("%s copy error\n", __func__);
		return count;
	}
	sscanf(&buf[0], "%d", &key_switch);
	TPD_ERR("%s write [%d]\n", __func__, key_switch);
	TPD_ERR("left:%s right:%s\n", key_switch?"key_back":"key_appselect",
		key_switch?"key_appselect":"key_back");
	return count;
}

static const struct file_operations key_switch_proc_fops = {
	.write = key_switch_write_func,
	.read =  key_switch_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static ssize_t key_disable_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_ERR("%s key_back:%s key_appselect:%s\n",__func__,key_back_disable?"disable":"enable",key_appselect_disable?"disable":"enable");
	ret = sprintf(page, "cmd:enable,disable\nkey_back:%s key_appselect:%s\n",key_back_disable?"disable":"enable",key_appselect_disable?"disable":"enable");
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t key_disable_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;
	if( count > sizeof(buf)){
		TPD_ERR("%s error\n",__func__);
		return count;
	}

	if(copy_from_user(buf, buffer, count))
	{
		TPD_ERR("%s copy error\n", __func__);
		return count;
	}
	if (NULL != strstr(buf,"disable"))
	{
		key_back_disable =true;
		key_appselect_disable = true;
	}
	else if (NULL != strstr(buf,"enable"))
	{
		key_back_disable =false;
		key_appselect_disable = false;
	}
	TPD_ERR("%s key_back:%d key_appselect:%d\n",__func__,key_back_disable,key_appselect_disable);
	return count;
}

static const struct file_operations key_disable_proc_fops = {
	.write = key_disable_write_func,
	.read =  key_disable_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif
static int init_synaptics_proc(void)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tmp  = NULL;
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if( prEntry_tp == NULL ){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create touchpanel\n");
	}

#ifdef SUPPORT_GESTURE
	prEntry_tmp = proc_create( "gesture_enable", 0666, prEntry_tp, &tp_gesture_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create gesture_enable\n");
	}
	prEntry_tmp = proc_create( "gesture_switch", 0666, prEntry_tp, &gesture_switch_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create gesture_switch\n");
	}
	prEntry_tmp = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create coordinate\n");
	}
#endif

#ifdef SUPPORT_GLOVES_MODE
	prEntry_tmp = proc_create( "glove_mode_enable", 0666, prEntry_tp,&glove_mode_enable_proc_fops);
	if(prEntry_tmp == NULL) {
		ret = -ENOMEM;
        TPD_ERR("Couldn't create glove_mode_enable\n");
	}
#endif

#ifdef SUPPORT_TP_SLEEP_MODE
	prEntry_tmp = proc_create("sleep_mode_enable", 0666, prEntry_tp, &sleep_mode_enable_proc_fops);
	if( prEntry_tmp == NULL ){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create sleep_mode_enable\n");
	}
#endif

#ifdef RESET_ONESECOND
	prEntry_tmp = proc_create( "tp_reset", 0666, prEntry_tp, &tp_reset_proc_fops);
	if( prEntry_tmp == NULL ){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create tp_reset\n");
	}
#endif
#ifdef ENABLE_TPEDGE_LIMIT
	prEntry_tmp = proc_create("tpedge_limit_enable", 0666, prEntry_tp, &proc_limit_enable);
	if( prEntry_tmp == NULL ){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create tp_limit_enable\n");
	}
#endif

	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
	prEntry_tmp = proc_create( "baseline_test", 0666, prEntry_tp, &tp_baseline_test_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create baseline_test\n");
	}
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\i2c_device_test"  begin
	prEntry_tmp = proc_create( "i2c_device_test", 0666, prEntry_tp, &i2c_device_test_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create i2c_device_test\n");
	}

	prEntry_tmp = proc_create( "radd", 0777, prEntry_tp, &base_register_address);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create radd\n");
	}
	prEntry_tmp = proc_create("vendor_id", 0444, prEntry_tp, &vendor_id_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create vendor_id\n");
	}
	prEntry_tmp = proc_create("changer_connet", 0666, prEntry_tp, &changer_ops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create changer_connet\n");
	}

	prEntry_tmp = proc_create("touch_press", 0666, prEntry_tp, &touch_press_status);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create touch_press\n");
	}
#ifdef SUPPORT_TP_TOUCHKEY
	prEntry_tmp = proc_create("key_switch", 0666, prEntry_tp, &key_switch_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create key_switch\n");
	}

	prEntry_tmp = proc_create("key_disable", 0666, prEntry_tp, &key_disable_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create key_disable\n");
	}
#endif

	/*morgan.gu add for logkit to dump main registor*/
	prEntry_tmp = proc_create("tp_main_reg", 0444,
		prEntry_tp, &tp_main_reg);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_ERR("Couldn't create tp_main_reg\n");
	}

	/*morgan.gu add for logkit to open more log*/
	prEntry_tmp = proc_create("tp_debug_log", 0664,
		prEntry_tp, &tp_debug_log_proc_fops);
	if (prEntry_tmp == NULL) {
		ret = -ENOMEM;
		TPD_ERR("Couldn't create tp_debug_log_proc_fops\n");
	}

	return ret;
}
/******************************end****************************/

/****************************S3203*****update**********************************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

static void re_scan_PDT(struct i2c_client *client)
{
	uint8_t buf[8];
	i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	SynaF34DataBase = buf[3];
	SynaF34QueryBase = buf[0];
	i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	SynaF01DataBase = buf[3];
	SynaF01CommandBase = buf[1];
	i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);

	SynaF34Reflash_BlockNum = SynaF34DataBase;
	SynaF34Reflash_BlockData = SynaF34DataBase + 1;
	SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
	SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
	SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
	SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
	i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
	SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	TPD_DEBUG("SynaFirmwareBlockSize 3310 is %d\n", SynaFirmwareBlockSize);
	SynaF34_FlashControl = SynaF34DataBase + 2;
}
struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
		(unsigned int)ptr[1] * 0x100 +
		(unsigned int)ptr[2] * 0x10000 +
		(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_DEBUG(" debug checksume is %x", header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" debug firmware_size is %x", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" debug header->config_size is %x", header->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_DEBUG(" debug header->contains_firmware_id is %x\n", header->contains_firmware_id);
	if( header->contains_firmware_id )
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

static int checkFlashState(struct i2c_client *client)
{
	int ret ;
	int count = 0;
	ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
	while ( (ret != 0x80)&&(count < 8) ) {
		msleep(3); //wait 3ms
		ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
		count++;
	}
	if(count == 8)
		return 1;
	else
		return 0;
}

static int synaptics_fw_check(struct synaptics_ts_data *ts )
{
	int ret;
	uint8_t buf[4];
	uint32_t bootloader_mode;
	int max_y_ic = 0;
	int max_x_ic = 0;
	if(!ts){
		TPD_ERR("%s ts is NULL\n",__func__);
		return -1;
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}

	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_ERR("failed to read product info \n");
		return -1;
	}
	/*read max_x ,max_y*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if (ret < 0) {
		ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
		if(ret < 0 ){
			TPD_ERR("synaptics_rmi4_i2c_write_byte failed for page select\n");
			return -1;
		}
	}

	i2c_smbus_read_i2c_block_data(ts->client, F12_2D_CTRL08, 4, buf);
	max_x_ic = ( (buf[1]<<8)&0xffff ) | (buf[0]&0xffff);
	max_y_ic = ( (buf[3]<<8)&0xffff ) | (buf[2]&0xffff);

	TPD_ERR("max_x = %d,max_y = %d; max_x_ic = %d,max_y_ic = %d\n",ts->max_x,ts->max_y,max_x_ic,max_y_ic);
	if((ts->max_x == 0) ||(ts->max_y ==0 )) {
		ts->max_x = max_x_ic;
		ts->max_y = max_y_ic;
	}
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40;
	TPD_DEBUG("afte fw update,program memory self-check bootloader_mode = 0x%x\n",bootloader_mode);

	if((max_x_ic == 0)||(max_y_ic == 0)||(bootloader_mode == 0x40)) {
		TPD_ERR("Something terrible wrong \n Trying Update the Firmware again\n");
		return -1;
	}
	return 0;
}

static void re_scan_PDT_s3508(struct i2c_client *client)
{
	uint8_t buf[8];
	i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	SynaF34DataBase = buf[3];
	SynaF34QueryBase = buf[0];
	i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	SynaF01DataBase = buf[3];
	SynaF01CommandBase = buf[1];
	i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);

	SynaF34Reflash_BlockNum = SynaF34DataBase;
	SynaF34Reflash_BlockData = SynaF34DataBase + 1;
	SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
	SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
	SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
	SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
	i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
	SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	TPD_DEBUG("SynaFirmwareBlockSize 3310 is %d\n", SynaFirmwareBlockSize);
	SynaF34_FlashControl = SynaF34DataBase + 2;
}

#define F01_BUID_ID_OFFSET 18

static int synaptics_rmi4_query_firmware_id_s3706(
		struct i2c_client *client,
		unsigned int *firmware_id_s3706,
		unsigned char *conifg_id_s3706)
{
	int retval;
	unsigned char build_id[3];
	unsigned int firmware_id;
	unsigned char config_id[32];

	retval = synaptics_rmi4_i2c_read_block(client,
			F01_RMI_QUERY_BASE + F01_BUID_ID_OFFSET,
			sizeof(build_id), build_id);
	if (retval < 0)
		return retval;
	firmware_id = (unsigned int)build_id[0] +
			(unsigned int)build_id[1] * 0x100 +
			(unsigned int)build_id[2] * 0x10000;
	*firmware_id_s3706 = firmware_id;

	retval = synaptics_rmi4_i2c_read_block(client,
			F34_FLASH_CTRL_BASE,
				sizeof(config_id), config_id);
	if (retval < 0)
		return retval;
	conifg_id_s3706 = config_id;
	return 0;
}


static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force)
{
	int ret,j;
	uint8_t buf[8];
	uint8_t bootloder_id[10];
	uint16_t block,firmware,configuration;
	uint32_t CURRENT_FIRMWARE_ID = 0 , FIRMWARE_ID = 0;
	unsigned char  CURRENT_CONFIG_ID[32] = {0};
	const uint8_t *Config_Data = NULL;
	const uint8_t *Firmware_Data = NULL;
	struct image_header_data header;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);
	TPD_DEBUG("%s is called\n", __func__);
	if(!client)
		return -1;
	if (!strncmp(ts->manu_name, "S3706", 5)) {
		synaptics_rmi4_query_firmware_id_s3706(
			client,
			&CURRENT_FIRMWARE_ID,
			CURRENT_CONFIG_ID);
		TPD_ERR("FW_ID:%d--CONFIG_ID %s FW_NAME:%s\n",
				CURRENT_FIRMWARE_ID,
				CURRENT_CONFIG_ID,
				ts->fw_name);
		rmi4_data_s3706->firmware_id = CURRENT_FIRMWARE_ID;
		rmi4_data_s3706->force = force;
		ret = synaptics_rmi4_fwu_init(rmi4_data_s3706, data, client);
		if (ret < 0) {
			return ret;
		}
	} else if (!strncmp(ts->manu_name, "S3718", 5)) {
		Config_Data = data + 0x8f0;
		ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);
		ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL00, 4, buf);
		CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
		if(1 == check_version)
			TPD_ERR("15801CURRENT_FW_ID:%x----, FW_ID:%x----,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
		else
			TPD_ERR("15801CURRENT_FW_ID:%xvB----, FW_ID:%xvB----,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
		//TPD_ERR("synaptics force is %d\n", force);
		if(!force) {
			if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
				return 0;
			}
		}
		ret = fwu_start_reflash(data,client);
		if (ret){
			return -1;
		}
	}else if(!strncmp(ts->manu_name,"s3508",5) || !strncmp(ts->manu_name,"15811",5)){
		parse_header(&header,data);
		if((header.firmware_size + header.config_size + 0x100) > data_len) {
			TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
			return -1;
		}
		Firmware_Data = data + 0x100;
		Config_Data = Firmware_Data + header.firmware_size;
		ret = i2c_smbus_write_byte_data(client, 0xff, 0x0);

		ret = i2c_smbus_read_i2c_block_data(client, F34_FLASH_CTRL00, 4, buf);
		CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
		TPD_ERR("15811CURRENT_FW_ID:%x----, FW_ID:%x----,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
		TPD_ERR("synaptics force is %d\n", force);
		if(!force) {
			if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
				return 0;
			}
		}
		re_scan_PDT_s3508(client);
		block = 16;
		TPD_DEBUG("block is %d \n",block);
		firmware = (header.firmware_size)/16;
		TPD_DEBUG("firmware is %d \n",firmware);
		configuration = (header.config_size)/16;
		TPD_DEBUG("configuration is %d \n",configuration);

		ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));
		TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPD_DEBUG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);

		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x0F);
		msleep(10);
		TPD_DEBUG("attn step 4\n");
		ret=checkFlashState(client);
		if(ret > 0) {
			TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
				return -1;
		}
		ret = i2c_smbus_read_byte_data(client,0x04);
		TPD_DEBUG("The status(device state) is %x\n",ret);
		ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
		TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
		ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);
		/********************get into prog end************/
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPD_DEBUG("ret is %d\n",ret);
		re_scan_PDT_s3508(client);
		i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
		msleep(2500);
		ret = i2c_smbus_read_byte_data(client, SynaF34_FlashControl);
		if(ret != 0x00)
			msleep(2000);
		ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl+1);
		TPDTM_DMESG("The status(erase) is %x\n",ret);
		TPD_ERR("15811update-----------------update------------------update!\n");
		TPD_DEBUG("cnt %d\n",firmware);
		for(j=0; j<firmware; j++) {
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16]);

			i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x02);
			ret=checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}
		//step 7 configure data
		//TPD_ERR("going to flash configuration area\n");
		//TPD_ERR("header.firmware_size is 0x%x\n", header.firmware_size);
		//TPD_ERR("bootloader_size is 0x%x\n", bootloader_size);
		for(j=0;j<configuration;j++) {
			//a)write SynaF34Reflash_BlockNum to access
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
			//b) write data

				i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Config_Data[j*16]);

			//c) issue write
			i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
			//d) wait attn
			ret = checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}
		//step 1 issue reset
		i2c_smbus_write_byte_data(client,SynaF01CommandBase,0X01);
	}else{
		parse_header(&header,data);
		if((header.firmware_size + header.config_size + 0x100) > data_len) {
			TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
			return -1;
		}

		Firmware_Data = data + 0x100;
		Config_Data = Firmware_Data + header.firmware_size;
		ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);

		ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL00, 4, buf);
		CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];

		//TPD_ERR("synaptics force is %d\n", force);
		if(!force) {
			if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
				return 0;
			}
		}
		re_scan_PDT(client);
		block = 16;
		TPD_DEBUG("block is %d \n",block);
		firmware = (header.firmware_size)/16;
		TPD_DEBUG("firmware is %d \n",firmware);
		configuration = (header.config_size)/16;
		TPD_DEBUG("configuration is %d \n",configuration);


		ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));
		TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPDTM_DMESG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);

		synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x0F);
		msleep(10);
		TPD_DEBUG("attn step 4\n");
		ret=checkFlashState(client);
		if(ret > 0) {
			TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
			return -1;
		}
		ret = i2c_smbus_read_byte_data(client,0x04);
		TPD_DEBUG("The status(device state) is %x\n",ret);
		ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
		TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
		ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);
		/********************get into prog end************/
		ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
		TPD_DEBUG("ret is %d\n",ret);
		re_scan_PDT(client);
		i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
		i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
		msleep(2000);
		ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
		TPDTM_DMESG("going to flash firmware area synaF34_FlashControl %d\n",ret);

		TPD_ERR("update-----------------firmware ------------------update!\n");
		TPD_DEBUG("cnt %d\n",firmware);
		for(j=0; j<firmware; j++) {
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16]);
			synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x02);
			ret=checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}
		//step 7 configure data
		//TPD_ERR("going to flash configuration area\n");
		//TPD_ERR("header.firmware_size is 0x%x\n", header.firmware_size);
		//TPD_ERR("bootloader_size is 0x%x\n", bootloader_size);
		TPD_ERR("update-----------------configuration ------------------update!\n");
		for(j=0;j<configuration;j++) {
			//a)write SynaF34Reflash_BlockNum to access
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
			//b) write data
			synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Config_Data[j*16]);
			//c) issue write
			synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x06);
			//d) wait attn
			ret = checkFlashState(client);
			if(ret > 0) {
				TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
				return -1;
			}
		}

		//step 1 issue reset
		synaptics_rmi4_i2c_write_byte(client,SynaF01CommandBase,0x01);
	}
	//step2 wait ATTN
	//delay_qt_ms(1000);
	mdelay(1500);
	synaptics_read_register_map(ts);
	//FW flash check!
	ret =synaptics_fw_check(ts);
	if(ret < 0 ) {
		TPD_ERR("Firmware self check failed\n");
		return -1;
	}
	TPD_ERR("Firmware self check success\n");
	return 0;
}
#ifdef ENABLE_TPEDGE_LIMIT
static void synaptics_tpedge_limitfunc(void)
{
	int limit_mode=0;
	int ret;

	if (version_is_s3508 == 2) {
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x0a;
	} else if (version_is_s3508 == 0) {
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x34;
	} else if (version_is_s3508 == 1) {
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x1b;
	} else
		F51_GRIP_CONFIGURATION = F51_CUSTOM_CTRL_BASE+0x0a;

	TPD_DEBUG("%s line %d F51_GRIP_CONFIGURATION = 0x%x\n",
				__func__, __LINE__, F51_GRIP_CONFIGURATION);
	//msleep(60);
	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x4);
	limit_mode = i2c_smbus_read_byte_data(ts_g->client,
						F51_GRIP_CONFIGURATION);
	TPD_ERR("%s limit_enable =%d,mode:0x%x !\n",
				__func__, limit_enable, limit_mode);
	if(limit_mode){
		i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x4);
		if(0 == limit_enable)
		{
			if(limit_mode & 0x1){
				//TPD_ERR("000 limit_enable:0x%xs  !\n",limit_mode);
				limit_mode = limit_mode & 0xFE;
				ret = i2c_smbus_write_byte_data(ts_g->client,
					F51_GRIP_CONFIGURATION, limit_mode);
			}
		}
		else if(1 == limit_enable)
		{
			if(!(limit_mode & 0x1)){
				//TPD_ERR("111 limit_enable:x%xs  !\n",limit_mode);
				limit_mode = limit_mode | 0x1;
				ret = i2c_smbus_write_byte_data(ts_g->client,
					F51_GRIP_CONFIGURATION, limit_mode);
			}
		}
	}
	i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0);
}

#endif
static int synaptics_soft_reset(struct synaptics_ts_data *ts)
{
	int ret;
	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!\n",__func__);
		return -1;
	}
	touch_disable(ts);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
	if (ret < 0){
		TPD_ERR("reset error ret=%d\n",ret);
	}
	TPD_ERR("%s !!!\n",__func__);
	msleep(100);
	touch_enable(ts);
#ifdef ENABLE_TPEDGE_LIMIT
	synaptics_tpedge_limitfunc();
#endif
	return ret;
}
static void synaptics_hard_reset(struct synaptics_ts_data *ts)
{
	if(ts->reset_gpio > 0)
	{
		gpio_set_value(ts->reset_gpio,0);
		msleep(5);
		gpio_set_value(ts->reset_gpio,1);
		msleep(100);
		TPD_ERR("%s !!!\n",__func__);
	}

}
static int synaptics_parse_dts(struct device *dev, struct synaptics_ts_data *ts)
{
	int rc;
	struct device_node *np;
	int temp_array[2];
	u32 voltage_supply[2];
	u32 current_supply;

	np = dev->of_node;
	ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, &(ts->irq_flags));
	if( ts->irq_gpio < 0 ){
		TPD_DEBUG("ts->irq_gpio not specified\n");
	}

	ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
	if( ts->reset_gpio < 0 ){
		TPD_DEBUG("ts->reset-gpio  not specified\n");
	}
	ts->v1p8_gpio = of_get_named_gpio(np, "synaptics,1v8-gpio", 0);
	if( ts->v1p8_gpio < 0 ){
		TPD_DEBUG("ts->1v8-gpio  not specified\n");
	}

	if (of_property_read_bool(np, "oem,support_1080x2160_tp"))
		ts->support_1080x2160_tp = true;
	else
		ts->support_1080x2160_tp = false;

	if (of_property_read_bool(np, "oem,support_1080x2340_tp"))
		ts->support_1080x2340_tp = true;
	else
		ts->support_1080x2340_tp = false;

	if(of_property_read_bool(np, "oem,support_hw_poweroff"))
		ts->support_hw_poweroff=true;
	else
		ts->support_hw_poweroff=false;

	TPD_ERR("%s ts->support_hw_poweroff =%d\n",__func__,ts->support_hw_poweroff);

	ts->enable2v8_gpio = of_get_named_gpio(np, "synaptics,enable2v8-gpio", 0);
	if( ts->enable2v8_gpio < 0 ){
		TPD_DEBUG("ts->enable2v8_gpio not specified\n");
	}

	rc = of_property_read_u32(np, "synaptics,max-num-support", &ts->max_num);
	if(rc){
		TPD_DEBUG("ts->max_num not specified\n");
		ts->max_num = 10;
	}

	rc = of_property_read_u32_array(np, "synaptics,button-map", button_map, 3);
	if(rc){
		TPD_DEBUG("button-map not specified\n");
		//button_map[0] = 180;
		//button_map[1] = 180;
		//button_map[2] = 2021;
	}
	TPD_DEBUG("synaptics:button map readed is %d %d %d\n", button_map[0], button_map[1], button_map[2]);

	rc = of_property_read_u32_array(np, "synaptics,tx-rx-num", tx_rx_num,2);
	if(rc){
		TPD_ERR("button-map not specified\n");
		TX_NUM =  30;
		RX_NUM =  17;
	}else{
		TX_NUM =  tx_rx_num[0];/*s3706 16 33*/
		RX_NUM =  tx_rx_num[1];
	}
	TPD_ERR("synaptics,tx-rx-num is %d %d \n", TX_NUM,RX_NUM);

	rc = of_property_read_u32_array(np, "synaptics,display-coords", temp_array, 2);
	if(rc){
		TPD_ERR("lcd size not specified\n");
		LCD_WIDTH = 1080;
		LCD_HEIGHT = 1920;
	}else{
		LCD_WIDTH = temp_array[0];
		LCD_HEIGHT = temp_array[1];
	}

	rc = of_property_read_u32_array(np, "synaptics,panel-coords", temp_array, 2);
	if(rc){
		ts->max_x = 1080;
		ts->max_y = 1920;
	}else{
		ts->max_x = temp_array[0];
		ts->max_y = temp_array[1];
	}

	TPDTM_DMESG("synaptic:ts->irq_gpio:%d irq_flags:%u max_num %d\n"\
        ,ts->irq_gpio, ts->irq_flags, ts->max_num);

	/***********power regulator_get****************/
	ts->vdd_2v8 = regulator_get(&ts->client->dev, "vdd_2v8");
	if( IS_ERR(ts->vdd_2v8) ){
		rc = PTR_ERR(ts->vdd_2v8);
		TPD_DEBUG("Regulator get failed vdd rc=%d\n", rc);
	}
	rc = of_property_read_u32(np,"synaptics,avdd-current", &current_supply);
	if (rc < 0) {
		TPD_ERR("%s: Failed to get regulator avdd current\n", __func__);

	} else {
		ts->regulator_avdd_current = current_supply;
		TPD_ERR("%s: avdd current = %d\n",
			__func__, ts->regulator_avdd_current);
		rc = regulator_set_load(ts->vdd_2v8,
			ts->regulator_avdd_current);
		if (rc < 0)
			TPD_ERR("%s: Failed to set avdd\n", __func__);
	}


	rc = of_property_read_u32_array(np, "synaptics,avdd-voltage", voltage_supply, 2);
	if (rc < 0) {
		TPD_ERR("%s: Failed to get regulator vdd voltage\n",__func__);
	} else {
		ts->regulator_avdd_vmin = voltage_supply[0];
		ts->regulator_avdd_vmax = voltage_supply[1];
		TPD_ERR("%s:avdd_vmin=%d,avdd_vmax=%d\n", __func__,
			ts->regulator_avdd_vmin, ts->regulator_avdd_vmax);

		rc = regulator_set_voltage(ts->vdd_2v8,
			ts->regulator_avdd_vmin, ts->regulator_avdd_vmax);
		if (rc < 0)
			TPD_ERR("%s:Failed to set avdd\n", __func__);
	}


	ts->vcc_i2c_1v8 = regulator_get(&ts->client->dev, "vcc_i2c_1v8");
	if( IS_ERR(ts->vcc_i2c_1v8) ){
		rc = PTR_ERR(ts->vcc_i2c_1v8);
		TPD_DEBUG("Regulator get failed vcc_i2c rc=%d\n", rc);
	}

	rc = of_property_read_u32(np,"synaptics,vdd-current", &current_supply);
	if (rc < 0) {
		TPD_ERR("%s: Failed to get regulator vdd current\n", __func__);
	} else {
		ts->regulator_vdd_current = current_supply;
		TPD_ERR("%s: vdd current = %d\n", __func__,
				ts->regulator_vdd_current);
		rc = regulator_set_load(ts->vcc_i2c_1v8,
			ts->regulator_vdd_current);
		if (rc < 0)
			TPD_ERR("%s: Failed to set vdd\n", __func__);
	}


	rc = of_property_read_u32_array(np, "synaptics,vdd-voltage", voltage_supply, 2);
	if (rc < 0) {
		TPD_ERR("%s: Failed to get regulator vdd voltage\n", __func__);
	} else {
		ts->regulator_vdd_vmin = voltage_supply[0];
		ts->regulator_vdd_vmax = voltage_supply[1];
		TPD_ERR("%s:vdd_vmin=%d,vdd_vmax=%d\n", __func__,
			ts->regulator_vdd_vmin, ts->regulator_vdd_vmax);

		rc = regulator_set_voltage(ts->vcc_i2c_1v8,
				ts->regulator_vdd_vmin, ts->regulator_vdd_vmax);
		if (rc < 0)
			TPD_ERR("%s:Failed to set vdd\n", __func__);
	}



	if( ts->reset_gpio > 0){
		if( gpio_is_valid(ts->reset_gpio) ){
			rc = gpio_request(ts->reset_gpio, "tp-s3320-reset");
			if(rc){
				TPD_ERR("unable to request reset_gpio [%d]\n", ts->reset_gpio);
			}
			gpio_direction_output(ts->reset_gpio, 0);
		}
	}
	if( ts->v1p8_gpio > 0){
		if( gpio_is_valid(ts->v1p8_gpio) ){
			rc = gpio_request(ts->v1p8_gpio, "tp-s3320-1v8");
			if(rc){
				TPD_ERR("unable to request v1p8_gpio [%d]\n", ts->v1p8_gpio);
			}
		}
	}

	if( ts->enable2v8_gpio > 0){
		if( gpio_is_valid(ts->enable2v8_gpio) ){
			rc = gpio_request(ts->enable2v8_gpio, "rmi4-enable2v8-gpio");
			if(rc)
				TPD_ERR("unable to request enable2v8_gpio [%d]\n", ts->enable2v8_gpio);

		}
	}

	return rc;
}

static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->pinctrl = devm_pinctrl_get((ts->dev));
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		retval = PTR_ERR(ts->pinctrl);
        TPD_ERR("%s pinctrl error!\n",__func__);
		goto err_pinctrl_get;
	}

	ts->pinctrl_state_active
		= pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
		retval = PTR_ERR(ts->pinctrl_state_active);
        TPD_ERR("%s pinctrl state active error!\n",__func__);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_state_suspend
		= pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
		retval = PTR_ERR(ts->pinctrl_state_suspend);
        TPD_ERR("%s pinctrl state suspend error!\n",__func__);
		goto err_pinctrl_lookup;
	}
	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ts->pinctrl);
err_pinctrl_get:
	ts->pinctrl = NULL;
	return retval;
}

#ifdef SUPPORT_VIRTUAL_KEY
#define VK_KEY_X    180
#define VK_CENTER_Y 2020//2260
#define VK_WIDTH    170
#define VK_HIGHT    200
static ssize_t vk_syna_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	int len ;

	len =  sprintf(buf,
	    __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT)  ":%d:%d:%d:%d"
	    ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":%d:%d:%d:%d"
	    ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":%d:%d:%d:%d" "\n",
	    VK_KEY_X,   VK_CENTER_Y, VK_WIDTH, VK_HIGHT,
	    VK_KEY_X*3, VK_CENTER_Y, VK_WIDTH, VK_HIGHT,
	    VK_KEY_X*5, VK_CENTER_Y, VK_WIDTH, VK_HIGHT);

	return len ;
}

static struct kobj_attribute vk_syna_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &vk_syna_show,
};

static struct attribute *syna_properties_attrs[] = {
    &vk_syna_attr.attr,
    NULL
};

static struct attribute_group syna_properties_attr_group = {
    .attrs = syna_properties_attrs,
};
static int synaptics_ts_init_virtual_key(struct synaptics_ts_data *ts )
{
	int ret = 0;

	/* virtual keys */
	if(ts->properties_kobj)
		return 0 ;
	ts->properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (ts->properties_kobj)
		ret = sysfs_create_group(ts->properties_kobj, &syna_properties_attr_group);

	if (!ts->properties_kobj || ret)
		printk("%s: failed to create board_properties\n", __func__);
	/* virtual keys */
	return ret;
}
#endif

static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_SYNAPTIC_RED
	struct remotepanel_data *premote_data = NULL;
#endif
	struct synaptics_ts_data *ts = NULL;
	int ret = -1;
	uint8_t buf[8];
	unsigned long int  CURRENT_FIRMWARE_ID = 0;
	uint32_t bootloader_mode;
	//uint32_t bootmode;

	TPD_ERR("%s  is called\n",__func__);

	ts = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if( ts == NULL ) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->dev = &client->dev;
	ts->loading_fw = false;
	ts->support_ft = true;
	ts_g = ts;
	get_tp_base = 0;

	synaptics_parse_dts(&client->dev, ts);
	ts->project_version = 0x03;
	if (of_property_read_bool(ts->dev->of_node, "oem,fajta"))
		ts->project_version = 0x03;
	/***power_init*****/
	ret = tpd_power(ts, 1);
	if( ret < 0 )
		TPD_ERR("regulator_enable is called\n");
	ret = synaptics_dsx_pinctrl_init(ts);
	if (!ret && ts->pinctrl) {
		ret = pinctrl_select_state(ts->pinctrl,
                ts->pinctrl_state_active);
	}

	msleep(100);//after power on tp need sometime from bootloader to ui mode
	mutex_init(&ts->mutex);
	mutex_init(&ts->mutexreport);
	atomic_set(&ts->irq_enable,0);

	ts->is_suspended = 0;
	atomic_set(&ts->is_stop,0);
	spin_lock_init(&ts->lock);
	/*****power_end*********/
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA) ){
		TPD_ERR("%s [ERR]need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
	if( ret < 0 ) {
		ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
		if( ret < 0 ) {
		#ifdef SUPPORT_VIRTUAL_KEY
                        virtual_key_enable = 0;//if touch is no valid report key
                #endif
                        TPD_ERR("tp is no exist!\n");
			goto err_check_functionality_failed;
		}
	}

	ts->i2c_device_test = ret;
	rmi4_data_s3706 = kzalloc(sizeof(struct synaptics_rmi4_data),
							GFP_KERNEL);
	if (rmi4_data_s3706 == NULL) {
		TPD_ERR("Request rmi4_data_s3706 failed\n");
		goto err_check_functionality_failed;
	}
	rmi4_data_s3706->reset_device = synaptics_rmi4_reset_device;
	mutex_init(&(rmi4_data_s3706->rmi4_reset_mutex));
	mutex_init(&(rmi4_data_s3706->rmi4_report_mutex));
	mutex_init(&(rmi4_data_s3706->rmi4_io_ctrl_mutex));
	mutex_init(&(rmi4_data_s3706->rmi4_exp_init_mutex));
	mutex_init(&(rmi4_data_s3706->rmi4_irq_enable_mutex));

	synaptics_read_register_map(ts);
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);

	bootloader_mode = bootloader_mode&0x40;
	TPD_ERR("before fw update bootloader_mode[0x%x]\n", bootloader_mode);

	memset(ts->fw_name, 0, TP_FW_NAME_MAX_LEN);
	memset(ts->test_limit_name, 0, TP_FW_NAME_MAX_LEN);

	//sprintf(ts->manu_name, "TP_SYNAPTICS");
	synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY11,\
        sizeof(ts->manu_name), ts->manu_name);
	if (!strncmp(ts->manu_name,"S3718",5)){
		strcpy(ts->fw_name,"tp/fw_synaptics_15801b.img");
		version_is_s3508 = 0;
	} else if (!strncmp(ts->manu_name, "S3706", 5)) {
		strlcpy(ts->fw_name, "tp/fw_synaptics_17819.img",
			sizeof(ts->fw_name));
		version_is_s3508 = 2;/*for s3706*/
		TX_NUM =  16;/*s3706 16 33*/
		RX_NUM =  33;
		if (ts->support_1080x2340_tp) {
			strlcpy(ts->fw_name, "tp/fw_synaptics_18801.img",
			    sizeof(ts->fw_name));
			ts->max_x = 1080;
			ts->max_y = 2340;
		} else {
			ts->max_x = 1080;
			ts->max_y = 2280;
		}
		F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x06;/*0x07 for s3508*/
		F12_2D_CTRL27 = F12_2D_CTRL_BASE + 0x09;
	} else {
		if (ts->support_1080x2160_tp)
			strlcpy(ts->fw_name, "tp/fw_synaptics_17801.img",
			    sizeof(ts->fw_name));
		else
			strlcpy(ts->fw_name, "tp/fw_synaptics_16859.img",
			    sizeof(ts->fw_name));

		version_is_s3508 = 1;
	}
	strcpy(ts->test_limit_name,"tp/14049/14049_Limit_jdi.img");

	TPD_DEBUG("synatpitcs fw_name=%s\n",
		ts->fw_name);
	TPD_DEBUG("synatpitcs manu_name:%s\n",
		ts->manu_name);

	synaptics_rmi4_i2c_read_block(ts->client,
		F34_FLASH_CTRL00, 8, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) |
						(buf[2]<<8) | buf[3];
	if (version_is_s3508 == 2)
		CURRENT_FIRMWARE_ID = CURRENT_FIRMWARE_ID << 32 |
				((buf[4]<<24) | (buf[5]<<16) |
				(buf[6]<<8) | buf[7]);
	TPD_ERR("CURRENT_FIRMWARE_ID = 0x%lx\n",
			CURRENT_FIRMWARE_ID);
	TP_FW = CURRENT_FIRMWARE_ID;
	snprintf(ts->fw_id,  20, "0x%lx", TP_FW);

	//push_component_info(TOUCH_KEY, ts->fw_id, ts->manu_name);
	//push_component_info(TP, ts->fw_id, ts->manu_name);

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if( !synaptics_wq ){
		ret = -ENOMEM;
		goto exit_createworkqueue_failed;
	}
	INIT_DELAYED_WORK(&ts->speed_up_work,speedup_synaptics_resume);


	memset(baseline,0,sizeof(baseline));
	get_base_report = create_singlethread_workqueue("get_base_report");
	if( !get_base_report ){
		ret = -ENOMEM;
		goto exit_createworkqueue_failed;
	}
	INIT_DELAYED_WORK(&ts->base_work,tp_baseline_get_work);

	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPD_ERR("synaptics_init_panel failed\n");
	}

	//Detect whether TP FW is error, max_x,max_y may be incoorect while it has been damaged!
	ret = synaptics_fw_check(ts);
	if(ret < 0 ) {
		force_update = 1;
		TPD_ERR("This FW need to be updated!\n");
	} else {
		force_update = 0;
	}
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if( ret < 0 ) {
		TPD_ERR(" synaptics_ts_probe: disable interrupt failed\n");
	}
	ret = synaptics_soft_reset(ts);
	if (ret < 0){
		TPD_ERR("%s faile to reset device\n",__func__);
	}
	ret = synaptics_input_init(ts);
	if(ret < 0) {
		TPD_ERR("synaptics_input_init failed!\n");
	}
#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret)
		TPD_ERR("Unable to register fb_notifier: %d\n", ret);
#elif defined(CONFIG_MSM_RDM_NOTIFY)
	ts->msm_drm_notif.notifier_call = msm_drm_notifier_callback;
	ret = msm_drm_register_client(&ts->msm_drm_notif);
	if (ret)
		TPD_ERR("Unable to register msm_drm_notifier: %d\n", ret);
#endif



#ifndef TPD_USE_EINT
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#ifdef TPD_USE_EINT
	/****************
	  shoud set the irq GPIO
	 *******************/
	if (gpio_is_valid(ts->irq_gpio)) {
        /* configure touchscreen irq gpio */
        ret = gpio_request(ts->irq_gpio,"tp-s3320-irq");
        if (ret) {
            TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
        }
        ret = gpio_direction_input(ts->irq_gpio);
        msleep(50);
        ts->irq = gpio_to_irq(ts->irq_gpio);
	}
	TPD_ERR("synaptic:ts->irq is %d\n",ts->irq);

	ret = request_threaded_irq(ts->irq, NULL,
			synaptics_irq_thread_fn,
			ts->irq_flags | IRQF_ONESHOT,
			TPD_DEVICE, ts);
	if(ret < 0)
		TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
	msleep(5);
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret < 0)
		TPD_ERR("%s enable interrupt error ret=%d\n",__func__,ret);
#endif

	if (device_create_file(&client->dev, &dev_attr_test_limit)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	TPD_DEBUG("synaptics_ts_probe: going to create files--tp_fw_update\n");
	if (device_create_file(&client->dev, &dev_attr_tp_fw_update)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if (device_create_file(&client->dev, &dev_attr_tp_doze_time)) {
		TPDTM_DMESG("device_create_file failt\n");
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_debug_log)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_baseline_image_with_cbc)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_baseline_image)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_delta_image)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if (ts->project_version == 0x03) {
		if (device_create_file(&client->dev,
			&dev_attr_tp_gesture_touch_hold)) {
			TPDTM_DMESG("tp_gesture_touch_hold failt\n");
			goto exit_init_failed;
		}
		if (device_create_file(&client->dev, &dev_attr_fp_irq)) {
			TPDTM_DMESG("driver_create_file fail fp_irq\n");
			goto exit_init_failed;
		}
		ts->fp_up_down = 0;
		ts->en_up_down = 0;
		ts->fp_aod_cnt = 0;
		ts->unlock_succes = 0;
	}

#ifdef SUPPORT_VIRTUAL_KEY
	synaptics_ts_init_virtual_key(ts);
#endif
#ifdef CONFIG_SYNAPTIC_RED
	premote_data = remote_alloc_panel_data();
	if(premote_data) {
		premote_data->client 		= client;
		premote_data->input_dev		= ts->input_dev;
		premote_data->pmutex		= &ts->mutex;
		premote_data->irq_gpio 		= ts->irq_gpio;
		premote_data->irq			= client->irq;
		premote_data->enable_remote = &(ts->enable_remote);
		register_remote_device(premote_data);

	}
#endif
	init_synaptics_proc();
	TPDTM_DMESG("synaptics_ts_probe 3203: normal end\n");
/*
	bootmode = get_boot_mode();
	TPD_ERR("synaptics bootmode %d  !\n", bootmode);
	if ((bootmode == MSM_BOOT_MODE__FACTORY)
		|| (bootmode == MSM_BOOT_MODE__RF)
		|| (bootmode == MSM_BOOT_MODE__WLAN)) {
		touch_disable(ts);
		TPD_ERR("synaptics ftm mode disable int \n");
		return 0;
	}
*/
	return 0;

exit_init_failed:
	free_irq(client->irq,ts);
exit_createworkqueue_failed:
	destroy_workqueue(synaptics_wq);
	synaptics_wq = NULL;
	destroy_workqueue(synaptics_report);
	synaptics_report = NULL;
	destroy_workqueue(get_base_report);
	get_base_report = NULL;

err_check_functionality_failed:
	tpd_power(ts, 0);
	/*add for morgan for if no tp no pull gpio*/
	if (ts->pinctrl) {
		ret = pinctrl_select_state(ts->pinctrl,
				ts->pinctrl_state_suspend);
	}
err_alloc_data_failed:
	tpd_i2c_driver.driver.pm=NULL;
	kfree(ts);
	ts = NULL;
	kfree(rmi4_data_s3706);
	rmi4_data_s3706 = NULL;
	ts_g = NULL;
	TPD_ERR("synaptics_ts_probe: not normal end\n");
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	int attr_count;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	TPD_ERR("%s is called\n",__func__);
#ifdef CONFIG_SYNAPTIC_RED
	unregister_remote_device();
#endif

#if defined(CONFIG_FB)
	if( fb_unregister_client(&ts->fb_notif) )
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_MSM_RDM_NOTIFY)
	if (msm_drm_unregister_client(&ts->msm_drm_notif))
		dev_err(&client->dev, "Error occurred while unregistering msm_drm_notifier.\n");
#endif

#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif

	for(attr_count = 0; attr_count < ARRAY_SIZE(attrs_oem); attr_count++){
		sysfs_remove_file(&ts->input_dev->dev.kobj, &attrs_oem[attr_count].attr);
	}
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
	tpd_power(ts,0);
	return 0;
}

static int synaptics_ts_suspend(struct device *dev)
{
	int ret,i;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		return -1;
	}
	TPD_DEBUG("%s enter\n", __func__);

	if (ts->pre_btn_state & 0x01){//if press key and suspend release key
		ts->pre_btn_state &= 0x02;//clear bit0
		input_report_key(ts->input_dev, OEM_KEY_BACK, 0);
		input_sync(ts->input_dev);
	}else if (ts->pre_btn_state & 0x02){
		ts->pre_btn_state &= 0x01;//clear bit1
		input_report_key(ts->input_dev, OEM_KEY_APPSELECT, 0);
		input_sync(ts->input_dev);
	}
	for (i = 0; i < ts->max_num; i++)
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);

#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif

#ifdef SUPPORT_GESTURE
	if( ts->gesture_enable ){
		atomic_set(&ts->is_stop,0);
		if (mutex_trylock(&ts->mutex)){
			touch_enable(ts);
			synaptics_enable_interrupt_for_gesture(ts, 1);
			mutex_unlock(&ts->mutex);
			TPD_ERR("enter gesture mode\n");
		}
		set_doze_time(2);
		//just for fajita
		if (ts->project_version == 0x03)
			mutex_lock(&ts->mutex);
			tp_single_tap_en(ts, true);
			mutex_unlock(&ts->mutex);
	}
	else{
		if (!ts->loading_fw) {
			//loading_fw tp cannot enter sleep mode;
			ret = synaptics_mode_change(0x01);
			//when gesture disable TP sleep eary
			if (ret < 0) {
				TPD_ERR("%s line%d ERROR %d!\n",
					__func__, __LINE__, ret);
			}
		}
	}
#endif
	TPD_DEBUG("%s normal end\n", __func__);
	return 0;
}

static void speedup_synaptics_resume(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = ts_g;

/*#ifdef SUPPORT_SLEEP_POWEROFF*/
	TPD_DEBUG("%s enter!\n", __func__);
	if (ts->support_hw_poweroff) {
		if (ts->gesture_enable == 0) {
		ret = tpd_power(ts, 1);
		if (ret < 0)
		TPD_ERR("%s power on err\n", __func__);
		if (ts->pinctrl) {
			ret = pinctrl_select_state(ts->pinctrl,
				ts->pinctrl_state_active);
		}
		}
	}
	TPD_DEBUG("%s end!\n", __func__);
/*#endif*/
}

static int synaptics_ts_resume(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	int i;

	TPD_DEBUG("%s enter!\n", __func__);

	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!\n",__func__);
		return -1;
	}

	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		goto ERR_RESUME;
	}
	for (i = 0; i < ts->max_num; i++)
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);

    //touch_enable(ts);

	TPD_DEBUG("%s:normal end!\n", __func__);
ERR_RESUME:
	return 0;
}

static int synaptics_i2c_suspend(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	TPD_DEBUG("%s: is called\n", __func__);
	if (ts->gesture_enable == 1){
		/*enable gpio wake system through intterrupt*/
		enable_irq_wake(ts->irq);
	}
//#ifdef SUPPORT_SLEEP_POWEROFF
	if(ts->loading_fw) {
		TPD_ERR("FW is updating while suspending");
		return -1;
	}
    if(ts->support_hw_poweroff && (ts->gesture_enable == 0)){
	    ret = tpd_power(ts,0);
	    if (ret < 0)
	        TPD_ERR("%s power off err\n",__func__);
		if (ts->pinctrl){
			ret = pinctrl_select_state(ts->pinctrl,
					ts->pinctrl_state_suspend);
		}
	}
//#endif
	return 0;
}

static int synaptics_i2c_resume(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	TPD_DEBUG("%s is called\n", __func__);
    queue_delayed_work(synaptics_wq,&ts->speed_up_work, msecs_to_jiffies(1));
	if (ts->gesture_enable == 1){
		/*disable gpio wake system through intterrupt*/
		disable_irq_wake(ts->irq);
	}
	return 0;
}

static int synaptics_mode_change(int mode)
{
	int ret;
	int tmp_mode;
	tmp_mode = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_CTRL00);
	tmp_mode = tmp_mode & 0xF8;//bit0-bit2(mode)
	tmp_mode = tmp_mode | mode;
	if (ts_g->changer_connet)
		tmp_mode = tmp_mode | 0x20;//set bit6(change status)
	else
		tmp_mode = tmp_mode & 0xDF;//clear bit6(change status)
	TPD_DEBUG("%s: set TP to mode[0x%x]\n", __func__,tmp_mode);
	ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, tmp_mode);
	if(ret<0)
		TPD_ERR("%s: set dose mode[0x%x] err!!\n", __func__,tmp_mode);
	return ret;
}
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	static int gesture_flag;

	struct synaptics_ts_data *ts = container_of(self, struct synaptics_ts_data, fb_notif);

	if(FB_EARLY_EVENT_BLANK != event && FB_EVENT_BLANK != event)
	return 0;
	if((evdata) && (evdata->data) && (ts) && (ts->client))
	{
		blank = evdata->data;
		TPD_DEBUG("%s blank[%d],event[0x%lx]\n", __func__,*blank,event);

		if ((*blank == FB_BLANK_UNBLANK)
		 && (event == FB_EARLY_EVENT_BLANK)) {
			if (gesture_flag == 1) {
				ts->gesture_enable = 0;
				DouTap_gesture = 0;
				synaptics_enable_interrupt_for_gesture(ts, 0);
				set_doze_time(1);
				gesture_flag = 0;
			} else if (gesture_flag == 2) {
				DouTap_gesture = 0;
				gesture_flag = 0;
			}
			if (ts->is_suspended == 1)
			{
				TPD_DEBUG("%s going TP resume start\n", __func__);
				ts->is_suspended = 0;
				queue_delayed_work(get_base_report, &ts->base_work,msecs_to_jiffies(80));
				synaptics_ts_resume(&ts->client->dev);
				//atomic_set(&ts->is_stop,0);
				TPD_DEBUG("%s going TP resume end\n", __func__);
			}
		} else if (*blank == FB_BLANK_NORMAL) {
			if (ts->gesture_enable == 0) {
				DouTap_gesture = 1;
				ts->gesture_enable = 1;
				i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
				synaptics_mode_change(0x80);
				synaptics_ts_suspend(&ts->client->dev);
				gesture_flag = 1;
			} else if ((ts->gesture_enable == 1) && (DouTap_gesture == 0)) {
				DouTap_gesture = 1;
				gesture_flag = 2;
			}
		}else if( *blank == FB_BLANK_POWERDOWN && (event == FB_EARLY_EVENT_BLANK )){
			if (gesture_flag == 1) {
				ts->gesture_enable = 0;
				DouTap_gesture = 0;
				synaptics_enable_interrupt_for_gesture(ts, 0);
				set_doze_time(1);
				ts->is_suspended = 0;
				gesture_flag = 0;
			} else if (gesture_flag == 2) {
				DouTap_gesture = 0;
				ts->is_suspended = 0;
				gesture_flag = 0;
			}
			if (ts->is_suspended == 0){
				TPD_DEBUG("%s : going TP suspend start\n", __func__);
				ts->is_suspended = 1;
				atomic_set(&ts->is_stop,1);
				if (!(ts->gesture_enable))
					touch_disable(ts);
				synaptics_ts_suspend(&ts->client->dev);
				TPD_DEBUG("%s : going TP suspend end\n", __func__);
			}
		}
	}
	return 0;
}
#elif defined(CONFIG_MSM_RDM_NOTIFY)
static int msm_drm_notifier_callback(
	struct notifier_block *self, unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;

	struct synaptics_ts_data *ts = container_of(
		self, struct synaptics_ts_data, msm_drm_notif);

	if (event != MSM_DRM_EARLY_EVENT_BLANK
		&& MSM_DRM_EVENT_BLANK != event)
	return 0;
	/*add for morgan for EID447*/
	if (evdata->id != MSM_DRM_PRIMARY_DISPLAY)
	return 0;
	if ((evdata) && (evdata->data) && (ts) && (ts->client)) {
		blank = evdata->data;
		TPD_ERR("%s blank[%d],event[0x%lx],evdata->id[%d]\n",
			__func__, *blank, event, evdata->id);

		if ((*blank == MSM_DRM_BLANK_UNBLANK_CUST)
		&& (event == MSM_DRM_EARLY_EVENT_BLANK)) {
			if (ts->is_suspended == 1) {
				TPD_DEBUG("%s TP resume start\n", __func__);
				ts->is_suspended = 0;
				queue_delayed_work(get_base_report,
					&ts->base_work, msecs_to_jiffies(80));
				synaptics_ts_resume(&ts->client->dev);
				//atomic_set(&ts->is_stop,0);
				TPD_DEBUG("%sTP resume end\n", __func__);
			}
		} else if (((*blank == MSM_DRM_BLANK_POWERDOWN_CUST)
			&& (event == MSM_DRM_EARLY_EVENT_BLANK))
			|| (*blank == MSM_DRM_BLANK_NORMAL)) {
			if (ts->is_suspended == 0) {
				TPD_DEBUG("%s:TP suspend start\n", __func__);
				ts->is_suspended = 1;
				atomic_set(&ts->is_stop, 1);
				cancel_delayed_work_sync(&ts->base_work);
				flush_workqueue(get_base_report);
				if (!(ts->gesture_enable))
					touch_disable(ts);
				synaptics_ts_suspend(&ts->client->dev);
				TPD_DEBUG("%s:TP suspend end\n", __func__);
			}
		}
	}
	return 0;


}
#endif

static int __init tpd_driver_init(void)
{
	TPD_ERR("%s enter\n", __func__);
	if( i2c_add_driver(&tpd_i2c_driver)!= 0 ){
		TPD_ERR("unable to add i2c driver.\n");
		return -1;
	}
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	i2c_del_driver(&tpd_i2c_driver);
	if(synaptics_wq ){
		destroy_workqueue(synaptics_wq);
		synaptics_wq = NULL;
	}
	return;
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("Synaptics S3203 Touchscreen Driver");
MODULE_LICENSE("GPL");
