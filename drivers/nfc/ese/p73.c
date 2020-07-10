/*
 * Copyright (C) 2012-2014 NXP Semiconductors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 /**
 * \addtogroup spi_driver
 *
 * @{ */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include "p73.h"
#include "../pn5xx.h"
#include <linux/oneplus/boot_mode.h>
extern long  pn544_dev_ioctl(struct file *filp, unsigned int cmd,
        unsigned long arg);

/* Device driver's configuration macro */
/* Macro to configure poll/interrupt based req*/
//#define P61_IRQ_ENABLE

/* Macro to configure Hard/Soft reset to P61 */
//#define P61_HARD_RESET

/* Macro to define SPI clock frequency */

//#define P61_SPI_CLOCK_7Mzh
#undef P61_SPI_CLOCK_7Mzh
#undef P61_SPI_CLOCK_10Mzh
#define P61_SPI_CLOCK_8Mzh

#ifdef P61_SPI_CLOCK_7Mzh
#define P61_SPI_CLOCK     7000000L;
#else
#ifdef P61_SPI_CLOCK_8Mzh
#define P61_SPI_CLOCK     8000000L;
#else
#ifdef P61_SPI_CLOCK_10Mzh
#define P61_SPI_CLOCK     10000000L;
#else
#define P61_SPI_CLOCK     4000000L;
#endif
#endif
#endif

/* size of maximum read/write buffer supported by driver */
#define MAX_BUFFER_SIZE   258U

/* Different driver debug lever */
enum P61_DEBUG_LEVEL {
    P61_DEBUG_OFF,
    P61_FULL_DEBUG
};

#define READ_THROUGH_PUT 0x01
#define WRITE_THROUGH_PUT 0x02
#define MXAX_THROUGH_PUT_TIME 999000L
/* Variable to store current debug level request by ioctl */
static unsigned char debug_level;

#define P61_DBG_MSG(msg...)  \
        switch(debug_level)      \
        {                        \
        case P61_DEBUG_OFF:      \
        break;                 \
        case P61_FULL_DEBUG:     \
        printk(KERN_INFO "[NXP-P61] :  " msg); \
        break; \
        default:                 \
        printk(KERN_ERR "[NXP-P61] :  Wrong debug level %d", debug_level); \
        break; \
        } \

#define P61_ERR_MSG(msg...) printk(KERN_ERR "[NFC-P61] : " msg );

/* Device specific macro and structure */
struct p61_dev {
    wait_queue_head_t read_wq; /* wait queue for read interrupt */
    struct mutex read_mutex; /* read mutex */
    struct mutex write_mutex; /* write mutex */
    struct spi_device *spi;  /* spi device structure */
    struct miscdevice p61_device; /* char device as misc driver */
    unsigned int rst_gpio; /* SW Reset gpio */
    unsigned int irq_gpio; /* P61 will interrupt DH for any ntf */
    bool irq_enabled; /* flag to indicate irq is used */
    unsigned char enable_poll_mode; /* enable the poll mode */
    spinlock_t irq_enabled_lock; /*spin lock for read irq */
    /* read buffer */
    size_t kbuflen;
    u8 *kbuf;
};

/* T==1 protocol specific global data */
const unsigned char SOF = 0xA5u;
struct p61_through_put {
    struct timeval rstart_tv;
    struct timeval rstop_tv;
    struct timeval wstart_tv;
    struct timeval wstop_tv;
    unsigned long total_through_put_wbytes;
    unsigned long total_through_put_rbytes;
    unsigned long  total_through_put_rtime;
    unsigned long total_through_put_wtime;
    bool enable_through_put_measure;
};
static struct p61_through_put p61_through_put_t;

static void p61_start_throughput_measurement(unsigned int type);
static void p61_stop_throughput_measurement(unsigned int type, int no_of_bytes);

static void p61_start_throughput_measurement(unsigned int type)
{
    if (type == READ_THROUGH_PUT)
    {
        memset(&p61_through_put_t.rstart_tv, 0x00, sizeof(struct timeval));
        do_gettimeofday(&p61_through_put_t.rstart_tv);
    }
    else if (type == WRITE_THROUGH_PUT)
    {
        memset(&p61_through_put_t.wstart_tv, 0x00, sizeof(struct timeval));
        do_gettimeofday(&p61_through_put_t.wstart_tv);

    }
    else
    {
        P61_DBG_MSG(KERN_ALERT " p61_start_throughput_measurement: wrong type = %d", type);
    }

}
static void p61_stop_throughput_measurement(unsigned int type, int no_of_bytes)
{
    if (type == READ_THROUGH_PUT)
    {
        memset(&p61_through_put_t.rstop_tv, 0x00, sizeof(struct timeval));
        do_gettimeofday(&p61_through_put_t.rstop_tv);
        p61_through_put_t.total_through_put_rbytes += no_of_bytes;
        p61_through_put_t.total_through_put_rtime += (p61_through_put_t.rstop_tv.tv_usec -
                                                     p61_through_put_t.rstart_tv.tv_usec) +
                                                     ((p61_through_put_t.rstop_tv.tv_sec -
                                                     p61_through_put_t.rstart_tv.tv_sec) * 1000000);

        if(p61_through_put_t.total_through_put_rtime >= MXAX_THROUGH_PUT_TIME)
        {
            printk(KERN_ALERT " **************** Read Throughput: **************");
            printk(KERN_ALERT " No of Read Bytes = %ld", p61_through_put_t.total_through_put_rbytes);
            printk(KERN_ALERT " Total Read Time (uSec) = %ld", p61_through_put_t.total_through_put_rtime);
            p61_through_put_t.total_through_put_rbytes = 0;
            p61_through_put_t.total_through_put_rtime = 0;
            printk(KERN_ALERT " **************** Read Throughput: **************");
        }
    printk(KERN_ALERT " No of Read Bytes = %ld", p61_through_put_t.total_through_put_rbytes);
    printk(KERN_ALERT " Total Read Time (uSec) = %ld", p61_through_put_t.total_through_put_rtime);
    }
    else if (type == WRITE_THROUGH_PUT)
    {
        memset(&p61_through_put_t.wstop_tv, 0x00, sizeof(struct timeval));
        do_gettimeofday(&p61_through_put_t.wstop_tv);
        p61_through_put_t.total_through_put_wbytes += no_of_bytes;
        p61_through_put_t.total_through_put_wtime += (p61_through_put_t.wstop_tv.tv_usec -
                                                     p61_through_put_t.wstart_tv.tv_usec) +
                                                     ((p61_through_put_t.wstop_tv.tv_sec -
                                                     p61_through_put_t.wstart_tv.tv_sec) * 1000000);

        if(p61_through_put_t.total_through_put_wtime >= MXAX_THROUGH_PUT_TIME)
        {
            printk(KERN_ALERT " **************** Write Throughput: **************");
            printk(KERN_ALERT " No of Write Bytes = %ld", p61_through_put_t.total_through_put_wbytes);
            printk(KERN_ALERT " Total Write Time (uSec) = %ld", p61_through_put_t.total_through_put_wtime);
            p61_through_put_t.total_through_put_wbytes = 0;
            p61_through_put_t.total_through_put_wtime = 0;
            printk(KERN_ALERT " **************** WRITE Throughput: **************");
        }
        printk(KERN_ALERT " No of Write Bytes = %ld", p61_through_put_t.total_through_put_wbytes);
    printk(KERN_ALERT " Total Write Time (uSec) = %ld", p61_through_put_t.total_through_put_wtime);
    }
    else
    {
        printk(KERN_ALERT " p61_stop_throughput_measurement: wrong type = %d", type);
    }
}
/**
 * \ingroup spi_driver
 * \brief Called from SPI LibEse to initilaize the P61 device
 *
 * \param[in]       struct inode *
 * \param[in]       struct file *
 *
 * \retval 0 if ok.
 *
*/

static int p61_dev_open(struct inode *inode, struct file *filp) {

    struct p61_dev
    *p61_dev = container_of(filp->private_data,
            struct p61_dev,
            p61_device);

    filp->private_data = p61_dev;
    P61_DBG_MSG(
            "%s : Major No: %d, Minor No: %d\n", __func__, imajor(inode), iminor(inode));

    return 0;
}

/**
 * \ingroup spi_driver
 * \brief To configure the P61_SET_PWR/P61_SET_DBG/P61_SET_POLL
 * \n         P61_SET_PWR - hard reset (arg=2), soft reset (arg=1)
 * \n         P61_SET_DBG - Enable/Disable (based on arg value) the driver logs
 * \n         P61_SET_POLL - Configure the driver in poll (arg = 1), interrupt (arg = 0) based read operation
 * \param[in]       struct file *
 * \param[in]       unsigned int
 * \param[in]       unsigned long
 *
 * \retval 0 if ok.
 *
*/

static long p61_dev_ioctl(struct file *filp, unsigned int cmd,
        unsigned long arg)
{
    int ret = 0;
    struct p61_dev *p61_dev = NULL;

    unsigned char buf[100];


    P61_DBG_MSG(KERN_ALERT "p61_dev_ioctl-Enter %u arg = %ld\n", cmd, arg);
    p61_dev = filp->private_data;

    switch (cmd) {
    case P61_SET_PWR:
        if (arg == 2)
        {
            /* Do nothing*/
        }
        else if (arg == 1)
        {
            P61_DBG_MSG(KERN_ALERT " Soft Reset");
            //gpio_set_value(p61_dev->rst_gpio, 1);
            //msleep(20);
            gpio_set_value(p61_dev->rst_gpio, 0);
            msleep(50);
            ret = spi_read (p61_dev -> spi,(void *) buf, sizeof(buf));
            msleep(50);
            gpio_set_value(p61_dev->rst_gpio, 1);
            msleep(20);

        }
        break;

    case P61_SET_DBG:
        debug_level = (unsigned char )arg;
        P61_DBG_MSG(KERN_INFO"[NXP-P61] -  Debug level %d", debug_level);
        break;

    case P61_SET_POLL:

        p61_dev-> enable_poll_mode = (unsigned char )arg;
        if (p61_dev-> enable_poll_mode == 0)
        {
            P61_DBG_MSG(KERN_INFO"[NXP-P61] - IRQ Mode is set \n");
        }
        else
        {
            P61_DBG_MSG(KERN_INFO"[NXP-P61] - Poll Mode is set \n");
            p61_dev->enable_poll_mode = 1;
        }
        break;
    case P61_SET_SPM_PWR:
        P61_DBG_MSG(KERN_ALERT " P61_SET_SPM_PWR: enter");
        ret = pn544_dev_ioctl(filp, P61_SET_SPI_PWR, arg);
        P61_DBG_MSG(KERN_ALERT " P61_SET_SPM_PWR: exit");
    break;
    case P61_GET_SPM_STATUS:
        P61_DBG_MSG(KERN_ALERT " P61_GET_SPM_STATUS: enter");
        ret = pn544_dev_ioctl(filp, P61_GET_PWR_STATUS, arg);
        P61_DBG_MSG(KERN_ALERT " P61_GET_SPM_STATUS: exit");
    break;
    case P61_SET_DWNLD_STATUS:
        P61_DBG_MSG(KERN_ALERT " P61_SET_DWNLD_STATUS: enter");
        ret = pn544_dev_ioctl(filp, PN544_SET_DWNLD_STATUS, arg);
        P61_DBG_MSG(KERN_ALERT " P61_SET_DWNLD_STATUS: =%lu exit",arg);
    break;
    case P61_SET_THROUGHPUT:
        p61_through_put_t.enable_through_put_measure = true;
        P61_DBG_MSG(KERN_INFO"[NXP-P61] -  P61_SET_THROUGHPUT enable %d", p61_through_put_t.enable_through_put_measure);
        break;
    case P61_GET_ESE_ACCESS:
        P61_DBG_MSG(KERN_ALERT " P61_GET_ESE_ACCESS: enter");
        ret = pn544_dev_ioctl(filp, P544_GET_ESE_ACCESS, arg);
        P61_DBG_MSG(KERN_ALERT " P61_GET_ESE_ACCESS ret: %d exit",ret);
    break;
    case P61_SET_POWER_SCHEME:
        P61_DBG_MSG(KERN_ALERT " P61_SET_POWER_SCHEME: enter");
        ret = pn544_dev_ioctl(filp, P544_SET_POWER_SCHEME, arg);
        P61_DBG_MSG(KERN_ALERT " P61_SET_POWER_SCHEME ret: %d exit",ret);
    break;
    case P61_INHIBIT_PWR_CNTRL:
        P61_DBG_MSG(KERN_ALERT " P61_INHIBIT_PWR_CNTRL: enter");
        ret = pn544_dev_ioctl(filp, P544_SECURE_TIMER_SESSION, arg);
        P61_DBG_MSG(KERN_ALERT " P61_INHIBIT_PWR_CNTRL ret: %d exit", ret);
    break;
    default:
        P61_DBG_MSG(KERN_ALERT " Error case");
        ret = -EINVAL;
    }

    P61_DBG_MSG(KERN_ALERT "p61_dev_ioctl-exit %u arg = %lu\n", cmd, arg);
    return ret;
}

/**
 * \ingroup spi_driver
 * \brief Write data to P61 on SPI
 *
 * \param[in]       struct file *
 * \param[in]       const char *
 * \param[in]       size_t
 * \param[in]       loff_t *
 *
 * \retval data size
 *
*/

static ssize_t p61_dev_write(struct file *filp, const char *buf, size_t count,
        loff_t *offset)
{

    int ret = -1;
    struct p61_dev *p61_dev;
    char *tmp = NULL;

    P61_DBG_MSG(KERN_ALERT "p61_dev_write -Enter count %zu\n", count);

    p61_dev = filp->private_data;

    mutex_lock(&p61_dev->write_mutex);
    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    tmp = memdup_user(buf, count);
    if (IS_ERR(tmp)) {
        pr_info("%s: memdup_user failed\n", __func__);
        ret = PTR_ERR(tmp);
        return ret;
    }
    if(p61_through_put_t.enable_through_put_measure)
        p61_start_throughput_measurement(WRITE_THROUGH_PUT);
    /* Write data */
    ret = spi_write(p61_dev->spi, tmp, count);
    if (ret < 0)
    {
        ret = -EIO;
    }
    else
    {
        ret = count;
        if(p61_through_put_t.enable_through_put_measure)
            p61_stop_throughput_measurement(WRITE_THROUGH_PUT, ret);
    }

    kfree(tmp);
    mutex_unlock(&p61_dev->write_mutex);
    P61_DBG_MSG(KERN_ALERT "p61_dev_write ret %d- Exit \n", ret);
    return ret;
}

/**
 * \ingroup spi_driver
 * \brief Used to read data from P61 in Poll/interrupt mode configured using ioctl call
 *
 * \param[in]       struct file *
 * \param[in]       char *
 * \param[in]       size_t
 * \param[in]       loff_t *
 *
 * \retval read size
 *
*/

static ssize_t p61_dev_read(struct file *filp, char *buf, size_t count,
        loff_t *offset)
{
    int ret = -EIO;
    struct p61_dev *p61_dev = filp->private_data;
    unsigned char *tmp = NULL;

    P61_DBG_MSG("p61_dev_read count %zu - Enter \n", count);

    mutex_lock(&p61_dev->read_mutex);
    if (count > MAX_BUFFER_SIZE)
    {
        count = MAX_BUFFER_SIZE;
    }

    tmp = p61_dev->kbuf;
    if (!tmp) {
        pr_info("%s: device doesn't exist anymore.\n", __func__);
        ret = -ENODEV;
        goto fail;
    }
    memset(tmp, 0x00, MAX_BUFFER_SIZE);

    if (p61_dev->enable_poll_mode)
    {
        P61_DBG_MSG(" %s Poll Mode Enabled \n", __FUNCTION__);

        P61_DBG_MSG(KERN_INFO"SPI_READ returned 0x%zx", count);
        ret = spi_read(p61_dev->spi, tmp, count);
        if (0 > ret)
        {
            P61_ERR_MSG(KERN_ALERT "spi_read failed [SOF] \n");
            goto fail;
        }
    }
    else
    {
        P61_DBG_MSG(" %s P61_IRQ_ENABLE not Enabled \n", __FUNCTION__);
        ret = spi_read(p61_dev->spi, tmp, count);
        if (0 > ret)
        {
            P61_DBG_MSG(KERN_INFO"SPI_READ returned 0x%x", ret);
            ret = -EIO;
            goto fail;
        }
    }


    if(p61_through_put_t.enable_through_put_measure)
        p61_start_throughput_measurement(READ_THROUGH_PUT);

    if(p61_through_put_t.enable_through_put_measure)
        p61_stop_throughput_measurement (READ_THROUGH_PUT, count);
    P61_DBG_MSG(KERN_INFO"total_count = %zu", count);

    if (copy_to_user(buf, tmp, count))
    {
        P61_ERR_MSG("%s : failed to copy to user space\n", __func__);
        ret = -EFAULT;
        goto fail;
    }
    P61_DBG_MSG("p61_dev_read ret %d Exit\n", ret);

    mutex_unlock(&p61_dev->read_mutex);

    return ret;

    fail:
    P61_ERR_MSG("Error p61_dev_read ret %d Exit\n", ret);
    mutex_unlock(&p61_dev->read_mutex);
    return ret;
}

/**
 * \ingroup spi_driver
 * \brief It will configure the GPIOs required for soft reset, read interrupt & regulated power supply to P61.
 *
 * \param[in]       struct p61_spi_platform_data *
 * \param[in]       struct p61_dev *
 * \param[in]       struct spi_device *
 *
 * \retval 0 if ok.
 *
*/

static int p61_hw_setup(struct p61_spi_platform_data *platform_data,
       struct p61_dev *p61_dev, struct spi_device *spi)
{
    int ret = -1;

    P61_DBG_MSG("Entry : %s\n", __FUNCTION__);
    ret = 0;
    P61_DBG_MSG("Exit : %s\n", __FUNCTION__);
    return ret;
}

/**
 * \ingroup spi_driver
 * \brief Set the P61 device specific context for future use.
 *
 * \param[in]       struct spi_device *
 * \param[in]       void *
 *
 * \retval void
 *
*/

static inline void p61_set_data(struct spi_device *spi, void *data)
{
    dev_set_drvdata(&spi->dev, data);
}

/**
 * \ingroup spi_driver
 * \brief Get the P61 device specific context.
 *
 * \param[in]       const struct spi_device *
 *
 * \retval Device Parameters
 *
*/

static inline void *p61_get_data(const struct spi_device *spi)
{
    return dev_get_drvdata(&spi->dev);
}

/* possible fops on the p61 device */
static const struct file_operations p61_dev_fops = {
        .owner = THIS_MODULE,
        .read = p61_dev_read,
        .write = p61_dev_write,
        .open = p61_dev_open,
        .unlocked_ioctl = p61_dev_ioctl,
};
static int p61_parse_dt(struct device *dev,
    struct p61_spi_platform_data *data)
{
    int errorno = 0;

    return errorno;
}

/**
 * \ingroup spi_driver
 * \brief To probe for P61 SPI interface. If found initialize the SPI clock, bit rate & SPI mode.
          It will create the dev entry (P61) for user space.
 *
 * \param[in]       struct spi_device *
 *
 * \retval 0 if ok.
 *
*/

static int p61_probe(struct spi_device *spi)
{
    int ret = -1;
    struct p61_spi_platform_data *platform_data = NULL;
    struct p61_spi_platform_data platform_data1;
    struct p61_dev *p61_dev = NULL;

    if (get_second_board_absent() == 1) {
	    pr_err("%s second board absent, don't probe p73",__func__);
	    goto err_exit;
    }

    P61_DBG_MSG("%s chip select : %d , bus number = %d \n",
            __FUNCTION__, spi->chip_select, spi->master->bus_num);

    ret = p61_parse_dt(&spi->dev, &platform_data1);
    if (ret) {
        pr_err("%s - Failed to parse DT\n", __func__);
        goto err_exit;
    }
    platform_data = &platform_data1;
    p61_dev = kzalloc(sizeof(*p61_dev), GFP_KERNEL);
    if (p61_dev == NULL)
    {
        P61_ERR_MSG("failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }
    p61_dev->kbuflen = MAX_BUFFER_SIZE;
    p61_dev->kbuf = kzalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
    if (!p61_dev->kbuf) {
        pr_err("failed to allocate memory for p61_dev->kbuf");
        ret = -ENOMEM;
        goto err_free_dev;
    }
    ret = p61_hw_setup (platform_data, p61_dev, spi);
    if (ret < 0)
    {
        P61_ERR_MSG("Failed to p61_enable_P61_IRQ_ENABLE\n");
        goto err_exit0;
    }

    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = P61_SPI_CLOCK;
    //spi->chip_select = SPI_NO_CS;
    ret = spi_setup(spi);
    if (ret < 0)
    {
        P61_ERR_MSG("failed to do spi_setup()\n");
        goto err_exit0;
    }

    p61_dev -> spi = spi;
    p61_dev -> p61_device.minor = MISC_DYNAMIC_MINOR;
    p61_dev -> p61_device.name = "p73";
    p61_dev -> p61_device.fops = &p61_dev_fops;
    p61_dev -> p61_device.parent = &spi->dev;
    p61_dev->irq_gpio = platform_data->irq_gpio;
    p61_dev->rst_gpio  = platform_data->rst_gpio;


    dev_set_drvdata(&spi->dev, p61_dev);

    /* init mutex and queues */
    init_waitqueue_head(&p61_dev->read_wq);
    mutex_init(&p61_dev->read_mutex);
    mutex_init(&p61_dev->write_mutex);

    ret = misc_register(&p61_dev->p61_device);
    if (ret < 0)
    {
        P61_ERR_MSG("misc_register failed! %d\n", ret);
        goto err_exit0;
    }

    p61_dev-> enable_poll_mode = 0; /* Default IRQ read mode */
    P61_DBG_MSG("Exit : %s\n", __FUNCTION__);
    return ret;
    misc_deregister(&p61_dev->p61_device);
    err_exit0:
    mutex_destroy(&p61_dev->read_mutex);
    mutex_destroy(&p61_dev->write_mutex);
    kfree(p61_dev->kbuf);
    err_free_dev:
    if(p61_dev != NULL)
        kfree(p61_dev);
    err_exit:
    P61_DBG_MSG("ERROR: Exit : %s ret %d\n", __FUNCTION__, ret);
    return ret;
}

/**
 * \ingroup spi_driver
 * \brief Will get called when the device is removed to release the resources.
 *
 * \param[in]       struct spi_device
 *
 * \retval 0 if ok.
 *
*/

static int p61_remove(struct spi_device *spi)
{
    struct p61_dev *p61_dev = p61_get_data(spi);
    P61_DBG_MSG("Entry : %s\n", __FUNCTION__);

    gpio_free(p61_dev->rst_gpio);

    mutex_destroy(&p61_dev->read_mutex);
    misc_deregister(&p61_dev->p61_device);

    if(p61_dev != NULL)
        kfree(p61_dev);
    P61_DBG_MSG("Exit : %s\n", __FUNCTION__);
    return 0;
}
static struct of_device_id p61_dt_match[] = {
    {
        .compatible = "nxp,p61",
    },
    {}
};
static struct spi_driver p61_driver = {
        .driver = {
                .name = "p61",
                .bus = &spi_bus_type,
                .owner = THIS_MODULE,
                .of_match_table = p61_dt_match,
        },
        .probe =  p61_probe,
        .remove = (p61_remove),
};

/**
 * \ingroup spi_driver
 * \brief Module init interface
 *
 * \param[in]       void
 *
 * \retval handle
 *
*/

static int __init p61_dev_init(void)
{
    debug_level = P61_DEBUG_OFF;

    P61_DBG_MSG("Entry : %s\n", __FUNCTION__);

    return spi_register_driver(&p61_driver);

    P61_DBG_MSG("Exit : %s\n", __FUNCTION__);
}
module_init( p61_dev_init);

/**
 * \ingroup spi_driver
 * \brief Module exit interface
 *
 * \param[in]       void
 *
 * \retval void
 *
*/

static void __exit p61_dev_exit(void)
{
    P61_DBG_MSG("Entry : %s\n", __FUNCTION__);

    spi_unregister_driver(&p61_driver);
    P61_DBG_MSG("Exit : %s\n", __FUNCTION__);
}
module_exit( p61_dev_exit);

MODULE_AUTHOR("BHUPENDRA PAWAR");
MODULE_DESCRIPTION("NXP P61 SPI driver");
MODULE_LICENSE("GPL");

/** @} */
