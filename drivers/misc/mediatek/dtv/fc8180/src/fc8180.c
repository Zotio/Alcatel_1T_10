/*****************************************************************************
	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8180.c

	Description : Driver source file

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <mt-plat/mt_gpio.h>
#include <mach/mt_clkbuf_ctl.h> /*  for clock buffer */
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>


#include "fc8180.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8180_regs.h"
#include "fc8180_isr.h"
#include "fci_hal.h"

struct ISDBT_INIT_INFO_T *hInit;

#define RING_BUFFER_SIZE	(188 * 32 * 8)

/* GPIO(RESET & INTRRUPT) Setting */
#define FC8180_NAME		"isdbt"
#ifndef CONFIG_OF
#define MSM_GPIO_OFFSET 902
#define GPIO_ISDBT_IRQ (111 + MSM_GPIO_OFFSET)
#define GPIO_ISDBT_PWR_EN (37 + MSM_GPIO_OFFSET)
#define GPIO_ISDBT_RST (80 + MSM_GPIO_OFFSET)
#else
extern struct of_device_id fc8180_match_table[];
s32 isdbt_chip_id(void);

static int irq_gpio;
static int enable_gpio;
//static int reset_gpio;

#define GPIO_ISDBT_IRQ		irq_gpio
#define GPIO_ISDBT_PWR_EN	enable_gpio
//#define GPIO_ISDBT_RST		reset_gpio
#endif
extern struct miscdevice fc8180_misc_device;
void isdbt_exit(void);

struct ISDBT_OPEN_INFO_T hOpen_Val;
u8 static_ringbuffer[RING_BUFFER_SIZE];

enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;
static DEFINE_MUTEX(ringbuffer_lock);
static DEFINE_MUTEX(driver_mode_lock);
struct wake_lock isdbt_wakelock;

static DECLARE_WAIT_QUEUE_HEAD(isdbt_isr_wait);
u32 bbm_xtal_freq;

static struct pinctrl *dtvctrl;
static struct pinctrl_state *dtv_spi_default;
static struct pinctrl_state *dtv_spi_resume;
static struct pinctrl_state *dtv_spi_suspend;
static struct pinctrl_state *dtv_pwr_high;
static struct pinctrl_state *dtv_pwr_low;
static struct pinctrl_state *dtv_irq_as_eint;
static struct pinctrl_state *dtv_irq_as_gpio;

#ifndef BBM_I2C_TSIF
static u8 isdbt_isr_sig;
static struct task_struct *isdbt_kthread;

static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
	isdbt_isr_sig = 1;
	wake_up_interruptible(&isdbt_isr_wait);
	return IRQ_HANDLED;
}
#endif

int isdbt_hw_setting(void)
{
	int err;

	print_log(0, "isdbt_hw_setting \n");

	clk_buf_ctrl(CLK_BUF_NFC, 1);
#if 0		
	err = gpio_request(GPIO_ISDBT_PWR_EN, "isdbt_en");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_en\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);
	err = gpio_export(GPIO_ISDBT_PWR_EN, 0);
	if (err)
		print_log(0, "%s: error %d gpio_export for %d\n",
			__func__, err, GPIO_ISDBT_PWR_EN);
	else {
		err = gpio_export_link(fc8180_misc_device.this_device,
			"isdbt_en", GPIO_ISDBT_PWR_EN);
		if (err)
			print_log(0, "%s: error %d gpio_export for %d\n",
				__func__, err, GPIO_ISDBT_PWR_EN);
	}
#endif
#ifndef BBM_I2C_TSIF

	err = gpio_request(GPIO_ISDBT_IRQ, "isdbt_irq");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_irq\n");
		goto gpio_isdbt_rst;
	}

	gpio_direction_input(GPIO_ISDBT_IRQ);

	err = request_irq(gpio_to_irq(GPIO_ISDBT_IRQ), isdbt_irq
		, IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8180_NAME, NULL);
	if (err < 0) {
		print_log(0,
			"isdbt_hw_setting: couldn't request gpio	\
			interrupt %d reason(%d)\n"
			, gpio_to_irq(GPIO_ISDBT_IRQ), err);
	goto request_isdbt_irq;
	}
	pinctrl_select_state(dtvctrl, dtv_pwr_low);
	pinctrl_select_state(dtvctrl, dtv_irq_as_eint);
#endif
	return 0;
#ifndef BBM_I2C_TSIF
request_isdbt_irq:
	gpio_free(GPIO_ISDBT_IRQ);
#endif
gpio_isdbt_rst:
	gpio_free(GPIO_ISDBT_PWR_EN);
//gpio_isdbt_en:
	return err;
}

/*POWER_ON & HW_RESET & INTERRUPT_CLEAR */
void isdbt_hw_init(void)
{
	wake_lock(&isdbt_wakelock);
	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_init \n");
	pinctrl_select_state(dtvctrl, dtv_pwr_high);
	pinctrl_select_state(dtvctrl, dtv_spi_default);
	mdelay(30);
	driver_mode = ISDBT_POWERON;
	mutex_unlock(&driver_mode_lock);
}

/*POWER_OFF */
void isdbt_hw_deinit(void)
{
	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_deinit \n");
	pinctrl_select_state(dtvctrl, dtv_pwr_low);
	driver_mode = ISDBT_POWEROFF;
	mutex_unlock(&driver_mode_lock);
	wake_unlock(&isdbt_wakelock);
	mdelay(5);
}
   
int data_callback(ulong hDevice, u8 *data, int len) 
{
	struct ISDBT_INIT_INFO_T *hInit;
	struct list_head *temp;
	hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	list_for_each(temp, &(hInit->hHead))
	{
		struct ISDBT_OPEN_INFO_T *hOpen;

		hOpen = list_entry(temp, struct ISDBT_OPEN_INFO_T, hList);

		if (hOpen->isdbttype == TS_TYPE) {
			mutex_lock(&ringbuffer_lock);
			if (fci_ringbuffer_free(&hOpen->RingBuffer) < len) {
				/*print_log(hDevice, "f"); */
				/* return 0 */;
				FCI_RINGBUFFER_SKIP(&hOpen->RingBuffer, len);
			}

			fci_ringbuffer_write(&hOpen->RingBuffer, data, len);

			wake_up_interruptible(&(hOpen->RingBuffer.queue));

			mutex_unlock(&ringbuffer_lock);
		}
	}

	return 0;
}


#ifndef BBM_I2C_TSIF
static int isdbt_thread(void *hDevice)
{
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	set_user_nice(current, -20);

	print_log(hInit, "isdbt_kthread enter\n");

	bbm_com_ts_callback_register((ulong)hInit, data_callback);

	while (1) {
		wait_event_interruptible(isdbt_isr_wait,
			isdbt_isr_sig || kthread_should_stop());

		mutex_lock(&driver_mode_lock);
		if (driver_mode == ISDBT_POWERON)
			bbm_com_isr(hInit);
		mutex_unlock(&driver_mode_lock);

		isdbt_isr_sig = 0;

		if (kthread_should_stop())
			break;
	}

	bbm_com_ts_callback_deregister();

	print_log(hInit, "isdbt_kthread exit\n");

	return 0;
}
#endif

#ifdef CONFIG_COMPAT
long isdbt_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long) compat_ptr(arg);

	return isdbt_ioctl(filp, cmd, arg);
}
#endif

const struct file_operations isdbt_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= isdbt_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= isdbt_compat_ioctl,
#endif
	.open		= isdbt_open,
	.read		= isdbt_read,
	.release	= isdbt_release,
};

struct miscdevice fc8180_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FC8180_NAME,
	.fops = &isdbt_fops,
};

int isdbt_open(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt open\n");

	hOpen = &hOpen_Val;
	hOpen->buf = &static_ringbuffer[0];
	hOpen->isdbttype = 0;

	if (list_empty(&(hInit->hHead)))
		list_add(&(hOpen->hList), &(hInit->hHead));

	hOpen->hInit = (HANDLE *)hInit;

	if (hOpen->buf == NULL) {
		print_log(hInit, "ring buffer malloc error\n");
		return -ENOMEM;
	}

	fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);

	filp->private_data = hOpen;

	return 0;
}

ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s32 avail;
	s32 non_blocking = filp->f_flags & O_NONBLOCK;
	struct ISDBT_OPEN_INFO_T *hOpen
		= (struct ISDBT_OPEN_INFO_T *)filp->private_data;
	struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
	ssize_t len, read_len = 0;

	if (!cibuf->data || !count)	{
		/*print_log(hInit, " return 0\n"); */
		return 0;
	}

	if (non_blocking && (fci_ringbuffer_empty(cibuf)))	{
		/*print_log(hInit, "return EWOULDBLOCK\n"); */
		return -EWOULDBLOCK;
	}

	if (wait_event_interruptible(cibuf->queue,
		!fci_ringbuffer_empty(cibuf))) {
		print_log(hInit, "return ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	mutex_lock(&ringbuffer_lock);

	avail = fci_ringbuffer_avail(cibuf);

	if (count >= avail)
		len = avail;
	else
		len = count - (count % 188);

	read_len = fci_ringbuffer_read_user(cibuf, buf, len);

	mutex_unlock(&ringbuffer_lock);

	return read_len;
}

int isdbt_release(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;

	print_log(hInit, "isdbt_release\n");
	isdbt_hw_deinit();

	hOpen = filp->private_data;

	hOpen->isdbttype = 0;
	if (!list_empty(&(hInit->hHead)))
		list_del(&(hOpen->hList));
	/*kfree(hOpen->buf);*/

	/*if (hOpen != NULL)
		kfree(hOpen);*/

	return 0;
}


#ifndef BBM_I2C_TSIF
void isdbt_isr_check(HANDLE hDevice)
{
	u8 isr_time = 0;

	bbm_com_write(hDevice, BBM_BUF_ENABLE, 0x00);

	while (isr_time < 10) {
		if (!isdbt_isr_sig)
			break;

		ms_wait(10);
		isr_time++;
	}

}
#endif

long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	s32 res = BBM_NOK;
	s32 err = 0;
	u32 size = 0;
	struct ISDBT_OPEN_INFO_T *hOpen;

	struct ioctl_info info;

	if (_IOC_TYPE(cmd) != IOCTL_MAGIC)
		return -EINVAL;
	if (_IOC_NR(cmd) >= IOCTL_MAXNR)
		return -EINVAL;

	hOpen = filp->private_data;

	size = _IOC_SIZE(cmd);
	if (size > sizeof(struct ioctl_info))
		size = sizeof(struct ioctl_info);
	switch (cmd) {
	case IOCTL_ISDBT_RESET:
		res = bbm_com_reset(hInit);
		print_log(hInit, "[FC8180] IOCTL_ISDBT_RESET\n");
		break;
	case IOCTL_ISDBT_INIT:
		res = bbm_com_i2c_init(hInit, FCI_HPI_TYPE);
		res |= bbm_com_probe(hInit);
		if (res) {
			print_log(hInit, "FC8180 Initialize Fail\n");
			break;
		}
		res = bbm_com_init(hInit);
		res |= bbm_com_tuner_select(hInit, FC8180_TUNER, 0);
		break;
	case IOCTL_ISDBT_BYTE_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_read(hInit, (u16)info.buff[0]
			, (u8 *)(&info.buff[1]));
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_WORD_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_read(hInit, (u16)info.buff[0]
			, (u16 *)(&info.buff[1]));
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_LONG_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_read(hInit, (u16)info.buff[0]
			, (u32 *)(&info.buff[1]));
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BULK_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if (info.buff[1] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 2)) {
			print_log(hInit, "[FC8300] BULK_READ sizeErr %d\n"
				, info.buff[1]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_bulk_read(hInit, (u16)info.buff[0]
			, (u8 *)(&info.buff[2]), info.buff[1]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BYTE_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_write(hInit, (u16)info.buff[0]
			, (u8)info.buff[1]);
		break;
	case IOCTL_ISDBT_WORD_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_write(hInit, (u16)info.buff[0]
			, (u16)info.buff[1]);
		break;
	case IOCTL_ISDBT_LONG_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_write(hInit, (u16)info.buff[0]
			, (u32)info.buff[1]);
		break;
	case IOCTL_ISDBT_BULK_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if (info.buff[1] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 2)) {
			print_log(hInit, "[FC8300] BULK_WRITE sizeErr %d\n"
				, info.buff[1]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_bulk_write(hInit, (u16)info.buff[0]
			, (u8 *)(&info.buff[2]), info.buff[1]);
		break;
	case IOCTL_ISDBT_TUNER_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if ((info.buff[1] > 1) || (info.buff[2] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 3))) {
			print_log(hInit, "[FC8300] TUNER_READ sizeErr AR[%d] Dat[%d]\n"
				, info.buff[1], info.buff[2]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_tuner_read(hInit, (u8)info.buff[0]
			, (u8)info.buff[1],  (u8 *)(&info.buff[3])
			, (u8)info.buff[2]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_TUNER_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if ((info.buff[1] > 1) || (info.buff[2] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 3))) {
			print_log(hInit, "[FC8300] TUNER_WRITE sizeErr AR[%d] Dat[%d]\n"
				, info.buff[1], info.buff[2]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_tuner_write(hInit, (u8)info.buff[0]
			, (u8)info.buff[1], (u8 *)(&info.buff[3])
			, (u8)info.buff[2]);
		break;
	case IOCTL_ISDBT_TUNER_SET_FREQ:
		{
			u32 f_rf;
			err = copy_from_user((void *)&info, (void *)arg, size);
			f_rf = (u32)info.buff[0];
#ifndef BBM_I2C_TSIF
			isdbt_isr_check(hInit);
#endif
			res = bbm_com_tuner_set_freq(hInit, f_rf);
#ifndef BBM_I2C_TSIF
			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_flush(&hOpen->RingBuffer);
			mutex_unlock(&ringbuffer_lock);
			bbm_com_write(hInit, BBM_BUF_ENABLE, 0x01);
#endif
		}
		break;
	case IOCTL_ISDBT_TUNER_SELECT:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_select(hInit  
			, (u32)info.buff[0], (u32)info.buff[1]);
		print_log(hInit, "[FC8180] IOCTL_ISDBT_TUNER_SELECT\n");
		break;
	case IOCTL_ISDBT_TS_START:
		hOpen->isdbttype = TS_TYPE;
		print_log(hInit, "[FC8180] IOCTL_ISDBT_TS_START\n");
		break;
	case IOCTL_ISDBT_TS_STOP:
		hOpen->isdbttype = 0;
		print_log(hInit, "[FC8180] IOCTL_ISDBT_TS_STOP\n");
		break;
	case IOCTL_ISDBT_POWER_ON:
		isdbt_hw_init();
#ifdef BBM_SPI_IF
		bbm_com_byte_write(hInit, BBM_DM_DATA, 0x00);
#endif
		print_log(hInit, "[FC8180] IOCTL_ISDBT_POWER_ON\n");
		break;
	case IOCTL_ISDBT_POWER_OFF:
		isdbt_hw_deinit();
		print_log(hInit, "[FC8180] IOCTL_ISDBT_POWER_OFF\n");
		break;
	case IOCTL_ISDBT_SCAN_STATUS:
		res = bbm_com_scan_status(hInit);
		print_log(hInit
			, "[FC8180] IOCTL_ISDBT_SCAN_STATUS : %d\n", res);
		break;
	case IOCTL_ISDBT_TUNER_GET_RSSI:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_get_rssi(hInit, (s32 *)&info.buff[0]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	default:
		print_log(hInit, "isdbt ioctl error!\n");
		res = BBM_NOK;
		break;
	}

	if (err < 0) {
		print_log(hInit, "copy to/from user fail : %d", err);
		res = BBM_NOK;
	}
	return res;
}

#ifdef CONFIG_OF
  struct of_device_id fc8180_match_table_dev[] = {
	{ .compatible = "emdoor,fc8180_spi_dev", },
	{ }
};

static int fc8180_dt_init(void)
{
	struct device_node *np;
	//u32 rc;
	struct platform_device *pdev = NULL;
	int ret;
	

	np = of_find_compatible_node(NULL, NULL,
		fc8180_match_table_dev[0].compatible);

	if (np) {
                pdev = of_find_device_by_node(np);
                if (pdev) {
                        dtvctrl = devm_pinctrl_get(&pdev->dev);
                        if (IS_ERR(dtvctrl)) {
                                ret = PTR_ERR(dtvctrl);
                                print_log(hInit, "%s can't find fingerprint pinctrl\n", __func__);
                                return ret;
                        } else {
				print_log(hInit, "dtvctrl found here!\n");
			}
                } else {
                        print_log(hInit, "%s platform device is null\n", __func__);
                }
        } else {
                print_log(hInit, "%s device node is null\n", __func__);
        }

	dtv_spi_default = pinctrl_lookup_state(dtvctrl, "dtv_spi_default");
        if (IS_ERR(dtv_spi_default)) {
                ret = PTR_ERR(dtv_spi_default);
                print_log(hInit, "%s : pinctrl err, dtv_spi_default\n", __func__);
        } else
		print_log(hInit, "dtv_spi_default found\n");

	dtv_spi_resume = pinctrl_lookup_state(dtvctrl, "dtv_spi_resume");
        if (IS_ERR(dtv_spi_resume)) {
                ret = PTR_ERR(dtv_spi_resume);
                print_log(hInit, "%s : pinctrl err, dtv_spi_resume\n", __func__);
        } else
		print_log(hInit, "dtv_spi_resume found\n");


	dtv_spi_suspend = pinctrl_lookup_state(dtvctrl, "dtv_spi_suspend");
        if (IS_ERR(dtv_spi_suspend)) {
                ret = PTR_ERR(dtv_spi_suspend);
                print_log(hInit, "%s : pinctrl err, dtv_spi_suspend\n", __func__);
        } else 
		print_log(hInit, "dtv_spi_suspend found\n");


	dtv_pwr_high = pinctrl_lookup_state(dtvctrl, "dtv_pwr_high");
        if (IS_ERR(dtv_pwr_high)) {
                ret = PTR_ERR(dtv_pwr_high);
                print_log(hInit, "%s : pinctrl err, dtv_pwr_high\n", __func__);
        } else
		print_log(hInit, "dtv_pwr_high found\n");


	dtv_pwr_low = pinctrl_lookup_state(dtvctrl, "dtv_pwr_low");
        if (IS_ERR(dtv_pwr_low)) {
                ret = PTR_ERR(dtv_pwr_low);
                print_log(hInit, "%s : pinctrl err, dtv_pwr_low\n", __func__);
        } else 
		print_log(hInit, "dtv_pwr_low found\n");


	dtv_irq_as_eint = pinctrl_lookup_state(dtvctrl, "dtv_irq_as_eint");
        if (IS_ERR(dtv_irq_as_eint)) {
                ret = PTR_ERR(dtv_irq_as_eint);
                print_log(hInit, "%s : pinctrl err, dtv_irq_as_eint\n", __func__);
        } else
		print_log(hInit, "pwr_irq_as_eint found\n");

	dtv_irq_as_gpio = pinctrl_lookup_state(dtvctrl, "dtv_irq_as_gpio");
        if (IS_ERR(dtv_irq_as_gpio)) {
                ret = PTR_ERR(dtv_irq_as_gpio);
                print_log(hInit, "%s : pinctrl err, dtv_irq_as_gpio\n", __func__);
        } else 
		print_log(hInit, "pwr_irq_as_gpio found\n");

#if  0
	enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (!gpio_is_valid(enable_gpio)) {
		print_log(hInit, "isdbt error getting enable_gpio\n");
		return -EINVAL;
	}

#endif	
	irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio)) {
		print_log(hInit, "isdbt error getting irq_gpio\n");
		return -EINVAL;
	}

	bbm_xtal_freq = DEFAULT_BBM_XTAL_FREQ;
	//rc = of_property_read_u32(np, "bbm-xtal-freq", &bbm_xtal_freq);
	//if (rc) 
	//	print_log(hInit, "no dt xtal-freq config, using default\n");

	return 0;
}
#else
static int fc8180_dt_init(void)
{
	bbm_xtal_freq = DEFAULT_BBM_XTAL_FREQ;
	return 0;
}
#endif
#define FC8180_CHIP_ID 0x8180
#define FC8180_CHIP_ID_REG 0x26

s32 isdbt_chip_id(void)
{
	s32 res;
	u16 addr, data;

	isdbt_hw_init();

	addr = FC8180_CHIP_ID_REG;
	res = bbm_com_word_read(hInit, addr, &data);
	if (res) {
		print_log(hInit, "%s reading chip id err %d\n", __func__, res);
		goto errout;
	}

	if (FC8180_CHIP_ID != data) {
		print_log(hInit, "%s wrong chip id %#x\n", __func__, data);
		res = -1;
	} else
		print_log(hInit, "%s reg %#x id %#x\n", __func__, addr, data);

errout:
	isdbt_hw_deinit();
	return res;
}

int isdbt_init(void)
{
	s32 res;

	print_log(hInit, "isdbt_init build\n");

	res = misc_register(&fc8180_misc_device);

	if (res < 0) {
		print_log(hInit, "isdbt init fail : %d\n", res);
		return res;
	}

	res = fc8180_dt_init();
	if (res) {
		misc_deregister(&fc8180_misc_device);
		return res;
	}
	wake_lock_init(&isdbt_wakelock, WAKE_LOCK_SUSPEND, fc8180_misc_device.name);
	isdbt_hw_setting();

	hInit = kmalloc(sizeof(struct ISDBT_INIT_INFO_T), GFP_KERNEL);


#if defined(BBM_I2C_TSIF) || defined(BBM_I2C_SPI)
	res = bbm_com_hostif_select(hInit, BBM_I2C);
#elif defined(BBM_SPI_IF)
	res = bbm_com_hostif_select(hInit, BBM_SPI);
#else
	res = bbm_com_hostif_select(hInit, BBM_PPI);
#endif

	if (res)
		print_log(hInit, "isdbt host interface select fail!\n");

#ifndef BBM_I2C_TSIF
	if (!isdbt_kthread)	{
		print_log(hInit, "kthread run\n");
		isdbt_kthread = kthread_run(isdbt_thread
			, (void *)hInit, "isdbt_thread");
	}
#endif

	INIT_LIST_HEAD(&(hInit->hHead));
	res = isdbt_chip_id();
//	if (res)
//		goto error_out;

	return 0;
//error_out:
	//isdbt_exit();
//	return -ENODEV;
}

void isdbt_exit(void)
{
	print_log(hInit, "isdbt isdbt_exit \n");

	isdbt_hw_deinit();

#ifndef BBM_I2C_TSIF
	free_irq(gpio_to_irq(GPIO_ISDBT_IRQ), NULL);
	gpio_free(GPIO_ISDBT_IRQ);
#endif
//	gpio_free(GPIO_ISDBT_PWR_EN);

#ifndef BBM_I2C_TSIF
	if (isdbt_kthread)
		kthread_stop(isdbt_kthread);

	isdbt_kthread = NULL;
#endif

	bbm_com_hostif_deselect(hInit);

	if (hInit != NULL)
		kfree(hInit);
	wake_lock_destroy(&isdbt_wakelock);
	misc_deregister(&fc8180_misc_device);

}

late_initcall(isdbt_init);
module_exit(isdbt_exit);

MODULE_LICENSE("Dual BSD/GPL");
