/*
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_ts.c
 *
 * @brief Driver for the Freescale Semiconductor MXC touchscreen.
 *
 * The touchscreen driver is designed as a standard input driver which is a
 * wrapper over low level PMIC driver. Most of the hardware configuration and
 * touchscreen functionality is implemented in the low level PMIC driver. During
 * initialization, this driver creates a kernel thread. This thread then calls
 * PMIC driver to obtain touchscreen values continously. These values are then
 * passed to the input susbsystem.
 *
 * @ingroup touchscreen
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_adc.h>

#define MXC_TS_NAME	"mxc_ts"
#define	MIN(num1,num2)	((num1 < num2 ? num1 : num2))
#define	MAX(num1,num2)	((num1 > num2 ? num1 : num2))
#define	MIN3(num1, num2, num3)	(MIN(num1, MIN(num2, num3)))
#define	MAX3(num1, num2, num3)	(MAX(num1, MAX(num2, num3)))


static struct input_dev *mxc_inputdev = NULL;
static u32 input_ts_installed;

static int ts_thread(void *arg)
{
	t_touch_screen ts;
#ifdef CONFIG_MACH_MX51_ERDOS
	int x, y, x_sum, y_sum, x_min, y_min, x_max, y_max;
	static unsigned int bx = 0;
	static unsigned int by = 0;
#endif
	s32 wait = 0;

	daemonize("mxc_ts");
	while (input_ts_installed) {
		try_to_freeze();
		memset(&ts, 0, sizeof(t_touch_screen));
		if (0 != pmic_adc_get_touch_sample(&ts, !wait))
			continue;
		if (!(ts.contact_resistance || wait))
			continue;

#ifdef CONFIG_MACH_MX51_ERDOS
		x_sum = (int)(ts.x_position + ts.x_position1 + ts.x_position2);
		y_sum = (int)(ts.y_position + ts.y_position1 + ts.y_position2);

		x_min = (int)
			MIN3(ts.x_position, ts.x_position1, ts.x_position2);
		y_min = (int)
			MIN3(ts.y_position, ts.y_position1, ts.y_position2);

		x_max = (int)
			MAX3(ts.x_position, ts.x_position1, ts.x_position2);
		y_max = (int)
			MAX3(ts.y_position, ts.y_position1, ts.y_position2);
		x = (x_sum - x_min - x_max);
		y = (y_sum - y_min - y_max);

		if ((bx - x) < 20) {
			x = bx;
		}
		else {
			bx = x;
		}

		if ((by - y) < 20) {
			y = by;
		}
		else {
			by = y;
		}

		x = (x * 1000 / 950 - 50) * 1024 / 1000;
		y = 600 - ((y * 970 / 905 - 65) * 600 / 970);

		if (x < 0)	x = 0;
		if (y < 0)	y = 0;
		if (x > 1024)	x = 1024;
		if (y > 600)	y = 600;

		ts.x_position = (unsigned int) x;
		ts.y_position = (unsigned int) y;

		if (ts.contact_resistance) {
			input_report_abs(mxc_inputdev, ABS_X, ts.x_position);
			input_report_abs(mxc_inputdev, ABS_Y, ts.y_position);
			input_report_key(mxc_inputdev, BTN_TOUCH, 1);
		} else {
			input_report_key(mxc_inputdev, BTN_TOUCH, 0);
		}

		input_report_abs(mxc_inputdev, ABS_PRESSURE,
		                 ts.contact_resistance);
#else
		input_report_abs(mxc_inputdev, ABS_X, ts.x_position);
		input_report_abs(mxc_inputdev, ABS_Y, ts.y_position);
		input_report_abs(mxc_inputdev, ABS_PRESSURE,
				 ts.contact_resistance);
#endif
		input_sync(mxc_inputdev);

		wait = ts.contact_resistance;
		msleep(20);
	}

	return 0;
}

static int __init mxc_ts_init(void)
{
	int retval;

	if (!is_pmic_adc_ready())
		return -ENODEV;

	mxc_inputdev = input_allocate_device();
	if (!mxc_inputdev) {
		printk(KERN_ERR
		       "mxc_ts_init: not enough memory for input device\n");
		return -ENOMEM;
	}

	mxc_inputdev->name = MXC_TS_NAME;
	mxc_inputdev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	mxc_inputdev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
#ifdef CONFIG_MACH_MX51_ERDOS
	input_set_abs_params(mxc_inputdev, ABS_X, 0, 1024, 0, 0);
	input_set_abs_params(mxc_inputdev, ABS_Y, 0, 600, 0, 0);
	input_set_capability(mxc_inputdev, EV_ABS, ABS_PRESSURE);
#else
	mxc_inputdev->absbit[0] =
	    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) | BIT_MASK(ABS_PRESSURE);
#endif
	retval = input_register_device(mxc_inputdev);
	if (retval < 0) {
		input_free_device(mxc_inputdev);
		return retval;
	}

	input_ts_installed = 1;
	kernel_thread(ts_thread, NULL, CLONE_VM | CLONE_FS);
	printk("mxc input touchscreen loaded\n");
	return 0;
}

static void __exit mxc_ts_exit(void)
{
	input_ts_installed = 0;
	input_unregister_device(mxc_inputdev);

	if (mxc_inputdev) {
		input_free_device(mxc_inputdev);
		mxc_inputdev = NULL;
	}
}

late_initcall(mxc_ts_init);
module_exit(mxc_ts_exit);

MODULE_DESCRIPTION("MXC input touchscreen driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
