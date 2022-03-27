/*!
 * @file  drivers/input/keyboard/mxc_touch_key.c
 *
 * @brief Driver for the CAP1014 Capacitive Touch Sensor.
 *
 * Copyright (C) 2009 Nissin Systems Co.,Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * modification information
 * ------------------------
 * 2009/08/08 : init_cap1014() call at fail SMBusRead/WriteByte()
 *               (maybe lock cap1014, restart cal1014)
 *              ETK_REG_BTN_SET_CNF don't set TIMEOUT.
 *
 */

/*!
 * Comment ETK_DEBUG to disable debug messages
 */
#define ETK_DEBUG	0

#if ETK_DEBUG
#define	DEBUG
#include <linux/kernel.h>
#endif

/*!
 * Appoint a way of a report of an event
 */
#define MXC_ETK_NO_HOLD 1

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/jiffies.h>

/*
 * Module header file
 */
#include <asm/mxc_touch_key.h>
#include <mach/mx51_erdos_dev_params.h>

extern void gpio_cap1014_wakeup(void);
extern void gpio_cap1014_reset(void);
static int  init_cap1014(void); 

static int mxc_etk_interrupt_execute(void);

static short code_teble[CODE_TABLE_MAX] = {
	KEY_HOMEPAGE,
	KEY_WWW,
	KEY_MAIL,
	KEY_F24,
	0x0000,
	0x0000,
	0x0000
};

/* etk device major number */
static int etk_major = 0;

/* class structure. */
static struct class *mxc_etk_class;

/* present status of key */
static BYTE BtnStatBank = 0;

/* etk device irq number */
static iomux_pin_name_t irq_gpio = MXC_GPIO_TO_IRQ(ETK_ALT_PIN);

/* definition of work queue*/
DECLARE_WORK(mxc_etk_wq, (work_func_t)mxc_etk_interrupt_execute);

/* I2C client data */
static struct i2c_client *smbus_client = NULL;

/* Input device structure. */
static struct input_dev *mxc_etk_dev = NULL;

static int          mxc_etk_proc_init;
static volatile int mxc_etk_smbus_fail_err;
static volatile int mxc_etk_smbus_fail;
static volatile int mxc_etk_smbus_fail_reset;
static volatile int mxc_etk_evented;		/* count event send */
static volatile int mxc_etk_scaned;		/* count mxc_etk_scan_keys called */

/*!
 * Function of read byte command from CAP1014 via SMBus.
 *
 * @param   argv_client  Handle to slave device
 *
 * @param   argv_reg  Address of read register
 *
 * @param   argv_data  Pointer in data buffer of read register
 *
 * @return  The function returns 0 on success and errno on failure
 */
static int SMBusReadByte(struct i2c_client *argv_client, unsigned char argv_reg, BYTE *argv_data ) 
{
#if 1
	int retval;

	if(argv_client	== NULL ||
	   argv_data	== NULL ){
		pr_debug("mxc_etk: %s argument error\n", __func__);
		return -EINVAL;
	}

	retval = i2c_smbus_read_byte_data(argv_client, argv_reg);
	if(retval < 0){
		pr_debug("mxc_etk: %s returned error %d\n", __func__, retval);
		mxc_etk_smbus_fail++;
		mxc_etk_smbus_fail_err++;
		if ((mxc_etk_proc_init == 0) && (2 <= mxc_etk_smbus_fail_err)) {
			mxc_etk_smbus_fail_reset++;
			init_cap1014 (); 
		}
		return retval;
	}
	mxc_etk_smbus_fail_err = 0;
	*argv_data = (BYTE)retval;
	pr_debug("mxc_etk: %s reg = 0x%02X  data = 0x%02X\n", __func__, argv_reg, *argv_data);
	
	return 0;
#else
	return 0;
#endif
}

/*!
 * Function of write byte command to CAP1014 via SMBus	
 *
 * @param   argv_client  Handle to slave device
 *
 * @param   argv_reg  Address of write register
 *
 * @param   argv_data  Data written in register
 *
 * @return  The function returns 0 on success and errno on failure
 */
static int SMBusWriteByte(struct i2c_client *argv_client, unsigned char argv_reg, BYTE argv_data ) 
{
#if 1
	int retval;

	if(argv_client	== NULL){
		pr_debug("mxc_etk: %s argument error\n", __func__);
		return -EINVAL;
	}

	pr_debug("mxc_etk: %s reg = 0x%02X  data = 0x%02X\n", __func__, argv_reg, argv_data);

	retval = i2c_smbus_write_byte_data(argv_client, argv_reg, argv_data);
	if(retval < 0){
		pr_debug("mxc_etk: %s returned error %d\n", __func__, retval);
		mxc_etk_smbus_fail++;
		mxc_etk_smbus_fail_err++;
		if ((mxc_etk_proc_init == 0) && (2 <= mxc_etk_smbus_fail_err)) {
			mxc_etk_smbus_fail_reset++;
			init_cap1014 (); 
		}
		return retval;
	}
	mxc_etk_smbus_fail_err = 0;
	
	return 0;
#else
	return 0;
#endif
}

/*!
 * Function of initialize to CAP1014 via SMBus	
 *
 * @return  The function returns 0 on success and errno on failure
 */
static int init_cap1014( void ) 
{
	int retval = 0;
	BYTE	bIdx;
	u_char	threshold = 0;

	const   BYTE InitTAbleH[] = {
		ETK_REG_DATA_SENS,			0x20,		// data sensitivity
		ETK_REG_BTN_SET_CNF,	0x61,	// Position, NT disabled, No key repeat, 
						// don't set TIMEOUT
												// repeat for slider only
		ETK_REG_SENS_ENBL,			0xff,		// sensor enable
		ETK_REG_INT_ENBL,			0xff,		// Interrupt associated with all sensors

	};

	/* mark initialize procedure */
	mxc_etk_proc_init = 1;

	/* reset cap104 module */
	gpio_cap1014_reset();

	// Init. CAP1014 registers
	for ( bIdx = 0 ; bIdx < sizeof(InitTAbleH)/sizeof(InitTAbleH[0]) ; bIdx +=2){
		retval = SMBusWriteByte( smbus_client, InitTAbleH[bIdx], InitTAbleH[bIdx+1]);
		if(retval != 0 ){
			pr_debug("mxc_etk: init_cap1014 error [set %d]\n", (bIdx / 2) );
			/* clean initialize procedure */
			mxc_etk_proc_init = 0;
			return retval;
		}
	}

	/* sensor 1 threshold */
	retval = mx51_erdos_dev_params_read(DEV_PARAMS_OFFSET_SENSOR,
					 DEV_PARAMS_SIZE_SENSOR,
					 &threshold);
	if (retval != DEV_PARAMS_SIZE_SENSOR || threshold > 0x7F){
		pr_debug("mxc_etk: non parameter\n");
		threshold = 0x27;
	}
	retval = SMBusWriteByte( smbus_client, ETK_REG_SENS_THREHOLDE1, threshold);
	if(retval != 0 ){
		pr_debug("mxc_etk: init_cap1014 error ETK_REG_SENS_THREHOLDE1\n");
		/* clean initialize procedure */
		mxc_etk_proc_init = 0;
		return retval;
	}

	/* clean initialize procedure */
	mxc_etk_proc_init = 0;
	return 0;
	
}

/*!
 * The current state of the key is acquired from the register, and it 
 * reports on press/release event compared with the preserved value. 
 *
 * @return  The function returns 0 on success and errno on failure
 */
static int mxc_etk_scan_keys(void)
{
	int retval			= 0;
	BYTE BSts1			= 0;
	BYTE BSts2			= 0;
	BYTE BtnStat		= 0;
	BYTE Idx			= 0;
	BYTE Mask			= 0;
	SCANCODE PutCode	= 0;

	mxc_etk_scaned++;

	/* get state of key 1-7 */
	retval = SMBusReadByte(smbus_client, ETK_REG_BTN_STAT1, &BSts1);
	if ( retval != 0 ){
		return retval;
	}
	retval = SMBusReadByte(smbus_client, ETK_REG_BTN_STAT2, &BSts2);
	if ( retval != 0 ){
		return retval;
	}
	BtnStat = (BSts1 & 0x3f) | (BSts2 << 6 & 0x40);	
	pr_debug("mxc_etk: BtnStat = 0x%04X\n", BtnStat);

	/* scan to key of interrupt */
	for ( Mask = 0x01, Idx = 0 ; Idx < ETK_BTN_MAX ; Mask <<= 1, Idx++){
		if ( (BtnStat & Mask) != (BtnStatBank & Mask) ){
			PutCode = code_teble[Idx];
			if ( (BtnStatBank & Mask) == 0){
				if( PutCode != 0 ){
					pr_debug("mxc_etk: PutCode = Make 0x%04X\n", PutCode);
#if MXC_ETK_NO_HOLD
					input_event(mxc_etk_dev, EV_KEY, PutCode, 1);
					msleep(3);
					input_event(mxc_etk_dev, EV_KEY, PutCode, 0);
#else 
					input_event(mxc_etk_dev, EV_KEY, PutCode, 1);
#endif
					BtnStatBank |= Mask;
					mxc_etk_evented++;
				}
			} else {
				if( PutCode != 0 ){
					pr_debug("mxc_etk: PutCode = Break 0x%04X\n", PutCode);
#if MXC_ETK_NO_HOLD
#else 
					input_event(mxc_etk_dev, EV_KEY, PutCode, 0);
#endif
					BtnStatBank &= ~Mask;
				}
			}
		} 
	}	// End of key scan
	pr_debug("mxc_etk: BtnStatBank = 0x%04X\n", BtnStatBank);

	return 0;
}
/*!
 * The INT bit is cleared. It reports on the event investigating details of 
 * interrupt afterwards. Interrupt is returned enable. 
 *
 * @return  The function returns 0 on success and errno on failure
 */
static int mxc_etk_interrupt_execute(void)
{
	BYTE MainStat	= 0;
	int retval		= 0;

	// clear INT status 
	retval = SMBusReadByte(smbus_client, ETK_REG_MAIN_STAT, &MainStat);
	if (retval != 0 ){
		goto END;
	}
	retval = SMBusWriteByte(smbus_client, ETK_REG_MAIN_STAT, MainStat & ~ETK_INT_BIT );
	if (retval != 0 ){
		goto END;
	}

	// New key pressed scan here !
	retval = mxc_etk_scan_keys();
	if(retval != 0){
		goto END;
	}

END:
	/* enable of interrupt */
	enable_irq(irq_gpio);

	return retval;
}

/*!
 * This function processes interrupt. 
 * Interrupt is disable. 
 * The investigation of the key is requested to the schedule work.
 *
 * @param   irq      The Interrupt number
 * @param   dev_id   Driver private data
 *
 * @result    The function returns IRQ_RETVAL(1) if interrupt was handled,
 *            returns IRQ_RETVAL(0) if the interrupt was not handled.
 *            IRQ_RETVAL is defined in include/linux/interrupt.h.
 */
static irqreturn_t mxc_etk_interrupt(int irq, void *dev_id)
{
	int retval = 0;

	/* disable of interrupt */
	disable_irq(irq_gpio);

	/* schedule of work queue*/
	retval = schedule_work(&mxc_etk_wq);
	if (retval == 0) {
		return IRQ_RETVAL(1);
	}

	return IRQ_RETVAL(0);
}

/*!
 * This function is called when the touch key driver is opened.
 * Since touch key initialization is done in __init, nothing is done in open.
 *
 * @param    dev    Pointer to device inode
 *
 * @result    The function always return 0
 */
static int mxc_etk_open(struct input_dev *dev)
{
	return 0;
}

/*!
 * This function is called close the touch key device.
 * Nothing is done in this function, since every thing is taken care in
 * __exit function.
 *
 * @param    dev    Pointer to device inode
 *
 */
static void mxc_etk_close(struct input_dev *dev)
{
}

/*!
 * The touch key driver is opened by the file operation, this function is called. 
 * Since touch key initialization is done in __init, nothing is done in open.
 *
 * @result    The function always return 0
 */
static int mxc_etk_fo_open(struct inode *inode, struct file *file)
{
	return 0;
}

/*!
 * The touch key driver is close by the file operation, this function is called. 
 * Nothing is done in this function, since every thing is taken care in
 * __exit function.
 *
 * @result    The function always return 0
 */
static int mxc_etk_fo_release(struct inode *inode, struct file *file)
{
	return 0;
}

/*!
 * The touch key driver is read by the file manipulation, this function is called.
 * Because it is not necessary to read function, nothing is done by this function.
 *
 * @result    The function always return 0
 */
static int mxc_etk_fo_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

/*!
 * The touch key driver is write by the file manipulation, this function is called.
 * Because it is not necessary to write function, nothing is done by this function.
 *
 * @result    The function always return 0
 */
static int mxc_etk_fo_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	return 0;
}

/*!
 * The touch key driver is called and when ioctl is done by the file manipulation,
 * this function is called. 
 * Each ioctl is processed. 
 *
 * @param    cmd    request command of ioctl
 *
 * @param    arg    Argument of ioctl request 
 *
 * @return  The function returns 0 on success and errno on failure
 */
static int mxc_etk_fo_ioctl(struct inode *inode, struct file *file,
						unsigned int cmd, unsigned long arg)
{
	struct etk_reg_op_pram reg_operation;

	switch (cmd) {
	default:
		return -ENOIOCTLCMD;
	case ETK_GET_REGISTER:
		if (copy_from_user(&reg_operation, (struct etk_reg_op_pram __user *)arg, 
						sizeof(struct etk_reg_op_pram))){
			return -EFAULT;
		}
		if ( SMBusReadByte(smbus_client, reg_operation.reg_add, &reg_operation.reg_val) != 0 ){
			return -EFAULT;
		}
		return copy_to_user((struct etk_reg_op_pram __user *)arg, &reg_operation,
						sizeof(struct etk_reg_op_pram __user));

	case ETK_SET_REGISTER:
		if (copy_from_user(&reg_operation, (struct etk_reg_op_pram __user *)arg,
						 sizeof(struct etk_reg_op_pram))){
			return -EFAULT;
		}
		if ( SMBusWriteByte(smbus_client, reg_operation.reg_add, reg_operation.reg_val) != 0 ){
			return -EFAULT;
		}
		return 0;

	case ETK_EXEC_CALIB:
		if ( SMBusWriteByte(smbus_client, ETK_REG_CALIB_ACT, ETK_CALB_ALL_BIT) != 0 ){
			return -EFAULT;
		}
		return 0;
	}
}

static struct file_operations mxc_etc_fops = {
	.owner = THIS_MODULE,
	.open = mxc_etk_fo_open,
	.release = mxc_etk_fo_release,
	.write = mxc_etk_fo_write,
	.read = mxc_etk_fo_read,
	.ioctl = mxc_etk_fo_ioctl,
};

#ifdef CONFIG_PM
/*!
 * When the suspend request comes from the power manager, this function is called.
 * Interrupt is disable, and the CAP1014 device is put into the state of DEEP SLEEP. 
 *
 * @param   pdev  the device structure used to give information on touch key
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function returns 0 on success and errno on failure
 */
static int mxc_etk_suspend(struct platform_device *pdev, pm_message_t state)
{
	BYTE stat	= 0;
	int retval;

	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(irq_gpio);
	} else {
		disable_irq(irq_gpio);
		retval = SMBusReadByte(smbus_client, ETK_REG_MAIN_STAT, &stat);
		if ( retval != 0 ){
			pr_debug("mxc_etk: %s SMBusReadByte returned error %d\n", __func__, retval);
			return retval;
		}
		retval = SMBusWriteByte(smbus_client, ETK_REG_MAIN_STAT, stat | ETK_DSLEEP_BIT);
		if ( retval != 0 ){
			pr_debug("mxc_etk: %s SMBusReadByte returned error %d stat = 0x%02X\n", __func__, retval, stat);
			return retval;
		}
	}

	return 0;
}

/*!
 * When the resume request comes from the power manager, this function is called.
 * The CAP1014 device is made to get up, and interrupt is enabled. 
 *
 * @param   pdev  the device structure used to give information on touch key
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int mxc_etk_resume(struct platform_device *pdev)
{
	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(irq_gpio);
	} else {
		gpio_cap1014_wakeup();
		enable_irq(irq_gpio);
	}

	return 0;
}

#else
#define mxc_etk_suspend  NULL
#define mxc_etk_resume   NULL
#endif				/* CONFIG_PM */

/*!
 * This function is called to free the allocated memory for local arrays
 */
static void mxc_etk_free_allocated(void)
{
	if (mxc_etk_dev)
		input_free_device(mxc_etk_dev);
}

/*!
 * This function is called during the registration process of the i2c device. 
 *
 * @param   client  Handle to slave device
 *
 * @return  The function always returns 0.
 */
static int i2c_mxc_etk_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	smbus_client = client;
	
	if(i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_EMUL)){
		pr_debug("mxc_etk: i2c_check_functionality[I2C_FUNC_SMBUS_EMUL] ...ok\n");
	}

	return 0;

}

/*!
 * This function is called during the expulsion procedure of the i2c device.  
 *
 * @param   client  Handle to slave device
 *
 * @return  The function always returns 0.
 */
static int i2c_mxc_etk_remove(struct i2c_client *client)
{
	kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id i2c_mxc_etk_id[] = {
	{ "mxc_etk", 0 },
	{ }
};

static struct i2c_driver i2c_mxc_etk_driver = {
	.driver = {
		   .name = "mxc_etk",
		   .owner = THIS_MODULE,
		   },
	.probe = i2c_mxc_etk_probe,
	.remove = i2c_mxc_etk_remove,
	.id_table = i2c_mxc_etk_id,
};

/*!
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions.
 *
 * @return  The function returns 0 on successful registration. Otherwise returns
 *          specific error code.
 */
static int mxc_etk_probe(struct platform_device *pdev)
{
	int cnt;
	int retval;

	/* register an i2c driver */
	retval = i2c_add_driver(&i2c_mxc_etk_driver);
	if (retval) {
		pr_debug("mxc_etk: i2c_add_driver returned error %d\n", retval);
		goto err1;
	}

	/* Initialization of cap1014 */
	retval = init_cap1014();
	if (retval != 0) {
		pr_debug("mxc_etk: init_cap1014 returned error %d\n", retval);
		goto err2;
	}

	/* register file operation interface */
	retval = register_chrdev(etk_major, MOD_NAME, &mxc_etc_fops);
	if (retval < 0) {
      printk("device registration error\n");
		goto err2;
    } 
	if (etk_major == 0){
		etk_major = retval;
	}
	mxc_etk_class = class_create(THIS_MODULE, MOD_NAME);
	device_create(mxc_etk_class, NULL, MKDEV(etk_major, 0),
				NULL, MOD_NAME);

	/* Setting of the registration information to the input device */
	mxc_etk_dev = input_allocate_device();
	if (!mxc_etk_dev) {
		printk(KERN_ERR
		       "mxc_etk_dev: not enough memory for input device\n");
		retval = -ENOMEM;
		goto err3;
	}
	mxc_etk_dev->name = "mxc_etk";
	mxc_etk_dev->open = mxc_etk_open;
	mxc_etk_dev->close = mxc_etk_close;

	/* The bit for the config of the keyboard is set */
	set_bit(EV_KEY, mxc_etk_dev->evbit);
	for(cnt=0; cnt<CODE_TABLE_MAX; cnt++){
		set_bit(code_teble[cnt], mxc_etk_dev->keybit);
	}

	/* Registration to the input device */
	retval = input_register_device(mxc_etk_dev);
	if (retval < 0) {
		printk(KERN_ERR
		       "mxc_etk_dev: failed to register input device\n");
		goto err4;
	}

	/* Repetition input is disable. */
	mxc_etk_dev->rep[REP_DELAY] = 0;
	mxc_etk_dev->rep[REP_PERIOD] = 0;

	/*
	 * Request for IRQ number for GPIO3_25. The Interrupt handler
	 * function (mxc_etk_interrupt) is called when interrupt occurs on
	 * GPIO3_25.
	 */
	retval = request_irq(irq_gpio, mxc_etk_interrupt, IRQ_TYPE_LEVEL_HIGH, MOD_NAME, NULL);
	if (retval) {
		pr_debug("mxc_etk: request_irq returned error %d\n", retval);
		goto err5;
	}

	/* Wakeup cannot do by default. */
	/* "What should do wakeup" doesn't set it as it. */
	device_init_wakeup(&pdev->dev, 0);

	return 0;
err5:
	input_unregister_device(mxc_etk_dev);
err4:
	mxc_etk_free_allocated();
err3:
	device_destroy(mxc_etk_class, MKDEV(etk_major, 0));
err2:
	i2c_del_driver(&i2c_mxc_etk_driver);
err1:
	return retval;
}

/*!
 * Dissociates the driver from the etk device.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mxc_etk_remove(struct platform_device *pdev)
{

	BtnStatBank = 0x00;

	free_irq(irq_gpio, MOD_NAME);

	input_unregister_device(mxc_etk_dev);

	mxc_etk_free_allocated();

	device_destroy(mxc_etk_class, MKDEV(etk_major, 0));

	i2c_del_driver(&i2c_mxc_etk_driver);

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_etk_driver = {
	.driver = {
		   .name = "mxc_touch_key",
		   .bus = &platform_bus_type,
		   },
	.suspend = mxc_etk_suspend,
	.resume = mxc_etk_resume,
	.probe = mxc_etk_probe,
	.remove = mxc_etk_remove
};

/*!
 * This function is called for module initialization.
 * It registers touch key char driver and requests for irq number. This
 * function does the initialization of the touch key device.
 *
 * @return  0 on success and a non-zero value on failure.
 */
static int __init mxc_etk_init(void)
{
	printk(KERN_INFO "MXC Electrostatic Touch Key driver loaded\n");
	platform_driver_register(&mxc_etk_driver);

	return 0;
}

/*!
 * This function is called whenever the module is removed from the kernel. 
 * It unregisters the touch key driver from kernel and frees the irq number. *
 */
static void __exit mxc_etk_cleanup(void)
{
	platform_driver_unregister(&mxc_etk_driver);
}

module_init(mxc_etk_init);
module_exit(mxc_etk_cleanup);

MODULE_DESCRIPTION("Capacitive Touch Sensor driver for CAP1014 devices");
MODULE_AUTHOR("Nissin Systems Co.,Ltd");
MODULE_LICENSE("GPL");
