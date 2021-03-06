/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*
 * Implementation based on rtc-ds1553.c
 */

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-mxc_v2.c
 * @brief Real Time Clock interface
 *
 * This file contains Real Time Clock interface for Linux.
 *
 * @ingroup RTC
 */

#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>
#include <asm/io.h>

#define SRTC_LPPDR_INIT       0x41736166	/* init for glitch detect */

#define SRTC_LPCR_SWR_LP      (1 << 0)	/* lp software reset */
#define SRTC_LPCR_EN_LP       (1 << 3)	/* lp enable */
#define SRTC_LPCR_WAE         (1 << 4)	/* lp wakeup alarm enable */
#define SRTC_LPCR_SAE         (1 << 5)	/* lp security alarm enable */
#define SRTC_LPCR_SI          (1 << 6)	/* lp security interrupt enable */
#define SRTC_LPCR_ALP         (1 << 7)	/* lp alarm flag */
#define SRTC_LPCR_LTC         (1 << 8)	/* lp lock time counter */
#define SRTC_LPCR_LMC         (1 << 9)	/* lp lock monotonic counter */
#define SRTC_LPCR_SV          (1 << 10)	/* lp security violation */
#define SRTC_LPCR_NSA         (1 << 11)	/* lp non secure access */
#define SRTC_LPCR_NVEIE       (1 << 12)	/* lp non valid state exit int en */
#define SRTC_LPCR_IEIE        (1 << 13)	/* lp init state exit int enable */
#define SRTC_LPCR_NVE         (1 << 14)	/* lp non valid state exit bit */
#define SRTC_LPCR_IE          (1 << 15)	/* lp init state exit bit */

#define SRTC_LPCR_ALL_INT_EN (SRTC_LPCR_WAE | SRTC_LPCR_SAE | \
			      SRTC_LPCR_SI | SRTC_LPCR_ALP | \
			      SRTC_LPCR_NVEIE | SRTC_LPCR_IEIE)

#define SRTC_LPSR_TRI         (1 << 0)	/* lp time read invalidate */
#define SRTC_LPSR_PGD         (1 << 1)	/* lp power supply glitc detected */
#define SRTC_LPSR_CTD         (1 << 2)	/* lp clock tampering detected */
#define SRTC_LPSR_ALP         (1 << 3)	/* lp alarm flag */
#define SRTC_LPSR_MR          (1 << 4)	/* lp monotonic counter rollover */
#define SRTC_LPSR_TR          (1 << 5)	/* lp time rollover */
#define SRTC_LPSR_EAD         (1 << 6)	/* lp external alarm detected */
#define SRTC_LPSR_IT0         (1 << 7)	/* lp IIM throttle */
#define SRTC_LPSR_IT1         (1 << 8)
#define SRTC_LPSR_IT2         (1 << 9)
#define SRTC_LPSR_SM0         (1 << 10)	/* lp security mode */
#define SRTC_LPSR_SM1         (1 << 11)
#define SRTC_LPSR_STATE_LP0   (1 << 12)	/* lp state */
#define SRTC_LPSR_STATE_LP1   (1 << 13)
#define SRTC_LPSR_NVES        (1 << 14)	/* lp non-valid state exit status */
#define SRTC_LPSR_IES         (1 << 15)	/* lp init state exit status */

#define MAX_PIE_NUM     15
#define MAX_PIE_FREQ    32768
#define MIN_PIE_FREQ	1

#define SRTC_PI0         (1 << 0)
#define SRTC_PI1         (1 << 1)
#define SRTC_PI2         (1 << 2)
#define SRTC_PI3         (1 << 3)
#define SRTC_PI4         (1 << 4)
#define SRTC_PI5         (1 << 5)
#define SRTC_PI6         (1 << 6)
#define SRTC_PI7         (1 << 7)
#define SRTC_PI8         (1 << 8)
#define SRTC_PI9         (1 << 9)
#define SRTC_PI10        (1 << 10)
#define SRTC_PI11        (1 << 11)
#define SRTC_PI12        (1 << 12)
#define SRTC_PI13        (1 << 13)
#define SRTC_PI14        (1 << 14)
#define SRTC_PI15        (1 << 15)

#define PIT_ALL_ON      (SRTC_PI1 | SRTC_PI2 | SRTC_PI3 | \
			SRTC_PI4 | SRTC_PI5 | SRTC_PI6 | SRTC_PI7 | \
			SRTC_PI8 | SRTC_PI9 | SRTC_PI10 | SRTC_PI11 | \
			SRTC_PI12 | SRTC_PI13 | SRTC_PI14 | SRTC_PI15)

#define SRTC_SWR_HP      (1 << 0)	/* hp software reset */
#define SRTC_EN_HP       (1 << 3)	/* hp enable */
#define SRTC_TS          (1 << 4)	/* time syncronize hp with lp */

#define SRTC_IE_AHP      (1 << 16)	/* Alarm HP Interrupt Enable bit */
#define SRTC_IE_WDHP     (1 << 18)	/* Write Done HP Interrupt Enable bit */
#define SRTC_IE_WDLP     (1 << 19)	/* Write Done LP Interrupt Enable bit */

#define SRTC_ISR_AHP     (1 << 16)	/* interrupt status: alarm hp */
#define SRTC_ISR_WDHP    (1 << 18)	/* interrupt status: write done hp */
#define SRTC_ISR_WDLP    (1 << 19)	/* interrupt status: write done lp */
#define SRTC_ISR_WPHP    (1 << 20)	/* interrupt status: write pending hp */
#define SRTC_ISR_WPLP    (1 << 21)	/* interrupt status: write pending lp */

#define SRTC_LPSCMR	0x00	/* LP Secure Counter MSB Reg */
#define SRTC_LPSCLR	0x04	/* LP Secure Counter LSB Reg */
#define SRTC_LPSAR	0x08	/* LP Secure Alarm Reg */
#define SRTC_LPSMCR	0x0C	/* LP Secure Monotonic Counter Reg */
#define SRTC_LPCR	0x10	/* LP Control Reg */
#define SRTC_LPSR	0x14	/* LP Status Reg */
#define SRTC_LPPDR	0x18	/* LP Power Supply Glitch Detector Reg */
#define SRTC_LPGR	0x1C	/* LP General Purpose Reg */
#define SRTC_HPCMR	0x20	/* HP Counter MSB Reg */
#define SRTC_HPCLR	0x24	/* HP Counter LSB Reg */
#define SRTC_HPAMR	0x28	/* HP Alarm MSB Reg */
#define SRTC_HPALR	0x2C	/* HP Alarm LSB Reg */
#define SRTC_HPCR	0x30	/* HP Control Reg */
#define SRTC_HPISR	0x34	/* HP Interrupt Status Reg */
#define SRTC_HPIENR	0x38	/* HP Interrupt Enable Reg */

#define SRTC_SECMODE_MASK	0x3	/* the mask of SRTC security mode */
#define SRTC_SECMODE_LOW	0x0	/* Low Security */
#define SRTC_SECMODE_MED	0x1	/* Medium Security */
#define SRTC_SECMODE_HIGH	0x2	/* High Security */
#define SRTC_SECMODE_RESERVED	0x3	/* Reserved */

struct rtc_drv_data {
	struct rtc_device *rtc;
	void __iomem *ioaddr;
	unsigned long baseaddr;
	int irq;
	struct clk *clk;
	bool irq_enable;
};

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-mxc.c
 * @brief Real Time Clock interface
 *
 * This file contains Real Time Clock interface for Linux.
 *
 * @ingroup RTC
 */

static unsigned long rtc_status;

static DEFINE_SPINLOCK(rtc_lock);

/*!
 * This function does write synchronization for writes to the lp srtc block.
 * To take care of the asynchronous CKIL clock, all writes from the IP domain
 * will be synchronized to the CKIL domain.
 */
static inline void rtc_write_sync_lp(void __iomem *ioaddr)
{
	while ((__raw_readl(ioaddr + SRTC_HPISR) & SRTC_ISR_WPLP) != 0)
		msleep(1);
	while ((__raw_readl(ioaddr + SRTC_HPISR) & SRTC_ISR_WDLP) == 0)
		msleep(1);
	__raw_writel(SRTC_ISR_WDLP, ioaddr + SRTC_HPISR);
	while ((__raw_readl(ioaddr + SRTC_HPISR) & SRTC_ISR_WPHP) != 0)
		msleep(1);
}

static inline void rtc_write_sync_lp_no_wait(void __iomem *ioaddr)
{
	while ((__raw_readl(ioaddr + SRTC_HPISR) & SRTC_ISR_WPLP) != 0);
	while ((__raw_readl(ioaddr + SRTC_HPISR) & SRTC_ISR_WDLP) == 0);
	__raw_writel(SRTC_ISR_WDLP, ioaddr + SRTC_HPISR);
	while ((__raw_readl(ioaddr + SRTC_HPISR) & SRTC_ISR_WPHP) != 0);
}

/*!
 * This function updates the RTC alarm registers and then clears all the
 * interrupt status bits.
 *
 * @param  alrm         the new alarm value to be updated in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int rtc_update_alarm(struct device *dev, struct rtc_time *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;

	now = __raw_readl(ioaddr + SRTC_LPSCMR);
	rtc_time_to_tm(now, &now_tm);

	alarm_tm.tm_year = now_tm.tm_year;
	alarm_tm.tm_mon = now_tm.tm_mon;
	alarm_tm.tm_mday = now_tm.tm_mday;

	alarm_tm.tm_hour = alrm->tm_hour;
	alarm_tm.tm_min = alrm->tm_min;
	alarm_tm.tm_sec = alrm->tm_sec;

	rtc_tm_to_time(&now_tm, &now);
	rtc_tm_to_time(&alarm_tm, &time);

	if (time < now) {
		time += 60 * 60 * 24;
		rtc_time_to_tm(time, &alarm_tm);
	}
	ret = rtc_tm_to_time(&alarm_tm, &time);

	__raw_writel(time, ioaddr + SRTC_LPSAR);

	/* clear alarm interrupt status bit */
	__raw_writel(SRTC_LPSR_ALP, ioaddr + SRTC_LPSR);

	return ret;
}

/*!
 * This function is the RTC interrupt service routine.
 *
 * @param  irq          RTC IRQ number
 * @param  dev_id       device ID which is not used
 *
 * @return IRQ_HANDLED as defined in the include/linux/interrupt.h file.
 */
static irqreturn_t mxc_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 lp_status, lp_cr;
	u32 events = 0;

	lp_status = __raw_readl(ioaddr + SRTC_LPSR);
	lp_cr = __raw_readl(ioaddr + SRTC_LPCR);

	/* update irq data & counter */
	if (lp_status & SRTC_LPSR_ALP) {
		if (lp_cr & SRTC_LPCR_ALP)
			events |= (RTC_AF | RTC_IRQF);

		/* disable further lp alarm interrupts */
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);
	}

	/* Update interrupt enables */
	__raw_writel(lp_cr, ioaddr + SRTC_LPCR);

	/* If no interrupts are enabled, turn off interrupts in kernel */
	if (((lp_cr & SRTC_LPCR_ALL_INT_EN) == 0) && (pdata->irq_enable)) {
		disable_irq(pdata->irq);
		pdata->irq_enable = false;
	}

	/* clear interrupt status */
	__raw_writel(lp_status, ioaddr + SRTC_LPSR);

	rtc_update_irq(pdata->rtc, 1, events);
	return IRQ_HANDLED;
}

/*!
 * This function is used to open the RTC driver.
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_open(struct device *dev)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	clk_enable(pdata->clk);

	if (test_and_set_bit(1, &rtc_status))
		return -EBUSY;
	return 0;
}

/*!
 * clear all interrupts and release the IRQ
 */
static void mxc_rtc_release(struct device *dev)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&rtc_lock, lock_flags);

	/* Disable all rtc interrupts */
	__raw_writel(__raw_readl(ioaddr + SRTC_LPCR) & ~(SRTC_LPCR_ALL_INT_EN),
		     ioaddr + SRTC_LPCR);

	/* Clear all interrupt status */
	__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);

	spin_unlock_irqrestore(&rtc_lock, lock_flags);

	rtc_write_sync_lp(ioaddr);
	clk_disable(pdata->clk);

	rtc_status = 0;
}

/*!
 * This function is used to support some ioctl calls directly.
 * Other ioctl calls are supported indirectly through the
 * arm/common/rtctime.c file.
 *
 * @param  cmd          ioctl command as defined in include/linux/rtc.h
 * @param  arg          value for the ioctl command
 *
 * @return  0 if successful or negative value otherwise.
 */
static int mxc_rtc_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long lock_flags = 0;
	u32 lp_cr;

	switch (cmd) {
	case RTC_AIE_OFF:
		spin_lock_irqsave(&rtc_lock, lock_flags);
		lp_cr = __raw_readl(ioaddr + SRTC_LPCR);
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);
		if (((lp_cr & SRTC_LPCR_ALL_INT_EN) == 0)
		    && (pdata->irq_enable)) {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
		__raw_writel(lp_cr, ioaddr + SRTC_LPCR);
		spin_unlock_irqrestore(&rtc_lock, lock_flags);
		return 0;

	case RTC_AIE_ON:
		spin_lock_irqsave(&rtc_lock, lock_flags);
		if (!pdata->irq_enable) {
			enable_irq(pdata->irq);
			pdata->irq_enable = true;
		}
		lp_cr = __raw_readl(ioaddr + SRTC_LPCR);
		lp_cr |= SRTC_LPCR_ALP | SRTC_LPCR_WAE;
		__raw_writel(lp_cr, ioaddr + SRTC_LPCR);
		spin_unlock_irqrestore(&rtc_lock, lock_flags);
		return 0;
	}

	return -ENOIOCTLCMD;
}

/*!
 * This function reads the current RTC time into tm in Gregorian date.
 *
 * @param  tm           contains the RTC time value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	rtc_time_to_tm(__raw_readl(ioaddr + SRTC_LPSCMR), tm);
	return 0;
}

/*!
 * This function sets the internal RTC time based on tm in Gregorian date.
 *
 * @param  tm           the time value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long time;
	int ret;
	ret = rtc_tm_to_time(tm, &time);
	if (ret != 0)
		return ret;

	__raw_writel(time, ioaddr + SRTC_LPSCMR);
	rtc_write_sync_lp(ioaddr);

	return 0;
}

/*!
 * This function reads the current alarm value into the passed in \b alrm
 * argument. It updates the \b alrm's pending field value based on the whether
 * an alarm interrupt occurs or not.
 *
 * @param  alrm         contains the RTC alarm value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	rtc_time_to_tm(__raw_readl(ioaddr + SRTC_LPSAR), &alrm->time);
	alrm->pending =
	    ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_ALP) != 0) ? 1 : 0;

	return 0;
}

/*!
 * This function sets the RTC alarm based on passed in alrm.
 *
 * @param  alrm         the alarm value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long lock_flags = 0;
	u32 lp_cr;
	int ret;

	if (rtc_valid_tm(&alrm->time)) {
		if (alrm->time.tm_sec > 59 ||
		    alrm->time.tm_hour > 23 || alrm->time.tm_min > 59) {
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&rtc_lock, lock_flags);
	lp_cr = __raw_readl(ioaddr + SRTC_LPCR);

	ret = rtc_update_alarm(dev, &alrm->time);
	if (ret)
		goto out;

	if (alrm->enabled)
		lp_cr |= (SRTC_LPCR_ALP | SRTC_LPCR_WAE);
	else
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);

	if (lp_cr & SRTC_LPCR_ALL_INT_EN) {
		if (!pdata->irq_enable) {
			enable_irq(pdata->irq);
			pdata->irq_enable = true;
		}
	} else {
		if (pdata->irq_enable) {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
	}

	__raw_writel(lp_cr, ioaddr + SRTC_LPCR);

out:
	spin_unlock_irqrestore(&rtc_lock, lock_flags);
	rtc_write_sync_lp(ioaddr);
	return ret;
}

/*!
 * This function is used to provide the content for the /proc/driver/rtc
 * file.
 *
 * @param  seq  buffer to hold the information that the driver wants to write
 *
 * @return  The number of bytes written into the rtc file.
 */
static int mxc_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	clk_enable(pdata->clk);
	seq_printf(seq, "alarm_IRQ\t: %s\n",
		   (((__raw_readl(ioaddr + SRTC_LPCR)) & SRTC_LPCR_ALP) !=
		    0) ? "yes" : "no");
	clk_disable(pdata->clk);

	return 0;
}

/*!
 * The RTC driver structure
 */
static struct rtc_class_ops mxc_rtc_ops = {
	.open = mxc_rtc_open,
	.release = mxc_rtc_release,
	.ioctl = mxc_rtc_ioctl,
	.read_time = mxc_rtc_read_time,
	.set_time = mxc_rtc_set_time,
	.read_alarm = mxc_rtc_read_alarm,
	.set_alarm = mxc_rtc_set_alarm,
	.proc = mxc_rtc_proc,
};

/*! MXC RTC Power management control */

static struct timespec mxc_rtc_delta;

static int mxc_rtc_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct timespec tv;
	struct resource *res;
	struct rtc_device *rtc;
	struct rtc_drv_data *pdata = NULL;
	struct mxc_srtc_platform_data *plat_data = NULL;
	void __iomem *ioaddr;
	void __iomem *srtc_secmode_addr;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->clk = clk_get(&pdev->dev, "rtc_clk");
	clk_enable(pdata->clk);
	pdata->baseaddr = res->start;
	pdata->ioaddr = ioremap(pdata->baseaddr, 0x40);
	ioaddr = pdata->ioaddr;

	/* Configure and enable the RTC */
	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq >= 0) {
		if (request_irq(pdata->irq, mxc_rtc_interrupt, IRQF_SHARED,
				pdev->name, pdev) < 0) {
			dev_warn(&pdev->dev, "interrupt not available.\n");
			pdata->irq = -1;
		} else {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
	}

	clk = clk_get(NULL, "rtc_clk");
	if (clk_get_rate(clk) != 32768) {
		printk(KERN_ALERT "rtc clock is not valid");
		ret = -EINVAL;
		clk_put(clk);
		goto err_out;
	}
	clk_put(clk);

	/* initialize glitch detect */
	__raw_writel(SRTC_LPPDR_INIT, ioaddr + SRTC_LPPDR);
	rtc_write_sync_lp_no_wait(ioaddr);

	/* clear lp interrupt status */
	__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);
	rtc_write_sync_lp_no_wait(ioaddr);

	plat_data = (struct mxc_srtc_platform_data *)pdev->dev.platform_data;
	clk = clk_get(NULL, "iim_clk");
	clk_enable(clk);
	srtc_secmode_addr = ioremap(plat_data->srtc_sec_mode_addr, 1);

	/* Check SRTC security mode */
	if (((__raw_readl(srtc_secmode_addr) & SRTC_SECMODE_MASK) ==
	    SRTC_SECMODE_LOW) && (cpu_is_mx51_rev(CHIP_REV_1_0) == 1)) {
		/* Workaround for MX51 TO1 due to inaccurate CKIL clock */
		__raw_writel(SRTC_LPCR_EN_LP, ioaddr + SRTC_LPCR);
		rtc_write_sync_lp_no_wait(ioaddr);
	} else {
		/* move out of init state */
		__raw_writel((SRTC_LPCR_IE | SRTC_LPCR_NSA),
			     ioaddr + SRTC_LPCR);

		rtc_write_sync_lp_no_wait(ioaddr);

		while ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_IES) == 0);

		/* move out of non-valid state */
		__raw_writel((SRTC_LPCR_IE | SRTC_LPCR_NVE | SRTC_LPCR_NSA |
			      SRTC_LPCR_EN_LP), ioaddr + SRTC_LPCR);

		rtc_write_sync_lp_no_wait(ioaddr);

		while ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_NVES) == 0);

		__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);
		rtc_write_sync_lp_no_wait(ioaddr);
	}
	clk_disable(clk);
	clk_put(clk);

	rtc = rtc_device_register(pdev->name, &pdev->dev,
				  &mxc_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		goto err_out;
	}

	pdata->rtc = rtc;
	platform_set_drvdata(pdev, pdata);

	tv.tv_nsec = 0;
	tv.tv_sec = __raw_readl(ioaddr + SRTC_LPSCMR);

	/* By default, devices should wakeup if they can */
	/* So srtc is set as "should wakeup" as it can */
	device_init_wakeup(&pdev->dev, 1);

	clk_disable(pdata->clk);

	return ret;

err_out:
	clk_disable(pdata->clk);
	iounmap(ioaddr);
	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdev);
	kfree(pdata);
	return ret;
}

static int __exit mxc_rtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	rtc_device_unregister(pdata->rtc);
	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdev);

	clk_disable(pdata->clk);
	clk_put(pdata->clk);
	kfree(pdata);
	return 0;
}

/*!
 * This function is called to save the system time delta relative to
 * the MXC RTC when enterring a low power state. This time delta is
 * then used on resume to adjust the system time to account for time
 * loss while suspended.
 *
 * @param   pdev  not used
 * @param   state Power state to enter.
 *
 * @return  The function always returns 0.
 */
static int mxc_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	struct timespec tv;

	clk_enable(pdata->clk);
	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(pdata->irq);
	} else {
		/* calculate time delta for suspend.  RTC precision is
		   1 second; adjust delta for avg 1/2 sec err */
		tv.tv_nsec = NSEC_PER_SEC >> 1;
		tv.tv_sec = __raw_readl(ioaddr + SRTC_LPSCMR);
		set_normalized_timespec(&mxc_rtc_delta,
					xtime.tv_sec - tv.tv_sec,
					xtime.tv_nsec - tv.tv_nsec);

		if (pdata->irq_enable)
			disable_irq(pdata->irq);
	}

	clk_disable(pdata->clk);

	return 0;
}

/*!
 * This function is called to correct the system time based on the
 * current MXC RTC time relative to the time delta saved during
 * suspend.
 *
 * @param   pdev  not used
 *
 * @return  The function always returns 0.
 */
static int mxc_rtc_resume(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	struct timespec tv;
	struct timespec ts;

	clk_enable(pdata->clk);

	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(pdata->irq);
	} else {
		if (pdata->irq_enable)
			enable_irq(pdata->irq);

		tv.tv_nsec = 0;
		tv.tv_sec = __raw_readl(ioaddr + SRTC_LPSCMR);

		/*
		 * restore wall clock using delta against this RTC;
		 * adjust again for avg 1/2 second RTC sampling error
		 */
		set_normalized_timespec(&ts,
					tv.tv_sec + mxc_rtc_delta.tv_sec,
					(NSEC_PER_SEC >> 1) +
					mxc_rtc_delta.tv_nsec);
		do_settimeofday(&ts);
	}

	clk_disable(pdata->clk);
	return 0;
}

/*!
 * Contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_rtc_driver = {
	.driver = {
		   .name = "mxc_rtc",
		   },
	.probe = mxc_rtc_probe,
	.remove = __exit_p(mxc_rtc_remove),
	.suspend = mxc_rtc_suspend,
	.resume = mxc_rtc_resume,
};

/*!
 * This function creates the /proc/driver/rtc file and registers the device RTC
 * in the /dev/misc directory. It also reads the RTC value from external source
 * and setup the internal RTC properly.
 *
 * @return  -1 if RTC is failed to initialize; 0 is successful.
 */
static int __init mxc_rtc_init(void)
{
	return platform_driver_register(&mxc_rtc_driver);
}

/*!
 * This function removes the /proc/driver/rtc file and un-registers the
 * device RTC from the /dev/misc directory.
 */
static void __exit mxc_rtc_exit(void)
{
	platform_driver_unregister(&mxc_rtc_driver);

}

module_init(mxc_rtc_init);
module_exit(mxc_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
