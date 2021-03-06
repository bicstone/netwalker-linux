/*
 * Freescale STMP378X voltage regulator low-level driver
 *
 * Embedded Alley Solutions, Inc <source@embeddedalley.com>
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/* #define DEBUG */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <mach/power.h>
#include <mach/regulator.h>
#include <mach/regs-power.h>
#include <mach/stmp3xxx.h>

static int get_voltage(struct stmp3xxx_regulator *sreg)
{
	struct stmp3xxx_platform_regulator_data *rdata = sreg->rdata;
	u32 val = __raw_readl(rdata->control_reg) & 0x1f;
	int uv  = rdata->min_voltage + val *
		  (rdata->max_voltage - rdata->min_voltage) / 0x1f;
	return uv;
}

static int get_bo_voltage(struct stmp3xxx_regulator *sreg)
{
	int uv;
	int offs;

	if (!sreg->parent)
		return -EINVAL;

	uv = get_voltage(sreg->parent);
	offs = (__raw_readl(sreg->parent->rdata->control_reg) & ~0x700) >> 8;
	return uv - 25000*offs;
}

static int set_voltage(struct stmp3xxx_regulator *sreg, int uv)
{
	u32 val, reg, i;

	pr_debug("%s: uv %d, min %d, max %d\n", __func__,
		uv, sreg->rdata->min_voltage, sreg->rdata->max_voltage);

	if (uv < sreg->rdata->min_voltage || uv > sreg->rdata->max_voltage)
		return -EINVAL;

	val = (uv - sreg->rdata->min_voltage) * 0x1f /
			(sreg->rdata->max_voltage - sreg->rdata->min_voltage);
	reg = (__raw_readl(sreg->rdata->control_reg) & ~0x1f);
	pr_debug("%s: calculated val %d\n", __func__, val);
	__raw_writel(val | reg, sreg->rdata->control_reg);
	for (i = 20; i; i--) {
		if (HW_POWER_STS_RD() & BM_POWER_STS_DC_OK)
			break;
		udelay(1);
	}

	if (i)
		goto out;

	__raw_writel(val | reg, sreg->rdata->control_reg);
	for (i = 40000; i; i--) {
		if (HW_POWER_STS_RD() & BM_POWER_STS_DC_OK)
			break;
		udelay(1);
	}

	if (i)
		goto out;

	for (i = 40000; i; i--) {
		if (HW_POWER_STS_RD() & BM_POWER_STS_DC_OK)
			break;
		udelay(1);
	}

out:
	return !i;
}

static int set_bo_voltage(struct stmp3xxx_regulator *sreg, int bo_uv)
{
	int uv;
	int offs;
	u32 reg;
	int i;

	if (!sreg->parent)
		return -EINVAL;

	uv = get_voltage(sreg->parent);
	offs = (uv - bo_uv) / 25000;
	if (offs < 0 || offs > 7)
		return -EINVAL;

	reg = (__raw_readl(sreg->parent->rdata->control_reg) & ~0x700);
	pr_debug("%s: calculated offs %d\n", __func__, offs);
	__raw_writel((offs << 8) | reg, sreg->parent->rdata->control_reg);

	for (i = 10000; i; i--) {
		if (HW_POWER_STS_RD() & BM_POWER_STS_DC_OK)
			break;
		udelay(1);
	}

	if (i)
		goto out;

	for (i = 10000; i; i--) {
		if (HW_POWER_STS_RD() & BM_POWER_STS_DC_OK)
			break;
		udelay(1);
	}

out:
	return !i;
}

static int enable(struct stmp3xxx_regulator *sreg)
{
	/* XXX: TODO */
	return 0;
}

static int disable(struct stmp3xxx_regulator *sreg)
{
	/* XXX: TODO */
	return 0;
}

static int is_enabled(struct stmp3xxx_regulator *sreg)
{
	/* XXX: TODO */
	return 1;
}

static int set_mode(struct stmp3xxx_regulator *sreg, int mode)
{
	int ret = 0;
	u32 val;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		val = __raw_readl(sreg->rdata->control_reg);
		__raw_writel(val | (1 << 17), sreg->rdata->control_reg);
		break;

	case REGULATOR_MODE_NORMAL:
		val = __raw_readl(sreg->rdata->control_reg);
		__raw_writel(val & ~(1<<17), sreg->rdata->control_reg);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int get_mode(struct stmp3xxx_regulator *sreg)
{
	u32 val = __raw_readl(sreg->rdata->control_reg) & (1 << 17);

	return val ? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;
}

static struct stmp3xxx_platform_regulator_data vddd_data = {
	.name		= "vddd",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.set_mode	= set_mode,
	.get_mode	= get_mode,
	.control_reg	= HW_POWER_VDDDCTRL_ADDR,
	.min_voltage	= 800000,
	.max_voltage	= 1575000,
};

static struct stmp3xxx_platform_regulator_data vdddbo_data = {
	.name		= "vddd_bo",
	.parent_name	= "vddd",
	.set_voltage	= set_bo_voltage,
	.get_voltage	= get_bo_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.set_mode	= set_mode,
	.get_mode	= get_mode,
	.min_voltage	= 800000,
	.max_voltage	= 1575000,
};

static struct stmp3xxx_platform_regulator_data vdda_data = {
	.name		= "vdda",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.set_mode	= set_mode,
	.get_mode	= get_mode,
	.control_reg	= HW_POWER_VDDACTRL_ADDR,
	.min_voltage	= 1500000,
	.max_voltage	= 2275000,
};

static struct stmp3xxx_platform_regulator_data vddio_data = {
	.name		= "vddio",
	.set_voltage	= set_voltage,
	.get_voltage	= get_voltage,
	.enable		= enable,
	.disable	= disable,
	.is_enabled	= is_enabled,
	.set_mode	= set_mode,
	.get_mode	= get_mode,
	.control_reg	= HW_POWER_VDDIOCTRL_ADDR,
	.min_voltage	= 2800000,
	.max_voltage	= 3575000,
};

static struct regulator_init_data vddd_init = {
	.constraints = {
		.name			= "vddd",
		.min_uV			= 800000,
		.max_uV			= 1575000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.input_uV		= 5000000,
	}
};

static struct regulator_init_data vdddbo_init = {
	.constraints = {
		.name			= "vdddbo",
		.min_uV			= 800000,
		.max_uV			= 1575000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.input_uV		= 5000000,
	}
};


static struct regulator_init_data vdda_init = {
	.constraints = {
		.name			= "vdda",
		.min_uV			= 1500000,
		.max_uV			= 2275000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.input_uV		= 5000000,
	}
};


static struct regulator_init_data vddio_init = {
	.constraints = {
		.name			= "vddio",
		.min_uV			= 2800000,
		.max_uV			= 3575000,
		.valid_modes_mask	= REGULATOR_MODE_FAST |
					  REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_MODE,
		.input_uV		= 5000000,
	}
};

/* now the current regulators */
/* Restriction: .... no set_current call on root regulator */
static int main_add_current(struct stmp3xxx_regulator *sreg,
			    int uA)
{

	pr_debug("%s: enter reg %s, uA=%d\n",
		 __func__, sreg->regulator.name, uA);
	if (uA > 0 && (sreg->cur_current + uA > sreg->rdata->max_current))
		return -EINVAL;
	else
		sreg->cur_current += uA;
	return 0;
}

static int cur_reg_set_current(struct stmp3xxx_regulator *sreg, int uA)
{
	int ret = 0;
	unsigned long flags;

	pr_debug("%s: enter reg %s, uA=%d\n",
		 __func__, sreg->regulator.name, uA);

	if (sreg->parent) {
		spin_lock_irqsave(&sreg->parent->lock, flags);
		ret = main_add_current(sreg->parent, uA - sreg->cur_current);
		spin_unlock_irqrestore(&sreg->parent->lock, flags);
	}


	if ((!ret) || (!sreg->parent))
		goto out;

	if (sreg->mode == REGULATOR_MODE_FAST)
		return ret;

	while (ret) {
		wait_event(sreg->parent->wait_q ,
			   (uA - sreg->cur_current <
			    sreg->parent->rdata->max_current -
			    sreg->parent->cur_current));
		spin_lock_irqsave(&sreg->parent->lock, flags);
		ret = main_add_current(sreg->parent, uA - sreg->cur_current);
		spin_unlock_irqrestore(&sreg->parent->lock, flags);
	}
out:
	if (sreg->parent && (uA - sreg->cur_current < 0))
		wake_up_all(&sreg->parent->wait_q);
	sreg->cur_current = uA;
	return 0;

}

static int cur_reg_get_current(struct stmp3xxx_regulator *sreg)
{
	return sreg->cur_current;
}

static int enable_cur_reg(struct stmp3xxx_regulator *sreg)
{
	/* XXX: TODO */
	return 0;
}

static int disable_cur_reg(struct stmp3xxx_regulator *sreg)
{
	/* XXX: TODO */
	return 0;
}

static int cur_reg_is_enabled(struct stmp3xxx_regulator *sreg)
{
	/* XXX: TODO */
	return 1;
}

static int cur_reg_set_mode(struct stmp3xxx_regulator *sreg, int mode)
{
	int ret = 0;

	switch (mode) {
	case REGULATOR_MODE_NORMAL:
	case REGULATOR_MODE_FAST:
		sreg->mode = mode;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int cur_reg_get_mode(struct stmp3xxx_regulator *sreg)
{
	return sreg->mode;
}

static struct stmp3xxx_platform_regulator_data overall_cur_data = {
	.name		= "overall_current",
	.set_current	= cur_reg_set_current,
	.get_current	= cur_reg_get_current,
	.enable		= enable_cur_reg,
	.disable	= disable_cur_reg,
	.is_enabled	= cur_reg_is_enabled,
	.set_mode	= cur_reg_set_mode,
	.get_mode	= cur_reg_get_mode,
	.max_current	= 0x7fffffff,
};

static struct regulator_init_data overall_cur_init = {
	.constraints = {
		.name			= "overall_current",
		.valid_modes_mask	= REGULATOR_MODE_NORMAL |
					  REGULATOR_MODE_FAST,
		.valid_ops_mask		= REGULATOR_CHANGE_CURRENT |
					  REGULATOR_CHANGE_MODE,
		.max_uA                 = 0x7fffffff,
		.min_uA                 = 0x0,
	}
};

static struct stmp3xxx_platform_regulator_data sibling_cur_data = {
	.parent_name	= "overall_current",
	.set_current	= cur_reg_set_current,
	.get_current	= cur_reg_get_current,
	.enable		= enable_cur_reg,
	.disable	= disable_cur_reg,
	.is_enabled	= cur_reg_is_enabled,
	.set_mode	= cur_reg_set_mode,
	.get_mode	= cur_reg_get_mode,
};

static struct platform_device *devices[] = {
	&stmp3xxx_keyboard,
	&stmp3xxx_touchscreen,
	&stmp3xxx_appuart,
	&stmp3xxx_dbguart,
	&stmp3xxx_watchdog,
	&stmp3xxx_rtc,
	&stmp3xxx_framebuffer,
	&stmp3xxx_backlight,
	&stmp3xxx_rotdec,
	&stmp378x_i2c,
	&stmp3xxx_persistent,
	&stmp3xxx_dcp_bootstream,
	&stmp3xxx_dcp,
	&stmp3xxx_mtest,
	&stmp3xxx_battery,
	&stmp3xxx_pxp,
};

static int sibling_current_devices_num;

int stmp3xxx_platform_add_regulator(const char *name, int count)
{
	int i;
	pr_debug("%s: name %s, count %d\n", __func__, name, count);
	for (i = sibling_current_devices_num;
	     i < sibling_current_devices_num + count;
	     i++) {
		struct regulator_init_data *sibling_init =
			kzalloc(sizeof(struct regulator_init_data),
			GFP_KERNEL);
		struct stmp3xxx_regulator *curr_reg =
			kzalloc(sizeof(struct stmp3xxx_regulator),
			GFP_KERNEL);
		struct stmp3xxx_platform_regulator_data *d =
			kzalloc(sizeof(struct stmp3xxx_platform_regulator_data),
			GFP_KERNEL);
		if (!d || !curr_reg || !sibling_init)
			return -ENOMEM;

		sibling_init->constraints.valid_modes_mask =
			REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST;
		sibling_init->constraints.valid_ops_mask =
			REGULATOR_CHANGE_CURRENT | REGULATOR_CHANGE_MODE;
		sibling_init->constraints.max_uA = 0x7fffffff;
		sibling_init->constraints.min_uA = 0x0;

		memcpy(d, &sibling_cur_data, sizeof(sibling_cur_data));
		d->parent_name = kstrdup(sibling_cur_data.parent_name,
					 GFP_KERNEL);
		snprintf(d->name, 80, "%s-%d",
			 name, i - sibling_current_devices_num + 1);
		sibling_init->constraints.name = kstrdup(d->name, GFP_KERNEL);
		curr_reg->rdata = d;
		stmp3xxx_register_regulator(curr_reg, 101 + i, sibling_init);
	}
	sibling_current_devices_num += count;
	return 0;
}

static struct stmp3xxx_regulator vddd_reg = {
		.rdata = &vddd_data,
};

static struct stmp3xxx_regulator vdda_reg = {
		.rdata = &vdda_data,
};

static struct stmp3xxx_regulator vddio_reg = {
		.rdata = &vddio_data,
};

static struct stmp3xxx_regulator vdddbo_reg = {
		.rdata = &vdddbo_data,
};

static struct stmp3xxx_regulator overall_cur_reg = {
		.rdata = &overall_cur_data,
};


static int __init regulators_init(void)
{
	int i;
	int retval = 0;
	u32 vddio = HW_POWER_VDDIOCTRL_RD() & ~0x1f;
	pr_debug("regulators_init \n");
	HW_POWER_VDDIOCTRL_WR(vddio | 0x14);
	vdddbo_reg.parent = &vddd_reg;
	stmp3xxx_register_regulator(&vddd_reg, STMP3XXX_VDDD, &vddd_init);
	stmp3xxx_register_regulator(&vdddbo_reg, STMP3XXX_VDDDBO, &vdddbo_init);
	stmp3xxx_register_regulator(&vdda_reg, STMP3XXX_VDDA, &vdda_init);
	stmp3xxx_register_regulator(&vddio_reg, STMP3XXX_VDDIO, &vddio_init);
	stmp3xxx_register_regulator(&overall_cur_reg,
		STMP3XXX_OVERALL_CUR, &overall_cur_init);

	for (i = 0; i < ARRAY_SIZE(devices); i++) {
		retval = stmp3xxx_platform_add_regulator(devices[i]->name, 1);
		if (retval)
			return retval;
	}
	stmp3xxx_platform_add_regulator("mmc_ssp", 2);
	stmp3xxx_platform_add_regulator("charger", 1);
	stmp3xxx_platform_add_regulator("power-test", 1);
	stmp3xxx_platform_add_regulator("cpufreq", 1);
	return 0;
}
postcore_initcall(regulators_init);
