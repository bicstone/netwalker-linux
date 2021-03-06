/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup GPIO_MX51 Board GPIO and Muxing Setup
 * @ingroup MSL_MX51
 */
/*!
 * @file mach-mx51/iomux.c
 *
 * @brief I/O Muxing control functions
 *
 * @ingroup GPIO_MX51
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include "iomux.h"

/*!
 * IOMUX register (base) addresses
 */
enum iomux_reg_addr {
	IOMUXGPR0 = IO_ADDRESS(IOMUXC_BASE_ADDR),
	IOMUXGPR1 = IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x004,
	IOMUXSW_MUX_CTL = IO_ADDRESS(IOMUXC_BASE_ADDR),
	IOMUXSW_MUX_END = IO_ADDRESS(IOMUXC_BASE_ADDR) + MUX_I_END,
	IOMUXSW_PAD_CTL = IO_ADDRESS(IOMUXC_BASE_ADDR) + PAD_I_START,
	IOMUXSW_INPUT_CTL = IO_ADDRESS(IOMUXC_BASE_ADDR),
};

#define MUX_PIN_NUM_MAX        ((MUX_I_END >> 2) + 1)

static u8 iomux_pin_res_table[MUX_PIN_NUM_MAX];
static DEFINE_SPINLOCK(gpio_mux_lock);

static inline u32 _get_mux_reg(iomux_pin_name_t pin)
{
	u32 mux_reg = PIN_TO_IOMUX_MUX(pin);

	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
		if ((pin == MX51_PIN_NANDF_RB5) ||
			(pin == MX51_PIN_NANDF_RB6) ||
			(pin == MX51_PIN_NANDF_RB7))
			; /* Do nothing */
		else if (mux_reg >= 0x2FC)
			mux_reg += 8;
		else if (mux_reg >= 0x130)
			mux_reg += 0xC;
	}
	mux_reg += IOMUXSW_MUX_CTL;
	return mux_reg;
}

static inline u32 _get_pad_reg(iomux_pin_name_t pin)
{
	u32 pad_reg = PIN_TO_IOMUX_PAD(pin);

	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
		if ((pin == MX51_PIN_NANDF_RB5) ||
			(pin == MX51_PIN_NANDF_RB6) ||
			(pin == MX51_PIN_NANDF_RB7))
			; /* Do nothing */
		else if (pad_reg == 0x4D0 - PAD_I_START)
			pad_reg += 0x4C;
		else if (pad_reg == 0x860 - PAD_I_START)
			pad_reg += 0x9C;
		else if (pad_reg >= 0x804 - PAD_I_START)
			pad_reg += 0xB0;
		else if (pad_reg >= 0x7FC - PAD_I_START)
			pad_reg += 0xB4;
		else if (pad_reg >= 0x4E4 - PAD_I_START)
			pad_reg += 0xCC;
		else
			pad_reg += 8;
	}
	pad_reg += IOMUXSW_PAD_CTL;
	return pad_reg;
}

static inline u32 _get_mux_end(void)
{
	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0)
		return(IO_ADDRESS(IOMUXC_BASE_ADDR) + (0x3F8 - 4));
	else
		return(IO_ADDRESS(IOMUXC_BASE_ADDR) + (0x3F0 - 4));
}

/*!
 * This function is used to configure a pin through the IOMUX module.
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return 		0 if successful; Non-zero otherwise
 */
static int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	u32 ret = 0;
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	u32 mux_reg = _get_mux_reg(pin);
	u32 mux_data = 0;
	u8 *rp;
	unsigned long flags;

	BUG_ON((mux_reg > _get_mux_end()) || (mux_reg < IOMUXSW_MUX_CTL));
	spin_lock_irqsave(&gpio_mux_lock, flags);

	if (config == IOMUX_CONFIG_GPIO)
		mux_data = PIN_TO_ALT_GPIO(pin);
	else
		mux_data = config;

	__raw_writel(mux_data, mux_reg);

	/*
	 * Log a warning if a pin changes ownership
	 */
	rp = iomux_pin_res_table + pin_index;
	if ((mux_data & *rp) && (*rp != mux_data)) {
		/*
		 * Don't call printk if we're tweaking the console uart or
		 * we'll deadlock.
		 */
		printk(KERN_ERR "iomux_config_mux: Warning: iomux pin"
		       " config changed, pin=%d, "
		       " prev=0x%x new=0x%x\n", mux_reg, *rp, mux_data);
		ret = -EINVAL;
	}
	*rp = mux_data;
	spin_unlock_irqrestore(&gpio_mux_lock, flags);
	return ret;
}

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	int ret = iomux_config_mux(pin, config);
	int gpio_port = GPIO_TO_PORT(IOMUX_TO_GPIO(pin));
	unsigned long flags;

	spin_lock_irqsave(&gpio_mux_lock, flags);

	if (!ret && (gpio_port != NON_GPIO_PORT)
	    && ((config == IOMUX_CONFIG_GPIO)
		|| (config == PIN_TO_ALT_GPIO(pin))))
		ret |= mxc_request_gpio(pin);

	spin_unlock_irqrestore(&gpio_mux_lock, flags);

	return ret;
}
EXPORT_SYMBOL(mxc_request_iomux);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	config as defined in \b #iomux_pin_ocfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_cfg_t config)
{
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	u8 *rp = iomux_pin_res_table + pin_index;
	int gpio_port = GPIO_TO_PORT(IOMUX_TO_GPIO(pin));
	unsigned long flags;

	spin_lock_irqsave(&gpio_mux_lock, flags);

	*rp = 0;
	if ((gpio_port != NON_GPIO_PORT)
	    && ((config == IOMUX_CONFIG_GPIO)
		|| (config == PIN_TO_ALT_GPIO(pin))))
		mxc_free_gpio(pin);

	spin_unlock_irqrestore(&gpio_mux_lock, flags);
}
EXPORT_SYMBOL(mxc_free_iomux);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
	u32 pad_reg = _get_pad_reg(pin);

	BUG_ON(pad_reg < IOMUXSW_PAD_CTL);
	__raw_writel(config, pad_reg);
}
EXPORT_SYMBOL(mxc_iomux_set_pad);

unsigned int mxc_iomux_get_pad(iomux_pin_name_t pin)
{
	u32 pad_reg = _get_pad_reg(pin);

	return __raw_readl(pad_reg);
}
EXPORT_SYMBOL(mxc_iomux_get_pad);

/*!
 * This function configures input path.
 *
 * @param  input        index of input select register as defined in \b #iomux_input_select_t
 * @param  config       the binary value of elements defined in \b #iomux_input_config_t
 *      */
void mxc_iomux_set_input(iomux_input_select_t input, u32 config)
{
	u32 reg;

	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
		if (input == MUX_IN_IPU_IPP_DI_0_IND_DISPB_SD_D_SELECT_INPUT)
			input -= 4;
		else if (input == MUX_IN_IPU_IPP_DI_1_IND_DISPB_SD_D_SELECT_INPUT)
			input -= 3;
		else if (input >= MUX_IN_KPP_IPP_IND_COL_6_SELECT_INPUT)
			input -= 2;
		else if (input >= MUX_IN_HSC_MIPI_MIX_PAR_SISG_TRIG_SELECT_INPUT)
			input -= 5;
		else if (input >= MUX_IN_HSC_MIPI_MIX_IPP_IND_SENS1_DATA_EN_SELECT_INPUT)
			input -= 3;
		else if (input >= MUX_IN_ECSPI2_IPP_IND_SS_B_3_SELECT_INPUT)
			input -= 2;
		else if (input >= MUX_IN_CCM_PLL1_BYPASS_CLK_SELECT_INPUT)
			input -= 1;

		reg = IOMUXSW_INPUT_CTL + (input << 2) + INPUT_CTL_START_TO1;
	} else {
		reg = IOMUXSW_INPUT_CTL + (input << 2) + INPUT_CTL_START;
	}

	BUG_ON(input >= MUX_INPUT_NUM_MUX);
	__raw_writel(config, reg);
}
EXPORT_SYMBOL(mxc_iomux_set_input);
