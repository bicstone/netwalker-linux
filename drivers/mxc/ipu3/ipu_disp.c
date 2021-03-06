/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ipu_disp.c
 *
 * @brief IPU display submodule API functions
 *
 * @ingroup IPU
 *
 * modification information
 * ------------------------
 * 2009/08/02 : merge L2.6.28_4.4.0_SS_Jul2009_source.
 *
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <asm/atomic.h>
#include "ipu_prv.h"
#include "ipu_regs.h"
#include "ipu_param_mem.h"

enum csc_type_t {
	RGB2YUV = 0,
	YUV2RGB,
	RGB2RGB,
	YUV2YUV,
	CSC_NONE,
	CSC_NUM
};

struct dp_csc_param_t {
	int mode;
	void *coeff;
};

#define SYNC_WAVE 0
#define ASYNC_SER_WAVE 6

/* DC display ID assignments */
#define DC_DISP_ID_SYNC(di)	(di)
#define DC_DISP_ID_SERIAL	2
#define DC_DISP_ID_ASYNC	3


/* all value below is determined by fix reg setting in _ipu_dmfc_init*/
#define DMFC_FIFO_SIZE_28	(128*4)
#define DMFC_FIFO_SIZE_29	(64*4)
#define DMFC_FIFO_SIZE_24	(64*4)
#define DMFC_FIFO_SIZE_27	(128*4)
#define DMFC_FIFO_SIZE_23	(128*4)

void _ipu_dmfc_init(void)
{
	/* disable DMFC-IC channel*/
	__raw_writel(0x2, DMFC_IC_CTRL);
	/* 1 - segment 0 and 1; 2, 1C and 2C unused */
	__raw_writel(0x00000088, DMFC_WR_CHAN);
	__raw_writel(0x202020F6, DMFC_WR_CHAN_DEF);
	/* 5B - segment 2 and 3; 5F - segment 4 and 5; */
	/* 6B - segment 6; 6F - segment 7 */
	__raw_writel(0x1F1E9694, DMFC_DP_CHAN);
	/* Enable chan 5 watermark set at 5 bursts and clear at 7 bursts */
	__raw_writel(0x2020F6F6, DMFC_DP_CHAN_DEF);
}

void _ipu_dmfc_set_wait4eot(int dma_chan, int width)
{
	u32 dmfc_gen1 = __raw_readl(DMFC_GENERAL1);

	if (dma_chan == 23) { /*5B*/
		if (DMFC_FIFO_SIZE_23/width > 3)
			dmfc_gen1 |= 1UL << 20;
		else
			dmfc_gen1 &= ~(1UL << 20);
	} else if (dma_chan == 24) { /*6B*/
		if (DMFC_FIFO_SIZE_24/width > 1)
			dmfc_gen1 |= 1UL << 22;
		else
			dmfc_gen1 &= ~(1UL << 22);
	} else if (dma_chan == 27) { /*5F*/
		if (DMFC_FIFO_SIZE_27/width > 2)
			dmfc_gen1 |= 1UL << 21;
		else
			dmfc_gen1 &= ~(1UL << 21);
	} else if (dma_chan == 28) { /*1*/
		if (DMFC_FIFO_SIZE_28/width > 2)
			dmfc_gen1 |= 1UL << 16;
		else
			dmfc_gen1 &= ~(1UL << 16);
	} else if (dma_chan == 29) { /*6F*/
		if (DMFC_FIFO_SIZE_29/width > 1)
			dmfc_gen1 |= 1UL << 23;
		else
			dmfc_gen1 &= ~(1UL << 23);
	}

	__raw_writel(dmfc_gen1, DMFC_GENERAL1);
}

static void _ipu_di_data_wave_config(int di,
				     int wave_gen,
				     int access_size, int component_size)
{
	u32 reg;
	reg = (access_size << DI_DW_GEN_ACCESS_SIZE_OFFSET) |
	    (component_size << DI_DW_GEN_COMPONENT_SIZE_OFFSET);
	__raw_writel(reg, DI_DW_GEN(di, wave_gen));
}

static void _ipu_di_data_pin_config(int di, int wave_gen, int di_pin, int set,
				    int up, int down)
{
	u32 reg;

	reg = __raw_readl(DI_DW_GEN(di, wave_gen));
	reg &= ~(0x3 << (di_pin * 2));
	reg |= set << (di_pin * 2);
	__raw_writel(reg, DI_DW_GEN(di, wave_gen));

	__raw_writel((down << 16) | up, DI_DW_SET(di, wave_gen, set));
}

static void _ipu_di_sync_config(int di, int wave_gen,
				int run_count, int run_src,
				int offset_count, int offset_src,
				int repeat_count, int cnt_clr_src,
				int cnt_polarity_gen_en,
				int cnt_polarity_clr_src,
				int cnt_polarity_trigger_src,
				int cnt_up, int cnt_down)
{
	if ((run_count >= 0x1000) || (offset_count >= 0x1000) || (repeat_count >= 0x1000) ||
		(cnt_up >= 0x400) || (cnt_down >= 0x400)) {
		dev_err(g_ipu_dev, "DI%d counters out of range.\n", di);
		return;
	}

	u32 reg;
	reg = (run_count << 19) | (++run_src << 16) |
	    (offset_count << 3) | ++offset_src;
	__raw_writel(reg, DI_SW_GEN0(di, wave_gen));
	reg = (cnt_polarity_gen_en << 29) | (++cnt_clr_src << 25) |
	    (++cnt_polarity_trigger_src << 12) | (++cnt_polarity_clr_src << 9);
	reg |= (cnt_down << 16) | cnt_up;
	if (repeat_count == 0) {
		/* Enable auto reload */
		reg |= 0x10000000;
	}
	__raw_writel(reg, DI_SW_GEN1(di, wave_gen));
	reg = __raw_readl(DI_STP_REP(di, wave_gen));
	reg &= ~(0xFFFF << (16 * ((wave_gen - 1) & 0x1)));
	reg |= repeat_count << (16 * ((wave_gen - 1) & 0x1));
	__raw_writel(reg, DI_STP_REP(di, wave_gen));
}

static void _ipu_dc_map_config(int map, int byte_num, int offset, int mask)
{
	int ptr = map * 3 + byte_num;
	u32 reg;

	reg = __raw_readl(DC_MAP_CONF_VAL(ptr));
	reg &= ~(0xFFFF << (16 * (ptr & 0x1)));
	reg |= ((offset << 8) | mask) << (16 * (ptr & 0x1));
	__raw_writel(reg, DC_MAP_CONF_VAL(ptr));

	reg = __raw_readl(DC_MAP_CONF_PTR(map));
	reg &= ~(0x1F << ((16 * (map & 0x1)) + (5 * byte_num)));
	reg |= ptr << ((16 * (map & 0x1)) + (5 * byte_num));
	__raw_writel(reg, DC_MAP_CONF_PTR(map));
}

static void _ipu_dc_map_clear(int map)
{
	u32 reg = __raw_readl(DC_MAP_CONF_PTR(map));
	__raw_writel(reg & ~(0xFFFF << (16 * (map & 0x1))),
		     DC_MAP_CONF_PTR(map));
}

static void _ipu_dc_write_tmpl(int word, u32 opcode, u32 operand, int map,
			       int wave, int glue, int sync)
{
	u32 reg;
	int stop = 1;

	reg = sync;
	reg |= (glue << 4);
	reg |= (++wave << 11);
	reg |= (++map << 15);
	reg |= (operand << 20) & 0xFFF00000;
	__raw_writel(reg, ipu_dc_tmpl_reg + word * 2);

	reg = (operand >> 12);
	reg |= opcode << 4;
	reg |= (stop << 9);
	__raw_writel(reg, ipu_dc_tmpl_reg + word * 2 + 1);
}

static void _ipu_dc_link_event(int chan, int event, int addr, int priority)
{
	u32 reg;

	reg = __raw_readl(DC_RL_CH(chan, event));
	reg &= ~(0xFFFF << (16 * (event & 0x1)));
	reg |= ((addr << 8) | priority) << (16 * (event & 0x1));
	__raw_writel(reg, DC_RL_CH(chan, event));
}

/*     Y = R *  .299 + G *  .587 + B *  .114;
       U = R * -.169 + G * -.332 + B *  .500 + 128.;
       V = R *  .500 + G * -.419 + B * -.0813 + 128.;*/
static const int rgb2ycbcr_coeff[5][3] = {
	{153, 301, 58},
	{-87, -170, 0x0100},
	{0x100, -215, -42},
	{0x0000, 0x0200, 0x0200},	/* B0, B1, B2 */
	{0x2, 0x2, 0x2},	/* S0, S1, S2 */
};

/*     R = (1.164 * (Y - 16)) + (1.596 * (Cr - 128));
       G = (1.164 * (Y - 16)) - (0.392 * (Cb - 128)) - (0.813 * (Cr - 128));
       B = (1.164 * (Y - 16)) + (2.017 * (Cb - 128); */
static const int ycbcr2rgb_coeff[5][3] = {
	{0x095, 0x000, 0x0CC},
	{0x095, 0x3CE, 0x398},
	{0x095, 0x0FF, 0x000},
	{0x3E42, 0x010A, 0x3DD6},	/*B0,B1,B2 */
	{0x1, 0x1, 0x1},	/*S0,S1,S2 */
};

#define mask_a(a) ((u32)(a) & 0x3FF)
#define mask_b(b) ((u32)(b) & 0x3FFF)

static int _rgb_to_yuv(int n, int red, int green, int blue)
{
	int c;
	c = red * rgb2ycbcr_coeff[n][0];
	c += green * rgb2ycbcr_coeff[n][1];
	c += blue * rgb2ycbcr_coeff[n][2];
	c /= 16;
	c += rgb2ycbcr_coeff[3][n] * 4;
	c += 8;
	c /= 16;
	if (c < 0)
		c = 0;
	if (c > 255)
		c = 255;
	return c;
}

/*
 * Row is for BG: 	RGB2YUV YUV2RGB RGB2RGB YUV2YUV CSC_NONE
 * Column is for FG:	RGB2YUV YUV2RGB RGB2RGB YUV2YUV CSC_NONE
 */
static struct dp_csc_param_t dp_csc_array[CSC_NUM][CSC_NUM] = {
{{DP_COM_CONF_CSC_DEF_BOTH, &rgb2ycbcr_coeff}, {0, 0}, {0, 0}, {DP_COM_CONF_CSC_DEF_BG, &rgb2ycbcr_coeff}, {DP_COM_CONF_CSC_DEF_BG, &rgb2ycbcr_coeff} },
{{0, 0}, {DP_COM_CONF_CSC_DEF_BOTH, &ycbcr2rgb_coeff}, {DP_COM_CONF_CSC_DEF_BG, &ycbcr2rgb_coeff}, {0, 0}, {DP_COM_CONF_CSC_DEF_BG, &ycbcr2rgb_coeff} },
{{0, 0}, {DP_COM_CONF_CSC_DEF_FG, &ycbcr2rgb_coeff}, {0, 0}, {0, 0}, {0, 0} },
{{DP_COM_CONF_CSC_DEF_FG, &rgb2ycbcr_coeff}, {0, 0}, {0, 0}, {0, 0}, {0, 0} },
{{DP_COM_CONF_CSC_DEF_FG, &rgb2ycbcr_coeff}, {DP_COM_CONF_CSC_DEF_FG, &ycbcr2rgb_coeff}, {0, 0}, {0, 0}, {0, 0} }
};

static enum csc_type_t fg_csc_type = CSC_NONE, bg_csc_type = CSC_NONE;

void __ipu_dp_csc_setup(int dp, struct dp_csc_param_t dp_csc_param,
			bool srm_mode_update)
{
	u32 reg;
	const int (*coeff)[5][3];

	if (dp_csc_param.mode >= 0) {
		reg = __raw_readl(DP_COM_CONF(dp));
		reg &= ~DP_COM_CONF_CSC_DEF_MASK;
		reg |= dp_csc_param.mode;
		__raw_writel(reg, DP_COM_CONF(dp));
	}

	coeff = dp_csc_param.coeff;

	if (coeff) {
		__raw_writel(mask_a((*coeff)[0][0]) |
				(mask_a((*coeff)[0][1]) << 16), DP_CSC_A_0(dp));
		__raw_writel(mask_a((*coeff)[0][2]) |
				(mask_a((*coeff)[1][0]) << 16), DP_CSC_A_1(dp));
		__raw_writel(mask_a((*coeff)[1][1]) |
				(mask_a((*coeff)[1][2]) << 16), DP_CSC_A_2(dp));
		__raw_writel(mask_a((*coeff)[2][0]) |
				(mask_a((*coeff)[2][1]) << 16), DP_CSC_A_3(dp));
		__raw_writel(mask_a((*coeff)[2][2]) |
				(mask_b((*coeff)[3][0]) << 16) |
				((*coeff)[4][0] << 30), DP_CSC_0(dp));
		__raw_writel(mask_b((*coeff)[3][1]) | ((*coeff)[4][1] << 14) |
				(mask_b((*coeff)[3][2]) << 16) |
				((*coeff)[4][2] << 30), DP_CSC_1(dp));
	}

	if (srm_mode_update) {
		reg = __raw_readl(IPU_SRM_PRI2) | 0x8;
		__raw_writel(reg, IPU_SRM_PRI2);
	}
}

int _ipu_dp_init(ipu_channel_t channel, uint32_t in_pixel_fmt,
		 uint32_t out_pixel_fmt)
{
	int in_fmt, out_fmt;
	int dp;
	int partial = false;

	if (channel == MEM_FG_SYNC) {
		dp = DP_SYNC;
		partial = true;
	} else if (channel == MEM_BG_SYNC) {
		dp = DP_SYNC;
		partial = false;
	} else if (channel == MEM_BG_ASYNC0) {
		dp = DP_ASYNC0;
		partial = false;
	} else {
		return -EINVAL;
	}

	in_fmt = format_to_colorspace(in_pixel_fmt);
	out_fmt = format_to_colorspace(out_pixel_fmt);

	if (partial) {
		if (in_fmt == RGB) {
			if (out_fmt == RGB)
				fg_csc_type = RGB2RGB;
			else
				fg_csc_type = RGB2YUV;
		} else {
			if (out_fmt == RGB)
				fg_csc_type = YUV2RGB;
			else
				fg_csc_type = YUV2YUV;
		}
	} else {
		if (in_fmt == RGB) {
			if (out_fmt == RGB)
				bg_csc_type = RGB2RGB;
			else
				bg_csc_type = RGB2YUV;
		} else {
			if (out_fmt == RGB)
				bg_csc_type = YUV2RGB;
			else
				bg_csc_type = YUV2YUV;
		}
	}

	__ipu_dp_csc_setup(dp, dp_csc_array[bg_csc_type][fg_csc_type], true);

	return 0;
}

void _ipu_dp_uninit(ipu_channel_t channel)
{
	int dp;
	int partial = false;

	if (channel == MEM_FG_SYNC) {
		dp = DP_SYNC;
		partial = true;
	} else if (channel == MEM_BG_SYNC) {
		dp = DP_SYNC;
		partial = false;
	} else if (channel == MEM_BG_ASYNC0) {
		dp = DP_ASYNC0;
		partial = false;
	} else {
		return;
	}

	if (partial)
		fg_csc_type = CSC_NONE;
	else
		bg_csc_type = CSC_NONE;

	__ipu_dp_csc_setup(dp, dp_csc_array[bg_csc_type][fg_csc_type], false);
}

void _ipu_dc_init(int dc_chan, int di, bool interlaced)
{
	u32 reg = 0;

	if ((dc_chan == 1) || (dc_chan == 5)) {
		if (interlaced) {
			_ipu_dc_link_event(dc_chan, DC_EVT_NL, 0, 3);
			_ipu_dc_link_event(dc_chan, DC_EVT_EOL, 0, 2);
			_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA, 0, 1);
		} else {
			if (di) {
				_ipu_dc_link_event(dc_chan, DC_EVT_NL, 2, 3);
				_ipu_dc_link_event(dc_chan, DC_EVT_EOL, 3, 2);
				_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA, 4, 1);
			} else {
				_ipu_dc_link_event(dc_chan, DC_EVT_NL, 5, 3);
				_ipu_dc_link_event(dc_chan, DC_EVT_EOL, 6, 2);
				_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA, 7, 1);
			}
		}
		_ipu_dc_link_event(dc_chan, DC_EVT_NF, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NFIELD, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_EOF, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_EOFIELD, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_CHAN, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_ADDR, 0, 0);

		/* Make sure other DC sync channel is not assigned same DI */
		reg = __raw_readl(DC_WR_CH_CONF(6 - dc_chan));
		if ((di << 2) == (reg & DC_WR_CH_CONF_PROG_DI_ID)) {
			reg &= ~DC_WR_CH_CONF_PROG_DI_ID;
			reg |= di ? 0 : DC_WR_CH_CONF_PROG_DI_ID;
			__raw_writel(reg, DC_WR_CH_CONF(6 - dc_chan));
		}

		reg = 0x2;
		reg |= DC_DISP_ID_SYNC(di) << DC_WR_CH_CONF_PROG_DISP_ID_OFFSET;
		reg |= di << 2;
		if (interlaced)
			reg |= DC_WR_CH_CONF_FIELD_MODE;
	} else if ((dc_chan == 8) || (dc_chan == 9)) {
		/* async channels */
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA_W_0, 0x64, 1);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA_W_1, 0x64, 1);

		reg = 0x3;
		reg |= DC_DISP_ID_SERIAL << DC_WR_CH_CONF_PROG_DISP_ID_OFFSET;
	}
	__raw_writel(reg, DC_WR_CH_CONF(dc_chan));

	__raw_writel(0x00000000, DC_WR_CH_ADDR(dc_chan));

	__raw_writel(0x00000084, DC_GEN);
}

void _ipu_dc_uninit(int dc_chan)
{
	if ((dc_chan == 1) || (dc_chan == 5)) {
		_ipu_dc_link_event(dc_chan, DC_EVT_NL, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_EOL, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NF, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NFIELD, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_EOF, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_EOFIELD, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_CHAN, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_ADDR, 0, 0);
	} else if ((dc_chan == 8) || (dc_chan == 9)) {
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_ADDR_W_0, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_ADDR_W_1, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_CHAN_W_0, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_CHAN_W_1, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA_W_0, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA_W_1, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_ADDR_R_0, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_ADDR_R_1, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_CHAN_R_0, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_CHAN_R_1, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA_R_0, 0, 0);
		_ipu_dc_link_event(dc_chan, DC_EVT_NEW_DATA_R_1, 0, 0);
	}
}

int _ipu_chan_is_interlaced(ipu_channel_t channel)
{
	if (channel == MEM_DC_SYNC)
		return !!(__raw_readl(DC_WR_CH_CONF_1) &
			  DC_WR_CH_CONF_FIELD_MODE);
	else if ((channel == MEM_BG_SYNC) || (channel == MEM_FG_SYNC))
		return !!(__raw_readl(DC_WR_CH_CONF_5) &
			  DC_WR_CH_CONF_FIELD_MODE);
	return 0;
}

void _ipu_dp_dc_enable(ipu_channel_t channel)
{
	uint32_t reg;
	uint32_t dc_chan;
	int irq = 0;

	if (channel == MEM_FG_SYNC)
		irq = IPU_IRQ_DP_SF_END;
	else if (channel == MEM_DC_SYNC)
		dc_chan = 1;
	else if (channel == MEM_BG_SYNC)
		dc_chan = 5;
	else
		return;

	if (channel == MEM_FG_SYNC) {
		/* Enable FG channel */
		reg = __raw_readl(DP_COM_CONF(DP_SYNC));
		__raw_writel(reg | DP_COM_CONF_FG_EN, DP_COM_CONF(DP_SYNC));

		reg = __raw_readl(IPU_SRM_PRI2) | 0x8;
		__raw_writel(reg, IPU_SRM_PRI2);
		return;
	}

	reg = __raw_readl(DC_WR_CH_CONF(dc_chan));
	reg |= 4 << DC_WR_CH_CONF_PROG_TYPE_OFFSET;
	__raw_writel(reg, DC_WR_CH_CONF(dc_chan));

	reg = __raw_readl(IPU_DISP_GEN);
	if (g_dc_di_assignment[dc_chan])
		reg |= DI1_COUNTER_RELEASE;
	else
		reg |= DI0_COUNTER_RELEASE;
	__raw_writel(reg, IPU_DISP_GEN);
}

static irqreturn_t dc_irq_handler(int irq, void *dev_id)
{
	u32 reg;
	uint32_t dc_chan;
	ipu_channel_t channel;
	struct completion *comp = dev_id;

	if (irq == IPU_IRQ_DP_SF_END) {
		channel = MEM_BG_SYNC;
		dc_chan = 5;
	} else if (irq == IPU_IRQ_DC_FC_1) {
		channel = MEM_DC_SYNC;
		dc_chan = 1;
	} else {
		return IRQ_HANDLED;
	}

	reg = __raw_readl(IPU_DISP_GEN);
	if (g_dc_di_assignment[dc_chan])
		reg &= ~DI1_COUNTER_RELEASE;
	else
		reg &= ~DI0_COUNTER_RELEASE;
	__raw_writel(reg, IPU_DISP_GEN);

	reg = __raw_readl(DC_WR_CH_CONF(dc_chan));
	reg &= ~DC_WR_CH_CONF_PROG_TYPE_MASK;
	__raw_writel(reg, DC_WR_CH_CONF(dc_chan));

	if (__raw_readl(IPUIRQ_2_STATREG(IPU_IRQ_VSYNC_PRE_0 + g_dc_di_assignment[dc_chan])) &
		IPUIRQ_2_MASK(IPU_IRQ_VSYNC_PRE_0 + g_dc_di_assignment[dc_chan]))
		dev_err(g_ipu_dev, "VSyncPre occurred before DI%d disable\n", g_dc_di_assignment[dc_chan]);

	complete(comp);
	return IRQ_HANDLED;
}

void _ipu_dp_dc_disable(ipu_channel_t channel)
{
	int ret;
	unsigned long lock_flags;
	uint32_t reg;
	uint32_t csc;
	uint32_t dc_chan;
	int irq = 0;
	int timeout = 50;
	DECLARE_COMPLETION_ONSTACK(dc_comp);

	if (channel == MEM_DC_SYNC) {
		dc_chan = 1;
		irq = IPU_IRQ_DC_FC_1;
	} else if (channel == MEM_BG_SYNC) {
		dc_chan = 5;
		irq = IPU_IRQ_DP_SF_END;
	} else if (channel == MEM_FG_SYNC) {
		/* Disable FG channel */

		spin_lock_irqsave(&ipu_lock, lock_flags);

		reg = __raw_readl(DP_COM_CONF(DP_SYNC));
		csc = reg & DP_COM_CONF_CSC_DEF_MASK;
		if (csc == DP_COM_CONF_CSC_DEF_FG)
			reg &= ~DP_COM_CONF_CSC_DEF_MASK;

		reg &= ~DP_COM_CONF_FG_EN;
		__raw_writel(reg, DP_COM_CONF(DP_SYNC));

		reg = __raw_readl(IPU_SRM_PRI2) | 0x8;
		__raw_writel(reg, IPU_SRM_PRI2);

		spin_unlock_irqrestore(&ipu_lock, lock_flags);

		__raw_writel(IPUIRQ_2_MASK(IPU_IRQ_DP_SF_END),
			     IPUIRQ_2_STATREG(IPU_IRQ_DP_SF_END));
		while ((__raw_readl(IPUIRQ_2_STATREG(IPU_IRQ_DP_SF_END)) &
			IPUIRQ_2_MASK(IPU_IRQ_DP_SF_END)) == 0) {
			msleep(2);
			timeout -= 2;
			if (timeout <= 0)
				break;
		}
		return;
	} else {
		return;
	}

	__raw_writel(IPUIRQ_2_MASK(IPU_IRQ_VSYNC_PRE_0 + g_dc_di_assignment[dc_chan]),
		     IPUIRQ_2_STATREG(IPU_IRQ_VSYNC_PRE_0 + g_dc_di_assignment[dc_chan]));
	ipu_clear_irq(irq);
	ret = ipu_request_irq(irq, dc_irq_handler, 0, NULL, &dc_comp);
	if (ret < 0) {
		dev_err(g_ipu_dev, "DC irq %d in use\n", irq);
		return;
	}
	ret = wait_for_completion_timeout(&dc_comp, msecs_to_jiffies(50));

	dev_dbg(g_ipu_dev, "DC stop timeout - %d * 10ms\n", 5 - ret);
	ipu_free_irq(irq, &dc_comp);
}

void _ipu_init_dc_mappings(void)
{
	/* IPU_PIX_FMT_RGB24 */
	_ipu_dc_map_clear(0);
	_ipu_dc_map_config(0, 0, 7, 0xFF);
	_ipu_dc_map_config(0, 1, 15, 0xFF);
	_ipu_dc_map_config(0, 2, 23, 0xFF);

	/* IPU_PIX_FMT_RGB666 */
	_ipu_dc_map_clear(1);
	_ipu_dc_map_config(1, 0, 5, 0xFC);
	_ipu_dc_map_config(1, 1, 11, 0xFC);
	_ipu_dc_map_config(1, 2, 17, 0xFC);

	/* IPU_PIX_FMT_YUV444 */
	_ipu_dc_map_clear(2);
	_ipu_dc_map_config(2, 0, 15, 0xFF);
	_ipu_dc_map_config(2, 1, 23, 0xFF);
	_ipu_dc_map_config(2, 2, 7, 0xFF);

	/* IPU_PIX_FMT_RGB565 */
	_ipu_dc_map_clear(3);
	_ipu_dc_map_config(3, 0, 4, 0xF8);
	_ipu_dc_map_config(3, 1, 10, 0xFC);
	_ipu_dc_map_config(3, 2, 15, 0xF8);

	/* IPU_PIX_FMT_LVDS666 */
	_ipu_dc_map_clear(4);
	_ipu_dc_map_config(4, 0, 5, 0xFC);
	_ipu_dc_map_config(4, 1, 13, 0xFC);
	_ipu_dc_map_config(4, 2, 21, 0xFC);
}

int _ipu_pixfmt_to_map(uint32_t fmt)
{
	switch (fmt) {
	case IPU_PIX_FMT_GENERIC:
	case IPU_PIX_FMT_RGB24:
		return 0;
	case IPU_PIX_FMT_RGB666:
		return 1;
	case IPU_PIX_FMT_YUV444:
		return 2;
	case IPU_PIX_FMT_RGB565:
		return 3;
	case IPU_PIX_FMT_LVDS666:
		return 4;
	}

	return -1;
}

/*!
 * This function sets the colorspace for of dp.
 * modes.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       param         	If it's not NULL, update the csc table
 *                              with this parameter.
 *
 * @return      N/A
 */
void _ipu_dp_set_csc_coefficients(ipu_channel_t channel, int32_t param[][3])
{
	int dp;
	struct dp_csc_param_t dp_csc_param;

	if (channel == MEM_FG_SYNC)
		dp = DP_SYNC;
	else if (channel == MEM_BG_SYNC)
		dp = DP_SYNC;
	else if (channel == MEM_BG_ASYNC0)
		dp = DP_ASYNC0;
	else
		return;

	dp_csc_param.mode = -1;
	dp_csc_param.coeff = param;
	__ipu_dp_csc_setup(dp, dp_csc_param, true);
}

/*!
 * This function is called to initialize a synchronous LCD panel.
 *
 * @param       disp            The DI the panel is attached to.
 *
 * @param       pixel_clk       Desired pixel clock frequency in Hz.
 *
 * @param       pixel_fmt       Input parameter for pixel format of buffer.
 *                              Pixel format is a FOURCC ASCII code.
 *
 * @param       width           The width of panel in pixels.
 *
 * @param       height          The height of panel in pixels.
 *
 * @param       hStartWidth     The number of pixel clocks between the HSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       hSyncWidth      The width of the HSYNC signal in units of pixel
 *                              clocks.
 *
 * @param       hEndWidth       The number of pixel clocks between the end of
 *                              valid data and the HSYNC signal for next line.
 *
 * @param       vStartWidth     The number of lines between the VSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       vSyncWidth      The width of the VSYNC signal in units of lines
 *
 * @param       vEndWidth       The number of lines between the end of valid
 *                              data and the VSYNC signal for next frame.
 *
 * @param       sig             Bitfield of signal polarities for LCD interface.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t ipu_init_sync_panel(int disp, uint32_t pixel_clk,
			    uint16_t width, uint16_t height,
			    uint32_t pixel_fmt,
			    uint16_t h_start_width, uint16_t h_sync_width,
			    uint16_t h_end_width, uint16_t v_start_width,
			    uint16_t v_sync_width, uint16_t v_end_width,
			    uint32_t v_to_h_sync, ipu_di_signal_cfg_t sig)
{
	unsigned long lock_flags;
	uint32_t field0_offset = 0;
	uint32_t field1_offset;
	uint32_t reg;
	uint32_t disp_gen, di_gen, vsync_cnt;
	uint32_t div, up;
	uint32_t h_total, v_total;
	int map;
	struct clk *di_clk;

	dev_dbg(g_ipu_dev, "panel size = %d x %d\n", width, height);

	if ((v_sync_width == 0) || (h_sync_width == 0))
		return EINVAL;

	h_total = width + h_sync_width + h_start_width + h_end_width;
	v_total = height + v_sync_width + v_start_width + v_end_width;

	/* Init clocking */
	dev_dbg(g_ipu_dev, "pixel clk = %d\n", pixel_clk);
	if (sig.ext_clk)
		di_clk = g_di_clk[disp];
	else
		di_clk = g_ipu_clk;

	/*
	 * Calculate divider
	 * Fractional part is 4 bits,
	 * so simply multiply by 2^4 to get fractional part.
	 */
	div = (clk_get_rate(di_clk) * 16) / pixel_clk;
	if (div < 0x10)	/* Min DI disp clock divider is 1 */
		div = 0x10;
	/*
	 * DI disp clock offset is zero,
	 * and fractional part is rounded off to 0.5.
	 */
	div &= 0xFF8;

	reg = __raw_readl(DI_GENERAL(disp));
	if (sig.ext_clk)
		__raw_writel(reg | DI_GEN_DI_CLK_EXT, DI_GENERAL(disp));
	else
		__raw_writel(reg & ~DI_GEN_DI_CLK_EXT, DI_GENERAL(disp));

	spin_lock_irqsave(&ipu_lock, lock_flags);

	disp_gen = __raw_readl(IPU_DISP_GEN);
	disp_gen &= disp ? ~DI1_COUNTER_RELEASE : ~DI0_COUNTER_RELEASE;
	__raw_writel(disp_gen, IPU_DISP_GEN);

	__raw_writel(div, DI_BS_CLKGEN0(disp));

	/* Setup pixel clock timing */
	/* FIXME: needs to be more flexible */
	/* Down time is half of period */
	__raw_writel((div / 16) << 16, DI_BS_CLKGEN1(disp));

	_ipu_di_data_wave_config(disp, SYNC_WAVE, div / 16 - 1, div / 16 - 1);
	_ipu_di_data_pin_config(disp, SYNC_WAVE, DI_PIN15, 3, 0, div / 16 * 2);

	div = div / 16;		/* Now divider is integer portion */

	map = _ipu_pixfmt_to_map(pixel_fmt);
	if (map < 0) {
		dev_dbg(g_ipu_dev, "IPU_DISP: No MAP\n");
		spin_unlock_irqrestore(&ipu_lock, lock_flags);
		return -EINVAL;
	}

	di_gen = 0;
	if (sig.ext_clk)
		di_gen |= DI_GEN_DI_CLK_EXT;

	if (sig.interlaced) {
		if (cpu_is_mx51_rev(CHIP_REV_2_0)) {
			/* Setup internal HSYNC waveform */
			_ipu_di_sync_config(
					disp, 			/* display */
					1, 				/* counter */
					h_total/2 - 1, 	/* run count */
					DI_SYNC_CLK,		 /* run_resolution */
					0, 				/* offset */
					DI_SYNC_NONE, 	/* offset resolution */
					0, 				/* repeat count */
					DI_SYNC_NONE, 	/* CNT_CLR_SEL */
					0, 				/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					0				/* COUNT DOWN */
					);

			/* Field 1 VSYNC waveform */
			_ipu_di_sync_config(
					disp, 			/* display */
					2, 				/* counter */
					h_total - 1, 		/* run count */
					DI_SYNC_CLK,		/* run_resolution */
					0, 				/* offset */
					DI_SYNC_NONE, 	/* offset resolution */
					0, 				/* repeat count */
					DI_SYNC_NONE, 	/* CNT_CLR_SEL */
					0, 				/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					4				/* COUNT DOWN */
					);

			/* Setup internal HSYNC waveform */
			_ipu_di_sync_config(
					disp, 			/* display */
					3, 				/* counter */
					v_total*2 - 1, 	/* run count */
					DI_SYNC_INT_HSYNC,	/* run_resolution */
					1, 				/* offset */
					DI_SYNC_INT_HSYNC, 	/* offset resolution */
					0, 				/* repeat count */
					DI_SYNC_NONE, 	/* CNT_CLR_SEL */
					0, 				/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					4				/* COUNT DOWN */
					);

			/* Active Field ? */
			_ipu_di_sync_config(
					disp, 			/* display */
					4, 				/* counter */
					v_total/2 - 1, 	/* run count */
					DI_SYNC_HSYNC,	/* run_resolution */
					v_start_width, 	/*  offset */
					DI_SYNC_HSYNC, 	/* offset resolution */
					2, 				/* repeat count */
					DI_SYNC_VSYNC, 	/* CNT_CLR_SEL */
					0, 				/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					0				/* COUNT DOWN */
					);

			/* Active Line */
			_ipu_di_sync_config(
					disp, 			/* display */
					5, 				/* counter */
					0, 				/* run count */
					DI_SYNC_HSYNC,	/* run_resolution */
					0, 				/*  offset */
					DI_SYNC_NONE, 	/* offset resolution */
					height/2, 		/* repeat count */
					4, 				/* CNT_CLR_SEL */
					0, 				/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					0				/* COUNT DOWN */
					);

			/* Field 0 VSYNC waveform */
			_ipu_di_sync_config(
					disp, 			/* display */
					6, 				/* counter */
					v_total - 1, 	/* run count */
					DI_SYNC_HSYNC,	/* run_resolution */
					0, 				/* offset */
					DI_SYNC_NONE, 	/* offset resolution */
					0, 				/* repeat count */
					DI_SYNC_NONE, 	/* CNT_CLR_SEL  */
					0, 				/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					0				/* COUNT DOWN */
					);

			/* DC VSYNC waveform */
			vsync_cnt = 7;
			_ipu_di_sync_config(
					disp, 			/* display */
					7, 				/* counter */
					v_total/2 - 1, 	/* run count */
					DI_SYNC_HSYNC,	/* run_resolution  */
					9, 				/* offset  */
					DI_SYNC_HSYNC, 	/* offset resolution */
					2, 				/* repeat count */
					DI_SYNC_VSYNC, 	/* CNT_CLR_SEL */
					0, 				/* CNT_POLARITY_GEN_EN */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					0				/* COUNT DOWN */
					);

			/* active pixel waveform */
			_ipu_di_sync_config(
					disp, 			/* display */
					8, 				/* counter */
					0, 	/* run count  */
					DI_SYNC_CLK,	/* run_resolution */
					h_start_width, 				/* offset  */
					DI_SYNC_CLK, 	/* offset resolution */
					width, 				/* repeat count  */
					5, 	/* CNT_CLR_SEL  */
					0, 				/* CNT_POLARITY_GEN_EN  */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL  */
					0, 				/* COUNT UP  */
					0				/* COUNT DOWN */
					);

			/* ??? */
			_ipu_di_sync_config(
					disp, 			/* display */
					9, 				/* counter */
					v_total - 1, 	/* run count */
					DI_SYNC_INT_HSYNC,	/* run_resolution */
					v_total/2, 			/* offset  */
					DI_SYNC_INT_HSYNC, 	/* offset resolution  */
					0, 				/* repeat count */
					DI_SYNC_HSYNC, 	/* CNT_CLR_SEL */
					0, 				/* CNT_POLARITY_GEN_EN  */
					DI_SYNC_NONE, 	/* CNT_POLARITY_CLR_SEL  */
					DI_SYNC_NONE, 	/* CNT_POLARITY_TRIGGER_SEL */
					0, 				/* COUNT UP */
					4				/* COUNT DOWN */
					);

			/* set gentime select and tag sel */
			reg = __raw_readl(DI_SW_GEN1(disp, 9));
			reg &= 0x1FFFFFFF;
			reg |= (3-1)<<29 | 0x00008000;
			__raw_writel(reg, DI_SW_GEN1(disp, 9));

			__raw_writel(v_total / 2 - 1, DI_SCR_CONF(disp));

			/* set y_sel = 1 */
			di_gen |= 0x10000000;
			di_gen |= DI_GEN_POLARITY_5;
			di_gen |= DI_GEN_POLARITY_8;
		} else {
			/* Setup internal HSYNC waveform */
			_ipu_di_sync_config(disp, 1, h_total - 1, DI_SYNC_CLK,
					0, DI_SYNC_NONE, 0, DI_SYNC_NONE, 0, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			field1_offset = v_sync_width + v_start_width + height / 2 +
				v_end_width;
			if (sig.odd_field_first) {
				field0_offset = field1_offset - 1;
				field1_offset = 0;
			}
			v_total += v_start_width + v_end_width;

			/* Field 1 VSYNC waveform */
			_ipu_di_sync_config(disp, 2, v_total - 1, 1,
					field0_offset,
					field0_offset ? 1 : DI_SYNC_NONE,
					0, DI_SYNC_NONE, 0,
					DI_SYNC_NONE, DI_SYNC_NONE, 0, 4);

			/* Setup internal HSYNC waveform */
			_ipu_di_sync_config(disp, 3, h_total - 1, DI_SYNC_CLK,
					0, DI_SYNC_NONE, 0, DI_SYNC_NONE, 0,
					DI_SYNC_NONE, DI_SYNC_NONE, 0, 4);

			/* Active Field ? */
			_ipu_di_sync_config(disp, 4,
					field0_offset ?
					field0_offset : field1_offset - 2,
					1, v_start_width + v_sync_width, 1, 2, 2,
					0, DI_SYNC_NONE, DI_SYNC_NONE, 0, 0);

			/* Active Line */
			_ipu_di_sync_config(disp, 5, 0, 1,
					0, DI_SYNC_NONE,
					height / 2, 4, 0, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			/* Field 0 VSYNC waveform */
			_ipu_di_sync_config(disp, 6, v_total - 1, 1,
					0, DI_SYNC_NONE,
					0, DI_SYNC_NONE, 0, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			/* DC VSYNC waveform */
			vsync_cnt = 7;
			_ipu_di_sync_config(disp, 7, 0, 1,
					field1_offset,
					field1_offset ? 1 : DI_SYNC_NONE,
					1, 2, 0, DI_SYNC_NONE, DI_SYNC_NONE, 0, 0);

			/* active pixel waveform */
			_ipu_di_sync_config(disp, 8, 0, DI_SYNC_CLK,
					h_sync_width + h_start_width, DI_SYNC_CLK,
					width, 5, 0, DI_SYNC_NONE, DI_SYNC_NONE,
					0, 0);

			/* ??? */
			_ipu_di_sync_config(disp, 9, v_total - 1, 2,
					0, DI_SYNC_NONE,
					0, DI_SYNC_NONE, 6, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

			reg = __raw_readl(DI_SW_GEN1(disp, 9));
			reg |= 0x8000;
			__raw_writel(reg, DI_SW_GEN1(disp, 9));

			__raw_writel(v_sync_width + v_start_width +
					v_end_width + height / 2 - 1, DI_SCR_CONF(disp));
		}

		/* Init template microcode */
		_ipu_dc_write_tmpl(0, WROD(0), 0, map, SYNC_WAVE, 0, 8);

		if (sig.Hsync_pol)
			di_gen |= DI_GEN_POLARITY_3;
		if (sig.Vsync_pol)
			di_gen |= DI_GEN_POLARITY_2;
	} else {
		/* Setup internal HSYNC waveform */
		_ipu_di_sync_config(disp, 1, h_total - 1, DI_SYNC_CLK,
					0, DI_SYNC_NONE, 0, DI_SYNC_NONE, 0, DI_SYNC_NONE,
					DI_SYNC_NONE, 0, 0);

		/* Setup external (delayed) HSYNC waveform */
		_ipu_di_sync_config(disp, DI_SYNC_HSYNC, h_total - 1,
				    DI_SYNC_CLK, div * v_to_h_sync, DI_SYNC_CLK,
#if 1 // 20090802
				    0, DI_SYNC_NONE, 1, DI_SYNC_NONE,
				    DI_SYNC_CLK, 0, h_sync_width * 2);
#else
				    0, DI_SYNC_NONE, 0, DI_SYNC_NONE,
				    DI_SYNC_NONE, 0, div * h_sync_width * 2);
#endif
		/* Setup VSYNC waveform */
		vsync_cnt = DI_SYNC_VSYNC;
		_ipu_di_sync_config(disp, DI_SYNC_VSYNC, v_total - 1,
				    DI_SYNC_INT_HSYNC, 0, DI_SYNC_NONE, 0,
				    DI_SYNC_NONE, 1, DI_SYNC_NONE,
				    DI_SYNC_INT_HSYNC, 0, v_sync_width * 2);
		__raw_writel(v_total - 1, DI_SCR_CONF(disp));

		/* Setup active data waveform to sync with DC */
		_ipu_di_sync_config(disp, 4, 0, DI_SYNC_HSYNC,
				    v_sync_width + v_start_width, DI_SYNC_HSYNC, height,
				    DI_SYNC_VSYNC, 0, DI_SYNC_NONE,
				    DI_SYNC_NONE, 0, 0);
//@@tsato
/*		_ipu_di_sync_config(disp, 4, 0, DI_SYNC_HSYNC,
				    v_start_width, DI_SYNC_HSYNC, height,
				    DI_SYNC_VSYNC, 0, DI_SYNC_NONE,
				    DI_SYNC_NONE, 0, 0);
*/
		_ipu_di_sync_config(disp, 5, 0, DI_SYNC_CLK,
				    h_sync_width + h_start_width, DI_SYNC_CLK,
				    width, 4, 0, DI_SYNC_NONE, DI_SYNC_NONE, 0,
				    0);

		/* reset all unused counters */
		__raw_writel(0, DI_SW_GEN0(disp, 6));
		__raw_writel(0, DI_SW_GEN1(disp, 6));
		__raw_writel(0, DI_SW_GEN0(disp, 7));
		__raw_writel(0, DI_SW_GEN1(disp, 7));
		__raw_writel(0, DI_SW_GEN0(disp, 8));
		__raw_writel(0, DI_SW_GEN1(disp, 8));
		__raw_writel(0, DI_SW_GEN0(disp, 9));
		__raw_writel(0, DI_SW_GEN1(disp, 9));

		reg = __raw_readl(DI_STP_REP(disp, 6));
		reg &= 0x0000FFFF;
		__raw_writel(reg, DI_STP_REP(disp, 6));
		__raw_writel(0, DI_STP_REP(disp, 7));
		__raw_writel(0, DI_STP_REP(disp, 9));

		/* Init template microcode */
		if (disp) {
		   _ipu_dc_write_tmpl(2, WROD(0), 0, map, SYNC_WAVE, 8, 5);
		   _ipu_dc_write_tmpl(3, WROD(0), 0, map, SYNC_WAVE, 4, 5);
		   _ipu_dc_write_tmpl(4, WROD(0), 0, map, SYNC_WAVE, 0, 5);
		} else {
		   _ipu_dc_write_tmpl(5, WROD(0), 0, map, SYNC_WAVE, 8, 5);
		   _ipu_dc_write_tmpl(6, WROD(0), 0, map, SYNC_WAVE, 4, 5);
		   _ipu_dc_write_tmpl(7, WROD(0), 0, map, SYNC_WAVE, 0, 5);
		}

		if (sig.Hsync_pol)
			di_gen |= DI_GEN_POLARITY_2;
		if (sig.Vsync_pol)
			di_gen |= DI_GEN_POLARITY_3;
	}

	__raw_writel(di_gen, DI_GENERAL(disp));
	__raw_writel((--vsync_cnt << DI_VSYNC_SEL_OFFSET) | 0x00000002,
		     DI_SYNC_AS_GEN(disp));

	reg = __raw_readl(DI_POL(disp));
	reg &= ~(DI_POL_DRDY_DATA_POLARITY | DI_POL_DRDY_POLARITY_15);
	if (sig.enable_pol)
		reg |= DI_POL_DRDY_POLARITY_15;
	if (sig.data_pol)
		reg |= DI_POL_DRDY_DATA_POLARITY;
	__raw_writel(reg, DI_POL(disp));

	__raw_writel(width, DC_DISP_CONF2(DC_DISP_ID_SYNC(disp)));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL(ipu_init_sync_panel);


int ipu_init_async_panel(int disp, int type, uint32_t cycle_time,
			 uint32_t pixel_fmt, ipu_adc_sig_cfg_t sig)
{
	unsigned long lock_flags;
	int map;
	u32 ser_conf = 0;
	u32 div;
	u32 di_clk = clk_get_rate(g_ipu_clk);

	/* round up cycle_time, then calcalate the divider using scaled math */
	cycle_time += (1000000000UL / di_clk) - 1;
	div = (cycle_time * (di_clk / 256UL)) / (1000000000UL / 256UL);

	map = _ipu_pixfmt_to_map(pixel_fmt);
	if (map < 0)
		return -EINVAL;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (type == IPU_PANEL_SERIAL) {
		__raw_writel((div << 24) | ((sig.ifc_width - 1) << 4),
			     DI_DW_GEN(disp, ASYNC_SER_WAVE));

		_ipu_di_data_pin_config(disp, ASYNC_SER_WAVE, DI_PIN_CS,
					0, 0, (div * 2) + 1);
		_ipu_di_data_pin_config(disp, ASYNC_SER_WAVE, DI_PIN_SER_CLK,
					1, div, div * 2);
		_ipu_di_data_pin_config(disp, ASYNC_SER_WAVE, DI_PIN_SER_RS,
					2, 0, 0);

		_ipu_dc_write_tmpl(0x64, WROD(0), 0, map, ASYNC_SER_WAVE, 0, 0);

		/* Configure DC for serial panel */
		__raw_writel(0x14, DC_DISP_CONF1(DC_DISP_ID_SERIAL));

		if (sig.clk_pol)
			ser_conf |= DI_SER_CONF_SERIAL_CLK_POL;
		if (sig.data_pol)
			ser_conf |= DI_SER_CONF_SERIAL_DATA_POL;
		if (sig.rs_pol)
			ser_conf |= DI_SER_CONF_SERIAL_RS_POL;
		if (sig.cs_pol)
			ser_conf |= DI_SER_CONF_SERIAL_CS_POL;
		__raw_writel(ser_conf, DI_SER_CONF(disp));
	}

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	return 0;
}
EXPORT_SYMBOL(ipu_init_async_panel);

/*!
 * This function sets the foreground and background plane global alpha blending
 * modes.
 *
 * @param       enable          Boolean to enable or disable global alpha
 *                              blending. If disabled, local blending is used.
 *
 * @param       alpha           Global alpha value.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_disp_set_global_alpha(ipu_channel_t channel, bool enable,
				  uint8_t alpha)
{
	uint32_t reg;
	uint32_t flow;
	unsigned long lock_flags;

	if (channel == MEM_BG_SYNC)
		flow = DP_SYNC;
	else if (channel == MEM_BG_ASYNC0)
		flow = DP_ASYNC0;
	else if (channel == MEM_BG_ASYNC1)
		flow = DP_ASYNC1;
	else
		return -EINVAL;

	if (!g_ipu_clk_enabled)
		clk_enable(g_ipu_clk);
	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (enable) {
		reg = __raw_readl(DP_GRAPH_WIND_CTRL(flow)) & 0x00FFFFFFL;
		__raw_writel(reg | ((uint32_t) alpha << 24),
			     DP_GRAPH_WIND_CTRL(flow));

		reg = __raw_readl(DP_COM_CONF(flow));
		__raw_writel(reg | DP_COM_CONF_GWAM, DP_COM_CONF(flow));
	} else {
		reg = __raw_readl(DP_COM_CONF(flow));
		__raw_writel(reg & ~DP_COM_CONF_GWAM, DP_COM_CONF(flow));
	}

	reg = __raw_readl(IPU_SRM_PRI2) | 0x8;
	__raw_writel(reg, IPU_SRM_PRI2);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	if (!g_ipu_clk_enabled)
		clk_disable(g_ipu_clk);

	return 0;
}
EXPORT_SYMBOL(ipu_disp_set_global_alpha);

/*!
 * This function sets the transparent color key for SDC graphic plane.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       enable          Boolean to enable or disable color key
 *
 * @param       colorKey        24-bit RGB color for transparent color key.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_disp_set_color_key(ipu_channel_t channel, bool enable,
			       uint32_t color_key)
{
	uint32_t reg, flow;
	int y, u, v;
	int red, green, blue;
	unsigned long lock_flags;

	if (channel == MEM_BG_SYNC)
		flow = DP_SYNC;
	else if (channel == MEM_BG_ASYNC0)
		flow = DP_ASYNC0;
	else if (channel == MEM_BG_ASYNC1)
		flow = DP_ASYNC1;
	else
		return -EINVAL;

	if (!g_ipu_clk_enabled)
		clk_enable(g_ipu_clk);

	/* Transform color key from rgb to yuv if CSC is enabled */
	reg = __raw_readl(DP_COM_CONF(flow));
	if ((reg & DP_COM_CONF_CSC_DEF_MASK) == DP_COM_CONF_CSC_DEF_BG) {
		red = (color_key >> 16) & 0xFF;
		green = (color_key >> 8) & 0xFF;
		blue = color_key & 0xFF;

		y = _rgb_to_yuv(0, red, green, blue);
		u = _rgb_to_yuv(0, red, green, blue);
		v = _rgb_to_yuv(0, red, green, blue);
		color_key = (y << 16) | (u << 8) | v;
	}

	spin_lock_irqsave(&ipu_lock, lock_flags);

	if (enable) {
		reg = __raw_readl(DP_GRAPH_WIND_CTRL(flow)) & 0xFF000000L;
		__raw_writel(reg | color_key, DP_GRAPH_WIND_CTRL(flow));

		reg = __raw_readl(DP_COM_CONF(flow));
		__raw_writel(reg | DP_COM_CONF_GWCKE, DP_COM_CONF(flow));
	} else {
		reg = __raw_readl(DP_COM_CONF(flow));
		__raw_writel(reg & ~DP_COM_CONF_GWCKE, DP_COM_CONF(flow));
	}

	reg = __raw_readl(IPU_SRM_PRI2) | 0x8;
	__raw_writel(reg, IPU_SRM_PRI2);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	if (!g_ipu_clk_enabled)
		clk_disable(g_ipu_clk);

	return 0;
}
EXPORT_SYMBOL(ipu_disp_set_color_key);

/*!
 * This function sets the window position of the foreground or background plane.
 * modes.
 *
 * @param       channel         Input parameter for the logical channel ID.
 *
 * @param       x_pos           The X coordinate position to place window at.
 *                              The position is relative to the top left corner.
 *
 * @param       y_pos           The Y coordinate position to place window at.
 *                              The position is relative to the top left corner.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t ipu_disp_set_window_pos(ipu_channel_t channel, int16_t x_pos,
				int16_t y_pos)
{
	u32 reg;
	unsigned long lock_flags;
	uint32_t flow = 0;

	if (channel == MEM_FG_SYNC)
		flow = DP_SYNC;
	else if (channel == MEM_FG_ASYNC0)
		flow = DP_ASYNC0;
	else if (channel == MEM_FG_ASYNC1)
		flow = DP_ASYNC1;
	else
		return -EINVAL;

	if (!g_ipu_clk_enabled)
		clk_enable(g_ipu_clk);

	spin_lock_irqsave(&ipu_lock, lock_flags);

	__raw_writel((x_pos << 16) | y_pos, DP_FG_POS(flow));

	reg = __raw_readl(IPU_SRM_PRI2) | 0x8;
	__raw_writel(reg, IPU_SRM_PRI2);

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
	if (!g_ipu_clk_enabled)
		clk_disable(g_ipu_clk);

	return 0;
}
EXPORT_SYMBOL(ipu_disp_set_window_pos);

void ipu_disp_direct_write(ipu_channel_t channel, u32 value, u32 offset)
{
	if (channel == DIRECT_ASYNC0)
		__raw_writel(value, ipu_disp_base[0] + offset);
	else if (channel == DIRECT_ASYNC1)
		__raw_writel(value, ipu_disp_base[1] + offset);
}
EXPORT_SYMBOL(ipu_disp_direct_write);

void ipu_reset_disp_panel(void)
{
	uint32_t tmp;

	tmp = __raw_readl(DI_GENERAL(1));
	__raw_writel(tmp | 0x08, DI_GENERAL(1));
	msleep(10); /* tRES >= 100us */
	tmp = __raw_readl(DI_GENERAL(1));
	__raw_writel(tmp & ~0x08, DI_GENERAL(1));
	msleep(60);

	return;
}
EXPORT_SYMBOL(ipu_reset_disp_panel);
