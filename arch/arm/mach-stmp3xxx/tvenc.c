/*
 * Freescale STMP378X dvi panel initialization
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

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <mach/regs-lcdif.h>
#include <mach/regs-lradc.h>
#include <mach/regs-pinctrl.h>
#include <mach/regs-clkctrl.h>
#include <mach/regs-pwm.h>
#include <mach/regs-apbh.h>
#include <mach/gpio.h>
#include <mach/pins.h>
#include <mach/lcdif.h>
#include <mach/cpu.h>
#include <mach/stmp3xxx.h>

#include "pinmux.h"

#include <mach/regs-tvenc.h>

enum {
	TVENC_MODE_OFF = 0,
	TVENC_MODE_NTSC,
	TVENC_MODE_PAL,
};

/* NTSC 720x480 mode */
#define NTSC_X_RES	720
#define NTSC_Y_RES	480
#define NTSC_H_BLANKING	262
#define NTSC_V_LINES	525

/* PAL 720x576 mode */
#define PAL_X_RES	720
#define PAL_Y_RES	576
#define PAL_H_BLANKING	274
#define PAL_V_LINES	625

/* frame size */
#define DVI_H_BLANKING(m)	(m == TVENC_MODE_NTSC ? \
				      NTSC_H_BLANKING : PAL_H_BLANKING)
#define DVI_V_LINES(m)		(m == TVENC_MODE_NTSC ? \
				      NTSC_V_LINES : PAL_V_LINES)
#define DVI_H_ACTIVE(m)		(m == TVENC_MODE_NTSC ? NTSC_X_RES : PAL_X_RES)
#define DVI_V_ACTIVE(m)		(m == TVENC_MODE_NTSC ? NTSC_Y_RES : PAL_Y_RES)
/* fileds range */
#define DVI_F1_START(m)		1
#define DVI_F1_END(m)		(DVI_V_LINES(m) / 2)
#define DVI_F2_START(m)		(DVI_F1_END(m) + 1)
#define DVI_F2_END(m)		DVI_V_LINES(m)
/* blanking range */
#define DVI_V1_BLANK_START(m)	DVI_F1_END(m)
#define DVI_V1_BLANK_END(m)	(DVI_V1_BLANK_START(m) + \
				 (DVI_V_LINES(m) - DVI_V_ACTIVE(m)) / 2)
#define DVI_V2_BLANK_START(m)	DVI_F2_END(m)
#define DVI_V2_BLANK_END(m)	((DVI_V2_BLANK_START(m) + \
				 (DVI_V_LINES(m) - DVI_V_ACTIVE(m)) / 2 - 1) % \
				 DVI_V_LINES(m))

static struct clk *lcd_clk;
static struct clk *clk_ref_vid;
static struct clk *clk_tv108M_ng;
static struct clk *clk_tv27M;

static int tvenc_mode;

static void init_tvenc_hw(int mode)
{
	/* Reset module */
	HW_TVENC_CTRL_SET(BM_TVENC_CTRL_SFTRST);
	udelay(10);

	/* Take module out of reset */
	HW_TVENC_CTRL_CLR(BM_TVENC_CTRL_SFTRST | BM_TVENC_CTRL_CLKGATE);

	if (mode == TVENC_MODE_NTSC) {
		/* Config NTSC-M mode, 8-bit Y/C in, SYNC out */
		HW_TVENC_CONFIG_CLR(BM_TVENC_CONFIG_SYNC_MODE |
				    BM_TVENC_CONFIG_PAL_SHAPE |
				    BM_TVENC_CONFIG_YGAIN_SEL |
				    BM_TVENC_CONFIG_CGAIN);
		HW_TVENC_CONFIG_SET(BM_TVENC_CONFIG_FSYNC_PHS |
				    BF_TVENC_CONFIG_SYNC_MODE(0x4));

		/* 859 pixels/line for NTSC */
		HW_TVENC_SYNCOFFSET_WR(857);

		HW_TVENC_COLORSUB0_WR(0x21F07C1F);
		HW_TVENC_COLORBURST_CLR(BM_TVENC_COLORBURST_NBA |
					BM_TVENC_COLORBURST_PBA);
		HW_TVENC_COLORBURST_SET(BF_TVENC_COLORBURST_NBA(0xc8) |
					BF_TVENC_COLORBURST_PBA(0));
	} else if (mode == TVENC_MODE_PAL) {
		/* Config PAL-B mode, 8-bit Y/C in, SYNC out */
		HW_TVENC_CONFIG_CLR(BM_TVENC_CONFIG_SYNC_MODE |
				    BM_TVENC_CONFIG_ENCD_MODE |
				    BM_TVENC_CONFIG_YGAIN_SEL |
				    BM_TVENC_CONFIG_CGAIN |
				    BM_TVENC_CONFIG_FSYNC_PHS);
		HW_TVENC_CONFIG_SET(BM_TVENC_CONFIG_PAL_SHAPE |
				    BF_TVENC_CONFIG_YGAIN_SEL(1) |
				    BF_TVENC_CONFIG_CGAIN(1) |
				    BF_TVENC_CONFIG_ENCD_MODE(0x1) |
				    BF_TVENC_CONFIG_SYNC_MODE(0x4));

		/* 863 pixels/line for PAL */
		HW_TVENC_SYNCOFFSET_WR(863);

		HW_TVENC_COLORSUB0_WR(0x2A098ACB);
		HW_TVENC_COLORBURST_CLR(BM_TVENC_COLORBURST_NBA |
					BM_TVENC_COLORBURST_PBA);
		HW_TVENC_COLORBURST_SET(BF_TVENC_COLORBURST_NBA(0xd6) |
					BF_TVENC_COLORBURST_PBA(0x2a));
	}

	/* Power up DAC */
	HW_TVENC_DACCTRL_WR(BM_TVENC_DACCTRL_GAINDN |
			    BM_TVENC_DACCTRL_GAINUP |
			    BM_TVENC_DACCTRL_PWRUP1 |
			    BM_TVENC_DACCTRL_DUMP_TOVDD1 |
			    BF_TVENC_DACCTRL_RVAL(3));

	/* set all to zero is a requirement for NTSC */
	HW_TVENC_MACROVISION0_WR(0);
	HW_TVENC_MACROVISION1_WR(0);
	HW_TVENC_MACROVISION2_WR(0);
	HW_TVENC_MACROVISION3_WR(0);
	HW_TVENC_MACROVISION4_WR(0);
}

static int init_panel(struct device *dev, dma_addr_t phys, int memsize,
		struct stmp3xxx_platform_fb_entry *pentry)
{
	int ret = 0;

	lcd_clk = clk_get(dev, "lcdif");
	clk_enable(lcd_clk);
	clk_set_rate(lcd_clk, 1000000/pentry->cycle_time_ns); /* kHz */

	clk_ref_vid = clk_get(NULL, "ref_vid");
	clk_tv108M_ng = clk_get(NULL, "tv108M_ng");
	clk_tv27M = clk_get(NULL, "tv27M");
	clk_enable(clk_ref_vid);
	clk_enable(clk_tv108M_ng);
	clk_enable(clk_tv27M);

	tvenc_mode = pentry->x_res == NTSC_Y_RES ? TVENC_MODE_NTSC : \
		     TVENC_MODE_PAL;

	init_tvenc_hw(tvenc_mode);

	setup_dvi_panel(DVI_H_ACTIVE(tvenc_mode), DVI_V_ACTIVE(tvenc_mode),
			DVI_H_BLANKING(tvenc_mode), DVI_V_LINES(tvenc_mode),
			DVI_V1_BLANK_START(tvenc_mode),
			DVI_V1_BLANK_END(tvenc_mode),
			DVI_V2_BLANK_START(tvenc_mode),
			DVI_V2_BLANK_END(tvenc_mode),
			DVI_F1_START(tvenc_mode), DVI_F1_END(tvenc_mode),
			DVI_F2_START(tvenc_mode), DVI_F2_END(tvenc_mode));

	ret = stmp3xxx_lcdif_dma_init(dev, phys, memsize, 1);

	return ret;
}

static void release_panel(struct device *dev,
			  struct stmp3xxx_platform_fb_entry *pentry)
{
	release_dvi_panel();

	stmp3xxx_lcdif_dma_release();

	clk_disable(clk_ref_vid);
	clk_disable(clk_tv108M_ng);
	clk_disable(clk_tv27M);
	clk_disable(lcd_clk);
	clk_put(clk_ref_vid);
	clk_put(clk_tv108M_ng);
	clk_put(clk_tv27M);
	clk_put(lcd_clk);
}

static int blank_panel(int blank)
{
	int ret = 0, count;

	switch (blank) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		HW_LCDIF_CTRL_CLR(BM_LCDIF_CTRL_BYPASS_COUNT);

		/* Wait until current transfer is complete, max 30ms */
		for (count = 30000; count > 0; count--) {
			if (HW_LCDIF_STAT_RD() & BM_LCDIF_STAT_TXFIFO_EMPTY)
				break;
			udelay(1);
		}
		break;

	case FB_BLANK_UNBLANK:
		HW_LCDIF_CTRL_SET(BM_LCDIF_CTRL_BYPASS_COUNT);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static struct stmp3xxx_platform_fb_entry ntsc_fb_entry = {
	.name		= "tvenc_ntsc",
	/* x/y swapped */
	.x_res		= NTSC_Y_RES,
	.y_res		= NTSC_X_RES,
	.bpp		= 32,
	/* the pix_clk should be near 27Mhz for proper syncronization */
	.cycle_time_ns	= 74,
	.lcd_type	= STMP3XXX_LCD_PANEL_DVI,
	.init_panel	= init_panel,
	.release_panel	= release_panel,
	.blank_panel	= blank_panel,
	.run_panel	= stmp3xxx_lcdif_run,
	.pan_display	= stmp3xxx_lcdif_pan_display,
};

static struct stmp3xxx_platform_fb_entry pal_fb_entry = {
	.name		= "tvenc_pal",
	/* x/y swapped */
	.x_res		= PAL_Y_RES,
	.y_res		= PAL_X_RES,
	.bpp		= 32,
	/* the pix_clk should be near 27Mhz for proper syncronization */
	.cycle_time_ns	= 74,
	.lcd_type	= STMP3XXX_LCD_PANEL_DVI,
	.init_panel	= init_panel,
	.release_panel	= release_panel,
	.blank_panel	= blank_panel,
	.run_panel	= stmp3xxx_lcdif_run,
	.pan_display	= stmp3xxx_lcdif_pan_display,
};

static int __init register_devices(void)
{
	stmp3xxx_lcd_register_entry(&ntsc_fb_entry,
				    stmp3xxx_framebuffer.dev.platform_data);
	stmp3xxx_lcd_register_entry(&pal_fb_entry,
				    stmp3xxx_framebuffer.dev.platform_data);

	return 0;
}
subsys_initcall(register_devices);
