/*
 * Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#ifndef __ASM_ARCH_MXC_H__
#define __ASM_ARCH_MXC_H__

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#ifndef __ASSEMBLY__
#include <linux/types.h>

/*!
 * @ingroup MSL_MX27 MSL_MX31 MSL_MXC91321  MSL_MX37
 */
/*!
 * gpio port structure
 */
struct mxc_gpio_port {
	u32 num;		/*!< gpio port number */
	u32 base;		/*!< gpio port base VA */
#ifdef MXC_GPIO_SPLIT_IRQ_2
	u16 irq_0_15, irq_16_31;
#else
	u16 irq;		/*!< irq number to the core */
#endif
	u16 virtual_irq_start;	/*!< virtual irq start number */
};
/*!
 * This structure is used to define the One wire platform data.
 * It includes search rom accelerator.
 */

struct mxc_w1_config {
	int search_rom_accelerator;
};
/*!
 * This structure is used to define the SPI master controller's platform
 * data. It includes the SPI  bus number and the maximum number of
 * slaves/chips it supports.
 */
struct mxc_spi_master {
	/*!
	 * SPI Master's bus number.
	 */
	unsigned int bus_num;
	/*!
	 * SPI Master's maximum number of chip selects.
	 */
	unsigned int maxchipselect;
	/*!
	 * CSPI Hardware Version.
	 */
	unsigned int spi_version;
	/*!
	 * CSPI chipselect pin table.
	 * Workaround for ecspi chipselect pin may not keep correct level when
	 * idle.
	 */
	void (*chipselect_active) (int cspi_mode, int status, int chipselect);
	void (*chipselect_inactive) (int cspi_mode, int status, int chipselect);
};

struct mxc_ipu_config {
	int rev;
	struct clk *di_clk[2];
};

struct mxc_ir_platform_data {
	int uart_ir_mux;
	int ir_rx_invert;
	int ir_tx_invert;
	struct clk *uart_clk;
};

struct mxc_i2c_platform_data {
	u32 i2c_clk;
};

/*
 * This struct is to define the number of SSIs on a platform,
 * DAM source port config, DAM external port config,
 * regulator names, and other stuff audio needs.
 */
struct mxc_audio_platform_data {
	int ssi_num;
	int src_port;
	int ext_port;

	int intr_id_hp;
	int ext_ram;
	struct clk *ssi_clk[2];
	char *regulator1;
	char *regulator2;

	int hp_irq;
	int (*hp_status) (void);

	char *vddio_reg;
	char *vdda_reg;
	char *vddd_reg;
	int vddio;		/* voltage of VDDIO (uv) */
	int vdda;		/* voltage of vdda (uv) */
	int vddd;		/* voltage of vddd (uv), 0 if not connected */
	int sysclk;

	int (*init) (void);	/* board specific init */
	int (*amp_enable) (int enable);
	int (*finit) (void);	/* board specific finit */
	void *priv;		/* used by board specific functions */
};

struct mxc_spdif_platform_data {
	int spdif_tx;
	int spdif_rx;
	int spdif_clk_44100;
	int spdif_clk_48000;
	int spdif_clkid;
	struct clk *spdif_clk;
	struct clk *spdif_core_clk;
};

struct mxc_asrc_platform_data {
	struct clk *asrc_core_clk;
	struct clk *asrc_audio_clk;
	unsigned int channel_bits;
};

struct mxc_bt_platform_data {
	char *bt_vdd;
	char *bt_vdd_parent;
	char *bt_vusb;
	char *bt_vusb_parent;
	void (*bt_reset) (void);
};

struct mxc_lightsensor_platform_data {
	char *vdd_reg;
	int rext;
};

struct mxc_fb_platform_data {
	struct fb_videomode *mode;
	char *mode_str;
	u32 interface_pix_fmt;
};

struct mxc_lcd_platform_data {
	char *io_reg;
	char *core_reg;
	char *analog_reg;
	void (*reset) (void);
};

struct mxc_dvfs_platform_data {
	/** Supply voltage regulator name string */
	char *reg_id;
	/* CPU clock name string */
	char *clk1_id;
	/* DVFS clock name string */
	char *clk2_id;
	/* GPC control reg address */
	unsigned int gpc_cntr_reg_addr;
	/* GPC voltage counter reg address */
	unsigned int gpc_vcr_reg_addr;
	/* DVFS threshold reg address */
	unsigned int dvfs_thrs_reg_addr;
	/* DVFS counters reg address */
	unsigned int dvfs_coun_reg_addr;
	/* DVFS EMAC reg address */
	unsigned int dvfs_emac_reg_addr;
	/* DVFS control reg address */
	unsigned int dvfs_cntr_reg_addr;
	/* DIV3CK mask */
	u32 div3ck_mask;
	/* DIV3CK offset */
	int div3ck_offset;
	/* DIV3CK value */
	int div3ck_val;
	/* EMAC value */
	int emac_val;
	/* Frequency increase threshold. Increase frequency change request
	   will be sent if DVFS counter value will be more than this value */
	int upthr_val;
	/* Frequency decrease threshold. Decrease frequency change request
	   will be sent if DVFS counter value will be less than this value */
	int dnthr_val;
	/* Panic threshold. Panic frequency change request
	   will be sent if DVFS counter value will be more than this value */
	int pncthr_val;
	/* The amount of times the up threshold should be exceeded
	   before DVFS will trigger frequency increase request */
	int upcnt_val;
	/* The amount of times the down threshold should be exceeded
	   before DVFS will trigger frequency decrease request */
	int dncnt_val;
	/* Delay time in us */
	int delay_time;
	/* Number of woking points supported */
	int num_wp;
};

struct mxc_tsc_platform_data {
	char *vdd_reg;
	int penup_threshold;
	void (*active) (void);
	void (*inactive) (void);
};

struct mxc_tvout_platform_data {
	char *io_reg;
	char *core_reg;
	char *analog_reg;
	u32 detect_line;
};

struct mxc_tvin_platform_data {
	char *dvddio_reg;
	char *dvdd_reg;
	char *avdd_reg;
	char *pvdd_reg;
	void (*pwdn) (int pwdn);
	void (*reset) (void);
};

/*! Platform data for the IDE drive structure. */
struct mxc_ide_platform_data {
	char *power_drive;	/*!< The power pointer */
	char *power_io;		/*!< The power pointer */
};

struct mxc_camera_platform_data {
	char *core_regulator;
	char *io_regulator;
	char *analog_regulator;
	char *gpo_regulator;
	u32 mclk;
	u32 csi;
};

/*gpo1-3 is in fixed state by hardware design,
 * only deal with reset pin and clock_enable pin
 * only poll mode can be used to control the chip,
 * interrupt mode is not supported by 3ds*/
struct mxc_fm_platform_data {
	char *reg_vio;
	char *reg_vdd;
	void (*gpio_get) (void);
	void (*gpio_put) (void);
	void (*reset) (void);
	void (*clock_ctl) (int flag);
	u8	sksnr; /*0,disable;1,most stop;0xf,fewest stop*/
	u8	skcnt; /*0,disable;1,most stop;0xf,fewest stop*/
	/*
	00 = 87.5-108 MHz (USA,Europe) (Default).
	01 = 76-108 MHz (Japan wide band).
	10 = 76-90 MHz (Japan).
	11 = Reserved.
	*/
	u8	band;
	/*
	00 = 200 kHz (USA, Australia) (default).
	01 = 100 kHz (Europe, Japan).
	10 = 50 kHz.
	*/
	u8	space;
	u8	seekth;
};

struct mxc_mma7450_platform_data {
	char *reg_dvdd_io;
	char *reg_avdd;
	void (*gpio_pin_get) (void);
	void (*gpio_pin_put) (void);
	int int1;
	int int2;
};

struct mxc_keyp_platform_data {
	u16 *matrix;
	void (*active) (void);
	void (*inactive) (void);
	char *vdd_reg;
};

struct mxc_unifi_platform_data {
	void (*hardreset) (int pin_level);
	void (*enable) (int en);
	/* power parameters */
	char *reg_gpo1;
	char *reg_gpo2;
	char *reg_1v5_ana_bb;
	char *reg_vdd_vpa;
	char *reg_1v5_dd;

	int host_id;

	void *priv;
};

struct mxc_gps_platform_data {
	char *core_reg;
	char *analog_reg;
	struct regulator *gps_regu_core;
	struct regulator *gps_regu_analog;
};

struct mxc_mlb_platform_data {
	u32 buf_address;
	u32 phy_address;
	char *reg_nvcc;
	char *mlb_clk;
};

struct flexcan_platform_data {
	char *core_reg;
	char *io_reg;
	void (*xcvr_enable) (int id, int en);
	void (*active) (int id);
	void (*inactive) (int id);
};

struct mxc_srtc_platform_data {
	u32 srtc_sec_mode_addr;
};

struct tve_platform_data {
	char *dac_reg;
	char *dig_reg;
};

extern void mxc_wd_reset(void);
unsigned long board_get_ckih_rate(void);

int mxc_snoop_set_config(u32 num, unsigned long base, int size);
int mxc_snoop_get_status(u32 num, u32 * statl, u32 * stath);

struct platform_device;
void mxc_pg_enable(struct platform_device *pdev);
void mxc_pg_disable(struct platform_device *pdev);

struct mxc_unifi_platform_data *get_unifi_plat_data(void);

struct mxc_sim_platform_data {
	unsigned int clk_rate;
	char *clock_sim;
	char *power_sim;
	int (*init)(struct platform_device *pdev);
	void (*exit)(void);
	unsigned int detect; /* 1 have detect pin, 0 not */
};

#endif				/* __ASSEMBLY__ */

#define MUX_IO_P		29
#define MUX_IO_I		24
#define IOMUX_TO_GPIO(pin) 	((((unsigned int)pin >> MUX_IO_P) * GPIO_NUM_PIN) + ((pin >> MUX_IO_I) & ((1 << (MUX_IO_P - MUX_IO_I)) -1)))
#define IOMUX_TO_IRQ(pin)	(MXC_GPIO_INT_BASE + IOMUX_TO_GPIO(pin))
#define GPIO_TO_PORT(n)		(n / GPIO_NUM_PIN)
#define GPIO_TO_INDEX(n)	(n % GPIO_NUM_PIN)

/* DMA driver defines */
#define MXC_IDE_DMA_WATERMARK	32	/* DMA watermark level in bytes */
#define MXC_IDE_DMA_BD_NR	(512/3/4)	/* Number of BDs per channel */

#ifndef IS_MEM_DEVICE_NONSHARED
/* all peripherals on MXC so far are below 0x80000000 but leave L2CC alone */
#define IS_MEM_DEVICE_NONSHARED(x)  ((x) < 0x80000000 && (x) != L2CC_BASE_ADDR)
#endif
/*!
 * DPTC GP and LP ID
 */
#define DPTC_GP_ID 0
#define DPTC_LP_ID 1

#ifndef __ASSEMBLY__
#include <linux/types.h>

struct cpu_wp {
	u32 pll_reg;
	u32 pll_rate;
	u32 cpu_rate;
	u32 pdr0_reg;
	u32 pdf;
	u32 mfi;
	u32 mfd;
	u32 mfn;
	u32 cpu_voltage;
	u32 cpu_podf;
};

#ifndef CONFIG_ARCH_MX51
struct cpu_wp *get_cpu_wp(int *wp);
#endif

enum mxc_cpu_pwr_mode {
	WAIT_CLOCKED,		/* wfi only */
	WAIT_UNCLOCKED,		/* WAIT */
	WAIT_UNCLOCKED_POWER_OFF,	/* WAIT + SRPG */
	STOP_POWER_ON,		/* just STOP */
	STOP_POWER_OFF,		/* STOP + SRPG */
};

void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode);
int tzic_enable_wake(int is_idle);
void gpio_activate_audio_ports(void);
void gpio_inactivate_audio_ports(void);
void gpio_activate_bt_audio_port(void);
void gpio_inactivate_bt_audio_port(void);
void gpio_activate_esai_ports(void);
void gpio_deactivate_esai_ports(void);

#endif

#endif				/*  __ASM_ARCH_MXC_H__ */
