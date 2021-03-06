/*
 * include/linux/fsl_devices.h
 *
 * Definitions for any platform device related flags or structures for
 * Freescale processor devices
 *
 * Maintainer: Kumar Gala <galak@kernel.crashing.org>
 *
 * Copyright 2004, 2009 Freescale Semiconductor, Inc
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _FSL_DEVICE_H_
#define _FSL_DEVICE_H_

#include <linux/types.h>
#include <linux/phy.h>

/*
 * Some conventions on how we handle peripherals on Freescale chips
 *
 * unique device: a platform_device entry in fsl_plat_devs[] plus
 * associated device information in its platform_data structure.
 *
 * A chip is described by a set of unique devices.
 *
 * Each sub-arch has its own master list of unique devices and
 * enumerates them by enum fsl_devices in a sub-arch specific header
 *
 * The platform data structure is broken into two parts.  The
 * first is device specific information that help identify any
 * unique features of a peripheral.  The second is any
 * information that may be defined by the board or how the device
 * is connected externally of the chip.
 *
 * naming conventions:
 * - platform data structures: <driver>_platform_data
 * - platform data device flags: FSL_<driver>_DEV_<FLAG>
 * - platform data board flags: FSL_<driver>_BRD_<FLAG>
 *
 */

struct gianfar_platform_data {
	/* device specific information */
	u32	device_flags;
	/* board specific information */
	u32	board_flags;
	int	mdio_bus;			/* Bus controlled by us */
	char	bus_id[MII_BUS_ID_SIZE];	/* Bus PHY is on */
	u32	phy_id;
	u8	mac_addr[6];
	phy_interface_t interface;
};

struct gianfar_mdio_data {
	/* board specific information */
	int	irq[32];
};

/* Flags related to gianfar device features */
#define FSL_GIANFAR_DEV_HAS_GIGABIT		0x00000001
#define FSL_GIANFAR_DEV_HAS_COALESCE		0x00000002
#define FSL_GIANFAR_DEV_HAS_RMON		0x00000004
#define FSL_GIANFAR_DEV_HAS_MULTI_INTR		0x00000008
#define FSL_GIANFAR_DEV_HAS_CSUM		0x00000010
#define FSL_GIANFAR_DEV_HAS_VLAN		0x00000020
#define FSL_GIANFAR_DEV_HAS_EXTENDED_HASH	0x00000040
#define FSL_GIANFAR_DEV_HAS_PADDING		0x00000080
#define FSL_GIANFAR_DEV_HAS_MAGIC_PACKET	0x00000100

/* Flags in gianfar_platform_data */
#define FSL_GIANFAR_BRD_HAS_PHY_INTR	0x00000001 /* set or use a timer */
#define FSL_GIANFAR_BRD_IS_REDUCED	0x00000002 /* Set if RGMII, RMII */

struct fsl_i2c_platform_data {
	/* device specific information */
	u32	device_flags;
};

/* Flags related to I2C device features */
#define FSL_I2C_DEV_SEPARATE_DFSRR	0x00000001
#define FSL_I2C_DEV_CLOCK_5200		0x00000002

enum fsl_usb2_operating_modes {
	FSL_USB2_MPH_HOST,
	FSL_USB2_DR_HOST,
	FSL_USB2_DR_DEVICE,
	FSL_USB2_DR_OTG,
};

enum fsl_usb2_phy_modes {
	FSL_USB2_PHY_NONE,
	FSL_USB2_PHY_ULPI,
	FSL_USB2_PHY_UTMI,
	FSL_USB2_PHY_UTMI_WIDE,
	FSL_USB2_PHY_SERIAL,
};

struct platform_device;
struct fsl_usb2_platform_data {
	/* board specific information */
	enum fsl_usb2_operating_modes	operating_mode;
	enum fsl_usb2_phy_modes		phy_mode;
	unsigned int			port_enables;

	char *name;		/* pretty print */
	int (*platform_init) (struct platform_device *);
	void (*platform_uninit) (struct fsl_usb2_platform_data *);
	void __iomem *regs;	/* ioremap'd register base */
	u32 xcvr_type;		/* PORTSC_PTS_* */
	char *transceiver;	/* transceiver name */
	unsigned power_budget;	/* for hcd->power_budget */
	struct platform_device *pdev;
	struct fsl_xcvr_ops *xcvr_ops;
	struct fsl_xcvr_power *xcvr_pwr;
	int (*gpio_usb_active) (void);
	void (*gpio_usb_inactive) (void);
	void (*usb_clock_for_pm) (bool);
	unsigned			big_endian_mmio : 1;
	unsigned			big_endian_desc : 1;
	unsigned			es : 1;	/* need USBMODE:ES */
	unsigned			have_sysif_regs : 1;
	unsigned			le_setup_buf : 1;
	unsigned change_ahb_burst:1;
	unsigned ahb_burst_mode:3;
	unsigned			suspended : 1;
	unsigned			already_suspended : 1;

	/* register save area for suspend/resume */
	u32				pm_command;
	u32				pm_status;
	u32				pm_intr_enable;
	u32				pm_frame_index;
	u32				pm_segment;
	u32				pm_frame_list;
	u32				pm_async_next;
	u32				pm_configured_flag;
	u32				pm_portsc;
};

/* Flags in fsl_usb2_mph_platform_data */
#define FSL_USB2_PORT0_ENABLED	0x00000001
#define FSL_USB2_PORT1_ENABLED	0x00000002
#define FSL_USB2_DONT_REMAP	0x10000000

struct fsl_spi_platform_data {
	u32 	initial_spmode;	/* initial SPMODE value */
	u16	bus_num;
	bool	qe_mode;
	/* board specific information */
	u16	max_chipselect;
	void	(*activate_cs)(u8 cs, u8 polarity);
	void	(*deactivate_cs)(u8 cs, u8 polarity);
	u32	sysclk;
};

struct fsl_ata_platform_data {
       int     adma_flag;      /* AMDA mode is used or not, 1:used.*/
       int     udma_mask;      /* UDMA modes h/w can handle */
       int     mwdma_mask;      /* MDMA modes h/w can handle */
       int     pio_mask;      /* PIO modes h/w can handle */
       int     fifo_alarm;     /* value for fifo_alarm reg */
       int     max_sg;         /* longest sglist h/w can handle */
       int     (*init)(struct platform_device *pdev);
       void    (*exit)(void);
	   char    *io_reg;
	   char    *core_reg;
};


struct mpc8xx_pcmcia_ops {
	void(*hw_ctrl)(int slot, int enable);
	int(*voltage_set)(int slot, int vcc, int vpp);
};

/* Returns non-zero if the current suspend operation would
 * lead to a deep sleep (i.e. power removed from the core,
 * instead of just the clock).
 */
int fsl_deep_sleep(void);

#endif /* _FSL_DEVICE_H_ */
