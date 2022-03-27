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

#ifndef __ASM_ARCH_MXC_BOARD_MX31PDK_H__
#define __ASM_ARCH_MXC_BOARD_MX31PDK_H__

#ifdef CONFIG_MACH_MX31_3DS
/*!
 * @defgroup BRDCFG_MX31 Board Configuration Options
 * @ingroup MSL_MX31
 */

/*!
 * @file mach-mx3/board-mx3_3stack.h
 *
 * @brief This file contains all the board level configuration options.
 *
 * It currently hold the options defined for MX31 3STACK Platform.
 *
 * @ingroup BRDCFG_MX31
 */

/*
 * Include Files
 */
#include <mach/mxc_uart.h>

/*!
 * @name MXC UART EVB board level configurations
 */
/*! @{ */
/*!
 * Specifies if the Irda transmit path is inverting
 */
#define MXC_IRDA_TX_INV         0
/*!
 * Specifies if the Irda receive path is inverting
 */
#define MXC_IRDA_RX_INV         0

/* UART 1 configuration */
/*!
 * This define specifies if the UART port is configured to be in DTE or
 * DCE mode. There exists a define like this for each UART port. Valid
 * values that can be used are \b MODE_DTE or \b MODE_DCE.
 */
#define UART1_MODE              MODE_DCE
/*!
 * This define specifies if the UART is to be used for IRDA. There exists a
 * define like this for each UART port. Valid values that can be used are
 * \b IRDA or \b NO_IRDA.
 */
#define UART1_IR                NO_IRDA
/*!
 * This define is used to enable or disable a particular UART port. If
 * disabled, the UART will not be registered in the file system and the user
 * will not be able to access it. There exists a define like this for each UART
 * port. Specify a value of 1 to enable the UART and 0 to disable it.
 */
#define UART1_ENABLED           1
/*! @} */
/* UART 2 configuration */
#define UART2_MODE              MODE_DCE
#define UART2_IR                NO_IRDA
#define UART2_ENABLED           1
/* UART 3 configuration */
#define UART3_MODE              MODE_DTE
#define UART3_IR                NO_IRDA
#define UART3_ENABLED           1
/* UART 4 configuration */
#define UART4_MODE              MODE_DTE
#define UART4_IR                NO_IRDA
#define UART4_ENABLED           0	/* Disable UART 4 as its pins are shared with ATA */
/* UART 5 configuration */
#define UART5_MODE              MODE_DTE
#define UART5_IR                NO_IRDA
#define UART5_ENABLED           0

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

#define DEBUG_BASE_ADDRESS	CS5_BASE_ADDR
/* LAN9217 ethernet base address */
#define LAN9217_BASE_ADDR	DEBUG_BASE_ADDRESS
/* External UART */
#define UARTA_BASE_ADDR		(DEBUG_BASE_ADDRESS + 0x8000)
#define UARTB_BASE_ADDR		(DEBUG_BASE_ADDRESS + 0x10000)

#define BOARD_IO_ADDR		(DEBUG_BASE_ADDRESS + 0x20000)
/* LED switchs */
#define LED_SWITCH_REG		0x00
/* buttons */
#define SWITCH_BUTTONS_REG	0x08
/* status, interrupt */
#define INTR_STATUS_REG		0x10
#define INTR_MASK_REG		0x38
#define INTR_RESET_REG		0x20
/* magic word for debug CPLD */
#define MAGIC_NUMBER1_REG	0x40
#define MAGIC_NUMBER2_REG	0x48
/* CPLD code version */
#define CPLD_CODE_VER_REG	0x50
/* magic word for debug CPLD */
#define MAGIC_NUMBER3_REG	0x58
/* module reset register*/
#define MODULE_RESET_REG	0x60
/* CPU ID and Personality ID */
#define MCU_BOARD_ID_REG	0x68

/* interrupts like external uart , external ethernet etc*/
#define EXPIO_PARENT_INT	IOMUX_TO_IRQ(MX31_PIN_GPIO1_1)

#define EXPIO_INT_ENET		(MXC_EXP_IO_BASE + 0)
#define EXPIO_INT_XUART_A 	(MXC_EXP_IO_BASE + 1)
#define EXPIO_INT_XUART_B 	(MXC_EXP_IO_BASE + 2)
#define EXPIO_INT_BUTTON_A 	(MXC_EXP_IO_BASE + 3)
#define EXPIO_INT_BUTTON_B 	(MXC_EXP_IO_BASE + 4)

/*! This is System IRQ used by LAN9217 */
#define LAN9217_IRQ	EXPIO_INT_ENET

/*! LED definition*/
#define MXC_BD_LED1	(1)
#define MXC_BD_LED2	(1 << 1)
#define MXC_BD_LED3	(1 << 2)
#define MXC_BD_LED4	(1 << 3)
#define MXC_BD_LED5	(1 << 4)
#define MXC_BD_LED6	(1 << 5)
#define MXC_BD_LED7	(1 << 6)
#define MXC_BD_LED8	(1 << 7)

#define MXC_BD_LED_ON(led)
#define MXC_BD_LED_OFF(led)

extern unsigned int sdhc_get_card_det_status(struct device *dev);
extern int sdhc_init_card_det(int id);
extern int sdhc_write_protect(struct device *dev);

extern int __init mx3_3stack_init_mc13783(void);

#endif				/* CONFIG_MACH_MX31_3DS */
#endif				/* __ASM_ARCH_MXC_BOARD_MX31PDK_H__ */
