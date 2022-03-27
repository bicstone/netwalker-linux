/*
 * STMP SPDIF Register Definitions
 *
 * Copyright 2008-2009 Freescale Semiconductor
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __ARCH_ARM___SPDIF_H
#define __ARCH_ARM___SPDIF_H  1

#include <mach/stmp3xxx_regs.h>

#define REGS_SPDIF_BASE (REGS_BASE + 0x54000)
#define REGS_SPDIF_BASE_PHYS (0x80054000)
#define REGS_SPDIF_SIZE 0x00002000
HW_REGISTER(HW_SPDIF_CTRL, REGS_SPDIF_BASE, 0x00000000)
#define HW_SPDIF_CTRL_ADDR (REGS_SPDIF_BASE + 0x00000000)
#define BM_SPDIF_CTRL_SFTRST 0x80000000
#define BM_SPDIF_CTRL_CLKGATE 0x40000000
#define BP_SPDIF_CTRL_DMAWAIT_COUNT      16
#define BM_SPDIF_CTRL_DMAWAIT_COUNT 0x001F0000
#define BF_SPDIF_CTRL_DMAWAIT_COUNT(v)  \
	(((v) << 16) & BM_SPDIF_CTRL_DMAWAIT_COUNT)
#define BM_SPDIF_CTRL_WAIT_END_XFER 0x00000020
#define BM_SPDIF_CTRL_WORD_LENGTH 0x00000010
#define BM_SPDIF_CTRL_FIFO_UNDERFLOW_IRQ 0x00000008
#define BM_SPDIF_CTRL_FIFO_OVERFLOW_IRQ 0x00000004
#define BM_SPDIF_CTRL_FIFO_ERROR_IRQ_EN 0x00000002
#define BM_SPDIF_CTRL_RUN 0x00000001
HW_REGISTER(HW_SPDIF_STAT, REGS_SPDIF_BASE, 0x00000010)
#define HW_SPDIF_STAT_ADDR (REGS_SPDIF_BASE + 0x00000010)
#define BM_SPDIF_STAT_PRESENT 0x80000000
#define BM_SPDIF_STAT_END_XFER 0x00000001
HW_REGISTER(HW_SPDIF_FRAMECTRL, REGS_SPDIF_BASE, 0x00000020)
#define HW_SPDIF_FRAMECTRL_ADDR (REGS_SPDIF_BASE + 0x00000020)
#define BM_SPDIF_FRAMECTRL_V_CONFIG 0x00020000
#define BM_SPDIF_FRAMECTRL_AUTO_MUTE 0x00010000
#define BM_SPDIF_FRAMECTRL_USER_DATA 0x00004000
#define BM_SPDIF_FRAMECTRL_V 0x00002000
#define BM_SPDIF_FRAMECTRL_L 0x00001000
#define BP_SPDIF_FRAMECTRL_CC      4
#define BM_SPDIF_FRAMECTRL_CC 0x000007F0
#define BF_SPDIF_FRAMECTRL_CC(v)  \
	(((v) << 4) & BM_SPDIF_FRAMECTRL_CC)
#define BM_SPDIF_FRAMECTRL_PRE 0x00000008
#define BM_SPDIF_FRAMECTRL_COPY 0x00000004
#define BM_SPDIF_FRAMECTRL_AUDIO 0x00000002
#define BM_SPDIF_FRAMECTRL_PRO 0x00000001
HW_REGISTER(HW_SPDIF_SRR, REGS_SPDIF_BASE, 0x00000030)
#define HW_SPDIF_SRR_ADDR (REGS_SPDIF_BASE + 0x00000030)
#define BP_SPDIF_SRR_BASEMULT      28
#define BM_SPDIF_SRR_BASEMULT 0x70000000
#define BF_SPDIF_SRR_BASEMULT(v)  \
	(((v) << 28) & BM_SPDIF_SRR_BASEMULT)
#define BP_SPDIF_SRR_RATE      0
#define BM_SPDIF_SRR_RATE 0x000FFFFF
#define BF_SPDIF_SRR_RATE(v)  \
	(((v) << 0) & BM_SPDIF_SRR_RATE)
HW_REGISTER(HW_SPDIF_DEBUG, REGS_SPDIF_BASE, 0x00000040)
#define HW_SPDIF_DEBUG_ADDR (REGS_SPDIF_BASE + 0x00000040)
#define BM_SPDIF_DEBUG_DMA_PREQ 0x00000002
#define BM_SPDIF_DEBUG_FIFO_STATUS 0x00000001
HW_REGISTER(HW_SPDIF_DATA, REGS_SPDIF_BASE, 0x00000050)
#define HW_SPDIF_DATA_ADDR (REGS_SPDIF_BASE + 0x00000050)
#define BP_SPDIF_DATA_HIGH      16
#define BM_SPDIF_DATA_HIGH 0xFFFF0000
#define BF_SPDIF_DATA_HIGH(v) \
	(((v) << 16) & BM_SPDIF_DATA_HIGH)
#define BP_SPDIF_DATA_LOW      0
#define BM_SPDIF_DATA_LOW 0x0000FFFF
#define BF_SPDIF_DATA_LOW(v)  \
	(((v) << 0) & BM_SPDIF_DATA_LOW)
HW_REGISTER_0(HW_SPDIF_VERSION, REGS_SPDIF_BASE, 0x00000060)
#define HW_SPDIF_VERSION_ADDR (REGS_SPDIF_BASE + 0x00000060)
#define BP_SPDIF_VERSION_MAJOR      24
#define BM_SPDIF_VERSION_MAJOR 0xFF000000
#define BF_SPDIF_VERSION_MAJOR(v) \
	(((v) << 24) & BM_SPDIF_VERSION_MAJOR)
#define BP_SPDIF_VERSION_MINOR      16
#define BM_SPDIF_VERSION_MINOR 0x00FF0000
#define BF_SPDIF_VERSION_MINOR(v)  \
	(((v) << 16) & BM_SPDIF_VERSION_MINOR)
#define BP_SPDIF_VERSION_STEP      0
#define BM_SPDIF_VERSION_STEP 0x0000FFFF
#define BF_SPDIF_VERSION_STEP(v)  \
	(((v) << 0) & BM_SPDIF_VERSION_STEP)
#endif /* __ARCH_ARM___SPDIF_H */