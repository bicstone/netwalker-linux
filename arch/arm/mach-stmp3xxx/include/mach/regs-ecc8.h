/*
 * STMP ECC8 Register Definitions
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

#ifndef __ARCH_ARM___ECC8_H
#define __ARCH_ARM___ECC8_H  1

#include <mach/stmp3xxx_regs.h>

#define REGS_ECC8_BASE (REGS_BASE + 0x8000)
#define REGS_ECC8_BASE_PHYS (0x80008000)
#define REGS_ECC8_SIZE 0x00002000
HW_REGISTER(HW_ECC8_CTRL, REGS_ECC8_BASE, 0x00000000)
#define HW_ECC8_CTRL_ADDR (REGS_ECC8_BASE + 0x00000000)
#define BM_ECC8_CTRL_SFTRST 0x80000000
#define BV_ECC8_CTRL_SFTRST__RUN   0x0
#define BV_ECC8_CTRL_SFTRST__RESET 0x1
#define BM_ECC8_CTRL_CLKGATE 0x40000000
#define BV_ECC8_CTRL_CLKGATE__RUN     0x0
#define BV_ECC8_CTRL_CLKGATE__NO_CLKS 0x1
#define BM_ECC8_CTRL_AHBM_SFTRST 0x20000000
#define BV_ECC8_CTRL_AHBM_SFTRST__RUN   0x0
#define BV_ECC8_CTRL_AHBM_SFTRST__RESET 0x1
#define BP_ECC8_CTRL_THROTTLE      24
#define BM_ECC8_CTRL_THROTTLE 0x0F000000
#define BF_ECC8_CTRL_THROTTLE(v)  \
	(((v) << 24) & BM_ECC8_CTRL_THROTTLE)
#define BM_ECC8_CTRL_DEBUG_STALL_IRQ_EN 0x00000400
#define BM_ECC8_CTRL_DEBUG_WRITE_IRQ_EN 0x00000200
#define BM_ECC8_CTRL_COMPLETE_IRQ_EN 0x00000100
#define BM_ECC8_CTRL_BM_ERROR_IRQ 0x00000008
#define BM_ECC8_CTRL_DEBUG_STALL_IRQ 0x00000004
#define BM_ECC8_CTRL_DEBUG_WRITE_IRQ 0x00000002
#define BM_ECC8_CTRL_COMPLETE_IRQ 0x00000001
HW_REGISTER_0(HW_ECC8_STATUS0, REGS_ECC8_BASE, 0x00000010)
#define HW_ECC8_STATUS0_ADDR (REGS_ECC8_BASE + 0x00000010)
#define BP_ECC8_STATUS0_HANDLE      20
#define BM_ECC8_STATUS0_HANDLE 0xFFF00000
#define BF_ECC8_STATUS0_HANDLE(v) \
	(((v) << 20) & BM_ECC8_STATUS0_HANDLE)
#define BP_ECC8_STATUS0_COMPLETED_CE      16
#define BM_ECC8_STATUS0_COMPLETED_CE 0x000F0000
#define BF_ECC8_STATUS0_COMPLETED_CE(v)  \
	(((v) << 16) & BM_ECC8_STATUS0_COMPLETED_CE)
#define BM_ECC8_STATUS0_RS8ECC_ENC_PRESENT 0x00008000
#define BM_ECC8_STATUS0_RS8ECC_DEC_PRESENT 0x00004000
#define BM_ECC8_STATUS0_RS4ECC_ENC_PRESENT 0x00002000
#define BM_ECC8_STATUS0_RS4ECC_DEC_PRESENT 0x00001000
#define BP_ECC8_STATUS0_STATUS_AUX      8
#define BM_ECC8_STATUS0_STATUS_AUX 0x00000F00
#define BF_ECC8_STATUS0_STATUS_AUX(v)  \
	(((v) << 8) & BM_ECC8_STATUS0_STATUS_AUX)
#define BV_ECC8_STATUS0_STATUS_AUX__NO_ERRORS	 0x0
#define BV_ECC8_STATUS0_STATUS_AUX__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS0_STATUS_AUX__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS0_STATUS_AUX__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS0_STATUS_AUX__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS0_STATUS_AUX__NOT_CHECKED       0xC
#define BV_ECC8_STATUS0_STATUS_AUX__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS0_STATUS_AUX__ALL_ONES	  0xF
#define BM_ECC8_STATUS0_ALLONES 0x00000010
#define BM_ECC8_STATUS0_CORRECTED 0x00000008
#define BM_ECC8_STATUS0_UNCORRECTABLE 0x00000004
HW_REGISTER_0(HW_ECC8_STATUS1, REGS_ECC8_BASE, 0x00000020)
#define HW_ECC8_STATUS1_ADDR (REGS_ECC8_BASE + 0x00000020)
#define BP_ECC8_STATUS1_STATUS_PAYLOAD7      28
#define BM_ECC8_STATUS1_STATUS_PAYLOAD7 0xF0000000
#define BF_ECC8_STATUS1_STATUS_PAYLOAD7(v) \
	(((v) << 28) & BM_ECC8_STATUS1_STATUS_PAYLOAD7)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD7__ALL_ONES	  0xF
#define BP_ECC8_STATUS1_STATUS_PAYLOAD6      24
#define BM_ECC8_STATUS1_STATUS_PAYLOAD6 0x0F000000
#define BF_ECC8_STATUS1_STATUS_PAYLOAD6(v)  \
	(((v) << 24) & BM_ECC8_STATUS1_STATUS_PAYLOAD6)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD6__ALL_ONES	  0xF
#define BP_ECC8_STATUS1_STATUS_PAYLOAD5      20
#define BM_ECC8_STATUS1_STATUS_PAYLOAD5 0x00F00000
#define BF_ECC8_STATUS1_STATUS_PAYLOAD5(v)  \
	(((v) << 20) & BM_ECC8_STATUS1_STATUS_PAYLOAD5)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD5__ALL_ONES	  0xF
#define BP_ECC8_STATUS1_STATUS_PAYLOAD4      16
#define BM_ECC8_STATUS1_STATUS_PAYLOAD4 0x000F0000
#define BF_ECC8_STATUS1_STATUS_PAYLOAD4(v)  \
	(((v) << 16) & BM_ECC8_STATUS1_STATUS_PAYLOAD4)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD4__ALL_ONES	  0xF
#define BP_ECC8_STATUS1_STATUS_PAYLOAD3      12
#define BM_ECC8_STATUS1_STATUS_PAYLOAD3 0x0000F000
#define BF_ECC8_STATUS1_STATUS_PAYLOAD3(v)  \
	(((v) << 12) & BM_ECC8_STATUS1_STATUS_PAYLOAD3)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD3__ALL_ONES	  0xF
#define BP_ECC8_STATUS1_STATUS_PAYLOAD2      8
#define BM_ECC8_STATUS1_STATUS_PAYLOAD2 0x00000F00
#define BF_ECC8_STATUS1_STATUS_PAYLOAD2(v)  \
	(((v) << 8) & BM_ECC8_STATUS1_STATUS_PAYLOAD2)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD2__ALL_ONES	  0xF
#define BP_ECC8_STATUS1_STATUS_PAYLOAD1      4
#define BM_ECC8_STATUS1_STATUS_PAYLOAD1 0x000000F0
#define BF_ECC8_STATUS1_STATUS_PAYLOAD1(v)  \
	(((v) << 4) & BM_ECC8_STATUS1_STATUS_PAYLOAD1)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD1__ALL_ONES	  0xF
#define BP_ECC8_STATUS1_STATUS_PAYLOAD0      0
#define BM_ECC8_STATUS1_STATUS_PAYLOAD0 0x0000000F
#define BF_ECC8_STATUS1_STATUS_PAYLOAD0(v)  \
	(((v) << 0) & BM_ECC8_STATUS1_STATUS_PAYLOAD0)
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__NO_ERRORS	 0x0
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__ONE_CORRECTABLE   0x1
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__TWO_CORRECTABLE   0x2
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__THREE_CORRECTABLE 0x3
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__FOUR_CORRECTABLE  0x4
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__FIVE_CORRECTABLE  0x5
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__SIX_CORRECTABLE   0x6
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__SEVEN_CORRECTABLE 0x7
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__EIGHT_CORRECTABLE 0x8
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__NOT_CHECKED       0xC
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__UNCORRECTABLE     0xE
#define BV_ECC8_STATUS1_STATUS_PAYLOAD0__ALL_ONES	  0xF
HW_REGISTER(HW_ECC8_DEBUG0, REGS_ECC8_BASE, 0x00000030)
#define HW_ECC8_DEBUG0_ADDR (REGS_ECC8_BASE + 0x00000030)
#define BP_ECC8_DEBUG0_KES_DEBUG_SYNDROME_SYMBOL      16
#define BM_ECC8_DEBUG0_KES_DEBUG_SYNDROME_SYMBOL 0x01FF0000
#define BF_ECC8_DEBUG0_KES_DEBUG_SYNDROME_SYMBOL(v)  \
	(((v) << 16) & BM_ECC8_DEBUG0_KES_DEBUG_SYNDROME_SYMBOL)
#define BV_ECC8_DEBUG0_KES_DEBUG_SYNDROME_SYMBOL__NORMAL    0x0
#define BV_ECC8_DEBUG0_KES_DEBUG_SYNDROME_SYMBOL__TEST_MODE 0x1
#define BM_ECC8_DEBUG0_KES_DEBUG_SHIFT_SYND 0x00008000
#define BM_ECC8_DEBUG0_KES_DEBUG_PAYLOAD_FLAG 0x00004000
#define BV_ECC8_DEBUG0_KES_DEBUG_PAYLOAD_FLAG__DATA 0x1
#define BV_ECC8_DEBUG0_KES_DEBUG_PAYLOAD_FLAG__AUX  0x1
#define BM_ECC8_DEBUG0_KES_DEBUG_MODE4K 0x00002000
#define BV_ECC8_DEBUG0_KES_DEBUG_MODE4K__4k 0x1
#define BV_ECC8_DEBUG0_KES_DEBUG_MODE4K__2k 0x1
#define BM_ECC8_DEBUG0_KES_DEBUG_KICK 0x00001000
#define BM_ECC8_DEBUG0_KES_STANDALONE 0x00000800
#define BV_ECC8_DEBUG0_KES_STANDALONE__NORMAL    0x0
#define BV_ECC8_DEBUG0_KES_STANDALONE__TEST_MODE 0x1
#define BM_ECC8_DEBUG0_KES_DEBUG_STEP 0x00000400
#define BM_ECC8_DEBUG0_KES_DEBUG_STALL 0x00000200
#define BV_ECC8_DEBUG0_KES_DEBUG_STALL__NORMAL 0x0
#define BV_ECC8_DEBUG0_KES_DEBUG_STALL__WAIT   0x1
#define BM_ECC8_DEBUG0_BM_KES_TEST_BYPASS 0x00000100
#define BV_ECC8_DEBUG0_BM_KES_TEST_BYPASS__NORMAL    0x0
#define BV_ECC8_DEBUG0_BM_KES_TEST_BYPASS__TEST_MODE 0x1
#define BP_ECC8_DEBUG0_DEBUG_REG_SELECT      0
#define BM_ECC8_DEBUG0_DEBUG_REG_SELECT 0x0000003F
#define BF_ECC8_DEBUG0_DEBUG_REG_SELECT(v)  \
	(((v) << 0) & BM_ECC8_DEBUG0_DEBUG_REG_SELECT)
HW_REGISTER_0(HW_ECC8_DBGKESREAD, REGS_ECC8_BASE, 0x00000040)
#define HW_ECC8_DBGKESREAD_ADDR (REGS_ECC8_BASE + 0x00000040)
#define BP_ECC8_DBGKESREAD_VALUES      0
#define BM_ECC8_DBGKESREAD_VALUES 0xFFFFFFFF
#define BF_ECC8_DBGKESREAD_VALUES(v)   (v)
HW_REGISTER_0(HW_ECC8_DBGCSFEREAD, REGS_ECC8_BASE, 0x00000050)
#define HW_ECC8_DBGCSFEREAD_ADDR (REGS_ECC8_BASE + 0x00000050)
#define BP_ECC8_DBGCSFEREAD_VALUES      0
#define BM_ECC8_DBGCSFEREAD_VALUES 0xFFFFFFFF
#define BF_ECC8_DBGCSFEREAD_VALUES(v)   (v)
HW_REGISTER_0(HW_ECC8_DBGSYNDGENREAD, REGS_ECC8_BASE, 0x00000060)
#define HW_ECC8_DBGSYNDGENREAD_ADDR (REGS_ECC8_BASE + 0x00000060)
#define BP_ECC8_DBGSYNDGENREAD_VALUES      0
#define BM_ECC8_DBGSYNDGENREAD_VALUES 0xFFFFFFFF
#define BF_ECC8_DBGSYNDGENREAD_VALUES(v)   (v)
HW_REGISTER_0(HW_ECC8_DBGAHBMREAD, REGS_ECC8_BASE, 0x00000070)
#define HW_ECC8_DBGAHBMREAD_ADDR (REGS_ECC8_BASE + 0x00000070)
#define BP_ECC8_DBGAHBMREAD_VALUES      0
#define BM_ECC8_DBGAHBMREAD_VALUES 0xFFFFFFFF
#define BF_ECC8_DBGAHBMREAD_VALUES(v)   (v)
HW_REGISTER_0(HW_ECC8_BLOCKNAME, REGS_ECC8_BASE, 0x00000080)
#define HW_ECC8_BLOCKNAME_ADDR (REGS_ECC8_BASE + 0x00000080)
#define BP_ECC8_BLOCKNAME_NAME      0
#define BM_ECC8_BLOCKNAME_NAME 0xFFFFFFFF
#define BF_ECC8_BLOCKNAME_NAME(v)   (v)
HW_REGISTER_0(HW_ECC8_VERSION, REGS_ECC8_BASE, 0x000000a0)
#define HW_ECC8_VERSION_ADDR (REGS_ECC8_BASE + 0x000000a0)
#define BP_ECC8_VERSION_MAJOR      24
#define BM_ECC8_VERSION_MAJOR 0xFF000000
#define BF_ECC8_VERSION_MAJOR(v) \
	(((v) << 24) & BM_ECC8_VERSION_MAJOR)
#define BP_ECC8_VERSION_MINOR      16
#define BM_ECC8_VERSION_MINOR 0x00FF0000
#define BF_ECC8_VERSION_MINOR(v)  \
	(((v) << 16) & BM_ECC8_VERSION_MINOR)
#define BP_ECC8_VERSION_STEP      0
#define BM_ECC8_VERSION_STEP 0x0000FFFF
#define BF_ECC8_VERSION_STEP(v)  \
	(((v) << 0) & BM_ECC8_VERSION_STEP)
#endif /* __ARCH_ARM___ECC8_H */