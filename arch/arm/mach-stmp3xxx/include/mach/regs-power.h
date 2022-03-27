/*
 * STMP POWER Register Definitions
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

#ifndef __ARCH_ARM___POWER_H
#define __ARCH_ARM___POWER_H  1

#include <mach/stmp3xxx_regs.h>

#define REGS_POWER_BASE (REGS_BASE + 0x44000)
#define REGS_POWER_BASE_PHYS (0x80044000)
#define REGS_POWER_SIZE 0x00002000
HW_REGISTER(HW_POWER_CTRL, REGS_POWER_BASE, 0x00000000)
#define HW_POWER_CTRL_ADDR (REGS_POWER_BASE + 0x00000000)
#define BM_POWER_CTRL_CLKGATE 0x40000000
#define BM_POWER_CTRL_PSWITCH_MID_TRAN 0x08000000
#define BM_POWER_CTRL_DCDC4P2_BO_IRQ 0x01000000
#define BM_POWER_CTRL_ENIRQ_DCDC4P2_BO 0x00800000
#define BM_POWER_CTRL_VDD5V_DROOP_IRQ 0x00400000
#define BM_POWER_CTRL_ENIRQ_VDD5V_DROOP 0x00200000
#define BM_POWER_CTRL_PSWITCH_IRQ 0x00100000
#define BM_POWER_CTRL_PSWITCH_IRQ_SRC 0x00080000
#define BM_POWER_CTRL_POLARITY_PSWITCH 0x00040000
#define BM_POWER_CTRL_ENIRQ_PSWITCH 0x00020000
#define BM_POWER_CTRL_POLARITY_DC_OK 0x00010000
#define BM_POWER_CTRL_DC_OK_IRQ 0x00008000
#define BM_POWER_CTRL_ENIRQ_DC_OK 0x00004000
#define BM_POWER_CTRL_BATT_BO_IRQ 0x00002000
#define BM_POWER_CTRL_ENIRQBATT_BO 0x00001000
#define BM_POWER_CTRL_VDDIO_BO_IRQ 0x00000800
#define BM_POWER_CTRL_ENIRQ_VDDIO_BO 0x00000400
#define BM_POWER_CTRL_VDDA_BO_IRQ 0x00000200
#define BM_POWER_CTRL_ENIRQ_VDDA_BO 0x00000100
#define BM_POWER_CTRL_VDDD_BO_IRQ 0x00000080
#define BM_POWER_CTRL_ENIRQ_VDDD_BO 0x00000040
#define BM_POWER_CTRL_POLARITY_VBUSVALID 0x00000020
#define BM_POWER_CTRL_VBUSVALID_IRQ 0x00000010
#define BM_POWER_CTRL_ENIRQ_VBUS_VALID 0x00000008
#define BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO 0x00000004
#define BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ 0x00000002
#define BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO 0x00000001
HW_REGISTER(HW_POWER_5VCTRL, REGS_POWER_BASE, 0x00000010)
#define HW_POWER_5VCTRL_ADDR (REGS_POWER_BASE + 0x00000010)
#define BP_POWER_5VCTRL_VBUSDROOP_TRSH      28
#define BM_POWER_5VCTRL_VBUSDROOP_TRSH 0x30000000
#define BF_POWER_5VCTRL_VBUSDROOP_TRSH(v)  \
	(((v) << 28) & BM_POWER_5VCTRL_VBUSDROOP_TRSH)
#define BP_POWER_5VCTRL_HEADROOM_ADJ      24
#define BM_POWER_5VCTRL_HEADROOM_ADJ 0x07000000
#define BF_POWER_5VCTRL_HEADROOM_ADJ(v)  \
	(((v) << 24) & BM_POWER_5VCTRL_HEADROOM_ADJ)
#define BM_POWER_5VCTRL_PWD_CHARGE_4P2 0x00100000
#define BP_POWER_5VCTRL_CHARGE_4P2_ILIMIT      12
#define BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT 0x0003F000
#define BF_POWER_5VCTRL_CHARGE_4P2_ILIMIT(v)  \
	(((v) << 12) & BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT)
#define BP_POWER_5VCTRL_VBUSVALID_TRSH      8
#define BM_POWER_5VCTRL_VBUSVALID_TRSH 0x00000700
#define BF_POWER_5VCTRL_VBUSVALID_TRSH(v)  \
	(((v) << 8) & BM_POWER_5VCTRL_VBUSVALID_TRSH)
#define BM_POWER_5VCTRL_PWDN_5VBRNOUT 0x00000080
#define BM_POWER_5VCTRL_ENABLE_LINREG_ILIMIT 0x00000040
#define BM_POWER_5VCTRL_DCDC_XFER 0x00000020
#define BM_POWER_5VCTRL_VBUSVALID_5VDETECT 0x00000010
#define BM_POWER_5VCTRL_VBUSVALID_TO_B 0x00000008
#define BM_POWER_5VCTRL_ILIMIT_EQ_ZERO 0x00000004
#define BM_POWER_5VCTRL_PWRUP_VBUS_CMPS 0x00000002
#define BM_POWER_5VCTRL_ENABLE_DCDC 0x00000001
HW_REGISTER(HW_POWER_MINPWR, REGS_POWER_BASE, 0x00000020)
#define HW_POWER_MINPWR_ADDR (REGS_POWER_BASE + 0x00000020)
#define BM_POWER_MINPWR_LOWPWR_4P2 0x00004000
#define BM_POWER_MINPWR_VDAC_DUMP_CTRL 0x00002000
#define BM_POWER_MINPWR_PWD_BO 0x00001000
#define BM_POWER_MINPWR_USE_VDDXTAL_VBG 0x00000800
#define BM_POWER_MINPWR_PWD_ANA_CMPS 0x00000400
#define BM_POWER_MINPWR_ENABLE_OSC 0x00000200
#define BM_POWER_MINPWR_SELECT_OSC 0x00000100
#define BM_POWER_MINPWR_VBG_OFF 0x00000080
#define BM_POWER_MINPWR_DOUBLE_FETS 0x00000040
#define BM_POWER_MINPWR_HALF_FETS 0x00000020
#define BM_POWER_MINPWR_LESSANA_I 0x00000010
#define BM_POWER_MINPWR_PWD_XTAL24 0x00000008
#define BM_POWER_MINPWR_DC_STOPCLK 0x00000004
#define BM_POWER_MINPWR_EN_DC_PFM 0x00000002
#define BM_POWER_MINPWR_DC_HALFCLK 0x00000001
HW_REGISTER(HW_POWER_CHARGE, REGS_POWER_BASE, 0x00000030)
#define HW_POWER_CHARGE_ADDR (REGS_POWER_BASE + 0x00000030)
#define BP_POWER_CHARGE_ADJ_VOLT      24
#define BM_POWER_CHARGE_ADJ_VOLT 0x07000000
#define BF_POWER_CHARGE_ADJ_VOLT(v)  \
	(((v) << 24) & BM_POWER_CHARGE_ADJ_VOLT)
#define BM_POWER_CHARGE_RSRVD3 0x00800000
#define BM_POWER_CHARGE_ENABLE_LOAD 0x00400000
#define BM_POWER_CHARGE_ENABLE_CHARGER_RESISTORS 0x00200000
#define BM_POWER_CHARGE_ENABLE_FAULT_DETECT 0x00100000
#define BM_POWER_CHARGE_CHRG_STS_OFF 0x00080000
#define BM_POWER_CHARGE_USE_EXTERN_R 0x00020000
#define BM_POWER_CHARGE_PWD_BATTCHRG 0x00010000
#define BP_POWER_CHARGE_STOP_ILIMIT      8
#define BM_POWER_CHARGE_STOP_ILIMIT 0x00000F00
#define BF_POWER_CHARGE_STOP_ILIMIT(v)  \
	(((v) << 8) & BM_POWER_CHARGE_STOP_ILIMIT)
#define BP_POWER_CHARGE_BATTCHRG_I      0
#define BM_POWER_CHARGE_BATTCHRG_I 0x0000003F
#define BF_POWER_CHARGE_BATTCHRG_I(v)  \
	(((v) << 0) & BM_POWER_CHARGE_BATTCHRG_I)
HW_REGISTER_0(HW_POWER_VDDDCTRL, REGS_POWER_BASE, 0x00000040)
#define HW_POWER_VDDDCTRL_ADDR (REGS_POWER_BASE + 0x00000040)
#define BP_POWER_VDDDCTRL_ADJTN      28
#define BM_POWER_VDDDCTRL_ADJTN 0xF0000000
#define BF_POWER_VDDDCTRL_ADJTN(v) \
	(((v) << 28) & BM_POWER_VDDDCTRL_ADJTN)
#define BM_POWER_VDDDCTRL_PWDN_BRNOUT 0x00800000
#define BM_POWER_VDDDCTRL_DISABLE_STEPPING 0x00400000
#define BM_POWER_VDDDCTRL_ENABLE_LINREG 0x00200000
#define BM_POWER_VDDDCTRL_DISABLE_FET 0x00100000
#define BP_POWER_VDDDCTRL_LINREG_OFFSET      16
#define BM_POWER_VDDDCTRL_LINREG_OFFSET 0x00030000
#define BF_POWER_VDDDCTRL_LINREG_OFFSET(v)  \
	(((v) << 16) & BM_POWER_VDDDCTRL_LINREG_OFFSET)
#define BP_POWER_VDDDCTRL_BO_OFFSET      8
#define BM_POWER_VDDDCTRL_BO_OFFSET 0x00000700
#define BF_POWER_VDDDCTRL_BO_OFFSET(v)  \
	(((v) << 8) & BM_POWER_VDDDCTRL_BO_OFFSET)
#define BP_POWER_VDDDCTRL_TRG      0
#define BM_POWER_VDDDCTRL_TRG 0x0000001F
#define BF_POWER_VDDDCTRL_TRG(v)  \
	(((v) << 0) & BM_POWER_VDDDCTRL_TRG)
HW_REGISTER_0(HW_POWER_VDDACTRL, REGS_POWER_BASE, 0x00000050)
#define HW_POWER_VDDACTRL_ADDR (REGS_POWER_BASE + 0x00000050)
#define BM_POWER_VDDACTRL_PWDN_BRNOUT 0x00080000
#define BM_POWER_VDDACTRL_DISABLE_STEPPING 0x00040000
#define BM_POWER_VDDACTRL_ENABLE_LINREG 0x00020000
#define BM_POWER_VDDACTRL_DISABLE_FET 0x00010000
#define BP_POWER_VDDACTRL_LINREG_OFFSET      12
#define BM_POWER_VDDACTRL_LINREG_OFFSET 0x00003000
#define BF_POWER_VDDACTRL_LINREG_OFFSET(v)  \
	(((v) << 12) & BM_POWER_VDDACTRL_LINREG_OFFSET)
#define BP_POWER_VDDACTRL_BO_OFFSET      8
#define BM_POWER_VDDACTRL_BO_OFFSET 0x00000700
#define BF_POWER_VDDACTRL_BO_OFFSET(v)  \
	(((v) << 8) & BM_POWER_VDDACTRL_BO_OFFSET)
#define BP_POWER_VDDACTRL_TRG      0
#define BM_POWER_VDDACTRL_TRG 0x0000001F
#define BF_POWER_VDDACTRL_TRG(v)  \
	(((v) << 0) & BM_POWER_VDDACTRL_TRG)
HW_REGISTER_0(HW_POWER_VDDIOCTRL, REGS_POWER_BASE, 0x00000060)
#define HW_POWER_VDDIOCTRL_ADDR (REGS_POWER_BASE + 0x00000060)
#define BP_POWER_VDDIOCTRL_ADJTN      20
#define BM_POWER_VDDIOCTRL_ADJTN 0x00F00000
#define BF_POWER_VDDIOCTRL_ADJTN(v)  \
	(((v) << 20) & BM_POWER_VDDIOCTRL_ADJTN)
#define BM_POWER_VDDIOCTRL_PWDN_BRNOUT 0x00040000
#define BM_POWER_VDDIOCTRL_DISABLE_STEPPING 0x00020000
#define BM_POWER_VDDIOCTRL_DISABLE_FET 0x00010000
#define BP_POWER_VDDIOCTRL_LINREG_OFFSET      12
#define BM_POWER_VDDIOCTRL_LINREG_OFFSET 0x00003000
#define BF_POWER_VDDIOCTRL_LINREG_OFFSET(v)  \
	(((v) << 12) & BM_POWER_VDDIOCTRL_LINREG_OFFSET)
#define BP_POWER_VDDIOCTRL_BO_OFFSET      8
#define BM_POWER_VDDIOCTRL_BO_OFFSET 0x00000700
#define BF_POWER_VDDIOCTRL_BO_OFFSET(v)  \
	(((v) << 8) & BM_POWER_VDDIOCTRL_BO_OFFSET)
#define BP_POWER_VDDIOCTRL_TRG      0
#define BM_POWER_VDDIOCTRL_TRG 0x0000001F
#define BF_POWER_VDDIOCTRL_TRG(v)  \
	(((v) << 0) & BM_POWER_VDDIOCTRL_TRG)
HW_REGISTER_0(HW_POWER_VDDMEMCTRL, REGS_POWER_BASE, 0x00000070)
#define HW_POWER_VDDMEMCTRL_ADDR (REGS_POWER_BASE + 0x00000070)
#define BM_POWER_VDDMEMCTRL_PULLDOWN_ACTIVE 0x00000400
#define BM_POWER_VDDMEMCTRL_ENABLE_ILIMIT 0x00000200
#define BM_POWER_VDDMEMCTRL_ENABLE_LINREG 0x00000100
#define BP_POWER_VDDMEMCTRL_TRG      0
#define BM_POWER_VDDMEMCTRL_TRG 0x0000001F
#define BF_POWER_VDDMEMCTRL_TRG(v)  \
	(((v) << 0) & BM_POWER_VDDMEMCTRL_TRG)
HW_REGISTER_0(HW_POWER_DCDC4P2, REGS_POWER_BASE, 0x00000080)
#define HW_POWER_DCDC4P2_ADDR (REGS_POWER_BASE + 0x00000080)
#define BP_POWER_DCDC4P2_DROPOUT_CTRL      28
#define BM_POWER_DCDC4P2_DROPOUT_CTRL 0xF0000000
#define BF_POWER_DCDC4P2_DROPOUT_CTRL(v) \
	(((v) << 28) & BM_POWER_DCDC4P2_DROPOUT_CTRL)
#define BP_POWER_DCDC4P2_ISTEAL_THRESH      24
#define BM_POWER_DCDC4P2_ISTEAL_THRESH 0x03000000
#define BF_POWER_DCDC4P2_ISTEAL_THRESH(v)  \
	(((v) << 24) & BM_POWER_DCDC4P2_ISTEAL_THRESH)
#define BM_POWER_DCDC4P2_ENABLE_4P2 0x00800000
#define BM_POWER_DCDC4P2_ENABLE_DCDC 0x00400000
#define BM_POWER_DCDC4P2_HYST_DIR 0x00200000
#define BM_POWER_DCDC4P2_HYST_THRESH 0x00100000
#define BP_POWER_DCDC4P2_TRG      16
#define BM_POWER_DCDC4P2_TRG 0x00070000
#define BF_POWER_DCDC4P2_TRG(v)  \
	(((v) << 16) & BM_POWER_DCDC4P2_TRG)
#define BP_POWER_DCDC4P2_BO      8
#define BM_POWER_DCDC4P2_BO 0x00001F00
#define BF_POWER_DCDC4P2_BO(v)  \
	(((v) << 8) & BM_POWER_DCDC4P2_BO)
#define BP_POWER_DCDC4P2_CMPTRIP      0
#define BM_POWER_DCDC4P2_CMPTRIP 0x0000001F
#define BF_POWER_DCDC4P2_CMPTRIP(v)  \
	(((v) << 0) & BM_POWER_DCDC4P2_CMPTRIP)
HW_REGISTER_0(HW_POWER_MISC, REGS_POWER_BASE, 0x00000090)
#define HW_POWER_MISC_ADDR (REGS_POWER_BASE + 0x00000090)
#define BP_POWER_MISC_FREQSEL      4
#define BM_POWER_MISC_FREQSEL 0x00000070
#define BF_POWER_MISC_FREQSEL(v)  \
	(((v) << 4) & BM_POWER_MISC_FREQSEL)
#define BM_POWER_MISC_RSRVD1 0x00000008
#define BM_POWER_MISC_DELAY_TIMING 0x00000004
#define BM_POWER_MISC_TEST 0x00000002
#define BM_POWER_MISC_SEL_PLLCLK 0x00000001
HW_REGISTER_0(HW_POWER_DCLIMITS, REGS_POWER_BASE, 0x000000a0)
#define HW_POWER_DCLIMITS_ADDR (REGS_POWER_BASE + 0x000000a0)
#define BP_POWER_DCLIMITS_POSLIMIT_BUCK      8
#define BM_POWER_DCLIMITS_POSLIMIT_BUCK 0x00007F00
#define BF_POWER_DCLIMITS_POSLIMIT_BUCK(v)  \
	(((v) << 8) & BM_POWER_DCLIMITS_POSLIMIT_BUCK)
#define BP_POWER_DCLIMITS_NEGLIMIT      0
#define BM_POWER_DCLIMITS_NEGLIMIT 0x0000007F
#define BF_POWER_DCLIMITS_NEGLIMIT(v)  \
	(((v) << 0) & BM_POWER_DCLIMITS_NEGLIMIT)
HW_REGISTER(HW_POWER_LOOPCTRL, REGS_POWER_BASE, 0x000000b0)
#define HW_POWER_LOOPCTRL_ADDR (REGS_POWER_BASE + 0x000000b0)
#define BM_POWER_LOOPCTRL_TOGGLE_DIF 0x00100000
#define BM_POWER_LOOPCTRL_HYST_SIGN 0x00080000
#define BM_POWER_LOOPCTRL_EN_CM_HYST 0x00040000
#define BM_POWER_LOOPCTRL_EN_DF_HYST 0x00020000
#define BM_POWER_LOOPCTRL_CM_HYST_THRESH 0x00010000
#define BM_POWER_LOOPCTRL_DF_HYST_THRESH 0x00008000
#define BM_POWER_LOOPCTRL_RCSCALE_THRESH 0x00004000
#define BP_POWER_LOOPCTRL_EN_RCSCALE      12
#define BM_POWER_LOOPCTRL_EN_RCSCALE 0x00003000
#define BF_POWER_LOOPCTRL_EN_RCSCALE(v)  \
	(((v) << 12) & BM_POWER_LOOPCTRL_EN_RCSCALE)
#define BP_POWER_LOOPCTRL_DC_FF      8
#define BM_POWER_LOOPCTRL_DC_FF 0x00000700
#define BF_POWER_LOOPCTRL_DC_FF(v)  \
	(((v) << 8) & BM_POWER_LOOPCTRL_DC_FF)
#define BP_POWER_LOOPCTRL_DC_R      4
#define BM_POWER_LOOPCTRL_DC_R 0x000000F0
#define BF_POWER_LOOPCTRL_DC_R(v)  \
	(((v) << 4) & BM_POWER_LOOPCTRL_DC_R)
#define BP_POWER_LOOPCTRL_DC_C      0
#define BM_POWER_LOOPCTRL_DC_C 0x00000003
#define BF_POWER_LOOPCTRL_DC_C(v)  \
	(((v) << 0) & BM_POWER_LOOPCTRL_DC_C)
HW_REGISTER_0(HW_POWER_STS, REGS_POWER_BASE, 0x000000c0)
#define HW_POWER_STS_ADDR (REGS_POWER_BASE + 0x000000c0)
#define BP_POWER_STS_PWRUP_SOURCE      24
#define BM_POWER_STS_PWRUP_SOURCE 0x3F000000
#define BF_POWER_STS_PWRUP_SOURCE(v)  \
	(((v) << 24) & BM_POWER_STS_PWRUP_SOURCE)
#define BP_POWER_STS_PSWITCH      20
#define BM_POWER_STS_PSWITCH 0x00300000
#define BF_POWER_STS_PSWITCH(v)  \
	(((v) << 20) & BM_POWER_STS_PSWITCH)
#define BM_POWER_STS_AVALID_STATUS 0x00020000
#define BM_POWER_STS_BVALID_STATUS 0x00010000
#define BM_POWER_STS_VBUSVALID_STATUS 0x00008000
#define BM_POWER_STS_SESSEND_STATUS 0x00004000
#define BM_POWER_STS_BATT_BO 0x00002000
#define BM_POWER_STS_VDD5V_FAULT 0x00001000
#define BM_POWER_STS_CHRGSTS 0x00000800
#define BM_POWER_STS_DCDC_4P2_BO 0x00000400
#define BM_POWER_STS_DC_OK 0x00000200
#define BM_POWER_STS_VDDIO_BO 0x00000100
#define BM_POWER_STS_VDDA_BO 0x00000080
#define BM_POWER_STS_VDDD_BO 0x00000040
#define BM_POWER_STS_VDD5V_GT_VDDIO 0x00000020
#define BM_POWER_STS_VDD5V_DROOP 0x00000010
#define BM_POWER_STS_AVALID 0x00000008
#define BM_POWER_STS_BVALID 0x00000004
#define BM_POWER_STS_VBUSVALID 0x00000002
#define BM_POWER_STS_SESSEND 0x00000001
HW_REGISTER(HW_POWER_SPEED, REGS_POWER_BASE, 0x000000d0)
#define HW_POWER_SPEED_ADDR (REGS_POWER_BASE + 0x000000d0)
#define BP_POWER_SPEED_STATUS      16
#define BM_POWER_SPEED_STATUS 0x00FF0000
#define BF_POWER_SPEED_STATUS(v)  \
	(((v) << 16) & BM_POWER_SPEED_STATUS)
#define BP_POWER_SPEED_CTRL      0
#define BM_POWER_SPEED_CTRL 0x00000003
#define BF_POWER_SPEED_CTRL(v)  \
	(((v) << 0) & BM_POWER_SPEED_CTRL)
HW_REGISTER_0(HW_POWER_BATTMONITOR, REGS_POWER_BASE, 0x000000e0)
#define HW_POWER_BATTMONITOR_ADDR (REGS_POWER_BASE + 0x000000e0)
#define BP_POWER_BATTMONITOR_BATT_VAL      16
#define BM_POWER_BATTMONITOR_BATT_VAL 0x03FF0000
#define BF_POWER_BATTMONITOR_BATT_VAL(v)  \
	(((v) << 16) & BM_POWER_BATTMONITOR_BATT_VAL)
#define BM_POWER_BATTMONITOR_EN_BATADJ 0x00000400
#define BM_POWER_BATTMONITOR_PWDN_BATTBRNOUT 0x00000200
#define BM_POWER_BATTMONITOR_BRWNOUT_PWD 0x00000100
#define BP_POWER_BATTMONITOR_BRWNOUT_LVL      0
#define BM_POWER_BATTMONITOR_BRWNOUT_LVL 0x0000001F
#define BF_POWER_BATTMONITOR_BRWNOUT_LVL(v)  \
	(((v) << 0) & BM_POWER_BATTMONITOR_BRWNOUT_LVL)
HW_REGISTER(HW_POWER_RESET, REGS_POWER_BASE, 0x00000100)
#define HW_POWER_RESET_ADDR (REGS_POWER_BASE + 0x00000100)
#define BP_POWER_RESET_UNLOCK      16
#define BM_POWER_RESET_UNLOCK 0xFFFF0000
#define BF_POWER_RESET_UNLOCK(v) \
	(((v) << 16) & BM_POWER_RESET_UNLOCK)
#define BV_POWER_RESET_UNLOCK__KEY 0x3E77
#define BM_POWER_RESET_PWD_OFF 0x00000002
#define BM_POWER_RESET_PWD 0x00000001
HW_REGISTER(HW_POWER_DEBUG, REGS_POWER_BASE, 0x00000110)
#define HW_POWER_DEBUG_ADDR (REGS_POWER_BASE + 0x00000110)
#define BM_POWER_DEBUG_VBUSVALIDPIOLOCK 0x00000008
#define BM_POWER_DEBUG_AVALIDPIOLOCK 0x00000004
#define BM_POWER_DEBUG_BVALIDPIOLOCK 0x00000002
#define BM_POWER_DEBUG_SESSENDPIOLOCK 0x00000001
HW_REGISTER(HW_POWER_SPECIAL, REGS_POWER_BASE, 0x00000120)
#define HW_POWER_SPECIAL_ADDR (REGS_POWER_BASE + 0x00000120)
#define BP_POWER_SPECIAL_TEST      0
#define BM_POWER_SPECIAL_TEST 0xFFFFFFFF
#define BF_POWER_SPECIAL_TEST(v)   (v)
HW_REGISTER_0(HW_POWER_VERSION, REGS_POWER_BASE, 0x00000130)
#define HW_POWER_VERSION_ADDR (REGS_POWER_BASE + 0x00000130)
#define BP_POWER_VERSION_MAJOR      24
#define BM_POWER_VERSION_MAJOR 0xFF000000
#define BF_POWER_VERSION_MAJOR(v) \
	(((v) << 24) & BM_POWER_VERSION_MAJOR)
#define BP_POWER_VERSION_MINOR      16
#define BM_POWER_VERSION_MINOR 0x00FF0000
#define BF_POWER_VERSION_MINOR(v)  \
	(((v) << 16) & BM_POWER_VERSION_MINOR)
#define BP_POWER_VERSION_STEP      0
#define BM_POWER_VERSION_STEP 0x0000FFFF
#define BF_POWER_VERSION_STEP(v)  \
	(((v) << 0) & BM_POWER_VERSION_STEP)
#endif /* __ARCH_ARM___POWER_H */
