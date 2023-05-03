/*
 * Memory setup for board based on EXYNOS4412
 *
 * Copyright (C) 2013 Samsung Electronics
 * Rajeshwari Shinde <rajeshwari.s@samsung.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <asm/arch/dmc.h>
#include "common_setup.h"
#include "exynos4412_setup.h"

struct mem_timings mem = {
	.direct_cmd_msr = {
		DIRECT_CMD1, DIRECT_CMD2, DIRECT_CMD3, DIRECT_CMD4
	},
	.timingref = TIMINGREF_VAL,
	.timingrow = TIMINGROW_VAL,
	.timingdata = TIMINGDATA_VAL,
	.timingpower = TIMINGPOWER_VAL,
	.zqcontrol = ZQ_CONTROL_VAL,
	.control0 = CONTROL0_VAL,
	.control1 = CONTROL1_VAL,
	.control2 = CONTROL2_VAL,
	.concontrol = CONCONTROL_VAL,
	.ivcontrol = IVCONTROL_VAL,
	.prechconfig = PRECHCONFIG,
	.memcontrol = MEMCONTROL_VAL,
	.memconfig0 = MEMCONFIG0_VAL,
	.memconfig1 = MEMCONFIG1_VAL,
	.dll_resync = FORCE_DLL_RESYNC,
	.dll_on = DLL_CONTROL_ON,
};
static void phy_control_reset(int ctrl_no, struct exynos4_dmc *dmc)
{
	if (ctrl_no) {
		writel((mem.control1 | (1 << mem.dll_resync)),
		       &dmc->phycontrol1);
		writel((mem.control1 | (0 << mem.dll_resync)),
		       &dmc->phycontrol1);
	} else {
		writel((mem.control0 | (0 << mem.dll_on)),
		       &dmc->phycontrol0);
		writel((mem.control0 | (1 << mem.dll_on)),
		       &dmc->phycontrol0);
	}
}

static void dmc_config_mrs(struct exynos4_dmc *dmc, int chip)
{
	int i;
	unsigned long mask = 0;

	if (chip)
		mask = DIRECT_CMD_CHIP1_SHIFT;

	for (i = 0; i < MEM_TIMINGS_MSR_COUNT; i++) {
		writel(mem.direct_cmd_msr[i] | mask,
		       &dmc->directcmd);
	}
}

static void exynos4412_dmc_init(void)
{
	struct exynos4_dmc *dmc1, *dmc2;
	
	/* DREX0 */
	dmc1 = (struct exynos4_dmc *)samsung_get_base_dmc_ctrl();
	/* DREX1 */
	dmc2 = (struct exynos4_dmc *)(samsung_get_base_dmc_ctrl()
					+ DMC_OFFSET);
	/*
	 * DLL Parameter Setting:
	 * Termination: Enable R/W
	 * Phase Delay for DQS Cleaning: 180' Shift
	 */
	writel(mem.control1, &dmc1->phycontrol1);
	writel(mem.control1, &dmc2->phycontrol1);

	/*
	 * ZQ Calibration
	 * Termination: Disable
	 * Auto Calibration Start: Enable
	 */
	writel(mem.zqcontrol, &dmc1->phyzqcontrol);
	writel(mem.zqcontrol, &dmc2->phyzqcontrol);
	sdelay(0x100000);

	/*
	 * Update DLL Information:
	 * Force DLL Resyncronization
	 */
	phy_control_reset(1, dmc1);
	phy_control_reset(0, dmc1);
	phy_control_reset(1, dmc2);
	phy_control_reset(0, dmc2);

	/* Set DLL Parameters */
	writel(mem.control1, &dmc1->phycontrol1);
	writel(mem.control1, &dmc2->phycontrol1);

	/* DLL Start */
	writel((mem.control0 | CTRL_START | CTRL_DLL_ON), &dmc1->phycontrol0);
	writel((mem.control0 | CTRL_START | CTRL_DLL_ON), &dmc2->phycontrol0);

	writel(mem.control2, &dmc1->phycontrol2);
	writel(mem.control2, &dmc2->phycontrol2);

	/* Set Clock Ratio of Bus clock to Memory Clock */
	writel(mem.concontrol, &dmc1->concontrol);
	writel(mem.concontrol, &dmc2->concontrol);

	/*
	 * Memor Burst length: 8
	 * Number of chips: 1
	 * Memory Bus width: 32 bit
	 * Memory Type: DDR3
	 * Additional Latancy for PLL: 1 Cycle
	 */
	writel(mem.memcontrol, &dmc1->memcontrol);
	writel(mem.memcontrol, &dmc2->memcontrol);

	writel(mem.memconfig0, &dmc1->memconfig0);
	writel(mem.memconfig1, &dmc2->memconfig0);

	/* Config Memory Channel Interleaving */
	writel(mem.ivcontrol, &dmc1->ivcontrol);
	writel(mem.ivcontrol, &dmc2->ivcontrol);

	/* Config Precharge Policy */
	writel(mem.prechconfig, &dmc1->prechconfig);
	writel(mem.prechconfig, &dmc2->prechconfig);

	/*
	 * TimingAref, TimingRow, TimingData, TimingPower Setting:
	 * Values as per Memory AC Parameters
	 */
	writel(mem.timingref, &dmc1->timingref);
	writel(mem.timingrow, &dmc1->timingrow);
	writel(mem.timingdata, &dmc1->timingdata);
	writel(mem.timingpower, &dmc1->timingpower);

	writel(mem.timingref, &dmc2->timingref);
	writel(mem.timingrow, &dmc2->timingrow);
	writel(mem.timingdata, &dmc2->timingdata);
	writel(mem.timingpower, &dmc2->timingpower);

	/* Chip0: NOP Command: Assert and Hold CKE to high level */
	writel(DIRECT_CMD_NOP, &dmc1->directcmd);
	writel(DIRECT_CMD_NOP, &dmc2->directcmd);
	sdelay(0x100000);

	/* Chip0: EMRS2, EMRS3, EMRS, MRS Commands Using Direct Command */
	dmc_config_mrs(dmc1, 0);
	dmc_config_mrs(dmc2, 0);
	sdelay(0x100000);

	/* Chip0: ZQINIT */
	writel(DIRECT_CMD_ZQ, &dmc1->directcmd);
	writel(DIRECT_CMD_ZQ, &dmc2->directcmd);
	sdelay(0x100000);

	phy_control_reset(1, dmc1);
	phy_control_reset(1, dmc2);
	sdelay(0x100000);

	/* turn on DREX0, DREX1 */
	writel((mem.concontrol | AREF_EN), &dmc1->concontrol);
	writel((mem.concontrol | AREF_EN), &dmc2->concontrol);
}

void mem_ctrl_init(int reset)
{
	exynos4412_dmc_init();
}