/*
 * Clock Initialization for board based on EXYNOS4412
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

#include <common.h>
#include <config.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clk.h>
#include <asm/arch/clock.h>
#include "common_setup.h"
#include "exynos4412_setup.h"

/*
 * system_clock_init: Initialize core clock and bus clock.
 * void system_clock_init(void)
 */
void system_clock_init(void)
{
	unsigned int set, clr;
	struct exynos4x12_clock *clk = (struct exynos4x12_clock *)
						samsung_get_base_clock();
	
	/* Set Lock Time */
	clr = PLL_LOCKTIME(65535);
	
	/** APLL LOCKTIME 1000 MHz */
	set = PLL_LOCKTIME(PDIV(3) * 270);
	clrsetbits_le32(&clk->apll_lock, clr, set);

	/** MPLL LOCKTIME 800 MHz */
	set = PLL_LOCKTIME(PDIV(3) * 270);
	clrsetbits_le32(&clk->mpll_lock, clr, set);

	/** EPLL LOCKTIME 400 MHz */
	set = PLL_LOCKTIME(PDIV(3) * 270);
	clrsetbits_le32(&clk->epll_lock, clr, set);

	/** VPLL LOCKTIME 100 MHz */
	set = PLL_LOCKTIME(PDIV(3) * 270);
	clrsetbits_le32(&clk->vpll_lock, clr, set);	

	/*
	 * Set CMU_CPU clocks src
	 * Bit values:                 0  ; 1
	 * MUX_APLL_SEL:        FIN_PLL   ; FOUT_APLL			1000 MHz
	 * MUX_CORE_SEL:        MOUT_APLL ; SCLK_MPLL			1000 MHz
	 * MUX_HPM_SEL:         MOUT_APLL ; SCLK_MPLL_USER_C	1000 MHz
	 * MUX_MPLL_USER_SEL_C: FIN_PLL   ; SCLK_MPLL			800 MHz
	*/
	clr = MUX_APLL_SEL(1) | MUX_CORE_SEL(1) |
		  MUX_HPM_SEL(1) | MUX_MPLL_USER_SEL_C(1);
	set = MUX_APLL_SEL(1) | MUX_CORE_SEL(0) | MUX_HPM_SEL(0) |
	      MUX_MPLL_USER_SEL_C(1);

	clrsetbits_le32(&clk->src_cpu, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_cpu) & MUX_STAT_CPU_CHANGING)
		continue;

	/* Set APLL to 1000MHz */
	clr = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(0) | PDIV(3) | MDIV(125) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->apll_con0, clr, set);

	/* Wait for PLL to be locked */
	while (!(readl(&clk->apll_con0) & PLL_LOCKED_BIT))
		continue;

	clr = AFC(31) | LOCK_CON_DLY(31) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(3) |FEED_EN(1)| AFC_ENB(1) |
	      DCC_ENB(1) | BYPASS(1) |RESV0(1) | RESV1(1);
	set = AFC(0) | LOCK_CON_DLY(8) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(0) |FEED_EN(0)| AFC_ENB(0) |
	      DCC_ENB(1) | BYPASS(0) |RESV0(0) | RESV1(0);

	clrsetbits_le32(&clk->apll_con1, clr, set);

	/*
	 * Set dividers for CMU_CPU
	 * coreout =      MOUT / (ratio + 1) = 1000 MHz (0)
	 * corem0 =     armclk / (ratio + 1) = 250 MHz (3)
	 * corem1 =     armclk / (ratio + 1) = 125 MHz (7)
	 * periph =     armclk / (ratio + 1) = 250 MHz (3)
	 * atbout =       MOUT / (ratio + 1) = 200 MHz (4)
	 * pclkdbgout = atbout / (ratio + 1) = 100 MHz (1)
	 * sclkapll = MOUTapll / (ratio + 1) = 500 MHz (1)
	 * core2out = core_out / (ratio + 1) = 1000 MHz (0) (armclk)
	*/
	clr = CORE_RATIO(7) | COREM0_RATIO(7) | COREM1_RATIO(7) |
	      PERIPH_RATIO(7) | ATB_RATIO(7) | PCLK_DBG_RATIO(7) |
	      APLL_RATIO(7) | CORE2_RATIO(7);

	set = CORE_RATIO(0) | COREM0_RATIO(3) | COREM1_RATIO(7) |
	      PERIPH_RATIO(3) | ATB_RATIO(4) | PCLK_DBG_RATIO(1) |
	      APLL_RATIO(1) | CORE2_RATIO(0);

	clrsetbits_le32(&clk->div_cpu0, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_cpu0) & DIV_STAT_CPU0_CHANGING)
		continue;

	/*
	 * doutcopy = MOUThpm / (ratio + 1) = 200 MHz (4)
	 * sclkhpm = doutcopy / (ratio + 1) = 200 MHz (0)
	 * cores_out = armclk / (ratio + 1) = 1000 MHz (0)
	 */
	clr = COPY_RATIO(7) | HPM_RATIO(7) | CORES_RATIO(7);
	set = COPY_RATIO(4) | HPM_RATIO(0) | CORES_RATIO(0);

	clrsetbits_le32(&clk->div_cpu1, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_cpu1) & DIV_STAT_CPU1_CHANGING)
		continue;

	/*
	 * Set CMU_DMC clocks src
	 * Bit values:             0  ; 1
	 * MUX_C2C_SEL:      SCLKMPLL ; SCLKAPLL		800 MHz
	 * MUX_DMC_BUS_SEL:  SCLKMPLL ; SCLKAPLL		800 MHz
	 * MUX_DPHY_SEL:     SCLKMPLL ; SCLKAPLL		800 MHz
	 * MUX_MPLL_SEL:     FINPLL   ; MOUT_MPLL_FOUT	800 MHz
	 * MUX_PWI_SEL:      0110 (MPLL); 0111 (EPLL); 1000 (VPLL); 1 (XusbXTI); 0(XXTI) 24 MHz
	 * MUX_G2D_ACP0_SEL: SCLKMPLL ; SCLKAPLL		800 MHz
	 * MUX_G2D_ACP1_SEL: SCLKEPLL ; SCLKVPLL		800 MHz
	 * MUX_G2D_ACP_SEL:  OUT_ACP0 ; OUT_ACP1		800 MHz
	*/
	clr = MUX_C2C_SEL(1) | MUX_DMC_BUS_SEL(1) |
		  MUX_DPHY_SEL(1) | MUX_MPLL_SEL(1) |
		  MUX_PWI_SEL(15) | MUX_G2D_ACP0_SEL(1) |
		  MUX_G2D_ACP1_SEL(1) | MUX_G2D_ACP_SEL(1);

	set = MUX_C2C_SEL(0) | MUX_DMC_BUS_SEL(0) | MUX_DPHY_SEL(0) |
	      MUX_MPLL_SEL(1) | MUX_PWI_SEL(1) | MUX_G2D_ACP0_SEL(0) |
	      MUX_G2D_ACP1_SEL(0) | MUX_G2D_ACP_SEL(0);

	clrsetbits_le32(&clk->src_dmc, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_dmc) & MUX_STAT_DMC_CHANGING)
		continue;

	/* Set MPLL to 800MHz */
	clr = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(0) | PDIV(3) | MDIV(100) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->mpll_con0, clr, set);

	/* Wait for PLL to be locked */
	while (!(readl(&clk->mpll_con0) & PLL_LOCKED_BIT))
		continue;

	clr = AFC(31) | LOCK_CON_DLY(31) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(3) |FEED_EN(1)| AFC_ENB(1) |
	      DCC_ENB(1) | BYPASS(1) |RESV0(1) | RESV1(1);
	set = AFC(0) | LOCK_CON_DLY(8) | LOCK_CON_IN(3) |
	      LOCK_CON_OUT(0) |FEED_EN(0)| AFC_ENB(0) |
	      DCC_ENB(1) | BYPASS(0) |RESV0(0) | RESV1(0);

	clrsetbits_le32(&clk->mpll_con1, clr, set);

	/*
	 * aclk_acp = MOUTdmc / (ratio + 1) = 200 MHz (3)
	 * pclk_acp = aclk_acp / (ratio + 1) = 100 MHz (1)
	 * sclk_dphy = MOUTdphy / (ratio + 1) = 400 MHz (1)
	 * sclk_dmc = MOUTdmc / (ratio + 1) = 400 MHz (1)
	 * aclk_dmcd = sclk_dmc / (ratio + 1) = 200 MHz (1)
	 * aclk_dmcp = aclk_dmcd / (ratio + 1) = 100 MHz (1)
	 */
	clr = ACP_RATIO(7) | ACP_PCLK_RATIO(7) | DPHY_RATIO(7) |
	      DMC_RATIO(7) | DMCD_RATIO(7) | DMCP_RATIO(7);

	set = ACP_RATIO(3) | ACP_PCLK_RATIO(1) | DPHY_RATIO(1) |
	      DMC_RATIO(1) | DMCD_RATIO(1) | DMCP_RATIO(1);

	clrsetbits_le32(&clk->div_dmc0, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_dmc0) & DIV_STAT_DMC0_CHANGING)
		continue;
	
	/*
	 * sclk_g2d_acp = MOUTg2d / (ratio + 1) = 200 MHz (3)
	 * sclk_c2c = MOUTc2c / (ratio + 1) = 400 MHz (1)
	 * aclk_c2c = sclk_c2c / (ratio + 1) = 200 MHz (1)
	 * sclk_pwi = MOUTpwi / (ratio + 1) = 12 MHz (1)
	 */
	clr = G2D_ACP_RATIO(15) | C2C_RATIO(7) | PWI_RATIO(15) |
	      C2C_ACLK_RATIO(7) | DVSEM_RATIO(127) | DPM_RATIO(127);

	set = G2D_ACP_RATIO(3) | C2C_RATIO(1) | PWI_RATIO(1) |
	      C2C_ACLK_RATIO(1) | DVSEM_RATIO(1) | DPM_RATIO(1);

	clrsetbits_le32(&clk->div_dmc1, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_dmc1) & DIV_STAT_DMC1_CHANGING)
		continue;

	/*
	 * src values: 0(XXTI); 1(XusbXTI); 2(SCLK_HDMI24M); 3(SCLK_USBPHY0);
	 *             5(SCLK_HDMIPHY); 6(SCLK_MPLL_USER_T); 7(SCLK_EPLL);
	 *             8(SCLK_VPLL)
	 *
	 * Set all to SCLK_MPLL_USER_T	800 MHz
	 */
	clr = UART0_SEL(15) | UART1_SEL(15) | UART2_SEL(15) |
	      UART3_SEL(15) | UART4_SEL(15);

	set = UART0_SEL(6) | UART1_SEL(6) | UART2_SEL(6) | UART3_SEL(6) |
	      UART4_SEL(6);

	clrsetbits_le32(&clk->src_peril0, clr, set);

	/*
	 * SCLK_UARTx = MOUTuartX / (ratio + 1) = 100 MHz (7)
	*/
	clr = UART0_RATIO(15) | UART1_RATIO(15) | UART2_RATIO(15) |
	      UART3_RATIO(15) | UART4_RATIO(15);

	set = UART0_RATIO(7) | UART1_RATIO(7) | UART2_RATIO(7) |
	      UART3_RATIO(7) | UART4_RATIO(7);

	clrsetbits_le32(&clk->div_peril0, clr, set);

	while (readl(&clk->div_stat_peril0) & DIV_STAT_PERIL0_CHANGING)
		continue;

	/* 
	 * src values: 0(XXTI); 1(XusbXTI); 2(SCLK_HDMI24M); 3(SCLK_USBPHY0);
	 *             5(SCLK_HDMIPHY); 6(SCLK_MPLL_USER_T); 7(SCLK_EPLL);
	 *             8(SCLK_VPLL)
	 *
	 * Set all to SCLK_MPLL_USER_T	800 MHz
	 */
	clr = MMC1_SEL(15) | MMC2_SEL(15) | MMC3_SEL(15) |
	      MMC4_SEL(15) | MIPIHSI_SEL(1);
	set = MMC1_SEL(6) | MMC2_SEL(6) | MMC3_SEL(6) |
	      MMC4_SEL(6) | MIPIHSI_SEL(0);

	clrsetbits_le32(&clk->src_fsys, clr, set);

	/*
	 * SCLK_MIPIHSI = MOUTMIPIHSI / (MIPIHSI_RATIO + 1) = 200 MHz (3)
	*/
	clr = MIPIHSI_RATIO(15);
	set = MIPIHSI_RATIO(3);

	clrsetbits_le32(&clk->div_fsys0, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys0) & DIV_STAT_FSYS0_CHANGING)
		continue;
	
	/*
	 * DOUTmmc1 = MOUTmmc1 / (ratio + 1) = 50 MHz (15)
	 * sclk_mmc1 = DOUTmmc1 / (ratio + 1) = 50 MHz (0)
	 * DOUTmmc0 = MOUTmmc0 / (ratio + 1) = 50 MHz (15)
	 * sclk_mmc0 = DOUTmmc0 / (ratio + 1) = 50 MHz (0)
	*/
	clr = MMC0_RATIO(15) | MMC0_PRE_RATIO(255) | MMC1_RATIO(15) |
	      MMC1_PRE_RATIO(255);

	set = MMC0_RATIO(15) | MMC0_PRE_RATIO(0) | MMC1_RATIO(15) |
	      MMC1_PRE_RATIO(0);

	clrsetbits_le32(&clk->div_fsys1, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys1) & DIV_STAT_FSYS1_CHANGING)
		continue;

	/*
	 * DOUTmmc3 = MOUTmmc3 / (ratio + 1) = 50 MHz (15)
	 * sclk_mmc3 = DOUTmmc3 / (ratio + 1) = 50 MHz (0)
	 * DOUTmmc2 = MOUTmmc2 / (ratio + 1) = 50 MHz (15)
	 * sclk_mmc2 = DOUTmmc2 / (ratio + 1) = 50 MHz (0)
	*/
	clr = MMC2_RATIO(15) | MMC2_PRE_RATIO(255) | MMC3_RATIO(15) |
	      MMC3_PRE_RATIO(255);

	set = MMC2_RATIO(15) | MMC2_PRE_RATIO(0) | MMC3_RATIO(15) |
	      MMC3_PRE_RATIO(0);

	clrsetbits_le32(&clk->div_fsys2, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys2) & DIV_STAT_FSYS2_CHANGING)
		continue;

	/*
	 * DOUTmmc4 = MOUTmmc4 / (ratio + 1) = 50 MHz (15)
	 * sclk_mmc4 = DOUTmmc4 / (ratio + 1) = 50 MHz (0)
	*/
	clr = MMC4_RATIO(15) | MMC4_PRE_RATIO(255);
	set = MMC4_RATIO(7) | MMC4_PRE_RATIO(0);

	clrsetbits_le32(&clk->div_fsys3, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_fsys3) & DIV_STAT_FSYS3_CHANGING)
		continue;

	/*
	 * Set CMU_CPU clocks src
	 * Bit values:                           0 ; 1
	 * MUX_ONENAND_1_SEL           MOUTONENAND ; SCLKVPLL	133 MHz
	 * MUX_EPLL_SEL                     FINPLL ; FOUTEPLL	400 MHz
	 * MUX_VPLL_SEL                     FINPLL ; FOUTEPLL	400 MHz
	 * MUX_ACLK_200_SEL               SCLKMPLL ; SCLKAPLL	800 MHz
	 * MUX_ACLK_100_SEL               SCLKMPLL ; SCLKAPLL	800 MHz
	 * MUX_ACLK_160_SEL               SCLKMPLL ; SCLKAPLL	800 MHz
	 * MUX_ACLK_133_SEL               SCLKMPLL ; SCLKAPLL	800 MHz
	 * MUX_ONENAND_SEL                ACLK_133 ; ACLK_160	133 MHz
	*/
	clr = MUX_ONENAND_1_SEL(1) | MUX_EPLL_SEL(1) | MUX_VPLL_SEL(1) |
	      MUX_ACLK_200_SEL(1) | MUX_ACLK_100_SEL(1) | MUX_ACLK_160_SEL(1) |
	      MUX_ACLK_133_SEL(1) | MUX_ONENAND_SEL(1);

	set = MUX_ONENAND_1_SEL(0) | MUX_EPLL_SEL(1) | MUX_VPLL_SEL(1) |
	      MUX_ACLK_200_SEL(0) | MUX_ACLK_100_SEL(0) | MUX_ACLK_160_SEL(0) |
	      MUX_ACLK_133_SEL(0) | MUX_ONENAND_SEL(0);

	clrsetbits_le32(&clk->src_top0, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_top0) & MUX_STAT_TOP0_CHANGING)
		continue;

	/*
	 * Bit values:                           0 ; 1
	 * MUX_ACLK_266_GPS_SEL    SCLKMPLL_USER_T ; SCLKAPLL				1000 MHz
	 * MUX_ACLK_400_MCUISP_SEL SCLKMPLL_USER_T ; SCLKAPLL				1000 MHz
	 * MUX_MPLL_USER_SEL_T              FINPLL ; SCLKMPLLL				800 MHz
	 * MUX_ACLK_266_GPS_SUB_SEL         FINPLL ; DIVOUT_ACLK_266_GPS	266 MHz
	 * MUX_ACLK_200_SUB_SEL             FINPLL ; DIVOUT_ACLK_200		200 MHz
	 * MUX_ACLK_400_MCUISP_SUB_SEL      FINPLL ; DIVOUT_ACLK_400_MCUISP	400 MHz
	*/
	clr = MUX_ACLK_266_GPS_SEL(1) | MUX_ACLK_400_MCUISP_SEL(1) |
	      MUX_MPLL_USER_SEL_T(1) | MUX_ACLK_266_GPS_SUB_SEL(1) |
	      MUX_ACLK_200_SUB_SEL(1) | MUX_ACLK_400_MCUISP_SUB_SEL(1);
	set = MUX_ACLK_266_GPS_SEL(0) | MUX_ACLK_400_MCUISP_SEL(0) |
	      MUX_MPLL_USER_SEL_T(1) | MUX_ACLK_266_GPS_SUB_SEL(1) |
	      MUX_ACLK_200_SUB_SEL(1) | MUX_ACLK_400_MCUISP_SUB_SEL(1);

	clrsetbits_le32(&clk->src_top1, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_top1) & MUX_STAT_TOP1_CHANGING)
		continue;

	/* Set EPLL to 400 MHz */
	clr = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(1) | PDIV(3) | MDIV(100) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->epll_con0, clr, set);

	/* Wait for PLL to be locked */
	while (!(readl(&clk->epll_con0) & PLL_LOCKED_BIT))
		continue;

	clr = K(65535) | MFR(255) | MRR(31) | SEL_PF(3);
	set = K(0) | MFR(1) | MRR(6) | SEL_PF(3);

	clrsetbits_le32(&clk->epll_con1, clr, set);

	clr = ICP_BOOST(7) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(1) | EV_AFC_ENB(1) | EV_DCC_ENB(1) | EXTAFC(1);
	set = ICP_BOOST(0) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(0) | EV_AFC_ENB(0) | EV_DCC_ENB(1) | EXTAFC(0);

	clrsetbits_le32(&clk->epll_con2, clr, set);

	/* Set VPLL to 100 MHz */
	clr = SDIV(7) | PDIV(63) | MDIV(1023) | FSEL(1) | PLL_ENABLE(1);
	set = SDIV(3) | PDIV(3) | MDIV(100) | FSEL(0) | PLL_ENABLE(1);

	clrsetbits_le32(&clk->vpll_con0, clr, set);

	/* Wait for PLL to be locked */
	while (!(readl(&clk->vpll_con0) & PLL_LOCKED_BIT))
		continue;

	clr = K(65535) | MFR(255) | MRR(31) | SEL_PF(3);
	set = K(0) | MFR(1) | MRR(6) | SEL_PF(3);

	clrsetbits_le32(&clk->vpll_con1, clr, set);

	clr = ICP_BOOST(7) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(1) | EV_AFC_ENB(1) | EV_DCC_ENB(1) | EXTAFC(1);
	set = ICP_BOOST(0) | EV_FSEL(1) | FVCO_EN(1) | EV_BYPASS(1) |
	      SSCG_EN(0) | EV_AFC_ENB(0) | EV_DCC_ENB(1) | EXTAFC(0);

	clrsetbits_le32(&clk->vpll_con2, clr, set);

	/*
	 * ACLK_200        = MOUTACLK_200        / (ACLK_200_RATIO + 1)        = 200 MHz (3)
	 * ACLK_100        = MOUTACLK_100        / (ACLK_100_RATIO + 1)        = 100 MHz (7)
	 * ACLK_160        = MOUTACLK_160        / (ACLK_160_RATIO + 1)        = 160 MHz (4)
	 * ACLK_133        = MOUTACLK_133        / (ACLK_133_RATIO + 1)        = 133 MHz (5)
	 * ONENAND         = MOUTONENAND_1       / (ONENAND_RATIO + 1)         = 133 MHz (0)
	 * ACLK_266_GPS    = MOUTACLK_266_GPS    / (ACLK_266_GPS_RATIO + 1)    = 266 MHz (2)
	 * ACLK_400_MCUISP = MOUTACLK_400_MCUISP / (ACLK_400_MCUISP_RATIO + 1) = 400 MHz (1)
	 */
	clr = ACLK_200_RATIO(7) | ACLK_100_RATIO(15) | ACLK_160_RATIO(7) | 
	      ACLK_133_RATIO(7) | ONENAND_RATIO(7) | ACLK_266_GPS_RATIO(7) | ACLK_400_MCUISP_RATIO(7);
	set = ACLK_200_RATIO(3) | ACLK_100_RATIO(7) | ACLK_160_RATIO(4) |
	      ACLK_133_RATIO(5) | ONENAND_RATIO(0) | ACLK_266_GPS_RATIO(2) | ACLK_400_MCUISP_RATIO(1);

	clrsetbits_le32(&clk->div_top, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_top) & DIV_STAT_TOP_CHANGING)
		continue;

	/*
	 * Bit values:                 0 ; 1
	 * MUX_GDL_SEL    		SCLKMPLL ; SCLKMPLL		800 MHz
	 * MUX_MPLL_USER_SEL_L 	FINPLL   ; FOUTMPLL		800 MHz
	*/
	clr = MUX_GDL_SEL(1) | MUX_MPLL_USER_SEL_L(1);
	set = MUX_GDL_SEL(0) | MUX_MPLL_USER_SEL_L(1);

	clrsetbits_le32(&clk->src_leftbus, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_leftbus) & MUX_STAT_LEFTBUS_CHANGING)
		continue;

	/*
	 * Bit values:                 0 ; 1
	 * MUX_GDR_SEL    		SCLKMPLL ; SCLKMPLL		800 MHz
	 * MUX_MPLL_USER_SEL_R 	FINPLL   ; FOUTMPLL		800 MHz
	*/
	clr = MUX_GDR_SEL(1) | MUX_MPLL_USER_SEL_R(1);
	set = MUX_GDR_SEL(0) | MUX_MPLL_USER_SEL_R(1);

	clrsetbits_le32(&clk->src_rightbus, clr, set);

	/* Wait for mux change */
	while (readl(&clk->mux_stat_rightbus) & MUX_STAT_RIGHTBUS_CHANGING)
		continue;

	/*
	 * ACLK_GDL = MOUTGDL / (GDL_RATIO + 1) = 200 MHz (3)
	 * ACLK_GPL = MOUTGPL / (GPL_RATIO + 1) = 100 MHz (1)
	*/
	clr = GDL_RATIO(7) | GPL_RATIO(7);
	set = GDL_RATIO(3) | GPL_RATIO(1);

	clrsetbits_le32(&clk->div_leftbus, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_leftbus) & DIV_STAT_LEFTBUS_CHANGING)
		continue;

	/**
	 * ACLK_GDR = MOUTGDR / (GDR_RATIO + 1) = 200 MHz (3)
	 * ACLK_GPR = MOUTGPR / (GPR_RATIO + 1) = 100 MHz (1)
	*/
	clr = GPR_RATIO(7) | GDR_RATIO(7);
	set = GPR_RATIO(1) | GDR_RATIO(3);

	clrsetbits_le32(&clk->div_rightbus, clr, set);

	/* Wait for divider ready status */
	while (readl(&clk->div_stat_rightbus) & DIV_STAT_RIGHTBUS_CHANGING)
		continue;	
}

/*
 * Set clock divisor value for booting from EMMC.
 * Set DWMMC channel-0 clk div to operate mmc0 device at 20MHz.
 */
void emmc_boot_clk_div_set(void)
{	
	/*
	 * For MOUTmmc0-3 = 800 MHz (MPLL)
	 *
	 * DOUTmmc1 = MOUTmmc1 / (ratio + 1) = 100 MHz (7)
	 * sclk_mmc1 = DOUTmmc1 / (ratio + 1) = 20 MHz (4)
	 * DOUTmmc0 = MOUTmmc0 / (ratio + 1) = 100 MHz (7)
	 * sclk_mmc0 = DOUTmmc0 / (ratio + 1) = 20 MHz (4)
	*/

	struct exynos4x12_clock *clk =
		(struct exynos4x12_clock *)samsung_get_base_clock();
	unsigned int div_mmc = 0;

	div_mmc |= MMC0_RATIO(7) | MMC0_PRE_RATIO(4) | MMC1_RATIO(7) | MMC1_PRE_RATIO(4);
	writel(div_mmc, (unsigned int) &clk->div_fsys1);
}