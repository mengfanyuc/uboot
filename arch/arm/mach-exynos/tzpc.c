// SPDX-License-Identifier: GPL-2.0+
/*
 * Lowlevel setup for SMDK5250 board based on S5PC520
 *
 * Copyright (C) 2012 Samsung Electronics
 */

#include <common.h>
#include <asm/arch/tzpc.h>
#include <asm/io.h>

#ifdef CONFIG_TARGET_ITOP4412
#include "common_setup.h"

void tzasc_init(void)
{
	struct exynos4412_tzasc *tzasc;
	unsigned int addr, start = 0, end = 0;
	
	start = samsung_get_base_dmc_tzasc();
	end = start + ((NR_TZASC_BANKS - 1) * DMC_OFFSET);
	
	/* Set sp<n> = b1111 non-secure wr secure wr all allowed */
	for (addr = start; addr <= end; addr += DMC_OFFSET) {
		tzasc = (struct exynos4412_tzasc *)addr;
		writel(RA0_VAL, &tzasc->region_attributes_0);
	}
}
#endif

/* Setting TZPC[TrustZone Protection Controller] */
void tzpc_init(void)
{
	struct exynos_tzpc *tzpc;
	unsigned int addr, start = 0, end = 0;

	start = samsung_get_base_tzpc();

	if (cpu_is_exynos5())
		end = start + ((EXYNOS5_NR_TZPC_BANKS - 1) * TZPC_BASE_OFFSET);
	else if (cpu_is_exynos4())
		end = start + ((EXYNOS4_NR_TZPC_BANKS - 1) * TZPC_BASE_OFFSET);

	for (addr = start; addr <= end; addr += TZPC_BASE_OFFSET) {
		tzpc = (struct exynos_tzpc *)addr;

		if (addr == start)
			writel(R0SIZE, &tzpc->r0size);

		writel(DECPROTXSET, &tzpc->decprot0set);
		writel(DECPROTXSET, &tzpc->decprot1set);

		if (cpu_is_exynos5() && (addr == end))
			break;

		writel(DECPROTXSET, &tzpc->decprot2set);
		writel(DECPROTXSET, &tzpc->decprot3set);
	}
}
