/*
 * arch/arm/mach-tegra/board-dt-tegra148.c
 *
 * NVIDIA Tegra148 device tree board support
 *
 * Copyright (C) 2012 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/of.h>

#include <asm/mach/arch.h>
#include <asm/hardware/gic.h>
#include <mach/iomap.h>

#include "board.h"
#include "clock.h"
#include "common.h"

static void __init tegra148_dt_init(void)
{
}

static const char * const tegra148_dt_board_compat[] = {
	"nvidia,tegra148",
	NULL
};

DT_MACHINE_START(TEGRA148_DT, "NVIDIA Tegra148 (Flattened Device Tree)")
	.soc		= &tegra_soc_desc,
	.map_io		= tegra_map_common_io,
	.init_early	= tegra14x_init_early,
	.init_irq	= tegra_dt_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra148_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= tegra148_dt_board_compat,
MACHINE_END
