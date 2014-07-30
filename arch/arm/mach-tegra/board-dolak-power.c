/*
 * arch/arm/mach-tegra/board-dolak-power.c
 *
 * Copyright (C) 2011-2012 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/gpio-regulator.h>

#include <mach/gpio-tegra.h>
#include <mach/iomap.h>
#include <mach/irqs.h>

#include "pm.h"
#include "board.h"
#include "gpio-names.h"

static int ac_online(void)
{
	return 1;
}

static struct regulator_consumer_supply gpio_reg_sdmmc3_vdd_sel_supply[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
};

static struct gpio_regulator_state gpio_reg_sdmmc3_vdd_sel_states[] = {
	{
		.gpios = 0,
		.value = 1800000,
	},
	{
		.gpios = 1,
		.value = 3300000,
	},
};

static struct gpio gpio_reg_sdmmc3_vdd_sel_gpios[] = {
	{
		.gpio = TEGRA_GPIO_PO4,
		.flags = 0,
		.label = "sdmmc3_vdd_sel",
	},
};

#define GPIO_REG(_id, _name, _input_supply, _active_high,	\
		_boot_state, _delay_us, _minmv, _maxmv)		\
	static struct regulator_init_data ri_data_##_name =	\
{								\
	.supply_regulator = NULL,				\
	.num_consumer_supplies =				\
		ARRAY_SIZE(gpio_reg_##_name##_supply),		\
	.consumer_supplies = gpio_reg_##_name##_supply,		\
	.constraints = {					\
		.name = "gpio_reg_"#_name,			\
		.min_uV = (_minmv)*1000,			\
		.max_uV = (_maxmv)*1000,			\
		.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
				REGULATOR_MODE_STANDBY),	\
		.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
				REGULATOR_CHANGE_STATUS |	\
				REGULATOR_CHANGE_VOLTAGE),	\
	},							\
};								\
static struct gpio_regulator_config gpio_reg_##_name##_pdata =	\
{								\
	.supply_name = "vddio_sdmmc",				\
	.enable_gpio = -EINVAL,					\
	.enable_high = _active_high,				\
	.enabled_at_boot = _boot_state,				\
	.startup_delay = _delay_us,				\
	.gpios = gpio_reg_##_name##_gpios,			\
	.nr_gpios = ARRAY_SIZE(gpio_reg_##_name##_gpios),	\
	.states = gpio_reg_##_name##_states,			\
	.nr_states = ARRAY_SIZE(gpio_reg_##_name##_states),	\
	.type = REGULATOR_VOLTAGE,				\
	.init_data = &ri_data_##_name,				\
};								\
static struct platform_device gpio_reg_##_name##_dev = {	\
	.name   = "gpio-regulator",				\
	.id = _id,						\
	.dev    = {						\
		.platform_data = &gpio_reg_##_name##_pdata,	\
	},							\
}

GPIO_REG(4, sdmmc3_vdd_sel, NULL, true, true, 0, 1000, 3300);

#define ADD_GPIO_REG(_name) (&gpio_reg_##_name##_dev)
static struct platform_device *gpio_regs_devices[] = {
	ADD_GPIO_REG(sdmmc3_vdd_sel),
};

static struct resource dolak_pda_resources[] = {
	[0] = {
		.name	= "ac",
	},
};

static struct pda_power_pdata dolak_pda_data = {
	.is_ac_online	= ac_online,
};

static struct platform_device dolak_pda_power_device = {
	.name		= "pda-power",
	.id		= -1,
	.resource	= dolak_pda_resources,
	.num_resources	= ARRAY_SIZE(dolak_pda_resources),
	.dev	= {
		.platform_data	= &dolak_pda_data,
	},
};

static struct tegra_suspend_platform_data dolak_suspend_data = {
	.cpu_timer	= 300,
	.cpu_off_timer	= 300,
	.suspend_mode	= TEGRA_SUSPEND_LP1,
	.core_timer	= 0x157e,
	.core_off_timer = 2000,
	.corereq_high	= false,
	.sysclkreq_high	= true,
};

int __init dolak_regulator_init(void)
{
	platform_device_register(&dolak_pda_power_device);
	return platform_add_devices(gpio_regs_devices,
		ARRAY_SIZE(gpio_regs_devices));
}

int __init dolak_suspend_init(void)
{
	tegra_init_suspend(&dolak_suspend_data);
	return 0;
}
