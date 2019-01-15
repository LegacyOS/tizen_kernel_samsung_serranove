/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include <asm/mach/arch.h>
#include <soc/qcom/socinfo.h>
#include <mach/board.h>
#include <mach/msm_memtypes.h>
#include <soc/qcom/rpm-smd.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/err.h>
#ifdef CONFIG_SEC_DEBUG
#include <mach/sec_debug.h>
#endif

#include "board-dt.h"
#include "platsmp.h"

#ifdef CONFIG_PROC_AVC
#include <linux/proc_avc.h>
#endif

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

//MSM8916 GPS
static struct platform_device msm8916_gps = {
	.name = "msm8916_gps",
	.id = -1,
};

static void samsung_sys_class_init(void)
{
	pr_info("samsung sys class init.\n");

	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class)) {
		pr_err("Failed to create class(sec)!\n");
		return;
	}

	pr_info("samsung sys class end.\n");
};

static void __init msm8916_early_memory(void)
{
	of_scan_flat_dt(dt_scan_for_memory_hole, NULL);
}

static void __init msm8916_dt_reserve(void)
{
	of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
}

static void __init msm8916_map_io(void)
{
	msm_map_msm8916_io();
}

static struct of_dev_auxdata msm8916_auxdata_lookup[] __initdata = {
	{}
};

#ifdef CONFIG_SEC_TETRA_PROJECT
#define BASE_GPIO 		902
#define SLIC_LDO_1V5_EN		(BASE_GPIO + 17)
#define SLIC_RST_N	(BASE_GPIO + 53)
#define SLIC_BOOSTER_A_EN (BASE_GPIO + 26)
#define SLIC_BOOSTER_B_EN (BASE_GPIO + 27)

static void  __init dxs_init(void)
{
	int ret;

	ret = gpio_request_one(SLIC_RST_N,
				GPIOF_OUT_INIT_HIGH, "slic_reset_n");
	if (ret)
		pr_err("%s, could not get gpio(SLIC_RST_N)\n", __func__);
	gpio_set_value(SLIC_RST_N, 1);

	ret = gpio_request_one(SLIC_LDO_1V5_EN,
				GPIOF_OUT_INIT_HIGH, "slic_ldo_en");
	if (ret)
		pr_err("%s, could not get gpio(SLIC_LDO_1V5_EN)\n", __func__);
	gpio_set_value(SLIC_LDO_1V5_EN, 1);

	ret = gpio_request_one(SLIC_BOOSTER_A_EN,
				GPIOF_OUT_INIT_HIGH, "slic_ldo_en");
	if (ret)
		pr_err("%s, could not get gpio(SLIC_BOOSTER_A_EN)\n", __func__);
	gpio_set_value(SLIC_BOOSTER_A_EN, 1);

	ret = gpio_request_one(SLIC_BOOSTER_B_EN,
				GPIOF_OUT_INIT_HIGH, "slic_ldo_en");
	if (ret)
		pr_err("%s, could not get gpio(SLIC_BOOSTER_B_EN)\n", __func__);
	gpio_set_value(SLIC_BOOSTER_B_EN, 1);
}
#endif
/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8916_add_drivers(void)
{
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
}

static void __init msm8916_init(void)
{
	struct of_dev_auxdata *adata = msm8916_auxdata_lookup;
#ifdef CONFIG_SEC_DEBUG
	sec_debug_init();
#endif

	/*
	 * populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init.  socinfo_init needs smem support so call
	 * msm_smem_init before it.
	 */
	of_platform_populate(NULL, of_default_bus_match_table, adata, NULL);
	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	samsung_sys_class_init();
	msm8916_add_drivers();

	//MSM8916 GPS
	platform_device_register(&msm8916_gps);
#ifdef CONFIG_SEC_TETRA_PROJECT
	dxs_init();
#endif
}

static const char *msm8916_dt_match[] __initconst = {
	"qcom,msm8916",
	"qcom,apq8016",
	NULL
};

static const char *msm8936_dt_match[] __initconst = {
	"qcom,msm8936",
	NULL
};

static const char *msm8939_dt_match[] __initconst = {
	"qcom,msm8939",
	NULL
};

DT_MACHINE_START(MSM8916_DT,
		"Qualcomm Technologies, Inc. MSM 8916 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8916_dt_match,
	.reserve = msm8916_dt_reserve,
	.init_very_early = msm8916_early_memory,
	.smp = &msm8916_smp_ops,
MACHINE_END

DT_MACHINE_START(MSM8939_DT,
		"Qualcomm Technologies, Inc. MSM 8939 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8939_dt_match,
	.reserve = msm8916_dt_reserve,
	.smp = &msm8936_smp_ops,
MACHINE_END

DT_MACHINE_START(MSM8936_DT,
		"Qualcomm Technologies, Inc. MSM 8936 (Flattened Device Tree)")
	.map_io = msm8916_map_io,
	.init_machine = msm8916_init,
	.dt_compat = msm8936_dt_match,
	.reserve = msm8916_dt_reserve,
	.init_very_early = msm8916_early_memory,
	.smp = &msm8936_smp_ops,
MACHINE_END
