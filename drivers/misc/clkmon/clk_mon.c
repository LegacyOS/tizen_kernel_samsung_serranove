/*
 * driver/misc/clkmon/clk_mon.c
 *
 * Copyright (C) 2013 Samsung Electronics co. ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/clk_mon.h>
#if defined(CONFIG_ARCH_MSM)
#include "clk_mon_msm.h"
#endif
#include "clk_mon_ioctl.h"
/* 8x26 is based on regulator framework and not Domain based framework
 * hence, we need to get the status of the Parent regulators to check if
 * it is ON/OFF
*/
#include <linux/regulator/consumer.h>

#define SIZE_REG				0x4
/* Bit 31 of each clock register indicates whether the Clock is ON/OFF */
#define CLK_STATUS_BIT_POS			31
#define CHECK_BIT_SET(var, pos)		((var) & (1<<(pos)))
#define BIT_ZERO				0x0
#define BIT_ONE					0x1

/* 8x26 is based on regulator concept and hence we get the list
 * of regulators in the system. They are source of supply for
 * the consumers that are under them.
*/

struct power_domain_mask power_domain_masks[] = {
	{"8916_l1"}, {"8916_l2"}, {"8916_l3"},
	{"8916_l3_corner_ao"}, {"8916_l3_corner_so"},
	{"8916_l4"}, {"8916_l5"}, {"8916_l6"},
	{"8916_l7"}, {"8916_l7_ao"}, {"8916_l7_so"},
	{"8916_l8"}, {"8916_l9"}, {"8916_l10"}, {"8916_l11"},
	{"8916_l12"}, {"8916_l13"}, {"8916_l14"}, {"8916_l15"},
	{"8916_l16"}, {"8916_l17"}, {"8916_l18"},
	{"8916_s1_corner"}, {"8916_s1_corner_ao"},
	{"8916_s1_floor_corner"}, {"8916_s2"}, {"8916_s3"},
	{"8916_s4"}, {"ZW_PMIC_3.3V"}, {"apc_corner"},
	{"gdsc_jpeg"}, {"gdsc_mdss"}, {"gdsc_oxili_gx"},
	{"gdsc_venus"}, {"gdsc_vfe"}, {"mem_acc_corner"},
	/* These are from the Maxim IC */
	{"vhrm_led_3.3v"}, {"vdd_mot_2.7v"}
};

#define CLK_MON_GCC_GPLL0_MODE	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x21000)
#define CLK_MON_GCC_GPLL0_L_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x21004)
#define CLK_MON_GCC_GPLL0_M_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x21008)
#define CLK_MON_GCC_GPLL0_N_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2100C)
#define CLK_MON_GCC_GPLL0_USER_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x21010)
#define CLK_MON_GCC_GPLL0_CONFIG_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x21014)
#define CLK_MON_GCC_GPLL0_STATUS	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2101C)
#define CLK_MON_GCC_GPLL1_MODE	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x20000)
#define CLK_MON_GCC_GPLL1_L_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x20004)
#define CLK_MON_GCC_GPLL1_M_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x20008)
#define CLK_MON_GCC_GPLL1_N_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2000C)
#define CLK_MON_GCC_GPLL1_USER_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x20010)
#define CLK_MON_GCC_GPLL1_CONFIG_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x20014)
#define CLK_MON_GCC_GPLL1_STATUS	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2001C)
#define CLK_MON_GCC_GPLL2_MODE	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4A000)
#define CLK_MON_GCC_GPLL2_L_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4A004)
#define CLK_MON_GCC_GPLL2_M_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4A008)
#define CLK_MON_GCC_GPLL2_N_VAL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4A00C)
#define CLK_MON_GCC_GPLL2_USER_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4A010)
#define CLK_MON_GCC_GPLL2_CONFIG_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4A014)
#define CLK_MON_GCC_GPLL2_STATUS	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4A01C)
#define CLK_MON_GCC_MSS_CFG_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x49000)
#define CLK_MON_GCC_MSS_Q6_BIMC_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x49004)
#define CLK_MON_GCC_USB_HS_BCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x41000)
#define CLK_MON_GCC_USB_HS_SYSTEM_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x41004)
#define CLK_MON_GCC_USB_HS_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x41008)
#define CLK_MON_GCC_USB_HS_SYSTEM_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x41010)
#define CLK_MON_GCC_USB2A_PHY_SLEEP_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4102C)
#define CLK_MON_GCC_SDCC1_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x42004)
#define CLK_MON_GCC_SDCC1_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x42018)
#define CLK_MON_GCC_SDCC1_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4201C)
#define CLK_MON_GCC_SDCC2_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x43004)
#define CLK_MON_GCC_SDCC2_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x43018)
#define CLK_MON_GCC_SDCC2_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4301C)
#define CLK_MON_GCC_BLSP1_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x01008)
#define CLK_MON_GCC_BLSP1_QUP1_SPI_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x02004)
#define CLK_MON_GCC_BLSP1_QUP1_I2C_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x02008)
#define CLK_MON_GCC_BLSP1_QUP1_I2C_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0200C)
#define CLK_MON_GCC_BLSP1_QUP2_I2C_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x03000)
#define CLK_MON_GCC_BLSP1_QUP3_I2C_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x04000)
#define CLK_MON_GCC_BLSP1_QUP4_I2C_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x05000)
#define CLK_MON_GCC_BLSP1_QUP5_I2C_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x06000)
#define CLK_MON_GCC_BLSP1_QUP6_I2C_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x07000)
#define CLK_MON_GCC_BLSP1_QUP1_SPI_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x02024)
#define CLK_MON_GCC_BLSP1_UART1_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0203C)
#define CLK_MON_GCC_BLSP1_UART1_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x02044)
#define CLK_MON_GCC_BLSP1_QUP2_SPI_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0300C)
#define CLK_MON_GCC_BLSP1_QUP2_I2C_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x03010)
#define CLK_MON_GCC_BLSP1_QUP2_SPI_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x03014)
#define CLK_MON_GCC_BLSP1_UART2_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0302C)
#define CLK_MON_GCC_BLSP1_UART2_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x03034)
#define CLK_MON_GCC_BLSP1_QUP3_SPI_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0401C)
#define CLK_MON_GCC_BLSP1_QUP3_I2C_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x04020)
#define CLK_MON_GCC_BLSP1_QUP3_SPI_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x04024)
#define CLK_MON_GCC_BLSP1_QUP4_SPI_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0501C)
#define CLK_MON_GCC_BLSP1_QUP4_I2C_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x05020)
#define CLK_MON_GCC_BLSP1_QUP4_SPI_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x05024)
#define CLK_MON_GCC_BLSP1_QUP5_SPI_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0601C)
#define CLK_MON_GCC_BLSP1_QUP5_I2C_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x06020)
#define CLK_MON_GCC_BLSP1_QUP5_SPI_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x06024)
#define CLK_MON_GCC_BLSP1_QUP6_SPI_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0701C)
#define CLK_MON_GCC_BLSP1_QUP6_I2C_APPS_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x07020)
#define CLK_MON_GCC_BLSP1_QUP6_SPI_APPS_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x07024)
#define CLK_MON_GCC_PDM_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x44004)
#define CLK_MON_GCC_PDM2_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4400C)
#define CLK_MON_GCC_PDM2_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x44010)
#define CLK_MON_GCC_PRNG_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x13004)
#define CLK_MON_GCC_BOOT_ROM_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x1300C)
#define CLK_MON_GCC_CRYPTO_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x16004)
#define CLK_MON_GCC_CRYPTO_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x1601C)
#define CLK_MON_GCC_CRYPTO_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x16020)
#define CLK_MON_GCC_CRYPTO_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x16024)
#define CLK_MON_GCC_GCC_XO_DIV4_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x30034)
#define CLK_MON_GCC_GFX_TBU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12010)
#define CLK_MON_GCC_VENUS_TBU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12014)
#define CLK_MON_GCC_MDP_TBU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x1201C)
#define CLK_MON_GCC_APSS_TCU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12018)
#define CLK_MON_GCC_GFX_TCU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12020)
#define CLK_MON_GCC_MSS_TBU_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12024)
#define CLK_MON_GCC_MSS_TBU_GSS_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12028)
#define CLK_MON_GCC_MSS_TBU_Q6_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x1202C)
#define CLK_MON_GCC_JPEG_TBU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12034)
#define CLK_MON_GCC_SMMU_CFG_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12038)
#define CLK_MON_GCC_VFE_TBU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x1203C)
#define CLK_MON_GCC_GTCU_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12044)
#define CLK_MON_GCC_GTCU_AHB_BRIDGE_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x12094)
#define CLK_MON_GCC_APCS_GPLL_ENA_VOTE	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x45000)
#define CLK_MON_GCC_APCS_CLOCK_BRANCH_ENA_VOTE	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x45004)
#define CLK_MON_GCC_APCS_CLOCK_SLEEP_ENA_VOTE	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x45008)
#define CLK_MON_GCC_APCS_SMMU_CLOCK_BRANCH_ENA_VOTE	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4500C)
#define CLK_MON_GCC_APSS_AHB_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x46000)
#define CLK_MON_GCC_GCC_DEBUG_CLK_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x74000)
#define CLK_MON_GCC_CLOCK_FRQ_MEASURE_CTL	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x74004)
#define CLK_MON_GCC_CLOCK_FRQ_MEASURE_STATUS	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x74008)
#define CLK_MON_GCC_GCC_PLLTEST_PAD_CFG	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x7400C)
#define CLK_MON_GCC_GP1_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x08000)
#define CLK_MON_GCC_GP1_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x08004)
#define CLK_MON_GCC_GP2_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x09000)
#define CLK_MON_GCC_GP2_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x09004)
#define CLK_MON_GCC_GP3_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0A000)
#define CLK_MON_GCC_GP3_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x0A004)
#define CLK_MON_GCC_SPDM_JPEG0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2F028)
#define CLK_MON_GCC_SPDM_MDP_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2F02C)
#define CLK_MON_GCC_SPDM_VCODEC0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2F034)
#define CLK_MON_GCC_SPDM_VFE0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2F038)
#define CLK_MON_GCC_SPDM_GFX3D_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2F03C)
#define CLK_MON_GCC_SPDM_PCLK0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2F044)
#define CLK_MON_GCC_SPDM_CSI0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x2F048)
#define CLK_MON_GCC_VCODEC0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4C000)
#define CLK_MON_GCC_VENUS0_BCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4C014)
#define CLK_MON_GCC_VENUS0_VCODEC0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4C01C)
#define CLK_MON_GCC_VENUS0_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4C020)
#define CLK_MON_GCC_VENUS0_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4C024)
#define CLK_MON_GCC_PCLK0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D000)
#define CLK_MON_GCC_MDP_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D014)
#define CLK_MON_GCC_VSYNC_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D02C)
#define CLK_MON_GCC_BYTE0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D044)
#define CLK_MON_GCC_ESC0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D05C)
#define CLK_MON_GCC_MDSS_BCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D074)
#define CLK_MON_GCC_MDSS_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D07C)
#define CLK_MON_GCC_MDSS_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D080)
#define CLK_MON_GCC_MDSS_PCLK0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D084)
#define CLK_MON_GCC_MDSS_MDP_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D088)
#define CLK_MON_GCC_MDSS_VSYNC_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D090)
#define CLK_MON_GCC_MDSS_BYTE0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D094)
#define CLK_MON_GCC_MDSS_ESC0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4D098)
#define CLK_MON_GCC_CSI0PHYTIMER_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E000)
#define CLK_MON_GCC_CAMSS_CSI0PHYTIMER_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E01C)
#define CLK_MON_GCC_CSI1PHYTIMER_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F000)
#define CLK_MON_GCC_CAMSS_CSI1PHYTIMER_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F01C)
#define CLK_MON_GCC_CSI0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E020)
#define CLK_MON_GCC_CAMSS_CSI0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E03C)
#define CLK_MON_GCC_CAMSS_CSI0_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E040)
#define CLK_MON_GCC_CAMSS_CSI0PHY_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E048)
#define CLK_MON_GCC_CAMSS_CSI0RDI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E050)
#define CLK_MON_GCC_CAMSS_CSI0PIX_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4E058)
#define CLK_MON_GCC_CSI1_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F020)
#define CLK_MON_GCC_CAMSS_CSI1_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F03C)
#define CLK_MON_GCC_CAMSS_CSI1_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F040)
#define CLK_MON_GCC_CAMSS_CSI1PHY_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F048)
#define CLK_MON_GCC_CAMSS_CSI1RDI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F050)
#define CLK_MON_GCC_CAMSS_CSI1PIX_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x4F058)
#define CLK_MON_GCC_CAMSS_ISPIF_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x50004)
#define CLK_MON_GCC_CCI_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x51000)
#define CLK_MON_GCC_CAMSS_CCI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x51018)
#define CLK_MON_GCC_CAMSS_CCI_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x5101C)
#define CLK_MON_GCC_MCLK0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x52000)
#define CLK_MON_GCC_CAMSS_MCLK0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x52018)
#define CLK_MON_GCC_MCLK1_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x53000)
#define CLK_MON_GCC_CAMSS_MCLK1_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x53018)
#define CLK_MON_GCC_CAMSS_GP0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x54000)
#define CLK_MON_GCC_CAMSS_GP0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x54018)
#define CLK_MON_GCC_CAMSS_GP1_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x55000)
#define CLK_MON_GCC_CAMSS_GP1_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x55018)
#define CLK_MON_GCC_CAMSS_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x5A014)
#define CLK_MON_GCC_CAMSS_TOP_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x56004)
#define CLK_MON_GCC_CAMSS_MICRO_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x5600C)
#define CLK_MON_GCC_CAMSS_MICRO_BCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x56008)
#define CLK_MON_GCC_JPEG0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x57000)
#define CLK_MON_GCC_CAMSS_JPEG0_BCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x57018)
#define CLK_MON_GCC_CAMSS_JPEG0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x57020)
#define CLK_MON_GCC_CAMSS_JPEG_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x57024)
#define CLK_MON_GCC_CAMSS_JPEG_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x57028)
#define CLK_MON_GCC_VFE0_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58000)
#define CLK_MON_GCC_CPP_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58018)
#define CLK_MON_GCC_CAMSS_VFE_BCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58030)
#define CLK_MON_GCC_CAMSS_VFE0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58038)
#define CLK_MON_GCC_CAMSS_CPP_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x5803C)
#define CLK_MON_GCC_CAMSS_CPP_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58040)
#define CLK_MON_GCC_CAMSS_VFE_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58044)
#define CLK_MON_GCC_CAMSS_VFE_AXI_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58048)
#define CLK_MON_GCC_CAMSS_CSI_VFE0_BCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x5804C)
#define CLK_MON_GCC_CAMSS_CSI_VFE0_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x58050)
#define CLK_MON_GCC_GFX3D_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x59000)
#define CLK_MON_GCC_OXILI_GFX3D_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x59020)
#define CLK_MON_GCC_OXILI_GMEM_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x59024)
#define CLK_MON_GCC_OXILI_AHB_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x59028)
#define CLK_MON_GCC_CAMSS_AHB_CMD_RCGR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x5A000)
#define CLK_MON_GCC_BIMC_GFX_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x31024)
#define CLK_MON_GCC_BIMC_GPU_CBCR	\
					(CLK_MON_CGATE_GCC_BLSP_BASE + 0x31040)

static struct clk_gate_mask clk_gate_masks[] = {
	{"apss_ahb", CLK_MON_GCC_APSS_AHB_CMD_RCGR},
	{"camss_ahb", CLK_MON_GCC_CAMSS_AHB_CMD_RCGR},
	{"csi0", CLK_MON_GCC_CSI0_CMD_RCGR},
	{"csi1", CLK_MON_GCC_CSI1_CMD_RCGR},
	{"vfe0", CLK_MON_GCC_VFE0_CMD_RCGR},
	{"gfx3d", CLK_MON_GCC_GFX3D_CMD_RCGR},
	{"blsp1_qup1_i2c_apps", CLK_MON_GCC_BLSP1_QUP1_I2C_APPS_CMD_RCGR},
	{"blsp1_qup1_spi_apps", CLK_MON_GCC_BLSP1_QUP1_SPI_APPS_CMD_RCGR},
	{"blsp1_qup2_i2c_apps", CLK_MON_GCC_BLSP1_QUP2_I2C_APPS_CMD_RCGR},
	{"blsp1_qup2_spi_apps", CLK_MON_GCC_BLSP1_QUP2_SPI_APPS_CMD_RCGR},
	{"blsp1_qup3_i2c_apps", CLK_MON_GCC_BLSP1_QUP3_I2C_APPS_CMD_RCGR},
	{"blsp1_qup3_spi_apps", CLK_MON_GCC_BLSP1_QUP3_SPI_APPS_CMD_RCGR},
	{"blsp1_qup4_i2c_apps", CLK_MON_GCC_BLSP1_QUP4_I2C_APPS_CMD_RCGR},
	{"blsp1_qup4_spi_apps", CLK_MON_GCC_BLSP1_QUP4_SPI_APPS_CMD_RCGR},
	{"blsp1_qup5_i2c_apps", CLK_MON_GCC_BLSP1_QUP5_I2C_APPS_CMD_RCGR},
	{"blsp1_qup5_spi_apps", CLK_MON_GCC_BLSP1_QUP5_SPI_APPS_CMD_RCGR},
	{"blsp1_qup6_i2c_apps", CLK_MON_GCC_BLSP1_QUP6_I2C_APPS_CMD_RCGR},
	{"blsp1_qup6_spi_apps", CLK_MON_GCC_BLSP1_QUP6_SPI_APPS_CMD_RCGR},
	{"blsp1_uart1_apps", CLK_MON_GCC_BLSP1_UART1_APPS_CMD_RCGR},
	{"blsp1_uart2_apps", CLK_MON_GCC_BLSP1_UART2_APPS_CMD_RCGR},
	{"cci", CLK_MON_GCC_CCI_CMD_RCGR},
	{"camss_gp0", CLK_MON_GCC_CAMSS_GP0_CMD_RCGR},
	{"camss_gp1", CLK_MON_GCC_CAMSS_GP1_CMD_RCGR},
	{"jpeg0", CLK_MON_GCC_JPEG0_CMD_RCGR},
	{"mclk0", CLK_MON_GCC_MCLK0_CMD_RCGR},
	{"mclk1", CLK_MON_GCC_MCLK1_CMD_RCGR},
	{"csi0phytimer", CLK_MON_GCC_CSI0PHYTIMER_CMD_RCGR},
	{"csi1phytimer", CLK_MON_GCC_CSI1PHYTIMER_CMD_RCGR},
	{"cpp", CLK_MON_GCC_CPP_CMD_RCGR},
	{"gp1", CLK_MON_GCC_GP1_CMD_RCGR},
	{"gp2", CLK_MON_GCC_GP2_CMD_RCGR},
	{"gp3", CLK_MON_GCC_GP3_CMD_RCGR},
	{"byte0", CLK_MON_GCC_BYTE0_CMD_RCGR},
	{"esc0", CLK_MON_GCC_ESC0_CMD_RCGR},
	{"mdp", CLK_MON_GCC_MDP_CMD_RCGR},
	{"pclk0", CLK_MON_GCC_PCLK0_CMD_RCGR},
	{"vsync", CLK_MON_GCC_VSYNC_CMD_RCGR},
	{"pdm2", CLK_MON_GCC_PDM2_CMD_RCGR},
	{"sdcc1_apps", CLK_MON_GCC_SDCC1_APPS_CMD_RCGR},
	{"sdcc2_apps", CLK_MON_GCC_SDCC2_APPS_CMD_RCGR},
	{"usb_hs_system", CLK_MON_GCC_USB_HS_SYSTEM_CMD_RCGR},
	{"vcodec0", CLK_MON_GCC_VCODEC0_CMD_RCGR},
	{"gcc_blsp1_ahb_clk", CLK_MON_GCC_BLSP1_AHB_CBCR},
	{"gcc_blsp1_qup1_i2c_apps_clk", CLK_MON_GCC_BLSP1_QUP1_I2C_APPS_CBCR},
	{"gcc_blsp1_qup1_spi_apps_clk", CLK_MON_GCC_BLSP1_QUP1_SPI_APPS_CBCR},
	{"gcc_blsp1_qup2_i2c_apps_clk", CLK_MON_GCC_BLSP1_QUP2_I2C_APPS_CBCR},
	{"gcc_blsp1_qup2_spi_apps_clk", CLK_MON_GCC_BLSP1_QUP2_SPI_APPS_CBCR},
	{"gcc_blsp1_qup3_i2c_apps_clk", CLK_MON_GCC_BLSP1_QUP3_I2C_APPS_CBCR},
	{"gcc_blsp1_qup3_spi_apps_clk", CLK_MON_GCC_BLSP1_QUP3_SPI_APPS_CBCR},
	{"gcc_blsp1_qup4_i2c_apps_clk", CLK_MON_GCC_BLSP1_QUP4_I2C_APPS_CBCR},
	{"gcc_blsp1_qup4_spi_apps_clk", CLK_MON_GCC_BLSP1_QUP4_SPI_APPS_CBCR},
	{"gcc_blsp1_qup5_i2c_apps_clk", CLK_MON_GCC_BLSP1_QUP5_I2C_APPS_CBCR},
	{"gcc_blsp1_qup5_spi_apps_clk", CLK_MON_GCC_BLSP1_QUP5_SPI_APPS_CBCR},
	{"gcc_blsp1_qup6_i2c_apps_clk", CLK_MON_GCC_BLSP1_QUP6_I2C_APPS_CBCR},
	{"gcc_blsp1_qup6_spi_apps_clk", CLK_MON_GCC_BLSP1_QUP6_SPI_APPS_CBCR},
	{"gcc_blsp1_uart1_apps_clk", CLK_MON_GCC_BLSP1_UART1_APPS_CBCR},
	{"gcc_blsp1_uart2_apps_clk", CLK_MON_GCC_BLSP1_UART2_APPS_CBCR},
	{"gcc_boot_rom_ahb_clk", CLK_MON_GCC_BOOT_ROM_AHB_CBCR},
	{"gcc_camss_cci_ahb_clk", CLK_MON_GCC_CAMSS_CCI_AHB_CBCR},
	{"gcc_camss_cci_clk", CLK_MON_GCC_CAMSS_CCI_CBCR},
	{"gcc_camss_csi0_ahb_clk", CLK_MON_GCC_CAMSS_CSI0_AHB_CBCR},
	{"gcc_camss_csi0_clk", CLK_MON_GCC_CAMSS_CSI0_CBCR},
	{"gcc_camss_csi0phy_clk", CLK_MON_GCC_CAMSS_CSI0PHY_CBCR},
	{"gcc_camss_csi0pix_clk", CLK_MON_GCC_CAMSS_CSI0PIX_CBCR},
	{"gcc_camss_csi0rdi_clk", CLK_MON_GCC_CAMSS_CSI0RDI_CBCR},
	{"gcc_camss_csi1_ahb_clk", CLK_MON_GCC_CAMSS_CSI1_AHB_CBCR},
	{"gcc_camss_csi1_clk", CLK_MON_GCC_CAMSS_CSI1_CBCR},
	{"gcc_camss_csi1phy_clk", CLK_MON_GCC_CAMSS_CSI1PHY_CBCR},
	{"gcc_camss_csi1pix_clk", CLK_MON_GCC_CAMSS_CSI1PIX_CBCR},
	{"gcc_camss_csi1rdi_clk", CLK_MON_GCC_CAMSS_CSI1RDI_CBCR},
	{"gcc_camss_csi_vfe0_clk", CLK_MON_GCC_CAMSS_CSI_VFE0_CBCR},
	{"gcc_camss_gp0_clk", CLK_MON_GCC_CAMSS_GP0_CBCR},
	{"gcc_camss_gp1_clk", CLK_MON_GCC_CAMSS_GP1_CBCR},
	{"gcc_camss_ispif_ahb_clk", CLK_MON_GCC_CAMSS_ISPIF_AHB_CBCR},
	{"gcc_camss_jpeg0_clk", CLK_MON_GCC_CAMSS_JPEG0_CBCR},
	{"gcc_camss_jpeg_ahb_clk", CLK_MON_GCC_CAMSS_JPEG_AHB_CBCR},
	{"gcc_camss_jpeg_axi_clk", CLK_MON_GCC_CAMSS_JPEG_AXI_CBCR},
	{"gcc_camss_mclk0_clk", CLK_MON_GCC_CAMSS_MCLK0_CBCR},
	{"gcc_camss_mclk1_clk", CLK_MON_GCC_CAMSS_MCLK1_CBCR},
	{"gcc_camss_micro_ahb_clk", CLK_MON_GCC_CAMSS_MICRO_AHB_CBCR},
	{"gcc_camss_csi0phytimer_clk", CLK_MON_GCC_CAMSS_CSI0PHYTIMER_CBCR},
	{"gcc_camss_csi1phytimer_clk", CLK_MON_GCC_CAMSS_CSI1PHYTIMER_CBCR},
	{"gcc_camss_ahb_clk", CLK_MON_GCC_CAMSS_AHB_CBCR},
	{"gcc_camss_top_ahb_clk", CLK_MON_GCC_CAMSS_TOP_AHB_CBCR},
	{"gcc_camss_cpp_ahb_clk", CLK_MON_GCC_CAMSS_CPP_AHB_CBCR},
	{"gcc_camss_cpp_clk", CLK_MON_GCC_CAMSS_CPP_CBCR},
	{"gcc_camss_vfe0_clk", CLK_MON_GCC_CAMSS_VFE0_CBCR},
	{"gcc_camss_vfe_ahb_clk", CLK_MON_GCC_CAMSS_VFE_AHB_CBCR},
	{"gcc_camss_vfe_axi_clk", CLK_MON_GCC_CAMSS_VFE_AXI_CBCR},
	{"crypto", CLK_MON_GCC_CRYPTO_CMD_RCGR},
	{"gcc_crypto_ahb_clk", CLK_MON_GCC_CRYPTO_AHB_CBCR},
	{"gcc_crypto_axi_clk", CLK_MON_GCC_CRYPTO_AXI_CBCR},
	{"gcc_crypto_clk", CLK_MON_GCC_CRYPTO_CBCR},
	{"gcc_oxili_gmem_clk", CLK_MON_GCC_OXILI_GMEM_CBCR},
	{"gcc_bimc_gfx_clk", CLK_MON_GCC_BIMC_GFX_CBCR},
	{"gcc_bimc_gpu_clk", CLK_MON_GCC_BIMC_GPU_CBCR},
	{"gcc_gp1_clk", CLK_MON_GCC_GP1_CBCR},
	{"gcc_gp2_clk", CLK_MON_GCC_GP2_CBCR},
	{"gcc_gp3_clk", CLK_MON_GCC_GP3_CBCR},
	{"gcc_mdss_ahb_clk", CLK_MON_GCC_MDSS_AHB_CBCR},
	{"gcc_mdss_axi_clk", CLK_MON_GCC_MDSS_AXI_CBCR},
	{"gcc_mdss_byte0_clk", CLK_MON_GCC_MDSS_BYTE0_CBCR},
	{"gcc_mdss_esc0_clk", CLK_MON_GCC_MDSS_ESC0_CBCR},
	{"gcc_mdss_mdp_clk", CLK_MON_GCC_MDSS_MDP_CBCR},
	{"gcc_mdss_pclk0_clk", CLK_MON_GCC_MDSS_PCLK0_CBCR},
	{"gcc_mdss_vsync_clk", CLK_MON_GCC_MDSS_VSYNC_CBCR},
	{"gcc_mss_cfg_ahb_clk", CLK_MON_GCC_MSS_CFG_AHB_CBCR},
	{"gcc_mss_q6_bimc_axi_clk", CLK_MON_GCC_MSS_Q6_BIMC_AXI_CBCR},
	{"gcc_oxili_ahb_clk", CLK_MON_GCC_OXILI_AHB_CBCR},
	{"gcc_oxili_gfx3d_clk", CLK_MON_GCC_OXILI_GFX3D_CBCR},
	{"gcc_pdm2_clk", CLK_MON_GCC_PDM2_CBCR},
	{"gcc_pdm_ahb_clk", CLK_MON_GCC_PDM_AHB_CBCR},
	{"gcc_prng_ahb_clk", CLK_MON_GCC_PRNG_AHB_CBCR},
	{"gcc_sdcc1_ahb_clk", CLK_MON_GCC_SDCC1_AHB_CBCR},
	{"gcc_sdcc1_apps_clk", CLK_MON_GCC_SDCC1_APPS_CBCR},
	{"gcc_sdcc2_ahb_clk", CLK_MON_GCC_SDCC2_AHB_CBCR},
	{"gcc_sdcc2_apps_clk", CLK_MON_GCC_SDCC2_APPS_CBCR},
	{"gcc_apss_tcu_clk", CLK_MON_GCC_APSS_TCU_CBCR},
	{"gcc_gfx_tcu_clk", CLK_MON_GCC_GFX_TCU_CBCR},
	{"gcc_gfx_tbu_clk", CLK_MON_GCC_GFX_TBU_CBCR},
	{"gcc_mdp_tbu_clk", CLK_MON_GCC_MDP_TBU_CBCR},
	{"gcc_venus_tbu_clk", CLK_MON_GCC_VENUS_TBU_CBCR},
	{"gcc_vfe_tbu_clk", CLK_MON_GCC_VFE_TBU_CBCR},
	{"gcc_jpeg_tbu_clk", CLK_MON_GCC_JPEG_TBU_CBCR},
	{"gcc_smmu_cfg_clk", CLK_MON_GCC_SMMU_CFG_CBCR},
	{"gcc_gtcu_ahb_clk", CLK_MON_GCC_GTCU_AHB_CBCR},
	{"gcc_usb2a_phy_sleep_clk", CLK_MON_GCC_USB2A_PHY_SLEEP_CBCR},
	{"gcc_usb_hs_ahb_clk", CLK_MON_GCC_USB_HS_AHB_CBCR},
	{"gcc_usb_hs_system_clk", CLK_MON_GCC_USB_HS_SYSTEM_CBCR},
	{"gcc_venus0_ahb_clk", CLK_MON_GCC_VENUS0_AHB_CBCR},
	{"gcc_venus0_axi_clk", CLK_MON_GCC_VENUS0_AXI_CBCR},
	{"gcc_venus0_vcodec0_clk", CLK_MON_GCC_VENUS0_VCODEC0_CBCR},
	/* Any Missing Clocks to be added here */
	{{0}, 0},
};

/* Useage - echo "Register_Address" > check_reg
	Eg. Input - echo 0xFC400644 > check_reg
	>> Output - cat check_reg
	>> [0xfc400644] 0x80000000
*/
static int clk_mon_ioc_check_reg(struct clk_mon_ioc_buf __user *uarg)
{
	struct clk_mon_ioc_buf *karg = NULL;
	void __iomem *v_addr = NULL;
	int size = sizeof(struct clk_mon_ioc_buf);
	int ret = -EFAULT;
	int i;

	if (!access_ok(VERIFY_WRITE, uarg, size))
		return -EFAULT;

	karg = kzalloc(size, GFP_KERNEL);

	if (!karg)
		return -ENOMEM;

	if (copy_from_user(karg, uarg, size)) {
		ret = -EFAULT;
		goto out;
	}

	for (i = 0; i < karg->nr_addrs; i++) {
		v_addr = ioremap((unsigned int)karg->reg[i].addr, SIZE_REG);
		karg->reg[i].value = ioread32(v_addr);
		iounmap(v_addr);
	}

	if (copy_to_user(uarg, karg, size)) {
		ret = -EFAULT;
		goto out;
	}
	ret = 0;

out:
	kfree(karg);
	return ret;
}

static int clk_mon_ioc_check_power_domain(struct clk_mon_ioc_buf __user *uarg)
{
	struct clk_mon_ioc_buf *karg = NULL;
	unsigned int dom_en = 0;
	int size = sizeof(struct clk_mon_ioc_buf);
	int ret = -EFAULT;
	int i;
	unsigned int num_domains = 0;
	static struct regulator *regulator_pm;

	if (!access_ok(VERIFY_WRITE, uarg, size))
		return -EFAULT;

	karg = kzalloc(size, GFP_KERNEL);

	if (!karg)
		return -ENOMEM;

	num_domains = sizeof(power_domain_masks)/sizeof(power_domain_masks[0]);

	for (i = 0; i < num_domains; i++) {
		regulator_pm = regulator_get(NULL, power_domain_masks[i].name);
		if (IS_ERR(regulator_pm)) {
			pr_err("%s - Failed to get [%s] regulator\n",
				__func__, power_domain_masks[i].name);
		} else {
			dom_en = regulator_is_enabled(regulator_pm);
			if (dom_en < 0)	{
				pr_err("%s - Getting status of [%s] regulator failed\n",
					__func__, power_domain_masks[i].name);
			} else {
				strlcpy(karg->reg[i].name,
					power_domain_masks[i].name,
					sizeof(karg->reg[i].name));
				karg->reg[i].value = dom_en;
				/* Free the regulator from the consumer list
				else suspend would be prevented
				*/
				regulator_put(regulator_pm);
				regulator_pm = NULL;
				karg->nr_addrs++;
			}
		}
	}

	if (copy_to_user(uarg, karg, size)) {
		ret = -EFAULT;
		goto out;
	}

	ret = 0;

out:
	kfree(karg);
	return ret;
}

static int clk_mon_ioc_check_clock_gating(struct clk_mon_ioc_buf __user *uarg)
{
	struct clk_mon_ioc_buf *karg = NULL;
	unsigned int val = 0, value = 0;
	int size = sizeof(struct clk_mon_ioc_buf);
	int ret = -EFAULT;
	int i;
	void __iomem *v_addr = NULL;

	if (!access_ok(VERIFY_WRITE, uarg, size))
		return -EFAULT;

	karg = kzalloc(size, GFP_KERNEL);

	if (!karg)
		return -ENOMEM;

	for (i = 0; clk_gate_masks[i].addr != 0; i++) {
		v_addr = ioremap((unsigned int)clk_gate_masks[i].addr,
				SIZE_REG);
		value = ioread32(v_addr);
		/* Bit 31 indicates whether CLK is ON/OFF
		CLK_OFF : 0 - CLK ON, 1 - CLK OFF
		*/
		val = CHECK_BIT_SET((unsigned int) value, CLK_STATUS_BIT_POS);
		/* The output contains the register_name, address & value */
		strlcpy(karg->reg[i].name,
			clk_gate_masks[i].name,
			sizeof(karg->reg[i].name));
		karg->reg[i].addr = (void *) (&(clk_gate_masks[i].addr));
		karg->reg[i].value = val;
		karg->nr_addrs++;
	}

	if (copy_to_user(uarg, karg, size)) {
		ret = -EFAULT;
		goto out;
	}

	ret = 0;

out:
	kfree(karg);
	return ret;
}

/* Useage - echo "Register_Address" "Value_to_be_set" > set_reg
	Eg. Input - echo 0xFC400648 0x1 > set_reg
	>> Output - cat check_reg
	>> [0xfc400648] 0x00000001
*/
static int clk_mon_ioc_set_reg(struct clk_mon_reg_info __user *uarg)
{
	struct clk_mon_reg_info *karg = NULL;
	void __iomem *v_addr = NULL;
	int size = sizeof(struct clk_mon_reg_info);
	int ret = 0;

	if (!access_ok(VERIFY_READ, uarg, size))
		return -EFAULT;

	karg = kzalloc(size, GFP_KERNEL);

	if (!karg)
		return -ENOMEM;

	if (copy_from_user(karg, uarg, size)) {
		ret = -EFAULT;
		goto out;
	}

	v_addr = ioremap((unsigned int)karg->addr, SIZE_REG);
	iowrite32(karg->value, v_addr);
	iounmap(v_addr);

	ret = 0;

out:
	kfree(karg);
	return ret;
}

static long clk_mon_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	struct clk_mon_ioc_buf __user *uarg = NULL;
	int ret = 0;

	pr_info("%s\n", __func__);

	if (!arg)
		return -EINVAL;

	uarg = (struct clk_mon_ioc_buf __user *)arg;

	switch (cmd) {
	case CLK_MON_IOC_CHECK_REG:
		ret = clk_mon_ioc_check_reg(uarg);
		break;
	case CLK_MON_IOC_CHECK_POWER_DOMAIN:
		ret = clk_mon_ioc_check_power_domain(uarg);
		break;
	case CLK_MON_IOC_CHECK_CLOCK_DOMAIN:
		ret = clk_mon_ioc_check_clock_gating(uarg);
		break;
	case CLK_MON_IOC_SET_REG:
		ret = clk_mon_ioc_set_reg(
				(struct clk_mon_reg_info __user *)arg);
		break;
	default:
		pr_err("%s:Invalid ioctl\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static unsigned int g_reg_addr;
static unsigned int g_reg_value;

static ssize_t clk_mon_store_check_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int reg_addr = 0;
	char *cur = NULL;
	int ret = 0;

	if (!buf)
		return -EINVAL;

	cur = strnstr(buf, "0x", sizeof(buf));

	if (cur && cur + 2)
		ret = sscanf(cur + 2, "%x", &reg_addr);

	if (!ret)
		return -EINVAL;

	g_reg_addr = reg_addr;

	return size;
}

static ssize_t clk_mon_show_check_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	void __iomem *v_addr = NULL;
	unsigned int p_addr = 0;
	unsigned int value = 0;
	ssize_t size = 0;

	if (!g_reg_addr)
		return -EINVAL;

	p_addr = g_reg_addr;
	v_addr = ioremap(p_addr, SIZE_REG);

	value = ioread32(v_addr);
	iounmap(v_addr);

	size += snprintf(buf + size, CLK_MON_BUF_SIZE,
		"[0x%x] 0x%x\n", p_addr, value) + 1;

	return size + 1;
}

static ssize_t clk_mon_store_set_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int reg_addr = 0;
	unsigned int reg_value = 0;
	void __iomem *v_addr = NULL;
	char tmp_addr[9] = {0};
	char *cur = NULL;

	if (!buf)
		return -EINVAL;

	cur = strnstr(buf, "0x", strlen(buf));

	if (!cur || !(cur + 2))
		return -EINVAL;

	strlcpy(tmp_addr, cur + 2, 8);

	if (!sscanf(tmp_addr, "%x", &reg_addr))
		return -EFAULT;

	cur = strnstr(&cur[2], "0x", strlen(&cur[2]));

	if (!cur || !(cur + 2))
		return -EINVAL;

	if (!sscanf(cur + 2, "%x", &reg_value))
		return -EFAULT;

	g_reg_addr  = reg_addr;
	g_reg_value = reg_value;

	v_addr = ioremap(g_reg_addr, SIZE_REG);
	iowrite32(g_reg_value, v_addr);
	iounmap(v_addr);

	return size;
}

static const int NR_BIT = 8 * sizeof(unsigned int);
static const int IDX_SHIFT = 5;

int clk_mon_power_domain(unsigned int *pm_status)
{
	unsigned int dom_en = 0;
	int i, bit_shift, idx;
	unsigned int num_domains = 0;
	static struct regulator *regulator_pm;
	/* In total, 62+ regulators are present */
	int bit_max = NR_BIT * PWR_DOMAINS_NUM;

	num_domains = sizeof(power_domain_masks)/sizeof(power_domain_masks[0]);

	/* Parse through the list of regulators & based on request from the
	consumers of the regulator, it would be enabled/disabled i.e. ON/OFF
	*/

	if (!pm_status || bit_max < 0 || num_domains <= 0)
		return -EINVAL;

	memset(pm_status, 0, sizeof(unsigned int) * PWR_DOMAINS_NUM);

	for (i = 0; i < num_domains; i++) {
		if (i > bit_max) {
			pr_err("%s: Error Exceed storage size %d(%d)\n",
				__func__, i, bit_max);
			break;
		}
		regulator_pm = regulator_get(NULL, power_domain_masks[i].name);
		if (IS_ERR(regulator_pm)) {
			pr_err("%s - Failed to get [%s] regulator\n",
				__func__, power_domain_masks[i].name);
		} else {
			idx = (i >> IDX_SHIFT);
			bit_shift = (i % NR_BIT);
			/* Check the regulator status */
			dom_en = regulator_is_enabled(regulator_pm);
			if (dom_en < 0) {
				pr_err("%s-Getting status of [%s] regulator failed\n",
					__func__, power_domain_masks[i].name);
			} else {
				if (dom_en)
					pm_status[idx] |= (0x1 << bit_shift);
				else
					pm_status[idx] &= ~(0x1 << bit_shift);
				regulator_put(regulator_pm);
				regulator_pm = NULL;
			}
		}
	}
	return i;
}

int clk_mon_get_power_info(unsigned int *pm_status, char *buf)
{
	int i, bit_shift, idx, size = 0;
	unsigned int num_domains = 0, dom_en = 0;
	int bit_max = NR_BIT * PWR_DOMAINS_NUM;

	num_domains = sizeof(power_domain_masks)/sizeof(power_domain_masks[0]);

	if  ((!pm_status) || (!buf) || (num_domains <= 0))
		return -EINVAL;

	for (i = 0; i < num_domains; i++) {
		if (i > bit_max) {
			pr_err("%s: Error Exceed storage size %d(%d)\n",
				__func__, i, NR_BIT);
			break;
		}

		bit_shift = i % NR_BIT;
		idx = i >> IDX_SHIFT;
		dom_en = 0;
		/* If the bit is set indicates that the regulator is enabled as
		observed in the API clk_mon_power_domain.
		*/
		dom_en = CHECK_BIT_SET(pm_status[idx], bit_shift);

		size += snprintf(buf + size, CLK_MON_BUF_SIZE,
				"[%-15s] %-3s\n",
				power_domain_masks[i].name,
				(dom_en) ? "on" : "off");
	}
	return size + 1;
}

int clk_mon_clock_gate(unsigned int *clk_status)
{
	int bit_max = NR_BIT * CLK_GATES_NUM;
	unsigned int val = 0, value = 0;
	void __iomem *v_addr = NULL;
	unsigned long addr = 0;
	unsigned int clk_dis = 0;
	int i, bit_shift, idx;

	if (!clk_status || bit_max < 0)
		return -EINVAL;

	memset(clk_status, 0, sizeof(unsigned int) * CLK_GATES_NUM);

	for (i = 0; clk_gate_masks[i].addr != 0; i++) {
		if (i >= bit_max) {
			pr_err("%s: Error Exceed storage size %d(%d)\n",
				__func__, i, bit_max);
			break;
		}

		if (addr != clk_gate_masks[i].addr) {
			/* addr = clk_gate_masks[i].addr; */
			v_addr = ioremap((unsigned int)clk_gate_masks[i].addr,
					SIZE_REG);
			value = ioread32(v_addr);
		}
		/* Bit 31 indicates whether CLK is ON/OFF
		 * 0 - CLK ON,1 - CLK OFF
		*/
		val = CHECK_BIT_SET((unsigned int) value, CLK_STATUS_BIT_POS);
		clk_dis = val;

		idx = i >> IDX_SHIFT;
		bit_shift = i % NR_BIT;

		if (clk_dis)
			clk_status[idx] &= ~(BIT_ONE << bit_shift);
		else
			clk_status[idx] |= (BIT_ONE << bit_shift);

		/* Unmap the memory */
		if (v_addr != NULL)
			iounmap(v_addr);
	}
	return i;
}

int clk_mon_get_clock_info(unsigned int *clk_status, char *buf)
{
	unsigned long addr = 0;
	int bit_max = NR_BIT * CLK_GATES_NUM;
	int bit_shift, idx;
	int size = 0;
	int val, i;
	void __iomem *v_addr = NULL;
	unsigned int value = 0;

	if (!clk_status || !buf)
		return -EINVAL;

	for (i = 0; clk_gate_masks[i].addr != 0; i++) {
		if (i >= bit_max) {
			pr_err("%s: Error Exceed storage size %d(%d)\n",
				__func__, i, bit_max);
			break;
		}

		if (addr != clk_gate_masks[i].addr) {
			addr = clk_gate_masks[i].addr;
			v_addr = ioremap((unsigned int)clk_gate_masks[i].addr,
					SIZE_REG);
			value = ioread32(v_addr);
			size += snprintf(buf + size, CLK_MON_BUF_SIZE,
				"\n[0x%x]\n",
				((unsigned int) clk_gate_masks[i].addr));
			if (v_addr != NULL)
				iounmap(v_addr);
		}

		bit_shift = i % NR_BIT;
		idx = i >> IDX_SHIFT;
		/* If the bit is set indicates that the clock is enabled as
		observed in the API clk_mon_clock_gate.
		*/
		val = CHECK_BIT_SET(clk_status[idx], bit_shift);

		size += snprintf(buf + size, CLK_MON_BUF_SIZE,
				" %-20s\t: %s\n", clk_gate_masks[i].name,
				(val) ? "on" : "off");
	}
	return size;
}

static ssize_t clk_mon_show_power_domain(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int val = 0;
	ssize_t size = 0;
	static struct regulator *regulator_pm;
	unsigned int num_domains = 0;
	int i;

	num_domains = sizeof(power_domain_masks)/sizeof(power_domain_masks[0]);

	memset(buf, 0, sizeof(buf));

	/* Parse through the list of regulators & based on request from the
	consumers of the regulator, it would be enabled/disabled i.e. ON/OFF
	*/
	for (i = 0; i < num_domains; i++) {
		regulator_pm = regulator_get(NULL, power_domain_masks[i].name);
		if (IS_ERR(regulator_pm)) {
			pr_err("Failed to get [%s] regulator\n",
				power_domain_masks[i].name);
		} else {
			val = regulator_is_enabled(regulator_pm);
			if (val < 0) {
				pr_err("Getting status of [%s] regulator failed\n",
					power_domain_masks[i].name);
			} else {
				regulator_put(regulator_pm);
				regulator_pm = NULL;
			}
		}
		size += snprintf(buf + size, CLK_MON_BUF_SIZE,
			" %-15s\t: %s\n",
			power_domain_masks[i].name, (val) ? "on" : "off");
	}
	return size + 1;
}

static ssize_t clk_mon_show_clock_gating(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	void __iomem *v_addr = NULL;
	unsigned int val  = 0;
	unsigned int value = 0;
	ssize_t size = 0;
	int i;

	for (i = 0; clk_gate_masks[i].addr != 0; i++) {
		v_addr = ioremap((unsigned int)clk_gate_masks[i].addr,
				SIZE_REG);
		value = ioread32(v_addr);
		val = CHECK_BIT_SET((unsigned int) value,
				CLK_STATUS_BIT_POS);

		size += snprintf(buf + size, CLK_MON_BUF_SIZE,
				" %-20s\t: %s\n",
				clk_gate_masks[i].name, (val) ? "off" : "on");
		iounmap(v_addr);
	}

	return size + 1;
}


static DEVICE_ATTR(check_reg, S_IRUSR | S_IWUSR,
		clk_mon_show_check_reg, clk_mon_store_check_reg);
static DEVICE_ATTR(set_reg, S_IWUSR, NULL, clk_mon_store_set_reg);
static DEVICE_ATTR(power_domain, S_IRUSR, clk_mon_show_power_domain, NULL);
static DEVICE_ATTR(clock_gating, S_IRUSR, clk_mon_show_clock_gating, NULL);

static struct attribute *clk_mon_attributes[] = {
	&dev_attr_check_reg.attr,
	&dev_attr_set_reg.attr,
	&dev_attr_power_domain.attr,
	&dev_attr_clock_gating.attr,
	NULL,
};

static struct attribute_group clk_mon_attr_group = {
	.attrs = clk_mon_attributes,
	.name  = "check",
};

static const struct file_operations clk_mon_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = clk_mon_ioctl,
};

static struct miscdevice clk_mon_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "clk_mon",
	.fops  = &clk_mon_fops,
};

static int __init clk_mon_init(void)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = misc_register(&clk_mon_device);

	if (ret) {
		pr_err("%s: Unable to register clk_mon_device\n", __func__);
		goto err_misc_register;
	}

	ret = sysfs_create_group(&clk_mon_device.this_device->kobj,
			&clk_mon_attr_group);

	if (ret) {
		pr_err("%s: Unable to Create sysfs node\n", __func__);
		goto err_create_group;
	}

	return 0;

err_create_group:
	misc_deregister(&clk_mon_device);
err_misc_register:
	return ret;
}

static void __exit clk_mon_exit(void)
{
	misc_deregister(&clk_mon_device);
}

module_init(clk_mon_init);
module_exit(clk_mon_exit);

MODULE_AUTHOR("Himanshu Sheth <himanshu.s@samsung.com>");
MODULE_DESCRIPTION("Clock Gate Monitor");
MODULE_LICENSE("GPL");
