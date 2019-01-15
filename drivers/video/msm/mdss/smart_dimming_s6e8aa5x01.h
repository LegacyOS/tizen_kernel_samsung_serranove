/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef SMART_DIMMING_S6E8AA5X01_H
#define SMART_DIMMING_S6E8AA5X01_H


/* color index */
enum {
	CI_RED = 0,
	CI_GREEN,
	CI_BLUE,
	CI_MAX,
};

enum {
	V0,
	V3,
	V11,
	V23,
	V35,
	V51,
	V87,
	V151,
	V203,
	V255,
	VMAX
};

#define VT						V0
#define NUM_VREF					VMAX	/*10*/
#define MULTIPLE_VREGOUT			5939	/* 5.8 * 1024 */
#define MAX_GAMMA				360		/*360cd*/
#define MAX_GRADATION			256
#define MTP_LEN					33
#define HBM_MTP1_LEN				6
#define HBM_MTP1_ELEN				39
#define HBM_MTP1_OFFSET			33
#define HBM_MTP2_LEN				15
#define HBM_MTP2_OFFSET			72
#define HBM_MTP2_ELEN			87
#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
#define MDNIE_MTP_LEN			4
#define MDNIE_MTP_OFFSET			3
#endif

#define TBL_INDEX_V0			0
#define TBL_INDEX_V3			3
#define TBL_INDEX_V11			11
#define TBL_INDEX_V23			23
#define TBL_INDEX_V35			35
#define TBL_INDEX_V51			51
#define TBL_INDEX_V87			87
#define TBL_INDEX_V151			151
#define TBL_INDEX_V203			203
#define TBL_INDEX_V255			255

#define GAMMA_CNT				33
#define MAX_GAMMA_CNT			62
#define GAMMA_CMD_CNT			34
#define RGB_COMPENSATION		30

#define ELVSS_DIM_OFFSET_LEN		1
#define ELVSS_DIM_MTP_OFFSET		21
#define ELVSS_TEMP_LEVEL1			14
#define ELVSS_TEMP_LEVEL2			20

#define DIM_BRIGHTNESS			17

struct v_constant {
	int nu;
	int de;
};


struct smart_dimming {
	int gamma[NUM_VREF][CI_MAX];
	int mtp[NUM_VREF][CI_MAX];
	int volt[MAX_GRADATION][CI_MAX];
	int volt_vt[CI_MAX];
	int look_volt[NUM_VREF][CI_MAX];
	unsigned char gamma_tbl[MAX_GAMMA_CNT][GAMMA_CMD_CNT];
	unsigned char elvss_dim_offset[ELVSS_DIM_OFFSET_LEN];
};

#endif /* SMART_DIMMING_S6E8AA5X01_H */
