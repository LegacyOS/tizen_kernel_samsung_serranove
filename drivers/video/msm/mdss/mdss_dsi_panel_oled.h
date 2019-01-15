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

#ifndef MDSS_DSI_PANEL_OLED_H
#define MDSS_DSI_PANEL_OLED_H


#include "mdss_dsi.h"

enum {
	TEMP_RANGE_0 = 0,	/* 0 < temperature*/
	TEMP_RANGE_1,		/*-20 < temperature < =0*/
	TEMP_RANGE_2,		/*temperature <=-20*/
};

typedef struct mdss_panel_oled_driver_data __mdss_panel_oled_driver_data;

struct panel_func {
	int (*probe)(struct mdss_panel_oled_driver_data* odd);
	int (*get_aid_index)(int level);
	int (*get_elvss_index)(int level);
	int (*init_gamma_tbl)(const unsigned char *data);
	int (*update_gamma_tbl)(int level, char *data);
	struct mtp_info (*get_mtp_info)(void);
	struct mtp_info (*get_hbm_mtp1_info)(void);
	struct mtp_info (*get_hbm_mtp2_info)(void);
	struct mtp_info (*get_elvss_dim_offset_info)(void);
	int (*set_elvss_dim_offset)(const unsigned char *data);
	int (*init_hbm_gamma)(const unsigned char *mtp1, const unsigned char *mtp2, char *data);
	int (*update_elvss_param1)(int temperature, int level, char *data);
	int (*update_elvss_param2)(int temperature, int level, char *data);
#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
	struct mtp_info (*get_mdnie_coordinate_info)(void);
#endif
	int (*dimming_adjust)(int new_lvl, int curr_lvl);
};

struct panel_cmd_list {
	struct dsi_panel_cmds chip_id_rx_cmd;
	struct dsi_panel_cmds aid_cmd;
	struct dsi_panel_cmds acl_on;
	struct dsi_panel_cmds acl_off;
	struct dsi_panel_cmds gamma_update;
	struct dsi_panel_cmds elvss_cmd;
	struct dsi_panel_cmds elvss_rx_cmd;
	struct dsi_panel_cmds elvss_tx_temp_cmd;
	struct dsi_panel_cmds test_key_enable;
	struct dsi_panel_cmds test_key_disable;
	struct dsi_panel_cmds read_mtp;
	struct dsi_panel_cmds hbm_tx_cmd;
	struct dsi_panel_cmds hbm_gamma_tx_cmd;
	struct dsi_panel_cmds hbm_off_cmd;
	struct dsi_panel_cmds hbm_read_mtp;
	struct dsi_panel_cmds g_para_tx_cmd;
#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
	struct dsi_panel_cmds mdnie_rx_cmd;
#endif
};

struct panel_stat {
	int hbm_on;
	int hbm_update;
	int need_check_chip_id;
	int need_check_mtp;
	int need_check_hbm_mtp;
	int need_check_elvss_offset;
#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
	int need_check_mdnie_coordinate;
#endif
	int temp_stage;
	unsigned char chip_id_t[5];
	int current_level;
};

struct mtp_info {
	int length;
	int offset;
};

struct mdss_panel_oled_driver_data {
	struct panel_func func;
	struct panel_cmd_list cmd_list;
	struct panel_stat stat;
	struct mdss_panel_data *pan_data;
#if defined(CONFIG_LCD_CLASS_DEVICE)
#if defined(CONFIG_BACKLIGHT_CLASS_DEVICE)
	struct backlight_device *bd;
#endif
#endif
};

struct mdss_panel_oled_driver_data* mdss_dsi_panel_oled_get_odd(void);
int mdss_dsi_panel_oled_init_mtp(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

int mdss_dsi_panel_oled_panel_on(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

int mdss_dsi_panel_oled_init(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device *parent,
			struct device *ld_dev);

#endif /* MDSS_DSI_PANEL_OLED_H */
