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
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include "mdss_dsi_panel_oled.h"
#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
#include "mdnie/mdnie_lite.h"
#endif

#define DEFAULT_BRIGHTNESS 100
#define MAX_BRIGHTNESS 100
#define MIN_BRIGHTNESS 0

struct mdss_panel_oled_driver_data oled_drv_data;

static void mdss_dsi_panel_oled_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds);
static int mdss_dsi_panel_oled_cmds_read(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_cmd_desc *cmd, int rlen);

int mdss_dsi_panel_check_mtp(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mtp_info info;
	int rlen = 0;
	int offset = 0;
	int ret = 0;
	char* mtp_buf = NULL;
	int step = 6;
	int remain = 0;
	int buf_offset = 0;

	pr_info("%s called!\n", __func__);

	info = oled_drv_data.func.get_mtp_info();
	remain = info.length;
	offset = info.offset;
	pr_info("%s rlen = %d, offset = %d!\n", __func__, rlen, offset);

	mtp_buf = kmalloc(remain, GFP_KERNEL);
	if (mtp_buf == NULL) {
		pr_err("mtp_buf NULL!\n");
		goto err2;
	}

	while (remain > 0)
	{
		if (remain >= step) {
			rlen = step;
			remain -= rlen;
		} else {
			rlen = remain;
			remain = 0;
		}

		if (offset > 0) {
			oled_drv_data.cmd_list.g_para_tx_cmd.cmds->payload[1] = offset;
			mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.g_para_tx_cmd);
		}

		ret = mdss_dsi_panel_oled_cmds_read(ctrl_pdata, oled_drv_data.cmd_list.read_mtp.cmds, rlen);
		if (ret != rlen) {
			pr_err("read mtp failed - rlen = %d, read = %d!\n", rlen, ret);
			goto err1;
		}
		memcpy(mtp_buf + buf_offset, ctrl_pdata->rx_buf.data, rlen);

		offset += rlen;
		buf_offset += rlen;
	}



	oled_drv_data.func.init_gamma_tbl(mtp_buf);

	kfree(mtp_buf);
	return 0;

err1:
	kfree(mtp_buf);
err2:
	return 1;
}

int mdss_dsi_panel_check_hbm_mtp(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mtp_info info;
	int rlen1 = 0;
	int rlen2 = 0;
	int offset1 = 0;
	int offset2 = 0;
	int ret = 0;
	char* hbm_mtp1_buf = NULL;
	char* hbm_mtp2_buf = NULL;

	pr_info("%s called!\n", __func__);

	info = oled_drv_data.func.get_hbm_mtp1_info();
	rlen1 = info.length;
	offset1 = info.offset;
	pr_info("%s rlen1 = %d, offset1= %d!\n", __func__, rlen1, offset1);
	if (rlen1 > 0) {
		hbm_mtp1_buf = kmalloc(rlen1, GFP_KERNEL);
		if (hbm_mtp1_buf == NULL) {
			pr_err("hbm_mtp1_buf NULL!\n");
			goto err2;
		}

		ret = mdss_dsi_panel_oled_cmds_read(ctrl_pdata,
					oled_drv_data.cmd_list.hbm_read_mtp.cmds, rlen1);
		if (ret != rlen1) {
			pr_err("read hbm mtp1 failed - rlen = %d, read = %d!\n", rlen1, ret);
			goto err1;
		}
		memcpy(hbm_mtp1_buf, ctrl_pdata->rx_buf.data, rlen1);
	}

	info = oled_drv_data.func.get_hbm_mtp2_info();
	rlen2 = info.length;
	offset2 = info.offset;
	pr_info("%s rlen2 = %d, offset2= %d!\n", __func__, rlen2, offset2);
	if (rlen2 > 0) {
		hbm_mtp2_buf = kmalloc(rlen2, GFP_KERNEL);
		if (hbm_mtp2_buf == NULL) {
			pr_err("hbm_mtp2_buf NULL!\n");
			goto err1;
		}

		ret = mdss_dsi_panel_oled_cmds_read(ctrl_pdata,
					oled_drv_data.cmd_list.hbm_read_mtp.cmds, rlen2);
		if (ret != rlen2) {
			pr_err("read hbm mtp2 failed - rlen = %d, read = %d!\n", rlen2, ret);
			goto err0;
		}
		memcpy(hbm_mtp2_buf, ctrl_pdata->rx_buf.data, rlen2);
	}

	oled_drv_data.func.init_hbm_gamma(&hbm_mtp1_buf[offset1], &hbm_mtp2_buf[offset2],
									oled_drv_data.cmd_list.hbm_gamma_tx_cmd.cmds->payload);

	kfree(hbm_mtp1_buf);
	kfree(hbm_mtp2_buf);
	return 0;

err0:
	kfree(hbm_mtp2_buf);
err1:
	kfree(hbm_mtp1_buf);
err2:
	return 1;
}

int mdss_dsi_panel_check_elvss_offset(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mtp_info info;
	int rlen = 0;
	int offset = 0;
	int ret = 0;
	char* mtp_buf = NULL;

	pr_info("%s called!\n", __func__);

	info = oled_drv_data.func.get_elvss_dim_offset_info();
	rlen = info.length;
	offset = info.offset;
	pr_info("%s rlen = %d, offset = %d!\n", __func__, rlen, offset);

	mtp_buf = kmalloc(rlen, GFP_KERNEL);
	if (mtp_buf == NULL) {
		pr_err("mtp_buf NULL!\n");
		goto err2;
	}

	if (offset > 0) {
		oled_drv_data.cmd_list.g_para_tx_cmd.cmds->payload[1] = offset;
		mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.g_para_tx_cmd);
	}

	ret = mdss_dsi_panel_oled_cmds_read(ctrl_pdata, oled_drv_data.cmd_list.elvss_rx_cmd.cmds, rlen);
	if (ret != rlen) {
		pr_err("read elvss offset failed - rlen = %d, read = %d!\n", rlen, ret);
		goto err1;
	}
	memcpy(mtp_buf, ctrl_pdata->rx_buf.data, rlen);
	oled_drv_data.func.set_elvss_dim_offset(mtp_buf);

	kfree(mtp_buf);
	return 0;

err1:
	kfree(mtp_buf);
err2:
	return 1;
}

int mdss_dsi_panel_check_chip_id(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int chip_id_len = 5;
	int ret = 0;

	pr_info("%s called!\n", __func__);


	ret = mdss_dsi_panel_oled_cmds_read(ctrl_pdata, oled_drv_data.cmd_list.chip_id_rx_cmd.cmds, chip_id_len);
	if (ret != chip_id_len) {
		pr_err("read chip id failed - chip_id_len = %d, read = %d!\n", chip_id_len, ret);
		goto err1;
	}
	memcpy(oled_drv_data.stat.chip_id_t, ctrl_pdata->rx_buf.data, chip_id_len);

	return 0;

err1:
	return 1;
}

static void merge_brightness_cmds(struct dsi_panel_cmds *dst, struct dsi_panel_cmds* src, char last)
{
	int i = 0;
	for (i = 0; i < src->cmd_cnt; i++) {
		dst->cmds[dst->cmd_cnt].dchdr.dtype = src->cmds[i].dchdr.dtype;
		dst->cmds[dst->cmd_cnt].dchdr.last = last;
		dst->cmds[dst->cmd_cnt].dchdr.vc = src->cmds[i].dchdr.vc;
		dst->cmds[dst->cmd_cnt].dchdr.ack = src->cmds[i].dchdr.ack;
		dst->cmds[dst->cmd_cnt].dchdr.wait = src->cmds[i].dchdr.wait;
		dst->cmds[dst->cmd_cnt].dchdr.dlen = src->cmds[i].dchdr.dlen;
		dst->cmds[dst->cmd_cnt].payload = src->cmds[i].payload;
		dst->cmd_cnt++;
	}
}
int mdss_dsi_panel_oled_set_elvss_offset(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int level)
{
	oled_drv_data.func.update_elvss_param1(oled_drv_data.stat.temp_stage, level, oled_drv_data.cmd_list.elvss_tx_temp_cmd.cmds[1].payload);
	oled_drv_data.func.update_elvss_param2(oled_drv_data.stat.temp_stage, level, oled_drv_data.cmd_list.elvss_tx_temp_cmd.cmds[3].payload);

	mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.elvss_tx_temp_cmd);

	return 0;
}

static int mdss_dsi_panel_oled_set_brightness(struct backlight_device *bl)
{
	struct dsi_panel_cmds merge;
	struct dsi_cmd_desc cmds_buf[20];
	struct dsi_panel_cmds aid;
	struct dsi_panel_cmds elvss;
	struct mdss_panel_data *pdata = oled_drv_data.pan_data;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int index = 0;
	int level = 50;

	merge.cmds = cmds_buf;
	merge.cmd_cnt = 0;
	merge.link_state = DSI_HS_MODE;

	level = bl->props.brightness;
	if (level < 0) {
		pr_err("%s: level[%d], abnormal!\n", __func__, level);
		return -1;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
								panel_data);
	if (ctrl_pdata->power == FB_BLANK_POWERDOWN) {
		pr_info("%s: skip brightness setting when lcd power off!\n", __func__);
		return 0;
	}

	if (oled_drv_data.stat.hbm_on) {
		if (oled_drv_data.stat.hbm_update) {
			merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.test_key_enable, 0);
			merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.hbm_tx_cmd, 0);
			merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.hbm_gamma_tx_cmd, 0);
			merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.test_key_disable, 1);
			mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &merge);
			oled_drv_data.stat.hbm_update = 0;
		}

		pr_info("%s: skip brightness setting when hbm on!\n", __func__);
		return 0;
	}

	if (level == MIN_BRIGHTNESS) {
		level = oled_drv_data.func.dimming_adjust(level, oled_drv_data.stat.current_level);
	}

	if (oled_drv_data.stat.current_level == level) {
		pr_info("%s: skip redundant brightness setting!\n", __func__);
		return 0;
	}
	oled_drv_data.stat.current_level = level;

	pr_info("%s: brightness = %d, need_check_mtp = %d, need_check_elvss_offset = %d, need_check_hbm_mtp = %d\n",
			__func__,  level,
			oled_drv_data.stat.need_check_mtp,
			oled_drv_data.stat.need_check_elvss_offset,
			oled_drv_data.stat.need_check_hbm_mtp);

	/* key on*/
	merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.test_key_enable, 0);

	if ((!oled_drv_data.stat.hbm_on) && (oled_drv_data.stat.hbm_update)){
		merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.hbm_off_cmd, 0);
		oled_drv_data.stat.hbm_update = 0;
		pr_info("%s: add hbm off cmd!\n", __func__);
	}

	index = oled_drv_data.func.get_aid_index(level);
	aid.cmds = &oled_drv_data.cmd_list.aid_cmd.cmds[index];
	aid.cmd_cnt = 1;
	aid.link_state = DSI_HS_MODE;
	pr_debug("%s: aid level[%d], max leve[%d]\n", __func__, index, oled_drv_data.cmd_list.aid_cmd.cmd_cnt);
	merge_brightness_cmds(&merge,  &aid, 0);

	index = oled_drv_data.func.get_elvss_index(level);
	elvss.cmds = &oled_drv_data.cmd_list.elvss_cmd.cmds[index];
	elvss.cmd_cnt = 1;
	elvss.link_state = DSI_HS_MODE;
	pr_debug("%s: elvss level[%d], max leve[%d]\n", __func__, index, oled_drv_data.cmd_list.elvss_cmd.cmd_cnt);
	merge_brightness_cmds(&merge,  &elvss, 0);

	/* make sure mtp have read */
	if (!oled_drv_data.stat.need_check_elvss_offset) {
		oled_drv_data.func.update_elvss_param1(oled_drv_data.stat.temp_stage, level, oled_drv_data.cmd_list.elvss_tx_temp_cmd.cmds[1].payload);
		oled_drv_data.func.update_elvss_param2(oled_drv_data.stat.temp_stage, level, oled_drv_data.cmd_list.elvss_tx_temp_cmd.cmds[3].payload);
		merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.elvss_tx_temp_cmd, 0);
	}

	if (level == MAX_BRIGHTNESS) {
		merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.acl_off, 0);
	} else {
		merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.acl_on, 0);
	}

	/* make sure mtp have read */
	if (!oled_drv_data.stat.need_check_mtp) {
		oled_drv_data.func.update_gamma_tbl(level, oled_drv_data.cmd_list.gamma_update.cmds->payload);
	}
	merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.gamma_update, 0);

	/* key off*/
	merge_brightness_cmds(&merge,  &oled_drv_data.cmd_list.test_key_disable, 1);

	mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &merge);
	return 0;
}

static int mdss_dsi_panel_oled_get_brightness(struct backlight_device *bl)
{
	pr_debug("%s\n", __func__);
	return bl->props.brightness;
}

static ssize_t panel_elvss_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", oled_drv_data.stat.temp_stage);
}

static ssize_t panel_elvss_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct mdss_panel_data *pdata = oled_drv_data.pan_data;
	struct backlight_device *bl = oled_drv_data.bd;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	unsigned long value;
	int rc = 0;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
								panel_data);

	rc = kstrtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;

	pr_info("%s:value = %lu.\n", __func__, value);

	if (value > TEMP_RANGE_2) {
		pr_warn("%s:unsupported temp range.\n", __func__);
		return -EPERM;
	}

	oled_drv_data.stat.temp_stage = value;

	if (ctrl_pdata->power == FB_BLANK_POWERDOWN) {
		pr_warn("%s:elvss control before lcd enable.\n", __func__);
		return -EPERM;
	}

	mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.test_key_enable);
	mdss_dsi_panel_oled_set_elvss_offset(ctrl_pdata, bl->props.brightness);
	mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.test_key_disable);

	return size;
}

static ssize_t panel_chip_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char temp[20];

	sprintf(temp, "%02x%02x%02x%02x%02x\n",
				oled_drv_data.stat.chip_id_t[0],
				oled_drv_data.stat.chip_id_t[1],
				oled_drv_data.stat.chip_id_t[2],
				oled_drv_data.stat.chip_id_t[3],
				oled_drv_data.stat.chip_id_t[4]);
	strcat(buf, temp);

	return strlen(buf);
}


static ssize_t panel_hbm_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", oled_drv_data.stat.hbm_on ? "on" : "off");
}

static ssize_t panel_hbm_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct backlight_device *bl = oled_drv_data.bd;

	if (!strncmp(buf, "on", 2))
		oled_drv_data.stat.hbm_on = 1;
	else if (!strncmp(buf, "off", 3))
		oled_drv_data.stat.hbm_on = 0;
	else {
		pr_err("%s, invalid comman (use on or off)d.\n", __func__);
		return size;
	}

	oled_drv_data.stat.hbm_update = 1;
	oled_drv_data.stat.current_level = -1;
	mdss_dsi_panel_oled_set_brightness(bl);
	return size;
}

static const struct backlight_ops oled_backlight_ops = {
	.update_status = mdss_dsi_panel_oled_set_brightness,
	.get_brightness = mdss_dsi_panel_oled_get_brightness,
};

static struct device_attribute dev_attrs[] = {
	__ATTR(chip_id, S_IRUGO, panel_chip_id_show, NULL),
	__ATTR(hbm, S_IRUGO | S_IWUSR, panel_hbm_show, panel_hbm_store),
	__ATTR(elvss, S_IRUGO | S_IWUSR, panel_elvss_show, panel_elvss_store),
};

static int mdss_dsi_panel_oled_cmds_read(struct mdss_dsi_ctrl_pdata *ctrl,
		struct dsi_cmd_desc *cmd, int rlen)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rbuf = ctrl->rx_buf.data;
	cmdreq.rlen = rlen;
	cmdreq.cb = NULL; /* call back */
	/*
	 * This mutex is to sync up with dynamic FPS changes
	 * so that DSI lockups shall not happen
	 */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	/*
	 * blocked here, untill call back called
	 */
	return ctrl->rx_buf.len;
}

static void mdss_dsi_panel_oled_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_oled_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}

static void mdss_panel_oled_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int ret = 0;

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.chip_id_rx_cmd,
		"samsung,chip_id_rx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,chip_id_rx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.aid_cmd,
		"samsung,aid_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,aid_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.acl_on,
		"samsung,acl_on_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,acl_on_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.acl_off,
		"samsung,acl_off_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,acl_off_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.gamma_update,
		"samsung,gamma_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,gamma_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.elvss_cmd,
		"samsung,elvss_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,elvss_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.test_key_enable,
		"samsung,test_key_enable_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse test_key_enable_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.test_key_disable,
		"samsung,test_key_disable_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,test_key_disable_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.read_mtp,
		"samsung,mtp_rx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,mtp_rx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.hbm_tx_cmd,
		"samsung,hbm_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,hbm_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.hbm_gamma_tx_cmd,
		"samsung,hbm_gamma_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,hbm_gamma_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.hbm_off_cmd,
		"samsung,hbm_off_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,hbm_off_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.hbm_read_mtp,
		"samsung,hbm_rx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,hbm_rx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.g_para_tx_cmd,
		"samsung,g_param_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,g_param_tx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.elvss_rx_cmd,
		"samsung,elvss_rx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,elvss_rx_cmds failed!\n", __func__);
	}

	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.elvss_tx_temp_cmd,
		"samsung,elvss_temp_tx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,samsung,elvss_temp_tx_cmds!\n", __func__);
	}

#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
	ret = mdss_dsi_oled_parse_dcs_cmds(np, &oled_drv_data.cmd_list.mdnie_rx_cmd,
		"samsung,mdnie_rx_cmds", NULL);
	if (ret != 0) {
		pr_err("%s parse samsung,samsung,elvss_temp_tx_cmds!\n", __func__);
	}
#endif
}

#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
unsigned char color_offset_t[4] = {0};
int mdss_dsi_panel_check_mdnie_coordinate(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mtp_info info;
	int rlen = 0;
	int offset = 0;
	int ret = 0;

	pr_info("%s called!\n", __func__);

	info = oled_drv_data.func.get_mdnie_coordinate_info();
	rlen = info.length;
	offset = info.offset;
	pr_info("%s rlen = %d, offset = %d!\n", __func__, rlen, offset);


	if (offset > 0) {
		oled_drv_data.cmd_list.g_para_tx_cmd.cmds->payload[1] = offset;
		mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.g_para_tx_cmd);
	}

	ret = mdss_dsi_panel_oled_cmds_read(ctrl_pdata, oled_drv_data.cmd_list.mdnie_rx_cmd.cmds, rlen);
	if (ret != rlen) {
		pr_err("read mdnie offset failed - rlen = %d, read = %d!\n", rlen, ret);
		goto err1;
	}
	memcpy(color_offset_t, ctrl_pdata->rx_buf.data, rlen);

	pr_info("mdnie color_offset_t = 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
		color_offset_t[0], color_offset_t[1], color_offset_t[2], color_offset_t[3]);

	return 0;

err1:
	return 1;
}

int mdss_dsi_panel_update_mdnie(void *pdata, struct mdnie_command *seq, u32 len)
{
	struct dsi_panel_cmds mdnie_cmds;
	struct dsi_cmd_desc cmds_buf[10];
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int i = 0;
	int j = 0;

	ctrl_pdata = (struct mdss_dsi_ctrl_pdata *) pdata;

	mdnie_cmds.link_state = DSI_LP_MODE;
	mdnie_cmds.cmd_cnt = 0;
	mdnie_cmds.cmds = cmds_buf;

	for (i = 0; i < len; i++) {
		for (j = 0; j < seq[i].size; j++) {
			mdnie_cmds.cmds[mdnie_cmds.cmd_cnt].dchdr.dtype = seq[i].sequence[j].dchdr.dtype;
			mdnie_cmds.cmds[mdnie_cmds.cmd_cnt].dchdr.last = seq[i].sequence[j].dchdr.last;
			mdnie_cmds.cmds[mdnie_cmds.cmd_cnt].dchdr.vc = seq[i].sequence[j].dchdr.vc;
			mdnie_cmds.cmds[mdnie_cmds.cmd_cnt].dchdr.ack = seq[i].sequence[j].dchdr.ack;
			mdnie_cmds.cmds[mdnie_cmds.cmd_cnt].dchdr.wait = seq[i].sequence[j].dchdr.wait;
			mdnie_cmds.cmds[mdnie_cmds.cmd_cnt].dchdr.dlen = seq[i].sequence[j].dchdr.dlen;
			mdnie_cmds.cmds[mdnie_cmds.cmd_cnt].payload = seq[i].sequence[j].payload;

			mdnie_cmds.cmd_cnt++;
		}
	}

	mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &mdnie_cmds);

	return 0;
}

int mdnie_get_color_coordinates(void *pdata, uint32_t *co_ord)
{
	if (co_ord) {
		co_ord[0] = color_offset_t[0] << 8 | color_offset_t[1]; /* X */
		co_ord[1] = color_offset_t[2] << 8 | color_offset_t[3]; /* Y */
		pr_info("mdnie: X = %d, Y = %d\n", co_ord[0], co_ord[1]);
	} else {
		pr_err("%s : wrong input param co_ord\n", __func__);
	}

	return 0;
}
#endif

struct mdss_panel_oled_driver_data* mdss_dsi_panel_oled_get_odd(void)
{
	return &oled_drv_data;
}

int mdss_dsi_panel_oled_init_mtp(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int i;

	/* key on*/
	mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.test_key_enable);

	if (oled_drv_data.stat.need_check_chip_id) {
		for (i = 0; i < 3 && oled_drv_data.stat.need_check_chip_id; i++) {
			oled_drv_data.stat.need_check_chip_id = mdss_dsi_panel_check_chip_id(ctrl_pdata);
		}
	}
	if (oled_drv_data.stat.need_check_chip_id) {
		pr_err("%s, read chip id failed!\n", __func__);
	}

	if (oled_drv_data.stat.need_check_mtp) {
		for (i = 0; i < 3 && oled_drv_data.stat.need_check_mtp; i++) {
			oled_drv_data.stat.need_check_mtp = mdss_dsi_panel_check_mtp(ctrl_pdata);
		}
	}
	if (oled_drv_data.stat.need_check_mtp) {
		pr_err("%s, mtp check failed!\n", __func__);
	}

	if (oled_drv_data.stat.need_check_hbm_mtp) {
		for (i = 0; i < 3 && oled_drv_data.stat.need_check_hbm_mtp; i++) {
			oled_drv_data.stat.need_check_hbm_mtp = mdss_dsi_panel_check_hbm_mtp(ctrl_pdata);
		}
	}
	if (oled_drv_data.stat.need_check_hbm_mtp) {
		pr_err("%s, hbm mtp check failed!\n", __func__);
	}

	if (oled_drv_data.stat.need_check_elvss_offset) {
		for (i = 0; i < 3 && oled_drv_data.stat.need_check_elvss_offset; i++) {
			oled_drv_data.stat.need_check_elvss_offset = mdss_dsi_panel_check_elvss_offset(ctrl_pdata);
		}
	}
	if (oled_drv_data.stat.need_check_elvss_offset) {
		pr_err("%s, elvss offset check failed!\n", __func__);
	}

#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
	if (oled_drv_data.stat.need_check_mdnie_coordinate) {
		for (i = 0; i < 3 && oled_drv_data.stat.need_check_mdnie_coordinate; i++) {
			oled_drv_data.stat.need_check_mdnie_coordinate = mdss_dsi_panel_check_mdnie_coordinate(ctrl_pdata);
		}
	}
	if (oled_drv_data.stat.need_check_mdnie_coordinate) {
		pr_err("%s, mdnie coordinate check failed!\n", __func__);
	}
#endif

	/* key off*/
	mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.test_key_disable);

	return 0;
}

int mdss_dsi_panel_oled_panel_on(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	oled_drv_data.stat.current_level = -1;

	if (oled_drv_data.stat.hbm_on) {
		pr_info("%s, hbm on\n", __func__);

		mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.test_key_enable);
		mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.hbm_tx_cmd);
		mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.hbm_gamma_tx_cmd);
		mdss_dsi_panel_oled_cmds_send(ctrl_pdata, &oled_drv_data.cmd_list.test_key_disable);
	}

	return 0;
}

int mdss_dsi_panel_oled_init(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata, struct device *parent,
			struct device *ld_dev)
{
	struct backlight_properties props = {0};
	struct backlight_device * bl = NULL;
	int ret = 0;
	int i = 0;

	pr_info("%s\n", __func__);

	oled_drv_data.stat.need_check_chip_id = 1;
	oled_drv_data.stat.need_check_mtp = 1;
	oled_drv_data.stat.need_check_hbm_mtp= 1;
	oled_drv_data.stat.need_check_elvss_offset = 1;
#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
	oled_drv_data.stat.need_check_mdnie_coordinate = 1;
#endif
	oled_drv_data.stat.hbm_on = 0;
	oled_drv_data.stat.hbm_update = 0;
	oled_drv_data.stat.temp_stage = TEMP_RANGE_0;
	oled_drv_data.stat.current_level = DEFAULT_BRIGHTNESS;
	oled_drv_data.pan_data = &ctrl_pdata->panel_data;

	if (oled_drv_data.func.probe) {
		ret = oled_drv_data.func.probe(&oled_drv_data);
		if (ret)
			goto error_exit;
	} else {
		pr_err("panel probe func NULL!\n");
		goto error_exit;
	}

	mdss_panel_oled_parse_dt(np, ctrl_pdata);

	props.max_brightness = MAX_BRIGHTNESS;
	props.type = BACKLIGHT_PLATFORM;

	bl = backlight_device_register("oled-bd", parent, &oled_drv_data,
					   &oled_backlight_ops, &props);
	if (IS_ERR(bl)) {
		pr_err("failed to register backlight for oled bd!\n");
		ret = PTR_ERR(bl);

		goto error_exit;
	}

	bl->props.max_brightness = MAX_BRIGHTNESS;
	bl->props.brightness = DEFAULT_BRIGHTNESS;
	oled_drv_data.bd = bl;
	dev_set_drvdata(parent, &oled_drv_data);

	for (i = 0; i < ARRAY_SIZE(dev_attrs); i++) {
		ret = device_create_file(ld_dev, &dev_attrs[i]);
		if (ret < 0) {
			pr_err("%s, failed to add dev sysfs entries\n", __func__);
			for (i--; i >= 0; i--)
				device_remove_file(ld_dev, &dev_attrs[i]);
		}
	}

#if defined (CONFIG_MDSS_MDNIE_LITE_TUNING)
	mdss_mdnie_register(ctrl_pdata, mdss_dsi_panel_update_mdnie, NULL, mdnie_get_color_coordinates);
#endif

	return ret;

 error_exit:
	if (bl)
		backlight_device_unregister(bl);

	return ret;
}

