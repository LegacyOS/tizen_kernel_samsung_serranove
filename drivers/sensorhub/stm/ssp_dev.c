/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include "ssp_dev.h"

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
static int ssp_check_lpmode(void)
{
#ifdef CONFIG_SAMSUNG_LPM_MODE
	if (poweroff_charging == 0x01)
		return true;
	else
		return false;
#else
	return false;
#endif
}
#endif

void ssp_enable(struct ssp_data *data, bool enable)
{
	ssp_infof("enable = %d, old enable = %d",
		enable, data->bSspShutdown);

	if (enable && data->bSspShutdown) {
		data->bSspShutdown = false;
		enable_irq(data->iIrq);
		enable_irq_wake(data->iIrq);
	} else if (!enable && !data->bSspShutdown) {
		data->bSspShutdown = true;
		disable_irq(data->iIrq);
		disable_irq_wake(data->iIrq);
	} else
		ssp_errf("enable error");
}
/************************************************************************/
/* interrupt happened due to transition/change of SSP MCU		*/
/************************************************************************/

static irqreturn_t sensordata_irq_thread_fn(int iIrq, void *dev_id)
{
	struct ssp_data *data = dev_id;
	struct timespec ts;

	ts = ktime_to_timespec(ktime_get_boottime());
	data->timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

	if (gpio_get_value(data->mcu_int1)) {
		ssp_info("MCU int HIGH");
		return IRQ_HANDLED;
	}
	select_irq_msg(data);
	data->uIrqCnt++;

	return IRQ_HANDLED;
}

/*************************************************************************/
/* initialize sensor hub						 */
/*************************************************************************/

static void initialize_variable(struct ssp_data *data)
{
	int iSensorIndex;

	for (iSensorIndex = 0; iSensorIndex < SENSOR_MAX; iSensorIndex++) {
		data->adDelayBuf[iSensorIndex] = DEFUALT_POLLING_DELAY;
		data->aiCheckStatus[iSensorIndex] = INITIALIZATION_STATE;
	}
	data->adDelayBuf[BIO_HRM_LIB] = (100 * NSEC_PER_MSEC);
	data->uSensorState = NORMAL_SENSOR_STATE_K;
	data->uMagCntlRegData = 1;

	data->bSspShutdown = true;
	data->bTimeSyncing = true;

	data->buf[GYROSCOPE_SENSOR].gyro_dps = GYROSCOPE_DPS2000;
	data->uProxHiThresh = data->uProxHiThresh_default;
	data->uProxLoThresh = data->uProxLoThresh_default;
	data->uIr_Current = DEFUALT_IR_CURRENT;

	data->pir_standard_threshold = 50;
	data->pir_long_threshold = 50;

#if CONFIG_SEC_DEBUG
	data->bMcuDumpMode = sec_debug_is_enabled();
#endif
	INIT_LIST_HEAD(&data->pending_list);

	initialize_function_pointer(data);
}

int initialize_mcu(struct ssp_data *data)
{
	int iRet = 0;

	clean_pending_list(data);

	iRet = get_chipid(data);
	ssp_info("MCU device ID = %d, reading ID = %d", DEVICE_ID, iRet);
	if (iRet != DEVICE_ID) {
		if (iRet < 0) {
			ssp_errf("MCU is not working : 0x%x", iRet);
		} else {
			ssp_errf("MCU identification failed");
			iRet = -ENODEV;
		}
		goto out;
	}

	iRet = set_sensor_position(data);
	if (iRet < 0) {
		ssp_errf("set_sensor_position failed");
		goto out;
	}

	data->uSensorState = get_sensor_scanning_info(data);
	if (data->uSensorState == 0) {
		ssp_errf("get_sensor_scanning_info failed");
		iRet = ERROR;
		goto out;
	}

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	iRet = initialize_magnetic_sensor(data);
	if (iRet < 0)
		ssp_errf("initialize magnetic sensor failed");
#endif

	data->uCurFirmRev = get_firmware_rev(data);
	ssp_info("MCU Firm Rev : New = %8u", data->uCurFirmRev);

out:
	return iRet;
}

static int initialize_irq(struct ssp_data *data)
{
	int iRet, iIrq;
	iIrq = gpio_to_irq(data->mcu_int1);

	ssp_info("requesting IRQ %d", iIrq);
	iRet = request_threaded_irq(iIrq, NULL, sensordata_irq_thread_fn,
			    IRQF_TRIGGER_FALLING|IRQF_ONESHOT, "SSP_Int", data);
	if (iRet < 0) {
		ssp_errf("request_irq(%d) failed for gpio %d (%d)",
		       iIrq, iIrq, iRet);
		goto err_request_irq;
	}

	/* start with interrupts disabled */
	data->iIrq = iIrq;
	disable_irq(data->iIrq);
	return 0;

err_request_irq:
	gpio_free(data->mcu_int1);
	return iRet;
}

static void work_function_firmware_update(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
			struct ssp_data, work_firmware);
	int iRet;

	ssp_infof();

	iRet = forced_to_download_binary(data, KERNEL_BINARY);
	if (iRet < 0) {
		ssp_infof("forced_to_download_binary failed!");
		data->uSensorState = 0;
		return;
	}

	queue_refresh_task(data, SSP_SW_RESET_TIME);

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	if (data->check_lpmode() == true) {
		data->bLpModeEnabled = true;
		ssp_info("LPM Charging...");
	} else {
		data->bLpModeEnabled = false;
		ssp_info("Normal Booting OK");
	}
#endif

	ssp_info("done");
}

#ifdef CONFIG_OF
static int ssp_parse_dt(struct device *dev,
		struct  ssp_data *data)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int errorno = 0;

	/* gpio pins */
	data->mcu_int1 = of_get_named_gpio_flags(np, "ssp,mcu_int1-gpio",
		0, &flags);
	if (data->mcu_int1 < 0) {
		ssp_err("could not get mcu_int1-gpio");
		errorno = data->mcu_int1;
		goto dt_exit;
	}

	data->mcu_int2 = of_get_named_gpio_flags(np, "ssp,mcu_int2-gpio",
		0, &flags);
	if (data->mcu_int2 < 0) {
		ssp_err("could not get mcu_int2-gpio");
		errorno = data->mcu_int2;
		goto dt_exit;
	}

	data->ap_int = of_get_named_gpio_flags(np, "ssp,ap_int-gpio",
		0, &flags);
	if (data->ap_int < 0) {
		ssp_err("could not get ap_int-gpio");
		errorno = data->ap_int;
		goto dt_exit;
	}

	data->rst = of_get_named_gpio_flags(np, "ssp,rst-gpio",
		0, &flags);
	if (data->rst < 0) {
		ssp_err("could not get rst-gpio");
		errorno = data->rst ;
		goto dt_exit;
	}

	if (of_property_read_u32(np, "ssp,ap-rev", &data->ap_rev))
		data->ap_rev = 0;

	/* sensor positions */
#ifdef CONFIG_SENSORS_SSP_ACCELEROMETER_SENSOR
	if (of_property_read_u32(np, "ssp,acc-position", &data->accel_position))
		data->accel_position = 0;
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	if (of_property_read_u32(np, "ssp,mag-position", &data->mag_position))
		data->mag_position = 0;
#endif
	ssp_info("acc-posi[%d] mag-posi[%d]",
			data->accel_position, data->mag_position);

#ifdef CONFIG_SENSORS_SSP_PROXIMITY_SENSOR
	/* prox thresh */
	if (of_property_read_u32(np, "ssp,prox_hi_threshold",
		&data->uProxHiThresh_default)) {
		ssp_errf("can not find prox_hi_threshold value");
		data->uProxHiThresh_default = DEFUALT_HIGH_THRESHOLD;
	}
	if (of_property_read_u32(np, "ssp,prox_lo_threshold",
		&data->uProxLoThresh_default)) {
		ssp_errf("can not find prox_lo_threshold value");
		data->uProxLoThresh_default = DEFUALT_LOW_THRESHOLD;
	}
	ssp_info("hi-thresh[%u] low-thresh[%u]",
		data->uProxHiThresh_default, data->uProxLoThresh_default);
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC_SENSOR
	/* mag matrix */
	if (of_property_read_u8_array(np, "ssp,mag-array",
		data->pdc_matrix, sizeof(data->pdc_matrix))) {
		ssp_err("no mag-array, set as 0");
	}
#endif

	/* firmware name */
	if (of_property_read_string(np, "ssp,fw_name", &data->fw_name))
		ssp_err("fail to get fw name");

	if (of_property_read_string(np, "ssp,ums_fw_name", &data->ums_fw_name))
		ssp_err("fail to get ums_fw name");

	/* set off gpio pins */
	errorno = gpio_request(data->mcu_int1, "mcu_ap_int1");
	if (errorno) {
		ssp_err("failed to request MCU_INT1 for SSP");
		goto dt_exit;
	}
	errorno = gpio_direction_input(data->mcu_int1);
	if (errorno) {
		ssp_err("failed to set mcu_int1 as input");
		goto dt_exit;
	}
	errorno = gpio_request(data->mcu_int2, "MCU_INT2");
	if (errorno) {
		ssp_err("failed to request MCU_INT2 for SSP");
		goto dt_exit;
	}
	gpio_direction_input(data->mcu_int2);

	errorno = gpio_request(data->ap_int, "AP_MCU_INT");
	if (errorno) {
		ssp_err("failed to request AP_INT for SSP");
		goto dt_exit;
	}
	gpio_direction_output(data->ap_int, 1);

	errorno = gpio_request(data->rst, "MCU_RST");
	if (errorno) {
		ssp_err("failed to request MCU_RST for SSP");
		goto dt_exit;
	}
	gpio_direction_output(data->rst, 1);

dt_exit:
	return errorno;
}
#else
static int ssp_parse_dt(struct device *dev,
		struct  ssp_data *data)
{
	return -ENODEV;
}
#endif

static int ssp_suspend(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	ssp_infof();
	data->uLastResumeState = MSG2SSP_AP_STATUS_SUSPEND;
	disable_debug_timer(data);

	if (SUCCESS != ssp_send_cmd(data, MSG2SSP_AP_STATUS_SUSPEND, 0))
		ssp_errf("MSG2SSP_AP_STATUS_SUSPEND failed");

	data->bTimeSyncing = false;
	disable_irq_nosync(data->iIrq);
	return 0;
}

static int ssp_resume(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	enable_irq(data->iIrq);
	ssp_infof();
	enable_debug_timer(data);

	if (SUCCESS != ssp_send_cmd(data, MSG2SSP_AP_STATUS_RESUME, 0))
		ssp_errf("MSG2SSP_AP_STATUS_RESUME failed");

	data->uLastResumeState = MSG2SSP_AP_STATUS_RESUME;

	return 0;
}

static const struct dev_pm_ops ssp_pm_ops = {
	.suspend = ssp_suspend,
	.resume = ssp_resume
};

static int ssp_probe(struct spi_device *spi)
{
	int iRet = 0;
	struct ssp_data *data;
	struct ssp_platform_data *pdata = NULL;

	ssp_infof();

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ssp_errf("failed to allocate memory for data");
		iRet = -ENOMEM;
		goto exit;
	}

	if (spi->dev.of_node) {
		iRet = ssp_parse_dt(&spi->dev, data);
		if (iRet) {
			ssp_errf("Failed to parse DT");
			goto err_setup;
		}
	} else {
		pdata = spi->dev.platform_data;
		if (pdata == NULL) {
			ssp_errf("failed to get device node");
			iRet = -ENODEV;
			goto err_setup;
		}

		data->set_mcu_reset = pdata->set_mcu_reset;
		data->rst = pdata->rst;
		data->ap_int = pdata->ap_int;
		data->mcu_int1 = pdata->mcu_int1;
		data->mcu_int2 = pdata->mcu_int2;
		data->iIrq = pdata->irq;
#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
		data->check_lpmode = pdata->check_lpmode;
#endif
#ifdef CONFIG_SENSORS_SSP_HRM_SENSOR
		data->hrm_sensor_power = pdata->hrm_sensor_power;
#endif
		/* Get sensor positions */
		if (pdata->get_positions) {
			pdata->get_positions(&data->accel_position,
					&data->mag_position);
		} else {
			data->accel_position = 0;
			data->mag_position = 0;
		}
		if (pdata->mag_matrix) {
			data->mag_matrix_size = pdata->mag_matrix_size;
			data->mag_matrix = pdata->mag_matrix;
		}

		if (data->set_mcu_reset == NULL) {
			ssp_errf("function callback is null");
			iRet = -EIO;
			goto err_setup;
		}
	}

	msleep(1500);
	ssp_info("rst = %d, ap_int = %d, mcu_int1 = %d, mcu_int2 = %d",
			(int)data->rst, (int)data->ap_int,
			(int)data->mcu_int1, (int)data->mcu_int2);
 	ssp_info("system Rev = 0x%x", data->ap_rev);

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	data->check_lpmode = ssp_check_lpmode;
#endif

	spi->mode = SPI_MODE_1;
	if (spi_setup(spi)) {
		ssp_errf("failed to setup spi");
		iRet = -ENODEV;
		goto err_setup;
	}

	data->fw_dl_state = FW_DL_STATE_NONE;
	data->spi = spi;
	spi_set_drvdata(spi, data);

#ifdef CONFIG_SENSORS_SSP_SHTC1
	mutex_init(&data->cp_temp_adc_lock);
	mutex_init(&data->bulk_temp_read_lock);
#endif
	mutex_init(&data->comm_mutex);
	mutex_init(&data->pending_mutex);

	pr_info("\n#####################################################\n");

	INIT_DELAYED_WORK(&data->work_firmware, work_function_firmware_update);
	INIT_DELAYED_WORK(&data->work_refresh, refresh_task);

	wake_lock_init(&data->ssp_wake_lock,
			WAKE_LOCK_SUSPEND, "ssp_wake_lock");

	iRet = initialize_input_dev(data);
	if (iRet < 0) {
		ssp_errf("could not create input device");
		goto err_input_register_device;
	}

	iRet = initialize_debug_timer(data);
	if (iRet < 0) {
		ssp_errf("could not create workqueue");
		goto err_create_workqueue;
	}

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	iRet = intialize_lpm_motion(data);
	if (iRet < 0) {
		ssp_errf("could not create workqueue");
		goto err_create_lpm_motion;
	}
#endif

	iRet = initialize_irq(data);
	if (iRet < 0) {
		ssp_errf("could not create irq");
		goto err_setup_irq;
	}

	iRet = initialize_sysfs(data);
	if (iRet < 0) {
		ssp_errf("could not create sysfs");
		goto err_sysfs_create;
	}

	initialize_variable(data);

	/* init sensorhub device */
	iRet = ssp_sensorhub_initialize(data);
	if (iRet < 0) {
		ssp_errf("ssp_sensorhub_initialize err(%d)", iRet);
		ssp_sensorhub_remove(data);
	}

	ssp_enable(data, true);
	/* check boot loader binary */
	data->fw_dl_state = check_fwbl(data);

	if (data->fw_dl_state == FW_DL_STATE_NONE) {
		iRet = initialize_mcu(data);
		if (iRet == ERROR) {
			toggle_mcu_reset(data);
		} else if (iRet < ERROR) {
			ssp_errf("initialize_mcu failed");
			goto err_read_reg;
		}
	}

	ssp_infof("probe success!");

	enable_debug_timer(data);

	if (data->fw_dl_state == FW_DL_STATE_NEED_TO_SCHEDULE) {
		ssp_info("Firmware update is scheduled");
		schedule_delayed_work(&data->work_firmware,
				msecs_to_jiffies(1000));
		data->fw_dl_state = FW_DL_STATE_SCHEDULED;
	} else if (data->fw_dl_state == FW_DL_STATE_FAIL) {
		data->bSspShutdown = true;
	}
	data->bProbeIsDone = true;
	iRet = 0;

#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	if (data->check_lpmode() == true) {
		ssp_charging_motion(data, 1);
		data->bLpModeEnabled = true;
		ssp_info("LPM Charging...");
	} else {
		data->bLpModeEnabled = false;
		ssp_info("Normal Booting OK");
	}
#endif

	goto exit;

err_read_reg:
	remove_sysfs(data);
err_sysfs_create:
	free_irq(data->iIrq, data);
	gpio_free(data->mcu_int1);
err_setup_irq:
#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	destroy_workqueue(data->lpm_motion_wq);
err_create_lpm_motion:
#endif
	destroy_workqueue(data->debug_wq);
err_create_workqueue:
#ifdef SSP_USE_IIO_BATCH
	remove_indio_dev(data);
#else
	remove_input_dev(data);
#endif
err_input_register_device:
	wake_lock_destroy(&data->ssp_wake_lock);
	cancel_delayed_work_sync(&data->work_firmware);
	cancel_delayed_work_sync(&data->work_refresh);
#ifdef CONFIG_SENSORS_SSP_TEMP_HUMID_SENSOR
	mutex_destroy(&data->cp_temp_adc_lock);
	mutex_destroy(&data->bulk_temp_read_lock);
#endif
	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);
err_setup:
	kfree(data);
	ssp_errf("probe failed!");
exit:
	pr_info("#####################################################\n\n");
	return iRet;
}

static void ssp_shutdown(struct spi_device *spi_dev)
{
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	ssp_infof();
	atomic_set(&data->apShutdownProgress, 1);
	if (data->bProbeIsDone == false)
		goto exit;

	disable_debug_timer(data);

	if (data->fw_dl_state >= FW_DL_STATE_SCHEDULED &&
			data->fw_dl_state < FW_DL_STATE_DONE) {
		ssp_errf("cancel_delayed_work_sync state = %d",
			data->fw_dl_state);
		cancel_delayed_work_sync(&data->work_firmware);
	}

	if (SUCCESS != ssp_send_cmd(data, MSG2SSP_AP_STATUS_SHUTDOWN, 0))
		ssp_errf("MSG2SSP_AP_STATUS_SHUTDOWN failed");

	ssp_enable(data, false);
	clean_pending_list(data);

	free_irq(data->iIrq, data);
	gpio_free(data->mcu_int1);

	remove_sysfs(data);

	ssp_sensorhub_remove(data);

	del_timer_sync(&data->debug_timer);
	cancel_work_sync(&data->work_debug);
	cancel_delayed_work_sync(&data->work_refresh);
#ifdef CONFIG_SENSORS_SSP_LPM_MOTION
	cancel_work_sync(&data->work_lpm_motion);
	destroy_workqueue(data->lpm_motion_wq);
#endif
	destroy_workqueue(data->debug_wq);
	wake_lock_destroy(&data->ssp_wake_lock);
#ifdef CONFIG_SENSORS_SSP_SHTC1
	mutex_destroy(&data->cp_temp_adc_lock);
	mutex_destroy(&data->bulk_temp_read_lock);
#endif
#ifdef CONFIG_SENSORS_SSP_STM
	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);
#endif
	toggle_mcu_reset(data);
	ssp_infof("done");
exit:
	kfree(data);
}

static const struct spi_device_id ssp_id[] = {
	{"ssp", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, ssp_id);

#ifdef CONFIG_OF
static struct of_device_id ssp_match_table[] = {
	{ .compatible = "ssp,STM32F",},
	{},
};
#else
define ssp_match_table NULL
#endif

static struct spi_driver ssp_driver = {
	.probe = ssp_probe,
	.shutdown = ssp_shutdown,
	.id_table = ssp_id,
	.driver = {
		.pm = &ssp_pm_ops,
		.owner = THIS_MODULE,
		.name = "ssp",
#ifdef CONFIG_OF
		.of_match_table = ssp_match_table
#endif
	},
};

module_spi_driver(ssp_driver);
MODULE_DESCRIPTION("Seamless Sensor Platform(SSP) dev driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
