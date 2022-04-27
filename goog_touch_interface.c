// SPDX-License-Identifier: GPL-2.0
/*
 * Google Touch Interface for Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <samsung/exynos_drm_connector.h>

#include "goog_touch_interface.h"
#include "../../../gs-google/drivers/soc/google/vh/kernel/systrace.h"

static struct class *gti_class;
static u8 gti_dev_num;

static ssize_t offload_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t offload_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t v4l2_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size);
static ssize_t v4l2_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static DEVICE_ATTR_RW(offload_enable);
static DEVICE_ATTR_RW(v4l2_enable);

static struct attribute *goog_attributes[] = {
	&dev_attr_offload_enable.attr,
	&dev_attr_v4l2_enable.attr,
	NULL,
};

static struct attribute_group goog_attr_group = {
	.attrs = goog_attributes,
};

static ssize_t offload_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (kstrtobool(buf, &gti->offload_enable))
		GOOG_ERR("invalid input!\n");
	else
		GOOG_LOG("offload_enable= %d.\n", gti->offload_enable);
	return size;
}

static ssize_t offload_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	size += scnprintf(buf, PAGE_SIZE, "offload_enable= %d.\n",
			gti->offload_enable);
	GOOG_LOG("%s", buf);
	return size;
}

static ssize_t v4l2_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	if (kstrtobool(buf, &gti->v4l2_enable))
		GOOG_ERR("invalid input!\n");
	else
		GOOG_LOG("v4l2_enable= %d.\n", gti->v4l2_enable);
	return size;
}

static ssize_t v4l2_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t size = 0;
	struct goog_touch_interface *gti = dev_get_drvdata(dev);

	size += scnprintf(buf, PAGE_SIZE, "v4l2_enable= %d.\n",
			gti->v4l2_enable);
	GOOG_LOG("%s", buf);
	return size;
}

static void panel_bridge_enable(struct drm_bridge *bridge)
{
	struct goog_touch_interface *gti =
		container_of(bridge, struct goog_touch_interface, panel_bridge);

	if (gti->panel_is_lp_mode) {
		GOOG_LOG("skip screen-on because of panel_is_lp_mode enabled!\n");
	} else {
		GOOG_LOG("screen-on.\n");
		gti->vendor_cb(gti->vendor_private_data,
			GTI_CMD_NOTIFY_DISPLAY_STATE, GTI_SUB_CMD_DISPLAY_STATE_ON, NULL, NULL);
	}
}

static void panel_bridge_disable(struct drm_bridge *bridge)
{
	struct goog_touch_interface *gti =
		container_of(bridge, struct goog_touch_interface, panel_bridge);

	if (bridge->encoder && bridge->encoder->crtc) {
		const struct drm_crtc_state *crtc_state = bridge->encoder->crtc->state;

		if (drm_atomic_crtc_effectively_active(crtc_state))
			return;
	}

	GOOG_LOG("screen-off.\n");
	gti->vendor_cb(gti->vendor_private_data,
		GTI_CMD_NOTIFY_DISPLAY_STATE, GTI_SUB_CMD_DISPLAY_STATE_OFF, NULL, NULL);
}

struct drm_connector *get_bridge_connector(struct drm_bridge *bridge)
{
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;

	drm_connector_list_iter_begin(bridge->dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (connector->encoder == bridge->encoder)
			break;
	}
	drm_connector_list_iter_end(&conn_iter);
	return connector;
}

static bool panel_bridge_is_lp_mode(struct drm_connector *connector)
{
	if (connector && connector->state) {
		struct exynos_drm_connector_state *s =
			to_exynos_connector_state(connector->state);

		return s->exynos_mode.is_lp_mode;
	}
	return false;
}

static void panel_bridge_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	bool panel_is_lp_mode;
	struct goog_touch_interface *gti =
		container_of(bridge, struct goog_touch_interface, panel_bridge);

	if (!gti->connector || !gti->connector->state)
		gti->connector = get_bridge_connector(bridge);

	panel_is_lp_mode = panel_bridge_is_lp_mode(gti->connector);
	if (gti->panel_is_lp_mode != panel_is_lp_mode) {
		u32 sub_cmd;

		GOOG_LOG("panel_is_lp_mode changed from %d to %d.\n",
			gti->panel_is_lp_mode, panel_is_lp_mode);
		if (panel_is_lp_mode)
			sub_cmd = GTI_SUB_CMD_DISPLAY_STATE_OFF;
		else
			sub_cmd = GTI_SUB_CMD_DISPLAY_STATE_ON;
		gti->vendor_cb(gti->vendor_private_data,
			GTI_CMD_NOTIFY_DISPLAY_STATE, sub_cmd, NULL, NULL);
	}
	gti->panel_is_lp_mode = panel_is_lp_mode;

	if (mode) {
		int vrefresh = drm_mode_vrefresh(mode);

		if (gti->display_vrefresh != vrefresh) {
			GOOG_DBG("display_vrefresh(Hz) changed to %d from %d.\n",
				vrefresh, gti->display_vrefresh);
			gti->display_vrefresh = vrefresh;
			gti->vendor_cb(gti->vendor_private_data,
				GTI_CMD_NOTIFY_DISPLAY_VREFRESH, (u32)vrefresh, NULL, NULL);
		}
	}
}

static const struct drm_bridge_funcs panel_bridge_funcs = {
	.enable = panel_bridge_enable,
	.disable = panel_bridge_disable,
	.mode_set = panel_bridge_mode_set,
};

static int register_panel_bridge(struct goog_touch_interface *gti)
{
	GOOG_LOG("\n");
#ifdef CONFIG_OF
	gti->panel_bridge.of_node = gti->vendor_dev->of_node;
#endif
	gti->panel_bridge.funcs = &panel_bridge_funcs;
	drm_bridge_add(&gti->panel_bridge);

	return 0;
}

static void unregister_panel_bridge(struct drm_bridge *bridge)
{
	struct drm_bridge *node;

	GOOG_LOG("\n");
	drm_bridge_remove(bridge);

	if (!bridge->dev) /* not attached */
		return;

	drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
	list_for_each_entry(node, &bridge->encoder->bridge_chain, chain_node) {
		if (node == bridge) {
			if (bridge->funcs->detach)
				bridge->funcs->detach(bridge);
			list_del(&bridge->chain_node);
			break;
		}
	}
	drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
	bridge->dev = NULL;
}

void goog_update_motion_filter(struct goog_touch_interface *gti, unsigned long slot_bit)
{
	const u32 mf_timeout_ms = 500;
	unsigned long touches = hweight_long(slot_bit);
	u32 next_state = gti->mf_state;

	switch (gti->mf_mode) {
	case GTI_MF_MODE_AUTO_REPORT:
	case GTI_MF_MODE_UNFILTER:
		next_state = GTI_MF_STATE_UNFILTERED;
		break;
	case GTI_MF_MODE_FILTER:
		next_state = GTI_MF_STATE_FILTERED;
		break;
	case GTI_MF_MODE_DYNAMIC:
	default:
		/*
		* Determine the next filter state. The motion filter is enabled by
		* default and it is disabled while a single finger is touching the
		* screen. If another finger is touched down or if a timeout expires,
		* the motion filter is reenabled and remains enabled until all fingers
		* are lifted.
		*/
		switch (next_state) {
		case GTI_MF_STATE_FILTERED:
			if (touches == 1) {
				next_state = GTI_MF_STATE_UNFILTERED;
				gti->mf_downtime = ktime_get();
			}
			break;
		case GTI_MF_STATE_UNFILTERED:
			if (touches == 0) {
				next_state = GTI_MF_STATE_FILTERED;
			} else if (touches > 1 ||
					ktime_after(ktime_get(),
					ktime_add_ms(gti->mf_downtime, mf_timeout_ms))) {
				next_state = GTI_MF_STATE_FILTERED_LOCKED;
			}
			break;
		case GTI_MF_STATE_FILTERED_LOCKED:
			if (touches == 0)
				next_state = GTI_MF_STATE_FILTERED;
			break;
		}
		break;
	}

	/* Send command to setup continuous report. */
	if ((next_state == GTI_MF_STATE_UNFILTERED) !=
		(gti->mf_state == GTI_MF_STATE_UNFILTERED)) {
		u32 enable = GTI_SUB_CMD_DISABLE;

		if (next_state == GTI_MF_STATE_UNFILTERED)
			enable = GTI_SUB_CMD_ENABLE;
		gti->vendor_cb(gti->vendor_private_data,
				GTI_CMD_SET_CONTINUOUS_REPORT, enable, NULL, NULL);
	}

	gti->mf_state = next_state;
}

bool goog_v4l2_read_frame_cb(struct v4l2_heatmap *v4l2)
{
	struct goog_touch_interface *gti = container_of(v4l2, struct goog_touch_interface, v4l2);
	bool ret = false;
	u32 v4l2_size = gti->v4l2.width * gti->v4l2.height * 2;

	if (gti->heatmap_buf && v4l2_size == gti->heatmap_buf_size) {
		memcpy(v4l2->frame, gti->heatmap_buf, v4l2_size);
		ret = true;
	} else {
		GOOG_ERR("wrong pointer(%p) or size (W: %lu, H: %lu) vs %u\n",
		gti->heatmap_buf, gti->v4l2.width, gti->v4l2.height, gti->heatmap_buf_size);
	}

	return ret;
}

void goog_v4l2_read(struct goog_touch_interface *gti, ktime_t timestamp)
{
	if (gti->v4l2_enable)
		heatmap_read(&gti->v4l2, ktime_to_ns(timestamp));
}

void goog_offload_populate_coordinate_channel(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel)
{
	int i;

	struct TouchOffloadDataCoord *dc =
		(struct TouchOffloadDataCoord *)frame->channel_data[channel];

	memset(dc, 0, frame->channel_data_size[channel]);
	dc->header.channel_type = TOUCH_DATA_TYPE_COORD;
	dc->header.channel_size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;

	for (i = 0; i < MAX_COORDS; i++) {
		dc->coords[i].x = gti->offload.coords[i].x;
		dc->coords[i].y = gti->offload.coords[i].y;
		dc->coords[i].major = gti->offload.coords[i].major;
		dc->coords[i].minor = gti->offload.coords[i].minor;
		dc->coords[i].pressure = gti->offload.coords[i].pressure;
		dc->coords[i].status = gti->offload.coords[i].status;
	}
}

void goog_offload_populate_mutual_channel(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel, u8 *buffer, u32 size)
{
	struct TouchOffloadData2d *mutual =
		(struct TouchOffloadData2d *)frame->channel_data[channel];

	mutual->tx_size = gti->offload.caps.tx_size;
	mutual->rx_size = gti->offload.caps.rx_size;
	mutual->header.channel_type = frame->channel_type[channel];
	mutual->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_2D(mutual->rx_size, mutual->tx_size);

	memcpy(mutual->data, buffer, size);
}

void goog_offload_populate_self_channel(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel, u8 *buffer, u32 size)
{
	struct TouchOffloadData1d *self =
		(struct TouchOffloadData1d *)frame->channel_data[channel];

	self->tx_size = gti->offload.caps.tx_size;
	self->rx_size = gti->offload.caps.rx_size;
	self->header.channel_type = frame->channel_type[channel];
	self->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_1D(self->rx_size, self->tx_size);

	memcpy(self->data, buffer, size);
}

void goog_offload_populate_frame(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame)
{
	static u64 index;
	u8 channel_type;
	int i;
	u8 *buffer;
	u32 size;
	int ret;
	u16 tx = gti->offload.caps.tx_size;
	u16 rx = gti->offload.caps.rx_size;

	frame->header.index = index++;
	frame->header.timestamp = gti->input_timestamp;

	ATRACE_BEGIN(__func__);

	/*
	 * TODO(b/201610482):
	 * Porting for other channels, like driver status, stylus status
	 * and others.
	 */
	/* Populate all channels */
	for (i = 0; i < frame->num_channels; i++) {
		channel_type = frame->channel_type[i];
		GOOG_DBG("#%d: get data(type %#x) from vendor driver", i, channel_type);
		buffer = NULL;
		size = 0;
		ret = 0;
		if (channel_type == TOUCH_DATA_TYPE_COORD) {
			ATRACE_BEGIN("populate coord");
			goog_offload_populate_coordinate_channel(gti, frame, i);
			ATRACE_END();
		} else if (channel_type & TOUCH_SCAN_TYPE_MUTUAL) {
			ATRACE_BEGIN("populate mutual data");
			ret = gti->vendor_cb(gti->vendor_private_data, GTI_CMD_GET_SENSOR_DATA,
					channel_type, &buffer, &size);
			if (ret == 0 && buffer && size == TOUCH_OFFLOAD_DATA_SIZE_2D(rx, tx)) {
				goog_offload_populate_mutual_channel(gti, frame, i, buffer, size);
				/* Backup strength data for v4l2. */
				if (channel_type & TOUCH_DATA_TYPE_STRENGTH)
					memcpy(gti->heatmap_buf, buffer, size);
			}
			ATRACE_END();
		} else if (channel_type & TOUCH_SCAN_TYPE_SELF) {
			ATRACE_BEGIN("populate self data");
			ret = gti->vendor_cb(gti->vendor_private_data, GTI_CMD_GET_SENSOR_DATA,
					channel_type, &buffer, &size);
			if (ret == 0 && buffer && size == TOUCH_OFFLOAD_DATA_SIZE_1D(rx, tx))
				goog_offload_populate_self_channel(gti, frame, i, buffer, size);
			ATRACE_END();
		}

		if (ret) {
			GOOG_DBG("skip to populate data(type %#x, ret %d)!\n",
				channel_type, ret);
		}
	}

	ATRACE_END();
}

void goog_offload_set_running(struct goog_touch_interface *gti, bool running)
{
	if (gti->offload.offload_running != running) {
		u32 setting = GTI_SUB_CMD_DISABLE;

		gti->offload.offload_running = running;
		if (running && gti->offload.config.filter_grip)
			setting = GTI_SUB_CMD_DISABLE;
		else
			setting = GTI_SUB_CMD_DRIVER_DEFAULT;
		gti->vendor_cb(gti->vendor_private_data,
				GTI_CMD_SET_GRIP, setting, NULL, NULL);
		gti->grip_setting = setting;

		if (running && gti->offload.config.filter_palm)
			setting = GTI_SUB_CMD_DISABLE;
		else
			setting = GTI_SUB_CMD_DRIVER_DEFAULT;
		gti->vendor_cb(gti->vendor_private_data,
				GTI_CMD_SET_PALM, setting, NULL, NULL);
		gti->palm_setting = setting;
	}
}

void goog_offload_input_report(void *handle,
		 struct TouchOffloadIocReport *report)
{
	struct goog_touch_interface *gti = (struct goog_touch_interface *)handle;
	bool touch_down = 0;
	unsigned int tool_type = MT_TOOL_FINGER;
	int i;
	unsigned long active_slot_bit = 0;

	ATRACE_BEGIN(__func__);

	goog_input_lock(gti);
	input_set_timestamp(gti->vendor_input_dev, report->timestamp);
	for (i = 0; i < MAX_COORDS; i++) {
		if (report->coords[i].status != COORD_STATUS_INACTIVE) {
			switch (report->coords[i].status) {
			case COORD_STATUS_EDGE:
			case COORD_STATUS_PALM:
			case COORD_STATUS_CANCEL:
				tool_type = MT_TOOL_PALM;
				break;
			case COORD_STATUS_FINGER:
			case COORD_STATUS_PEN:
			default:
				tool_type = MT_TOOL_FINGER;
				break;
			}
			__set_bit(i, &active_slot_bit);
			input_mt_slot(gti->vendor_input_dev, i);
			touch_down = 1;
			input_report_key(gti->vendor_input_dev, BTN_TOUCH, touch_down);
			input_mt_report_slot_state(gti->vendor_input_dev, tool_type, 1);
			input_report_abs(gti->vendor_input_dev, ABS_MT_POSITION_X,
				report->coords[i].x);
			input_report_abs(gti->vendor_input_dev, ABS_MT_POSITION_Y,
				report->coords[i].y);
			input_report_abs(gti->vendor_input_dev, ABS_MT_TOUCH_MAJOR,
				report->coords[i].major);
			input_report_abs(gti->vendor_input_dev, ABS_MT_TOUCH_MINOR,
				report->coords[i].minor);
			input_report_abs(gti->vendor_input_dev, ABS_MT_PRESSURE,
				report->coords[i].pressure);
		} else {
			__clear_bit(i, &active_slot_bit);
			input_mt_slot(gti->vendor_input_dev, i);
			input_report_abs(gti->vendor_input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(gti->vendor_input_dev, MT_TOOL_FINGER, 0);
			input_report_abs(gti->vendor_input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}
	input_report_key(gti->vendor_input_dev, BTN_TOUCH, touch_down);
	input_sync(gti->vendor_input_dev);
	goog_input_unlock(gti);

	if (touch_down)
		goog_v4l2_read(gti, report->timestamp);

	goog_update_motion_filter(gti, active_slot_bit);

	ATRACE_END();
}

int goog_offload_probe(struct goog_touch_interface *gti)
{
	int ret;
	u16 values[2];
	struct device_node *np = gti->vendor_dev->of_node;

	if (of_property_read_u8_array(np, "goog,touch_offload_id",
					  gti->offload_id_byte, 4)) {
		GOOG_LOG("set default offload id: GOOG!\n");
		gti->offload_id_byte[0] = 'G';
		gti->offload_id_byte[1] = 'O';
		gti->offload_id_byte[2] = 'O';
		gti->offload_id_byte[3] = 'G';
	}

	gti->offload.caps.touch_offload_major_version = TOUCH_OFFLOAD_INTERFACE_MAJOR_VERSION;
	gti->offload.caps.touch_offload_minor_version = TOUCH_OFFLOAD_INTERFACE_MINOR_VERSION;
	gti->offload.caps.device_id = gti->offload_id;

	if (of_property_read_u16_array(np, "goog,display-resolution",
					  values, 2) == 0) {
		gti->offload.caps.display_width = values[0];
		gti->offload.caps.display_height = values[1];
	} else {
		GOOG_ERR("Plesae set \"goog,display-resolution\" in dts!");
	}

	if (of_property_read_u16_array(np, "goog,channel-num",
					  values, 2) == 0) {
		gti->offload.caps.tx_size = values[0];
		gti->offload.caps.rx_size = values[1];
	} else {
		GOOG_ERR("Plesae set \"goog,channel-num\" in dts!");
		ret = -EINVAL;
		goto err_offload_probe;
	}

	/*
	 * TODO(b/201610482): Set more offload caps from parameters or from dtsi?
	 */
	gti->offload.caps.heatmap_size = HEATMAP_SIZE_FULL;
	gti->offload.caps.bus_type = BUS_TYPE_SPI;
	if (of_property_read_u32(np, "spi-max-frequency",
			&gti->offload.caps.bus_speed_hz))
		gti->offload.caps.bus_speed_hz = 0;

	gti->offload.caps.touch_data_types =
		TOUCH_DATA_TYPE_COORD | TOUCH_DATA_TYPE_STRENGTH |
		TOUCH_DATA_TYPE_RAW | TOUCH_DATA_TYPE_BASELINE;
	gti->offload.caps.touch_scan_types =
		TOUCH_SCAN_TYPE_MUTUAL;

	gti->offload.caps.continuous_reporting = true;
	gti->offload.caps.noise_reporting = false;
	gti->offload.caps.cancel_reporting = false;
	gti->offload.caps.size_reporting = true;
	gti->offload.caps.filter_grip = true;
	gti->offload.caps.filter_palm = true;
	gti->offload.caps.num_sensitivity_settings = 1;

	gti->offload.hcallback = (void *)gti;
	gti->offload.report_cb = goog_offload_input_report;
	ret = touch_offload_init(&gti->offload);
	if (ret) {
		GOOG_ERR("offload init failed, ret %d!\n", ret);
		goto err_offload_probe;
	}

	gti->offload_enable = of_property_read_bool(np, "goog,offload-enable");
	GOOG_LOG("offload configucation: %d * %d (%d * %d)\n",
		gti->offload.caps.display_width, gti->offload.caps.display_height,
		gti->offload.caps.tx_size, gti->offload.caps.rx_size);

	GOOG_LOG("offload ID: \"%c%c%c%c\" / 0x%08X, offload_enable=%d.\n",
		gti->offload_id_byte[0], gti->offload_id_byte[1], gti->offload_id_byte[2],
		gti->offload_id_byte[3], gti->offload_id, gti->offload_enable);

	gti->heatmap_buf_size = gti->offload.caps.tx_size * gti->offload.caps.rx_size * sizeof(u16);
	gti->heatmap_buf = devm_kzalloc(gti->vendor_dev, gti->heatmap_buf_size, GFP_KERNEL);
	if (!gti->heatmap_buf) {
		GOOG_ERR("heamap alloc failed!\n");
		ret = -ENOMEM;
		goto err_offload_probe;
	}

	/*
	 * Heatmap_probe must be called before irq routine is registered,
	 * because heatmap_read is called from the irq context.
	 * If the ISR runs before heatmap_probe is finished, it will invoke
	 * heatmap_read and cause NPE, since read_frame would not yet be set.
	 */
	gti->v4l2.parent_dev = gti->vendor_dev;
	gti->v4l2.input_dev = gti->vendor_input_dev;
	gti->v4l2.read_frame = goog_v4l2_read_frame_cb;
	gti->v4l2.width = gti->offload.caps.tx_size;
	gti->v4l2.height = gti->offload.caps.rx_size;

	/* 120 Hz operation */
	gti->v4l2.timeperframe.numerator = 1;
	if (of_property_read_u32(np, "goog,report-rate",
			&gti->v4l2.timeperframe.denominator))
		gti->v4l2.timeperframe.denominator = 120;

	ret = heatmap_probe(&gti->v4l2);
	if (ret) {
		GOOG_ERR("v4l2 init failed, ret %d!\n", ret);
		goto err_offload_probe;
	}
	gti->v4l2_enable = of_property_read_bool(np, "goog,v4l2-enable");
	GOOG_LOG("v4l2 W/H=(%lu, %lu), v4l2_enable=%d.\n",
		gti->v4l2.width, gti->v4l2.height, gti->v4l2_enable);

err_offload_probe:
	return ret;
}

void goog_offload_remove(struct goog_touch_interface *gti)
{
	touch_offload_cleanup(&gti->offload);
}

bool goog_input_legacy_report(struct goog_touch_interface *gti)
{
	if (!gti->offload.offload_running || gti->force_legacy_report)
		return true;

	return false;
}

int goog_input_process(struct goog_touch_interface *gti)
{
	int ret = 0;
	struct touch_offload_frame **frame = &gti->offload_frame;

	if (!gti->coord_changed)
		return -EPERM;

	if (gti->offload_enable) {
		ret = touch_offload_reserve_frame(&gti->offload, frame);
		if (ret != 0 || frame == NULL) {
			GOOG_ERR("could not reserve a frame(ret %d)!\n", ret);
			/* Stop offload when there are no buffers available. */
			goog_offload_set_running(gti, false);
			/*
			 * TODO(b/193467748):
			 * How to handle current coord if offload running
			 * terminating in the halfway(not beginning case)?
			 */
			ret = -EBUSY;
		} else {
			goog_offload_set_running(gti, true);
			goog_offload_populate_frame(gti, *frame);
			ret = touch_offload_queue_frame(&gti->offload, *frame);
			if (ret)
				GOOG_ERR("failed to queue reserved frame(ret %d)!\n", ret);
			else
				gti->offload_frame = NULL;
		}
	}

	/*
	 * If offload is NOT running, read heatmap directly by callback.
	 * Otherwise, heatmap will be handled for both offload and v4l2
	 * during goog_offload_populate_frame().
	 */
	if (!gti->offload.offload_running && gti->v4l2_enable) {
		int ret;
		u8 *buffer = NULL;
		u32 size = 0;

		ret = gti->vendor_cb(gti->vendor_private_data, GTI_CMD_GET_SENSOR_DATA,
				(TOUCH_SCAN_TYPE_MUTUAL | TOUCH_DATA_TYPE_STRENGTH),
				&buffer, &size);
		if (ret == 0 && buffer && size)
			memcpy(gti->heatmap_buf, buffer, size);
		goog_v4l2_read(gti, gti->input_timestamp);
		goog_update_motion_filter(gti, gti->active_slot_bit);
	}

	gti->input_timestamp_changed = false;
	gti->coord_changed = false;

	return ret;
}
EXPORT_SYMBOL(goog_input_process);

void goog_input_lock(struct goog_touch_interface *gti)
{
	mutex_lock(&gti->input_lock);
}
EXPORT_SYMBOL(goog_input_lock);

void goog_input_unlock(struct goog_touch_interface *gti)
{
	mutex_unlock(&gti->input_lock);
}
EXPORT_SYMBOL(goog_input_unlock);

void goog_input_set_timestamp(
		struct goog_touch_interface *gti,
		struct input_dev *dev, ktime_t timestamp)
{
	/* Specific case to handle all fingers release. */
	if (!ktime_compare(timestamp, KTIME_RELEASE_ALL)) {
		GOOG_DBG("Enable force_legacy_report for all fingers release.\n");
		/* Enable FW palm and grip for low power sensing during suspend. */
		if (gti->offload.offload_running) {
			gti->vendor_cb(gti->vendor_private_data,
					GTI_CMD_SET_GRIP, GTI_SUB_CMD_ENABLE, NULL, NULL);
			gti->vendor_cb(gti->vendor_private_data,
					GTI_CMD_SET_PALM, GTI_SUB_CMD_ENABLE, NULL, NULL);
		}
		timestamp = ktime_get();
		gti->force_legacy_report = true;
	} else {
		/* Once device is from suspend to resume, recover last grip/palm state. */
		if (gti->offload.offload_running && gti->force_legacy_report) {
			gti->vendor_cb(gti->vendor_private_data,
					GTI_CMD_SET_GRIP, gti->grip_setting, NULL, NULL);
			gti->vendor_cb(gti->vendor_private_data,
					GTI_CMD_SET_PALM, gti->palm_setting, NULL, NULL);
		}
		GOOG_DBG("Disable force_legacy_report as usual state.\n");
		gti->force_legacy_report = false;
	}

	if (goog_input_legacy_report(gti))
		input_set_timestamp(dev, timestamp);

	gti->input_timestamp = timestamp;
	gti->input_timestamp_changed = true;
}
EXPORT_SYMBOL(goog_input_set_timestamp);

void goog_input_mt_slot(
		struct goog_touch_interface *gti,
		struct input_dev *dev, int slot)
{
	if (goog_input_legacy_report(gti))
		input_mt_slot(dev, slot);

	if (slot < MAX_COORDS) {
		gti->slot = slot;
		/*
		 * Make sure the input timestamp should be set before updating 1st mt_slot.
		 * This is for input report switch between offload and legacy.
		 */
		if (!gti->coord_changed && !gti->input_timestamp_changed)
			GOOG_ERR("please exec goog_input_set_timestamp before %s!\n", __func__);
		gti->coord_changed = true;
	}
}
EXPORT_SYMBOL(goog_input_mt_slot);

void goog_input_mt_report_slot_state(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int tool_type, bool active)
{
	if (goog_input_legacy_report(gti))
		input_mt_report_slot_state(dev, tool_type, active);

	if (tool_type == MT_TOOL_FINGER) {
		if (active) {
			gti->offload.coords[gti->slot].status = COORD_STATUS_FINGER;
			__set_bit(gti->slot, &gti->active_slot_bit);
		} else {
			gti->offload.coords[gti->slot].status = COORD_STATUS_INACTIVE;
			__clear_bit(gti->slot, &gti->active_slot_bit);
		}
	}
}
EXPORT_SYMBOL(goog_input_mt_report_slot_state);

void goog_input_report_abs(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value)
{
	if (goog_input_legacy_report(gti))
		input_report_abs(dev, code, value);

	if (gti->slot < MAX_COORDS) {
		switch (code) {
		case ABS_MT_POSITION_X:
			gti->offload.coords[gti->slot].x = value;
			break;
		case ABS_MT_POSITION_Y:
			gti->offload.coords[gti->slot].y = value;
			break;
		case ABS_MT_TOUCH_MAJOR:
			gti->offload.coords[gti->slot].major = value;
			break;
		case ABS_MT_TOUCH_MINOR:
			gti->offload.coords[gti->slot].minor = value;
			break;
		case ABS_MT_PRESSURE:
			gti->offload.coords[gti->slot].pressure = value;
			break;
		default:
			break;
		}
	}
}
EXPORT_SYMBOL(goog_input_report_abs);

void goog_input_report_key(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value)
{
	if (goog_input_legacy_report(gti))
		input_report_key(dev, code, value);
}
EXPORT_SYMBOL(goog_input_report_key);

void goog_input_sync(struct goog_touch_interface *gti, struct input_dev *dev)
{
	if (goog_input_legacy_report(gti))
		input_sync(dev);
}
EXPORT_SYMBOL(goog_input_sync);

struct goog_touch_interface *goog_touch_interface_probe(
		void *private_data,
		struct device *dev,
		struct input_dev *input_dev,
		int (*vendor_cb)(void *private_data, u32 cmd, u32 sub_cmd, u8 **buffer, u32 *size))
{
	struct goog_touch_interface *gti =
		devm_kzalloc(dev, sizeof(struct goog_touch_interface), GFP_KERNEL);

	if (gti) {
		gti->vendor_private_data = private_data;
		gti->vendor_dev = dev;
		gti->vendor_input_dev = input_dev;
		gti->vendor_cb = vendor_cb;
		gti->mf_mode = GTI_MF_MODE_DEFAULT;
		mutex_init(&gti->input_lock);
		goog_offload_probe(gti);
		register_panel_bridge(gti);
	}

	if (!gti_class)
		gti_class = class_create(THIS_MODULE, "goog_touch_interface");

	if (gti && gti_class) {
		char *name = kasprintf(GFP_KERNEL, "gti.%d", gti_dev_num);

		if (name &&
			!alloc_chrdev_region(&gti->dev_id, 0, 1, name)) {
			gti->dev = device_create(gti_class, NULL,
					gti->dev_id, gti, name);
			if (gti->dev) {
				gti_dev_num++;
				GOOG_LOG("device create \"%s\".\n", name);
				if (gti->vendor_dev) {
					sysfs_create_link(&gti->dev->kobj,
						&gti->vendor_dev->kobj, "vendor");
				}
				if (gti->vendor_input_dev) {
					sysfs_create_link(&gti->dev->kobj,
						&gti->vendor_input_dev->dev.kobj, "vendor_input");
				}
			}
		}
		kfree(name);
	}

	if (gti && gti->dev) {
		int ret;

		ret = sysfs_create_group(&gti->dev->kobj, &goog_attr_group);
		if (ret)
			GOOG_ERR("sysfs_create_group() failed, ret= %d!\n", ret);
	}

	return gti;
}
EXPORT_SYMBOL(goog_touch_interface_probe);

int goog_touch_interface_remove(struct goog_touch_interface *gti)
{
	if (!gti)
		return -ENODEV;

	unregister_panel_bridge(&gti->panel_bridge);
	if (gti->vendor_dev)
		sysfs_remove_link(&gti->dev->kobj, "vendor");
	if (gti->vendor_input_dev)
		sysfs_remove_link(&gti->dev->kobj, "vendor_input");

	if (gti_class) {
		unregister_chrdev_region(gti->dev_id, 1);
		device_destroy(gti_class, gti->dev_id);
		gti_dev_num--;
	}

	gti->offload_enable = false;
	gti->v4l2_enable = false;
	goog_offload_remove(gti);
	heatmap_remove(&gti->v4l2);
	devm_kfree(gti->vendor_dev, gti->heatmap_buf);
	devm_kfree(gti->vendor_dev, gti);

	if (gti_class && !gti_dev_num)
		class_destroy(gti_class);

	return 0;
}
EXPORT_SYMBOL(goog_touch_interface_remove);

MODULE_DESCRIPTION("Google Touch Interface");
MODULE_AUTHOR("Super Liu<supercjliu@google.com>");
MODULE_LICENSE("GPL v2");
