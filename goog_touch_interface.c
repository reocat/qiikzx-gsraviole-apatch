// SPDX-License-Identifier: GPL-2.0
/*
 * Google Touch Interface for Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/of.h>

#include "goog_touch_interface.h"
#include "../../../gs-google/drivers/soc/google/vh/kernel/systrace.h"

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
		struct touch_offload_frame *frame, int channel, u8 *ptr, u32 size)
{
	struct TouchOffloadData2d *mutual =
		(struct TouchOffloadData2d *)frame->channel_data[channel];

	mutual->tx_size = gti->offload.caps.tx_size;
	mutual->rx_size = gti->offload.caps.rx_size;
	mutual->header.channel_type = frame->channel_type[channel];
	mutual->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_2D(mutual->rx_size, mutual->tx_size);

	memcpy(mutual->data, ptr, size);
}

void goog_offload_populate_self_channel(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame, int channel, u8 *ptr, u32 size)
{
	struct TouchOffloadData1d *self =
		(struct TouchOffloadData1d *)frame->channel_data[channel];

	self->tx_size = gti->offload.caps.tx_size;
	self->rx_size = gti->offload.caps.rx_size;
	self->header.channel_type = frame->channel_type[channel];
	self->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_1D(self->rx_size, self->tx_size);

	memcpy(self->data, ptr, size);
}

void goog_offload_populate_frame(struct goog_touch_interface *gti,
		struct touch_offload_frame *frame)
{
	static u64 index;
	u8 channel_type;
	int i;
	u8 *ptr;
	u32 size;
	int ret;

	frame->header.index = index++;
	frame->header.timestamp = gti->timestamp;

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
		ptr = NULL;
		size = 0;
		ret = 0;
		if (channel_type == TOUCH_DATA_TYPE_COORD) {
			ATRACE_BEGIN("populate coord");
			goog_offload_populate_coordinate_channel(gti, frame, i);
			ATRACE_END();
		} else if (channel_type & TOUCH_SCAN_TYPE_MUTUAL) {
			ATRACE_BEGIN("populate mutual data");
			ret = gti->get_channel_data_cb(gti->vendor_private_data, channel_type,
						&ptr, &size);
			if (ret == 0 && ptr && size) {
				goog_offload_populate_mutual_channel(gti, frame, i, ptr, size);
				/* Backup strength data for v4l2. */
				if (channel_type & TOUCH_DATA_TYPE_STRENGTH)
					memcpy(gti->heatmap_buf, ptr, size);
			}
			ATRACE_END();
		} else if (channel_type & TOUCH_SCAN_TYPE_SELF) {
			ATRACE_BEGIN("populate self data");
			ret = gti->get_channel_data_cb(gti->vendor_private_data, channel_type,
						&ptr, &size);
			if (ret == 0 && ptr && size)
				goog_offload_populate_self_channel(gti, frame, i, ptr, size);
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
	if (gti->offload.offload_running != running)
		gti->offload.offload_running = running;

	/*
	 * TODO(b/201610482): Enable/disable FW grip and palm.
	 */
}

void goog_offload_input_report(void *handle,
		 struct TouchOffloadIocReport *report)
{
	struct goog_touch_interface *gti = (struct goog_touch_interface *)handle;
	bool touch_down = 0;
	unsigned int tool_type = MT_TOOL_FINGER;
	int i;

	ATRACE_BEGIN(__func__);

	goog_input_lock(gti);
	input_set_timestamp(gti->input_dev, report->timestamp);
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
			input_mt_slot(gti->input_dev, i);
			touch_down = 1;
			input_report_key(gti->input_dev, BTN_TOUCH, touch_down);
			input_mt_report_slot_state(gti->input_dev, tool_type, 1);
			input_report_abs(gti->input_dev, ABS_MT_POSITION_X,
				report->coords[i].x);
			input_report_abs(gti->input_dev, ABS_MT_POSITION_Y,
				report->coords[i].y);
			input_report_abs(gti->input_dev, ABS_MT_TOUCH_MAJOR,
				report->coords[i].major);
			input_report_abs(gti->input_dev, ABS_MT_TOUCH_MINOR,
				report->coords[i].minor);
			input_report_abs(gti->input_dev, ABS_MT_PRESSURE,
				report->coords[i].pressure);
		} else {
			input_mt_slot(gti->input_dev, i);
			input_report_abs(gti->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(gti->input_dev, MT_TOOL_FINGER, 0);
			input_report_abs(gti->input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}
	input_report_key(gti->input_dev, BTN_TOUCH, touch_down);
	input_sync(gti->input_dev);
	goog_input_unlock(gti);

	if (touch_down)
		goog_v4l2_read(gti, report->timestamp);

	/*
	 * TODO(b/201610482): Enable/disable continuous reporting.
	 */

	ATRACE_END();
}

int goog_offload_probe(struct goog_touch_interface *gti)
{
	int ret;
	u16 values[2];
	struct device_node *np = gti->dev->of_node;

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
	gti->heatmap_buf = devm_kzalloc(gti->dev, gti->heatmap_buf_size, GFP_KERNEL);
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
	gti->v4l2.parent_dev = gti->dev;
	gti->v4l2.input_dev = gti->input_dev;
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
	if (!gti->offload.offload_running) {
		int ret;
		u8 *ptr = NULL;
		u32 size = 0;

		ret = gti->get_channel_data_cb(gti->vendor_private_data,
				(TOUCH_SCAN_TYPE_MUTUAL | TOUCH_DATA_TYPE_STRENGTH),
				&ptr, &size);
		if (ret == 0 && ptr && size)
			memcpy(gti->heatmap_buf, ptr, size);
		goog_v4l2_read(gti, gti->timestamp);
	}

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
		timestamp = ktime_get();
		gti->force_legacy_report = true;
	} else {
		GOOG_DBG("Disable force_legacy_report as usual state.\n");
		gti->force_legacy_report = false;
	}

	if (goog_input_legacy_report(gti))
		input_set_timestamp(dev, timestamp);
	gti->timestamp = timestamp;
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
		if (active)
			gti->offload.coords[gti->slot].status = COORD_STATUS_FINGER;
		else
			gti->offload.coords[gti->slot].status = COORD_STATUS_INACTIVE;
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
		void *vendor_private_data,
		struct device *dev,
		struct input_dev *input_dev,
		int (*get_data_cb)(void *, u32, u8 **, u32 *))
{
	struct goog_touch_interface *gti =
		devm_kzalloc(dev, sizeof(struct goog_touch_interface), GFP_KERNEL);

	if (gti) {
		gti->vendor_private_data = vendor_private_data;
		gti->dev = dev;
		gti->input_dev = input_dev;
		gti->get_channel_data_cb = get_data_cb;
		mutex_init(&gti->input_lock);
		goog_offload_probe(gti);
	}

	return gti;
}
EXPORT_SYMBOL(goog_touch_interface_probe);

int goog_touch_interface_remove(struct goog_touch_interface *gti)
{
	gti->offload_enable = false;
	gti->v4l2_enable = false;
	goog_offload_remove(gti);
	heatmap_remove(&gti->v4l2);
	devm_kfree(gti->dev, gti->heatmap_buf);
	devm_kfree(gti->dev, gti);
	return 0;
}
EXPORT_SYMBOL(goog_touch_interface_remove);

MODULE_DESCRIPTION("Google Touch Interface");
MODULE_AUTHOR("Super Liu<supercjliu@google.com>");
MODULE_LICENSE("GPL v2");
