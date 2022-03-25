/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Touch Interface for Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#ifndef _GOOG_TOUCH_INTERFACE_
#define _GOOG_TOUCH_INTERFACE_

#include "heatmap.h"
#include "touch_offload.h"
#include "uapi/input/touch_offload.h"

#define GOOG_LOG_NAME "GTI"
#define GOOG_DBG(fmt, args...)    pr_debug("[%s] %s: " fmt, GOOG_LOG_NAME,\
					__func__, ##args)
#define GOOG_LOG(fmt, args...)    pr_info("[%s] %s: " fmt, GOOG_LOG_NAME,\
					__func__, ##args)
#define GOOG_ERR(fmt, args...)    pr_err("[%s] %s: " fmt, GOOG_LOG_NAME,\
					__func__, ##args)
#define MAX_COORDS 10

#define KTIME_RELEASE_ALL (ktime_set(0, 0))


enum {
	/*
	 * GTI_CMD_GET_SENSOR_DATA:
	 *   specific sub_cmd: compose of TOUCH_DATA_TYPE_* and TOUCH_SCAN_TYPE_*.
	 *   buffer: the address of buffer.
	 *   size: the size of buffer.
	 */
	GTI_CMD_GET_SENSOR_DATA,

	/*
	 * GTI_CMD_SET_GRIP:
	 *   common sub_cmd: GTI_SUB_CMD_*.
	 *   buffer: N/A.
	 *   size: N/A.
	 */
	GTI_CMD_SET_GRIP,

	/*
	 * GTI_CMD_SET_PALM:
	 *   common sub_cmd: GTI_SUB_CMD_*.
	 *   buffer: N/A.
	 *   size: N/A.
	 */
	GTI_CMD_SET_PALM,

	/*
	 * GTI_CMD_SET_CONTINUOUS_REPORT:
	 *   common sub_cmd: GTI_SUB_CMD_*.
	 *   buffer: N/A.
	 *   size: N/A.
	 */
	GTI_CMD_SET_CONTINUOUS_REPORT,
};

enum {
	GTI_SUB_CMD_DISABLE = 0,
	GTI_SUB_CMD_ENABLE,
	GTI_SUB_CMD_DRIVER_DEFAULT,
};

/**
 * struct goog_touch_interfac - Google touch interface data for Pixel.
 * @vendor_private_data: the private data pointer that used by touch vendor driver.
 * @dev: pointer to struct device that used by touch vendor driver.
 * @input_dev: poiner to struct inpu_dev that used by touch vendor driver.
 * @input_lock: protect the input report between non-offload and offload.
 * @offload: struct that used by touch offload.
 * @offload_frame: reserved frame that used by touch offload.
 * @v4l2: struct that used by v4l2.
 * @timestamp: irq timestamp from touch vendor driver.
 * @grip_setting: current grip setting.
 * @palm_setting: current palm setting.
 * @force_legacy_report: force to directly report input by kernel input API.
 * @offload_enable: touch offload is enabled or not.
 * @v4l2_enable: v4l2 is enabled or not.
 * @coord_changed: coords was changed and wait to push frame into touch offload.
 * @offload_id: id that used by touch offload.
 * @heatmap_buf: heatmap buffer that used by v4l2.
 * @heatmap_buf_size: heatmap buffer size that used by v4l2.
 * @slot: slot id that current used by input report.
 * @vendor_cb: touch vendor driver function callback.
 */

struct goog_touch_interface {
	void *vendor_private_data;
	struct device *dev;
	struct input_dev *input_dev;
	struct mutex input_lock;
	struct touch_offload_context offload;
	struct touch_offload_frame *offload_frame;
	struct v4l2_heatmap v4l2;
	ktime_t timestamp;

	u32 grip_setting;
	u32 palm_setting;

	bool force_legacy_report;
	bool offload_enable;
	bool v4l2_enable;
	bool coord_changed;
	union {
	u8 offload_id_byte[4];
	u32 offload_id;
	};
	u8 *heatmap_buf;
	u32 heatmap_buf_size;
	int slot;

	int (*vendor_cb)(void *vendor_private_data,
				u32 cmd, u32 sub_cmd, u8 **buffer, u32 *size);
};


inline bool goog_input_legacy_report(struct goog_touch_interface *gti);
inline void goog_input_lock(struct goog_touch_interface *gti);
inline void goog_input_unlock(struct goog_touch_interface *gti);
inline void goog_input_set_timestamp(
		struct goog_touch_interface *gti,
		struct input_dev *dev, ktime_t timestamp);
inline void goog_input_mt_slot(
		struct goog_touch_interface *gti,
		struct input_dev *dev, int slot);
inline void goog_input_mt_report_slot_state(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int tool_type, bool active);
inline void goog_input_report_abs(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value);
inline void goog_input_report_key(
		struct goog_touch_interface *gti,
		struct input_dev *dev, unsigned int code, int value);
inline void goog_input_sync(struct goog_touch_interface *gti, struct input_dev *dev);

int goog_input_process(struct goog_touch_interface *gti);
struct goog_touch_interface *goog_touch_interface_probe(
		void *vendor_private_data,
		struct device *dev,
		struct input_dev *input_dev,
		int (*vendor_cb)(void *private_data, u32 cmd, u32 sub_cmd, u8 **buffer, u32 *size));
int goog_touch_interface_remove(struct goog_touch_interface *gti);

#endif // _GOOG_TOUCH_INTERFACE_

