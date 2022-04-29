/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Touch Interface for Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */

#ifndef _GOOG_TOUCH_INTERFACE_
#define _GOOG_TOUCH_INTERFACE_

#include <drm/drm_panel.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>

#include "heatmap.h"
#include "touch_offload.h"
#include "uapi/input/touch_offload.h"

#if IS_ENABLED(CONFIG_VH_SYSTRACE)
#include <trace/hooks/systrace.h>
#else
#define ATRACE_BEGIN(f)
#define ATRACE_END()
#endif

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

	/*
	 * GTI_CMD_NOTIFY_DISPLAY_STATE:
	 *   specific sub_cmd: GTI_SUB_CMD_DISPLAY_STATE_*.
	 *   buffer: N/A.
	 *   size: N/A.
	 */
	GTI_CMD_NOTIFY_DISPLAY_STATE,

	/*
	 * GTI_CMD_NOTIFY_DISPLAY_VREFRESH:
	 *   specific sub_cmd: display vrefresh in Hz.
	 *   buffer: N/A.
	 *   size: N/A.
	 */
	GTI_CMD_NOTIFY_DISPLAY_VREFRESH,

};

enum {
	GTI_SUB_CMD_DISABLE = 0,
	GTI_SUB_CMD_ENABLE,
	GTI_SUB_CMD_DRIVER_DEFAULT,
};

enum {
	GTI_SUB_CMD_DISPLAY_STATE_OFF = 0,
	GTI_SUB_CMD_DISPLAY_STATE_ON,
};

/**
 * Motion filter finite state machine (FSM) state.
 *   GTI_MF_STATE_FILTERED: default coordinate filtering
 *   GTI_MF_STATE_UNFILTERED: coordinate unfiltering for single-touch.
 *   GTI_MF_STATE_FILTERED_LOCKED: filtered coordinates. Locked until
 *                                 touch is lifted or timeout.
 */
enum {
	GTI_MF_STATE_FILTERED = 0,
	GTI_MF_STATE_UNFILTERED,
	GTI_MF_STATE_FILTERED_LOCKED,
};

/**
 * Motion filter mode.
 *   GTI_MF_MODE_UNFILTER: enable unfilter by continuous reporting.
 *   GTI_MF_MODE_DYNAMIC: dynamic control for motion filter.
 *   GTI_MF_MODE_FILTER: only report touch if coord report changed.
 *   GTI_MF_MODE_AUTO: for development case.
 */
enum {
	GTI_MF_MODE_UNFILTER = 0,
	GTI_MF_MODE_DEFAULT,
	GTI_MF_MODE_DYNAMIC = GTI_MF_MODE_DEFAULT,
	GTI_MF_MODE_FILTER,
	GTI_MF_MODE_AUTO_REPORT,
};

/**
 * struct goog_touch_interfac - Google touch interface data for Pixel.
 * @vendor_private_data: the private data pointer that used by touch vendor driver.
 * @vendor_dev: pointer to struct device that used by touch vendor driver.
 * @vendor_input_dev: poiner to struct inpu_dev that used by touch vendor driver.
 * @dev: pointer to struct device that used by google touch interface driver.
 * @input_lock: protect the input report between non-offload and offload.
 * @offload: struct that used by touch offload.
 * @offload_frame: reserved frame that used by touch offload.
 * @v4l2: struct that used by v4l2.
 * @panel_bridge: struct that used to register panel bridge notification.
 * @connector: struct that used to get panel status.
 * @input_timestamp: input timestamp from touch vendor driver.
 * @mf_downtime: timestamp for motion filter control.
 * @display_vrefresh: display vrefresh in Hz.
 * @grip_setting: current grip setting.
 * @palm_setting: current palm setting.
 * @mf_mode: current motion filter mode.
 * @mf_state: current motion filter state.
 * @panel_is_lp_mode: display is in low power mode.
 * @force_legacy_report: force to directly report input by kernel input API.
 * @offload_enable: touch offload is enabled or not.
 * @v4l2_enable: v4l2 is enabled or not.
 * @coord_changed: coords was changed and wait to push frame into touch offload.
 * @input_timestamp_changed: input timestamp changed from touch vendor driver.
 * @offload_id: id that used by touch offload.
 * @heatmap_buf: heatmap buffer that used by v4l2.
 * @heatmap_buf_size: heatmap buffer size that used by v4l2.
 * @slot: slot id that current used by input report.
 * @active_slot_bit: bitmap of active slot from legacy report.
 * @dev_id: dev_t used for google interface driver.
 * @vendor_cb: touch vendor driver function callback.
 */

struct goog_touch_interface {
	void *vendor_private_data;
	struct device *vendor_dev;
	struct input_dev *vendor_input_dev;
	struct device *dev;
	struct mutex input_lock;
	struct touch_offload_context offload;
	struct touch_offload_frame *offload_frame;
	struct v4l2_heatmap v4l2;
	struct drm_bridge panel_bridge;
	struct drm_connector *connector;
	ktime_t input_timestamp;
	ktime_t mf_downtime;

	int display_vrefresh;
	u32 grip_setting;
	u32 palm_setting;
	u32 mf_mode;
	u32 mf_state;

	bool panel_is_lp_mode;
	bool force_legacy_report;
	bool offload_enable;
	bool v4l2_enable;
	bool coord_changed;
	bool input_timestamp_changed;
	union {
	u8 offload_id_byte[4];
	u32 offload_id;
	};
	u8 *heatmap_buf;
	u32 heatmap_buf_size;
	int slot;
	unsigned long active_slot_bit;
	dev_t dev_id;

	int (*vendor_cb)(void *private_data,
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
		void *private_data,
		struct device *dev,
		struct input_dev *input_dev,
		int (*vendor_cb)(void *private_data, u32 cmd, u32 sub_cmd, u8 **buffer, u32 *size));
int goog_touch_interface_remove(struct goog_touch_interface *gti);

#endif // _GOOG_TOUCH_INTERFACE_

