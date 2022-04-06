/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Touch Interface NOP for Pixel devices.
 *
 * Copyright 2022 Google LLC.
 */
#ifndef _GOOG_TOUCH_INTERFACE_NOP_
#define _GOOG_TOUCH_INTERFACE_NOP_

#include <linux/input/mt.h>

#define KTIME_RELEASE_ALL (ktime_set(0, 0))

struct goog_touch_interface {
	void *private_data;
};

static inline void goog_input_lock(struct goog_touch_interface *gti)
{
}

static inline void goog_input_unlock(struct goog_touch_interface *gti)
{
}

static inline void goog_input_set_timestamp(
    struct goog_touch_interface *gti,
    struct input_dev *dev, ktime_t timestamp)
{
	input_set_timestamp(dev, timestamp);
}

static inline void goog_input_mt_slot(
    struct goog_touch_interface *gti,
    struct input_dev *dev, int slot)
{
	input_mt_slot(dev, slot);
}

static inline void goog_input_mt_report_slot_state(
    struct goog_touch_interface *gti,
    struct input_dev *dev, unsigned int tool_type, bool active)
{
	input_mt_report_slot_state(dev, tool_type, active);
}

static inline void goog_input_report_abs(
    struct goog_touch_interface *gti,
    struct input_dev *dev, unsigned int code, int value)
{
	input_report_abs(dev, code, value);
}

static inline void goog_input_report_key(
    struct goog_touch_interface *gti,
    struct input_dev *dev, unsigned int code, int value)
{
	input_report_key(dev, code, value);
}

static inline void goog_input_sync(struct goog_touch_interface *gti, struct input_dev *dev)
{
	input_sync(dev);
}

static inline int goog_input_process(struct goog_touch_interface *gti)
{
	return 0;
}

static inline struct goog_touch_interface *goog_touch_interface_probe(
	void *private_data,
	struct device *dev,
	struct input_dev *input_dev,
	int (*vendor_cb)(void *private_data, u32 cmd, u32 sub_cmd, u8 **buffer, u32 *size))
{
	static struct goog_touch_interface gti[1];

	return gti;
}

static inline int goog_touch_interface_remove(struct goog_touch_interface *gti)
{
	return 0;
}

#endif // _GOOG_TOUCH_INTERFACE_NOP_