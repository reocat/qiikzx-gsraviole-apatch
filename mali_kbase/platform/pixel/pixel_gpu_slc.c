// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2023 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>

/* UAPI includes */
#include <uapi/gpu/arm/midgard/platform/pixel/pixel_gpu_common_slc.h>
/* Back-door mali_pixel include */
#include <uapi/gpu/arm/midgard/platform/pixel/pixel_memory_group_manager.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_slc.h"

/**
 * gpu_pixel_handle_buffer_liveness_update_ioctl() - See gpu_slc_liveness_update
 *
 * @kctx:   The &struct kbase_context corresponding to a user space context which sent the liveness
 *          update
 * @update: See struct kbase_ioctl_buffer_liveness_update
 *
 * Context: Process context. Takes and releases the GPU power domain lock. Expects the caller to
 *          hold the DVFS lock.
 */
int gpu_pixel_handle_buffer_liveness_update_ioctl(struct kbase_context* kctx,
                                                  struct kbase_ioctl_buffer_liveness_update* update)
{
	(void)kctx, (void)update;
	return 0;
}

/**
 * gpu_slc_kctx_init() - Called when a kernel context is created
 *
 * @kctx: The &struct kbase_context that is being initialized
 *
 * This function is called when the GPU driver is initializing a new kernel context. This event is
 * used to set up data structures that will be used to track this context's usage of the SLC.
 *
 * Return: Returns 0 on success, or an error code on failure.
 */
int gpu_slc_kctx_init(struct kbase_context *kctx)
{
	(void)kctx;
	return 0;
}

/**
 * gpu_slc_kctx_term() - Called when a kernel context is terminated
 *
 * @kctx: The &struct kbase_context that is being terminated
 */
void gpu_slc_kctx_term(struct kbase_context *kctx)
{
	(void)kctx;
}

/**
 * gpu_slc_kctx_active() - Called when a kernel context is (re)activated
 *
 * @kctx: The &struct kbase_context that is now active
 */
void gpu_slc_kctx_active(struct kbase_context *kctx)
{
	(void)kctx;
}

/**
 * gpu_slc_kctx_idle() - Called when a kernel context is idled
 *
 * @kctx: The &struct kbase_context that is now idle
 */
void gpu_slc_kctx_idle(struct kbase_context *kctx)
{
	(void)kctx;
}

/**
 * gpu_slc_init - Initialize the SLC context for the GPU
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: On success, returns 0. On failure an error code is returned.
 */
int gpu_slc_init(struct kbase_device *kbdev)
{
	return 0;
}

/**
 * gpu_slc_term() - Terminates the Pixel GPU SLC context.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 */
void gpu_slc_term(struct kbase_device *kbdev)
{
	(void)kbdev;
}
