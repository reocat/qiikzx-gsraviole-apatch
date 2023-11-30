// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */

#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dev_printk.h>
/* Pixel integration includes */
#include "pixel_slc.h"


/**
 * DOC: PBHA
 *
 * Borr does not have "real" PBHA support. However, since we only use a 36-bit PA on the bus,
 * AxADDR[39:36] is wired up to the GPU AxUSER[PBHA] field seen by the rest of the system.
 * Those AxADDR bits come from [39:36] in the page descriptor.
 *
 * Odin and Turse have "real" PBHA support using a dedicated output signal and page descriptor field.
 * The AxUSER[PBHA] field is driven by the GPU's PBHA signal, and AxADDR[39:36] is dropped.
 * The page descriptor PBHA field is [62:59].
 *
 * We could write to both of these locations, as each SoC only reads from its respective PBHA
 * location with the other being ignored or dropped.
 *
 * b/148988078 contains confirmation of the above description.
 */
#if IS_ENABLED(CONFIG_SOC_GS101)
#define PBHA_BIT_POS  (36)
#else
#define PBHA_BIT_POS  (59)
#endif
#define PBHA_BIT_MASK (0xf)


/**
 * slc_wipe_pbha - Clear any set PBHA bits from the pte.
 *
 * @pte: The pte to strip of PBHA.
 *
 * Return: The PTE with all PBHA stripped.
 */
u64 slc_wipe_pbha(u64 pte)
{
	return pte & ~((u64)PBHA_BIT_MASK << PBHA_BIT_POS);
}

/**
 * slc_set_pbha - Apply the PBHA to @pte.
 *
 * @data: The &struct slc_data tracking partition information.
 * @pte:  The pte to modify.
 *
 * Return: On success, returns a modified PTE. On failure the original PTE is returned.
 */
u64 slc_set_pbha(struct slc_data const *data, u64 pte)
{
	/* Clear any bits set in the PBHA range */
	pte = slc_wipe_pbha(pte);

	/* Apply the PBHA for the given virtual partition */
	return pte | (((u64)data->partition.pbha) & PBHA_BIT_MASK) << PBHA_BIT_POS;
}

/**
 * enable_partition - Enable @pt
 *
 * @data: The &struct slc_data tracking partition information.
 * @pt:   The &struct slc_partition representing the partition to enable.
 */
static void enable_partition(struct slc_data *data, struct slc_partition *pt)
{
	/* Skip if already enabled */
	if (pt->enabled)
		return;

	(void)pt_client_enable(data->pt_handle, pt->index);
	pt->enabled = true;

	dev_dbg(data->dev, "enabled partition %d", pt->index);
}

/**
 * disable_partition - Disable @pt
 *
 * @data: The &struct slc_data tracking partition information.
 * @pt:   The &struct slc_partition representing the partition to disable.
 */
static void disable_partition(struct slc_data *data, struct slc_partition *pt)
{
	/* Skip if not enabled */
	if (!pt->enabled)
		return;

	pt_client_disable_no_free(data->pt_handle, pt->index);
	pt->enabled = false;

	dev_dbg(data->dev, "disabled partition %d", pt->index);
}

/**
 * init_partition - Register and initialize a partition with the SLC driver.
 *
 * @data:  The &struct slc_data tracking partition information.
 * @pt:    The &struct slc_partition to store the configured partition information.
 * @index: The index of the partition, relative to the DT node.
 *
 * Returns EINVAL on error, otherwise 0.
 */
static int init_partition(struct slc_data *data, struct slc_partition *pt, u32 index)
{
	ptid_t ptid;
	ptpbha_t pbha;
	int err = -EINVAL;

	ptid = pt_client_enable(data->pt_handle, index);
	if (ptid == PT_PTID_INVALID) {
		dev_err(data->dev, "failed to enable pt: %d\n", index);
		goto err_exit;
	}

	pbha = pt_pbha(data->dev->of_node, index);
	if (pbha == PT_PBHA_INVALID) {
		dev_err(data->dev, "failed to get PBHA for pt: %d\n", index);
		goto err_exit;
	}

	/* This retains the allocated ptid */
	pt_client_disable_no_free(data->pt_handle, index);

	/* Success */
	err = 0;

	*pt = (struct slc_partition) {
		.index = index,
		.ptid = ptid,
		.pbha = pbha,
		.enabled = false,
	};

err_exit:
	return err;
}


/**
 * term_partition - Disable and free a partition, unregistering it.
 *
 * @data:  The &struct slc_data tracking partition information.
 * @pt:    The &struct slc_partition to terminate.
 *
 * Returns EINVAL on error, otherwise 0.
 */
static void term_partition(struct slc_data *data, struct slc_partition *pt)
{
	disable_partition(data, pt);
	pt_client_free(data->pt_handle, pt->index);
}

/**
 * slc_init_data - Read all SLC partition information, init the partitions, and track within @data.
 *
 * @data:  The &struct slc_data tracking partition information.
 * @dev:   The platform device associated with the parent node.
 *
 * Return: On success, returns 0. On failure an error code is returned.
 */
int slc_init_data(struct slc_data *data, struct device* dev)
{
	int ret = -EINVAL;

	if (data == NULL || dev == NULL)
		goto err_exit;

	/* Inherit the platform device */
	data->dev = dev;

	/* Register our node with the SLC driver.
	 * This detects our partitions defined within the DT.
	 */
	data->pt_handle = pt_client_register(data->dev->of_node, NULL, NULL);
	if (IS_ERR(data->pt_handle)) {
		ret = PTR_ERR(data->pt_handle);
		dev_err(data->dev, "pt_client_register failed with: %d\n", ret);
		goto err_exit;
	}

	if ((ret = init_partition(data, &data->partition, 0)))
		goto pt_init_err_exit;

	return 0;

pt_init_err_exit:
	pt_client_unregister(data->pt_handle);

err_exit:
	return ret;
}

/**
 * slc_term_data - Tear down SLC partitions and free tracking data.
 *
 * @data:  The &struct slc_data tracking partition information.
 */
void slc_term_data(struct slc_data *data)
{
	term_partition(data, &data->partition);

	pt_client_unregister(data->pt_handle);
}
