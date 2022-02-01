/*
 * Copyright (c) 2022, matthias.fend@emfend.at
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_mailbox

#include <errno.h>
#include <string.h>
#include <device.h>
#include <drivers/ipm.h>
#include <fsl_mu.h>

/* This MU supports four receive and four transmit registers with a width of 32 bits */
#define IMX_REV2_REGISTER_COUNT    4
#define IMX_REV2_MAX_DATA_SIZE     sizeof(uint32_t)

struct imx_rev2_config {
	MU_Type *base;
	void (*irq_config_func)(const struct device *dev);
};

struct imx_rev2_data {
	ipm_callback_t callback;
	void *user_data;
};

static inline uint32_t get_rx_full_flag(uint32_t id)
{
	const uint32_t flag[] = {
		[0] = kMU_Rx0FullFlag,
		[1] = kMU_Rx1FullFlag,
		[2] = kMU_Rx2FullFlag,
		[3] = kMU_Rx3FullFlag,
	};

	if (id < ARRAY_SIZE(flag)) {
		return flag[id];
	}

	return 0;
}

static inline uint32_t get_tx_empty_flag(uint32_t id)
{
	const uint32_t flag[] = {
		[0] = kMU_Tx0EmptyFlag,
		[1] = kMU_Tx1EmptyFlag,
		[2] = kMU_Tx2EmptyFlag,
		[3] = kMU_Tx3EmptyFlag,
	};

	if (id < ARRAY_SIZE(flag)) {
		return flag[id];
	}

	return 0;
}

static void imx_rev2_isr(const struct device *dev)
{
	const struct imx_rev2_config *config = dev->config;
	struct imx_rev2_data *data = dev->data;
	MU_Type *base = config->base;
	uint32_t status_flags;
	uint32_t data32;
	uint32_t id;

	status_flags = MU_GetStatusFlags(base);

	for (id = 0; id < IMX_REV2_REGISTER_COUNT; id++) {
		if (status_flags & get_rx_full_flag(id)) {
			data32 = MU_ReceiveMsg(base, id);
			if (data->callback) {
				data->callback(dev, data->user_data, id, &data32);
			}
		}
	}

	/* ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
	 * exception return operation might vector to incorrect interrupt.
	 * For Cortex-M7, if core speed much faster than peripheral register write speed,
	 * the peripheral interrupt flags may be still set after exiting ISR, this results to
	 * the same error similar with errata 83869
	 */
#if (defined __CORTEX_M) && ((__CORTEX_M == 4U) || (__CORTEX_M == 7U))
	__DSB();
#endif
}

static int imx_rev2_ipm_send(const struct device *dev, int wait, uint32_t id, const void *data,
			     int size)
{
	const struct imx_rev2_config *config = dev->config;
	MU_Type *base = config->base;
	uint32_t data32;
	const uint32_t tx_empty_flag = get_tx_empty_flag(id);

	if (id >= IMX_REV2_REGISTER_COUNT) {
		return -EINVAL;
	}

	if (size > IMX_REV2_MAX_DATA_SIZE) {
		return -EMSGSIZE;
	}

	if (!(MU_GetStatusFlags(base) & tx_empty_flag)) {
		return -EBUSY;
	}

	data32 = 0;
	memcpy(&data32, data, size);
	MU_SendMsgNonBlocking(base, id, data32);

	if (wait) {
		while (!(MU_GetStatusFlags(base) & tx_empty_flag)) {
		}
	}

	return 0;
}

static int imx_rev2_ipm_max_data_size_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IMX_REV2_MAX_DATA_SIZE;
}

static uint32_t imx_rev2_ipm_max_id_val_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return (IMX_REV2_REGISTER_COUNT - 1);
}

static void imx_rev2_ipm_register_callback(const struct device *dev, ipm_callback_t cb,
					   void *user_data)
{
	struct imx_rev2_data *driver_data = dev->data;

	driver_data->callback = cb;
	driver_data->user_data = user_data;
}

static int imx_rev2_ipm_set_enabled(const struct device *dev, int enable)
{
	const struct imx_rev2_config *config = dev->config;
	const uint32_t mask = kMU_Rx0FullInterruptEnable | kMU_Rx1FullInterruptEnable |
			      kMU_Rx2FullInterruptEnable | kMU_Rx3FullInterruptEnable;

	if (enable) {
		MU_EnableInterrupts(config->base, mask);
	} else {
		MU_DisableInterrupts(config->base, mask);
	}

	return 0;
}

static int imx_rev2_init(const struct device *dev)
{
	const struct imx_rev2_config *config = dev->config;

	MU_Init(config->base);

	config->irq_config_func(dev);

	return 0;
}

static const struct ipm_driver_api imx_rev2_driver_api = {
	.send = imx_rev2_ipm_send,
	.register_callback = imx_rev2_ipm_register_callback,
	.max_data_size_get = imx_rev2_ipm_max_data_size_get,
	.max_id_val_get = imx_rev2_ipm_max_id_val_get,
	.set_enabled = imx_rev2_ipm_set_enabled
};

static void imx_rev2_irq_config_func(const struct device *dev);

static const struct imx_rev2_config imx_rev2_config = {
	.base = (MU_Type *)DT_INST_REG_ADDR(0),
	.irq_config_func = imx_rev2_irq_config_func,
};

static struct imx_rev2_data imx_rev2_data;

DEVICE_DT_INST_DEFINE(0, &imx_rev2_init, NULL, &imx_rev2_data, &imx_rev2_config, PRE_KERNEL_1,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &imx_rev2_driver_api);

static void imx_rev2_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), imx_rev2_isr, DEVICE_DT_INST_GET(0),
		    0);

	irq_enable(DT_INST_IRQN(0));
}
