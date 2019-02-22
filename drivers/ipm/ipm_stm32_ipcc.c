/*
 * Copyright (c) 2019 ST Microelectronics Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <clock_control.h>
#include <device.h>
#include <errno.h>
#include <ipm.h>
#include <soc.h>

#include <clock_control/stm32_clock_control.h>

#define LOG_LEVEL CONFIG_IPM_LOG_LEVEL
#include <logging/log.h>

LOG_MODULE_REGISTER(ipm_stm32_ipcc);

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct stm32_ipcc_mailbox_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)							\
	((struct stm32_ipcc_mbx_data * const)(dev)->driver_data)
#define MBX_STRUCT(dev)					\
	((IPCC_TypeDef *)(DEV_CFG(dev))->uconf.base)

#define IPCC_ALL_MR_TXF_CH_MASK 0xFFFF0000
#define IPCC_ALL_MR_RXO_CH_MASK 0x0000FFFF
#define IPCC_ALL_SR_CH_MASK	0x0000FFFF

#if (CONFIG_IPM_STM32_IPCC_PROCID == 1)

#define IPCC_EnableIT_TXF(hipcc)  LL_C1_IPCC_EnableIT_TXF(hipcc)
#define IPCC_DisableIT_TXF(hipcc)  LL_C1_IPCC_DisableIT_TXF(hipcc)

#define IPCC_EnableIT_RXO(hipcc)  LL_C1_IPCC_EnableIT_RXO(hipcc)
#define IPCC_DisableIT_RXO(hipcc)  LL_C1_IPCC_DisableIT_RXO(hipcc)

#define IPCC_EnableReceiveChannel(hipcc, ch)	\
			LL_C1_IPCC_EnableReceiveChannel(hipcc, 1 << ch)
#define IPCC_EnableTransmitChannel(hipcc, ch)	\
			LL_C1_IPCC_EnableTransmitChannel(hipcc, 1 << ch)
#define IPCC_DisableReceiveChannel(hipcc, ch)	\
			LL_C2_IPCC_DisableReceiveChannel(hipcc, 1 << ch)
#define IPCC_DisableTransmitChannel(hipcc, ch)	\
			LL_C1_IPCC_DisableTransmitChannel(hipcc, 1 << ch)

#define IPCC_ClearFlag_CHx(hipcc, ch)	LL_C1_IPCC_ClearFlag_CHx(hipcc, 1 << ch)
#define IPCC_SetFlag_CHx(hipcc, ch)	LL_C1_IPCC_SetFlag_CHx(hipcc, 1 << ch)

#define IPCC_IsActiveFlag_CHx(hipcc, ch)	\
			LL_C1_IPCC_IsActiveFlag_CHx(hipcc, 1 << ch)

#define IPCC_ReadReg(hipcc, reg) READ_REG(hipcc->C1##reg)

#define IPCC_ReadReg_SR(hipcc) READ_REG(hipcc->C1TOC2SR)
#define IPCC_ReadOtherInstReg_SR(hipcc) READ_REG(hipcc->C2TOC1SR)

#else

#define IPCC_EnableIT_TXF(hipcc)  LL_C2_IPCC_EnableIT_TXF(hipcc)
#define IPCC_DisableIT_TXF(hipcc)  LL_C2_IPCC_DisableIT_TXF(hipcc)

#define IPCC_EnableIT_RXO(hipcc)  LL_C2_IPCC_EnableIT_RXO(hipcc)
#define IPCC_DisableIT_RXO(hipcc)  LL_C2_IPCC_DisableIT_RXO(hipcc)

#define IPCC_EnableReceiveChannel(hipcc, ch)	\
			LL_C2_IPCC_EnableReceiveChannel(hipcc, 1 << ch)
#define IPCC_EnableTransmitChannel(hipcc, ch)	\
			LL_C2_IPCC_EnableTransmitChannel(hipcc, 1 << ch)
#define IPCC_DisableReceiveChannel(hipcc, ch)	\
			LL_C2_IPCC_DisableReceiveChannel(hipcc, 1 << ch)
#define IPCC_DisableTransmitChannel(hipcc, ch)	\
			LL_C2_IPCC_DisableTransmitChannel(hipcc, 1 << ch)

#define IPCC_ClearFlag_CHx(hipcc, ch)	LL_C2_IPCC_ClearFlag_CHx(hipcc, 1 << ch)
#define IPCC_SetFlag_CHx(hipcc, ch)	LL_C2_IPCC_SetFlag_CHx(hipcc, 1 << ch)

#define IPCC_IsActiveFlag_CHx(hipcc, ch)	\
			LL_C2_IPCC_IsActiveFlag_CHx(hipcc, 1 << ch)

#define IPCC_ReadReg(hipcc, reg) READ_REG(hipcc->C2##reg)

#define IPCC_ReadReg_SR(hipcc) READ_REG(hipcc->C2TOC1SR)
#define IPCC_ReadOtherInstReg_SR(hipcc) READ_REG(hipcc->C1TOC2SR)

#endif

struct stm32_ipcc_mailbox_config {
	void (*irq_config_func)(struct device *dev);
	struct stm32_pclken pclken;
};

struct stm32_ipcc_mbx_data {
	IPCC_TypeDef *hipcc;
	u32_t num_ch;
	ipm_callback_t callback;
	void *callback_ctx;
	struct device *clock;
};

static struct stm32_ipcc_mbx_data stm32_IPCC_data;

static void stm32_ipcc_mailbox_rx_isr(void *arg)
{
	struct device *dev = arg;
	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	unsigned int value = 0;
	u32_t mask, i;

	mask = (~IPCC_ReadReg(data->hipcc, MR)) & IPCC_ALL_MR_RXO_CH_MASK;
	mask &= IPCC_ReadOtherInstReg_SR(data->hipcc) & IPCC_ALL_SR_CH_MASK;

	for (i = 0; i < data->num_ch; i++) {
		if (!((1 << i) & mask)) {
			continue;
		}
		LOG_DBG("%s channel = %x\r\n", __func__, i);
		/* mask the channel Free interrupt  */
		IPCC_DisableReceiveChannel(data->hipcc, i);

		if (data->callback) {
			/* Only one MAILBOX, id is unused and set to 0 */
			data->callback(data->callback_ctx, i, &value);
		}
		/* clear status to acknoledge message reception */
		IPCC_ClearFlag_CHx(data->hipcc, i);
		IPCC_EnableReceiveChannel(data->hipcc, i);
	}
}

static void stm32_ipcc_mailbox_tx_isr(void *arg)
{
	struct device *dev = arg;
	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	u32_t mask, i;

	mask = (~IPCC_ReadReg(data->hipcc, MR)) & IPCC_ALL_MR_TXF_CH_MASK;
	mask = mask >> IPCC_C1MR_CH1FM_Pos;

	mask &= IPCC_ReadReg_SR(data->hipcc) & IPCC_ALL_SR_CH_MASK;

	for (i = 0; i <  data->num_ch; i++) {
		if (!((1 << i) & mask)) {
			continue;
		}
		LOG_DBG("%s channel = %x\r\n", __func__, i);
		/* mask the channel Free interrupt */
		IPCC_DisableTransmitChannel(data->hipcc, i);
	}
}

static int stm32_ipcc_mailbox_ipm_send(struct device *d, int wait, u32_t id,
				       const void *buff, int size)
{
	struct stm32_ipcc_mbx_data *data = d->driver_data;

	ARG_UNUSED(wait);
	ARG_UNUSED(buff);
	ARG_UNUSED(size);

	if (id >= data->num_ch) {
		LOG_ERR("invalid id (%d)\r\n", id);
		return  -EINVAL;
	}

	LOG_DBG("Send msg on channel %d\r\n", id);

	/* Check that the channel is free (otherwise return error) */
	if (IPCC_IsActiveFlag_CHx(data->hipcc, id)) {
		LOG_DBG("Waiting for channel to be freed\r\n");
		while (IPCC_IsActiveFlag_CHx(data->hipcc, id)) {
			;
		}
	}
	IPCC_EnableTransmitChannel(data->hipcc, id);
	/* Inform A7 (either new message, or buf free) */
	IPCC_SetFlag_CHx(data->hipcc, id);

	return 0;
}

static int stm32_ipcc_mailbox_ipm_max_data_size_get(struct device *d)
{
	ARG_UNUSED(d);

	/* Only a single 32-bit register available */
	return 1;
}

static u32_t stm32_ipcc_mailbox_ipm_max_id_val_get(struct device *d)
{
	struct stm32_ipcc_mbx_data *data = DEV_DATA(d);

	return data->num_ch - 1;
}

static void stm32_ipcc_mailbox_ipm_register_callback(struct device *d,
						     ipm_callback_t cb,
						     void *context)
{
	struct stm32_ipcc_mbx_data *data = DEV_DATA(d);

	data->callback = cb;
	data->callback_ctx = context;
}

static int stm32_ipcc_mailbox_ipm_set_enabled(struct device *dev, int enable)
{
	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	u32_t i;

	/* For now: nothing to be done */
	LOG_DBG("%s %s mailbox\r\n", __func__, enable ? "enable" : "disable");
	if (enable) {
		/* Disable RX and TX interrupts */
		IPCC_EnableIT_TXF(data->hipcc);
		IPCC_EnableIT_RXO(data->hipcc);
		for (i = 0; i < data->num_ch; i++) {
			IPCC_EnableReceiveChannel(data->hipcc, i);
		}
	} else {
		/* Disable RX and TX interrupts */
		IPCC_DisableIT_TXF(data->hipcc);
		IPCC_DisableIT_RXO(data->hipcc);
		for (i = 0; i < data->num_ch; i++) {
			IPCC_EnableReceiveChannel(data->hipcc, i);
		}
	}

	return 0;
}

static inline void _stm32_ipcc_get_clock(struct device *dev)
{
	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);

	__ASSERT_NO_MSG(clk);

	data->clock = clk;
}

static int stm32_ipcc_mailbox_init(struct device *dev)
{

	struct stm32_ipcc_mbx_data *data = DEV_DATA(dev);
	const struct stm32_ipcc_mailbox_config *config = DEV_CFG(dev);
	u32_t i;

	_stm32_ipcc_get_clock(dev);
	/* enable clock */
	if (clock_control_on(data->clock,
			     (clock_control_subsys_t *)&config->pclken) != 0) {
		return -EIO;
	}

	data->hipcc = IPCC;

	/* Disable RX and TX interrupts */
	IPCC_DisableIT_TXF(data->hipcc);
	IPCC_DisableIT_RXO(data->hipcc);

	data->num_ch = LL_IPCC_GetChannelConfig(data->hipcc);

	for (i = 0; i < data->num_ch; i++) {
		/* Clear RX status */
		IPCC_ClearFlag_CHx(data->hipcc, i);
		/* mask RX and TX interrupts */
		IPCC_DisableReceiveChannel(data->hipcc, i);
		IPCC_DisableTransmitChannel(data->hipcc, i);
	}

	config->irq_config_func(dev);

	return 0;
}

static const struct ipm_driver_api stm32_ipcc_mailbox_driver_api = {
	.send = stm32_ipcc_mailbox_ipm_send,
	.register_callback = stm32_ipcc_mailbox_ipm_register_callback,
	.max_data_size_get = stm32_ipcc_mailbox_ipm_max_data_size_get,
	.max_id_val_get = stm32_ipcc_mailbox_ipm_max_id_val_get,
	.set_enabled = stm32_ipcc_mailbox_ipm_set_enabled,
};

static void stm32_ipcc_mailbox_config_func(struct device *dev);

/* Config MAILBOX 0 */
static const struct stm32_ipcc_mailbox_config stm32_ipcc_mailbox_0_config = {
	.irq_config_func = stm32_ipcc_mailbox_config_func,
	.pclken = { .bus = DT_INST_0_ST_STM32_IPCC_MAILBOX_CLOCK_BUS,
		    .enr = DT_INST_0_ST_STM32_IPCC_MAILBOX_CLOCK_BITS
	},

};

DEVICE_AND_API_INIT(mailbox_0, DT_INST_0_ST_STM32_IPCC_MAILBOX_LABEL,
		    &stm32_ipcc_mailbox_init,
		    &stm32_IPCC_data, &stm32_ipcc_mailbox_0_config,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &stm32_ipcc_mailbox_driver_api);

static void stm32_ipcc_mailbox_config_func(struct device *dev)
{
	IRQ_CONNECT(DT_INST_0_ST_STM32_IPCC_MAILBOX_IRQ_RXO,
		    DT_INST_0_ST_STM32_IPCC_MAILBOX_IRQ_RXO_PRIORITY,
		    stm32_ipcc_mailbox_rx_isr, DEVICE_GET(mailbox_0), 0);

	IRQ_CONNECT(DT_INST_0_ST_STM32_IPCC_MAILBOX_IRQ_TXF,
		    DT_INST_0_ST_STM32_IPCC_MAILBOX_IRQ_TXF_PRIORITY,
		    stm32_ipcc_mailbox_tx_isr, DEVICE_GET(mailbox_0), 0);

	irq_enable(DT_INST_0_ST_STM32_IPCC_MAILBOX_IRQ_RXO);
	irq_enable(DT_INST_0_ST_STM32_IPCC_MAILBOX_IRQ_TXF);
}
