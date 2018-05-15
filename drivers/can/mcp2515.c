/*
 * Copyright (c) 2018 Karsten Koenig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include "can.h"

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_CAN_LEVEL
#include <logging/sys_log.h>


static int mcp2515_configure(struct device *dev, enum can_mode mode,
		u32_t bitrate)
{
	return -EIO;
}

int mcp2515_send(struct device *dev, struct can_msg *msg, s32_t timeout,
		   can_tx_callback_t callback)
{
	return CAN_TX_UNKNOWN;
}


int mcp2515_attach_msgq(struct device *dev, struct k_msgq *msgq,
			  const struct can_filter *filter)
{
	return CAN_NO_FREE_FILTER;
}

int mcp2515_attach_isr(struct device *dev, can_rx_callback_t isr,
			 const struct can_filter *filter)
{

	return CAN_NO_FREE_FILTER;
}

void mcp2515_detach(struct device *dev, int filter_nr)
{

}

static const struct can_driver_api can_api_funcs = {
	.configure = mcp2515_configure,
	.send = mcp2515_send,
	.attach_msgq = mcp2515_attach_msgq,
	.attach_isr = mcp2515_attach_isr,
	.detach = mcp2515_detach
};

#ifdef CONFIG_CAN_1


#endif /*CONFIG_CAN_1*/
