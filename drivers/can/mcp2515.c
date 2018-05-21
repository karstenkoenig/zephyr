/*
 * Copyright (c) 2018 Karsten Koenig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <spi.h>
#include <gpio.h>
#include <can.h>

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_CAN_LEVEL
#include <logging/sys_log.h>



#define DEV_CFG(dev) ((const struct mcp2515_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct mcp2515_data *const)(dev)->driver_data)


struct mcp2515_data {
	struct device *spi;
	struct spi_config spi_cfg;
	struct device *int_gpio;
	struct gpio_callback int_gpio_cb;
};

struct mcp2515_config {
	const char *spi_port;
	u8_t int_pin;
	const char *int_port;
	u8_t spi_cs_pin;
	const char *spi_cs_port;
	u32_t spi_freq;
	u8_t spi_slave;
};

#ifdef CONFIG_CAN_MCP2515_GPIO_SPI_CS
struct spi_cs_control mcp2515_cs_ctrl;
#endif

#define MCP2515_OPCODE_RESET	0xC0
#define MCP2515_OPCODE_READ		0x03
#define MCP2515_OPCODE_WRITE	0x02

static int mcp2515_soft_reset(struct device *dev)
{
	struct mcp2515_data *data = DEV_DATA(dev);
	u8_t opcode_buf[1] = { MCP2515_OPCODE_RESET };
	const struct spi_buf tx_buf = {
		.buf = opcode_buf,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	return spi_write(data->spi, &data->spi_cfg, &tx);
}

static int mcp2515_write_reg(struct device *dev, u8_t reg_addr, u8_t* buf_data, u8_t buf_len)
{
	struct mcp2515_data *data = DEV_DATA(dev);

	u8_t opcode_buf[2];
	opcode_buf[0] = MCP2515_OPCODE_WRITE;
	opcode_buf[1] = reg_addr;

	struct spi_buf tx_buf[2];

	tx_buf[0].buf = opcode_buf;
	tx_buf[0].len = 2;

	tx_buf[1].buf = buf_data;
	tx_buf[1].len = buf_len;

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	return spi_write(data->spi, &data->spi_cfg, &tx);
}


static int mcp2515_read_reg(struct device *dev, u8_t reg_addr, u8_t* buf_data, u8_t buf_len)
{
	struct mcp2515_data *data = DEV_DATA(dev);

	u8_t opcode_buf[2];
	opcode_buf[0] = MCP2515_OPCODE_READ;
	opcode_buf[1] = reg_addr;

	struct spi_buf tx_buf[2];

	tx_buf[0].buf = opcode_buf;
	tx_buf[0].len = 2;

	tx_buf[1].buf = NULL;
	tx_buf[1].len = buf_len;

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	struct spi_buf rx_buf[2];

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 2;

	rx_buf[1].buf = buf_data;
	rx_buf[1].len = buf_len;

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	return spi_transceive(data->spi, &data->spi_cfg, &tx, &rx);
}

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

static void mcp2514_int_gpio_callback(struct device *dev,
				       struct gpio_callback *cb,
				       u32_t pins)
{
}

static const struct can_driver_api can_api_funcs = {
	.configure = mcp2515_configure,
	.send = mcp2515_send,
	.attach_msgq = mcp2515_attach_msgq,
	.attach_isr = mcp2515_attach_isr,
	.detach = mcp2515_detach
};


static int mcp2515_init(struct device *dev)
{
	const struct mcp2515_config *cfg = DEV_CFG(dev);
	struct mcp2515_data *data = DEV_DATA(dev);

	/* SPI config */
	data->spi_cfg.operation = SPI_WORD_SET(8);
	data->spi_cfg.frequency = cfg->spi_freq;
	data->spi_cfg.slave = cfg->spi_slave;

	data->spi = device_get_binding(cfg->spi_port);
	if (!data->spi) {
		SYS_LOG_ERR("SPI master port %s not found", cfg->spi_port);
		return -EINVAL;
	}

#ifdef CAN_MCP2515_GPIO_SPI_CS
	mcp2515_cs_ctrl.gpio_dev = device_get_binding(cfg->spi_cs_port);
	if (!mcp2515_cs_ctrl.gpio_dev) {
		SYS_LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	mcp2515_cs_ctrl.gpio_pin = CONFIG_MCP2515_GPIO_SPI_CS_PIN;
	mcp2515_cs_ctrl.delay = 0;

	data->spi_cfg.cs = &mcp2515_cs_ctrl;
#else
	data->spi_cfg.cs = NULL;
#endif /* CAN_MCP2515_GPIO_SPI_CS */


	/* Initialize INT GPIO */
	data->int_gpio = device_get_binding(cfg->int_port);
	if (data->int_gpio == NULL) {
		SYS_LOG_ERR("GPIO port %s not found", cfg->int_port);
		return -EINVAL;
	}

	if (gpio_pin_configure(data->int_gpio, cfg->int_pin,
			       (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE
			       | GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE))) {
		SYS_LOG_ERR("Unable to configure GPIO pin %u",
				cfg->int_pin);
		return -EINVAL;
	}

	gpio_init_callback(&(data->int_gpio_cb), mcp2514_int_gpio_callback,
			   BIT(cfg->int_pin));

	if (gpio_add_callback(data->int_gpio, &(data->int_gpio_cb))) {
		return -EINVAL;
	}

	if (gpio_pin_enable_callback(data->int_gpio, cfg->int_pin)) {
		return -EINVAL;
	}

	if (mcp2515_soft_reset(dev)) {
		SYS_LOG_ERR("Soft-reset failed");

		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_CAN_1

static struct mcp2515_data mcp2515_data_1;

static const struct mcp2515_config mcp2515_config_1 = {
	.spi_port = CONFIG_CAN_MCP2515_SPI_PORT_NAME,
	.spi_freq = CONFIG_CAN_MCP2515_SPI_FREQ,
	.spi_slave = CONFIG_CAN_MCP2515_SPI_SLAVE,
	.int_pin = CONFIG_CAN_MCP2515_INT_PIN,
	.int_port = CONFIG_CAN_MCP2515_INT_PORT_NAME,
#ifdef CONFIG_CAN_MCP2515_GPIO_SPI_CS
	.spi_cs_pin = CONFIG_CAN_MCP2515_SPI_CS_PIN,
	.spi_cs_port = CONFIG_CAN_MCP2515_SPI_CS_PORT_NAME,
#endif /* CAN_MCP2515_GPIO_SPI_CS */
};

DEVICE_AND_API_INIT(can_mcp2515_1, CONFIG_CAN_MCP2515_NAME, &mcp2515_init,
		&mcp2515_data_1, &mcp2515_config_1,
		POST_KERNEL, CONFIG_CAN_MCP2515_INIT_PRIORITY,
		&can_api_funcs);

#endif /* CONFIG_CAN_1 */
