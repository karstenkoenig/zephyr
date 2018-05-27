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

#define MCP2515_TX_CNT 	3

#define DEV_CFG(dev) ((const struct mcp2515_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct mcp2515_data *const)(dev)->driver_data)

struct mcp2515_frame {

};

struct mcp2515_tx_cb {
	struct k_sem sem;
	can_tx_callback_t cb;
};

struct mcp2515_data {
	struct device *spi;
	struct spi_config spi_cfg;
	struct device *int_gpio;
	struct gpio_callback int_gpio_cb;
	struct k_thread int_thread;
	k_thread_stack_t *int_thread_stack;
	struct k_sem int_sem;
	struct k_mutex tx_mutex;
	struct mcp2515_tx_cb tx_cb[MCP2515_TX_CNT];
	u8_t tx_busy_map;
	enum can_mode mode;
};

struct mcp2515_config {
	const char *spi_port;
	u8_t int_pin;
	const char *int_port;
	size_t int_thread_stack_size;
	int int_thread_priority;
	u8_t spi_cs_pin;
	const char *spi_cs_port;
	u32_t spi_freq;
	u8_t spi_slave;
};

#ifdef CONFIG_CAN_MCP2515_GPIO_SPI_CS
struct spi_cs_control mcp2515_cs_ctrl;
#endif

#define MCP2515_OPCODE_WRITE		0x02
#define MCP2515_OPCODE_READ			0x03
#define MCP2515_OPCODE_BIT_MODIFY	0x05
#define MCP2515_OPCODE_READ_STATUS	0xA0
#define MCP2515_OPCODE_RESET		0xC0


#define MCP2515_ADDR_CANSTAT		0x0E
#define MCP2515_ADDR_CANCTRL		0x0F
#define MCP2515_ADDR_CNF3			0x28
#define MCP2515_ADDR_CNF2			0x29
#define MCP2515_ADDR_CNF1			0x2A
#define MCP2515_ADDR_CANINTE		0x2B
#define MCP2515_ADDR_CANINTF		0x2C

#define MCP2515_MODE_NORMAL			0x00
#define MCP2515_MODE_LOOPBACK		0x02
#define MCP2515_MODE_SILENT			0x03
#define MCP2515_MODE_CONFIGURATION	0x04

/* MCP2515_STATUS */
#define MCP2515_STATUS_RX0IF			BIT(0)
#define MCP2515_STATUS_RX1IF			BIT(1)
#define MCP2515_STATUS_TX0REQ			BIT(2)
#define MCP2515_STATUS_TX0IF			BIT(3)
#define MCP2515_STATUS_TX1REQ			BIT(4)
#define MCP2515_STATUS_TX1IF			BIT(5)
#define MCP2515_STATUS_TX2REQ			BIT(6)
#define MCP2515_STATUS_TX2IF			BIT(7)

static int mcp2515_soft_reset(struct device *dev)
{
	struct mcp2515_data *dev_data = DEV_DATA(dev);
	u8_t opcode_buf[1] = { MCP2515_OPCODE_RESET };
	const struct spi_buf tx_buf = {
		.buf = opcode_buf,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	return spi_write(dev_data->spi, &dev_data->spi_cfg, &tx);
}

static int mcp2515_write_reg(struct device *dev, u8_t reg_addr, u8_t* buf_data, u8_t buf_len)
{
	struct mcp2515_data *dev_data = DEV_DATA(dev);

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

	return spi_write(dev_data->spi, &dev_data->spi_cfg, &tx);
}

static int mcp2515_bit_modify(struct device *dev, u8_t reg_addr, u8_t mask, u8_t data)
{
	struct mcp2515_data *dev_data = DEV_DATA(dev);

	u8_t cmd_buf[4];
	cmd_buf[0] = MCP2515_OPCODE_BIT_MODIFY;
	cmd_buf[1] = reg_addr;
	cmd_buf[2] = mask;
	cmd_buf[3] = data;

	struct spi_buf tx_buf[1];

	tx_buf[0].buf = cmd_buf;
	tx_buf[0].len = 4;

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 1
	};

	return spi_write(dev_data->spi, &dev_data->spi_cfg, &tx);
}


static int mcp2515_read_reg(struct device *dev, u8_t reg_addr, u8_t* buf_data, u8_t buf_len)
{
	struct mcp2515_data *dev_data = DEV_DATA(dev);

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

	return spi_transceive(dev_data->spi, &dev_data->spi_cfg, &tx, &rx);
}

static int mcp2515_read_status(struct device *dev, u8_t* status)
{
	struct mcp2515_data *dev_data = DEV_DATA(dev);
	u8_t opcode_buf[2] = { MCP2515_OPCODE_READ_STATUS , NULL};
	const struct spi_buf tx_buf = {
		.buf = opcode_buf,
		.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf[2];

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	rx_buf[1].buf = status;
	rx_buf[1].len = 1;

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	return spi_transceive(dev_data->spi, &dev_data->spi_cfg, &tx, &rx);
}

const int mcp2515_request_operation_mode(struct device *dev, u8_t mode)
{
	mcp2515_bit_modify(dev, MCP2515_ADDR_CANCTRL, 0x07 << 5, mode << 5);

	u8_t canstat;
	mcp2515_read_reg(dev, MCP2515_ADDR_CANSTAT, &canstat, 1);

	if ((canstat >> 5) != mode) {
		SYS_LOG_ERR("Failed to set MCP2515 operation mode");
		return -EIO;
	}

	return 0;
}

static u8_t mcp2515_get_mcp2515_opmode(enum can_mode mode)
{
	switch (mode) {
	case CAN_NORMAL_MODE:
		return MCP2515_MODE_NORMAL;
	case CAN_SILENT_MODE:
		return MCP2515_MODE_SILENT;
	case CAN_LOOPBACK_MODE:
		return MCP2515_MODE_LOOPBACK;
	default:
		SYS_LOG_ERR("Unsupported CAN Mode %u", mode);
		return MCP2515_MODE_SILENT;
	}
}

static void mcp2515_convert_can_msg_to_mcp2515_frame(const struct can_msg source, struct mcp2515_frame target)
{

}

static void mcp2515_convert_mcp2515_frame_to_can_msg()
{

}

static int mcp2515_configure(struct device *dev, enum can_mode mode,
		u32_t bitrate)
{
	struct mcp2515_data *dev_data = DEV_DATA(dev);

	u8_t config_buf[4]; // CNF3, CNF2, CNF1, CANINTE

	mcp2515_soft_reset(dev); // will enter configuration mode

#warning implement validation - modes and Time quants

	// CNF1; SJW<7:6> | BRP<5:0>
	const u8_t sjw = (CONFIG_CAN_SJW - 1) << 6;
	const u8_t bit_length = 1 + CONFIG_CAN_PROP_SEG + CONFIG_CAN_PHASE_SEG1 +
			CONFIG_CAN_PHASE_SEG2;
	// This could create a terrible bitrate for badly chosen parameters
	u8_t brp = (CONFIG_CAN_MCP2515_OSC_FREQ / (bit_length * bitrate * 2)) - 1;

	u8_t cnf1 = sjw | brp;

	// CNF2; BTLMODE<7>|SAM<6>|PHSEG1<5:3>|PRSEG<2:0>
	const u8_t btlmode = 1 << 7;
	const u8_t sam = 0 << 6;
	const u8_t phseg1 = (CONFIG_CAN_PHASE_SEG1 - 1) << 3;
	const u8_t prseg = (CONFIG_CAN_PROP_SEG - 1);

	const u8_t cnf2 = btlmode | sam | phseg1 | prseg;

	// CNF3; SOF<7>|WAKFIL<6>|UND<5:3>|PHSEG2<2:0>
	const u8_t sof = 0 << 7;
	const u8_t wakfil = 0 << 6;
	const u8_t und = 0 << 3;
	const u8_t phseg2 = (CONFIG_CAN_PHASE_SEG2 - 1);

	const u8_t cnf3 = sof | wakfil | und | phseg2;

	// CANINTE
	// MERRE<7>:WAKIE<6>:ERRIE<5>:TX2IE<4>:TX1IE<3>:TX0IE<2>:RX1IE<1>:RX0IE<0>
	const u8_t caninte = 0x1F; // all TX and RX buffer interrupts


	config_buf[0] = cnf3;
	config_buf[1] = cnf2;
	config_buf[2] = cnf1;
	config_buf[3] = caninte;

	mcp2515_write_reg(dev, MCP2515_ADDR_CNF3, config_buf, 4);

	dev_data->mode = mode;
	return mcp2515_request_operation_mode(dev, mcp2515_get_mcp2515_opmode(mode));
}

int mcp2515_send(struct device *dev, struct can_msg *msg, s32_t timeout,
		   can_tx_callback_t callback)
{

	int tx_idx = 0;
	struct mcp2515_data *dev_data = DEV_DATA(dev);

	if (timeout != K_NO_WAIT) {
		SYS_LOG_ERR("mcp2515_send timeout not supported");
		return CAN_TX_UNKNOWN;
	}

	k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);

	// find a free tx slot
	for (; tx_idx < MCP2515_TX_CNT; tx_idx++) {
		if ((BIT(tx_idx) & dev_data->tx_busy_map) == 0) {
			dev_data->tx_busy_map |= BIT(tx_idx);
			break;
		}
	}

	if (tx_idx == MCP2515_TX_CNT) {
		k_mutex_unlock(&dev_data->tx_mutex);
		SYS_LOG_WRN("no free tx fifo available");
		return CAN_TX_ERR;
	}

	if (callback != NULL) {
		dev_data->tx_cb[tx_idx].cb = callback;
	}

	// SPI send



	k_mutex_unlock(&dev_data->tx_mutex);

	if (callback == NULL) {
		k_sem_take(&dev_data->tx_cb[tx_idx].sem, K_FOREVER);
		k_sem_give(&dev_data->tx_cb[tx_idx].sem);
		return 0;
	}

	return 0;
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
static void mcp2515_rx(struct device *dev) {

}
static void mcp2515_handle_interrupts(struct device *dev)
{
	u8_t status;
	u8_t tx_cleared = 0;
	struct mcp2515_data *dev_data = DEV_DATA(dev);


	// mcp2515->send() calls can change the status
	k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);
	mcp2515_read_status(dev, &status);
	if (BIT(0) & dev_data->tx_busy_map) {
		if (status & (MCP2515_STATUS_TX0IF)) {
			tx_cleared |= BIT(0);
		}
	}
	if (BIT(1) & dev_data->tx_busy_map) {
		if (status & (MCP2515_STATUS_TX1IF)) {
			tx_cleared |= BIT(1);
		}
	}
	if (BIT(2) & dev_data->tx_busy_map) {
		if (status & (MCP2515_STATUS_TX2IF)) {
			tx_cleared |= BIT(2);
		}
	}
	k_mutex_unlock(&dev_data->tx_mutex);

	if (status & (MCP2515_STATUS_RX0IF | MCP2515_STATUS_RX1IF)) {
		mcp2515_rx(dev);
	}

	u8_t tx_idx = 0;
	for (; tx_idx < MCP2515_TX_CNT; tx_idx++) {
		if (BIT(tx_idx) & tx_cleared) {
			if (dev_data->tx_cb[tx_idx].cb == NULL) {
				k_sem_give(&dev_data->tx_cb[tx_idx].sem);
			} else {
				dev_data->tx_cb[tx_idx].cb(0); // TODO error feedback
			}
		}
		k_mutex_lock(&dev_data->tx_mutex, K_FOREVER);
		dev_data->tx_busy_map &= ~BIT(tx_idx);
		k_mutex_unlock(&dev_data->tx_mutex);
	}




}

static void mcp2515_int_thread(struct device *dev)
{
	struct mcp2515_data *dev_data = DEV_DATA(dev);
	while (1) {
		k_sem_take(&dev_data->int_sem, K_FOREVER);
		mcp2515_handle_interrupts(dev);
	}
}

static void mcp2515_int_gpio_callback(struct device *dev,
				       struct gpio_callback *cb,
				       u32_t pins)
{
	struct mcp2515_data *dev_data =
		CONTAINER_OF(cb, struct mcp2515_data, int_gpio_cb);

	k_sem_give(&dev_data->int_sem);
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
	const struct mcp2515_config *dev_cfg = DEV_CFG(dev);
	struct mcp2515_data *dev_data = DEV_DATA(dev);

	dev_data->mode = CAN_SILENT_MODE;

	/* SPI config */
	dev_data->spi_cfg.operation = SPI_WORD_SET(8);
	dev_data->spi_cfg.frequency = dev_cfg->spi_freq;
	dev_data->spi_cfg.slave = dev_cfg->spi_slave;

	dev_data->spi = device_get_binding(dev_cfg->spi_port);
	if (!dev_data->spi) {
		SYS_LOG_ERR("SPI master port %s not found", dev_cfg->spi_port);
		return -EINVAL;
	}

#ifdef CAN_MCP2515_GPIO_SPI_CS
	mcp2515_cs_ctrl.gpio_dev = device_get_binding(dev_cfg->spi_cs_port);
	if (!mcp2515_cs_ctrl.gpio_dev) {
		SYS_LOG_ERR("Unable to get GPIO SPI CS device");
		return -ENODEV;
	}

	mcp2515_cs_ctrl.gpio_pin = CONFIG_MCP2515_GPIO_SPI_CS_PIN;
	mcp2515_cs_ctrl.delay = 0;

	dev_data->spi_cfg.cs = &mcp2515_cs_ctrl;
#else
	dev_data->spi_cfg.cs = NULL;
#endif /* CAN_MCP2515_GPIO_SPI_CS */


	/* Initialize INT GPIO */
	dev_data->int_gpio = device_get_binding(dev_cfg->int_port);
	if (dev_data->int_gpio == NULL) {
		SYS_LOG_ERR("GPIO port %s not found", dev_cfg->int_port);
		return -EINVAL;
	}

	if (gpio_pin_configure(dev_data->int_gpio, dev_cfg->int_pin,
			       (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE
			       | GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE))) {
		SYS_LOG_ERR("Unable to configure GPIO pin %u",
				dev_cfg->int_pin);
		return -EINVAL;
	}

	gpio_init_callback(&(dev_data->int_gpio_cb), mcp2515_int_gpio_callback,
			   BIT(dev_cfg->int_pin));

	if (gpio_add_callback(dev_data->int_gpio, &(dev_data->int_gpio_cb))) {
		return -EINVAL;
	}

	if (gpio_pin_enable_callback(dev_data->int_gpio, dev_cfg->int_pin)) {
		return -EINVAL;
	}

	if (mcp2515_soft_reset(dev)) {
		SYS_LOG_ERR("Soft-reset failed");

		return -EIO;
	}

	/* Start interruption handler thread */
	k_thread_create(&dev_data->int_thread, dev_data->int_thread_stack,
			dev_cfg->int_thread_stack_size,
			(k_thread_entry_t) mcp2515_int_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(dev_cfg->int_thread_priority), 0, K_NO_WAIT);

	dev_data->tx_cb[0].cb = NULL;
	k_sem_init(&dev_data->tx_cb[0].sem, 0, 1);
	dev_data->tx_cb[1].cb = NULL;
	k_sem_init(&dev_data->tx_cb[1].sem, 0, 1);
	dev_data->tx_cb[2].cb = NULL;
	k_sem_init(&dev_data->tx_cb[2].sem, 0, 1);

	dev_data->tx_busy_map = 0;

	return 0;
}

#ifdef CONFIG_CAN_1

static K_THREAD_STACK_DEFINE(mcp2515_int_thread_stack, CONFIG_CAN_MCP2515_INT_THREAD_STACK_SIZE);

static struct mcp2515_data mcp2515_data_1 = {
	.int_thread_stack = mcp2515_int_thread_stack,
	.int_sem  = _K_SEM_INITIALIZER(mcp2515_data_1.int_sem,
		       0, UINT_MAX),
};

static const struct mcp2515_config mcp2515_config_1 = {
	.spi_port = CONFIG_CAN_MCP2515_SPI_PORT_NAME,
	.spi_freq = CONFIG_CAN_MCP2515_SPI_FREQ,
	.spi_slave = CONFIG_CAN_MCP2515_SPI_SLAVE,
	.int_pin = CONFIG_CAN_MCP2515_INT_PIN,
	.int_port = CONFIG_CAN_MCP2515_INT_PORT_NAME,
	.int_thread_stack_size = CONFIG_CAN_MCP2515_INT_THREAD_STACK_SIZE,
	.int_thread_priority = CONFIG_CAN_MCP2515_INT_THREAD_PRIO,
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
