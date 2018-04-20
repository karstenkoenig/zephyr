/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <misc/__assert.h>
#include <board.h>
#include <string.h>
#include <uart.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* Change this if you have an LED connected to a custom port */
#define PORT0   LED0_GPIO_PORT
#define PORT1   LED1_GPIO_PORT

/* Change this if you have an LED connected to a custom pin */
#define LED0    LED0_GPIO_PIN
#define LED1    LED1_GPIO_PIN


K_PIPE_DEFINE(my_pipe, 128, 4);


void uart_in(void)
{
	static struct device *uart_console_dev;
	uart_console_dev = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	unsigned char uart_in_char = 0;
	size_t bytes_written;
	while (1) {
		if (uart_poll_in(uart_console_dev, &uart_in_char) == 0) {
			k_pipe_put(&my_pipe, &uart_in_char, 1, &bytes_written, 1, K_FOREVER);
		}
	}
}

void uart_out(void)
{
	static struct device *uart_console_dev;
	uart_console_dev = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	unsigned char uart_out_char = 0;
	size_t bytes_read;

	while (1) {
		k_pipe_get(&my_pipe, &uart_out_char, 1, &bytes_read, 1, K_FOREVER);
		if (bytes_read == 1) {
			uart_poll_out(uart_console_dev, uart_out_char);
		}

	}
}

K_THREAD_DEFINE(uart_in_id, STACKSIZE, uart_in, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);

K_THREAD_DEFINE(uart_out_id, STACKSIZE, uart_out, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);
