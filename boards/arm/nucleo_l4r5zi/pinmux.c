/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <pinmux.h>
#include <sys_io.h>
#include <soc.h>
#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for NUCLEO-L4R5ZI board */
static const struct pin_config pinconf[] = {
#ifdef CONFIG_UART_STM32_PORT_1
	{STM32_PIN_PA9,  STM32L4X_PINMUX_FUNC_PA9_USART1_TX},
	{STM32_PIN_PA10, STM32L4X_PINMUX_FUNC_PA10_USART1_RX},
#endif	/* CONFIG_UART_STM32_PORT_1 */
#ifdef CONFIG_UART_STM32_PORT_2
	{STM32_PIN_PA2, STM32L4X_PINMUX_FUNC_PA2_USART2_TX},
	{STM32_PIN_PA3, STM32L4X_PINMUX_FUNC_PA3_USART2_RX},
#endif	/* CONFIG_UART_STM32_PORT_2 */
#ifdef CONFIG_UART_STM32_PORT_3
	{STM32_PIN_PB10, STM32L4X_PINMUX_FUNC_PB10_USART3_TX},
	{STM32_PIN_PB11, STM32L4X_PINMUX_FUNC_PB11_USART3_RX},
#endif	/* CONFIG_UART_STM32_PORT_3 */
#ifdef CONFIG_I2C_1
	{STM32_PIN_PB6, STM32L4X_PINMUX_FUNC_PB6_I2C1_SCL},
	{STM32_PIN_PB7, STM32L4X_PINMUX_FUNC_PB7_I2C1_SDA},
#endif /* CONFIG_I2C_1 */
#ifdef CONFIG_PWM_STM32_2
	{STM32_PIN_PA0, STM32L4X_PINMUX_FUNC_PA0_PWM2_CH1},
#endif /* CONFIG_PWM_STM32_2 */
#ifdef CONFIG_SPI_1
	/* SPI1 on the Arduino connectors pins A2, D3, D12, D11 */
	{STM32_PIN_PA4, STM32L4X_PINMUX_FUNC_PA4_SPI1_NSS},
	/* SPI1_SCK should output on PA5, but is used for LD2 */
	{STM32_PIN_PB3, STM32L4X_PINMUX_FUNC_PB3_SPI1_SCK},
	{STM32_PIN_PA6, STM32L4X_PINMUX_FUNC_PA6_SPI1_MISO},
	{STM32_PIN_PA7, STM32L4X_PINMUX_FUNC_PA7_SPI1_MOSI},
#endif /* CONFIG_SPI_1 */
#ifdef CONFIG_SPI_2
	/* SPI2 on the ST Morpho Connector CN10 pins 16, 30, 28, 26*/
	{STM32_PIN_PB12, STM32L4X_PINMUX_FUNC_PB12_SPI2_NSS},
	{STM32_PIN_PB13, STM32L4X_PINMUX_FUNC_PB13_SPI2_SCK},
	{STM32_PIN_PB14, STM32L4X_PINMUX_FUNC_PB14_SPI2_MISO},
	{STM32_PIN_PB15, STM32L4X_PINMUX_FUNC_PB15_SPI2_MOSI},
#endif /* CONFIG_SPI_1 */
#ifdef CONFIG_SPI_3
	/* SPI3 on the ST Morpho Connector CN7 pins 17, 1, 2, 3*/
	{STM32_PIN_PA15, STM32L4X_PINMUX_FUNC_PA15_SPI3_NSS},
	{STM32_PIN_PC10, STM32L4X_PINMUX_FUNC_PC10_SPI3_SCK},
	{STM32_PIN_PC11, STM32L4X_PINMUX_FUNC_PC11_SPI3_MISO},
	{STM32_PIN_PC12, STM32L4X_PINMUX_FUNC_PC12_SPI3_MOSI},
#endif /* CONFIG_SPI_3 */
};

static int pinmux_stm32_init(struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	/* Enable the VddIO2 rail without double checking.
	 *
	 * It is needed for PG[15:2] pins, which includes is LPUART1 PG7 and PG8
	 * are connected to the VCOM of the ST-Link
	 * For further information check 'Independent I/O supply rail' in the
	 * reference manual.
	 * */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	SET_BIT(PWR->CR2, PWR_CR2_IOSV);

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
	 CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
