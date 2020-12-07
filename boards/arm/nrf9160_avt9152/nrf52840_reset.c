/*
 * Copyright (c) 2019 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <device.h>

#define NRF52_RESET_NODE    DT_ALIAS(nrf52_reset)

#define RESET_PORT          DT_GPIO_LABEL(NRF52_RESET_NODE, gpios)
#define RESET_PIN           DT_GPIO_PIN(NRF52_RESET_NODE, gpios)



int bt_hci_transport_setup(const struct device *h4)
{
	int err;
	char c;
	const struct device *port;

	port = device_get_binding(RESET_PORT);
	if (!port) {
		return -EIO;
	}

	err = gpio_pin_configure(port, RESET_PIN, GPIO_OUTPUT_LOW);
	if (err) {
		return err;
	}

	/* Wait for the nRF52840 peripheral to stop sending data.
	 *
	 * It is critical (!) to wait here, so that all bytes
	 * on the lines are received and drained correctly.
	 */
	k_sleep(K_MSEC(10));

	/* Drain bytes */
	while (uart_fifo_read(h4, &c, 1)) {
		continue;
	}

	/* We are ready, let the nRF52840 run to main */
	err = gpio_pin_set(port, RESET_PIN, 1);
	if (err) {
		return err;
	}

	return 0;
}
