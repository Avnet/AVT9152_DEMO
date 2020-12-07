/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <init.h>
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(board_control, CONFIG_BOARD_AVT9152EVB_LOG_LEVEL);

#define VDD_MEAS_EN_NODE 	DT_ALIAS(vdd_meas_enable)
#define PWR_GOOD_NODE		DT_ALIAS(pwr_good)
#define NRF91_RESET_NODE 	DT_ALIAS(nrf91_reset)

#define VDD_MEAS_EN_PORT	DT_GPIO_LABEL(VDD_MEAS_EN_NODE, gpios)
#define VDD_MEAS_EN_PIN		DT_GPIO_PIN(VDD_MEAS_EN_NODE, gpios)

#define PWR_GOOD_PORT		DT_GPIO_LABEL(PWR_GOOD_NODE, gpios)
#define PWR_GOOD_PIN		DT_GPIO_PIN(PWR_GOOD_NODE, gpios)

#define NRF91_RESET_PORT	DT_GPIO_LABEL(NRF91_RESET_NODE, gpios)
#define NRF91_RESET_PIN		DT_GPIO_PIN(NRF91_RESET_NODE, gpios)


static int init(const struct device *dev)
{
	int err;
	const struct device *p0;
	const struct device *p1;
	const struct device *p2;

	p0 = device_get_binding(VDD_MEAS_EN_PORT);
	if (!p0) {
		LOG_ERR("GPIO device " VDD_MEAS_EN_PORT	" not found!");
		return -EIO;
	}

	p1 = device_get_binding(PWR_GOOD_PORT);
	if (!p1) {
		LOG_ERR("GPIO device " PWR_GOOD_PORT " not found!");
		return -EIO;
	}

	p2 = device_get_binding(NRF91_RESET_PORT);
	if (!p1) {
		LOG_ERR("GPIO device " NRF91_RESET_PORT " not found!");
		return -EIO;
	}

	/* Configure pins on each port */
	gpio_flags_t flags = DT_GPIO_FLAGS(VDD_MEAS_EN_NODE, gpios);
	err = gpio_pin_configure(p0, VDD_MEAS_EN_PIN, GPIO_OUTPUT_LOW | flags);
	if (err) {
		LOG_ERR("Error while configuring pin P0.%02d (err: %d)", VDD_MEAS_EN_PIN, err);
		return -EIO;
	}
	
	flags = DT_GPIO_FLAGS(PWR_GOOD_NODE, gpios);
	err = gpio_pin_configure(p1, PWR_GOOD_PIN, GPIO_INPUT | flags);
	if (err) {
		LOG_ERR("Error while configuring pin P1.%02d (err: %d)", PWR_GOOD_PIN, err);
		return -EIO;
	}

	flags = DT_GPIO_FLAGS(NRF91_RESET_NODE, gpios);
	err = gpio_pin_configure(p1, NRF91_RESET_PIN, GPIO_OUTPUT_HIGH | flags);
	if (err) {
		LOG_ERR("Error while configuring pin P2.%02d (err: %d)", NRF91_RESET_PIN, err);
		return -EIO;
	}

	LOG_INF("Board configured.");

	return 0;
}

SYS_INIT(init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
