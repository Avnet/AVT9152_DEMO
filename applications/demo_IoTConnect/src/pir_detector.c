/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */

#include <zephyr.h>
#include <device.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include "te23142771_def.h"
#include "pir_detector.h"


#include <logging/log.h>
LOG_MODULE_REGISTER(pir, LOG_LEVEL_INF);

#define U5_I2C_NODE                 DT_ALIAS(sens_i2c)
#define U5_I2C_DEVICE               DT_LABEL(U5_I2C_NODE)

#define U5_EVENT_OUT_NODE           DT_ALIAS(ard_d0)
#define U5_EVENT_OUT_PORT_DEVICE    DT_GPIO_LABEL(U5_EVENT_OUT_NODE, gpios)
#define U5_EVENT_OUT_PIN_NUMBER     DT_GPIO_PIN(U5_EVENT_OUT_NODE, gpios)


static const struct device      *u5_i2c_dev = NULL;
static const struct device      *u5_event_out_port_dev = NULL;
static struct gpio_callback     u5_event_out_cb;

static struct k_work_q          *caller_work_q = NULL;
static struct k_delayed_work    event_out_detect_work;
static struct k_delayed_work    observe_cool_off_work;

static pir_event_handler_t      handler = NULL;

#define PIR_DETECT_COOL_OFF_PERIOD_SEC  2

static uint32_t    pir_on;
static uint32_t    prev_pir_on;

static void analyze_data(void)
{
    if ((prev_pir_on != pir_on) && ((prev_pir_on == 0) || (pir_on == 0))) {
        bool pir_detected = (prev_pir_on == 0);

        LOG_DBG("U5 2314277-1 Ambimate PIR/MOTION %s %d %d.\r\n", 
                        pir_detected ? "ON" : "OFF", prev_pir_on, pir_on);
        prev_pir_on = pir_on;
        if (handler != NULL) {
            handler(pir_detected);
        }
    }
}

static void event_out_detect_work_fn(struct k_work *work)
{
    int err;
    uint8_t reg_value;

    if (pir_on > 0) {
        k_delayed_work_cancel(&observe_cool_off_work);
        pir_on++;
        if (pir_on == 0) {
            pir_on = 1;
        }
    } else {
        pir_on++;
    }
    err = k_delayed_work_submit_to_queue(caller_work_q, &observe_cool_off_work, 
                                    K_SECONDS(PIR_DETECT_COOL_OFF_PERIOD_SEC));
    if (err != 0) {
        LOG_ERR("k_delayed_work_submit_to_queue(observe_cool_off) "
                        "failed (err: %d)", err);
    }

    err = i2c_reg_read_byte(u5_i2c_dev, TE_2314277_1_7BIT_ADDR, 
                            TE_2314277_1_REG_STATUS, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read TE_2314277_1_REG_STATUS");
        return;
    }
    LOG_DBG("TE_2314277_1_REG_STATUS 0x%02x", reg_value);

    analyze_data();
}

static void observe_cool_off_work_fn(struct k_work *work)
{
    pir_on = 0;
    analyze_data();
}

static void u5_event_out_asserted(const struct device *gpiob, 
                                    struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("u5_event_out_asserted");
    int err = k_delayed_work_submit_to_queue(caller_work_q, 
                                        &event_out_detect_work, K_NO_WAIT);
    if (err != 0) {
        LOG_ERR("k_delayed_work_submit_to_queue(event_out_detect) "
                        "failed (err: %d)", err);
    }
}

static void deconfig_int_pins(void)
{
    if (u5_event_out_port_dev != NULL) {
        (void) gpio_pin_interrupt_configure(u5_event_out_port_dev, 
                                    U5_EVENT_OUT_PIN_NUMBER, GPIO_INT_DISABLE);
        (void) gpio_remove_callback(u5_event_out_port_dev, &u5_event_out_cb);
        (void) gpio_pin_configure(u5_event_out_port_dev, 
                                    U5_EVENT_OUT_PIN_NUMBER, GPIO_DISCONNECTED);
    }
}

static int config_int_pins(void)
{
    int err;

    u5_event_out_port_dev = device_get_binding(U5_EVENT_OUT_PORT_DEVICE);
    if (u5_event_out_port_dev == NULL) {
        LOG_ERR("%s pin not found",  DT_PROP(U5_EVENT_OUT_NODE, label));
        return -ENODEV;
    }

    err = gpio_pin_configure(u5_event_out_port_dev, U5_EVENT_OUT_PIN_NUMBER, 
                                GPIO_INPUT | GPIO_PULL_DOWN);
    if (err != 0) {
        LOG_ERR("%s pin configure failed %d", 
                    DT_PROP(U5_EVENT_OUT_NODE, label), err);
        goto cleanup;
    }
    gpio_init_callback(&u5_event_out_cb, u5_event_out_asserted, 
                        BIT(U5_EVENT_OUT_PIN_NUMBER));
    err = gpio_add_callback(u5_event_out_port_dev, &u5_event_out_cb);
    if (err != 0) {
        LOG_ERR("%s pin add callback failed %d", 
                    DT_PROP(U5_EVENT_OUT_NODE, label), err);
        goto cleanup;
    }
    err = gpio_pin_interrupt_configure(u5_event_out_port_dev, 
                            U5_EVENT_OUT_PIN_NUMBER, GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0) {
        LOG_ERR("%s pin interrupt configure failed %d", 
                    DT_PROP(U5_EVENT_OUT_NODE, label), err);
        goto cleanup;
    }
    LOG_DBG("U5 EVENT OUT (%s pin) configured", 
                DT_PROP(U5_EVENT_OUT_NODE, label));

    return 0;

cleanup:
    deconfig_int_pins();
    return err;
}

int pir_detector_init_and_start(struct k_work_q *work_q, 
                                pir_event_handler_t event_handler)
{
    int err;
    uint8_t reg_value;

    if ((work_q == NULL) || (event_handler == NULL)) {
        return -EINVAL;
    }

    if (caller_work_q != NULL) {
        /* already init and started */
        return -EALREADY;
    }

    err = config_int_pins();
    if (err != 0) {
        return err;
    }

    caller_work_q = work_q;
    handler = event_handler;

    k_delayed_work_init(&event_out_detect_work, event_out_detect_work_fn);
    k_delayed_work_init(&observe_cool_off_work, observe_cool_off_work_fn);

    u5_i2c_dev = device_get_binding(U5_I2C_DEVICE);
    if (u5_i2c_dev == NULL) {
        LOG_ERR("TE 2314277-1 not found");
        err = -ENODEV;
        goto cleanup;
    }

    err = i2c_reg_read_byte(u5_i2c_dev, TE_2314277_1_7BIT_ADDR, 
                            TE_2314277_1_REG_OPTIONAL_SENSORS, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read TE_2314277_1_REG_OPTIONAL_SENSORS %d", err);
        goto cleanup;
    }
    /* TE sensor installed on AVT9152 has no optional sensors */
    if (reg_value != TE_2314277_1_OPTIONAL_SENSORS_NONE) {
        LOG_ERR("Unexpected TE_2314277_1_OPTIONAL_SENSORS_NONE value 0x%02x", 
                    reg_value);
        err = -ENOTSUP;
        goto cleanup;
    }

    /* to clear event out */
    err = i2c_reg_read_byte(u5_i2c_dev, TE_2314277_1_7BIT_ADDR, 
                            TE_2314277_1_REG_STATUS, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read TE_2314277_1_REG_STATUS");
        goto cleanup;
    }

    return 0;

cleanup:
    handler = NULL;
    caller_work_q = NULL;
    deconfig_int_pins();
    return err;
}

void pir_detector_stop_and_unit(void)
{
    if (caller_work_q == NULL) {
        /* not init and started */
        return;
    }

    handler = NULL;
    caller_work_q = NULL;
    deconfig_int_pins();
}

