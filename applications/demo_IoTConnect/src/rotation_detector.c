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

#include "i3g4250d_def.h"
#include "rotation_detector.h"


#include <logging/log.h>
LOG_MODULE_REGISTER(rotation, LOG_LEVEL_INF);

#define MAX_OBSERVE_DURATION    5   /* seconds */
/* (mdps/digit) typical for FS = 2000 dps */
#define U3_SENSITIVITY          70.0f
/* 
 * zth at 35 °C fs 2000 ==> (0.04 * 10) + 187.5 ==> 187.8 dps 
 *                  ==> (187800 mdps / 70 mdps/lsb) ==> 2682.86 ==> 2683
 */
#define U3_ZTH                  2683

#define U3_I2C_NODE             DT_ALIAS(sens_i2c)
#define U3_I2C_DEVICE           DT_LABEL(U3_I2C_NODE)

#define U3_INT1_NODE            DT_ALIAS(ard_a2)
#define U3_INT1_PORT_DEVICE     DT_GPIO_LABEL(U3_INT1_NODE, gpios)
#define U3_INT1_PIN_NUMBER      DT_GPIO_PIN(U3_INT1_NODE, gpios)

#define U3_INT2_NODE            DT_ALIAS(ard_a3)
#define U3_INT2_PORT_DEVICE     DT_GPIO_LABEL(U3_INT2_NODE, gpios)
#define U3_INT2_PIN_NUMBER      DT_GPIO_PIN(U3_INT2_NODE, gpios)

static const struct device      *u3_i2c_dev = NULL;
static const struct device      *u3_int1_port_dev = NULL;
static const struct device      *u3_int2_port_dev = NULL;
static struct gpio_callback     u3_int1_cb;
static struct gpio_callback     u3_int2_cb;

static struct k_work_q          *caller_work_q = NULL;
static struct k_delayed_work    z_rotation_notify_work;
static struct k_sem             cross_zth_sem;
static struct k_sem             data_ready_sem;

#define THREAD_STACK_SIZE       KB(1)
#define THREAD_PRIORITY         K_LOWEST_APPLICATION_THREAD_PRIO

static struct k_thread          z_rotation_detect_thread;
static k_tid_t                  z_rotation_detect_thread_id = NULL;
static K_THREAD_STACK_DEFINE(z_rotation_detect_thread_stack, THREAD_STACK_SIZE);
static bool                     do_thread_abort = false;

static bool                     is_waiting_for_new_detection = false;
static rotation_handler_t       handler = NULL;
static float                    max_z_rotation_speed;


static void z_rotation_notify_work_fn(struct k_work *work)
{
    LOG_DBG("z_rotation_notify_work_fn");
    if (handler != NULL) {
        handler(max_z_rotation_speed);
    }
}

static void u3_int1_asserted(const struct device *gpiob, 
                                struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("u3_int1_asserted (waiting_new_detection %d)", 
                is_waiting_for_new_detection);
    if (is_waiting_for_new_detection == true) {
        is_waiting_for_new_detection = false;
        k_sem_give(&cross_zth_sem);
    }
}

static void u3_int2_asserted(const struct device *gpiob, 
                                struct gpio_callback *cb, uint32_t pins)
{
#if 0
    LOG_DBG("u3_int2_asserted (waiting_new_detection %d)", 
                is_waiting_for_new_detection);
#endif
    if (is_waiting_for_new_detection == false) {
        k_sem_give(&data_ready_sem);
    }
}

static void deconfig_int_pins(void)
{
    if (u3_int1_port_dev != NULL) {
        (void) gpio_pin_interrupt_configure(u3_int1_port_dev, 
                                        U3_INT1_PIN_NUMBER, GPIO_INT_DISABLE);
        (void) gpio_remove_callback(u3_int1_port_dev, &u3_int1_cb);
        (void) gpio_pin_configure(u3_int1_port_dev, U3_INT1_PIN_NUMBER, 
                                    GPIO_DISCONNECTED);
    }
    if (u3_int2_port_dev != NULL) {
        (void) gpio_pin_interrupt_configure(u3_int2_port_dev, 
                                        U3_INT2_PIN_NUMBER, GPIO_INT_DISABLE);
        (void) gpio_remove_callback(u3_int2_port_dev, &u3_int2_cb);
        (void) gpio_pin_configure(u3_int2_port_dev, U3_INT2_PIN_NUMBER, 
                                    GPIO_DISCONNECTED);
    }
}

static int config_int_pins(void)
{
    int err;

    u3_int1_port_dev = device_get_binding(U3_INT1_PORT_DEVICE);
    if (u3_int1_port_dev == NULL) {
        LOG_ERR("%s pin not found",  DT_PROP(U3_INT1_NODE, label));
        return -ENODEV;
    }

    err = gpio_pin_configure(u3_int1_port_dev, U3_INT1_PIN_NUMBER, 
                                GPIO_INPUT | GPIO_PULL_DOWN);
    if (err != 0) {
        LOG_ERR("%s pin configure failed %d", 
                    DT_PROP(U3_INT1_NODE, label), err);
        goto cleanup;
    }
    gpio_init_callback(&u3_int1_cb, u3_int1_asserted, BIT(U3_INT1_PIN_NUMBER));
    err = gpio_add_callback(u3_int1_port_dev, &u3_int1_cb);
    if (err != 0) {
        LOG_ERR("%s pin add callback failed %d", 
                    DT_PROP(U3_INT1_NODE, label), err);
        goto cleanup;
    }
    err = gpio_pin_interrupt_configure(u3_int1_port_dev, U3_INT1_PIN_NUMBER, 
                                        GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0) {
        LOG_ERR("%s pin interrupt configure failed %d", 
                    DT_PROP(U3_INT1_NODE, label), err);
        goto cleanup;
    }
    LOG_DBG("U3 INT1 (%s pin) configured", DT_PROP(U3_INT1_NODE, label));

    u3_int2_port_dev = device_get_binding(U3_INT2_PORT_DEVICE);
    if (u3_int2_port_dev == NULL) {
        LOG_ERR("%s pin not found", DT_PROP(U3_INT2_NODE, label));
        return -ENODEV;
    }

    err = gpio_pin_configure(u3_int2_port_dev, U3_INT2_PIN_NUMBER, 
                                GPIO_INPUT | GPIO_PULL_DOWN);
    if (err != 0) {
        LOG_ERR("%s pin configure failed %d", 
                    DT_PROP(U3_INT2_NODE, label), err);
        goto cleanup;
    }
    gpio_init_callback(&u3_int2_cb, u3_int2_asserted, BIT(U3_INT2_PIN_NUMBER));
    err = gpio_add_callback(u3_int2_port_dev, &u3_int2_cb);
    if (err != 0) {
        LOG_ERR("%s pin add callback failed %d", 
                    DT_PROP(U3_INT2_NODE, label), err);
        goto cleanup;
    }
    err = gpio_pin_interrupt_configure(u3_int2_port_dev, 
                                U3_INT2_PIN_NUMBER, GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0) {
        LOG_ERR("%s pin interrupt configure failed %d", 
                    DT_PROP(U3_INT2_NODE, label), err);
        goto cleanup;
    }
    LOG_DBG("U3 INT2 (%s pin) configured", DT_PROP(U3_INT2_NODE, label));

    return 0;

cleanup:
    deconfig_int_pins();
    return err;
}

static int cross_zth_on_int1_enable(void)
{
    int err;
    uint8_t reg_value;

    /* INT1_THS_ZH (36h): ZTH MSB Configuration */
    reg_value = (U3_ZTH >> 8) & 0xFF;
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_INT1_THS_ZH, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_INT1_THS_ZH");
        goto cleanup;
    }

    /* INT1_THS_ZL (37h): ZTH LSB Configuration */
    reg_value = U3_ZTH & 0xFF;
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_INT1_THS_ZL, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_INT1_THS_ZL");
        goto cleanup;
    }

    /* INT1_DURATION (38h): INT1 Duration set */
    reg_value = I3G4250D_REG_INT1_DURATION_WAIT | 4; /* duration=4 */
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_INT1_DURATION, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_INT1_DURATION");
        goto cleanup;
    }

    /* INT1_CFG (30h): INT1 on ZHIE */
    reg_value = 0x20; /* ZHIE */
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_INT1_CFG, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_INT1_CFG");
        goto cleanup;
    }

    /* CTRL_REG3 (22h): I1 interrupt on INT1 pin */
    reg_value = I3G4250D_REG_CTRL_REG3_I1_INT1;
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_CTRL_REG3, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG3");
        goto cleanup;
    }

    return 0;

cleanup:
    return err;
}

static int disable_interrupts(void)
{
    int err, first_err;
    uint8_t reg_value;

    /* CTRL_REG1 (20h): power down sensor */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_CTRL_REG1, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG1 %d", err);
        first_err = err;
    }

    /* CTRL_REG3 (22h): disable interrupts */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_CTRL_REG3, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG3 %d", err);
        if (first_err == 0) {
            first_err = err;
        }
    }

    /* INT1_CFG (30h): clear int1 configuration */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_INT1_CFG, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_INT1_CFG");
        if (first_err == 0) {
            first_err = err;
        }
    }

    return first_err;
}

static void z_rotation_detect_thread_fn(void *arg1, void *arg2, void *arg3)
{
    int64_t start_time;
    uint16_t num_readings;
    uint16_t abs_max_z;
    int16_t gyro_z;
    int16_t direction;
    uint8_t reg_value;
    uint8_t z_value[2];
    bool stop_reading;
    bool stop_computing;
    int err;

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_DBG("START");

    while (do_thread_abort == false) {
        num_readings = 0;
        direction = 0;
        abs_max_z = 0;
        stop_reading = false;
        stop_computing = false;

        is_waiting_for_new_detection = true;
        if (cross_zth_on_int1_enable() != 0) {
            is_waiting_for_new_detection = false;
            goto cleanup;
        }

        /* CTRL_REG1 (20h): start sensor */
        reg_value = 0x0C; /* ODR 100Hz, normal mode, Zen */
        err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                    I3G4250D_REG_CTRL_REG1, reg_value);
        if (err != 0) {
            LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG1 %d", err);
            goto cleanup;
        }

        k_sem_take(&cross_zth_sem, SYS_TIMEOUT_MS(SYS_FOREVER_MS));
        if (do_thread_abort == true) {
            goto cleanup;
        }

        /* do we stiil need to read the INT1_SRC? expect both IA ZH to be SET */
        err = i2c_reg_read_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                    I3G4250D_REG_INT1_SRC, &reg_value);
        if (err != 0) {
            LOG_ERR("Fail to read I3G4250D_REG_INT1_SRC %d", err);
            goto cleanup;
        }
        LOG_DBG("INT1_SRC 0x%02x", reg_value);

        /* CTRL_REG3 (22h): enable I2_DRDY */
        reg_value = I3G4250D_REG_CTRL_REG3_I2_DRDY;
        err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                    I3G4250D_REG_CTRL_REG3, reg_value);
        if (err != 0) {
            LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG3 %d", err);
            goto cleanup;
        }

        /* start reading z data to get the max rotation speed */
        start_time = k_uptime_get();
        int64_t stop_compute_time = start_time;
        int64_t delta_time;
        while (stop_reading == false) {
            k_sem_take(&data_ready_sem, SYS_TIMEOUT_MS(SYS_FOREVER_MS));
            if (do_thread_abort == true) {
                goto cleanup;
            }

            if (stop_computing == false) {
                stop_compute_time = start_time;
                delta_time = k_uptime_delta(&stop_compute_time);
                stop_computing = (delta_time >= 
                                    (MAX_OBSERVE_DURATION * MSEC_PER_SEC));
            }

            /* STATUS_REG (27h): expecting Z data  */
            err = i2c_reg_read_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                        I3G4250D_REG_STATUS_REG, &reg_value);
            if (err != 0) {
                LOG_ERR("Fail to read I3G4250D_REG_STATUS_REG %d", err);
                goto cleanup;
            }
            if ((reg_value & 0x0C /* ZYXDA | ZDA */) == 0) {
                LOG_DBG("unexpected STATUS_REG 0x%02x", reg_value);
                continue;
            }

            /* OUT_Z (2Ch, 2Dh): read Z data  */
            err = i2c_burst_read(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                        I3G4250D_REG_OUT_Z_L | I3G4250D_AUTO_INCR_BITMASK, 
                        z_value, 2);
            if (err != 0) {
                LOG_ERR("Fail to read I3G4250D_REG_OUT_Z %d", err);
                goto cleanup;
            }

            /* check data */
            gyro_z = (int16_t)((z_value[1] << 8) + z_value[0]);
            if (direction == 0)
            {
                if (gyro_z > 0)
                {
                    direction = 1;
                }
                else if (gyro_z < 0)
                {
                    direction = -1;
                }
            }
            LOG_DBG("gyro_z %d", gyro_z);
            if ((gyro_z == 0) || ((direction & 0x8000) ^ (gyro_z & 0x8000)))
            {
                stop_reading = true;
            } else {
                if (stop_computing == false)
                {
                    abs_max_z = MAX(abs_max_z, ((gyro_z < 0) ? 
                                        (gyro_z * -1) : gyro_z));
                    if (num_readings < 0xFFFF) {
                        num_readings++;
                    }
                }
            }
        }
        /* (stop_reading == true) */
        LOG_DBG("start %d end %d observed %d", 
                    (uint32_t)start_time, (uint32_t)k_uptime_get(), 
                    (uint32_t)delta_time);

        /* calculate RPM and notify application */
        max_z_rotation_speed = abs_max_z * U3_SENSITIVITY / 6000.0f; /* RPM */
        LOG_DBG("RPM: %d.%02d direction %d (readings %d)",
                (int)max_z_rotation_speed,
                (int)(100 * (max_z_rotation_speed - (int)max_z_rotation_speed)),
                direction, num_readings);
        max_z_rotation_speed *= direction;

        /* CTRL_REG3 (22h): disable interrupts */
        reg_value = 0x00;
        err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                    I3G4250D_REG_CTRL_REG3, reg_value);
        if (err != 0) {
            LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG3 %d", err);
            goto cleanup;
        }

        if (delta_time > 300) {
            err = k_delayed_work_submit_to_queue(caller_work_q, 
                                    &z_rotation_notify_work, K_NO_WAIT);
            if (err != 0) {
                LOG_ERR("k_delayed_work_submit_to_queue(rotation_notify) "
                        "failed (err: %d)", err);
            }
        }

    }

cleanup:
    LOG_DBG("EXIT");
    disable_interrupts();
    z_rotation_detect_thread_id = NULL;
}

int rotation_detector_init_and_start(struct k_work_q *work_q, 
                                        rotation_handler_t event_handler)
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
    k_delayed_work_init(&z_rotation_notify_work, z_rotation_notify_work_fn);
    k_sem_init(&cross_zth_sem, 0, 1);
    k_sem_init(&data_ready_sem, 0, 1);

    u3_i2c_dev = device_get_binding(U3_I2C_DEVICE);
    if (u3_i2c_dev == NULL) {
        LOG_ERR("STM LIS2DH12 not found");
        err = -ENODEV;
        goto cleanup;
    }

    err = i2c_reg_read_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_WHO_AM_I, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read I3G4250D_REG_WHO_AM_I %d", err);
        goto cleanup;
    }
    if (reg_value != I3G4250D_WHO_AM_I) {
        LOG_ERR("Unexpected I3G4250D_REG_WHO_AM_I value 0x%02x", reg_value);
        err = -ENOTSUP;
        goto cleanup;
    }

    /* in case of nrf91 reset after sensor is started */
    /* CTRL_REG1 (20h): power down sensor */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_CTRL_REG1, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG1 %d", err);
        goto cleanup;
    }

    /* Initialization of sensor */

    /* CTRL_REG4 (23h): Set Full-scale to 2000 dps */
    reg_value = I3G4250D_REG_CTRL_REG4_FS(LIS2DH12_FS_2000dps);
    err = i2c_reg_write_byte(u3_i2c_dev, I3G4250D_7BIT_ADDR, 
                                I3G4250D_REG_CTRL_REG4, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set I3G4250D_REG_CTRL_REG4 %d", err);
        goto cleanup;
    }

    do_thread_abort = false;
    z_rotation_detect_thread_id = k_thread_create(&z_rotation_detect_thread, 
                    z_rotation_detect_thread_stack,
                    K_THREAD_STACK_SIZEOF(z_rotation_detect_thread_stack),
                    z_rotation_detect_thread_fn, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);

    if (z_rotation_detect_thread_id == NULL) {
        goto cleanup;
    }

    return 0;

cleanup:
    handler = NULL;
    caller_work_q = NULL;
    deconfig_int_pins();
    return err;
}

void rotation_detector_stop_and_unint(void)
{
    if (caller_work_q == NULL) {
        /* not init and started */
        return;
    }

    handler = NULL;
    caller_work_q = NULL;
    deconfig_int_pins();

    if (z_rotation_detect_thread_id != NULL) {
        do_thread_abort = true;
        if (is_waiting_for_new_detection == true) {
            k_sem_give(&cross_zth_sem);
        } else {
            k_sem_give(&data_ready_sem);
        }
        while (z_rotation_detect_thread_id != NULL) {
            k_sleep(SYS_TIMEOUT_MS(100));
        }
    }
}

