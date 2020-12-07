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

#include "lis2dh12_def.h"
#include "orientation_tap_detector.h"


#include <logging/log.h>
LOG_MODULE_REGISTER(orientation_tap, LOG_LEVEL_INF);

#define U2_ORIENTATION_THS_MG   864 /* ~ 60 degrees tilt */

#define U2_I2C_NODE             DT_ALIAS(sens_i2c)
#define U2_I2C_DEVICE           DT_LABEL(U2_I2C_NODE)

#define U2_INT1_NODE            DT_ALIAS(ard_d1)
#define U2_INT1_PORT_DEVICE     DT_GPIO_LABEL(U2_INT1_NODE, gpios)
#define U2_INT1_PIN_NUMBER      DT_GPIO_PIN(U2_INT1_NODE, gpios)

#define U2_INT2_NODE            DT_ALIAS(ard_d6)
#define U2_INT2_PORT_DEVICE     DT_GPIO_LABEL(U2_INT2_NODE, gpios)
#define U2_INT2_PIN_NUMBER      DT_GPIO_PIN(U2_INT2_NODE, gpios)

static const struct device      *u2_i2c_dev = NULL;
static const struct device      *u2_int1_port_dev = NULL;
static const struct device      *u2_int2_port_dev = NULL;
static struct gpio_callback     u2_int1_cb;
static struct gpio_callback     u2_int2_cb;

static struct k_work_q          *caller_work_q = NULL;
static struct k_delayed_work    orientation_detect_work;
static struct k_delayed_work    tap_detect_work;

static orientation_tap_handler_t    handler = NULL;
static orientation_state_t          orientation = ORIENTATION_NOT_KNOWN;

static orientation_state_t u2_orientation_state(uint8_t src)
{
/*
 * (a)1000100 (0x04) reverse portait
 * (b)1000010 (0x02) reverse landscape
 * (c)1000001 (0x01) landscape
 * (d)1001000 (0x08) portrait
 * (e)1100000 (0x20) facing up
 * (f)1010000 (0x10) facing down
*/
    switch (src & 0x3F)
    {
        case 0x01:
            return ORIENTATION_LANDSCAPE;
        case 0x02:
            return ORIENTATION_REVERSE_LANDSCAPE;
        case 0x04:
            return ORIENTATION_REVERSE_PORTRAIT;
        case 0x08:
            return ORIENTATION_PORTRAIT;
        case 0x10:
            return ORIENTATION_FACE_DOWN;
        case 0x20:
            return ORIENTATION_FACE_UP;
    }
    
    return ORIENTATION_NOT_KNOWN;
}

static void orientation_detect_work_fn(struct k_work *work)
{
    int err;
    uint8_t reg_value;
    orientation_tap_event_t event;
    orientation_state_t new_orientation;

    err = i2c_reg_read_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                            LIS2DH12_REG_INT1_SRC, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read LIS2DH12_REG_INT1_SRC");
        return;
    }
    LOG_DBG("INT1_SRC 0x%02x", reg_value);

    new_orientation = u2_orientation_state(reg_value);
    if (orientation != new_orientation) {
        orientation = new_orientation;
        if (handler != NULL) {
#if defined (GET_ORIENTATION_XYZ) && (GET_ORIENTATION_XYZ > 0)
            uint8_t buf[7] = {0};
            err = i2c_reg_read_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                    LIS2DH12_REG_STATUS, &buf[0]);
            if (err != 0) {
                LOG_ERR("Fail to read LIS2DH12_REG_STATUS");
            } else {
                LOG_DBG("STATUS 0x%02x", buf[0]);
            }
            if (buf[0] & 0x08) {//ZYXDA
                err = i2c_burst_read(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                        LIS2DH12_REG_OUT_X_L | LIS2DH12_AUTO_INCR, &buf[0], 6);
                if (err != 0) {
                    LOG_ERR("Fail to read LIS2DH12_REG_OUT_X_L");
                } else {
                    // high resolution 12-bit data left-justified
                    event.x = ((int16_t)((buf[1] << 8) + buf[0]) >> 4);
                    event.y = ((int16_t)((buf[3] << 8) + buf[2]) >> 4);
                    event.z = ((int16_t)((buf[5] << 8) + buf[4]) >> 4);
                    LOG_DBG("raw 0x%02x 0x%02x X %d", buf[1], buf[0], event.x);
                    LOG_DBG("raw 0x%02x 0x%02x Y %d", buf[3], buf[2], event.y);
                    LOG_DBG("raw 0x%02x 0x%02x Z %d", buf[5], buf[4], event.z);
                }
            }
            else
            {
                event.x = 0x4000; // invalid value
                event.y = 0x4000;
                event.z = 0x4000;
            }
#endif
            event.orientation = orientation;
            event.tap_detected = false;
            handler(event);
        }
    }
}

static void tap_detect_work_fn(struct k_work *work)
{
    int err;
    uint8_t reg_value;
    orientation_tap_event_t event;

    err = i2c_reg_read_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                            LIS2DH12_REG_CLICK_SRC, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read LIS2DH12_REG_CLICK_SRC");
        return;
    }
    LOG_DBG("CLICK_SRC 0x%02x", reg_value);
    if (reg_value) {
        if (handler != NULL) {
            event.orientation = orientation;
            event.tap_detected = true;
            handler(event);
        }
    }
}

static void u2_int1_asserted(const struct device *gpiob, 
                                struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("u2_int1_asserted");
    int err = k_delayed_work_submit_to_queue(caller_work_q, 
                                        &orientation_detect_work, K_NO_WAIT);
    if (err != 0) {
        LOG_ERR("k_delayed_work_submit_to_queue(orientation_detect) "
                        "failed (err: %d)", err);
    }
}

static void u2_int2_asserted(const struct device *gpiob, 
                                struct gpio_callback *cb, uint32_t pins)
{
    LOG_DBG("u2_int2_asserted");
    int err = k_delayed_work_submit_to_queue(caller_work_q, &tap_detect_work, 
                                        K_NO_WAIT);
    if (err != 0) {
        LOG_ERR("k_delayed_work_submit_to_queue(tap_detect) "
                        "failed (err: %d)", err);
    }
}

static void deconfig_int_pins(void)
{
    if (u2_int1_port_dev != NULL) {
        (void) gpio_pin_interrupt_configure(u2_int1_port_dev, 
                                        U2_INT1_PIN_NUMBER, GPIO_INT_DISABLE);
        (void) gpio_remove_callback(u2_int1_port_dev, &u2_int1_cb);
        (void) gpio_pin_configure(u2_int1_port_dev, U2_INT1_PIN_NUMBER, 
                                    GPIO_DISCONNECTED);
    }
    if (u2_int2_port_dev != NULL) {
        (void) gpio_pin_interrupt_configure(u2_int2_port_dev, 
                                        U2_INT2_PIN_NUMBER, GPIO_INT_DISABLE);
        (void) gpio_remove_callback(u2_int2_port_dev, &u2_int2_cb);
        (void) gpio_pin_configure(u2_int2_port_dev, U2_INT2_PIN_NUMBER, 
                                    GPIO_DISCONNECTED);
    }
}

static int config_int_pins(void)
{
    int err;

    u2_int1_port_dev = device_get_binding(U2_INT1_PORT_DEVICE);
    if (u2_int1_port_dev == NULL) {
        LOG_ERR("%s pin not found",  DT_PROP(U2_INT1_NODE, label));
        return -ENODEV;
    }

    err = gpio_pin_configure(u2_int1_port_dev, U2_INT1_PIN_NUMBER, 
                                GPIO_INPUT | GPIO_PULL_DOWN);
    if (err != 0) {
        LOG_ERR("%s pin configure failed %d", 
                    DT_PROP(U2_INT1_NODE, label), err);
        goto cleanup;
    }
    gpio_init_callback(&u2_int1_cb, u2_int1_asserted, BIT(U2_INT1_PIN_NUMBER));
    err = gpio_add_callback(u2_int1_port_dev, &u2_int1_cb);
    if (err != 0) {
        LOG_ERR("%s pin add callback failed %d", 
                    DT_PROP(U2_INT1_NODE, label), err);
        goto cleanup;
    }
    err = gpio_pin_interrupt_configure(u2_int1_port_dev, U2_INT1_PIN_NUMBER, 
                                        GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0) {
        LOG_ERR("%s pin interrupt configure failed %d", 
                    DT_PROP(U2_INT1_NODE, label), err);
        goto cleanup;
    }
    LOG_DBG("U2 INT1 (%s pin) configured", DT_PROP(U2_INT1_NODE, label));

    u2_int2_port_dev = device_get_binding(U2_INT2_PORT_DEVICE);
    if (u2_int2_port_dev == NULL) {
        LOG_ERR("%s pin not found", DT_PROP(U2_INT2_NODE, label));
        return -ENODEV;
    }

    err = gpio_pin_configure(u2_int2_port_dev, U2_INT2_PIN_NUMBER, 
                                GPIO_INPUT | GPIO_PULL_DOWN);
    if (err != 0) {
        LOG_ERR("%s pin configure failed %d", 
                    DT_PROP(U2_INT2_NODE, label), err);
        goto cleanup;
    }
    gpio_init_callback(&u2_int2_cb, u2_int2_asserted, BIT(U2_INT2_PIN_NUMBER));
    err = gpio_add_callback(u2_int2_port_dev, &u2_int2_cb);
    if (err != 0) {
        LOG_ERR("%s pin add callback failed %d", 
                    DT_PROP(U2_INT2_NODE, label), err);
        goto cleanup;
    }
    err = gpio_pin_interrupt_configure(u2_int2_port_dev, 
                                U2_INT2_PIN_NUMBER, GPIO_INT_EDGE_TO_ACTIVE);
    if (err != 0) {
        LOG_ERR("%s pin interrupt configure failed %d", 
                    DT_PROP(U2_INT2_NODE, label), err);
        goto cleanup;
    }
    LOG_DBG("U2 INT2 (%s pin) configured", DT_PROP(U2_INT2_NODE, label));

    return 0;

cleanup:
    deconfig_int_pins();
    return err;
}

static uint8_t u2_get_ths_mg_per_lsb(uint8_t full_scale)
{
    /*
     * 1 LSb = 16 mg @ FS = 2 g
     * 1 LSb = 32 mg @ FS = 4 g
     * 1 LSb = 62 mg @ FS = 8 g
     * 1 LSb = 186 mg @ FS = 16 g
    */
    const uint8_t ths_mg_per_lsb[4] = {16, 32, 62, 186};

    __ASSERT(full_scale <= 3, "Invalid full scale");

    return (ths_mg_per_lsb[full_scale]);
}

/* dm00454782-DT0097 - 
 * Setting up 6D orientation detection with ST’s MEMS accelerometers 
 */
static int postion_recognition_enable(void)
{
    int err;
    uint8_t reg_value;

    /* CTRL_REG3 (22h): IA1 interrupt on INT1 pin */
    reg_value = LIS2DH12_REG_CTRL_REG3_I1_IA1;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG3, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG3");
        goto cleanup;
    }

    /* 6D orientation enable */

    /* CTRL_REG2 (30h): INT1 Configuration */
    reg_value = 0xFF; /* AOI 6D ZHIE ZLIE YHIE YLIE XHIE XLIE */
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_INT1_CFG, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_INT1_CFG");
        goto cleanup;
    }

    /* INT1_THS (32h): INT1 Threshold set */
    reg_value = U2_ORIENTATION_THS_MG / u2_get_ths_mg_per_lsb(LIS2DH12_FS_2g);
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_INT1_THS, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_INT1_THS");
        goto cleanup;
    }

    /* INT1_DURATION (33h): INT1 Duration set */
    reg_value = 0x60; /* 96×(1/400 Hz) = 240 ms */
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_INT1_DURATION, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_INT1_DURATION");
        goto cleanup;
    }

    return 0;

cleanup:
    return err;
}

/* en.DM00503898 - DT0101 - Setting up single-tap and double-tap recognition */
static int double_tap_recognition_enable(void)
{
    int err;
    uint8_t reg_value;

    /* CTRL_REG2 (21h): Enable HP filter on tap detection */
    reg_value = LIS2DH12_REG_CTRL_REG2_HPCLICK;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG2, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG2");
        goto cleanup;
    }

    /* CTRL_REG6 (25h): TAP interrupt on INT2 pin */
    reg_value = LIS2DH12_REG_CTRL_REG6_I2_CLICK;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG6, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG6");
        goto cleanup;
    }

    /* Double-tap recognition enable */

    /* CLICK_CFG (38h): Tap config */
    reg_value = LIS2DH12_REG_CLICK_CFG_XD | LIS2DH12_REG_CLICK_CFG_YD | 
                LIS2DH12_REG_CLICK_CFG_ZD;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CLICK_CFG, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CLICK_CFG");
        goto cleanup;
    }

    /* CLICK_THS (3Ah): Tap threshold set */
    reg_value = 0x30; /* 48 × FS / 128 = 750 mg where FS is ±2 g */
    reg_value |= LIS2DH12_REG_CLICK_THS_LIR_CLICK;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CLICK_THS, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CLICK_THS");
        goto cleanup;
    }

    /* TIME_LIMIT (3Bh): Tap time limit set */
    reg_value = 0x18; /* 24 × 1 / ODR = 60 ms where ODR is 400 Hz */
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_TIME_LIMIT, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_TIME_LIMIT");
        goto cleanup;
    }

    /* TIME_LATENCY (3Ch): Tap time latency set */
    reg_value = 0x0C; /* 12 × 1 / ODR = 30 ms */
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_TIME_LATENCY, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_TIME_LATENCY");
        goto cleanup;
    }

    /* TIME_WINDOW (3Dh): Tap time window set */
    reg_value = 0xE0; /* 224 × 1 / ODR = 560 ms where ODR is 400 Hz */
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_TIME_WINDOW, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_TIME_WINDOW");
        goto cleanup;
    }

    return 0;

cleanup:
    return err;
}

int orientation_tap_detector_init_and_start(struct k_work_q *work_q, 
                                        orientation_tap_handler_t event_handler)
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
    k_delayed_work_init(&orientation_detect_work, orientation_detect_work_fn);
    k_delayed_work_init(&tap_detect_work, tap_detect_work_fn);

    u2_i2c_dev = device_get_binding(U2_I2C_DEVICE);
    if (u2_i2c_dev == NULL) {
        LOG_ERR("STM LIS2DH12 not found");
        err = -ENODEV;
        goto cleanup;
    }

    err = i2c_reg_read_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_WHO_AM_I, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read LIS2DH12_REG_WHO_AM_I %d", err);
        goto cleanup;
    }
    if (reg_value != LIS2DH12_WHO_AM_I) {
        LOG_ERR("Unexpected LIS2DH12_REG_WHO_AM_I value 0x%02x", reg_value);
        err = -ENOTSUP;
        goto cleanup;
    }

    /* in case of nrf91 reset after sensor is started */
    /* CTRL_REG1 (20h):ODR power down */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG1, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG1 %d", err);
    }

    /* Initialization of sensor */

    /* CTRL_REG4 (23h): Set Full-scale to +/-2g */
    reg_value = LIS2DH12_REG_CTRL_REG4_FS(LIS2DH12_FS_2g) | 
                                            LIS2DH12_REG_CTRL_REG4_HR;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG4, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG4 %d", err);
        goto cleanup;
    }

    err = postion_recognition_enable();
    if (err != 0) {
        LOG_ERR("Fail to configure for position change detection %d", err);
        goto cleanup;
    }
    err = double_tap_recognition_enable();
    if (err != 0) {
        LOG_ERR("Fail to configure for double tap detection %d", err);
        goto cleanup;
    }

    /* Start sensor */

    /* CTRL_REG1 (20h): Start sensor at ODR 400Hz */
    reg_value = LIS2DH12_REG_CTRL_REG1_ODR(LIS2DH12_ODR_400_HZ) | 
                                            LIS2DH12_REG_CTRL_REG1_XEN | 
                                            LIS2DH12_REG_CTRL_REG1_YEN | 
                                            LIS2DH12_REG_CTRL_REG1_ZEN;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG1, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG1 %d", err);
        goto cleanup;
    }

    if (gpio_pin_get_raw(u2_int1_port_dev, U2_INT1_PIN_NUMBER) == 1) {
        LOG_INF("U2 INT1 asserted upon start sensor");
        orientation_detect_work_fn(NULL);
    }

    return 0;

cleanup:
    handler = NULL;
    caller_work_q = NULL;
    deconfig_int_pins();
    return err;
}

void orientation_tap_detector_stop_and_unint(void)
{
    int err;
    uint8_t reg_value;

    if (caller_work_q == NULL) {
        /* not init and started */
        return;
    }

    handler = NULL;
    caller_work_q = NULL;
    deconfig_int_pins();

    /* CTRL_REG1 (20h):ODR power down */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG1, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG1 %d", err);
    }

    /* CTRL_REG3 (22h): Disable interrupts */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG3, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG3");
    }

    /* CTRL_REG6 (25h): Disable interrupts */
    reg_value = 0x00;
    err = i2c_reg_write_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_CTRL_REG6, reg_value);
    if (err != 0) {
        LOG_ERR("Fail to set LIS2DH12_REG_CTRL_REG6");
    }

    /*
     * By design, when the device from high-resolution configuration (HR) is 
     * set to power-down mode (PD), it is recommended to read register 
     * REFERENCE (26h) for a complete reset of the filtering block before 
     * switching to normal/high-performance mode again for proper device 
     * functionality.
     */
    err = i2c_reg_read_byte(u2_i2c_dev, LIS2DH12_7BIT_ADDR, 
                                LIS2DH12_REG_REFERENCE, &reg_value);
    if (err != 0) {
        LOG_ERR("Fail to read LIS2DH12_REG_REFERENCE %d", err);
    }
}
