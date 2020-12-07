/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>
#include "env_sensors_regs.h"
#include "env_sensors.h"


#define SENSORS_TASK_STACK_SIZE     KB(1)
#define SENSORS_TASK_PRIORITY       K_LOWEST_APPLICATION_THREAD_PRIO


#define SENSORS_DATA_UPDATE_INTERVAL_S      1

#include <logging/log.h>
LOG_MODULE_REGISTER(env_sensors, LOG_LEVEL_INF);


K_THREAD_STACK_DEFINE(sensors_task_stack_area, SENSORS_TASK_STACK_SIZE);
K_SEM_DEFINE(m_sens_data_sem, 1, 1);
K_SEM_DEFINE(m_evt_data_sem, 1, 1);

struct k_thread sensors_task_thread_data;
k_tid_t m_sensors_task_tid;

static const struct device *sensors_i2c_dev = NULL;
static env_sensors_data_t m_sens_data;
static bool m_task_stop;
static bool m_task_stopped;


static bool i2c_read_reg(const struct device *dev, uint8_t slave_addr, uint8_t reg, uint8_t *p_reg_val){

    struct i2c_msg msgs[2];

    msgs[0].buf = &reg;
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = p_reg_val;
    msgs[1].len = 1U;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    return i2c_transfer(dev, msgs, 2, slave_addr);
}

static bool i2c_read_regs(const struct device *dev, uint8_t slave_addr, uint8_t start_reg, uint8_t *p_reg_vals, uint32_t regs_to_read){

    struct i2c_msg msgs[2];

    msgs[0].buf = &start_reg;
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = p_reg_vals;
    msgs[1].len = regs_to_read;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    return i2c_transfer(dev, msgs, 2, slave_addr);
}

static int i2c_write_reg(const struct device *dev, uint8_t slave_addr, uint8_t reg, uint8_t reg_val){

    struct i2c_msg msgs[1];
    uint8_t payload[2];

    payload[0] = reg;
    payload[1] = reg_val;

    msgs[0].buf = payload;
    msgs[0].len = 2U;
    msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    return i2c_transfer(dev, msgs, 1, slave_addr);
}

static void reset_sensors_data(){

    m_sens_data.pressure = 0.0;
    m_sens_data.temperature = 0.0;
    m_sens_data.humidity = 0.0;
    m_sens_data.illumination = 0;
}

static void read_sensors_data(){
    
    int err_code;
    uint8_t rx[5];
    env_sensors_data_t sd;

    memset(&sd, 0, sizeof(sd));
     
    /* Read Pressure and temperature */
    // trigger one-shot 
    err_code = i2c_write_reg(sensors_i2c_dev, LPS22HB_7BIT_ADDR, LPS22HB_REG_CTRL2, LPS22HB_REG_CTRL2_ONE_SHOT | LPS22HB_REG_CTRL2_ADD_INC_EN);
    if (err_code != 0){
        LOG_ERR("U4 LPS22HB Pressure+Temp ONE_SHOT failed 0x%0x.", err_code);
        goto read_TE_2314277_1;
    }
    do
    {
        err_code = i2c_read_reg(sensors_i2c_dev, LPS22HB_7BIT_ADDR, LPS22HB_REG_CTRL2, &rx[0]);
        if (err_code != 0){
            LOG_ERR("U4 LPS22HB Pressure+Temp read CTRL2 failed 0x%0x.", err_code);
            goto read_TE_2314277_1;
        }
    } while ((rx[0] & LPS22HB_REG_CTRL2_ONE_SHOT) != 0);

    // status register T_DA=1 P_DA=1
    err_code = i2c_read_reg(sensors_i2c_dev, LPS22HB_7BIT_ADDR, LPS22HB_REG_STATUS, &rx[0]);
    if (err_code != 0){
        LOG_ERR("U4 LPS22HB Pressure+Temp read STATUS failed 0x%0x.", err_code);
        goto read_TE_2314277_1;
    }
    if ((rx[0] & 0x03) != 0x03){
        LOG_ERR("U4 LPS22HB Pressure+Temp T_DA & P_DA expected to be set.");
        goto read_TE_2314277_1;
    }

    // read data
    err_code = i2c_read_regs(sensors_i2c_dev, LPS22HB_7BIT_ADDR, LPS22HB_REG_PRESS_OUT_XL, rx, 5);
    if (err_code != 0){
        LOG_ERR("U4 LPS22HB Pressure+Temp read data failed 0x%0x.", err_code);
        goto read_TE_2314277_1;
    }
    {
        uint32_t pres;
        uint16_t temp;
        pres = ((uint32_t) rx[0]) |
               (((uint32_t) rx[1]) << 8) |
               (((uint32_t) rx[2]) << 16);
        pres <<= 8;
        temp = ((uint16_t) rx[3]) |
               (((uint16_t) rx[4]) << 8);
        // Dividing by 256 because signed integer can't be shifted by 8
        sd.pressure    = *((int32_t *) &pres) / 256.0 / 4096.0;
        sd.temperature = *((int16_t *) &temp) / 100.0;
    }
    
read_TE_2314277_1:

    /* Read humidity and illuminiation */

    err_code = i2c_write_reg(sensors_i2c_dev, TE_2314277_1_7BIT_ADDR, TE_2314277_1_REG_SCAN_START_BYTE,
                            TE_2314277_1_REG_SCAN_HUMIDITY_BITMASK | TE_2314277_1_REG_SCAN_LIGHT_BITMASK);
    if (err_code != 0){
        LOG_ERR("U5 2314277-1 Ambimate write SCAN_START_BYTE failed 0x%0x.", err_code);
        return;
    }
    // wait until measurement is complete
    do {
        k_sleep(SYS_TIMEOUT_MS(10));
        err_code = i2c_read_reg(sensors_i2c_dev, TE_2314277_1_7BIT_ADDR, TE_2314277_1_REG_SCAN_START_BYTE, &rx[0]);
    } while ((err_code == 0) && (rx[0] > 0));
    if (err_code != 0){
        LOG_ERR("U5 2314277-1 Ambimate read SCAN_START_BYTE failed 0x%0x.", err_code);
        return;
    }
    err_code = i2c_read_regs(sensors_i2c_dev, TE_2314277_1_7BIT_ADDR, TE_2314277_1_REG_HUMIDITY_H, rx, 4);
    if (err_code != 0){
        LOG_ERR("U5 2314277-1 Ambimate read measurement data failed 0x%0x.", err_code);
        return;
    }

    // convert the raw data to engineering units
    sd.humidity = (rx[0] * 256.0 + rx[1]) / 10.0;
    sd.illumination = (rx[2] * 256 + rx[3]);

    //transfer
    k_sem_take(&m_sens_data_sem, SYS_TIMEOUT_MS(SYS_FOREVER_MS));
    memcpy(&m_sens_data, &sd, sizeof(env_sensors_data_t));
    k_sem_give(&m_sens_data_sem);

}

static void sensors_task(void *p1, void *p2, void *p3){

    m_task_stopped = false;

    LOG_DBG(">>");

    reset_sensors_data();

    while(!m_task_stop){

        read_sensors_data();

        k_sleep(SYS_TIMEOUT_MS(SENSORS_DATA_UPDATE_INTERVAL_S * MSEC_PER_SEC));

    }
    
    m_task_stopped = true;

    LOG_DBG("<<");
}

bool env_sensors_start(void){

    if(!m_task_stopped){
        return false;
    }
    
    m_task_stop = false;

    m_sensors_task_tid = k_thread_create(&sensors_task_thread_data, sensors_task_stack_area,
                                 K_THREAD_STACK_SIZEOF(sensors_task_stack_area),
                                 sensors_task,
                                 NULL, NULL, NULL,
                                 SENSORS_TASK_PRIORITY, 0, K_NO_WAIT);

    return true;
}

bool env_sensors_stop(void){

    uint32_t start_time;

    m_task_stop = true;

    start_time = k_uptime_get_32();

    while(m_task_stopped == false && (k_uptime_get_32() - start_time) < 2000){
        k_sleep(SYS_TIMEOUT_MS(10));
    }

    if(m_task_stopped == false){
        LOG_DBG("Sensors task fail to terminate itself. Force task abort!"); 
        k_thread_abort(m_sensors_task_tid);
        m_task_stopped = true;
    }

    m_task_stop = false;
    return true;
}

bool env_sensors_is_running(void){

    return !m_task_stopped;

}


#define SENS_I2C_NODE       DT_ALIAS(sens_i2c)
bool env_sensors_init(void){
    
    uint8_t reg_val;

    if(sensors_i2c_dev == NULL){
        sensors_i2c_dev = device_get_binding(DT_LABEL(SENS_I2C_NODE));
    
        if(sensors_i2c_dev == NULL){
            LOG_ERR("Unable to bind device %s.", DT_LABEL(DT_ALIAS(sens_i2c)));
            return false;
        }

        /************************************/
        /* Test LPS22HB I2C communication */
        /************************************/
        //Read LPS22HB WHO_AM_I registers
        reg_val = 0xff;
        if(i2c_read_reg(sensors_i2c_dev, LPS22HB_7BIT_ADDR, LPS22HB_REG_WHO_AM_I, &reg_val)==0){
            if(reg_val != LPS22HB_WHO_AM_I){
                LOG_ERR("Read LPS22HB sensor NOT OK. Expected value %d but received %d.",
                        LPS22HB_WHO_AM_I, reg_val);
                return false;
            }
        }else{
            LOG_ERR("Read LPS22HB sensor fail.");
            return false;
        }

        /************************************/
        /* Test 2314277_1 I2C communication */
        /************************************/
        //Read 2314277_1 OPTIONS registers
        if(i2c_read_reg(sensors_i2c_dev, TE_2314277_1_7BIT_ADDR, TE_2314277_1_REG_OPTIONAL_SENSORS, &reg_val)!=0){
            LOG_ERR("Read 2314277_1 sensor fail.");
            return false;
        }

        reset_sensors_data();

        m_task_stop = false;
        m_task_stopped = true;

    }
    return true;
}

void env_sensors_uninit(){

    sensors_i2c_dev = NULL;
}

bool env_sensors_get_data(env_sensors_data_t *p_sens_data){

    if(sensors_i2c_dev == NULL){
        return false;
    }
    k_sem_take(&m_sens_data_sem, SYS_TIMEOUT_MS(SYS_FOREVER_MS));

    memcpy(p_sens_data, &m_sens_data, sizeof(env_sensors_data_t));

    k_sem_give(&m_sens_data_sem);

    return true;
}
