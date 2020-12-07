/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */
 
#ifndef ENV_SENSORS_H
#define ENV_SENSORS_H

typedef struct {

    float pressure;
    float temperature;
    float humidity;
    uint16_t illumination;

} env_sensors_data_t;


bool env_sensors_init(void);
void env_sensors_uninit(void);
bool env_sensors_start(void);
bool env_sensors_stop(void);
bool env_sensors_is_running(void);
bool env_sensors_get_data(env_sensors_data_t *p_sens_data);


#endif // ENV_SENSORS_H
