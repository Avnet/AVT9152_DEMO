/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */
 
#ifndef ROTATION_DETECTOR_H_
#define ROTATION_DETECTOR_H_

/**@file
 *
 * @brief   Rotation on Z-axis detector module.
 *
 * Module that uses STM I3G4250D gyroscope to detect device's maximum 
 * rotational speed on Z-axis from the first 5 seconds since rotation is 
 * detected or until a rotation direction change or the rotation stops.
 *
 */


/**@brief Function to get informed of detected rotation on Z-axis. */
typedef void (*rotation_handler_t)(float max_rotation_speed);


/**@brief Initializes and start the rotation on Z-axis detector function.
 *
 * @param[in] work_q Workqueue where processing is done and event_handler
 *                   will be called.
 * @param[in] event_handler Callback function to call on Z-axis rotation 
 *                          detection.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 *
 */
int rotation_detector_init_and_start(struct k_work_q *work_q, 
                                    rotation_handler_t event_handler);

/**@brief Stopped and uninitializes the rotation on Z-axis detector function. */
void rotation_detector_stop_and_unint(void);


#endif /* ORIENTATION_TAP_DETECTOR_H_ */
