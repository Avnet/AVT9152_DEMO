/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */
 
#ifndef ORIENTATION_TAP_DETECTOR_H_
#define ORIENTATION_TAP_DETECTOR_H_

/**@file
 *
 * @brief   Orientation and tap detector module.
 *
 * Module that uses STM LIS2DH12 accelerometer to detect device's orientation 
 * and tap event.
 */

#define GET_ORIENTATION_XYZ     0


/**@brief Orientation states. */
/* sequence is based on the bitpos provided in LIS2DH12_REG_INT1_SRC */
typedef enum {
    ORIENTATION_NOT_KNOWN,
    ORIENTATION_LANDSCAPE,
    ORIENTATION_REVERSE_LANDSCAPE,
    ORIENTATION_REVERSE_PORTRAIT,
    ORIENTATION_PORTRAIT,
    ORIENTATION_FACE_DOWN,
    ORIENTATION_FACE_UP,
}orientation_state_t;

/**
 * @brief Defines event parameters provided to the application.
 */
typedef struct {
    orientation_state_t orientation;
    bool                tap_detected;
#if defined (GET_ORIENTATION_XYZ) && (GET_ORIENTATION_XYZ > 0)
    int16_t               x;
    int16_t               y;
    int16_t               z;
#endif
} orientation_tap_event_t;

/**@brief Function to get informed of orientation change or tap detection. */
typedef void (*orientation_tap_handler_t)(
                                orientation_tap_event_t orientation_tap_event);


/**@brief Initializes and start the orientation tap detector function.
 *
 * @param[in] work_q Workqueue where processing is done and event_handler
 *                   will be called.
 * @param[in] event_handler Callback function to call on orientation change or 
 *                          tap detection.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 *
 */
int orientation_tap_detector_init_and_start(struct k_work_q *work_q, 
                                    orientation_tap_handler_t event_handler);

/**@brief Stopped and uninitializes the orientation tap detector function. */
void orientation_tap_detector_stop_and_unint(void);


#endif /* ORIENTATION_TAP_DETECTOR_H_ */
