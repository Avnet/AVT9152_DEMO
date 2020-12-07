/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */
 
#ifndef PIR_DETECTOR_H_
#define PIR_DETECTOR_H_

/**@file
 *
 * @brief   PIR detector module.
 *
 * Module that uses TE_2314277 for PIR motion sensing.
 */



/**@brief Function to get informed of PIR on/off state changes. */
typedef void (*pir_event_handler_t)(bool pir_detected);


/**@brief Initializes and start the pir detector function.
 *
 * @param[in] work_q Workqueue where processing is done and event_handler
 *                   will be called.
 * @param[in] event_handler Callback function to call on pir on/off detection.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 *
 */
int pir_detector_init_and_start(struct k_work_q *work_q, 
                                pir_event_handler_t event_handler);

/**@brief Stopped and uninitializes the pir detector function. */
void pir_detector_stop_and_unit(void);


#endif /* PIR_DETECTOR_H_ */
