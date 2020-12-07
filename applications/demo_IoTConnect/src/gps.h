#ifndef GPS_H
#define GPS_H

#include <nrf_socket.h>

bool gps_init(void);
bool gps_start(void);
bool gps_stop(void);
bool gps_is_active(void);
bool gps_acquire_loc(float *p_latitude, float *p_longitude);
bool gps_acquire_latest_pvt_data(nrf_gnss_pvt_data_frame_t *p_pvt);


#endif