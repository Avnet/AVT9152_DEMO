/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <device.h>
#include <sys_clock.h>
#include <nrf_socket.h>
#include <net/socket.h>
#include <modem/at_cmd.h>
#include <bsd.h>
#ifdef CONFIG_SUPL_CLIENT_LIB
#include <modem/lte_lc.h>
#include <supl_os_client.h>
#include <supl_session.h>
#include "supl_support.h"
#endif
#include "gps.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(app_gps, LOG_LEVEL_DBG);

#define GPS_PROCESS_STACK_SIZE  KB(4)
#define GPS_PROCESS_PRIORITY    K_LOWEST_APPLICATION_THREAD_PRIO

K_THREAD_STACK_DEFINE(gps_process_stack_area, GPS_PROCESS_STACK_SIZE);

struct k_thread gps_process_thread_data;
k_tid_t gps_process_tid = NULL;

static int gnss_fd = -1;
nrf_gnss_data_frame_t gps_data;
nrf_gnss_data_frame_t last_fix;
static bool got_first_fix = false;
static bool m_gps_process_stop = true;
static bool m_gps_process_stopped = true;
static bool agps_data_request = false;
static bool gps_started = false;

static int process_gps_data()
{
    int retval;

    retval = nrf_recv(gnss_fd,
              &gps_data,
              sizeof(nrf_gnss_data_frame_t),
              NRF_MSG_DONTWAIT);

    if (retval > 0) {

        switch (gps_data.data_id) {
        case NRF_GNSS_PVT_DATA_ID:

            if ((gps_data.pvt.flags &
                NRF_GNSS_PVT_FLAG_FIX_VALID_BIT)
                == NRF_GNSS_PVT_FLAG_FIX_VALID_BIT) {

                if (!got_first_fix) {
                    got_first_fix = true;
                }

                memcpy(&last_fix,
                       &gps_data,
                       sizeof(nrf_gnss_data_frame_t));

            }
            break;

        case NRF_GNSS_NMEA_DATA_ID:
            break;
        case NRF_GNSS_AGPS_DATA_ID:
            agps_data_request = true;
            return 0;
            break;

        default:
            break;
        }
    }

    return retval;
}

#ifdef CONFIG_SUPL_CLIENT_LIB
int inject_agps_type(void *agps,
             size_t agps_size,
             nrf_gnss_agps_data_type_t type,
             void *user_data)
{
    ARG_UNUSED(user_data);
    int retval = nrf_sendto(gnss_fd,
                agps,
                agps_size,
                0,
                &type,
                sizeof(type));

    if (retval != 0) {
        LOG_ERR("Failed to send AGNSS data, type: %d (err: %d)",
               type,
               errno);
        return -1;
    }

    LOG_INF("Injected AGPS data, flags: %d, size: %d", type, agps_size);

    return 0;
}
#endif

static bool gps_open(int gps_fd){

    int retval;
    nrf_gnss_fix_retry_t    fix_retry    = 0;
    nrf_gnss_fix_interval_t fix_interval = 1;
    nrf_gnss_delete_mask_t  delete_mask  = 0;
    nrf_gnss_nmea_mask_t    nmea_mask    = 0;

    retval = nrf_setsockopt(gps_fd,
                NRF_SOL_GNSS,
                NRF_SO_GNSS_FIX_RETRY,
                &fix_retry,
                sizeof(fix_retry));

    if (retval != 0) {
        LOG_ERR("ERR: Failed to set fix retry value");
        //nrf_close(gnss_fd);
        //gnss_fd = -1;
        return false;
    }

    retval = nrf_setsockopt(gps_fd,
                NRF_SOL_GNSS,
                NRF_SO_GNSS_FIX_INTERVAL,
                &fix_interval,
                sizeof(fix_interval));

    if (retval != 0) {
        LOG_ERR("ERR: Failed to set fix interval value");
        return false;
    }

    retval = nrf_setsockopt(gps_fd,
                NRF_SOL_GNSS,
                NRF_SO_GNSS_NMEA_MASK,
                &nmea_mask,
                sizeof(nmea_mask));

    if (retval != 0) {
        LOG_ERR("ERR: Failed to set nmea mask");
        //nrf_close(gnss_fd);
        //gnss_fd = -1;
        return -1;
    }

    retval = nrf_setsockopt(gps_fd,
                NRF_SOL_GNSS,
                NRF_SO_GNSS_START,
                &delete_mask,
                sizeof(delete_mask));

    if (retval != 0) {
        LOG_ERR("ERR: Failed to start GPS");
        //nrf_close(gnss_fd);
        //gnss_fd = -1;
        return false;
    }

    gps_started = true;
    got_first_fix = false;
    agps_data_request = false;
    
    return true;
}

static bool gps_close(int gps_fd){

    int retval;
    nrf_gnss_delete_mask_t  delete_mask  = 0;


    retval = nrf_setsockopt(gps_fd,
                NRF_SOL_GNSS,
                NRF_SO_GNSS_STOP,
                &delete_mask,
                sizeof(delete_mask));

    if (retval != 0) {
        LOG_ERR("ERR: Failed to stop GPS");
        return false;
    }
    gps_started = false;
    return true;
}

static void gps_process(void *p1, void *p2, void *p3){

    m_gps_process_stop = false;
    m_gps_process_stopped = false;

    LOG_DBG("START");

    while(m_gps_process_stop == false){

        do {
            /* Loop until we don't have more
             * data to read
             */
        } while (process_gps_data() > 0);

#ifdef CONFIG_SUPL_CLIENT_LIB
        if(agps_data_request){
            agps_data_request = false;
            LOG_INF("New AGPS data requested, contacting SUPL server, flags %d",
                   gps_data.agps.data_flags);
            gps_close(gnss_fd);
      if (open_supl_socket() == 0) {
          LOG_INF("Starting SUPL session");
          supl_session(&gps_data.agps);
          LOG_INF("Done");
          close_supl_socket();
      }
            gps_open(gnss_fd);
            k_sleep(SYS_TIMEOUT_MS(2000));
        }
#endif

        k_sleep(SYS_TIMEOUT_MS(500));

    }

    m_gps_process_stopped = true;

}

#define AT_MAGPIO           "AT\%XMAGPIO=1,0,0,1,1,1574,1577"//"AT\%XMAGPIO"
#define CME_ERR_GPS_NOT_FIX             516


bool gps_start(void){

    if(!gps_started){

        gnss_fd = nrf_socket(NRF_AF_LOCAL, NRF_SOCK_DGRAM, NRF_PROTO_GNSS);

        if (gnss_fd < 0) {
            LOG_ERR("ERR: Could not init socket (err: %d)", gnss_fd);
            return false;
        }

        if(gps_open(gnss_fd) == false){
            return false;
        }

        gps_process_tid = k_thread_create(&gps_process_thread_data, gps_process_stack_area,
                                     K_THREAD_STACK_SIZEOF(gps_process_stack_area),
                                     gps_process,
                                     NULL, NULL, NULL,
                                     GPS_PROCESS_PRIORITY, 0, K_NO_WAIT);

    }else{
        return false;
    }
    return true;
}

bool gps_stop(){
    
    int retval;
    uint32_t start_time;
    
    if(gps_started && m_gps_process_stopped == false){

        m_gps_process_stop = true;

        start_time = k_uptime_get_32();

        while(m_gps_process_stopped == false && (k_uptime_get_32() - start_time) < 2000){
            k_sleep(SYS_TIMEOUT_MS(10));
        }
        //k_thread_abort(gps_process_tid);
        if(m_gps_process_stopped == false){
            LOG_ERR("ERR: GPS process task fail to terminate itself. Force task abort!"); 
            k_thread_abort(gps_process_tid);
            m_gps_process_stopped = true;
        }

        if(gps_close(gnss_fd)==false){
            return false;
        }

        retval = nrf_close(gnss_fd);

        if (retval != 0) {
            LOG_ERR("ERR: Failed to close socket! (err: %d)", retval);
            return false;
        }

        gnss_fd = -1;
        return true;

    }else{

        return false;
    }
}

bool gps_init(void){

    const char at_commands[][31]  = { AT_MAGPIO };
    enum at_cmd_state state;
#ifdef CONFIG_SUPL_CLIENT_LIB
    static struct supl_api supl_api = {
        .read       = supl_recv,
        .write      = supl_send,
        .handler    = inject_agps_type,
        .logger     = supl_log,
        .counter_ms = k_uptime_get
    };
#endif

    for(int i=0; i<ARRAY_SIZE(at_commands); i++){
        if (at_cmd_write(at_commands[i], NULL, 0, &state) != 0) {
            LOG_ERR("ERR: Fail to send %s", at_commands[i]);
            return false;
        }
        if(state != AT_CMD_OK){
            LOG_ERR("[ERR: %s returns error!", at_commands[i]);
            return false;
        }
    }
#ifdef CONFIG_SUPL_CLIENT_LIB
    int rc = supl_init(&supl_api);

    if (rc != 0) {
        return false;
    }
#endif
    gps_started = false;
    return true;
}

bool gps_is_active(void){

    return gps_started;
}

bool gps_acquire_loc(float *p_latitude, float *p_longitude){

    if(!gps_started){
        return false;
    }
    if(got_first_fix){
        *p_latitude = last_fix.pvt.latitude;
        *p_longitude = last_fix.pvt.longitude;
        return true;
    }
    return false;
}

bool gps_acquire_latest_pvt_data(nrf_gnss_pvt_data_frame_t *p_pvt){

    if(!gps_started){
        return false;
    }

    if(got_first_fix){
        memcpy(p_pvt, &last_fix.pvt, sizeof(nrf_gnss_pvt_data_frame_t));
        return true;
    }

    return false;
}
