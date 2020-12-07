/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */

#include <logging/log.h>

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <device.h>
#include <drivers/uart.h>
#include <fw_info.h>
#include <modem/bsdlib.h>
#include <modem/modem_key_mgmt.h>
#include <bsd.h>
#include <net/fota_download.h>
#include <power/reboot.h>
#include <dfu/mcuboot.h>
#include <modem/lte_lc.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/nvs.h>
#include <ncs_version.h>

#include "slm_at_host.h"
#include "orientation_tap_detector.h"
#include "rotation_detector.h"
#include "pir_detector.h"
#include "env_sensors.h"

#include "gps.h"

#include "IoTConnect_Config.h"
#include "IoTConnect.h"

#define ENABLE_BLE_BEACON
#if defined (ENABLE_BLE_BEACON)
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#endif


LOG_MODULE_REGISTER(app, CONFIG_DEMO_IOTCONNECT_LOG_LEVEL);


/* used by slm_at_xxx */
struct at_param_list at_param_list;
char rsp_buf[CONFIG_AT_CMD_RESPONSE_MAX_LEN];

/* Stack definition for application workqueue */
#define APPLICATION_WORKQUEUE_STACK_SIZE    KB(4)
#define APPLICATION_WORKQUEUE_PRIORITY      CONFIG_SYSTEM_WORKQUEUE_PRIORITY
K_THREAD_STACK_DEFINE(application_stack_area, APPLICATION_WORKQUEUE_STACK_SIZE);

struct k_work_q application_work_q; /* also being used in slm_gps.c */
static struct k_work exit_cmd_mode_work;
static struct k_delayed_work check_location_work;
static struct k_delayed_work get_env_sensor_reading_work;
static struct nvs_fs m_fs;
#define LTE_MODE_NVS_ID      1

static enum lte_lc_system_mode m_system_mode;

/* forward declaration */
#if defined (GET_ORIENTATION_XYZ) && (GET_ORIENTATION_XYZ > 0)
static void send_orientation_xyz_data(int16_t x, int16_t y, int16_t z);
#endif
static void send_env_sensor_data(env_sensors_data_t *p_sdata);
static void send_orientation_data(orientation_state_t orientation);
static void send_pir_data(bool detected);
static void send_rotation_data(float max_rotation_speed);
static void iotconnect_event_handler(IoTConnect_event_t *p_evt);
static void start_command_mode(void);
static void lte_mode_nvs_read(uint8_t *mode);
static int lte_mode_nvs_write(uint8_t mode);
static void send_device_static_properties_data(void);
static void get_device_properties_data(void);

static char m_recv_buf[IOTCONNECT_PAYLOAD_BUFFER_SIZE];

static enum lte_lc_nw_reg_status m_lte_lc_nw_reg_status;

#define IMEI_LEN        15
static uint8_t m_imei[IMEI_LEN + 5]; /* account for \r\n */

static uint8_t m_mfw_ver[30];
static uint8_t m_iccid[23];
static uint16_t m_mcc;
static uint16_t m_mnc;
static uint16_t m_lac;
static uint32_t m_cid;

typedef enum {
    CLOUD_STATE_IOTCONNECT_SDK_UNINT,
    CLOUD_STATE_DISCONNECTED,
    CLOUD_STATE_CONNECTING,
    CLOUD_STATE_CONNECTED
} cloud_conn_state_t;
static cloud_conn_state_t m_cloud_conn_state;
static IoTConnect_init_t m_iotconnect_init = {
#if defined (CONFIG_IOTCONNECT_DEVICE_UNIQUE_ID)
    .dev_unique_id = CONFIG_IOTCONNECT_DEVICE_UNIQUE_ID,
#endif
    .dev_cpid = CONFIG_IOTCONNECT_DEVICE_CP_ID,
    .env = CONFIG_IOTCONNECT_DEVICE_ENV,
    .discovery_connection_sec_tag = CONFIG_IOTCONNECT_DISCOVERY_SEC_TAG,
    .mqtt_connection_sec_tag = CONFIG_IOTCONNECT_MQTT_SEC_TAG,
    .systime_sync_interval = CONFIG_IOTCONNECT_SYSTEM_TIME_UPDATE,
    .evt_cb_func = iotconnect_event_handler
};

static char m_ota_ack_id[IOTCONNECT_SDK_GUID_MAX_LEN];
static uint16_t m_ota_ack_msg_id = 0;
static bool m_is_ongoing_ota = false;
static bool m_do_reboot = false;
static bool m_do_switch_to_command_mode = false;
static bool m_is_location_tracking_started = false;
static bool m_do_read_iccid = true;
static bool m_do_read_mobile_network_info = true;

static orientation_state_t m_current_orientation;

#if defined (ENABLE_BLE_BEACON)
/*
 * Set Advertisement data. Based on the Eddystone specification:
 * https://github.com/google/eddystone/blob/master/protocol-specification.md
 * https://github.com/google/eddystone/tree/master/eddystone-url
 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
    BT_DATA_BYTES(BT_DATA_SVC_DATA16,
              0xaa, 0xfe, /* Eddystone UUID */
              0x10, /* Eddystone-URL frame type */
              0x00, /* Calibrated Tx power at 0m */
              0x03, /* URL Scheme Prefix https:// */
              'a', 'v', 't', '9', '1', '5', '2', '.', 'g', 'i', 't', 'h', 'u', 'b', '.', 'i', 'o')
};

static void bt_ready(int err)
{
    if (err != 0) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    size_t count = 1;
    bt_addr_le_t addr;
    bt_id_get(&addr, &count);
    if (count) {
        LOG_INF("bdaddr ==> %02x:%02x:%02x:%02x:%02x:%02x", 
                addr.a.val[5], addr.a.val[4], addr.a.val[3], 
                addr.a.val[2],addr.a.val[1], addr.a.val[0]);
    }

    /* Start advertising */
    struct bt_le_adv_param adv_param = /* BT_LE_ADV_NCONN */
                        BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY, 
                                            BT_GAP_ADV_FAST_INT_MIN_2, 
                                            BT_GAP_ADV_FAST_INT_MAX_2, NULL);
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err != 0) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Beacon started");
}
#endif


void bsd_recoverable_error_handler(uint32_t err)
{
    LOG_ERR("bsdlib recoverable error: %u", err);
}

static bool has_needed_certificates(void)
{
    int err;
    bool exists;
    uint8_t unused;
    uint8_t count = 0;

    err = modem_key_mgmt_exists(CONFIG_IOTCONNECT_MQTT_SEC_TAG,
                    MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
                    &exists, &unused);
    if (err) {
        LOG_ERR("Failed to check for CA_CHAIN. err %d", err);
        return err;
    }
    if (exists == false) {
        LOG_INF("Missing IoTConnect CA Certifiacte");
    }
    count += exists;
    err = modem_key_mgmt_exists(CONFIG_IOTCONNECT_MQTT_SEC_TAG,
                    MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
                    &exists, &unused);
    if (err) {
        LOG_ERR("Failed to check for PUBLIC_CERT. err %d", err);
        return err;
    }	
    if (exists == false) {
        LOG_INF("Missing IoTConnect Target Client Certificate");
    }
    count += exists;
    err = modem_key_mgmt_exists(CONFIG_IOTCONNECT_MQTT_SEC_TAG,
                    MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
                    &exists, &unused);
    if (err) {
        LOG_ERR("Failed to check for PRIVATE_CERT. err %d", err);
        return err;
    }
    if (exists == false) {
        LOG_INF("Missing IoTConnect Target Private Key");
    }
    count += exists;
    err = modem_key_mgmt_exists(CONFIG_IOTCONNECT_DISCOVERY_SEC_TAG,
                    MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
                    &exists, &unused);
    if (err) {
        LOG_ERR("Failed to check for discovery CA_CHAIN. err %d", err);
        return err;
    }
    if (exists == false) {
        LOG_INF("Missing IoTConnect discovery CA Certifiacte");
    }
    count += exists;

    return (count == 4);
}

static char * get_orientation_string(orientation_state_t orientation)
{
    static char orientation_str_list[7][3] = {
                                    "--", "L", "RL", "RP", "P", "FD", "FU"};
    return orientation_str_list[orientation];
}

static void handle_orientation_change(orientation_state_t new_orientation)
{
    m_current_orientation = new_orientation;
    LOG_INF("Device orientation: %s", 
                get_orientation_string(m_current_orientation));
    send_orientation_data(m_current_orientation);
}

static void handle_tap_detection(orientation_state_t event_orientation)
{
    if (event_orientation != m_current_orientation) {
        LOG_INF("tap @ %d <> curr %d", 
                    event_orientation, m_current_orientation);
    }
    LOG_INF("Tap detected with device orientation %s", 
                    get_orientation_string(m_current_orientation));
    if (m_current_orientation == ORIENTATION_FACE_DOWN) {
        m_do_switch_to_command_mode = true;
        LOG_INF("Preparing to switch to command mode...");
    }
#if 0 /* test gps */
    else if ((m_current_orientation == ORIENTATION_FACE_UP) && 
            (m_is_ongoing_ota == false)){
        if (gps_is_active() == false) {
            if (gps_start() == false) {
                LOG_ERR("Failed to start GPS");
            }else{
                LOG_INF("GPS started");
                m_is_location_tracking_started = true;
                int err = k_delayed_work_submit_to_queue(&application_work_q, 
                                                &check_location_work, 
                                                K_SECONDS(5));
                if (err != 0) {
                    LOG_ERR("k_delayed_work_submit_to_queue(check_location) "
                                    "failed (err: %d)", err);
                }
            }
        } else {
            if (gps_stop() == false) {
                LOG_ERR("Failed to stop GPS");
            }else{
                LOG_INF("GPS stopped");
                m_is_location_tracking_started = false;
                k_delayed_work_cancel(&check_location_work);
            }
        }
    }
#endif
}

static void orientation_tap_event_handler(orientation_tap_event_t  event)
{
    if (event.tap_detected == false) {
        handle_orientation_change(event.orientation);
#if defined (GET_ORIENTATION_XYZ) && (GET_ORIENTATION_XYZ > 0)
        if (event.x < 0x800) { // xyz is 12-bit 2's complement
            LOG_INF("x %d y %d z %d", event.x, event.y, event.z);
            send_orientation_xyz_data(event.x, event.y, event.z);
        }
#endif
    } else {
        handle_tap_detection(event.orientation);
    }
}

static void rotation_event_handler(float max_rotation_speed)
{
    char temp[20];
    snprintf(temp, sizeof(temp), "%.02f RPM", max_rotation_speed);
    LOG_INF("Max rotation speed on Z-axis: %s", temp);
    send_rotation_data(max_rotation_speed);
}

static void pir_event_handler(bool pir_detected)
{
    LOG_INF("PIR: %s", pir_detected ? "ON" : "OFF");
    send_pir_data(pir_detected);
}

static void c2d_data_handler(IoTConnect_dev_c2d_event_t *p_evt)
{
    if (m_is_ongoing_ota == true) {
        /*
         * ignore all messages
         * noticed failing to finish download new image if 
         * continue to publish mqtt messages
         */
        p_evt->ack_act = IoTConnect_ack_act_no_ack;
        return;
    }

    if (p_evt->cmd_type == IoTConnect_cmd_type_device) {
        // let's see what command was issued
        LOG_INF("command ==> %s", p_evt->data.dev.p_cmd);

        if (strcmp(p_evt->data.dev.p_cmd, "device_reboot") == 0) {
                    m_do_reboot = true;
        }
#if 0 /* this section can be enabled if to allow enabling/disabling of 
         pir detection and gps location tracking through command */
        else if (strcmp(p_evt->data.dev.p_cmd, "pir_on 1") == 0) {
            int err = pir_detector_init_and_start(&application_work_q, 
                                                    pir_event_handler);
            if (err == -EALREADY) {
                LOG_INF("Motion PIR detection ALREADY started");
            } else if (err != 0) {
                LOG_ERR("Failed to start motion PIR detection");
                p_evt->ack_act = IoTConnect_ack_act_ack_fail;
            } else {
                LOG_INF("Motion PIR detection started");
            }
        } else if (strcmp(p_evt->data.dev.p_cmd, "pir_on 0") == 0) {
            pir_detector_stop_and_unit();
            LOG_INF("Motion PIR detection stopped");
        } else if (strcmp(p_evt->data.dev.p_cmd, "gps_on 1") == 0) {
            if (gps_is_active() == false) {
                if (gps_start() == false) {
                    LOG_ERR("Failed to start GPS");
                    p_evt->ack_act = IoTConnect_ack_act_ack_fail;
                }else{
                    LOG_INF("GPS started");
                    m_is_location_tracking_started = true;
                    int err = k_delayed_work_submit_to_queue(
                                                    &application_work_q, 
                                                    &check_location_work, 
                                                    K_SECONDS(5));
                    if (err != 0) {
                        LOG_ERR("k_delayed_work_submit_to_queue(check_location)"
                                        " failed (err: %d)", err);
                    }
                }
            } else {
                LOG_INF("GPS ALREADY started");
            }
        } else if (strcmp(p_evt->data.dev.p_cmd, "gps_on 0") == 0) {
            if (gps_is_active() == true) {
                if (gps_stop() == false) {
                    LOG_ERR("Failed to stop GPS");
                    p_evt->ack_act = IoTConnect_ack_act_ack_fail;
                }else{
                    LOG_INF("GPS stopped");
                    m_is_location_tracking_started = false;
                    k_delayed_work_cancel(&check_location_work);
                }
            } else {
                LOG_ERR("GPS not started");
            }
        }
#endif
        else {
            /*TODO: other comamnd handling */
            if (p_evt->ack_act == IoTConnect_ack_act_ack_ok) {
                p_evt->ack_act = IoTConnect_ack_act_ack_fail;
            }
        }
    } else if (p_evt->cmd_type == IoTConnect_cmd_type_ota) {
        if (m_is_ongoing_ota == true) {
            // reject new request
            p_evt->ack_act = IoTConnect_ack_act_ack_fail;
            return;
        }

        /* let's see the version */
        LOG_INF("sw ver ==> %s", p_evt->data.ota.p_sw_ver);
        LOG_INF("hw ver ==> %s", p_evt->data.ota.p_hw_ver);

        /* let's check the URL list */
        LOG_INF("Number of URLs %d", p_evt->data.ota.url_count);

        if (p_evt->data.ota.url_count > 0) {
            /* lets get the 1st url */
            char *url_value = (char *)p_evt->data.ota.p_urls[0];

            /* get hostname and filename */
            /* assume always https */
            char *hostname = strstr(url_value, "https://");
            __ASSERT(hostname != NULL, "not able to find https://");
            hostname += strlen("https://");
            char *filename = strstr(hostname, "/");
            __ASSERT(filename != NULL, "not able to find / before filename");
            *filename = '\0';
            filename += 1;
            LOG_INF("Hostname ==> %s", hostname);
            LOG_INF("filename ==> %s", filename);

            /* let's save the ackId for later to send */
            strcpy(m_ota_ack_id, p_evt->p_ack_id);
            p_evt->ack_act = IoTConnect_ack_act_no_ack;

            int retval = fota_download_start(hostname, filename, 
                                    CONFIG_IOTCONNECT_MQTT_SEC_TAG, NULL, 0);
            if (retval == 0) {
                m_is_ongoing_ota = true;
            }
            LOG_INF("fota_download_start() retval %d", retval);

        }
    } else {
        /*TODO: handle other possible C2D commands listed in IoTConnect.h */
        LOG_ERR("Unexpected cmdType 0x%02x received", (uint8_t)p_evt->cmd_type);
    }
}

static void twin_c2d_data_handler(IoTConnect_twin_c2d_event_t *p_evt)
{
    IoTConnect_c2d_twin_data_t *p_data = 
                            (IoTConnect_c2d_twin_data_t *)p_evt->p_twin_data;

    for (int i = 0; i < p_evt->twin_data_count; i++) {
        LOG_INF("Key=\"%s\", desired value=\"%s\"", 
                    p_data[i].p_key, p_data[i].p_value);

        if (strcmp(p_data[i].p_key, "PIR_ON") == 0) {
            if (p_data[i].p_value[0] == '1') {
                int err = pir_detector_init_and_start(&application_work_q, 
                                                        pir_event_handler);
                if (err == -EALREADY) {
                    LOG_INF("Motion PIR detection ALREADY started");
                } else if (err != 0) {
                    LOG_ERR("Failed to start motion PIR detection");
                } else {
                    LOG_INF("Motion PIR detection started");
                }
            } else {
                pir_detector_stop_and_unit();
                LOG_INF("Motion PIR detection stopped");
            }
        } else if (strcmp(p_data[i].p_key, "GPS_ON") == 0) {
            if (p_data[i].p_value[0] == '1') {
                if (gps_is_active() == false) {
                    if (gps_start() == false) {
                        LOG_ERR("Failed to start GPS");
                    } else {
                        LOG_INF("GPS started");
                        m_is_location_tracking_started = true;
                        int err = k_delayed_work_submit_to_queue(
                                                        &application_work_q, 
                                                        &check_location_work, 
                                                        K_SECONDS(5));
                        if (err != 0) {
                            LOG_ERR("k_delayed_work_submit_to_queue"
                                    "(check_location) failed (err: %d)", err);
                        }
                    }
                } else {
                    LOG_INF("GPS ALREADY started");
                }
            } else {
                if (gps_is_active() == true) {
                    if (gps_stop() == false) {
                        LOG_ERR("Failed to stop GPS");
                    } else {
                        LOG_INF("GPS stopped");
                        m_is_location_tracking_started = false;
                        k_delayed_work_cancel(&check_location_work);
                    }
                }
            }
        }
    }
}


static void iotconnect_event_handler(IoTConnect_event_t *p_evt)
{
    switch(p_evt->evt_type) {
        case IOTCONNECT_EVT_SDK:
            switch (p_evt->evt.sdk.evt_id) {
                case IOTCONNECT_SDK_EVT_CONNECTED:
                    LOG_INF("IOTCONNECT_SDK_EVT_CONNECTED result %d", 
                                p_evt->evt.sdk.data.err_code);
                    m_cloud_conn_state = CLOUD_STATE_CONNECTED;
                    send_device_static_properties_data();
                    get_device_properties_data();
                    break;

                case IOTCONNECT_SDK_EVT_DISCONNECTED:
                    LOG_INF("IOTCONNECT_SDK_EVT_DISCONNECTED result %d", 
                                p_evt->evt.sdk.data.err_code);
                    m_cloud_conn_state = CLOUD_STATE_DISCONNECTED;
                    pir_detector_stop_and_unit();
                    if (m_is_location_tracking_started == true) {
                        m_is_location_tracking_started = false;
                        k_delayed_work_cancel(&check_location_work);
                        gps_stop();
                    }
                    break;

                case IOTCONNECT_SDK_EVT_DATA_SEND_OK:
                    LOG_INF("IOTCONNECT_SDK_EVT_DATA_SEND_OK msg_id %d", 
                                p_evt->evt.sdk.data.msg_id);
                    if (p_evt->evt.sdk.data.msg_id == m_ota_ack_msg_id) {
                        m_ota_ack_msg_id = 0;
                    }
                    break;

                case IOTCONNECT_SDK_EVT_DATA_SEND_ERR:
                    LOG_INF("IOTCONNECT_SDK_EVT_DATA_SEND_ERR result %d", 
                                p_evt->evt.sdk.data.err_code);
                    if (p_evt->evt.sdk.data.msg_id == m_ota_ack_msg_id) {
                        /*TODO: shall we retry? */
                        m_ota_ack_msg_id = 0;
                    }
                    break;

                case IOTCONNECT_SDK_EVT_CONNECT_FAIL:
                    LOG_INF("IOTCONNECT_SDK_EVT_CONNECT_FAIL result %d", 
                                p_evt->evt.sdk.data.err_code);
                    m_cloud_conn_state = CLOUD_STATE_DISCONNECTED;
                    break;

                case IOTCONNECT_SDK_EVT_DISCONNECT_FAIL:
                    LOG_INF("IOTCONNECT_SDK_EVT_DISCONNECT_FAIL result %d", 
                                p_evt->evt.sdk.data.err_code);
                    break;

                case IOTCONNECT_SDK_EVT_TOPICS_SUB_ERR:
                    LOG_INF("IOTCONNECT_SDK_EVT_TOPICS_SUB_ERR result %d", 
                                p_evt->evt.sdk.data.err_code);
                    break;
            }
            break;

        case IOTCONNECT_EVT_DEV_C2D_DATA:
            LOG_INF("IOTCONNECT_EVT_DEV_C2D_DATA");
            LOG_INF("%s", m_recv_buf);

            if ((m_do_reboot == false) && 
                (m_do_switch_to_command_mode == false)) {
                c2d_data_handler(&(p_evt->evt.dev_data));
            }
            break;

        case IOTCONNECT_EVT_TWIN_C2D_DATA:
            LOG_INF("IOTCONNECT_EVT_TWIN_C2D_DATA");

            if ((m_is_ongoing_ota == false) &&
                (m_do_reboot == false) && 
                (m_do_switch_to_command_mode == false)) {
                twin_c2d_data_handler(&(p_evt->evt.twin_data));
            } else {
                for (int i = 0; i < p_evt->evt.twin_data.twin_data_count; i++) {
                    LOG_INF("Ignore Key=\"%s\", desired value=\"%s\"", 
                                p_evt->evt.twin_data.p_twin_data[i].p_key, 
                                p_evt->evt.twin_data.p_twin_data[i].p_value);
                }
            }
            break;
    }
}

void fota_dl_handler(const struct fota_download_evt *evt)
{
    IoTConnect_dev_status_t status;
    bool send_ack = false;
    int err;

    switch (evt->id) {
#if defined (CONFIG_FOTA_DOWNLOAD_PROGRESS_EVT)
    case FOTA_DOWNLOAD_EVT_PROGRESS:
        LOG_INF("FOTA_DOWNLOAD_EVT_PROGRESS %d", evt->progress);
        break;
#endif
    case FOTA_DOWNLOAD_EVT_ERROR:
        LOG_INF("FOTA_DOWNLOAD_EVT_ERROR");
        status = IoTConnect_ack_status_ota_fail;
        send_ack = true;
        break;
    case FOTA_DOWNLOAD_EVT_FINISHED:
        LOG_INF("FOTA_DOWNLOAD_EVT_FINISHED");
        status = IoTConnect_ack_status_ota_ok;
        send_ack = true;
        break;

    default:
        break;
    }

    if (send_ack == true) {
        __ASSERT(m_ota_ack_id[0] != 0, "m_ota_ack_id is NULL");
        m_ota_ack_msg_id = 0;
        err = IoTConnect_device_send_ack(IoTConnect_msg_type_ota_ack, 
                                    m_ota_ack_id, status, &m_ota_ack_msg_id);
        if (err != 0) {
            /*TODO: shall we retry? */
            m_ota_ack_msg_id = 0;
            LOG_ERR("IoTConnect_device_send_ack() failed %d", err);
        }
        if (status == (IoTConnect_dev_status_t)IoTConnect_ack_status_ota_ok) {
            LOG_DBG("ota ack message id %d", m_ota_ack_msg_id);
            m_do_reboot = true;
        } else {
            m_ota_ack_msg_id = 0;
            m_is_ongoing_ota = false;
        }
        m_ota_ack_id[0] = 0;
    }
}

static int send_data_to_cloud(IoTConnect_attr_t *p_attr, 
                                    int in_attr_count, char *log_str)
{
    uint16_t msg_id;

    int err = IoTConnect_device_send_data(p_attr, in_attr_count, &msg_id);
    if (err != 0) {
        LOG_ERR("IoTConnect_device_send_data(%s) failed %d", log_str, err);
    } else {
        LOG_INF("send_data(%s) msg_id %d", log_str, msg_id);
    }

    return err;
}

static void get_device_properties_data(void)
{
    int err = IoTConnect_device_get_all_twin_data();

    if (err != 0) {
        LOG_ERR("IoTConnect_device_get_all_tiwn_data() failed %d", err);
    }
}

/* copied from lte_lc.c */
static bool response_is_valid(const char *response, size_t response_len,
              const char *check)
    {
    if ((response == NULL) || (check == NULL)) {
        LOG_ERR("Invalid pointer provided");
        return false;
    }

    if ((response_len < strlen(check)) ||
        (memcmp(response, check, response_len) != 0)) {
        return false;
    }

    return true;
}

static int get_mobile_network_info(void)
{
    /* sample response: %XMONITOR: 1,"EDAV","EDAV","26295","00B7",7,4,"00011B07",7,2300,63,39,"","11100000","11100000","00000000" */
#define PLMN_PARAM_INDEX        4
#define TAC_PARAM_INDEX         5
#define CELLID_PARAM_INDEX      8
#define XMONITOR_PARAM_COUNT_OF_INTEREST    9
#define XMONITOR_RESPONSE_PREFIX            "%XMONITOR"

    char buf[150];
    char tmp_buf[10];
    size_t len;
    int err;
    struct at_param_list resp_list = {0};
    uint32_t plmn = 0;
    char  response_prefix[sizeof(XMONITOR_RESPONSE_PREFIX)] = {0};
    size_t response_prefix_len = sizeof(response_prefix);

    err = at_params_list_init(&resp_list, XMONITOR_PARAM_COUNT_OF_INTEREST);
    if (err) {
        LOG_ERR("Could not init AT params list, error: %d", err);
        return err;
    }

    err = at_cmd_write("AT%XMONITOR", buf, sizeof(buf), NULL);
    if (err) {
        LOG_ERR("Could not issue AT%%XMONITOR, error: %d", err);
    } else {
        LOG_DBG("AT%%XMONITOR ==> %s", buf);
        /* Parse response and populate AT parameter list */
        err = at_parser_max_params_from_str(buf, NULL, &resp_list,
                                            XMONITOR_PARAM_COUNT_OF_INTEREST);
        if (err != -E2BIG) {
            LOG_ERR("Could not parse %%XMONITOR response, error: %d", err);
            goto clean_exit;
        }

        /* Check if AT command response starts with %XSYSTEMMODE */
        err = at_params_string_get(&resp_list, 0,
                                   response_prefix, &response_prefix_len);
        if (err) {
            LOG_ERR("Could not get response prefix, error: %d", err);
            goto clean_exit;
        }

        if (!response_is_valid(response_prefix, response_prefix_len,
                                XMONITOR_RESPONSE_PREFIX)) {
            LOG_ERR("Invalid XMONITOR response");
            err = -EIO;
            goto clean_exit;
        }

        /* Get the PLMN from the response */
        len = sizeof(tmp_buf) - 1;
        err = at_params_string_get(&resp_list, PLMN_PARAM_INDEX, tmp_buf, &len);
        if (err) {
            LOG_ERR("Could not get PLMN, error: %d", err);
            goto clean_exit;
        }
        /* mcc is the first 3 digits followed by mnc of 2 or 3 digits */
        tmp_buf[len] = '\0';
        plmn = strtoul(tmp_buf, NULL, 10);
        uint16_t   divisor;
        if (len == 5) {
            divisor = 100;
        } else {
            divisor = 1000;
        }
        m_mcc = plmn / divisor;
        m_mnc = plmn % divisor;
        LOG_INF("PLMN %d ==> mcc %d mnc %d", plmn, m_mcc, m_mnc);

        /* Get the TAC from the response */
        len = sizeof(tmp_buf) - 1;
        err = at_params_string_get(&resp_list, TAC_PARAM_INDEX, tmp_buf, &len);
        if (err) {
            LOG_ERR("Could not get TAC, error: %d", err);
            goto clean_exit;
        }
        tmp_buf[len] = '\0';
        m_lac = strtoul(tmp_buf, NULL, 16);
        LOG_INF("TAC %s (hex) ==> %d (dec)", tmp_buf, m_lac);

        /* Get the CELLID from the response */
        len = sizeof(tmp_buf) - 1;
        err = at_params_string_get(&resp_list, CELLID_PARAM_INDEX, 
                                    tmp_buf, &len);
        if (err) {
            LOG_ERR("Could not get CELLID, error: %d", err);
            goto clean_exit;
        }
        tmp_buf[len] = '\0';
        m_cid = strtoul(tmp_buf, NULL, 16);
        LOG_INF("CELLID %s (hex) ==> %d (dec)", tmp_buf, m_cid);
    }

clean_exit:
    at_params_list_free(&resp_list);
    return err;
}

static void send_device_static_properties_data(void)
{
    uint16_t msg_id;
    int prop_num = 3;
    IoTConnect_twin_prop_t dev_prop[] = {
        {
            .name = "Modem_IMEI",
            .data_type = IoTConnect_twin_prop_data_type_string,
        },
        {
            .name = "Modem_FW_Ver",
            .data_type = IoTConnect_twin_prop_data_type_string,
        },
        {
            .name = "App_FW_Ver",
            .data_type = IoTConnect_twin_prop_data_type_string,
        },
        {
            .name = "SIM_ICCID",
            .data_type = IoTConnect_twin_prop_data_type_string,
        },
        {
            .name = "MCC",
            .data_type = IoTConnect_twin_prop_data_type_integer,
        },
        {
            .name = "MNC",
            .data_type = IoTConnect_twin_prop_data_type_integer,
        },
        {
            .name = "LAC",
            .data_type = IoTConnect_twin_prop_data_type_integer,
        },
        {
            .name = "CID",
            .data_type = IoTConnect_twin_prop_data_type_integer,
        }
    };

    if (m_do_read_iccid == true) {
        char tmp[40];
        memset(m_iccid, 0, sizeof(m_iccid));
        int err = at_cmd_write("AT%XICCID", tmp, sizeof(tmp), NULL);
        if (err) {
            LOG_ERR("Could not get SIM card ICCID, error: %d", err);
        } else {
            char *p = strstr(tmp, "\r");
            if (p != NULL) {
                *p = 0;
            }
            p = strstr(tmp, " ");
            if (p != NULL) {
                strncpy(m_iccid, p+1, 22);
            }
            LOG_INF("AT%%XICCID ==> %s", m_iccid);
            m_do_read_iccid = false;
        }
    }

    if (m_do_read_mobile_network_info == true) {
        if (get_mobile_network_info() == 0) {
            m_do_read_mobile_network_info = false;
        }
    }

    strcpy(dev_prop[0].v.str_value, m_imei);
    strcpy(dev_prop[1].v.str_value, m_mfw_ver);
    strcpy(dev_prop[2].v.str_value, CONFIG_MCUBOOT_IMAGE_VERSION);
    if (m_do_read_iccid == false) {
        strcpy(dev_prop[3].v.str_value, m_iccid);
        prop_num += 1;
        if (m_do_read_mobile_network_info == false) {
            dev_prop[4].v.int_value = m_mcc;
            dev_prop[5].v.int_value = m_mnc;
            dev_prop[6].v.int_value = m_lac;
            dev_prop[7].v.int_value = m_cid;
            prop_num += 4;
        }
    }

    int err = IoTConnect_device_send_twin_data(dev_prop, prop_num, &msg_id);
    if (err != 0) {
        LOG_ERR("IoTConnect_device_send_twin_data() failed %d", err);
    } else {
        LOG_INF("send_device_properties msg_id %d", msg_id);
    }

}

static int16_t get_rsrp_in_abs_dbm(void)
{
    /* sample response: +CESQ: 99,99,255,255,31,62 */
#define RSRP_PARAM_INDEX    6
#define RSRP_PARAM_COUNT    7
#define CESQ_RESPONSE_PREFIX     "+CESQ"
    uint16_t rsrp = 0;
    char buf[40];
    int err;
    struct at_param_list resp_list = {0};
    int16_t result = 0;
    char  response_prefix[sizeof(CESQ_RESPONSE_PREFIX)] = {0};
    size_t response_prefix_len = sizeof(response_prefix);

    err = at_params_list_init(&resp_list, RSRP_PARAM_COUNT);
    if (err) {
        LOG_ERR("Could not init AT params list, error: %d", err);
        return result;
    }

    err = at_cmd_write("AT+CESQ", buf, sizeof(buf), NULL);
    if (err) {
        LOG_ERR("Could not issue AT+CESQ, error: %d", err);
    } else {
        /* Parse response and populate AT parameter list */
        err = at_parser_params_from_str(buf,
                        NULL,
                        &resp_list);
        if (err) {
            LOG_ERR("Could not parse +CESQ response, error: %d", err);
            goto clean_exit;
        }

        /* Check if AT command response starts with +CESQ */
        err = at_params_string_get(&resp_list, 0,
                                   response_prefix, &response_prefix_len);
        if (err) {
            LOG_ERR("Could not get response prefix, error: %d", err);
            goto clean_exit;
        }

        if (!response_is_valid(response_prefix, response_prefix_len,
                       CESQ_RESPONSE_PREFIX)) {
            LOG_ERR("Invalid CESQ response");
            goto clean_exit;
        }

        /* Get the RSRP from the response */
        err = at_params_short_get(&resp_list, RSRP_PARAM_INDEX, &rsrp);
        if (err) {
            LOG_ERR("Could not get rsrp, error: %d", err);
            goto clean_exit;
        }

        /* Convert to dBm */
        if (rsrp > 97) {
            result = 0;
        } else {
            result = -141 + rsrp;
        }
        LOG_INF("RSRP %d ==> %d dBm", rsrp, result);
    }

clean_exit:
    at_params_list_free(&resp_list);
    return result;
}

static void send_env_sensor_data(env_sensors_data_t *p_sdata)
{
    static int16_t sent_rsrp = 255;
    int16_t new_rsrp;
    int attr_count;

    if ((m_cloud_conn_state != CLOUD_STATE_CONNECTED) || 
        (m_is_ongoing_ota == true)) {
        return;
    }

    new_rsrp = get_rsrp_in_abs_dbm();

    IoTConnect_attr_t attr_data[] = {
        {
            .name = "Pressure",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_float,
            .v.float_value.value = p_sdata->pressure,
            .v.float_value.precision = IoTConnect_float_2_decimal_place
        },
        {
            .name = "Temperature",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_float,
            .v.float_value.value = p_sdata->temperature,
            .v.float_value.precision = IoTConnect_float_2_decimal_place
        },
        {
            .name = "Humidity",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_float,
            .v.float_value.value = p_sdata->humidity,
            .v.float_value.precision = IoTConnect_float_2_decimal_place
        },
        {
            .name = "Light",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_integer,
            .v.int_value = p_sdata->illumination,
        },
        {
            .name = "RSRP",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_signed_integer,
            .v.int_value = new_rsrp,
        }
    };
    attr_count = ARRAY_SIZE(attr_data);
    if (new_rsrp == sent_rsrp) {
        attr_count--;
    } else {
        sent_rsrp = new_rsrp;
    }

    (void)send_data_to_cloud(attr_data, attr_count, "env_sensor");
}

#if defined (GET_ORIENTATION_XYZ) && (GET_ORIENTATION_XYZ > 0)
static void send_orientation_xyz_data(int16_t x, int16_t y, int16_t z)
{
    if ((m_cloud_conn_state != CLOUD_STATE_CONNECTED) || 
        (m_is_ongoing_ota == true)) {
        return;
    }

    IoTConnect_attr_t attr_data[] = {
        {
            .name = "Acc_X",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_signed_integer,
            .v.int_value = x,
        },
        {
            .name = "Acc_Y",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_signed_integer,
            .v.int_value = y,
        },
        {
            .name = "Acc_Z",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_signed_integer,
            .v.int_value = z,
        }
    };

    (void)send_data_to_cloud(attr_data, ARRAY_SIZE(attr_data), 
                            "orientation xyz");
}
#endif

static void send_orientation_data(orientation_state_t orientation)
{
    if ((m_cloud_conn_state != CLOUD_STATE_CONNECTED) || 
        (m_is_ongoing_ota == true)) {
        return;
    }

    IoTConnect_attr_t attr_data = {
        .name = "Orientation",
        .p_parent_name = NULL,
        .data_type = IoTConnect_attr_data_type_string
    };
    strcpy(attr_data.v.str_value, get_orientation_string(orientation));

    (void)send_data_to_cloud(&attr_data, 1, "orientation");
}

static void send_pir_data(bool detected)
{
    if ((m_cloud_conn_state != CLOUD_STATE_CONNECTED) || 
        (m_is_ongoing_ota == true)) {
        return;
    }

    IoTConnect_attr_t attr_data = {
        .name = "PIR",
        .p_parent_name = NULL,
        .data_type = IoTConnect_attr_data_type_integer,
        .v.int_value = (detected ? 1:0)
    };

    (void)send_data_to_cloud(&attr_data, 1, "pir");
}

static void send_rotation_data(float max_rotation_speed)
{
    if ((m_cloud_conn_state != CLOUD_STATE_CONNECTED) || 
        (m_is_ongoing_ota == true)) {
        return;
    }

    IoTConnect_attr_t attr_data = {
        .name = "Max_Rotation_on_Z",
        .p_parent_name = NULL,
        .data_type = IoTConnect_attr_data_type_float,
        .v.float_value.value = max_rotation_speed,
        .v.float_value.precision = IoTConnect_float_2_decimal_place
    };

    (void)send_data_to_cloud(&attr_data, 1, "rotation");
}

static void get_env_sensor_reading_work_fn(struct k_work *work)
{
    if ((m_do_reboot == true) || (m_do_switch_to_command_mode == true)) {
        return;
    }
    
    env_sensors_data_t sdata;
    char temp[100];

    env_sensors_get_data(&sdata);

    snprintf(temp, sizeof(temp), "Pressure: %.02f hPa, Temperature: %.02f "
                "Celcius, Humidity: %.02f %%RH, Light: %d Lux", 
                sdata.pressure, sdata.temperature, 
                sdata.humidity, sdata.illumination);
    LOG_INF("%s", temp);
    send_env_sensor_data(&sdata);

    int err = k_delayed_work_submit_to_queue(&application_work_q, 
                        &get_env_sensor_reading_work, 
                        K_SECONDS(CONFIG_DEMO_ENVIRONMENT_DATA_SEND_INTERVAL));
    if (err != 0) {
        LOG_ERR("k_delayed_work_submit_to_queue(env_sensor_read) "
                        "failed (err: %d)", err);
    }
}

static void check_location_work_fn(struct k_work *work)
{
    static nrf_gnss_pvt_data_frame_t old_pvt = {0};
    nrf_gnss_pvt_data_frame_t new_pvt;
    int next_check = 5;
    int err;

    if (m_is_location_tracking_started == false) {
        return;
    }

    if (gps_acquire_latest_pvt_data(&new_pvt) == false) {
        goto resched;
    }

    if (memcmp(&new_pvt, &old_pvt, sizeof(old_pvt)) == 0) {
        goto resched;
    }

    char temp[100];
    snprintf(temp, sizeof(temp), 
                "Latitude:%.05f, Longitude:%.05f, Altitude:%.03f", 
                new_pvt.latitude, new_pvt.longitude, new_pvt.altitude);
    LOG_INF("%s", temp);

    if ((m_cloud_conn_state != CLOUD_STATE_CONNECTED) || 
        (m_is_ongoing_ota == true)) {
        goto resched;
    }

    IoTConnect_attr_t attr_data[] = {
        {
            .name = "GPS_Latitude",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_float,
            .v.float_value.value = new_pvt.latitude,
            .v.float_value.precision = IoTConnect_float_5_decimal_place,
        },
        {
            .name = "GPS_Longitude",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_float,
            .v.float_value.value = new_pvt.longitude,
            .v.float_value.precision = IoTConnect_float_5_decimal_place,
        },
        {
            .name = "GPS_Altitude",
            .p_parent_name = NULL,
            .data_type = IoTConnect_attr_data_type_float,
            .v.float_value.value = new_pvt.altitude,
            .v.float_value.precision = IoTConnect_float_3_decimal_place,
        },
    };

    (void)send_data_to_cloud(attr_data, ARRAY_SIZE(attr_data), "location");
    memcpy(&old_pvt, &new_pvt, sizeof(old_pvt));
    next_check = 30;

resched:
    err = k_delayed_work_submit_to_queue(&application_work_q, 
                            &check_location_work, K_SECONDS(next_check));
    if (err != 0) {
        LOG_ERR("k_delayed_work_submit_to_queue(check_location) "
                        "failed (err: %d)", err);
    }
}

static void print_lte_lc_nw_reg_status(enum lte_lc_nw_reg_status status) {
    switch (status) {
        case LTE_LC_NW_REG_NOT_REGISTERED:
            LOG_INF("nw_reg_status NOT_REGISTERED");
            break;
        case LTE_LC_NW_REG_REGISTERED_HOME:
            LOG_INF("nw_reg_status REGISTERED_HOME");
            break;
        case LTE_LC_NW_REG_SEARCHING:
            LOG_INF("nw_reg_status SEARCHING");
            break;
        case LTE_LC_NW_REG_REGISTRATION_DENIED:
            LOG_ERR("nw_reg_status REGISTRATION_DENIED");
            break;
        case LTE_LC_NW_REG_UNKNOWN:
            LOG_WRN("nw_reg_status UNKNOWN");
            break;
        case LTE_LC_NW_REG_REGISTERED_ROAMING:
            LOG_INF("nw_reg_status REGISTERED_ROAMING");
            break;
        case LTE_LC_NW_REG_REGISTERED_EMERGENCY:
            LOG_WRN("nw_reg_status REGISTERED_EMERGENCY");
            break;
        case LTE_LC_NW_REG_UICC_FAIL:
            LOG_ERR("nw_reg_status UICC_FAIL");
            break;
        default:
            LOG_INF("nw_reg_status %d", status);
    }
}

static char rrc_mode_str_list[2][10] = {"IDLE", "CONNECTED"};
static void lte_lc_evt_handler(const struct lte_lc_evt *const evt) {
    if (evt->type == LTE_LC_EVT_NW_REG_STATUS) {
        print_lte_lc_nw_reg_status(evt->nw_reg_status);
        if (evt->nw_reg_status == LTE_LC_NW_REG_NOT_REGISTERED) {
            m_do_read_iccid = true;
            m_do_read_mobile_network_info = true;
        }
        m_lte_lc_nw_reg_status = evt->nw_reg_status;
    } else if (evt->type == LTE_LC_EVT_EDRX_UPDATE) {
        LOG_DBG("eDRX value: %d.%02d s, PTW: %d.%02d s",
                (int)evt->edrx_cfg.edrx,
                (int)(100 * (evt->edrx_cfg.edrx - (int)evt->edrx_cfg.edrx)),
                (int)evt->edrx_cfg.ptw,
                (int)(100 * (evt->edrx_cfg.ptw - (int)evt->edrx_cfg.ptw)));
    } else if (evt->type == LTE_LC_EVT_RRC_UPDATE) {
        LOG_INF("rrc %s", rrc_mode_str_list[evt->rrc_mode]);
    }
}

static void start_demo_proper(void)
{
    int err;
    IoTConnect_dev_status_t dev_status;
#if !defined (CONFIG_IOTCONNECT_DEVICE_UNIQUE_ID)
    strcpy(m_iotconnect_init.dev_unique_id, m_imei);
#endif

    m_cloud_conn_state = CLOUD_STATE_IOTCONNECT_SDK_UNINT;
    LOG_INF("Inside Demo proper");

    err = fota_download_init(fota_dl_handler);
    if (err != 0) {
        LOG_ERR("fota_download_init() failed %d", err);
        while (1); //TODO
    }

    LOG_INF("Connecting to LTE network...");
    err = lte_lc_init();
    if (err != 0) {
        LOG_ERR("lte_lc_init() failed %d", err);
        while (1); //TODO
    }

    uint8_t mode;
    lte_mode_nvs_read(&mode);
    if (mode == 1) {
        m_system_mode = LTE_LC_SYSTEM_MODE_NBIOT_GPS;
    } else {
        m_system_mode = LTE_LC_SYSTEM_MODE_LTEM_GPS;
    }

    m_lte_lc_nw_reg_status = LTE_LC_NW_REG_NOT_REGISTERED;
#define LTE_LC_CONNECT_WORKAROUND_EN    1
#if defined (LTE_LC_CONNECT_WORKAROUND_EN) && (LTE_LC_CONNECT_WORKAROUND_EN > 0)
    /* our intention is to set the preferred system mode through 
     * lte_lc_system_mode_set() and register to network by lte_lc_normal()
     * but there is this problem of lte_lc::at_handler() calling k_sem_give() 
     * once registered to network that is initialized by 
     * lte_lc::w_lte_lc_connect() for its blocking implementaion
     * to workaround this problem, we have to call lte_lc_connect_async() 
     * followed by lte_lc_offline()
    */
    err = lte_lc_connect_async(lte_lc_evt_handler);
    if (err != 0) {
        LOG_ERR("lte_lc_connect_async() failed %d", err);
        while (1); //TODO
    }
    err = lte_lc_offline();
    if (err != 0) {
        LOG_ERR("lte_lc_offline() failed %d", err);
        while (1); //TODO
    }
#endif

    err = lte_lc_system_mode_set(m_system_mode);
    if (err != 0) {
        LOG_ERR("lte_lc_system_mode_set() failed %d", err);
        while (1); //TODO
    }
    else {
        LOG_DBG("LTE_LC system mode %d", m_system_mode);
    }
    err = lte_lc_edrx_req(true);
    if (err != 0) {
        LOG_ERR("lte_lc_edrx_req() failed %d", err);
        while (1); //TODO
    }

#if !defined (LTE_LC_CONNECT_WORKAROUND_EN) || \
                (LTE_LC_CONNECT_WORKAROUND_EN == 0)
    lte_lc_register_handler(lte_lc_evt_handler);
#endif
    err = lte_lc_normal();
    if (err != 0) {
        LOG_ERR("lte_lc_normal() failed %d", err);
        while (1); //TODO
    }

    m_current_orientation = ORIENTATION_NOT_KNOWN;
    err = orientation_tap_detector_init_and_start(&application_work_q, 
                                            orientation_tap_event_handler);
    if (err != 0) {
        LOG_ERR("orientation_tap_detector_init_and_start() failed %d", err);
    } else {
        LOG_INF("Orientation and tap detection started");
    }

    int64_t network_connect_start_time = k_uptime_get();
    int64_t check_time;
    int64_t print_time = network_connect_start_time;

    while (m_do_switch_to_command_mode == false) {

        /* to get updates from lte_lc_evt_handler() */
        k_sleep(SYS_TIMEOUT_MS(500));

        if ((m_lte_lc_nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) || 
            (m_lte_lc_nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING) ||
            (m_lte_lc_nw_reg_status == LTE_LC_NW_REG_REGISTERED_EMERGENCY)) {
            if (((mode == 0) && 
                    (m_system_mode != LTE_LC_SYSTEM_MODE_LTEM_GPS)) || 
                ((mode == 1) && 
                    (m_system_mode != LTE_LC_SYSTEM_MODE_NBIOT_GPS))) {
                lte_mode_nvs_write((m_system_mode == 
                                    LTE_LC_SYSTEM_MODE_LTEM_GPS) ? 0 : 1);
            }
            break; // from loop
        }

        /*
         * try the other lte mode if fail to attach with current mode 
         * after some time
         */
        check_time = network_connect_start_time;
        if (k_uptime_delta(&check_time) >= 
                            (CONFIG_LTE_NETWORK_TIMEOUT * MSEC_PER_SEC)) {
            err = lte_lc_offline();
            if (err != 0) {
                LOG_ERR("lte_lc_offline() failed %d", err);
                while (1); //TODO
            }
            if (m_system_mode == LTE_LC_SYSTEM_MODE_LTEM_GPS) {
                m_system_mode = LTE_LC_SYSTEM_MODE_NBIOT_GPS;
                LOG_INF("Still not connected to LTEM network after "
                                "%d seconds, switching to NBIOT", 
                                CONFIG_LTE_NETWORK_TIMEOUT);
            } else {
                m_system_mode = LTE_LC_SYSTEM_MODE_LTEM_GPS;
                LOG_INF("Still not connected to NBIOT network after "
                                "%d seconds, switching to LTEM", 
                                CONFIG_LTE_NETWORK_TIMEOUT);
            }
            err = lte_lc_system_mode_set(m_system_mode);
            if (err != 0) {
                LOG_ERR("lte_lc_system_mode_set() failed %d", err);
                while (1); //TODO
            }
            else {
                LOG_DBG("LTE_LC system mode %d", m_system_mode);
            }
            err = lte_lc_normal();
            if (err != 0) {
                LOG_ERR("lte_lc_normal() failed %d", err);
                while (1); //TODO
            }
            network_connect_start_time = k_uptime_get();
        } else if (m_lte_lc_nw_reg_status == LTE_LC_NW_REG_SEARCHING) {
            check_time = print_time;
            if (k_uptime_delta(&check_time) >= (30 * MSEC_PER_SEC)) {
                LOG_INF("searching for network...");
                print_time = check_time;
            }
        }
    }

    if (m_do_switch_to_command_mode == true) {
        goto switch_to_command_mode;
    }
    LOG_INF("LTE network connected (%s)", 
            (m_system_mode == LTE_LC_SYSTEM_MODE_LTEM_GPS) ? 
                "LTEM" : "NBIOT");

    err = rotation_detector_init_and_start(&application_work_q, 
                                                rotation_event_handler);
    if (err != 0) {
        LOG_ERR("rotation_detector_init_and_start() failed %d", err);
    } else {
        LOG_INF("Rotation detection on Z-axis started");
    }

    if (env_sensors_init() == false) {
        LOG_ERR("sensors_init() failed!");
    } else {
        env_sensors_start();
        LOG_INF("env sensors polling start");
        err = k_delayed_work_submit_to_queue(&application_work_q, 
                        &get_env_sensor_reading_work, 
                        K_SECONDS(CONFIG_DEMO_ENVIRONMENT_DATA_SEND_INTERVAL));
        if (err != 0) {
            LOG_ERR("k_delayed_work_submit_to_queue(env_sensor_read) failed "
                    "(err: %d)", err);
        }
    }

    if (gps_init() == false) {
        LOG_ERR("gps_init() failed!");
    }

    while (1) {
        if ((m_do_reboot == true) || (m_do_switch_to_command_mode == true)) {
            k_delayed_work_cancel(&get_env_sensor_reading_work);
            orientation_tap_detector_stop_and_unint();
            rotation_detector_stop_and_unint();
            pir_detector_stop_and_unit();
            if (m_is_location_tracking_started == true) {
                m_is_location_tracking_started = false;
                k_delayed_work_cancel(&check_location_work);
                gps_stop();
            }
            if (env_sensors_is_running()) {
                env_sensors_stop();
            }
            env_sensors_uninit();
            while ((m_ota_ack_msg_id > 0) && 
                    (m_cloud_conn_state == CLOUD_STATE_CONNECTED)) {
                if (IoTConnect_poll(CONFIG_IOTCONNECT_POLL_TIMEOUT_MS) == 
                                    -ECONNRESET) {
                    m_cloud_conn_state = CLOUD_STATE_DISCONNECTED;
                }
            }
            if (IoTConnect_disconnect() == 0) {
                while (m_cloud_conn_state == CLOUD_STATE_CONNECTED) {
                    if (IoTConnect_poll(CONFIG_IOTCONNECT_POLL_TIMEOUT_MS) == 
                                        -ECONNRESET) {
                        m_cloud_conn_state = CLOUD_STATE_DISCONNECTED;
                    }
                }
            }
            lte_lc_power_off();
            lte_lc_register_handler(NULL);
            if (m_do_reboot == true) {
                sys_reboot(SYS_REBOOT_COLD);
            } else /* (m_do_switch_to_command_mode == true) */{
                IoTConnect_uninit();
                m_cloud_conn_state = CLOUD_STATE_IOTCONNECT_SDK_UNINT;
                start_command_mode();
                break; /* from while(1) */
            }
        }

        if (m_cloud_conn_state == CLOUD_STATE_IOTCONNECT_SDK_UNINT) {
            if ((m_lte_lc_nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) || 
                (m_lte_lc_nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
                LOG_INF("Initializing IoTConnect SDK...");
                err = IoTConnect_init(&m_iotconnect_init, &dev_status);
                if (err != 0 ) {
                    if (err == -EACCES) {
                        LOG_ERR("IoTConnect_init() failed! Device status: "
                                "(%d) ", dev_status);

                        switch (dev_status) {
                            case IoTConnect_dev_not_register:
                                LOG_ERR("Not register");
                                break;
                            case IoTConnect_dev_auto_register:
                                LOG_ERR("Auto register");
                                break;
                            case IoTConnect_dev_not_found:
                                LOG_ERR("Not found");
                                break;
                            case IoTConnect_dev_inactive:
                                LOG_ERR("Inactive");
                                break;
                            case IoTConnect_dev_moved:
                                LOG_ERR("Moved");
                                break;
                            case IoTConnect_dev_cpid_not_found:
                                LOG_ERR("CPID not found");
                                break;
                            case IoTConnect_dev_status_unknown:
                                LOG_ERR("Unknown");
                                break;
                            default:
                                ;
                        }
                    } else {
                        LOG_ERR("IoTConnect_init() failed! %d", err);
                    }
                } else {
                    m_cloud_conn_state = CLOUD_STATE_DISCONNECTED;
                    LOG_INF("IoTConnect SDK init done");
                }
            }
        }

        if (m_cloud_conn_state > CLOUD_STATE_IOTCONNECT_SDK_UNINT) {
            err = IoTConnect_poll(CONFIG_IOTCONNECT_POLL_TIMEOUT_MS);
            if (err != 0) {
                if ((m_lte_lc_nw_reg_status == 
                                        LTE_LC_NW_REG_REGISTERED_HOME) || 
                    (m_lte_lc_nw_reg_status == 
                                        LTE_LC_NW_REG_REGISTERED_ROAMING)) {
                    LOG_ERR("IoTConnect_poll() failed %d", err);
                }
                if (err == -ECONNRESET) {
                    m_cloud_conn_state = CLOUD_STATE_DISCONNECTED;
                    LOG_INF("IOTCONNECT_DISCONNECTED reason ECONNRESET");
                }
            }
        }

        if (m_cloud_conn_state == CLOUD_STATE_DISCONNECTED) {
            if ((m_lte_lc_nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) || 
                (m_lte_lc_nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
                err = IoTConnect_connect();
                if (err != 0) {
                    LOG_ERR("IoTConnect_connect() failed %d", err);
                } else {
                    m_cloud_conn_state = CLOUD_STATE_CONNECTING;
                }
            }
        }

        if ((m_lte_lc_nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) && 
            (m_lte_lc_nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            /* to get updates from lte_lc_evt_handler() */
            k_sleep(SYS_TIMEOUT_MS(500));
        }
    }
    return;

switch_to_command_mode:
    orientation_tap_detector_stop_and_unint();
    lte_lc_power_off();
    lte_lc_register_handler(NULL);
    start_command_mode();
}

static void exit_cmd_mode_work_fn(struct k_work *work)
{
    slm_at_host_uninit();
    at_params_list_free(&at_param_list);
    sys_reboot(SYS_REBOOT_COLD); /* ==> to start_demo_proper() clean */
}

bool do_exit_slm_at_host(void)
{
    if (has_needed_certificates() == false) {
        return false;
    }

    k_work_submit_to_queue(&application_work_q, &exit_cmd_mode_work);
    return true;
}

static void start_command_mode(void)
{
    int err;

    LOG_INF("Inside Command Mode");
    m_do_switch_to_command_mode = false;

    /* Initialize AT Parser */
    err = at_params_list_init(&at_param_list, CONFIG_DEMO_AT_MAX_PARAM);
    if (err) {
        LOG_ERR("at_params_list_init() failed %d", err);
        while (1); //TODO
    }

    err = slm_at_host_init(lte_mode_nvs_read, lte_mode_nvs_write);
    if (err) {
        LOG_ERR("slm_at_host_init() failed %d", err);
        while (1); //TODO
    }
}

static void handle_bsdlib_init_ret(void)
{
    int ret = bsdlib_get_init_ret();

    /* Handle return values relating to modem firmware update */
    switch (ret) {
    case MODEM_DFU_RESULT_OK:
        LOG_INF("MODEM UPDATE OK. Will run new firmware");
        sys_reboot(SYS_REBOOT_COLD);
        break;
    case MODEM_DFU_RESULT_UUID_ERROR:
    case MODEM_DFU_RESULT_AUTH_ERROR:
        LOG_ERR("MODEM UPDATE ERROR %d. Will run old firmware", ret);
        sys_reboot(SYS_REBOOT_COLD);
        break;
    case MODEM_DFU_RESULT_HARDWARE_ERROR:
    case MODEM_DFU_RESULT_INTERNAL_ERROR:
        LOG_ERR("MODEM UPDATE FATAL ERROR %d. Modem failiure", ret);
        sys_reboot(SYS_REBOOT_COLD);
        break;
    default:
        break;
    }
}

void lte_mode_nvs_read(uint8_t *mode)
{
    ssize_t  bytes;
    uint8_t lte_mode;

    bytes = nvs_read(&m_fs, LTE_MODE_NVS_ID, &lte_mode, sizeof(lte_mode));
    if (bytes > 0) {
        *mode = lte_mode;
        LOG_INF("NVS LTE Mode: %d (%s)", *mode, 
                    (*mode == 0) ? "LTEM" : "NBIOT");
    } else {
        *mode = 0;
        LOG_INF("NVS LTE Mode: not found, default to %d", *mode);
    }
}

int lte_mode_nvs_write(uint8_t mode)
{
    ssize_t  bytes;

    bytes = nvs_write(&m_fs, LTE_MODE_NVS_ID, &mode, sizeof(mode));
    if (bytes != sizeof(mode)) {
        LOG_INF("nvs_write() failed");
        return -EIO;
    }

    return 0;
}


void main(void)
{
    int err;

#if 0
    /* using __TIME__ ensure that a new binary will be built on every
     * compile which is convient when testing firmware upgrade.
     */
    LOG_DBG("build time: " __DATE__ " " __TIME__);
#endif

    LOG_INF("NCS version %s", NCS_VERSION_STRING);
    LOG_INF("Demo version %s", CONFIG_MCUBOOT_IMAGE_VERSION);

#if 0
    const struct fw_info *app_fw_info = fw_info_find(PM_APP_ADDRESS);
    if (app_fw_info != NULL) {
        LOG_INF("FW_INFO_FIRMWARE_VERSION %d", app_fw_info->version);
    }
#endif

    handle_bsdlib_init_ret();

    err = at_cmd_write("AT+CGMR", m_mfw_ver, sizeof(m_mfw_ver), NULL);
    if (err) {
        LOG_ERR("Could not get modem firmware version, error: %d", err);
    } else {
        char *p = strstr(m_mfw_ver, "\r");
        if (p != NULL) {
            *p = '\0';
        }
        LOG_INF("AT+CGMR ==> %s", m_mfw_ver);
    }

    err = at_cmd_write("AT+CGSN", m_imei, sizeof(m_imei), NULL);
    if (err) {
        LOG_ERR("Could not get imei, error: %d", err);
    } else {
        char *p = strstr(m_imei, "\r");
        if (p != NULL) {
            *p = '\0';
        }
        LOG_INF("AT+CGSN ==> %s", m_imei);
    }

    /* initialize nvs and check if LTE_MODE_NVS_ID exists */
    struct flash_pages_info info;
    uint8_t lte_mode;
    m_fs.offset = FLASH_AREA_OFFSET(nvs_storage);
    err = flash_get_page_info_by_offs(
                    device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL),
                    m_fs.offset, &info);
    if (err) {
        LOG_ERR("flash_get_page_info_by_offs() failed %d", err);
    }
    m_fs.sector_size = info.size; /* sector_size equal to the pagesize */
    m_fs.sector_count = 3U;

    err = nvs_init(&m_fs, DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);
    if (err) {
        LOG_ERR("nvs_init() failed %d", err);
    }
    lte_mode_nvs_read(&lte_mode);

    k_work_q_start(&application_work_q, application_stack_area,
                   K_THREAD_STACK_SIZEOF(application_stack_area),
                   APPLICATION_WORKQUEUE_PRIORITY);
    k_work_init(&exit_cmd_mode_work, exit_cmd_mode_work_fn);
    k_delayed_work_init(&check_location_work, check_location_work_fn);
    k_delayed_work_init(&get_env_sensor_reading_work, 
                            get_env_sensor_reading_work_fn);

#if defined (ENABLE_BLE_BEACON)
    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err != 0) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
    }
#endif

    /*TODO: is this the right place to confirm the new image? or 
     *      should make sure is able to connect to the cloud first?
     */
    boot_write_img_confirmed();

    if (has_needed_certificates() == false) {
        start_command_mode();
    } else {
        start_demo_proper();
    }
}

