/*
 * Copyright (c) 2020 Avnet Asia Pte. Ltd.
 *
 * SPDX-License-Identifier: LicenseRef-BSD-3-Clause
 */

#ifndef IOTCONNECT_H
#define IOTCONNECT_H

/**@file
 *
 * @brief   IoTConnect SDK module.
 *
 * Provides API to connect and send/receive data to/from IoTConnect over secured MQTT.
 * Implemented with reference to
 * 1) https://help.iotconnect.io/documentation/sdk-reference/device-sdks-flavors/build-your-own-iotconnect-sdk/
 * 2) https://help.iotconnect.io/documentation/sdk-reference/device-sdks-flavors/download-c-lang-sdk/
 *
 */


#define IOTCONNECT_MESSAGE_BUFFER_SIZE     2048
#define IOTCONNECT_PAYLOAD_BUFFER_SIZE     2048


#define IOTCONNECT_SDK_UNIQUE_ID_MAX_LEN    32
#define IOTCONNECT_SDK_CPID_MAX_LEN         33
#define IOTCONNECT_SDK_ENV_MAX_LEN          20
#define IOTCONNECT_SDK_PROTOCOL_MAX_LEN     10
#define IOTCONNECT_SDK_BASE_URL_MAX_LEN     100
#define IOTCONNECT_SDK_HOST_MAX_LEN         100
#define IOTCONNECT_SDK_USER_NAME_MAX_LEN    200
#define IOTCONNECT_SDK_PASSWORD_MAX_LEN     20
#define IOTCONNECT_SDK_TOPIC_MAX_LEN        100
#define IOTCONNECT_SDK_GUID_MAX_LEN         40

#define IOTCONNECT_SDK_DEV_ATTR_NAME_MAX_LEN        20
#define IOTCONNECT_SDK_DEV_ATTR_STR_VALUE_MAX_LEN   20

#define IOTCONNECT_SDK_DEV_TWIN_PROP_NAME_MAX_LEN        20
#define IOTCONNECT_SDK_DEV_TWIN_PROP_STR_VALUE_MAX_LEN   30

/** @brief Asynchronous events notified by the module. */
#define IOTCONNECT_EVT_SDK                  0   /* From MQTT events except MQTT_EVT_PUBLISH */
#define IOTCONNECT_EVT_DEV_C2D_DATA         1   /* Has devicebound data from IoTConnect */
#define IOTCONNECT_EVT_TWIN_C2D_DATA        2   /* Has twin related data from IoTConnect */

/** @brief IOTCONNECT_EVT_SDK events notified by the module. */
#define IOTCONNECT_SDK_EVT_NONE             0
#define IOTCONNECT_SDK_EVT_CONNECTED        1   /* Connected to IoTConnect */
#define IOTCONNECT_SDK_EVT_DISCONNECTED     2   /* Disconnected from IoTConnect */
#define IOTCONNECT_SDK_EVT_DATA_SEND_OK     3   /* Receive positve ack for data sent with QoS 1. */
#define IOTCONNECT_SDK_EVT_DATA_SEND_ERR    4   /* Receive MQTT_EVT_PUBACK with non-zero result. */
#define IOTCONNECT_SDK_EVT_CONNECT_FAIL     5   /* Receive MQTT_EVT_CONNACK with non-zero result */
#define IOTCONNECT_SDK_EVT_DISCONNECT_FAIL  6   /* Receive MQTT_EVT_DISCONNECT with non-zero result */
#define IOTCONNECT_SDK_EVT_TOPICS_SUB_ERR   7   /* Receive MQTT_EVT_SUBACK with non-zero result */

/** @brief Prossible response code from IoTConnect_init(). */
typedef enum {
    IoTConnect_dev_status_unknown,
    IoTConnect_dev_OK,
    IoTConnect_dev_not_register,
    IoTConnect_dev_auto_register,
    IoTConnect_dev_not_found,
    IoTConnect_dev_inactive,
    IoTConnect_dev_moved,
    IoTConnect_dev_cpid_not_found
} IoTConnect_dev_status_t;

/** @brief Types of C2D commands */
typedef enum {
    IoTConnect_cmd_type_device = 0x01,    /* Device Command */
    IoTConnect_cmd_type_ota = 0x02,       /* Firmware update command */
    IoTConnect_cmd_type_module = 0x03,    /* Module update command */
    IoTConnect_cmd_type_template_attr_changed = 0x10,     /* On change of attribute in portal; platform will send this command to re-sync attribute */
    IoTConnect_cmd_type_template_setting_changed = 0x11,  /* On change of setting or twin in portal; platform will send this command to re-sync setting */
    IoTConnect_cmd_type_password_changed = 0x12,  /* Force device to change it password by calling sync with protocol set to true */
    IoTConnect_cmd_type_device_changed = 0x13,    /* On add or remove of child device in portal; platform will send this command to re-sync device */
    IoTConnect_cmd_type_rule_changed = 0x15,      /* On add or remove of rule in portal; platform will send this command to re-sync rule [For Edge only] */
    IoTConnect_cmd_type_hard_stop = 0x99,         /* Force device to stop calling sync and discovery and close open MQTT connection */
} IoTConnect_c2d_cmd_type_t;

/** @brief Types of C2D Device Command Acknowledgement */
typedef enum {
    IoTConnect_msg_type_device_ack = 5, /* command acknowledgement message type for Device Command */
    IoTConnect_msg_type_ota_ack = 11    /* command acknowledgement message type for Firmware update command */
} IoTConnect_d2c_msg_type_t;

/** @brief Status Code values for  C2D Device Command and Firmware update command */
typedef enum {
    IoTConnect_ack_status_device_ok = 6,    /* Device Command failed with some reason */
    IoTConnect_ack_status_device_fail = 4,  /* Device Command executed successfully */
    IoTConnect_ack_status_ota_ok = 7,       /* Firmware update command failed with some reason */
    IoTConnect_ack_status_ota_fail = 4,     /* Firmware update command executed successfully */
} IoTConnect_ack_status_t;

/** @brief Action the module will perform upon return from the application event handler for IOTCONNECT_EVT_DEV_C2D_DATA */
typedef enum {
    IoTConnect_ack_act_no_ack,      /* Ack not expected or do not send an ack */
    IoTConnect_ack_act_ack_ok,      /* Success status code */
    IoTConnect_ack_act_ack_fail,    /* Fail status code */
} IoTConnect_ack_action_t;

/** @brief Types of attribute key-val pair accepted by IoTConnect_device_send_xxx() to be published */
typedef enum {
    IoTConnect_attr_data_type_object,
    IoTConnect_attr_data_type_string,
    IoTConnect_attr_data_type_integer,
    IoTConnect_attr_data_type_signed_integer,
    IoTConnect_attr_data_type_float
} IoTConnect_attr_data_t;

/** @brief Types of TWIN property key-val pair accepted by IoTConnect_device_send_xxx() to be published */
typedef enum {
    IoTConnect_twin_prop_data_type_string,
    IoTConnect_twin_prop_data_type_integer,
    IoTConnect_twin_prop_data_type_float
} IoTConnect_twin_prop_data_t;

/** @brief Number of decimal places when converting float to string */
typedef enum {
    IoTConnect_float_1_decimal_place = 1,
    IoTConnect_float_2_decimal_place,
    IoTConnect_float_3_decimal_place,
    IoTConnect_float_4_decimal_place,
    IoTConnect_float_5_decimal_place
} IoTConnect_float_precision_t;

/** @brief Parameters for a float-type attribute */
typedef struct {
    float value;
    IoTConnect_float_precision_t precision;
} IoTConnect_float_value_t;

/** @brief Parameters for an attribute key-val pair to encode and publish using IoTConnect_device_send_xxx() */
typedef struct {
    /** Key name*/
    char name[IOTCONNECT_SDK_DEV_ATTR_NAME_MAX_LEN];
    /** Name of the attribute this attribute belongs to */
    char *p_parent_name;
    /** Data type  */
    IoTConnect_attr_data_t data_type;
    /** Value data for non-IoTConnect_attr_data_type_object attribute*/
    union {
        /** for string-typed data */
        char str_value[IOTCONNECT_SDK_DEV_ATTR_STR_VALUE_MAX_LEN];
        /** for integer-typed data */
        uint32_t int_value;
        /** for float-typed data */
        IoTConnect_float_value_t float_value;
    }v;
} IoTConnect_attr_t;

/** @brief Parameters for a TWIN property key-val pair to encode and publish using IoTConnect_device_send_xxx() */
typedef struct {
    /** Key name */
    char name[IOTCONNECT_SDK_DEV_TWIN_PROP_NAME_MAX_LEN];
    /** Data type */
    IoTConnect_twin_prop_data_t data_type;
    /** Value data for TWIN property. */
    union {
        /** for string-typed data */
        char str_value[IOTCONNECT_SDK_DEV_TWIN_PROP_STR_VALUE_MAX_LEN];
        /** for integer-typed data */
        uint32_t int_value;
        /** for float-typed data */
        IoTConnect_float_value_t float_value;
    }v;
} IoTConnect_twin_prop_t;

/** @brief Data for IOTCONNECT_EVT_SDK event. */
typedef struct {
    /** Type of sdk event. */
    uint8_t evt_id;
    struct {
        /** Result code from related MQTT event. */
        uint32_t err_code;
        /** Message id being ack'd..Applicable only to IOTCONNECT_SDK_EVT_DATA_SEND_xxx event */
        uint16_t msg_id;
    }data;
} IoTConnect_sdk_event_t;

/** @brief Data for Device Command. */
typedef struct {
    /** Command received. */
    const char *p_cmd;
} IoTConnect_c2d_dev_data_t;

/** @brief Data for Firmware Update Command. */
typedef struct {
    /** Software version string. */
    const char *p_sw_ver;
    /** Hardware version string. */
    const char *p_hw_ver;
    /** Number of firmware URLs in p_urls. */
    uint8_t url_count;
    /** List of firmware URLs. */
    const char **p_urls;
} IoTConnect_c2d_ota_data_t;

/** @brief Data for IOTCONNECT_EVT_DEV_C2D_DATA event. */
typedef struct {
    /** Type of command received. */
    IoTConnect_c2d_cmd_type_t cmd_type;
    /** Unique identifier of the device. */
    const char *p_guid;
    /** ackId expected when acknowledging this message. */
    const char *p_ack_id;
    /** [in] default to IoTConnect_ack_act_ack_ok if an ack is expected; 
             otherwise IoTConnect_ack_act_no_ack*/
    /** [out] value other than IoTConnect_ack_act_no_ack will trigger an IoTConnect_device_send_ack()
              upon return from the application event handler */
    IoTConnect_ack_action_t ack_act;
    union {
        /** Data associated with Device Command. */
        IoTConnect_c2d_dev_data_t dev;
        /** Data associated with Firmware Update Command. */
        IoTConnect_c2d_ota_data_t ota;
    }data;
} IoTConnect_dev_c2d_event_t;

/** @brief Data for TWIN property data. */
typedef struct {
    /** TWIN property key name. */
    char *p_key;
    /** TWIN property desired value */
    char *p_value;

} IoTConnect_c2d_twin_data_t;

/** @brief Data for IOTCONNECT_EVT_TWIN_C2D_DATA event. */
typedef struct {
    /** IoTConnect_c2d_twin_data_t array pointer pointing to multiple TWIN data pair. */
    IoTConnect_c2d_twin_data_t *p_twin_data;
    /** Number of C2D TWIN data. */
    uint8_t twin_data_count;

} IoTConnect_twin_c2d_event_t;

/**@brief Asynchronous events received from the module. */
typedef struct {
    /** The event that occurred. */
    uint8_t evt_type;
    /** Data associated with the module events. */
    union {
        /** Data associated with IOTCONNECT_EVT_SDK event. */
        IoTConnect_sdk_event_t sdk;
        /** Data associated with IOTCONNECT_EVT_DEV_C2D_DATA event. */
        IoTConnect_dev_c2d_event_t dev_data;
        /** Data associated with IOTCONNECT_EVT_TWIN_C2D_DATA event. */
        IoTConnect_twin_c2d_event_t twin_data;
    } evt;
} IoTConnect_event_t;

typedef void (*IOTConnectEventCallback)(IoTConnect_event_t *p_evt);

/**@brief Initialization parameters for the module. */
typedef struct {
    /** Device ID. */
    char dev_unique_id[IOTCONNECT_SDK_UNIQUE_ID_MAX_LEN];
    /** CPID assigned to your IotConnect account. */
    char dev_cpid[IOTCONNECT_SDK_CPID_MAX_LEN];
    /** ENV assigned to your IotConnect account. */
    char env[IOTCONNECT_SDK_ENV_MAX_LEN];
    /** Security tag where the certificate to validate discovery.iotconnect.io is stored. */
    int discovery_connection_sec_tag;
    /** Security tag where the certificates and key to create secure mqtt connection is stored. */
    int mqtt_connection_sec_tag;
    /** How often to synchronize the system time against an NTP server */
    uint32_t systime_sync_interval;
    /** Event handler that is registered with the module. */
    IOTConnectEventCallback evt_cb_func;
} IoTConnect_init_t;


/**
 * @brief API to get the credentials and configuration details from IotConnect 
 *        corresponding to the provided device information.
 *
 * @note  This API must be called once with return value of 0 before the rest of
 *        the APIs can be used.
 *
 * @param[in] p_init Initialization parameters.
 *
 * @param[out] p_dev_status Response code from IoTConnect Identity API response.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_init(IoTConnect_init_t *p_init, IoTConnect_dev_status_t *p_dev_status);

/**
 * @brief API to clear and release resources used by the module.
 *        This API can only be called when it is not connected to IoTConnect.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_uninit(void);

/**
 * @brief API to establish secure MQTT client connection to IotConnect as well 
 *        as subscribe to devicebound and twin topics.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_connect(void);

/**
 * @brief API to unsubscribe topics and terminate the connection 
 *
 * @retval 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_disconnect(void);

/**
 * @brief API to encode and publish device message to IoTConnect.
 *
 * @param[in] p_attr List of attribute key-val pairs to include in the message.
 *
 * @param[in] attr_count Number of attribute key-val pairs in p_attr.
 *
 * @param[out] p_msg_id Message id used for the publish message.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_device_send_data(IoTConnect_attr_t *p_attr, int attr_count, uint16_t *p_msg_id);

/**
 * @brief API to query all device TWIN property from IoTConnect.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_device_get_all_twin_data(void);

/**
 * @brief API to encode and publish device TWIN property to IoTConnect.
 *
 * @param[in] p_twin_prop List of TWIN property key-val pairs to include in the message.
 *
 * @param[in] prop_count_count Number of TWIN properties key-val pairs in p_twin_prop.
 *
 * @param[out] p_msg_id Message id used for the publish message.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_device_send_twin_data(IoTConnect_twin_prop_t *p_twin_prop, int prop_count, uint16_t *p_msg_id);

/**
 * @brief API to encode and publish an acknowledgement to a C2D message to IoTConnect.
 *
 * @param[in] msg_type Type of C2D message to acknowledge.
 *
 * @param[in] p_ack_id The ackId provided with the C2D message to be acknowledged.
 *
 * @param[in] status Ackknowledge status.
 *
 * @param[out] p_msg_id Message id used for the publish message.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_device_send_ack(IoTConnect_d2c_msg_type_t msg_type, const char *p_ack_id, IoTConnect_ack_status_t status, uint16_t *p_msg_id);

/**
 * @brief API to receive incoming MQTT packet if available as well as send Ping 
 *        request to keep the connection alive.
 *
 * @note  This API should be called periodically.
 *
 * @param[in] poll_max_timeout_ms Maximum number of milliseconds to poll for event.
 *
 * @return 0 If successful.
 *           Otherwise, a (negative) error code is returned.
 */
int IoTConnect_poll(uint32_t poll_max_timeout_ms);


#endif /* IOTCONNECT_H */
