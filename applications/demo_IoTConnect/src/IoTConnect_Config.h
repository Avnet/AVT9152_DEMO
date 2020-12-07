#ifndef IOTCONNECT_CONFIG_H
#define IOTCONNECT_CONFIG_H

#define CONFIG_IOTCONNECT_DISCOVERY_SEC_TAG         623
#define CONFIG_IOTCONNECT_MQTT_SEC_TAG              624

#define CONFIG_IOTCONNECT_POLL_TIMEOUT_MS           500

#define CONFIG_IOTCONNECT_SYSTEM_TIME_UPDATE        (60 * 60)  // in seconds

/* IoTConnect credential configuration */

/* comment out CONFIG_IOTCONNECT_DEVICE_UNIQUE_ID to use device IMEI as its unique id */
#if 0
#define CONFIG_IOTCONNECT_DEVICE_UNIQUE_ID          "avt9152testdev0001"
#endif
#define CONFIG_IOTCONNECT_DEVICE_CP_ID              "149847e8f3ab4f11a90878bfa7da9922"
#define CONFIG_IOTCONNECT_DEVICE_ENV                "avnet"


#endif /* IOTCONNECT_CONFIG_H */



