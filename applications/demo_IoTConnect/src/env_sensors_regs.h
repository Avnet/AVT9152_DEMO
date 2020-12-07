
#ifndef ENV_SENSORS_REGS_H
#define ENV_SENSORS_REGS_H


//U4 LPS22HBTR 24bit-Pressure, 16bit-Temperature Sensor (7bit address 0x5D) WHO_AM_I = 0xB1
#define LPS22HB_7BIT_ADDR                   0x5D
#define LPS22HB_REG_WHO_AM_I                0x0F
#define LPS22HB_REG_CTRL1                   0x10
#define LPS22HB_REG_CTRL2                   0x11
#define LPS22HB_REG_CTRL3                   0x12
#define LPS22HB_REG_STATUS                  0x27
#define LPS22HB_REG_PRESS_OUT_XL            0x28
#define LPS22HB_REG_PRESS_OUT_L             0x29
#define LPS22HB_REG_PRESS_OUT_H             0x2A
#define LPS22HB_REG_TEMP_OUT_L              0x2B
#define LPS22HB_REG_TEMP_OUT_H              0x2C

#define LPS22HB_WHO_AM_I                    0xB1

#define LPS22HB_REG_CTRL2_ONE_SHOT          BIT(0)
#define LPS22HB_REG_CTRL2_SWRESET           BIT(2)
#define LPS22HB_REG_CTRL2_ADD_INC_EN        BIT(4) // Register address automatically incremented during a multiple byte access with a serial interface (I2C or SPI). Default value: 1 (0: disable; 1 enable)
#define LPS22HB_REG_CTRL3_INT_ACTIVE_LOW    BIT(7) // Interrupt active-high/low. Default value: 0 (0: active high; 1: active low)
#define LPS22HB_REG_CTRL3_DRDY_EN           BIT(2) // Data-ready signal on INT_DRDY pin. Default value: 0 (0: disable; 1: enable)

// U5 2314277-1 Ambimate (Humidity, Light, Motion, Temperature) Sensor (7bit address: 0x2A) Optional Sensors = 0x00
#define TE_2314277_1_7BIT_ADDR              0x2A

#define TE_2314277_1_REG_STATUS             0x00
#define TE_2314277_1_REG_TEMPERATURE_H      0x01
#define TE_2314277_1_REG_TEMPERATURE_L      0x02
#define TE_2314277_1_REG_HUMIDITY_H         0x03
#define TE_2314277_1_REG_HUMIDITY_L         0x04
#define TE_2314277_1_REG_LIGHT_H            0x05
#define TE_2314277_1_REG_LIGHT_LOW          0x06
#define TE_2314277_1_REG_AUDIO_H            0x07
#define TE_2314277_1_REG_AUDIO_L            0x08
#define TE_2314277_1_REG_BATTERY_VOLTS_H    0x09
#define TE_2314277_1_REG_BATTERY_VOLTS_L    0x0A

#define TE_2314277_1_REG_FIRMWARE_VERSION     0x80
#define TE_2314277_1_REG_FIRMWARE_SUBVERSION  0x81
#define TE_2314277_1_REG_OPTIONAL_SENSORS     0x82

#define TE_2314277_1_REG_SCAN_START_BYTE      0xC0
#define TE_2314277_1_REG_RESET                0xF0

#define TE_2314277_1_RESET                    0xA5

#define TE_2314277_1_OPTIONAL_SENSORS_NONE    0x00
#define TE_2314277_1_OPTIONAL_CO2_BITMASK     BIT(0)
#define TE_2314277_1_OPTIONAL_MIC_BITMASK     BIT(2)

#define TE_2314277_1_REG_SCAN_STATUS_BITMASK    BIT(0)
#define TE_2314277_1_REG_SCAN_TEMP_BITMASK      BIT(1)
#define TE_2314277_1_REG_SCAN_HUMIDITY_BITMASK  BIT(2)
#define TE_2314277_1_REG_SCAN_LIGHT_BITMASK     BIT(3)
#define TE_2314277_1_REG_SCAN_AUDIO_BITMASK     BIT(4)
#define TE_2314277_1_REG_SCAN_BATT_BITMASK      BIT(5)
#define TE_2314277_1_REG_SCAN_GAS_BITMASK       BIT(6)

#define TE_2314277_1_REG_STATUS_MOTION_EVT_BITMASK  BIT(0)
#define TE_2314277_1_REG_STATUS_AUDIO_EVT_BITMASK   BIT(1)
#define TE_2314277_1_REG_STATUS_PIR_EVT_BITMASK     BIT(7)


#endif // ENV_SENSORS_REGS_H
