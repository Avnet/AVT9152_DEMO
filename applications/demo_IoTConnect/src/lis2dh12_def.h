
#ifndef LIS2DH12_DEF_H_
#define LIS2DH12_DEF_H_


#define LIS2DH12_7BIT_ADDR                  0x19

#define LIS2DH12_REG_STATUS_REG_AUX         0x07
#define LIS2DH12_REG_OUT_TEMP_L             0x0C
#define LIS2DH12_REG_OUT_TEMP_H             0x0D
#define LIS2DH12_REG_WHO_AM_I               0x0F
#define LIS2DH12_REG_CTRL_REG0              0x1E
#define LIS2DH12_REG_TEMP_CFG_REG           0x1F
#define LIS2DH12_REG_CTRL_REG1              0x20
#define LIS2DH12_REG_CTRL_REG2              0x21
#define LIS2DH12_REG_CTRL_REG3              0x22
#define LIS2DH12_REG_CTRL_REG4              0x23
#define LIS2DH12_REG_CTRL_REG5              0x24
#define LIS2DH12_REG_CTRL_REG6              0x25
#define LIS2DH12_REG_REFERENCE              0x26
#define LIS2DH12_REG_STATUS                 0x27
#define LIS2DH12_REG_OUT_X_L                0x28
#define LIS2DH12_REG_OUT_X_H                0x29
#define LIS2DH12_REG_OUT_Y_L                0x2A
#define LIS2DH12_REG_OUT_Y_H                0x2B
#define LIS2DH12_REG_OUT_Z_L                0x2C
#define LIS2DH12_REG_OUT_Z_H                0x2D
#define LIS2DH12_REG_FIFO_CTRL_REG          0x2E
#define LIS2DH12_REG_FIFO_SRC_REG           0x2F
#define LIS2DH12_REG_INT1_CFG               0x30
#define LIS2DH12_REG_INT1_SRC               0x31
#define LIS2DH12_REG_INT1_THS               0x32
#define LIS2DH12_REG_INT1_DURATION          0x33
#define LIS2DH12_REG_INT2_CFG               0x34
#define LIS2DH12_REG_INT2_SRC               0x35
#define LIS2DH12_REG_INT2_THS               0x36
#define LIS2DH12_REG_INT2_DURATION          0x37
#define LIS2DH12_REG_CLICK_CFG              0x38
#define LIS2DH12_REG_CLICK_SRC              0x39
#define LIS2DH12_REG_CLICK_THS              0x3A
#define LIS2DH12_REG_TIME_LIMIT             0x3B
#define LIS2DH12_REG_TIME_LATENCY           0x3C
#define LIS2DH12_REG_TIME_WINDOW            0x3D
#define LIS2DH12_REG_ACT_THS                0x3E
#define LIS2DH12_REG_ACT_DUR                0x3F

#define LIS2DH12_WHO_AM_I                   0x33
#define LIS2DH12_AUTO_INCR                  0x80

#define LIS2DH12_REG_STATUS_REG_AUX_TDA         BIT(2) // Temperature new data available. Default value: 0
#define LIS2DH12_REG_STATUS_REG_AUX_TOR         BIT(6) // Temperature data overrun. Default value: 0

#define LIS2DH12_REG_CTRL_REG0_REQUIRED         BIT(4)
#define LIS2DH12_REG_CTRL_REG0_SDO_PU_DISC      BIT(7) // Disconnect SDO/SA0 pull-up. Default value: 00010000 (0: pull-up connected to SDO/SA0 pin; 1: pull-up disconnected to SDO/SA0 pin)

#define LIS2DH12_REG_TEMP_CFG_REG_TEMP_EN       (BIT(6) | BIT(7)) // Temperature sensor (T) enable. Default value: 00

#define LIS2DH12_REG_CTRL_REG1_XEN              BIT(0) // X-axis enable. Default value: 1
#define LIS2DH12_REG_CTRL_REG1_YEN              BIT(1) // Y-axis enable. Default value: 1
#define LIS2DH12_REG_CTRL_REG1_ZEN              BIT(2) // Z-axis enable. Default value: 1
#define LIS2DH12_REG_CTRL_REG1_LPEN             BIT(3) // Low-power mode enable. Default value: 0
#define LIS2DH12_REG_CTRL_REG1_ODR(odr)         ((odr) << 4)
#define LIS2DH12_ODR_POWER_DOWN                 0 // Power-down mode
#define LIS2DH12_ODR_1_HZ                       1 // HR / Normal / Low-power mode (1 Hz)
#define LIS2DH12_ODR_10_HZ                      2 // HR / Normal / Low-power mode (10 Hz)
#define LIS2DH12_ODR_25_HZ                      3 // HR / Normal / Low-power mode (25 Hz)
#define LIS2DH12_ODR_50_HZ                      4 // HR / Normal / Low-power mode (50 Hz)
#define LIS2DH12_ODR_100_HZ                     5 // HR / Normal / Low-power mode (100 Hz)
#define LIS2DH12_ODR_200_HZ                     6 // HR / Normal / Low-power mode (200 Hz)
#define LIS2DH12_ODR_400_HZ                     7 // HR/ Normal / Low-power mode (400 Hz)
#define LIS2DH12_ODR_LP_1620_HZ                 8 // Low-power mode (1.620 kHz)
#define LIS2DH12_ODR_1344_HZ_OR_LP_5376         9 // HR/ Normal (1.344 kHz); Low-power mode (5.376 kHz)

#define LIS2DH12_REG_CTRL_REG2_HPCLICK          BIT(2) // High-pass filter enabled for CLICK function. (0: filter bypassed; 1: filter enabled)

#define LIS2DH12_REG_CTRL_REG3_I1_OVERRUN       BIT(1) // FIFO overrun interrupt on INT1 pin. Default value: 0 (0: disable; 1: enable)
#define LIS2DH12_REG_CTRL_REG3_I1_WTM           BIT(2) // FIFO watermark interrupt on INT1 pin. Default value: 0 (0: disable; 1: enable)
#define LIS2DH12_REG_CTRL_REG3_I1_ZYXDA         BIT(4) // ZYXDA interrupt on INT1 pin. Default value: 0 (0: disable; 1: enable)
#define LIS2DH12_REG_CTRL_REG3_I1_IA2           BIT(5) // IA2 interrupt on INT1 pin. Default value: 0 (0: disable; 1: enable)
#define LIS2DH12_REG_CTRL_REG3_I1_IA1           BIT(6) // IA1 interrupt on INT1 pin. Default value: 0 (0: disable; 1: enable)
#define LIS2DH12_REG_CTRL_REG3_I1_CLICK         BIT(7) // CLICK interrupt on INT1 pin. Default value: 0 (0: disable; 1: enable)

#define LIS2DH12_REG_CTRL_REG4_SIM              BIT(0) // SPI serial interface mode selection. Default value: 0 (0: 4-wire interface; 1: 3-wire interface).
#define LIS2DH12_REG_CTRL_REG4_ST0              BIT(1) // Self test 0
#define LIS2DH12_REG_CTRL_REG4_ST1              BIT(2) // Self test 1
#define LIS2DH12_REG_CTRL_REG4_HR               BIT(3) // High-resolution mode enable.
#define LIS2DH12_REG_CTRL_REG4_FS(fs)           ((fs) << 4)
#define LIS2DH12_FS_2g                          0 // Full-scale ±2 g
#define LIS2DH12_FS_4g                          1 // Full-scale ±4 g
#define LIS2DH12_FS_8g                          2 // Full-scale ±8 g
#define LIS2DH12_FS_16g                         3 // Full-scale±16 g
#define LIS2DH12_REG_CTRL_REG4_BLE              BIT(6) // Big/Little Endian data selection. Default value: 0. (0: data LSb at lower address; 1: data MSb at lower address) The BLE function can be activated only in high-resolution mode 
#define LIS2DH12_REG_CTRL_REG4_BDU              BIT(7) // Block data update. Default value: 0. (0: continuous update; 1: output registers not updated until MSB and LSB have been read)

#define LIS2DH12_REG_CTRL_REG5_BOOT             BIT(7) // Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content)

#define LIS2DH12_REG_CTRL_REG6_INT_ACTIVE_LOW   BIT(1) // INT_POLARITY INT1 and INT2 pin polarity. Default value: 0 (0: active-high; 1: active-low)
#define LIS2DH12_REG_CTRL_REG6_I2_ACT           BIT(3) // Enable activity interrupt on INT2 pin. Default value: 0 (0: disabled; 1: enabled)
#define LIS2DH12_REG_CTRL_REG6_I2_BOOT_EN       BIT(4) // I2_BOOT Enable boot on INT2 pin. Default value: 0 (0: disabled; 1: enabled)
#define LIS2DH12_REG_CTRL_REG6_I2_IA2           BIT(5) // Enable interrupt 2 function on INT2 pin. Default value: 0 (0: function disabled; 1: function enabled)
#define LIS2DH12_REG_CTRL_REG6_I2_IA1           BIT(6) // Enable interrupt 1 function on INT2 pin. Default value: 0 (0: function disabled; 1: function enabled)
#define LIS2DH12_REG_CTRL_REG6_I2_CLICK         BIT(7) // Click interrupt on INT2 pin. Default value: 0 (0: disabled; 1: enabled)

#define LIS2DH12_REG_CLICK_CFG_XS               BIT(0) // Enable interrupt single-click on X-axis. Default value: 0 (0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold)
#define LIS2DH12_REG_CLICK_CFG_XD               BIT(1) // Enable interrupt double-click on X-axis. Default value: 0 (0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold)
#define LIS2DH12_REG_CLICK_CFG_YS               BIT(2) // Enable interrupt single-click on Y-axis. Default value: 0 (0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold)
#define LIS2DH12_REG_CLICK_CFG_YD               BIT(3) // Enable interrupt double-click on Y-axis. Default value: 0 (0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold)
#define LIS2DH12_REG_CLICK_CFG_ZS               BIT(4) // Enable interrupt single-click on Z-axis. Default value: 0 (0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold)
#define LIS2DH12_REG_CLICK_CFG_ZD               BIT(5) // Enable interrupt double-click on Z-axis. Default value: 0 (0: disable interrupt request; 1: enable interrupt request on measured accel. value higher than preset threshold)

#define LIS2DH12_REG_CLICK_THS_LIR_CLICK        BIT(7) // If the LIR_Click bit is not set, the interrupt is kept high for the duration of the latency window. If the LIR_Click bit is set, the interrupt is kept high until the CLICK_SRC (39h) register is read.

#define LIS2DH12_MODE_HIGH_RESOLUTION           0x00
#define LIS2DH12_MODE_NORMAL                    0x01
#define LIS2DH12_MODE_LOW_POWER                 0x02

#endif /* LIS2DH12_DEF_H_ */
