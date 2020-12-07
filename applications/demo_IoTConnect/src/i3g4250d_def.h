
#ifndef I3G4250D_DEF_H_
#define I3G4250D_DEF_H_


#define I3G4250D_7BIT_ADDR                  0x69
#define I3G4250D_REG_WHO_AM_I               0x0F
#define I3G4250D_REG_CTRL_REG1              0x20
#define I3G4250D_REG_CTRL_REG2              0x21
#define I3G4250D_REG_CTRL_REG3              0x22
#define I3G4250D_REG_CTRL_REG4              0x23
#define I3G4250D_REG_CTRL_REG5              0x24
#define I3G4250D_REG_REFERENCE              0x25
#define I3G4250D_REG_STATUS_REG             0x27
#define I3G4250D_REG_OUT_X_L                0x28
#define I3G4250D_REG_OUT_X_H                0x29
#define I3G4250D_REG_OUT_Y_L                0x2A
#define I3G4250D_REG_OUT_Y_H                0x2B
#define I3G4250D_REG_OUT_Z_L                0x2C
#define I3G4250D_REG_FIFO_CTRL_REG          0x2E
#define I3G4250D_REG_FIFO_SRC_REG           0x2F
#define I3G4250D_REG_INT1_CFG               0x30
#define I3G4250D_REG_INT1_SRC               0x31
#define I3G4250D_REG_INT1_THS_XH            0x32
#define I3G4250D_REG_INT1_THS_XL            0x33
#define I3G4250D_REG_INT1_THS_YH            0x34
#define I3G4250D_REG_INT1_THS_YL            0x35
#define I3G4250D_REG_INT1_THS_ZH            0x36
#define I3G4250D_REG_INT1_THS_ZL            0x37
#define I3G4250D_REG_INT1_DURATION          0x38

#define I3G4250D_WHO_AM_I                   0xD3
#define I3G4250D_AUTO_INCR_BITMASK          0x80

#define I3G4250D_REG_CTRL_REG3_I2_DRDY      BIT(3) // Data ready on DRDY/INT2. Default value 0. (0: disable; 1: enable)
#define I3G4250D_REG_CTRL_REG3_H_LACTIVE    BIT(5) // Interrupt active configuration on INT1. Default value 0. (0: high; 1: low)
#define I3G4250D_REG_CTRL_REG3_I1_BOOT_EN   BIT(6) // I1_BOOT Boot status available on INT1. Default value 0. (0: disable; 1: enable)
#define I3G4250D_REG_CTRL_REG3_I1_INT1      BIT(7) // Interrupt enable on the INT1 pin. Default value 0. (0: disable; 1: enable)

#define I3G4250D_REG_CTRL_REG4_FS(fs)       ((fs) << 4)
#define LIS2DH12_FS_245dps                  0
#define LIS2DH12_FS_500dps                  1
#define LIS2DH12_FS_2000dps                 2

#define I3G4250D_REG_CTRL_REG5_BOOT         BIT(7) // Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content)

#define I3G4250D_REG_INT1_DURATION_WAIT     BIT(7) // WAIT enable. Default value: 0 (0: disable; 1: enable)

#endif /* I3G4250D_DEF_H_ */
