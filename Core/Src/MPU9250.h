/*
 * MPU9250.h
 *
 *  Created on: 9 de jun de 2019
 *      Author: Eduardo Lacerda Campos
 */

#ifndef SENSOR_MPU9250_H_
#define SENSOR_MPU9250_H_

#include <math.h>
#include <assert.h>
#include <unistd.h>
#include "main.h"
#include "cmsis_os.h"


#define MPU9250_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU9250_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)


//MPU9250 Registers
#define MPU9250_RG_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RG_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RG_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RG_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU9250_RG_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU9250_RG_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU9250_RG_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU9250_RG_XA_OFFS_L_TC     0x07
#define MPU9250_RG_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU9250_RG_YA_OFFS_L_TC     0x09
#define MPU9250_RG_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU9250_RG_ZA_OFFS_L_TC     0x0B
#define MPU9250_RG_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU9250_RG_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU9250_RG_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU9250_RG_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU9250_RG_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9250_RG_XG_OFFS_USRL     0x14
#define MPU9250_RG_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9250_RG_YG_OFFS_USRL     0x16
#define MPU9250_RG_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU9250_RG_ZG_OFFS_USRL     0x18
#define MPU9250_RG_SMPLRT_DIV       0x19
#define MPU9250_RG_CONFIG           0x1A
#define MPU9250_RG_GYRO_CONFIG      0x1B
#define MPU9250_RG_ACCEL_CONFIG     0x1C
#define MPU9250_RG_FF_THR           0x1D
#define MPU9250_RG_FF_DUR           0x1E
#define MPU9250_RG_MOT_THR          0x1F
#define MPU9250_RG_MOT_DUR          0x20
#define MPU9250_RG_ZRMOT_THR        0x21
#define MPU9250_RG_ZRMOT_DUR        0x22
#define MPU9250_RG_FIFO_EN          0x23
#define MPU9250_RG_I2C_MST_CTRL     0x24
#define MPU9250_RG_I2C_SLV0_ADDR    0x25
#define MPU9250_RG_I2C_SLV0_REG     0x26
#define MPU9250_RG_I2C_SLV0_CTRL    0x27
#define MPU9250_RG_I2C_SLV1_ADDR    0x28
#define MPU9250_RG_I2C_SLV1_REG     0x29
#define MPU9250_RG_I2C_SLV1_CTRL    0x2A
#define MPU9250_RG_I2C_SLV2_ADDR    0x2B
#define MPU9250_RG_I2C_SLV2_REG     0x2C
#define MPU9250_RG_I2C_SLV2_CTRL    0x2D
#define MPU9250_RG_I2C_SLV3_ADDR    0x2E
#define MPU9250_RG_I2C_SLV3_REG     0x2F
#define MPU9250_RG_I2C_SLV3_CTRL    0x30
#define MPU9250_RG_I2C_SLV4_ADDR    0x31
#define MPU9250_RG_I2C_SLV4_REG     0x32
#define MPU9250_RG_I2C_SLV4_DO      0x33
#define MPU9250_RG_I2C_SLV4_CTRL    0x34
#define MPU9250_RG_I2C_SLV4_DI      0x35
#define MPU9250_RG_I2C_MST_STATUS   0x36
#define MPU9250_RG_INT_PIN_CFG      0x37
#define MPU9250_RG_INT_ENABLE       0x38
#define MPU9250_RG_DMP_INT_STATUS   0x39
#define MPU9250_RG_INT_STATUS       0x3A
#define MPU9250_RG_ACCEL_XOUT_H     0x3B
#define MPU9250_RG_ACCEL_XOUT_L     0x3C
#define MPU9250_RG_ACCEL_YOUT_H     0x3D
#define MPU9250_RG_ACCEL_YOUT_L     0x3E
#define MPU9250_RG_ACCEL_ZOUT_H     0x3F
#define MPU9250_RG_ACCEL_ZOUT_L     0x40
#define MPU9250_RG_TEMP_OUT_H       0x41
#define MPU9250_RG_TEMP_OUT_L       0x42
#define MPU9250_RG_GYRO_XOUT_H      0x43
#define MPU9250_RG_GYRO_XOUT_L      0x44
#define MPU9250_RG_GYRO_YOUT_H      0x45
#define MPU9250_RG_GYRO_YOUT_L      0x46
#define MPU9250_RG_GYRO_ZOUT_H      0x47
#define MPU9250_RG_GYRO_ZOUT_L      0x48
#define MPU9250_RG_EXT_SENS_DATA_00 0x49
#define MPU9250_RG_EXT_SENS_DATA_01 0x4A
#define MPU9250_RG_EXT_SENS_DATA_02 0x4B
#define MPU9250_RG_EXT_SENS_DATA_03 0x4C
#define MPU9250_RG_EXT_SENS_DATA_04 0x4D
#define MPU9250_RG_EXT_SENS_DATA_05 0x4E
#define MPU9250_RG_EXT_SENS_DATA_06 0x4F
#define MPU9250_RG_EXT_SENS_DATA_07 0x50
#define MPU9250_RG_EXT_SENS_DATA_08 0x51
#define MPU9250_RG_EXT_SENS_DATA_09 0x52
#define MPU9250_RG_EXT_SENS_DATA_10 0x53
#define MPU9250_RG_EXT_SENS_DATA_11 0x54
#define MPU9250_RG_EXT_SENS_DATA_12 0x55
#define MPU9250_RG_EXT_SENS_DATA_13 0x56
#define MPU9250_RG_EXT_SENS_DATA_14 0x57
#define MPU9250_RG_EXT_SENS_DATA_15 0x58
#define MPU9250_RG_EXT_SENS_DATA_16 0x59
#define MPU9250_RG_EXT_SENS_DATA_17 0x5A
#define MPU9250_RG_EXT_SENS_DATA_18 0x5B
#define MPU9250_RG_EXT_SENS_DATA_19 0x5C
#define MPU9250_RG_EXT_SENS_DATA_20 0x5D
#define MPU9250_RG_EXT_SENS_DATA_21 0x5E
#define MPU9250_RG_EXT_SENS_DATA_22 0x5F
#define MPU9250_RG_EXT_SENS_DATA_23 0x60
#define MPU9250_RG_MOT_DETECT_STATUS    0x61
#define MPU9250_RG_I2C_SLV0_DO      0x63
#define MPU9250_RG_I2C_SLV1_DO      0x64
#define MPU9250_RG_I2C_SLV2_DO      0x65
#define MPU9250_RG_I2C_SLV3_DO      0x66
#define MPU9250_RG_I2C_MST_DELAY_CTRL   0x67
#define MPU9250_RG_SIGNAL_PATH_RESET    0x68
#define MPU9250_RG_MOT_DETECT_CTRL      0x69
#define MPU9250_RG_USER_CTRL        0x6A
#define MPU9250_RG_PWR_MGMT_1       0x6B
#define MPU9250_RG_PWR_MGMT_2       0x6C
#define MPU9250_RG_BANK_SEL         0x6D
#define MPU9250_RG_MEM_START_ADDR   0x6E
#define MPU9250_RG_MEM_R_W          0x6F
#define MPU9250_RG_DMP_CFG_1        0x70
#define MPU9250_RG_DMP_CFG_2        0x71
#define MPU9250_RG_FIFO_COUNTH      0x72
#define MPU9250_RG_FIFO_COUNTL      0x73
#define MPU9250_RG_FIFO_R_W         0x74
#define MPU9250_RG_WHO_AM_I         0x75

//Registers values
#define MPU9250_GYRO_FS_250         0x00
#define MPU9250_GYRO_FS_500         (0x01<<3)
#define MPU9250_GYRO_FS_1000        (0x02<<3)
#define MPU9250_GYRO_FS_2000 		(0x03<<3)
#define MPU9250_GYRO_MASK 			0b00011000

#define MPU9250_CLOCK_INTERNAL          0x00
#define MPU9250_CLOCK_PLL_XGYRO         0x01
#define MPU9250_CLOCK_PLL_YGYRO         0x02
#define MPU9250_CLOCK_PLL_ZGYRO         0x03
#define MPU9250_CLOCK_PLL_EXT32K        0x04
#define MPU9250_CLOCK_PLL_EXT19M        0x05
#define MPU9250_CLOCK_KEEP_RESET 		0x07
#define MPU9250_CLOCK_MASK  			0b00000111

#define MPU9250_ACCEL_FS_2          0x00
#define MPU9250_ACCEL_FS_4          (0x01<<3)
#define MPU9250_ACCEL_FS_8          (0x02<<3)
#define MPU9250_ACCEL_FS_16 		(0x03<<3)
#define MPU9250_ACCEL_MASK 			0b00011000


#define MPU9250_PWR1_DEVICE_RESET_BIT   7
#define MPU9250_PWR1_SLEEP_BIT          6
#define MPU9250_PWR1_CYCLE_BIT          5
#define MPU9250_PWR1_TEMP_DIS_BIT       3
#define MPU9250_PWR1_CLKSEL_BIT         2

#define AK8963_CNTL1			0x0A
#define AK8963_CNTL1_CONT_MODE_POWERDOWN		0x00
#define AK8963_CNTL1_CONT_MODE_1		0x02
#define AK8963_CNTL1_CONT_MODE_2		0x06
#define AK8963_CNTL1_CONT_MODE_SELF_TEST		0x08
#define AK8963_CNTL1_16BIT				0x10

#define AK8963_CNTL2			0x0B
#define AK8963_CNTL2_RTS				0x01

#define AK8963_ASTC_SELF		0x40
#define AK8963_ASTC				0x0C

#define MPU9250_MAG_ADDRESS	0x0C

#define AK8963_STATUS1			0x02

#define AK8963_ASAX_AD			0x10


#ifndef MATRIX_H_
	typedef float cell;
#endif


class MPU9250 {
public:
	MPU9250();
	virtual ~MPU9250();
	cell data_a[3];//acceleration
	cell data_w[3];//angular velocity
	cell data_m[3];//magnetic field
	HAL_StatusTypeDef Status;
	HAL_StatusTypeDef Status_mag;

	cell &ax=data_a[0];
	cell &ay=data_a[1];
	cell &az=data_a[2];

	cell &gx=data_w[0];
	cell &gy=data_w[1];
	cell &gz=data_w[2];

	cell &mx=data_m[0];
	cell &my=data_m[1];
	cell &mz=data_m[2];

	cell temperatura;

	void Setup(I2C_HandleTypeDef* Dev_);
	bool Read(void);
	bool Read_mag(void);
	bool Read_mag_RAW(void);

	uint8_t buffer[14];

	int16_t bias[3];
	int16_t gx_temp;
	int16_t gy_temp;
	int16_t gz_temp;

private:
	I2C_HandleTypeDef *Dev;
	uint8_t devAddr;

	double asax;
	double asay;
	double asaz;

	double bias_m[3]={-3.2063, 34.4018, -39.4864};

	cell Gyro_LSB;
	cell Gravy_LSB;
	cell Mag_LSB;

	void set_reg(uint8_t reg,uint8_t source, uint8_t mask);

	void Reset();
	void setClockSource(uint8_t source);
	void setFullScaleGyroRange(uint8_t range);
	void setFullScaleAccelRange(uint8_t range);
	void setSleepMode(bool mode);
	bool Bias_Gyro_Calc(void);
	bool Self_Test_Mag(void);
};

#endif /* SENSOR_MPU9250_H_ */
