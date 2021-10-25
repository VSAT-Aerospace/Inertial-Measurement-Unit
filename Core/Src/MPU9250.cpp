/*
 * MPU9250.cpp
 *
 *  Created on: 9 de jun de 2019
 *      Author: educampos
 */

#include "MPU9250.h"

MPU9250::MPU9250() {
	devAddr = MPU9250_ADDRESS_AD0_LOW;
	Dev=NULL;
	Gyro_LSB=0;
	Gravy_LSB=0;
	temperatura =0;
	bias[0]=0;
	bias[1]=0;
	bias[2]=0;
	gx_temp=0;
	gy_temp=0;
	gz_temp=0;

}

MPU9250::~MPU9250() {
	// TODO Auto-generated destructor stub
}


void MPU9250::Setup(I2C_HandleTypeDef* Dev_){

	Dev = Dev_;

	uint8_t value[2];
	HAL_StatusTypeDef status;
	//reset and wake up procedure
	do{
		//restart
		value[0]= MPU9250_RG_PWR_MGMT_1;
		value[1]= 0x80;
		status = HAL_I2C_Master_Transmit(Dev,devAddr, value, 2, 100);
		if (status!= HAL_OK)
			continue;

		usleep(100000);
		// resets the signal paths for all sensors
		value[0]= MPU9250_RG_USER_CTRL;
		value[1] =1;
		status =HAL_I2C_Master_Transmit(Dev,devAddr, value,2 , 100);
		if (status!= HAL_OK)
			continue;

		usleep(100000);
		value[0]=MPU9250_RG_SIGNAL_PATH_RESET;
		value[1]=0x07;
		status =HAL_I2C_Master_Transmit(Dev,devAddr, value, 2, 100);
		if (status!= HAL_OK)
			continue;

		usleep(100000);
		value[0]=MPU9250_RG_WHO_AM_I;
		status =HAL_I2C_Master_Transmit(Dev,devAddr, value, 1, 100);
		status =HAL_I2C_Master_Receive(Dev,devAddr, value, 1, 100);
		assert(value[0] == 0x71);
	}while(status != HAL_OK);

	value[0]=MPU9250_RG_USER_CTRL;
	status =HAL_I2C_Master_Transmit(Dev,devAddr, value, 1, 100);
	HAL_I2C_Master_Receive(Dev,devAddr, value, 1, 100);
	assert(value == 0x00);

	//wake up and clock source
	value[0]=MPU9250_RG_PWR_MGMT_1;
	value[1]=1;
	HAL_I2C_Master_Transmit(Dev,devAddr, value, 2, 100);
	//Sample Rate Divider
	do{
		value[0]=MPU9250_RG_SMPLRT_DIV;
		value[1]=7;
		HAL_I2C_Master_Transmit(Dev,devAddr, value, 2, 100);

		value[0]=MPU9250_RG_SMPLRT_DIV;
		status =HAL_I2C_Master_Transmit(Dev,devAddr, value, 1, 100);
		HAL_I2C_Master_Receive(Dev,devAddr, value, 1, 100);
	}while(value[0]!=7);


	setFullScaleGyroRange(MPU9250_GYRO_FS_250);
	Gyro_LSB = (M_PI/180.0) * 250.0/pow(2,15); //for MPU9250_GYRO_FS_250 -> LSB = 131

	value[0]=MPU9250_RG_CONFIG;
	status =HAL_I2C_Master_Transmit(Dev,devAddr, value, 1, 100);
	HAL_I2C_Master_Receive(Dev, devAddr, value +1, 1, 100);
	value[1] = (value[1]& 0xF8) | 0x06 ;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,2,100);

	setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
	Gravy_LSB = 2/pow(2,15);

	//---------------------------------------------------------------------
	//The AK8975 is actually its own separate TWI device inside the MPU9150.
	//To be able to communicate with it, we must enable "bypass mode" by setting

	//the BYPASS_EN bit in register 0x37 "INT Pin / Bypass Enable Configuration"
	value[0]=MPU9250_RG_INT_PIN_CFG;
	value[1]=0x02;
	HAL_I2C_Master_Transmit(Dev,devAddr, value, 2, 100);

	Mag_LSB = 0.15;

	//verify is the device is connected
	value[0] = MPU9250_MAG_ADDRESS;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
	HAL_I2C_Master_Receive(Dev,devAddr, value , 1,100);
	assert(value[0]==0x48);

	//read FUSE
	uint8_t buffer[3];
	value[0] = AK8963_ASAX_AD;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
	HAL_I2C_Master_Receive(Dev,MPU9250_MAG_ADDRESS, buffer , 3 ,100);

    asax = ((int16_t)buffer[0]-128)*0.5/128+1;
    asay = ((int16_t)buffer[1]-128)*0.5/128+1;
    asaz = ((int16_t)buffer[2]-128)*0.5/128+1;

	//reset the device
    value[0]=AK8963_CNTL2;
    value[1]=AK8963_CNTL2_RTS;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS,value,2,100);
	usleep(100000);

	Self_Test_Mag();

	//Continuous measurement mode 2 ->  100Hz
	//16Bit mode
    value[0]=AK8963_CNTL1;
    value[1]=AK8963_CNTL1_CONT_MODE_2|AK8963_CNTL1_16BIT;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS, value,2,100);
	usleep(100000);


	Bias_Gyro_Calc();

}


bool MPU9250::Bias_Gyro_Calc(void){

	assert(Dev!=NULL);
	HAL_StatusTypeDef Status_local;

	const uint8_t samples = 32;
	uint8_t value[1];

	int16_t gx_=0;
	int16_t gy_=0;
	int16_t gz_=0;
	for(uint8_t aux = 0 ; aux <samples; ){

		value[0] = MPU9250_RG_ACCEL_XOUT_H;
		HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
		Status_local = HAL_I2C_Master_Receive(Dev,devAddr,buffer, 14 , 100);

		//only uses valid samples
		//TODO - Avoid deadlook
		if (Status_local == HAL_OK){
			gx_ += (((int16_t)buffer[8])  << 8) + (int16_t)buffer[9];
			gy_ += ((((int16_t)buffer[10]) << 8) + (int16_t)buffer[11]);
			gz_ += (((int16_t)buffer[12]) << 8) + (int16_t)buffer[13];
			usleep(10000);
			aux++;
		}
	}

	bias[0]=gx_/samples;
	bias[1]=gy_/samples;
	bias[2]=gz_/samples;

	if (Status_local == HAL_OK)
		return true;
	else
		return false;

}

bool MPU9250::Read(void){

	assert(Dev!=NULL);

	uint8_t value[1];
	value[0] = MPU9250_RG_ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
	Status = HAL_I2C_Master_Receive(Dev,devAddr,buffer, 14, 100);

	int16_t ax_temp = (((int16_t)buffer[0]) << 8) + (int16_t)buffer[1];
	int16_t ay_temp = (((int16_t)buffer[2]) << 8) + (int16_t)buffer[3];
	int16_t az_temp = (((int16_t)buffer[4]) << 8) + (int16_t)buffer[5];

	ax = -ax_temp*Gravy_LSB;
	ay = -ay_temp*Gravy_LSB;
	az = -az_temp*Gravy_LSB;

	cell norm = sqrt(ax*ax + ay*ay + az*az);

	ax = ax/norm;
	ay = ay/norm;
	az = az/norm;

	//Angular velocity in rad/s
	gx_temp = (((int16_t)buffer[8]) << 8) + (int16_t)buffer[9];
	gy_temp = (((int16_t)buffer[10]) << 8) + (int16_t)buffer[11];
	gz_temp = (((int16_t)buffer[12]) << 8) + (int16_t)buffer[13];

	gx = (gx_temp-bias[0])*Gyro_LSB;
	gy = (gy_temp-bias[1])*Gyro_LSB;
	gz = (gz_temp-bias[2])*Gyro_LSB;

	return Status;

}

bool MPU9250::Read_mag_RAW(void){

	assert(Dev!=NULL);

	uint8_t value[1];
	value[0] = AK8963_STATUS1;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
	Status_mag = HAL_I2C_Master_Receive(Dev,MPU9250_MAG_ADDRESS,value, 1, 100);

	if(Status_mag != HAL_OK)
		return false;

//	if((value&0x01)!=0x01)
//		return false;

	uint8_t local_buffer[7];
	value[0] = 0x03;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
	this->Status_mag = HAL_I2C_Master_Receive(Dev,MPU9250_MAG_ADDRESS,local_buffer, 7, 100);

	assert(this->Status_mag==HAL_OK);

	int16_t mx_temp = (((int16_t)local_buffer[1]) << 8) + (int16_t)local_buffer[0];
	int16_t my_temp = (((int16_t)local_buffer[3]) << 8) + (int16_t)local_buffer[2];
	int16_t mz_temp = (((int16_t)local_buffer[5]) << 8) + (int16_t)local_buffer[4];

	mx = asax*mx_temp*Mag_LSB;
	my = asay*my_temp*Mag_LSB;
	mz = asaz*mz_temp*Mag_LSB;

	if((local_buffer[6]&0x08)==0x08)
		return false;

	return true;

}

bool MPU9250::Read_mag(void){

	assert(Dev!=NULL);

	uint8_t value[1];
	value[0] = AK8963_STATUS1;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
	Status_mag = HAL_I2C_Master_Receive(Dev,MPU9250_MAG_ADDRESS,value, 1, 100);

	if(Status_mag != HAL_OK)
		return false;

//	if((value&0x01)!=0x01)
//		return false;

	uint8_t local_buffer[7];
	value[0] = 0x03;
	HAL_I2C_Master_Transmit(Dev,devAddr, value ,1,100);
	this->Status_mag = HAL_I2C_Master_Receive(Dev,MPU9250_MAG_ADDRESS,local_buffer, 7, 100);

	assert(this->Status_mag==HAL_OK);

	int16_t mx_temp = (((int16_t)local_buffer[1]) << 8) + (int16_t)local_buffer[0];
	int16_t my_temp = (((int16_t)local_buffer[3]) << 8) + (int16_t)local_buffer[2];
	int16_t mz_temp = (((int16_t)local_buffer[5]) << 8) + (int16_t)local_buffer[4];

	mx = asax*mx_temp*Mag_LSB -bias_m[0];
	my = asay*my_temp*Mag_LSB -bias_m[1];
	mz = asaz*mz_temp*Mag_LSB -bias_m[2];

	cell norm = sqrt(mx*mx + my*my  + mz*mz);

	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;


	if((local_buffer[6]&0x08)==0x08)
		return false;

	return true;

}


/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 */
void MPU9250::setClockSource(uint8_t source) {

	set_reg(MPU9250_RG_PWR_MGMT_1,source,MPU9250_CLOCK_MASK);
}

/** Set full-scale gyroscope range.
 */
void MPU9250::setFullScaleGyroRange(uint8_t range) {

    set_reg(MPU9250_RG_GYRO_CONFIG,range,MPU9250_GYRO_MASK);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void MPU9250::setFullScaleAccelRange(uint8_t range) {

    set_reg(MPU9250_RG_ACCEL_CONFIG,range,MPU9250_ACCEL_MASK);
}


/** Set sleep mode status.
When set to 1, this bit puts the MPU-60X0 into sleep mode.
*/
void MPU9250::setSleepMode(bool mode) {

    set_reg(MPU9250_RG_PWR_MGMT_1, ((uint8_t)mode)<<MPU9250_PWR1_SLEEP_BIT ,1<<MPU9250_PWR1_SLEEP_BIT);
}

void MPU9250::Reset(){

	uint8_t value[2];
	value[0] = MPU9250_RG_PWR_MGMT_1;
	value[1] = 0x01<<MPU9250_PWR1_DEVICE_RESET_BIT;

	HAL_I2C_Master_Transmit(Dev,devAddr,value , 2,100);

	usleep(100000);
	value[0]=MPU9250_RG_PWR_MGMT_1;
	value[1]=0xFF;
	while((value[1] & (0x01<<MPU9250_PWR1_DEVICE_RESET_BIT)) != 0){

		HAL_I2C_Master_Transmit(Dev,devAddr,value , 1,100);
		HAL_I2C_Master_Receive(Dev, devAddr,value + 1, 1, 100);
	}

}

void MPU9250::set_reg(uint8_t reg, uint8_t source, uint8_t mask) {

	uint8_t value[2];
	value[0]= reg;
	HAL_I2C_Master_Transmit(Dev,devAddr, value, 1 , 100);
	HAL_I2C_Master_Receive(Dev,devAddr, value+1, 1 , 100);

	value[1] &= ~(mask);
	value[1] |=  source;

	HAL_I2C_Master_Transmit(Dev,devAddr, value, 2, 100);
}

bool MPU9250::Self_Test_Mag(void){

	assert(Dev!=NULL);

	uint8_t value[2];

	//Set Power-down mode. (MODE[3:0]=“0000”)
	value[0]= AK8963_CNTL1;
	value[1] = AK8963_CNTL1_16BIT|AK8963_CNTL1_CONT_MODE_POWERDOWN;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS, value, 2, 100);
	usleep(100000);

	//Write “1” to SELF bit of ASTC register (other bits in this register should be kept “0”)
	value[0]= AK8963_ASTC;
	value[1] = AK8963_ASTC_SELF;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS, value, 2, 100);
	//Set Self-test Mode. (MODE[3:0]=“1000”)
	value[0]= AK8963_CNTL1;
	value[1] = AK8963_CNTL1_16BIT|AK8963_CNTL1_CONT_MODE_SELF_TEST;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS,  value, 2 ,100);

	//Check Data Ready or not by any of the following method.
	value[0]= AK8963_STATUS1;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS, value, 1, 100);
	do{
		Status_mag = HAL_I2C_Master_Receive(Dev,MPU9250_MAG_ADDRESS,value,1, 100);
	}while ((value[0]&0x01)!=0x01);

	// Read measurement data (HXL to HZH)
	uint8_t local_buffer[7];
	value[0]= 0x03;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS, value, 1, 100);
	this->Status_mag = HAL_I2C_Master_Receive(Dev,MPU9250_MAG_ADDRESS, local_buffer, 7, 100);

	assert(this->Status_mag== HAL_OK);

	int16_t mx_temp = (((int16_t)local_buffer[1]) << 8) + (int16_t)local_buffer[0];
	int16_t my_temp = (((int16_t)local_buffer[3]) << 8) + (int16_t)local_buffer[2];
	int16_t mz_temp = (((int16_t)local_buffer[5]) << 8) + (int16_t)local_buffer[4];


	// Write “0” to SELF bit of ASTC register
	value[0] =AK8963_ASTC;
	value[1]=0;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS, value, 2,100);


	//Set Power-down mode. (MODE[3:0]=“0000”)
	value[0] = AK8963_CNTL1;
	value[1] = AK8963_CNTL1_16BIT|AK8963_CNTL1_CONT_MODE_POWERDOWN;
	HAL_I2C_Master_Transmit(Dev,MPU9250_MAG_ADDRESS,  value, 2 ,100);
	usleep(100000);

	assert(abs(mx_temp)<200);
	assert(abs(my_temp)<200);
	assert(abs(mz_temp+2000)<1200);

	if(Status_mag != HAL_OK)
		return false;


	return true;


}
