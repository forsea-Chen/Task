/*
 * mpu6050.cpp
 *
 *  Created on: 2019年10月18日
 *      Author: Administrator
 */
#include "mpu6050.h"
mpu_def mpu_data;
void mpu6050_init()
    {
	uint8_t txdata[1];
	txdata[0]=0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, MPU6050_RA_PWR_MGMT_1, 1, txdata, 1, 1000);
	HAL_Delay(500);
	txdata[0]=0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, MPU6050_RA_PWR_MGMT_1, 1, txdata, 1, 1000);
	txdata[0]=0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, MPU6050_RA_SMPLRT_DIV, 1, txdata, 1, 1000);
	txdata[0]=0x06;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, MPU6050_RA_CONFIG, 1, txdata, 1, 1000);
	txdata[0]=0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, MPU6050_RA_ACCEL_CONFIG, 1, txdata, 1, 1000);
	txdata[0]=0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, MPU6050_RA_GYRO_CONFIG, 1, txdata, 1, 1000);
    }
void mpu6050_read()
    {
	uint8_t rxdata[14];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADD, MPU6050_RA_ACCEL_XOUT_H, 1, rxdata, 14, 1000);
        memcpy(&mpu_data,rxdata,sizeof(mpu_def));
    }
