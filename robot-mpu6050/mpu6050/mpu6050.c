#include "mpu6050.h"
static int16_t Mpu6050Addr = 0x68;
MPU6050DATATYPE Mpu6050_Data;
int8_t Sensor_I2C2_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Read(&hi2c2,DevAddr,MemAddr,1,oData,DataLen,1000);
}
int8_t Sensor_I2C2_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Write(&hi2c2,DevAddr,MemAddr,1,iData,DataLen,1000);
}
int16_t Sensor_I2C2_Serch(void)
{
	for(uint8_t i = 1; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c2, i, 1, 1000) == HAL_OK)
		{
			Mpu6050Addr = i;
			return i;
		}
	}
	return 0x68;
}


int8_t MPU6050_Init(int16_t Addr)
{
    uint8_t check;
    uint8_t write_val;
    HAL_StatusTypeDef status;

    // 1. 读取 WHO_AM_I 并检查 I2C 通信状态
    status = Sensor_I2C2_Read(Addr, WHO_AM_I, &check, 1);
    
    if (status != HAL_OK) {
        return -2; // I2C 通信错误
    }

    if (check != 0x68) {
        return -1; // 设备ID不匹配
    }

    // 2. 唤醒并设置时钟源
    write_val = 0x00; // 内部振荡器
    if(Sensor_I2C2_Write(Addr, PWR_MGMT_1, &write_val, 1) != HAL_OK) return -3;

    // 3. 配置数字低通滤波器 (DLPF) - 推荐添加此步骤
    // 例如：设置 DLPF 为 5 (截止频率 10Hz/10Hz)
    write_val = 0x05; 
    if(Sensor_I2C2_Write(Addr, CONFIG, &write_val, 1) != HAL_OK) return -4;

    // 4. 配置采样率分频 (1kHz / (1+7) = 125Hz)
    write_val = 0x07;
    if(Sensor_I2C2_Write(Addr, SMPLRT_DIV, &write_val, 1) != HAL_OK) return -5;

    // 5. 加速度配置：±2g (0x00)
    write_val = 0x00; 
    if(Sensor_I2C2_Write(Addr, ACCEL_CONFIG, &write_val, 1) != HAL_OK) return -6;

    // 6. 陀螺仪配置：±250 deg/s (0x00)
    write_val = 0x00; 
    if(Sensor_I2C2_Write(Addr, GYRO_CONFIG, &write_val, 1) != HAL_OK) return -7;

    return 0; // 初始化成功
}



void MPU6050_Read_Accel(void)
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是加速度X高 - 加速度X低 - 加速度Y高位 - 加速度Y低位 - 加速度Z高位 - 加速度度Z低位
	Sensor_I2C2_Read(Mpu6050Addr, ACCEL_XOUT_H, Read_Buf, 6); 
	
	Mpu6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Accel_X = Mpu6050_Data.Accel_X / 16384.0f;
	Mpu6050_Data.Accel_Y = Mpu6050_Data.Accel_Y / 16384.0f;
	Mpu6050_Data.Accel_Z = Mpu6050_Data.Accel_Z / 16384.0f;
	
}
void MPU6050_Read_Gyro(void)
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是角度X高 - 角度X低 - 角度Y高位 - 角度Y低位 - 角度Z高位 - 角度Z低位
	Sensor_I2C2_Read(Mpu6050Addr, GYRO_XOUT_H, Read_Buf, 6); 
	
	Mpu6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Gyro_X = Mpu6050_Data.Gyro_X / 131.0f;
	Mpu6050_Data.Gyro_Y = Mpu6050_Data.Gyro_Y / 131.0f;
	Mpu6050_Data.Gyro_Z = Mpu6050_Data.Gyro_Z / 131.0f;
	
}
void MPU6050_Read_Temp(void)
{
    uint8_t Read_Buf[2];
	
	Sensor_I2C2_Read(Mpu6050Addr, TEMP_OUT_H, Read_Buf, 2); 
	
	Mpu6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050_Data.Temp = 36.53f + (Mpu6050_Data.Temp / 340.0f);
}