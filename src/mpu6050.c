#include "mpu6050.h"

#define FS_SEL  3
#define AFS_SEL 3

void MPU6050_init(MPU6050_t *pMPU6050, uint8_t AD0pin)
{
    pMPU6050->address = AD0pin ? BASE_ADDRESS + 1 : BASE_ADDRESS;

    if (pMPU6050->accel_range)
    {
        mpu6050_set_accel_range(pMPU6050, pMPU6050->accel_range);
    }

    if (pMPU6050->gyro_range)
    {
        mpu6050_set_gyro_range(pMPU6050, pMPU6050->gyro_range);
    }
}

void MPU6050_getAcceleration(MPU6050_t *pMPU6050, int16_t *px, int16_t *py, int16_t *pz)
{
    uint8_t sendBuffer;
    uint8_t receiveBuffer[6];
    sendBuffer = ACCEL_XOUT_H;
    pMPU6050->I2C_Send(pMPU6050->address, &sendBuffer, 1);
    pMPU6050->I2C_Receive(pMPU6050->address, receiveBuffer, 6);

    *px = receiveBuffer[0] << 8 | receiveBuffer[1];
    *py = receiveBuffer[2] << 8 | receiveBuffer[3];
    *pz = receiveBuffer[4] << 8 | receiveBuffer[5];
}

void MPU6050_getRotation(MPU6050_t *pMPU6050, int16_t *px, int16_t *py, int16_t *pz)
{
    uint8_t sendBuffer;
    uint8_t receiveBuffer[6];
    sendBuffer = GYRO_XOUT_H;
    pMPU6050->I2C_Send(pMPU6050->address, &sendBuffer, sizeof(sendBuffer));
    pMPU6050->I2C_Receive(pMPU6050->address, receiveBuffer, sizeof(receiveBuffer));

    *px = receiveBuffer[0] << 8 | receiveBuffer[1];
    *py = receiveBuffer[2] << 8 | receiveBuffer[3];
    *pz = receiveBuffer[4] << 8 | receiveBuffer[5];
}

void mpu6050_set_accel_range(MPU6050_t *pMPU6050, range_t range)
{
    uint8_t sendBuffer[2];
    sendBuffer[0] = ACCEL_CONFIG;
    sendBuffer[1] = range << AFS_SEL;
    pMPU6050->I2C_Send(pMPU6050->address, sendBuffer, sizeof(sendBuffer));

    pMPU6050->accel_range = range;
}

void mpu6050_set_gyro_range(MPU6050_t * pMPU6050, range_t range)
{
    uint8_t sendBuffer[2];
    sendBuffer[0] = GYRO_CONFIG;
    sendBuffer[1] = range << FS_SEL;
    pMPU6050->I2C_Send(pMPU6050->address, sendBuffer, sizeof(sendBuffer));

    pMPU6050->gyro_range = range;
}