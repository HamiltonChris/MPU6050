#include "mpu6050.h"


void MPU6050_init(MPU6050_t *pMPU6050, uint8_t AD0pin)
{
    pMPU6050->address = AD0pin ? BASE_ADDRESS + 1 : BASE_ADDRESS;


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