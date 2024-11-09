/** @file mpu6050.c
*
* @brief A driver for the mpu6050 accelerometer and gyroscope.
*
* @par
*
*/

#include "mpu6050.h"

#define FS_SEL  3
#define AFS_SEL 3

void mpu6050_init(mpu6050_t * p_mpu6050, uint8_t ad0_pin)
{
    p_mpu6050->address = ad0_pin ? BASE_ADDRESS + 1 : BASE_ADDRESS;

    if (p_mpu6050->accel_range)
    {
        mpu6050_set_accel_range(p_mpu6050, p_mpu6050->accel_range);
    }

    if (p_mpu6050->gyro_range)
    {
        mpu6050_set_gyro_range(p_mpu6050, p_mpu6050->gyro_range);
    }
}

void mpu6050_getAcceleration(const mpu6050_t * p_mpu6050, int16_t * p_x, int16_t * p_y, int16_t * p_z)
{
    uint8_t sendBuffer;
    uint8_t receiveBuffer[6];

    sendBuffer = ACCEL_XOUT_H;
    p_mpu6050->i2c_send(p_mpu6050->address, &sendBuffer, 1);
    p_mpu6050->i2c_receive(p_mpu6050->address, receiveBuffer, 6);

    *p_x = receiveBuffer[0] << 8 | receiveBuffer[1];
    *p_y = receiveBuffer[2] << 8 | receiveBuffer[3];
    *p_z = receiveBuffer[4] << 8 | receiveBuffer[5];
}

void mpu6050_getRotation(const mpu6050_t * p_mpu6050, int16_t * p_x, int16_t * p_y, int16_t * p_z)
{
    uint8_t sendBuffer;
    uint8_t receiveBuffer[6];

    sendBuffer = GYRO_XOUT_H;
    p_mpu6050->i2c_send(p_mpu6050->address, &sendBuffer, sizeof(sendBuffer));
    p_mpu6050->i2c_receive(p_mpu6050->address, receiveBuffer, sizeof(receiveBuffer));

    *p_x = receiveBuffer[0] << 8 | receiveBuffer[1];
    *p_y = receiveBuffer[2] << 8 | receiveBuffer[3];
    *p_z = receiveBuffer[4] << 8 | receiveBuffer[5];
}

void mpu6050_set_accel_range(mpu6050_t * p_mpu6050, range_t range)
{
    uint8_t sendBuffer[2];
    sendBuffer[0] = ACCEL_CONFIG;
    sendBuffer[1] = range << AFS_SEL;
    p_mpu6050->i2c_send(p_mpu6050->address, sendBuffer, sizeof(sendBuffer));

    p_mpu6050->accel_range = range;
}

void mpu6050_set_gyro_range(mpu6050_t * p_mpu6050, range_t range)
{
    uint8_t sendBuffer[2];
    sendBuffer[0] = GYRO_CONFIG;
    sendBuffer[1] = range << FS_SEL;
    p_mpu6050->i2c_send(p_mpu6050->address, sendBuffer, sizeof(sendBuffer));

    p_mpu6050->gyro_range = range;
}

bool mpu6050_test_connection(const mpu6050_t * p_mpu6050)
{
    uint8_t sendBuffer;
    uint8_t receiveBuffer;
    sendBuffer = WHO_AM_I;
    p_mpu6050->i2c_send(p_mpu6050->address, &sendBuffer, sizeof(sendBuffer));
    p_mpu6050->i2c_receive(p_mpu6050->address, &receiveBuffer, sizeof(receiveBuffer));

    return p_mpu6050->address == receiveBuffer;
}

/*** end of file ***/
