/** @file mpu6050.c
*
* @brief A driver for the mpu6050 accelerometer and gyroscope.
*
* @par
*
*/

#include "mpu6050.h"


static void set_bits(const mpu6050_t * p_mpu6050,
                     uint8_t address,
                     uint8_t mask,
                     uint8_t bits);

void mpu6050_init(mpu6050_t * p_mpu6050, bool ad0_pin)
{
    p_mpu6050->address = ad0_pin ? BASE_ADDRESS + 1 : BASE_ADDRESS;

    mpu6050_enable_sleep(p_mpu6050, false);

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
    set_bits(p_mpu6050,
             ACCEL_CONFIG,
             (uint8_t) 0x3 << ACCEL_CONFIG_AFS_SEL_BIT,
             (uint8_t) range << ACCEL_CONFIG_AFS_SEL_BIT);

    p_mpu6050->accel_range = range;
}

void mpu6050_set_gyro_range(mpu6050_t * p_mpu6050, range_t range)
{
    set_bits(p_mpu6050,
             GYRO_CONFIG,
             (uint8_t) 0x3 << GYRO_CONFIG_FS_SEL_BIT,
             (uint8_t) range << GYRO_CONFIG_FS_SEL_BIT);
    p_mpu6050->gyro_range = range;
}

void mpu6050_enable_sleep(const mpu6050_t * p_mpu6050, bool enable)
{
    uint8_t bits = enable ? 1 << PWR_MGMT_SLEEP_BIT : 0;

    set_bits(p_mpu6050, PWR_MGMT_1, 1 << PWR_MGMT_SLEEP_BIT, bits);
}

void mpu6050_disable_temp_sensor(const mpu6050_t * p_mpu6050, bool disable)
{
    uint8_t bits = disable ? 1 << PWR_MGMT_TEMP_DIS_BIT : 0;

    set_bits(p_mpu6050, PWR_MGMT_1, 1 << PWR_MGMT_TEMP_DIS_BIT, bits);
}

bool mpu6050_test_connection(const mpu6050_t * p_mpu6050)
{
    uint8_t sendBuffer;
    uint8_t receiveBuffer;
    sendBuffer = WHO_AM_I;
    p_mpu6050->i2c_send(p_mpu6050->address, &sendBuffer, sizeof(sendBuffer));
    p_mpu6050->i2c_receive(p_mpu6050->address, &receiveBuffer, sizeof(receiveBuffer));

    return BASE_ADDRESS == receiveBuffer;
}

static void set_bits(const mpu6050_t * p_mpu6050,
                     uint8_t address,
                     uint8_t mask,
                     uint8_t bits)
{
    uint8_t buffer[2];

    buffer[0] = address;
    buffer[1] = 0;
    p_mpu6050->i2c_send(p_mpu6050->address, buffer, sizeof(buffer[0]));
    p_mpu6050->i2c_receive(p_mpu6050->address, &buffer[1], sizeof(buffer[1]));
    buffer[1] &= ~(mask);
    buffer[1] |= bits;
    p_mpu6050->i2c_send(p_mpu6050->address, buffer, sizeof(buffer));
}

/*** end of file ***/
