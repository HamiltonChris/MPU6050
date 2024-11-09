/** @file mpu6050.h
*
* @brief A driver for the mpu6050 accelerometer and gyroscope.
*
* @par
*
*/

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

#define BASE_ADDRESS                0x68

// Registers
#define GYRO_CONFIG                 0x27
#define ACCEL_CONFIG                0x28

#define ACCEL_XOUT_H                0x3B
#define ACCEL_XOUT_L                0x3C
#define ACCEL_YOUT_H                0x3D
#define ACCEL_YOUT_L                0x3E
#define ACCEL_ZOUT_H                0x3F
#define ACCEL_ZOUT_L                0x40

#define GYRO_XOUT_H                 0x43
#define GYRO_XOUT_L                 0x44
#define GYRO_YOUT_H                 0x45
#define GYRO_YOUT_L                 0x46
#define GYRO_ZOUT_H                 0x47
#define GYRO_ZOUT_L                 0x48

#define PWR_MGMT_1                  0x6B
#define PWR_MGMT_2                  0x6C

#define WHO_AM_I                    0x75

#define GYRO_CONFIG_FS_SEL_BIT      3
#define ACCEL_CONFIG_AFS_SEL_BIT    3

#define PWR_MGMT_SLEEP_BIT          6

typedef enum range_e
{
    SHORTSCALE      = 0,
    MEDIUMSCALE     = 1,
    LARGESCALE      = 2,
    EXTRALARGESCALE = 3,
} range_t;

typedef struct mpu6050_s
{
    uint8_t address;
    range_t accel_range;
    range_t gyro_range;
    void (*i2c_send)(uint8_t, uint8_t *, uint8_t);
    void (*i2c_receive)(uint8_t, uint8_t *, uint8_t);
} mpu6050_t;

void mpu6050_init(mpu6050_t * p_mpu6050, bool ad0_pin);
void mpu6050_getAcceleration(const mpu6050_t * p_mpu6050, int16_t * p_x, int16_t * p_y, int16_t * p_z);
void mpu6050_getRotation(const mpu6050_t * p_mpu6050, int16_t * p_x, int16_t * p_y, int16_t * p_z);
void mpu6050_set_accel_range(mpu6050_t * p_mpu6050, range_t range);
void mpu6050_set_gyro_range(mpu6050_t * p_mpu6050, range_t range);
void mpu6050_enable_sleep(const mpu6050_t * p_mpu6050, bool enable);
bool mpu6050_test_connection(const mpu6050_t * p_mpu6050);

#endif // MPU6050_H

/*** end of file ***/
