
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define BASE_ADDRESS 0x68

// Registers
#define GYRO_CONFIG  0x27
#define ACCEL_CONFIG 0x28

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define WHO_AM_I     0x75

typedef enum range_e
{
    SHORTSCALE      = 0,
    MEDIUMSCALE     = 1,
    LARGESCALE      = 2,
    EXTRALARGESCALE = 3,
} range_t;

typedef struct MPU6050_s
{
    uint8_t address;
    range_t accel_range;
    range_t gyro_range;
    void (*I2C_Send)(uint8_t, uint8_t*, uint8_t);
    void (*I2C_Receive)(uint8_t, uint8_t*, uint8_t);
} MPU6050_t;

void MPU6050_init(MPU6050_t *pMPU6050, uint8_t AD0pin);
void MPU6050_getAcceleration(MPU6050_t *pMPU6050, int16_t *px, int16_t *py, int16_t *pz);
void MPU6050_getRotation(MPU6050_t *pMPU6050, int16_t *px, int16_t *py, int16_t *pz);
void mpu6050_set_accel_range(MPU6050_t * pMPU6050, range_t range);
void mpu6050_set_gyro_range(MPU6050_t * pMPU6050, range_t range);

#endif // MPU6050_H