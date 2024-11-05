
#ifndef MPU6050_H
#define MPU6050_H

#define BASE_ADDRESS 0x68

// Registers
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define WHO_AM_I     0x75

#include <stdint.h>

typedef struct MPU6050_s
{
    uint8_t address;
    void (*I2C_Send)(uint8_t, uint8_t*, uint8_t);
    void (*I2C_Receive)(uint8_t, uint8_t*, uint8_t);
} MPU6050_t;

void MPU6050_init(MPU6050_t *pMPU6050, uint8_t AD0pin);
void MPU6050_getAcceleration(MPU6050_t *pMPU6050, int16_t *px, int16_t *y, int16_t *pz);


#endif // MPU6050_H