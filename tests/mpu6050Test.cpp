/** @file mpu6050Test.cpp
*
* @brief Unit tests for the mpu6050 driver module.
*
* @par
*
*/ 

extern "C"
{
#include "mpu6050.h"
}

#include <string.h>

#include "CppUTest/TestHarness.h"

mpu6050_t *p_mpu6050;
static uint8_t registers[WHO_AM_I + 1];
static uint8_t currentRegister;

static void dummy_send(uint8_t address, uint8_t * buffer, uint8_t size);
static void dummy_receive(uint8_t address, uint8_t * buffer, uint8_t size);

TEST_GROUP(MPU6050)
{
    void setup()
    {
        memset(registers, 0, sizeof(registers));
        currentRegister = 0;

        registers[WHO_AM_I] = BASE_ADDRESS;

        p_mpu6050 = (mpu6050_t*)malloc(sizeof(mpu6050_t));
        p_mpu6050->i2c_send = &dummy_send;
        p_mpu6050->i2c_receive = &dummy_receive;
        mpu6050_init(p_mpu6050, 0);
    }

    void teardown()
    {
        free(p_mpu6050);
        p_mpu6050 = NULL;
    }
};

TEST(MPU6050, init)
{
    mpu6050_t mpu6050;
    uint8_t AD0pin = 0;

    mpu6050.address = 0xFF;
    mpu6050.accel_range = EXTRALARGESCALE;
    mpu6050.gyro_range = LARGESCALE;
    mpu6050.i2c_send = &dummy_send;
    mpu6050.i2c_receive = &dummy_receive;

    mpu6050_init(&mpu6050, AD0pin);

    BYTES_EQUAL(BASE_ADDRESS, mpu6050.address);
    BYTES_EQUAL(EXTRALARGESCALE << 3, registers[ACCEL_CONFIG]);
    BYTES_EQUAL(LARGESCALE << 3, registers[GYRO_CONFIG]);
}

TEST(MPU6050, AD0High)
{
    mpu6050_t mpu6050;
    uint8_t ad0_pin = 1;

    mpu6050.address = 0;
    mpu6050.i2c_send = &dummy_send;
    mpu6050.i2c_receive = &dummy_receive;

    mpu6050_init(&mpu6050, ad0_pin);

    BYTES_EQUAL(BASE_ADDRESS + 1, mpu6050.address);
}

TEST(MPU6050, ReadAccelerometer)
{
    int16_t x = 0xFFFF;
    int16_t y = 0xFFFF;
    int16_t z = 0xFFFF;

    mpu6050_getAcceleration(p_mpu6050, &x, &y, &z);

    LONGS_EQUAL(0, x);
    LONGS_EQUAL(0, y);
    LONGS_EQUAL(0, z);
}

TEST(MPU6050, ReadAccMaxRanges)
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    registers[ACCEL_XOUT_H] = 0xFF;
    registers[ACCEL_XOUT_L] = 0xFF; 
    registers[ACCEL_YOUT_H] = 0x80;
    registers[ACCEL_YOUT_L] = 0x00;
    registers[ACCEL_ZOUT_H] = 0x7F;
    registers[ACCEL_ZOUT_L] = 0xFF;

    mpu6050_getAcceleration(p_mpu6050, &x, &y, &z);

    LONGS_EQUAL(-1, x);
    LONGS_EQUAL(-32768, y);
    LONGS_EQUAL(32767, z);
}

TEST(MPU6050, ReadGyroscope)
{
    int16_t x = 0xFFFF;
    int16_t y = 0xFFFF;
    int16_t z = 0xFFFF;

    mpu6050_getRotation(p_mpu6050, &x, &y, &z);

    LONGS_EQUAL(0, x);
    LONGS_EQUAL(0, y);
    LONGS_EQUAL(0, z);
}


TEST(MPU6050, ReadGyroMaxRanges)
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    registers[GYRO_XOUT_H] = 0xFF;
    registers[GYRO_XOUT_L] = 0xFF; 
    registers[GYRO_YOUT_H] = 0x80;
    registers[GYRO_YOUT_L] = 0x00;
    registers[GYRO_ZOUT_H] = 0x7F;
    registers[GYRO_ZOUT_L] = 0xFF;

    mpu6050_getRotation(p_mpu6050, &x, &y, &z);

    LONGS_EQUAL(-1, x);
    LONGS_EQUAL(-32768, y);
    LONGS_EQUAL(32767, z);
}

TEST(MPU6050, AccelerometerScale)
{
    mpu6050_set_accel_range(p_mpu6050, LARGESCALE);

    BYTES_EQUAL(LARGESCALE << 3, registers[ACCEL_CONFIG]);
    BYTES_EQUAL(LARGESCALE, p_mpu6050->accel_range);
}

TEST(MPU6050, GyroscopeScale)
{
    mpu6050_set_gyro_range(p_mpu6050, MEDIUMSCALE);

    BYTES_EQUAL(MEDIUMSCALE << 3, registers[GYRO_CONFIG]);
    BYTES_EQUAL(MEDIUMSCALE, p_mpu6050->gyro_range);
}

TEST(MPU6050, TestConnection)
{
    CHECK_TRUE(mpu6050_test_connection(p_mpu6050));
}

static void dummy_send(uint8_t address, uint8_t * buffer, uint8_t size)
{
    if (size && buffer)
    {
        currentRegister = buffer[0];
        for (uint8_t i = 1; i < size; i++)
        {
            registers[currentRegister++] = buffer[i];
        }
    }
}

static void dummy_receive(uint8_t address, uint8_t * buffer, uint8_t size)
{
    if (buffer)
    {
        for (uint8_t i = 0; i < size; i++)
        {
            buffer[i] = registers[currentRegister++];
        }
    }
}

/*** end of file ***/
