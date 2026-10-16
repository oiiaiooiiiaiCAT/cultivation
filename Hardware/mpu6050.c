/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 六轴传感器硬件驱动（硬件 I2C2 + HAL 库）
  ******************************************************************************
  */
#include "mpu6050.h"

#include <math.h>
#include <string.h>

MPU6050_TypeDef g_mpu6050;
/* 当前仅使用一颗 MPU6050，通信配置由驱动内部保存。 */
static I2C_HandleTypeDef *mpuI2c;
static uint8_t mpuAddress;

/**
  * @brief  写单个寄存器（硬件 I2C）
  */
static HAL_StatusTypeDef MPU6050_WriteReg(MPU6050_TypeDef *dev, uint8_t reg, uint8_t data)
{
    if (dev == NULL || mpuI2c == NULL) return HAL_ERROR;
    return HAL_I2C_Mem_Write(mpuI2c, (uint16_t)(mpuAddress << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, MPU6050_I2C_TIMEOUT_MS);
}

/**
  * @brief  从指定寄存器开始突发读取 len 个字节（硬件 I2C）
  */
static HAL_StatusTypeDef MPU6050_ReadRegs(MPU6050_TypeDef *dev, uint8_t reg,
                                          uint8_t *buf, uint16_t len)
{
    if (dev == NULL || mpuI2c == NULL) return HAL_ERROR;
    return HAL_I2C_Mem_Read(mpuI2c, (uint16_t)(mpuAddress << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, MPU6050_I2C_TIMEOUT_MS);
}

/**
  * @brief  初始化 MPU6050
  * @param  dev  : 设备结构体
  * @param  hi2c : 硬件 I2C 句柄，本工程为 &hi2c2
  * @param  addr : 7 位从机地址，AD0 接地为 0x68
  */
HAL_StatusTypeDef MPU6050_Init(MPU6050_TypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    if (dev == NULL || hi2c == NULL || (addr != 0x68U && addr != 0x69U)) return HAL_ERROR;
    memset(dev, 0, sizeof(*dev));
    mpuI2c = hi2c;
    mpuAddress = addr;
    dev->initialized = 0;

    /* 复位 MPU6050 */
    if (MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x80) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(100);

    /* PWR_MGMT_1 = 0x01：选择 X 轴陀螺仪作为时钟源，退出休眠 */
    if (MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) return HAL_ERROR;
    /* PWR_MGMT_2 = 0x00：所有轴正常工作 */
    if (MPU6050_WriteReg(dev, MPU6050_REG_PWR_MGMT_2, 0x00) != HAL_OK) return HAL_ERROR;
    /* 采样率分频：1kHz/(1+4) = 200Hz */
    if (MPU6050_WriteReg(dev, MPU6050_REG_SMPLRT_DIV, 0x04) != HAL_OK) return HAL_ERROR;
    /* CONFIG = 0x03：DLPF 44Hz，延时约 4.9ms，兼顾噪声与相位滞后 */
    if (MPU6050_WriteReg(dev, MPU6050_REG_CONFIG, 0x03) != HAL_OK) return HAL_ERROR;
    /* GYRO_CONFIG = 0x18：陀螺仪 ±2000dps */
    if (MPU6050_WriteReg(dev, MPU6050_REG_GYRO_CONFIG, 0x18) != HAL_OK) return HAL_ERROR;
    /* ACCEL_CONFIG = 0x08：加速度 ±4g */
    if (MPU6050_WriteReg(dev, MPU6050_REG_ACCEL_CONFIG, 0x08) != HAL_OK) return HAL_ERROR;

    dev->gyroBias[0] = 0.0f;
    dev->gyroBias[1] = 0.0f;
    dev->gyroBias[2] = 0.0f;

    if (MPU6050_GetID(dev) != 0x68U) return HAL_ERROR;
    dev->initialized = 1;

    return HAL_OK;
}

/**
  * @brief  读取 WHO_AM_I
  */
uint8_t MPU6050_GetID(MPU6050_TypeDef *dev)
{
    uint8_t id = 0;
    if (MPU6050_ReadRegs(dev, MPU6050_REG_WHO_AM_I, &id, 1) != HAL_OK)
    {
        return 0;
    }
    return id;
}

/**
  * @brief  读取全部数据并换算工程量
  *         一次硬件 I2C 突发读取 0x3B 起 14 字节：
  *         AccX/Y/Z(6) + Temp(2) + GyroX/Y/Z(6)
  */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_TypeDef *dev)
{
    uint8_t buf[14];
    if (dev == NULL || !dev->initialized) return HAL_ERROR;
    if (MPU6050_ReadRegs(dev, MPU6050_REG_ACCEL_XOUT_H, buf, 14) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 直接换算需要的六轴数据；温度字节随突发读取但不保存。 */
    dev->acc_g[0] = (float)(int16_t)((buf[0] << 8) | buf[1]) / MPU6050_ACC_LSB_PER_G;
    dev->acc_g[1] = (float)(int16_t)((buf[2] << 8) | buf[3]) / MPU6050_ACC_LSB_PER_G;
    dev->acc_g[2] = (float)(int16_t)((buf[4] << 8) | buf[5]) / MPU6050_ACC_LSB_PER_G;
    dev->gyro_dps[0] = (float)(int16_t)((buf[8] << 8) | buf[9]) / MPU6050_GYRO_LSB_PER_DPS;
    dev->gyro_dps[1] = (float)(int16_t)((buf[10] << 8) | buf[11]) / MPU6050_GYRO_LSB_PER_DPS;
    dev->gyro_dps[2] = (float)(int16_t)((buf[12] << 8) | buf[13]) / MPU6050_GYRO_LSB_PER_DPS;

    return HAL_OK;
}

/**
  * @brief  静止采样，检查波动及重力幅值，仅成功时提交三轴零偏
  * @param  samples : 采样次数，至少2次，启动默认500次
  */
HAL_StatusTypeDef MPU6050_CalibrateGyro(MPU6050_TypeDef *dev, uint16_t samples)
{
    float gyroMean[3] = {0.0f, 0.0f, 0.0f};
    float gyroVarianceSum[3] = {0.0f, 0.0f, 0.0f};
    float accMean[3] = {0.0f, 0.0f, 0.0f};
    float accVarianceSum[3] = {0.0f, 0.0f, 0.0f};
    float gyroMinimum[3];
    float gyroMaximum[3];
    uint16_t sampleIndex;
    uint8_t axis;
    uint8_t stationary = 1U;

    if (dev == NULL) return HAL_ERROR;
    dev->gyroCalibrated = 0U;
    if (!dev->initialized || samples < 2U) return HAL_ERROR;
    for (sampleIndex = 0U; sampleIndex < samples; sampleIndex++)
    {
        float accelerationSquared = 0.0f;
        HAL_Delay(MPU6050_CAL_PERIOD_MS);
        if (MPU6050_ReadAll(dev) != HAL_OK) return HAL_ERROR;
        for (axis = 0U; axis < 3U; axis++)
        {
            float gyro = dev->gyro_dps[axis];
            float acceleration = dev->acc_g[axis];
            float gyroDelta;
            float accDelta;
            if (!isfinite(gyro) || !isfinite(acceleration)) return HAL_ERROR;
            if (fabsf(gyro) > MPU6050_CAL_GYRO_MAX_DPS) stationary = 0U;
            accelerationSquared += acceleration * acceleration;
            if (sampleIndex == 0U)
            {
                gyroMinimum[axis] = gyro;
                gyroMaximum[axis] = gyro;
            }
            if (gyro < gyroMinimum[axis]) gyroMinimum[axis] = gyro;
            if (gyro > gyroMaximum[axis]) gyroMaximum[axis] = gyro;
            gyroDelta = gyro - gyroMean[axis];
            gyroMean[axis] += gyroDelta / (float)(sampleIndex + 1U);
            gyroVarianceSum[axis] += gyroDelta * (gyro - gyroMean[axis]);
            accDelta = acceleration - accMean[axis];
            accMean[axis] += accDelta / (float)(sampleIndex + 1U);
            accVarianceSum[axis] += accDelta * (acceleration - accMean[axis]);
        }
        if (accelerationSquared < MPU6050_CAL_ACC_MIN_G * MPU6050_CAL_ACC_MIN_G ||
            accelerationSquared > MPU6050_CAL_ACC_MAX_G * MPU6050_CAL_ACC_MAX_G)
            stationary = 0U;
    }
    for (axis = 0U; axis < 3U; axis++)
    {
        float gyroVariance = gyroVarianceSum[axis] / (float)(samples - 1U);
        float accVariance = accVarianceSum[axis] / (float)(samples - 1U);
        if (!isfinite(gyroMean[axis]) || !isfinite(gyroVariance) ||
            !isfinite(accVariance)) return HAL_ERROR;
        if (gyroVariance > MPU6050_CAL_GYRO_STD_DPS * MPU6050_CAL_GYRO_STD_DPS ||
            accVariance > MPU6050_CAL_ACC_STD_G * MPU6050_CAL_ACC_STD_G ||
            gyroMaximum[axis] - gyroMinimum[axis] > MPU6050_CAL_GYRO_RANGE_DPS)
            stationary = 0U;
    }
    if (!stationary) return HAL_BUSY;
    memcpy(dev->gyroBias, gyroMean, sizeof(gyroMean));
    dev->gyroCalibrated = 1U;
    return HAL_OK;
}
