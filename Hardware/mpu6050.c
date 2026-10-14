/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 六轴传感器硬件驱动（硬件 I2C2 + HAL 库）
  ******************************************************************************
  */
#include "mpu6050.h"

MPU6050_TypeDef g_mpu6050;
#include <string.h>

/**
  * @brief  写单个寄存器（硬件 I2C）
  */
static HAL_StatusTypeDef MPU6050_WriteReg(MPU6050_TypeDef *dev, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->address << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &data, 1, MPU6050_I2C_TIMEOUT_MS);
}

/**
  * @brief  从指定寄存器开始突发读取 len 个字节（硬件 I2C）
  */
static HAL_StatusTypeDef MPU6050_ReadRegs(MPU6050_TypeDef *dev, uint8_t reg,
                                          uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->address << 1), reg,
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
    dev->hi2c = hi2c;
    dev->address = addr;
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
    /* ACCEL_CONFIG = 0x10：加速度 ±8g */
    if (MPU6050_WriteReg(dev, MPU6050_REG_ACCEL_CONFIG, 0x10) != HAL_OK) return HAL_ERROR;

    dev->gyroBias[0] = 0.0f;
    dev->gyroBias[1] = 0.0f;
    dev->gyroBias[2] = 0.0f;

    dev->whoAmI = MPU6050_GetID(dev);
    if (dev->whoAmI != 0x68U) return HAL_ERROR;
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

    dev->accRaw[0]  = (int16_t)((buf[0]  << 8) | buf[1]);
    dev->accRaw[1]  = (int16_t)((buf[2]  << 8) | buf[3]);
    dev->accRaw[2]  = (int16_t)((buf[4]  << 8) | buf[5]);
    dev->tempRaw    = (int16_t)((buf[6]  << 8) | buf[7]);
    dev->gyroRaw[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    dev->gyroRaw[1] = (int16_t)((buf[10] << 8) | buf[11]);
    dev->gyroRaw[2] = (int16_t)((buf[12] << 8) | buf[13]);

    dev->acc_g[0] = (float)dev->accRaw[0] / MPU6050_ACC_LSB_PER_G;
    dev->acc_g[1] = (float)dev->accRaw[1] / MPU6050_ACC_LSB_PER_G;
    dev->acc_g[2] = (float)dev->accRaw[2] / MPU6050_ACC_LSB_PER_G;

    dev->gyro_dps[0] = (float)dev->gyroRaw[0] / MPU6050_GYRO_LSB_PER_DPS;
    dev->gyro_dps[1] = (float)dev->gyroRaw[1] / MPU6050_GYRO_LSB_PER_DPS;
    dev->gyro_dps[2] = (float)dev->gyroRaw[2] / MPU6050_GYRO_LSB_PER_DPS;

    dev->temp_c = (float)dev->tempRaw / 340.0f + 36.53f;

    return HAL_OK;
}

/**
  * @brief  陀螺仪零偏标定（静止、水平状态下调用）
  * @param  samples : 采样次数，建议 1000~2000
  */
void MPU6050_CalibrateGyro(MPU6050_TypeDef *dev, uint16_t samples)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t valid = 0;
    uint16_t i;

    if (dev == NULL || !dev->initialized || samples == 0U) return;
    for (i = 0; i < samples; i++)
    {
        if (MPU6050_ReadAll(dev) == HAL_OK)
        {
            sum[0] += dev->gyro_dps[0];
            sum[1] += dev->gyro_dps[1];
            sum[2] += dev->gyro_dps[2];
            valid++;
        }
        HAL_Delay(5);
    }

    if (valid < (uint32_t)samples * 4U / 5U || valid == 0U)
    {
        dev->initialized = 0U;
        return;
    }
    if (valid > 0)
    {
        dev->gyroBias[0] = sum[0] / valid;
        dev->gyroBias[1] = sum[1] / valid;
        dev->gyroBias[2] = sum[2] / valid;
    }
}
