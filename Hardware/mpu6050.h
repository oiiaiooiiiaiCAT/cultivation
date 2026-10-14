/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 六轴传感器硬件驱动（硬件 I2C2 + HAL 库）
  *
  * 硬件连接：
  *   MPU6050 SCL -> PB10 (I2C2_SCL)
  *   MPU6050 SDA -> PB11 (I2C2_SDA)
  *   AD0 接地，7 位从机地址 0x68
  *
  * 全部读写均使用 STM32 硬件 I2C 外设（HAL_I2C_Mem_Read / Mem_Write），
  * 不使用任何软件模拟时序。
  ******************************************************************************
  */
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* MPU6050 7 位从机地址（AD0 = 0），HAL 传输时需左移 1 位 */
#define MPU6050_ADDR                0x68U
#define MPU6050_I2C_TIMEOUT_MS      5U

/* MPU6050 寄存器地址 */
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_TEMP_OUT_H      0x41U
#define MPU6050_REG_GYRO_XOUT_H     0x43U
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_PWR_MGMT_2      0x6CU
#define MPU6050_REG_WHO_AM_I        0x75U

/* 量程配置（本驱动固定使用：加速度 ±8g，陀螺仪 ±2000dps） */
#define MPU6050_ACC_SCALE_G         8.0f    /* ±8g  -> 4096 LSB/g   */
#define MPU6050_GYRO_SCALE_DPS      2000.0f /* ±2000 dps -> 16.4 LSB/(dps) */
#define MPU6050_ACC_LSB_PER_G       4096.0f
#define MPU6050_GYRO_LSB_PER_DPS    16.4f

/**
  * @brief MPU6050 姿态与融合数据，不保存通信句柄及未使用的原始/温度数据
  */
typedef struct
{
    float acc_g[3];              /* 三轴加速度，g；用于 pitch/roll 融合。 */
    float gyro_dps[3];           /* 三轴角速度，deg/s；未去零偏。 */
    float gyroBias[3];           /* 静止标定零偏，deg/s。 */
    float pitch;                 /* 融合俯仰角，deg；包含安装补偿。 */
    float roll;                  /* 融合横滚角，deg；用于倾倒保护和显示。 */
    float yaw;                   /* 陀螺积分偏航角，deg；用于现有显示。 */
    uint32_t lastUpdate;          /* 姿态更新时间，HAL tick/ms。 */
    uint8_t initialized;          /* 初始化及静止标定有效。 */
    uint8_t healthy;              /* 最近一次姿态更新成功。 */
} MPU6050_TypeDef;

#define MPU6050_PITCH_OFFSET_DEG 4.0f

extern MPU6050_TypeDef g_mpu6050;

/* 初始化 MPU6050（配置量程、DLPF、采样率并解除休眠） */
HAL_StatusTypeDef MPU6050_Init(MPU6050_TypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);

/* 读取 WHO_AM_I（正常返回 0x68） */
uint8_t MPU6050_GetID(MPU6050_TypeDef *dev);

/* 硬件 I2C 突发读取加速度、温度、陀螺仪共 14 字节，并完成工程量换算 */
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_TypeDef *dev);

/* 静止状态下标定陀螺仪零偏（标定期间车体必须保持静止、水平） */
void MPU6050_CalibrateGyro(MPU6050_TypeDef *dev, uint16_t samples);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
