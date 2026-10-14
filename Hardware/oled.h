/**
  ******************************************************************************
  * @file    oled.h
  * @brief   0.96 寸 OLED(SSD1306, 128x64) 硬件驱动（硬件 I2C1 + HAL 库）
  *
  * 硬件连接：
  *   OLED SCL -> PB8 (I2C1_SCL，已在 CubeMX 中重映射)
  *   OLED SDA -> PB9 (I2C1_SDA，已在 CubeMX 中重映射)
  *
  * 全部数据/命令均通过 STM32 硬件 I2C1 外设发送
  * （HAL_I2C_Master_Transmit），不使用软件模拟时序。
  ******************************************************************************
  */
#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* SSD1306 7 位从机地址 0x3C（HAL 传输时左移 1 位 = 0x78） */
#define OLED_ADDR                   0x3CU
#define OLED_I2C_TIMEOUT_MS         50U

/* 硬件 I2C 单包最多发送字节数（控制字节 + 数据），留足余量取 32 */
#define OLED_I2C_BUF_SIZE           32U

/**
  * @brief OLED 页面状态（当前仅使用一块 OLED）
  */
typedef struct
{
    uint8_t displayMode;          /* 仅保留页面状态，I2C 通信由驱动内部管理。 */
} OLED_TypeDef;

extern OLED_TypeDef g_oled;

/* 初始化 OLED */
void OLED_Init(OLED_TypeDef *oled, I2C_HandleTypeDef *hi2c);

/* 清屏 */
void OLED_Clear(OLED_TypeDef *oled);

/* 设置光标：Page 0~7（行），X 0~127（列） */
void OLED_SetCursor(OLED_TypeDef *oled, uint8_t page, uint8_t x);

/* 字符/字符串显示，Line 1~4，Column 1~16 */
void OLED_ShowChar(OLED_TypeDef *oled, uint8_t line, uint8_t column, char ch);
void OLED_ShowString(OLED_TypeDef *oled, uint8_t line, uint8_t column, const char *str);

/* 数字显示 */
void OLED_ShowNum(OLED_TypeDef *oled, uint8_t line, uint8_t column, uint32_t num, uint8_t len);
void OLED_ShowSignedNum(OLED_TypeDef *oled, uint8_t line, uint8_t column, int32_t num, uint8_t len);
void OLED_ShowHexNum(OLED_TypeDef *oled, uint8_t line, uint8_t column, uint32_t num, uint8_t len);

/* 浮点数显示：IntLen 整数位长度(不含符号)，DecLen 小数位长度 */
void OLED_ShowFloat(OLED_TypeDef *oled, uint8_t line, uint8_t column,
                    float num, uint8_t intLen, uint8_t decLen);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
