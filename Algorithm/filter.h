/**
  ******************************************************************************
  * @file    filter.h
  * @brief   一阶低通滤波算法（First-order IIR Low-Pass Filter）
  *
  * 差分方程：
  *    y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  *
  * 其中 alpha 由截止频率 fc 和采样频率 fs 决定（模拟 RC 等效离散化）：
  *    RC = 1 / (2*pi*fc)
  *    alpha = dt / (RC + dt) = 2*pi*fc*dt / (1 + 2*pi*fc*dt)
  *
  * alpha 越小，滤波越强、相位滞后越大；alpha 越大，越跟随原始信号。
  ******************************************************************************
  */
#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
  * @brief 一阶低通滤波器结构体
  */
typedef struct
{
    float   alpha;          /* 新采样权重，范围 0~1 */
    float   y;              /* 上一次滤波输出 */
    uint8_t initialized;    /* 首次更新直接采用输入值作为初值 */
} LowPassFilter_TypeDef;

/* 直接指定 alpha 初始化 */
void  Filter_LPF_InitAlpha(LowPassFilter_TypeDef *f, float alpha);

/* 按截止频率/采样频率初始化 */
void  Filter_LPF_InitCutoff(LowPassFilter_TypeDef *f, float cutoffHz, float sampleHz);

/* 输入一个新采样，返回滤波结果 */
float Filter_LPF_Update(LowPassFilter_TypeDef *f, float x);

/* 复位为指定值 */
void  Filter_LPF_Reset(LowPassFilter_TypeDef *f, float value);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */
