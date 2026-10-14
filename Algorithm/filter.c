/**
  ******************************************************************************
  * @file    filter.c
  * @brief   一阶低通滤波算法
  ******************************************************************************
  */
#include "filter.h"

/* 圆周率常量（不依赖 math.h 的宏） */
#define FILTER_PI                 3.14159265358979f

/**
  * @brief  直接指定 alpha 初始化
  */
void Filter_LPF_InitAlpha(LowPassFilter_TypeDef *f, float alpha)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }
    f->alpha = alpha;
    f->y = 0.0f;
    f->initialized = 0;
}

/**
  * @brief  按截止频率与采样频率初始化
  * @param  cutoffHz : 截止频率 Hz
  * @param  sampleHz : 采样频率 Hz（即该滤波器 Update 的调用频率）
  */
void Filter_LPF_InitCutoff(LowPassFilter_TypeDef *f, float cutoffHz, float sampleHz)
{
    float dt;
    float rc;
    float alpha;

    if (cutoffHz <= 0.0f || sampleHz <= 0.0f)
    {
        Filter_LPF_InitAlpha(f, 1.0f);
        return;
    }

    dt = 1.0f / sampleHz;
    rc = 1.0f / (2.0f * FILTER_PI * cutoffHz);
    alpha = dt / (rc + dt);

    Filter_LPF_InitAlpha(f, alpha);
}

/**
  * @brief  一阶低通滤波更新
  *         y[n] = alpha*x[n] + (1-alpha)*y[n-1]
  */
float Filter_LPF_Update(LowPassFilter_TypeDef *f, float x)
{
    if (!f->initialized)
    {
        f->y = x;
        f->initialized = 1;
        return x;
    }

    f->y = f->alpha * x + (1.0f - f->alpha) * f->y;
    return f->y;
}

/**
  * @brief  复位滤波器
  */
void Filter_LPF_Reset(LowPassFilter_TypeDef *f, float value)
{
    f->y = value;
    f->initialized = 1;
}
