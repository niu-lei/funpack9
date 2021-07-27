/**
 ******************************************************************************
 * @file    main.h
 * @author  MEMS Software Solutions Team
 * @brief   This file contains definitions for the main.c file.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
  
#if (defined (USE_IKS01A2))
  
  #if (defined (USE_STM32F4XX_NUCLEO))
  #include "nucleo_f401re_bus.h"
  #include "nucleo_f401re_errno.h"
    
  #elif (defined (USE_STM32L4XX_NUCLEO))
  #include "nucleo_l476rg_bus.h"
  #include "nucleo_l476rg_errno.h"
  #endif

#include "iks01a2_env_sensors.h"
#include "iks01a2_env_sensors_ex.h"
#include "iks01a2_motion_sensors.h"
#include "iks01a2_motion_sensors_ex.h"

#elif (defined (USE_IKS01A3))
  
  #if (defined (USE_STM32F4XX_NUCLEO))
  #include "nucleo_f401re_bus.h"
  #include "nucleo_f401re_errno.h"
  
  #elif (defined (USE_STM32L4XX_NUCLEO))
  #include "nucleo_l476rg_bus.h"
  #include "nucleo_l476rg_errno.h"
  #endif
  
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"
#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
  
#elif (defined (USE_SENSORTILE))
  
#include "sensortile_bus.h"
#include "sensortile_errno.h"
  
#include "SensorTile_env_sensors.h"
#include "SensorTile_env_sensors_ex.h"
#include "SensorTile_motion_sensors.h"
#include "SensorTile_motion_sensors_ex.h"
  
#elif (defined (USE_SENSORTILEBOX))
  
#include "SensorTile.box_bus.h"
#include "SensorTile.box_errno.h"
  
#include "SensorTile.box_env_sensors.h"
#include "SensorTile.box_env_sensors_ex.h"
#include "SensorTile.box_motion_sensors.h"
#include "SensorTile.box_motion_sensors_ex.h"
  
#elif (defined (USE_STWIN))
  
#include "STWIN_bus.h"
#include "STWIN_errno.h"
  
#include "STWIN_env_sensors.h"
#include "STWIN_env_sensors_ex.h"
#include "STWIN_motion_sensors.h"
#include "STWIN_motion_sensors_ex.h"

#else
#error Not supported platform
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct customValues_s {
  uint8_t cust_bits;
  int32_t cust_ints[4];
  float cust_floats[4];
} customValues_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/

/* Timer used for AlgoBuilder handler */
#define TIM_AB                          TIM3
#define TIM_AB_CLK_ENABLE               __TIM3_CLK_ENABLE
#define TIM_AB_CLK_DISABLE              __TIM3_CLK_DISABLE
/* Definition for TIMx's NVIC */
#define TIM_AB_IRQn                     TIM3_IRQn
#define TIM_AB_IRQHandler               TIM3_IRQHandler

/* Timer used for Algorithm handlers */
#define TIM_ALGO                        TIM1
#define TIM_ALGO_CLK_ENABLE             __TIM1_CLK_ENABLE
#define TIM_ALGO_CLK_DISABLE            __TIM1_CLK_DISABLE
/* Definition for TIMx's NVIC */
#define TIM_ALGO_IRQn                   TIM1_CC_IRQn
#define TIM_ALGO_IRQHandler             TIM1_CC_IRQHandler

/* Enable sensor masks */
#define PRESSURE_SENSOR                 ((uint32_t)0x00000001)
#define TEMPERATURE_SENSOR              ((uint32_t)0x00000002)
#define HUMIDITY_SENSOR                 ((uint32_t)0x00000004)
#define ACCELEROMETER_SENSOR            ((uint32_t)0x00000010)
#define GYROSCOPE_SENSOR                ((uint32_t)0x00000020)
#define MAGNETIC_SENSOR                 ((uint32_t)0x00000040)

/* Errors reported to application */
#define AB_ERROR_NONE                   ((uint32_t)0)
#define AB_ERROR_ODR_CHANGED            ((uint32_t)0x00000001)
#define AB_ERROR_FS_CHANGED             ((uint32_t)0x00000002)

#define AB_ERROR_FLAG_OVERRUN           ((uint32_t)0x80000000)

#define AB_ERROR_CATEGORY_ERROR         ((uint32_t)0x08000000)
#define AB_ERROR_CATEGORY_WARNING       ((uint32_t)0x04000000)
#define AB_ERROR_CATEGORY_LOG           ((uint32_t)0x02000000)
#define AB_ERROR_CATEGORY_DEBUG         ((uint32_t)0x01000000)

/* Exported variables --------------------------------------------------------*/
extern uint64_t StartTime;

/* Exported functions ------------------------------------------------------- */
void RTC_DateTimeRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t hh, uint8_t mm, uint8_t ss);
void SerializeToMsg(uint8_t Dest, void *Source, uint32_t Len);
void EnterDFU(void);
void TIM_AB_Config(uint32_t freq);
void TIM_AB_Start(void);
void TIM_AB_Stop(void);
void TIM_AL_Start(void);
void TIM_AL_Stop(void);
uint64_t DWT_GetTickUS(void);
void Error_Handler(void);
#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
