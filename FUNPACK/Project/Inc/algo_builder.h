/**
 ******************************************************************************
 * @file       algo_builder.h
 * @author     MEMS Application Team
 * @brief      Header for algo_builder.c
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
#ifndef __ALGO_BUILDER_H
#define __ALGO_BUILDER_H

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "main.h"
#include "serial_protocol.h"
#include "sensor_commands.h"
#include "arm_math.h"
#include "ab_buffers.h"
#include "ab_display.h"
#include "ab_fft.h"
#include "ab_sensor_hub.h"
#include "ab_sequential_logic.h"
#include "ab_signal.h"
#include "ab_user_input.h"

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define MODULO %

/* Exported defines ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern volatile int Message_Length;

/* Exported functions ------------------------------------------------------- */
void AB_Init(void);
void AB_Handler(void);

#endif /* __ALGO_BUILDER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
