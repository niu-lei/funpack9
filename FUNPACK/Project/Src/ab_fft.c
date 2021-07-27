/**
 ******************************************************************************
 * @file       ab_user_input.c
 * @author     MEMS Application Team
 * @brief      AlgoBuilder FFT function block
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

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "ab_fft.h"
#include "ab_sensor_hub.h"
#include "main.h"

/** @addtogroup AlgoBuilder_Firmware
  * @{
  */

/** @addtogroup AlgoBuilder_FFT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern sensor_hub_data_t sensor_hub_data;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize FFT function block
 */
void FFT_Init(arm_rfft_fast_instance_f32 *fft_instance, uint16_t lenght)
{
  arm_rfft_fast_init_f32(fft_instance, lenght);
}

/**
 * @brief  FFT calculation function block
 */
void FFT_Calc(arm_rfft_fast_instance_f32 *fft_instance, float *in, float *out, int32_t *size, int32_t *full, float *input_buffer, int32_t *buffer_index, uint32_t window, uint16_t lenght)
{ 
  float fft_tmp[1024];
  float scale_factor;
  uint32_t i;
  
  *size = lenght;
  
  switch (window)
  {
    case 0:
      input_buffer[*buffer_index] = in[0];
    break;
    
    case 1:
      input_buffer[*buffer_index] = Hanning[*buffer_index * (1024 / lenght)] * in[0];
    break;
    
    case 2:
      input_buffer[*buffer_index] = Hamming[*buffer_index * (1024 / lenght)] * in[0];
    break;
    
    case 3:
      input_buffer[*buffer_index] = FlatTop[*buffer_index * (1024 / lenght)] * in[0];
    break;
    
    default:
      input_buffer[*buffer_index] = in[0];
    break;
  }

  if (*buffer_index == (lenght - 1))
  {
    arm_rfft_fast_f32(fft_instance, (float32_t*) input_buffer, (float32_t*) fft_tmp, 0U);
    arm_cmplx_mag_f32((float32_t*) fft_tmp, (float32_t*) out, lenght / 2);
    /* Correction of DC component accroding to FFT desciption (https://www.keil.com/pack/doc/CMSIS/DSP/html/group__RealFFT.html) */
    out[0] = fabs(fft_tmp[0]);
    
    switch (window)
    {
      case 0:
        scale_factor = 1000.0f * 1.0f;
      break;
      
      case 1:
        scale_factor = 1000.0f * HanningSF;
      break;
      
      case 2:
        scale_factor = 1000.0f * HammingSF;
      break;
      
      case 3:
        scale_factor = 1000.0f * FlatTopSF;
      break;
      
      default:
        scale_factor = 1000.0f * 1.0f;
      break;
    } 
    
    out[0] = 1 * out[0] / lenght * scale_factor;
    
    for (i = 1; i < lenght/2; i++)
    {
      out[i] = 2 * out[i] / lenght * scale_factor;
    }
    
    *buffer_index = 0;
    *full = 1;
  }
  else
  {
    *buffer_index = *buffer_index + 1;
    *full = 0;
  }
}

/**
 * @brief  Detection of maximum peak in the spectrum
 */
void FFT_Peak(float32_t *in, float *amplitude, float *frequency, uint16_t lenght, uint8_t exclude_dc)
{
  uint32_t index;
  
  if (exclude_dc == 0)
  {
    arm_max_f32(in, lenght/2, amplitude, &index);
  }
  else
  {
    arm_max_f32(in + 1, (lenght/2 - 1), amplitude, &index);
    index = index + 1;
  }
  
  *frequency = index * (float) sensor_hub_data.meas_data_rate_Hz / (float) lenght;
}

/**
 * @}
 */

/**
 * @}
 */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
