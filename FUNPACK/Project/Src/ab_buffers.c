/**
 ******************************************************************************
 * @file       ab_buffers.c
 * @author     MEMS Application Team
 * @brief      AlgoBuilder buffer function blocks 
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
#include "ab_buffers.h"
#include "cube_hal.h"

/** @addtogroup AlgoBuilder_Firmware
  * @{
  */

/** @addtogroup AlgoBuilder_Buffers
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Stores data into linear buffer
 */
void LinearBuffer(float *in, float *out, int32_t *size, int32_t *full, int32_t *mem, int32_t buffer_size)
{
  *size = buffer_size;
  out[*mem] = in[0];
  
  if (*mem == (buffer_size - 1))
  {
    *mem = 0;
    *full = 1;
  }
  else
  {
    *mem = *mem + 1;
    *full = 0;
  }
}

/**
 * @brief  Stores data into circular buffer
 */
void CircularBuffer(float *in, float *out, int32_t *size, int32_t *full, int32_t *mem, int32_t buffer_size)
{
  *size = buffer_size;
  out[*mem] = in[0];
  *mem = (*mem + 1) % buffer_size;
  
  if (*mem == (buffer_size - 1))
  {
    *full = 1;
  }
}
 
 /**
 * @brief  Extract part of data from input buffer
 */
void SubBuffer(float *in, int32_t in_size, float *out, int32_t out_size, int32_t start)
{
  if ((start + out_size) > in_size) return;
  
  for (uint32_t i = start; i < (start + out_size); i++)
  {
    out[i-start] = in[i];
  }
}
/**
 * @}
 */

/**
 * @}
 */
 
 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
