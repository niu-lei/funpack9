/**
 ******************************************************************************
 * @file       ab_signal.h
 * @author     MEMS Application Team
 * @brief      Header for ab_signal.c
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
#ifndef __AB_SIGNAL_H
#define __AB_SIGNAL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "cube_hal.h"
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define FALLING_EDGE  1 // negative zero crossing
#define RISING_EDGE   2 // positive zero crossing
#define NEGATIVE_PEAK 1
#define POSITIVE_PEAK 2

/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void PulseWidth(int32_t *in, int32_t *out, int32_t edge, int32_t *mem);
void PulseGenerator(int32_t *in, int32_t *out, int32_t time, int32_t *mem);
void MovingAverage(float *in, float *out, uint32_t length, float *mem1, int32_t *mem2);
void Counter(int32_t *in, int32_t *out, int32_t edge, int32_t *mem);

void FIR_Filter_Init(arm_fir_instance_f32 *pfilter, uint16_t numTaps, float32_t *pcoeffs, float32_t *pstate);
void FIR_Filter(arm_fir_instance_f32 *pfilter, float32_t *psrc, float32_t *pdst);
void IIR_Filter_Init(arm_biquad_casd_df1_inst_f32 *pfilter, uint8_t numStages, float32_t *pcoeffs, float32_t *pstate);
void IIR_Filter(arm_biquad_casd_df1_inst_f32 *pfilter, float32_t *psrc, float32_t *pdst);

void Integrator(float* in, float *out, float *mem);
void Derivator(float* in, float *out, float *mem);

void SignalSwitchInt(int32_t *in1, int32_t *in2, int32_t *control, int32_t *out);
void SignalSwitchFloat(float *in1, float *in2, int32_t *control, float *out);

void Feature_Computation(float *in, int32_t *reset, float *max, float *min, float *mean, float *var, float *peak, float *energy, uint32_t length, int32_t *mem, float *buffer);

void Feature_Computation_2_Init(float *buffer, int n_items);
void Feature_Computation_2(float *in, int32_t *reset, float *max, float *min, float *mean, float *var, float *peak, float *energy, int32_t *updated, uint32_t length, int32_t *mem, float *buffer);

void Signal_Delay(float* in, float *out, uint32_t length, float *mem1, int32_t *mem2);

void Zero_Crossing(float* in, int32_t *out, uint32_t edge, float *mem);
void Zero_Crossing_Hyst(float* in, int32_t *out, uint32_t edge, float hyst, int32_t *stat);
void Peak_Detector(float* in, int32_t *out, uint32_t peak, float threshold, float *mem);
void Peak_Detector_RelTh(float *in, int32_t *out, uint32_t peak, float threshold, int32_t reset, int32_t *stat, float *mem);
void Spike_Detector(float* in, int32_t *out, uint32_t peak, float threshold, float *mem);

#endif /* __AB_SIGNAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
