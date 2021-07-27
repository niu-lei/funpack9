/**
 ******************************************************************************
 * @file       ab_sensor_hub.c
 * @author     MEMS Application Team
 * @brief      AlgoBuilder signal processing functions
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
#include "ab_signal.h"
#include "ab_sensor_hub.h"
#include "cube_hal.h"
#include "float.h"

/** @addtogroup AlgoBuilder_Firmware
  * @{
  */

/** @addtogroup AlgoBuilder_Signal_Processing
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
 * @brief  Measures pulse width
 */
void PulseWidth(int32_t *in, int32_t *out, int32_t edge, int32_t *mem)
{
  if (mem[0] == 0)
  {
    if (in[0] == edge)
    {
      mem[0] = 1;
      mem[1] = HAL_GetTick();
    }
    *out = 0;
  }
  else if (mem[0] == 1)
  {
    if (in[0] == !edge)
    {
      *out = HAL_GetTick() - mem[1];
      mem[0] = 0;
      mem[1] = 0;
    }
	else
	{
	  *out = 0;
	}
  }
}

/**
 * @brief  Generated pulse with selected width
 */
void PulseGenerator(int32_t *in, int32_t *out, int32_t time, int32_t *mem)
{
  uint32_t delay;

  if (mem[0] == 0)
  {
	if (in[0] != 0)
	{
	  mem[0] = 1;
	  mem[1] = HAL_GetTick();
	  *out = 1;
	}
  }
  else if (mem[0] == 1)
  {
	delay = HAL_GetTick() - mem[1];

	if (delay >= time)
	{
	  *out = 0;
	  mem[0] = 0;
	  mem[1] = 0;
	}
	else
	{
	  *out = 1;
	}
  }
}

/**
 * @brief  Calculates moving average
 */
void MovingAverage(float *in, float *out, uint32_t length, float *mem1, int32_t *mem2)
{
  float sum = 0;

  mem2[0] = (mem2[0] + 1) % length;

  mem1[mem2[0]] = *in;

  for (int32_t i = 0; i < length; i++)
  {
	sum += mem1[i];
  }

  *out = sum / length;
}

/**
 * @brief  Counts number of pulses
 */
void Counter(int32_t *in, int32_t *out, int32_t edge, int32_t *mem)
{
  if (mem[0] == 0)
  {
    if (in[0] != edge)
	{
	  mem[0] = 1;
	  mem[1] += 1;
	}
  }
  else if (mem[0] == 1)
  {
	if (in[0] == edge)
	{
	  mem[0] = 0;
	}
  }
   
  if (edge == 1) *out = (mem[1] - 1); else *out = mem[1];
}

/**
 * @brief  Initializes FIR filter
 */
void FIR_Filter_Init(arm_fir_instance_f32 *pfilter, uint16_t numTaps, float32_t *pcoeffs, float32_t *pstate)
{
  arm_fir_init_f32(pfilter, numTaps, pcoeffs, pstate, 1);
}

/**
 * @brief  Update FIR filter output value
 */
void FIR_Filter(arm_fir_instance_f32 *pfilter, float32_t *psrc, float32_t *pdst)
{
  arm_fir_f32(pfilter, psrc, pdst, 1);
}

/**
 * @brief Initializes IIR filter
 */
void IIR_Filter_Init(arm_biquad_casd_df1_inst_f32 *pfilter, uint8_t numStages, float32_t *pcoeffs, float32_t *pstate)
{
  arm_biquad_cascade_df1_init_f32(pfilter, numStages, pcoeffs, pstate);
}

/**
 * @brief  Update IIR filter output value
 */
void IIR_Filter(arm_biquad_casd_df1_inst_f32 *pfilter, float32_t *psrc, float32_t *pdst)
{
  arm_biquad_cascade_df1_f32(pfilter, psrc, pdst, 1);
}

/**
 * @brief  Integrates input value
 */
void Integrator(float* in, float *out, float *mem)
{
  mem[0] = mem[0] + in[0] * (1.0f / (float) sensor_hub_data.data_rate_Hz);
  out[0] = mem[0];
}

/**
 * @brief  Derivates input value
 */
void Derivator(float* in, float *out, float *mem)
{

  out[0] = (in[0] - mem[0]) / (1.0f / (float) sensor_hub_data.data_rate_Hz);
  mem[0] = in[0];
}

/**
 * @brief  Switchs between two integer input values
 */
void SignalSwitchInt(int32_t *in1, int32_t *in2, int32_t *control, int32_t *out)
{
  if (control[0] == 0)
  {
    out[0] = in1[0];
  }
  else
  {
    out[0] = in2[0];
  }
}

/**
 * @brief  Switchs between two float input values
 */

void SignalSwitchFloat(float *in1, float *in2, int32_t *control, float *out)
{
  if (control[0] == 0)
  {
	out[0] = in1[0];
  }
  else
  {
    out[0] = in2[0];
  }
}

/**
 * @brief  Calculates signal features
 */
void Feature_Computation(float *in, int32_t *reset, float *max, float *min, float *mean, float *var, float *peak, float *energy, uint32_t length, int32_t *mem, float *buffer)
{
  /* mem[0] ... index to the buffer */
  /* mem[1] ... buffer full flag */

  float mean_temp = 0;
  float var_temp = 0;
  float energy_temp = 0;

  float imax = 0;
  float imin = 0;
  float imean = 0;
  float tmp;

  max[0] = 0;
  min[0] = 0;
  mean[0] = 0;
  var[0] = 0;
  peak[0] = 0;
  energy[0] = 0;

  /* Clear buffer if reset occurs */
  if (reset[0] != 0)
  {
	for (int i = 0; i < length; i++)
	{
	  buffer[i] = 0;
	}

	mem[0] = 0;
	mem[1] = 0;

	return;
  }

  /* Add value into buffer */
  buffer[mem[0]] = in[0];
  mem[0] = (mem[0] + 1) % length;

  /* Set flag if buffer is full */
  if ((mem[1] == 0) && (mem[0] == 0))
  {
    mem[1] = 1;
  }

  /* If buffer is full calculate the features */
  if (mem[1] == 1)
  {
	imax = 1.175494e-38;
	imin = 3.402823e+38;

	for (int i = 0; i < length; i++)
	{
	  if (imax < buffer[i]) imax = buffer[i];
	  if (imin > buffer[i]) imin = buffer[i];
	  mean_temp += buffer[i];
	  energy_temp += buffer[i] * buffer[i];
	}

    imean = mean_temp / length;

    for (int j = 0; j < length; j++)
    {
      tmp = (buffer[j] - imean);
      var_temp += tmp * tmp;
    }

    max[0] = imax;
    min[0] = imin;
    mean[0] = imean;
    var[0] = var_temp / length;
    peak[0] = imax - imin;
    energy[0] = energy_temp;
  }
}

/**
 * @brief  Delays input signal to the output
 */
void Signal_Delay(float* in, float *out, uint32_t length, float *mem1, int32_t *mem2)
{
  out[0] = mem1[(mem2[0] + 1) % length];
  mem1[mem2[0]] = in[0];
  mem2[0] = (mem2[0] + 1) % length;
}

/**
 * @brief  Initialize data buffer for feature calculation
 */
void Feature_Computation_2_Init(float *buffer, int n_items)
{
  memset(buffer, 0, n_items * sizeof(float));
}

/**
 * @brief  Calculates signal features
 * increased speed (one-pass) and accuracy (compensated summation), samples are not buffered, computation is spread evenly
 */
void Feature_Computation_2(float *in, int32_t *reset, float *max, float *min, float *mean, float *var, float *peak2peak, float *energy, int32_t *updated, uint32_t length, int32_t *mem, float *buffer)
{
  /*
  mem[0] is the index to the buffer, mem[1] is the buffer full flag
  
  buffer for samples is re-purposed to store intermediate results
  buffer[0] = max
  buffer[1] = min
  buffer[2] = mean
  buffer[3] = var
  buffer[4] = energy
  buffer[5] = resmeanincr
  buffer[6] = resvarincr
  buffer[7] = max - previous value from whole buffer
  buffer[8] = min - previous value from whole buffer
  buffer[9] = mean - previous value from whole buffer
  buffer[10] = var - previous value from whole buffer
  buffer[11] = energy - previous value from whole buffer
  buffer[12] = peak2peak - previous value from whole buffer
  */  

  if (*reset)
  {
    mem[0] = mem[1] = 0;
    memset(buffer, 0, 13 * sizeof(float));
    *max = *min = *mean = *var = *peak2peak = *energy = 0;
    *updated = 0;
    return;
  }

  int32_t idx = mem[0];
  mem[0] = (idx + 1) % length;
 
  float t = (*in); 
  if (idx == 0) 
  { 
    buffer[0] = buffer[1] = buffer[2] = t; /* tmax = tmin = tmean = t where t is 1st sample */
    buffer[3] = 0.0; /* tvar = 0.0; */
    buffer[4] = t*t; /* tenergy = t*t; */
    buffer[5] = buffer[6] = 0.0; /* resmeanincr = 0.0, resvarincr = 0.0, residual increment for mean and var */
  }
  
  if (idx > 0) 
  {
    float n = (float)(idx+1);
    
    /* one-pass algorithm with Kahan compensated summation */
    volatile float old, incr, totincr; /* volatile to block unwanted optimizations */
    float tmp;
    
    if (t > buffer[0]) /* max */
    {
      buffer[0] = t; 
    }
    
    if (t < buffer[1]) /* min */
    {
      buffer[1] = t;
    }
    
    buffer[4] += t*t; /* energy */
      
    old = buffer[2]; /* mean */
    incr = ((t - old) - buffer[5])/n; /* increment is d = (t-mean)/n but here mean = tmean+resmeanincr */
    totincr = incr + buffer[5];       /* total target increment to be applied is incr + residual stored in resmeanincr */
    buffer[2] += totincr;             /* because of rounding actual increment is not equal to target increment */
    buffer[5] = (old - buffer[2]);    /* residual increment                                  */
    buffer[5] += totincr;             /*                    is target minus actual increment */

    old = buffer[3]; /* var */
    
    tmp = incr;                         /* */
    tmp *= incr*(n*(n-1.0f));           /* */
    incr = ((tmp - old) - buffer[6])/n; /* increment is d = (d*d*(n-1)-var/n) but here var = tvar+resvarincr */
    
    totincr = incr + buffer[6]; /* total target increment to be applied is incr + residual stored in resvarincr */
    buffer[3] += totincr; /* because of rounding actual increment is not equal to target increment */
    buffer[6] = (old - buffer[3]); /* */
    buffer[6] += totincr;          /* residual increment is target minus actual increment */
  }
  
  if (idx == (length-1)) 
  { 
    mem[1] = 1; 
    *max = buffer[7] = buffer[0];     /* max; */
    *min = buffer[8] = buffer[1];     /* min; */
    *mean = buffer[9] = buffer[2];    /* mean */
    *var = buffer[10] = buffer[3];    /* var */
    *energy = buffer[11] = buffer[4]; /*energy; */
    *peak2peak = buffer[12] = buffer[0] - buffer[1];
    *updated = 1;
  }
  else
  {
    *max = buffer[7];       /* max; */
    *min = buffer[8];       /* min; */
    *mean = buffer[9];      /* mean */
    *var = buffer[10];      /* var */
    *energy = buffer[11];   /*energy; */
    *peak2peak = buffer[12];/*peak2peak */
    *updated = 0;
  }
}

/**
 * @brief  Detects zero crossings
 * if first or second sample is at zero, then zero crossing is NOT detected!
 */
void Zero_Crossing(float* in, int32_t *out, uint32_t edge, float *mem)
{
  /* *in is current new sample, *mem is previous older sample */
  int8_t negative_zerocrossing = (((*mem) > 0) && ((*in) < 0));
  int8_t positive_zerocrossing = (((*mem) < 0) && ((*in) > 0));
  int8_t detected;
  
  switch (edge)
  { 
    case FALLING_EDGE: 
      detected = negative_zerocrossing; 
      break;
      
    case RISING_EDGE:  
      detected = positive_zerocrossing; 
      break; 
    
    default:
      detected = negative_zerocrossing || positive_zerocrossing; 
    break;
  }
  
  (*out) = (int32_t)detected;
  (*mem) = (*in);
}

/**
 * @brief  Detects zero crossings with hysteresis
 * hysteresis allows to reject crossings due to noise
 */
void Zero_Crossing_Hyst(float* in, int32_t *out, uint32_t edge, float hyst, int32_t *stat)
{
  /* *in is current new sample, *stat is status */
  int8_t negative_zerocrossing = ((*in) < -hyst); /* check if going below low threshold */
  int8_t positive_zerocrossing = ((*in) > +hyst); /* check if going above high threshold */
  int8_t detected = 0; /* default is no event */
  
  if ((negative_zerocrossing) && (*stat != RISING_EDGE)) /* waiting to go below low threshold */
  {
    if (edge != RISING_EDGE)
    {
      detected = 1; /* signal event if wanted (that is: not only rising wanted) */
    }
    
    *stat = RISING_EDGE; /* then wait to go above high threshold */
  }
  
  if ((positive_zerocrossing) && (*stat != FALLING_EDGE)) /* waiting to go above high threshold */
  {
    if (edge != FALLING_EDGE)
    {
      detected = 1; /* signal event if wanted (that is: not only falling wanted) */
    }
    
    *stat = FALLING_EDGE; /* then wait to go below low threshold */
  }
  
  (*out) = (int32_t)detected;
}

/**
 * @brief  Detects peeks
 * noise may trigger peak detection as soon as signal is above threshold
 */
void Peak_Detector(float* in, int32_t *out, uint32_t peak, float threshold, float *mem)
{
  /* *in is current new sample, mem[1] is previous older, mem[0] is previous oldest */
  int8_t negative_peak;
  int8_t positive_peak;
  int8_t detected = fabs(*in) > threshold; /* to process all samples threshold must be negative! */
  
  if (detected)
  { 
    negative_peak = (mem[1] < mem[0]) && (mem[1] < (*in)); /* mem[1] is lower */
    positive_peak = (mem[1] > mem[0]) && (mem[1] > (*in)); /* mem[1] is higher */
    
    switch (peak)
    {
      case NEGATIVE_PEAK: 
        detected = negative_peak; 
        break;
        
      case POSITIVE_PEAK: 
        detected = positive_peak; 
        break;
        
      default: 
        detected = negative_peak || positive_peak; 
        break;
    }
  }
  
  (*out) = (uint32_t)detected;
  
  mem[0] = mem[1];
  mem[1] = (*in);
}

/**
 * @brief  Detects peeks
 * true peak detector with relative (not absolute) threshold
 */
void Peak_Detector_RelTh(float *in, int32_t *out, uint32_t peak, float threshold, int32_t reset, int32_t *stat, float *mem)
{
  float t = (*in);
  int8_t negative_peak = 0;
  int8_t positive_peak = 0;
  int8_t detected;
  
  /* *in is current new sample, mem[0] is max, mem[1] is min */
  if (reset) 
  {
    *stat = 0; 
    mem[0] = -FLT_MAX; 
    mem[1] = +FLT_MAX; 
    *out = 0;
    return;
  }
  
  if (*stat == 0) 
  { 
    /* at startup unknown search status */
    if (t > mem[0])
    {
      mem[0] = t;
    }
    
    if (t < mem[1])
    {
      mem[1] = t;
    }
    
    if ((mem[0] - t) >= threshold)
    { 
      /* going down, look for min */
      *stat = NEGATIVE_PEAK;
      mem[1] = +FLT_MAX; 
    }
    
    if ((t - mem[1]) >= threshold)
    { 
      /* going up, look for max */
      *stat = POSITIVE_PEAK; 
      mem[0] = -FLT_MAX; 
    } 
  }

  if (*stat == POSITIVE_PEAK) 
  {
    if (t > mem[0])
    {
      mem[0]=t;
    }
    
    if ((mem[0] - t) >= threshold)
    { 
      positive_peak = 1; 
      *stat = NEGATIVE_PEAK; 
      mem[1] = +FLT_MAX; 
    }
  } 
  else 
  { 
    /* stat == NEGATIVE_PEAK */
    if (t < mem[1])
    {
      mem[1]=t;
    }
    
    if ((t - mem[1]) >= threshold)
    { 
      negative_peak = 1; 
      *stat = POSITIVE_PEAK; 
      mem[0] = -FLT_MAX; 
    }
  }
  
  switch (peak)
  {
    case NEGATIVE_PEAK: 
      detected = negative_peak; 
      break;
      
    case POSITIVE_PEAK: 
      detected = positive_peak; 
      break;
      
    default: 
      detected = negative_peak || positive_peak; 
      break;
  }
  
  (*out) = (uint32_t)detected;
}

/**
 * @brief  Detects spikes
 * noise is rejected by higher threshold
 */
void Spike_Detector(float* in, int32_t *out, uint32_t peak, float threshold, float *mem)
{
  /* *in is current new sample, mem[1] is previous older, mem[0] is previous oldest */
  float left_diff = (*in)-mem[1];
  float right_diff = mem[1]-mem[0];
  int8_t negative_peak = (right_diff < -threshold) && ( left_diff > +threshold); /* mem[1] is lower */
  int8_t positive_peak = (right_diff > +threshold) && ( left_diff < -threshold); /* mem[1] is higher */
  int8_t detected;
  
  switch (peak)
  {
    case NEGATIVE_PEAK: 
      detected = negative_peak; 
      break;
      
    case 
      POSITIVE_PEAK: detected = positive_peak; 
      break;
      
    default: 
      detected = negative_peak || positive_peak; 
      break;
  }
  
  (*out) = (uint32_t)detected;
  
  mem[0] = mem[1];
  mem[1] = (*in);
}

/**
 * @}
 */

/**
 * @}
 */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
