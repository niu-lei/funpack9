/**
 ******************************************************************************
 * @file       ab_sensor_hub.h
 * @author     MEMS Application Team
 * @brief      Header for ab_sensor_hub.c
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
#ifndef __AB_SENSOR_HUB_H
#define __AB_SENSOR_HUB_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ab_libraries.h"

#if (MotionAC)
#include "motion_ac.h"
#endif
#if (MotionAW)
#include "motion_aw.h"
#endif
#if (MotionEC)
#include "motion_ec.h"
#endif
#if (MotionFX)
#include "motion_fx.h"
#endif
#if (MotionGC)
#include "motion_gc.h"
#endif
#if (MotionID)
#include "motion_id.h"
#endif
#if (MotionMC)
#include "motion_mc.h"
#endif
#if (MotionPM)
#include "motion_pm.h"
#endif
#if (MotionPW)
#include "motion_pw.h"
#endif
#if (MotionTL)
#include "motion_tl.h"
#endif

#include "demo_serial.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
  DRC_TIMER = 0,
  DRC_ACCELEROMETER = 1,
  DRC_GYROSCOPE = 2,
  DRC_OFFLINE = 255,
} data_rate_control_t;

typedef struct {
  data_rate_control_t data_rate_control;
  uint32_t data_rate_Hz;
  uint32_t meas_data_rate_Hz; 
#if (defined (USE_IKS01A2))
  IKS01A2_MOTION_SENSOR_Axes_t acc;
  IKS01A2_MOTION_SENSOR_Axes_t gyr;
  IKS01A2_MOTION_SENSOR_Axes_t mag;
#elif (defined (USE_IKS01A3))
  IKS01A3_MOTION_SENSOR_Axes_t acc;
  IKS01A3_MOTION_SENSOR_Axes_t gyr;
  IKS01A3_MOTION_SENSOR_Axes_t mag;
#elif (defined (USE_SENSORTILE))
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyr;
  BSP_MOTION_SENSOR_Axes_t mag;
#elif (defined (USE_SENSORTILEBOX))
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyr;
  BSP_MOTION_SENSOR_Axes_t mag;
#elif (defined (USE_STWIN))
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyr;
  BSP_MOTION_SENSOR_Axes_t mag;
#else
#error Not supported platform
#endif
  uint64_t timestamp_us;
  float acceleration[3];
  float angular_rate[3];
  float magnetic_field[3];
  float pressure;
  float humidity;
  float temperature;
#if (MotionAC)
  MAC_output_t mac;
#endif
#if (MotionAW)
  MAW_output_t maw;
#endif
#if (MotionEC)
  MEC_output_t mec;
  float acc_matrix[9];
  float mag_matrix[9];
#endif
#if (MotionFX)
  MFX_MagCal_output_t mfx_magcal;
  MFX_output_t mfx;
#endif
#if (MotionGC)
  MGC_output_t mgc;
#endif
#if (MotionID)
  MID_output_t mid;
#endif
#if (MotionMC)
  MMC_Output_t mmc;
#endif
#if (MotionPM)
  MPM_output_t mpm;
#endif
#if (MotionPW)
  MPW_output_t mpw;
  uint8_t activity_recognition_enable;
#endif
#if (MotionTL)
  MTL_output_t mtl;
  MTL_angle_mode_t mtl_mode;
#endif
  uint8_t acceleration_enable;
  uint8_t angular_rate_enable;
  uint8_t magnetic_field_enable;
  uint8_t pressure_enable;
  uint8_t humidity_enable;
  uint8_t temperature_enable;
  uint8_t motion_ac_enable;
  uint8_t motion_aw_enable;
  uint8_t motion_ec_enable;
  uint8_t motion_fx_enable;
  uint8_t motion_gc_enable;
  uint8_t motion_id_enable;
  uint8_t motion_mc_enable;
  uint8_t motion_pm_enable;
  uint8_t motion_pw_enable;
  uint8_t motion_tl_enable;
  uint32_t iteration_counter;
  uint8_t fsm_enable;
  uint8_t fsm_data[16];
  uint8_t mlc_enable;
  uint8_t mlc_data[8];
  uint8_t sdcard_enable;
} sensor_hub_data_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define FROM_MG_TO_G          0.001f
#define FROM_G_TO_MG          1000.0f
#define FROM_MDPS_TO_DPS      0.001f
#define FROM_DPS_TO_MDPS      1000.0f
#define FROM_MGAUSS_TO_UT50   (0.1f/50.0f)
#define FROM_MGAUSS_TO_UT     0.1f
#define FROM_UT_TO_UT50       0.02f
#define FROM_UT_TO_MGAUSS     10.0f
#define FROM_UT50_TO_UT       50.0f
#define FROM_UT50_TO_MGAUSS   500.0f

#define SAMPLETODISCARD       15
#define GBIAS_ACC_TH_SC_6X    (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_6X   (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_6X    (2.0f*0.001500f)
#define GBIAS_ACC_TH_SC_9X    (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_9X   (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_9X    (2.0f*0.001500f)

/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Sensor_Hub_Init(uint32_t data_rate_control, uint32_t data_rate, uint32_t accel_fs, uint32_t gyro_fs);
void Sensor_Hub_Handler(void **pdata);
void Sensor_Hub_Offline_Handler(void **pdata);

void Accelero_Init(void);
void Gyro_Init(void);
void Magneto_Init(void);
void Pressure_Init(void);
void Temperature_Init(void);
void Humidity_Init(void);

void Algo_16Hz_Handler(void);
void Algo_25Hz_Handler(void);
void Algo_50Hz_Handler(void);
void Algo_100Hz_Handler(void);

#if (MotionAC)
void MotionAC_Init(uint32_t mode, float threshold);
#endif
#if (MotionAW)
void MotionAW_Init(void);
#endif
#if (MotionEC)
void MotionEC_Init(void);
#endif
#if (MotionFX)
void MotionFX_Init(void);
#endif
#if (MotionGC)
void MotionGC_Init(float acc_thr, float gyro_thr, float filter_const, int fast_start, float max_acc, float max_gyro);
#endif
#if (MotionID)
void MotionID_Init(void);
#endif
#if (MotionMC)
void MotionMC_Init(void);
#endif
#if (MotionPM)
void MotionPM_Init(void);
#endif
#if (MotionPW)
void MotionPW_Init(int32_t activity_recognition);
#endif
#if (MotionTL)
void MotionTL_Init(int32_t mode);
#endif

void Accelero_Sensor_GetData(void *pdata, float *data);
void Gyro_Sensor_GetData(void *pdata, float *data);
void Magneto_Sensor_GetData(void *pdata, float *data);
void Pressure_Sensor_GetData(void *pdata, float *data);
void Temperature_Sensor_GetData(void *pdata, float *data);
void Humidity_Sensor_GetData(void *pdata, float *data);

void Quaternions9X_GetData(void *pdata, float *data);
void Rotation9X_GetData(void *pdata, float *data);
void Gravity9X_GetData(void *pdata, float *data);
void LinearAcceleration9X_GetData(void *pdata, float *data);
void Heading9X_GetData(void *pdata, float *data);
void MagnetoCal_GetData(void *pdata, float *data);
void MagnetoCal_GetCalData(void *pdata, float *data, int32_t *quality);

void AccCal_GetData(void *pdata, int32_t *reset, float *data1, int32_t *data2);
void ActivityWrist_GetData(void *pdata, int32_t *reset, int32_t *data);
void MotionEC_Quternions_GetData(void *pdata, float *data);
void MotionEC_Rotation_GetData(void *pdata, float *data);
void MotionEC_Gravity_GetData(void *pdata, float *data);
void MotionEC_LinearAcceleration_GetData(void *pdata, float *data);
void MotionEC_VirtualGyroscope_GetData(void *pdata, float *data);
void GyroCal_GetData(void *pdata, int32_t *reset, float *data);
void MotionIntensity_GetData(void *pdata, int32_t *reset, int32_t *data);
void MagCal_GetData(void *pdata, int32_t *reset, float *data1, int32_t *data2);
void PedometerMobile_GetData(void *pdata, int32_t *reset, int32_t *data1, int32_t *data2);
void PedometerWrist_GetData(void *pdata, int32_t *reset, int32_t *data1, int32_t *data2);
void TiltSensing_GetData(void *pdata, float *data);

void FSM_MLC_Init(int32_t fsm_number, int32_t mlc_number);
void FSM_MLC_GetData(void *pdata, int32_t *fsm_data, int32_t *mlc_data, int32_t fsm_number, int32_t mlc_number);
void FSM_MLC_Check_ODR_FS_Change(void);

void TeslaToGauss(float *in, float *out);
void GetIterationCounter(void *pdata, int32_t *out);

#endif /* __AB_SENSOR_HUB_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
