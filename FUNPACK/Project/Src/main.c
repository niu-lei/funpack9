/**
 ******************************************************************************
 * @file       main.c
 * @author     MEMS Application Team
 * @brief      Main program body
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
#include "main.h"

#if (defined (USE_NUCLEO))
#include "com.h"
#elif (defined (USE_SENSORTILE) || defined (USE_SENSORTILEBOX) || defined (USE_STWIN))
#include "vcom.h"
#else
#error Not supported platform
#endif

#include "demo_serial.h"
#include "algo_builder.h"
#include "ab_display.h"

#if (defined (USE_BLE_OUTPUT))
#include "ab_ble.h"
#endif

/** @addtogroup AlgoBuilder_Firmware
  * @{
  */

/** @addtogroup Main
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DEFAULT_uhCCR1_Val  100 /* 10kHz/100 value */
#define DEFAULT_uhCCR2_Val  200 /* 10kHz/50 value */
#define DEFAULT_uhCCR3_Val  625 /* 10kHz/16 value */
#define DEFAULT_uhCCR4_Val  400 /* 10kHz/25 value */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef RtcHandle;
TIM_HandleTypeDef AB_TimHandle;
TIM_HandleTypeDef AL_TimHandle;
TMsg MsgDat;
TMsg MsgCmd;

extern sensor_hub_data_t sensor_hub_data;
extern offline_data_t offline_data;

uint64_t StartTime = 0;

volatile uint32_t sensor_read_request = 0;
volatile uint32_t update_16Hz = 0;
volatile uint32_t update_25Hz = 0;
volatile uint32_t update_50Hz = 0;
volatile uint32_t update_100Hz = 0;

volatile uint8_t DataLoggerActive = 0;
uint32_t Sensors_Enabled = 0;
volatile uint8_t DataLoggerStatusChanged = 0;

/* Extern variables ----------------------------------------------------------*/
extern int use_LSI;
int RTC_SYNCH_PREDIV;
volatile int Message_Length;

/* Private function prototypes -----------------------------------------------*/
static void TIM_AL_Config(void);
static void Time_Handler(TMsg *Msg);
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void Init_Sensors(void);
static void DeInit_Sensors(void);
extern void SystemClock_Config(void);

static uint32_t DWT_Delay_Init(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use X_NUCLEO_IKS01A2 expansion board to send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like
 *         TeraTerm and specific application like Sensors_DataLog, which is developed by STMicroelectronics
 *         and provided with this package.
 * @param  None
 * @retval Integer
 */
int main(void)
{  
  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
   */
  HAL_Init();

  /* Reset USB */
  __HAL_RCC_USB_OTG_FS_FORCE_RESET();
  __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialzie Error queue*/
  Init_Error();

  /* Initialize GPIOs */
  MX_GPIO_Init();

  /* Initialize CRC */
  MX_CRC_Init();

  /* Initialize (disabled) Sensors */
  Init_Sensors();

#if (defined (USE_NUCLEO))
  /* Initialize UART */
  USARTConfig();
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  /* Initialize VCOM */
  VCOM_init();
#else
#error Not supported platform
#endif

  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  
  /* Initialize cycle counter */
  DWT_Delay_Init();

  /* Initialize timers for algorithms synchronization */
  TIM_AL_Config();

  /* Blink with LED */
#if (defined (USE_NUCLEO))
  BSP_LED_On(LED2);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  BSP_LED_On(LED1);
#else
#error Not supported platform
#endif
 
  HAL_Delay(500);

#if (defined (USE_NUCLEO))
  BSP_LED_Off(LED2);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  BSP_LED_Off(LED1);
#else
#error Not supported platform
#endif

#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
  /* Not supported feature */
#elif ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  Datalog_Set_RTC_Handle(&RtcHandle);
#else
#error Not supported platform
#endif

  /* AlgoBuilder initialization */
  AB_Init();

#if (defined (USE_BLE_OUTPUT))
  BLE_init(&MsgCmd);
#endif
  
  /* Check if UCF loading changed ODR or FS */
  FSM_MLC_Check_ODR_FS_Change();
  
  /* To start reading in case of interrupt driven flow */
  sensor_read_request = 1;

  /* Ensure that User Button pressing will be evaluated from this point */
  DataLoggerActive = 0;
  DataLoggerStatusChanged = 0;

  while(1)
  {
    /* Process incoming data */
#if (defined (USE_NUCLEO))      
    if ((UART_ReceivedMSG((TMsg*) &MsgCmd)) && (MsgCmd.Data[0] == DEV_ADDR))
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
    if (VCOM_receive_MSG((TMsg*) &MsgCmd, TMsg_MaxLen) == OK)
#else
#error Not supported platform
#endif
    {
      HandleMSG((TMsg *)&MsgCmd, 0);
    }

#if (defined (USE_BLE_OUTPUT))
    BLE_handle_Event();
#endif

    if (DataLoggerStatusChanged != 0)
    {
      if (DataLoggerActive == 1)
      {
        TIM_AB_Start();
        TIM_AL_Start();
        
        StartTime = DWT_GetTickUS();
        
#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
        /* Not supported feature */
#elif  ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
        if (sensor_hub_data.sdcard_enable == 1)
        {
          Datalog_Open();
        }
#else
#error Not supported platform
#endif
      }
      else
      {
        TIM_AB_Stop();
        TIM_AL_Stop();
        
#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
        /* Not supported feature */
#elif  ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
        if (sensor_hub_data.sdcard_enable == 1)
        {
          Datalog_Close();
        }
#else
#error Not supported platform
#endif
      }
      
      MsgCmd.Data[0] = 1;
      MsgCmd.Data[1] = DEV_ADDR;
      MsgCmd.Data[2] = CMD_Get_Streaming_Status + CMD_Reply_Add;
      MsgCmd.Data[3] = DataLoggerActive;
      MsgCmd.Len = 4;
      
#if (defined (USE_NUCLEO))
      UART_SendMsg(&MsgCmd);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  #if (defined (USE_BLE_OUTPUT))
      BLE_send_Com(&MsgCmd);
      HAL_Delay(30);
  #else
      VCOM_send_MSG(&MsgCmd);
  #endif
#else
#error Not supported platform
#endif
      
      DataLoggerStatusChanged = 0;
    }
    
    /* Execute AlgoBuilder Handler and send data stream */
    if (sensor_read_request && DataLoggerActive)
    {
      sensor_read_request = 0;

      Time_Handler(&MsgDat);

#if (defined (USE_NUCLEO))
      BSP_LED_On(LED2);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  #if !defined (USE_BLE_OUTPUT)
      BSP_LED_On(LED1);
  #endif
#else
#error Not supported platform
#endif
      
      AB_Handler();

#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
       /* Not supported feature */
#elif  ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
      Datalog_Sync_Handler();
#else
#error Not supported platform
#endif
      
#if (defined (USE_NUCLEO))
      BSP_LED_Off(LED2);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  #if !defined (USE_BLE_OUTPUT)
      BSP_LED_Off(LED1);
  #endif
#else
#error Not supported platform
#endif

      INIT_STREAMING_HEADER(&MsgDat);
      MsgDat.Len += Message_Length + 6;
      
#if (defined (USE_NUCLEO))
      UART_SendMsg(&MsgDat);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  #if (defined (USE_BLE_OUTPUT))
      BLE_send_Data(&MsgDat);
  #else
      VCOM_send_MSG(&MsgDat);
  #endif
#else
#error Not supported platform
#endif
      
    }
  }
}

/**
  * @brief  Initialize all sensors
  * @param  None
  * @retval None
  */
static void Init_Sensors(void)
{
#if (defined (USE_IKS01A2))
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);
  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);
  
  (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
  (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_GYRO);
  (void)IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
  (void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_TEMPERATURE);
  (void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_HUMIDITY);
  (void)IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE);
#elif (defined (USE_IKS01A3))
  (void)IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO);
  (void)IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2MDL_0, MOTION_MAGNETO);
  (void)IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  (void)IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE);
  
  (void)IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LSM6DSO_0, MOTION_ACCELERO);
  (void)IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LSM6DSO_0, MOTION_GYRO);
  (void)IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LIS2MDL_0, MOTION_MAGNETO);
  (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_TEMPERATURE);
  (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_HUMIDITY);
  (void)IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_PRESSURE);
#elif (defined (USE_SENSORTILE))
  (void)BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_ACCELERO | MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Init(LSM303AGR_MAG_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Init(HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Init(LPS22HB_0, ENV_PRESSURE);
  
  (void)BSP_MOTION_SENSOR_Enable(LSM6DSM_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Enable(LSM6DSM_0, MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Enable(LSM303AGR_MAG_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Enable(HTS221_0, ENV_TEMPERATURE);
  (void)BSP_ENV_SENSOR_Enable(HTS221_0, ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Enable(LPS22HB_0, ENV_PRESSURE);
#elif (defined (USE_SENSORTILEBOX))
  (void)BSP_MOTION_SENSOR_Init(LSM6DSOX_0, MOTION_ACCELERO | MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Init(LIS2MDL_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Init(HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Init(LPS22HH_0, ENV_PRESSURE);
  
  (void)BSP_MOTION_SENSOR_Enable(LSM6DSOX_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Enable(LSM6DSOX_0, MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Enable(LIS2MDL_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Enable(HTS221_0, ENV_TEMPERATURE);
  (void)BSP_ENV_SENSOR_Enable(HTS221_0, ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Enable(LPS22HH_0, ENV_PRESSURE);
#elif (defined (USE_STWIN))
  (void)BSP_MOTION_SENSOR_Init(ISM330DHCX_0, MOTION_ACCELERO | MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Init(IIS2MDC_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Init(HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Init(LPS22HH_0, ENV_PRESSURE);
  
  (void)BSP_MOTION_SENSOR_Enable(ISM330DHCX_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Enable(ISM330DHCX_0, MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Enable(IIS2MDC_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Enable(HTS221_0, ENV_TEMPERATURE);
  (void)BSP_ENV_SENSOR_Enable(HTS221_0, ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Enable(LPS22HH_0, ENV_PRESSURE);
#else
#error Not supported platform
#endif
}

/**
  * @brief  DeInitialize all sensors
  * @param  None
  * @retval None
  */
static void DeInit_Sensors(void)
{
#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
  /* Not supported feature */
#elif  (defined (USE_SENSORTILEBOX))
  (void)BSP_MOTION_SENSOR_Disable(LSM6DSOX_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Disable(LSM6DSOX_0, MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Disable(LIS2MDL_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Disable(HTS221_0, ENV_TEMPERATURE);
  (void)BSP_ENV_SENSOR_Disable(HTS221_0, ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Disable(LPS22HH_0, ENV_PRESSURE);
  
  (void)BSP_MOTION_SENSOR_DeInit(LSM6DSOX_0);
  (void)BSP_MOTION_SENSOR_DeInit(LIS2MDL_0);
  (void)BSP_ENV_SENSOR_DeInit(HTS221_0);
  (void)BSP_ENV_SENSOR_DeInit(LPS22HH_0);
#elif  (defined (USE_STWIN))
  (void)BSP_MOTION_SENSOR_Disable(ISM330DHCX_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Disable(ISM330DHCX_0, MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Disable(IIS2MDC_0, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Disable(HTS221_0, ENV_TEMPERATURE);
  (void)BSP_ENV_SENSOR_Disable(HTS221_0, ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Disable(LPS22HH_0, ENV_PRESSURE);
  
  (void)BSP_MOTION_SENSOR_DeInit(ISM330DHCX_0);
  (void)BSP_MOTION_SENSOR_DeInit(IIS2MDC_0);
  (void)BSP_ENV_SENSOR_DeInit(HTS221_0);
  (void)BSP_ENV_SENSOR_DeInit(LPS22HH_0);
#else
#error Not supported platform
#endif
}

/**
  * @brief  GPIO init function.
  * @param  None
  * @retval None
  * @details GPIOs initialized are User LED(PA5) and User Push Button(PC1)
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
#if (defined (USE_NUCLEO))
  /* Initialize LED */
  BSP_LED_Init(LED2);
  /* Initialize push button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#elif (defined (USE_SENSORTILE))
  /* Initialize LED */
  BSP_LED_Init(LED1);
#elif ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  /* Initialize LED */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  /* Initialize push button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#else
#error Not supported platform
#endif
  
  GPIO_InitTypeDef GPIO_InitStructureInt1;

#if (defined (USE_NUCLEO))
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = GPIO_PIN_5;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructureInt1);
  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x00, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)))
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = GPIO_PIN_2;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructureInt1);
  
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0x00, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
#elif (defined (USE_STWIN))
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = GPIO_PIN_8;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructureInt1);
  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0x00, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#else
#error Not supported platform
#endif
}

/**
  * @brief  CRC init function.
  * @param  None
  * @retval None
  */
static void MX_CRC_Init(void)
{
  __CRC_CLK_ENABLE();
}

/**
  * @brief  TIM_AB config function
  * @param  None
  * @retval None
  * @details This function initialize the Timer used to synchronize the AB algorithm
  */
void TIM_AB_Config(uint32_t freq)
{
  uint32_t uwPrescalerValue;

  /* Compute the prescaler value to have TIM counter clock equal to 20 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 20000) - 1);

  /* Calculate Period */
  uint32_t period = (20000 / freq) - 1;

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  AB_TimHandle.Instance = TIM_AB;
  AB_TimHandle.Init.Prescaler = uwPrescalerValue;
  AB_TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  AB_TimHandle.Init.Period = period;
  AB_TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&AB_TimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&AB_TimHandle, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&AB_TimHandle, &sMasterConfig);
}

/**
  * @brief  TIM_AB start timer
  * @param  None
  * @retval None
  */
void TIM_AB_Start(void)
{
  HAL_TIM_Base_Start_IT(&AB_TimHandle);
}

/**
  * @brief  TIM_AB stop timer
  * @param  None
  * @retval None
  */
void TIM_AB_Stop(void)
{
  HAL_TIM_Base_Stop_IT(&AB_TimHandle);
}

/**
  * @brief  TIM_AL config function
  * @param  None
  * @retval None
  * @details This function initialize the timer used to synchronize the enabled algorithms
  */
void TIM_AL_Config(void)
{
  uint32_t uwPrescalerValue;
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  /* Set TIM1 instance */
  AL_TimHandle.Instance = TIM1;
  AL_TimHandle.Init.Prescaler = uwPrescalerValue;
  AL_TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  AL_TimHandle.Init.Period = 0xFFFF;
  AL_TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_OC_Init(&AL_TimHandle);

  /* Configure the Output Compare channels */
  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  HAL_TIM_OC_ConfigChannel(&AL_TimHandle, &sConfig, TIM_CHANNEL_1);

  /* Output Compare Toggle Mode configuration: Channel2 */
  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  HAL_TIM_OC_ConfigChannel(&AL_TimHandle, &sConfig, TIM_CHANNEL_2);

  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  HAL_TIM_OC_ConfigChannel(&AL_TimHandle, &sConfig, TIM_CHANNEL_3);

  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  HAL_TIM_OC_ConfigChannel(&AL_TimHandle, &sConfig, TIM_CHANNEL_4);
}

/**
  * @brief  TIM_AL start timer
  * @param  None
  * @retval None
  */
void TIM_AL_Start(void)
{
  HAL_TIM_OC_Start_IT(&AL_TimHandle, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&AL_TimHandle, TIM_CHANNEL_2);
  HAL_TIM_OC_Start_IT(&AL_TimHandle, TIM_CHANNEL_3);
  HAL_TIM_OC_Start_IT(&AL_TimHandle, TIM_CHANNEL_4);
}

/**
  * @brief  TIM_AL stop timer
  * @param  None
  * @retval None
  */
void TIM_AL_Stop(void)
{
  HAL_TIM_OC_Stop_IT(&AL_TimHandle, TIM_CHANNEL_1);
  HAL_TIM_OC_Stop_IT(&AL_TimHandle, TIM_CHANNEL_2);
  HAL_TIM_OC_Stop_IT(&AL_TimHandle, TIM_CHANNEL_3);
  HAL_TIM_OC_Stop_IT(&AL_TimHandle, TIM_CHANNEL_4);
}

/**
 * @brief  Handles the precise time
 * @param  Msg the time part of the stream
 * @retval None
 */
static void Time_Handler(TMsg *Msg)
{
  uint64_t time_us;

  if (sensor_hub_data.data_rate_control == DRC_OFFLINE)
  {
    memcpy(&Msg->Data[3], &offline_data.timestamp_us, 6);
    sensor_hub_data.timestamp_us = offline_data.timestamp_us;
  }
  else
  {
    time_us = DWT_GetTickUS() - StartTime;
    memcpy(&Msg->Data[3], &time_us, 6);
    sensor_hub_data.timestamp_us = time_us;
  }
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void)
{
/* DFU bootloader clears the RTCAPBEN bit (normally after reset this bit is set by default), it is necessary to manually setr this bit  */
#if (defined (USE_NUCLEO) || defined (USE_SENSORTILE))
  /* Not supported */
#elif (defined (USE_SENSORTILEBOX) || defined (USE_STWIN))
  __HAL_RCC_RTCAPB_CLK_ENABLE();
#else
#error Not supported platform
#endif
    
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef        RCC_OscInitStruct;

  /*##-1- Configue LSE as RTC clock soucre ###################################*/
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    use_LSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSI;
  }
  else
  {
    /* We use LSE */
    use_LSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSE;
  }
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 24
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /* Configure the Date */
  /* Set Date: Monday January 1st 2001 */
  sdatestructure.Year    = 0x01;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
	/* Initialization Error */
	Error_Handler();
  }

  /* Configure the Time */
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT_24;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
	/* Initialization Error */
	Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_DateTimeRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat = RTC_HOURFORMAT_24;
  stimestructure.Hours = hh;
  stimestructure.Minutes = mm;
  stimestructure.Seconds = ss;
  stimestructure.SubSeconds = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  RTC_DateTypeDef sdatestructure;
  
  sdatestructure.Year = y;
  sdatestructure.Month = m;
  sdatestructure.Date = d;
  sdatestructure.WeekDay = 0;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }  
}

/**
 * @brief  Initializes DWT_Clock_Cycle_Count for DWT_GetTickUS function
 * @retval Error DWT counter (1: clock cycle counter not started, 0: clock cycle counter works)
 */
static uint32_t DWT_Delay_Init(void)
{
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable TRC */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;

  /* Enable clock cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

  /* Delay 1ms */
  HAL_Delay(1);

  /* Check if clock cycle counter has started */
  if (DWT->CYCCNT)
  {
     return 0; /* Clock cycle counter started */
  }
  else
  {
    return 1; /* Clock cycle counter not started */
  }
}

/**
 * @brief  Get relative time in micro seconds
 * @note   Call at least every 2^32 cycles. Do not call from interrupt context!
 * @retval Relative time in micro seconds
 */
uint64_t DWT_GetTickUS(void)
{
  static uint64_t last_cycle_count_64 = 0;
  uint32_t clock_MHz = HAL_RCC_GetHCLKFreq() / 1000000;

  last_cycle_count_64 += DWT->CYCCNT - (uint32_t)(last_cycle_count_64);

  return (uint64_t)(last_cycle_count_64 / clock_MHz);
}

/**
 * @brief  Build an array from the source data (LSB first)
 * @param  Dest destination
 * @param  Source source
 * @param  Len number of bytes
 * @retval None
 */
void SerializeToMsg(uint8_t Dest, void *Source, uint32_t Len)
{
  memcpy(&MsgDat.Data[Dest], Source, Len);
}

/**
 * @brief  Enter DFU mode (only for SensorTile.box)
 * @param  None
 * @retval None
 */
void EnterDFU(void)
{
#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
  /* Not supported feature */
  /* Call sensor deinitialization funtion to avoid warning */
  DeInit_Sensors();
#elif  ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  
  typedef void (*pFunction)(void);
  pFunction JumpToDFU;

  /* DeInitialize VCOM */
  VCOM_deinit();

  /* DeInitialize All Sensors */
  DeInit_Sensors();
  
  /* Let Windows OS detect that VCOM device was disconnected */
  HAL_Delay(2000);
   
  /* Disable systick timer and reset it to default values */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  
  /* Disable interrupts */
  __disable_irq();
  
  /* ARM Cortex-M Programming Guide to Memory Barrier Instructions.*/
  __DSB();
  
  /* */
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* o remap the System Memory bootloader at address 0x00000000 */
  __HAL_REMAPMEMORY_SYSTEMFLASH();
  
  __DSB();
  __ISB();
  
  JumpToDFU = (void (*)(void)) (*((uint32_t *)(0x00000000 + 4)));
  
  /* Set Stack Pointer */
#if defined (__IAR_SYSTEMS_ICC__)
  __set_MSP(*((uint32_t*) 0x00000000));
#elif defined (__CC_ARM)
  __set_MSP(*((uint32_t*) 0x00000000));
#elif defined (__GNUC__)
  __ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");
#endif
  
  /* Jump to DFU Bootloader */
  JumpToDFU();
  
  while (1);
#else
#error Not supported platform
#endif
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if (defined (USE_NUCLEO))
  if (GPIO_Pin == GPIO_PIN_5)
  {
    sensor_read_request = 1;
  }   
  
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    if (DataLoggerStatusChanged == 0)
    {
      if (DataLoggerActive == 0)
      {
        DataLoggerActive = 1;
      }
      else
      {
        DataLoggerActive = 0;
      }
      DataLoggerStatusChanged = 1;
    }
  }
  
#elif (defined (USE_SENSORTILE)) 
  if (GPIO_Pin == GPIO_PIN_2)
  {
    sensor_read_request = 1;
  } 
  
#elif (defined (USE_SENSORTILEBOX))
  if (GPIO_Pin == GPIO_PIN_2)
  {
    sensor_read_request = 1;
  }
  
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    if (DataLoggerStatusChanged == 0)
    {
      if (DataLoggerActive == 0)
      {
        DataLoggerActive = 1;
      }
      else
      {
        DataLoggerActive = 0;
      }
      DataLoggerStatusChanged = 1;
    }
  }
  #if (defined (USE_BLE_OUTPUT))
  BLE_EXTI_Callback(GPIO_Pin);
  #endif
  
#elif (defined (USE_STWIN))
  if (GPIO_Pin == GPIO_PIN_8)
  {
    sensor_read_request = 1;
  }
  
  if (GPIO_Pin == USER_BUTTON_PIN)
  {
    if (DataLoggerStatusChanged == 0)
    {
      if (DataLoggerActive == 0)
      {
        DataLoggerActive = 1;
      }
      else
      {
        DataLoggerActive = 0;
      }
      DataLoggerStatusChanged = 1;
    }
  }
  #if (defined (USE_BLE_OUTPUT))
  BLE_EXTI_Callback(GPIO_Pin);
  #endif

#else
#error Not supported platform
#endif
}

/**
  * @brief  Period elapsed callback
  * @param  htim pointer to a TIM_HandleTypeDef structure that contains
  *              the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM_AB)
  {
    sensor_read_request = 1;
  }
  
#if (defined (USE_NUCLEO))
  /* Not supported feature */
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  if (htim->Instance == TIMx)
  {
    CDC_TIM_PeriodElapsedCallback(htim);
  }
#else
#error Not supported platform
#endif
}

/**
  * @brief  Output Compare callback in non blocking mode
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;

  /* TIM1_CH1 toggling with frequency = 100Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&AL_TimHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
    update_100Hz = 1;
  }

  /* TIM1_CH2 toggling with frequency = 50Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&AL_TimHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
    update_50Hz = 1;
  }

  /* TIM1_CH3 toggling with frequency = 16Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&AL_TimHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
    update_16Hz = 1;
  }

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&AL_TimHandle, TIM_CHANNEL_4, (uhCapture + DEFAULT_uhCCR4_Val));
    update_25Hz = 1;
  }
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
 * @}
 */

/**
 * @}
 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
