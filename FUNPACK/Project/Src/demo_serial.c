/**
 ******************************************************************************
 * @file       DemoSerial.c
 * @author     MEMS Application Team
 * @brief      Handler Serial Protocol
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
#include "demo_serial.h"
#include "ab_sensor_hub.h"
#include "main.h"

#if (defined (USE_NUCLEO))
#include "com.h"
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
#include "vcom.h"
#else
#error Not supported platform
#endif

#if (defined (USE_BLE_OUTPUT))
#include "ab_ble.h"
#endif

#include "sensor_unicleo_id.h"
#include "sensor_commands.h"

/** @addtogroup AlgoBuilder_Firmware
  * @{
  */

/** @addtogroup Communication
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#if (defined (USE_BLE_OUTPUT))
#define COMMUNICATION "BLE"
#else
#define COMMUNICATION "COM"
#endif

#if (defined (STM32F4))
#define MCU_FAMILY "STM32F4"
#elif (defined (STM32L4))
#define MCU_FAMILY "STM32L4"
#else
#define MCU_FAMILY ""
#endif
    
#define INIT_ERROR_QUEUE_SIZE 4  
    
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t SenderInterface = 0;
offline_data_t offline_data;
static uint32_t last_run_16Hz = 0;
static uint32_t last_run_25Hz = 0;
static uint32_t last_run_50Hz = 0;
static uint32_t last_run_100Hz = 0;

extern volatile uint32_t sensor_read_request;
extern sensor_hub_data_t sensor_hub_data;

extern volatile uint32_t update_16Hz;
extern volatile uint32_t update_25Hz;
extern volatile uint32_t update_50Hz;
extern volatile uint32_t update_100Hz;
extern char Identification_String[];

#if (defined (USE_IKS01A2))
const uint8_t PresentationString[] = {"MEMS shield demo,201,9.0.0,0.0.0,IKS01A2"};
#elif (defined (USE_IKS01A3))
const uint8_t PresentationString[] = {"MEMS shield demo,201,9.0.0,0.0.0,IKS01A3"};
#elif (defined (USE_SENSORTILE))
const uint8_t PresentationString[] = {"MEMS shield demo,201,9.0.0,0.0.0,STLKT01V1"};
#elif (defined (USE_SENSORTILEBOX))
const uint8_t PresentationString[] = {"MEMS shield demo,201,9.0.0,0.0.0,MKSBOX1V1"};
#elif (defined (USE_STWIN))
const uint8_t PresentationString[] = {"MEMS shield demo,201,9.0.0,0.0.0,STWINKT1"};
#else
#error Not supported platform
#endif

const char Firmware_Info[] = {"@(#),"__DATE__","__TIME__","MCU_FAMILY","COMMUNICATION",@($)"};  

volatile uint8_t DataStreamingDest = 1;

uint32_t init_error_queue[INIT_ERROR_QUEUE_SIZE];
uint32_t init_error_queue_index;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Build the reply header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
}

/**
  * @brief  Initialize the streaming header
  * @param  Msg the pointer to the header to be initialized
  * @retval None
  */
void INIT_STREAMING_HEADER(TMsg *Msg)
{
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  Msg->Len = 3;
}

/**
  * @brief  Initialize the streaming message
  * @param  Msg the pointer to the message to be initialized
  * @retval None
  */
void INIT_STREAMING_MSG(TMsg *Msg)
{
  uint8_t i;

  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  for(i = 3; i < STREAMING_MSG_LENGTH + 3; i++)
  {
    Msg->Data[i] = 0;
  }
  Msg->Len = 3;

}

/**
  * @brief  Handle a message
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int HandleMSG(TMsg *Msg, uint32_t from_ble)
//  DestAddr | SouceAddr | CMD | PAYLOAD
//      1          1        1       N
{
  uint32_t i;

  if (Msg->Len < 2) return 0;
  if (Msg->Data[0] != DEV_ADDR) return 0;
  
  switch (Msg->Data[2])   // CMD
  {
    case CMD_Ping:
      if (Msg->Len != 3) return 0;
      
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_Enter_DFU_Mode:
      if (Msg->Len != 3) return 0;
      
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      SendMSG(Msg, from_ble);
      HAL_Delay(500);
      EnterDFU();
      return 1;

    case CMD_Read_PresString:
      if (Msg->Len != 3) return 0;
      
      BUILD_REPLY_HEADER(Msg);
      i = 0;
      while (i < (sizeof(PresentationString) - 1))
      {
        Msg->Data[3 + i] = PresentationString[i];
        i++;
      }
      Msg->Len = 3 + i;
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_Get_Streaming_Status:
      if (Msg->Len != 3) return 0;
      
      BUILD_REPLY_HEADER(Msg);
      Msg->Data[3] = DataLoggerActive;
      Msg->Len = 4;
      SendMSG(Msg, from_ble);
      return 1;
      
    case CMD_PRESSURE_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );
      
#if (defined (USE_IKS01A2))
      Serialize_s32(&Msg->Data[3], LPS22HB_UNICLEO_ID, 4);
#elif (defined (USE_IKS01A3))
      Serialize_s32(&Msg->Data[3], LPS22HH_UNICLEO_ID, 4);
#elif (defined (USE_SENSORTILE))
      Serialize_s32(&Msg->Data[3], LPS22HB_UNICLEO_ID, 4);
#elif (defined (USE_SENSORTILEBOX))
      Serialize_s32(&Msg->Data[3], LPS22HH_UNICLEO_ID, 4);
#elif (defined (USE_STWIN))
      Serialize_s32(&Msg->Data[3], LPS22HH_UNICLEO_ID, 4);
#else
#error Not supported platform
#endif
      
      Msg->Len = 3 + 4;
          
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_HUMIDITY_TEMPERATURE_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );
      
#if (defined (USE_IKS01A2))
      Serialize_s32(&Msg->Data[3], HTS221_UNICLEO_ID, 4);
#elif (defined (USE_IKS01A3))
      Serialize_s32(&Msg->Data[3], HTS221_UNICLEO_ID, 4);
#elif (defined (USE_SENSORTILE))
      Serialize_s32(&Msg->Data[3], HTS221_UNICLEO_ID, 4);
#elif (defined (USE_SENSORTILEBOX))
      Serialize_s32(&Msg->Data[3], HTS221_UNICLEO_ID, 4);
#elif (defined (USE_STWIN))
      Serialize_s32(&Msg->Data[3], HTS221_UNICLEO_ID, 4);
#else
#error Not supported platform
#endif 
      
      Msg->Len = 3 + 4;
      
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_ACCELERO_GYRO_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );

#if (defined (USE_IKS01A2))
      Serialize_s32(&Msg->Data[3], LSM6DSL_UNICLEO_ID, 4);
#elif (defined (USE_IKS01A3))
      Serialize_s32(&Msg->Data[3], LSM6DSO_UNICLEO_ID, 4);
#elif (defined (USE_SENSORTILE))
      Serialize_s32(&Msg->Data[3], LSM6DSM_UNICLEO_ID, 4);
#elif (defined (USE_SENSORTILEBOX))
      Serialize_s32(&Msg->Data[3], LSM6DSOX_UNICLEO_ID, 4);
#elif (defined (USE_STWIN))
      Serialize_s32(&Msg->Data[3], ISM330DHCX_UNICLEO_ID, 4);
#else
#error Not supported platform
#endif  

      Msg->Len = 3 + 4;

      SendMSG(Msg, from_ble); 
      return 1;

    case CMD_MAGNETO_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );

#if (defined (USE_IKS01A2))
      Serialize_s32(&Msg->Data[3], LSM303AGR_UNICLEO_ID_MAG, 4);
#elif (defined (USE_IKS01A3))
      Serialize_s32(&Msg->Data[3], LIS2MDL_UNICLEO_ID, 4);
#elif (defined (USE_SENSORTILE))
      Serialize_s32(&Msg->Data[3], LSM303AGR_UNICLEO_ID_MAG, 4);
#elif (defined (USE_SENSORTILEBOX))
      Serialize_s32(&Msg->Data[3], LIS2MDL_UNICLEO_ID, 4);
#elif (defined (USE_STWIN))
      Serialize_s32(&Msg->Data[3], IIS2MDC_UNICLEO_ID, 4);
#else
#error Not supported platform
#endif
      
      Msg->Len = 3 + 4;
      
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_Start_Data_Streaming:
      if (Msg->Len < 3) return 0;

      Sensors_Enabled = Deserialize(&Msg->Data[3], 4);
      TIM_AB_Start();
      TIM_AL_Start();
      DataLoggerActive = 1;
      
      StartTime = DWT_GetTickUS();

#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
      /* Not supported feature */
#elif ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
      if (sensor_hub_data.sdcard_enable == 1)
      {
        Datalog_Open();
      }
#else
#error Not supported platform
#endif
      
      DataStreamingDest = Msg->Data[1];
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_Stop_Data_Streaming:
      if (Msg->Len < 3) return 0;
      
      Sensors_Enabled = 0;
      DataLoggerActive = 0;
      TIM_AB_Stop();
      TIM_AL_Stop();
      
#if ((defined (USE_NUCLEO)) || (defined (USE_SENSORTILE)))
      /* Not supported feature */
#elif ((defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
      if (sensor_hub_data.sdcard_enable == 1)
      {
        Datalog_Close();
      }
#else
#error Not supported platform
#endif

      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      
#if (defined (USE_NUCLEO))
      UART_SendMsg(Msg);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))         
      if (from_ble == 0)
      {
        VCOM_send_MSG(Msg);
      }
  #if (defined (USE_BLE_OUTPUT))
      else
      {
        HAL_Delay(50);
        BLE_send_Com(Msg);
      }
  #endif
#else
#error Not supported platform
#endif
      return 1;

    case CMD_Start_Data_Sending:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      
#if (defined (USE_NUCLEO))
      UART_SendMsg(Msg);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))         
      if (from_ble == 0)
      {
        VCOM_send_MSG(Msg);
      }
  #if (defined (USE_BLE_OUTPUT))
      else
      {
        send_ble_data = TRUE;
        HAL_Delay(50);
        BLE_send_Com(Msg);
      }
  #endif
#else
#error Not supported platform
#endif
      return 1;
      
    case CMD_Stop_Data_Sending:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      
#if (defined (USE_NUCLEO))
      UART_SendMsg(Msg);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))         
      if (from_ble == 0)
      {
        VCOM_send_MSG(Msg);
      }
  #if (defined (USE_BLE_OUTPUT))
      else
      {
        send_ble_data = FALSE;
        HAL_Delay(50);
        BLE_send_Com(Msg);
      }
  #endif
#else
#error Not supported platform
#endif
      return 1;
      
    case CMD_Set_DateTime:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      RTC_DateTimeRegulate(Msg->Data[6], Msg->Data[7], Msg->Data[8], Msg->Data[3], Msg->Data[4], Msg->Data[5]);
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_Sensor:
      if (Msg->Len < 5) return 0;
      Handle_Sensor_command(Msg, from_ble);
      return 1;
      
    case CMD_Offline_Data:
      if (Msg->Len != 57) return 0;
      
      offline_data.timestamp_us = 0;
      memcpy(&offline_data.timestamp_us, &Msg->Data[3], 6);
      
      memcpy(&offline_data.pressure, &Msg->Data[9], 4);
      memcpy(&offline_data.temperature, &Msg->Data[13], 4);
      memcpy(&offline_data.humidity, &Msg->Data[17], 4);
      
      memcpy(&offline_data.acceleration_x_mg, &Msg->Data[21], 4);
      memcpy(&offline_data.acceleration_y_mg, &Msg->Data[25], 4);
      memcpy(&offline_data.acceleration_z_mg, &Msg->Data[29], 4);
      
      memcpy(&offline_data.angular_rate_x_mdps, &Msg->Data[33], 4);
      memcpy(&offline_data.angular_rate_y_mdps, &Msg->Data[37], 4);
      memcpy(&offline_data.angular_rate_z_mdps, &Msg->Data[41], 4);

      memcpy(&offline_data.magnetic_field_x_mgauss, &Msg->Data[45], 4);
      memcpy(&offline_data.magnetic_field_y_mgauss, &Msg->Data[49], 4);
      memcpy(&offline_data.magnetic_field_z_mgauss, &Msg->Data[53], 4);      
      
      offline_data.timestamp_ms += 1000 / sensor_hub_data.data_rate_Hz;
            
      if ((offline_data.timestamp_ms - last_run_16Hz) >= 62) // 16Hz
      {
        update_16Hz = 1;
        last_run_16Hz = offline_data.timestamp_ms;
      }
      
      if ((offline_data.timestamp_ms - last_run_25Hz) >= 40) // 25Hz
      {
        update_25Hz = 1;
        last_run_25Hz = offline_data.timestamp_ms;
      }
      
      if ((offline_data.timestamp_ms - last_run_50Hz) >= 20) // 50Hz
      {
        update_50Hz = 1;
        last_run_50Hz = offline_data.timestamp_ms;
      }
      
      if ((offline_data.timestamp_ms - last_run_100Hz) >= 10) // 100Hz
      {
        update_100Hz = 1;
        last_run_100Hz = offline_data.timestamp_ms;
      }
              
      DataLoggerActive = 1;
      sensor_read_request = 1;
      return 1;
      
    case CMD_GetFW_Info:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      
      memcpy(&Msg->Data[3], &sensor_hub_data.data_rate_Hz, 4);
      Msg->Len += 4;
      
      i = 0;
      while (Identification_String[i] != 0)
      {
        Msg->Data[7+i] = Identification_String[i];
        i++;  
      }
      Msg->Data[7+i] = 0;
      i++;
      Msg->Len += i;
      
      SendMSG(Msg, from_ble);
      return 1;
      
    case CMD_Report_Error:
      if (Msg->Len < 3) return 0;
      else
      {
        uint32_t count;
        uint32_t *p_uint32;
        
        BUILD_REPLY_HEADER(Msg);
        Msg->Len = 3;
     
        p_uint32 = (uint32_t*)(&(Msg->Data[7]));
        Get_Init_Error(&count, p_uint32);
        p_uint32 = (uint32_t*)(&(Msg->Data[3]));
        *p_uint32 = count;
        Msg->Len += (count + 1) * 4;
      }
      
      SendMSG(Msg, from_ble);
      return 1;

    case CMD_Get_Bin_Info:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      
      i = 0;
      while (Firmware_Info[i] != 0)
      {
        Msg->Data[3+i] = Firmware_Info[i];
        i++;
      }
      Msg->Data[3+i] = 0;
      i++;
      Msg->Len += i;
      
      SendMSG(Msg, from_ble);
      return 1;

    default:
      return 0;
  }
}

/**
 * @brief  This function initialize the error queue
 * @param  None
 * @retval None
 */
void Init_Error(void)
{
  init_error_queue_index = 0;
  memset(init_error_queue, 0, INIT_ERROR_QUEUE_SIZE * sizeof(uint32_t));
}

/**
 * @brief  This function reports errors to application
 * @param  err - reported error code
 * @retval None
 */
void Report_Error(uint32_t err)
{
  TMsg *pmsg;
  uint8_t buffer[24];
  pmsg = (TMsg*)buffer;
  pmsg->Data[0] = 1;
  pmsg->Data[1] = DEV_ADDR;
  pmsg->Data[2] = CMD_Report_Error + CMD_Reply_Add;
  uint32_t *p = (uint32_t*)(&(pmsg->Data[3]));
  *p = 1;
  p++;
  *p = err;
  pmsg->Len = 11;
#if (defined (USE_NUCLEO))
  UART_SendMsg(pmsg);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))
  #if (defined (USE_BLE_OUTPUT))
  BLE_send_Com(pmsg);
  HAL_Delay(30);
  #else
  VCOM_send_MSG(pmsg);
  #endif
#else
#error Not supported platform
#endif
}

/**
 * @brief  This function will put initialization errors into queue to not disapear,
 *         because they occured before connection was established .
 *         Application can read these errors using CMD_Report_Error command.
 * @param  err - reported error code
 * @retval None
 */
void Report_Init_Error(uint32_t err)
{
  if (init_error_queue_index < INIT_ERROR_QUEUE_SIZE)
  {
    init_error_queue[init_error_queue_index] = err;
    init_error_queue_index++;
  }
  else
  {
    init_error_queue[INIT_ERROR_QUEUE_SIZE-1] |= AB_ERROR_FLAG_OVERRUN;
  }
}

/**
 * @brief  This function will put initialization errors into buffer.
 * @param  count  - will receive number of errors in buffer
 *         buffer - will receive error list
 * @retval None
 */
void Get_Init_Error(uint32_t *count, uint32_t *buffer)
{
  *count = init_error_queue_index;
  uint32_t i;
  
  for (i=0; i<init_error_queue_index; i++)
  {
    buffer[i] = init_error_queue[i];
  }
}

void SendMSG(TMsg *Msg, uint32_t from_ble)
{
#if (defined (USE_NUCLEO))
  UART_SendMsg(Msg);
#elif ((defined (USE_SENSORTILE)) || (defined (USE_SENSORTILEBOX)) || (defined (USE_STWIN)))        
  if (from_ble == 0)
  {
    VCOM_send_MSG(Msg);
  }
  #if (defined (USE_BLE_OUTPUT))
  else
  {
    BLE_send_Com(Msg);
  }
  #endif
#else
#error Not supported platform
#endif
}

/**
 * @}
 */

/**
 * @}
 */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
