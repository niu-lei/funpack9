/**
 ******************************************************************************
 * @file       sensor_commands.c
 * @author     MEMS Application Team
 * @brief      Handle commands for sensor
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
#include "sensor_commands.h"
#include "demo_serial.h"
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

/** @addtogroup AlgoBuilder_Firmware
  * @{
  */

/** @addtogroup Configuration
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern sDISPLAY_INFO display_info_list[];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Handle Sensors command
  * @param  Msg the pointer to the message to be handled
  * @param  custom_values the pointer to the custom values
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int Handle_Sensor_command(TMsg *Msg, uint32_t from_ble)
{
  /* Commands */
  switch (Msg->Data[3])
  {
    case SC_GET_CONFIG_STRING:
      return SC_Get_Config_String(Msg, Msg->Data[4], from_ble);

    case SC_GET_CUSTOM_CONFIG:
      return SC_Get_Custom_Config(Msg, from_ble);
    
    case SC_SET_CUSTOM_VALUES:
      return SC_Set_Custom_Values(Msg, from_ble);

    case SC_GET_CUSTOM_VALUES:
      return SC_Get_Custom_Values(Msg, from_ble);
      
    default:
      return 0;
  }
}

int SC_Get_Custom_Config(TMsg *Msg, uint32_t from_ble)
{
  int i = 0;
  int j = 0;
  int conn_index;
  int index;
  sCONFIG_RECORD *cr;

  BUILD_REPLY_HEADER(Msg);

  Msg->Data[5] = 0; //number of records
  index = 1;
  i = 0;
  
  while (display_info_list[i].info_index != 0)
  {
    display_info_list[i].already_processed = 0;
    i++;
  }
  i = 0;
  while (display_info_list[i].info_index != 0)
  {
    if (display_info_list[i].info_type > INFO_TYPE_AFLOAT) //output configurations only
    {
      if (display_info_list[i].already_processed == 0) 
      {
        cr = (sCONFIG_RECORD*)&(Msg->Data[5 + index]);
        display_info_list[i].already_processed = 1;  //mark as already processed
        cr->var_count =  display_info_list[i].variable_count;
        cr->var_type = display_info_list[i].variable_type;
        conn_index = 0;
        cr->conn[conn_index++] = display_info_list[i].info_index;
        cr->conn_size = 1;
        Msg->Data[5]++; //number of records
        if (display_info_list[i].info_type != INFO_TYPE_FFT)
        {
          j=0;
          while (display_info_list[j].info_index != 0)
          {
            if (display_info_list[j].info_type > INFO_TYPE_AFLOAT) //output configurations only
            {
              if (display_info_list[j].already_processed == 0)
              {
                if (display_info_list[j].stream_position == display_info_list[i].stream_position &&
                    display_info_list[j].info_type != INFO_TYPE_FFT) //add other outputs with the same stream address
                {
                  cr->conn[conn_index++] = display_info_list[j].info_index;
                  cr->conn_size++;
                  display_info_list[j].already_processed = 1;  //mark as already processed
                }
              }
            }
            j++;
          }
        }
        index += 3 + conn_index;  //add connections count
      }
    }
    i++;
  }
  //input configurations
  i = 0;
  while (display_info_list[i].info_index != 0)
  {
    if (display_info_list[i].info_type <= INFO_TYPE_AFLOAT) //input configurations only
    {
      if (display_info_list[i].already_processed == 0) 
      {
        cr = (sCONFIG_RECORD*)&(Msg->Data[5 + index]);
        display_info_list[i].already_processed = 1;  //mark as already processed
        cr->var_count =  display_info_list[i].variable_count;
        cr->var_type = display_info_list[i].variable_type;
        cr->var_type += VAR_TYPE_INPUT;
        cr->conn[0] = display_info_list[i].info_index;
        cr->conn_size = 1;
        Msg->Data[5]++; //number of records
        index += 4;
      }
    }
    i++;
  }

  Msg->Len = 5 + index;  
  SendMSG(Msg, from_ble);
  return 1;
}

int SC_Get_Config_String(TMsg *Msg, uint8_t id, uint32_t from_ble)
{
  int i = 0;
  int j = 0;
  BUILD_REPLY_HEADER(Msg);

  while (display_info_list[i].info_index != 0)
  {
    if (display_info_list[i].info_index == id)
    {
      for (j = 0; j < strlen(display_info_list[i].config_string); j++)
      {
            Msg->Data[5 + j] = display_info_list[i].config_string[j];
      }
    }
    i++;
  }

  Msg->Len = 5 + j;
  SendMSG(Msg, from_ble);
  return 1;
}

int SC_Set_Custom_Values(TMsg *Msg, uint32_t from_ble)
{
  int i = 0;
  while (display_info_list[i].info_index != 0)
  {
    if (display_info_list[i].info_index == Msg->Data[4])
    {
      if (display_info_list[i].info_type == INFO_TYPE_ABITS)
      {
        if (display_info_list[i].p_node != 0)
        {
          for (int j=0; j<display_info_list[i].variable_count; j++)
          {
            ((uint32_t*)(display_info_list[i].p_node))[j] = Msg->Data[5] & (1 << j) ? 1 : 0;
          }
        }
      }
      if (display_info_list[i].info_type == INFO_TYPE_AINT32)
      {
        if (display_info_list[i].p_node != 0)
        {
          memcpy(display_info_list[i].p_node, Msg->Data + 5, display_info_list[i].variable_count * 4);
        }
      }
      if (display_info_list[i].info_type == INFO_TYPE_AFLOAT)
      {
        if (display_info_list[i].p_node != 0)
        {
          memcpy(display_info_list[i].p_node, Msg->Data + 5, display_info_list[i].variable_count * 4);
        }
      }
    }
    i++;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Len = 5;  
  SendMSG(Msg, from_ble);  
  return 1;
}

int SC_Get_Custom_Values(TMsg *Msg, uint32_t from_ble)
{
  int i = 0;
  int j = 0;
  while (display_info_list[i].info_index != 0)
  {
    if (display_info_list[i].info_index == Msg->Data[4])
    {
      if (display_info_list[i].info_type == INFO_TYPE_ABITS)
      {
        if (display_info_list[i].p_node != 0)
        {
          Msg->Data[5] = 0;
          for (j=0; j<display_info_list[i].variable_count; j++)
          {
            Msg->Data[5] |= ((uint32_t*)(display_info_list[i].p_node))[j] != 0 ? (1 << j) : 0;
          }
        }
        Msg->Len = 6;
        break;
      }
      if (display_info_list[i].info_type == INFO_TYPE_AINT32)
      {
        if (display_info_list[i].p_node != 0)
        {
          memcpy(Msg->Data + 5, display_info_list[i].p_node, display_info_list[i].variable_count * 4);
        }
        Msg->Len = 5 + display_info_list[i].variable_count * 4;
        break;
      }
      if (display_info_list[i].info_type == INFO_TYPE_AFLOAT)
      {
        if (display_info_list[i].p_node != 0)
        {
          memcpy(Msg->Data + 5, display_info_list[i].p_node, display_info_list[i].variable_count * 4);
        }
        Msg->Len = 5 + display_info_list[i].variable_count * 4;
        break;
      }
    }
    i++;
  }

  BUILD_REPLY_HEADER(Msg);
  SendMSG(Msg, from_ble);  
  return 1;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
