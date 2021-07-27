/**
  ******************************************************************************
  * @file    BLESensors\Src\main.c
  * @author  SRA - Central Labs
  * @version V1.0.0
  * @date    01-Sep-2019
  * @brief   Main program body
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

/**
 *
 * @page BLESensors BLE trasmission of Sensor's data
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * This firmware package includes Components Device Drivers, Board Support Package
 * and example application for the following STMicroelectronics elements:
 * - STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board that contains the following components:
 *     - MEMS sensor devices: HTS221, LPS22HH, LIS2MDL, LSM6DSOX
 *     - analog microphone 
 *
 * <b>Example Application</b>
 *
 * The Example application initializes all the Components and Library creating some Custom Bluetooth services:
 * - The first service exposes all the HW characteristics:
 *    - Temperature, Humidity, Pressure, Magnetometer, Gyroscope and Accelerometer
 *    - Battery status
 * - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 * - The last Service is used for configuration purpose
 *
 * This example must be used with the related BlueMS Android/iOS application available on Play/itune store
 * (Version 4.1.0 or higher), in order to read the sent information by Bluetooth Low Energy protocol
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "ab_ble.h"
#include "main.h"
#include "ab_sensor_hub.h"

/* Private typedef -----------------------------------------------------------*/
#pragma pack(1)
typedef struct 
{
  uint8_t not_empty;
  uint8_t config_index;
  uint16_t buffer_index;
  uint16_t size;
  uint16_t read_index;
  uint8_t msg_count;
  uint8_t msg_index;
} FFT_Buffer_Info;
#pragma pack()

/* Private define ------------------------------------------------------------*/
#define FFT_BUFFER_SIZE 5130
#define FFT_INFO_SIZE 10
#define FFT_VALUES 37

/* Imported Variables --------------------------------------------------------*/
extern sensor_hub_data_t sensor_hub_data;

/* Exported Variables --------------------------------------------------------*/

uint8_t bdaddr[6];
char BoardName[8]={NAME_BLUEMS,0};

/* Private variables ---------------------------------------------------------*/
volatile        int      hci_event          = 0;
float fft_buffer[FFT_BUFFER_SIZE];
FFT_Buffer_Info fft_buffer_info[FFT_INFO_SIZE];
uint16_t fft_buffer_unused_index = 0;
uint8_t fft_buffer_info_count = 0;
uint8_t fft_buffer_info_last = 0;

/* Private function prototypes -----------------------------------------------*/


void APP_UserEvtRx(void *pData);


void BLE_init(TMsg *Msg)
{
  setTMsgPointer(Msg);

  HCI_TL_SPI_Reset();
  
  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();
}

void BLE_handle_Event()
{
  
  if(set_connectable)
  {
    set_connectable = 0;
    setConnectable();
  }
  /* Handle BLE event */
  if(hci_event)
  {
    hci_event = 0;
    hci_user_evt_proc();
  }
  
  if (fft_buffer_info_count > 0)
  {
    if (fft_buffer_info_last < FFT_INFO_SIZE)
    {
      FFT_Buffer_Info *bi = &fft_buffer_info[fft_buffer_info_last];
      if (bi->not_empty != 0)
      {
        if (send_ble_data && W2ST_CHECK_CONNECTION(W2ST_CONNECT_ALGOB))
        {
          uint8_t data[154];
          uint32_t size = 0;
          data[0] = 1;
          data[1] = 0x32;
          data[2] = 0x7d;
          data[3] = bi->config_index;
          data[4] = bi->msg_count;
          uint32_t count = bi->size - (bi->msg_index * FFT_VALUES);
          if (count > FFT_VALUES) count = FFT_VALUES;  
          data[5] = bi->msg_index + 1;
          memcpy(&data[6], &fft_buffer[bi->buffer_index + bi->read_index], count * sizeof(float));
          size = count * sizeof(float) + 6;
          HAL_Delay(1);
          tBleStatus ret = ALGOB_Update(data, size);
           
          if (ret == BLE_STATUS_SUCCESS)
          {
             BSP_LED_Off(LED_BLE);
             bi->msg_index += 1;
             bi->read_index += count;
             if (bi->msg_count == bi->msg_index)
             {
               bi->not_empty = 0;
               bi->msg_index = 0;
               bi->read_index = 0;
               fft_buffer_info_last++;
               if (fft_buffer_info_last >= fft_buffer_info_count)
               {
                 fft_buffer_info_last = 0;
               }
             }
          }
          else
          {
            BSP_LED_On(LED_BLE);
          }
        }
      }
      else
      {
        fft_buffer_info_last++;
        if (fft_buffer_info_last >= fft_buffer_info_count)
        { 
          fft_buffer_info_last = 0;
        }
      }
    }
  }

}

void BLE_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == HCI_TL_SPI_EXTI_PIN)
  {
    hci_tl_lowlevel_isr();
    hci_event=1;
  }
}
extern int RTC_SYNCH_PREDIV;
extern RTC_HandleTypeDef RtcHandle;

tBleStatus BLE_send_Data(TMsg *Msg)
{
  if (send_ble_data && W2ST_CHECK_CONNECTION(W2ST_CONNECT_ALGOB))
  {
    tBleStatus ret = ALGOB_Update(Msg->Data, Msg->Len);
     
    if (ret == BLE_STATUS_SUCCESS)
    {
       BSP_LED_Off(LED_BLE);
    }
    else
    {
      BSP_LED_On(LED_BLE);
    }
    return ret;
  }
  return BLE_STATUS_FAILED;
}
tBleStatus BLE_send_Com(TMsg *Msg)
{
  if (send_ble_comm)
  {
    tBleStatus ret = ALGOB_COM_Update(Msg->Data, Msg->Len);
     
    if (ret == BLE_STATUS_SUCCESS)
    {
       BSP_LED_Off(LED_BLE);
    }
    else
    {
      HAL_Delay(20);
      ret = ALGOB_COM_Update(Msg->Data, Msg->Len);
       
      if (ret == BLE_STATUS_SUCCESS)
      {
         BSP_LED_Off(LED_BLE);
      }
      else
      {
         BSP_LED_On(LED_BLE);
      }
    }
    return ret;
  }
  return BLE_STATUS_FAILED;
}

/**
  * @brief  Send a FFT data via BLE
  * @param  data pointer to FFT data
  * @param  size of data (number of floats)
  * @param  config_index of related configuration record
  * @retval None
  */
tBleStatus BLE_send_FFT(float *data, uint32_t size, uint8_t config_index)
{
  uint32_t i;
  FFT_Buffer_Info *bi = 0;

  size += 1; // + ODR

  for (i=0; i<fft_buffer_info_count; i++)
  {
    if (fft_buffer_info[i].config_index == config_index)
    {
      bi = &fft_buffer_info[i];
      break;
    }
  }
  
  if (bi == 0)
  {
    if (i < FFT_INFO_SIZE)
    {
      if ((fft_buffer_unused_index + size) <= FFT_BUFFER_SIZE)
      {
        bi = &fft_buffer_info[i];
        bi->not_empty = 0;
        bi->config_index = config_index;
        bi->read_index = 0;
        bi->size = size;
        bi->buffer_index = fft_buffer_unused_index;
        bi->msg_index = 0;
        bi->msg_count = (size / FFT_VALUES) + 1;
        
        fft_buffer_unused_index += size;
        fft_buffer_info_count++;
      }
      else
      {
        //buffer limit achieved
      }
    }
    else
    {
      //error insufficient resources - only FFT_INFO_SIZE FFT's is supported
    }
  }
  
  if (bi != 0)
  {
    if (bi->not_empty == 0)
    {
      bi->not_empty = 1;
      bi->read_index = 0;
      bi->msg_index = 0;
      fft_buffer[bi->buffer_index] = (float)sensor_hub_data.meas_data_rate_Hz;
      memcpy(&fft_buffer[bi->buffer_index + 1], data, (size - 1) * sizeof(float));
    }
  }
  return BLE_STATUS_SUCCESS;
}



/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
void Init_BlueNRG_Stack(void)
{
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  tBleStatus ret;
//  uint8_t data_len_out;

  /* Initialize the BlueNRG HCI */
  hci_init(APP_UserEvtRx, NULL);

  /* we will let the BLE chip to use its Random MAC address */
//#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */
//  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, &data_len_out, bdaddr);
//
//  if(ret != BLE_STATUS_SUCCESS){
//    STBOX1_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
//    goto fail;
//  }

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  if(ret != BLE_STATUS_SUCCESS){
     STBOX1_PRINTF("\r\nSetting Public BD_ADDR failed\r\n");
     goto fail;
  }
 
  ret = aci_gatt_init();
  if(ret != BLE_STATUS_SUCCESS){
     STBOX1_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, strlen(BoardName), &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS){
     STBOX1_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(BoardName), (uint8_t *)BoardName);

  if(ret != BLE_STATUS_SUCCESS){
     STBOX1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    goto fail;
  }

  /* Set the I/O capability  Otherwise the Smartphone will propose a Pin
   * that will be acepted without any control */
  if(aci_gap_set_io_capability(IO_CAP_DISPLAY_ONLY)==BLE_STATUS_SUCCESS) {
    STBOX1_PRINTF("I/O Capability Configurated\r\n");
  } else {
    STBOX1_PRINTF("Error Setting I/O Capability\r\n");
  }

  ret = aci_gap_set_authentication_requirement(BONDING,
                                               MITM_PROTECTION_REQUIRED,
                                               SC_IS_SUPPORTED,
                                               KEYPRESS_IS_NOT_SUPPORTED,
                                               7, 
                                               16,
                                               DONOT_USE_FIXED_PIN_FOR_PAIRING,
                                               123456,
                                               0x01);
  if (ret != BLE_STATUS_SUCCESS) {
     STBOX1_PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  STBOX1_PRINTF("\r\nSERVER: BLE Stack Initialized \r\n"
         "\t\tBoardName= %s\r\n"
         "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n",
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4); /* -2,1 dBm */

  return;

fail:
  return;
}

/** @brief HCI Transport layer user function
  * @param void *pData pointer to HCI event data
  * @retval None
  */
void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT) {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT) {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++) {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code) {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    } else if(event_pckt->evt == EVT_VENDOR) {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;        

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++) {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code) {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    } else {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++) {
        if (event_pckt->evt == hci_events_table[i].evt_code) {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_ALGOB_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     STBOX1_PRINTF("ALGOB Service W2ST added successfully\r\n");
  } else {
     STBOX1_PRINTF("\r\nError while adding ALGOB Service W2ST\r\n");
  }
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off(LED_BLE);
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
