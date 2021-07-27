/**
  ******************************************************************************
  * @file    BLESensors\Inc\sensor_service.h 
  * @author  SRA - Central Labs
  * @version V1.0.0
  * @date    01-Sep-2019
  * @brief   Sensors services APIs
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
#ifndef _SENSOR_SERVICE_H_
#define _SENSOR_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
//#include "TargetFeatures.h"
#if (defined (USE_SENSORTILEBOX))
#include "SensorTile.box_motion_sensors.h"
#elif (defined (USE_STWIN))
#include "STWIN_motion_sensors.h"
#elif (defined (USE_NUCLEO))
/* No specific include required */
#elif (defined (USE_SENSORTILE))
/* No specific include required */
#else
#error Not supported platform
#endif
   
#include "bluenrg1_hal_aci.h"
#include "bluenrg1_gatt_aci.h"
#include "bluenrg1_gap_aci.h"
#include "bluenrg1_hci_le.h"

#include <stdlib.h>
#include "demo_serial.h"
#include "vcom.h"

/* Exported Defines --------------------------------------------------------*/
/*
  #define STBOX1_PRINTF(...) do {\
    uint8_t buffer[128];\
    TMsg *pmsg = (TMsg*)buffer;\
    pmsg->Len = sprintf((char *)(pmsg->Data), __VA_ARGS__);\
    VCOM_send_DBG(pmsg);\
  } while (0); 

  #define STBOX1_PRINTF_DATA(len, data) Printf_Data(len, data);
*/
  #define STBOX1_PRINTF(...) 
  #define STBOX1_PRINTF_DATA(len, data)


/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

#ifdef ACC_BLUENRG_CONGESTION
/* For defining how many events skip when there is a congestion */
#define ACC_BLUENRG_CONGESTION_SKIP 10
#endif /* ACC_BLUENRG_CONGESTION */

/* Define the Max dimesion of the Bluetooth characteristics for each packet  */
#define ALGOB_MAX_CHAR_LEN 155

/* BLE Characteristic connection control */
   
/* BLE Characteristic connection control */

   /* ALGOB Feature */
#define W2ST_CONNECT_ALGOB         (1)
#define W2ST_CONNECT_ALGOB_COM     (2)

#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

/* Exported Types ------------------------------------------------------- */

/* Exported Variables ------------------------------------------------------- */
extern volatile uint32_t ConnectionBleStatus;

/* Exported functions ------------------------------------------------------- */
extern tBleStatus Add_ALGOB_ServW2ST_Service(void);
extern tBleStatus ALGOB_Update(uint8_t *data, uint32_t size);
extern tBleStatus ALGOB_COM_Update(uint8_t *data, uint32_t size);
extern void       setConnectable(void);
extern void       setNotConnectable(void);
extern void       setConnectionParameters(int min , int max, int latency , int timeout );
extern void       HCI_Event_CB(void *pckt);
extern void       setTMsgPointer(TMsg *pmsg);
extern void       Printf_Data(uint8_t len, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
