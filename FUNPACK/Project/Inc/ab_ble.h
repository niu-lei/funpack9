
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AB_BLE_H
#define __AB_BLE_H

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"

#include "hci.h"
#include "hci_tl.h"
#include "hci_tl_interface.h"
#include "STBOX1_config.h"
#include "sensor_service.h"
#include "serial_protocol.h"

#if (defined (USE_SENSORTILEBOX))
#include "SensorTile.box.h"
#include "SensorTile.box_bc.h"
#include "SensorTile.box_env_sensors.h"
#include "SensorTile.box_motion_sensors.h"
#define LED_BLE LED_BLUE
#elif (defined (USE_STWIN))
#include "STWIN.h"
#include "STWIN_bc.h"
#include "STWIN_env_sensors.h"
#include "STWIN_motion_sensors.h"
#define LED_BLE LED_ORANGE
#elif (defined (USE_NUCLEO))
/* No specific include required */
#elif (defined (USE_SENSORTILE))
/* No specific include required */
#else
#error Not supported platform
#endif
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
/* Exported defines and variables  ------------------------------------------------------- */

extern uint8_t set_connectable;
extern volatile uint8_t send_ble_data;
extern volatile uint8_t send_ble_comm;

/* Exported defines ------------------------------------------------------- */
/* Every second for Led Blinking and for Updating the Battery status */
#define STBOX1_UPDATE_LED_BATTERY 10000
  
/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */

/* Exported variables ------------------------------------------------------- */
extern TIM_HandleTypeDef    TimCCHandle;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(void);
extern void LedInitTargetPlatform(void);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);
extern void LedToggleTargetPlatform(void);

extern void Init_BlueNRG_Custom_Services(void);
extern void Init_BlueNRG_Stack(void);

extern void BLE_init(TMsg *Msg);
extern tBleStatus BLE_send_Data(TMsg *Msg);
extern tBleStatus BLE_send_Com(TMsg *Msg);
extern tBleStatus BLE_send_FFT(float *data, uint32_t size, uint8_t config_index);
extern void BLE_EXTI_Callback(uint16_t GPIO_Pin);
void BLE_handle_Event(void);

#ifdef __cplusplus
}
#endif

#endif /* __AB_BLE_H */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
