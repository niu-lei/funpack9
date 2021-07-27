/**
  ******************************************************************************
  * @file    BLESensors\Src\sensor_service.c
  * @author  SRA - Central Labs
  * @version V1.0.0
  * @date    01-Sep-2019
  * @brief   Add 4 bluetooth services using vendor specific profiles.
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
#include <stdio.h>
#include "ab_ble.h"
#include "main.h"
#include "sensor_service.h"
#include "bluenrg1_l2cap_aci.h"

#include "uuid_ble_service.h"

/* Exported variables --------------------------------------------------------*/
uint8_t set_connectable = TRUE;
volatile uint8_t send_ble_data = FALSE;
volatile uint8_t send_ble_comm = FALSE;

volatile uint32_t ConnectionBleStatus  =0;

/* Imported Variables --------------------------------------------------------*/
extern uint8_t bdaddr[6];

extern char BoardName[8];

/* Private variables ---------------------------------------------------------*/
static uint16_t ALGOBServW2STHandle;
static uint16_t ALGOBCharHandle;
static uint16_t ALGOBComCharHandle;

static uint16_t connection_handle = 0;

Service_UUID_t service_uuid;
Char_UUID_t char_uuid;

TMsg *Msg;
/* Private functions ---------------------------------------------------------*/

static void Read_Request_CB(uint16_t handle);
static void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t send_msg_timeout_active = 0;
uint32_t safe_aci_gatt_update_char_value_timeout = 0;

/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle,
                      uint16_t charHandle,
                      uint8_t charValOffset,
                      uint8_t charValueLen,
                      uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;

  uint32_t t = HAL_GetTick();
  if ((send_msg_timeout_active > 0) && (t - safe_aci_gatt_update_char_value_timeout < 200))
  {
  } else
  {
    send_msg_timeout_active = 0;
    safe_aci_gatt_update_char_value_timeout = HAL_GetTick();
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);

    if (ret != BLE_STATUS_SUCCESS)
    {
      send_msg_timeout_active = 1;
    }
  }

  return ret;
}

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{
  send_msg_timeout_active = 0;
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */

void setTMsgPointer(TMsg *pmsg)
{
  Msg = pmsg;
}

/**
 * @brief  Add the ALGOB Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ALGOB_ServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_ALGOB_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 1+3*2,&ALGOBServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto EndLabel;
  }
  
  COPY_ALGOB_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(ALGOBServW2STHandle, UUID_TYPE_128, &char_uuid, ALGOB_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ALGOBCharHandle);

  COPY_ALGOB_COM_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  ret =  aci_gatt_add_char(ALGOBServW2STHandle, UUID_TYPE_128, &char_uuid, ALGOB_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ALGOBComCharHandle);
  
EndLabel:
  return ret;
}

/**
 * @brief  Update ALGOB characteristics value
 * @param  int8_t *data payload
 * @param  uint32_t size payload size
 
 * @retval tBleStatus      Status
 */
tBleStatus ALGOB_Update(uint8_t *data, uint32_t size)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  if (size > ALGOB_MAX_CHAR_LEN) size = ALGOB_MAX_CHAR_LEN;
 
  if ( W2ST_CHECK_CONNECTION(W2ST_CONNECT_ALGOB))
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(ALGOBServW2STHandle, ALGOBCharHandle, 0, size, data);
  }
  
  return ret;
}

tBleStatus ALGOB_COM_Update(uint8_t *data, uint32_t size)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  if (size > ALGOB_MAX_CHAR_LEN) size = ALGOB_MAX_CHAR_LEN;
 
  if ( W2ST_CHECK_CONNECTION(W2ST_CONNECT_ALGOB_COM))
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(ALGOBServW2STHandle, ALGOBComCharHandle, 0, size, data);
  }
  
  return ret;
}

/**
 * @brief  Puts the device in connectable mode.
 * @param  None
 * @retval None
 */
void setConnectable(void)
{
  uint8_t local_name[6] = {AD_TYPE_COMPLETE_LOCAL_NAME,BoardName[0],BoardName[1],BoardName[2],BoardName[3],BoardName[4]};
  uint8_t transmission_power[3] = {2,AD_TYPE_TX_POWER_LEVEL,0x00}; /* 0 dBm Transmission Power */
  uint8_t uuid_list[17];
  uint8_t *uuid = uuid_list + 1;
  uuid_list[0] = AD_TYPE_128_BIT_SERV_UUID;
  COPY_ALGOB_W2ST_SERVICE_UUID(uuid);

  tBleStatus RetStatus;

  /* disable scan response */
  RetStatus = hci_le_set_scan_response_data(0,NULL);
  if(RetStatus !=BLE_STATUS_SUCCESS) {
    STBOX1_PRINTF("Error hci_le_set_scan_response_data [%x]\r\n",RetStatus);
    goto EndLabel;
  }

  RetStatus = aci_gap_set_discoverable(ADV_IND, 0, 0,
                           RANDOM_ADDR,
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name,
                           17, uuid_list,
                           0, 0);
  if(RetStatus !=BLE_STATUS_SUCCESS) {
    STBOX1_PRINTF("Error aci_gap_set_discoverable [%x]\r\n",RetStatus);
  } else {
    STBOX1_PRINTF("aci_gap_set_discoverable OK\r\n");
  }

  /* Send Advertising data */
  RetStatus = aci_gap_update_adv_data(3, transmission_power);
  if(RetStatus !=BLE_STATUS_SUCCESS) {
    STBOX1_PRINTF("Error aci_gap_update_adv_data [%x]\r\n",RetStatus);
  } else {
    STBOX1_PRINTF("aci_gap_update_adv_data OK\r\n");
  }

EndLabel:
  return;
}
/**
 * @brief  Exits the device from connectable mode.
 * @param  None
 * @retval None
 */
void setNotConnectable(void)
{
  aci_gap_set_non_discoverable();
}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application
 * is subscribed or not to the one service
 * @param uint16_t Connection_Handle
 * @param uint16_t attr_handle Handle of the attribute
 * @param uint16_t Offset eventual Offset (not used)
 * @param uint8_t data_length length of the data
 * @param uint8_t *att_data attribute data
 * @retval None
 */
static void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if(attr_handle == ALGOBCharHandle + 2)
  {
     if (att_data[0] == 01)
     {
       W2ST_ON_CONNECTION(W2ST_CONNECT_ALGOB);
       send_ble_data = FALSE;
     }
     else if (att_data[0] == 0)
     {
       W2ST_OFF_CONNECTION(W2ST_CONNECT_ALGOB);
       send_ble_data = FALSE;
     }
  }
  else if(attr_handle == ALGOBComCharHandle + 2)
  {
     if (att_data[0] == 01)
     {
       W2ST_ON_CONNECTION(W2ST_CONNECT_ALGOB_COM);
       send_ble_comm = TRUE;
     }
     else if (att_data[0] == 0)
     {
       W2ST_OFF_CONNECTION(W2ST_CONNECT_ALGOB_COM);
       send_ble_comm = FALSE;
     }
  }
  else if (attr_handle == ALGOBComCharHandle + 1)
  {
    memcpy(Msg->Data, att_data, data_length);
    Msg->Len = data_length;
    HandleMSG(Msg, TRUE);
    HAL_Delay(20);
  }
}

/**
 * @brief  This function Updates the White list for BLE Connection
 * @param None
 * @retval None
 */
static void UpdateWhiteList(void)
{
  tBleStatus RetStatus;
  uint8_t NumOfAddresses;
  Bonded_Device_Entry_t BondedDeviceEntry[3];

  RetStatus =  aci_gap_get_bonded_devices(&NumOfAddresses, BondedDeviceEntry);

  if (RetStatus == BLE_STATUS_SUCCESS) {
    if (NumOfAddresses > 0) {
      STBOX1_PRINTF("Bonded with %d Device(s): \r\n", NumOfAddresses);
      RetStatus = aci_gap_configure_whitelist();
      if (RetStatus != BLE_STATUS_SUCCESS) {
        STBOX1_PRINTF("aci_gap_configure_whitelist() failed:0x%02x\r\n", RetStatus);
      } else {
        STBOX1_PRINTF("aci_gap_configure_whitelist --> SUCCESS\r\n");
      }
    }
  }
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : aci_gap_bond_lost_event
 * Description    : This event is generated on the slave when a 
 *                  ACI_GAP_SLAVE_SECURITY_REQUEST is called to reestablish the bond.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_bond_lost_event(void) {
  aci_gap_allow_rebond(connection_handle);
  STBOX1_PRINTF("aci_gap_allow_rebond()\r\n");
}

/*******************************************************************************
 * Function Name  : aci_gap_pairing_complete_event
 * Description    : This event is generated when the pairing process has completed 
 *                  successfully or a pairing procedure timeout has occurred or 
 *                  the pairing has failed
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{

  if(Status==0x00) {
    STBOX1_PRINTF("Pairing Completed\r\n");
  } else {
    STBOX1_PRINTF("Pairing Not Completed for [%s] with reason=%x\r\n",
                  (Status==0x01) ? "Timeout" : "Failed",Reason);
  }

  UpdateWhiteList();
  HAL_Delay(100);
}

/*******************************************************************************
 * Function Name  : aci_gap_pairing_complete_event
 * Description    : This command should be send by the host in response to
 *                  aci_gap_pass_key_req_event
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_pass_key_req_event(uint16_t Connection_Handle)
{
  tBleStatus status;

  status = aci_gap_pass_key_resp(connection_handle, 123456/*Peripheral_Pass_Key*/);
  if (status != BLE_STATUS_SUCCESS) {
    STBOX1_PRINTF("aci_gap_pass_key_resp failed:0x%02x\r\n", status);
  } else {
    STBOX1_PRINTF("aci_gap_pass_key_resp OK\r\n");
  }

}

/*******************************************************************************
 * Function Name  : aci_gap_numeric_comparison_value_event
 * Description    : This event is sent only during SC v.4.2 Pairing
 *                  when Numeric Comparison Association model is selected
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gap_numeric_comparison_value_event(uint16_t Connection_Handle, uint32_t Numeric_Value)
{
  STBOX1_PRINTF("aci_gap_numeric_comparison_value_event Numeric_Value=%ld\r\n",Numeric_Value);

  /* Confirm Yes... without control of Numeric Value received from Master */
  aci_gap_numeric_comparison_value_confirm_yesno(Connection_Handle,0x01);
}

/*******************************************************************************
 * Function Name  : hci_encryption_change_event
 * Description    : It is used to indicate that the change of the encryption
 *                  mode has been completed
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_encryption_change_event(uint8_t Status,uint16_t Connection_Handle,uint8_t Encryption_Enabled)
{
  STBOX1_PRINTF("hci_encryption_change_event\r\n");  
}

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{ 
  /* Save Current device's information */
  connection_handle = Connection_Handle;

  ConnectionBleStatus=0;
  //send_ble_data = FALSE;

  /* Just in order to be sure to switch off the User Led */
  LedOffTargetPlatform();

  HAL_Delay(100);

}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{  
  /* No Device Connected */
  connection_handle =0;

  /* Make the device connectable again. */
  set_connectable = TRUE;
  //send_ble_data = FALSE;
  ConnectionBleStatus=0;

  HAL_Delay(100);

}/* end hci_disconnection_complete_event() */

/*******************************************************************************
 * Function Name  : aci_gatt_read_permit_req_event.
 * Description    : This event is given when a read request is received
 *                  by the server from the client.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  Read_Request_CB(Attribute_Handle);    
}

/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event is given when an attribute change his value.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_Request_CB(Connection_Handle, Attr_Handle, Offset, Attr_Data_Length, Attr_Data);
}

void hci_read_remote_version_information_complete_event(uint8_t Status,
                                                        uint16_t Connection_Handle,
                                                        uint8_t Version,
                                                        uint16_t Manufacturer_Name,
                                                        uint16_t Subversion)
{
  STBOX1_PRINTF("\r\nhci_read_remote_version_information_complete_event\r\n"
                "\tStatus = 0x%X"
                "\tConnection_Handle = 0x%X"
                "\tVersion = 0x%X"
                "\tManufacturer_Name = 0x%X"
                "\tSubversion = 0x%X", Status, Connection_Handle, Version, Manufacturer_Name, Subversion);
}

void Printf_Data(uint8_t len, uint8_t *data)
{
  uint8_t buffer[256];
  TMsg *pmsg = (TMsg*)buffer;
  pmsg->Len = 0;
  uint8_t i;
  for (i=0; i<len; i++)
  {
    if (i%8 == 0 && i != 0)
    {
      VCOM_send_DBG(pmsg);
      pmsg->Len = 0;
    }
    pmsg->Len += sprintf((char *)(&pmsg->Data[pmsg->Len]), "%02X ", data[i]);
  }
  VCOM_send_DBG(pmsg);
}

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
