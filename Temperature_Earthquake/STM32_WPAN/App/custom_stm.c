/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @brief   Custom GATT service (TMPEQ) for LightBlue
  ******************************************************************************
  */
/* USER CODE END Header */

#include "common_blesvc.h"
#include "custom_stm.h"

/* UUID base: 0000xxxx-8E22-4541-9D4C-21EDAE82ED19 */
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do { \
    (uuid_struct)[0] = (uuid_0);  (uuid_struct)[1] = (uuid_1);  (uuid_struct)[2] = (uuid_2);  (uuid_struct)[3] = (uuid_3); \
    (uuid_struct)[4] = (uuid_4);  (uuid_struct)[5] = (uuid_5);  (uuid_struct)[6] = (uuid_6);  (uuid_struct)[7] = (uuid_7); \
    (uuid_struct)[8] = (uuid_8);  (uuid_struct)[9] = (uuid_9);  (uuid_struct)[10] = (uuid_10); (uuid_struct)[11] = (uuid_11); \
    (uuid_struct)[12] = (uuid_12); (uuid_struct)[13] = (uuid_13); (uuid_struct)[14] = (uuid_14); (uuid_struct)[15] = (uuid_15); \
  } while (0)

#define COPY_TMPEQ_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x8E,0x22,0x45,0x41,0x9D,0x4C,0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_TEMP_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x8E,0x22,0x45,0x41,0x9D,0x4C,0x21,0xED,0xAE,0x82,0xED,0x19)
#define COPY_EQ_ANOM_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x03,0x8E,0x22,0x45,0x41,0x9D,0x4C,0x21,0xED,0xAE,0x82,0xED,0x19)

#define UUID_CHAR_USER_DESC                  (0x2901)

#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET 2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET      1

typedef struct
{
  uint16_t ServiceHdle;
  uint16_t TempCharHdle;
  uint16_t EqAnomCharHdle;
} CustomContext_t;

static CustomContext_t s_custom_ctx;

static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value = SVCCTL_EvtNotAck;
  hci_event_pckt *event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_read_permit_req_event_rp0 *read_req;
  Custom_STM_App_Notification_evt_t notification;

  if (event_pckt->evt != HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE)
  {
    return return_value;
  }

  blecore_evt = (evt_blecore_aci*)event_pckt->data;
  switch (blecore_evt->ecode)
  {
    case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
      attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
      if (attribute_modified->Attr_Handle == (s_custom_ctx.TempCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        if (attribute_modified->Attr_Data[0] & COMSVC_Notification)
        {
          notification.Custom_Evt_Opcode = CUSTOM_STM_TEMP_VAL_NOTIFY_ENABLED_EVT;
        }
        else
        {
          notification.Custom_Evt_Opcode = CUSTOM_STM_TEMP_VAL_NOTIFY_DISABLED_EVT;
        }
        notification.ConnectionHandle = attribute_modified->Connection_Handle;
        notification.DataTransferedLength = 0u;
        Custom_STM_App_Notification(&notification);
        return_value = SVCCTL_EvtAckFlowEnable;
      }
      else if (attribute_modified->Attr_Handle == (s_custom_ctx.EqAnomCharHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        if (attribute_modified->Attr_Data[0] & COMSVC_Notification)
        {
          notification.Custom_Evt_Opcode = CUSTOM_STM_EQ_ANOM_NOTIFY_ENABLED_EVT;
        }
        else
        {
          notification.Custom_Evt_Opcode = CUSTOM_STM_EQ_ANOM_NOTIFY_DISABLED_EVT;
        }
        notification.ConnectionHandle = attribute_modified->Connection_Handle;
        notification.DataTransferedLength = 0u;
        Custom_STM_App_Notification(&notification);
        return_value = SVCCTL_EvtAckFlowEnable;
      }
      break;

    case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE:
      read_req = (aci_gatt_read_permit_req_event_rp0*)blecore_evt->data;
      if (read_req->Attribute_Handle == (s_custom_ctx.TempCharHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
      {
        notification.Custom_Evt_Opcode = CUSTOM_STM_TEMP_VAL_READ_EVT;
        notification.ConnectionHandle = read_req->Connection_Handle;
        notification.DataTransferedLength = 0u;
        Custom_STM_App_Notification(&notification);
        aci_gatt_allow_read(read_req->Connection_Handle);
        return_value = SVCCTL_EvtAckFlowEnable;
      }
      break;

    default:
      break;
  }

  return return_value;
}

void Custom_STM_Init(void) //Here the Characteristics are set
{
  Char_UUID_t uuid;
  Char_Desc_Uuid_t desc_uuid;
  tBleStatus ret;
  uint16_t desc_handle;
  const uint8_t temp_desc[] = "Temperature";
  const uint8_t eq_desc[] = "Earthquake";

  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  COPY_TMPEQ_SERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *)&uuid,
                             PRIMARY_SERVICE,
                             12,
                             &s_custom_ctx.ServiceHdle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return;
  }

  COPY_TEMP_CHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(s_custom_ctx.ServiceHdle,
                          UUID_TYPE_128,
                          &uuid,
                          2,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &s_custom_ctx.TempCharHdle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return;
  }

  desc_uuid.Char_UUID_16 = UUID_CHAR_USER_DESC;
  (void)aci_gatt_add_char_desc(s_custom_ctx.ServiceHdle,
                               s_custom_ctx.TempCharHdle,
                               UUID_TYPE_16,
                               &desc_uuid,
                               sizeof(temp_desc) - 1u,
                               sizeof(temp_desc) - 1u,
                               temp_desc,
                               ATTR_PERMISSION_NONE,
                               ATTR_ACCESS_READ_ONLY,
                               GATT_DONT_NOTIFY_EVENTS,
                               0x10,
                               CHAR_VALUE_LEN_CONSTANT,
                               &desc_handle);

  COPY_EQ_ANOM_CHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(s_custom_ctx.ServiceHdle,
                          UUID_TYPE_128,
                          &uuid,
                          2,
                          CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &s_custom_ctx.EqAnomCharHdle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return;
  }

  desc_uuid.Char_UUID_16 = UUID_CHAR_USER_DESC;
  (void)aci_gatt_add_char_desc(s_custom_ctx.ServiceHdle,
                               s_custom_ctx.EqAnomCharHdle,
                               UUID_TYPE_16,
                               &desc_uuid,
                               sizeof(eq_desc) - 1u,
                               sizeof(eq_desc) - 1u,
                               eq_desc,
                               ATTR_PERMISSION_NONE,
                               ATTR_ACCESS_READ_ONLY,
                               GATT_DONT_NOTIFY_EVENTS,
                               0x10,
                               CHAR_VALUE_LEN_CONSTANT,
                               &desc_handle);
}

tBleStatus Custom_STM_Update_Char(Custom_STM_Char_Opcode_t char_opcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_FAILED;

  switch (char_opcode)
  {
    case CUSTOM_STM_TEMP_VAL:
      ret = aci_gatt_update_char_value(s_custom_ctx.ServiceHdle,
                                       s_custom_ctx.TempCharHdle,
                                       0,
                                       2,
                                       pPayload);
      break;

    case CUSTOM_STM_EQ_ANOM:
      ret = aci_gatt_update_char_value(s_custom_ctx.ServiceHdle,
                                       s_custom_ctx.EqAnomCharHdle,
                                       0,
                                       2,
                                       pPayload);
      break;

    default:
      break;
  }

  return ret;
}
