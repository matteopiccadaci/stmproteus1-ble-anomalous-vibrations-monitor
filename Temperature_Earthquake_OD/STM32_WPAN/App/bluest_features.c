/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/bluest_features.c
  * @brief   Minimal BlueST Features service (Env + NEAI AD)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "common_blesvc.h"
#include "bluest_features.h"

//With large use of AI, taken and adpted from FP-AI-PDMWBSOC2 firmware
#ifndef COPY_UUID_128
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do { \
    (uuid_struct)[0] = (uuid_0);  (uuid_struct)[1] = (uuid_1);  (uuid_struct)[2] = (uuid_2);  (uuid_struct)[3] = (uuid_3); \
    (uuid_struct)[4] = (uuid_4);  (uuid_struct)[5] = (uuid_5);  (uuid_struct)[6] = (uuid_6);  (uuid_struct)[7] = (uuid_7); \
    (uuid_struct)[8] = (uuid_8);  (uuid_struct)[9] = (uuid_9);  (uuid_struct)[10] = (uuid_10); (uuid_struct)[11] = (uuid_11); \
    (uuid_struct)[12] = (uuid_12); (uuid_struct)[13] = (uuid_13); (uuid_struct)[14] = (uuid_14); (uuid_struct)[15] = (uuid_15); \
  } while (0)
#endif

/* BlueST Features service UUID */
#define COPY_FEATURES_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11, \
                                                              0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Environmental characteristic UUID base */
#define COPY_ENVIRONMENTAL_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01, \
                                                                0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* NEAI Anomaly Detection characteristic UUID */
#define COPY_NEAI_AD_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x19,0x00,0x02, \
                                                         0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET 2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET      1

// These are the handles that make the desired characteristics visible in the app
static uint16_t s_feat_svc_hdle; 
static uint16_t s_env_char_hdle;
static uint16_t s_ad_char_hdle;

static uint8_t s_env_notify_enabled = 0;
static uint8_t s_ad_notify_enabled = 0;

static void Store_LE_16(uint8_t *buf, uint16_t val)
{
  buf[0] = (uint8_t)(val & 0xFFu);
  buf[1] = (uint8_t)((val >> 8) & 0xFFu);
}

static void Store_LE_32(uint8_t *buf, uint32_t val)
{
  buf[0] = (uint8_t)(val & 0xFFu);
  buf[1] = (uint8_t)((val >> 8) & 0xFFu);
  buf[2] = (uint8_t)((val >> 16) & 0xFFu);
  buf[3] = (uint8_t)((val >> 24) & 0xFFu);
}

static SVCCTL_EvtAckStatus_t BlueST_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value = SVCCTL_EvtNotAck;
  hci_event_pckt *event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_read_permit_req_event_rp0 *read_req;

  if (event_pckt->evt != HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE)
  {
    return return_value;
  }

  blecore_evt = (evt_blecore_aci*)event_pckt->data;
  switch (blecore_evt->ecode)
  {
    case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
      attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
      if (attribute_modified->Attr_Handle == (s_env_char_hdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        s_env_notify_enabled = (attribute_modified->Attr_Data[0] == COMSVC_Notification) ? 1u : 0u;
        return_value = SVCCTL_EvtAckFlowEnable;
      }
      else if (attribute_modified->Attr_Handle == (s_ad_char_hdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
      {
        s_ad_notify_enabled = (attribute_modified->Attr_Data[0] == COMSVC_Notification) ? 1u : 0u;
        return_value = SVCCTL_EvtAckFlowEnable;
      }
      break;

    case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE:
      read_req = (aci_gatt_read_permit_req_event_rp0*)blecore_evt->data;
      if (read_req->Attribute_Handle == (s_env_char_hdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
      {
        return_value = SVCCTL_EvtAckFlowEnable;
        aci_gatt_allow_read(read_req->Connection_Handle);
      }
      break;

    default:
      break;
  }

  return return_value;
}

void BlueST_Init(void)
{
  Char_UUID_t uuid;
  tBleStatus ret;

  SVCCTL_RegisterSvcHandler(BlueST_Event_Handler);

  /* Features service */
  COPY_FEATURES_SERVICE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *)&uuid,
                             PRIMARY_SERVICE,
                             10,
                             &s_feat_svc_hdle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return;
  }

  /* Environmental char: enable Temp1 only (uuid[14] bit 0x04) */
  COPY_ENVIRONMENTAL_CHAR_UUID(uuid.Char_UUID_128);
  uuid.Char_UUID_128[14] |= 0x04u;
  ret = aci_gatt_add_char(s_feat_svc_hdle,
                          UUID_TYPE_128,
                          &uuid,
                          4,
                          CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &s_env_char_hdle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return;
  }

  /* NEAI Anomaly Detection char */
  COPY_NEAI_AD_CHAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(s_feat_svc_hdle,
                          UUID_TYPE_128,
                          &uuid,
                          9,
                          CHAR_PROP_NOTIFY | CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &s_ad_char_hdle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    return;
  }
}

void BlueST_Update_Env(int16_t temp_tenth_c)
{
  uint8_t buff[4];

  /* timestamp (1/10 s) */
  Store_LE_16(buff, (uint16_t)(HAL_GetTick() / 10u));
  Store_LE_16(&buff[2], (uint16_t)temp_tenth_c);
  if (s_env_notify_enabled != 0u)
  {
    (void)aci_gatt_update_char_value(s_feat_svc_hdle, s_env_char_hdle, 0, sizeof(buff), buff);
  }
  else
  {
    (void)aci_gatt_update_char_value(s_feat_svc_hdle, s_env_char_hdle, 0, sizeof(buff), buff);
  }
}

void BlueST_Update_AD(uint8_t phase, uint8_t state, uint8_t progress, uint8_t status, uint8_t similarity)
{
  uint8_t buff[9];

  Store_LE_16(buff, (uint16_t)(HAL_GetTick() >> 3));
  Store_LE_16(&buff[2], 0xFFFFu);
  buff[4] = phase;
  buff[5] = state;
  buff[6] = progress;
  buff[7] = status;
  buff[8] = similarity;

  if (s_ad_notify_enabled != 0u)
  {
    (void)aci_gatt_update_char_value(s_feat_svc_hdle, s_ad_char_hdle, 0, sizeof(buff), buff);
  }
  else
  {
    (void)aci_gatt_update_char_value(s_feat_svc_hdle, s_ad_char_hdle, 0, sizeof(buff), buff);
  }
}
