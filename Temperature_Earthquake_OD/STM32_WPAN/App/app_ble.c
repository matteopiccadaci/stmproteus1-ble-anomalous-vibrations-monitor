/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_ble.c
  * @author  MCD Application Team
  * @brief   BLE Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"

#include "svc_ctl.h"
#include "custom_app.h"

/* USER CODE BEGIN Includes */
#include "app_conf.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  uint8_t ioCapability;
  uint8_t mitm_mode;
  uint8_t bonding_mode;
  uint8_t encryptionKeySizeMin;
  uint8_t encryptionKeySizeMax;
  uint8_t initiateSecurity;
} tSecurityParams;

typedef struct _tBLEProfileGlobalContext
{
  tSecurityParams bleSecurityParam;
  uint16_t gapServiceHandle;
  uint16_t devNameCharHandle;
  uint16_t appearanceCharHandle;
  uint16_t connectionHandle;

  uint8_t advtServUUIDlen;
  uint8_t advtServUUID[100];
} BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;

  uint8_t Advertising_mgr_timer_Id;
  uint8_t SwitchOffGPIO_timer_Id;
} BleApplicationContext_t;

/* Private defines -----------------------------------------------------------*/
#define FAST_ADV_TIMEOUT        (30*1000*1000/CFG_TS_TICK_VAL)  /**< 30s */
#define INITIAL_ADV_TIMEOUT     (60*1000*1000/CFG_TS_TICK_VAL)  /**< 60s */

#define BD_ADDR_SIZE_LOCAL      6
#define BLE_DEFAULT_PIN         (111111)

/* Private macro -------------------------------------------------------------*/
#ifndef ADV_TYPE
#define ADV_TYPE GAP_GENERAL_DISCOVERABLE_MODE
#endif

#ifndef ADV_FILTER
#define ADV_FILTER NO_WHITE_LIST_USE
#endif

#ifndef AD_TYPE_MANUFACTURER_SPECIFIC_DATA
#define AD_TYPE_MANUFACTURER_SPECIFIC_DATA  (0xFFu)
#endif

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t a_MBdAddr[BD_ADDR_SIZE_LOCAL] =
{
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
};

static uint8_t a_BdAddrUdn[BD_ADDR_SIZE_LOCAL];

static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IR;
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ER;

PLACE_IN_SECTION("TAG_OTA_END")   const uint32_t MagicKeywordValue   = 0x94448A29;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

static BleApplicationContext_t BleApplicationContext;
static uint16_t AdvIntervalMin, AdvIntervalMax;

Custom_App_ConnHandle_Not_evt_t HandleNotification;

// Here is the part taken from the original firmware to make it display on ST BLE Sensor app
static const uint8_t a_LocalName[] =
{
  AD_TYPE_COMPLETE_LOCAL_NAME,
  'T','E','M','P','E','Q','A'
};

// This part was not automatically included from the CubeMX editor


#define ST_BLE_MANUF_ID_L  (0x30u)
#define ST_BLE_MANUF_ID_H  (0x00u)
#define ST_BLE_SDK_VERSION (0x02u)
#define ST_BLE_BOARD_ID    (0x0Fu) // STEVAL-PROTEUS1 


#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
#define SIZE_TAB_CONN_INT 2
float a_ConnInterval[SIZE_TAB_CONN_INT] = {50, 1000}; /* ms */
uint8_t index_con_int, mutex;
#endif

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx(void *p_Payload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t Status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress(void);
static void Build_StBleSensor_Adv(uint8_t *adv_data, uint8_t *adv_len);
static void Build_StaticRandomAddr(uint8_t out_addr[6]);

static void Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Adv_Cancel(void);
static void Adv_Cancel_Req(void);
static void Switch_OFF_GPIO(void);

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
static void BLE_SVC_L2CAP_Conn_Update(uint16_t ConnectionHandle);
static void Connection_Interval_Update_Req(void);
#endif

static void Build_StBleSensor_Adv(uint8_t *adv_data, uint8_t *adv_len)
{
  uint8_t idx = 0;
  const uint8_t *bd = BleGetBdAddress();

  /* Flags */
  adv_data[idx++] = 0x02;
  adv_data[idx++] = AD_TYPE_FLAGS;
  adv_data[idx++] = FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED;

  /* Complete local name (7 chars like ST pack) */
  adv_data[idx++] = 8u;
  adv_data[idx++] = AD_TYPE_COMPLETE_LOCAL_NAME;
  adv_data[idx++] = 'T';
  adv_data[idx++] = 'E';
  adv_data[idx++] = 'M';
  adv_data[idx++] = 'P';
  adv_data[idx++] = 'E';
  adv_data[idx++] = 'Q';
  adv_data[idx++] = 'A';

  /* Manufacturer specific (BlueST, SDK v2 layout) */
  adv_data[idx++] = 15u;
  adv_data[idx++] = AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
  adv_data[idx++] = ST_BLE_MANUF_ID_L;
  adv_data[idx++] = ST_BLE_MANUF_ID_H;
  adv_data[idx++] = ST_BLE_SDK_VERSION;
  adv_data[idx++] = ST_BLE_BOARD_ID;
  adv_data[idx++] = 0x00; /* FW ID */
  adv_data[idx++] = 0x00; /* custom bytes */
  adv_data[idx++] = 0x00;
  adv_data[idx++] = 0x00;
  /* BLE MAC, MSB first */
  adv_data[idx++] = bd[5];
  adv_data[idx++] = bd[4];
  adv_data[idx++] = bd[3];
  adv_data[idx++] = bd[2];
  adv_data[idx++] = bd[1];
  adv_data[idx++] = bd[0];

  *adv_len = idx;
}


static void Build_StaticRandomAddr(uint8_t out_addr[6])
{
  const uint8_t *bd = BleGetBdAddress();

  out_addr[0] = bd[0];
  out_addr[1] = bd[1];
  out_addr[2] = bd[2];
  out_addr[3] = bd[3];
  out_addr[4] = bd[4];
  /* Set two MSB to 1 for static random address */
  out_addr[5] = (uint8_t)((bd[5] & 0x3Fu) | 0xC0u);
}

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(void)
{
  SHCI_CmdStatus_t status;

  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},
    {
      0,
      0,
      CFG_BLE_NUM_GATT_ATTRIBUTES,
      CFG_BLE_NUM_GATT_SERVICES,
      CFG_BLE_ATT_VALUE_ARRAY_SIZE,
      CFG_BLE_NUM_LINK,
      CFG_BLE_DATA_LENGTH_EXTENSION,
      CFG_BLE_PREPARE_WRITE_LIST_SIZE,
      CFG_BLE_MBLOCK_COUNT,
      CFG_BLE_MAX_ATT_MTU,
      CFG_BLE_PERIPHERAL_SCA,
      CFG_BLE_CENTRAL_SCA,
      CFG_BLE_LS_SOURCE,
      CFG_BLE_MAX_CONN_EVENT_LENGTH,
      CFG_BLE_HSE_STARTUP_TIME,
      CFG_BLE_VITERBI_MODE,
      CFG_BLE_OPTIONS,
      0,
      CFG_BLE_MAX_COC_INITIATOR_NBR,
      CFG_BLE_MIN_TX_POWER,
      CFG_BLE_MAX_TX_POWER,
      CFG_BLE_RX_MODEL_CONFIG,
      CFG_BLE_MAX_ADV_SET_NBR,
      CFG_BLE_MAX_ADV_DATA_LEN,
      CFG_BLE_TX_PATH_COMPENS,
      CFG_BLE_RX_PATH_COMPENS,
      CFG_BLE_CORE_VERSION,
      CFG_BLE_OPTIONS_EXT,
      CFG_BLE_MAX_ADD_EATT_BEARERS
    }
  };

  /* Initialize BLE Transport Layer */
  Ble_Tl_Init();

  /* Do not allow standby in the application */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /* Register HCI async events task */
  UTIL_SEQ_RegTask(1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /* Start BLE Stack on CPU2 */
  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
  if (status != SHCI_Success)
  {
    APP_DBG_MSG("Fail: SHCI_C2_BLE_Init result: 0x%02x\n\r", status);
    Error_Handler();
  }

  /* Init HCI/GATT/GAP */
  Ble_Hci_Gap_Gatt_Init();

  /* Init services */
  SVCCTL_Init();

  /* Init app context */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

  /* Register ADV cancel task */
  UTIL_SEQ_RegTask(1<<CFG_TASK_ADV_CANCEL_ID, UTIL_SEQ_RFU, Adv_Cancel);

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
  UTIL_SEQ_RegTask(1<<CFG_TASK_CONN_UPDATE_REG_ID, UTIL_SEQ_RFU, Connection_Interval_Update_Req);
  index_con_int = 0;
  mutex = 1;
#endif

  /* Init Custom Application */
  Custom_APP_Init();

  /* Create timers (usati dal template) */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Cancel_Req);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.SwitchOffGPIO_timer_Id), hw_ts_SingleShot, Switch_OFF_GPIO);

  /* Intervalli ADV */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /* Start advertising */
  Adv_Request(APP_BLE_FAST_ADV);
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
  hci_event_pckt    *p_event_pckt;
  evt_le_meta_event *p_meta_evt;
  evt_blecore_aci   *p_blecore_evt;

  hci_le_connection_complete_event_rp0 *p_connection_complete_event;
  hci_disconnection_complete_event_rp0 *p_disconnection_complete_event;

  /* pairing */
  aci_gap_pairing_complete_event_rp0 *p_pairing_complete;

  p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;

  switch (p_event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;

      if (p_disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
      {
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;
        BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

        /* Notifica custom app */
        HandleNotification.Custom_Evt_Opcode = CUSTOM_DISCON_HANDLE_EVT;
        HandleNotification.ConnectionHandle = 0;
        Custom_APP_Notification(&HandleNotification);
      }

      /* restart advertising */
      Adv_Request(APP_BLE_FAST_ADV);
      break;
    }

    case HCI_LE_META_EVT_CODE:
    {
      p_meta_evt = (evt_le_meta_event*) p_event_pckt->data;

      switch (p_meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          p_connection_complete_event = (hci_le_connection_complete_event_rp0 *) p_meta_evt->data;

          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

          BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = p_connection_complete_event->Connection_Handle;

          /* Notifica custom app */
          HandleNotification.Custom_Evt_Opcode = CUSTOM_CONN_HANDLE_EVT;
          HandleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          Custom_APP_Notification(&HandleNotification);

          break;
        }

        default:
          break;
      }
      break;
    }

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
    {
      p_blecore_evt = (evt_blecore_aci*) p_event_pckt->data;

      switch (p_blecore_evt->ecode)
      {
        case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
        {
          uint32_t pin = BLE_DEFAULT_PIN;
          (void)aci_gap_pass_key_resp(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, pin);
          break;
        }

        case ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE:
        {
          (void)aci_gap_numeric_comparison_value_confirm_yesno(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, YES);
          break;
        }

        case ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE:
        {
          p_pairing_complete = (aci_gap_pairing_complete_event_rp0*)p_blecore_evt->data;
          UNUSED(p_pairing_complete);
          break;
        }

        case ACI_GATT_INDICATION_VSEVT_CODE:
        {
          (void)aci_gatt_confirm_indication(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
          break;
        }

        default:
          break;
      }
      break;
    }

    default:
      break;
  }

  return SVCCTL_UserEvtFlowEnable;
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
  return BleApplicationContext.Device_Connection_Status;
}

/*************************************************************
 * LOCAL FUNCTIONS
 *************************************************************/
static void Ble_Tl_Init(void)
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;
  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);
}

static void Ble_Hci_Gap_Gatt_Init(void)
{
  uint8_t role = 0;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *p_bd_addr;
  uint8_t random_addr[6];
  uint16_t a_appearance[1] = {BLE_CFG_GAP_APPEARANCE};
  tBleStatus ret;

  /* HCI reset */
  ret = hci_reset();
  UNUSED(ret);

  /* BD address */
  p_bd_addr = BleGetBdAddress();
  (void)aci_hal_write_config_data(CONFIG_DATA_PUBLIC_ADDRESS_OFFSET,
                                 CONFIG_DATA_PUBLIC_ADDRESS_LEN,
                                 (uint8_t*)p_bd_addr);
  /* Static random address (BlueST uses static random) */
  Build_StaticRandomAddr(random_addr);
  (void)aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET,
                                 CONFIG_DATA_RANDOM_ADDRESS_LEN,
                                 random_addr);

  /* IR/ER */
  (void)aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)a_BLE_CfgIrValue);
  (void)aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)a_BLE_CfgErValue);

  /* TX power */
  (void)aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /* GATT */
  (void)aci_gatt_init();

  /* GAP roles */
#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif
#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  /* GAP init + set device name in GAP Service */
  if (role > 0)
  {
    const char *name = CFG_GAP_DEVICE_NAME;

    (void)aci_gap_init(role,
                       CFG_PRIVACY,
                       CFG_GAP_DEVICE_NAME_LENGTH,
                       &gap_service_handle,
                       &gap_dev_name_char_handle,
                       &gap_appearance_char_handle);

    (void)aci_gatt_update_char_value(gap_service_handle,
                                     gap_dev_name_char_handle,
                                     0,
                                     strlen(name),
                                     (uint8_t *)name);

    (void)aci_gatt_update_char_value(gap_service_handle,
                                     gap_appearance_char_handle,
                                     0,
                                     2,
                                     (uint8_t *)&a_appearance);
  }

  /* Default PHY */
  (void)hci_le_set_default_phy(ALL_PHYS_PREFERENCE, TX_2M_PREFERRED, RX_2M_PREFERRED);

  /* IO cap + auth req */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  (void)aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;

  (void)aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               USE_FIXED_PIN_FOR_PAIRING_FORBIDDEN,
                                               0,
                                               CFG_IDENTITY_ADDRESS);

  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    (void)aci_gap_configure_whitelist();
  }
}

static void Adv_Request(APP_BLE_ConnStatus_t NewStatus)
{
  tBleStatus ret;
  uint16_t Min_Inter, Max_Inter;
  uint8_t adv_data[31];
  uint8_t adv_len = 0;
  uint8_t adv_addr_type = GAP_STATIC_RANDOM_ADDR;

  if (NewStatus == APP_BLE_FAST_ADV)
  {
    Min_Inter = AdvIntervalMin;
    Max_Inter = AdvIntervalMax;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

  /* stop timer (safe) */
  HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

  /* If switching ADV mode, stop previous adv */
  if ((NewStatus == APP_BLE_LP_ADV) &&
      ((BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV) ||
       (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_ADV)))
  {
    (void)aci_gap_set_non_discoverable();
  }

  BleApplicationContext.Device_Connection_Status = NewStatus;

  /* Avvio advertising standard, poi override con frame BlueST */
  ret = aci_gap_set_discoverable(ADV_TYPE,
                                 Min_Inter,
                                 Max_Inter,
                                 adv_addr_type,
                                 ADV_FILTER,
                                 sizeof(a_LocalName),
                                 (uint8_t*)a_LocalName,
                                 0,
                                 NULL,
                                 0, 0);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("aci_gap_set_discoverable FAIL: 0x%x\n", ret);
    return;
  }

  /* Override ADV data with BlueST manufacturer frame so ST BLE Sensor recognizes the board */
  Build_StBleSensor_Adv(adv_data, &adv_len);
  if (adv_len > 0u)
  {
    (void)aci_gap_update_adv_data(adv_len, adv_data);
  }


  if (NewStatus == APP_BLE_FAST_ADV)
  {
    HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, INITIAL_ADV_TIMEOUT);
  }
}

static const uint8_t* BleGetBdAddress(void)
{
  uint8_t *p_otp_addr;
  const uint8_t *p_bd_addr;
  uint32_t udn, company_id, device_id;

  udn = LL_FLASH_GetUDN();

  if (udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id  = LL_FLASH_GetDeviceID();

    a_BdAddrUdn[0] = (uint8_t)(udn & 0x000000FF);
    a_BdAddrUdn[1] = (uint8_t)((udn & 0x0000FF00) >> 8);
    a_BdAddrUdn[2] = (uint8_t)device_id;
    a_BdAddrUdn[3] = (uint8_t)(company_id & 0x000000FF);
    a_BdAddrUdn[4] = (uint8_t)((company_id & 0x0000FF00) >> 8);
    a_BdAddrUdn[5] = (uint8_t)((company_id & 0x00FF0000) >> 16);

    p_bd_addr = (const uint8_t *)a_BdAddrUdn;
  }
  else
  {
    p_otp_addr = OTP_Read(0);
    if (p_otp_addr)
    {
      p_bd_addr = ((OTP_ID0_t*)p_otp_addr)->bd_address;
    }
    else
    {
      p_bd_addr = a_MBdAddr;
    }
  }

  return p_bd_addr;
}

/* ADV cancel ---------------------------------------------------------------*/
static void Adv_Cancel(void)
{
  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_SERVER)
  {
    (void)aci_gap_set_non_discoverable();
    BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  }
}

static void Adv_Cancel_Req(void)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_CANCEL_ID, CFG_SCH_PRIO_0);
}

static void Switch_OFF_GPIO(void)
{
  /* optional */
}

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
void BLE_SVC_L2CAP_Conn_Update(uint16_t ConnectionHandle)
{
  if (mutex == 1)
  {
    mutex = 0;
    index_con_int = (index_con_int + 1) % SIZE_TAB_CONN_INT;
    uint16_t interval_min = CONN_P(a_ConnInterval[index_con_int]);
    uint16_t interval_max = CONN_P(a_ConnInterval[index_con_int]);

    (void)aci_l2cap_connection_parameter_update_req(ConnectionHandle,
                                                    interval_min, interval_max,
                                                    L2CAP_PERIPHERAL_LATENCY,
                                                    L2CAP_TIMEOUT_MULTIPLIER);
  }
}

static void Connection_Interval_Update_Req(void)
{
  if (BleApplicationContext.Device_Connection_Status != APP_BLE_FAST_ADV &&
      BleApplicationContext.Device_Connection_Status != APP_BLE_IDLE)
  {
    BLE_SVC_L2CAP_Conn_Update(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
  }
}
#endif

/*************************************************************
 * WRAP FUNCTIONS
 *************************************************************/
void hci_notify_asynch_evt(void* p_Data)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
}

void hci_cmd_resp_release(uint32_t Flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
}

void hci_cmd_resp_wait(uint32_t Timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
}

static void BLE_UserEvtRx(void *p_Payload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *p_param = (tHCI_UserEvtRxParam *)p_Payload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(p_param->pckt->evtserial));
  p_param->status = (svctl_return_status != SVCCTL_UserEvtFlowDisable)
                  ? HCI_TL_UserEventFlow_Enable
                  : HCI_TL_UserEventFlow_Disable;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t Status)
{
  uint32_t task_id_list;

  switch (Status)
  {
    case HCI_TL_CmdBusy:
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);
      break;

    case HCI_TL_CmdAvailable:
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);
      break;

    default:
      break;
  }
}

void SVCCTL_ResumeUserEventFlow(void)
{
  hci_resume_flow();
}
