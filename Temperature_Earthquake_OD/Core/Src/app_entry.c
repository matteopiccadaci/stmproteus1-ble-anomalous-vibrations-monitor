/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_entry.c
  * @author  MCD Application Team
  * @brief   Entry point of the application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "main.h"
#include "app_entry.h"
#include "app_ble.h"
#include "ble.h"
#include "tl.h"
#include "stm32_seq.h"
#include "shci_tl.h"
#include "stm32_lpm.h"
#include "app_debug.h"
#include "dbg_trace.h"
#include "shci.h"
#include "otp.h"

/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "custom_app.h"
#include "NanoEdgeAI.h"
#include "custom_app.h"
#include "i2c.h"
#include "spi.h"
#include "../../Drivers/BSP/Components/ism330dhcx/ism330dhcx_reg.h"
#include "../../Drivers/BSP/Components/ism330dhcx/ism330dhcx_reg.c"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define POOL_SIZE (CFG_TLBLE_EVT_QUEUE_LENGTH*4U*DIVC((sizeof(TL_PacketHeader_t) + TL_BLE_EVENT_FRAME_SIZE), 4U))

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t EvtPool[POOL_SIZE];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t SystemCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t SystemSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t BleSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions prototypes-----------------------------------------------*/
static void Config_HSE(void);
static void Reset_Device(void);
#if (CFG_HW_RESET_BY_FW == 1)
static void Reset_IPCC(void);
static void Reset_BackupDomain(void);
#endif /* CFG_HW_RESET_BY_FW == 1*/
static void System_Init(void);
static void SystemPower_Config(void);
static void appe_Tl_Init(void);
static void APPE_SysStatusNot(SHCI_TL_CmdStatus_t status);
static void APPE_SysUserEvtRx(void * pPayload);
static void APPE_SysEvtReadyProcessing(void * pPayload);
static void APPE_SysEvtError(void * pPayload);
static void Init_Rtc(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void MX_APPE_Config(void)
{
  /**
   * The OPTVERR flag is wrongly set at power on
   * It shall be cleared before using any HAL_FLASH_xxx() api
   */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  /**
   * Reset some configurations so that the system behave in the same way
   * when either out of nReset or Power On
   */
  Reset_Device();

  /* Configure HSE Tuning */
  Config_HSE();

  return;
}

void MX_APPE_Init(void)
{
  System_Init();       /**< System initialization */

  SystemPower_Config(); /**< Configure the system Power Mode */

  HW_TS_Init(hw_ts_InitMode_Full, &hrtc); /**< Initialize the TimerServer */

/* USER CODE BEGIN APPE_Init_1 */
APP_DBG_MSG("SERIAL\r\n");

/* USER CODE END APPE_Init_1 */
  appe_Tl_Init();	/* Initialize all transport layers */

  /**
   * From now, the application is waiting for the ready event (VS_HCI_C2_Ready)
   * received on the system channel before starting the Stack
   * This system event is received with APPE_SysUserEvtRx()
   */
/* USER CODE BEGIN APPE_Init_2 */

/* USER CODE END APPE_Init_2 */

   return;
}

void Init_Smps(void)
{
#if (CFG_USE_SMPS != 0)
  /**
   *  Configure and enable SMPS
   *
   *  The SMPS configuration is not yet supported by CubeMx
   *  when SMPS output voltage is set to 1.4V, the RF output power is limited to 3.7dBm
   *  the SMPS output voltage shall be increased for higher RF output power
   */
  LL_PWR_SMPS_SetStartupCurrent(LL_PWR_SMPS_STARTUP_CURRENT_80MA);
  LL_PWR_SMPS_SetOutputVoltageLevel(LL_PWR_SMPS_OUTPUT_VOLTAGE_1V40);
  LL_PWR_SMPS_Enable();
#endif /* CFG_USE_SMPS != 0 */

  return;
}

void Init_Exti(void)
{
  /* Enable IPCC(36), HSEM(38) wakeup interrupts on CPU1 */
  LL_EXTI_EnableIT_32_63(LL_EXTI_LINE_36 | LL_EXTI_LINE_38);

  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Reset_Device(void)
{
#if (CFG_HW_RESET_BY_FW == 1)
  Reset_BackupDomain();

  Reset_IPCC();
#endif /* CFG_HW_RESET_BY_FW == 1 */

  return;
}

#if (CFG_HW_RESET_BY_FW == 1)
static void Reset_BackupDomain(void)
{
  if ((LL_RCC_IsActiveFlag_PINRST() != FALSE) && (LL_RCC_IsActiveFlag_SFTRST() == FALSE))
  {
    HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */

    /**
     *  Write twice the value to flush the APB-AHB bridge
     *  This bit shall be written in the register before writing the next one
     */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
  }

  return;
}

static void Reset_IPCC(void)
{
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_IPCC);

  LL_C1_IPCC_ClearFlag_CHx(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_ClearFlag_CHx(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableTransmitChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableTransmitChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableReceiveChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableReceiveChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  return;
}
#endif /* CFG_HW_RESET_BY_FW == 1 */

static void Config_HSE(void)
{
    OTP_ID0_t * p_otp;

  /**
   * Read HSE_Tuning from OTP
   */
  p_otp = (OTP_ID0_t *) OTP_Read(0);
  if (p_otp)
  {
    LL_RCC_HSE_SetCapacitorTuning(p_otp->hse_tuning);
  }

  return;
}

static void System_Init(void)
{
  Init_Smps();

  Init_Exti();

  Init_Rtc();

  return;
}

static void Init_Rtc(void)
{
  /* Disable RTC registers write protection */
  LL_RTC_DisableWriteProtection(RTC);

  LL_RTC_WAKEUP_SetClock(RTC, CFG_RTC_WUCKSEL_DIVIDER);

  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);

  return;
}

/**
 * @brief  Configure the system for power optimization
 *
 * @note  This API configures the system to be ready for low power mode
 *
 * @param  None
 * @retval None
 */
static void SystemPower_Config(void)
{
  /**
   * Select HSI as system clock source after Wake Up from Stop mode
   */
  LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);

  /* Initialize low power manager */
  UTIL_LPM_Init();
  /* Initialize the CPU2 reset value before starting CPU2 with C2BOOT */
  LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);

#if (CFG_USB_INTERFACE_ENABLE != 0)
  /**
   *  Enable USB power
   */
  HAL_PWREx_EnableVddUSB();
#endif /* CFG_USB_INTERFACE_ENABLE != 0 */

  return;
}

static void appe_Tl_Init(void)
{
  TL_MM_Config_t tl_mm_config;
  SHCI_TL_HciInitConf_t SHci_Tl_Init_Conf;
  /**< Reference table initialization */
  TL_Init();

  /**< System channel initialization */
  UTIL_SEQ_RegTask(1<< CFG_TASK_SYSTEM_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, shci_user_evt_proc);
  SHci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&SystemCmdBuffer;
  SHci_Tl_Init_Conf.StatusNotCallBack = APPE_SysStatusNot;
  shci_init(APPE_SysUserEvtRx, (void*) &SHci_Tl_Init_Conf);

  /**< Memory Manager channel initialization */
  tl_mm_config.p_BleSpareEvtBuffer = BleSpareEvtBuffer;
  tl_mm_config.p_SystemSpareEvtBuffer = SystemSpareEvtBuffer;
  tl_mm_config.p_AsynchEvtPool = EvtPool;
  tl_mm_config.AsynchEvtPoolSize = POOL_SIZE;
  TL_MM_Init(&tl_mm_config);

  TL_Enable();

  return;
}

static void APPE_SysStatusNot(SHCI_TL_CmdStatus_t status)
{
  UNUSED(status);
  return;
}

/**
 * The type of the payload for a system user event is tSHCI_UserEvtRxParam
 * When the system event is both :
 *    - a ready event (subevtcode = SHCI_SUB_EVT_CODE_READY)
 *    - reported by the FUS (sysevt_ready_rsp == FUS_FW_RUNNING)
 * The buffer shall not be released
 * (eg ((tSHCI_UserEvtRxParam*)pPayload)->status shall be set to SHCI_TL_UserEventFlow_Disable)
 * When the status is not filled, the buffer is released by default
 */
static void APPE_SysUserEvtRx(void * pPayload)
{
  TL_AsynchEvt_t *p_sys_event;
  WirelessFwInfo_t WirelessInfo;

  p_sys_event = (TL_AsynchEvt_t*)(((tSHCI_UserEvtRxParam*)pPayload)->pckt->evtserial.evt.payload);

  switch(p_sys_event->subevtcode)
  {
  case SHCI_SUB_EVT_CODE_READY:
    /* Read the firmware version of both the wireless firmware and the FUS */
    SHCI_GetWirelessFwInfo(&WirelessInfo);
    APP_DBG_MSG("Wireless Firmware version %d.%d.%d\n", WirelessInfo.VersionMajor, WirelessInfo.VersionMinor, WirelessInfo.VersionSub);
    APP_DBG_MSG("Wireless Firmware build %d\n", WirelessInfo.VersionReleaseType);
    APP_DBG_MSG("FUS version %d.%d.%d\n", WirelessInfo.FusVersionMajor, WirelessInfo.FusVersionMinor, WirelessInfo.FusVersionSub);

    APP_DBG_MSG(">>== SHCI_SUB_EVT_CODE_READY\n\r");
    APPE_SysEvtReadyProcessing(pPayload);
    break;

  case SHCI_SUB_EVT_ERROR_NOTIF:
    APP_DBG_MSG(">>== SHCI_SUB_EVT_ERROR_NOTIF \n\r");
    APPE_SysEvtError(pPayload);
    break;

  case SHCI_SUB_EVT_BLE_NVM_RAM_UPDATE:
    APP_DBG_MSG(">>== SHCI_SUB_EVT_BLE_NVM_RAM_UPDATE -- BLE NVM RAM HAS BEEN UPDATED BY CPU2 \n");
    APP_DBG_MSG("     - StartAddress = %lx , Size = %ld\n",
                ((SHCI_C2_BleNvmRamUpdate_Evt_t*)p_sys_event->payload)->StartAddress,
                ((SHCI_C2_BleNvmRamUpdate_Evt_t*)p_sys_event->payload)->Size);
    break;

  case SHCI_SUB_EVT_NVM_START_WRITE:
    APP_DBG_MSG("==>> SHCI_SUB_EVT_NVM_START_WRITE : NumberOfWords = %ld\n",
                ((SHCI_C2_NvmStartWrite_Evt_t*)p_sys_event->payload)->NumberOfWords);
    break;

  case SHCI_SUB_EVT_NVM_END_WRITE:
    APP_DBG_MSG(">>== SHCI_SUB_EVT_NVM_END_WRITE\n\r");
    break;

  case SHCI_SUB_EVT_NVM_START_ERASE:
    APP_DBG_MSG("==>>SHCI_SUB_EVT_NVM_START_ERASE : NumberOfSectors = %ld\n",
                ((SHCI_C2_NvmStartErase_Evt_t*)p_sys_event->payload)->NumberOfSectors);
    break;

  case SHCI_SUB_EVT_NVM_END_ERASE:
    APP_DBG_MSG(">>== SHCI_SUB_EVT_NVM_END_ERASE\n\r");
    break;

  default:
    break;
  }

  return;
}

/**
 * @brief Notify a system error coming from the M0 firmware
 * @param  ErrorCode  : errorCode detected by the M0 firmware
 *
 * @retval None
 */
static void APPE_SysEvtError(void * pPayload)
{
  TL_AsynchEvt_t *p_sys_event;
  SCHI_SystemErrCode_t *p_sys_error_code;

  p_sys_event = (TL_AsynchEvt_t*)(((tSHCI_UserEvtRxParam*)pPayload)->pckt->evtserial.evt.payload);
  p_sys_error_code = (SCHI_SystemErrCode_t*) p_sys_event->payload;

  APP_DBG_MSG(">>== SHCI_SUB_EVT_ERROR_NOTIF WITH REASON %x \n\r",(*p_sys_error_code));

  if ((*p_sys_error_code) == ERR_BLE_INIT)
  {
    /* Error during BLE stack initialization */
    APP_DBG_MSG(">>== SHCI_SUB_EVT_ERROR_NOTIF WITH REASON - ERR_BLE_INIT \n");
  }
  else
  {
    APP_DBG_MSG(">>== SHCI_SUB_EVT_ERROR_NOTIF WITH REASON - BLE ERROR \n");
  }
  return;
}

static void APPE_SysEvtReadyProcessing(void * pPayload)
{
  TL_AsynchEvt_t *p_sys_event;
  SHCI_C2_Ready_Evt_t *p_sys_ready_event;

  SHCI_C2_CONFIG_Cmd_Param_t config_param = {0};
  uint32_t RevisionID=0;
  uint32_t DeviceID=0;

  p_sys_event = (TL_AsynchEvt_t*)(((tSHCI_UserEvtRxParam*)pPayload)->pckt->evtserial.evt.payload);
  p_sys_ready_event = (SHCI_C2_Ready_Evt_t*) p_sys_event->payload;

  if (p_sys_ready_event->sysevt_ready_rsp == WIRELESS_FW_RUNNING)
  {
    /**
    * The wireless firmware is running on the CPU2
    */
    APP_DBG_MSG(">>== WIRELESS_FW_RUNNING \n");

    /* Traces channel initialization */
    APPD_EnableCPU2();

    /* Enable all events Notification */
    config_param.PayloadCmdSize = SHCI_C2_CONFIG_PAYLOAD_CMD_SIZE;
    config_param.EvtMask1 = SHCI_C2_CONFIG_EVTMASK1_BIT0_ERROR_NOTIF_ENABLE
      +  SHCI_C2_CONFIG_EVTMASK1_BIT1_BLE_NVM_RAM_UPDATE_ENABLE
        +  SHCI_C2_CONFIG_EVTMASK1_BIT2_THREAD_NVM_RAM_UPDATE_ENABLE
          +  SHCI_C2_CONFIG_EVTMASK1_BIT3_NVM_START_WRITE_ENABLE
            +  SHCI_C2_CONFIG_EVTMASK1_BIT4_NVM_END_WRITE_ENABLE
              +  SHCI_C2_CONFIG_EVTMASK1_BIT5_NVM_START_ERASE_ENABLE
                +  SHCI_C2_CONFIG_EVTMASK1_BIT6_NVM_END_ERASE_ENABLE;

    /* Read revision identifier */
    /**
    * @brief  Return the device revision identifier
    * @note   This field indicates the revision of the device.
    * @rmtoll DBGMCU_IDCODE REV_ID        LL_DBGMCU_GetRevisionID
    * @retval Values between Min_Data=0x00 and Max_Data=0xFFFF
    */
    RevisionID = LL_DBGMCU_GetRevisionID();

    APP_DBG_MSG(">>== DBGMCU_GetRevisionID= %lx \n\r", RevisionID);

    config_param.RevisionID = (uint16_t)RevisionID;

    DeviceID = LL_DBGMCU_GetDeviceID();
    APP_DBG_MSG(">>== DBGMCU_GetDeviceID= %lx \n\r", DeviceID);
    config_param.DeviceID = (uint16_t)DeviceID;
    (void)SHCI_C2_Config(&config_param);

    APP_BLE_Init();
    UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
  }
  else if (p_sys_ready_event->sysevt_ready_rsp == FUS_FW_RUNNING)
  {
    /**
    * The FUS firmware is running on the CPU2
    * In the scope of this application, there should be no case when we get here
    */
    APP_DBG_MSG(">>== SHCI_SUB_EVT_CODE_READY - FUS_FW_RUNNING \n\r");

    /* The packet shall not be released as this is not supported by the FUS */
    ((tSHCI_UserEvtRxParam*)pPayload)->status = SHCI_TL_UserEventFlow_Disable;
  }
  else
  {
    APP_DBG_MSG(">>== SHCI_SUB_EVT_CODE_READY - UNEXPECTED CASE \n\r");
  }

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
/* ===== ISM330DHCX minimal SPI wrapper (reg driver) ===== */
static stmdev_ctx_t s_ism_ctx;
static uint8_t s_ism_inited = 0;

#define ISM330_SPI_TIMEOUT_MS  (100u)
#define ISM330_SPI            (&hspi1)

#define ISM330_CS_GPIO_Port    SPI_ISM330DHCX_CS_GPIO_Port
#define ISM330_CS_Pin          SPI_ISM330DHCX_CS_Pin

static void ISM330_FifoReset(void);
static void ISM330_FifoInit(void);

static inline void ISM330_CS_LOW(void)
{
  HAL_GPIO_WritePin(ISM330_CS_GPIO_Port, ISM330_CS_Pin, GPIO_PIN_RESET);
}

static inline void ISM330_CS_HIGH(void)
{
  HAL_GPIO_WritePin(ISM330_CS_GPIO_Port, ISM330_CS_Pin, GPIO_PIN_SET);
}

/* NOTE SPI:
   - bit7=1 -> read
   - bit6=1 -> auto-increment (multi-byte)
*/
static int32_t ism_platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;

  uint8_t addr = (uint8_t)(reg & 0x7Fu);
  if (len > 1u)
  {
    addr |= 0x40u;
  }

  ISM330_CS_LOW();
  if (HAL_SPI_Transmit(hspi, &addr, 1, ISM330_SPI_TIMEOUT_MS) != HAL_OK)
  {
    ISM330_CS_HIGH();
    return -1;
  }

  if (HAL_SPI_Transmit(hspi, (uint8_t*)bufp, len, ISM330_SPI_TIMEOUT_MS) != HAL_OK)
  {
    ISM330_CS_HIGH();
    return -1;
  }

  ISM330_CS_HIGH();
  return 0;
}

static int32_t ism_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)handle;

  uint8_t addr = (uint8_t)(reg | 0x80u);
  if (len > 1u)
  {
    addr |= 0x40u;
  }

  ISM330_CS_LOW();
  if (HAL_SPI_Transmit(hspi, &addr, 1, ISM330_SPI_TIMEOUT_MS) != HAL_OK)
  {
    ISM330_CS_HIGH();
    return -1;
  }

  if (HAL_SPI_Receive(hspi, bufp, len, ISM330_SPI_TIMEOUT_MS) != HAL_OK)
  {
    ISM330_CS_HIGH();
    return -1;
  }

  ISM330_CS_HIGH();
  return 0;
}

static void ISM330_InitOnce(void)
{
  if (s_ism_inited) return;

  s_ism_ctx.write_reg = ism_platform_write;
  s_ism_ctx.read_reg  = ism_platform_read;
  s_ism_ctx.handle    = (void*)ISM330_SPI;

  /* CS idle high */
  ISM330_CS_HIGH();
  HAL_Delay(5);

  uint8_t whoami = 0;
  if (ism330dhcx_device_id_get(&s_ism_ctx, &whoami) != 0)
  {
    APP_DBG_MSG("ISM330DHCX WHOAMI read FAIL\\r\\n");
  }
  APP_DBG_MSG("ISM330DHCX WHOAMI = 0x%02X\\r\\n", whoami);

  /* reset */
  (void)ism330dhcx_reset_set(&s_ism_ctx, 1);
  HAL_Delay(20);

  (void)ism330dhcx_block_data_update_set(&s_ism_ctx, PROPERTY_ENABLE);
  (void)ism330dhcx_auto_increment_set(&s_ism_ctx, PROPERTY_ENABLE);

  (void)ism330dhcx_xl_full_scale_set(&s_ism_ctx, ISM330DHCX_8g);
  (void)ism330dhcx_xl_data_rate_set(&s_ism_ctx, ISM330DHCX_XL_ODR_1666Hz);
  ISM330_FifoInit();

  s_ism_inited = 1;
}

static void ISM330_FifoReset(void)
{
  (void)ism330dhcx_fifo_mode_set(&s_ism_ctx, ISM330DHCX_BYPASS_MODE);
  (void)ism330dhcx_fifo_mode_set(&s_ism_ctx, ISM330DHCX_STREAM_MODE);
}

static void ISM330_FifoInit(void)
{
  (void)ism330dhcx_fifo_mode_set(&s_ism_ctx, ISM330DHCX_BYPASS_MODE);
  (void)ism330dhcx_fifo_watermark_set(&s_ism_ctx, 1);
  (void)ism330dhcx_fifo_stop_on_wtm_set(&s_ism_ctx, 0);
  (void)ism330dhcx_fifo_xl_batch_set(&s_ism_ctx, ISM330DHCX_XL_BATCHED_AT_1667Hz);
  (void)ism330dhcx_fifo_gy_batch_set(&s_ism_ctx, ISM330DHCX_GY_NOT_BATCHED);
  (void)ism330dhcx_fifo_temp_batch_set(&s_ism_ctx, ISM330DHCX_TEMP_NOT_BATCHED);
  (void)ism330dhcx_fifo_timestamp_decimation_set(&s_ism_ctx, ISM330DHCX_NO_DECIMATION);
  (void)ism330dhcx_fifo_mode_set(&s_ism_ctx, ISM330DHCX_STREAM_MODE);
}

static int ISM330_ReadAcc_mg(float *ax, float *ay, float *az)
{
  uint8_t data[6];
  int16_t raw[3] = {0};
  ism330dhcx_fifo_tag_t tag;

  if (ism330dhcx_fifo_sensor_tag_get(&s_ism_ctx, &tag) != 0)
    return -1;
  if (ism330dhcx_fifo_out_raw_get(&s_ism_ctx, data) != 0)
    return -1;

  if (tag != ISM330DHCX_XL_NC_TAG &&
      tag != ISM330DHCX_XL_NC_T_1_TAG &&
      tag != ISM330DHCX_XL_NC_T_2_TAG &&
      tag != ISM330DHCX_XL_2XC_TAG &&
      tag != ISM330DHCX_XL_3XC_TAG)
  {
    return 2; /* not an accelerometer sample */
  }

  raw[0] = (int16_t)((uint16_t)data[1] << 8 | data[0]);
  raw[1] = (int16_t)((uint16_t)data[3] << 8 | data[2]);
  raw[2] = (int16_t)((uint16_t)data[5] << 8 | data[4]);

  /* FS=8g -> mg */
  float mx = ism330dhcx_from_fs8g_to_mg(raw[0]); //This function (8g) because I set this scale in the datalogger firmware generated by NanoEdgeAI
  float my = ism330dhcx_from_fs8g_to_mg(raw[1]);
  float mz = ism330dhcx_from_fs8g_to_mg(raw[2]);

  *ax = mx;
  *ay = my;
  *az = mz;
  return 0;
} // Taken from the Original example Original example 
/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += HAL_GetTickFreq();
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
    /************************************************************************************
     * ENTER SLEEP MODE
     ***********************************************************************************/
    LL_LPM_EnableSleep(); /**< Clear SLEEPDEEP bit of Cortex System Control Register */

    /**
     * This option is used to ensure that store operations are completed
     */
  #if defined (__CC_ARM) || defined (__ARMCC_VERSION)
    __force_stores();
  #endif /* __ARMCC_VERSION */

    __WFI();
  }
}

#define NEAI_STILL_DYN_MG    80.0f
#define NEAI_STILL_CLEAR_HITS 3u

#define NEAI_EXPECTED_LEN    ((uint16_t)(NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER))

void MX_APPE_Process(void)
{
  /* USER CODE BEGIN MX_APPE_Process_1 */
  static uint8_t  neai_inited  = 0;
  static uint8_t  old_anomaly = 0;
  static uint8_t  still_cnt = 0;
  static float    g_est_mg = 1000.0f;
  static float    neai_input[NEAI_EXPECTED_LEN];
  static uint16_t neai_pos = 0;

  static uint32_t send_t0   = 0;
  static uint32_t ble_led_t0 = 0; // time for BLE LED

  const uint32_t send_period_ms   = 200u;  
  const float    G_ALPHA          = 0.01f;  
  extern uint16_t Connection_Handle; // Generica GATT
  APP_BLE_ConnStatus_t ble_st = APP_BLE_Get_Server_Connection_Status(); // APP BlueST
  uint32_t ble_led_period_ms = (ble_st == APP_BLE_CONNECTED_SERVER) ? 1000u : 200u; // Blink 1 sec if connected, 200ms if not

  if ((HAL_GetTick() - ble_led_t0) >= ble_led_period_ms)
  {
    ble_led_t0 = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
  }

  if (Connection_Handle == 0u) // reset if not connected
  {
    neai_inited = 0;
    old_anomaly = 0; // Was there an anomaly?
    still_cnt = 0;
    g_est_mg = 1000.0f;
    neai_pos = 0;
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }
  else
  {
    if (!neai_inited)
    {
      ISM330_InitOnce();
      enum neai_state init_state = neai_outlier_init();
      old_anomaly = 0;
      still_cnt = 0;
      g_est_mg = 1000.0f;
      neai_pos = 0;
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
      ISM330_FifoReset();

      neai_inited = (init_state == NEAI_OK) ? 1u : 0u;
      send_t0     = HAL_GetTick();
    }

    uint32_t now = HAL_GetTick();

    /* Lettura FIFO IMU */
    if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)
    {
      uint8_t fifo_ovr = 0u;
      uint16_t fifo_level = 0u;

      (void)ism330dhcx_fifo_ovr_flag_get(&s_ism_ctx, &fifo_ovr);
      if (fifo_ovr != 0u)
      {
        ISM330_FifoReset();
      }

      if (ism330dhcx_fifo_data_level_get(&s_ism_ctx, &fifo_level) == 0)
      {
        uint16_t to_read = fifo_level;
        if (to_read > 256u) to_read = 256u; // Ensure the limit for each cycle is 256 (since we are sure the correct buffer size is 512)

        for (uint16_t i = 0; i < to_read; i++)
        {
          float ax=0.0f, ay=0.0f, az=0.0f;

          int acc_rc = ISM330_ReadAcc_mg(&ax, &ay, &az);
          if (acc_rc == 2)
          {
            continue;
          }
          if (acc_rc != 0)
          {
            break;
          }

          float mag = sqrtf((ax * ax) + (ay * ay) + (az * az));
          g_est_mg = g_est_mg + G_ALPHA * (mag - g_est_mg);
          float dyn = fabsf(mag - g_est_mg); // This is an evaluation to understand if we are still or moving
          if ((neai_pos + 2u) < NEAI_EXPECTED_LEN) // to be sure that we read exactly 3 values
          {
            float ax_s = ax * 0.001f; // from mg to G
            float ay_s = ay * 0.001f;
            float az_s = az * 0.001f;

            neai_input[neai_pos++] = ax_s;
            neai_input[neai_pos++] = ay_s;
            neai_input[neai_pos++] = az_s;
          }

          if (neai_pos >= NEAI_EXPECTED_LEN) // When we are sure that we took 3 values (X,Y,Z)
          {
            neai_pos = 0;
            uint8_t is_outlier = 0u;
            enum neai_state ds = NEAI_NOT_INITIALIZED;

            if (neai_inited != 0u)
            {
              ds = neai_outlier(neai_input, &is_outlier);
              if (ds != NEAI_OK)
              {
                neai_inited = 0u;
              }
            }

            if (ds == NEAI_OK)
            {

              if (dyn <= NEAI_STILL_DYN_MG) // Still
              {
                if (still_cnt < 255u) still_cnt++;
              }
              else // Moving
              {
                still_cnt = 0u;
              }

              if (still_cnt >= NEAI_STILL_CLEAR_HITS) // Clear anomaly if still for enough windows
              {
                is_outlier = 0u;
                old_anomaly = 0u;
              }
              else
              {
                old_anomaly = (is_outlier != 0u) ? 1u : 0u;
              }
            }

            if ((ds == NEAI_OK) && ((now - send_t0) >= send_period_ms))
            {
              send_t0 = now;
              Custom_APP_Send_Anomaly(old_anomaly, old_anomaly, 0u);
            }
          }
        }
      }
    }
  }

  Custom_APP_Process(); // Function that activates it all

  /* USER CODE END MX_APPE_Process_1 */

  UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);

  /* USER CODE BEGIN MX_APPE_Process_2 */
  /* USER CODE END MX_APPE_Process_2 */
}


void UTIL_SEQ_Idle(void)
{
#if (CFG_LPM_SUPPORTED == 1)
  UTIL_LPM_EnterLowPower();
#endif /* CFG_LPM_SUPPORTED == 1 */
  return;
}

void shci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask(1<<CFG_TASK_SYSTEM_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void shci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt(1<< CFG_IDLEEVT_SYSTEM_HCI_CMD_EVT_RSP_ID);
  return;
}

void shci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt(1<< CFG_IDLEEVT_SYSTEM_HCI_CMD_EVT_RSP_ID);
  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */

/* USER CODE END FD_WRAP_FUNCTIONS */
