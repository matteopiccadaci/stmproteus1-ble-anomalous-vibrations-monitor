/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  ******************************************************************************
  */
/* USER CODE END Header */

//This part is used for apps like LightBlue (generic custom GATT client)

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* USER CODE BEGIN Includes */
#include "bluest_features.h"
#include "i2c.h" 
#include "../../Drivers/BSP/Components/stts22h/stts22h_reg.h"
#include "../../Drivers/BSP/Components/stts22h/stts22h_reg.c"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t  Temp_val_Notification_Status;
  uint8_t  Eq_anom_Notification_Status;
  uint16_t ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
/* === STTS22H context (stts22h_reg + HAL I2C) === */
typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t addr;
} stts22h_bus_t;

static stmdev_ctx_t s_stts22h_ctx;
static stts22h_bus_t s_stts22h_bus = { 0 };
static uint8_t      s_stts22h_inited = 0;
static uint8_t      s_stts22h_found  = 0;

//Notify temperature every second 
#define TEMP_NOTIFY_PERIOD_MS  (1000u)

/* Payload:
   TEMP_VAL = int16 little-endian, unità 0.01°C
   EQ_ANOM  = [0]=outlier(0/1) [1]=flag (0/1) o codice diagnostico */
#define TEMP_PAYLOAD_LEN  (2u)
#define ANOM_PAYLOAD_LEN  (2u)

// tracking 
static uint32_t s_last_temp_notify_ms = 0;

// last anomaly state
static uint8_t s_last_anom = 0;
static uint8_t s_last_sim  = 0;

// buffer (one for characteristic)
static uint8_t s_temp_buf[TEMP_PAYLOAD_LEN];
static uint8_t s_anom_buf[ANOM_PAYLOAD_LEN];

// default value for temperature
static float s_last_tempC = 25.0f;

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STTS22H_I2C_ADDR   (STTS22H_I2C_ADD_L << 1) // 7-bit address shifted for HAL
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
static Custom_App_Context_t Custom_App_Context;
uint16_t Connection_Handle;

/* Private function prototypes -----------------------------------------------*/
static void Custom_Temp_val_Update_Char(void);
static void Custom_Temp_val_Send_Notification(void);
static void Custom_Eq_anom_Update_Char(void);
static void Custom_Eq_anom_Send_Notification(void);

/* USER CODE BEGIN PFP */
//Functions taken from FP-AI-PDMWBSOC2 firmware, found in teh Drivers folder
static int32_t stts22h_platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t stts22h_platform_read (void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void    stts22h_platform_delay(uint32_t ms);

static void   STTS22H_InitOnce(void);
static float  STTS22H_ReadTempC(void);
static int32_t STTS22H_Probe(void);
static int32_t STTS22H_TryBusAddr(I2C_HandleTypeDef *hi2c, uint16_t addr);

static void   Custom_TempNotify_Second(void);

static void   pack_i16_be(uint8_t out[2], int16_t v); // Big endian leads to better visibility on my BLE client
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/


void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  switch (pNotification->Custom_Evt_Opcode)
  {
    case CUSTOM_STM_TEMP_VAL_READ_EVT:
      Custom_Temp_val_Update_Char();
      break;

    case CUSTOM_STM_TEMP_VAL_NOTIFY_ENABLED_EVT:
      Custom_App_Context.Temp_val_Notification_Status = 1;
      Custom_Temp_val_Send_Notification();
      break;

    case CUSTOM_STM_TEMP_VAL_NOTIFY_DISABLED_EVT:
      Custom_App_Context.Temp_val_Notification_Status = 0;
      break;

    case CUSTOM_STM_EQ_ANOM_READ_EVT:
      Custom_Eq_anom_Update_Char();
      break;

    case CUSTOM_STM_EQ_ANOM_NOTIFY_ENABLED_EVT:
      Custom_App_Context.Eq_anom_Notification_Status = 1;
      Custom_Eq_anom_Send_Notification();
      break;

    case CUSTOM_STM_EQ_ANOM_NOTIFY_DISABLED_EVT:
      Custom_App_Context.Eq_anom_Notification_Status = 0;
      break;

    default:
      break;
  }
} // Taken from FP-AI-PDMWBSOC2 firmware


void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  switch (pNotification->Custom_Evt_Opcode)
  {
    case CUSTOM_CONN_HANDLE_EVT:
      Custom_App_Context.ConnectionHandle = pNotification->ConnectionHandle;
      Connection_Handle = pNotification->ConnectionHandle;

      s_last_temp_notify_ms = HAL_GetTick();
      STTS22H_InitOnce();
      break;

    case CUSTOM_DISCON_HANDLE_EVT:
      Custom_App_Context.ConnectionHandle = 0;
      Connection_Handle = 0;

      Custom_App_Context.Temp_val_Notification_Status = 0;
      Custom_App_Context.Eq_anom_Notification_Status  = 0;
      break;

    default:
      break;
  }
}

void Custom_APP_Init(void)
{
  Custom_App_Context.Temp_val_Notification_Status = 0;
  Custom_App_Context.Eq_anom_Notification_Status  = 0;
  Custom_App_Context.ConnectionHandle             = 0;
  Connection_Handle                               = 0;

  s_last_temp_notify_ms = HAL_GetTick();
  STTS22H_InitOnce();
  Custom_STM_Init();
  BlueST_Init();
}

/* USER CODE BEGIN FD */
void Custom_APP_Process(void)
{
  Custom_TempNotify_Second();
}

void Custom_APP_Send_Anomaly(uint8_t is_anom, uint8_t similarity, uint8_t dyn_scaled)
{
  s_last_anom = is_anom;
  s_last_sim  = similarity;

  // Red LED on if an anomaly is detected
  if (is_anom != 0u)
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  }

  (void)dyn_scaled;
  s_anom_buf[0] = s_last_anom;
  s_anom_buf[1] = s_last_sim;

  (void)Custom_STM_Update_Char(CUSTOM_STM_EQ_ANOM, s_anom_buf);

  // BlueST AD feature 
  BlueST_Update_AD(0x02u, 0x00u, 0x64u, s_last_anom ? 0x01u : 0x00u, s_last_sim);

}
/* USER CODE END FD */

/*************************************************************
 * LOCAL FUNCTIONS
 *************************************************************/

__USED void Custom_Temp_val_Update_Char(void)
{
  float tC = STTS22H_ReadTempC();
  int16_t t01 = (int16_t)(tC * 10.0f);

  pack_i16_be(s_temp_buf, t01);
  (void)Custom_STM_Update_Char(CUSTOM_STM_TEMP_VAL, s_temp_buf);
}

void Custom_Temp_val_Send_Notification(void)
{
  float tC = STTS22H_ReadTempC();
  int16_t t01 = (int16_t)(tC * 10.0f);

  // Custom GATT + BlueST use 0.1°C 
  pack_i16_be(s_temp_buf, t01);
  (void)Custom_STM_Update_Char(CUSTOM_STM_TEMP_VAL, s_temp_buf);
  // BlueST Environmental: Temp in 0.1°C
  BlueST_Update_Env(t01);
}

__USED void Custom_Eq_anom_Update_Char(void) // __USED is similar to volatile, used to remove warning
{
  s_anom_buf[0] = s_last_anom;
  s_anom_buf[1] = s_last_sim;

  (void)Custom_STM_Update_Char(CUSTOM_STM_EQ_ANOM, s_anom_buf);
}

void Custom_Eq_anom_Send_Notification(void)
{
  s_anom_buf[0] = s_last_anom;
  s_anom_buf[1] = s_last_sim;
  (void)Custom_STM_Update_Char(CUSTOM_STM_EQ_ANOM, s_anom_buf);
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
static void Custom_TempNotify_Second(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - s_last_temp_notify_ms) >= TEMP_NOTIFY_PERIOD_MS)
  {
    s_last_temp_notify_ms = now;
    Custom_Temp_val_Send_Notification();
  }
} // Counts one second (TEMP_NOTIFY_PERIOD_MS) and notifies temperature

static void STTS22H_InitOnce(void)
{
  if (s_stts22h_inited)
  {
    return;
  }

  if (STTS22H_Probe() != 0)
  {
    return;
  }

  s_stts22h_ctx.write_reg = stts22h_platform_write;
  s_stts22h_ctx.read_reg  = stts22h_platform_read;
  s_stts22h_ctx.mdelay    = stts22h_platform_delay;
  s_stts22h_ctx.handle    = &s_stts22h_bus;

  uint8_t id = 0;
  (void)stts22h_dev_id_get(&s_stts22h_ctx, &id);
  if (id != STTS22H_ID)
  {
    s_stts22h_inited = 0;
    s_stts22h_found  = 0;
    return;
  }

  (void)stts22h_block_data_update_set(&s_stts22h_ctx, PROPERTY_ENABLE);
  (void)stts22h_auto_increment_set(&s_stts22h_ctx, PROPERTY_ENABLE);

  (void)stts22h_temp_data_rate_set(&s_stts22h_ctx, STTS22H_25Hz);

  stts22h_platform_delay(20);

  s_stts22h_inited = 1;
} // Taken from FP-AI-PDMWBSOC2 firmware (drivers)

static float STTS22H_ReadTempC(void)
{
  if (!s_stts22h_inited)
    STTS22H_InitOnce();

  if (s_stts22h_bus.hi2c == NULL)
    return s_last_tempC; // Check for the bus to be free, otherwise default to last value

  for (uint32_t i = 0; i < 10; i++)
  {
    if (HAL_I2C_GetState(s_stts22h_bus.hi2c) == HAL_I2C_STATE_READY)
      break;
    HAL_Delay(1);
  } // Check for the bus to be free, otherwise default to last value

  if (HAL_I2C_GetState(s_stts22h_bus.hi2c) != HAL_I2C_STATE_READY)
    return s_last_tempC;

  int16_t raw = 0;
  uint8_t drdy = 0;

  // wait for a new sample (max ~50ms)
  for (uint32_t i = 0; i < 50; i++)
  {
    if (stts22h_temp_flag_data_ready_get(&s_stts22h_ctx, &drdy) == 0 && drdy != 0)
    {
      break;
    }
    HAL_Delay(1);
  }

  int32_t ret = stts22h_temperature_raw_get(&s_stts22h_ctx, &raw);

  if (ret != 0)
  {
    /* recovery: re-init */
    s_stts22h_inited = 0;
    s_stts22h_found  = 0;
    STTS22H_InitOnce();
    return s_last_tempC;
  }

  float t = stts22h_from_lsb_to_celsius(raw);
  s_last_tempC = t;
  return t;
}

static int32_t stts22h_platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  stts22h_bus_t *bus = (stts22h_bus_t *)handle;
  if (bus == NULL || bus->hi2c == NULL)
  {
    return -1;
  }
  I2C_HandleTypeDef *hi2c = bus->hi2c;

  if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
  {
    return -1;
  }

  if (HAL_I2C_Mem_Write(hi2c, bus->addr, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100) == HAL_OK)
  {
    return 0;
  }
  return -1;
}

static int32_t stts22h_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  stts22h_bus_t *bus = (stts22h_bus_t *)handle;
  if (bus == NULL || bus->hi2c == NULL)
  {
    return -1;
  }
  I2C_HandleTypeDef *hi2c = bus->hi2c;

  if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
  {
    return -1;
  }

  if (HAL_I2C_Mem_Read(hi2c, bus->addr, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100) == HAL_OK)
  {
    return 0;
  }
  return -1;
}

static void stts22h_platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}


static void pack_i16_be(uint8_t out[2], int16_t v)
{
  out[0] = (uint8_t)((v >> 8) & 0xFF);
  out[1] = (uint8_t)(v & 0xFF);
}

static int32_t STTS22H_TryBusAddr(I2C_HandleTypeDef *hi2c, uint16_t addr)
{
  uint8_t id = 0;

  if (hi2c == NULL)
  {
    return -1;
  }

  if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
  {
    return -1;
  }

  if (HAL_I2C_Mem_Read(hi2c, addr, STTS22H_WHOAMI, I2C_MEMADD_SIZE_8BIT, &id, 1, 100) != HAL_OK)
  {
    return -1;
  }

  if (id != STTS22H_ID)
  {
    return -1;
  }

  s_stts22h_bus.hi2c = hi2c;
  s_stts22h_bus.addr = addr;
  s_stts22h_found = 1;
  return 0;
}

static int32_t STTS22H_Probe(void)
{
  static const uint8_t addr_candidates[] =
  {
    STTS22H_I2C_ADD_L,
    STTS22H_I2C_ADD_H,
    STTS22H_I2C_ADD_15K,
    STTS22H_I2C_ADD_56K,
    0x3Cu,
    0x3Du
  };

  I2C_HandleTypeDef *buses[] = { &hi2c1, &hi2c3 };

  if (s_stts22h_found != 0)
  {
    return 0;
  }

  for (uint32_t b = 0; b < (sizeof(buses) / sizeof(buses[0])); b++)
  {
    for (uint32_t i = 0; i < (sizeof(addr_candidates) / sizeof(addr_candidates[0])); i++)
    {
      uint16_t addr7 = addr_candidates[i];

      if (STTS22H_TryBusAddr(buses[b], (uint16_t)(addr7 << 1)) == 0)
      {
        return 0;
      }
      if (STTS22H_TryBusAddr(buses[b], addr7) == 0)
      {
        return 0;
      }
    }
  }

  return -1;
}
/* USER CODE END FD_LOCAL_FUNCTIONS */
