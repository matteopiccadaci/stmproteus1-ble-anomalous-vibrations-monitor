/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    App/custom_stm.h
 * @brief   Custom GATT service (TMPEQ) for LightBlue
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ble_types.h"

typedef enum
{
  CUSTOM_STM_TEMP_VAL,
  CUSTOM_STM_EQ_ANOM,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  CUSTOM_STM_TEMP_VAL_READ_EVT,
  CUSTOM_STM_TEMP_VAL_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_TEMP_VAL_NOTIFY_DISABLED_EVT,
  CUSTOM_STM_EQ_ANOM_READ_EVT,
  CUSTOM_STM_EQ_ANOM_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_EQ_ANOM_NOTIFY_DISABLED_EVT,
} Custom_STM_Opcode_evt_t;

typedef struct
{
  Custom_STM_Opcode_evt_t Custom_Evt_Opcode;
  uint16_t ConnectionHandle;
  uint8_t  DataTransfered[2];
  uint8_t  DataTransferedLength;
} Custom_STM_App_Notification_evt_t;

void Custom_STM_Init(void);
tBleStatus Custom_STM_Update_Char(Custom_STM_Char_Opcode_t char_opcode, uint8_t *pPayload);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_STM_H */
