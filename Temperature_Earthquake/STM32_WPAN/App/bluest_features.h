/* USER CODE BEGIN Header */
/**
  ******************************************************************************
 * @file    App/bluest_features.h
 * @brief   BlueST Features service (Env + NEAI AD)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BLUEST_FEATURES_H
#define BLUEST_FEATURES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void BlueST_Init(void);
void BlueST_Update_Env(int16_t temp_tenth_c);
void BlueST_Update_AD(uint8_t phase, uint8_t state, uint8_t progress, uint8_t status, uint8_t similarity);

#ifdef __cplusplus
}
#endif

#endif /* BLUEST_FEATURES_H */
