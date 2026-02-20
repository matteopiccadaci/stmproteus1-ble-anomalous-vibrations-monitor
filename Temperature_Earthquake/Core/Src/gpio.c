/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_3_Pin|RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ST1PS02_D1_Pin|ST1PS02_D2_Pin|ST1PS02_D0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_IIS3DWB_CS_Pin|SPI_ISM330DHCX_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_MOSIs_Pin|GPIO_2_Pin|GPIO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPDT_SEL_1_Pin|SPDT_SEL_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PRIMBATMS_CTRL_Pin|LED_GREEN_Pin|LED_RED_Pin|STSAFE_nRESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STBC02_SW_SEL_Pin|GPIO_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ST1PS02_AUX_Pin|STBC02_CEN_Pin|MX66L2G45G_nRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : STTS22H_INT_Pin */
  GPIO_InitStruct.Pin = STTS22H_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STTS22H_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_3_Pin ST1PS02_D1_Pin ST1PS02_D2_Pin RESET_Pin
                           ST1PS02_D0_Pin */
  GPIO_InitStruct.Pin = GPIO_3_Pin|ST1PS02_D1_Pin|ST1PS02_D2_Pin|RESET_Pin
                          |ST1PS02_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_MISOs_Pin */
  GPIO_InitStruct.Pin = SPI2_MISOs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI2_MISOs_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IIS2DLPC_INT1_Pin IIS3DWB_INT2_Pin ISM330DHCX_INT2_Pin IIS2DLPC_INT2_Pin */
  GPIO_InitStruct.Pin = IIS2DLPC_INT1_Pin|IIS3DWB_INT2_Pin|ISM330DHCX_INT2_Pin|IIS2DLPC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_IIS3DWB_CS_Pin SPI_ISM330DHCX_CS_Pin */
  GPIO_InitStruct.Pin = SPI_IIS3DWB_CS_Pin|SPI_ISM330DHCX_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_USER_Pin */
  GPIO_InitStruct.Pin = BUTTON_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_MOSIs_Pin GPIO_2_Pin GPIO_1_Pin */
  GPIO_InitStruct.Pin = SPI2_MOSIs_Pin|GPIO_2_Pin|GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ISM330DHCX_INT1_Pin IIS3DWB_INT1_Pin */
  GPIO_InitStruct.Pin = ISM330DHCX_INT1_Pin|IIS3DWB_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Pin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPDT_SEL_1_Pin SPDT_SEL_2_Pin */
  GPIO_InitStruct.Pin = SPDT_SEL_1_Pin|SPDT_SEL_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STBC02_WAKE_UP_Pin */
  GPIO_InitStruct.Pin = STBC02_WAKE_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STBC02_WAKE_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PRIMBATMS_CTRL_Pin ST1PS02_AUX_Pin LED_GREEN_Pin LED_RED_Pin
                           MX66L2G45G_nRST_Pin STSAFE_nRESET_Pin */
  GPIO_InitStruct.Pin = PRIMBATMS_CTRL_Pin|ST1PS02_AUX_Pin|LED_GREEN_Pin|LED_RED_Pin
                          |MX66L2G45G_nRST_Pin|STSAFE_nRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : STBC02_SW_SEL_Pin */
  GPIO_InitStruct.Pin = STBC02_SW_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(STBC02_SW_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STBC02_CEN_Pin */
  GPIO_InitStruct.Pin = STBC02_CEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBC02_CEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ST1PS02_PGOOD_Pin */
  GPIO_InitStruct.Pin = ST1PS02_PGOOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ST1PS02_PGOOD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_4_Pin */
  GPIO_InitStruct.Pin = GPIO_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
