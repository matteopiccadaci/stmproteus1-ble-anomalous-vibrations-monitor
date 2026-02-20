/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STTS22H_INT_Pin GPIO_PIN_1
#define STTS22H_INT_GPIO_Port GPIOA
#define STTS22H_INT_EXTI_IRQn EXTI1_IRQn
#define ADC_Pin GPIO_PIN_0
#define ADC_GPIO_Port GPIOA
#define GPIO_3_Pin GPIO_PIN_3
#define GPIO_3_GPIO_Port GPIOC
#define SPI2_MISOs_Pin GPIO_PIN_2
#define SPI2_MISOs_GPIO_Port GPIOC
#define LPUART1_TX_Pin GPIO_PIN_1
#define LPUART1_TX_GPIO_Port GPIOC
#define QUADSPI_BK1_IO0_Pin GPIO_PIN_9
#define QUADSPI_BK1_IO0_GPIO_Port GPIOB
#define LPUART1_RX_Pin GPIO_PIN_0
#define LPUART1_RX_GPIO_Port GPIOC
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB
#define USART1_CTS_Pin GPIO_PIN_4
#define USART1_CTS_GPIO_Port GPIOB
#define USART1_RTS_Pin GPIO_PIN_3
#define USART1_RTS_GPIO_Port GPIOB
#define ST1PS02_D1_Pin GPIO_PIN_10
#define ST1PS02_D1_GPIO_Port GPIOC
#define ST1PS02_D2_Pin GPIO_PIN_11
#define ST1PS02_D2_GPIO_Port GPIOC
#define IIS2DLPC_INT1_Pin GPIO_PIN_12
#define IIS2DLPC_INT1_GPIO_Port GPIOC
#define IIS2DLPC_INT1_EXTI_IRQn EXTI15_10_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI_IIS3DWB_CS_Pin GPIO_PIN_15
#define SPI_IIS3DWB_CS_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_0
#define SPI2_NSS_GPIO_Port GPIOD
#define SPI2_SCK_Pin GPIO_PIN_1
#define SPI2_SCK_GPIO_Port GPIOD
#define BUTTON_USER_Pin GPIO_PIN_13
#define BUTTON_USER_GPIO_Port GPIOB
#define BUTTON_USER_EXTI_IRQn EXTI15_10_IRQn
#define IIS3DWB_INT2_Pin GPIO_PIN_6
#define IIS3DWB_INT2_GPIO_Port GPIOC
#define IIS3DWB_INT2_EXTI_IRQn EXTI9_5_IRQn
#define I2C3_SDA_Pin GPIO_PIN_14
#define I2C3_SDA_GPIO_Port GPIOB
#define SPI2_MOSIs_Pin GPIO_PIN_15
#define SPI2_MOSIs_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_13
#define RESET_GPIO_Port GPIOC
#define GPIO_2_Pin GPIO_PIN_12
#define GPIO_2_GPIO_Port GPIOB
#define ISM330DHCX_INT1_Pin GPIO_PIN_4
#define ISM330DHCX_INT1_GPIO_Port GPIOE
#define ISM330DHCX_INT1_EXTI_IRQn EXTI4_IRQn
#define PRIMBATMS_ADC_Pin GPIO_PIN_5
#define PRIMBATMS_ADC_GPIO_Port GPIOC
#define QUADSPI_BK1_NCS_Pin GPIO_PIN_11
#define QUADSPI_BK1_NCS_GPIO_Port GPIOB
#define GPIO_1_Pin GPIO_PIN_10
#define GPIO_1_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_2
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI2_IRQn
#define BATMS_ADC_Pin GPIO_PIN_4
#define BATMS_ADC_GPIO_Port GPIOC
#define SPDT_SEL_1_Pin GPIO_PIN_8
#define SPDT_SEL_1_GPIO_Port GPIOA
#define SPDT_SEL_2_Pin GPIO_PIN_9
#define SPDT_SEL_2_GPIO_Port GPIOA
#define I2C3_SCL_Pin GPIO_PIN_7
#define I2C3_SCL_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI_ISM330DHCX_CS_Pin GPIO_PIN_4
#define SPI_ISM330DHCX_CS_GPIO_Port GPIOA
#define QUADSPI_CLK_Pin GPIO_PIN_3
#define QUADSPI_CLK_GPIO_Port GPIOA
#define STBC02_WAKE_UP_Pin GPIO_PIN_0
#define STBC02_WAKE_UP_GPIO_Port GPIOH
#define STBC02_WAKE_UP_EXTI_IRQn EXTI0_IRQn
#define LED_BLUE_Pin GPIO_PIN_1
#define LED_BLUE_GPIO_Port GPIOH
#define PRIMBATMS_CTRL_Pin GPIO_PIN_14
#define PRIMBATMS_CTRL_GPIO_Port GPIOD
#define STBC02_SW_SEL_Pin GPIO_PIN_1
#define STBC02_SW_SEL_GPIO_Port GPIOE
#define ST1PS02_AUX_Pin GPIO_PIN_13
#define ST1PS02_AUX_GPIO_Port GPIOD
#define STBC02_CEN_Pin GPIO_PIN_12
#define STBC02_CEN_GPIO_Port GPIOD
#define QUADSPI_BK1_IO3_Pin GPIO_PIN_7
#define QUADSPI_BK1_IO3_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_2
#define LED_GREEN_GPIO_Port GPIOD
#define ST1PS02_D0_Pin GPIO_PIN_9
#define ST1PS02_D0_GPIO_Port GPIOC
#define SPI2_MISOp_Pin GPIO_PIN_3
#define SPI2_MISOp_GPIO_Port GPIOD
#define ISM330DHCX_INT2_Pin GPIO_PIN_7
#define ISM330DHCX_INT2_GPIO_Port GPIOC
#define ISM330DHCX_INT2_EXTI_IRQn EXTI9_5_IRQn
#define IIS3DWB_INT1_Pin GPIO_PIN_3
#define IIS3DWB_INT1_GPIO_Port GPIOE
#define IIS3DWB_INT1_EXTI_IRQn EXTI3_IRQn
#define SPI2_MOSIp_Pin GPIO_PIN_4
#define SPI2_MOSIp_GPIO_Port GPIOD
#define ST1PS02_PGOOD_Pin GPIO_PIN_9
#define ST1PS02_PGOOD_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_8
#define LED_RED_GPIO_Port GPIOD
#define PWM_Pin GPIO_PIN_15
#define PWM_GPIO_Port GPIOD
#define MX66L2G45G_nRST_Pin GPIO_PIN_10
#define MX66L2G45G_nRST_GPIO_Port GPIOD
#define GPIO_4_Pin GPIO_PIN_2
#define GPIO_4_GPIO_Port GPIOE
#define STBC02_CHG_Pin GPIO_PIN_0
#define STBC02_CHG_GPIO_Port GPIOE
#define QUADSPI_BK1_IO1_Pin GPIO_PIN_5
#define QUADSPI_BK1_IO1_GPIO_Port GPIOD
#define QUADSPI_BK1_IO2_Pin GPIO_PIN_6
#define QUADSPI_BK1_IO2_GPIO_Port GPIOD
#define STSAFE_nRESET_Pin GPIO_PIN_11
#define STSAFE_nRESET_GPIO_Port GPIOD
#define IIS2DLPC_INT2_Pin GPIO_PIN_8
#define IIS2DLPC_INT2_GPIO_Port GPIOC
#define IIS2DLPC_INT2_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
