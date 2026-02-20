/**
  ******************************************************************************
  * @file    mx66l2g_conf_template.h
  * @brief   This file contains all the description of the MX66L2G QSPI memory.
  *          This file should be copied to the application folder and renamed to
  *          mx66l2g_conf.h.
  ******************************************************************************
  @verbatim
  MX66L2G Dummy Clock Cycle :
  Dummy clock cycle configued by bit 7, 6 of Configuration Register.
  The setting related to SCLK frequency & I/O interface number.
  MX66L2G Dummy Clock setting
           |             Instruction format
     DUMMY |1-1-1, 1-1-2     |1-2-2     |1-4-4
     [1:0] |1-1-4, 1-1-1(DTR)|1-2-2(DTR)|4-4-4, 4-4-4(DTR)
    -------+-----------------+----------+-----------------
      00   |    8 clocks     | 4 clocks |    6 clocks
      01   |    6 clocks     | 6 clocks |    4 clocks
      10   |    8 clocks     | 8 clocks |    8 clocks
      11   |   10 clocks     |10 clocks |   10 clocks    <-- Driver setting
    DUMMY{1:0] = 11, means 10 dummy clock cycle at max. SCLK frequency supported.
    We don't need to change it between any interface change.
  @endverbatim 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MX66L2G_CONF_H
#define __MX66L2G_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx.h"
#include "stm32wbxx_hal.h"
   
#define CONF_MX66L2G_READ_ENHANCE      1                       /* MMP performance enhance reade enable/disable */

#define CONF_QSPI_ODS                   MX66L2G_CR_ODS_26


#define CONF_QSPI_DUMMY_CLOCK           MX66L2G_CR_DUMMY_00    /* MX66L2G dummy clock cycle index */

#if (CONF_QSPI_DUMMY_CLOCK == MX66L2G_CR_DUMMY_00)
#define MX66L2G_DUMMY_CLOCK_11x        8
#define MX66L2G_DUMMY_CLOCK_122        4
#define MX66L2G_DUMMY_CLOCK_x44        6
#elif (CONF_QSPI_DUMMY_CLOCK == MX66L2G_CR_DUMMY_01)
#define MX66L2G_DUMMY_CLOCK_11x        6
#define MX66L2G_DUMMY_CLOCK_122        6
#define MX66L2G_DUMMY_CLOCK_x44        4
#elif (CONF_QSPI_DUMMY_CLOCK == MX66L2G_CR_DUMMY_10)
#define MX66L2G_DUMMY_CLOCK_11x        8
#define MX66L2G_DUMMY_CLOCK_122        8
#define MX66L2G_DUMMY_CLOCK_x44        8
#elif (CONF_QSPI_DUMMY_CLOCK == MX66L2G_CR_DUMMY_11)
#define MX66L2G_DUMMY_CLOCK_11x        10
#define MX66L2G_DUMMY_CLOCK_122        10
#define MX66L2G_DUMMY_CLOCK_x44        10
#endif

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __MX66L2G_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
