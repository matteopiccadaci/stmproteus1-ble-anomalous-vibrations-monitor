/**
  ******************************************************************************
  * @file    mx66l2g.c
  * @brief   MX66L2G driver file
  ******************************************************************************
  @verbatim
  MX66L2G action :
    QE(Quad Enable, Non-volatile) bit of Status Register
    QE = 0; WP# pin active
            Accept 1-1-1, 1-1-2, 1-2-2 commands
    QE = 1; WP# become SIO2 pin, RESET# become SIO3 pin
            Accept 1-1-1, 1-1-2, 1-2-2, 1-1-4, 1-4-4 commands
    Enter QPI mode by issue EQIO(0x35) command from 1-1-1 mode
            Accept 4-4-4 commands
    Exit QPI mode by issue RSTQIO(0xF5) command from 4-4-4 mode
            Accept commands, dependents on QE bit status
    Memory Read commands support STR(Single Transfer Rate) &
    DTR(Double Transfer Rate) mode
 *
  MX66L2G Dummy Clock Cycle :
      Dummy clock cycle configured by bit 7, 6 of Configuration Register.
    The setting related to SCLK frequency & I/O interface number.
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

/* Includes ------------------------------------------------------------------*/
#include "mx66l2g.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */


/** @addtogroup MX66L2G
 * @{
 */

/** @addtogroup MX66L2G_PUBLIC_FUNCTIONS
  * @{
  */

/* Function by commands combined *********************************************/
/**
  * @brief  Get Flash information
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_GetFlashInfo(MX66L2G_Info_t *pInfo)
{
  /* Configure the structure with the memory configuration */
  pInfo->FlashID            = MX66L2G_FLASH_ID;
  pInfo->FlashSize          = MX66L2G_FLASH_SIZE;
  pInfo->EraseSectorSize    = MX66L2G_SECTOR_4K;
  pInfo->EraseSectorsNumber = (MX66L2G_FLASH_SIZE/MX66L2G_SECTOR_4K);
  pInfo->ProgPageSize       = MX66L2G_PAGE_SIZE;
  pInfo->ProgPagesNumber    = (MX66L2G_FLASH_SIZE/MX66L2G_PAGE_SIZE);

  pInfo->EraseBlockSize     = MX66L2G_BLOCK_32K;
  pInfo->EraseBlockNumber   = (MX66L2G_FLASH_SIZE/MX66L2G_BLOCK_32K);
  pInfo->EraseBlockSize1    = MX66L2G_BLOCK_64K;
  pInfo->EraseBlockNumber1  = (MX66L2G_FLASH_SIZE/MX66L2G_BLOCK_64K);
  return MX66L2G_OK;
}

/**
  * @brief  Polling WIP(Write In Progress) bit become to 0
  *         SPI/QPI; 1-0-1/4-0-4
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_AutoPollingMemReady(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t Timeout)
{
  QSPI_CommandTypeDef      s_command;
  QSPI_AutoPollingTypeDef  s_config;

  /* Configure automatic polling mode to wait for memory ready */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  s_config.Match           = 0;
  s_config.Mask            = MX66L2G_SR_WIP;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(Ctx, &s_command, &s_config, Timeout) != HAL_OK)
  {
    return MX66L2G_ERROR_AUTOPOLLING;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Reads an amount of data from the QSPI memory on STR mode.
  *         SPI/QPI; 1-1-1/1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address (3/4 Byte)
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_EnableMemoryMappedMode34ByteSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize)
{
  QSPI_CommandTypeDef      s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  switch(Mode)
  {
  case MX66L2G_SPI_NORMAL_MODE :         // 1-1-1 read command with 0 dummy cycle
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = 0;
    s_command.DataMode           = QSPI_DATA_1_LINE;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_MODE :               // 1-1-1 read command, Power on H/W default setting
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode           = QSPI_DATA_1_LINE;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_2IO_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_2_LINES;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_122;
    s_command.DataMode           = QSPI_DATA_2_LINES;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_1I2O_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode           = QSPI_DATA_2_LINES;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4IO_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 2) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_1I4O_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction        = MX66L2G_4IO_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 2) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Configure the command for the read instruction */
  s_command.AddressSize       = AddrSize;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod     = 0;

  if (HAL_QSPI_MemoryMapped(Ctx, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return MX66L2G_ERROR_MEMORYMAPPED;
  }

  return MX66L2G_OK;
}

/**
  * @brief  3 or 4 Byte address Reads an amount of data from the QSPI memory on DTR mode.
  *         SPI/QPI; 1-1-1/1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address (3/4 Byte)
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_EnableMemoryMappedMode34ByteDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize)
{
  QSPI_CommandTypeDef      s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  switch(Mode)
  {
  //case MX66L2G_SPI_NORMAL_MODE :        // 1-1-1 read command with 0 dummy cycle
  //case MX66L2G_SPI_MODE :               // 1-1-1 read command, Power on H/W default setting
  //  s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction        = MX66L2G_FAST_READ_DTR_CMD;
  //  s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
  //  s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
  //  s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
  //  s_command.DataMode           = QSPI_DATA_1_LINE;
  //  s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
  //  break;

  //case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
  //case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
  //  s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction        = MX66L2G_2IO_FAST_READ_DTR_CMD;
  //  s_command.AddressMode        = QSPI_ADDRESS_2_LINES;
  //  s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
  //  s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_122;
  //  s_command.DataMode           = QSPI_DATA_2_LINES;
  //  s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
  //  break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read command
  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 1) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction        = MX66L2G_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 1) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Configure the command for the read instruction */
  s_command.AddressSize       = AddrSize;
  s_command.DdrMode           = QSPI_DDR_MODE_ENABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_HALF_CLK_DELAY;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod     = 0;

  if (HAL_QSPI_MemoryMapped(Ctx, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return MX66L2G_ERROR_MEMORYMAPPED;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Reads an amount of data from the QSPI memory on STR mode.
  *         SPI/QPI; 1-1-1/1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address (4 Byte)
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_EnableMemoryMappedModeSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef      s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  switch(Mode)
  {
  case MX66L2G_SPI_NORMAL_MODE :        // 1-1-1 read command with 0 dummy cycle
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = 0;
    s_command.DataMode           = QSPI_DATA_1_LINE;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_MODE :               // 1-1-1 command, Power on H/W default setting
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode           = QSPI_DATA_1_LINE;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_2IO_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_2_LINES;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_122;
    s_command.DataMode           = QSPI_DATA_2_LINES;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_1I2O_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode           = QSPI_DATA_2_LINES;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 2) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_1I4O_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
    s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 2) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Configure the command for the read instruction */
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod     = 0;

  if (HAL_QSPI_MemoryMapped(Ctx, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return MX66L2G_ERROR_MEMORYMAPPED;
  }

  return MX66L2G_OK;
}

/**
  * @brief  4 Byte address Reads an amount of data from the QSPI memory on DTR mode.
  *         SPI/QPI; 1-1-1/1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read (4 Byte)
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_EnableMemoryMappedModeDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef      s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  switch(Mode)
  {
  //case MX66L2G_SPI_NORMAL_MODE :        // 1-1-1 read command with 0 dummy cycle
  //case MX66L2G_SPI_MODE :               // 1-1-1 read command, Power on H/W default setting
  //  s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction        = MX66L2G_4_BYTE_ADDR_FAST_READ_DTR_CMD;
  //  s_command.AddressMode        = QSPI_ADDRESS_1_LINE;
  //  s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
  //  s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_11x;
  //  s_command.DataMode           = QSPI_DATA_1_LINE;
  //  s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
  //  break;

  //case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
  //case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
  //  s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction        = MX66L2G_4_BYTE_ADDR_2IO_FAST_READ_DTR_CMD;
  //  s_command.AddressMode        = QSPI_ADDRESS_2_LINES;
  //  s_command.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
  //  s_command.DummyCycles        = MX66L2G_DUMMY_CLOCK_122;
  //  s_command.DataMode           = QSPI_DATA_2_LINES;
  //  s_command.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
  //  break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read command
  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 1) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 read command
    s_command.InstructionMode    = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction        = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode        = QSPI_ADDRESS_4_LINES;
    s_command.AlternateByteMode  = CONF_MX66L2G_READ_ENHANCE ? QSPI_ALTERNATE_BYTES_4_LINES : QSPI_ALTERNATE_BYTES_NONE;
    s_command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    s_command.AlternateBytes     = 0x5A;
    s_command.DummyCycles        = CONF_MX66L2G_READ_ENHANCE ? (MX66L2G_DUMMY_CLOCK_x44 - 1) : MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode           = QSPI_DATA_4_LINES;
    s_command.SIOOMode           = CONF_MX66L2G_READ_ENHANCE ? QSPI_SIOO_INST_ONLY_FIRST_CMD : QSPI_SIOO_INST_EVERY_CMD;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Configure the command for the read instruction */
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.DdrMode           = QSPI_DDR_MODE_ENABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_HALF_CLK_DELAY;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod     = 0;

  if (HAL_QSPI_MemoryMapped(Ctx, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return MX66L2G_ERROR_MEMORYMAPPED;
  }

  return MX66L2G_OK;
}


/* Read/Write Array Commands (3/4 Byte Address Command Set) ********************/
/**
  * @brief  3 or 4 Byte address Reads an amount of data from the QSPI memory on STR mode.
  *         SPI/QPI; 1-1-1/1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_34ByteAddressReadSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;
  QSPI_HandleTypeDef *hQSPI = Ctx;

  switch(Mode)
  {
  case MX66L2G_SPI_NORMAL_MODE :        // 1-1-1 read command with 0 dummy cycle
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = 0;
    s_command.DataMode        = QSPI_DATA_1_LINE;
    break;

  case MX66L2G_SPI_MODE :               // 1-1-1 command, Power on H/W default setting
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode        = QSPI_DATA_1_LINE;
    break;

  case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_2IO_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_2_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_122;
    s_command.DataMode        = QSPI_DATA_2_LINES;
    break;

  case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_1I2O_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode        = QSPI_DATA_2_LINES;
    break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4IO_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_1I4O_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction     = MX66L2G_4IO_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Initialize the read command */
  s_command.AddressSize       = AddrSize;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Set S# timing for Read command */
  MODIFY_REG(hQSPI->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_1_CYCLE);

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  /* Restore S# timing for nonRead commands */
  MODIFY_REG(hQSPI->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_4_CYCLE);

  return MX66L2G_OK;
}

/**
  * @brief  3 or 4 Byte address Reads an amount of data from the QSPI memory on DTR mode.
  *         SPI/QPI; 1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_34ByteAddressReadDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;

  switch(Mode)
  {
  //case MX66L2G_SPI_NORMAL_MODE :        // 1-1-1 read command with 0 dummy cycle
  //case MX66L2G_SPI_MODE :               // 1-1-1 read command
  //  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction     = MX66L2G_FAST_READ_DTR_CMD;
  //  s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
  //  s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
  //  s_command.DataMode        = QSPI_DATA_1_LINE;
  //  break;

  //case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
  //case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
  //  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction     = MX66L2G_2IO_FAST_READ_DTR_CMD;
  //  s_command.AddressMode     = QSPI_ADDRESS_2_LINES;
  //  s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_122;
  //  s_command.DataMode        = QSPI_DATA_2_LINES;
  //  break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read command
  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction     = MX66L2G_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Initialize the read command */
  s_command.AddressSize       = AddrSize;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_ENABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_HALF_CLK_DELAY;    // DTR mode used
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  3 or 4 Byte address Writes an amount of data to the QSPI memory.
  *         SPI/QPI; 1-1-1/1-4-4/4-4-4
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write. Range 1 ~ 256
  * @retval QSPI memory status
  */
int32_t MX66L2G_34ByteAddressPageProgram(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;

  /* Setup erase command */
  switch(Mode)
  {
  case MX66L2G_SPI_NORMAL_MODE :            // 1-1-1 read command with 0 dummy cycle
  case MX66L2G_SPI_MODE :                   // 1-1-1 commands, Power on H/W default setting
  case MX66L2G_SPI_2IO_MODE :               // 1-2-2 commands
  case MX66L2G_SPI_1I2O_MODE :              // 1-1-2 commands
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_PAGE_PROG_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DataMode        = QSPI_DATA_1_LINE;
    break;

  case MX66L2G_SPI_4IO_MODE :               // 1-4-4 program commands
  case MX66L2G_SPI_1I4O_MODE :              // 1-1-4 commands
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_QUAD_PAGE_PROG_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_QPI_MODE :                   // 4-4-4 commands
    s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction     = MX66L2G_PAGE_PROG_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  s_command.AddressSize       = AddrSize;
  s_command.Address           = WriteAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  3 or 4 Byte address Erases the specified block of the QSPI memory.
  *         MX66L2G support 4K, 32K, 64K size block erase commands.
  *         SPI/QPI; 1-1-0/4-4-0
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
int32_t MX66L2G_34ByteAddressBlockErase(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint32_t BlockAddress, MX66L2G_EraseTypeDef BlockSize)
{
  QSPI_CommandTypeDef s_command;

  /* Setup erase command */
  switch(BlockSize)
  {
  case MX66L2G_ERASE_4K :
    s_command.Instruction     = MX66L2G_SECTOR_ERASE_4K_CMD;
    break;

  case MX66L2G_ERASE_32K :
    s_command.Instruction     = MX66L2G_BLOCK_ERASE_32K_CMD;
    break;

  case MX66L2G_ERASE_64K :
    s_command.Instruction     = MX66L2G_BLOCK_ERASE_64K_CMD;
    break;

  case MX66L2G_ERASE_CHIP :
    return MX66L2G_ChipErase(Ctx, Mode);

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Initialize the erase command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.AddressMode       = (Mode == MX66L2G_QPI_MODE) ? QSPI_ADDRESS_4_LINES : QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = AddrSize;
  s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Whole chip erase.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ChipErase(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the erase command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_CHIP_ERASE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}


/* Read/Write Array Commands (4 Byte Address Command Set) *********************/
/**
  * @brief  4 Byte address Reads an amount of data from the QSPI memory on STR mode.
  *         SPI/QPI; 1-1-1/1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;
  QSPI_HandleTypeDef *hQSPI = Ctx;

  switch(Mode)
  {
  case MX66L2G_SPI_NORMAL_MODE :        // 1-1-1 read command with 0 dummy cycle
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = 0;
    s_command.DataMode        = QSPI_DATA_1_LINE;
    break;

  case MX66L2G_SPI_MODE :               // 1-1-1 read command, Power on H/W default setting
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode        = QSPI_DATA_1_LINE;
    break;

  case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_2IO_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_2_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_122;
    s_command.DataMode        = QSPI_DATA_2_LINES;
    break;

  case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_1I2O_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode        = QSPI_DATA_2_LINES;
    break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_1I4O_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Initialize the read command */
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Set S# timing for Read command */
  MODIFY_REG(hQSPI->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_1_CYCLE);

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  /* Restore S# timing for nonRead commands */
  MODIFY_REG(hQSPI->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_4_CYCLE);

  return MX66L2G_OK;
}

/**
  * @brief  4 Byte address Reads an amount of data from the QSPI memory on DTR mode.
  *         SPI/QPI; 1-1-1/1-2-2/1-4-4/4-4-4
  * @param  pData: Pointer to data to be read
  * @param  ReadAddr: Read start address
  * @param  Size: Size of data to read
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;

  switch(Mode)
  {
  //case MX66L2G_SPI_NORMAL_MODE :        // 1-1-1 read command with 0 dummy cycle
  //case MX66L2G_SPI_MODE :               // 1-1-1 read command, Power on H/W default setting
  //  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction     = MX66L2G_4_BYTE_ADDR_FAST_READ_DTR_CMD;
  //  s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
  //  s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_11x;
  //  s_command.DataMode        = QSPI_DATA_1_LINE;
  //  break;

  //case MX66L2G_SPI_2IO_MODE :           // 1-2-2 read command
  //case MX66L2G_SPI_1I2O_MODE :          // 1-1-2 read command
  //  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
  //  s_command.Instruction     = MX66L2G_4_BYTE_ADDR_2IO_FAST_READ_DTR_CMD;
  //  s_command.AddressMode     = QSPI_ADDRESS_2_LINES;
  //  s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_122;
  //  s_command.DataMode        = QSPI_DATA_2_LINES;
  //  break;

  case MX66L2G_SPI_4IO_MODE :           // 1-4-4 read commands
  case MX66L2G_SPI_1I4O_MODE :          // 1-1-4 read command
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_QPI_MODE :               // 4-4-4 commands
    s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_DTR_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DummyCycles     = MX66L2G_DUMMY_CLOCK_x44;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Initialize the read command */
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_ENABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_HALF_CLK_DELAY;    // DTR mode used
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  4 Byte address Writes an amount of data to the QSPI memory.
  *         SPI/QPI; 1-1-1/1-4-4/4-4-4
  * @param  pData: Pointer to data to be written
  * @param  WriteAddr: Write start address
  * @param  Size: Size of data to write. Range 1 ~ 256
  * @retval QSPI memory status
  */
int32_t MX66L2G_PageProgram(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;

  /* Setup erase command */
  switch(Mode)
  {
  case MX66L2G_SPI_NORMAL_MODE :            // 1-1-1 read command with 0 dummy cycle
  case MX66L2G_SPI_MODE :                   // 1-1-1 commands, Power on H/W default setting
  case MX66L2G_SPI_2IO_MODE :               // 1-2-2 commands
  case MX66L2G_SPI_1I2O_MODE :              // 1-1-2 read commands
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_PAGE_PROG_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_1_LINE;
    s_command.DataMode        = QSPI_DATA_1_LINE;
    break;

  case MX66L2G_SPI_4IO_MODE :               // 1-4-4 program commands
  case MX66L2G_SPI_1I4O_MODE :              // 1-1-4 read commands
    s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_QUAD_PAGE_PROG_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  case MX66L2G_QPI_MODE :                   // 4-4-4 commands
    s_command.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_PAGE_PROG_CMD;
    s_command.AddressMode     = QSPI_ADDRESS_4_LINES;
    s_command.DataMode        = QSPI_DATA_4_LINES;
    break;

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = WriteAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  4 Byte address Erases the specified block of the QSPI memory.
  *         MX66L2G support 4K, 32K, 64K size block erase commands.
  *         SPI/QPI; 1-1-0/4-4-0
  * @param  BlockAddress: Block address to erase
  * @retval QSPI memory status
  */
int32_t MX66L2G_BlockErase(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t BlockAddress, MX66L2G_EraseTypeDef BlockSize)
{
  QSPI_CommandTypeDef s_command;

  /* Setup erase command */
  switch(BlockSize)
  {
  case MX66L2G_ERASE_4K :
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_SECTOR_ERASE_4K_CMD;
    break;

  case MX66L2G_ERASE_32K :
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_BLOCK_ERASE_32K_CMD;
    break;

  case MX66L2G_ERASE_64K :
    s_command.Instruction     = MX66L2G_4_BYTE_ADDR_BLOCK_ERASE_64K_CMD;
    break;

  case MX66L2G_ERASE_CHIP :
    return MX66L2G_ChipErase(Ctx, Mode);

  default :
    return MX66L2G_ERROR_PARAMETER;
  }

  /* Initialize the erase command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.AddressMode       = (Mode == MX66L2G_QPI_MODE) ? QSPI_ADDRESS_4_LINES : QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}


/* Register/Setting Commands **************************************************/
/**
  * @brief  This function send a Write Enable and wait it is effective.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteEnable(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_config.Match           = MX66L2G_SR_WEL;
  s_config.Mask            = MX66L2G_SR_WEL;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction    = MX66L2G_READ_STATUS_REG_CMD;
  s_command.DataMode       = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(Ctx, &s_command, &s_config, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_AUTOPOLLING;
  }

  return MX66L2G_OK;
}

/**
  * @brief  This function reset the (WEL) Write Enable Latch bit.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteDisable(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef     s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_DISABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

#if 0
/**
  * @brief  Factory mode enable. For accelerate erase speed. MX66L2G command.
  *         Suspend command is not acceptable under factory mode.
  *         SPI/QPI; 1-0-0/4-0-0
  * @retval QSPI memory status
  */
int32_t MX66L2G_FactoryModeEnable(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize Factory mode enable command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_FACTORY_MODE_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}
#endif

/**
  * @brief  Read Flash Status register value
  *         SPI/QPI; 1-0-1/4-0-4
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadStatusRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Value)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reading of status register */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, Value, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read Flash Configuration register value
  *         SPI/QPI; 1-0-1/4-0-4
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadConfigurationRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Value)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reading of configuration register */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, Value, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write status & configuration register.
  *         SPI/QPI; 1-0-1/4-0-4
  * @param  Value for write to status & configuration register.
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteStatusConfigurationRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Value, uint32_t Length)
{
  QSPI_CommandTypeDef s_command;

  /* Update the configuration register with new value */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_STATUS_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = Length;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the write volatile configuration register command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, Value, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read Extended Address Register.
  *         This register use with 3 Byte address command
  *         SPI/QPI; 1-0-1/4-0-4
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadExtendedAddressRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *EAR)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_EXTENDED_ADDR_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, EAR, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write Extended Address Register.
  *         This register use with 3 Byte address command
  *         SPI/QPI; 1-0-1/4-0-4
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteExtendedAddressRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t EAR)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_EXTENDED_ADDR_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, &EAR, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write Protect Selection; WPSEL,
  *         Enter and enable individual block protect mode
  *         SPI/QPI; 1-0-0/4-0-0
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteProtectSelection(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize Write Protect Selection command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_PROTECT_SELECTION_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  This function put QSPI memory in QPI mode (Quad I/O) from SPI mode.
  *         SPI -> QPI; 1-x-x -> 4-4-4
  *         SPI; 1-0-0
  * @param  Ctx: QSPI handle
  * @retval QSPI memory status
  */
int32_t MX66L2G_EnterQPIMode(void *Ctx)
{
  QSPI_CommandTypeDef      s_command;

  /* Initialize the QPI enable command */
  /* QSPI memory is supported to be in SPI mode, so CMD on 1 LINE */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_ENABLE_QSPI_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  This function put QSPI memory in SPI mode (Single I/O) from QPI mode.
  *         QPI -> SPI; 4-4-4 -> 1-x-x
  *         QPI; 4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ExitQPIMode(void *Ctx)
{
  QSPI_CommandTypeDef      s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = MX66L2G_RESET_QSPI_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Flash enter 4 Byte address mode. Effect 3/4 address byte commands only.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_Enter4BytesAddressMode(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_ENTER_4_BYTE_ADDR_MODE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Flash exit 4 Byte address mode. Effect 3/4 address byte commands only.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_Exit4BytesAddressMode(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_EXIT_4_BYTE_ADDR_MODE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/*
  * @brief  Program/Erases suspend. Interruption Program/Erase operations.
  *         After the device has entered Erase-Suspended mode,
  *         system can read any address except the block/sector being Program/Erased.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ProgramEraseSuspend(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_PROG_ERASE_SUSPEND_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/*
  * @brief  Program/Erases resume. Resume from Program/Erases suspend.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ProgramEraseResume(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_PROG_ERASE_RESUME_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/*
  * @brief  Deep power down.
  *         The device is not active and all Write/Program/Erase instruction are ignored.
  *         tDP time is required before enter Deep power down after CS# goes high.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_EnterDeepPowerDown(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_DEEP_POWER_DOWN_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

#if 1
/*
  * @brief  Release from deep power down.
  *         After CS# go high, system need wait tRES1 time for device ready.
  *         SPI/QPI; 1-0-0/4-0-0
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReleaseFromDeepPowerDown(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_RELEASE_FROM_DEEP_POWER_DOWN_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}
#endif

/**
  * @brief  Set burst length.
  *         The SPI and QPI mode 4READ and 4READ4B read commands support the wrap around feature.
  *         Read commands QPI "EBh" "ECh" and SPI "EBh" "ECh" support this feature.
  *         SPI/QPI; 1-0-1/4-0-4
  * @param  Value of burst length.
  * @retval QSPI memory status
  */
int32_t MX66L2G_SetBurstLength(void *Ctx, MX66L2G_InterfaceTypeDef Mode, MX66L2G_WrapLengthTypeDef BurstLength)
{
  QSPI_CommandTypeDef s_command;
  uint8_t reg = BurstLength;

  /* Initialize the set burst length command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_SET_BURST_LENGTH_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read flash fast boot register value, SPI only.
  *         SPI; 1-0-1
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadFastBootRegister(void *Ctx, uint8_t *FastBoot)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reading of fast boot register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_FAST_BOOT_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 4;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, FastBoot, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write fast boot register. Always write 4 byte data, SPI only.
  *         SPI; 1-0-1
  * @param  Value for write to status register.
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteFastBootRegister(void *Ctx, uint8_t *FastBoot)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the write fast boot register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_FAST_BOOT_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 4;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, FastBoot, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Erases fast boot register, SPI only.
  *         SPI; 1-0-0
  * @retval QSPI memory status
  */
int32_t MX66L2G_EraseFastBootRegister(void *Ctx)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Erases fast boot register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_ERASE_FAST_BOOT_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}


/* ID/Security Commands *******************************************************/
/**
  * @brief  Read Flash 3 Byte IDs.
  *         Manufacturer ID, Memory type, Memory density
  *         SPI/QPI; 1-0-1/4-0-4
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadID(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *ID)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = (Mode == MX66L2G_QPI_MODE) ? MX66L2G_MULTIPLE_IO_READ_ID_CMD : MX66L2G_READ_ID_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 3;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, ID, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read Flash Electronic Signature
  *         SPI/QPI; 1-1-1/4-4-4
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadElectronicSignature(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *ID)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_ELECTRONIC_ID_CMD;
  s_command.AddressMode       = (Mode == MX66L2G_QPI_MODE) ? QSPI_ADDRESS_4_LINES : QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = 0x0055AA55;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, ID, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read Flash Electronic Manufacturer & Device ID
  *         SPI; 1-1-1
  * @param  Component object pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadElectronicManufacturerDeviceID(void *Ctx, uint32_t Addr, uint8_t *ID)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read ID command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_ELECTRONIC_MANFACTURER_ID_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = Addr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 2;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, ID, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Reads an amount of SFDP data from the QSPI memory.
  *         SFDP : Serial Flash Discoverable Parameter
  *         SPI/QPI; 1-1-1/4-4-4
  * @param  pData: Pointer to data to be read
  *         ReadAddr: Read start address
  *         Size: Size of data to read in Byte
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadSFDP(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the read SFDP command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_SERIAL_FLASH_DISCO_PARAM_CMD;
  s_command.AddressMode       = (Mode == MX66L2G_QPI_MODE) ? QSPI_ADDRESS_4_LINES : QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = MX66L2G_DUMMY_CLOCK_READ_SFDP;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = Size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Enter Secured OTP mode
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_EnterSecuredOTP(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Enter Secured OTP command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_ENTER_SECURED_OTP_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Exit Secured OTP mode
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_ExitSecuredOTP(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Exit Secured OTP command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_EXIT_SECURED_OTP_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read Security Register value
  *         SPI/QPI; 1-0-1, 4-0-4
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadSecurityRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Security)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Read Security Register command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_SECURITY_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = (Mode == MX66L2G_QPI_MODE) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, Security, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write Security Register. To set the "Lock-Down" bit as 1.
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteSecurityRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Write security register with new value */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_SECURITY_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the write security register command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Gang block lock. Whole chip write protect.
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_GangBlockLock(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize gang block lock command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_GANG_BLOCK_LOCK_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if(HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Gang block unlock. Whole chip unprotect.
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_GangBlockUnlock(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the gang block unlock command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_GANG_BLOCK_UNLOCK_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if(HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write lock register. 2 Byte, SPI only
  *         SPI; 1-0-1
  * @param  Value for write to register.
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteLockRegister(void *Ctx, uint8_t *Lock)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Write Lock Register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_LOCK_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 2;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx,  &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, Lock, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read Lock Register value. 2 Byte, SPI only
  *         SPI; 1-0-1
  * @param  Read back data store pointer
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadLockRegister(void *Ctx, uint8_t *Lock)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Read Lock Register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_LOCK_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 2;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, Lock, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write password register. Always write 8 byte data, SPI only
  *         SPI; 1-0-1
  * @param  Value for write to status register.
  * @retval QSPI memory status
  */
int32_t MX66L2G_WritePasswordRegister(void *Ctx, uint8_t *Password)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Write password register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_PASSWORD_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 8;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, Password, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read password register value. 8 Byte, SPI only
  *         SPI; 1-0-1
  * @param  8 Byte password pointer.
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadPasswordRegister(void *Ctx, uint8_t *Password)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Read password register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_PASSWORD_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 8;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, Password, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Password unlock. Always write 8 byte data, SPI only
  *         SPI; 1-0-1
  * @param  8 Byte password pointer for compare to Password register.
  * @retval QSPI memory status
  */
int32_t MX66L2G_PasswordUnlock(void *Ctx, uint8_t *Password)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Password unlock command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_PASSWORD_UNLOCK_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 8;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, Password, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Write SPB. SPB bit program. SPI only
  *         SPI; 1-1-0
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteSPB(void *Ctx, uint32_t WriteAddr)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Write SPB command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_SPB_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = WriteAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Erase SPB (ESSPB), All SPB bit erase. SPI only
  *         SPI; 1-0-0
  * @retval QSPI memory status
  */
int32_t MX66L2G_EraseSPB(void *Ctx)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the All SPB bit erase command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_ERASE_SPB_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read SPB status. SPI only
  *         SPI; 1-1-1
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadSPBStatus(void *Ctx, uint32_t ReadAddr, uint8_t *SPBStatus)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reading of SPB lock status command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_SPB_STATUS_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, SPBStatus, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}

#if 0
/**
  * @brief  SPB lock set. SPI only
  *         SPI; 1-0-0
  * @retval QSPI memory status
   */
int32_t MX66L2G_SPBLockSet(void *Ctx)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the SPB lock set command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_SPB_LOCK_SET_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read SPB lock register value. SPI only
  *         SPI; 1-0-1
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadSPBLockRegister(void *Ctx, uint8_t *SPBRegister)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reading of SPB lock register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_SPB_LOCK_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, SPBRegister, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}
#endif

/**
  * @brief  Write DPB register. SPI only
  *         SPI; 1-1-1
  * @retval QSPI memory status
  */
int32_t MX66L2G_WriteDPBRegister(void *Ctx, uint32_t WriteAddr, uint8_t DPBStatus)
{
  QSPI_CommandTypeDef s_command;
  uint8_t reg = DPBStatus;

  /* Initialize the Write DPB register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_WRITE_DPB_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = WriteAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, &reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Read DPB register, 1 Byte. SPI only
  *         SPI; 1-1-1
  * @retval QSPI memory status
  */
int32_t MX66L2G_ReadDPBRegister(void *Ctx, uint32_t ReadAddr, uint8_t *DPBRegister)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the Read DPB register command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_READ_DPB_REGISTER_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = ReadAddr;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 1;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(Ctx, DPBRegister, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_RECEIVE;
  }

  return MX66L2G_OK;
}


/* Reset Commands *************************************************************/
/**
  * @brief  No Operation command for exit from reset enable mode
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_NoOperation(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reset enable command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_NO_OPERATION_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Flash reset enable command
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_ResetEnable(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reset enable command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_RESET_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Flash reset memory command
  *         SPI/QPI; 1-0-0, 4-0-0
  * @param  Ctx, Component object pointer
  *         Mode, Interface select
  * @retval QSPI memory status
  */
int32_t MX66L2G_ResetMemory(void *Ctx, MX66L2G_InterfaceTypeDef Mode)
{
  QSPI_CommandTypeDef s_command;

  /* Initialize the reset enable command */
  s_command.InstructionMode   = (Mode == MX66L2G_QPI_MODE) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX66L2G_RESET_MEMORY_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_COMMAND;
  }

  return MX66L2G_OK;
}

/**
  * @brief  Issue Performance Enhance Mode Reset COmmand
  *         3 Byte address mode 8 clock
  *         4 Byte address mode 10 clock
  * @retval QSPI memory status
  */
int32_t MX66L2G_PerformanceEnhanceModeReset(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize)
{
  QSPI_CommandTypeDef s_command;
  uint8_t SPIBuffer[] = {0x11, 0x11, 0x11, 0x11};
  uint8_t QPIBuffer[] = {0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t *Pointer;

  Pointer = (Mode == MX66L2G_QPI_MODE) ? QPIBuffer : SPIBuffer;

  /* Initialize command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = (Mode == MX66L2G_QPI_MODE) ? 0xFF : 0x11;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  //s_command.AddressSize       = AddressSize;
  //s_command.Address           = Address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.NbData            = (AddrSize == QSPI_ADDRESS_32_BITS) ? 4 : 3;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(Ctx, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {

    return MX66L2G_ERROR_COMMAND;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(Ctx, Pointer, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX66L2G_ERROR_TRANSMIT;
  }

  return MX66L2G_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
