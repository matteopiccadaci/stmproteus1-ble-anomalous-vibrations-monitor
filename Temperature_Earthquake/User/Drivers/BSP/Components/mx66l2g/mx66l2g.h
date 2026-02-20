/**
  ******************************************************************************
  * @file    mx66l2g.h
  * @brief   This file contains all the description of the MX66L2G QSPI memory
  ******************************************************************************
  @verbatim
  MX66L2G action :
    QE(Quad Enable, Non-volatile) bit of Status Register
    QE = 0; WP# pin active
            Accept 1-1-1, 1-1-2, 1-2-2 commands
    QE = 1; WP# become SIO2 pin
            Accept 1-1-1, 1-1-2, 1-2-2, 1-1-4, 1-4-4 commands
    Enter QPI mode by issue EQIO(0x35) command from 1-1-1 mode
            Accept 4-4-4 commands
    Exit QPI mode by issue RSTQIO(0xF5) command from 4-4-4 mode
            Accept commands, dependent QE bit status
    Memory Read commands support STR(Single Transfer Rate) &
    DTR(Double Transfer Rate) mode
  *
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
#ifndef __MX66L2G_H
#define __MX66L2G_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "mx66l2g_conf.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */


/** @defgroup MX66L2G
  * @brief    This file provides a set of functions needed to drive the
  *           mx66l2g qspi nor flash memory.
  * @{
  */

/** @defgroup MX66L2G_PUBLIC_DEFINES
  * @{
  */
  
/* Device ID **************************************************/
#define MX66L2G_MANUFACTURER_ID             ((uint8_t)0xC2)
#define MX66L2G_MEMORY_TYPE                 ((uint8_t)0x20)
#define MX66L2G_MEMORY_CAPACITY             ((uint8_t)0x1C)

/* Device Size, Block, Sector, Page & Secure OTP Size ************************/
#define MX66L2G_FLASH_ID                    0x001C20C2U                             /* ID 0xC2 0x20 0x1C */
#define MX66L2G_FLASH_SIZE                  ((uint32_t) 2 * 1024 * 1024 * 1024 / 8) /* 2 GBits => 256 MBytes */
#define MX66L2G_BLOCK_64K                   ((uint32_t) 64 * 1024)                  /* 4096 blocks of 64 KBytes */
#define MX66L2G_BLOCK_32K                   ((uint32_t) 32 * 1024)                  /* 8192 blocks of 32 KBytes */
#define MX66L2G_SECTOR_4K                   ((uint32_t) 4 * 1024)                   /* 655536 sectors of 4 KBytes */
#define MX66L2G_PAGE_SIZE                   ((uint32_t) 256)                        /* 1048576 pages of 256 Bytes */
#define MX66L2G_SECURE_OTP_SIZE             ((uint32_t) 4 * 1024 / 8)               /* 4K-bit => 512 Bytes secured OTP */

/**
  * @brief  MX66L2G Configuration
  * CS# Deselect Time :
  *   From Read to next Read 7 ns
  *   From Write/Erase/Program to Read Status Register 30 ns
  */
#define MX66L2G_DUMMY_CLOCK_READ_SFDP       8

#define MX66L2G_WRITE_REG_MAX_TIME          40       /* Write Status Register Cycle Time */
#define MX66L2G_SECTOR_4K_ERASE_MAX_TIME    400      /* Sector Erase Cycle Time (4KB) */
#define MX66L2G_BLOCK_32K_ERASE_MAX_TIME    1000     /* Block Erase Cycle Time (32KB) */
#define MX66L2G_BLOCK_64K_ERASE_MAX_TIME    2000     /* Block Erase Cycle Time (64KB) */
#define MX66L2G_CHIP_ERASE_MAX_TIME         300000   /* Chip Erase Cycle Time */

/* MX66L2G Component Error codes *********************************************/
#define MX66L2G_OK                           0
#define MX66L2G_ERROR_INIT                  -1
#define MX66L2G_ERROR_COMMAND               -2
#define MX66L2G_ERROR_TRANSMIT              -3
#define MX66L2G_ERROR_RECEIVE               -4
#define MX66L2G_ERROR_AUTOPOLLING           -5
#define MX66L2G_ERROR_MEMORYMAPPED          -6
#define MX66L2G_ERROR_PARAMETER             -7

/******************************************************************************
  * @brief  MX66L2G Commands
  ****************************************************************************/
/***** Read/Write Array Commands (3/4 Byte Address Command Set) **************/
/* Read Operations */
#define MX66L2G_READ_CMD                             0x03   // READ, Normal Read 3/4 Byte Address; SPI 1-1-1
#define MX66L2G_FAST_READ_CMD                        0x0B   // FAST READ, Fast Read 3/4 Byte Address; SPI 1-1-1
#define MX66L2G_2IO_FAST_READ_CMD                    0xBB   // 2READ, 2 x I/O Read 3/4 Byte Address; SPI 1-2-2
#define MX66L2G_1I2O_FAST_READ_CMD                   0x3B   // DREAD, 1I 2O Read 3/4 Byte Address; SPI 1-1-2
#define MX66L2G_4IO_FAST_READ_CMD                    0xEB   // 4READ, 4 I/O Read 3/4 Byte Address; SPI/QPI 1-4-4/4-4-4
#define MX66L2G_1I4O_FAST_READ_CMD                   0x6B   // QREAD, 1I 4O Read 3/4 Byte Address; SPI 1-1-4
//#define MX66L2G_FAST_READ_DTR_CMD                    0x0D   // FASTDTRD, Fast DTR Read 3/4 Byte Address; SPI 1-1-1
//#define MX66L2G_2IO_FAST_READ_DTR_CMD                0xBD   // 2DTRD, Dual I/O DTR Read 3/4 Byte Address; SPI 1-2-2
#define MX66L2G_4IO_FAST_READ_DTR_CMD                0xED   // 4DTRD, Quad I/O DTR Read 3/4 Byte Address; SPI/QPI 1-4-4/4-4-4

/* Program Operations */
#define MX66L2G_PAGE_PROG_CMD                        0x02   // PP, Page Program 3/4 Byte Address; SPI/QPI 1-1-1/4-4-4
#define MX66L2G_QUAD_PAGE_PROG_CMD                   0x38   // 4PP, Quad Page Program 3/4 Byte Address; SPI 1-4-4

/* Erase Operations */
#define MX66L2G_SECTOR_ERASE_4K_CMD                  0x20   // SE, Sector Erase 4KB 3/4 Byte Address; SPI/QPI 1-1-0/4-4-0
#define MX66L2G_BLOCK_ERASE_32K_CMD                  0x52   // BE32K, Block Erase 32KB 3/4 Byte Address; SPI/QPI 1-1-0/4-4-0
#define MX66L2G_BLOCK_ERASE_64K_CMD                  0xD8   // BE, Block Erase 64KB 3/4 Byte Address; SPI/QPI 1-1-0/4-4-0
#define MX66L2G_CHIP_ERASE_CMD                       0xC7   // CE, Chip Erase 0 Byte Address; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_BULK_ERASE_CMD                       0x60   // CE, Chip Erase 0 Byte Address; SPI/QPI 1-0-0/4-0-0

/***** Read/Write Array Commands (4 Byte Address Command Set) ****************/
/* Read Operations */
#define MX66L2G_4_BYTE_ADDR_READ_CMD                 0x13   // READ4B, Normal Read 4 Byte address; SPI 1-1-1
#define MX66L2G_4_BYTE_ADDR_FAST_READ_CMD            0x0C   // FAST READ4B, Fast Read 4 Byte address; SPI 1-1-1
#define MX66L2G_4_BYTE_ADDR_2IO_FAST_READ_CMD        0xBC   // 2READ4B, Read by 2 x I/O 4 Byte address; SPI 1-2-2
#define MX66L2G_4_BYTE_ADDR_1I2O_FAST_READ_CMD       0x3C   // DREAD4B, Read by Dual Output 4 Byte address; SPI 1-1-2
#define MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_CMD        0xEC   // 4READ4B, Read by 4 x I/O 4 Byte address; SPI/QPI 1-4-4/4-4-4
#define MX66L2G_4_BYTE_ADDR_1I4O_FAST_READ_CMD       0x6C   // QREAD4B, Read by Quad Output 4 Byte address; SPI 1-1-4
//#define MX66L2G_4_BYTE_ADDR_FAST_READ_DTR_CMD        0x0E   // FRDTRD4B, Fast DTR Read 4 Byte address; SPI 1-1-1
//#define MX66L2G_4_BYTE_ADDR_2IO_FAST_READ_DTR_CMD    0xBE   // 2DTRD4B, DTR read by 2 x I/O 4 Byte address; SPI 1-2-2
#define MX66L2G_4_BYTE_ADDR_4IO_FAST_READ_DTR_CMD    0xEE   // 4DTRD4B, Quad I/O DTR Read 4 Byte address; SPI/QPI 1-4-4/4-4-4

/* Program Operations */
#define MX66L2G_4_BYTE_ADDR_PAGE_PROG_CMD            0x12   // PP4B, Page Program 4 Byte address; SPI/QPI 1-1-1/4-4-4
#define MX66L2G_4_BYTE_ADDR_QUAD_PAGE_PROG_CMD       0x3E   // 4PP4B, Quad Input Page Program 4 Byte address; SPI 1-4-4

/* Erase Operations */
#define MX66L2G_4_BYTE_ADDR_BLOCK_ERASE_64K_CMD      0xDC   // BE4B, Block Erase 64KB 4 Byte address; SPI/QPI 1-1-0/4-4-0
#define MX66L2G_4_BYTE_ADDR_BLOCK_ERASE_32K_CMD      0x5C   // BE32K4B, Block Erase 32KB 4 Byte address; SPI/QPI 1-1-0/4-4-0
#define MX66L2G_4_BYTE_ADDR_SECTOR_ERASE_4K_CMD      0x21   // SE4B, Sector Erase 4KB 4 Byte address; SPI/QPI 1-1-0/4-4-0

/***** Register/Setting Commands *********************************************/
#define MX66L2G_WRITE_ENABLE_CMD                     0x06   // WREN, Write Enable; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_WRITE_DISABLE_CMD                    0x04   // WRDI, Write Disable; SPI/QPI 1-0-0/4-0-0

//#define MX66L2G_FACTORY_MODE_ENABLE_CMD              0x41   // FMEN, Factory Mode Enable; SPI/QPI 1-0-0/4-0-0

#define MX66L2G_READ_STATUS_REG_CMD                  0x05   // RDSR, Read Status Register; SPI/QPI 1-0-1/4-0-4
#define MX66L2G_READ_CFG_REG_CMD                     0x15   // RDCR, Read Configuration Register; SPI/QPI 1-0-1/4-0-4
#define MX66L2G_WRITE_STATUS_CFG_REG_CMD             0x01   // WRSR, Write Status/Configuration Register; SPI/QPI 1-0-1/4-0-4

#define MX66L2G_READ_EXTENDED_ADDR_REG_CMD           0xC8   // RDEAR, Read Extended Address Register; SPI/QPI 1-0-1/4-0-4
#define MX66L2G_WRITE_EXTENDED_ADDR_REG_CMD          0xC5   // WREAR, Write Extended Address Register; SPI/QPI 1-0-1/4-0-4
#define MX66L2G_WRITE_PROTECT_SELECTION_CMD          0x68   // WPSEL, Write Protect Selection; SPI/QPI 1-0-0/4-0-0

#define MX66L2G_ENABLE_QSPI_CMD                      0x35   // EQIO, Enable QPI; SPI 1-0-0
#define MX66L2G_RESET_QSPI_CMD                       0xF5   // RSTQIO, Reset QPI; QPI 4-0-0

#define MX66L2G_ENTER_4_BYTE_ADDR_MODE_CMD           0xB7   // EN4B, Enter 4-Byte mode (3/4 Byte address commands); SPI/QPI 1-0-0/4-0-0
#define MX66L2G_EXIT_4_BYTE_ADDR_MODE_CMD            0xE9   // EX4B, Exit 4-Byte mode (3/4 Byte address commands); SPI/QPI 1-0-0/4-0-0

#define MX66L2G_PROG_ERASE_SUSPEND_CMD               0xB0   // PGM/ERS Suspend, Suspends Program/Erase; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_PROG_ERASE_RESUME_CMD                0x30   // PGM/ERS Resume, Resumes Program/Erase; SPI/QPI 1-0-0/4-0-0

#define MX66L2G_DEEP_POWER_DOWN_CMD                  0xB9   // DP, Deep power down; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_RELEASE_FROM_DEEP_POWER_DOWN_CMD     0xAB   // RDP, Release from Deep Power down; SPI/QPI 1-0-0/4-0-0

#define MX66L2G_SET_BURST_LENGTH_CMD                 0xC0   // SBL, Set burst length; SPI/QPI 1-0-1/4-0-4

#define MX66L2G_READ_FAST_BOOT_REGISTER_CMD          0x16   // RDFBR, Read Fast Boot Register; SPI 1-0-1
#define MX66L2G_WRITE_FAST_BOOT_REGISTER_CMD         0x17   // WRFBR, Write Fast Boot Register; SPI 1-0-1
#define MX66L2G_ERASE_FAST_BOOT_REGISTER_CMD         0x18   // ESFBR, Erase Fast Boot Register; SPI 1-0-0

/***** ID/Security Commands **************************************************/
/* Identification Operations */
#define MX66L2G_READ_ID_CMD                          0x9F   // RDID, Read IDentification; SPI 1-0-1
#define MX66L2G_READ_ELECTRONIC_ID_CMD               0xAB   // RES, Read Electronic ID; SPI/QPI 1-1-1/4-4-4
#define MX66L2G_READ_ELECTRONIC_MANFACTURER_ID_CMD   0x90   // REMS, Read Electronic Manufacturer ID & Device ID; SPI 1-1-1
#define MX66L2G_MULTIPLE_IO_READ_ID_CMD              0xAF   // QPIID, QPI ID Read; QPI 4-0-4

#define MX66L2G_READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A   // RDSFDP, Read Serial Flash Discoverable Parameter; SPI/QPI 1-1-1/4-4-4

#define MX66L2G_ENTER_SECURED_OTP_CMD                0xB1   // ENSO, Enter Secured OTP; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_EXIT_SECURED_OTP_CMD                 0xC1   // EXSO, Exit Secured OTP; SPI/QPI 1-0-0/4-0-0

#define MX66L2G_READ_SECURITY_REGISTER_CMD           0x2B   // RDSCUR, Read Security Register; SPI/QPI 1-0-1/4-0-4
#define MX66L2G_WRITE_SECURITY_REGISTER_CMD          0x2F   // WRSCUR, Write Security Register; SPI/QPI 1-0-0/4-0-0

#define MX66L2G_GANG_BLOCK_LOCK_CMD                  0x7E   // GBLK, Gang Block Lock; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_GANG_BLOCK_UNLOCK_CMD                0x98   // GBULK, Gang Block Unlock; SPI/QPI 1-0-0/4-0-0

#define MX66L2G_WRITE_LOCK_REG_CMD                   0x2C   // WRLR, Write Lock Register; SPI 1-0-1
#define MX66L2G_READ_LOCK_REG_CMD                    0x2D   // RDLR, Read Lock Register; SPI 1-0-1

#define MX66L2G_WRITE_PASSWORD_REGISTER_CMD          0x28   // WRPASS, Write Password Register; SPI 1-0-1
#define MX66L2G_READ_PASSWORD_REGISTER_CMD           0x27   // RDPASS, Read Password Register; SPI 1-0-1
#define MX66L2G_PASSWORD_UNLOCK_CMD                  0x29   // PASSULK, Password Unlock; SPI 1-0-1

#define MX66L2G_WRITE_SPB_CMD                        0xE3   // WRSPB, SPB bit program 4 Byte address; SPI 1-1-0
#define MX66L2G_ERASE_SPB_CMD                        0xE4   // ESSPB, All SPB bit Erase; SPI 1-0-0
#define MX66L2G_READ_SPB_STATUS_CMD                  0xE2   // RDSPB, Read SPB Status 4 Byte address; SPI 1-1-1

//#define MX66L2G_SPB_LOCK_SET_CMD                     0xA6   // SPBLK, SPB Lock set; SPI 1-0-0
//#define MX66L2G_READ_SPB_LOCK_REGISTER_CMD           0xA7   // RDSPBLK, SPB Lock Register Read; SPI 1-0-1

#define MX66L2G_WRITE_DPB_REGISTER_CMD               0xE1   // WRDPB, Write DPB Register; SPI 1-1-1
#define MX66L2G_READ_DPB_REGISTER_CMD                0xE0   // RDDPB, Read DPB Register; SPI 1-1-1

/***** Reset Commands ********************************************************/
#define MX66L2G_NO_OPERATION_CMD                     0x00   // NOP, No Operation; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_RESET_ENABLE_CMD                     0x66   // RSETEN, Reset Enable; SPI/QPI 1-0-0/4-0-0
#define MX66L2G_RESET_MEMORY_CMD                     0x99   // RST, Reset Memory; SPI/QPI 1-0-0/4-0-0


/******************************************************************************
  * @brief  MX66L2G Registers
  ****************************************************************************/
/* Status Register */
#define MX66L2G_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
#define MX66L2G_SR_WEL                      ((uint8_t)0x02)    /*!< Write enable latch */
#define MX66L2G_SR_BP                       ((uint8_t)0x3C)    /*!< Block protected against program and erase operations */
#define MX66L2G_SR_QE                       ((uint8_t)0x40)    /*!< QE bit; Quad IO mode enabled if =1 */
#define MX66L2G_SR_SRWD                     ((uint8_t)0x80)    /*!< Status register write enable/disable */

/* Configuration Register */
#define MX66L2G_CR_ODS                      ((uint8_t)0x07)    /*!< Output driver strength */
#define MX66L2G_CR_TB                       ((uint8_t)0x08)    /*!< Top/Bottom bit used to configure the block protect area */
#define MX66L2G_CR_PBE                      ((uint8_t)0x10)    /*!< Preamble Bit Enable */
#define MX66L2G_CR_4BYTE                    ((uint8_t)0x20)    /*!< 3-Bytes or 4-Bytes addressing */
#define MX66L2G_CR_DC                       ((uint8_t)0xC0)    /*!< Dummy Clock Cycles setting */

/* Fast Boot Register */
#define MX66L2G_FBR_FBE                     ((uint32_t)0x00000001) /*!< FastBoot Enable */
#define MX66L2G_FBR_FBSD                    ((uint32_t)0x00000006) /*!< FastBoot Start Delay Cycle */
#define MX66L2G_FBR_FBSD_7                  ((uint32_t)0x00000000) /*!< FastBoot Start 7 Delay Cycle */
#define MX66L2G_FBR_FBSD_9                  ((uint32_t)0x00000001) /*!< FastBoot Start 9 Delay Cycle */
#define MX66L2G_FBR_FBSD_11                 ((uint32_t)0x00000002) /*!< FastBoot Start 11 Delay Cycle */
#define MX66L2G_FBR_FBSD_13                 ((uint32_t)0x00000003) /*!< FastBoot Start 12 Delay Cycle */
#define MX66L2G_FBR_FBSA                    ((uint32_t)0xFFFFFFF0) /*!< FastBoot Start Address */

/* Security Register */
#define MX66L2G_SCUR_OTP_INDICATOR          ((uint8_t)0x01)    /*!< Secured OTP indicator bit */
#define MX66L2G_SCUR_LDSO                   ((uint8_t)0x02)    /*!< Indicate if Lock-down Secured OTP */
#define MX66L2G_SCUR_PSB                    ((uint8_t)0x04)    /*!< Program Suspend bit */
#define MX66L2G_SCUR_ESB                    ((uint8_t)0x08)    /*!< Erase Suspend bit */
#define MX66L2G_SCUR_P_FAIL                 ((uint8_t)0x20)    /*!< Indicate Program failed */
#define MX66L2G_SCUR_E_FAIL                 ((uint8_t)0x40)    /*!< Indicate Erase failed */
#define MX66L2G_SCUR_WPSEL                  ((uint8_t)0x80)    /*!< Write Protection Selection */

/* Lock Register */
#define MX66L2G_LR_PASSWORD_LOCK            ((uint16_t)0x0004) /*!< Password Protection Mode Lock bit (OTP) */
#define MX66L2G_LR_SPBLKDN                  ((uint16_t)0x0040) /*!< SPB Lock Down bit (SPBLKDN) (OTP) */

/* SPB Lock Register */
/* #define MX66L2G_SPBLR_SPBLK                 ((uint8_t)0x01) */    /*!< SPB Lock Bit */

/**
 * @}
 */

/** @defgroup MX66L2G_PUBLIC_TYPES
  * @{
  */

typedef struct
{
  uint32_t FlashID;             /*!< ID of the flash */
  uint32_t FlashSize;           /*!< Byte size of the flash */
  uint32_t EraseSectorSize;     /*!< Size of sectors for the erase operation */
  uint32_t EraseSectorsNumber;  /*!< Number of sectors for the erase operation */
  uint32_t ProgPageSize;        /*!< Size of pages for the program operation */
  uint32_t ProgPagesNumber;     /*!< Number of pages for the program operation */

  uint32_t EraseBlockSize;      /*!< Size of block for the erase operation */
  uint32_t EraseBlockNumber;    /*!< Number of block for the erase operation */
  uint32_t EraseBlockSize1;     /*!< Size 1 of block for the erase operation */
  uint32_t EraseBlockNumber1;   /*!< Number 1 of block for the erase operation */
} MX66L2G_Info_t;

typedef enum
{
  MX66L2G_SPI_NORMAL_MODE = 0,         // 1-1-1 read command with 0 dummy cycle
  MX66L2G_SPI_MODE,                    // 1-1-1 commands, Power on H/W default setting
  MX66L2G_SPI_2IO_MODE,                // 1-2-2 read command
  MX66L2G_SPI_1I2O_MODE,               // 1-1-2 read command
  MX66L2G_SPI_4IO_MODE,                // 1-4-4 read command
  MX66L2G_SPI_1I4O_MODE,               // 1-1-4 read command
  MX66L2G_QPI_MODE,                    // 4-4-4 commands
} MX66L2G_InterfaceTypeDef;

typedef enum
{
  MX66L2G_STR_TRANSFER = 0,             /* Single Transfer Rate */
  MX66L2G_DTR_TRANSFER                  /* Double Transfer Rate */
} MX66L2G_TransferTypeDef;

typedef enum
{
  MX66L2G_ERASE_CHIP = 0,               // Whole chip erase
  MX66L2G_ERASE_4K   = (4 * 1024),      // 4K size Sector erase
  MX66L2G_ERASE_32K  = (32 * 1024),     // 32K size Block erase
  MX66L2G_ERASE_64K  = (64 * 1024)      // 64K size Block erase
} MX66L2G_EraseTypeDef;

typedef enum
{
  MX66L2G_BURST_READ_WRAP_8 = 0,
  MX66L2G_BURST_READ_WRAP_16,
  MX66L2G_BURST_READ_WRAP_32,
  MX66L2G_BURST_READ_WRAP_64,
  MX66L2G_BURST_READ_WRAP_NONE = 0x1F   // Disable wrap function
} MX66L2G_WrapLengthTypeDef;

/* Configuration Register ODS & Dummy clock cycle setting ******************************/
typedef enum
{
  MX66L2G_CR_ODS_146 = 0,               /*!< ((uint8_t)0x00) Output driver strength 146 ohms */
  MX66L2G_CR_ODS_76,                    /*!< ((uint8_t)0x01) Output driver strength 76 ohms */
  MX66L2G_CR_ODS_52,                    /*!< ((uint8_t)0x02) Output driver strength 52 ohms */
  MX66L2G_CR_ODS_41,                    /*!< ((uint8_t)0x03) Output driver strength 41 ohms */
  MX66L2G_CR_ODS_34,                    /*!< ((uint8_t)0x04) Output driver strength 34 ohms */
  MX66L2G_CR_ODS_30,                    /*!< ((uint8_t)0x05) Output driver strength 30 ohms */
  MX66L2G_CR_ODS_26,                    /*!< ((uint8_t)0x06) Output driver strength 26 ohms */
  MX66L2G_CR_ODS_24                     /*!< ((uint8_t)0x07) Output driver strength 24 ohms (default)*/
} MX66L2G_ODSTypeDef;

/* MX66L2G Dummy Clock setting
 *          |             Instruction format
 *    DUMMY |1-1-1, 1-1-2     |1-2-2     |1-4-4
 *    [1:0] |1-1-4, 1-1-1(DTR)|1-2-2(DTR)|4-4-4, 4-4-4(DTR)
 *   -------+-----------------+----------+-----------------
 *     00   |    8 clocks     | 4 clocks |    6 clocks
 *     01   |    6 clocks     | 6 clocks |    4 clocks
 *     10   |    8 clocks     | 8 clocks |    8 clocks
 *     11   |   10 clocks     |10 clocks |   10 clocks
 */
typedef enum
{
  MX66L2G_CR_DUMMY_00 = 0,     /*!<  8 clocks for 1-1-x; 4 clocks for 1-2-2; 6 clocks for x-4-4 */
  MX66L2G_CR_DUMMY_01,         /*!<  6 clocks for 1-1-x; 6 clocks for 1-2-2; 4 clocks for x-4-4 */
  MX66L2G_CR_DUMMY_10,         /*!<  8 clocks */
  MX66L2G_CR_DUMMY_11          /*!< 10 clocks */
} MX66L2G_DummyValueTypeDef;

typedef enum
{
  MX66L2G_CR_DUMMY11x_08 = 0,  /*!< 1-1-1 133MHz, 1-1-2 133MHz, 1-1-4 133MHz, 1-1-1 DTR 66MHz */
  MX66L2G_CR_DUMMY11x_6,       /*!< 1-1-1 133MHz, 1-1-2 133MHz, 1-1-4 104MHz, 1-1-1 DTR 66MHz */
  MX66L2G_CR_DUMMY11x_8,       /*!< 1-1-1 133MHz, 1-1-2 133MHz, 1-1-4 133MHz, 1-1-1 DTR 66MHz */
  MX66L2G_CR_DUMMY11x_10       /*!< 1-1-1 166MHz, 1-1-2 166MHz, 1-1-4 166MHz, 1-1-1 DTR 83MHz */
} MX66L2G_Dummy11xTypeDef;

typedef enum
{
  MX66L2G_CR_DUMMY12x_4 = 0,   /*!< 1-2-2  84MHz, 1-2-2 DTR 52MHz */
  MX66L2G_CR_DUMMY12x_6,       /*!< 1-2-2 104MHz, 1-2-2 DTR 66MHz */
  MX66L2G_CR_DUMMY12x_8,       /*!< 1-2-2 133MHz, 1-2-2 DTR 66MHz */
  MX66L2G_CR_DUMMY12x_10       /*!< 1-2-2 166MHz, 1-2-2 DTR 83MHz */
} MX66L2G_Dummy122TypeDef;

typedef enum
{
  MX66L2G_CR_DUMMYx44_6 = 0,   /*!< 1-4-4/4-4-4  84MHz, 4-4-4 DTR  52MHz */
  MX66L2G_CR_DUMMYx44_4,       /*!< 1-4-4/4-4-4  70MHz, 4-4-4 DTR  42MHz */
  MX66L2G_CR_DUMMYx44_8,       /*!< 1-4-4/4-4-4 104MHz, 4-4-4 DTR  66MHz */
  MX66L2G_CR_DUMMYx44_10       /*!< 1-4-4/4-4-4 133MHz, 4-4-4 DTR 100MHz */
} MX66L2G_Dummyx44TypeDef;

/**
 * @}
 */

/** @defgroup MX66L2G_PUBLIC_FUNCTIONS
  * @{
  */

/* Function by commands combined *********************************************/
int32_t MX66L2G_GetFlashInfo(MX66L2G_Info_t *pInfo);
int32_t MX66L2G_AutoPollingMemReady(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t Timeout);

int32_t MX66L2G_EnableMemoryMappedMode34ByteSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize);
int32_t MX66L2G_EnableMemoryMappedMode34ByteDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize);

int32_t MX66L2G_EnableMemoryMappedModeSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_EnableMemoryMappedModeDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode);

/* Read/Write Array Commands (3/4 Byte Address Command Set) ******************/
int32_t MX66L2G_34ByteAddressReadSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t MX66L2G_34ByteAddressReadDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t MX66L2G_34ByteAddressPageProgram(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
int32_t MX66L2G_34ByteAddressBlockErase(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize, uint32_t BlockAddress, MX66L2G_EraseTypeDef BlockSize);
int32_t MX66L2G_ChipErase(void *Ctx, MX66L2G_InterfaceTypeDef Mode);

/* Read/Write Array Commands (4 Byte Address Command Set) ********************/
int32_t MX66L2G_ReadSTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t MX66L2G_ReadDTR(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t MX66L2G_PageProgram(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
int32_t MX66L2G_BlockErase(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t BlockAddress, MX66L2G_EraseTypeDef BlockSize);

/* Register/Setting Commands *************************************************/
int32_t MX66L2G_WriteEnable(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_WriteDisable(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_FactoryModeEnable(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ReadStatusRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Value);
int32_t MX66L2G_ReadConfigurationRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Value);
int32_t MX66L2G_WriteStatusConfigurationRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Value, uint32_t Length);
int32_t MX66L2G_ReadExtendedAddressRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *EAR);
int32_t MX66L2G_WriteExtendedAddressRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t EAR);
int32_t MX66L2G_WriteProtectSelection(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_EnterQPIMode(void *Ctx);
int32_t MX66L2G_ExitQPIMode(void *Ctx);
int32_t MX66L2G_Enter4BytesAddressMode(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_Exit4BytesAddressMode(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ProgramEraseSuspend(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ProgramEraseResume(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_EnterDeepPowerDown(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ReleaseFromDeepPowerDown(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_SetBurstLength(void *Ctx, MX66L2G_InterfaceTypeDef Mode, MX66L2G_WrapLengthTypeDef BurstLength);
int32_t MX66L2G_ReadFastBootRegister(void *Ctx, uint8_t *FastBoot);
int32_t MX66L2G_WriteFastBootRegister(void *Ctx, uint8_t *FastBoot);
int32_t MX66L2G_EraseFastBootRegister(void *Ctx);

/* ID/Security Commands ******************************************************/
int32_t MX66L2G_ReadID(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *ID);
int32_t MX66L2G_ReadElectronicSignature(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *ID);
int32_t MX66L2G_ReadElectronicManufacturerDeviceID(void *Ctx, uint32_t Addr, uint8_t *ID);
int32_t MX66L2G_ReadSFDP(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t MX66L2G_EnterSecuredOTP(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ExitSecuredOTP(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ReadSecurityRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint8_t *Security);
int32_t MX66L2G_WriteSecurityRegister(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_GangBlockLock(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_GangBlockUnlock(void *Ctx, MX66L2G_InterfaceTypeDef Mode);

int32_t MX66L2G_WriteLockRegister(void *Ctx, uint8_t *Lock);
int32_t MX66L2G_ReadLockRegister(void *Ctx, uint8_t *Lock);
int32_t MX66L2G_WritePasswordRegister(void *Ctx, uint8_t *Password);
int32_t MX66L2G_ReadPasswordRegister(void *Ctx, uint8_t *Password);
int32_t MX66L2G_PasswordUnlock(void *Ctx, uint8_t *Password);
int32_t MX66L2G_WriteSPB(void *Ctx, uint32_t WriteAddr);
int32_t MX66L2G_EraseSPB(void *Ctx);
int32_t MX66L2G_ReadSPBStatus(void *Ctx, uint32_t ReadAddr, uint8_t *SPBStatus);
//int32_t MX66L2G_SPBLockSet(void *Ctx);
//int32_t MX66L2G_ReadSPBLockRegister(void *Ctx, uint8_t *SPBRegister);
int32_t MX66L2G_WriteDPBRegister(void *Ctx, uint32_t WriteAddr, uint8_t DPBStatus);
int32_t MX66L2G_ReadDPBRegister(void *Ctx, uint32_t ReadAddr, uint8_t *DPBRegister);

/* Reset Commands ************************************************************/
int32_t MX66L2G_NoOperation(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ResetEnable(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_ResetMemory(void *Ctx, MX66L2G_InterfaceTypeDef Mode);
int32_t MX66L2G_PerformanceEnhanceModeReset(void *Ctx, MX66L2G_InterfaceTypeDef Mode, uint32_t AddrSize);

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

#ifdef __cplusplus
}
#endif

#endif /* __MX66L2G_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
