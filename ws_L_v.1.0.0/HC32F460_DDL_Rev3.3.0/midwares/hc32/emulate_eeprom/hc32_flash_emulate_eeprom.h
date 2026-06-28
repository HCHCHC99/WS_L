/**
 *******************************************************************************
 * @file  hc32_flash_emulate_eeprom.h
 * @brief This file contains all the functions prototypes of emulate eeprom.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2024-06-30       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

#ifndef __HC32_FLASH_EMULATE_EEPROM_H
#define __HC32_FLASH_EMULATE_EEPROM_H

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "flash_conf.h"

/**
 * @addtogroup LL_EMULATE_EEPROM_LIB
 * @{
 */

/**
 * @addtogroup HC32_FLASH_EMULATE_EEPROM
 * @{
 */

/*******************************************************************************
 * Global type definitions ('typedef')
*******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup EEPROM_Global_Macros EEPROM Global Macros
 * @{
 */

/**
 * @defgroup EEPROM_Generic_Error_Codes EEPROM Generic Error Codes
 * @{
 */
#define EE_OK                       (LL_OK)
#define EE_ERR                      (LL_ERR)
#define EE_ERR_NOT_RDY              (LL_ERR_NOT_RDY)
#define EE_ERR_INVD_PARAM           (LL_ERR_INVD_PARAM)
#define EE_ERR_SECT_FULL            (LL_ERR_BUF_FULL)
#define EE_ERR_SECT_INVALID         (0xAAU)
/**
 * @}
 */

/**
 * @defgroup First_EEPROM_Valid_Data_Addr First EEPROM Valid Data Addr
 * @{
 */
#define EE_SEC_VALID_ADDR_FIRST     sizeof(ee_data_addr_size_t) + sizeof(ee_data_size_t)
/**
 * @}
 */

/**
 * @defgroup EE_SECTOR_ADDR EE Sector Addr
 * @{
 */
#define SECT0_BASE_ADDR             (EEPROM_START_ADDR + 0UL)
#define SECT0_END_ADDR              (EEPROM_START_ADDR + (EEPROM_SECTOR_SIZE - 1UL))

#define SECT1_BASE_ADDR             (EEPROM_START_ADDR + EEPROM_SECTOR_SIZE)
#define SECT1_END_ADDR              (EEPROM_START_ADDR + (2UL * EEPROM_SECTOR_SIZE - 1UL))
/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup EEPROM_Global_Functions
 * @{
 */
int32_t EE_FlashInit(void);
int32_t EE_WriteNBytes(ee_addr_size_t virtAddress, ee_data_size_t *data, ee_addr_size_t length);
int32_t EE_ReadNBytes(ee_addr_size_t virtAddress, ee_data_size_t *data, ee_addr_size_t length);
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
#endif /* __HC32_FLASH_EMULATE_EEPROM_H */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
