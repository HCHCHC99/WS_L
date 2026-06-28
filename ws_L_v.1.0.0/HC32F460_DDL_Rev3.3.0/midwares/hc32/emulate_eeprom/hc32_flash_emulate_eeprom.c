/**
 *******************************************************************************
 * @file  hc32_flash_emulate_eeprom.c
 * @brief Main program of HC32 family for flash emulate eeprom.
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

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_flash_emulate_eeprom.h"

/**
 * @defgroup LL_EMULATE_EEPROM_LIB LL EMULATE EEPROM Lib
 * @{
 */

/**
 * @defgroup HC32_FLASH_EMULATE_EEPROM HC32 FLASH EMULATE EEPROM
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 * @defgroup EEPROM_Local_Types EEPROM Local Types
 * @{
 */

/**
 * @defgroup EEPROM_Address_Offset EEPROM Address Offset
 * @{
 */

/* used to store next addr which data is written in flash */
typedef struct {
    uint32_t u32WriteAddr;
    uint32_t u32ReadAddr;
} stc_eeprom_addr_offset_info_t;

/**
 * @}
 */

/**
 * @defgroup EEPROM_Operate_Status EEPROM Operate Status
 * @{
 */

typedef struct {
    int32_t i32EEStatus;
    ee_addr_size_t remainLen;
    ee_data_size_t *pData;
} stc_eeprom_operate_status_t;

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup EEPROM_Local_Macros EEPROM Local Macros
 * @{
 */

/* Used Flash sectors for EEPROM emulation */
#define SECT0                       (0x00U)
#define SECT1                       (0x01U)

/* Sector status definitions */
#define ERASED                      ((ee_data_addr_size_t)0xFFFFFFFFUL) /* Sector is empty */
#define RECEIVE_DATA                ((ee_data_addr_size_t)0xEEEEEEEEUL) /* Sector is marked to receive data */
#define VALID_SECTOR                ((ee_data_addr_size_t)0x00000000UL) /* Sector containing valid data */

/* Valid sectors in read and write defines */
#define READ_FROM_VALID_SECTOR      (0x00U)
#define WRITE_IN_VALID_SECTOR       (0x01U)

#define EE_ADDR_OFFSET              (sizeof(ee_data_addr_size_t) - sizeof(ee_data_size_t))

#define EE_UNLOCK()                 EFM_FWMC_Cmd(ENABLE)
#define EE_LOCK()                   EFM_FWMC_Cmd(DISABLE)

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static int32_t EE_Reset(void);
static void EE_UpdatePositionIndex(void);
static uint8_t EE_FindValidSector(uint8_t u8OperationType);
static uint8_t EE_VerifySectorFullyErased(uint32_t Address);
static int32_t EE_ReadDataformFlash(ee_addr_size_t VirtAddress, ee_data_size_t *Data);
static stc_eeprom_operate_status_t EE_CheckSectorFullWrite(ee_addr_size_t VirtAddress, ee_data_size_t *Data, ee_addr_size_t u16Len);
/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/**
 * @defgroup EEPROM_Local_Variables EEPROM Local Variables
 * @{
 */

static ee_data_size_t DataVar = 0;
static stc_eeprom_addr_offset_info_t stc_ee_addr_offset = {0};

/**
 * @}
 */
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @defgroup EEPROM_Local_Functions EEPROM Local Functions
 * @{
 */

static int32_t Flash_EraseSector(uint32_t u32Addr)
{
    int32_t  i32FlashStatus = EE_ERR_NOT_RDY;
    ee_data_size_t i;

    EE_UNLOCK();
    for (i = 0; i < EEPROM_SECTOR_SIZE / EFM_SECTOR_SIZE; i++) {
        i32FlashStatus = EFM_SectorErase(u32Addr + i * EFM_SECTOR_SIZE);
    }
    EE_LOCK();
    return i32FlashStatus;
}

static int32_t Flash_WriteOneData(uint32_t u32Addr, ee_data_addr_size_t data)
{
    int32_t i32FlashStatus;

    EE_UNLOCK();
    i32FlashStatus = EFM_Program(u32Addr, (const uint8_t *)&data, sizeof(ee_data_addr_size_t));
    EE_LOCK();
    return i32FlashStatus;
}

/**
  * @brief  Update EEPROM Write/Read Position Index.
  * @param  None
  * @retval None
  */
static void EE_UpdatePositionIndex(void)
{
    uint8_t u8ValidSector;
    uint32_t u32Address, u32SectorEndAddress;

    u8ValidSector = EE_FindValidSector(WRITE_IN_VALID_SECTOR);

    if (u8ValidSector == EE_ERR_SECT_INVALID) {
        return ;
    }
    /* Get the valid Sector start Address */
    u32Address = (uint32_t)(EEPROM_START_ADDR + (uint32_t)(u8ValidSector * EEPROM_SECTOR_SIZE));
    /* Get the valid Sector end Address */
    u32SectorEndAddress = (uint32_t)((EEPROM_START_ADDR - 1UL) + (uint32_t)((u8ValidSector + 1UL) * EEPROM_SECTOR_SIZE));

    while (u32Address < u32SectorEndAddress) {
        /* Find Available address in FLASH */
        if ((*(__IO ee_data_addr_size_t *)u32Address) == (ee_data_addr_size_t)0xFFFFFFFFUL) {
            break;
        } else {
            /* Next address */
            u32Address = u32Address + sizeof(ee_data_addr_size_t);
        }
    }

    stc_ee_addr_offset.u32WriteAddr = (uint32_t)((u32Address - (uint32_t)(EEPROM_START_ADDR + u8ValidSector * EEPROM_SECTOR_SIZE)));
    if (u8ValidSector == EE_FindValidSector(READ_FROM_VALID_SECTOR)) {
        stc_ee_addr_offset.u32ReadAddr = stc_ee_addr_offset.u32WriteAddr;
    }
}

/**
  * @brief  Verify if specified sector is fully erased.
  * @param  [in] u32Address     sector address
  *   This parameter can be one of the following values:
  *     @arg SECT0_BASE_ADDR    Sector0 base address
  *     @arg SECT1_BASE_ADDR    Sector1 base address
  * @retval uint8_t:
  *         - 0: Sector not erased
  *         - 1: Sector erased
  */
static uint8_t EE_VerifySectorFullyErased(uint32_t u32Address)
{
    uint8_t u8ReadStatus = 1U;
    ee_data_addr_size_t AddressValue;
    uint32_t EndSectorAddr = u32Address + EEPROM_SECTOR_SIZE - 1UL ;

    while (u32Address <= EndSectorAddr) {
        /* Get the current address value */
        AddressValue = (*(__IO ee_data_addr_size_t *)u32Address);

        /* Compare the value with ERASED */
        if (AddressValue != ERASED) {
            u8ReadStatus = 0U;
            break;
        }
        /* Next address */
        u32Address = u32Address + sizeof(ee_data_addr_size_t);
    }

    return u8ReadStatus;
}

/**
  * @brief  Read specified virtual address data from FLASH
  * @param  [in] VirtAddress            Virtual eeprom address
  * @param  [out] Data                  Pointer of receive data
  * @retval int32_t:
  *         - EE_OK: valid virtual EEPROM address was found
  *         - EE_ERR: valid virtual EEPROM address was not found
  *         - EE_ERR_SECT_INVALID: No valid sector was found
  */
static int32_t EE_ReadDataformFlash(ee_addr_size_t VirtAddress, ee_data_size_t *Data)
{
    uint8_t u8ValidSector;
    ee_addr_size_t AddressValue;
    int32_t i32ReadStatus = EE_ERR;
    uint32_t u32Address, u32SectorStartAddress;

    /* Get active Sector for read operation */
    u8ValidSector = EE_FindValidSector(READ_FROM_VALID_SECTOR);
    if (u8ValidSector == EE_ERR_SECT_INVALID) {
        return (int32_t)EE_ERR_SECT_INVALID;
    }

    /* Get the valid Sector start Address */
    u32SectorStartAddress = (uint32_t)(EEPROM_START_ADDR + (uint32_t)(u8ValidSector * EEPROM_SECTOR_SIZE));
    /* Get the valid Sector read Address */
    if (stc_ee_addr_offset.u32ReadAddr == 0UL) {
        u32Address = (uint32_t)((EEPROM_START_ADDR - EE_ADDR_OFFSET) + (uint32_t)((1UL + u8ValidSector) * EEPROM_SECTOR_SIZE));
    } else {
        u32Address = (uint32_t)((EEPROM_START_ADDR + stc_ee_addr_offset.u32ReadAddr - EE_ADDR_OFFSET) + (uint32_t)(u8ValidSector * EEPROM_SECTOR_SIZE));
    }

    while (u32Address > (u32SectorStartAddress + EE_ADDR_OFFSET)) {
        /* Get the current address value */
        AddressValue = (*(__IO ee_addr_size_t *)u32Address);

        /* Compare the value with the virtual address */
        if (AddressValue == VirtAddress) {
            *Data = (*(__IO ee_data_size_t *)(u32Address - sizeof(ee_data_size_t)));
            i32ReadStatus = EE_OK;
            break;
        } else {
            /* Next address */
            u32Address = u32Address - sizeof(ee_data_addr_size_t);
        }
    }

    return i32ReadStatus;
}

/**
  * @brief  Erases SECTOR0 and SECTOR1 and writes VALID_SECTOR header to SECTOR0
  * @param  None
  * @retval int32_t:
  *         - EE_OK: Program successful.
  *         - EE_ERR_NOT_RDY: EFM if not ready
  */
static int32_t EE_Reset(void)
{
    int32_t i32FlashStatus;
    /* Erase Sector0 */
    if (0U == EE_VerifySectorFullyErased(SECT0_BASE_ADDR)) {
        i32FlashStatus = Flash_EraseSector(SECT0_BASE_ADDR);
        if (i32FlashStatus != EE_OK) {
            return i32FlashStatus;
        }
    }
    /* Set Sector0 as valid sector */
    i32FlashStatus = Flash_WriteOneData(SECT0_BASE_ADDR, VALID_SECTOR);

    if (i32FlashStatus != EE_OK) {
        return i32FlashStatus;
    }

    /* Erase Sector1 */
    if (0U == EE_VerifySectorFullyErased(SECT1_BASE_ADDR)) {
        i32FlashStatus = Flash_EraseSector(SECT1_BASE_ADDR);
        if (i32FlashStatus != EE_OK) {
            return i32FlashStatus;
        }
    }

    return EE_OK;
}

/**
  * @brief  Find valid Sector for write or read operation
  * @param  [in] u8OperationType            operation condition.
  *   This parameter can be one of the following values:
  *     @arg READ_FROM_VALID_SECTOR         read operation
  *     @arg WRITE_IN_VALID_SECTOR          write operation
  * @retval uint8_t:
  *         - SECT0: Sector 0 valid
  *         - SECT1: Sector 1 valid
  */
static uint8_t EE_FindValidSector(uint8_t u8OperationType)
{
    ee_data_addr_size_t SectorStatus0, SectorStatus1;

    SectorStatus0 = (*(__IO ee_data_addr_size_t *)SECT0_BASE_ADDR);
    SectorStatus1 = (*(__IO ee_data_addr_size_t *)SECT1_BASE_ADDR);

    /**
    * write operation: sector which status is RECEIVE_DATA is higher than
    * status is VALID_SECTOR
    * read operation: depend on sector status is VALID_SECTOR
    */
    switch (u8OperationType) {
        case WRITE_IN_VALID_SECTOR:
            if (SectorStatus1 == VALID_SECTOR) {
                if (SectorStatus0 == RECEIVE_DATA) {
                    return SECT0;
                } else {
                    return SECT1;
                }
            } else if (SectorStatus0 == VALID_SECTOR) {
                if (SectorStatus1 == RECEIVE_DATA) {
                    return SECT1;
                } else {
                    return SECT0;
                }
            } else {
                return EE_ERR_SECT_INVALID;
            }
        case READ_FROM_VALID_SECTOR:
            if (SectorStatus0 == VALID_SECTOR) {
                return SECT0;
            } else if (SectorStatus1 == VALID_SECTOR) {
                return SECT1;
            } else {
                return EE_ERR_SECT_INVALID;
            }
        default:
            return SECT0;
    }
}

/**
  * @brief  Check if current active sector is full and Write virtual eeprom address and data in FLASH
  * @param  [in] VirtAddress        virtual eeprom address
  * @param  [in] Data               pointer of data to be written
  * @param  [in] length             length of data
  * @retval stc_eeprom_operate_status_t
  */
static stc_eeprom_operate_status_t EE_CheckSectorFullWrite(ee_addr_size_t VirtAddress, ee_data_size_t *Data, ee_addr_size_t length)
{
    stc_eeprom_operate_status_t stc_ee_operate_status;
    uint8_t u8ValidSector;
    uint32_t u32Address, u32SectorEndAddress;

    stc_ee_operate_status.remainLen = length;
    stc_ee_operate_status.pData = Data;

    u8ValidSector = EE_FindValidSector(WRITE_IN_VALID_SECTOR);

    if (u8ValidSector == EE_ERR_SECT_INVALID) {
        stc_ee_operate_status.i32EEStatus = (int32_t)EE_ERR_SECT_INVALID;
        return stc_ee_operate_status;
    }
    /* Get the valid Sector start Address */
    u32Address = (uint32_t)(EEPROM_START_ADDR + (uint32_t)(u8ValidSector * EEPROM_SECTOR_SIZE)) + stc_ee_addr_offset.u32WriteAddr;
    /* Get the valid Sector end Address */
    u32SectorEndAddress = (uint32_t)((EEPROM_START_ADDR - 1UL) + (uint32_t)((u8ValidSector + 1UL) * EEPROM_SECTOR_SIZE));

    while (u32Address < u32SectorEndAddress) {
        /* Find Available address in FLASH */
        if ((*(__IO ee_data_addr_size_t *)u32Address) == (ee_data_addr_size_t)0xFFFFFFFFUL) {
            stc_ee_operate_status.i32EEStatus = Flash_WriteOneData(u32Address,
                                                                   (((ee_data_addr_size_t)VirtAddress << 8U * sizeof(ee_data_size_t)) | *stc_ee_operate_status.pData));
            if (stc_ee_operate_status.i32EEStatus != EE_OK) {
                return stc_ee_operate_status;
            }

            stc_ee_addr_offset.u32WriteAddr = (u32Address + sizeof(ee_data_addr_size_t) - (uint32_t)(EEPROM_START_ADDR + u8ValidSector * EEPROM_SECTOR_SIZE));
            if (EE_FindValidSector(READ_FROM_VALID_SECTOR) == u8ValidSector) {
                stc_ee_addr_offset.u32ReadAddr = stc_ee_addr_offset.u32WriteAddr;
            }

            if (--stc_ee_operate_status.remainLen == 0U) {
                /* Write done */
                return stc_ee_operate_status;
            }
            VirtAddress ++;
            stc_ee_operate_status.pData ++;
        } else {
            /* Next address */
            u32Address = u32Address + sizeof(ee_data_addr_size_t);
        }
    }
    /* Valid sector is full */
    stc_ee_operate_status.i32EEStatus = EE_ERR_SECT_FULL;

    return stc_ee_operate_status;
}

/**
  * @brief  Swap sector and transfer valid data to new active sector
  * @param  [in] VirtAddress        virtual eeprom address
  * @param  [in] Data               pointer of data to be written
  * @param  [in] length             length of data
  * @retval int32_t:
  *         - EE_OK: Program successful.
  *         - EE_ERR_NOT_RDY: EFM if not ready
  *         - EE_ERR_SECT_FULL: Valid sector is full
  *         - EE_ERR_SECT_INVALID: No valid sector was found
  */
static int32_t EE_SectorSwapWrite(ee_addr_size_t VirtAddress, ee_data_size_t *Data, ee_addr_size_t length)
{
    stc_eeprom_operate_status_t stc_ee_operate_status;
    uint8_t u8ValidSector;
    ee_addr_size_t VarIdx;
    int32_t i32FlashStatus;
    uint32_t u32NewSectorAddress;
    uint32_t u32OldSectorAddress;
    int32_t i32ReadStatus;

    u8ValidSector = EE_FindValidSector(READ_FROM_VALID_SECTOR);
    if (u8ValidSector == SECT1) {
        u32NewSectorAddress = SECT0_BASE_ADDR;
        u32OldSectorAddress = SECT1_BASE_ADDR;
    } else if (u8ValidSector == SECT0) {
        u32NewSectorAddress = SECT1_BASE_ADDR;
        u32OldSectorAddress = SECT0_BASE_ADDR;
    } else {
        return (int32_t)EE_ERR_SECT_INVALID;
    }

    /* Set the new Sector status to RECEIVE_DATA status */
    i32FlashStatus = Flash_WriteOneData(u32NewSectorAddress, RECEIVE_DATA);
    if (i32FlashStatus != EE_OK) {
        return i32FlashStatus;
    }

    stc_ee_addr_offset.u32WriteAddr = 0U;
    /* Write the variable passed as parameter in the new active sector */
    stc_ee_operate_status = EE_CheckSectorFullWrite(VirtAddress, Data, length);
    if (stc_ee_operate_status.i32EEStatus != EE_OK) {
        return stc_ee_operate_status.i32EEStatus;
    }

    /* Transfer data from old to the new active sector */
    for (VarIdx = 0; VarIdx < EE_CAPACITY; VarIdx++) {
        if (VarIdx != VirtAddress) {
            /* Transfer other valid virtual eeprom address and data except VirtAddress to new active sector */
            i32ReadStatus = EE_ReadDataformFlash(VarIdx, &DataVar);
            /* Valid virtual address was found */
            if (EE_OK == i32ReadStatus) {
                stc_ee_operate_status = EE_CheckSectorFullWrite(VarIdx, &DataVar, 1U);
                if (stc_ee_operate_status.i32EEStatus != EE_OK) {
                    return stc_ee_operate_status.i32EEStatus;
                }
            }
        } else {
            VarIdx += length - 1U;
        }
    }

    /* Erase the old Sector: Set old Sector status to ERASED status */
    i32FlashStatus = Flash_EraseSector(u32OldSectorAddress);
    if (i32FlashStatus != EE_OK) {
        return i32FlashStatus;
    }

    /* Set new Sector status to VALID_SECTOR status */
    i32FlashStatus = Flash_WriteOneData(u32NewSectorAddress, VALID_SECTOR);
    if (i32FlashStatus != EE_OK) {
        return i32FlashStatus;
    }

    /* Update read index loction */
    stc_ee_addr_offset.u32ReadAddr = stc_ee_addr_offset.u32WriteAddr;

    return i32FlashStatus;
}

/**
 * @}
 */

/**
 * @defgroup EEPROM_Global_Functions EEPROM Global Functions
 * @{
 */

/**
  * @brief  Restore the sectors to a expectations state after power loss
  * @param  None
  * @retval int32_t:
  *         - EE_OK: Init successful.
  *         - EE_ERR_NOT_RDY: EFM if not ready
  *         - EE_ERR_SECT_FULL: Valid sector is full
  *         - EE_ERR_SECT_INVALID: No valid sector was found
  */
int32_t EE_FlashInit(void)
{
    ee_data_addr_size_t SectorStatus0, SectorStatus1;
    ee_addr_size_t VarIdx;
    uint32_t u32Index = 0xFFFFFFFFUL;
    int32_t  i32FlashStatus;
    stc_eeprom_operate_status_t stc_ee_operate_status;

    /* Get Sector0 status */
    SectorStatus0 = (*(__IO ee_data_addr_size_t *)SECT0_BASE_ADDR);
    /* Get Sector1 status */
    SectorStatus1 = (*(__IO ee_data_addr_size_t *)SECT1_BASE_ADDR);

    /**
     * The possible states of the sectors are as below:
     * ---------------------------------------------------------------------|
     *              |                   sector0                             |
     *              |-------------------------------------------------------|
     *  sector1     |   erase       |       receive     |       valid       |
     * -------------|---------------|-------------------|-------------------|
     *  erase       |   invalid     |   erase 1 &       |   erase 1 &       |
     *              |   EE_Reset    |   mark 0 valid    |   use 0 as valid  |
     * -------------|---------------|-------------------|-------------------|
     *              |               |                   |   transfer data   |
     *              |  erase 0      |   invalid         |   form 0 to 1     |
     *   receive    |  mark 1 valid |   EE_Reset        |   mark 1 valid    |
     *              |               |                   |   erase 0         |
     * -------------|---------------|-------------------|-------------------|
     *              |               |   transfer data   |                   |
     *  valid       |   erase0      |   form 1 to 0     |   invalid         |
     *              | use 1 as valid|   mark 0 valid    |   EE_Reset        |
     *              |               |   erase 1         |                   |
     * ---------------------------------------------------------------------|
    */
    switch (SectorStatus0) {
        case ERASED:
            if (SectorStatus1 == VALID_SECTOR) {
                /* Erase Sector0 */
                if (0U == EE_VerifySectorFullyErased(SECT0_BASE_ADDR)) {
                    i32FlashStatus = Flash_EraseSector(SECT0_BASE_ADDR);
                    if (i32FlashStatus != EE_OK) {
                        return i32FlashStatus;
                    }
                }
            } else if (SectorStatus1 == RECEIVE_DATA) {
                /* Erase Sector0 */
                if (0U == EE_VerifySectorFullyErased(SECT0_BASE_ADDR)) {
                    i32FlashStatus = Flash_EraseSector(SECT0_BASE_ADDR);
                    if (i32FlashStatus != EE_OK) {
                        return i32FlashStatus;
                    }
                }
                /* Mark Sector1 as valid */
                i32FlashStatus = Flash_WriteOneData(SECT1_BASE_ADDR, VALID_SECTOR);
                if (i32FlashStatus != EE_OK) {
                    return i32FlashStatus;
                }
            } else {
                /* Erase both Sector0 and Sector1 and set Sector0 as valid sector */
                i32FlashStatus = EE_Reset();
                if (i32FlashStatus != EE_OK) {
                    return i32FlashStatus;
                }
            }
            break;

        case RECEIVE_DATA:
            if (SectorStatus1 == VALID_SECTOR) {
                /* Transfer data from Sector1 to Sector0 */
                for (VarIdx = 0; VarIdx < EE_CAPACITY; VarIdx++) {
                    if ((*(__IO ee_addr_size_t *)(SECT0_BASE_ADDR + EE_SEC_VALID_ADDR_FIRST)) == VarIdx) {
                        u32Index = VarIdx;
                    }
                    if (VarIdx != u32Index) {
                        /* Transfer valid virtual eeprom address and data to sector0 */
                        if (EE_OK == EE_ReadDataformFlash(VarIdx, &DataVar)) {
                            /* Valid virtual address was found */
                            stc_ee_operate_status = EE_CheckSectorFullWrite(VarIdx, &DataVar, 1U);
                            if (stc_ee_operate_status.i32EEStatus != EE_OK) {
                                return stc_ee_operate_status.i32EEStatus;
                            }
                        }
                    }
                }
                /* Erase Sector1 */
                if (0U == EE_VerifySectorFullyErased(SECT1_BASE_ADDR)) {
                    i32FlashStatus = Flash_EraseSector(SECT1_BASE_ADDR);
                    if (i32FlashStatus != EE_OK) {
                        return i32FlashStatus;
                    }
                }
                /* Mark Sector0 as valid */
                i32FlashStatus = Flash_WriteOneData(SECT0_BASE_ADDR, VALID_SECTOR);
                if (i32FlashStatus != EE_OK) {
                    return i32FlashStatus;
                }
            } else if (SectorStatus1 == ERASED) {
                /* Erase Sector1 */
                if (0U == EE_VerifySectorFullyErased(SECT1_BASE_ADDR)) {
                    i32FlashStatus = Flash_EraseSector(SECT1_BASE_ADDR);
                    if (i32FlashStatus != EE_OK) {
                        return i32FlashStatus;
                    }
                }
                /* Mark Sector0 as valid */
                i32FlashStatus = Flash_WriteOneData(SECT0_BASE_ADDR, VALID_SECTOR);
                if (i32FlashStatus != EE_OK) {
                    return i32FlashStatus;
                }
            } else {
                i32FlashStatus = EE_Reset();
                if (i32FlashStatus != EE_OK) {
                    return i32FlashStatus;
                }
            }
            break;
        case VALID_SECTOR:
            if (SectorStatus1 == ERASED) {
                if (0U == EE_VerifySectorFullyErased(SECT1_BASE_ADDR)) {
                    i32FlashStatus = Flash_EraseSector(SECT1_BASE_ADDR);
                    if (i32FlashStatus != EE_OK) {
                        return i32FlashStatus;
                    }
                }
            } else if (SectorStatus1 == RECEIVE_DATA) {
                for (VarIdx = 0; VarIdx < EE_CAPACITY; VarIdx++) {
                    if ((*(__IO ee_addr_size_t *)(SECT1_BASE_ADDR + EE_SEC_VALID_ADDR_FIRST)) == VarIdx) {
                        u32Index = VarIdx;
                    }
                    if (VarIdx != u32Index) {
                        if (EE_OK == EE_ReadDataformFlash(VarIdx, &DataVar)) {
                            stc_ee_operate_status = EE_CheckSectorFullWrite(VarIdx, &DataVar, 1U);
                            if (stc_ee_operate_status.i32EEStatus != EE_OK) {
                                return stc_ee_operate_status.i32EEStatus;
                            }
                        }
                    }
                }
                if (0U == EE_VerifySectorFullyErased(SECT0_BASE_ADDR)) {
                    i32FlashStatus = Flash_EraseSector(SECT0_BASE_ADDR);
                    if (i32FlashStatus != EE_OK) {
                        return i32FlashStatus;
                    }
                }
                i32FlashStatus = Flash_WriteOneData(SECT1_BASE_ADDR, VALID_SECTOR);
                if (i32FlashStatus != EE_OK) {
                    return i32FlashStatus;
                }
            } else {
                i32FlashStatus = EE_Reset();
                if (i32FlashStatus != EE_OK) {
                    return i32FlashStatus;
                }
            }
            break;

        default:
            i32FlashStatus = EE_Reset();
            if (i32FlashStatus != EE_OK) {
                return i32FlashStatus;
            }
            break;
    }

    EE_UpdatePositionIndex();

    return EE_OK;
}

/**
  * @brief  Write specified length data to FLASH
  * @param  [in] virtAddress        virtual eeprom address
  * @param  [in] data               pointer of data to be written
  * @param  [in] length             length of data
  * @retval int32_t:
  *         - EE_OK: Program successful.
  *         - EE_ERR: Parameters eeror
  *         - EE_ERR_NOT_RDY: EFM if not ready
  *         - EE_ERR_SECT_FULL: Valid sector is full
  *         - EE_ERR_SECT_INVALID: No valid sector was found
  */
int32_t EE_WriteNBytes(ee_addr_size_t virtAddress, ee_data_size_t *data, ee_addr_size_t length)
{
    stc_eeprom_operate_status_t stc_ee_operate_status;

    if (virtAddress + length > EE_CAPACITY) {
        return EE_ERR_INVD_PARAM;
    }

    if (0U == length) {
        return EE_ERR_INVD_PARAM;
    }

    stc_ee_operate_status = EE_CheckSectorFullWrite(virtAddress, data, length);

    if (stc_ee_operate_status.i32EEStatus == EE_ERR_SECT_FULL) {
        stc_ee_operate_status.i32EEStatus = \
                                            EE_SectorSwapWrite(
                                                virtAddress + (length - stc_ee_operate_status.remainLen),
                                                stc_ee_operate_status.pData, stc_ee_operate_status.remainLen);
    }

    return stc_ee_operate_status.i32EEStatus;
}

/**
  * @brief  Read specified length data from FLASH
  * @param  [in] virtAddress        virtual eeprom address
  * @param  [out] data              pointer of data to be store
  * @param  [in] length             length of data
  * @retval int32_t:
  *         - EE_OK: Read successful.
  *         - EE_ERR: Data not be found.
  *         - EE_ERR_INVD_PARAM: Parameters error.
  *         - EE_ERR_SECT_INVALID: No valid sector was found.
  */
int32_t EE_ReadNBytes(ee_addr_size_t virtAddress, ee_data_size_t *data, ee_addr_size_t length)
{
    ee_addr_size_t index;
    int32_t ret = EE_OK;

    if (virtAddress + length > EE_CAPACITY) {
        return EE_ERR_INVD_PARAM;
    }

    if (0U == length) {
        return EE_ERR_INVD_PARAM;
    }

    for (index = 0U; index < length; index++) {
        ret = EE_ReadDataformFlash(virtAddress + index, data++);
        if (EE_OK != ret) {
            return ret;
        }
    }

    return ret;
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

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
