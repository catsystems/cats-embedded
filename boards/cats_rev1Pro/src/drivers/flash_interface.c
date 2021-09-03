/**
 ******************************************************************************
 * @file    EEPROM_Emul/Porting/STM32L4/flash_interface.c
 * @author  MCD Application Team
 * @brief   This file provides all the EEPROM emulation flash interface functions.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drivers/eeprom_emul.h"
#include "drivers/flash_interface.h"

/** @addtogroup EEPROM_Emulation
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#if defined(FLASH_BANK_2)
static uint32_t GetBankNumber(uint32_t Address);
#endif

/* Exported functions --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup EEPROM_Private_Functions
 * @{
 */

/**
 * @brief  Write a double word at the given address in Flash
 * @param  Address Where to write
 * @param  Data What to write
 * @retval EE_Status
 *           - EE_OK: on success
 *           - EE_WRITE_ERROR: if an error occurs
 */
HAL_StatusTypeDef FI_WriteDoubleWord(uint32_t Address, uint64_t Data) {
  return HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, Data);
}

/**
 * @brief  Erase a page in polling mode
 * @param  Page Page number
 * @param  NbPages Number of pages to erase
 * @retval EE_Status
 *           - EE_OK: on success
 *           - EE error code: if an error occurs
 */
EE_Status FI_PageErase(uint32_t Page, uint16_t NbPages) {
  EE_Status status = EE_OK;
  FLASH_EraseInitTypeDef s_eraseinit;
  uint32_t bank = FLASH_BANK_1, page_error = 0U;

#if defined(FLASH_BANK_2)
  bank = GetBankNumber(PAGE_ADDRESS(Page));
#endif

  s_eraseinit.TypeErase = FLASH_TYPEERASE_PAGES;
  s_eraseinit.NbPages = NbPages;
  s_eraseinit.Page = Page;
  s_eraseinit.Banks = bank;

  /* Erase the Page: Set Page status to ERASED status */
  if (HAL_FLASHEx_Erase(&s_eraseinit, &page_error) != HAL_OK) {
    status = EE_ERASE_ERROR;
  }
  return status;
}

/**
 * @brief  Erase a page with interrupt enabled
 * @param  Page Page number
 * @param  NbPages Number of pages to erase
 * @retval EE_Status
 *           - EE_OK: on success
 *           - EE error code: if an error occurs
 */
EE_Status FI_PageErase_IT(uint32_t Page, uint16_t NbPages) {
  EE_Status status = EE_OK;
  FLASH_EraseInitTypeDef s_eraseinit;
  uint32_t bank = FLASH_BANK_1;

#if defined(FLASH_BANK_2)
  bank = GetBankNumber(PAGE_ADDRESS(Page));
#endif

  s_eraseinit.TypeErase = FLASH_TYPEERASE_PAGES;
  s_eraseinit.NbPages = NbPages;
  s_eraseinit.Page = Page;
  s_eraseinit.Banks = bank;

  /* Erase the Page: Set Page status to ERASED status */
  if (HAL_FLASHEx_Erase_IT(&s_eraseinit) != HAL_OK) {
    status = EE_ERASE_ERROR;
  }
  return status;
}

/**
 * @brief  Flush the caches if needed to keep coherency when the flash content is modified
 */
void FI_CacheFlush() {
  /* To keep its coherency, flush the D-Cache: its content is not updated after a flash erase. */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}

#if defined(FLASH_BANK_2)
/**
 * @brief  Gets the bank of a given address
 * @param  Address Address of the FLASH Memory
 * @retval Bank_Number The bank of a given address
 */
static uint32_t GetBankNumber(uint32_t Address) {
  uint32_t bank = 0U;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0U) {
    /* No Bank swap */
    if (Address < (FLASH_BASE + FLASH_BANK_SIZE)) {
      bank = FLASH_BANK_1;
    } else {
      bank = FLASH_BANK_2;
    }
  } else {
    /* Bank swap */
    if (Address < (FLASH_BASE + FLASH_BANK_SIZE)) {
      bank = FLASH_BANK_2;
    } else {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}
#endif

/**
 * @brief  Delete corrupted Flash address, can be called from NMI. No Timeout.
 * @param  Address Address of the FLASH Memory to delete
 * @retval EE_Status
 *           - EE_OK: on success
 *           - EE error code: if an error occurs
 */
EE_Status FI_DeleteCorruptedFlashAddress(uint32_t Address) {
  uint32_t dcachetoreactivate = 0U;
  EE_Status status = EE_OK;

  /* Deactivate the data cache if they are activated to avoid data misbehavior */
  if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != RESET) {
    /* Disable data cache  */
    __HAL_FLASH_DATA_CACHE_DISABLE();
    dcachetoreactivate = 1U;
  }

  /* Set FLASH Programmation bit */
  SET_BIT(FLASH->CR, FLASH_CR_PG);

  /* Program double word of value 0 */
  *(__IO uint32_t*)(Address) = (uint32_t)0U;
  *(__IO uint32_t*)(Address + 4U) = (uint32_t)0U;

  /* Wait programmation completion */
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
  }

  /* Check if error occured */
  if ((__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR)) || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PROGERR)) ||
      (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR)) || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR)) ||
      (__HAL_FLASH_GET_FLAG(FLASH_FLAG_SIZERR)) || (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR))) {
    status = EE_DELETE_ERROR;
  }

  /* Check FLASH End of Operation flag  */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)) {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  }

  /* Clear FLASH Programmation bit */
  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

  /* Flush the caches to be sure of the data consistency */
  if (dcachetoreactivate == 1U) {
    /* Reset data cache */
    __HAL_FLASH_DATA_CACHE_RESET();
    /* Enable data cache */
    __HAL_FLASH_DATA_CACHE_ENABLE();
  }

  /* Clear FLASH ECCD bit */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCD);

  return status;
}

/**
 * @brief  Check if the configuration is 128-bits bank or 2*64-bits bank
 * @param  None
 * @retval EE_Status
 *           - EE_OK: on success
 *           - EE error code: if an error occurs
 */
EE_Status FI_CheckBankConfig(void) {
#if defined(FLASH_OPTR_DBANK)
  FLASH_OBProgramInitTypeDef sOBCfg;
  EE_Status status;

  /* Request the Option Byte configuration :
     - User and RDP level are always returned
     - WRP and PCROP are not requested */
  sOBCfg.WRPArea = 0xFF;
  sOBCfg.PCROPConfig = 0xFF;
  HAL_FLASHEx_OBGetConfig(&sOBCfg);

  /* Check the value of the DBANK user option byte */
  if ((sOBCfg.USERConfig & OB_DBANK_64_BITS) != 0) {
    status = EE_OK;
  } else {
    status = EE_INVALID_BANK_CFG;
  }

  return status;
#else
  /* No feature 128-bits single bank, so always 64-bits dual bank */
  return EE_OK;
#endif
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
