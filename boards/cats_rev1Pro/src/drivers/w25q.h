/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Driver for Winbond W25Q chips.
 *
 * Inspired by:
 *   - https://github.com/nimaltd/w25qxx
 *   - https://www.fatalerrors.org/a/stm32h7-peripheral-configuration-quick-reference-qspi-part.html
 */

#pragma once

#include <stdbool.h>
#include "target.h"

typedef enum {
  W25QINVALID = 0,
  W25Q10 = 1,
  W25Q20,
  W25Q40,
  W25Q80,
  W25Q16,
  W25Q32,
  W25Q64,
  W25Q128,
  W25Q256,
  W25Q512,
} w25q_id_e;

typedef struct {
  w25q_id_e id;
  uint16_t page_size;
  uint32_t page_count;
  uint32_t sector_size;
  uint32_t sector_count;
  uint32_t block_size;
  uint32_t block_count;
  uint32_t capacity_in_kilobytes;
  uint8_t lock;
} w25q_t;

typedef enum {
  W25Q_OK = 0,
  W25Q_ERR_INIT,
  W25Q_ERR_WRITE_ENABLE,
  W25Q_ERR_AUTOPOLLING,
  W25Q_ERR_ERASE,
  W25Q_ERR_TRANSMIT,
  W25Q_ERR_TRANSMIT_CMD,
  W25Q_ERR_INVALID_PARAM
} w25q_status_e;

/**
 * Initializes the W25Q flash chip.
 *
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_init(void);

/**
 * Sends the reset command.
 *
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_reset(void);

/**
 * Reads the JEDEC ID of the flash chip.
 *
 * @param device_id - location in which to store the device ID
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_read_id(uint32_t *device_id);

/**
 * Erases a given 4K sector.
 *
 * @param sector_idx - Index of the sector to be erased
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_sector_erase(uint32_t sector_idx);

/**
 * Erases a given 32K block.
 *
 * @param block_idx - Address of the 32K block to be deleted
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_block_erase_32k(uint32_t block_idx);

/**
 * Erases a given 64K block.
 *
 * @param block_idx - Address of the 64K block to be deleted
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_block_erase_64k(uint32_t block_idx);

/**
 * Erase the entire chip.
 *
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_chip_erase(void);

/**
 * Write up to one page to the flash.
 *
 * @param buf - Data to be written
 * @param write_addr - Location on the flash chip in which to write the data
 * @param num_bytes_to_write - Amount of data to write, up to 256
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_write_page(uint8_t *buf, uint32_t write_addr, uint16_t num_bytes_to_write);

/**
 * Write up to flash capacity.
 *
 * @param buf - Data to be written
 * @param write_addr - Location on the flash chip in which to write the data
 * @param num_bytes_to_write - Amount of data to write
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_write_buffer(uint8_t *buf, uint32_t write_addr, uint32_t num_bytes_to_write);

/**
 * Read the
 *
 * @param buf - Buffer in which to store the read data
 * @param write_addr - Location on the flash chip from which to read the data
 * @param num_bytes_to_write - Amount of data to read
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_read_buffer(uint8_t *buf, uint32_t read_addr, uint32_t num_bytes_to_read);

/**
 * Read status registers from the flash.
 *
 * @param status_reg_num[in] - status register number: 1, 2 or 3
 * @param status_reg_val[out] - value of the status register
 * @return W25Q_OK if successful, W25Q_ERR_* otherwise
 */
w25q_status_e w25q_read_status_reg(uint8_t status_reg_num, uint8_t *status_reg_val);

uint32_t w25q_sector_to_page(uint32_t sector_num);

uint32_t w25q_block_to_page(uint32_t block_num);

/**
 * Check whether a given sector is empty.
 *
 * @param sector_idx - Index of the sector
 * @return true if the sector is empty, false otherwise
 */
bool w25q_is_sector_empty(uint32_t sector_idx);

w25q_status_e w25q_read_sector(uint8_t *buf, uint32_t sector_num, uint32_t offset_in_bytes,
                               uint32_t bytes_to_read_up_to_sector_size);

w25q_status_e w25q_write_sector(uint8_t *buf, uint32_t sector_num, uint32_t offset_in_bytes,
                                uint32_t bytes_to_write_up_to_sector_size);

w25q_status_e w25qxx_write_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                       uint32_t bytes_to_write_up_to_page_size);

w25q_status_e w25qxx_read_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                      uint32_t NumByteToRead_up_to_PageSize);

extern w25q_t w25q;
