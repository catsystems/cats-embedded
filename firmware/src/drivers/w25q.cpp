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

#include "drivers/w25q.h"
#include "target.h"
#include "util/log.h"
#include "util/task_util.h"

w25q_t w25q = {.id = W25QINVALID};

/* Settings */
#define W25Q_CMD_ENABLE_RESET           0x66
#define W25Q_CMD_RESET_DEVICE           0x99
#define W25Q_CMD_JEDEC_ID               0x9F
#define W25Q_CMD_WRITE_ENABLE           0x06
#define W25Q_CMD_ENTER_4_BYTE_ADDR_MODE 0xB7

/* Status registers */
#define W25Q_CMD_READ_STATUS_REG1  0x05
#define W25Q_CMD_READ_STATUS_REG2  0x35
#define W25Q_CMD_READ_STATUS_REG3  0x15
#define W25Q_CMD_WRITE_STATUS_REG1 0x01
#define W25Q_CMD_WRITE_STATUS_REG2 0x31
#define W25Q_CMD_WRITE_STATUS_REG3 0x11

/* Erasure commands */
#define W25Q_CMD_SECTOR_ERASE_4_BYTE_ADDR    0x21
#define W25Q_CMD_SECTOR_ERASE_3_BYTE_ADDR    0x20
#define W25Q_CMD_BLOCK_ERASE_32K             0x52
#define W25Q_CMD_BLOCK_ERASE_64K_4_BYTE_ADDR 0xDC
#define W25Q_CMD_BLOCK_ERASE_64K_3_BYTE_ADDR 0xD8
#define W25Q_CMD_CHIP_ERASE                  0xC7

#define W25Q_CMD_FAST_READ_3_BYTE_ADDR    0x0B
#define W25Q_CMD_FAST_READ_4_BYTE_ADDR    0x0C
#define W25Q_CMD_PAGE_PROGRAM_3_BYTE_ADDR 0x02
#define W25Q_CMD_PAGE_PROGRAM_4_BYTE_ADDR 0x12

/* Status register 1 flash busy bit */
#define W25Q_STATUS_REG1_BUSY 0x01
/* Status register 1 write enabled bit */
#define W25Q_STATUS_REG1_WEL 0x02

static inline void w25qxx_spi_transmit(uint8_t data) { HAL_SPI_Transmit(&FLASH_SPI_HANDLE, &data, 1, 100); }

static inline uint8_t w25qxx_spi_receive() {
  uint8_t ret = 0;
  HAL_SPI_Receive(&FLASH_SPI_HANDLE, &ret, 1, 100);
  return ret;
}

static inline void w25qxx_wait_for_write_end(void) {
  // sysDelay(1);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_READ_STATUS_REG1);
  uint8_t status_reg_val = 0x00;
  do {
    status_reg_val = w25qxx_spi_receive();
    sysDelay(1);
  } while ((status_reg_val & W25Q_STATUS_REG1_BUSY) == 1);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static inline void w25q_send_4_byte_addr(uint32_t address) {
  uint8_t buf[4];
  uint8_t *addr_ptr = (uint8_t *)&address;
  //  buf[0] = (address & 0xFF000000) >> 24;
  //  buf[1] = (address & 0xFF0000) >> 16;
  //  buf[2] = (address & 0xFF00) >> 8;
  //  buf[3] = address & 0xFF;
  buf[0] = addr_ptr[3];
  buf[1] = addr_ptr[2];
  buf[2] = addr_ptr[1];
  buf[3] = addr_ptr[0];
  HAL_SPI_Transmit(&FLASH_SPI_HANDLE, buf, sizeof(buf), 100);
}

static inline void w25q_send_3_byte_addr(uint32_t address) {
  uint8_t buf[3];
  buf[0] = (address & 0xFF0000) >> 16;
  buf[1] = (address & 0xFF00) >> 8;
  buf[2] = address & 0xFF;
  //  uint8_t *addr_ptr = (uint8_t *)&address;
  //  buf[0] = addr_ptr[3];
  //  buf[1] = addr_ptr[2];
  //  buf[2] = addr_ptr[1];
  HAL_SPI_Transmit(&FLASH_SPI_HANDLE, buf, sizeof(buf), 100);
}

// Write enable
int8_t w25q_write_enable(void) {
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_WRITE_ENABLE);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  // sysDelay(1);
  return W25Q_OK;
}

static inline void w25qxx_write_status_register(uint8_t status_register, uint8_t data) {
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  if (status_register == 1) {
    w25qxx_spi_transmit(W25Q_CMD_WRITE_STATUS_REG1);
  } else if (status_register == 2) {
    w25qxx_spi_transmit(W25Q_CMD_WRITE_STATUS_REG2);
  } else {
    w25qxx_spi_transmit(W25Q_CMD_WRITE_STATUS_REG3);
  }
  w25qxx_spi_transmit(data);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

w25q_status_e w25q_init(void) {
  w25q.lock = 1;
  // while (osKernelGetTickCount() < 100) sysDelay(1);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  sysDelay(100);

  uint32_t device_id = 0;
  if (w25q_read_id(&device_id) != W25Q_OK) {
    return W25Q_ERR_INIT;
  }

  log_debug("W25Q ID:0x%lX", device_id);

  switch (device_id & 0x0000FFFF) {
    case 0x4020:
      w25q.id = W25Q512;
      w25q.block_count = 1024;
      log_debug("W25Q Chip: W25Q512");
      break;
    case 0x4019:
      w25q.id = W25Q256;
      w25q.block_count = 512;
      log_debug("W25Q Chip: W25Q256");
      break;
    case 0x4018:
      w25q.id = W25Q128;
      w25q.block_count = 256;
      log_debug("W25Q Chip: W25Q128");
      break;
    case 0x4017:
      w25q.id = W25Q64;
      w25q.block_count = 128;
      log_debug("W25Q Chip: W25Q64");
      break;
    case 0x4016:
      w25q.id = W25Q32;
      w25q.block_count = 64;
      log_debug("W25Q Chip: W25Q32");
      break;
    case 0x4015:
      w25q.id = W25Q16;
      w25q.block_count = 32;
      log_debug("W25Q Chip: W25Q16");
      break;
    case 0x4014:
      w25q.id = W25Q80;
      w25q.block_count = 16;
      log_debug("W25Q Chip: W25Q80");
      break;
    case 0x4013:
      w25q.id = W25Q40;
      w25q.block_count = 8;
      log_debug("W25Q Chip: W25Q40");
      break;
    case 0x4012:
      w25q.id = W25Q20;
      w25q.block_count = 4;
      log_debug("W25Q Chip: W25Q20");
      break;
    case 0x4011:
      w25q.id = W25Q10;
      w25q.block_count = 2;
      log_debug("W25Q Chip: W25Q10");
      break;
    default:
      log_debug("W25Q Unknown ID");
      return W25Q_ERR_INIT;
  }
  w25q.sector_count = w25q.block_count * 16;
  w25q.page_count = (w25q.sector_count * w25q.sector_size) / w25q.page_size;
  w25q.block_size = w25q.sector_size * 16;
  w25q.capacity_in_kilobytes = (w25q.sector_count * w25q.sector_size) / 1024;
  w25q.needs_4_byte_addressing = w25q.id >= W25Q256;

  /* If the flash is >= 256MBits, enter 4-byte addressing mode */
  if (w25q.needs_4_byte_addressing) {
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    w25qxx_spi_transmit(W25Q_CMD_ENTER_4_BYTE_ADDR_MODE);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  }

  uint8_t status1 = 0;
  uint8_t status2 = 0;
  uint8_t status3 = 0;

  w25q_read_status_reg(1, &status1);
  w25q_read_status_reg(2, &status2);
  w25q_read_status_reg(3, &status3);

  log_debug("Flash statuses: %x %x %x", status1, status2, status3);

  log_debug("w25qxx Page Size: %hu B", w25q.page_size);
  log_debug("w25qxx Page Count: %lu", w25q.page_count);
  log_debug("w25qxx Sector Size: %lu B", w25q.sector_size);
  log_debug("w25qxx Sector Count: %lu", w25q.sector_count);
  log_debug("w25qxx Block Size: %lu B", w25q.block_size);
  log_debug("w25qxx Block Count: %lu", w25q.block_count);
  log_debug("w25qxx Capacity: %lu KB", w25q.capacity_in_kilobytes);
  log_debug("w25qxx Init Done");

  w25q.lock = 0;
  w25q.initialized = true;
  return W25Q_OK;
}

w25q_status_e w25q_read_id(uint32_t *w25q_id) {
  uint32_t temp0 = 0, temp1 = 0, temp2 = 0;
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_JEDEC_ID);
  temp0 = w25qxx_spi_receive();
  temp1 = w25qxx_spi_receive();
  temp2 = w25qxx_spi_receive();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  *w25q_id = (temp0 << 16) | (temp1 << 8) | temp2;

  return W25Q_OK;
}

w25q_status_e w25q_read_status_reg(uint8_t status_reg_num, uint8_t *status_reg_val) {
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  if (status_reg_num == 1) {
    w25qxx_spi_transmit(W25Q_CMD_READ_STATUS_REG1);
    *status_reg_val = w25qxx_spi_receive();
  } else if (status_reg_num == 2) {
    w25qxx_spi_transmit(W25Q_CMD_READ_STATUS_REG2);
    *status_reg_val = w25qxx_spi_receive();
  } else {
    w25qxx_spi_transmit(W25Q_CMD_READ_STATUS_REG3);
    *status_reg_val = w25qxx_spi_receive();
  }
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  return W25Q_OK;
}

w25q_status_e w25q_sector_erase(uint32_t sector_idx) {
  while (w25q.lock == 1) sysDelay(1);
  w25q.lock = 1;

  w25qxx_wait_for_write_end();
  sector_idx = sector_idx * w25q.sector_size;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  if (w25q.needs_4_byte_addressing) {
    w25qxx_spi_transmit(W25Q_CMD_SECTOR_ERASE_4_BYTE_ADDR);
    w25q_send_4_byte_addr(sector_idx);
  } else {
    w25qxx_spi_transmit(W25Q_CMD_SECTOR_ERASE_3_BYTE_ADDR);
    w25q_send_3_byte_addr(sector_idx);
  }
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();

  sysDelay(1);
  w25q.lock = 0;

  return W25Q_OK;
}

bool w25q_is_sector_empty(uint32_t sector_idx) {
  uint8_t buf[32] = {};
  uint32_t i;
  bool sector_empty = true;
  uint32_t curr_address = sector_idx * w25q.sector_size;
  for (i = 0; (i < w25q.sector_size) && sector_empty; i += 32) {
    w25q_read_buffer(buf, curr_address, 32);
    for (uint32_t x = 0; x < 32; x++) {
      if (buf[x] != 0xFF) {
        sector_empty = false;
        break;
      }
    }
    curr_address += 32;
  }
  return sector_empty;
}

w25q_status_e w25q_block_erase_32k(uint32_t block_idx) {
  while (w25q.lock == 1) sysDelay(1);
  w25q.lock = 1;

  w25qxx_wait_for_write_end();
  block_idx = block_idx * w25q.block_size;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_BLOCK_ERASE_32K);
  w25q_send_3_byte_addr(block_idx);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  sysDelay(1);
  w25q.lock = 0;
  return W25Q_OK;
}

w25q_status_e w25q_block_erase_64k(uint32_t block_idx) {
  while (w25q.lock == 1) sysDelay(1);
  w25q.lock = 1;

  w25qxx_wait_for_write_end();
  block_idx = block_idx * w25q.block_size;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  if (w25q.needs_4_byte_addressing) {
    w25qxx_spi_transmit(W25Q_CMD_BLOCK_ERASE_64K_4_BYTE_ADDR);
    w25q_send_4_byte_addr(block_idx);
  } else {
    w25qxx_spi_transmit(W25Q_CMD_BLOCK_ERASE_64K_3_BYTE_ADDR);
    w25q_send_3_byte_addr(block_idx);
  }
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  sysDelay(1);
  w25q.lock = 0;
  return W25Q_OK;
}

w25q_status_e w25q_chip_erase(void) {
  while (w25q.lock == 1) sysDelay(1);
  w25q.lock = 1;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_CHIP_ERASE);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  sysDelay(10);
  w25q.lock = 0;
  return W25Q_OK;
}

w25q_status_e w25qxx_write_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                                uint32_t bytes_to_write_up_to_page_size) {
  while (w25q.lock == 1) sysDelay(1);
  w25q.lock = 1;
  if (((bytes_to_write_up_to_page_size + offset_in_bytes) > w25q.page_size) || (bytes_to_write_up_to_page_size == 0))
    bytes_to_write_up_to_page_size = w25q.page_size - offset_in_bytes;
  if ((offset_in_bytes + bytes_to_write_up_to_page_size) > w25q.page_size)
    bytes_to_write_up_to_page_size = w25q.page_size - offset_in_bytes;

  page_num = (page_num * w25q.page_size) + offset_in_bytes;
  w25qxx_wait_for_write_end();
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  if (w25q.needs_4_byte_addressing) {
    w25qxx_spi_transmit(W25Q_CMD_PAGE_PROGRAM_4_BYTE_ADDR);
    w25q_send_4_byte_addr(page_num);
  } else {
    w25qxx_spi_transmit(W25Q_CMD_PAGE_PROGRAM_3_BYTE_ADDR);
    w25q_send_3_byte_addr(page_num);
  }
  HAL_SPI_Transmit(&FLASH_SPI_HANDLE, buf, bytes_to_write_up_to_page_size, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  w25q.lock = 0;
  return W25Q_OK;
}

w25q_status_e w25q_write_sector(uint8_t *buf, uint32_t sector_num, uint32_t offset_in_bytes,
                                uint32_t bytes_to_write_up_to_sector_size) {
  if ((bytes_to_write_up_to_sector_size > w25q.sector_size) || (bytes_to_write_up_to_sector_size == 0))
    bytes_to_write_up_to_sector_size = w25q.sector_size;
  if (offset_in_bytes >= w25q.sector_size) {
    return W25Q_ERR_TRANSMIT;
  }

  uint32_t start_page = w25q_sector_to_page(sector_num) + (offset_in_bytes / w25q.page_size);
  uint32_t local_offset = offset_in_bytes % w25q.page_size;
  int32_t bytes_to_write;
  if ((offset_in_bytes + bytes_to_write_up_to_sector_size) > w25q.sector_size)
    bytes_to_write = w25q.sector_size - offset_in_bytes;
  else
    bytes_to_write = bytes_to_write_up_to_sector_size;

  do {
    w25qxx_write_page(buf, start_page, local_offset, bytes_to_write);
    // log_debug("Page %lu written", start_page);
    start_page++;
    bytes_to_write -= w25q.page_size - local_offset;
    buf += w25q.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_write > 0);
  return W25Q_OK;
}

w25q_status_e w25q_write_buffer(uint8_t *buf, uint32_t write_addr, uint32_t num_bytes_to_write) {
  uint32_t bytes_to_write_up_to_sector_size = num_bytes_to_write;
  uint32_t offset_in_bytes = write_addr;
  uint32_t sector_num = num_bytes_to_write;

  if ((bytes_to_write_up_to_sector_size > w25q.sector_size) || (bytes_to_write_up_to_sector_size == 0))
    bytes_to_write_up_to_sector_size = w25q.sector_size;

  if (offset_in_bytes >= w25q.sector_size) {
    return W25Q_ERR_TRANSMIT;
  }

  uint32_t start_page = w25q_sector_to_page(sector_num) + (offset_in_bytes / w25q.page_size);
  uint32_t local_offset = offset_in_bytes % w25q.page_size;
  int32_t bytes_to_write;
  if ((offset_in_bytes + bytes_to_write_up_to_sector_size) > w25q.sector_size)
    bytes_to_write = w25q.sector_size - offset_in_bytes;
  else
    bytes_to_write = bytes_to_write_up_to_sector_size;

  do {
    w25qxx_write_page(buf, start_page, local_offset, bytes_to_write);
    start_page++;
    bytes_to_write -= w25q.page_size - local_offset;
    buf += w25q.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_write > 0);
  return W25Q_OK;
}

w25q_status_e w25qxx_read_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                               uint32_t NumByteToRead_up_to_PageSize) {
  while (w25q.lock == 1) sysDelay(1);
  w25q.lock = 1;
  if ((NumByteToRead_up_to_PageSize > w25q.page_size) || (NumByteToRead_up_to_PageSize == 0))
    NumByteToRead_up_to_PageSize = w25q.page_size;
  if ((offset_in_bytes + NumByteToRead_up_to_PageSize) > w25q.page_size)
    NumByteToRead_up_to_PageSize = w25q.page_size - offset_in_bytes;
  page_num = page_num * w25q.page_size + offset_in_bytes;
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  if (w25q.needs_4_byte_addressing) {
    w25qxx_spi_transmit(W25Q_CMD_FAST_READ_4_BYTE_ADDR);
    w25q_send_4_byte_addr(page_num);
  } else {
    w25qxx_spi_transmit(W25Q_CMD_FAST_READ_3_BYTE_ADDR);
    w25q_send_3_byte_addr(page_num);
  }
  w25qxx_spi_transmit(0);
  HAL_SPI_Receive(&FLASH_SPI_HANDLE, buf, NumByteToRead_up_to_PageSize, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  // sysDelay(1);
  w25q.lock = 0;
  return W25Q_OK;
}

w25q_status_e w25q_read_sector(uint8_t *buf, uint32_t sector_num, uint32_t offset_in_bytes,
                               uint32_t bytes_to_read_up_to_sector_size) {
  if ((bytes_to_read_up_to_sector_size > w25q.sector_size) || (bytes_to_read_up_to_sector_size == 0))
    bytes_to_read_up_to_sector_size = w25q.sector_size;

  if (offset_in_bytes >= w25q.sector_size) {
    return W25Q_ERR_TRANSMIT;
  }

  uint32_t start_page = w25q_sector_to_page(sector_num) + (offset_in_bytes / w25q.page_size);
  uint32_t local_offset = offset_in_bytes % w25q.page_size;
  int32_t bytes_to_read;
  if ((offset_in_bytes + bytes_to_read_up_to_sector_size) > w25q.sector_size)
    bytes_to_read = w25q.sector_size - offset_in_bytes;
  else
    bytes_to_read = bytes_to_read_up_to_sector_size;

  do {
    w25qxx_read_page(buf, start_page, local_offset, bytes_to_read);
    start_page++;
    bytes_to_read -= w25q.page_size - local_offset;
    buf += w25q.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_read > 0);
  return W25Q_OK;
}

w25q_status_e w25q_read_buffer(uint8_t *buf, uint32_t read_addr, uint32_t num_bytes_to_read) {
  uint32_t bytes_to_read_up_to_sector_size = num_bytes_to_read;
  uint32_t offset_in_bytes = read_addr;
  uint32_t sector_num = read_addr;

  if ((bytes_to_read_up_to_sector_size > w25q.sector_size) || (bytes_to_read_up_to_sector_size == 0))
    bytes_to_read_up_to_sector_size = w25q.sector_size;
  if (offset_in_bytes >= w25q.sector_size) {
    return W25Q_ERR_TRANSMIT;
  }

  uint32_t start_page = w25q_sector_to_page(sector_num) + (offset_in_bytes / w25q.page_size);
  uint32_t local_offset = offset_in_bytes % w25q.page_size;
  int32_t bytes_to_read;
  if ((offset_in_bytes + bytes_to_read_up_to_sector_size) > w25q.sector_size) {
    bytes_to_read = w25q.sector_size - offset_in_bytes;
  } else {
    bytes_to_read = bytes_to_read_up_to_sector_size;
  }
  do {
    w25qxx_read_page(buf, start_page, local_offset, bytes_to_read);
    start_page++;
    bytes_to_read -= w25q.page_size - local_offset;
    buf += w25q.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_read > 0);
  return W25Q_OK;
}

uint32_t w25q_sector_to_page(uint32_t sector_idx) { return (sector_idx * w25q.sector_size) / w25q.page_size; }

uint32_t w25q_block_to_page(uint32_t block_idx) { return (block_idx * w25q.block_size) / w25q.page_size; }
