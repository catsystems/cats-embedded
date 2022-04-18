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
#include "util/log.h"
#include "target.h"

#define CATS_FLASH_SPI 1
#define CATS_FLASH_QSPI 2

#ifdef CATS_VEGA
#define CATS_FLASH_MODE CATS_FLASH_SPI
#endif

#ifdef CATS_ORION
#define CATS_FLASH_MODE CATS_FLASH_QSPI
#endif


w25q_t w25q = {.id = W25QINVALID};

/* Settings */
#define W25Q_CMD_ENABLE_RESET              0x66
#define W25Q_CMD_RESET_DEVICE              0x99
#define W25Q_CMD_JEDEC_ID                  0x9F
#define W25Q_CMD_WRITE_ENABLE              0x06
#define W25Q_CMD_ENTER_4_BYTE_ADDRESS_MODE 0xB7

/* Status registers */
#define W25Q_CMD_READ_STATUS_REG1 0x05
#define W25Q_CMD_READ_STATUS_REG2 0x35
#define W25Q_CMD_READ_STATUS_REG3 0x15

/* Erasure commands */
#define W25Q_CMD_SECTOR_ERASE    0x21
#define W25Q_CMD_BLOCK_ERASE_32K 0x52
#define W25Q_CMD_BLOCK_ERASE_64K 0xDC
#define W25Q_CMD_CHIP_ERASE      0xC7

/* Write command */
#define W25Q_CMD_QUAD_INPUT_PAGE_PROGRAM 0x34

/* Read command */
#define W25Q_CMD_FAST_READ_QUAD_IO 0xEC

/* Status register 1 flash busy bit */
#define W25Q_STATUS_REG1_BUSY 0x01
/* Status register 1 write enabled bit */
#define W25Q_STATUS_REG1_WEL 0x02

#define W25Q_PAGE_SIZE 256

// Chip erase waiting timeout
#define W25Q_CHIP_ERASE_TIMEOUT_MAX 200000U



#if CATS_FLASH_MODE == CATS_FLASH_QSPI
// Write enable
int8_t w25q_write_enable(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressMode = QSPI_ADDRESS_NONE,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .DataMode = QSPI_DATA_NONE,
      .DummyCycles = 0,
      .Instruction = W25Q_CMD_WRITE_ENABLE,
  };

  // Send write enable command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return W25Q_ERR_WRITE_ENABLE;
  // Keep querying W25Q_CMD_READ_STATUS_REG1 register, read w25qxx in the status byte_ Status_ REG1_ Wel is compared
  // with 0x02 Read status register 1 bit 1 (read-only), WEL write enable flag bit. When the flag bit is 1, it means
  // that write operation can be performed

  s_command.Instruction = W25Q_CMD_READ_STATUS_REG1;  // Read status information register
  s_command.DataMode = QSPI_DATA_1_LINE;              // 1-line data mode
  s_command.NbData = 1;                               // Data length

  QSPI_AutoPollingTypeDef s_config = {
      .Match = 0x02,
      .Mask = W25Q_STATUS_REG1_WEL,
      .MatchMode = QSPI_MATCH_MODE_AND,
      .StatusBytesSize = 1,
      .Interval = 0x10,
      .AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE,
  };

  // Send polling wait command
  if (HAL_QSPI_AutoPolling(&FLASH_QSPI_HANDLE, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_ERR_AUTOPOLLING;

  return W25Q_OK;
}

w25q_status_e w25q_init(void) {
  w25q_reset();
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
  w25q.page_size = 256;
  w25q.sector_size = 4096;  // 4kB
  w25q.sector_count = w25q.block_count * 16;
  w25q.page_count = (w25q.sector_count * w25q.sector_size) / w25q.page_size;
  w25q.block_size = w25q.sector_size * 16;
  w25q.capacity_in_kilobytes = (w25q.sector_count * w25q.sector_size) / 1024;

  osDelay(10);

  uint8_t status1 = 0;
  uint8_t status2 = 0;
  uint8_t status3 = 0;
  w25q_read_status_reg(1, &status1);
  w25q_read_status_reg(2, &status2);
  w25q_read_status_reg(3, &status3);

  log_debug("Flash statuses: %x %x %x", status1, status2, status3);

  return W25Q_OK;
}

// Polling to confirm whether the FLASH is idle (used to wait for the end of communication, etc.)
int8_t w25q_auto_polling_mem_ready(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressMode = QSPI_ADDRESS_NONE,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .DataMode = QSPI_DATA_1_LINE,
      .DummyCycles = 0,
      .Instruction = W25Q_CMD_READ_STATUS_REG1,
  };

  // Keep querying W25Q_CMD_READ_STATUS_REG1 register, read w25q in the status byte_ Status_ REG1_ Busy is compared
  // with 0 Read status register 1 bit 0 (read-only), Busy flag bit, when erasing / writing data / writing command will
  // be set to 1, idle or communication end to 0
  QSPI_AutoPollingTypeDef s_config = {
      .Match = 0,
      .MatchMode = QSPI_MATCH_MODE_AND,
      .Interval = 0x10,
      .AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE,
      .StatusBytesSize = 1,

      .Mask = W25Q_STATUS_REG1_BUSY,
  };

  // Send polling wait command
  if (HAL_QSPI_AutoPolling(&FLASH_QSPI_HANDLE, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_ERR_AUTOPOLLING;

  return W25Q_OK;  // Communication ended normally
}

// FLASH software reset
w25q_status_e w25q_reset(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressMode = QSPI_ADDRESS_NONE,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .DataMode = QSPI_DATA_NONE,
      .DummyCycles = 0,
      .Instruction = W25Q_CMD_ENABLE_RESET,
  };

  // Send reset enable command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_ERR_INIT;  // If the sending fails, an error message is returned
  // Use the automatic polling flag bit to wait for the end of communication
  if (w25q_auto_polling_mem_ready() != W25Q_OK) return W25Q_ERR_AUTOPOLLING;

  s_command.Instruction = W25Q_CMD_RESET_DEVICE;  // Reset device command

  // Send reset device command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_ERR_INIT;  // If the sending fails, an error message is returned

  // Use the automatic polling flag bit to wait for the end of communication
  if (w25q_auto_polling_mem_ready() != W25Q_OK) return W25Q_ERR_AUTOPOLLING;

  s_command.Instruction = W25Q_CMD_ENTER_4_BYTE_ADDRESS_MODE;

  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_ERR_INIT;  // If the sending fails, an error message is returned

  // Use the automatic polling flag bit to wait for the end of communication
  if (w25q_auto_polling_mem_ready() != W25Q_OK) return W25Q_ERR_AUTOPOLLING;

  return W25Q_OK;
}

w25q_status_e w25q_read_id(uint32_t *w25q_id) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressSize = QSPI_ADDRESS_32_BITS,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .AddressMode = QSPI_ADDRESS_NONE,
      .DataMode = QSPI_DATA_1_LINE,
      .DummyCycles = 0,
      .NbData = 3,
      .Instruction = W25Q_CMD_JEDEC_ID,
  };

  uint8_t qspi_receive_buff[3];  // Store data read by QSPI

  // Send command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_ERR_INIT;  // If the sending fails, an error message is returned
  // receive data
  if (HAL_QSPI_Receive(&FLASH_QSPI_HANDLE, qspi_receive_buff, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Q_ERR_TRANSMIT;  // If the reception fails, an error message is returned

  // Combine the obtained data into ID
  *w25q_id = (qspi_receive_buff[0] << 16) | (qspi_receive_buff[1] << 8) | qspi_receive_buff[2];
  return W25Q_OK;
}

w25q_status_e w25q_read_status_reg(uint8_t status_reg_num, uint8_t *status_reg_val) {
  if (status_reg_num < 1 || status_reg_num > 3) {
    return W25Q_ERR_INVALID_PARAM;
  }
  uint8_t status_reg_cmd = 0x0;
  switch (status_reg_num) {
    case 1:
      status_reg_cmd = W25Q_CMD_READ_STATUS_REG1;
      break;
    case 2:
      status_reg_cmd = W25Q_CMD_READ_STATUS_REG2;
      break;
    case 3:
      status_reg_cmd = W25Q_CMD_READ_STATUS_REG3;
      break;
    default:
      return W25Q_ERR_INVALID_PARAM;
  }

  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressSize = QSPI_ADDRESS_32_BITS,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .AddressMode = QSPI_ADDRESS_NONE,
      .DataMode = QSPI_DATA_1_LINE,
      .DummyCycles = 0,
      .NbData = 1,
      .Instruction = status_reg_cmd,
  };

  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return W25Q_ERR_INIT;

  // receive data
  if (HAL_QSPI_Receive(&FLASH_QSPI_HANDLE, status_reg_val, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return W25Q_ERR_TRANSMIT;

  return W25Q_OK;
}

w25q_status_e w25q_sector_erase(uint32_t sector_idx) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressSize = QSPI_ADDRESS_32_BITS,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .AddressMode = QSPI_ADDRESS_1_LINE,
      .DataMode = QSPI_DATA_NONE,
      .DummyCycles = 0,
      .Address = sector_idx * w25q.sector_size,
      .Instruction = W25Q_CMD_SECTOR_ERASE,
  };

  if (w25q_write_enable() != W25Q_OK) {
    return W25Q_ERR_WRITE_ENABLE;
  }

  // Issue erase command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_ERASE;  // Erase failed
  }

  // Use the automatic polling flag bit to wait for the end of erasure
  if (w25q_auto_polling_mem_ready() != W25Q_OK) {
    return W25Q_ERR_AUTOPOLLING;
  }
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

/* TODO: Since blocks on W25Q are 64k and this function accepts a block index this means that it can only erase the
 * first halves of 64k blocks. This should be fixed if this function gets used at some point. */
w25q_status_e w25q_block_erase_32k(uint32_t block_idx) {
  QSPI_CommandTypeDef s_command = {.InstructionMode = QSPI_INSTRUCTION_1_LINE,
                                   .AddressSize = QSPI_ADDRESS_32_BITS,
                                   .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
                                   .DdrMode = QSPI_DDR_MODE_DISABLE,
                                   .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
                                   .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
                                   .AddressMode = QSPI_ADDRESS_1_LINE,
                                   .DataMode = QSPI_DATA_NONE,
                                   .DummyCycles = 0,
                                   .Address = block_idx * w25q.block_size,
                                   .Instruction = W25Q_CMD_BLOCK_ERASE_32K};

  if (w25q_write_enable() != W25Q_OK) {
    return W25Q_ERR_WRITE_ENABLE;
  }

  // Issue erase command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_ERASE;
  }

  // Use the automatic polling flag bit to wait for the end of erasure
  if (w25q_auto_polling_mem_ready() != W25Q_OK) {
    return W25Q_ERR_AUTOPOLLING;
  }
  return W25Q_OK;
}

w25q_status_e w25q_block_erase_64k(uint32_t block_idx) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressSize = QSPI_ADDRESS_32_BITS,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .AddressMode = QSPI_ADDRESS_1_LINE,
      .DataMode = QSPI_DATA_NONE,
      .DummyCycles = 0,
      .Address = block_idx * w25q.block_size,
      .Instruction = W25Q_CMD_BLOCK_ERASE_64K,
  };

  if (w25q_write_enable() != W25Q_OK) {
    return W25Q_ERR_WRITE_ENABLE;
  }

  // Issue erase command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_ERASE;
  }

  // Use the automatic polling flag bit to wait for the end of erasure
  if (w25q_auto_polling_mem_ready() != W25Q_OK) {
    return W25Q_ERR_AUTOPOLLING;
  }
  return W25Q_OK;
}

w25q_status_e w25q_chip_erase(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressSize = QSPI_ADDRESS_32_BITS,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .AddressMode = QSPI_ADDRESS_NONE,
      .DataMode = QSPI_DATA_NONE,
      .DummyCycles = 0,
      .Instruction = W25Q_CMD_CHIP_ERASE,
  };

  // Send write enable
  if (w25q_write_enable() != W25Q_OK) {
    return W25Q_ERR_WRITE_ENABLE;
  }
  // Issue erase command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_ERASE;
  }

  // Keep querying W25Q_CMD_READ_STATUS_REG1 register, read w25qxx in the status byte_ Status_ REG1_ Busy keeps
  // comparing with 0 Read status register 1 bit 0 (read-only), Busy flag bit, when erasing / writing data / writing
  // command will be set to 1, idle or communication end to 0
  QSPI_AutoPollingTypeDef s_config = {
      .Match = 0,
      .MatchMode = QSPI_MATCH_MODE_AND,
      .Interval = 0x10,
      .AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE,
      .StatusBytesSize = 1,
      .Mask = W25Q_STATUS_REG1_BUSY,
  };

  s_command.Instruction = W25Q_CMD_READ_STATUS_REG1;
  s_command.DataMode = QSPI_DATA_1_LINE;
  s_command.NbData = 1;

  if (HAL_QSPI_AutoPolling(&FLASH_QSPI_HANDLE, &s_command, &s_config, W25Q_CHIP_ERASE_TIMEOUT_MAX) != HAL_OK) {
    return W25Q_ERR_AUTOPOLLING;
  }
  return W25Q_OK;
}

/* write in */
w25q_status_e w25q_write_page(uint8_t *buf, uint32_t write_addr, uint16_t num_bytes_to_write) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressSize = QSPI_ADDRESS_32_BITS,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .AddressMode = QSPI_ADDRESS_1_LINE,
      .DataMode = QSPI_DATA_4_LINES,
      .DummyCycles = 0,
      .NbData = num_bytes_to_write,
      .Address = write_addr,
      .Instruction = W25Q_CMD_QUAD_INPUT_PAGE_PROGRAM,

  };

  if (w25q_write_enable() != W25Q_OK) {
    return W25Q_ERR_WRITE_ENABLE;
  }
  // Write command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_TRANSMIT_CMD;
  }
  // Start data transfer
  if (HAL_QSPI_Transmit(&FLASH_QSPI_HANDLE, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_TRANSMIT;
  }
  // Use the automatic polling flag bit to wait for the end of the write
  if (w25q_auto_polling_mem_ready() != W25Q_OK) {
    return W25Q_ERR_AUTOPOLLING;
  }
  return W25Q_OK;
}

w25q_status_e w25q_write_buffer(uint8_t *buf, uint32_t write_addr, uint32_t num_bytes_to_write) {
  uint32_t end_addr, current_size, current_addr;
  uint8_t *write_data;

  // Calculates the remaining space on the current page
  current_size = W25Q_PAGE_SIZE - (write_addr % W25Q_PAGE_SIZE);

  // Determine whether the remaining space of the current page is enough to write all data
  if (current_size > num_bytes_to_write) {
    // If it is enough, the current length is obtained directly
    current_size = num_bytes_to_write;
  }

  current_addr = write_addr;
  end_addr = write_addr + num_bytes_to_write;
  write_data = buf;

  w25q_status_e write_err = W25Q_OK;
  do {
    // Send write enable
    if (w25q_write_enable() != W25Q_OK) {
      return W25Q_ERR_WRITE_ENABLE;
    }

    // Write data by page
    else if ((write_err = w25q_write_page(write_data, current_addr, current_size)) != W25Q_OK) {
      return write_err;
    }
    // Use the automatic polling flag bit to wait for the end of the write
    else if (w25q_auto_polling_mem_ready() != W25Q_OK) {
      return W25Q_ERR_AUTOPOLLING;
    } else  // Write data by page successfully, prepare for the next data write
    {
      current_addr += current_size;  // Calculate the next write address
      write_data += current_size;    // Gets the address of the data store to be written next time
      // Calculate the length of the next write
      current_size = ((current_addr + W25Q_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : W25Q_PAGE_SIZE;
    }
  } while (current_addr < end_addr);  // Judge whether all data are written

  return W25Q_OK;
}

/* read */
w25q_status_e w25q_read_buffer(uint8_t *buf, uint32_t read_addr, uint32_t num_bytes_to_read) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,
      .AddressSize = QSPI_ADDRESS_32_BITS,
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,
      .DdrMode = QSPI_DDR_MODE_DISABLE,
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,
      .AddressMode = QSPI_ADDRESS_4_LINES,
      .DataMode = QSPI_DATA_4_LINES,
      .DummyCycles = 6,
      .NbData = num_bytes_to_read,
      .Address = read_addr,
      .Instruction = W25Q_CMD_FAST_READ_QUAD_IO,
  };

  // Send read command
  if (HAL_QSPI_Command(&FLASH_QSPI_HANDLE, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_TRANSMIT_CMD;
  }

  //	receive data
  if (HAL_QSPI_Receive(&FLASH_QSPI_HANDLE, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Q_ERR_TRANSMIT;
  }

  // Use automatic polling flag bits to wait for the end of reception
  if (w25q_auto_polling_mem_ready() != W25Q_OK) {
    return W25Q_ERR_AUTOPOLLING;
  }
  return W25Q_OK;
}

uint32_t w25q_sector_to_page(uint32_t sector_idx) { return (sector_idx * w25q.sector_size) / w25q.page_size; }

uint32_t w25q_block_to_page(uint32_t block_idx) { return (block_idx * w25q.block_size) / w25q.page_size; }

#elif CATS_FLASH_MODE == CATS_FLASH_SPI

static inline void w25qxx_spi_transmit(uint8_t data) {
  HAL_SPI_Transmit(&FLASH_SPI_HANDLE, &data, 1, 100);
}

static inline uint8_t w25qxx_spi_receive() {
  uint8_t ret;
  HAL_SPI_Receive(&FLASH_SPI_HANDLE, &ret, 1, 100);
  return ret;
}

static inline void w25qxx_wait_for_write_end(void) {
  // osDelay(1);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x05);
  uint8_t status_reg_val = 0x00;
  do {
    status_reg_val = w25qxx_spi_receive();
    osDelay(1);
  } while ((status_reg_val & 0x01) == 0x01);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static inline void w25qxx_send_address(uint32_t address) {
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

// Write enable
int8_t w25q_write_enable(void) {
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x06);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  // osDelay(1);
  return W25Q_OK;
}

w25q_status_e w25q_init(void) {
  //w25q_reset();
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
  w25q.page_size = 256;
  w25q.sector_size = 4096;  // 4kB
  w25q.sector_count = w25q.block_count * 16;
  w25q.page_count = (w25q.sector_count * w25q.sector_size) / w25q.page_size;
  w25q.block_size = w25q.sector_size * 16;
  w25q.capacity_in_kilobytes = (w25q.sector_count * w25q.sector_size) / 1024;

  osDelay(10);

  uint8_t status1 = 0;
  uint8_t status2 = 0;
  uint8_t status3 = 0;
  w25q_read_status_reg(1, &status1);
  w25q_read_status_reg(2, &status2);
  w25q_read_status_reg(3, &status3);

  log_debug("Flash statuses: %x %x %x", status1, status2, status3);

  return W25Q_OK;
}

w25q_status_e w25q_read_id(uint32_t  *w25q_id) {
  uint32_t temp0 = 0, temp1 = 0, temp2 = 0;
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  //(0x9F);
  w25qxx_spi_transmit(0x9F);
  temp0 = w25qxx_spi_receive();
  temp1 = w25qxx_spi_receive();
  temp2 = w25qxx_spi_receive();
  //  temp0 = w25qxx_spi_receive(W25QXX_DUMMY_BYTE);
  //  temp1 = w25qxx_spi_receive(W25QXX_DUMMY_BYTE);
  //  temp2 = w25qxx_spi_receive(W25QXX_DUMMY_BYTE);
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
  while (w25q.lock == 1) osDelay(1);
  w25q.lock = 1;

  w25qxx_wait_for_write_end();
  sector_idx = sector_idx * w25q.sector_size;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_SECTOR_ERASE);
  w25qxx_send_address(sector_idx);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();

  osDelay(1);
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
  while (w25q.lock == 1) osDelay(1);
  w25q.lock = 1;

  w25qxx_wait_for_write_end();
  block_idx = block_idx * w25q.block_size;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_BLOCK_ERASE_32K);
  w25qxx_send_address(block_idx);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  osDelay(1);
  w25q.lock = 0;
  return W25Q_OK;
}

w25q_status_e w25q_block_erase_64k(uint32_t block_idx) {
  while (w25q.lock == 1) osDelay(1);
  w25q.lock = 1;

  w25qxx_wait_for_write_end();
  block_idx = block_idx * w25q.block_size;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_BLOCK_ERASE_64K);
  w25qxx_send_address(block_idx);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  osDelay(1);
  w25q.lock = 0;
  return W25Q_OK;
}

w25q_status_e w25q_chip_erase(void) {
  while (w25q.lock == 1) osDelay(1);
  w25q.lock = 1;
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(W25Q_CMD_CHIP_ERASE);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  osDelay(10);
  w25q.lock = 0;
  return W25Q_OK;
}

void w25qxx_write_page(uint8_t *buf, uint32_t page_num,
                       uint32_t offset_in_bytes,
                       uint32_t bytes_to_write_up_to_page_size) {
  while (w25q.lock == 1) osDelay(1);
  w25q.lock = 1;
  if (((bytes_to_write_up_to_page_size + offset_in_bytes) > w25q.page_size) ||
      (bytes_to_write_up_to_page_size == 0))
    bytes_to_write_up_to_page_size = w25q.page_size - offset_in_bytes;
  if ((offset_in_bytes + bytes_to_write_up_to_page_size) > w25q.page_size)
    bytes_to_write_up_to_page_size = w25q.page_size - offset_in_bytes;

  page_num = (page_num * w25q.page_size) + offset_in_bytes;
  w25qxx_wait_for_write_end();
  w25q_write_enable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  /* TODO: This should probably be 0x12 */
  /* TODO: most likely the last address bits should be 0 */
  w25qxx_spi_transmit(0x12);
  w25qxx_send_address(page_num);
  HAL_SPI_Transmit(&FLASH_SPI_HANDLE, buf, bytes_to_write_up_to_page_size, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
  w25q.lock = 0;
}




w25q_status_e w25q_write_buffer(uint8_t *buf, uint32_t write_addr, uint32_t num_bytes_to_write) {

  uint32_t bytes_to_write_up_to_sector_size = num_bytes_to_write;
  uint32_t offset_in_bytes = write_addr;
  uint32_t sector_num = num_bytes_to_write;

  if ((bytes_to_write_up_to_sector_size > w25q.sector_size) ||
      (bytes_to_write_up_to_sector_size == 0))
    bytes_to_write_up_to_sector_size = w25q.sector_size;

  if (offset_in_bytes >= w25q.sector_size) {
    return W25Q_ERR_TRANSMIT;
  }
  uint32_t start_page;
  int32_t bytes_to_write;
  uint32_t local_offset;
  if ((offset_in_bytes + bytes_to_write_up_to_sector_size) > w25q.sector_size)
    bytes_to_write = w25q.sector_size - offset_in_bytes;
  else
    bytes_to_write = bytes_to_write_up_to_sector_size;
  start_page =
      w25q_sector_to_page(sector_num) + (offset_in_bytes / w25q.page_size);
  local_offset = offset_in_bytes % w25q.page_size;
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

void w25qxx_read_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                      uint32_t NumByteToRead_up_to_PageSize) {
  while (w25q.lock == 1) osDelay(1);
  w25q.lock = 1;
  if ((NumByteToRead_up_to_PageSize > w25q.page_size) ||
      (NumByteToRead_up_to_PageSize == 0))
    NumByteToRead_up_to_PageSize = w25q.page_size;
  if ((offset_in_bytes + NumByteToRead_up_to_PageSize) > w25q.page_size)
    NumByteToRead_up_to_PageSize = w25q.page_size - offset_in_bytes;
  page_num = page_num * w25q.page_size + offset_in_bytes;
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x0C);
  w25qxx_send_address(page_num);
  //  if (w25qxx.id >= W25Q256) w25qxx_spi_transmit((page_num & 0xFF000000) >>
  //  24); w25qxx_spi_transmit((page_num & 0xFF0000) >> 16);
  //  w25qxx_spi_transmit((page_num & 0xFF00) >> 8);
  //  w25qxx_spi_transmit(page_num & 0xFF);
  w25qxx_spi_transmit(0);
  HAL_SPI_Receive(&FLASH_SPI_HANDLE, buf, NumByteToRead_up_to_PageSize, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  // osDelay(1);
  w25q.lock = 0;
}

w25q_status_e w25q_read_buffer(uint8_t *buf, uint32_t read_addr, uint32_t num_bytes_to_read) {

  uint32_t bytes_to_read_up_to_sector_size = num_bytes_to_read;
  uint32_t offset_in_bytes = read_addr;
  uint32_t sector_num = read_addr;

  if ((bytes_to_read_up_to_sector_size > w25q.sector_size) ||
      (bytes_to_read_up_to_sector_size == 0))
    bytes_to_read_up_to_sector_size = w25q.sector_size;
  if (offset_in_bytes >= w25q.sector_size) {
    return W25Q_ERR_TRANSMIT;
  }
  uint32_t start_page;
  int32_t bytes_to_read;
  uint32_t local_offset;
  if ((offset_in_bytes + bytes_to_read_up_to_sector_size) > w25q.sector_size)
    bytes_to_read = w25q.sector_size - offset_in_bytes;
  else
    bytes_to_read = bytes_to_read_up_to_sector_size;
  start_page =
      w25q_sector_to_page(sector_num) + (offset_in_bytes / w25q.page_size);
  local_offset = offset_in_bytes % w25q.page_size;
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

#endif
