#include "drivers/w25q256.h"
#include "util/log.h"

extern QSPI_HandleTypeDef hqspi;

w25q_t w25q = {.id = W25QINVALID};

// Write enable
int8_t w25q_write_enable(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressMode = QSPI_ADDRESS_NONE,                // No address mode
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  // No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .DataMode = QSPI_DATA_NONE,                      // No data mode
      .DummyCycles = 0,                                // Number of empty periods
      .Instruction = W25Q_CMD_WRITE_ENABLE,            // Send write enable command
  };                                                   // QSPI transport configuration

  // Send write enable command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return W25Qxx_ERROR_WRITEENABLE;
  // Keep querying W25Q_CMD_READ_STATUS_REG1 register, read w25qxx in the status byte_ Status_ REG1_ Wel is compared
  // with 0x02 Read status register 1 bit 1 (read-only), WEL write enable flag bit. When the flag bit is 1, it means
  // that write operation can be performed

  s_command.Instruction = W25Q_CMD_READ_STATUS_REG1;  // Read status information register
  s_command.DataMode = QSPI_DATA_1_LINE;              // 1-line data mode
  s_command.NbData = 1;                               // Data length

  QSPI_AutoPollingTypeDef s_config = {
      .Match = 0x02,                 // Match value
      .Mask = W25Q_STATUS_REG1_WEL,  // Read status register 1 bit 1 (read-only), WEL write enable flag bit. When
      // the flag bit is 1, it means that write operation can be performed
      .MatchMode = QSPI_MATCH_MODE_AND,             // Sum operation
      .StatusBytesSize = 1,                         // Status bytes
      .Interval = 0x10,                             // Polling interval
      .AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE,  // Auto stop mode
  };                                                // Polling comparison related configuration parameters

  // Send polling wait command
  if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_AUTOPOLLING;  // Polling waiting for no response, error returned

  return QSPI_W25Qxx_OK;  // Communication ended normally
}

static void MX_QUADSPI_Init(void) {
  hqspi.Instance = QUADSPI;  // QSPI peripherals

  // When the memory mapping mode is used, the frequency division coefficient here cannot be set to 0, otherwise the
  // reading error will occur
  hqspi.Init.ClockPrescaler = 1;  // The QSPI core clock is divided by 1 + 1 to get the QSPI communication driver clock
  hqspi.Init.FifoThreshold = 4;   // FIFO threshold
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;  // Sample after half CLK cycle
  hqspi.Init.FlashSize = 25;  // FLASH size, the number of bytes in FLASH = 2^[FSIZE+1], for 8MB W25Q64 set to 22
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;  // Time for chip selection to keep high level
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;                   // Mode 0
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;                       // Using QSPI1
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;              // Turn off dual flash mode
  // Application configuration
  HAL_QSPI_Init(&hqspi);
}

/* Check W25Q64 */
int8_t w25q_init(void) {
  MX_QUADSPI_Init();                    // Initialize QSPI
  w25q_reset();                         // reset
  uint32_t device_id = w25q_read_id();  // Read ID

  // Check peripheral devices

  log_debug("w25qxx ID:0x%lX", device_id);

  switch (device_id & 0x0000FFFF) {
    case 0x4020:  // 	w25q512
      w25q.id = W25Q512;
      w25q.block_count = 1024;
      log_debug("w25q Chip: w25q512");
      break;
    case 0x4019:  // 	w25q256
      w25q.id = W25Q256;
      w25q.block_count = 512;
      log_debug("w25q Chip: w25q256");
      break;
    case 0x4018:  // 	w25q128
      w25q.id = W25Q128;
      w25q.block_count = 256;
      log_debug("w25q Chip: w25q128");
      break;
    case 0x4017:  //	w25q64
      w25q.id = W25Q64;
      w25q.block_count = 128;
      log_debug("w25q Chip: w25q64");
      break;
    case 0x4016:  //	w25q32
      w25q.id = W25Q32;
      w25q.block_count = 64;
      log_debug("w25q Chip: w25q32");
      break;
    case 0x4015:  //	w25q16
      w25q.id = W25Q16;
      w25q.block_count = 32;
      log_debug("w25q Chip: w25q16");
      break;
    case 0x4014:  //	w25q80
      w25q.id = W25Q80;
      w25q.block_count = 16;
      log_debug("w25q Chip: w25q80");
      break;
    case 0x4013:  //	w25q40
      w25q.id = W25Q40;
      w25q.block_count = 8;
      log_debug("w25q Chip: w25q40");
      break;
    case 0x4012:  //	w25q20
      w25q.id = W25Q20;
      w25q.block_count = 4;
      log_debug("w25q Chip: w25q20");
      break;
    case 0x4011:  //	w25q10
      w25q.id = W25Q10;
      w25q.block_count = 2;
      log_debug("w25q Chip: w25q10");
      break;
    default:
      log_debug("w25q Unknown ID");
      return QSPI_W25Qxx_OK;
  }
  w25q.page_size = 256;
  w25q.sector_size = 0x1000;  // 4kB
  w25q.sector_count = w25q.block_count * 16;
  w25q.page_count = (w25q.sector_count * w25q.sector_size) / w25q.page_size;
  w25q.block_size = w25q.sector_size * 16;
  w25q.capacity_in_kilobytes = (w25q.sector_count * w25q.sector_size) / 1024;
  return QSPI_W25Qxx_OK;  // Return success flag
}

// Polling to confirm whether the FLASH is idle (used to wait for the end of communication, etc.)
int8_t w25q_auto_polling_mem_ready(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // 	One line command mode
      .AddressMode = QSPI_ADDRESS_NONE,                // 	No address mode
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  //	No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // 	Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // 	Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            //	Every time the data is transmitted, an instruction is sent
      .DataMode = QSPI_DATA_1_LINE,                    // 	1-line data mode
      .DummyCycles = 0,                                //	Number of empty periods
      .Instruction = W25Q_CMD_READ_STATUS_REG1,        // 	Read status information register
  };

  // Keep querying W25Q_CMD_READ_STATUS_REG1 register, read w25qxx in the status byte_ Status_ REG1_ Busy is compared
  // with 0 Read status register 1 bit 0 (read-only), Busy flag bit, when erasing / writing data / writing command will
  // be set to 1, idle or communication end to 0
  QSPI_AutoPollingTypeDef s_config = {
      .Match = 0,                                   //	Match value
      .MatchMode = QSPI_MATCH_MODE_AND,             //	Sum operation
      .Interval = 0x10,                             //	Polling interval
      .AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE,  // Auto stop mode
      .StatusBytesSize = 1,                         //	Status bytes
      // Mask the status bytes received in polling mode, and only compare the needed bits
      .Mask = W25Q_STATUS_REG1_BUSY,
  };

  // Send polling wait command
  if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response

  return QSPI_W25Qxx_OK;  // Communication ended normally
}

// FLASH software reset
int8_t w25q_reset(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressMode = QSPI_ADDRESS_NONE,                // No address mode
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  // No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .DataMode = QSPI_DATA_NONE,                      // No data mode
      .DummyCycles = 0,                                // Number of empty periods
      .Instruction = W25Q_CMD_ENABLE_RESET,            // Execute reset enable command
  };                                                   // QSPI transport configuration

  // Send reset enable command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_INIT;  // If the sending fails, an error message is returned
  // Use the automatic polling flag bit to wait for the end of communication
  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response

  s_command.Instruction = W25Q_CMD_RESET_DEVICE;  // Reset device command

  // Send reset device command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_INIT;  // If the sending fails, an error message is returned

  // Use the automatic polling flag bit to wait for the end of communication
  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response

  if (w25q_write_enable() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_WRITEENABLE;
  }

  osDelay(1000);

  s_command.Instruction = 0x11;

  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_INIT;  // If the sending fails, an error message is returned

  uint8_t ucRegister3 = 0x62;
  // Start data transfer
  if (HAL_QSPI_Transmit(&hqspi, &ucRegister3, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_TRANSMIT;  // Transmission data error
  }

  // Use the automatic polling flag bit to wait for the end of communication
  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response

  return QSPI_W25Qxx_OK;  // Reset successfully
}

uint32_t w25q_read_id(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,             // 24 bit address
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  // No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_NONE,                // No address mode
      .DataMode = QSPI_DATA_1_LINE,                    // 1-line data mode
      .DummyCycles = 0,                                // Number of empty periods
      .NbData = 3,                                     // Length of transmitted data
      .Instruction = W25Q_CMD_JEDEC_ID,                // Execute read device ID command
  };                                                   // QSPI transport configuration

  uint8_t QSPI_ReceiveBuff[3];  // Store data read by QSPI
  uint32_t W25Qxx_ID = 0;       // Device ID

  // Send command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_INIT;  // If the sending fails, an error message is returned
  // receive data
  if (HAL_QSPI_Receive(&hqspi, QSPI_ReceiveBuff, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_TRANSMIT;  // If the reception fails, an error message is returned

  // Combine the obtained data into ID
  W25Qxx_ID = (QSPI_ReceiveBuff[0] << 16) | (QSPI_ReceiveBuff[1] << 8) | QSPI_ReceiveBuff[2];
  return W25Qxx_ID;  // Return ID
}

uint8_t w25q_read_status_reg1(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,             // 24 bit address
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  // No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_NONE,                // No address mode
      .DataMode = QSPI_DATA_1_LINE,                    // 1-line data mode
      .DummyCycles = 0,                                // Number of empty periods
      .NbData = 1,                                     // Length of transmitted data
      .Instruction = W25Q_CMD_READ_STATUS_REG1,        // Execute read device ID command
  };                                                   // QSPI transport configuration

  uint8_t QSPI_ReceiveBuff = 0;  // Store data read by QSPI

  // Send command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_INIT;  // If the sending fails, an error message is returned
  // receive data
  if (HAL_QSPI_Receive(&hqspi, &QSPI_ReceiveBuff, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_TRANSMIT;  // If the reception fails, an error message is returned

  return QSPI_ReceiveBuff;  // Return ID
}

uint8_t w25q_read_status_reg2(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,             // 24 bit address
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  // No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_NONE,                // No address mode
      .DataMode = QSPI_DATA_1_LINE,                    // 1-line data mode
      .DummyCycles = 0,                                // Number of empty periods
      .NbData = 1,                                     // Length of transmitted data
      .Instruction = W25Q_CMD_READ_STATUS_REG2,        // Execute read device ID command
  };                                                   // QSPI transport configuration

  uint8_t QSPI_ReceiveBuff = 0;  // Store data read by QSPI

  // Send command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_INIT;  // If the sending fails, an error message is returned
  // receive data
  if (HAL_QSPI_Receive(&hqspi, &QSPI_ReceiveBuff, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_TRANSMIT;  // If the reception fails, an error message is returned

  return QSPI_ReceiveBuff;  // Return ID
}

uint8_t w25q_read_status_reg3(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,             // 24 bit address
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  // No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_NONE,                // No address mode
      .DataMode = QSPI_DATA_1_LINE,                    // 1-line data mode
      .DummyCycles = 0,                                // Number of empty periods
      .NbData = 1,                                     // Length of transmitted data
      .Instruction = W25Q_CMD_READ_STATUS_REG3,        // Execute read device ID command
  };                                                   // QSPI transport configuration

  uint8_t QSPI_ReceiveBuff = 0;  // Store data read by QSPI

  // Send command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_INIT;  // If the sending fails, an error message is returned
  // receive data
  if (HAL_QSPI_Receive(&hqspi, &QSPI_ReceiveBuff, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return W25Qxx_ERROR_TRANSMIT;  // If the reception fails, an error message is returned

  return QSPI_ReceiveBuff;  // Return ID
}

/* Erase */
// Here, the original document is copied, and the instructions are repeated without comment
int8_t w25q_sector_erase(uint32_t sector_address) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,             // 24 bit address mode
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  //	No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_1_LINE,              // 1-line address mode
      .DataMode = QSPI_DATA_NONE,                      // No data
      .DummyCycles = 0,                                // Number of empty periods
      .Address = sector_address,                       // Address to erase
      .Instruction = W25Q_CMD_SECTOR_ERASE,            // Sector erase command
  };                                                   // QSPI transport configuration

  // Send write enable
  if (w25q_write_enable() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_WRITEENABLE;  // Write enable failed
  }
  // Issue erase command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_Erase;  // Erase failed
  }
  // Use the automatic polling flag bit to wait for the end of erasure
  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response
  }
  return QSPI_W25Qxx_OK;  // Erase succeeded
}

bool w25q_is_empty_sector(uint32_t sector_address) {
  uint8_t buf[32];
  uint32_t i;
  for (i = 0; i < 4096; i += 32) {
    w25q_read_buffer(buf, sector_address, 32);
    if (buf[31] != 0xFF) break;
  }
  if (i < 4096)
    return false;
  else
    return true;
}
//
// int8_t w25q_block_erase_32k(uint32_t SectorAddress) {
//  QSPI_CommandTypeDef s_command;  // QSPI transport configuration
//
//  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;      // One line command mode
//  s_command.AddressSize = QSPI_ADDRESS_32_BITS;             // 24 bit address mode
//  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;  //	No alternate bytes
//  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;                // Disable DDR mode
//  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;   // Data delay in DDR mode is not used here
//  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;      // Every time the data is transmitted, an instruction is sent
//  s_command.AddressMode = QSPI_ADDRESS_1_LINE;        // 1-line address mode
//  s_command.DataMode = QSPI_DATA_NONE;                // No data
//  s_command.DummyCycles = 0;                          // Number of empty periods
//  s_command.Address = SectorAddress;                  // Address to erase
//  s_command.Instruction = W25Q_CMD_BlockErase_32K;  // Block erase command, each erase 32K bytes
//
//  // Send write enable
//  if (w25q_write_enable() != QSPI_W25Qxx_OK) {
//    return W25Qxx_ERROR_WRITEENABLE;  // Write enable failed
//  }
//  // Issue erase command
//  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
//    return W25Qxx_ERROR_Erase;  // Erase failed
//  }
//  // Use the automatic polling flag bit to wait for the end of erasure
//  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) {
//    return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response
//  }
//  return QSPI_W25Qxx_OK;  // Erase succeeded
//}

int8_t w25q_block_erase_64k(uint32_t sector_address) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,             // 24 bit address mode
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  //	No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_1_LINE,              // 1-line address mode
      .DataMode = QSPI_DATA_NONE,                      // No data
      .DummyCycles = 0,                                // Number of empty periods
      .Address = sector_address,                       // Address to erase
      .Instruction = W25Q_CMD_BLOCK_ERASE_64K,         // Block erase command, erase 64K bytes each time
  };                                                   // QSPI transport configuration

  // Send write enable
  if (w25q_write_enable() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_WRITEENABLE;  // Write enable failed
  }
  // Issue erase command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_Erase;  // Erase failed
  }
  // Use the automatic polling flag bit to wait for the end of erasure
  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response
  }
  return QSPI_W25Qxx_OK;  // Erase succeeded
}

int8_t w25q_chip_erase(void) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,      // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,             // 24 bit address mode
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,  //	No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,   // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,            // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_NONE,                // No address
      .DataMode = QSPI_DATA_NONE,                      // No data
      .DummyCycles = 0,                                // Number of empty periods
      .Instruction = W25Q_CMD_CHIP_ERASE,              // Erase command to erase the whole piece
  };                                                   // QSPI transport configuration

  // Send write enable
  if (w25q_write_enable() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_WRITEENABLE;  // Write enable failed
  }
  // Issue erase command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_Erase;  // Erase failed
  }

  // Keep querying W25Q_CMD_READ_STATUS_REG1 register, read w25qxx in the status byte_ Status_ REG1_ Busy keeps
  // comparing with 0 Read status register 1 bit 0 (read-only), Busy flag bit, when erasing / writing data / writing
  // command will be set to 1, idle or communication end to 0

  QSPI_AutoPollingTypeDef s_config = {
      .Match = 0,                                   //	Match value
      .MatchMode = QSPI_MATCH_MODE_AND,             //	Sum operation
      .Interval = 0x10,                             //	Polling interval
      .AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE,  // Auto stop mode
      .StatusBytesSize = 1,                         //	Status bytes
      .Mask =
          W25Q_STATUS_REG1_BUSY,  // Mask the status bytes received in polling mode, and only compare the needed bits
  };  // Polling wait configuration parameters

  s_command.Instruction = W25Q_CMD_READ_STATUS_REG1;  // Read status information register
  s_command.DataMode = QSPI_DATA_1_LINE;              // 1-line data mode
  s_command.NbData = 1;                               // Data length

  // The typical reference time of W25Q64 whole chip erasure is 20s, and the maximum time is 100S. Here, the timeout
  // value W25Q_CHIP_ERASE_TIMEOUT_MAX is 100S
  if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, W25Q_CHIP_ERASE_TIMEOUT_MAX) != HAL_OK) {
    return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response
  }
  return QSPI_W25Qxx_OK;
}

/* write in */
int8_t w25q_write_page(uint8_t* buf, uint32_t write_addr, uint16_t num_bytes_to_write) {
  QSPI_CommandTypeDef s_command = {
      .InstructionMode = QSPI_INSTRUCTION_1_LINE,       // One line command mode
      .AddressSize = QSPI_ADDRESS_32_BITS,              // 24 bit address
      .AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE,   // No alternate bytes
      .DdrMode = QSPI_DDR_MODE_DISABLE,                 // Disable DDR mode
      .DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY,    // Data delay in DDR mode is not used here
      .SIOOMode = QSPI_SIOO_INST_EVERY_CMD,             // Every time the data is transmitted, an instruction is sent
      .AddressMode = QSPI_ADDRESS_1_LINE,               // 1-line address mode
      .DataMode = QSPI_DATA_4_LINES,                    // 4-wire data mode
      .DummyCycles = 0,                                 // Number of empty periods
      .NbData = num_bytes_to_write,                     // The maximum length of data is 256 bytes
      .Address = write_addr,                            // Address to write to W25Qxx
      .Instruction = W25Q_CMD_QUAD_INPUT_PAGE_PROGRAM,  // 1-1-4 mode (1 line instruction, 1 line address, 4 line data),
                                                        // page programming instruction
  };                                                    // QSPI transport configuration

  // Write enable
  if (w25q_write_enable() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_WRITEENABLE;  // Write enable failed
  }
  // Write command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_TRANSMIT;  // Transmission data error
  }
  // Start data transfer
  if (HAL_QSPI_Transmit(&hqspi, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_TRANSMIT;  // Transmission data error
  }
  // Use the automatic polling flag bit to wait for the end of the write
  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response
  }
  return QSPI_W25Qxx_OK;  // Write data successfully
}

int8_t w25q_write_buffer(uint8_t* buf, uint32_t write_addr, uint32_t num_bytes_to_write) {
  uint32_t end_addr, current_size, current_addr;
  uint8_t* write_data;  // Data to write

  current_size =
      W25Qxx_PageSize - (write_addr % W25Qxx_PageSize);  // Calculates the remaining space on the current page

  if (current_size >
      num_bytes_to_write)  // Determine whether the remaining space of the current page is enough to write all data
  {
    current_size = num_bytes_to_write;  // If it is enough, the current length is obtained directly
  }

  current_addr = write_addr;                   // Get the address to write to
  end_addr = write_addr + num_bytes_to_write;  // Calculation end address
  write_data = buf;                            // Gets the data to be written

  do {
    // Send write enable
    if (w25q_write_enable() != QSPI_W25Qxx_OK) {
      return W25Qxx_ERROR_WRITEENABLE;
    }

    // Write data by page
    else if (w25q_write_page(write_data, current_addr, current_size) != QSPI_W25Qxx_OK) {
      return W25Qxx_ERROR_TRANSMIT;
    }

    // Use the automatic polling flag bit to wait for the end of the write
    else if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) {
      return W25Qxx_ERROR_AUTOPOLLING;
    }

    else  // Write data by page successfully, prepare for the next data write
    {
      current_addr += current_size;  // Calculate the next write address
      write_data += current_size;    // Gets the address of the data store to be written next time
      // Calculate the length of the next write
      current_size = ((current_addr + W25Qxx_PageSize) > end_addr) ? (end_addr - current_addr) : W25Qxx_PageSize;
    }
  } while (current_addr < end_addr);  // Judge whether all data are written

  return QSPI_W25Qxx_OK;  // Write data successfully
}

/* read */
int8_t w25q_read_buffer(uint8_t* buf, uint32_t read_addr, uint32_t num_bytes_to_read) {
  QSPI_CommandTypeDef s_command = {};  // QSPI transport configuration

  s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;      // One line command mode
  s_command.AddressSize = QSPI_ADDRESS_32_BITS;             // 24 bit address
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;  // No alternate bytes
  s_command.DdrMode = QSPI_DDR_MODE_DISABLE;                // Disable DDR mode
  s_command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;   // Data delay in DDR mode is not used here
  s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;  // Every time the data is transmitted, an instruction is sent
  s_command.AddressMode = QSPI_ADDRESS_4_LINES;   // 4-wire address mode
  s_command.DataMode = QSPI_DATA_4_LINES;         // 4-wire data mode
  s_command.DummyCycles = 6;                      // Number of empty periods
  s_command.NbData = num_bytes_to_read;  // The maximum data length should not exceed the size of the flash chip
  s_command.Address = read_addr;         // To read the address of W25Qxx
  s_command.Instruction = W25Q_CMD_FAST_READ_QUAD_IO;  // 1-4-4 mode (1 line instruction, 4 line address, 4 line data),
                                                       // fast read instruction

  // Send read command
  if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_TRANSMIT;  // Transmission data error
  }

  //	receive data

  if (HAL_QSPI_Receive(&hqspi, buf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    return W25Qxx_ERROR_TRANSMIT;  // Transmission data error
  }

  // Use automatic polling flag bits to wait for the end of reception
  if (w25q_auto_polling_mem_ready() != QSPI_W25Qxx_OK) {
    return W25Qxx_ERROR_AUTOPOLLING;  // Polling wait no response
  }
  return QSPI_W25Qxx_OK;  // Read data successfully
}