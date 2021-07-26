#pragma once

/* Initial code taken from https://www.fatalerrors.org/a/stm32h7-peripheral-configuration-quick-reference-qspi-part.html
 */

#include <stdbool.h>
#include "stm32l433xx.h"
#include "main.h"

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
} w25q_id;

typedef struct {
  w25q_id id;
  uint16_t page_size;
  uint32_t page_count;
  uint32_t sector_size;
  uint32_t sector_count;
  uint32_t block_size;
  uint32_t block_count;
  uint32_t capacity_in_kilobytes;
} w25q_t;

#define QSPI_W25Qxx_OK            0   // w25qxx communication is normal
#define W25Qxx_ERROR_INIT         1   // initialization error
#define W25Qxx_ERROR_WRITEENABLE  -2  // write enable error
#define W25Qxx_ERROR_AUTOPOLLING  -3  // polling wait error, no response
#define W25Qxx_ERROR_Erase        -4  // erase error
#define W25Qxx_ERROR_TRANSMIT     -5  // transfer error
#define W25Qxx_ERROR_MemoryMapped -6  // memory mapping mode error

#define W25Q_CMD_ENABLE_RESET 0x66  // enable reset
#define W25Q_CMD_RESET_DEVICE 0x99  // reset device
#define W25Q_CMD_JEDEC_ID     0x9F  // JEDEC ID
#define W25Q_CMD_WRITE_ENABLE 0X06  // write enable

#define W25Q_CMD_SECTOR_ERASE    0x21  // sector erase, 4K bytes, reference erase time 45ms
#define W25Q_CMD_BlockErase_32K  0x52  // block erase, 32K bytes, reference erase time 120ms
#define W25Q_CMD_BLOCK_ERASE_64K 0xDC  // block erase, 64K bytes, reference erase time 150ms
#define W25Q_CMD_CHIP_ERASE      0xC7  // whole chip erase, reference erase time 20S

#define W25Q_CMD_QUAD_INPUT_PAGE_PROGRAM \
  0x34  // in 1-1-4 mode (1-line instruction, 1-line address, 4-line data), page programming instruction, reference
        // write time 0.4ms
#define W25Q_CMD_FAST_READ_QUAD_IO \
  0xEC  // in 1-4-4 mode (1-wire instruction, 4-wire address, 4-wire data), read instructions quickly

#define W25Q_CMD_READ_STATUS_REG1 0X05  // read status register 1
#define W25Q_CMD_READ_STATUS_REG2 0x35  // read status register 1
#define W25Q_CMD_READ_STATUS_REG3 0X15  // read status register 1

#define W25Q_STATUS_REG1_BUSY \
  0x01  // read status register 1 bit 0 (read-only), busy flag bit. When erasing / writing data / writing command is in
        // progress, it will be set to 1
#define W25Q_STATUS_REG1_WEL \
  0x02  // read the first bit of status register 1 (read-only). WEL write enable flag bit. When the flag bit is 1, it
        // means that write operation can be performed

#define W25Qxx_PageSize  256       // page size, 256 bytes
#define W25Q512_FLASH_ID 0Xef4020  // W25Q512 JEDEC ID
#define W25Q_CHIP_ERASE_TIMEOUT_MAX \
  100000U  // timeout waiting time. The maximum time required for W25Q64 to erase the whole chip is 100S
#define W25Qxx_Mem_Addr > 0x90000000  // address of memory mapping mode

/*----------------------------------------------- Function declaration--------------------------------------------*/

int8_t w25q_init(void);       // W25Qxx initialization
int8_t w25q_reset(void);      // Reset device
uint32_t w25q_read_id(void);  // Read device ID

int8_t w25q_sector_erase(uint32_t sector_address);     // Sector erase, 4K bytes, reference erase time 45ms
int8_t w25q_block_erase_32k(uint32_t sector_address);  // Block erase, 32K bytes, reference erase time 120ms
int8_t w25q_block_erase_64k(uint32_t sector_address);  // Block erase, 64K bytes, reference erase time 150ms, the actual
                                                       // use of the proposed 64K erase, erase the fastest time
int8_t w25q_chip_erase(void);                          // The reference erasing time is 20S

// Write by page, up to 256 bytes
int8_t w25q_write_page(uint8_t *buf, uint32_t write_addr, uint16_t num_bytes_to_write);

// Write data, the maximum can not exceed the num_bytes_to_write of the flash chip
int8_t w25q_write_buffer(uint8_t *buf, uint32_t write_addr, uint32_t num_bytes_to_write);

// Read data, the maximum can not exceed the size of the flash chip
int8_t w25q_read_buffer(uint8_t *buf, uint32_t read_addr, uint32_t num_bytes_to_read);

uint8_t w25q_read_status_reg1(void);
uint8_t w25q_read_status_reg2(void);
uint8_t w25q_read_status_reg3(void);

bool w25q_is_empty_sector(uint32_t sector_address);

extern w25q_t w25q;