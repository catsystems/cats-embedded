#ifndef w25q256_H
#define w25q256_H

/* Initial code taken from https://www.fatalerrors.org/a/stm32h7-peripheral-configuration-quick-reference-qspi-part.html
 */

#include "stm32l433xx.h"
#include "main.h"

/*----------------------------------------------- Named parameter macro-------------------------------------------*/

#define QSPI_W25Qxx_OK            0   // w25qxx communication is normal
#define W25Qxx_ERROR_INIT         1   // initialization error
#define W25Qxx_ERROR_WRITEENABLE  -2  // write enable error
#define W25Qxx_ERROR_AUTOPOLLING  -3  // polling wait error, no response
#define W25Qxx_ERROR_Erase        -4  // erase error
#define W25Qxx_ERROR_TRANSMIT     -5  // transfer error
#define W25Qxx_ERROR_MemoryMapped -6  // memory mapping mode error

#define W25Qxx_CMD_EnableReset 0x66  // enable reset
#define W25Qxx_CMD_ResetDevice 0x99  // reset device
#define W25Qxx_CMD_JedecID     0x9F  // JEDEC ID
#define W25Qxx_CMD_WriteEnable 0X06  // write enable

#define W25Qxx_CMD_SectorErase    0x20  // sector erase, 4K bytes, reference erase time 45ms
#define W25Qxx_CMD_BlockErase_32K 0x52  // block erase, 32K bytes, reference erase time 120ms
#define W25Qxx_CMD_BlockErase_64K 0xD8  // block erase, 64K bytes, reference erase time 150ms
#define W25Qxx_CMD_ChipErase      0xC7  // whole chip erase, reference erase time 20S

#define W25Qxx_CMD_QuadInputPageProgram \
  0x32  // in 1-1-4 mode (1-line instruction, 1-line address, 4-line data), page programming instruction, reference
        // write time 0.4ms
#define W25Qxx_CMD_FastReadQuad_IO \
  0xEB  // in 1-4-4 mode (1-wire instruction, 4-wire address, 4-wire data), read instructions quickly

#define W25Qxx_CMD_ReadStatus_REG1 0X05  // read status register 1
#define W25Qxx_CMD_ReadStatus_REG2 0x35  // read status register 1
#define W25Qxx_CMD_ReadStatus_REG3 0X15  // read status register 1
#define W25Qxx_Status_REG1_BUSY \
  0x01  // read status register 1 bit 0 (read-only), busy flag bit. When erasing / writing data / writing command is in
        // progress, it will be set to 1
#define W25Qxx_Status_REG1_WEL \
  0x02  // read the first bit of status register 1 (read-only). WEL write enable flag bit. When the flag bit is 1, it
        // means that write operation can be performed

#define W25Qxx_PageSize  256  // page size, 256 bytes
#define W25Qxx_FlashSize 0x800000 / / W25Q64 size, 8M bytes
#define W25Qxx_FLASH_ID  0Xef4019  // W25Q64 JEDEC ID
#define W25Qxx_ChipErase_TIMEOUT_MAX \
  100000U  // timeout waiting time. The maximum time required for W25Q64 to erase the whole chip is 100S
#define W25Qxx_Mem_Addr > 0x90000000  // address of memory mapping mode

/*----------------------------------------------- Pin configuration macro------------------------------------------*/

#define QUADSPI_CLK_PIN         GPIO_PIN_2                    // QUADSPI_CLK pin
#define QUADSPI_CLK_PORT        GPIOB                         // QUADSPI_CLK pin port
#define QUADSPI_CLK_AF          GPIO_AF9_QUADSPI              // QUADSPI_CLK IO port multiplexing
#define GPIO_QUADSPI_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE()  // QUADSPI_CLK pin clock enable

#define QUADSPI_BK1_NCS_PIN         GPIO_PIN_6                    // QUADSPI_BK1_NCS pin
#define QUADSPI_BK1_NCS_PORT        GPIOB                         // QUADSPI_BK1_NCS pin port
#define QUADSPI_BK1_NCS_AF          GPIO_AF10_QUADSPI             // QUADSPI_BK1_NCS IO port multiplexing
#define GPIO_QUADSPI_BK1_NCS_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE()  // QUADSPI_BK1_NCS pin clock enable

#define QUADSPI_BK1_IO0_PIN         GPIO_PIN_11                   // QUADSPI_BK1_IO0 pin
#define QUADSPI_BK1_IO0_PORT        GPIOD                         // QUADSPI_BK1_IO0 pin port
#define QUADSPI_BK1_IO0_AF          GPIO_AF9_QUADSPI              // QUADSPI_BK1_IO0 IO port multiplexing
#define GPIO_QUADSPI_BK1_IO0_ENABLE __HAL_RCC_GPIOD_CLK_ENABLE()  // QUADSPI_BK1_IO0 pin clock enable

#define QUADSPI_BK1_IO1_PIN         GPIO_PIN_2                    // QUADSPI_BK1_IO1 pin
#define QUADSPI_BK1_IO1_PORT        GPIOE                         // QUADSPI_BK1_IO1 pin port
#define QUADSPI_BK1_IO1_AF          GPIO_AF9_QUADSPI              // QUADSPI_BK1_IO1 IO port multiplexing
#define GPIO_QUADSPI_BK1_IO1_ENABLE __HAL_RCC_GPIOE_CLK_ENABLE()  // QUADSPI_BK1_IO1 pin clock enable

#define QUADSPI_BK1_IO2_PIN         GPIO_PIN_12                   // QUADSPI_BK1_IO2 pin
#define QUADSPI_BK1_IO2_PORT        GPIOD                         // QUADSPI_BK1_IO2 pin port
#define QUADSPI_BK1_IO2_AF          GPIO_AF9_QUADSPI              // QUADSPI_BK1_IO2 IO port multiplexing
#define GPIO_QUADSPI_BK1_IO2_ENABLE __HAL_RCC_GPIOD_CLK_ENABLE()  // QUADSPI_BK1_IO2 pin clock enable

#define QUADSPI_BK1_IO3_PIN         GPIO_PIN_13                   // QUADSPI_BK1_IO3 pin
#define QUADSPI_BK1_IO3_PORT        GPIOD                         // QUADSPI_BK1_IO3 pin port
#define QUADSPI_BK1_IO3_AF          GPIO_AF9_QUADSPI              // QUADSPI_BK1_IO3 IO port multiplexing
#define GPIO_QUADSPI_BK1_IO3_ENABLE __HAL_RCC_GPIOD_CLK_ENABLE()  // QUADSPI_BK1_IO3 pin clock enable

/*----------------------------------------------- Function declaration--------------------------------------------*/

int8_t QSPI_W25Qxx_Init(void);              // W25Qxx initialization
int8_t QSPI_W25Qxx_Reset(void);             // Reset device
uint32_t QSPI_W25Qxx_ReadID(void);          // Read device ID
int8_t QSPI_W25Qxx_MemoryMappedMode(void);  // Enter memory mapping mode

int8_t QSPI_W25Qxx_SectorErase(uint32_t SectorAddress);     // Sector erase, 4K bytes, reference erase time 45ms
int8_t QSPI_W25Qxx_BlockErase_32K(uint32_t SectorAddress);  // Block erase, 32K bytes, reference erase time 120ms
int8_t QSPI_W25Qxx_BlockErase_64K(
    uint32_t SectorAddress);  // Block erase, 64K bytes, reference erase time 150ms, the actual use of the proposed 64K
                              // erase, erase the fastest time
int8_t QSPI_W25Qxx_ChipErase(void);  // The reference erasing time is 20S

// Write by page, up to 256 bytes
int8_t QSPI_W25Qxx_WritePage(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

// Write data, the maximum can not exceed the size of the flash chip
int8_t QSPI_W25Qxx_WriteBuffer(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);

// Read data, the maximum can not exceed the size of the flash chip
int8_t QSPI_W25Qxx_ReadBuffer(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);

uint8_t QSPI_W25Qxx_ReadStatus1(void);
uint8_t QSPI_W25Qxx_ReadStatus2(void);
uint8_t QSPI_W25Qxx_ReadStatus3(void);

extern QSPI_HandleTypeDef hqspi;

#endif