
#include "drivers/w25qxx.h"
#include "drivers/w25qxxConf.h"
#include "stm32l4xx_hal.h"
#include "util/log.h"
#include "config/globals.h"

#define W25QXX_DUMMY_BYTE 0xA5

w25qxx_t w25qxx = {.id = W25QINVALID};

/** Private Function Declarations **/

static inline void w25qxx_send_address(uint32_t address);
static inline void w25qxx_spi_transmit(uint8_t data);
static inline uint8_t w25qxx_spi_receive();
static inline uint8_t w25qxx_spi_transmit_receive(uint8_t data);
static inline uint32_t w25qxx_read_id(void);
static inline void w25qxx_read_uniq_id(void);
static inline void w25qxx_write_enable(void);
static inline void w25qxx_write_disable(void);
static inline uint8_t w25qxx_read_status_register(uint8_t status_register);
static inline void w25qxx_write_status_register(uint8_t status_register, uint8_t data);
static inline void w25qxx_wait_for_write_end(void);

/** Exported Function Definitions **/

bool w25qxx_init(void) {
  w25qxx.lock = 1;
  while (osKernelGetTickCount() < 100) osDelay(1);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  osDelay(100);
  uint32_t id;

  log_debug("w25qxx Init Begin...");

  id = w25qxx_read_id();

  log_debug("w25qxx ID:0x%lX", id);

  switch (id & 0x0000FFFF) {
    case 0x4020:  // 	w25q512
      w25qxx.id = W25Q512;
      w25qxx.block_count = 1024;
      log_debug("w25qxx Chip: w25q512");
      break;
    case 0x4019:  // 	w25q256
      w25qxx.id = W25Q256;
      w25qxx.block_count = 512;
      log_debug("w25qxx Chip: w25q256");
      break;
    case 0x4018:  // 	w25q128
      w25qxx.id = W25Q128;
      w25qxx.block_count = 256;
      log_debug("w25qxx Chip: w25q128");
      break;
    case 0x4017:  //	w25q64
      w25qxx.id = W25Q64;
      w25qxx.block_count = 128;
      log_debug("w25qxx Chip: w25q64");
      break;
    case 0x4016:  //	w25q32
      w25qxx.id = W25Q32;
      w25qxx.block_count = 64;
      log_debug("w25qxx Chip: w25q32");
      break;
    case 0x4015:  //	w25q16
      w25qxx.id = W25Q16;
      w25qxx.block_count = 32;
      log_debug("w25qxx Chip: w25q16");
      break;
    case 0x4014:  //	w25q80
      w25qxx.id = W25Q80;
      w25qxx.block_count = 16;
      log_debug("w25qxx Chip: w25q80");
      break;
    case 0x4013:  //	w25q40
      w25qxx.id = W25Q40;
      w25qxx.block_count = 8;
      log_debug("w25qxx Chip: w25q40");
      break;
    case 0x4012:  //	w25q20
      w25qxx.id = W25Q20;
      w25qxx.block_count = 4;
      log_debug("w25qxx Chip: w25q20");
      break;
    case 0x4011:  //	w25q10
      w25qxx.id = W25Q10;
      w25qxx.block_count = 2;
      log_debug("w25qxx Chip: w25q10");
      break;
    default:
      log_debug("w25qxx Unknown ID");
      w25qxx.lock = 0;
      return false;
  }
  w25qxx.page_size = 256;
  w25qxx.sector_size = 0x1000;
  w25qxx.sector_count = w25qxx.block_count * 16;
  w25qxx.page_count = (w25qxx.sector_count * w25qxx.sector_size) / w25qxx.page_size;
  w25qxx.block_size = w25qxx.sector_size * 16;
  w25qxx.capacity_in_kilobytes = (w25qxx.sector_count * w25qxx.sector_size) / 1024;

  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0xB7);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);

  w25qxx_read_uniq_id();
  w25qxx_read_status_register(1);
  w25qxx_read_status_register(2);
  w25qxx_read_status_register(3);

  log_debug("w25qxx Page Size: %hu B", w25qxx.page_size);
  log_debug("w25qxx Page Count: %lu", w25qxx.page_count);
  log_debug("w25qxx Sector Size: %lu B", w25qxx.sector_size);
  log_debug("w25qxx Sector Count: %lu", w25qxx.sector_count);
  log_debug("w25qxx Block Size: %lu B", w25qxx.block_size);
  log_debug("w25qxx Block Count: %lu", w25qxx.block_count);
  log_debug("w25qxx Capacity: %lu KB", w25qxx.capacity_in_kilobytes);
  log_debug("w25qxx Init Done");

  w25qxx.lock = 0;
  return true;
}

void w25qxx_erase_chip(void) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
#if (CATS_W25QXX_DEBUG == 1)
  uint32_t start_time = osKernelGetTickCount();
  log_debug("w25qxx EraseChip Begin...");
#endif
  w25qxx_write_enable();
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0xC7);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx EraseBlock done after %lu ms!", osKernelGetTickCount() - start_time);
#endif
  osDelay(10);
  w25qxx.lock = 0;
}

void w25qxx_erase_sector(uint32_t sector_addr) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
#if (CATS_W25QXX_DEBUG == 1)
  uint32_t start_time = osKernelGetTickCount();
  log_debug("w25qxx EraseSector %lu Begin...", sector_addr);
#endif
  w25qxx_wait_for_write_end();
  sector_addr = sector_addr * w25qxx.sector_size;
  w25qxx_write_enable();
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x21);
  w25qxx_send_address(sector_addr);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx EraseSector done after %lu ms", osKernelGetTickCount() - start_time);
#endif
  osDelay(1);
  w25qxx.lock = 0;
}

void W25qxx_EraseBlock(uint32_t block_num) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx EraseBlock %lu Begin...", block_num);
  osDelay(100);
  uint32_t start_time = osKernelGetTickCount();
#endif
  w25qxx_wait_for_write_end();
  block_num = block_num * w25qxx.sector_size * 16;
  w25qxx_write_enable();
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0xD8);
  w25qxx_send_address(block_num);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx EraseBlock done after %lu ms", osKernelGetTickCount() - start_time);
  osDelay(100);
#endif
  osDelay(1);
  w25qxx.lock = 0;
}

uint32_t w25qxx_page_to_sector(uint32_t page_num) { return ((page_num * w25qxx.page_size) / w25qxx.sector_size); }

uint32_t w25qxx_page_to_block(uint32_t page_num) { return ((page_num * w25qxx.page_size) / w25qxx.block_size); }

uint32_t w25qxx_sector_to_block(uint32_t sector_num) { return ((sector_num * w25qxx.sector_size) / w25qxx.block_size); }

uint32_t w25qxx_sector_to_page(uint32_t sector_num) { return (sector_num * w25qxx.sector_size) / w25qxx.page_size; }

uint32_t w25qxx_block_to_page(uint32_t block_num) { return (block_num * w25qxx.block_size) / w25qxx.page_size; }

bool w25qxx_is_empty_page(uint32_t page_num, uint32_t offset_in_bytes, uint32_t bytes_to_check_up_to_page_size) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
  if (((bytes_to_check_up_to_page_size + offset_in_bytes) > w25qxx.page_size) || (bytes_to_check_up_to_page_size == 0))
    bytes_to_check_up_to_page_size = w25qxx.page_size - offset_in_bytes;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckPage:%lu, Offset:%lu, Bytes:%lu begin...", page_num, offset_in_bytes,
            bytes_to_check_up_to_page_size);
  osDelay(100);
  uint32_t start_time = osKernelGetTickCount();
#endif
  uint8_t buf[32];
  uint32_t work_address;
  uint32_t i;
  for (i = offset_in_bytes; i < w25qxx.page_size; i += sizeof(buf)) {
    work_address = (i + page_num * w25qxx.page_size);
    HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
    w25qxx_spi_transmit(0x0C);
    w25qxx_send_address(work_address);
    w25qxx_spi_transmit(0);
    HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, sizeof(buf), 100);
    HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
    for (int x = 0; x < sizeof(buf); x++) {
      if (buf[x] != 0xFF) goto NOT_EMPTY;
    }
  }
  if ((w25qxx.page_size + offset_in_bytes) % sizeof(buf) != 0) {
    i -= sizeof(buf);
    for (; i < w25qxx.page_size; i++) {
      work_address = (i + page_num * w25qxx.page_size);
      HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
      w25qxx_spi_transmit(0x0C);
      w25qxx_send_address(work_address);
      w25qxx_spi_transmit(0);
      HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, 1, 100);
      HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
      if (buf[0] != 0xFF) goto NOT_EMPTY;
    }
  }
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckPage is Empty in %lu ms", osKernelGetTickCount() - start_time);
  osDelay(100);
#endif
  w25qxx.lock = 0;
  return true;
NOT_EMPTY:
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckPage is Not Empty in %lu ms", osKernelGetTickCount() - start_time);
  osDelay(100);
#endif
  w25qxx.lock = 0;
  return false;
}

bool w25qxx_is_empty_sector(uint32_t sector_num, uint32_t offset_in_bytes, uint32_t bytes_to_check_up_to_sector_size) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
  if ((bytes_to_check_up_to_sector_size > w25qxx.sector_size) || (bytes_to_check_up_to_sector_size == 0))
    bytes_to_check_up_to_sector_size = w25qxx.sector_size;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckSector:%lu, Offset:%lu, Bytes:%lu begin...", sector_num, offset_in_bytes,
            bytes_to_check_up_to_sector_size);
  osDelay(100);
  uint32_t start_time = osKernelGetTickCount();
#endif
  uint8_t buf[32];
  uint32_t work_address;
  uint32_t i;
  for (i = offset_in_bytes; i < w25qxx.sector_size; i += sizeof(buf)) {
    work_address = (i + sector_num * w25qxx.sector_size);
    HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
    w25qxx_spi_transmit(0x0C);
    w25qxx_send_address(work_address);
    w25qxx_spi_transmit(0);
    HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, sizeof(buf), 100);
    HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
    for (int x = 0; x < sizeof(buf); x++) {
      if (buf[x] != 0xFF) goto NOT_EMPTY;
    }
  }
  if ((w25qxx.sector_size + offset_in_bytes) % sizeof(buf) != 0) {
    i -= sizeof(buf);
    for (; i < w25qxx.sector_size; i++) {
      work_address = (i + sector_num * w25qxx.sector_size);
      HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
      w25qxx_spi_transmit(0x0C);
      w25qxx_send_address(work_address);
      w25qxx_spi_transmit(0);
      HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, 1, 100);
      HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
      if (buf[0] != 0xFF) goto NOT_EMPTY;
    }
  }
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckSector is Empty in %lu ms", osKernelGetTickCount() - start_time);
  osDelay(100);
#endif
  w25qxx.lock = 0;
  return true;
NOT_EMPTY:
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckSector is Not Empty in %lu ms", osKernelGetTickCount() - start_time);
  osDelay(100);
#endif
  w25qxx.lock = 0;
  return false;
}

bool w25qxx_is_empty_block(uint32_t block_num, uint32_t offset_in_bytes, uint32_t bytes_to_check_up_to_block_size) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
  if ((bytes_to_check_up_to_block_size > w25qxx.block_size) || (bytes_to_check_up_to_block_size == 0))
    bytes_to_check_up_to_block_size = w25qxx.block_size;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckBlock:%lu, Offset:%lu, Bytes:%lu begin...", block_num, offset_in_bytes,
            bytes_to_check_up_to_block_size);
  osDelay(100);
  uint32_t start_time = osKernelGetTickCount();
#endif
  uint8_t buf[32];
  uint32_t work_address;
  uint32_t i;
  for (i = offset_in_bytes; i < w25qxx.block_size; i += sizeof(buf)) {
    work_address = (i + block_num * w25qxx.block_size);
    HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
    w25qxx_spi_transmit(0x0C);
    w25qxx_send_address(work_address);
    w25qxx_spi_transmit(0);
    HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, sizeof(buf), 100);
    HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
    for (int x = 0; x < sizeof(buf); x++) {
      if (buf[x] != 0xFF) goto NOT_EMPTY;
    }
  }
  if ((w25qxx.block_size + offset_in_bytes) % sizeof(buf) != 0) {
    i -= sizeof(buf);
    for (; i < w25qxx.block_size; i++) {
      work_address = (i + block_num * w25qxx.block_size);
      HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
      w25qxx_spi_transmit(0x0C);
      w25qxx_send_address(work_address);
      w25qxx_spi_transmit(0);
      HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, 1, 100);
      HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
      if (buf[0] != 0xFF) goto NOT_EMPTY;
    }
  }
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckBlock is Empty in %lu ms", osKernelGetTickCount() - start_time);
  osDelay(100);
#endif
  w25qxx.lock = 0;
  return true;
NOT_EMPTY:
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx CheckBlock is Not Empty in %lu ms", osKernelGetTickCount() - start_time);
  osDelay(100);
#endif
  w25qxx.lock = 0;
  return false;
}

void w25qxx_write_byte(uint8_t byte, uint32_t byte_address) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
#if (CATS_W25QXX_DEBUG == 1)
  uint32_t start_time = osKernelGetTickCount();
  log_debug("w25qxx WriteByte 0x%02X at address %lu begin...", byte, byte_address);
#endif
  w25qxx_wait_for_write_end();
  w25qxx_write_enable();
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x02);
  w25qxx_send_address(byte_address);
  w25qxx_spi_transmit(byte);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx WriteByte done after %lu ms", osKernelGetTickCount() - start_time);
#endif
  w25qxx.lock = 0;
}

void w25qxx_write_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                       uint32_t bytes_to_write_up_to_page_size) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
  if (((bytes_to_write_up_to_page_size + offset_in_bytes) > w25qxx.page_size) || (bytes_to_write_up_to_page_size == 0))
    bytes_to_write_up_to_page_size = w25qxx.page_size - offset_in_bytes;
  if ((offset_in_bytes + bytes_to_write_up_to_page_size) > w25qxx.page_size)
    bytes_to_write_up_to_page_size = w25qxx.page_size - offset_in_bytes;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx WritePage:%lu, Offset:%lu ,Writes %lu Bytes, begin...", page_num, offset_in_bytes,
            bytes_to_write_up_to_page_size);
  osDelay(100);
  uint32_t start_time = osKernelGetTickCount();
#endif
  page_num = (page_num * w25qxx.page_size) + offset_in_bytes;
  w25qxx_wait_for_write_end();
  w25qxx_write_enable();
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  /* TODO: This should probably be 0x12 */
  /* TODO: most likely the last address bits should be 0 */
  w25qxx_spi_transmit(0x12);
  w25qxx_send_address(page_num);
  HAL_SPI_Transmit(&CATS_W25QXX_SPI, buf, bytes_to_write_up_to_page_size, 100);
  // spi_transmit(&SPI2_FLASH, buf, bytes_to_write_up_to_page_size);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  w25qxx_wait_for_write_end();
#if (CATS_W25QXX_DEBUG == 1)
  start_time = osKernelGetTickCount() - start_time;
  for (uint32_t i = 0; i < bytes_to_write_up_to_page_size; i++) {
    if ((i % 8 == 0) && (i > 2)) {
      log_rawr("\n");
      osDelay(10);
    }
    log_rawr("0x%02X,", buf[i]);
  }
  log_rawr("\n");
  log_debug("w25qxx WritePage done after %lu ms", start_time);
  osDelay(100);
#endif
  // osDelay(1);
  w25qxx.lock = 0;
}

void w25qxx_write_sector(uint8_t *buf, uint32_t sector_num, uint32_t offset_in_bytes,
                         uint32_t bytes_to_write_up_to_sector_size) {
  if ((bytes_to_write_up_to_sector_size > w25qxx.sector_size) || (bytes_to_write_up_to_sector_size == 0))
    bytes_to_write_up_to_sector_size = w25qxx.sector_size;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx WriteSector:%lu, Offset:%lu ,Write %lu Bytes, begin...", sector_num, offset_in_bytes,
            bytes_to_write_up_to_sector_size);
  osDelay(100);
#endif
  if (offset_in_bytes >= w25qxx.sector_size) {
#if (CATS_W25QXX_DEBUG == 1)
    log_debug("---w25qxx WriteSector Faild!");
    osDelay(100);
#endif
    return;
  }
  uint32_t start_page;
  int32_t bytes_to_write;
  uint32_t local_offset;
  if ((offset_in_bytes + bytes_to_write_up_to_sector_size) > w25qxx.sector_size)
    bytes_to_write = w25qxx.sector_size - offset_in_bytes;
  else
    bytes_to_write = bytes_to_write_up_to_sector_size;
  start_page = w25qxx_sector_to_page(sector_num) + (offset_in_bytes / w25qxx.page_size);
  local_offset = offset_in_bytes % w25qxx.page_size;
  do {
    w25qxx_write_page(buf, start_page, local_offset, bytes_to_write);
    // log_debug("Page %lu written", start_page);
    start_page++;
    bytes_to_write -= w25qxx.page_size - local_offset;
    buf += w25qxx.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_write > 0);
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("---w25qxx WriteSector Done");
  osDelay(100);
#endif
}

void w25qxx_write_block(uint8_t *buf, uint32_t block_num, uint32_t offset_in_bytes,
                        uint32_t bytes_to_write_up_to_block_size) {
  if ((bytes_to_write_up_to_block_size > w25qxx.block_size) || (bytes_to_write_up_to_block_size == 0))
    bytes_to_write_up_to_block_size = w25qxx.block_size;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx WriteBlock:%lu, Offset:%lu ,Write %lu Bytes, begin...", block_num, offset_in_bytes,
            bytes_to_write_up_to_block_size);
  osDelay(100);
#endif
  if (offset_in_bytes >= w25qxx.block_size) {
#if (CATS_W25QXX_DEBUG == 1)
    log_debug("---w25qxx WriteBlock Faild!");
    osDelay(100);
#endif
    return;
  }
  uint32_t start_page;
  int32_t bytes_to_write;
  uint32_t local_offset;
  if ((offset_in_bytes + bytes_to_write_up_to_block_size) > w25qxx.block_size)
    bytes_to_write = w25qxx.block_size - offset_in_bytes;
  else
    bytes_to_write = bytes_to_write_up_to_block_size;
  start_page = w25qxx_block_to_page(block_num) + (offset_in_bytes / w25qxx.page_size);
  local_offset = offset_in_bytes % w25qxx.page_size;
  do {
    w25qxx_write_page(buf, start_page, local_offset, bytes_to_write);
    start_page++;
    bytes_to_write -= w25qxx.page_size - local_offset;
    buf += w25qxx.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_write > 0);
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("---w25qxx WriteBlock Done");
  osDelay(100);
#endif
}

void w25qxx_read_byte(uint8_t *buf, uint32_t byte_address) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
#if (CATS_W25QXX_DEBUG == 1)
  uint32_t start_time = osKernelGetTickCount();
  log_debug("w25qxx ReadByte at address %lu begin...", byte_address);
#endif
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x0C);
  w25qxx_send_address(byte_address);
  //  if (w25qxx.id >= W25Q256) w25qxx_spi_transmit((byte_address & 0xFF000000)
  //  >> 24); w25qxx_spi_transmit((byte_address & 0xFF0000) >> 16);
  //  w25qxx_spi_transmit((byte_address & 0xFF00) >> 8);
  //  w25qxx_spi_transmit(byte_address & 0xFF);
  w25qxx_spi_transmit(0);
  *buf = w25qxx_spi_receive();
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx ReadByte 0x%02X done after %lu ms", *buf, osKernelGetTickCount() - start_time);
#endif
  w25qxx.lock = 0;
}

void w25qxx_read_bytes(uint8_t *buf, uint32_t read_address, uint32_t bytes_to_read) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
#if (CATS_W25QXX_DEBUG == 1)
  uint32_t start_time = osKernelGetTickCount();
  log_debug("w25qxx ReadBytes at Address:%lu, %lu Bytes  begin...", read_address, bytes_to_read);
#endif
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x0C);
  w25qxx_send_address(read_address);
  //  if (w25qxx.id >= W25Q256) w25qxx_spi_transmit((read_address & 0xFF000000)
  //  >> 24); w25qxx_spi_transmit((read_address & 0xFF0000) >> 16);
  //  w25qxx_spi_transmit((read_address & 0xFF00) >> 8);
  //  w25qxx_spi_transmit(read_address & 0xFF);
  w25qxx_spi_transmit(0);
  HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, bytes_to_read, 2000);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
#if (CATS_W25QXX_DEBUG == 1)
  start_time = osKernelGetTickCount() - start_time;
  for (uint32_t i = 0; i < bytes_to_read; i++) {
    if ((i % 8 == 0) && (i > 2)) {
      log_rawr("\n");
      osDelay(10);
    }
    log_rawr("0x%02X,", buf[i]);
  }
  log_rawr("\n");
  log_debug("w25qxx ReadBytes done after %lu ms", start_time);
  osDelay(100);
#endif
  osDelay(1);
  w25qxx.lock = 0;
}

void w25qxx_read_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                      uint32_t NumByteToRead_up_to_PageSize) {
  while (w25qxx.lock == 1) osDelay(1);
  w25qxx.lock = 1;
  if ((NumByteToRead_up_to_PageSize > w25qxx.page_size) || (NumByteToRead_up_to_PageSize == 0))
    NumByteToRead_up_to_PageSize = w25qxx.page_size;
  if ((offset_in_bytes + NumByteToRead_up_to_PageSize) > w25qxx.page_size)
    NumByteToRead_up_to_PageSize = w25qxx.page_size - offset_in_bytes;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx ReadPage:%lu, Offset:%lu ,Read %lu Bytes, begin...", page_num, offset_in_bytes,
            NumByteToRead_up_to_PageSize);
  osDelay(100);
  uint32_t start_time = osKernelGetTickCount();
#endif
  page_num = page_num * w25qxx.page_size + offset_in_bytes;
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x0C);
  w25qxx_send_address(page_num);
  //  if (w25qxx.id >= W25Q256) w25qxx_spi_transmit((page_num & 0xFF000000) >>
  //  24); w25qxx_spi_transmit((page_num & 0xFF0000) >> 16);
  //  w25qxx_spi_transmit((page_num & 0xFF00) >> 8);
  //  w25qxx_spi_transmit(page_num & 0xFF);
  w25qxx_spi_transmit(0);
  HAL_SPI_Receive(&CATS_W25QXX_SPI, buf, NumByteToRead_up_to_PageSize, 100);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
#if (CATS_W25QXX_DEBUG == 1)
  start_time = osKernelGetTickCount() - start_time;
  for (uint32_t i = 0; i < NumByteToRead_up_to_PageSize; i++) {
    if ((i % 8 == 0) && (i > 2)) {
      log_rawr("\n");
      osDelay(10);
    }
    log_rawr("0x%02X,", buf[i]);
  }
  log_rawr("\n");
  log_debug("w25qxx ReadPage done after %lu ms", start_time);
  osDelay(100);
#endif
  // osDelay(1);
  w25qxx.lock = 0;
}

void w25qxx_read_sector(uint8_t *buf, uint32_t sector_num, uint32_t offset_in_bytes,
                        uint32_t bytes_to_read_up_to_sector_size) {
  if ((bytes_to_read_up_to_sector_size > w25qxx.sector_size) || (bytes_to_read_up_to_sector_size == 0))
    bytes_to_read_up_to_sector_size = w25qxx.sector_size;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx ReadSector:%lu, Offset:%lu ,Read %lu Bytes, begin...", sector_num, offset_in_bytes,
            bytes_to_read_up_to_sector_size);
  osDelay(100);
#endif
  if (offset_in_bytes >= w25qxx.sector_size) {
#if (CATS_W25QXX_DEBUG == 1)
    log_debug("---w25qxx ReadSector Faild!");
    osDelay(100);
#endif
    return;
  }
  uint32_t start_page;
  int32_t bytes_to_read;
  uint32_t local_offset;
  if ((offset_in_bytes + bytes_to_read_up_to_sector_size) > w25qxx.sector_size)
    bytes_to_read = w25qxx.sector_size - offset_in_bytes;
  else
    bytes_to_read = bytes_to_read_up_to_sector_size;
  start_page = w25qxx_sector_to_page(sector_num) + (offset_in_bytes / w25qxx.page_size);
  local_offset = offset_in_bytes % w25qxx.page_size;
  do {
    w25qxx_read_page(buf, start_page, local_offset, bytes_to_read);
    start_page++;
    bytes_to_read -= w25qxx.page_size - local_offset;
    buf += w25qxx.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_read > 0);
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("---w25qxx ReadSector Done");
  osDelay(100);
#endif
}

void w25qxx_read_block(uint8_t *buf, uint32_t block_num, uint32_t offset_in_bytes,
                       uint32_t bytes_to_read_up_to_block_size) {
  if ((bytes_to_read_up_to_block_size > w25qxx.block_size) || (bytes_to_read_up_to_block_size == 0))
    bytes_to_read_up_to_block_size = w25qxx.block_size;
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("w25qxx ReadBlock:%lu, Offset:%lu ,Read %lu Bytes, begin...", block_num, offset_in_bytes,
            bytes_to_read_up_to_block_size);
  osDelay(100);
#endif
  if (offset_in_bytes >= w25qxx.block_size) {
#if (CATS_W25QXX_DEBUG == 1)
    log_debug("w25qxx ReadBlock Faild!");
    osDelay(100);
#endif
    return;
  }
  uint32_t start_page;
  int32_t bytes_to_read;
  uint32_t local_offset;
  if ((offset_in_bytes + bytes_to_read_up_to_block_size) > w25qxx.block_size)
    bytes_to_read = w25qxx.block_size - offset_in_bytes;
  else
    bytes_to_read = bytes_to_read_up_to_block_size;
  start_page = w25qxx_block_to_page(block_num) + (offset_in_bytes / w25qxx.page_size);
  local_offset = offset_in_bytes % w25qxx.page_size;
  do {
    w25qxx_read_page(buf, start_page, local_offset, bytes_to_read);
    start_page++;
    bytes_to_read -= w25qxx.page_size - local_offset;
    buf += w25qxx.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_read > 0);
#if (CATS_W25QXX_DEBUG == 1)
  log_debug("---w25qxx ReadBlock Done");
  osDelay(100);
#endif
}

/** Private Function Definitions **/

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
  HAL_SPI_Transmit(&CATS_W25QXX_SPI, buf, sizeof(buf), 100);
}

static inline void w25qxx_spi_transmit(uint8_t data) { HAL_SPI_Transmit(&CATS_W25QXX_SPI, &data, 1, 100); }

static inline uint8_t w25qxx_spi_receive() {
  uint8_t ret;
  HAL_SPI_Receive(&CATS_W25QXX_SPI, &ret, 1, 100);
  return ret;
}

static inline uint8_t w25qxx_spi_transmit_receive(uint8_t data) {
  uint8_t ret;
  HAL_SPI_TransmitReceive(&CATS_W25QXX_SPI, &data, &ret, 1, 100);
  return ret;
}

static inline uint32_t w25qxx_read_id(void) {
  uint32_t temp0 = 0, temp1 = 0, temp2 = 0;
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  //(0x9F);
  w25qxx_spi_transmit(0x9F);
  temp0 = w25qxx_spi_receive();
  temp1 = w25qxx_spi_receive();
  temp2 = w25qxx_spi_receive();
  //  temp0 = w25qxx_spi_receive(W25QXX_DUMMY_BYTE);
  //  temp1 = w25qxx_spi_receive(W25QXX_DUMMY_BYTE);
  //  temp2 = w25qxx_spi_receive(W25QXX_DUMMY_BYTE);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  return (temp0 << 16) | (temp1 << 8) | temp2;
}

static inline void w25qxx_read_uniq_id(void) {
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x4B);
  for (int i = 0; i < 4; i++) w25qxx_spi_receive();
  for (int i = 0; i < 8; i++) w25qxx.uniq_id[i] = w25qxx_spi_receive();
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
}

/* TODO: run this at initialization */
static inline void w25qxx_write_enable(void) {
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x06);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  // osDelay(1);
}

static inline void w25qxx_write_disable(void) {
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x04);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  osDelay(1);
}

static inline uint8_t w25qxx_read_status_register(uint8_t status_register) {
  uint8_t status = 0;
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  if (status_register == 1) {
    w25qxx_spi_transmit(0x05);
    status = w25qxx_spi_receive();
    w25qxx.status_reg_1 = status;
  } else if (status_register == 2) {
    w25qxx_spi_transmit(0x35);
    status = w25qxx_spi_receive();
    w25qxx.status_reg_2 = status;
  } else {
    w25qxx_spi_transmit(0x15);
    status = w25qxx_spi_receive();
    w25qxx.status_reg_3 = status;
  }
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
  return status;
}

static inline void w25qxx_write_status_register(uint8_t status_register, uint8_t data) {
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  if (status_register == 1) {
    w25qxx_spi_transmit(0x01);
    w25qxx.status_reg_1 = data;
  } else if (status_register == 2) {
    w25qxx_spi_transmit(0x31);
    w25qxx.status_reg_2 = data;
  } else {
    w25qxx_spi_transmit(0x11);
    w25qxx.status_reg_3 = data;
  }
  w25qxx_spi_transmit(data);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
}

static inline void w25qxx_wait_for_write_end(void) {
  // osDelay(1);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_RESET);
  w25qxx_spi_transmit(0x05);
  do {
    w25qxx.status_reg_1 = w25qxx_spi_receive();
    osDelay(1);
  } while ((w25qxx.status_reg_1 & 0x01) == 0x01);
  HAL_GPIO_WritePin(CATS_W25QXX_CS_GPIO, CATS_W25QXX_CS_PIN, GPIO_PIN_SET);
}
