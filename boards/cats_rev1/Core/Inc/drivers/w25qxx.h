/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw

  Version:    1.1.1


  Reversion History:

  (1.1.1)
  Fix some errors.

  (1.1.0)
  Fix some errors.

  (1.0.0)
  First release.
*/

#ifndef CATS_W25QXX_H
#define CATS_W25QXX_H

#include <stdbool.h>
#include <stdint.h>

/** Exported Types **/

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

} w25qxx_id_t;

typedef struct {
  w25qxx_id_t id;
  uint8_t uniq_id[8];
  uint16_t page_size;
  uint32_t page_count;
  uint32_t sector_size;
  uint32_t sector_count;
  uint32_t block_size;
  uint32_t block_count;
  uint32_t capacity_in_kilobytes;
  uint8_t status_reg_1;
  uint8_t status_reg_2;
  uint8_t status_reg_3;
  uint8_t lock;
} w25qxx_t;

/** Exported Functions **/

bool w25qxx_init(void);

void w25qxx_erase_chip(void);
void w25qxx_erase_sector(uint32_t sector_num);
void w25qxx_erase_block(uint32_t block_num);

uint32_t w25qxx_page_to_sector(uint32_t page_num);
uint32_t w25qxx_page_to_block(uint32_t page_num);
uint32_t w25qxx_sector_to_block(uint32_t sector_num);
uint32_t w25qxx_sector_to_page(uint32_t sector_num);
uint32_t w25qxx_block_to_page(uint32_t block_num);

bool w25qxx_is_empty_page(uint32_t page_num, uint32_t offset_in_bytes,
                          uint32_t bytes_to_check_up_to_page_size);
bool w25qxx_is_empty_sector(uint32_t sector_num, uint32_t offset_in_bytes,
                            uint32_t bytes_to_check_up_to_sector_size);
bool w25qxx_is_empty_block(uint32_t block_num, uint32_t offset_in_bytes,
                           uint32_t bytes_to_check_up_to_block_size);

void w25qxx_write_byte(uint8_t byte, uint32_t byte_address);
void w25qxx_write_page(uint8_t *buf, uint32_t page_num,
                       uint32_t offset_in_bytes,
                       uint32_t bytes_to_write_up_to_page_size);
void w25qxx_write_sector(uint8_t *buf, uint32_t sector_num,
                         uint32_t offset_in_bytes,
                         uint32_t bytes_to_write_up_to_sector_size);
void w25qxx_write_block(uint8_t *buf, uint32_t block_num,
                        uint32_t offset_in_bytes,
                        uint32_t bytes_to_write_up_to_block_size);

void w25qxx_read_byte(uint8_t *buf, uint32_t byte_address);
void w25qxx_read_bytes(uint8_t *buf, uint32_t read_address,
                       uint32_t bytes_to_read);
void w25qxx_read_page(uint8_t *buf, uint32_t page_num, uint32_t offset_in_bytes,
                      uint32_t bytes_to_read_up_to_page_size);
void w25qxx_read_sector(uint8_t *buf, uint32_t sector_num,
                        uint32_t offset_in_bytes,
                        uint32_t bytes_to_read_up_to_sector_size);
void w25qxx_read_block(uint8_t *buf, uint32_t block_num,
                       uint32_t offset_in_bytes,
                       uint32_t bytes_to_read_up_to_block_size);

/** Externs **/

extern w25qxx_t w25qxx;

#endif
