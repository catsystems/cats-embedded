#pragma once

#include <stdbool.h>
#include <unity.h>
#include <Crc.hpp>

void test_crc_8() {
  uint8_t buffer[10];
  buffer[0] = 0x33;
  uint8_t result = crc8(&buffer[0U], 1U);
  TEST_ASSERT_EQUAL_CHAR(0x96, result);

  buffer[0] = 0xFF;
  result = crc8(&buffer[0U], 1U);
  TEST_ASSERT_EQUAL_CHAR(0xAC, result);

  buffer[0] = 0x00;
  buffer[1] = 0x10;
  buffer[2] = 0x20;
  buffer[3] = 0x30;
  result = crc8(&buffer[0U], 4U);
  TEST_ASSERT_EQUAL_CHAR(0xBD, result);

  buffer[4] = 0x40;
  buffer[5] = 0x50;
  buffer[6] = 0x60;
  buffer[7] = 0x70;
  buffer[8] = 0x80;
  buffer[9] = 0x90;
  result = crc8(&buffer[0U], 10U);
  TEST_ASSERT_EQUAL_CHAR(0xDC, result);

  // constant_1 = random_data1 + crc(random_data1)
  // constant_2 = random_data2 + crc(random_data2)
  // constant_1 == constant_2

  buffer[0] = 0x55;
  buffer[1] = 0x66;
  buffer[2] = crc8(&buffer[0], 2U);
  result = crc8(&buffer[0], 3U);

  buffer[0] = 0x11;
  buffer[1] = 0x22;
  buffer[2] = crc8(&buffer[0], 2U);
  uint8_t result2 = crc8(&buffer[0], 3U);
  TEST_ASSERT_EQUAL_CHAR(result2, result);

  buffer[0] = 0x55;
  buffer[1] = 0x66;
  buffer[2] = 0x77;
  buffer[3] = 0x88;
  buffer[4] = crc8(&buffer[0], 4U);
  result = crc8(&buffer[0], 5U);

  buffer[0] = 0x11;
  buffer[1] = 0x22;
  buffer[2] = 0xAC;
  buffer[3] = 0xF6;
  buffer[4] = crc8(&buffer[0], 4U);
  result2 = crc8(&buffer[0], 5U);
  TEST_ASSERT_EQUAL_CHAR(result2, result);
}

void test_crc_32() {
  uint8_t buffer[10];
  buffer[0] = 0x11;
  uint32_t result = crc32(&buffer[0U], 1U);
  TEST_ASSERT_EQUAL_UINT32(3098726271U, result);

  buffer[0] = 0xFF;
  result = crc32(&buffer[0U], 1U);
  TEST_ASSERT_EQUAL_UINT32(4278190080U, result);

  buffer[0] = 0x00;
  buffer[1] = 0x10;
  buffer[2] = 0x20;
  buffer[3] = 0x30;
  result = crc32(&buffer[0U], 4U);
  TEST_ASSERT_EQUAL_UINT32(2386520162U, result);

  buffer[4] = 0x40;
  buffer[5] = 0x50;
  buffer[6] = 0x60;
  buffer[7] = 0x70;
  buffer[8] = 0x80;
  buffer[9] = 0x90;
  result = crc32(&buffer[0U], 10U);
  TEST_ASSERT_EQUAL_UINT32(2370813176U, result);

  buffer[0] = 0x01;
  result = crc32(&buffer[0U], 10U);
  TEST_ASSERT_EQUAL_UINT32(1653462982U, result);
}