#ifdef NATIVE

#include "tests/TestCrc.h"

#include <unity.h>

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_8);
  RUN_TEST(test_crc_32);
  UNITY_END();
}

#endif