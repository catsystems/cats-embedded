//
// Created by stoja on 20.12.20.
//

#include "cmsis_os.h"
#include "usb_device.h"
#include "util.h"
#include "drivers/w25qxx.h"

void vTaskInit(void* argument) {
  osDelay(3000);
  #if (configUSE_TRACE_FACILITY == 1)
    vTraceEnable(TRC_START_AWAIT_HOST);
    HAL_GPIO_TogglePin(GPIOC, LED_STATUS_Pin);
  #endif
  //  osDelay(5000);
  //  W25qxx_Init();
  //  UsbPrint("Deleting everything from sector 1...");
  //  W25qxx_EraseSector(1);
  //  /* Infinite loop */
  //  UsbPrint("Deleting done\n");
  //  uint8_t* write_buf = calloc(256, sizeof(uint8_t));  //{ 0 };
  //  uint8_t* read_buf = calloc(256, sizeof(uint8_t));   //{ 0 };
  //  uint32_t i = 16;
  //  for (uint16_t j = 0; j < 256; j++) {
  //    write_buf[j] = 255 - j;
  //  }
  for (;;) {
    //    W25qxx_WritePage(write_buf, i, 0, 256);
    //    W25qxx_ReadPage(read_buf, i, 0, 256);
    //    UsbPrint("Read Buffer, page = %d\n", i);
    //    for (uint16_t j = 0; j < 256; j++) {
    //      UsbPrint("%d, ", read_buf[j]);
    //      read_buf[j] = 0;
    //    }
    //    UsbPrint("\n");
    //    ++i;
    osDelay(100);
  }
  /* USER CODE END 5 */
}
