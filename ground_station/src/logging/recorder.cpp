
#include "recorder.hpp"

bool Recorder::begin() {
  int32_t number = 0;

  if (!fatfs.chdir(directory)) {
    console.error.print("[REC] Open directory failed");
    console.error.println(directory);
    fatfs.mkdir(&directory[1]);
    console.log.println("[REC] Crating directory");
    if (!fatfs.chdir(directory)) {
      console.error.println("[REC] Open directory failed");
      return false;
    }
  }

  do {
    snprintf(fileName, 30, "log_%03ld.csv", number);
    number++;
  } while (fatfs.exists(fileName));

  queue = xQueueCreate(64, sizeof(RecorderElement));
  xTaskCreate(recordTask, "task_recorder", 4096, this, 1, nullptr);
  initialized = true;
  return initialized;
}

void Recorder::createFile() {
  file = fatfs.open(fileName, FILE_WRITE);
  console.log.println(fileName);
  if (!file) {
    console.error.println("[REC] Open file failed");
    return;
  }
  fileCreated = true;
  file.println(
      "link,ts[deciseconds],state,errors,lat[deg/10000],lon[deg/10000],altitude[m],velocity[m/"
      "s],battery[decivolts],pyro1,pyro2");
}

void Recorder::recordTask(void *pvParameter) {
  auto *ref = static_cast<Recorder *>(pvParameter);
  char line[128];
  uint32_t count = 0;
  RecorderElement element{};
  while (ref->initialized) {
    if (xQueueReceive(ref->queue, &element, portMAX_DELAY) == pdPASS) {
      if (!ref->fileCreated) {
        ref->createFile();
      }
      const auto &data = element.data;
      snprintf(line, 128, "%hu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", element.source, data.timestamp, data.state, data.errors,
               data.lat, data.lon, data.altitude, data.velocity, data.voltage,
               static_cast<bool>(data.pyro_continuity & 0x01), static_cast<bool>(data.pyro_continuity & 0x02));
      ref->file.println(line);
      count++;

      if (count == 10) {
        count = 0;
        ref->file.sync();
      }
    }
  }
  vTaskDelete(nullptr);
}
