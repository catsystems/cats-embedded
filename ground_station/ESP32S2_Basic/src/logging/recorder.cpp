
#include "recorder.h"

bool Recorder::begin(){

    char fileName [30];
    int32_t number = 0;

    if(!fatfs.chdir(directory)){
        console.error.print("[REC] Open directory failed"); console.error.println(directory);
        fatfs.mkdir(&directory[1]);
        console.log.println("[REC] Crating directory");
        if(!fatfs.chdir(directory)){
            console.error.println("[REC] Open directory failed");
            return false;
        }
    }

    do{
        snprintf(fileName, 30, "log_%03d.csv", number);
        number++;
    } while(fatfs.exists(fileName));

    file = fatfs.open(fileName, FILE_WRITE);
    console.log.println(fileName);
    if(!file)
    {
        console.error.println("[REC] Open file failed");
        return false;
    }

    file.println("ts,state,errors,lat,lon,altitude,velocity,battery,pyro1,pyro2");

    queue = xQueueCreate(10, sizeof(packedRXMessage));
    xTaskCreate(recordTask, "task_recorder", 4096, this, 1, NULL);
    initialized = true;
    return initialized;
}

void Recorder::recordTask(void* pvParameter){
    Recorder* ref = (Recorder*)pvParameter;
    char line [128];
    uint32_t count = 0;
    packedRXMessage element;
    while(ref->initialized){
        if(xQueueReceive(ref->queue, &element, portMAX_DELAY) == pdPASS){
            snprintf(line, 128, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d", element.timestamp, element.state, element.errors,
            element.lat, element.lon, element.altitude, element.velocity, element.voltage, (bool)(element.pyro_continuity&0x01), (bool)(element.pyro_continuity & 0x02));
            ref->file.println(line);
            count++;

            if(count == 10){
                count = 0;
                ref->file.sync();
            }
        }
    }
    vTaskDelete(NULL);
}