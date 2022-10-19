#pragma once

#include <Arduino.h>
#include "telemetry/telemetryData.h"
#include "utils.h"

class Recorder {
    public:
        Recorder(const char* directory) : directory(directory) {}
        bool begin();

        void enable(){
            enabled = true;
        }

        void disable(){
            enabled = false;
        }

        void record(packedRXMessage* data){
            if(enabled){
                xQueueSend(queue, data, 0);
            }
        }      

    private:
        bool initialized = false;
        bool enabled = false;

        const char* directory; 

        QueueHandle_t queue;
        File file;

        static void recordTask (void* pvParameter);
};