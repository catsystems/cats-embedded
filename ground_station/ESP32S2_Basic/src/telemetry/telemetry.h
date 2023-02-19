#pragma once
#include <Arduino.h>
#include "telemetry_reg.h"
#include "parser.h"
#include "telemetryData.h"

class Telemetry {
    public:
        Telemetry(HardwareSerial& serial, int rxPin, int txPin) : serial(serial), rxPin(rxPin), txPin(txPin){}
        void begin();

        void setLinkPhrase(uint8_t* phrase, uint32_t length);
        void setLinkPhrase(String phrase);

        void setTestingPhrase(uint8_t* phrase, uint32_t length);
        void setTestingPhrase(String phrase);

        void setDirection(transmission_direction_e dir);
        void setMode(transmission_mode_e mode);

        void sendTXPayload(uint8_t* payload, uint32_t length);
        
        void exitTesting();
        void enterTesting();
        void triggerEvent(uint8_t event);

        void disable() {
            if(linkInitialized) {
                sendDisable();
                linkInitialized = false;
            }
        }

        void enable() {
            if(!linkInitialized) {
                sendEnable();
                linkInitialized = true;
            }
        }

        TelemetryData data;
        TelemetryInfo info;
        TelemetryLocation location;
        TelemetryTime time;
    
    private:
        void initLink();

        void sendLinkPhraseCrc(uint32_t crc, uint32_t length);
        void sendSetting(uint8_t command, uint8_t value);
        void sendEnable();
        void sendDisable();
        

        static void update (void *pvParameter);

        volatile bool initialized = false;
        volatile bool linkInitialized = false;

        HardwareSerial serial;
        
        Parser parser;
        int txPin;
        int rxPin;

        uint8_t linkPhrase[8] = {};
        uint8_t testingPhrase[8] = {};
        uint32_t testingCrc = 0;

        bool requestExitTesting = false;
        bool triggerAction = false;
        uint32_t triggerActionStart = 0;
        bool newSetting = false;
        transmission_direction_e transmissionDirection = RX_DIR;
        transmission_mode_e transmissionMode = UNIDIRECTIONAL;

        struct packed_testing_msg_t {
            uint8_t header;
            uint32_t passcode;
            uint8_t event;
            uint8_t enable_pyros;
            uint32_t dummy1;
            uint32_t dummy2;
        } __attribute__((packed)) testingMsg;
        
};