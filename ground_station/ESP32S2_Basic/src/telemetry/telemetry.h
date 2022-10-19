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

        void setDirection(transmission_direction_e dir);
        void setMode(transmission_mode_e mode);

        void sendTXPayload(uint8_t* payload, uint32_t length);
        void arm();

        bool isArmed() { return rocketArmed; }

        TelemetryData data;
        TelemetryInfo info;
        TelemetryLocation location;
        TelemetryTime time;
    
    private:
        void initLink();

        void sendLinkPhrase(uint8_t* phrase, uint32_t length);
        void sendSetting(uint8_t command, uint8_t value);
        void sendEnable();
        void sendDisable();
        

        static void update (void *pvParameter);

        volatile bool initialized = false;
        volatile bool linkInitialized = false;

        bool arming = false;
        bool rocketArmed = false;

        HardwareSerial serial;
        
        Parser parser;
        int txPin;
        int rxPin;

        packedRXMessage armingMsg;

        uint8_t linkPhrase[8] = {};
        bool newSetting = false;
        transmission_direction_e transmissionDirection = RX_DIR;
        transmission_mode_e transmissionMode = UNIDIRECTIONAL;

        
};