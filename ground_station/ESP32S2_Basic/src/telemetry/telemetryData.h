#pragma once

#include <Arduino.h>
#include "console.h"

typedef struct {
  uint8_t state : 3;
  uint8_t errors : 4;
  uint16_t timestamp : 15;
  int32_t lat : 22;
  int32_t lon : 22;
  int32_t altitude : 17;
  int16_t velocity : 10;
  uint16_t voltage : 8;
  uint16_t continuity : 3;
  // fill up to 16 bytes
  uint8_t : 0;  // sent
  uint8_t d1;   // dummy
  uint8_t d2;   // dummy
  uint8_t d3;   // dummy
} __attribute__((packed)) packedRXMessage;


class TelemetryData {
    public:
        void commit(uint8_t* data, uint32_t length){
            memcpy(&rxData, data, length);
            lastCommitTime = millis();
            updated = true;
        }

        bool isUpdated() const {
            
            return updated; 
        }

        int16_t velocity(){
            updated = false;
            return rxData.velocity;
        }

        int32_t altitude(){
            updated = false;
            return rxData.altitude;
        }

        uint16_t ts(){
            updated = false;
            return rxData.timestamp;
        }

        float lat(){
            updated = false;
            return (float) rxData.lat / 10000.0f;
        }

        float lon(){
            updated = false;
            return (float) rxData.lon / 10000.0f;
        }

        int8_t d1(){
            return rxData.d1;
        }

        uint16_t state(){
            updated = false;
            return rxData.state;
        }

        packedRXMessage rxData;

    
    private:
        bool updated;
        uint32_t lastCommitTime;        

};

typedef struct {
    uint8_t lq;
    int8_t rssi;
    int8_t snr;
} __attribute__((packed)) TelemetryInfoData;

class TelemetryInfo {
    public:
        void commit(uint8_t* data, uint32_t length){
            memcpy(&infoData, data, sizeof(infoData));
            lastCommitTime = millis();
            updated = true;
        }

        bool isUpdated() const {
            return updated; 
        }

        int16_t snr(){
            updated = false;
            return (int16_t)infoData.snr;
        }

        int16_t rssi(){
            updated = false;
            return (int16_t)infoData.rssi;
        }

        uint16_t lq(){
            updated = false;
            return (uint16_t)infoData.lq;
        }


    private:
        TelemetryInfoData infoData;
        uint32_t lastCommitTime;
        bool updated;
};

typedef struct {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
} __attribute__((packed)) TelemetryTimeData;

class TelemetryTime {
    public:
        void commit(uint8_t* data, uint32_t length){
            memcpy(&timeData, data, sizeof(TelemetryTimeData));
            lastCommitTime = millis();
            updated = true;
            wasUpdated = true;
        }

        bool isUpdated() const {
            return updated; 
        }

        uint8_t second() {
            updated = false;
            return timeData.second;
        }

        uint8_t minute() {
            updated = false;
            return timeData.minute;
        }

        uint8_t hour() {
            updated = false;
            return timeData.hour;
        }


    private:
        TelemetryTimeData timeData;
        uint32_t lastCommitTime;
        bool updated;
        bool wasUpdated = false;
};

typedef struct {
    float lat;
    float lon;
    int32_t alt;
} __attribute__((packed)) TelemetryLocationData;

class TelemetryLocation {
    public:
        void commit(uint8_t* data, uint32_t length){
            memcpy(&locationData, data, sizeof(TelemetryLocationData));
            lastCommitTime = millis();
            updated = true;
            wasUpdated = true;
        }

        bool isUpdated() const {
            return updated; 
        }

        bool isValid() const {
            if(locationData.lat && locationData.lon && wasUpdated){
                return true;
            } 
            return false;
        }

        float lat(){
            updated = false;
            return locationData.lat;
        }

        float lon(){
            updated = false;
            return locationData.lon;
        }

        int16_t alt(){
            updated = false;
            return (uint16_t)locationData.alt;
        }

    private:
        TelemetryLocationData locationData;
        uint32_t lastCommitTime;
        bool updated;
        bool wasUpdated = false;
};
