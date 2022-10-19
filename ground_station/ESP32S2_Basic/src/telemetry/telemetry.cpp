#include "telemetry/telemetry.h"
#include "crc.h"
#include "console.h"

#define TASK_TELE_FREQ 100

void Telemetry::begin(){
    serial.begin(115200, SERIAL_8N1, rxPin, txPin);
    parser.init(&data, &info, &location, &time);
    initialized = true;

    armingMsg.d1 = 0xAA;
    armingMsg.d2 = 0xBB;
    armingMsg.d3 = 0xCC;

    xTaskCreate(update, "task_telemetry", 2048, this, 1, NULL);
}

void Telemetry::setLinkPhrase(uint8_t* phrase, uint32_t length){
    memset(linkPhrase, 0, 8);
    memcpy(linkPhrase, phrase, length);
    newSetting = true;
}

void Telemetry::setLinkPhrase(String phrase){
    uint32_t length = phrase.length();
    memset(linkPhrase, 0, 8);
    memcpy(linkPhrase, phrase.c_str(), length);
    newSetting = true;
}

void Telemetry::setDirection(transmission_direction_e dir){
    if(dir != transmissionDirection){
        transmissionDirection = dir;
        newSetting = true;
    }
}

void Telemetry::setMode(transmission_mode_e mode){
    if(mode != transmissionMode){
        transmissionMode = mode;
        newSetting = true;
    }
}

void Telemetry::initLink(){
    if(linkInitialized){
        sendDisable();
    }

    linkInitialized = true;
    
    vTaskDelay(100);
    sendSetting(CMD_DIRECTION, transmissionDirection);
    vTaskDelay(100);
    sendSetting(CMD_MODE, transmissionMode);
    vTaskDelay(100);
    sendSetting(CMD_PA_GAIN, 0);
    vTaskDelay(100);

    if(linkPhrase[0] != 0){
        sendLinkPhrase(linkPhrase, 8);
        vTaskDelay(100);
        sendEnable();
        console.warning.println("[TELE] Link Enabled");
    }
}

void Telemetry::arm(){
    sendTXPayload((uint8_t*)&armingMsg, 16);
    vTaskDelay(50);
    setMode(BIDIRECTIONAL);
    rocketArmed = false;
    arming = true;
}

void Telemetry::update(void *pvParameter){
    Telemetry* ref = (Telemetry*)pvParameter;

    while(ref->initialized){
        TickType_t task_last_tick = xTaskGetTickCount();

        if(ref->newSetting){
            ref->newSetting = false;
            ref->initLink();
        }

        if(ref->arming){
            if(ref->data.isUpdated()){
                if(ref->data.state() > 0){
                    ref->arming = false;
                    ref->rocketArmed = true;
                    ref->setMode(UNIDIRECTIONAL);
                }
            }
        }
        
        while(ref->serial.available()){
           ref->parser.process(ref->serial.read()); 
        }

        vTaskDelayUntil(&task_last_tick, (const TickType_t) 1000 / TASK_TELE_FREQ);
    }
}

void Telemetry::sendLinkPhrase(uint8_t* phrase, uint32_t length){
  uint8_t out[11]; // 1 OP + 1 LEN + 8 DATA + 1 CRC
  out[0] = CMD_LINK_PHRASE;
  out[1] = (uint8_t)length;
  memcpy(&out[2], phrase, length);
  out[length+2] = crc8(out, length+2);

  serial.write(out, length+3);
}

void Telemetry::sendSetting(uint8_t command, uint8_t value){
    uint8_t out[4]; // 1 OP + 1 LEN + 1 DATA + 1 CRC
    out[0] = command;
    out[1] = 1;
    out[2] = value;
    out[3] = crc8(out, 3);

    serial.write(out, 4);
}

void Telemetry::sendEnable(){
    uint8_t out[3]; // 1 OP + 1 LEN + 1 DATA + 1 CRC
    out[0] = CMD_ENABLE;
    out[1] = 0;
    out[2] = crc8(out, 2);

    serial.write(out, 3);
}

void Telemetry::sendDisable(){
    uint8_t out[3]; // 1 OP + 1 LEN + 1 DATA + 1 CRC
    out[0] = CMD_DISBALE;
    out[1] = 0;
    out[2] = crc8(out, 2);

    serial.write(out, 3);
}

void Telemetry::sendTXPayload(uint8_t* payload, uint32_t length){
    uint8_t out[19]; // 1 OP + 1 LEN + 16 DATA + 1 CRC
    out[0] = CMD_TX;
    out[1] = (uint8_t)length;
    memcpy(&out[2], payload, length);
    out[length+2] = crc8(out, length+2);

    serial.write(out, length+3);
}



