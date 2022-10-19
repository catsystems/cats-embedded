#pragma once

typedef enum {
    DIVERSITY = 0,
    DUAL = 1
} ReceiverTelemetryMode_e;

typedef struct {
    int16_t timeZoneOffset;
    bool neverStopLogging;

    ReceiverTelemetryMode_e receiverMode;
    char linkPhrase1[9];
    char linkPhrase2[9];
} systemConfig_t;

class Config{
    public:
        Config() {
            snprintf(config.linkPhrase1, 8, "cats1");
            snprintf(config.linkPhrase2, 8, "cats2");
            config.neverStopLogging = false;
            config.receiverMode = DIVERSITY;
            config.timeZoneOffset = 1;
        }
        systemConfig_t config;
    private:

};

extern Config systemConfig;