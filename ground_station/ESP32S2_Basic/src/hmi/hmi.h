#pragma once
#include <Arduino.h>
#include "JC_Button.h"
#include "window.h"
#include "logging/recorder.h"


class Hmi {
    public:
        Hmi(const char* dir) : recorder(dir),  
                upButton(4), 
                downButton(1), 
                leftButton(3), 
                rightButton(5), 
                centerButton(2), 
                okButton(6), 
                backButton(7) {}

        void begin();

    private:
        enum State{
        MENU = 0,
        LIVE = 1,
        RECOVERY = 2,
        SETTINGS = 3,
        };


        State state = MENU;
        Recorder recorder;

        uint32_t settingSubMenu = 0;
        int32_t settingIndex = -1;
        char keyboardString[9] = {};

        static void update (void *pvParameter);

        void fsm();
        void initMenu();
        void menu();
        void initLive();
        void live();
        void initRecovery();
        void recovery();
        void initSettings();
        void settings();

        bool initialized = false;
        bool isLogging = false;
        bool boxWindow = false;
        uint32_t armChannel = 0;

        Button upButton;
        Button downButton;
        Button leftButton;
        Button rightButton;
        Button centerButton;

        Button okButton;
        Button backButton;

        Window window;

        uint32_t menuIndex = 0;

};