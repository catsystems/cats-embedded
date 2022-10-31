
#include "hmi.h"
#include "console.h"
#include "telemetry/telemetry.h"
#include "navigation.h"
#include <timeLib.h>

extern Telemetry link1;
extern Telemetry link2;

extern Navigation navigation;

void Hmi::begin(){
    upButton.begin();
    downButton.begin();
    leftButton.begin();
    rightButton.begin();
    centerButton.begin();

    okButton.begin();
    backButton.begin();

    recorder.begin();
    recorder.enable();

    window.begin();
    initialized = true;
    xTaskCreate(update, "task_hmi", 8196, this, 1, NULL);
}

void Hmi::fsm(){
    switch(state){
        case MENU:
            menu();
        break;

        case LIVE:
            live();
        break;

        case RECOVERY:
            recovery();
        break;

        case SETTINGS:
            settings();
        break;
    }
}

/* MENU */

void Hmi::initMenu(){
    window.initMenu(menuIndex);
}

void Hmi::menu(){
    uint32_t oldIndex = menuIndex;
    if(rightButton.wasPressed() && menuIndex < 2){
        menuIndex++;
    }
    
    if(leftButton.wasPressed() && menuIndex > 0) {
        menuIndex--;
    }

    if(menuIndex != oldIndex){
        window.updateMenu(menuIndex);
    }

    if(okButton.wasPressed() || centerButton.wasPressed()){
        state = (State)(menuIndex + 1);
        if(state == LIVE) {
            initLive();
        } else if(state == RECOVERY){
            initRecovery();
        } else {
            initSettings();
        }
    }
}

/* LIVE */

void Hmi::initLive(){
    window.initLive();
}

void Hmi::live(){

    if(boxWindow){
        /* Arming Window Mode */
        bool exit = false;

        if(armChannel == 1){
            exit = link1.isArmed();
        } else {
            exit = link2.isArmed();
        }

        if(backButton.wasPressed() || exit){
            window.initLive();
            boxWindow = false;
        }

        if(okButton.wasPressed()){
            if(armChannel == 1){
                link1.arm();
            } else {
                link2.arm();
            }
        }
    } else {
        /* Normal Mode */
        if(link1.data.isUpdated() && link1.info.isUpdated()){
        window.updateLive(&link1.data, &link1.info, 0);
        } else if (link1.info.isUpdated()){
            window.updateLive(&link1.info, 0);
        }

        if(link2.data.isUpdated() && link2.info.isUpdated()){
            if(link2.data.state() > 1){
                recorder.record(&link2.data.rxData);
                isLogging = true;
            } else {
                isLogging = false;
            }
            window.updateLive(&link2.data, &link2.info, 1);
        } else if (link2.info.isUpdated()){
            window.updateLive(&link2.info, 1);
        }

        if(backButton.wasPressed()){
            state = MENU;
            window.initMenu(menuIndex);
        }

        if(leftButton.pressedFor(3000) && !link1.data.state()){
        window.initBox("ARM CH1?");
        boxWindow = true;
        armChannel = 1;
        }

        if(rightButton.pressedFor(3000) && !link2.data.state()){
            window.initBox("ARM CH2?");
            boxWindow = true;
            armChannel = 2;
        }
    }    

}

/* RECOVERY */

void Hmi::initRecovery(){
    window.initRecovery();

    
}

void Hmi::recovery(){
    
    EarthPoint3D a;
    EarthPoint3D b;

    a = navigation.getPointA();
    b = navigation.getPointB();
    if(a.lat && a.lon && b.lat && b.lon){
        //window.updateRecovery(a, b, navigation.getHorizontalDistance(), navigation.getBearing());
        
    } else {
        //window.updateRecovery(a, b);
    }

    if(navigation.isUpdated()){
        window.updateRecovery(&navigation);
    }
    
    if(backButton.wasPressed()){
        state = MENU;
        window.initMenu(menuIndex);
    }
}

/* SETTINGS */

void Hmi::initSettings(){
    settingSubMenu = 0;
    settingIndex = -1;
    window.initSettings(settingSubMenu);
}

void Hmi::settings(){
    static bool keyboardActive = false;
    static int32_t i = 0;
    if(keyboardActive){
        if(rightButton.wasPressed() || rightButton.pressedFor(500)){
            
            if(i != 9 && i != 19 && i != 28 && i != 37) {
                i++;
                window.updateKeyboard(keyboardString, i);
            }   
        }
        if(leftButton.wasPressed() || leftButton.pressedFor(500)){
            if(i != -1 && i != 0 && i != 10 && i != 20 && i != 29) {
                i--;
                window.updateKeyboard(keyboardString, i);
            }   
        }
        if(downButton.wasPressed() || downButton.pressedFor(500)){
            if (i == -1){
                i = 7;
                window.updateKeyboard(keyboardString, i);
            } else if (i < 15){
                i += 10;
                window.updateKeyboard(keyboardString, i);
            } else if (i < 29){
                i += 9;
                window.updateKeyboard(keyboardString, i);
            }  
        }
        if(upButton.wasPressed() || upButton.pressedFor(500)){
            if(i > 9){
                if(i < 25){
                    i -= 10;
                    window.updateKeyboard(keyboardString, i);
                } else if (i < 38){
                    i -= 9;
                    window.updateKeyboard(keyboardString, i);
                }
            } else {
                i = -1;
                window.updateKeyboard(keyboardString, i);
            } 
        }
        if(okButton.wasPressed() || okButton.pressedFor(500)){
            if(i == 29) { // shift
                window.updateKeyboard(keyboardString, i, true);
            } else if (i == 37) { // enter
                memcpy((char*)settingsTable[settingSubMenu][settingIndex].dataPtr, keyboardString, 8);
                window.initSettings(settingSubMenu);
                window.updateSettings(settingIndex);
                keyboardActive = false;
            } else {
                window.updateKeyboard(keyboardString, i, true);
            }
        }

        if(backButton.wasPressed()){
            window.initSettings(settingSubMenu);
            window.updateSettings(settingIndex);
            keyboardActive = false;
        }


    } else {
        if(settingIndex == -1){
        if(rightButton.wasPressed() && settingSubMenu < 1){
        settingSubMenu++;
        window.initSettings(settingSubMenu);
        }

        if(leftButton.wasPressed() && settingSubMenu > 0){
            settingSubMenu--;
            window.initSettings(settingSubMenu);
        }
        } else {

            if(settingsTable[settingSubMenu][settingIndex].type == NUMBER){
                if((rightButton.wasPressed() || rightButton.pressedFor(500)) && \
                    *(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr < \
                    settingsTable[settingSubMenu][settingIndex].config.minmax.max) {
                        (*(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr)++;
                        window.updateSettings(settingIndex);
                }
                if((leftButton.wasPressed() || leftButton.pressedFor(500)) && \
                    *(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr > \
                    settingsTable[settingSubMenu][settingIndex].config.minmax.min) {
                        (*(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr)--;
                        window.updateSettings(settingIndex);
                }
            }

            if(settingsTable[settingSubMenu][settingIndex].type == TOGGLE){
                if(rightButton.wasPressed() && *(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr == 0) {
                    (*(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr) = 1;
                    window.updateSettings(settingIndex);
                }
                if(leftButton.wasPressed() && *(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr == 1) {
                    (*(int16_t*)settingsTable[settingSubMenu][settingIndex].dataPtr) = 0;
                    window.updateSettings(settingIndex);
                }
            }

            if(settingsTable[settingSubMenu][settingIndex].type == STRING){
                if(okButton.wasPressed()){
                    memcpy(keyboardString, (char*)settingsTable[settingSubMenu][settingIndex].dataPtr, 8);
                    
                    window.initKeyboard(keyboardString, settingsTable[settingSubMenu][settingIndex].config.stringLength);
                    keyboardActive = true;
                }
            }
        }

        if(downButton.wasPressed() && settingIndex < settingsTableValueCount[settingSubMenu]-1){
            settingIndex++;
            window.updateSettings(settingIndex);
        }

        if(upButton.wasPressed() && settingIndex > -1){
            settingIndex--;
            window.updateSettings(settingIndex);
        }

        

        if(backButton.wasPressed()){
            state = MENU;
            window.initMenu(menuIndex);
        }
    }
    
}

void Hmi::update(void *pvParameter){
    Hmi* ref = (Hmi*)pvParameter;

    ref->window.logo();

    vTaskDelay(2000);

    int oldUsbStatus = 0;
    ref->window.initBar();
    ref->initMenu();

    uint32_t barUpdate = millis();
    bool timeValid = false;

    while(ref->initialized){
        TickType_t task_last_tick = xTaskGetTickCount();

        ref->fsm();

        if(link1.data.isUpdated()){
            //ref->window.updateBar(link1.data.ts());
        }

        if(millis() - barUpdate >= 1000){
            barUpdate = millis();
            float voltage = analogRead(18)*0.00059154929;
            if(link2.time.isUpdated()){
                setTime(link2.time.hour(), link2.time.minute(), link2.time.second(),0,0,0);
                adjustTime(systemConfig.config.timeZoneOffset * 3600);
                timeValid = true;
            }
            ref->window.updateBar(voltage, digitalRead(21), ref->isLogging, link2.location.isValid(), timeValid);
        }

        
        
        
        ref->upButton.read();
        ref->downButton.read();
        ref->leftButton.read();
        ref->rightButton.read();
        ref->centerButton.read();

        ref->okButton.read();
        ref->backButton.read();
        vTaskDelayUntil(&task_last_tick, (const TickType_t) 1000 / 50);
    }
}