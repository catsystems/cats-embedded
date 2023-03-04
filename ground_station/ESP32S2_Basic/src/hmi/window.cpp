#include "bmp.h"
#include "window.h"
#include <TimeLib.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>


void Window::begin(){
    display.begin();
    display.clearDisplay();
    display.setRotation(0);
}

void Window::logo(){
    display.drawBitmap(140,20, cats_logo, 120, 200, BLACK);
    display.refresh();
}

void Window::drawCentreString(const char *buf, int x, int y){
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, 0, y, &x1, &y1, &w, &h); //calc width of new string
    display.setCursor(x - w / 2, y);
    display.print(buf);
}

void Window::drawCentreString(String& buf, int x, int y) {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, 0, y, &x1, &y1, &w, &h); //calc width of new string
    display.setCursor(x - w / 2, y);
    display.print(buf);
}

void Window::initBar(){
    // Memory
    display.setFont(NULL);
    display.drawBitmap(5,1, bar_memory, 16, 16, BLACK);
    display.setTextColor(BLACK);
    display.setTextSize(2);
    display.setCursor((5+16+5),2);
    display.print("100");

    // Battery
    display.drawRoundRect(371,3,24,12,2,BLACK);
    display.fillRect(395,5,3,8,BLACK);

    display.drawLine(0,18,400,18,BLACK);
}

void Window::updateBar(float batteryVoltage, bool usb, bool logging, bool location, bool time){
    
    static int32_t oldHour = 0;
    static int32_t oldMinute = 0;
    static float oldBatteryVoltage = 0;
    static bool oldUsbStatus = false;
    static bool oldLocationStatus = false;
    static bool oldLoggingStatus = false;

    static uint32_t blinkStatus = 0;

    // Logging
    if(logging != oldLoggingStatus){
        display.drawBitmap(65,1, bar_download, 16, 16, !logging);
    }
    if(logging){
        display.drawBitmap(65,1, bar_download, 16, 16, blinkStatus);
    }

    //Location
    if(location != oldLocationStatus){
        display.drawBitmap(329,1, bar_location, 16, 16, !location);
    }
    

    if((minute() != oldMinute || hour() != oldHour) && time){
        display.setFont(NULL);
        display.setTextSize(2);

        display.setTextColor(WHITE);

        String t = String(oldHour) + ":";
        if(oldMinute < 10) t += '0';
        t += String(oldMinute);

        drawCentreString(t, 200, 2);

        oldHour = hour();
        oldMinute = minute();
        display.setTextColor(BLACK);

        t = String(oldHour) + ":";
        if(oldMinute < 10) t += '0';
        t += String(oldMinute);

        drawCentreString(t, 200, 2);
        
    }

    // USB
    if(usb != oldUsbStatus){
        
        oldUsbStatus = usb;

        display.fillRect(373,5,6,8,WHITE);
        display.fillRect(380,5,6,8,WHITE);
        display.fillRect(387,5,6,8,WHITE);

        display.drawBitmap(376,1, bar_flash, 16, 16, !usb);
    } 

    // Battery
    if(batteryVoltage != oldBatteryVoltage && !usb){
        if(batteryVoltage > 3.3f){
            display.fillRect(373,5,6,8,BLACK);
            display.drawRoundRect(371,3,24,12,2,BLACK);
            display.fillRect(395,5,3,8,BLACK);
        } else {
            display.fillRect(373,5,6,8,WHITE);
            display.drawRoundRect(371,3,24,12,2,blinkStatus);
            display.fillRect(395,5,3,8,blinkStatus);
        }
        if(batteryVoltage > 3.6f){
            display.fillRect(380,5,6,8,BLACK);
        } else {
            display.fillRect(380,5,6,8,WHITE);
        }
        if(batteryVoltage > 3.9f){
            display.fillRect(387,5,6,8,BLACK);
        } else {
            display.fillRect(387,5,6,8,WHITE);
        }
    }

    blinkStatus = !blinkStatus;
    
    display.refresh();
}

void Window::initMenu(uint32_t index){
    display.fillRect(0,19,400,222, WHITE);

    display.setFont(&FreeSans9pt7b);
    display.setTextSize(1);
    display.setTextColor(BLACK);

    drawCentreString("Live", 75, 127);
    drawCentreString("Recovery", 200, 127);
    drawCentreString("Testing", 325, 127);

    drawCentreString("Data", 75, 233);
    drawCentreString("Sensors", 200, 233);
    drawCentreString("Settings", 325, 233);

    display.drawBitmap(43,38, menu_live, 64, 64, BLACK);
    display.drawRoundRect(35,30,80,80,9,BLACK);

    display.drawBitmap(43,143, menu_data, 64, 64, BLACK);
    display.drawRoundRect(35,135,80,80,9,BLACK);
  
    display.drawBitmap(168,38, menu_recover, 64, 64, BLACK);
    display.drawRoundRect(160,30,80,80,10,BLACK);

    display.drawBitmap(168,143, menu_sensors, 64, 64, BLACK);
    display.drawRoundRect(160,135,80,80,10,BLACK);
  
    display.drawBitmap(293,38, menu_testing, 64, 64, BLACK);
    display.drawRoundRect(285,30,80,80,10,BLACK);

    display.drawBitmap(293,143, menu_settings, 64, 64, BLACK);
    display.drawRoundRect(285,135,80,80,10,BLACK);
    updateMenu(index);

  
    display.refresh();
}

void Window::updateMenu(uint32_t index){
    static int oldHighlight = 0;

    /* Pait over last selcted with white */

    int xPos = (oldHighlight % 3) * 125 + 36;
    int yPos = (oldHighlight / 3) * 105 + 31;

    display.drawRoundRect(xPos,yPos,78,78,9,WHITE);

    xPos = (index % 3) * 125 + 36;
    yPos = (index / 3) * 105 + 31;
    display.drawRoundRect(xPos,yPos,78,78,9,BLACK);

    display.refresh();

    oldHighlight = index;
}

void Window::initLive(){
    display.fillRect(0,19,400,222, WHITE);

    display.drawLine(199,18,199,240, BLACK);
    display.drawLine(200,18,200,240, BLACK);


    display.drawLine(0,49,400,49, BLACK);

    display.drawBitmap(5, 50, live_altitude, 24, 24, BLACK);
    display.drawBitmap(5, 75, live_speed, 24, 24, BLACK);
    display.drawBitmap(5, 100, live_lat, 24, 24, BLACK);
    display.drawBitmap(5, 125, live_lon, 24, 24, BLACK);
    display.drawBitmap(3, 150, live_battery, 24, 24, BLACK);

    display.drawBitmap(120, 149, live_one, 24, 24, BLACK);
    display.drawBitmap(158, 149, live_two, 24, 24, BLACK);

    display.drawBitmap(320, 149, live_one, 24, 24, BLACK);
    display.drawBitmap(358, 149, live_two, 24, 24, BLACK);

    display.drawBitmap(205, 50, live_altitude, 24, 24, BLACK);
    display.drawBitmap(205, 75, live_speed, 24, 24, BLACK);
    display.drawBitmap(205, 100, live_lat, 24, 24, BLACK);
    display.drawBitmap(205, 125, live_lon, 24, 24, BLACK);
    display.drawBitmap(203, 150, live_battery, 24, 24, BLACK);


    display.setFont(&FreeSans9pt7b);
    display.setTextSize(1);
    display.fillRect(0,202,399,240,BLACK);
    display.setTextColor(WHITE);

    connected[0] = false;
    connected[1] = false;

    display.setCursor(245, 227);
    display.print("Disconnected");

    display.setCursor(45, 227);
    display.print("Disconnected");

    display.setTextColor(BLACK);
    display.setFont(NULL);
    display.refresh();
}

void Window::updateLive(TelemetryInfo* info, uint32_t index){
    if(index > 1) return;

    // Read some random data to reset the updated flag
    info->lq();

    updateLiveInfo(&infoData[index], index, BLACK);

    memcpy(&infoData[index], info, sizeof(infoData[0]));
    dataAge[index] = millis() - lastTeleData[index];
    updateLiveInfo(&infoData[index], index, WHITE);
}

void Window::updateLive(TelemetryData* data, TelemetryInfo* info, uint32_t index){
    if(index > 1) return;

    lastTeleData[index] = millis();

    // Read some random data to reset the updated flag
    data->state();
    info->lq();

    updateLiveData(&teleData[index], index, WHITE);
    updateLiveInfo(&infoData[index], index, BLACK);

    //display.fillRect(10,19,190,200, WHITE);

    memcpy(&teleData[index], data, sizeof(teleData[0]));
    memcpy(&infoData[index], info, sizeof(infoData[0]));
    
    dataAge[index] = 0;

    updateLiveData(&teleData[index], index, BLACK);
    updateLiveInfo(&infoData[index], index, WHITE);
}

const char* const stateName [] = {
    "INVALID", "CALIB", "READY", "THRUST", "COAST", "DROGUE", "MAIN", "DOWN"
    };

const char* const errorName [] = {
    "No Config", "Log Full", "Filter Error", "Overheating", "Continuity Error" 
};

void Window::updateLiveData(TelemetryData* data, uint32_t index, uint32_t color){

    int xOffset = index * 200;

    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    display.setTextColor(color);

    if(data->testingMode()) {
        drawCentreString("TESTING", xOffset+100, 42);
        display.fillRect(xOffset+1, 50, 198, 151, WHITE);
        display.setCursor(xOffset + 20,80);
        display.print("DO NOT FLY!");
        return;
    } else {
        drawCentreString(stateName[data->state()], xOffset+100, 42);
    }

    display.setCursor(xOffset + 35,70);
    display.print(data->altitude());
    display.print(" m");

    display.setCursor(xOffset + 35,95);
    display.print(data->velocity());
    display.print(" m/s");

    display.setCursor(xOffset + 35,120);
    display.print(data->lat(), 4);
    display.print(" N");

    display.setCursor(xOffset + 35,145);
    display.print(data->lon(), 4);
    display.print(" E");

    display.setCursor(xOffset + 35,170);
    display.print(data->voltage());
    display.print(" V");

    if(data->pyroContinuity() & 0x01){
        display.drawBitmap(xOffset + 142, 156, live_checkmark, 16, 16, color);
    } else {
        display.drawBitmap(xOffset + 142, 156, live_cross, 16, 16, color);
    }

    if(data->pyroContinuity() & 0x02){
        display.drawBitmap(xOffset + 180, 156, live_checkmark, 16, 16, color);
    } else {
        display.drawBitmap(xOffset + 180, 156, live_cross, 16, 16, color);
    }
    
    display.setFont(&FreeSans9pt7b);
    display.setCursor(xOffset + 35,192);
    

    if(data->errors() & 0x04) {
        display.print(errorName[2]);
    } else if (data->errors() & 0x10) {
        display.print(errorName[4]);
    } else if (data->errors() & 0x02) {
        display.print(errorName[1]);
    } else if (data->errors() & 0x01) {
        display.print(errorName[0]);
    } else if (data->errors() & 0x08) {
        display.print(errorName[3]);
    }

    display.setFont(&FreeSans9pt7b);
    display.setTextSize(1);
    display.setTextColor(!color);
}

void Window::updateLiveInfo(TelemetryInfo* info, uint32_t index, uint32_t color){

    int xOffset = index * 200;

    display.setFont(&FreeSans9pt7b);
    display.setTextSize(1);
    display.setTextColor(color);

    if(dataAge[index] > 4900){
        if(color == WHITE){
            connected[index] = false;
            display.fillRect(xOffset+0,202,199,240,BLACK);
            display.setCursor(xOffset+45, 227);
            display.print("Disconnected");
            info->lq();
        }
    } else {
        if(connected[index] == false){
            if(color == WHITE){
                display.fillRect(xOffset+0,202,199,240,BLACK);
                connected[index] = true;
                display.setCursor(xOffset+5,217);
                display.print("AGE");
                display.setCursor(xOffset+100,217);
                display.print("SNR");
                display.setCursor(xOffset+5, 237);
                display.print("LQ");
                display.setCursor(xOffset+100, 237);
                display.print("RSSI");
            }
        }
        display.setCursor(xOffset+50,217);
        display.print((float)dataAge[index]/1000.0f,1);
        display.setCursor(xOffset+145,217);
        display.print(info->snr());
        display.setCursor(xOffset+50, 237);
        display.print(info->lq());
        display.setCursor(xOffset+145, 237);
        display.print(info->rssi());
    }

    
}

void Window::initRecovery(){
    display.fillRect(0,19,400,222, WHITE);
    
    display.drawCircle(300, 125, 80, BLACK);

    display.drawBitmap(5,40,rocket_recovery,32,32,BLACK);

    display.drawBitmap(40, 30, live_lat, 24, 24, BLACK);
    display.drawBitmap(40, 55, live_lon, 24, 24, BLACK);

    display.drawBitmap(5, 100,house_recovery,32,32,BLACK);
    display.drawBitmap(40, 90, live_lat, 24, 24, BLACK);
    display.drawBitmap(40, 115, live_lon, 24, 24, BLACK);

    display.refresh();
}

void Window::updateRecovery(Navigation* navigation){
    display.fillRect(60,19,400,222, WHITE);
    
    float angle = navigation->getNorth();
    
    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);

    display.setCursor(70, 50);
    display.print(navigation->getPointB().lat, 4);

    display.setCursor(70, 75);
    display.print(navigation->getPointB().lon, 4);

    
    display.setCursor(70, 110);
    display.print(navigation->getPointA().lat, 4);

    display.setCursor(70, 135);
    display.print(navigation->getPointA().lon, 4);

    display.setCursor(70, 170);
    display.print(navigation->getDistance());
    display.print("m");


    display.setFont(&FreeSans9pt7b);

    float radius = 90;
    float correctionFactor = 0.06;
    
    int x = radius * cos(angle - PI/2);
    int y = radius * sin(angle - PI/2) + 125;
    drawCentreString("N", x+300, y + correctionFactor*y);

    x = radius * cos(angle);
    y = radius * sin(angle) +125;
    drawCentreString("E", x+300, y + correctionFactor*y);

    x = radius * cos(angle + PI/2);
    y = radius * sin(angle + PI/2) +125;
    drawCentreString("S", x+300, y + correctionFactor*y);

    x = radius * cos(angle + PI);
    y = radius * sin(angle + PI) +125;
    drawCentreString("W", x+300, y + correctionFactor*y);

    angle = navigation->getAzimuth() + angle - PI/2;

    x = 70 * cos(angle) + 300;
    y = 70 * sin(angle) + 125;
    int x1 = 30 * cos(angle + 0.2) + 300;
    int y1 = 30 * sin(angle + 0.2) + 125;
    int x2 = 30 * cos(angle - 0.2) + 300;
    int y2 = 30 * sin(angle - 0.2) + 125;

    display.drawCircle(300, 125, 80, BLACK);
    
    if(navigation->getDistance() > 20){
        display.fillTriangle(x,y,x1,y1,x2,y2,BLACK);
    } else {
        display.drawCircle(300,125,6,BLACK);
    }
    
    
    display.refresh();
}

void Window::initBox(const char* text){
    display.fillRect(60,60,280,120, WHITE);
    display.drawRect(60,60,280,120, BLACK);

    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    display.setTextColor(BLACK);

    drawCentreString(text, 200, 110);

    display.setFont(&FreeSans9pt7b);

    display.setCursor(80, 160);
    display.print("Cancel (B)");

    display.setCursor(255, 160);
    display.print("OK (A)");

    display.refresh();
}

void Window::initTestingBox(uint32_t index){
    display.fillRect(60,60,280,120, WHITE);
    display.drawRect(60,60,280,120, BLACK);

    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    display.setTextColor(BLACK);

    drawCentreString("Trigger Event?", 200, 110);

    display.setFont(&FreeSans9pt7b);

    display.setCursor(80, 160);
    display.print("Cancel (B)");

    display.setCursor(255, 160);
    display.print("OK (A)");

    display.refresh();
}

void Window::initTesting(){
    display.fillRect(0,19,400,222, WHITE);

    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(10, 120);
    display.setTextSize(1);
    
    drawCentreString("Read this before continuing!", 200, 50);
    
    display.setFont(&FreeSans9pt7b);
    
    display.setCursor(6, 80);
    display.print("Testing mode enables manual triggering of");
    display.setCursor(6, 100);
    display.print("events and the corresponding actions associated");
    display.setCursor(6, 120);
    display.print("with them. This feature should only be used for");
    display.setCursor(6, 140);
    display.print("testing purposes and never during flight."); 
    display.setCursor(6, 160);
    display.print("CATS GmbH is not responsible for any potential");
    display.setCursor(6, 180);
    display.print("injuries or material damage caused by operation");
    display.setCursor(6, 200);
    display.print("of the CATS flight computers.");

    display.setCursor(6, 225);
    display.print("Cancel (B)");

    display.setCursor(290, 225);
    display.print("Continue (A)");

    display.refresh();
}

void Window::initTestingConfirmed(bool connected, bool testingEnabled) {
    display.fillRect(0,19,400,222, WHITE);
    display.setTextSize(1);
    if(connected) {
        if(testingEnabled) {
            display.drawRect(90, 95, 220, 50, BLACK);
            display.drawRect(91, 96, 218, 48, BLACK);
            display.drawRect(92, 97, 216, 46, BLACK);
            display.setFont(&FreeSansBold12pt7b);
            drawCentreString("ARM", 200, 127);

            display.setFont(&FreeSans9pt7b);
            display.setCursor(6, 225);
            display.print("Cancel (B)");

            display.setCursor(290, 225);
            display.print("Continue (A)");
        } else {
            display.setFont(&FreeSansBold9pt7b);
            drawCentreString("Not in Testing Mode.", 200, 100);
            display.setFont(&FreeSans9pt7b);
            display.setCursor(6, 130);
            display.print("Connect the flight computer to the Configurator,");
            display.setCursor(6, 150);
            display.print("enable 'Testing Mode' and set 'Testing Phrase'.");

            display.setFont(&FreeSans9pt7b);
            display.setCursor(6, 225);
            display.print("Cancel (B)");
        }
    } else {
        display.setFont(&FreeSansBold9pt7b);
        drawCentreString("No Connection", 200, 100);
        display.setFont(&FreeSans9pt7b);
        display.setCursor(6, 130);
        display.print("Make sure the 'Link Phrase 1' is set correctly on");
        display.setCursor(6, 150);
        display.print("both the flight computer and the ground station.");

        display.setFont(&FreeSans9pt7b);
        display.setCursor(6, 225);
        display.print("Cancel (B)");
    }

    display.refresh();
}
void Window::initTestingFailed() {
    display.fillRect(0,19,400,222, WHITE);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    drawCentreString("Could not ARM System", 200, 100);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 130);
    display.print("Make sure 'Test Phrase' is set correctly on both");
    display.setCursor(6, 150);
    display.print("the flight computer and the ground station.");

    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 225);
    display.print("Cancel (B)");

    display.refresh();
}

void Window::initTestingLost() {
    display.fillRect(0,19,400,222, WHITE);
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setFont(&FreeSansBold9pt7b);
    drawCentreString("Connection Lost", 200, 100);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 130);
    display.print("For safety reason the ground station disconnects");
    display.setCursor(6, 150);
    display.print("very quickly. Make sure you have a good");
    display.setCursor(6, 170);
    display.print("connection before continuing.");
    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 225);
    display.print("Cancel (B)");

    display.refresh();
}

void Window::initTestingWait() {
    display.fillRect(0,19,400,222, WHITE);
    display.setTextSize(1);
    display.setFont(&FreeSansBold9pt7b);
    drawCentreString("Starting testing mode...", 200, 100);
    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 130);
    display.print("This may take a few seconds.");

    display.setFont(&FreeSans9pt7b);
    display.setCursor(6, 225);
    display.print("Cancel (B)");

    display.refresh();
}


void Window::initTestingReady() {
    display.fillRect(0,19,400,222, WHITE);

    display.drawLine(199,18,199,220, BLACK);
    display.drawLine(200,18,200,220, BLACK);

    display.drawLine(0,19,399,19, BLACK);

    display.drawLine(0,68,399,68, BLACK);
    display.drawLine(0,69,399,69, BLACK);

    display.drawLine(0,118,399,118, BLACK);
    display.drawLine(0,119,399,119, BLACK);

    display.drawLine(0,168,399,168, BLACK);
    display.drawLine(0,169,399,169, BLACK);

    display.drawLine(0,218,399,218, BLACK);
    display.drawLine(0,219,399,219, BLACK);

    display.setTextSize(1);
    display.setFont(&FreeSans12pt7b);

    drawCentreString(eventName[0], 100, 51);
    drawCentreString(eventName[1], 100, 101);
    drawCentreString(eventName[2], 100, 151);
    drawCentreString(eventName[3], 100, 201);

    drawCentreString(eventName[4], 300, 51);
    drawCentreString(eventName[5], 300, 101);
    drawCentreString(eventName[6], 300, 151);
    drawCentreString(eventName[7], 300, 201);
}

void Window::updateTesting(uint32_t index) {
    static uint32_t oldIndex = 0;
    uint32_t xOffset = 201 * (oldIndex / 4);
    uint32_t yOffset = 50 * (oldIndex % 4) + 20;

    display.fillRect(xOffset, yOffset, 199, 48, WHITE);

    display.setTextSize(1);
    display.setFont(&FreeSans12pt7b);
    display.setTextColor(BLACK);

    xOffset = 200 * (oldIndex / 4) + 100;
    yOffset = 50 * (oldIndex % 4) + 51;

    drawCentreString(eventName[oldIndex], xOffset, yOffset);

    xOffset = 201 * (index / 4);
    yOffset = 50 * (index % 4) + 20;

    display.fillRect(xOffset, yOffset, 199, 48, BLACK);

    display.setTextColor(WHITE);
    xOffset = 200 * (index / 4) + 100;
    yOffset = 50 * (index % 4) + 51;
    drawCentreString(eventName[index], xOffset, yOffset);

    oldIndex = index;
    display.refresh();
}

void Window::initData(){
    display.fillRect(0,19,400,222, WHITE);

    display.refresh();
}

void Window::initSesnors(){
    display.fillRect(0,19,400,222, WHITE);

    display.refresh();
}

void Window::initSettings(uint32_t submenu){
    display.fillRect(0,19,400,222, WHITE);

    display.drawLine(0,49,400,49, BLACK);
    

    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.fillRect(0,19,400,30, BLACK);
    drawCentreString(settingPageName[submenu], 200, 42);

    display.setTextColor(BLACK);
    for(int i = 0; i < settingsTableValueCount[submenu]; i++){
        addSettingEntry(i, &settingsTable[submenu][i]);
    }

    if(submenu == 0){
        display.fillTriangle(386, 33, 378, 25, 378, 41, WHITE);
    } else {
        display.fillTriangle(13, 33, 21, 25, 21, 41, WHITE);
    }
    
    oldSettingsIndex = -1;
    subMenuSettingIndex = submenu;

    display.drawLine(0,177,400,177, BLACK);
    display.refresh();
}

void Window::addSettingEntry(uint32_t settingIndex, const device_settings_t* setting, bool color){
    uint32_t y = 75 + 30 * settingIndex;

    display.setTextColor(color);

    display.setCursor(20, y);
    display.print(setting->name);
    

    if(setting->type == TOGGLE){
        bool data = *(bool*)setting->dataPtr;
        drawCentreString(lookup_tables[setting->config.lookup].values[static_cast<uint16_t>(data)], 305, y);

        y -= 23;
        if(data == false){
            display.fillTriangle(386,y+14, 378, y+6, 378, y+22, color);
        } else {
            display.fillTriangle(224,y+14, 232, y+6, 232, y+22, color);
        }
    }
    else if(setting->type == STRING){
        drawCentreString((const char*)setting->dataPtr, 305, y);
    }
    else if(setting->type == NUMBER){
        char buffer[8];
        snprintf(buffer, 8, "%+d", *(int16_t*)setting->dataPtr);
        drawCentreString(buffer, 305, y);

        y -= 23;
        if(setting->config.minmax.max == *(int16_t*)setting->dataPtr){
            display.fillTriangle(386,y+14, 378, y+6, 378, y+22, !color);
        } else {
            display.fillTriangle(386,y+14, 378, y+6, 378, y+22, color);
        }

        if(setting->config.minmax.min == *(int16_t*)setting->dataPtr){
            display.fillTriangle(224,y+14, 232, y+6, 232, y+22, !color);
        } else {
            display.fillTriangle(224,y+14, 232, y+6, 232, y+22, color);
        }

    }
}

void Window::updateSettings(int32_t index){
    
    display.setTextSize(1);

    if(oldSettingsIndex >= 0) {
        if(oldSettingsIndex != index){
            highlightSetting(oldSettingsIndex, BLACK);
        }
    } else {
        if(subMenuSettingIndex == 0) display.fillTriangle(386, 33, 378, 25, 378, 41, BLACK);
        else display.fillTriangle(13, 33, 21, 25, 21, 41, BLACK);
    }

    if(index >= 0) {
        highlightSetting(index, WHITE);
    } else {
        if(subMenuSettingIndex == 0) display.fillTriangle(386, 33, 378, 25, 378, 41, WHITE);
        else display.fillTriangle(13, 33, 21, 25, 21, 41, WHITE);
        display.fillRect(0,178,400,62, WHITE);
    }
    
    oldSettingsIndex = index;
    display.refresh();
}

void Window::highlightSetting(uint32_t index, bool color){
    display.setFont(&FreeSans12pt7b);
    uint32_t yPos = 52 + 30 * index;
    display.fillRect(0,yPos,400,30,!color);
    if(subMenuSettingIndex == 0) addSettingEntry(index, &settingsTable[subMenuSettingIndex][index], color);
    else addSettingEntry(index, &settingsTable[subMenuSettingIndex][index], color);

    display.fillRect(0,178,400,62, WHITE);
    display.setFont(&FreeSans9pt7b);
    display.setTextColor(BLACK);
    display.setCursor(10,195);
    display.print(settingsTable[subMenuSettingIndex][index].description1);
    display.setCursor(10,215);
    display.print(settingsTable[subMenuSettingIndex][index].description2);

}

int keybXY[38][2] = {  {  20, 125}, //'1'
                       {  60, 125}, //'2'
                       { 100, 125}, //'3'
                       { 140, 125}, //'4'
                       { 180, 125}, //'5'
                       { 220, 125}, //'6'
                       { 260, 125}, //'7'
                       { 300, 125}, //'8'
                       { 340, 125}, //'9'
                       { 380, 125}, //'0' ---
                       {  20, 155}, //'Q'
                       {  60, 155}, //'W'
                       { 100, 155}, //'E'
                       { 140, 155}, //'R'
                       { 180, 155}, //'T'
                       { 220, 155}, //'Y'
                       { 260, 155}, //'U'
                       { 300, 155}, //'I'
                       { 340, 155}, //'O'
                       { 380, 155}, //'P' ---
                       {  40, 185}, //'A'
                       {  80, 185}, //'S'
                       { 120, 185}, //'D'
                       { 160, 185}, //'F'
                       { 200, 185}, //'G'
                       { 240, 185}, //'H'
                       { 280, 185}, //'J'
                       { 320, 185}, //'K'
                       { 360, 185}, //'L' ---
                       {  20, 215}, //'SHIFT'
                       {  60, 215}, //'Z'
                       { 100, 215}, //'X'
                       { 140, 215}, //'C'
                       { 180, 215}, //'V'
                       { 220, 215}, //'B'
                       { 260, 215}, //'N'
                       { 300, 215}, //'M'
                       { 340, 215}, //'ENTER'
                       };

char keybChar[38] = { '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
                      'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P',
                         'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L',
                      ' ', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', ' ' };

void Window::initKeyboard(char* text, uint32_t maxLength){  
    if(maxLength != 0) keyboardTextMaxLength = maxLength; 
    display.fillRect(0,19,400,222, WHITE);
    
    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    display.setTextColor(BLACK);
    
    display.setCursor(140,80);
    display.print(text);

    display.setFont();
    display.setTextSize(2);

    for(int i = 0; i < 38; i++){
        if(i == oldKey){
            highlightKeyboardKey(i, BLACK);
        } else if (i != 29 && i != 37){
            if(!upperCase && i > 9) display.drawChar(keybXY[i][0], keybXY[i][1], keybChar[i]+32, BLACK, WHITE, 2);
            else display.drawChar(keybXY[i][0], keybXY[i][1], keybChar[i], BLACK, WHITE, 2);
        }
    }
    if (oldKey != 29) display.drawBitmap(keybXY[29][0]-4, keybXY[29][1]-1, shift_keyboard, 16, 16, BLACK);
    if (oldKey != 37) display.drawBitmap(keybXY[37][0]-4, keybXY[37][1]-1, enter_keyboard, 16, 16, BLACK);
    if (oldKey != -1) display.drawBitmap(280, 60, backspace_keyboard, 24, 24, BLACK);
    else highlightKeyboardKey(-1, BLACK);

    display.refresh();
}

void Window::updateKeyboard(char* text, int32_t keyHighlight, bool keyPressed){
    
    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    display.setTextColor(BLACK);

    if(keyPressed){
        if(keyHighlight == 29){ // SHIFT
            upperCase = !upperCase;
            initKeyboard(text);
        } else if (keyHighlight == -1){ // BACKSPACE
            updateKeyboardText(text, WHITE);
            if(strlen(text) > 0) text[strlen(text)-1] = 0;
            updateKeyboardText(text, BLACK);
        } else { // KEY
            if(strlen(text) < keyboardTextMaxLength){
                updateKeyboardText(text, WHITE);
                if(keyHighlight > 9) text[strlen(text)] = keybChar[keyHighlight] + !upperCase*32;
                else text[strlen(text)] = keybChar[keyHighlight];
                updateKeyboardText(text, BLACK);
            }
        }
    }

    display.setFont();
    display.setTextSize(2);

    highlightKeyboardKey(oldKey, WHITE);

    highlightKeyboardKey(keyHighlight, BLACK);
    
    oldKey = keyHighlight;
    display.refresh();
} 

void Window::highlightKeyboardKey(int32_t key, bool color){
    

    if(key == -1){
        display.fillCircle(291, 71, 16, color);
        display.drawBitmap(280, 60, backspace_keyboard, 24, 24, !color);
    } else {
        display.fillCircle(keybXY[key][0]+4, keybXY[key][1]+7, 16, color);
    }
    
    if(key == 29){
        display.drawBitmap(keybXY[29][0]-4, keybXY[29][1]-1, shift_keyboard, 16, 16, !color);       
    } else if(key == 37){
        display.drawBitmap(keybXY[37][0]-4, keybXY[37][1]-1, enter_keyboard, 16, 16, !color);
    } else {
        if(!upperCase && key > 9) display.drawChar(keybXY[key][0], keybXY[key][1], keybChar[key]+32, !color, color, 2);
        else display.drawChar(keybXY[key][0], keybXY[key][1], keybChar[key], !color, color, 2);
    }
}

void Window::updateKeyboardText(char* text, bool color){
    display.setTextColor(color);
    display.setCursor(140,80);
    display.print(text);
}
