#include "bmp.h"
#include "window.h"
#include <TimeLib.h>


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

    display.drawBitmap(35,80, menu_live, 80, 80, BLACK);
    display.drawRoundRect(25,70,100,100,9,BLACK);
  
    display.drawBitmap(160,80, menu_recover, 80, 80, BLACK);
    display.drawRoundRect(150,70,100,100,10,BLACK);
  
    display.drawBitmap(285,80, menu_settings, 80, 80, BLACK);
    display.drawRoundRect(275,70,100,100,10,BLACK);
    updateMenu(index);

  
    display.refresh();
}

void Window::updateMenu(uint32_t index){
    static int oldHighlight = 0;

    /* Pait over last selcted with white */
    int xPos = oldHighlight * 125 + 25;
    display.drawRoundRect(xPos+1,71,98,98,9,WHITE);
    display.drawRoundRect(xPos+2,72,96,96,8,WHITE);

    xPos = index * 125 + 25;
    display.drawRoundRect(xPos+1,71,98,98,9,BLACK);
    display.drawRoundRect(xPos+2,72,96,96,8,BLACK);


    display.setFont(&FreeSans18pt7b);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    if(oldHighlight == 0) {
    drawCentreString("Live Data", 200, 210);
    } else if (oldHighlight == 1) {
    drawCentreString("Recovery", 200, 210);
    } else {
    drawCentreString("Settings", 200, 210);
    }

    display.setTextColor(BLACK);
    if(index == 0) {
    drawCentreString("Live Data", 200, 210);
    } else if (index == 1) {
    drawCentreString("Recovery", 200, 210);
    } else {
    drawCentreString("Settings", 200, 210 );
    }

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

    display.drawBitmap(205, 50, live_altitude, 24, 24, BLACK);
    display.drawBitmap(205, 75, live_speed, 24, 24, BLACK);
    display.drawBitmap(205, 100, live_lat, 24, 24, BLACK);
    display.drawBitmap(205, 125, live_lon, 24, 24, BLACK);


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

    
    display.refresh();
}

void Window::updateLive(TelemetryData* data, TelemetryInfo* info, uint32_t index){
    if(index > 1) return;

    lastTeleData[index] = millis();

    // Read some random data to reset the updated flag
    data->state();
    info->lq();

    updateLiveData(&teleData[index], index, WHITE);
    updateLiveInfo(&infoData[index], index, BLACK);

    memcpy(&teleData[index], data, sizeof(teleData[0]));
    memcpy(&infoData[index], info, sizeof(infoData[0]));
    
    dataAge[index] = 0;

    updateLiveData(&teleData[index], index, BLACK);
    updateLiveInfo(&infoData[index], index, WHITE);

    

    display.refresh();
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
    display.fillRect(200,19,400,222, WHITE);
    
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
    
    display.fillTriangle(x,y,x1,y1,x2,y2,BLACK);
    //display.fill

    
    display.refresh();
}

const char* const stateName [] = {
    "MOVING", "READY", "THRUST", "COAST", "APOGEE", "DROGUE", "MAIN", "DOWN"
    };

void Window::updateLiveData(TelemetryData* data, uint32_t index, uint32_t color){

    int xOffset = index * 200;

    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    display.setTextColor(color);

    drawCentreString(stateName[data->state()], xOffset+100, 42);

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
    display.print(data->d1());
    display.print(" \%");

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

}

void Window::initSettings(uint32_t submenu){
    display.fillRect(0,19,400,222, WHITE);

    display.drawLine(0,49,400,49, BLACK);

    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);

    drawCentreString(settingPageName[submenu], 200, 42);

    for(int i = 0; i < settingsTableValueCount[submenu]; i++){
            addSettingEntry(i, &settingsTable[submenu][i]);
    }

    if(submenu == 0){
        display.fillTriangle(386, 33, 378, 25, 378, 41, BLACK);
    } else {
        display.fillTriangle(13, 33, 21, 25, 21, 41, BLACK);
    }
    
    oldSettingsIndex = -1;
    subMenuSettingIndex = submenu;

    display.refresh();
}

void Window::addSettingEntry(uint32_t settingIndex, const device_settings_t* setting, bool color){
    uint32_t y = 75 + 30 * settingIndex;

    display.setTextColor(color);

    display.setCursor(20, y);
    display.print(setting->name);
    

    if(setting->type == TOGGLE){
        drawCentreString(lookup_tables[setting->config.lookup].values[*(int16_t*)setting->dataPtr], 305, y);

        y -= 23;
        if(*(int16_t*)setting->dataPtr == 0){
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
    display.setFont(&FreeSans12pt7b);
    display.setTextSize(1);
    
    if(oldSettingsIndex >= 0) {
        if(oldSettingsIndex != index){
            highlightSetting(oldSettingsIndex, BLACK);
        }
    } else {
        if(subMenuSettingIndex == 0) display.fillTriangle(386, 33, 378, 25, 378, 41, WHITE);
        else display.fillTriangle(13, 33, 21, 25, 21, 41, WHITE);
    }

    if(index >= 0) {
        highlightSetting(index, WHITE);
    } else {
        if(subMenuSettingIndex == 0) display.fillTriangle(386, 33, 378, 25, 378, 41, BLACK);
        else display.fillTriangle(13, 33, 21, 25, 21, 41, BLACK);
    }

    oldSettingsIndex = index;
    display.refresh();
}

void Window::highlightSetting(uint32_t index, bool color){
    uint32_t yPos = 52 + 30 * index;
    display.fillRect(0,yPos,400,30,!color);
    if(subMenuSettingIndex == 0) addSettingEntry(index, &settingsTable[subMenuSettingIndex][index], color);
    else addSettingEntry(index, &settingsTable[subMenuSettingIndex][index], color);
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
