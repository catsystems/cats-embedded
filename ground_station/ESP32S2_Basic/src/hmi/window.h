#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>

#include "telemetry/telemetryData.h"
#include "navigation.h"
#include "settings.h"

#define BLACK 0
#define WHITE 1

#define SHARP_SCK  36
#define SHARP_MOSI 35
#define SHARP_SS   34

typedef struct {
  time_t time;
  uint32_t storage;
  bool saving;
  bool locationLock;
  bool usbDetection;
} topBarData;


class Window{
  public:
    Window() : display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 400, 240) {}
    void begin();
    void logo();

    void initBar();
    void updateBar(float batteryVoltage, bool usb = false, bool logging = false, bool location = false, bool time = false);

    void initMenu(uint32_t index);
    void updateMenu(uint32_t index);

    void initLive();
    void updateLive(TelemetryInfo* info, uint32_t index);
    void updateLive(TelemetryData* data, TelemetryInfo* info, uint32_t index);

    void initRecovery();
    void updateRecovery(Navigation* navigation);

    void initSettings(uint32_t submenu);
    void updateSettings(int32_t index);

    void initBox(const char* text);

    void initKeyboard(char* text, uint32_t maxLength = 0);
    void updateKeyboard(char* text, int32_t keyHighlight, bool keyPressed = false);

  private:
    void updateLiveData(TelemetryData* data, uint32_t index, uint32_t color);
    void updateLiveInfo(TelemetryInfo* info, uint32_t index, uint32_t color);
    void drawCentreString(const char *buf, int x, int y);
    void drawCentreString(String& buf, int x, int y);

    void addSettingEntry(uint32_t settingIndex, const device_settings_t* setting, bool color = BLACK);
    void highlightSetting(uint32_t index, bool color);
    
    void highlightKeyboardKey(int32_t key, bool color);
    void updateKeyboardText(char* text, bool color);
    Adafruit_SharpMem display; 

    bool connected[2];
    uint32_t lastTeleData[2];
    uint32_t dataAge[2];
    topBarData barData;
    TelemetryData teleData[2];
    TelemetryInfo infoData[2];

    int32_t oldSettingsIndex;
    uint32_t subMenuSettingIndex;
    
    bool upperCase = true;
    int32_t oldKey = 0;
    uint32_t keyboardTextMaxLength = 0;
};