
#include "systemParser.h"
#include "config.h"
#include "console.h"

SystemParser systemParser;
Config systemConfig;

/*
if(systemParser.loadFile(configFileName))
  {
    if(systemParser.getUsbVid() != -1)
    {
      vid = systemParser.getUsbVid();
    }
    if(systemParser.getUsbPid() != -1)
    {
      pid = systemParser.getUsbPid();
    }
    if(strlen(systemParser.getUsbSerial()))
    {
      serial = systemParser.getUsbSerial();
    }
    console.ok.println("[UTILS] System config loading was successful.");
  }
  else
  {
    console.error.println("[UTILS] System config loading failed.");
    status = false;
  }
  */

void Config::save()
{
  systemParser.setTestingPhrase(config.testingPhrase);
  systemParser.setLinkPhrase1(config.linkPhrase1);
  systemParser.setLinkPhrase2(config.linkPhrase2);
  systemParser.setTelemetryMode(config.receiverMode);
  systemParser.setNeverStopLoggingFlag(config.neverStopLogging);
  systemParser.setTimeZone(config.timeZoneOffset);
  systemParser.saveFile("/config.json");
}

void Config::load()
{ 
  systemParser.loadFile("/config.json");
  console.log.println("Load config file");
  bool mode;
  bool stop;
  if (!systemParser.getTestingPhrase(config.testingPhrase)) {
    strncpy(config.testingPhrase, "", 1);
    console.error.println("Failed");
  } else {
    console.log.println(config.testingPhrase);
  }
  if (!systemParser.getLinkPhrase1(config.linkPhrase1)) {
    strncpy(config.linkPhrase1, "", 1);
    console.error.println("Failed");
  } else {
    console.log.println(config.linkPhrase1);
  }
  if (!systemParser.getLinkPhrase2(config.linkPhrase2)) {
    strncpy(config.linkPhrase2, "", 1);
    console.error.println("Failed");
  } else {
    console.log.println(config.linkPhrase2);
  }
  if (!systemParser.getTelemetryMode(mode)) {
    mode = false;
  } else {
    console.log.println(mode);
  }
  if (!systemParser.getNeverStopLoggingFlag(stop)) {
    config.neverStopLogging = false;
  } else {
    console.log.println(config.neverStopLogging);
  }
  if (!systemParser.getTimeZone(config.timeZoneOffset)) {
    config.timeZoneOffset = 0;
  } else {
    console.log.println(config.timeZoneOffset);
  }
  config.neverStopLogging = static_cast<uint8_t>(stop);
  config.receiverMode = static_cast<ReceiverTelemetryMode_e>(mode);
}