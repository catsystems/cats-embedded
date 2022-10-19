
#include "systemParser.h"
#include "config.h"

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