/*************************************************************
project: <Dc/Dcc Controller>
author: <Thierry PARIS>
description: <Dcc Ethernet Controller sample>
*************************************************************/

#include "DCCpp.h"

#if !defined(USE_TEXTCOMMAND) || !defined(USE_WIFI)
#error To be able to compile this sample,the lines #define USE_TEXTCOMMAND and #define USE_WIFI must be uncommented in DCCpp.h
#endif


const char* ssid     = "mySsid";
const char* password = "myPswd";

WiFiServer DCCPP_INTERFACE(2560);
void setup()
{
  Serial.begin(115200);
  Serial.println("DCCpp Wifi for ESP32 0.1");

  DCCpp::beginWifi(ssid, password, EthernetProtocol::TCP);
  
  DCCpp::begin();
  // Configuration for my LMD18200. See the page 'Configuration lines' in the documentation for other samples.
  /* My configuration for ESP32
  DIR -> GPIO_32
  PWM -> EN
  MAX471 -> GPIO_36 (A0)
  */
  DCCpp::beginMain(UNDEFINED_PIN, 32, UNDEFINED_PIN, A0);
  
}

void loop()
{
  DCCpp::loop();
}
