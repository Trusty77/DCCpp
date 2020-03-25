/**********************************************************************

Comm.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "Config.h"

#ifdef USE_WIFI                    // WiFi Selected
  #include <WiFi.h>
  extern WiFiServer DCCPP_INTERFACE;
#endif 


#ifdef USE_ETHERNET                 // Ethernet Shield Card Selected

  #ifdef USE_ETHERNET_WIZNET_5100
    #define COMM_SHIELD_NAME      "ARDUINO-CC ETHERNET SHIELD (WIZNET 5100)"
    #include <Ethernet.h>         // built-in Arduino.cc library

  #elif defined(USE_ETHERNET_WIZNET_5500)
    #define COMM_SHIELD_NAME      "ARDUINO-ORG ETHERNET-2 SHIELD (WIZNET 5500)"
    #include <Ethernet2.h>        // https://github.com/arduino-org/Arduino

  #elif defined(USE_ETHERNET_WIZNET_5200)
    #define COMM_SHIELD_NAME      "SEEED STUDIO ETHERNET SHIELD (WIZNET 5200)"
    #include <EthernetV2_0.h>     // https://github.com/Seeed-Studio/Ethernet_Shield_W5200

  #elif defined(USE_ETHERNET_ENC28J60)
    #define COMM_SHIELD_NAME      "BASIC ETHERNET SHIELD (ENC28J60)"
  #include <UIPEthernet.h>     // https://github.com/ntruchsess/arduino_uip/tree/Arduino_1.5.x

  #endif

  extern EthernetServer DCCPP_INTERFACE;
#endif  
