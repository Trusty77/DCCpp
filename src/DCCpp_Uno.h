/**********************************************************************

DCCpp_Uno.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman
Adapted for DCCpp by Thierry PARIS

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#ifndef DCCpp_Uno_h
#define DCCpp_Uno_h

#include "DCCpp.h"

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "2.0.0"

/////////////////////////////////////////////////////////////////////////////////////
// SELECT COMMUNICATION INTERACE
/////////////////////////////////////////////////////////////////////////////////////
//#if defined(USE_ETHERNET) || defined(USE_WIFI)
#if defined(USE_ETHERNET)
	#define DCCPP_INTERFACE eServer
	#define SDCARD_CS 4

#elif defined(USE_WIFI)
	#define DCCPP_INTERFACE eServer
  
#else
  // define DCCPP_INTERFACE as serial in all cases to be able at least to print diagnostic messages on console
  #define DCCPP_INTERFACE Serial

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SET WHETHER TO SHOW PACKETS - DIAGNOSTIC MODE ONLY
/////////////////////////////////////////////////////////////////////////////////////

// If SHOW_PACKETS is set to 1, then for select main operations track commands that modify an internal DCC packet register,
// if printFlag for that command is also set to 1, DCC++ BASE STATION will additionally return the 
// DCC packet contents of the modified register in the following format:

//    <* REG: B1 B2 ... Bn CSUM / REPEAT>
//
//    REG: the number of the main operations track packet register that was modified
//    B1: the first hexadecimal byte of the DCC packet
//    B2: the second hexadecimal byte of the DCC packet
//    Bn: the nth hexadecimal byte of the DCC packet
//    CSUM: a checksum byte that is required to be the final byte in any DCC packet
//    REPEAT: the number of times the DCC packet was re-transmitted to the tracks after its iniital transmission
 
//#define SHOW_PACKETS  0       // set to zero to disable printing of every packet for select main operations track commands

/////////////////////////////////////////////////////////////////////////////////////

#endif
