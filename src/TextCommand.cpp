/**********************************************************************

TextCommand.cpp
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "Arduino.h"
 
 // See TextCommand::parse() below for defined text commands.
 
 #include "TextCommand.h"
 #ifdef USE_TEXTCOMMAND
 
 #ifdef VISUALSTUDIO
 #include "string.h"
 #include "iostream"
 #else
 extern unsigned int __heap_start;
 extern void *__brkval;
 #endif
 
 
 char TextCommand::commandString[MAX_COMMAND_LENGTH+1];
 
 
 void TextCommand::init(volatile RegisterList *_mRegs, volatile RegisterList *_pRegs, CurrentMonitor *_mMonitor){
   commandString[0] = 0;
 } // TextCommand:TextCommand
 
 
 void TextCommand::process(){
   char c;
   
 #if defined(USE_ETHERNET) || defined(USE_WIFI)
   #if defined(USE_ETHERNET)
      EthernetClient client = DCCPP_INTERFACE.available();
      //-> Ajout CB
   #elif defined(USE_WIFI)
      WiFiClient client = DCCPP_INTERFACE.available();
   #endif
 
   if (client) {
      if (DCCppConfig::Protocol == EthernetProtocol::HTTP) {
        DCCPP_INTERFACE.println("HTTP/1.1 200 OK");
        DCCPP_INTERFACE.println("Content-Type: text/html");
        DCCPP_INTERFACE.println("Access-Control-Allow-Origin: *");
        DCCPP_INTERFACE.println("Connection: close");
        DCCPP_INTERFACE.println("");
      }
 
     while (client.connected() && client.available()) {        // while there is data on the network
        
       c = client.read();
       if (c == '<')                    // start of new command
         commandString[0] = 0;
       else if (c == '>')               // end of new command
       {
         if (parse(commandString) == false)
         {
             #if defined(DCCPP_DEBUG_MODE)
                       Serial.println("invalid command !");
             #endif
         }
       }
       else if (strlen(commandString) < MAX_COMMAND_LENGTH)    // if comandString still has space, append character just read from network
         sprintf(commandString, "%s%c", commandString, c);     // otherwise, character is ignored (but continue to look for '<' or '>')
     } // while
 
     if (DCCppConfig::Protocol == EthernetProtocol::HTTP)
       client.stop();
   }
   
  #else  // SERIAL case
     while (DCCPP_INTERFACE.available()>0) {    // while there is data on the serial line
       c = DCCPP_INTERFACE.read();
       if (c == '<')                    // start of new command
         commandString[0] = 0;
       else if (c == '>')               // end of new command
         parse(commandString);
       else if (strlen(commandString) < MAX_COMMAND_LENGTH)  // if commandString still has space, append character just read from serial line
         sprintf(commandString, "%s%c", commandString, c); // otherwise, character is ignored (but continue to look for '<' or '>')
     } // while
   
   #endif
 } // TextCommand:process
   
///////////////////////////////////////////////////////////////////////////////

bool TextCommand::parse(char *com){


#ifdef DCCPP_DEBUG_MODE
	Serial.print(com[0]);
	Serial.println(F(" command"));
#ifdef VISUALSTUDIO
	std::cout << com <<  " command received" << std::endl; 
#endif
#endif

  switch(com[0]){

	case 't':       
		/**	\addtogroup commandsGroup
		SET ENGINE THROTTLES USING 128-STEP SPEED CONTROL
		-------------------------------------------------

		<b>
		\verbatim
		<t REGISTER CAB SPEED DIRECTION>
		\endverbatim
		</b>
	
	   sets the throttle for a given register/cab combination 
	   
	   - <b>REGISTE%R</b>: an internal register number, from 1 through MAX_MAIN_REGISTERS (inclusive), to store the DCC packet used to control this throttle setting
	   - <b>CAB</b>:  the short (1-127) or long (128-10293) address of the engine decoder
	   - <b>SPEED</b>: throttle speed from 0-126, or -1 for emergency stop (resets SPEED to 0)
	   - <b>DIRECTION</b>: 1=forward, 0=reverse.  Setting direction when speed=0 or speed=-1 only effects directionality of cab lighting for a stopped train
	   
	   returns: <b>\<T REGISTE%R SPEED DIRECTION\></b>
	   */

	  DCCpp::mainRegs.setThrottle(com+1);
		return true;

	case 'f':       
		/**	\addtogroup commandsGroup
		OPERATE ENGINE DECODER FUNCTIONS F0-F28
		---------------------------------------
		
		<b>
		\verbatim
		<f CAB BYTE1 [BYTE2]>
		\endverbatim
		\verbatim
		<f -1 REGISTER CAB BYTE1 [BYTE2]>
		\endverbatim
		</b>

		turns on and off engine decoder functions F0-F28 (F0 is sometimes called FL)  
		NOTE: setting requests transmitted directly to mobile engine decoder --- current state of engine functions is not stored by this program
   
		- <b>REGISTE%R</b>: an internal register number, from 1 through MAX_MAIN_REGISTERS (inclusive), to store the DCC packet used to control these functions. REGISTER is only used if the first argument is -1.
		- <b>CAB</b>:  the short (1-127) or long (128-10293) address of the engine decoder
   
		To set functions F0-F4 on (=1) or off (=0):
     
		- <b>BYTE1</b>:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
		- <b>BYTE2</b>:  omitted
  
		To set functions F5-F8 on (=1) or off (=0):
  
		- <b>BYTE1</b>:  176 + F5*1 + F6*2 + F7*4 + F8*8
		- <b>BYTE2</b>:  omitted
  
		To set functions F9-F12 on (=1) or off (=0):
  
		- <b>BYTE1</b>:  160 + F9*1 +F10*2 + F11*4 + F12*8
		- <b>BYTE2</b>:  omitted
  
		To set functions F13-F20 on (=1) or off (=0):
  
		- <b>BYTE1</b>: 222 
		- <b>BYTE2</b>: F13*1 + F14*2 + F15*4 + F16*8 + F17*16 + F18*32 + F19*64 + F20*128
  
		To set functions F21-F28 on (=1) of off (=0):
  
		- <b>BYTE1</b>: 223
		- <b>BYTE2</b>: F21*1 + F22*2 + F23*4 + F24*8 + F25*16 + F26*32 + F27*64 + F28*128
  
		returns: NONE
		*/

		DCCpp::mainRegs.setFunction(com+1);
	  return true;

	case 'a':       
		/**	\addtogroup commandsGroup
		OPERATE STATIONARY ACCESSORY DECODERS 
		-------------------------------------
		
		<b>
		\verbatim
		<a ADDRESS SUBADDRESS ACTIVATE>
		\endverbatim
		</b>

		turns an accessory (stationary) decoder on or off
   
		- <b>ADDRESS</b>:  the primary address of the decoder (0-511)
		- <b>SUBADDRESS</b>: the sub-address of the decoder (0-3)
		- <b>ACTIVATE</b>: 1=on (set), 0=off (clear)
   
		Note that many decoders and controllers combine the ADDRESS and SUBADDRESS into a single number, N,
		from  1 through a max of 2044, where
   
		N = (ADDRESS - 1) * 4 + SUBADDRESS + 1, for all ADDRESS>0
   
		OR
   
		- <b>ADDRESS</b> = INT((N - 1) / 4) + 1
		- <b>SUBADDRESS</b> = (N - 1) % 4
   
		However, this general command simply sends the appropriate DCC instruction packet to the main tracks
		to operate connected accessories.  It does not store or retain any information regarding the current
		status of that accessory. To have this sketch store and retain the direction of DCC-connected turnouts, as 
		well as automatically invoke the required <b>\<a\></b> command as needed, first define/edit/delete turnouts 
		using the following variations of the "T" command.

		returns: NONE
		*/

	  DCCpp::mainRegs.setAccessory(com+1);
		return true;

#ifdef USE_TURNOUT
	case 'T':
/*
* *** SEE TURNOUT.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "T" COMMAND
* USED TO CREATE/EDIT/REMOVE/SHOW TURNOUT DEFINITIONS
*/

	  return Turnout::parse(com+1);
#endif

#ifdef USE_OUTPUT

	case 'Z':
/**** SEE OUTPUT.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "Z" COMMAND
*   USED TO CREATE / EDIT / REMOVE / SHOW OUTPUT DEFINITIONS
*/

	  return Output::parse(com+1);
#endif

#ifdef USE_SENSOR

	case 'S': 
/*   
 *   *** SEE SENSOR.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "S" COMMAND
 *   USED TO CREATE/EDIT/REMOVE/SHOW SENSOR DEFINITIONS
 */
	  return Sensor::parse(com+1);	  

#ifdef DCCPP_PRINT_DCCPP
	case 'Q':
		/**	\addtogroup commandsGroup
		SHOW STATUS OF ALL SENSORS
		--------------------------

		<b>
		\verbatim
		<Q>
		\endverbatim
		</b>

		returns: the status of each sensor ID in the form <b>\<Q ID\></b> (active) or <b>\<q ID\></b> (not active)
		*/

	  Sensor::status();
		return true;
#endif
#endif

	case 'w':      
		
		/**	\addtogroup commandsGroup
		WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON MAIN OPERATIONS TRACK
		----------------------------------------------------------------------------

		<b>
		\verbatim
		<w CAB CV VALUE>
		\endverbatim
		</b>

		writes, without any verification, a Configuration Variable to the decoder of an engine on the main operations track
    
		- <b>CAB</b>:  the short (1-127) or long (128-10293) address of the engine decoder 
		- <b>CV</b>: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
		- <b>VALUE</b>: the value to be written to the Configuration Variable memory location (0-255)
    
		returns: NONE
		*/    

	  DCCpp::mainRegs.writeCVByteMain(com+1);
		return true;


	case 'b':      
		/**	\addtogroup commandsGroup
		WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON MAIN OPERATIONS TRACK
		---------------------------------------------------------------------------

		<b>
		\verbatim
		<b CAB CV BIT VALUE>
		\endverbatim
		</b>

		writes, without any verification, a single bit within a Configuration Variable to the decoder of an engine on the main operations track
    
		- <b>CAB</b>:  the short (1-127) or long (128-10293) address of the engine decoder 
		- <b>CV</b>: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
		- <b>BIT</b>: the bit number of the Configuration Variable register to write (0-7)
		- <b>VALUE</b>: the value of the bit to be written (0-1)
    
		returns: NONE
		*/        

	  DCCpp::mainRegs.writeCVBitMain(com+1);
		return true;

	case 'W':      
		/**	\addtogroup commandsGroup
		WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON PROGRAMMING TRACK
		------------------------------------------------------------------------

		<b>
		\verbatim
		<W CV VALUE CALLBACKNUM CALLBACKSUB>
		\endverbatim
		</b>

		writes, and then verifies, a Configuration Variable to the decoder of an engine on the programming track
    
		- <b>CV</b>: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
		- <b>VALUE</b>: the value to be written to the Configuration Variable memory location (0-255) 
		- <b>CALLBACKNUM</b>: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
		- <b>CALLBACKSUB</b>: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
    
		returns: <b>\<r CALLBACKNUM|CALLBACKSUB|CV Value\></b>
		where VALUE is a number from 0-255 as read from the requested CV, or -1 if verification read fails
		*/    

	  DCCpp::progRegs.writeCVByte(com+1);
		return true;

	case 'B':      
		/**	\addtogroup commandsGroup
		WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON PROGRAMMING TRACK
		-----------------------------------------------------------------------

		<b>
		\verbatim
		<B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
		\endverbatim
		</b>

		writes, and then verifies, a single bit within a Configuration Variable to the decoder of an engine on the programming track
		
		- <b>CV</b>: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
		- <b>BIT</b>: the bit number of the Configuration Variable memory location to write (0-7)
		- <b>VALUE</b>: the value of the bit to be written (0-1)
		- <b>CALLBACKNUM</b>: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
		- <b>CALLBACKSUB</b>: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
		
		returns: <b>\<r CALLBACKNUM|CALLBACKSUB|CV BIT VALUE\></b>
		where VALUE is a number from 0-1 as read from the requested CV bit, or -1 if verification read fails
		*/

	  DCCpp::progRegs.writeCVBit(com+1);
		return true;

	case 'R':     
		/**	\addtogroup commandsGroup
		READ CONFIGURATION VARIABLE BYTE FROM ENGINE DECODER ON PROGRAMMING TRACK
		-------------------------------------------------------------------------

		<b>
		\verbatim
		<R CV CALLBACKNUM CALLBACKSUB>
		\endverbatim
		</b>

		reads a Configuration Variable from the decoder of an engine on the programming track
		
		- <b>CV</b>: the number of the Configuration Variable memory location in the decoder to read from (1-1024)
		- <b>CALLBACKNUM</b>: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
		- <b>CALLBACKSUB</b>: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
		
		returns: <b>\<r CALLBACKNUM|CALLBACKSUB|CV VALUE\></b>
		where VALUE is a number from 0-255 as read from the requested CV, or -1 if read could not be verified
		*/

	  DCCpp::progRegs.readCV(com+1);
		return true;

	case '1':      
		/**	\addtogroup commandsGroup
		TURN ON POWER FROM MOTOR SHIELD TO TRACKS
		-----------------------------------------

		<b>
		\verbatim
		<1>
		\endverbatim
		</b>

		enables power from the motor shield to the main operations and programming tracks
    
		returns: <b>\<p1\></b>
		*/    

	  DCCpp::powerOn();
		return true;
		  
	case '0':     
		/**	\addtogroup commandsGroup
		TURN OFF POWER FROM MOTOR SHIELD TO TRACKS
		------------------------------------------

		<b>
		\verbatim
		<0>
		\endverbatim
		</b>

		disables power from the motor shield to the main operations and programming tracks
    
		returns: <b>\<p0\></b>
		*/

		DCCpp::powerOff();
		return true;

	case 'c':     
		/**	\addtogroup commandsGroup
		READ MAIN OPERATIONS TRACK CURRENT
		----------------------------------

		<b>
		\verbatim
		<c>
		\endverbatim
		</b>

		reads current being drawn on main operations track
    
		returns: <b>\<a CURRENT\> </b>
		where CURRENT = 0-1024, based on exponentially-smoothed weighting scheme
		*/

	  DCCPP_INTERFACE.print("<a");
	  DCCPP_INTERFACE.print(int(DCCpp::getCurrentMain()));
	  DCCPP_INTERFACE.print(">");
#if !defined(USE_ETHERNET)
	  DCCPP_INTERFACE.println("");
#endif
		return true;

	case 's':
		/**	\addtogroup commandsGroup
		STATUS OF DCC++ BASE STATION
		----------------------------

		<b>
		\verbatim
		<s>
		\endverbatim
		</b>

		returns status messages containing track power status, throttle status, turn-out status, and a version number
		NOTE: this is very useful as a first command for an interface to send to this sketch in order to verify connectivity and update any GUI to reflect actual throttle and turn-out settings
    
		returns: series of status messages that can be read by an interface to determine status of DCC++ Base Station and important settings
		*/

	  if (DCCppConfig::SignalEnablePinMain == UNDEFINED_PIN || digitalRead(DCCppConfig::SignalEnablePinMain) == HIGH)
			DCCPP_INTERFACE.print("<p0>");
		if (DCCppConfig::SignalEnablePinProg == UNDEFINED_PIN || digitalRead(DCCppConfig::SignalEnablePinProg) == HIGH)
			DCCPP_INTERFACE.print("<p1>");
#if !defined(USE_ETHERNET)
	  DCCPP_INTERFACE.println("");
#endif

	  for(int i=1;i<=MAX_MAIN_REGISTERS;i++){
		if(DCCpp::mainRegs.speedTable[i]==0)
		  continue;
		DCCPP_INTERFACE.print("<T");
		DCCPP_INTERFACE.print(i); DCCPP_INTERFACE.print(" ");
		if(DCCpp::mainRegs.speedTable[i]>0){
		  DCCPP_INTERFACE.print(DCCpp::mainRegs.speedTable[i]);
		  DCCPP_INTERFACE.print(" 1>");
		} else{
		  DCCPP_INTERFACE.print(- DCCpp::mainRegs.speedTable[i]);
		  DCCPP_INTERFACE.print(" 0>");
		}          
#if !defined(USE_ETHERNET)
		DCCPP_INTERFACE.println("");
#endif
	  }
	  DCCPP_INTERFACE.print("<iDCCpp LIBRARY BASE STATION FOR ARDUINO ");
	  //DCCPP_INTERFACE.print(ARDUINO_TYPE);
	  //DCCPP_INTERFACE.print(" / ");
	  //DCCPP_INTERFACE.print(MOTOR_SHIELD_NAME);
	  DCCPP_INTERFACE.print(": V-");
	  DCCPP_INTERFACE.print(VERSION);
	  DCCPP_INTERFACE.print(" / ");
	  DCCPP_INTERFACE.print(__DATE__);
	  DCCPP_INTERFACE.print(" ");
	  DCCPP_INTERFACE.print(__TIME__);
	  DCCPP_INTERFACE.print(">");
#if !defined(USE_ETHERNET)
	  DCCPP_INTERFACE.println("");
#endif

	  DCCPP_INTERFACE.print("<N ");
#if defined(USE_ETHERNET)
		DCCPP_INTERFACE.print("ETHERNET :");
		DCCPP_INTERFACE.print(Ethernet.localIP());
		DCCPP_INTERFACE.print(">");
#if !defined(USE_ETHERNET)
		DCCPP_INTERFACE.println("");
#endif
#else
	  DCCPP_INTERFACE.println("SERIAL>");
#endif

#ifdef DCCPP_PRINT_DCCPP
#ifdef USE_TURNOUT
	  Turnout::show();
#endif
#ifdef USE_OUTPUT
	  Output::show();
#endif
#ifdef USE_SENSOR
	  Sensor::show();
#endif
#endif
		return true;

#ifdef USE_EEPROM
	case 'E':     
		/**	\addtogroup commandsGroup
		STORE SETTINGS IN EEPROM
		------------------------

		<b>
		\verbatim
		<E>
		\endverbatim
		</b>

		Stores settings for turnouts and sensors EEPROM
    
		returns: <b>\<e nTurnouts nSensors\></b>
		*/
	 
		EEStore::store();
		DCCPP_INTERFACE.print("<e ");
		DCCPP_INTERFACE.print(EEStore::data.nTurnouts);
		DCCPP_INTERFACE.print(" ");
		DCCPP_INTERFACE.print(EEStore::data.nSensors);
		DCCPP_INTERFACE.print(" ");
		DCCPP_INTERFACE.print(EEStore::data.nOutputs);
		DCCPP_INTERFACE.print(">");
#if !defined(USE_ETHERNET)
		DCCPP_INTERFACE.println("");
#endif
		return true;

	case 'e':     
		/**	\addtogroup commandsGroup
		CLEAR SETTINGS IN EEPROM
		------------------------

		<b>
		\verbatim
		<e>
		\endverbatim
		</b>

		clears settings for Turnouts in EEPROM
    
		returns: <b>\<O\></b>
		*/
	 
		EEStore::clear();
		DCCPP_INTERFACE.print("<O>");
#if !defined(USE_ETHERNET)
		DCCPP_INTERFACE.println("");
#endif
		return true;
#endif

	case ' ':
		/**	\addtogroup commandsGroup
		PRINT CARRIAGE RETURN IN SERIAL MONITOR WINDOW
		----------------------------------------------

		<b>
		\verbatim
		< >
		\endverbatim
		</b>

		simply prints a carriage return - useful when interacting with ArduIno through serial monitor window
    
		returns: a carriage return
		*/

	  DCCPP_INTERFACE.println("");
		return true;

///          
/// THE FOLLOWING COMMANDS ARE NOT NEEDED FOR NORMAL OPERATIONS AND ARE ONLY USED FOR TESTING AND DEBUGGING PURPOSES
/// PLEASE SEE SPECIFIC WARNINGS IN EACH COMMAND BELOW
///

	case 'D':       
		/**	\addtogroup commandsGroup
		ENTER DIAGNOSTIC MODE
		---------------------

		<b>
		\verbatim
		<D>
		\endverbatim
		</b>

		changes the clock speed of the chip and the pre-scaler for the timers so that you can visually see the DCC signals flickering with an LED
		SERIAL COMMUNICATION WILL BE INTERUPTED ONCE THIS COMMAND IS ISSUED - MUST RESET BOARD OR RE-OPEN SERIAL WINDOW TO RE-ESTABLISH COMMS
		*/

		Serial.println("\nEntering Diagnostic Mode...");
		delay(1000);

		DCCpp::setDebugDccMode();

		return true;

	case 'M':       
		/**	\addtogroup commandsGroup
		WRITE A DCC PACKE%T TO ONE OF THE REGISTERS DRIVING THE MAIN OPERATIONS TRACK
		-----------------------------------------------------------------------------

		<b>
		\verbatim
		<M REGISTER BYTE1 BYTE2 [BYTE3] [BYTE4] [BYTE5]>
		\endverbatim
		</b>

		writes a DCC packet of two, three, four, or five hexadecimal bytes to a register driving the main operations track
		fOR DEBUGGING AND TESTING PURPOSES ONLY.  DO NOT USE UNLESS YOU KNOW HOW TO CONSTRUCT NMRA DCC PACKETS - YOU CAN INADVERTENTLY RE-PROGRAM YOUR ENGINE DECODER
   
		- <b>REGISTE%R</b>: an internal register number, from 0 through MAX_MAIN_REGISTERS (inclusive), to write (if REGISTE%R=0) or write and store (if REGISTE%R>0) the packet 
		- <b>BYTE1</b>:  first hexadecimal byte in the packet
		- <b>BYTE2</b>:  second hexadecimal byte in the packet
		- <b>BYTE3</b>:  optional third hexadecimal byte in the packet
		- <b>BYTE4</b>:  optional fourth hexadecimal byte in the packet
		- <b>BYTE5</b>:  optional fifth hexadecimal byte in the packet
  
		returns: NONE   
		*/

	  DCCpp::mainRegs.writeTextPacket(com+1);
		return true;

	case 'P':       
		/**	\addtogroup commandsGroup
		WRITE A DCC PACKE%T TO ONE OF THE REGISTERS DRIVING THE MAIN OPERATIONS TRACK
		-----------------------------------------------------------------------------

		<b>
		\verbatim
		<P REGISTER BYTE1 BYTE2 [BYTE3] [BYTE4] [BYTE5]>
		\endverbatim
		</b>

		writes a DCC packet of two, three, four, or five hexadecimal bytes to a register driving the programming track
		FOR DEBUGGING AND TESTING PURPOSES ONLY.  DO NOT USE UNLESS YOU KNOW HOW TO CONSTRUCT NMRA DCC PACKETS - YOU CAN INADVERTENTLY RE-PROGRAM YOUR ENGINE DECODER
   
		- <b>REGISTE%R</b>: an internal register number, from 0 through MAX_MAIN_REGISTERS (inclusive), to write (if REGISTE%R=0) or write and store (if REGISTE%R>0) the packet 
		- <b>BYTE1</b>:  first hexadecimal byte in the packet
		- <b>BYTE2</b>:  second hexadecimal byte in the packet
		- <b>BYTE3</b>:  optional third hexadecimal byte in the packet
		- <b>BYTE4</b>:  optional fourth hexadecimal byte in the packet
		- <b>BYTE5</b>:  optional fifth hexadecimal byte in the packet
   
		returns: NONE   
		*/

	  DCCpp::progRegs.writeTextPacket(com+1);
		return true;
			
#ifndef VISUALSTUDIO
	case 'F':     
		/**	\addtogroup commandsGroup
		ATTEMPTS TO DETERMINE HOW MUCH FREE SRAM IS AVAILABLE IN ARDUINO
		----------------------------------------------------------------

		<b>
		\verbatim
		<F>
		\endverbatim
		</b>

		measure amount of free SRAM memory left on the Arduino based on trick found on the Internet.
		Useful when setting dynamic array sizes, considering the Uno only has 2048 bytes of dynamic SRAM.
		Unfortunately not very reliable --- would be great to find a better method
     
		returns: <b>\<f MEM\></b>
		where MEM is the number of free bytes remaining in the Arduino's SRAM
		*/

	  int v; 
		DCCPP_INTERFACE.print("<f");
#ifdef ARDUINO_ARCH_AVR
		DCCPP_INTERFACE.print((int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
#endif
		DCCPP_INTERFACE.print(">");
#if !defined(USE_ETHERNET)
		DCCPP_INTERFACE.println("");
#endif
		return true;
#endif

	case 'L':     
		/**	\addtogroup commandsGroup
		LISTS BIT CONTENTS OF ALL INTERNAL DCC PACKE%T REGISTERS
		-------------------------------------------------------

		<b>
		\verbatim
		<L>
		\endverbatim
		</b>

		lists the packet contents of the main operations track registers and the programming track registers
		FOR DIAGNOSTIC AND TESTING USE ONLY
		*/

	  DCCPP_INTERFACE.println("");
	  for(Register *p = DCCpp::mainRegs.reg; p <= DCCpp::mainRegs.maxLoadedReg;p++){
		DCCPP_INTERFACE.print("M"); DCCPP_INTERFACE.print((int)(p - DCCpp::mainRegs.reg)); DCCPP_INTERFACE.print(":\t");
		DCCPP_INTERFACE.print((int)p); DCCPP_INTERFACE.print("\t");
		DCCPP_INTERFACE.print((int)(p->activePacket)); DCCPP_INTERFACE.print("\t");
		DCCPP_INTERFACE.print(p->activePacket->nBits); DCCPP_INTERFACE.print("\t");
		for(int i=0;i<10;i++){
		  DCCPP_INTERFACE.print(p->activePacket->buf[i],HEX); DCCPP_INTERFACE.print("\t");
		}
		DCCPP_INTERFACE.println("");
	  }
	  for(Register *p = DCCpp::progRegs.reg; p <= DCCpp::progRegs.maxLoadedReg;p++){
		DCCPP_INTERFACE.print("P"); DCCPP_INTERFACE.print((int)(p - DCCpp::progRegs.reg)); DCCPP_INTERFACE.print(":\t");
		DCCPP_INTERFACE.print((int)p); DCCPP_INTERFACE.print("\t");
		DCCPP_INTERFACE.print((int)p->activePacket); DCCPP_INTERFACE.print("\t");
		DCCPP_INTERFACE.print(p->activePacket->nBits); DCCPP_INTERFACE.print("\t");
		for(int i=0;i<10;i++){
		  DCCPP_INTERFACE.print(p->activePacket->buf[i],HEX); DCCPP_INTERFACE.print("\t");
		}
		DCCPP_INTERFACE.println("");
	  }
	  DCCPP_INTERFACE.println("");
		return true;
	} // switch

	return false;
}; // SerialCommand::parse

///////////////////////////////////////////////////////////////////////////////

#endif
