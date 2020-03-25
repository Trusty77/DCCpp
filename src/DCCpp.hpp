//-------------------------------------------------------------------
#ifndef __DCCpp_hpp__
#define __DCCpp_hpp__
//-------------------------------------------------------------------

#include "DCCpp.h"

/**
This is a class to handle decoder functions.
An instance of this class handle the status of the functions of one decoder.
A function can be active or not. 
*/
class FunctionsState
{
	private:
		/**Four bytes : 32 bits : from function 0 to 28, maximum for DCC.
		A bit at true is an activated function.*/
		byte activeFlags[4];

		/**This is the last states sent to the decoder by DCC.
		Used to be sure to send only changed states.
		*/
		byte activeFlagsSent[4];

		inline byte byteNumber(byte inFunctionNumber) { return inFunctionNumber / 8; }
		inline byte bitNumber(byte inFunctionNumber) { return inFunctionNumber % 8; }

	public:
		/** Initialize the instance.
		*/
		FunctionsState();

		/** Reset all functions to inactive.
		*/
		void clear();
		/** Activate one function. The allowed number goes from 0 to 28, maximum for DCC.
		@param inFunctionNumber	Number of the function to activate.
		*/
		void activate(byte inFunctionNumber);
		/** Inactivate one function. The allowed number goes from 0 to 28, maximum for DCC.
		@param inFunctionNumber	Number of the function to inactivate.
		*/
		void inactivate(byte inFunctionNumber);
		/** Check if the given function is activated. The allowed number goes from 0 to 28, maximum for DCC.
		@param inFunctionNumber	Number of the function to activate.
		@return True if the given function is activated.
		*/
		bool isActivated(byte inFunctionNumber);
		/**Copy the current active flags into 'sent' active flags.*/
		void statesSent();
		/** Check if the given function had its activation flag changed when sent to the decoder last time.
		The allowed number goes from 0 to 28, maximum for DCC.
		@param inFunctionNumber	Number of the function to check.
		@return True if the given function activation flag is different between activFlags and sentActiveFlags.
		*/
		bool isActivationChanged(byte inFunctionNumber);

#ifdef DCCPP_DEBUG_MODE
		/** Print the list of activated functions.
		@remark Only available if DCCPP_DEBUG_MODE is defined.
		*/
		void printActivated();
#endif
};

/**
This is the main class of the library. All data and functions are static.
There is no needs to instantiate this class.
*/
class DCCpp
{
	private:
		static bool programMode;
		static bool panicStopped;

		static bool setThrottle(volatile RegisterList *inReg, int nReg, int inLocoId, int inStepsNumber, int inNewSpeed, bool inForward);
		//static int readCv(volatile RegisterList *inReg, byte inCvId, int callBack = 100, int callBackSub = 200) { return inReg->readCV(inCvId, callBack, callBackSub); }
		static void writeCv(volatile RegisterList *inReg, int inCvId, byte inCvValue, int callBack = 100, int callBackSub = 200);
		static int identifyLocoId(volatile RegisterList *inReg);
		static void setFunctions(volatile RegisterList *inReg, int nReg, int inLocoId, FunctionsState &inStates);

	public:
		static byte ackThreshold;
		static volatile RegisterList mainRegs, progRegs;
		static CurrentMonitor mainMonitor;
		static CurrentMonitor progMonitor;
	
	public:
		// begins
		/** Begins the DCCpp library.
		*/
		static void begin();
#ifndef USE_ONLY1_INTERRUPT
		/** Initializes the main track.
		@param inOptionalDirectionMotor	Pin for the rerouting of shields direction pin, set it to UNDEFINED_PIN if not used.
		@param inSignalPin	Pin for the signal pin, the one driven by an interruption, set it to UNDEFINED_PIN if not used (but the line will be always down...).
		@param inSignalEnablePin	Pin for the enable/PWM pin, set it to UNDEFINED_PIN if not used.
		@param inCurrentMonitor	Pin for the current monitor analog pin, set it to UNDEFINED_PIN if not used.
		*/
		static void beginMain(uint8_t inOptionalDirectionMotor, uint8_t inSignalPin, uint8_t inSignalEnablePin, uint8_t inCurrentMonitor);
		/** Initializes the programming track.
		@param inOptionalDirectionMotor	Pin for the rerouting of shields direction pin, set it to UNDEFINED_PIN if not used.
		@param inSignalPin	Pin for the signal pin, the one driven by an interruption, set it to UNDEFINED_PIN if not used (but the line will be always down...).
		@param inSignalEnablePin	Pin for the enable/PWM pin, set it to UNDEFINED_PIN if not used.
		@param inCurrentMonitor	Pin for the current monitor analog pin, set it to UNDEFINED_PIN if not used.
		*/
		static void beginProg(uint8_t inOptionalDirectionMotor, uint8_t inSignalPin, uint8_t inSignalEnablePin, uint8_t inCurrentMonitor);

#ifdef ARDUINO_ARCH_AVR
		/** Initializes the main track for an Arduino Motor Shield.
		*/
		static inline void beginMainMotorShield() { beginMain(MOTOR_SHIELD_DIRECTION_MOTOR_CHANNEL_PIN_A, DCC_SIGNAL_PIN_MAIN, MOTOR_SHIELD_SIGNAL_ENABLE_PIN_MAIN, MOTOR_SHIELD_CURRENT_MONITOR_PIN_MAIN); }
		/** Initializes the programming track for an Arduino Motor Shield.
		*/
		static inline void beginProgMotorShield() { beginProg(MOTOR_SHIELD_DIRECTION_MOTOR_CHANNEL_PIN_B, DCC_SIGNAL_PIN_PROG, MOTOR_SHIELD_SIGNAL_ENABLE_PIN_PROG, MOTOR_SHIELD_CURRENT_MONITOR_PIN_PROG); }

		/** Initializes the main track for a Pololu MC33926 Motor Shield.
		*/
		static inline void beginMainPololu() { beginMain(POLOLU_DIRECTION_MOTOR_CHANNEL_PIN_A, DCC_SIGNAL_PIN_MAIN, POLOLU_SIGNAL_ENABLE_PIN_MAIN, POLOLU_CURRENT_MONITOR_PIN_MAIN); }
		/** Initializes the programming track for a Pololu MC33926 Motor Shield.
		*/
		static inline void beginProgPololu() { beginProg(POLOLU_DIRECTION_MOTOR_CHANNEL_PIN_B, DCC_SIGNAL_PIN_PROG, POLOLU_SIGNAL_ENABLE_PIN_PROG, POLOLU_CURRENT_MONITOR_PIN_PROG); }
#endif
#else
		/** Initializes the main track.
		@param inOptionalDirectionMotor	Pin for the rerouting of shields direction pin, set it to UNDEFINED_PIN if not used.
		@param inSignalPin	Pin for the signal pin, the one driven by an interruption, set it to UNDEFINED_PIN if not used (but the line will be always down...).
		@param inSignalEnablePin	Pin for the enable/PWM pin, set it to UNDEFINED_PIN if not used.
		@param inCurrentMonitor	Pin for the current monitor analog pin, set it to UNDEFINED_PIN if not used.
		*/
		static void beginMain(uint8_t inSignalPin, uint8_t inSignalEnablePin, uint8_t inCurrentMonitor);
		/** Initializes the programming track.
		@param inOptionalDirectionMotor	Pin for the rerouting of shields direction pin, set it to UNDEFINED_PIN if not used.
		@param inSignalPin	Pin for the signal pin, the one driven by an interruption, set it to UNDEFINED_PIN if not used (but the line will be always down...).
		@param inSignalEnablePin	Pin for the enable/PWM pin, set it to UNDEFINED_PIN if not used.
		@param inCurrentMonitor	Pin for the current monitor analog pin, set it to UNDEFINED_PIN if not used.
		*/
		static void beginProg(uint8_t inSignalPin, uint8_t inSignalEnablePin, uint8_t inCurrentMonitor);

#ifdef ARDUINO_ARCH_AVR
		/** Initializes the main track for an Arduino Motor Shield.
		*/
		static inline void beginMainMotorShield() { beginMain(DCC_SIGNAL_PIN_MAIN, MOTOR_SHIELD_SIGNAL_ENABLE_PIN_MAIN, MOTOR_SHIELD_CURRENT_MONITOR_PIN_MAIN); }
		/** Initializes the programming track for an Arduino Motor Shield.
		*/
		static inline void beginProgMotorShield() { beginProg(DCC_SIGNAL_PIN_PROG, MOTOR_SHIELD_SIGNAL_ENABLE_PIN_PROG, MOTOR_SHIELD_CURRENT_MONITOR_PIN_PROG); }

		/** Initializes the main track for a Pololu MC33926 Motor Shield.
		*/
		static inline void beginMainPololu() { beginMain(DCC_SIGNAL_PIN_MAIN, POLOLU_SIGNAL_ENABLE_PIN_MAIN, POLOLU_CURRENT_MONITOR_PIN_MAIN); }
		/** Initializes the programming track for a Pololu MC33926 Motor Shield.
		*/
		static inline void beginProgPololu() { beginProg(DCC_SIGNAL_PIN_PROG, POLOLU_SIGNAL_ENABLE_PIN_PROG, POLOLU_CURRENT_MONITOR_PIN_PROG); }
#endif
#endif

		// The next two functions are redefined for each kind of Arduino... See DccSignal*.cpp
		/** Initializes the main track.
		@param inSignalPin	Pin for the signal pin, the one driven by an interruption, set it to UNDEFINED_PIN if not used (but the line will be always down...).
		*/
		static void beginMainDccSignal(uint8_t inSignalPin);
		/** Initializes the programming track.
		@param inSignalPin	Pin for the signal pin, the one driven by an interruption, set it to UNDEFINED_PIN if not used (but the line will be always down...).
		*/
		static void beginProgDccSignal(uint8_t inSignalPin);
		/** Set the tracks in slow 'debug' mode
		only for tests.
		*/
		static void setDebugDccMode();

//#ifdef USE_ETHERNET
#if defined(USE_ETHERNET)
		/** Initializes the Ethernet link.
		@param inMac Mac address of this network element.
		@param inIp	IP address, or NULL for DHCP configuration.
		@param inProtocol Can be HTTP or TCP. Default is TCP.
		*/
		static void beginEthernet(uint8_t *inMac, uint8_t *inIp, EthernetProtocol inProtocol = EthernetProtocol::TCP);
#endif

//#ifdef USE_WIFI
#if defined(USE_WIFI)
		/** Initializes the Wifi link.
		@param inSsid SSID of this network element.
		@param inPassword	PASSWORD of this network element.
		@param inProtocol Can be HTTP or TCP. Default is TCP.
		*/
		static void beginWifi(const char *inSsid, const char *inPassword, EthernetProtocol inProtocol = EthernetProtocol::TCP);
#endif

		// DCCpp global functions

		/** Checks if the given RegisterList is from the main track or not.
		@param apRegs RegisterList to check.
		@return true if the RegisterList is mainRegs, the one from the main track.
		*/
		static bool IsMainTrack(volatile RegisterList *apRegs) { return apRegs == &mainRegs; }
		
		/** Main loop function of the library.
		*/
		static void loop();

		/** Stop/restore the power on all the tracks.
		@param inStop If true, stop the power, otherwise restore the power.
		*/
		static void panicStop(bool inStop);

		/** Set the power on all the tracks by default, or on the chosen track according .
		@param inMain If true, power on the main track.
		@param inProg If true, power on the programmation track.
		*/
		static void powerOn(bool inMain = true, bool inProg = true);

		/** Stop the power on all the tracks by default, or on the chosen track according .
		@param inMain If true, power off the main track.
		@param inProg If true, power off the programmation track.
		*/
		static void powerOff(bool inMain = true, bool inProg = true);

		/** Set the minimum threshold value to validate a CV reading or writing..
		@param inNewValue	Maximum value between 0 and 1023. Default is 30. The threshold that the exponentially-smoothed analogRead samples (after subtracting the baseline current) must cross to establish ACKNOWLEDGEMENT.
		@return Previous value.
		*/
		static byte setAckThreshold(byte inNewValue);

		/** Set the maximum current value before an event 'too much current consumption detected !' for main track.
		@param inMax	Maximum value between 0 and 1023. Default is 300.
		*/
		static inline void setCurrentSampleMaxMain(float inMax) { mainMonitor.currentSampleMax = inMax; }

		/** Set the maximum current value before an event 'too much current consumption detected !' for programming track.
		@param inMax	Maximum value between 0 and 1023. Default is 300.
		*/
		static inline void setCurrentSampleMaxProg(float inMax) { progMonitor.currentSampleMax = inMax; }

		/** Get the actual analog level for the current detection pin for the main track.
		@return Current value between 0 and 1023 using an exponential smoother...
		*/
		static inline float getCurrentMain() { return mainMonitor.pin == UNDEFINED_PIN ? 0 : mainMonitor.current; }

		/** Get the actual analog level for the current detection pin for the programming track.
		@return Current value between 0 and 1023.
		*/
		static inline float getCurrentProg() { return progMonitor.pin == UNDEFINED_PIN ? 0 : progMonitor.current; }

		// Main track functions

		/** For the given decoder id, set the speed and the direction on the main track.
		@param nReg	Register number. Avoid register 0, used for one shot commands like accessories or CV programming.
		@param inLocoId	Decoder address in short or long format.
		@param inStepsNumber	According to the decoder configuration, set it to 14, 28 or 128 .
		@param inNewSpeed	Speed of the loco, between 2 and the steps number - 1 (13, 27 or 127). 0 means normal complete stop. 1 means emergency stop.
		@param inForward	True means forward move, false means backward.	
		*/
		static inline bool setSpeedMain(int nReg, int inLocoId, int inStepsNumber, int inNewSpeed, bool inForward) { return setThrottle(&(mainRegs), nReg, inLocoId, inStepsNumber, inNewSpeed, inForward); }

		/** Try to identify the address of a decoder on the main track. Be sure there is only one loco on the track to call this function !
		@return CV 1 value: the loco decoder Id or -1 if no decoder identified.
		*/
		static inline int identifyLocoIdMain() { return identifyLocoId(&(mainRegs)); }

		/** Try to read a CV from a decoder on the main track.
		Be sure there is only one loco on the track before calling this function !
		@param inCvId	CV id from 0 to 255.
		@param callBack		an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function. Default 100.
		@param callBackSub	a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function. Default 200
		@return CV value: the CV value if the value cannot be read.
		*/
		static inline int readCvMain(int inCvId, int callBack = 100, int callBackSub = 200) { return mainRegs.readCVmain(inCvId, callBack, callBackSub); }

		/** Write the given CV on the main track.
		Be sure there is only one loco on the track before calling this function !
		@param inCvId	CV id from 0 to 255.
		@param inValue	CV new value from 0 to 255.
		@param callBack		an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function. Default 100.
		@param callBackSub	a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function. Default 200
		*/
		static inline void writeCvMain(int inCvId, byte inValue, int callBack = 100, int callBackSub = 200) { writeCv(&(mainRegs), inCvId, inValue, callBack, callBackSub); }

		/** Set the functions states of the given decoder on the main track.
		@param nReg	Register number. Avoid register 0, used for one shot commands like accessories or CV programming.
		@param inLocoId	Decoder address in short or long format.
		@param inStates	FunctionsState class with the wanted new status.
		*/
		static inline void setFunctionsMain(int nReg, int inLocoId, FunctionsState &inStates) { setFunctions(&(mainRegs), nReg, inLocoId, inStates); }

		// Programming track functions

		/** For the given decoder id, set the speed and the direction on the programming track.
		@param nReg	Register number. Avoid register 0, used for one shot commands like accessories or CV programming.
		@param inLocoId	Decoder address in short or long format.
		@param inStepsNumber	According to the decoder configuration, set it to 14, 28 or 128 .
		@param inNewSpeed	Speed of the loco, between 2 and the steps number - 1 (13, 27 or 127). 0 means normal complete stop. 1 means emergency stop.
		@param inForward	True means forward move, false means backward.
		*/
		static inline bool setSpeedProg(int nReg, int inLocoId, int inStepsNumber, int inNewSpeed, bool inForward) { return setThrottle(&(progRegs), nReg, inLocoId, inStepsNumber, inNewSpeed, inForward); }

		/** Try to identify the address of a decoder on the programming track. Be sure there is only one loco on the track to call this function !
		@return CV 1 value: the loco decoder Id or -1 if no decoder identified.
		*/
		static inline int identifyLocoIdProg() { return identifyLocoId(&(progRegs)); }

		/** Try to read a CV from a decoder on the programming track.
		Be sure there is only one loco on the track before calling this function !
		@param inCvId	CV id from 0 to 255.
		@param callBack		an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function. Default 100.
		@param callBackSub	a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function. Default 200
		@return CV value: the CV value if the value cannot be read.
		*/
		static inline int readCvProg(int inCvId, int callBack = 100, int callBackSub = 200) { return progRegs.readCV(inCvId, callBack, callBackSub); }

		/** Write the given CV on the programming track.
		@param inCvId	CV id from 0 to 255.
		@param inValue	CV new value from 0 to 255.
		@param callBack		an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function. Default 100.
		@param callBackSub	a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function. Default 200
		*/
		static inline void writeCvProg(int inCvId, byte inValue, int callBack = 100, int callBackSub = 200) { writeCv(&(progRegs), inCvId, inValue, callBack, callBackSub); }

		/** Set the functions states of the given decoder on the programming track.
		@param nReg	Register number. Avoid register 0, used for one shot commands like accessories or CV programming.
		@param inLocoId	Decoder address in short or long format.
		@param inStates	FunctionsState class with the wanted new status.
		*/
		static inline void setFunctionsProg(int nReg, int inLocoId, FunctionsState &inStates) { setFunctions(&(progRegs), nReg, inLocoId, inStates); }

		// Accessories

		/** Activates or not the given accessory
		@param inAddress	main address of the accessory decoder, from 0 to 511.
		@param inSubAddress	accessory number inside the decoder, from 0 to 3.
		@param inActivate	True to activate the accessory, false to deactivate.
		*/
		static void setAccessory(int inAddress, byte inSubAddress, byte inActivate);

public:

#ifdef DCCPP_DEBUG_MODE
		/** BEFORE activating the DCC mode, this function will check for power connections by setting the voltage alternatively in one side and the other for the given delay.
		If two leds are connected to the power module to check the DCC signal, they will go on alternatively.
		@param aDirPin pin for the direction of the tension.
		@param inDelay milliseconds to see the test.
		*/
		static void CheckPowerConnectionsWithLeds(uint8_t aDirPin, unsigned int inDelay);
#endif

#ifdef DCCPP_PRINT_DCCPP
	/** This define is empty if DCCPP_PRINT_DCCPP is not defined. */
	#define PRINT_DCCPP	DCCpp::showConfiguration();
		/** Print the list of activated functions.
		@remark Only available if DCCPP_PRINT_DCCPP is defined.
		*/
		static void showConfiguration();
#else
	#define PRINT_DCCPP
#endif
};

//-------------------------------------------------------------------
#endif
//-------------------------------------------------------------------
