/**********************************************************************

Sensor.cpp
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "Sensor.h"
#ifdef USE_SENSOR
#include "DCCpp_Uno.h"
#include "EEStore.h"
#ifdef USE_EEPROM
#include "EEPROM.h"
#endif
#include "Comm.h"

///////////////////////////////////////////////////////////////////////////////

void Sensor::begin(int snum, int pin, int pullUp) {
#ifdef DCCPP_DEBUG_MODE
	if (EEStore::eeStore != NULL)
	{
		INTERFACE.println(F("Sensor::begin() must be called BEFORE DCCpp.begin() !"));
	}
#endif

	if (firstSensor == NULL) {
		firstSensor = this;
	}
	else if (get(snum) == NULL) {
		Sensor *tt = firstSensor;
		while (tt->nextSensor != NULL)
			tt = tt->nextSensor;
		tt->nextSensor = this;
	}

	this->set(snum, pin, pullUp);

#ifdef DCCPP_DEBUG_MODE
	INTERFACE.println("<O>");
#endif
}

///////////////////////////////////////////////////////////////////////////////

void Sensor::set(int snum, int pin, int pullUp) {
	this->data.snum = snum;
	this->data.pin = pin;
	this->data.pullUp = (pullUp == 0 ? LOW : HIGH);
	this->active = false;
	this->signal = 1;
#ifdef VISUALSTUDIO
	ArduiEmulator::Arduino::dontCheckNextPinAccess = true;
#endif
	digitalWrite(pin, pullUp);   // don't use Arduino's internal pull-up resistors for external infrared sensors --- each sensor must have its own 1K external pull-up resistor
#ifdef VISUALSTUDIO
	ArduiEmulator::Arduino::dontCheckNextPinAccess = true;
#endif
	pinMode(pin, INPUT);         // force mode to input
}

///////////////////////////////////////////////////////////////////////////////

Sensor* Sensor::get(int n) {
	Sensor *tt;
	for (tt = firstSensor; tt != NULL && tt->data.snum != n; tt = tt->nextSensor);
	return(tt);
}
///////////////////////////////////////////////////////////////////////////////

void Sensor::remove(int n) {
	Sensor *tt, *pp;

	for (tt = firstSensor; tt != NULL && tt->data.snum != n; pp = tt, tt = tt->nextSensor);

	if (tt == NULL) {
		INTERFACE.println("<X>");
		return;
	}

	if (tt == firstSensor)
		firstSensor = tt->nextSensor;
	else
		pp->nextSensor = tt->nextSensor;

	free(tt);

	INTERFACE.println("<O>");
}

///////////////////////////////////////////////////////////////////////////////

int Sensor::count() {
	int count = 0;
	Sensor *tt;
	for (tt = firstSensor; tt != NULL; tt = tt->nextSensor)
		count++;
	return count;
}

///////////////////////////////////////////////////////////////////////////////
  
void Sensor::check(){
  Sensor *tt;

  for(tt=firstSensor;tt!=NULL;tt=tt->nextSensor){
    tt->signal=(float)(tt->signal*(1.0-SENSOR_DECAY)+digitalRead(tt->data.pin)*SENSOR_DECAY);
    
    if(!tt->active && tt->signal<0.5){
      tt->active=true;
      INTERFACE.print("<Q");
      INTERFACE.print(tt->data.snum);
      INTERFACE.println(">");
    } else if(tt->active && tt->signal>0.9){
      tt->active=false;
      INTERFACE.print("<q");
      INTERFACE.print(tt->data.snum);
      INTERFACE.println(">");
    }
  } // loop over all sensors
    
} // Sensor::check

#ifdef DCCPP_PRINT_DCCPP
  ///////////////////////////////////////////////////////////////////////////////

void Sensor::show() {
	Sensor *tt;

	if (firstSensor == NULL) {
		INTERFACE.println("<X>");
		return;
	}

	for (tt = firstSensor; tt != NULL; tt = tt->nextSensor) {
		INTERFACE.print("<Q");
		INTERFACE.print(tt->data.snum);
		INTERFACE.print(" ");
		INTERFACE.print(tt->data.pin);
		INTERFACE.print(" ");
		INTERFACE.print(tt->data.pullUp);
		INTERFACE.println(">");
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sensor::status() {
	Sensor *tt;

	if (firstSensor == NULL) {
		INTERFACE.println("<X>");
		return;
	}

	for (tt = firstSensor; tt != NULL; tt = tt->nextSensor) {
		INTERFACE.print(tt->active ? "<Q" : "<q");
		INTERFACE.print(tt->data.snum);
		INTERFACE.println(">");
	}
}

#endif

#ifdef USE_EEPROM
///////////////////////////////////////////////////////////////////////////////

void Sensor::load() {
	struct SensorData data;
	Sensor *tt;

	for (int i = 0; i<EEStore::eeStore->data.nSensors; i++) {
#ifdef VISUALSTUDIO
		EEPROM.get(EEStore::pointer(), (void *)&(data), sizeof(SensorData));	// ArduiEmulator version...
#else
		EEPROM.get(EEStore::pointer(), data);
#endif
#if defined(USE_TEXTCOMMAND)
		tt = create(data.snum, data.pin, data.pullUp);
#else
		tt = get(data.snum);
#ifdef DCCPP_DEBUG_MODE
		if (tt == NULL)
			INTERFACE.println(F("Sensor::begin() must be called BEFORE Sensor::load() !"));
		else
#endif
			tt->set(data.snum, data.pin, data.pullUp);
#endif
		EEStore::advance(sizeof(tt->data));
	}
}

///////////////////////////////////////////////////////////////////////////////

void Sensor::store() {
	Sensor *tt;

	tt = firstSensor;
	EEStore::eeStore->data.nSensors = 0;

	while (tt != NULL) {
#ifdef VISUALSTUDIO
		EEPROM.put(EEStore::pointer(), (void *)&(tt->data), sizeof(SensorData));	// ArduiEmulator version...
#else
		EEPROM.put(EEStore::pointer(), tt->data);
#endif
		EEStore::advance(sizeof(tt->data));
		tt = tt->nextSensor;
		EEStore::eeStore->data.nSensors++;
	}
}
#endif

#if defined(USE_TEXTCOMMAND)
///////////////////////////////////////////////////////////////////////////////

void Sensor::parse(char *c) {
	int n, s, m;
	//  Sensor *t;

	switch (sscanf(c, "%d %d %d", &n, &s, &m)) {

	case 3:                     // argument is string with id number of sensor followed by a pin number and pullUp indicator (0=LOW/1=HIGH)
		create(n, s, m);
		break;

	case 1:                     // argument is a string with id number only
		remove(n);
		break;

#ifdef DCCPP_PRINT_DCCPP
	case -1:                    // no arguments
		show();
		break;

	case 2:                     // invalid number of arguments
		INTERFACE.print("<X>");
		break;
#endif
	}
}

///////////////////////////////////////////////////////////////////////////////

Sensor *Sensor::create(int snum, int pin, int pullUp) {
	Sensor *tt = new Sensor();

	if (tt == NULL) {       // problem allocating memory
#ifdef DCCPP_DEBUG_MODE
		INTERFACE.println("<X>");
#endif
		return(tt);
	}

	tt->begin(snum, pin, pullUp);

	return(tt);
}

#endif

///////////////////////////////////////////////////////////////////////////////

Sensor *Sensor::firstSensor=NULL;

#endif