/*
  HM55BCompass.h - 
*/
#ifndef BladeControl_h
#define BladeControl_h

#include "Arduino.h"

#define BLADE_CONTROL_OFF_STATE 0
#define BLADE_CONTROL_ON_STATE 1

#define BLADE_OFF_STATE 0
#define BLADE_READY_STATE 1
#define BLADE_START_STATE 2
#define BLADE_ON_STATE 3
#define DEBOUNCE_MILLIS 100
#define OUT_INTERVAL 500

#define RAW_BLADE_SW_ON HIGH
#define RAW_BLADE_SW_OFF LOW

#define MIN_OFF_TIME 5000
#define MIN_READY_TIME 1500
#define MIN_START_TIME 1000

class BladeControl
{
  public:
    BladeControl(byte pinA, byte pinB);
    int update_blade(int desBladeControl, int& cmdA, int& cmdB);
  private:
    byte m_bladePinA, m_bladePinB;
    
    int m_bladeControlState;
    
    int m_prevBladeState;
    int m_bladeState;
    int m_desState;
    
    unsigned long bladeOffMillis;
    unsigned long bladeReadyMillis;
    unsigned long bladeStartMillis;
    unsigned long bladeOnMillis;
    unsigned long cmdMillis;
    
    void turn_blade_off();
};

#endif
