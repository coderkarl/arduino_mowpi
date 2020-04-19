/*
  BladeControl.cpp
  Karl Kirsch, 4/2/2020
*/

//#include "Arduino.h"
#include "BladeControl.h"

//Class functions

BladeControl::BladeControl(byte pinA, byte pinB)
{
    bladeOffMillis = 0;
    bladeReadyMillis = 0;
    bladeStartMillis = 0;
    bladeOnMillis = 0;
    cmdMillis = 0;
    
    m_bladeControlState = BLADE_CONTROL_OFF_STATE;
    
    m_prevBladeState = -1;
    m_bladeState = BLADE_OFF_STATE;
    m_desState = BLADE_OFF_STATE;
    
    m_bladePinA = pinA;
    m_bladePinB = pinB;
    // prevent relay chatter?
    digitalWrite(m_bladePinA, HIGH);
    digitalWrite(m_bladePinB, HIGH);

    pinMode(m_bladePinA, OUTPUT);
    pinMode(m_bladePinB, OUTPUT);

    turn_blade_off();
    cmdMillis = millis(); 
}

//*************Motor Control Functions*****************
int BladeControl::update_blade(int desBladeControl, int& cmdA, int& cmdB)
{
  //debounce control_state switching
  if(desBladeControl != m_bladeControlState && (millis() - cmdMillis) > DEBOUNCE_MILLIS)
  {
    m_bladeControlState = desBladeControl;
  }
  else if(desBladeControl == m_bladeControlState)
  {
    cmdMillis = millis();
  }
  //DONE debounce state switching

  m_prevBladeState = m_bladeState;
  //Serial.print("prev: "); Serial.println(prevBladeState);

  if(m_bladeControlState == BLADE_CONTROL_OFF_STATE)
  {
    m_desState = BLADE_OFF_STATE;
    m_bladeState = BLADE_OFF_STATE;
    if(m_prevBladeState != BLADE_OFF_STATE)
    {
      cmdA = 0;
      cmdB = 0;
      turn_blade_off();
    }
  }

  switch (m_bladeState)
  {
    case BLADE_OFF_STATE:
      if(m_bladeControlState == BLADE_CONTROL_ON_STATE)
      {
          m_desState = BLADE_READY_STATE;
      }
      
      if(m_desState == BLADE_READY_STATE && (millis()-bladeOffMillis) > MIN_OFF_TIME)
      {
        m_bladeState = BLADE_READY_STATE;
        cmdA = 1;
        cmdB = 0;
        digitalWrite(m_bladePinA, RAW_BLADE_SW_ON); // LOW is ON for active low
        digitalWrite(m_bladePinB, RAW_BLADE_SW_OFF); // HIGH is OFF for active low
        bladeReadyMillis = millis();
        //Serial.println("BLADE_READY");
      }
      break;
    case BLADE_READY_STATE:
      //Serial.println("blade_ready");
      if(m_bladeControlState == BLADE_CONTROL_ON_STATE)
      {
          m_desState = BLADE_START_STATE;
      }
      
      if(m_desState == BLADE_START_STATE && (millis()-bladeReadyMillis) > MIN_READY_TIME)
      {
        m_bladeState = BLADE_START_STATE;
        cmdA = 1;
        cmdB = 1;
        digitalWrite(m_bladePinA, RAW_BLADE_SW_ON); // LOW is ON for active low
        digitalWrite(m_bladePinB, RAW_BLADE_SW_ON); // HIGH is OFF for active low
        bladeStartMillis = millis();
        //Serial.println("BLADE_START");
      }
      break;
    case BLADE_START_STATE:
      if(m_bladeControlState == BLADE_CONTROL_ON_STATE)
      {
          m_desState = BLADE_ON_STATE;
      }
      
      if(m_desState == BLADE_OFF_STATE || m_desState == BLADE_READY_STATE)
      {
        m_bladeState = BLADE_OFF_STATE;
        cmdA = 0;
        cmdB = 0;
        turn_blade_off();
      }
      else if(m_desState == BLADE_ON_STATE && (millis()-bladeStartMillis) > MIN_START_TIME)
      {
        m_bladeState = BLADE_ON_STATE;
        cmdA = 0;
        cmdB = 1;
        digitalWrite(m_bladePinA, RAW_BLADE_SW_OFF); // LOW is ON for active low
        digitalWrite(m_bladePinB, RAW_BLADE_SW_ON); // HIGH is OFF for active low
        bladeOnMillis = millis();
        //Serial.println("BLADE_ON");
      }
      break;
    case BLADE_ON_STATE:      
      if(m_desState == BLADE_OFF_STATE || m_desState == BLADE_READY_STATE)
      {
        m_bladeState = BLADE_OFF_STATE;
        cmdA = 0;
        cmdB = 0;
        turn_blade_off();
      }
      break;
    default:
      m_bladeState = BLADE_OFF_STATE;
      cmdA = 0;
      cmdB = 0;
      turn_blade_off();
  }
  return m_bladeState;
}

void BladeControl::turn_blade_off()
{
  digitalWrite(m_bladePinA, RAW_BLADE_SW_OFF); // HIGH is OFF for active low
  digitalWrite(m_bladePinB, RAW_BLADE_SW_OFF); // HIGH is OFF for active low
  bladeOffMillis = millis();
}
