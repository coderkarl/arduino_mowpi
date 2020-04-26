// https://github.com/Nikkilae/PPM-reader
#include <PPMReader.h>
// #include <InterruptHandler.h>   <-- You may need this on some versions of Arduino

#include <BladeControl.h>
// ln -s <full BladeControl path> <full Arduino libraries path/BladeControl>
// ln -s ~/Projects/mower_chassis/arduino_mowpi/BladeControl ~/Arduino/libraries/BladeControl

#define DEBUG_PERIOD 500
#define CMD_DEBOUNCE 100

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
// PPMReader uses hard interrupts, uno pins 2 or 3
// attachInterrupt(digitalPinToInterrupt(interruptPin), RISING);
int interruptPin = 3;
int channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);

long timeDEBUG, timeCMD;

int16_t steer_pwm = 1500;
int16_t speed_pwm = 1500;
int16_t blade_pwm = 1500;
int16_t auto_pwm = 1500;

int16_t scaled_speed_power = 0;
int16_t scaled_steer_power = 0;

//RC CHANNELS
#define RC_STEER 1
#define RC_SPEED 2
#define RC_THROTTLE 3
#define RC_BLADE 4
#define RC_AUTO 5

//**************Encoders*************
#include <PinChangeInt.h>
#define RIGHT_ENC_A A2
#define RIGHT_ENC_B A3
#define LEFT_ENC_A A4
#define LEFT_ENC_B A5

int16_t enc_left = 0, enc_right = 0;
unsigned long serialdata;
int inbyte = 0;

//********** Blade Control ************
# define BLADE_PIN_A 11
# define BLADE_PIN_B 12
BladeControl blade_control(BLADE_PIN_A, BLADE_PIN_B);

// Sabertooth Describe configuration
//   Set the battery type and voltage fixed in Describe.
//   Otherwise it assumes full voltage at startup
//   Reverse motor direction(s) if needed

#include <USBSabertooth_NB.h>

#include <AltSoftSerial.h>

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Teensy 3.0 & 3.1  21        20         22
// Teensy 2.0         9        10       (none)
// Teensy++ 2.0      25         4       26, 27
// Arduino Uno        9         8         10
// Arduino Leonardo   5        13       (none)
// Arduino Mega      46        48       44, 45

AltSoftSerial altSerial;
USBSabertoothSerial saberSerial(altSerial);
USBSabertooth ST(saberSerial, 128); // Address 128, and use saberSerial as the serial port.

// Sabertooth feedback data
int battery_ST = 0;
int current1_ST = 0;
double filt_current1_ST = 0;
int current2_ST = 0;
double filt_current2_ST = 0;

//CircularBuffer current1_buffer(10);

#define LEFT_MOTOR 1
#define RIGHT_MOTOR 2

// With USBSabertooth, use:
//  ST.drive(-2047 to 2047) in manual mode
//  ST.turn(-2047 to 2047) in manual mode
//  ST.motor(LEFT_MOTOR, -2047 to 2047) in auto mode given speed m/s, omega rad/sec
//  ST.motor(RIGHT_MOTOR, -2047 to 2047) in auto mode given speed, omega
//  Consider always using PID for motor left/right given speed, omega
//    Map manual mode speed, steer pwm to speed, omega

void setup() {
  Serial.begin(115200);
  timeDEBUG = millis();
  timeCMD = millis();

  altSerial.begin(9600);
  //ST.autobaud();


  //pinMode(7, INPUT_PULLUP);

  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  attachPinChangeInterrupt(RIGHT_ENC_A, right_enc_tick, CHANGE);
  attachPinChangeInterrupt(LEFT_ENC_A, left_enc_tick, CHANGE);

  ST.drive(scaled_speed_power);
  ST.turn(scaled_steer_power);

  // start reading from sabertooth
  ST.async_getBattery( 1, 0 );  // request battery voltage with context 0

  //Serial.println("Setup complete");
}

void loop() {
  /*if(digitalRead(7) == HIGH)
  {
    enc_left += 1;
    enc_right += 1;
  }*/
  int bladeCmdA;
  int bladeCmdB;
  int bladeState = -1;
  if(900 < blade_pwm && blade_pwm < 1100)
  {
    bladeState = blade_control.update_blade(BLADE_CONTROL_ON_STATE, bladeCmdA, bladeCmdB);
  }
  else
  {
    bladeState = blade_control.update_blade(BLADE_CONTROL_OFF_STATE, bladeCmdA, bladeCmdB);
  }

  update_ST_data();

  steer_pwm = ppm.latestValidChannelValue(RC_STEER, 0);
  speed_pwm = ppm.latestValidChannelValue(RC_SPEED, 0);
  blade_pwm = ppm.latestValidChannelValue(RC_BLADE, 0);
  auto_pwm = ppm.latestValidChannelValue(RC_AUTO, 0);
  
  if(timeSince(timeDEBUG) > DEBUG_PERIOD)
  {
    // Print latest valid values from all channels
    /*for (int channel = 1; channel <= channelAmount; ++channel) {
        unsigned long value = ppm.latestValidChannelValue(channel, 0);
        Serial.print(String(value) + " ");
    }
    Serial.println();*/
    /*
    Serial.print(battery_ST);
    Serial.println(" Volts x10, battery voltage *************");
    Serial.print("steer:"); Serial.print(steer_pwm); Serial.print("\t");
    Serial.print("speed:"); Serial.print(speed_pwm); Serial.print("\t");
    Serial.print("blade:"); Serial.print(blade_pwm); Serial.print("\t");
    Serial.print("auto:"); Serial.print(auto_pwm); Serial.print("\t");
    Serial.print("enc_left:"); Serial.print(enc_left); Serial.print("\t");
    Serial.print("enc_right:"); Serial.println(enc_right);
    
    Serial.print("bladeState: "); Serial.print(bladeState);
    Serial.print(" ("); Serial.print(bladeCmdA); Serial.print(","); Serial.print(bladeCmdB);
    Serial.println(")");
    
    Serial.print("speed, steer power: ");
    Serial.print(scaled_speed_power); Serial.print(","); Serial.println(scaled_steer_power);

    Serial.print("current1, filt: ");
    Serial.print(current1_ST); Serial.print(","); Serial.println(filt_current1_ST);

    Serial.print("current2, filt: ");
    Serial.print(current2_ST); Serial.print(","); Serial.println(filt_current2_ST);
    */
    
    timeDEBUG = millis();
  }
  
  if(abs(speed_pwm - 1500) < 300 && abs(steer_pwm-1500) < 300 && (millis() - timeCMD > CMD_DEBOUNCE) )
  {
    timeCMD = millis();
    // power -2047 to 2047
    scaled_speed_power = (speed_pwm - 1500)*2;
    scaled_steer_power = (steer_pwm - 1500)*2;
  }
  else if(abs(speed_pwm - 1500) > 500 || abs(steer_pwm-1500) > 500)
  {
    scaled_speed_power = 0;
    scaled_steer_power = 0;
  }
  ST.drive(scaled_speed_power);
  ST.turn(-scaled_steer_power);

  // A3/4/ encoder request
  // A1/1/speed_byte,curv_byte
  // A2/1/autoblade/
  
  if(Serial.read() == 'A'){
    getSerial();
    switch(serialdata)
    {
      case 1: // A1/
      {
        getSerial(); // A1/1,2,3,4/
        switch (serialdata)
        {
          case 1: // A1/1/
          {
            // Read the next two numbers <raw speed 0 to 240>/<raw omega 0 to 240>/
            int speed_cm = getSerial()-120;
            int omega_deg = getSerial()-120;
            
            // Read the next two bytes as int8 speed cm/s, int8 omega deg/sec
            //int speed_cm = Serial.read() - 120;
            //int omega_deg = Serial.read() - 120;
            
            /*Serial.println("Speed cm, Omega deg");
            Serial.println(speed_cm);
            Serial.println(omega_deg);*/
            break;
          }
        }
        break;
      } // end A1/
      case 3: // A3/
      {
        getSerial(); // A3/1,2,3,4/
        switch (serialdata)
        {
          case 4: // A3/4/
          {
            //encLeft/Right read and reset
            Serial.println(enc_left);
            enc_left = 0;
            Serial.println(enc_right);
            enc_right = 0;           
            break;
          }
        }
        break;
      } // end A3/
    } // end switch serialdata
  }
  
}

long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    if (inbyte > 0 && inbyte != '/')
    {
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}

uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}

// enc_tick() functions only count half of quadrature counts
//*************************************************************************************
void left_enc_tick()
{
  // modify using PORT operations for efficiency
  if(digitalRead(LEFT_ENC_A) != digitalRead(LEFT_ENC_B))
  {
    enc_left--;
  }
  else
  {
    enc_left++;
  }
}

//*************************************************************************************
void right_enc_tick()
{
  // modify using PORT operations for efficiency
  if(digitalRead(RIGHT_ENC_A) != digitalRead(RIGHT_ENC_B))
  {
    enc_right--;
  }
  else
  {
    enc_right++;
  }
}

//*************************************************************************************
void update_ST_data()
{
  int result = 0;
  int context = 0;
  
  if ( saberSerial.reply_available( &result, &context ) )
  {
    switch ( context )
    {
      case SABERTOOTH_GET_ERROR:
         //Serial.println("ERROR");
      
      case SABERTOOTH_GET_TIMED_OUT:
         ST.async_getBattery( 1, 0 );  // try again from the beginning
         //Serial.println("TIME OUT");
         break;
           
      case 0:  // get Battery  
         battery_ST = result;
         ST.async_getCurrent( 1, 3 );  // request motor 1 current with context 3
         break;
           
      case 3: // get current
         current1_ST = result+20;
         filt_current1_ST = filt_current1_ST*0.95 + double(current1_ST)*0.05;
         ST.async_getCurrent( 2, 4 );  // request motor 2 current with context 4
         break;

      case 4: // get current
         current2_ST = result;
         filt_current2_ST = filt_current2_ST*0.95 + double(current2_ST)*0.05;
         ST.async_getBattery( 1, 0 );  // request battery voltage with context 0
         break;
       
    }
  }
}
