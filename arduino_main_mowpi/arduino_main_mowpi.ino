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
int16_t auto_pwm = 1500;

int16_t scaled_speed_power = 0;
int16_t scaled_steer_power = 0;

//RC CHANNELS
#define RC_STEER 1
#define RC_SPEED 2
#define RC_THROTTLE 3
#define RC_AUTO 4

//**************Encoders*************
#include <PinChangeInt.h>
#define RIGHT_ENC_A A2
#define RIGHT_ENC_B A3
#define LEFT_ENC_A A4
#define LEFT_ENC_B A5

int16_t enc_left = 0, enc_right = 0;

//********** Blade Control ************
# define BLADE_PIN_A 11
# define BLADE_PIN_B 12
BladeControl blade_control(BLADE_PIN_A, BLADE_PIN_B);

// Sabertooth Describe configuration
//   Set the battery type and voltage fixed in Describe.
//   Otherwise it assumes full voltage at startup
//   Reverse motor direction(s) if needed

#include <Sabertooth.h>

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
Sabertooth ST(128, altSerial); // Address 128, and use SWSerial as the serial port.

void setup() {
  Serial.begin(115200);
  timeDEBUG = millis();
  timeCMD = millis();

  altSerial.begin(9600);
  ST.autobaud();

  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  attachPinChangeInterrupt(RIGHT_ENC_A, right_enc_tick, CHANGE);
  attachPinChangeInterrupt(LEFT_ENC_A, left_enc_tick, CHANGE);

  ST.drive(scaled_speed_power);
  ST.turn(scaled_steer_power);

  Serial.println("Setup complete");
}

void loop() {
  steer_pwm = ppm.latestValidChannelValue(RC_STEER, 0);
  speed_pwm = ppm.latestValidChannelValue(RC_SPEED, 0);
  auto_pwm = ppm.latestValidChannelValue(RC_AUTO, 0);
  
  int bladeCmdA = -1, bladeCmdB = -1;
  int bladeState = -1;
  if(900 < auto_pwm && auto_pwm < 1100)
  {
    bladeState = blade_control.update_blade(BLADE_CONTROL_ON_STATE, bladeCmdA, bladeCmdB);
  }
  else
  {
    bladeState = blade_control.update_blade(BLADE_CONTROL_OFF_STATE, bladeCmdA, bladeCmdB);
  }
  
  if(timeSince(timeDEBUG) > DEBUG_PERIOD)
  {
    // Print latest valid values from all channels
    /*for (int channel = 1; channel <= channelAmount; ++channel) {
        unsigned long value = ppm.latestValidChannelValue(channel, 0);
        Serial.print(String(value) + " ");
    }
    Serial.println();*/
    Serial.print("steer:"); Serial.print(steer_pwm); Serial.print("\t");
    Serial.print("speed:"); Serial.print(speed_pwm); Serial.print("\t");
    Serial.print("auto:"); Serial.print(auto_pwm); Serial.print("\t");
    Serial.print("enc_left:"); Serial.print(enc_left); Serial.print("\t");
    Serial.print("enc_right:"); Serial.println(enc_right);
    
    Serial.print("bladeState: "); Serial.print(bladeState);
    Serial.print(" ("); Serial.print(bladeCmdA); Serial.print(","); Serial.print(bladeCmdB);
    Serial.println(")");
    
    Serial.print("speed, steer power: ");
    Serial.print(scaled_speed_power); Serial.print(","); Serial.println(scaled_steer_power);
    timeDEBUG = millis();
  }
  
  if(abs(speed_pwm - 1500) < 300 && abs(steer_pwm-1500) < 300 && (millis() - timeCMD > CMD_DEBOUNCE) )
  {
    timeCMD = millis();
    // power -127 to 127
    scaled_speed_power = (speed_pwm - 1500)/8;
    scaled_steer_power = (steer_pwm - 1500)/8;
  }
  else if(abs(speed_pwm - 1500) > 500 || abs(steer_pwm-1500) > 500)
  {
    scaled_speed_power = 0;
    scaled_steer_power = 0;
  }
  ST.drive(scaled_speed_power);
  ST.turn(-scaled_steer_power);
}

uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}

int16_t flip_sig(int16_t sig_in)
{
  return 3000-sig_in;
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
