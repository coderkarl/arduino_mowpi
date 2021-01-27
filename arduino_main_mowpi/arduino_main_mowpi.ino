// BNO055

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055();
#define GYRO_BIAS_DEG 0.00
#define GYRO_SCALE_FACTOR 0.978
bool gyroReady = false;
#define GYRO_PERIOD 20
long gyro_time;
float delta_yaw_deg = 0.0;
float yaw_deg = 0.0;
float gyro_z = 0.0;



// https://github.com/Nikkilae/PPM-reader
#include <PPMReader.h>
// #include <InterruptHandler.h>   <-- You may need this on some versions of Arduino

#include <BladeControl.h>
// ln -s <full BladeControl path> <full Arduino libraries path/BladeControl>
// ln -s ~/Projects/mower_chassis/arduino_mowpi/BladeControl ~/Arduino/libraries/BladeControl

#define DEBUG_PERIOD 500
#define CMD_PERIOD 100
#define BLADE_DB 500
#define CMD_FILT_FACTOR 0.5

#define BOT_RADIUS_CM 35.0

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
// PPMReader uses hard interrupts, uno pins 2 or 3
// attachInterrupt(digitalPinToInterrupt(interruptPin), RISING);
int interruptPin = 3;
int channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);
#define PPM_ERROR_PIN 10
bool ppm_error_state = false;

long timeDEBUG, timeCMD;
long timeREQBLADE;
bool blade_on = false;
bool blade_des = false;

int16_t prev_steer_pwm = 1500, steer_pwm = 1500;
int16_t prev_speed_pwm = 1500, speed_pwm = 1500;
int16_t prev_blade_pwm = 1500, blade_pwm = 1500;
int16_t prev_auto_pwm = 1500,  auto_pwm = 1500;

int16_t scaled_speed_power = 0;
int16_t scaled_steer_power = 0;

int speed_cm = 0;
int omega_deg = 0;
int left_auto_output = 0;
int right_auto_output = 0;

//RC CHANNELS
#define RC_STEER 1
#define RC_SPEED 2
#define RC_THROTTLE 5
#define RC_BLADE 4
#define RC_AUTO 3

//**************Encoders*************
// Port B, pin 8 to 13
// Port C, analog
// Port D, pin 0 to 7
//#define NO_PORTB_PINCHANGES
//#define NO_PORTD_PINCHANGES
#include <PinChangeInt.h>
#define RIGHT_ENC_A A0 //A0, 7
#define RIGHT_ENC_B A1 //A1, 6
#define LEFT_ENC_A A2 //A2, 5
#define LEFT_ENC_B A3 //A3, 4

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
  
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // Send CAN ERROR MESSAGE, Software Reset, continue, do NOT use while(1)
    //while(1);
  }
  else
  {
    delay(200);
    bno.setExtCrystalUse(true);
    gyroReady = true;
  }
  gyro_time = millis();
  
  
  timeDEBUG = millis();
  timeCMD = millis();

  altSerial.begin(9600);
  //ST.autobaud();


  //pinMode(7, INPUT_PULLUP);
  pinMode(PPM_ERROR_PIN, OUTPUT);
  digitalWrite(PPM_ERROR_PIN, LOW);

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
  
  if(timeSince(gyro_time) > GYRO_PERIOD)
  {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float dt = (float)timeSince(gyro_time)/1000.0;
    gyro_time = millis();
    gyro_z = (gyro.z()+GYRO_BIAS_DEG) * GYRO_SCALE_FACTOR;
    delta_yaw_deg += gyro_z*dt;
    yaw_deg += gyro_z*dt;
  }
  
  
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
    if(!blade_des)
    {
      blade_des = true;
      timeREQBLADE = millis();
    }
    else if(!blade_on && timeSince(timeREQBLADE) > BLADE_DB)
    {
      blade_on = true;
    }
    
    if(blade_on)
    {
      bladeState = blade_control.update_blade(BLADE_CONTROL_ON_STATE, bladeCmdA, bladeCmdB);
    }
  }
  else
  {
    if(blade_des)
    {
      blade_des = false;
      timeREQBLADE = millis();
    }
    else if(blade_on && timeSince(timeREQBLADE) > BLADE_DB)
    {
      blade_on = false;
    }
    
    if(!blade_on)
    {
      bladeState = blade_control.update_blade(BLADE_CONTROL_OFF_STATE, bladeCmdA, bladeCmdB);
    }
  }

  update_ST_data();

  steer_pwm = ppm.latestValidChannelValue(RC_STEER, 1500);
  speed_pwm = ppm.latestValidChannelValue(RC_SPEED, 1500);
  blade_pwm = ppm.latestValidChannelValue(RC_BLADE, 1500);
  auto_pwm = ppm.latestValidChannelValue(RC_AUTO, 1500);
  if( (abs(steer_pwm - 1500) > 300) || blade_pwm < 1400)
  {
    if(!ppm_error_state)
    {
      digitalWrite(PPM_ERROR_PIN, HIGH);
      ppm_error_state = true;
    }
  }
  else if(ppm_error_state)
  {
    ppm_error_state = false;
    digitalWrite(PPM_ERROR_PIN, LOW);
  }

  /*if(abs(steer_pwm - prev_steer_pwm) > 200 && abs(steer_pwm - 1500) > 50)
  {
    steer_pwm = prev_steer_pwm;
  }
  if(abs(speed_pwm - prev_speed_pwm) > 200 && abs(speed_pwm - 1500) > 50)
  {
    speed_pwm = prev_speed_pwm;
  }
  if(abs(blade_pwm - prev_blade_pwm) > 200 && abs(blade_pwm - 1500) > 50)
  {
    blade_pwm = prev_blade_pwm;
  }
  if(abs(auto_pwm - prev_auto_pwm) > 200 && abs(auto_pwm - 1500) > 50)
  {
    auto_pwm = prev_auto_pwm;
  }*/
  prev_steer_pwm = steer_pwm;
  prev_speed_pwm = speed_pwm;
  prev_blade_pwm = blade_pwm;
  prev_auto_pwm = auto_pwm;
  
  if(timeSince(timeDEBUG) > DEBUG_PERIOD)
  {
    /*Serial.print("gyro z: ");
    Serial.println(gyro_z);
    Serial.print("delta_yaw_deg: ");
    Serial.println(delta_yaw_deg);
    Serial.print("yaw_deg: ");
    Serial.println(yaw_deg);
    Serial.print("enc_left:"); Serial.print(enc_left); Serial.print("\t");
    Serial.print("enc_right:"); Serial.println(enc_right);
    */
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

  if(millis() - timeCMD > CMD_PERIOD)
  {
    timeCMD = millis();
    if(auto_pwm < 1700)
    {
      if(abs(speed_pwm - 1500) <= 500 && abs(steer_pwm-1500) <= 500 )
      {
        // power -2047 to 2047
        scaled_speed_power = ((float)(speed_pwm - 1500))*1.5;
        scaled_steer_power = ((float)(steer_pwm - 1500))*1.5;
      }
      else
      {
        scaled_speed_power = 0;
        scaled_steer_power = 0;
      }
      ST.drive(scaled_speed_power);
      ST.turn(-scaled_steer_power);
      left_auto_output = 0;
      right_auto_output = 0;
    }
    else //auto mode!
    {
      float left_cm = speed_cm - BOT_RADIUS_CM*float(omega_deg)*3.14/180.0;
      float right_cm = speed_cm + BOT_RADIUS_CM*float(omega_deg)*3.14/180.0;
      left_cm = min(left_cm, 150);
      left_cm = max(left_cm, -150);
      right_cm = min(right_cm, 150);
      right_cm = max(right_cm, -150);
      int left_output = left_cm * 7; //-2047 to 2047, 120 cm/sec maps to 600 for now
      int right_output = right_cm * 7;
      left_auto_output = left_auto_output * CMD_FILT_FACTOR + left_output * (1 - CMD_FILT_FACTOR);
      right_auto_output = right_auto_output * CMD_FILT_FACTOR + right_output * (1 - CMD_FILT_FACTOR);
      ST.motor(LEFT_MOTOR, left_auto_output);
      ST.motor(RIGHT_MOTOR, right_auto_output);
    }
  }

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
            speed_cm = getSerial()-120;
            omega_deg = getSerial()-120;
            
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
            Serial.println(int(gyro_z*100));
            //Serial.println(int(delta_yaw_deg*1000));
            delta_yaw_deg = 0.0;
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
