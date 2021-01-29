#include <SPI.h>
#include <RH_RF95.h>
//for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define PACKET_TIMEOUT 300
long timeNewPacket;

// Read "PPM" data directly from radio packets
//RC CHANNELS
#define RC_STEER 0
#define RC_SPEED 1
#define RC_BLADE 3
#define RC_AUTO 2

#define NUM_CHANNELS 8
#define DEFAULT_PULSE_LENGTH 1500
volatile uint16_t ppm[NUM_CHANNELS];

#define LED 13

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

#include <BladeControl.h>
// ln -s <full BladeControl path> <full Arduino libraries path/BladeControl>
// ln -s ~/Projects/mower_chassis/arduino_mowpi/BladeControl ~/Arduino/libraries/BladeControl

#define DEBUG_PERIOD 500
#define CMD_PERIOD 100
#define BLADE_DB 500
#define CMD_FILT_FACTOR 0.5

#define BOT_RADIUS_CM 35.0

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

//*****************ENCODERS****************
#include <YetAnotherPcInt.h>
#define RIGHT_ENC_A 11
#define RIGHT_ENC_B 10
#define LEFT_ENC_A 9
#define LEFT_ENC_B 6
volatile int16_t enc_left = 0, enc_right = 0;

unsigned long serialdata;
int inbyte = 0;

//********** Blade Control ************
# define BLADE_PIN_A 12 // micro_PCB IN1 --> relay IN2 (rewire blade control wires)
# define BLADE_PIN_B A5 // micro_PCB IN2 --> relay IN1 (rewire blade control wires)
BladeControl blade_control(BLADE_PIN_A, BLADE_PIN_B);

// Sabertooth Describe configuration
//   Set the battery type and voltage fixed in Describe.
//   Otherwise it assumes full voltage at startup
//   Reverse motor direction(s) if needed

#include <USBSabertooth_NB.h>

USBSabertoothSerial saberSerial(Serial1);
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
  timeNewPacket = millis();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1) {
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13,HIGH);
      delay(500);
    }
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1) {
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13,HIGH);
      delay(500);
    }
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  
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

  Serial1.begin(9600);
  //ST.autobaud();

  //ENCODERS
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  PcInt::attachInterrupt(RIGHT_ENC_A, right_enc_tick, CHANGE);
  PcInt::attachInterrupt(LEFT_ENC_A, left_enc_tick, CHANGE);

  ST.drive(scaled_speed_power);
  ST.turn(scaled_steer_power);

  // start reading from sabertooth
  ST.async_getBattery( 1, 0 );  // request battery voltage with context 0

  //Serial.println("Setup complete");
}

void loop()
{

  if(timeSince(timeNewPacket) > PACKET_TIMEOUT)
  {
    set_default_ppm();
  }
  
  if(timeSince(gyro_time) > GYRO_PERIOD)
  {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float dt = (float)timeSince(gyro_time)/1000.0;
    gyro_time = millis();
    gyro_z = (gyro.z()+GYRO_BIAS_DEG) * GYRO_SCALE_FACTOR;
    delta_yaw_deg += gyro_z*dt;
    yaw_deg += gyro_z*dt;
  }
  
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
      steer_pwm = ppm[RC_STEER];
      speed_pwm = ppm[RC_SPEED];
      blade_pwm = ppm[RC_BLADE];
      auto_pwm = ppm[RC_AUTO];
    
      prev_steer_pwm = steer_pwm;
      prev_speed_pwm = speed_pwm;
      prev_blade_pwm = blade_pwm;
      prev_auto_pwm = auto_pwm;
    
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
  } // end if Serial.read() == 'A'

  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      if(buf[0] == 0xCC && buf[1] == 0xAA)
      {
        for(int k=0; k<4; ++k)
        {
          int16_t pulse_val = buf[2*k+2]*255 + buf[2*k+3]; //1000 to 2000
          
          if(pulse_val < 900 || pulse_val > 2010)
          {
            pulse_val = 1500;
          }
          else if(pulse_val < 1001)
          {
            pulse_val = 1001;
          }
          else if(pulse_val > 1999)
          {
            pulse_val = 1999;
          }
          ppm[k] = pulse_val; //Consider hard coding to 1500 and see if it removes hiccups
        }
        timeNewPacket = millis();
      }
    }
    else
    {
      Serial.println("Receive failed");
    }
  } // end if rf95 data available
  
} // end loop

void set_default_ppm()
{
  for(int i = 0; i < NUM_CHANNELS; ++i){
    ppm[i]= DEFAULT_PULSE_LENGTH;
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
