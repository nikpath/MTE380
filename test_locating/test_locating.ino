/*!
 * This is the locating test code
 * 
 */
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include <L298NX2.h>

DFRobot_VL53L0X s_front;

//pin declaration
//motor pins

//button pin
/*
// Pin definition - motor 1 (right)
const unsigned int IN1 = 2;
const unsigned int IN2 = 3;
const unsigned int EN_A = 9;

//pin definition - motor 2 (left)
const unsigned int IN1_B = 4;
const unsigned int IN2_B = 5;
const unsigned int EN_B = 10;
*/

int cur_dist = 0;

void setup() {
  //Motor setup
  //L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B );
  
  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    //do nothing
  }

  // Set initial speed
  //motors.setSpeed(70);
  
  //TOF setup
  //initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  s_front.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  s_front.setMode(s_front.eContinuous,s_front.eHigh);
  //Laser rangefinder begins to work
  s_front.start();
}

void loop() 
{
  //Get the distance
  cur_dist = s_front.getDistance();
  if(cur_dist < 70){
    Serial.print("stop motors");
    //motors.stop();
  } else {
    Serial.print("running motors");
    //motors.forward();
    delay(3000);
  }
  Serial.print("Distance: ");Serial.println(s_front.getDistance());
  //The delay is added to demonstrate the effect, and if you do not add the delay,
  //it will not affect the measurement accuracy
  delay(500);
}
