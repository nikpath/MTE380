/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
// Include the (new) library
#include <L298NX2.h>

// Pin definition
const unsigned int EN_A = 3; //left motor (relative to the front)
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

const unsigned int IN1_B = 7; //right motor
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

MPU6050 mpu(Wire);
float ax;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
}

void loop() {  
  //calibration
  ax=0;
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  Serial.println(F("Motor stop for 1 second"));
  motors.stop();
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  while (abs(ax) <80){
  mpu.update();
  ax=mpu.getAngleZ();
  //motor turning code (method 1)
  /*
  motors.setSpeedA(255);
  motors.setSpeedB(90);
  motors.forward();
  delay(100);
  */
  //or alternative turning method (method 2)
   motors.setSpeedA(90);
   motors.setSpeedB(0);
   motors.forwardB();
   motors.backwardA();
   delay(100);
   
  //these are for checking & can be deleted later
    Serial.print("turning\n");
    Serial.print(F("ANGLE     x: "));
    Serial.print(ax);
    Serial.print("\n");
    delay(100);
  }
  motors.stop();
  delay (30000);
  Serial.print("Turning done");
  }
  
