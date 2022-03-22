/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_VL53L0X.h"
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
float az;

//tof initial
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
int s_front,s_side;

// set the pins to shutdown
#define SHT_LOX1 8
#define SHT_LOX2 9

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure_front;
VL53L0X_RangingMeasurementData_t measure_side;

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  while (! Serial) { delay(1); }
  
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  
  Serial.println("Shutdown pins inited...");
  
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  
  Serial.println("Both in reset mode...(pins are low)");
  
  
  Serial.println("Starting...");
  setID();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
}

void loop() {
  //calibration
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  Serial.println(F("Motor stop for 1 second"));
  motors.stop();
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  while (abs(az) <90){
  mpu.update();
  az=mpu.getAngleZ();
  //motor turning code (method 1)
  motors.setSpeedA(255);
  motors.setSpeedB(90);
  motors.forward();
  delay(100);
  /*or alternative turning method (method 2)
   motors.setSpeedA(90);
   motors.setSpeedB(90);
   motors.forwardA();
   motors.backwardB();
   delay(100);
   */
  //these are for checking & can be deleted later
    Serial.print("turning\n");
    Serial.print(F("ANGLE     Z: "));
    Serial.print(az);
    Serial.print("\n");
    delay(100);
  }
  motors.stop();
  delay (100);
  az=0;
  Serial.print("Turning done");
  }
