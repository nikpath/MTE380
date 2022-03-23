/*
 * Tests straight driving, starts and stops based on tof
 */

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_VL53L0X.h"
#include <L298NX2.h>
#include <MPU6050_light.h>

//gyro initial
MPU6050 mpu(Wire);
float az;
int num=0;

//motor initial
const unsigned int EN_A = 3; //left motor (relative to the front)
const unsigned int IN1_A = 5;
const unsigned int IN2_A = 6;

const unsigned int IN1_B = 7; //right motor
const unsigned int IN2_B = 8;
const unsigned int EN_B = 9;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

//tof initial
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
int s_front,s_side;

// set the pins to shutdown
#define SHT_LOX1 12
#define SHT_LOX2 13

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
  Serial.begin(115200);
  Wire.begin();
  // wait until serial port opens for native USB devices
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

void basic_run() { //this just drives forward until the distance to the wall is 5cm
  s_front = 0;
  motors.stop();
  delay(1000);
  if(measure_front.RangeStatus != 4) {     // if not out of range
    s_front = measure_front.RangeMilliMeter;
    
    while(s_front > 50) {
      //full speed ahead
      motors.setSpeedA(255);
      motors.setSpeedB(255);
      motors.forward();
      delay(100);
  
      Serial.print("Distance:");  
      Serial.print(s_front);
      Serial.print("mm");    
    }  
  } else {
    Serial.print("Out of range");
  }

}

void test_run() { //this stops at multiple distances while driving to test that the sensor can detect those distances
  s_front = 0;
  int distance = 950; //initial distance it should stop at
  
  for(int i = 0; i <= 4; i++) {
    //decrement distance
    distance = 950 - i*300;
    Serial.print("SHOULD STOP AT:");  
    Serial.print(distance);
  
    s_front = measure_front.RangeMilliMeter;
    
    while(s_front > distance) {
      //full speed ahead
      motors.setSpeedA(255);
      motors.setSpeedB(255);
      motors.forward();
      delay(100);
  
      Serial.print("Distance:");  
      Serial.print(s_front);
      Serial.print("mm");    
    }
    
    motors.stop();
    delay(1000);  
  }
}
void loop() {
  basic_run();
  Serial.println();
}
