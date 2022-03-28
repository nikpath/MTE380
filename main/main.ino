/*
 * This is the main function, this is what will run our robot
 * TODO:
 * check delays -> remove/reduce if not needed
 * copy in turning code (whichever one we choose)
 * check all hardcoded values -> change them based on testing
 */

// include libraries
#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <L298NX2.h>

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

L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

//tof initial
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

void setID() { //set addresses for devices on i2c bus (tof and gyro)
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

void turn_90() { //does a 90deg turn to the right
  motors.setSpeedA(191);
  motors.setSpeedB(191);
  motors.forwardA();
  motors.backwardB();
  delay(400); //delay 5s
  /*or alternative turning method (method 2)
   motors.setSpeedA(90);
   motors.setSpeedB(90);
   motors.forwardA();
   motors.backwardB();
   delay(5000);
   */
  motors.stop();
  delay (400);
}

void drive_until(int wall_distance) { //drives until the distance to the wall is detected
  lox1.rangingTest(&measure_front, false); //need this for sensor reading to work!
  s_front = measure_front.RangeMilliMeter;
  while(s_front > wall_distance) {
    lox1.rangingTest(&measure_front, false); //need this for sensor reading to work!
    s_front = measure_front.RangeMilliMeter;
    //full speed ahead
    motors.setSpeedA(191);
    motors.setSpeedB(191);
    motors.backward();
    delay(1000);
    motors.setSpeedA(255);
    motors.setSpeedB(255);
    motors.backward();
    delay(1000);
  }
  
  motors.stop();
  delay(1000);  
    
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
  while(status!=0){Serial.print("gyro not working");   } // stop everything if could not connect to MPU6050

}

void basic_main() {
  int i = 0; //number of turns completed
  int n_turns = 10; //total number of turns to be made
  int wall_distance = 100;
  
  while(i <= 10) {
    drive_until(wall_distance);
    if(i != 10) { //still have turns to complete
      turn_90();
      i++;
      switch(i) { //change distance at which to stop based on number of turns completed
        case 4:
          wall_distance = 370;
          break;
        case 8:
          wall_distance = 670;
          break;  
      }
    }
  }
  //should be at center now
}

void loop() {
  basic_main();

}
