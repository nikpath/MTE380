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

//global variables
float side=100;
float err=0;
float tilt_angle;

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

void adjust(float error){
 //distance-, closer to wall, turn right
 //while (error < 0 && get_sidedistance() < side){ 
 if(error < 0) {
  
  motors.setSpeedA(255);
  motors.setSpeedB(200);
  motors.forwardA();
  motors.backwardB();
  delay (200);
 }
  //distance+, away from wall, turn left
 //while (error > 0 && get_sidedistance() > side){
 else {
  motors.setSpeedA(160);
  motors.setSpeedB(255);
  motors.forwardA();
  motors.backwardB();
  delay(200);
 }
  //back to original speed
  motors.setSpeedA(255);
  motors.setSpeedB(255);
}

float calculate_offset(int distance) {
  float offset = -0.0001*pow(distance, 2) + 0.1222*distance - 18.768;
  if (distance == 100){
    offset=offset+60;
    //10.748;
  }
  if (distance ==400){
     offset=offset+100;
     //17.088;
  }
   if (distance ==700){
     offset=offset+120;
     //17.088;
  }
  return offset;
}

float get_frontdistance() {
  float total = 0;
  for(int i = 0; i < 2; i++){
    lox1.rangingTest(&measure_front, false); //need this for sensor reading to work!
    float s_f = measure_front.RangeMilliMeter;
    total += s_f;
  }
  return (total/2);
}

float get_sidedistance() {
  float total = 0;
  for(int i = 0; i < 2; i++){
    lox2.rangingTest(&measure_side, false); //need this for sensor reading to work!
    float s_s = measure_side.RangeMilliMeter;
    total += s_s;
  }
  return (total/2);
}

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
  
  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);
  
  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  delay(10);
}

void turn_90() { //does a 90deg turn to the right
  motors.stop();
  delay (300);
  motors.setSpeedA(255);
  motors.setSpeedB(255);
  motors.forward();
  delay(460); //545
  motors.stop();
  delay (300);
}


void drive_until(float wall_distance) { //drives until the distance to the wall is detected
  s_front = get_frontdistance();
  //tilt_angle=mpu.getAngleY();
  while(s_front > wall_distance) {
    //|| (abs(tilt_angle)>=10)
    //full speed ahead
    motors.setSpeedA(255); 
    motors.setSpeedB(255); 
    motors.forwardA();
    motors.backwardB();
    
    //check if we are going into/out of the trap
    //if so, keep current motion state and don't update front/side distance sensor
   // tilt_angle=mpu.getAngleY();
    //checking side distance while driving & calculate error
    float side_new = get_sidedistance(); 
    err=side_new - side;
    Serial.print("side_new:   ");
    Serial.println(side_new);
    Serial.print("Error:   ");
    Serial.println(err);
    if (abs(err/side)>0.2) {
      adjust(err);
      Serial.print("Calculated Difference:   ");
      Serial.println(abs(err/side));
    } 
    delay(100);
    //renew front distance 
    s_front = get_frontdistance();
  }
  motors.stop();
  delay(100);   
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
  int i = 1; //number of turns completed
  int n_turns = 10; //total number of turns to be made
  float wall_distance = 100;
  side=side+calculate_offset(side);
  while(i <= 10) {
    Serial.println(i);
    wall_distance = wall_distance+calculate_offset (wall_distance);
    drive_until(wall_distance);
    if(i != 10) { //still have turns to complete
      if(i == 4){
        Serial.println("AAA!");
         motors.setSpeedA(255); 
         motors.setSpeedB(255); 
         motors.forwardA();
         motors.backwardB();
        delay(2200);
        motors.stop();
        Serial.println("motor stop!");
        delay(3000);
        Serial.println("delay done!");
      }
      if (i==10){
        i++;
      }
      if (i!=4){
         turn_90();
      }
      delay(100);     
      i++;
      delay(10);
      switch(i) { //change distance at which to stop based on number of turns completed
        case 4:
          wall_distance = 500;
          delay(10);
          break;
        case 5:
          side=side+300;
          side=side+calculate_offset(side);
          delay(10);
          break;
        case 9:
          wall_distance = 700;
          side=side+300;
          side=side+calculate_offset(side);
          delay(10);
          break;  
      }
    }
  }
  motors.stop();
  delay(100000);
  //should be at center now
}
void go_in_circle() {
  int turns = 3;
  int i = 0;
  float wall_distance = 100;
  side=side+calculate_offset(side);
  while(i < turns){
    wall_distance = wall_distance + calculate_offset(wall_distance);
    drive_until(wall_distance);
    turn_90();
    i++;
  }
  motors.stop();  
  delay(100000);
}

void loop() {
  //only run the trip once 
  go_in_circle();
}  
