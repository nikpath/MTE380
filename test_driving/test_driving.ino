
/*
 * Tests straight driving, starts and stops based on tof
 */
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

float calculate_offset(int distance) {
  //y = -0.0001x2 + 0.1222x - 18.768
  float offset = -0.0001*pow(distance, 2) + 0.1222*distance - 18.768;
  return offset;
}

float get_distance() {
  float total = 0;
  for(int i = 0; i < 3; i++){
    lox1.rangingTest(&measure_front, false); //need this for sensor reading to work!
    s_front = measure_front.RangeMilliMeter;
    total += s_front;
  }
  return (total/3);
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

void basic_run() { //this just drives forward until the distance to the wall is 5cm
  motors.stop();
  delay(1000);

  lox1.rangingTest(&measure_front, false); 
  s_front = measure_front.RangeMilliMeter;
    Serial.print("Distance:");  
    Serial.print(s_front);
    Serial.print("mm");    
    
  while(s_front > 50) {
    lox1.rangingTest(&measure_front, false); //need this for sensor reading to work!
    s_front = measure_front.RangeMilliMeter;
    //full speed ahead
    motors.setSpeedA(255);
    motors.setSpeedB(255);
    motors.backwardA();
    motors.forwardB();
    delay(100);

    Serial.print("Distance:");  
    Serial.print(s_front);
    Serial.print("mm"); 
    Serial.println();   
  }

  Serial.println();

}

void test_run() { //this stops at multiple distances while driving to test that the sensor can detect those distances
  int distance = 950; //initial distance it should stop at
  int offset = 35;
  int acc_distance = 950;
  float cur_dist = 0;
  for(int i = 0; i <= 4; i++) {
    //decrement distance
    acc_distance = 950 - i*300;
    offset = calculate_offset(acc_distance);
    distance = acc_distance + offset;
    Serial.print("WANTED DISTANCE:");  
    Serial.print(acc_distance);
    Serial.println();
    Serial.print("SHOULD READ:");  
    Serial.print(distance);
    Serial.println();
    
    lox1.rangingTest(&measure_front, false); 
    s_front = measure_front.RangeMilliMeter;
    
    while(cur_dist > distance) {
      cur_dist = get_distance();
      Serial.print("current distance:");  
      Serial.print(cur_dist);
      Serial.println(); 
         
      //full speed ahead
      motors.setSpeedA(127);
      motors.setSpeedB(127);
      motors.backward();
      delay(100);
  
      Serial.print("Distance:");  
      Serial.print(s_front);
      Serial.print("mm");
      Serial.println();    
    }
    
    motors.stop();
    delay(3000);  
    Serial.println();
  }
  delay(30000);
}
void loop() {
  basic_run();
}
