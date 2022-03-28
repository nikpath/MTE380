// include libraries
#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include <MPU6050_light.h>

//gyro initial
MPU6050 mpu(Wire);
float az;
int num=0;

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

void loop() 
{
  lox1.rangingTest(&measure_front, false); //need this for sensor reading to work!
  s_front = measure_front.RangeMilliMeter;
  Serial.print(s_front);
  Serial.print("mm");    
  delay(500);
}
