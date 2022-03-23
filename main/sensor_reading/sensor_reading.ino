/*
 * Getting readings from sensors: i2c
 * 2 tof
 * 1 gyro
 */

#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include <MPU6050_light.h>

//gyro initial
MPU6050 mpu(Wire);
float az;
int num=0;

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

void read_dual_sensors() {
  
  lox1.rangingTest(&measure_front, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure_side, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print("1: ");
  if(measure_front.RangeStatus != 4) {     // if not out of range
    s_front = measure_front.RangeMilliMeter;    
    Serial.print(s_front);
    Serial.print("mm");    
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" ");

  // print sensor two reading
  Serial.print("2: ");
  if(measure_side.RangeStatus != 4) {
    s_side = measure_side.RangeMilliMeter;
    Serial.print(s_side);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();
}

void i2c_scan() {
  byte error, address;
    int nDevices;
   
    Serial.println("Scanning...");
   
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
   
      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address<16)
          Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
   
        nDevices++;
      }
      else if (error==4)
      {
        Serial.print("Unknown error at address 0x");
        if (address<16)
          Serial.print("0");
        Serial.println(address,HEX);
      }    
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");
   
    delay(5000);           // wait 5 seconds for next scan
}

void mpu_test() {
  if (num ==0){
    //calibration
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    Serial.println(F("Motor stop for 1 second"));
    /*
     stop motor
     */
    delay(1000);
    mpu.calcOffsets(true,true); // gyro and accelero
    Serial.println("Done!\n");
    while (abs(az) <90){
    mpu.update();
    az=mpu.getAngleZ();
    /*
     mortor turning
     */
    //these are for checking & can be deleted later
      Serial.print("turning\n");
      Serial.print(F("ANGLE     Z: "));
      Serial.print(az);
      Serial.print("\n");
      delay(100);
    }
    /*
     stop motor again
     */
    az=0;
    Serial.print("Turning done");
    num=num+1; //num is like a switch
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

void loop() {
  //i2c_scan();
  //read_dual_sensors();
  delay(1000);
  //mpu_test();

  if (num == 0){
    //calibration
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    Serial.println(F("Motor stop for 1 second"));
    /*
     stop motor
     */
    delay(1000);
    mpu.calcOffsets(true,true); // gyro and accelero
    Serial.println("Done!\n");
    while (abs(az) <90){
    mpu.update();
    az=mpu.getAngleZ();
    /*
     mortor turning
     */
    //these are for checking & can be deleted later
      Serial.print("turning\n");
      Serial.print(F("ANGLE     Z: "));
      Serial.print(az);
      Serial.print("\n");
      delay(100);
    }
    /*
     stop motor again
     */
    az=0;
    Serial.print("Turning done");
    num=num+1; //num is like a switch
  }

  lox1.rangingTest(&measure_front, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure_side, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print("1: ");
  if(measure_front.RangeStatus != 4) {     // if not out of range
    s_front = measure_front.RangeMilliMeter;    
    Serial.print(s_front);
    Serial.print("mm");    
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" ");

  // print sensor two reading
  Serial.print("2: ");
  if(measure_side.RangeStatus != 4) {
    s_side = measure_side.RangeMilliMeter;
    Serial.print(s_side);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();

}
