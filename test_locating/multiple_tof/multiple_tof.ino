#include <vl53l0x_platform.h>
#include <vl53l0x_def.h>
#include <vl53l0x_platform_log.h>
#include <vl53l0x_api_core.h>
#include <vl53l0x_i2c_platform.h>
#include <vl53l0x_tuning.h>
#include <vl53l0x_api.h>
#include <vl53l0x_device.h>
#include <vl53l0x_api_ranging.h>
#include <vl53l0x_api_strings.h>
#include <vl53l0x_interrupt_threshold_settings.h>
#include <Adafruit_VL53L0X.h>
#include <vl53l0x_api_calibration.h>
#include <vl53l0x_types.h>

/*
 * Testing multiple tof sensors
 * Seeing how to work with the I2C bus 
 * This function polls/gets reading from each sensor one at a time (front, left, back, right)
 */

#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"


DFRobot_VL53L0X s_front;
DFRobot_VL53L0X s_left;
DFRobot_VL53L0X s_back;
DFRobot_VL53L0X s_right;
// I2C address scanner program

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Scanner");
  
}

void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");

      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");

      Serial.println(address,HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");

  delay(5000); // wait 5 seconds for next scan
}
