
#include "MPU9250.h" // using lib from: https://github.com/bolderflight/MPU9250
// read the help page and header file for a great deal of information!!

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68. The MPU-9250 supports I2C, up to 400 kHz
// #define PI 3.1415926535897932384626433832795;

MPU9250 IMU(Wire, 0x68);
int status;

long previousMillis = 0; // will store last time LED was updated
long interval = 19; // interval at which to blink (milliseconds), 1 ms shorter than desired (time to finish processing)
long dt; // change in time actual (milliseconds)
const float minGyroValue = 0.25; // min +/-Gyro value, (converted to rad/s)


float checkDeadbandValue (float val, float minVal)
// return value with deadband
{
  float curVal = val;
  if (val < minVal and val > -minVal)
  {
    curVal = 0;
  }
  return curVal;
}


void setup() {
  Serial.begin(115200); // serial to display data
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  /* Advanced Items:*/
  // IMU.set___(bias, scale)
  //IMU.setAccelCalX( 0.410690795, 1.000380467); // sets the accelerometer bias (m/s/s) and scale factor in the X direction
  //IMU.setAccelCalY( 5.14925E-05, 1.002806316); // sets the accelerometer bias (m/s/s) and scale factor in the Y direction
  //IMU.setAccelCalZ(-0.178361616, 1.009922876); // sets the accelerometer bias (m/s/s) and scale factor in the Z direction

  //IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G); // setting the accelerometer full scale range to +/-8G
  //IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS); // setting the gyroscope full scale range to +/-500 deg/s
  //IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ); // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ); // setting DLPF bandwidth to 5 Hz
  IMU.setSrd(19); // setting SRD to 19 for a 50 Hz update rate

  Serial.println("IMU Check");
  Serial.print("Accelerometer bias in the X direction, m/s/s: "); Serial.println(IMU.getAccelBiasX_mss(), 6);
  Serial.print("Accelerometer scale factor in the X direction "); Serial.println(IMU.getAccelScaleFactorX(), 6);
  Serial.print("Accelerometer bias in the Y direction, m/s/s ");  Serial.println(IMU.getAccelBiasY_mss(), 6);
  Serial.print("Accelerometer scale factor in the Y direction "); Serial.println(IMU.getAccelScaleFactorY(), 6);
  Serial.print("Accelerometer bias in the Z direction, m/s/s ");  Serial.println(IMU.getAccelBiasZ_mss(), 6);
  Serial.print("Accelerometer scale factor in the Z direction "); Serial.println(IMU.getAccelScaleFactorZ(), 6);
  delay(500);
}

void loop() {
  unsigned long currentMillis = millis(); // read the sensor

  if (currentMillis - previousMillis > interval) {
    dt = currentMillis - previousMillis;
    previousMillis = currentMillis; // save the last time you blinked the LED

    IMU.readSensor();
    Serial.print(dt);
    Serial.print("\t");
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print(checkDeadbandValue(IMU.getGyroX_rads(), minGyroValue), 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print(checkDeadbandValue(IMU.getGyroY_rads(), minGyroValue), 6);
    Serial.print("\t");
    //Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print(checkDeadbandValue(IMU.getGyroZ_rads(), minGyroValue), 6);

    /*
      Serial.print("\t");
      Serial.print(IMU.getMagX_uT(),6);
      Serial.print("\t");
      Serial.print(IMU.getMagY_uT(),6);
      Serial.print("\t");
      Serial.print(IMU.getMagZ_uT(),6);
      Serial.print("\t");
      Serial.println(IMU.getTemperature_C(),6);
    */
    Serial.println();
  }
}
