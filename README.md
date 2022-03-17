# MTE380
 MTE380 terrainmaster code

Test_driving
  L298NX2_simple -> test driving two motors together (no sensors)
  test_driving -> uses both motors and one tof sensor -> start and stop based on distance 

Test_locating
  dual_sensor_test -> test i2c bus with two sensors hooked up
  i2c_scanner -> scans and prints adress of all devices on bus
  multiple_tof -> reading multiple sensors might incorporate gyro as well later
  test_locating -> actual locating test code, right now ony uses one sensor, will update to read from all four

Test_turning
  -> gyro+motors