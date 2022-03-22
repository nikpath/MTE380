# MTE380
 MTE380 terrainmaster code
## File Structure:
MTE380
|-main
	|-sensor-reading
	|-main
|-test_turning
	|-turn_test_without_gyro
	|-turn_test_with_gyro
|-test_driving
	|-test_driving
|-test_locating
	|-dual_sensor_test
	|-test_locating
	|-i2c_scanner
	|-multiple_tof

## File notes:
Main will hold our main code 

sensor-reading -> for reading from multiple sensor on the i2c bus (incomplete)

test_turning -> two tests, one that uses the gyro to turn, the other uses hardcoded timing to turn 90deg

test_driving -> basic_run() function just drives until the tof sensor reads a distance <5cm and test_run() is what we'll use on testing day because it drives but stops at distances of 95, 65, 35 and 5cm

test_locating -> just has a bunch of random code for testing the sensors
	Dual_sensor_test -> reads from both tof sensors
	I2c_scanner -> returns addresses of all devices on the i2c bus, good for testing/debugging and checking that all your devices are accounted for
	Multipl_tof -> handles multiple tof sensors

## TODO
	- Run test codes with full assembly
	- Might have to remove the "while (! Serial) { delay(1); }" line in setup() functions since USB won't be connected during actual run
	- Create full circuit diagram with pin connections
	- Finalize and incorporate encoders
	- Start on main functions

## THINGS TO ADD:
	- Encoders -> verifies distance travelled, tells us if we are stuck
		○ PID control with encoder (see Motor_PID arduino library)
	- Side tof sensor verification code to make sure driving in straight line/correct straightness
	- Gyro for sensing tilt when moving in and out of traps
		○ Speed changing when moving in and out
		○ No sensor reading unless on flat ground