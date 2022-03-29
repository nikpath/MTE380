# MTE380
 MTE380 terrainmaster code
## File Structure:
MTE380<br>
|-main<br>
	|-sensor-reading<br>
	|-main<br>
|-test_turning<br>
	|-turn_test_without_gyro<br>
	|-turn_test_with_gyro<br>
|-test_driving<br>
	|-test_driving<br>
|-test_locating<br>
	|-dual_sensor_test<br>
	|-test_locating<br>
	|-i2c_scanner<br>
	|-multiple_tof<br>

## File notes:
main -> contains algorithm for spiral path, still need to incorporate offset, drive straight adjustment, and correct turning function <br>
ALSO: currently, drive_until function is accelerating and decelerrating, alternate 1 sec. be sure to change this if you just want it to drive at one speed

sensor-reading -> basic sensor reading code

test_turning -> two tests, one that uses the gyro to turn (please double check offset on angle)
the other uses hardcoded timing to turn 90deg

test_driving -> basic_run() function just drives until the tof sensor reads a distance <5cm and 
test_run() stops at distances of 95, 65, 35 and 5 -> THIS CODE HAS THE CORRECT OFFSET FUNCTION

test_locating -> just has a bunch of random code for testing the sensors
	Dual_sensor_test -> reads from both tof sensors
	I2c_scanner -> returns addresses of all devices on the i2c bus, good for testing/debugging and checking that all your devices are accounted for
	Multipl_tof -> handles multiple tof sensors

## TODO:
	- Side tof sensor verification code to make sure driving in straight line/correct straightness
	- Gyro for sensing tilt when moving in and out of traps
		○ No sensor reading unless on flat ground
	- Speed changing when moving in and out of traps -> acceleration function (must also come up with map to know where the trap is)