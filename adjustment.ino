
//initial distance may change
float side=50;
float err=0;

void setup() {
  // put your setup code here, to run once:

}

/*void adjust(error){
 //distance--, closer to wall, turn right
 if (error < 0){ 
  motors.setSpeedA(90);
  motors.setSpeedB(255);
  delay (200)
 }
  //distance++, away from wall, turn left
 else{
  motors.setSpeedA(255);
  motors.setSpeedB(90);
  delay (200)
 }
  //back to original speed
  motors.setSpeedA(255);
  motors.setSpeedB(255);
}*/

void loop() {
  
  side_new= //reading
  error=side_new - side;
  //if the error is larger than 40%
  if (abs(err/side)>0.4){
    adjust(err);
  } 
  //update front distance, also update side distance

  //check tilted angle and accelerate
  tilt_angle=mpu.getAngleY();
  if (abs(tilt_angle) > 30){
    //full speed
     motors.setSpeedA(255);
      motors.setSpeedB(255);
  }
  //------

}
