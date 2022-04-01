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
#include <L298NX2.h>
#include "ArduPID.h"

//global vars
bool can_turn = false;
int turnCount=0;
double turn_offset = 10;

//controllers
ArduPID side_controller;
ArduPID front_controller;
ArduPID turn_controller;

//PID for Driving Straight
double setpoint_s = 0;
double input_s;
double output_s;
double p = 72000;
double i = 0;
double d = 0;

/* old values
 * double p = 15000;
double i = 0;
double d = 22;
*/
//PID for Drive to turning point
double setpoint_f = 114;
double input_f;
double output_f;
double p2 = 4;
double i2 = 0;
double d2 = 0;

//PID for turning
double setpoint_t = 0; 
double input_t;
double output_t;
double p3 = 10;
double i3= 0;
double d3 = 0;

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
#define LOX3_ADDRESS 0x32
int s_front,s_side, s_side_2;

// set the pins to shutdown
#define SHT_LOX1 12
#define SHT_LOX2 13
#define SHT_LOX3 11

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure_front;
VL53L0X_RangingMeasurementData_t measure_side;
VL53L0X_RangingMeasurementData_t measure_side_2;

void setID() { //set addresses for devices on i2c bus (tof and gyro)
  Serial.println("doing set id");
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and reseting others
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

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

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);
  
  //initing LOX3
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }

  delay(1000);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  //set up PID for driving straight
  side_controller.begin(&input_s, &output_s, &setpoint_s, p, i, d);
  side_controller.setOutputLimits(-1750, 1750);
  //side_controller.setBias(50000.0 / 2.0);
  side_controller.setWindUpLimits(-10.0, 10.0); // Groth bounds for the integral term to prevent integral wind-up
  
  side_controller.start();
  side_controller.reverse();

  //set up PID for drive and turn
  front_controller.begin(&input_f, &output_f, &setpoint_f, p2, i2, d2);
  front_controller.setOutputLimits(-1750, 1750);
  //side_controller.setBias(255.0 / 2.0);
  front_controller.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  front_controller.start();
  front_controller.reverse();

  //set up PID for turning
  turn_controller.begin(&input_t, &output_t, &setpoint_t, p3, i3, d3);
  turn_controller.setOutputLimits(-1750, 1750);
  //side_controller.setBias(255.0 / 2.0);
  turn_controller.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  turn_controller.start();
  turn_controller.reverse();
  
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  
  Serial.println("Shutdown pins inited...");
  
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  
  Serial.println("Both in reset mode...(pins are low)");
  
  
  Serial.println("Starting...");
  setID();
  
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
    //Serial.println(s_s);
    total += s_s;
  }
  return (total/2);
}

float get_side_2_distance() {
  float total = 0;
  for(int i = 0; i < 2; i++){
    lox3.rangingTest(&measure_side_2, false); //need this for sensor reading to work!
    float s_s = measure_side_2.RangeMilliMeter;
    //Serial.println(s_s);
    total += s_s;
  }
  return (total/2);
}

void turn_90() { //does a 90deg turn to the right
  //Serial.println("turning 90");
  motors.stop();
  delay (300);
  motors.setSpeedA(200);
  motors.setSpeedB(200);
  motors.forward();
  delay(478);
  motors.stop();
  can_turn = false; //just did turn so don't turn again
  delay (300);
}

void controllerSide(double output) {
  double speedA = 0;
  double speedB = 0;

  if(output>0){
    //away from wall, turn left
    speedA = 255 - output/231;
    //range is 125 - 255 = 130
    //255 - 30000/x = 125
    //x = 231;
    speedB = 248;
    //Serial.println("Turning left!!!");
    }
   else{
    //close to wall, turn right
    speedA = 255;
    speedB = 248 - output/-231;
    //Serial.println("Turning right!!!");
    }
    motors.setSpeedA(speedA); 
    motors.setSpeedB(speedB);
}

void drive_straight() {
  input_s = get_sidedistance() - get_side_2_distance()-10;

  side_controller.compute();
  side_controller.debug(&Serial, "side_controller", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
  Serial.println();
  controllerSide(output_s); // Replace with plant control signal
}

void controllerFront(double output){
  double speedA = 0; //70+output2/10;
  double speedB = 0; //85+output2/10;
  
  /*
  input = 200, output = 280
  input = min, output = -920
  input = max, output = 1750
  */
  
  if(output > 250) { //away from wall, keep driving
    //Serial.println("driving straight");
    drive_straight();
  } else if (output > 0) { //close to wall, start stopping
      //Serial.println("slowing down");
      //motor range 70 - 255: 185
      //output range 0 - 280: 280
      //185 = 280x
      //x = 0.661
      speedA = 70+output*0.661; //fcn(output2)
      speedB = 70+output*0.661; //fcn(output2)
      motors.setSpeedA(speedA);
      motors.setSpeedB(speedB);
  } else { //have reached turning point
      Serial.println("stop!!!");
      motors.stop();
      motors.setSpeedA(0);
      motors.setSpeedB(0);
      can_turn = true;
      delay(1000);
  } 
    /*else { //close to wall (overshoot), back up
    speedA = 0; //fcn(output2)
    speedB = 0; //fcn(output2)
    }*/   
  
  /*x < output2 < y
  if(speedA == 70) { //stop motors
    motors.stop();
    delay(1000);
  }*/
  
}

void drive_front(){
  input_f = get_frontdistance() - setpoint_f;
  front_controller.compute();
  /*front_controller.debug(&Serial, "front_controller", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);*/
  controllerFront(output_f);
}

void controllerTurn(double output_t){
  //set motors for turning here
  double delaytime=0;
  if (abs(output_t)<60){
    turnCount++;   
  }else if (output_t<0){
    //sidereading 1 < sidereading 2 -> tilt right, turn right -> B higher
      delaytime = abs(output_t/3);
      motors.setSpeedA(110); 
      motors.setSpeedB(110);
      motors.forward();
      delay(delaytime);
      motors.stop();
      delay(10);
  }else if (output_t>0){
      delaytime = abs(output_t/3);
      motors.setSpeedB(110); 
      motors.setSpeedA(110);
      motors.backward();
      delay(delaytime);
      motors.stop();
      delay(10);
  }
}

void drive_turn(){
  input_t = get_sidedistance() - get_side_2_distance()- turn_offset;
  
  /*Serial.println("sidereading1:");
  Serial.println(get_sidedistance());
  Serial.println("sidereadindg2:");
  Serial.println(get_side_2_distance());*/
 // Serial.println("difference:");
  //Serial.println(input_t);
  delay(10);
  
  turn_controller.compute();
  /*
  turn_controller.debug(&Serial, "turn_controller", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
                                              */
  controllerTurn(output_t);
}

/*void funky_main() {
  int i = 0; //number of turns completed
  int n_turns = 10; //total number of turns to be made
  float wall_distance = 130;
  while(i < 10) { //still have turns to complete
    //navigate danger zones
    switch(i){
      case(3): //pit_pit
        blind_drive(2200);
        break;
      case(4): //gravel_normal_gravel
        blind_drive(2200);
        break;
      case(5): //gravel_gravel
        blind_drive(2000);
        break;
      case(6): //sand_sand
        blind_drive(2000);
        break;
      case(7): //sand
        blind_drive(1500);
        break;
      case(8): //sand
        blind_drive(1500);
        break;
    }
    
    switch(i){ //incrementing wall distances
      case(3):
        wall_distance = 430;
        setpoint_s = 420; 
        break;
      case(7):
        wall_distance = 730;
        setpoint_s = 720;
        break;
    }
    drive_until(wall_distance);
    turn_90();
    i++;
  }
  drive_until(wall_distance); //drive the last stretch
  delay(100000);
}*/


void loop() {
  int i = 0;
  int x = 0;
  while(i <= 10) { //go in one outer spiral
    //Serial.println(can_turn);
    can_turn = false;
    if(i == 3){ //hardcode driving for sand pit
      //Serial.println("in if statement 3");
      while (turnCount != 3){ //adjust after turning
        //Serial.println("adjusting before hard drive");
        drive_turn();
      }
      turnCount = 0;
      
      while(x < 13) {
        //Serial.println("hard drive");
        drive_straight();
        motors.forwardA();
        motors.backwardB();
        delay(100);
        x++;
      }
      motors.stop();
      delay(1000);
      x = 0;
      setpoint_f = 240; //increase stopping distance
      turn_offset = 15;
      while (turnCount != 3){ //adjust after turning
       // Serial.println("adjusting after hard drive");
        //Serial.println(turnCount);
        drive_turn();
      }
      turnCount = 0;
 
    }
    //Serial.println("outside if 3");

    if(i == 4){ //hard drive
      while(x < 11) {
        //Serial.println("hard drive");
        drive_straight();
        motors.forwardA();
        motors.backwardB();
        delay(100);
        x++;
      }
      x = 0;
      setpoint_f = 255;
    }
    if(i == 5){
      while(x < 8) {
        //Serial.println("hard drive");
        drive_straight();
        motors.forwardA();
        motors.backwardB();
        delay(100);
        x++;
      }
      x = 0;
    }
    if(i == 6 ){
      while(x < 8) {
        //Serial.println("hard drive");
        drive_straight();
        motors.forwardA();
        motors.backwardB();
        delay(100);
        x++;
      }
      x = 0;
    }
    if(i == 7 ){
      while(x < 7) {
        //Serial.println("hard drive");
        drive_straight();
        motors.forwardA();
        motors.backwardB();
        delay(100);
        x++;
      }
      x = 0;
      setpoint_f = 600;
      turn_offset = 25.5;
 
    }
    if(i == 8){
      while(x < 6) {
        //Serial.println("hard drive");
        drive_straight();
        motors.forwardA();
        motors.backwardB();
        delay(100);
        x++;
      }
      x = 0;
    }
    if( i > 8){
      while(x < 3) {
        //Serial.println("hard drive");
        drive_straight();
        motors.forwardA();
        motors.backwardB();
        delay(100);
        x++;
      }
      x = 0;
    }
    while(!can_turn) { //get to turning point
      //Serial.println("drive front");
      drive_front();
      motors.forwardA();
      motors.backwardB();
      delay(100);
    }
    if(i != 10){
      //Serial.println("turning 90");
      turn_90();
      while (turnCount != 3){ //adjust after turning
        drive_turn();
      }
      turnCount = 0;
      //Serial.println("turned 90");
    }
    //Serial.println("increment i");
    i++;
    turnCount = 0;
  }
  motors.stop();
  delay(10000);
   /*while(1){
    drive_front();
    delay(100); 
   }*/
  //Serial.println(get_frontdistance());
}  
