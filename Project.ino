#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <MPU6050_tockn.h>

//servo declaration
Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo basket;
Servo arm;


I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

long Prev_Left_Motor_Position;
long Prev_Right_Motor_Position;
long Curr_Left_Motor_Position;
long Curr_Right_Motor_Position;

MPU6050 mpu6050(Wire);

int US_check = 0;

//pins (changeable)
const int ultrasonic_Front_IN = 6;
const int ultrasonic_Front_OUT = 5;
const int ultrasonic_Left_IN = 8;
const int ultrasonic_Left_OUT = 7;
const int ultrasonic_Right_IN = 4;
const int ultrasonic_Right_OUT = 0;
const int motor_Right = 11;
const int motor_Left = 10;
const int infrared_Left = A0;
const int infrared_Right = A1;
const int infrared_Front = A3;

//const int bumper = 2;

//calibration values
unsigned int motor_Speed = 1900;        //run speed
unsigned int motor_Speed_Offset = 200;
unsigned int motor_Left_Speed;
unsigned int motor_Right_Speed;
unsigned int distance_Front = 20;  //FM US distance
unsigned int distance_Left = 20;  //FM US distance
unsigned int distance_Right = 20;  //FM US distance
unsigned int distance_US = 20;


//helper variables
unsigned long ul_Echo_Time;
bool Left_turn;
bool Right_turn;
bool hit = false;
bool isReturning = false;
bool turnt = false;
unsigned int timeCharged = 0;
int direction_East = 0;
int degree_Tolerance = 5;
bool isReturned = false;
bool infraredSeen_Right = false;
bool infraredSeen_Left = false;
bool infraredSeen_Front = false;
int infrared_Max = 800;
int returnState = 0;
int mode = 2;
bool firstRun = true;
int distance = 20;
int bearing = 0;
double initAngle = 0; //initial angle of compass to be stored
unsigned int infraredSeen_Last = 0;
bool infraredSeen = false;
int lastmode = 0;

//dummy function for gyroscope
double getDegrees() {
  double currAngle = mpu6050.getAngleZ();
  if (currAngle < 0)
  {
    currAngle = currAngle + (((int(currAngle)) / 360) * -360) + 360;
  }
  if (currAngle > 360)
  {
    currAngle = currAngle - (((int(currAngle)) / 360) * 360);
  }
  return currAngle;
}

void check_US() // Function to check ultrasonics
{
  int middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT); // Change to actual variable
  //Serial.println(middle);
  if (middle <= 15) {
    int left = Ping(ultrasonic_Left_IN, ultrasonic_Left_OUT); // change to actual variables for pins of US
    int right = Ping(ultrasonic_Right_IN, ultrasonic_Right_OUT);
//    Serial.print("Right ");
//    Serial.println(right);
//    Serial.print("Left ");
//    Serial.println(left);
    if (right < left) {
      servo_LeftMotor.writeMicroseconds(1300);
      servo_RightMotor.writeMicroseconds(1500);
      Left_turn = true;
      Right_turn = false;
    }
    else if (left <= right) {
      servo_LeftMotor.writeMicroseconds(1500);
      servo_RightMotor.writeMicroseconds(1300);
      Right_turn = true;
      Left_turn = false;
    }
    Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
    Prev_Right_Motor_Position = encoder_RightMotor.getRawPosition();
  }
  else {
    // keep current speeds
      servo_LeftMotor.writeMicroseconds(1700);
      servo_RightMotor.writeMicroseconds(1700);
  }
}


// measure distance to target using ultrasonic sensor
int Ping(int ci_Ultrasonic_Data, int ci_Ultrasonic_Ping)
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  return (ul_Echo_Time / 58);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // setup compass
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(ultrasonic_Front_IN, INPUT); // Set up Ultra Sonics
  pinMode(ultrasonic_Front_OUT, OUTPUT);
  pinMode(ultrasonic_Left_IN, INPUT);
  pinMode(ultrasonic_Left_OUT, OUTPUT);
  pinMode(ultrasonic_Right_IN, INPUT);
  pinMode(ultrasonic_Right_OUT, OUTPUT);

  // set up drive motors
  pinMode(motor_Right, OUTPUT);
  servo_RightMotor.attach(motor_Right);
  pinMode(motor_Left, OUTPUT);
  servo_LeftMotor.attach(motor_Left);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  pinMode(A2, INPUT_PULLUP);
  //attachInterrupt(0, bumpers, RISING);

  delay(3000);
  servo_LeftMotor.writeMicroseconds(1750);
  servo_RightMotor.writeMicroseconds(1700);

  initAngle = mpu6050.getAngleZ();
  mpu6050.update();
  arm.attach(9);
  basket.attach(3);  

}


//insert dump code here (it is okay to have stopping code)
void dump() {
   arm.write(10);
  basket.write(180);
  delay(1000);
  arm.write(40);
  delay(1000);
  arm.write(80);
  delay(100);
  basket.write(140);
  delay(1000);
  arm.write(120);
  delay(1000);
  basket.write(120);
  delay(1000);
  arm.write(150);
  delay(100);
  basket.write(95);
  delay(2000);
  arm.write(170);
  delay(100);
  basket.write(115);
  delay(1000);
}

void loop() {
    mpu6050.update();
//  Serial.println("D,B,Mode");
 // Serial.println(mpu6050.getAngleZ());
  //Serial.print(getDegrees());
  if (mode!=lastmode){
      Serial.print(mode);
      Serial.print(" ");
      Serial.print(getDegrees());
      Serial.println(" ");
  }
  lastmode = mode;
  

  //always check IRs first and front distance
  infraredSeen_Right = (analogRead(infrared_Right) < infrared_Max); //resets to see iff infrareds are seen
  infraredSeen_Left = (analogRead(infrared_Left) < infrared_Max);
  infraredSeen_Front = (analogRead(infrared_Front) < infrared_Max);
  distance_Front = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT);
 if(infraredSeen_Front){
  Serial.println("Front Seen!");
 }
 if(infraredSeen_Right){
  Serial.println("Right Seen!");
 }
  if(infraredSeen_Left){
  Serial.println("Left Seen!");
 }
  
  if (infraredSeen_Left || infraredSeen_Right || infraredSeen_Front) {
    infraredSeen_Last = millis();
    infraredSeen = true;
  }
  else {
    infraredSeen = false;
  }

  switch (mode) {
    //normal run code (until 30s)
    case 1:
      //rotate towards S after 30s, then change modes
      Curr_Left_Motor_Position = Prev_Left_Motor_Position;
      if (millis() - timeCharged >= 30000) {

        if (Curr_Left_Motor_Position - Prev_Left_Motor_Position <= 600) {
          Prev_Left_Motor_Position = Curr_Left_Motor_Position;
          Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();    //gets motor position
        }
        else {
          mode++;
        }
      }
      break;

    //coming back code (assume that there is one infrared in back, right and left)
    //note: backwards driving towards east: more important that it makes it in than pick up shitsecond round
    case 2:
      //infrared not seen for 20s, rotate around
      if (millis() - infraredSeen_Last >= 20000) {
        infraredSeen_Last = millis();
        mode = 55;
      }
      else if (infraredSeen_Front) {
        infraredSeen_Last = millis();
        bearing  = getDegrees();
        if (bearing >= 175 && bearing <= 185) {
          mode++;
        }
        //go South
        else if (bearing > 180) {
          mode = 99;
        }
        //go North (rotate counterclockwise until bearing = 270)
        else {
          mode = 88;
        }
      }
      //rotate Westwards
      else if (infraredSeen_Right || infraredSeen_Left) {
        infraredSeen_Last = millis();
        //rotate clockwise until front sees
        if (infraredSeen_Right) {
          mode = 77;
        }
        //rotate counterclockwise until front sees
        else {
          mode = 66;
        }
      }
      else {
        check_US();
      }

      break;

    //reverse code
    case 3:
      //if angle greater than +-3deg, turn
      if (getDegrees() <= 3 || getDegrees() >= 358) {
        servo_LeftMotor.writeMicroseconds(1700);
        servo_RightMotor.writeMicroseconds(1300);
      }
      else {
        mode++;
      }
      break;

    //back in code (go back set distance + constant)
    case 4:
      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();    //gets motor position
      //resets charge time and move on
      if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= distance + 2) {
        servo_LeftMotor.writeMicroseconds(1500);    //stops motors
        servo_RightMotor.writeMicroseconds(1500);   //stops motors
        timeCharged = millis();
        mode++;
      }
      //go backwards
      else {
        servo_LeftMotor.writeMicroseconds(1300);
        servo_RightMotor.writeMicroseconds(1300);
      }
      break;

    //dumping code, then reset
    case 5:
      dump();
      //reset after dumped
      if (firstRun) {
        firstRun = false;
        mode = 0;
      }
      else {
        Serial.println("FINISHED");
      }
      break;
    //rotates around for X seconds to reset
    case 55:
      if (infraredSeen) {
        mode = 2;
      }
      //rotate
      else if (millis() - infraredSeen_Last < 3000) {
        servo_LeftMotor.writeMicroseconds(1700);
        servo_RightMotor.writeMicroseconds(1300);
      }
      else {
        mode = 2;
      }
      break;
    //rotate counter-c until front sees
    case 66:
        //spin time limit of 4 sec to avoid getting stuck
      if (millis() - infraredSeen_Last>=4000){
        mode = 2;
      }
      if (infraredSeen_Front) {
        mode = 2;
      }
      else {
        servo_LeftMotor.writeMicroseconds(1700);
        servo_RightMotor.writeMicroseconds(1300);
      }
      break;

    //rotate c until front sees
    case 77:
    //spin time limit of 4 sec to avoid getting stuck
      if (millis() - infraredSeen_Last>=4000){
        mode = 2;
      }
      if (infraredSeen_Front) {
        mode = 2;
      }
      else {
        servo_LeftMotor.writeMicroseconds(1300);
        servo_RightMotor.writeMicroseconds(1700);
      }
      break;


    //rotate until getdegrees facing N
    case 88:
      if (getDegrees() < 92 && getDegrees() > 88) {
        mode = 2;
      }
      else {
        servo_LeftMotor.writeMicroseconds(1300);
        servo_RightMotor.writeMicroseconds(1700);
      }
      break;

    //rotate until getdegrees facing S
    case 99:
      if (getDegrees() < 272 && getDegrees() > 268) {
        mode = 2;
      }
      else {
        servo_LeftMotor.writeMicroseconds(1700);
        servo_RightMotor.writeMicroseconds(1300);
      }
      break;
  }

}
