#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <MPU6050_tockn.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

long Prev_Left_Motor_Position;
long Prev_Right_Motor_Position;
long Curr_Left_Motor_Position;
long Curr_Right_Motor_Position;

MPU6050 mpu6050(Wire);

int US_check = 0;

//pins (changeable)
const int ultrasonic_Front_IN = 2;
const int ultrasonic_Front_OUT = 2;
const int ultrasonic_Left_IN = 4;
const int ultrasonic_Left_OUT = 4;
const int ultrasonic_Right_IN = 5;
const int ultrasonic_Right_OUT = 5;
const int motor_Right = 6;
const int motor_Left = 7;
const int infared_Front = 8;
const int infared_Back = 9;

//calibration values
unsigned int motor_Speed = 1900;        //run speed
unsigned int motor_Speed_Offset = 200;
unsigned int motor_Left_Speed;
unsigned int motor_Right_Speed;
unsigned int ultrasonic_Front_Middle_Distance = 500;  //FM US distance
unsigned int ultrasonic_Left_Distance = 500;          //L US distance 
unsigned int ultrasonic_Right_Distance = 500;         //L US distance 


//helper variables
boolean ultrasonic_Clear[5] = {true,true,true,true,true}; //state for ultrasonic sensors (FL,FM,FR,L,R). Stores if value is lower than limit (aka ur gonna crash soon)
int timeFlag = 0;
int runFlag = 0;
int turnTime45 = 500;    //time to spin when stuck
int turnTime180 = 2000;   //rotation time
int infaredCheckInterval = 10000;   //how frequent to check for infared
int infaredCheckTime = 1000;        //how long to spin for when checking infared
int infaredNumber = 0;
unsigned long ul_Echo_Time;
boolean Encoder_Turn = true;
boolean East_check;
int ZigZag = 0;


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

  delay(5000);
  servo_LeftMotor.writeMicroseconds(1700);
  servo_RightMotor.writeMicroseconds(1700);
}

void loop() {

  US_check = check_US();
 
  //run 1(0-30sec)
  if (millis<30000){
    
     if (US_check == 1){
       servo_LeftMotor.writeMicroseconds(1300);
       servo_RightMotor.writeMicroseconds(1700);
       Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
       if (Curr_Right_Motor_Position - Prev_Right_Motor_Position >= 200){ // Maybe change 200
        servo_LeftMotor.writeMicroseconds(1700);
        servo_RightMotor.writeMicroseconds(1700);
        US_check = 0; 
       }
     }
     else if (US_check == 2){
       servo_LeftMotor.writeMicroseconds(1700);
       servo_RightMotor.writeMicroseconds(1300);
       Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
       if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 200){ // Maybe change 200
        servo_LeftMotor.writeMicroseconds(1700);
        servo_RightMotor.writeMicroseconds(1700);
        US_check = 0; 
       }
     }
  }

  if ((millis > 30000) && (direction == false){
    servo_LeftMotor.writeMicroseconds(1700);
    servo_RightMotor.writeMicroseconds(1300);
  }
  
  if ((millis > 30000) && (direction == true)){
    if (ZigZag == 0){
      servo_LeftMotor.writeMicroseconds(1700);
      servo_RightMotor.writeMicroseconds(1300);
      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); // Add Prev update into US check
       if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 200){ // Maybe change 200
        ZigZag = 1;
       }
    else if (ZigZag == 1){
      servo_LeftMotor.writeMicroseconds(1300);
      servo_RightMotor.writeMicroseconds(1700);
      Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
       if (Curr_Right_Motor_Position - Prev_Right_Motor_Position >= 200){ // Maybe change 200
        ZigZag = 0;
        } 
    }
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

  return (ul_Echo_Time/58);
#endif
}

int check_US() // Function to check ultrasonics
{
  int check = 0;
  int middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT); // Change to actual variable
  if (middle <= 7){
    int left = Ping(ultrasonic_Left_IN, ultrasonic_Left_OUT); // change to actual variables for pins of US
    int right = Ping(ultrasonic_Right_IN, ultrasonic_Right_OUT);
    if (right < left){
      servo_LeftMotor.writeMicroseconds(1300);
      servo_RightMotor.writeMicroseconds(1700);
      check = 1;
    }
    else {
      servo_LeftMotor.writeMicroseconds(1700);
      servo_RightMotor.writeMicroseconds(1300);
      check = 2;
    }
    Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
    Prev_Right_Motor_Position = encoder_RightMotor.getRawPosition();
  }
  else {
    continue; // keep current speeds
  }
  return check;
}

//functions
void rotate90(){
  int preSpin = millis();
  int postSpin = preSpin;
  
  //note: while function is acceptable for resets
  //note2: this will be a stationary turn. does not need to account for obstacles
  while ((postSpin-preSpin)>=turnTime45){
    //turning right for first run cycle
    if (runFlag == 0){
      motor_Left_Speed = 1600;
      motor_Right_Speed = 800;
    }

    //turning left for second run cycle
    else{
      motor_Left_Speed = 800;
      motor_Right_Speed = 1600; 
    } 
  }
}

void rotate180(){
  int preSpin = millis();
  int postSpin = preSpin;
  
  //note: while function is acceptable for resets
  //note2: this will be a stationary turn. does not need to account for obstacles
  while ((postSpin-preSpin)>=turnTime180){
      motor_Left_Speed = 1600;
      motor_Right_Speed = 800;
  }
}

void infaredCheck(){
  int preSpin = millis();
  int postSpin = preSpin;
  while(postSpin - preSpin >= infaredCheckTime){
    //returns 0 if found on front
    if (digitalRead(infared_Front)==0){
      infaredNumber = 1;
    }
    //returns 1 if seen from back
    else if (digitalRead(infared_Back)==0){
      infaredNumber = -1;
    }
      motor_Left_Speed = 1600;
      motor_Right_Speed = 800;
  }
  //returns -1 if not found
  infaredNumber = 0;
}
