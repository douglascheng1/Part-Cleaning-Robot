int state;

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

//pins (changeable)
const int ultrasonic_Front_IN = 6;
const int ultrasonic_Front_OUT = 5;
const int ultrasonic_Left_IN = 8;
const int ultrasonic_Left_OUT = 7;
const int ultrasonic_Right_IN = 4;
const int ultrasonic_Right_OUT = 3;
const int motor_Right = 11;
const int motor_Left = 10;
const int infared_Front = 8;
const int infared_Back = 9;
//const int bumper = 2;

//calibration values
unsigned int motor_Speed = 1900;        //run speed
unsigned int motor_Speed_Offset = 200;
unsigned int motor_Left_Speed;
unsigned int motor_Right_Speed;
unsigned int ultrasonic_Front_Middle_Distance = 500;  //FM US distance
unsigned int ultrasonic_Left_Distance = 500;          //L US distance 
unsigned int ultrasonic_Right_Distance = 500;         //L US distance 


//helper variables
int timeFlag = 0;
int runFlag = 0;
int turnTime45 = 500;    //time to spin when stuck
int turnTime180 = 2000;   //rotation time
int infaredCheckInterval = 10000;   //how frequent to check for infared
int infaredCheckTime = 1000;        //how long to spin for when checking infared
int infaredNumber = 0;
unsigned long ul_Echo_Time;
boolean East_check;
int ZigZag = 0;
boolean Left_turn;
boolean Right_turn;
boolean right_hit = false;
const int left_bumper = 9;
boolean left_hit = false;
int US_check = 0;

void check_US() // Function to check ultrasonics
{
  int middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT); // Change to actual variable
  Serial.println(middle);
  if (middle <= 15){
    US_check++;
    if (US_check >=20){
      int left = Ping(ultrasonic_Left_IN, ultrasonic_Left_OUT); // change to actual variables for pins of US
      int right = Ping(ultrasonic_Right_IN, ultrasonic_Right_OUT);
      Serial.print("Right ");
      Serial.println(right);
      Serial.print("Left ");
      Serial.println(left);
      if (right < left){
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
  }
  else {
   US_check = 0;
     // keep current speeds
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
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  // setup compass
  Wire.begin();
 // mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

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
  pinMode(left_bumper, INPUT_PULLUP);

  delay(3000);
  servo_LeftMotor.writeMicroseconds(1730);
  servo_RightMotor.writeMicroseconds(1700);
}

void loop() {

 //delay(80);
 check_US();

 //Serial.print("CURR: ");
 //Serial.println(Curr_Left_Motor_Position);
 
 
  //run 1(0-30sec)
  if (right_hit == false && left_hit == false && digitalRead(A2) == HIGH && digitalRead(left_bumper) == HIGH){
  
    if (1==1){
      
       if (Left_turn == true){
         servo_LeftMotor.writeMicroseconds(1300);
         servo_RightMotor.writeMicroseconds(1500);
         Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
         //Serial.println(Curr_Right_Motor_Position - Prev_Right_Motor_Position);
         if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position) >= 300){ // Maybe change 200
          servo_LeftMotor.writeMicroseconds(1730);
          servo_RightMotor.writeMicroseconds(1700);
          Left_turn = false;
         }
       }
       else if (Right_turn == true){
         servo_LeftMotor.writeMicroseconds(1500);
         servo_RightMotor.writeMicroseconds(1300);
         Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
         if ((Prev_Right_Motor_Position - Curr_Right_Motor_Position) >= 300){ // Maybe change 200
          servo_LeftMotor.writeMicroseconds(1730);
          servo_RightMotor.writeMicroseconds(1700);
          Right_turn = false;
         }
       }
    }
  }
  
  else if (right_hit == true){
    servo_LeftMotor.writeMicroseconds(1300);
    servo_RightMotor.writeMicroseconds(1500);
    Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); 
    if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position)>600){
     servo_LeftMotor.writeMicroseconds(1700);
     servo_RightMotor.writeMicroseconds(1700);
     right_hit = false;
    }
  }
  else if (left_hit == true){
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1300);
    Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition(); 
    if ((Prev_Right_Motor_Position - Curr_Right_Motor_Position)>600){
     servo_LeftMotor.writeMicroseconds(1700);
     servo_RightMotor.writeMicroseconds(1700);
     left_hit = false;
    }
  }
  else if (digitalRead(A2) == LOW){
    right_hit = true;
    left_hit = false;
    servo_LeftMotor.writeMicroseconds(1300);
    servo_RightMotor.writeMicroseconds(1500);
    Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); 
  }
  else if (digitalRead(left_bumper) == LOW){
    right_hit = false;
    left_hit = true;
    servo_LeftMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1300);
    Prev_Right_Motor_Position = encoder_RightMotor.getRawPosition(); 
  }

  
//  if ((millis > 30000) && (direction == false){
//    servo_LeftMotor.writeMicroseconds(1700);
//    servo_RightMotor.writeMicroseconds(1300);
//  }
//  
//  if ((millis > 30000) && (direction == true)){
//    if (ZigZag == 0){
//      servo_LeftMotor.writeMicroseconds(1700);
//      servo_RightMotor.writeMicroseconds(1300);
//      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); // Add Prev update into US check
//       if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 200){ // Maybe change 200
//        ZigZag = 1;
//       }
//    else if (ZigZag == 1){
//      servo_LeftMotor.writeMicroseconds(1300);
//      servo_RightMotor.writeMicroseconds(1700);
//      Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
//       if (Curr_Right_Motor_Position - Prev_Right_Motor_Position >= 200){ // Maybe change 200
//        ZigZag = 0;
//        } 
//    }
//  }

}
