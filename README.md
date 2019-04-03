int state = 0;

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <MPU6050_tockn.h>
#include <SoftwareSerial.h>

SoftwareSerial front_ir(A3, 13);
SoftwareSerial left_ir(A0, 13);
SoftwareSerial right_ir(A1, 13);// RX, TX

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
const int ultrasonic_Right_OUT = 0;
const int motor_Right = 11;
const int motor_Left = 10;

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
boolean East_check = false;
int ZigZag = 0;
boolean Left_turn;
boolean Right_turn;
boolean right_hit = false;
const int left_bumper = 1;
boolean left_hit = false;
int US_check = 0;
const double alpha = 0.9;
int last_middle = 0;
int last_right = 0;
int last_left = 0;
long start_time = 0;
long end_time = 0;
boolean front_IR = false; // A3
boolean left_IR = false; // A0
boolean right_IR = false; // A1
boolean IR_seen = false;
boolean IR_seen_again = false;
boolean North_check = false;
boolean South_check = false;
boolean West_check = false;
int dir;
boolean done_turn = false;
int turn = 0;
double compass_offset;
int right_count = 0;
int left_count = 0;
int front_count = 0;
long last_time;
int IR_state;

void check_US() // Function to check ultrasonics
{
  int middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT); // Change to actual variable
  //Serial.print(middle);Serial.print('\t');
  middle = (1 - alpha) * middle + alpha * last_middle;
  last_middle = middle;
  Serial.println(middle);
  if (middle <= 15) {
    int left = Ping(ultrasonic_Left_IN, ultrasonic_Left_OUT); // change to actual variables for pins of US
    int right = Ping(ultrasonic_Right_IN, ultrasonic_Right_OUT);
    Serial.print("Right ");
    Serial.println(right);
    Serial.print("Left ");
    Serial.println(left);
    //      right = (1-alpha)*right + alpha*last_right;
    //      right = (1-alpha)*left + alpha*last_left;
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
    //Serial.println(Prev_Left_Motor_Position);
  }
  else {
    // keep current speeds
  }
}

//void check_US_wayback() // Function to check ultrasonics
//{
//  int middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT); // Change to actual variable
//  //Serial.print(middle);Serial.print('\t');
//  middle = (1-alpha)*middle + alpha*last_middle;
//  last_middle = middle;
//  Serial.println(middle);
//  if (middle <= 15){
//      int left = Ping(ultrasonic_Left_IN, ultrasonic_Left_OUT); // change to actual variables for pins of US
//      int right = Ping(ultrasonic_Right_IN, ultrasonic_Right_OUT);
//      Serial.print("Right ");
//      Serial.println(right);
////      Serial.print("Left ");
////      Serial.println(left);
////      right = (1-alpha)*right + alpha*last_right;
////      right = (1-alpha)*left + alpha*last_left;
//      if (right < left){
//        servo_LeftMotor.writeMicroseconds(1300);
//        servo_RightMotor.writeMicroseconds(1500);
//        Left_turn = true;
//        Right_turn = false;
//      }
//      else if (left <= right) {
//        servo_LeftMotor.writeMicroseconds(1500);
//        servo_RightMotor.writeMicroseconds(1300);
//        Right_turn = true;
//        Left_turn = false;
//      }
//      Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
//      Prev_Right_Motor_Position = encoder_RightMotor.getRawPosition();
//      //Serial.println(Prev_Left_Motor_Position);
//      turn = 1;
//    }
//  else {
//    turn = 0;
//     // keep current speeds
//  }
//}

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

int getDirection() {
  mpu6050.update();
  double Angle = mpu6050.getAngleZ();
  Angle = Angle  - compass_offset;
  double currAngle;
  if (Angle < 0)
  {
    currAngle = Angle + (((int(Angle)) / 360) * -360) + 360;
  }
  else if (Angle > 360)
  {
    currAngle = Angle - (((int(Angle)) / 360) * 360);
  }
  else if (Angle >= 0 && Angle <= 360) {
    currAngle = Angle;
  }

  if (currAngle <= 5 || currAngle >= 355) {
    dir = 1; // W
  }
  if (currAngle > 5 && currAngle < 85) {
    dir = 2; // SW
  }
  if (currAngle >= 85 && currAngle <= 95) {
    dir = 3; // S
  }
  if (currAngle > 95 && currAngle < 178) {
    dir = 4; // SE
  }
  if (currAngle >= 178 && currAngle <= 182) {
    dir = 5; // E
  }
  if (currAngle > 182 && currAngle < 265) {
    dir = 6; // NE
  }
  if (currAngle >= 265 && currAngle <= 275) {
    dir = 7; // N
  }
  if (currAngle > 275 && currAngle < 355) {
    dir = 8; // NW
  }
  //Serial.println(currAngle);
  return dir;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // set the data rate for the SoftwareSerial port
//  front_ir.begin(2400);
//  left_ir.begin(2400);
//  right_ir.begin(2400);
  //mySerial.println("Hello, world?");

  // setup compass
  Wire.begin();
  //  mpu6050.begin();
  //  mpu6050.calcGyroOffsets(true);

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

  last_middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT);
  last_left = Ping(ultrasonic_Left_IN, ultrasonic_Left_OUT);
  last_right = Ping(ultrasonic_Right_IN, ultrasonic_Right_OUT);
  //Serial.println(last_middle);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A3, INPUT);

  //  delay(3000);
  //  servo_LeftMotor.writeMicroseconds(1700);
  //  servo_RightMotor.writeMicroseconds(1700);
  start_time = millis();
  Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  state = 0;

  //  mpu6050.update();
  //  double start_Angle=mpu6050.getAngleZ();
  //  compass_offset = start_Angle;
  //  Serial.println(compass_offset);

  last_time = millis();
}

void loop() {
  if ((millis()-last_time)>=50){
    last_time = millis();
    IR_state++;
    if (IR_state == 1){
      front_ir.begin(2400);
    }
    else if (IR_state == 2){
      left_ir.begin(2400);
    }
    else if (IR_state == 3){
      right_ir.begin(2400);
      IR_state = 0;
    }
  }
  
  if (front_ir.available()) {
    Serial.print("Front ");
    int x = front_ir.read();
    if ((x >= 49 && x <= 53) || (x >= 64 && x <= 68)) {
      Serial.println("FOUND");
    }
    else {
      Serial.println("NOT FOUND");
    }
  }
  if (right_ir.available()) {
    Serial.print("Right ");
    int x = right_ir.read();
    if ((x >= 49 && x <= 53) || (x >= 64 && x <= 68)) {
      Serial.println("FOUND");
    }
    else {
      Serial.println("NOT FOUND");
    }
  }
  if (left_ir.available()) {
    Serial.print("Left ");
    int x = left_ir.read();
    if ((x >= 49 && x <= 53) || (x >= 64 && x <= 68)) {
      Serial.println("FOUND");
    }
    else {
      Serial.println("NOT FOUND");
    }
  }


  //  mpu6050.update();
  //  getDirection();



  // // Serial.println(digitalRead(A0));
  //
  // //delay(100);
  // check_US();
  //
  // //Serial.print("CURR: ");
  // //Serial.println(Curr_Left_Motor_Position);
  //
  // if (end_time - start_time < 500){
  //  end_time = millis();
  //  Right_turn = false;
  //  Left_turn = false;
  // }
  //
  //
  //  //run 1(0-30sec)
  //  if (right_hit == false && left_hit == false && digitalRead(A2) == HIGH && digitalRead(left_bumper) == HIGH){
  //
  //    if (1==1){
  //
  //       if (Left_turn == true){
  //         servo_LeftMotor.writeMicroseconds(1300);
  //         servo_RightMotor.writeMicroseconds(1500);
  //         Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  ////         Serial.print("CURR");
  ////         Serial.println(Curr_Left_Motor_Position);
  //         if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position) >= 300){ // Maybe change 200
  //          servo_LeftMotor.writeMicroseconds(1700);
  //          servo_RightMotor.writeMicroseconds(1700);
  //          Left_turn = false;
  //         }
  //       }
  //       else if (Right_turn == true){
  //         servo_LeftMotor.writeMicroseconds(1500);
  //         servo_RightMotor.writeMicroseconds(1300);
  //         Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
  //         if ((Prev_Right_Motor_Position - Curr_Right_Motor_Position) >= 300){ // Maybe change 200
  //          servo_LeftMotor.writeMicroseconds(1700);
  //          servo_RightMotor.writeMicroseconds(1700);
  //          Right_turn = false;
  //         }
  //       }
  //    }
  //  }
  //
  //  else if (right_hit == true){
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1500);
  //    Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //    if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position)>600){
  //     servo_LeftMotor.writeMicroseconds(1700);
  //     servo_RightMotor.writeMicroseconds(1700);
  //     right_hit = false;
  //    }
  //  }
  //  else if (left_hit == true){
  //    servo_LeftMotor.writeMicroseconds(1500);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
  //    if ((Prev_Right_Motor_Position - Curr_Right_Motor_Position)>600){
  //     servo_LeftMotor.writeMicroseconds(1700);
  //     servo_RightMotor.writeMicroseconds(1700);
  //     left_hit = false;
  //    }
  //  }
  //  else if (digitalRead(A2) == LOW){
  //    right_hit = true;
  //    left_hit = false;
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1500);
  //    Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //  }
  //  else if (digitalRead(left_bumper) == LOW){
  //    right_hit = false;
  //    left_hit = true;
  //    servo_LeftMotor.writeMicroseconds(1500);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    Prev_Right_Motor_Position = encoder_RightMotor.getRawPosition();
  //  }


  //if (getDirection() == 5){
  //  East_check = true;
  //    //Serial.println("TRUE");
  //}
  //else if (getDirection() !=5) {
  //  East_check = false;}
  //mpu6050.update();
  //if (IR_seen == false){
  //  if (state == 0){
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //   if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 2400){
  //      Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //      state = 1;
  //    }
  //  }
  //  else if (state == 1){
  //    if (East_check == false){
  //      servo_LeftMotor.writeMicroseconds(1700);
  //      servo_RightMotor.writeMicroseconds(1300);
  //    }
  //    else if (East_check == true) {
  //      Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //      state = 2;
  //    }
  //  }
  //  else if (state == 2){
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1700);
  //    //Serial.println("HIII");
  //    Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //    if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 1200){
  //      Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //      state = 0;
  //    }
  //  }
  //}
  //mpu6050.update();
  //if (digitalRead(A3) == LOW && IR_seen == false){
  //  front_count++;
  //  if (front_count == 3){
  //    IR_seen = true;
  //    front_IR = true;
  //    //servo_LeftMotor.writeMicroseconds(1500);
  //    //servo_RightMotor.writeMicroseconds(1500);
  //    Serial.print("FRONT");
  //    left_count = 0;
  //    right_count = 0;
  //    front_count = 0;
  //  }
  //}
  //mpu6050.update();
  //if  (digitalRead(A0) == LOW && IR_seen == false){
  //  left_count++;
  //  if (left_count == 3){
  //    IR_seen = true;
  //    left_IR = true;
  //    //servo_LeftMotor.writeMicroseconds(1500);
  //    //servo_RightMotor.writeMicroseconds(1500);
  //    Serial.print("LEFT");
  //    left_count = 0;
  //    right_count = 0;
  //    front_count = 0;
  //  }
  //}
  //mpu6050.update();
  //else if (digitalRead(A1) == LOW && IR_seen == false){
  //  right_count++;
  //  if (right_count == 3){
  //    IR_seen = true;
  //    right_IR = true;
  //    //servo_LeftMotor.writeMicroseconds(1500);
  //    //servo_RightMotor.writeMicroseconds(1500);
  //    Serial.print("RIGHT");
  //    left_count = 0;
  //    right_count = 0;
  //    front_count = 0;
  //  }
  //}
  //mpu6050.update();
  //
  //if (left_IR == true || right_IR == true || front_IR == true){
  //  if (getDirection()==1){
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1300);
  //  }
  //  else {
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //  }
  //}

  //if (front_IR == true){
  //  if (getDirection() == 5){
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    if (getDirection() == 1){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      front_IR = false;
  //      IR_seen_again = true;
  //    }
  //  }
  //  else if (getDirection() == 6){
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1700);
  //    if (getDirection() == 7){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      front_IR = false;
  //      North_check = true;
  //    }
  //  }
  //  else if (getDirection() == 4){
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    if (getDirection() == 3){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      front_IR = false;
  //      South_check = true;
  //    }
  //  }
  //
  //}
  //mpu6050.update();
  //else if (left_IR == true){
  //  if (getDirection() == 3){
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    if (getDirection() == 1){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      left_IR = false;
  //      IR_seen_again = true;
  //    }
  //  }
  //  else if (getDirection() == 4){
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1700);
  //    if (getDirection() == 7){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      left_IR = false;
  //      North_check = true;
  //    }
  //  }
  //  else if (getDirection() == 2){
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1700);
  //    if (getDirection() == 3){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      left_IR = false;
  //      South_check = true;
  //    }
  //  }
  //}
  //mpu6050.update();
  //if (right_IR == true){
  //  if (getDirection() == 7){
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1700);
  //    if (getDirection() == 1){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      right_IR = false;
  //      IR_seen_again = true;
  //    }
  //  }
  //  else if (getDirection() == 8){
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    if (getDirection() == 7){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      right_IR = false;
  //      North_check = true;
  //    }
  //  }
  //  else if (getDirection() == 6){
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //    if (getDirection() == 3){
  //      servo_LeftMotor.writeMicroseconds(1500);
  //      servo_RightMotor.writeMicroseconds(1500);
  //      right_IR = false;
  //      South_check = true;
  //    }
  //  }
  //}
  //mpu6050.update();
  //
  //if (IR_seen_again == true){
  //  if (getDirection() == 1){
  //    servo_LeftMotor.writeMicroseconds(1300);
  //    servo_RightMotor.writeMicroseconds(1300);
  //  }
  //  else {
  //    servo_LeftMotor.writeMicroseconds(1700);
  //    servo_RightMotor.writeMicroseconds(1300);
  //  }
  //}
  //
  //if (North_check == true){
  //  servo_LeftMotor.writeMicroseconds(1700);
  //  servo_RightMotor.writeMicroseconds(1700);
  //  if (digitalRead(A1) == LOW){
  //    IR_seen_again = true;
  //  }
  //}
  //if (South_check == true){
  //  servo_LeftMotor.writeMicroseconds(1700);
  //  servo_RightMotor.writeMicroseconds(1700);
  //  if (digitalRead(A0) == LOW){
  //    IR_seen_again = true;
  //  }
  //}
  //mpu6050.update();
  // for turning on the way back where you need to go forward after turn

  //  if (turn == 1){
  //       if (Left_turn == true){
  //         servo_LeftMotor.writeMicroseconds(1300);
  //         servo_RightMotor.writeMicroseconds(1500);
  //         Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  ////         Serial.print("CURR");
  ////         Serial.println(Curr_Left_Motor_Position);
  //         if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position) >= 300){
  //          Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();// Maybe change 200
  //          servo_LeftMotor.writeMicroseconds(1740);
  //          servo_RightMotor.writeMicroseconds(1700);
  //          Left_turn = false;
  //          done_turn = true;
  //         }
  //       }
  //       else if (Right_turn == true){
  //         servo_LeftMotor.writeMicroseconds(1500);
  //         servo_RightMotor.writeMicroseconds(1300);
  //         Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
  //         if ((Prev_Right_Motor_Position - Curr_Right_Motor_Position) >= 300){
  //          Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();// Maybe change 200
  //          servo_LeftMotor.writeMicroseconds(1740);
  //          servo_RightMotor.writeMicroseconds(1700);
  //          Right_turn = false;
  //          done_turn = true;
  //         }
  //       }
  //       if (done_turn == true){
  //        servo_LeftMotor.writeMicroseconds(1740);
  //        servo_RightMotor.writeMicroseconds(1700);
  //        Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //        if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position) >= 400){
  //          done_turn = false;
  //          turn = 0;
  //        }
  //       }
  //  }

  //  if (digitalRead(A0)==HIGH && East_check == false){
  //    if (ZigZag == 0){
  //      servo_LeftMotor.writeMicroseconds(1700);
  //      servo_RightMotor.writeMicroseconds(1300);
  //      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); // Add Prev update into US check
  //       if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 500){
  //        Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //        ZigZag = 1;// Maybe change 200
  //       }
  //       }
  //    else if (ZigZag == 1){
  //      servo_LeftMotor.writeMicroseconds(1700);
  //      servo_RightMotor.writeMicroseconds(1700);
  //      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); // Add Prev update into US check
  //       if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 500){
  //        Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();// Maybe change 200
  //        ZigZag = 2;
  //    }
  //    }
  //    else if (ZigZag == 2){
  //      servo_LeftMotor.writeMicroseconds(1300);
  //      servo_RightMotor.writeMicroseconds(1700);
  //      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  //       if (Prev_Left_Motor_Position - Curr_Left_Motor_Position >= 500){
  //        Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();// Maybe change 200
  //        ZigZag = 3;
  //        }
  //    }
  //    else if (ZigZag == 3){
  //      servo_LeftMotor.writeMicroseconds(1700);
  //      servo_RightMotor.writeMicroseconds(1700);
  //      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); // Add Prev update into US check
  //       if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 500){
  //        Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();// Maybe change 200
  //        ZigZag = 0;
  //    }
  //    }
  //  }
  //  else {
  //    East_check = true;
  //    servo_LeftMotor.writeMicroseconds(1500);
  //    servo_RightMotor.writeMicroseconds(1500);
  //  }
  //}
}
