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
const int startButton = 1;
const int ultrasonic_Front_IN = 2;
const int ultrasonic_Front_OUT = 2;
const int ultrasonic_Left_IN = 4;
const int ultrasonic_Left_OUT = 4;
const int ultrasonic_Right_IN = 5;
const int ultrasonic_Right_OUT = 5;
const int motor_Right = 6;
const int motor_Left = 7;
const int infared = 8;

//calibration values
unsigned int motor_Speed = 1900;        //run speed
unsigned int motor_Speed_Offset = 200;
unsigned int motor_Left_Speed;
unsigned int motor_Right_Speed;
unsigned int ultrasonic_Front_Distance = 500;    //FL US distance
unsigned int ultrasonic_Left_Distance = 500;          //L US distance
unsigned int ultrasonic_Right_Distance = 500;         //L US distance


//helper variables


//helper variables
boolean ultrasonic_Clear[3] = {true, true, true}; //state for ultrasonic sensors (F,L,R). Stores if value is lower than limit (aka ur gonna crash soon)
unsigned int infaredCheckInterval = 10000;   //how frequent to check for infared
unsigned int infaredCheckTime = 1000;        //how long to spin for when checking infared
unsigned int infaredLastCheckTime = 0;
unsigned int timeCharged = 0;
boolean infaredSeen = false;
char bearing;
unsigned long ul_Echo_Time;
boolean Encoder_Turn = true;


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
  switch (digitalRead(startButton)) {
  case 0:
    break;
  case 1:

    //checks to see if it will bump into things
      ultrasonic_Clear[0] = analogRead(ultrasonic_Front_IN) > ultrasonic_Front_Distance;
      ultrasonic_Clear[1] = analogRead(ultrasonic_Left_IN) > ultrasonic_Left_Distance;
      ultrasonic_Clear[2] = analogRead(ultrasonic_Right_IN) > ultrasonic_Right_Distance;


      
      US_check = check_US();

      //run 1(0-30sec)
      if (millis < 30000) {

        if (US_check == 1) {
          servo_LeftMotor.writeMicroseconds(1300);
          servo_RightMotor.writeMicroseconds(1700);
          Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
          if (Curr_Right_Motor_Position - Prev_Right_Motor_Position >= 200) { // Maybe change 200
            servo_LeftMotor.writeMicroseconds(1700);
            servo_RightMotor.writeMicroseconds(1700);
            US_check = 0;
          }
        }
        else if (US_check == 2) {
          servo_LeftMotor.writeMicroseconds(1700);
          servo_RightMotor.writeMicroseconds(1300);
          Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          if (Curr_Left_Motor_Position - Prev_Left_Motor_Position >= 200) { // Maybe change 200
            servo_LeftMotor.writeMicroseconds(1700);
            servo_RightMotor.writeMicroseconds(1700);
            US_check = 0;
          }
        }
      }
      if ((millis() - timeCharged >= 30000) && (millis() - timeCharged <= 90000) && !infaredSeen) {
        //spin around
        if (!digitalRead(infared)) {
          infaredSeen = true;
          break;
        }
        else if (millis() - infaredLastCheckTime >= infaredCheckInterval) {
          //spin 360 (or N-E/E-N if possible)
          if (!digitalRead(infared)) {
            infaredSeen = true;
            break;
          }

        }
      }

      if (infaredSeen) {
        switch (bearing) {
          case 'E':
            servo_LeftMotor.write(900);
            servo_RightMotor.write(900);
            break;
            //let NE  = A
          case 'A':
            servo_LeftMotor.write(200);
            servo_RightMotor.write(900);
            break;
            //let SE = B
          case 'B':
            servo_LeftMotor.write(900);
            servo_RightMotor.write(200);
            break;
          //backwards hard right
          case 'N':
            servo_LeftMotor.write(900);
            servo_RightMotor.write(200);
            break;
          //backwards hard left
          case 'S':
            servo_LeftMotor.write(200);
            servo_RightMotor.write(900);
        }
        //not really sure how to use servo encoders yet
        delay(100);
        infaredSeen = false;
      }



      //run 1(0-30sec)
      else {
        servo_LeftMotor.write(200);
        servo_RightMotor.write(200);
      }
      break;
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

int check_US() // Function to check ultrasonics
{
  int check = 0;
  int middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT); // Change to actual variable
  if (middle <= 7) {
    int left = Ping(ultrasonic_Left_IN, ultrasonic_Left_OUT); // change to actual variables for pins of US
    int right = Ping(ultrasonic_Right_IN, ultrasonic_Right_OUT);
    if (right < left) {
      servo_LeftMotor.writeMicroseconds(1300);
      servo_RightMotor.writeMicroseconds(1700);
      check = 1;
    }
    else {
      servo_LeftMotor.writeMicroseconds(1700);
      servo_RightMotor.writeMicroseconds(1300);
      check = 2;
    }

  }
  Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
  Prev_Right_Motor_Position = encoder_RightMotor.getRawPosition();
  return check;
}
