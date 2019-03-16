#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//pins (changeable)
const int ultrasonic_Front_Left = 1;
const int ultrasonic_Front_Middle = 2;
const int ultrasonic_Front_Right = 3;
const int ultrasonic_Left = 4;
const int ultrasonic_Right = 5;
const int motor_Right = 6;
const int motor_Left = 7;
const int infared_Front = 8;
const int infared_Back = 9;

//calibration values
unsigned int motor_Speed = 1900;        //run speed
unsigned int motor_Speed_Offset = 200;
unsigned int motor_Left_Speed;
unsigned int motor_Right_Speed;
unsigned int ultrasonic_Front_Left_Distance = 500;    //FL US distance
unsigned int ultrasonic_Front_Middle_Distance = 500;  //FM US distance
unsigned int ultrasonic_Front_Right_Distance = 500;   //FL US distance
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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // set up drive motors
  pinMode(motor_Right, OUTPUT);
  servo_RightMotor.attach(motor_Right);
  pinMode(motor_Left, OUTPUT);
  servo_LeftMotor.attach(motor_Left);

}

void loop() {

  //checks to see if it will bump into things
  ultrasonic_Clear[0] = analogRead(ultrasonic_Front_Left)>ultrasonic_Front_Left_Distance;
  ultrasonic_Clear[1] = analogRead(ultrasonic_Front_Middle)>ultrasonic_Front_Middle_Distance;
  ultrasonic_Clear[2] = analogRead(ultrasonic_Front_Right)>ultrasonic_Front_Right_Distance;
  ultrasonic_Clear[3] = analogRead(ultrasonic_Left)>ultrasonic_Left_Distance;
  ultrasonic_Clear[4] = analogRead(ultrasonic_Right)>ultrasonic_Right_Distance;
 
  //run 1(0-30sec)
  if (millis<30000){
    
    //forwards motion (all true)
    if (ultrasonic_Clear[0] && ultrasonic_Clear[1]&& ultrasonic_Clear[2] && ultrasonic_Clear[3] && ultrasonic_Clear[4]){
      motor_Left_Speed = motor_Speed;
      motor_Right_Speed = motor_Speed;
    }
    //slight left
    if (ultrasonic_Clear[0] && ultrasonic_Clear[1]&& ultrasonic_Clear[2] && !ultrasonic_Clear[3] && ultrasonic_Clear[4]){
      motor_Left_Speed = motor_Speed - motor_Speed_Offset;
      motor_Right_Speed = motor_Speed = motor_Speed_Offset;
    }
    //slight right
    if (ultrasonic_Clear[0] && ultrasonic_Clear[1]&& ultrasonic_Clear[2] && ultrasonic_Clear[3] && !ultrasonic_Clear[4]){
      motor_Left_Speed = motor_Speed + motor_Speed_Offset;
      motor_Right_Speed = motor_Speed - motor_Speed_Offset;
    }
    //turn right if stuck
    else{
      rotate90();
    }
  }

  //coming back code (30-90sec)
  else if ((((millis<90000) && (millis>30000))||(millis<180000)&&(millis>120000)) && infaredNumber==0){
    runFlag = 1;
  }
  //2nd run (opposite direction, same code) (90-120s)
  else if (millis<120000){
        //forwards motion (all true)
    if (ultrasonic_Clear[0] && ultrasonic_Clear[1]&& ultrasonic_Clear[2] && ultrasonic_Clear[3] && ultrasonic_Clear[4]){
      motor_Left_Speed = motor_Speed;
      motor_Right_Speed = motor_Speed;
    }
    //slight left
    if (ultrasonic_Clear[0] && ultrasonic_Clear[1]&& ultrasonic_Clear[2] && !ultrasonic_Clear[3] && ultrasonic_Clear[4]){
      motor_Left_Speed = motor_Speed - motor_Speed_Offset;
      motor_Right_Speed = motor_Speed = motor_Speed_Offset;
    }
    //slight right
    if (ultrasonic_Clear[0] && ultrasonic_Clear[1]&& ultrasonic_Clear[2] && ultrasonic_Clear[3] && !ultrasonic_Clear[4]){
      motor_Left_Speed = motor_Speed + motor_Speed_Offset;
      motor_Right_Speed = motor_Speed - motor_Speed_Offset;
    }
    //turn left
    else{
      rotate90();
    }
  }
  //stop
  else{
    servo_LeftMotor.write(200);
    servo_RightMotor.write(200);
  }

}
