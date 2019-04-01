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
const int ultrasonic_Front_IN = 6;
const int ultrasonic_Front_OUT = 5;
const int ultrasonic_Left_IN = 8;
const int ultrasonic_Left_OUT = 7;
const int ultrasonic_Right_IN = 4;
const int ultrasonic_Right_OUT = 3;
const int motor_Right = 11;
const int motor_Left = 10;
const int infrared_Left = 8;
const int infrared_Right = 9;
const int infrared_Front = 11;

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
bool isStuck;
bool prevSeen = false;
bool currSeen = false;
unsigned int prevIRCheck = 0;
bool infraredSeen_Last = false;
bool backwards = false;
int startpoint = 0;
int midpoint = 0;
bool centered = false;
int bearing;
bool mode9 = false;

//dummy function for gyroscope
int getDegrees(){
  return 1;
}

void check_US() // Function to check ultrasonics
{
  int middle = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT); // Change to actual variable
  Serial.println(middle);
  if (middle <= 15){
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
  else {
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
  //attachInterrupt(0, bumpers, RISING);

  delay(3000);
  servo_LeftMotor.writeMicroseconds(1750);
  servo_RightMotor.writeMicroseconds(1700);
}


//insert dump code here (it is okay to have stopping code)
void dump(){
  
}

void loop() {

  
  switch(mode){
    //gyro calibration (set current bearing to west)
    case 0:
    break;

    //normal run code (until 30s)
    case 1:
    //rotate towards S after 30s, then change modes
    Curr_Left_Motor_Position = Prev_Left_Motor_Position;
    if (millis() - timeCharged>=30000){

      if(Curr_Left_Motor_Position - Prev_Left_Motor_Position <= 600){
        Prev_Left_Motor_Position = Curr_Left_Motor_Position;
        Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();    //gets motor position
      }
        else{
      mode++;
    }
    }
    break;

    //coming back code (assume that there is one infrared in back, right and left)
    //note: backwards driving towards east: more important that it makes it in than pick up shitsecond round
    case 2:
    infraredSeen_Right = (analogRead(infrared_Right)<infrared_Max);   //resets to see iff infrareds are seen
    infraredSeen_Left = (analogRead(infrared_Left)<infrared_Max);
    infraredSeen_Front = (analogRead(infrared_Front)<infrared_Max);        
    distance_Front = Ping(ultrasonic_Front_IN, ultrasonic_Front_OUT);
    if(1){
      
    }
    else if(infraredSeen_Front){
      bearing  = getDegrees();
      mode9 = true;
      mode = 9;
    }
    //rotate Westwards
    else if (infraredSeen_Right||infraredSeen_Left){
      mode = 3;
    }



    break;

    //reverse code
    case 3:
    //if angle greater than +-3deg, turn
    if (getDegrees()<=3||getDegrees()>=358){
          servo_LeftMotor.writeMicroseconds(1700);
          servo_RightMotor.writeMicroseconds(1300);      
    }
    else{
      mode++;
    }
    break;


    
    //back in code (go back set distance + constant)
    case 4:
    Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();    //gets motor position
    //resets charge time and move on
    if(Curr_Left_Motor_Position - Prev_Left_Motor_Position >=distance+2){
      servo_LeftMotor.writeMicroseconds(1500);    //stops motors
      servo_RightMotor.writeMicroseconds(1500);   //stops motors
      timeCharged=millis();
      mode++;
    }
    //go backwards
    else{
          servo_LeftMotor.writeMicroseconds(1300);
          servo_RightMotor.writeMicroseconds(1300);
    }
    break;

    //dumping code, then reset
    case 5:
    dump();
    //reset after dumped
    if(firstRun){
      firstRun = false;
      mode=0;
      mode9 = false;
    }
    else{
      Serial.println("FINISHED");
    }
    break;

    case 8:
    if (infraredSeen_Right||infraredSeen_Left){
      distance = abs(tan(bearing))*abs(encoder_LeftMotor.getRawPosition() - Prev_Left_Motor_Position);
      mode = 3;
    }
    else{
        servo_LeftMotor.writeMicroseconds(1300);
        servo_RightMotor.writeMicroseconds(1300); 
    }
    
    break;

    
    case 9:
    //right
    if (bearing>270){
      if(getDegrees()>=175&&getDegrees()<=185){
          servo_LeftMotor.writeMicroseconds(1700);
          servo_RightMotor.writeMicroseconds(1300);  
      }
      else{
        mode=8;
      }
    }
    //left
    else{
        if(getDegrees()>=175&&getDegrees()<=185){
          servo_LeftMotor.writeMicroseconds(1300);
          servo_RightMotor.writeMicroseconds(1700);  
      }
      else{
        mode=8;
      }
    }
    Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition();

    break;
  }
  
  
  /*
 if (millis - timeCharged<30000){
   check_US();
   delay(100);
  //run 1(0-30sec)
  if (digitalRead(A2)==HIGH && hit == false){
    if (1==1){
      
       if (Left_turn == true){
         servo_LeftMotor.writeMicroseconds(1300);
         servo_RightMotor.writeMicroseconds(1500);
         Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
         //Serial.println(Curr_Right_Motor_Position - Prev_Right_Motor_Position);
         if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position) >= 300){ // Maybe change 200
          servo_LeftMotor.writeMicroseconds(1750);
          servo_RightMotor.writeMicroseconds(1700);
          Left_turn = false;
         }
       }
       else if (Right_turn == true){
         servo_LeftMotor.writeMicroseconds(1500);
         servo_RightMotor.writeMicroseconds(1300);
         Curr_Right_Motor_Position = encoder_RightMotor.getRawPosition();
         if ((Prev_Right_Motor_Position - Curr_Right_Motor_Position) >= 300){ // Maybe change 200
          servo_LeftMotor.writeMicroseconds(1750);
          servo_RightMotor.writeMicroseconds(1700);
          Right_turn = false;
         }
       }
    }
  }
  else if (digitalRead(A2)==HIGH && hit == true){
  servo_LeftMotor.writeMicroseconds(1300);
  servo_RightMotor.writeMicroseconds(1500);
  Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); 
  if ((Prev_Left_Motor_Position - Curr_Left_Motor_Position)>600){
   servo_LeftMotor.writeMicroseconds(1700);
   servo_RightMotor.writeMicroseconds(1700);
   hit = false;
  }
}
  else if (digitalRead(A2) == LOW){
    hit = true;
    servo_LeftMotor.writeMicroseconds(1300);
    servo_RightMotor.writeMicroseconds(1500);
    Prev_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); 
    //Serial.println(Prev_Left_Motor_Position);
  }
 //Serial.print("CURR: ");
 //Serial.println(Curr_Left_Motor_Position);
 
 }

 
//reset to east before heading back
if(millis() - timeCharged>30000&&returnState==0){
  if (abs(getDegrees() - direction_East)<=degree_Tolerance){
    returnState=1;
    servo_LeftMotor.writeMicroseconds(1600);
    servo_RightMotor.writeMicroseconds(1600);
  }
  //rotates
  else{
    servo_LeftMotor.writeMicroseconds(1400);
    servo_RightMotor.writeMicroseconds(1600);
  }
}

if(millis() - timeCharged>30000&&returnState==1&&!infraredSeen_Right&&!infraredSeen_Left){
     if (analogRead(infrared_Right)<=infrared_Max){
         infraredSeen_Right = true;
         returnState=2;
     }
     else if(analogRead(infrared_Left)<=infrared_Max){
      infraredSeen_Left = true;
      returnState=2;
     }
     else{
      check_US2();
      delay(100);
      Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); 
      if (abs(Prev_Left_Motor_Position - Curr_Left_Motor_Position)>600){
      servo_LeftMotor.writeMicroseconds(1700);
      servo_RightMotor.writeMicroseconds(1700);
      }
     }     
}
//either left or right is seen (not correct atm) fix 600 so that it is exactly 90deg
if(millis() - timeCharged>30000&&returnState==2){
  
  Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); 
  Prev_Left_Motor_Position = Curr_Left_Motor_Position;
  //rotate counterclockwise (90 deg)
  if (infraredSeen_Right){
    
  while(abs(Prev_Left_Motor_Position - Curr_Left_Motor_Position)>600){
      servo_LeftMotor.writeMicroseconds(1400);
      servo_RightMotor.writeMicroseconds(1600);
     
     Curr_Left_Motor_Position = Prev_Left_Motor_Position;
  }
  }
  //rotate clockwise (90deg)
  else{
      while(abs(Prev_Left_Motor_Position - Curr_Left_Motor_Position)>600){

      servo_LeftMotor.writeMicroseconds(1600);
      servo_RightMotor.writeMicroseconds(1400);      
      Curr_Left_Motor_Position = Prev_Left_Motor_Position;
  }
  }

      if (abs(Prev_Left_Motor_Position - Curr_Left_Motor_Position)>600){
      servo_LeftMotor.writeMicroseconds(1700);
      servo_RightMotor.writeMicroseconds(1700);
      }
  returnState==3;
}

//back in
else if (returnState==3){
  Prev_Left_Motor_Position = Curr_Left_Motor_Position;
  Curr_Left_Motor_Position = encoder_LeftMotor.getRawPosition(); 
      servo_LeftMotor.writeMicroseconds(1300);
      servo_RightMotor.writeMicroseconds(1300);
  if (Curr_Left_Motor_Position==Prev_Left_Motor_Position){
    returnState=4;
  }
}

//dump code
else if (returnState==4){
//dumping code insert here
  returnState=0;
  timeCharged=millis();

  
}

 else{
  
 }

*/
}
