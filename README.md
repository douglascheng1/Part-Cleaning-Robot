# 2202-Project
Group Project for 2202

char direction = 'f';
unsigned long ul_Echo_Time;
unsigned long currentMillis = millis();
unsigned long previousMillis = millis();

void setup(){}

void loop()
{
  check_US();
  currentMillis = millis();
  if (direction == 'f') {
  
    if (currentMillis - previousMillis >= 60 000){
      previousMillis = currentMillis;
      direction = 'b';
     }
  }
  
  
  else if (direction == 'b') {
  }
}


// measure distance to target using ultrasonic sensor
int Ping(int ci_Ultrasonic_Ping, int ci_Ultrasonic_Data)
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

void check_US() // Function to check ultrasonics
{
  int middle = Ping(Middle_US_IN, Middle_US_OUT); // Change to actual variable
  if (middle <= 7){
    int left = Ping(Left_US_IN, Left_US_OUT); // change to actual variables for pins of US
    int right = Ping(Right_US_IN, Right_US_OUT);
    if (right < left){
      servo_LeftMotor.writeMicroseconds(1300);
      servo_RightMotor.writeMicroseconds(1900);
    }
    else {
      servo_LeftMotor.writeMicroseconds(1900);
      servo_RightMotor.writeMicroseconds(1300);
    }
    delayMicroseconds(500);
    servo_LeftMotor.writeMicroseconds(1900);
    servo_RightMotor.writeMicroseconds(1900);
    
  }
  else {
    continue; // keep current speeds
  }
}
