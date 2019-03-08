# 2202-Project
Group Project for 2202

char direction = 'f';


void setup(){}

void loop()
{
  unsigned long currentMillis = millis();
  if (direction == 'f') {
   check_US();
  
  
  if (currentMillis - previousMillis >= 60 000){
    direction = 'b';
   }
  }
  
  
  else if (direction == 'b') {
  }
}


// measure distance to target using ultrasonic sensor
int Ping(int ci_Ultrasonic_Ping)
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  return ul_Echo_Time/58;
#endif
}

void check_US() // Function to check ultrasonics
{
  int left = Ping(Left_US);
  int right = Ping(Right_US);
  int middle = Ping(Middle_US);
  if (left <= 7){
    servo_LeftMotor.writeMicroseconds(200);
    servo_LeftMotor.writeMicroseconds(1900);
  }
  else if (right <= 7){
    servo_LeftMotor.writeMicroseconds(1900);
    servo_LeftMotor.writeMicroseconds(200);
  }
  else if (middle <= 7){
    servo_LeftMotor.writeMicroseconds(1300);
    servo_LeftMotor.writeMicroseconds(1700);
  }
}
