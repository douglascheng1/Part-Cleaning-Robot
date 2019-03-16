 switch(digitalRead(startButton){
    case 0:
    break;
    case 1:
    
      //checks to see if it will bump into things
      ultrasonic_Clear[0] = analogRead(ultrasonic_Front)>ultrasonic_Front_Distance;
      ultrasonic_Clear[1] = analogRead(ultrasonic_Left)>ultrasonic_Left_Distance;
      ultrasonic_Clear[2] = analogRead(ultrasonic_Right)>ultrasonic_Right_Distance;
      
      if ((millis() - timeCharged >= 30000) && (millis() - timeCharged <= 90000) && !infaredSeen){
        //spin around
        if (!digitalRead(infared)){
          infaredSeen = true;
          break;
        }
        else if (millis() - infaredLastCheckTime >=infaredCheckInterval){
          //spin 360 (or N-E/E-N if possible)
          if (!digitalRead(infared)){
            infaredSeen = true;
            break;
          }
          
        }
      }

      if (infaredSeen){
        switch(bearing){
          case "E":
          servo_LeftMotor.write(900);
          servo_RightMotor.write(900);
          break;
          case "NE":          
          servo_LeftMotor.write(200);
          servo_RightMotor.write(900);
          break;
          case "SE":
          servo_LeftMotor.write(900);
          servo_RightMotor.write(200);
          break;
          //backwards hard right
          case "N":
          servo_LeftMotor.write(900);
          servo_RightMotor.write(200);
          break;
          //backwards hard left
          case "S":
          servo_LeftMotor.write(200);
          servo_RightMotor.write(900);
        }
        //not really sure how to use servo encoders yet
        delay(100);
        infaredSeen = false;
      }

  
 
      //run 1(0-30sec)
      else{
        servo_LeftMotor.write(200);
        servo_RightMotor.write(200);
      }
    break;
  }
