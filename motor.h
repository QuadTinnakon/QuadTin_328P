/*
project_Quad_rotor v1.1  
1. Automatic  Takeoff 
2. 1 waypoint navigation
3. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
//PID By tinnakon_za@hotmail.com
*///motor
int MOTOR_Front_PIN = 3;
int MOTOR_Right_PIN = 10;
int MOTOR_Left_PIN = 11;
int MOTOR_Back_PIN = 9;

int motor_Front = 1000;
int motor_Right = 1000;
int motor_Left = 1000;
int motor_Back = 1000;

void motor_command_all() 
{
  for (int j = 0 ; j <= 50 ; j++)
  {
  analogWrite(MOTOR_Front_PIN, 125);
  analogWrite(MOTOR_Right_PIN, 125);
  analogWrite(MOTOR_Left_PIN, 125);
  analogWrite(MOTOR_Back_PIN, 125);
  delay(20);
}

}
//motor command
void motor_initialize() 
{
  pinMode(MOTOR_Front_PIN,OUTPUT);  
  pinMode(MOTOR_Right_PIN,OUTPUT); 
  pinMode(MOTOR_Left_PIN,OUTPUT); 
  pinMode(MOTOR_Back_PIN,OUTPUT); 
  motor_command_all();
}

void motor_command() 
{
  analogWrite(MOTOR_Front_PIN, motor_Front/8);
  analogWrite(MOTOR_Right_PIN, motor_Right/8);
  analogWrite(MOTOR_Left_PIN, motor_Left/8);
  analogWrite(MOTOR_Back_PIN, motor_Back/8);
}
  /********************************************************************/
  /****           ESCs calibration                                 ****/
  /********************************************************************/
void ESC_calibration () {
    for (int i = 0; i < 5; i++)
  {
  computeRC();
  if(CH_THR > MAXCHECK)
  {
   ESC_calibra = 1; 
  }
  else
  {
    ESC_calibra = 0;
  }
  delay(20);
  }
  while(ESC_calibra == 1){
   computeRC();
   motor_Front = (CH_THR - 500)*1.5;
   motor_Right = (CH_THR - 500)*1.5;
   motor_Left = (CH_THR - 500)*1.5;
   motor_Back = (CH_THR - 500)*1.5;
   
   motor_Front = constrain(motor_Front, MINCOMMAND, MAXCOMMAND);
   motor_Right = constrain(motor_Right, MINCOMMAND, MAXCOMMAND);
   motor_Left = constrain(motor_Left, MINCOMMAND, MAXCOMMAND);
   motor_Back = constrain(motor_Back, MINCOMMAND, MAXCOMMAND);
   
   analogWrite(MOTOR_Front_PIN, motor_Front/8);
   analogWrite(MOTOR_Right_PIN, motor_Right/8);
   analogWrite(MOTOR_Left_PIN, motor_Left/8);
   analogWrite(MOTOR_Back_PIN, motor_Back/8);
   
   Serial.print(motor_Front);Serial.print("\t");
   Serial.print(motor_Right);Serial.print("\t");    
   Serial.print(motor_Left);Serial.print("\t");     
   Serial.print(motor_Back);Serial.println("\t");
   
     if(Status_LED == LOW)
     Status_LED = HIGH;
     else
     Status_LED = LOW;
     digitalWrite(13, Status_LED);
     digitalWrite(30, Status_LED);
     digitalWrite(31, Status_LED);
   delay(100);
  }
}

