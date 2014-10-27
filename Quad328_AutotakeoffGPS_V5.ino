/*
project_Quad_rotor v1.1  
1. Automatic  Takeoff 
2. 1 waypoint navigation
3. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
date: 28-03-2557(2014)  V.4
date: 2-04-2557(2014)  V.5

support:  Board MWC_328P
• Atmega328P
• ITG3205 Triple Axis Gyro
• BMA180 Accelerometer
• BMP085 Barometer
• HMC5883L Magnetometer
• HMC5883L Magnetometer

Quad=> +
              F
             D3 >>          
              /
L  D11 << ----    -----  D10 <<   R      
              /
             D9 >>
              B
              
---------motor---------
Front  => D3
Right => D10
Left  => D11
Back => D9

----------rx-----------           
Throttle  => D2
Aileron   => D4
Elevator  => D5
Ruder     => D6
Aux       => D7              
*/
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "sensor.h"
#include "ahrs.h"
#include "multi_rx.h"
#include "motor.h"
//#include "bmp085.h"
#include "Kalmantin.h"
#include "Control_PID.h"

void setup()
{
  Serial.begin(38400);//38400
  configureReceiver();
  motor_initialize();
  ESC_calibration();
  Wire.begin();
  pinMode(13, OUTPUT);pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=),(31=B=STABLEPIN),(30 C) //GPS FIG LEDPIN 
  digitalWrite(13, HIGH);
  delay(20);
  AccelerometerInit();
  delay(20);
  GyroInit();
  delay(20);  
  MagInt();
  delay(20); 
      for (int i = 0; i < 10; i++) 
    {
     AccelerometerRead();
     GyroRead();
     MagRead();
     delay(20);
    }
    digitalWrite(13, LOW);
  sensor_Calibrate();
  ahrs_initialize();
  
  sensorPreviousTime = micros();
  previousTime = micros();
  GPS_loopTimer = millis();
}


void loop()
{
  while(Serial.available() >= 22)
   {
     //digitalWrite(31, HIGH);
     byte byteGPS=Serial.read(); //data 18 byte
     if(byteGPS == 0x24)////The header byte is represented by value 36 decimal or 0x24 hex.
     {
       for(int i=0; i<21; i++)    // Build Command From Serial Buffer //data 17 byte
       {
        currentCommand[i] = Serial.read();       
       }
union ua1_tag 
{
    byte b[4];
    float fval;
} ua1;
ua1.b[0] = currentCommand[0];
ua1.b[1] = currentCommand[1];
ua1.b[2] = currentCommand[2];
ua1.b[3] = currentCommand[3];
GPS_LAT1 = ua1.fval;
ua1.b[0] = currentCommand[4];
ua1.b[1] = currentCommand[5];
ua1.b[2] = currentCommand[6];
ua1.b[3] = currentCommand[7];
GPS_LON1 = ua1.fval;
ua1.b[0] = currentCommand[8];
ua1.b[1] = currentCommand[9];
ua1.b[2] = currentCommand[10];
ua1.b[3] = currentCommand[11];
GPS_speed1 = ua1.fval - 15.5;//cm/s
ua1.b[0] = currentCommand[12];
ua1.b[1] = currentCommand[13];
ua1.b[2] = currentCommand[14];
ua1.b[3] = currentCommand[15];
GPS_hz = ua1.fval;
ua1.b[0] = currentCommand[16];
ua1.b[1] = currentCommand[17];
ua1.b[2] = currentCommand[18];
ua1.b[3] = currentCommand[19];
GPS_vz = ua1.fval;
GPS_FIX1 = currentCommand[20];
  //Diff speed
  Dt_GPS = (float)(millis() - GPS_loopTimer)/ 1000.0;
  GPS_loopTimer = millis();
  if(Dt_GPS <= 0 || Dt_GPS > 10.0)
  {
    Dt_GPS = 0.2;
  }
  actual_speedX = (GPS_LAT1 - GPS_LAT1_old)*1000000000.0*Dt_GPS;//cm/s  10000000.0
  actual_speedY = (GPS_LON1 - GPS_LON1_old)*1000000000.0*Dt_GPS;//cm/s
     }
   }
///////////Roop///////////////////////////////////////////////   
  Dt_sensor = micros() - sensorPreviousTime;
  if(Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
      // Read data (not faster then every 1 ms) 
    if(Dt_sensor >= 1000 && sensorSamples < 3)// = 2760 us
    {  
        sensorPreviousTime = micros();
        sensor_readSum();
    }
// 100 Hz task loop (10 ms)  , 5000 = 0.02626 ms
   Dt_roop = micros() - previousTime;
   if(Dt_roop <= 0)
   {
    Dt_roop = 10001; 
   }   
    if (Dt_roop >= 10000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop / 1000000.0;
      frameCounter++;
      //Read sensor
      sensor_Get(); //Moving average  filter 3 
      GyroXf = (GyroX3 + GyroX2 + GyroX)/3.0;
      GyroYf = (GyroY3 + GyroY2 + GyroY)/3.0;
      GyroZf = (GyroZ3 + GyroZ2 + GyroZ)/3.0;
      GyroX3 = GyroX2;
      GyroX2 = GyroX;
      GyroY3 = GyroY2;
      GyroY2 = GyroY;
      GyroZ3 = GyroZ2;
      GyroZ2 = GyroZ;
      AccXf = AccXf + (AccX - AccXf)*18.4*G_Dt;//15.4  //Low pass filter
      AccYf = AccYf + (AccY - AccYf)*18.4*G_Dt;//15.4
      AccZf = AccZf + (AccZ - AccZf)*18.4*G_Dt;//15.4
      
      //ahrs_updateIMU(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, G_Dt);
      ahrs_updateMARG(GyroXf, GyroYf, GyroZf, -AccXf, -AccYf, AccZf, c_magnetom_x, -c_magnetom_y, c_magnetom_z, G_Dt);
      x_angle = ahrs_r*RAD_TO_DEG;
      y_angle = ahrs_p*RAD_TO_DEG;
      
      //x_angle = kalmanCalculateX(ahrs_r*RAD_TO_DEG, GyroX*RAD_TO_DEG, G_Dt);
      //y_angle = kalmanCalculateY(ahrs_p*RAD_TO_DEG, GyroY*RAD_TO_DEG, G_Dt);
      
      //Observer hz kalman , GPS_hz , GPS_vz
      float temp_vz = accrZ_cutg + 16.72*(GPS_vz - vz_hat);//15.5
      vz_hat = vz_hat + temp_vz*G_Dt;
      vz_hat = constrain(vz_hat, -1.0, 1.0);//+-1 m/s
      float temp_hz = vz_hat + 12.54*(GPS_hz - Altitude_hat);//12.54 4.5
      Altitude_hat = Altitude_hat + temp_hz*G_Dt;
      Altitude_hat = constrain(Altitude_hat, 0.0, 1.5);//1.5 m By Ultrasonic
      
      //GPS Speed Low Pass Filter
     actual_speedXf = actual_speedXf + (actual_speedX - actual_speedXf)*1.17*G_Dt;//8.4  //cm/s
     actual_speedYf = actual_speedYf + (actual_speedY - actual_speedYf)*1.17*G_Dt;//1.17
     actual_speedXf = constrain(actual_speedXf, -200, 200);//+-200 cm/s
     actual_speedYf = constrain(actual_speedYf, -200, 200);//+-200 cm/s
//PID Control///////////
      Control_PIDRate();
//end PID Control///////////         
//////Out motor///////////
      motor_Front = CH_THR + uthrottle + u_pitch - u_yaw;
      motor_Right = CH_THR + uthrottle - u_roll + u_yaw;
      motor_Left = CH_THR + uthrottle + u_roll + u_yaw;
      motor_Back = CH_THR + uthrottle - u_pitch - u_yaw;
      
         if (CH_THR < MINCHECK) 
        {
          roll_I_rate = 0;
          pitch_I_rate = 0;
          yaw_I_rate = 0;
          motor_Front = 1000;
          motor_Right = 1000;
          motor_Left = 1000;
          motor_Back = 1000;
        }
        
        if(armed == 1)
        {
         motor_Front = constrain(motor_Front, MINTHROTTLE, MAXCOMMAND);
         motor_Right = constrain(motor_Right, MINTHROTTLE, MAXCOMMAND);
         motor_Left = constrain(motor_Left, MINTHROTTLE, MAXCOMMAND);
         motor_Back = constrain(motor_Back, MINTHROTTLE, MAXCOMMAND);
         motor_command();
        }
        else
        {
          motor_Front = 1000;
          motor_Right = 1000;
          motor_Left = 1000;
          motor_Back = 1000;
          motor_command();
        }
////////end Out motor//////
      
        // 50 Hz tak (20 ms)
        if (frameCounter % TASK_50HZ == 0) 
        {
         computeRC();
 //////ARM and DISARM your helicopter///////////////
       if (CH_THR < MINCHECK) 
        {
            if (CH_RUD > MAXCHECK && armed == 0) 
            {
                armed = 1;
                digitalWrite(30, HIGH);
            }
            if (CH_RUD < MINCHECK && armed == 1) 
            {
                armed = 0;
                digitalWrite(30, LOW);
            }
        }
  //////end  ARM and DISARM your helicopter///////////////       
   //Altitude control and 1 waypoint navigation
  if(AUX_2 > 1750 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff = 1;
     if(Altitude_hat >= h_control && endAuto == 1)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
    }
    if(time_ss > 10 && endAuto == 1)//Landing and position hold mode
    {
      takeoff = 0;
      target_LAT = GPS_LAT1;
      target_LON = GPS_LON1;
    }
  }
  else
  {
    takeoff = 0;
    hz_I = 0.0;
    uthrottle = 0.0;
    Control_XBf = 0.0;
    Control_YBf = 0.0;
  }
}//end roop 50 Hz 
        
        // 10 Hz task (100 ms)
            if (frameCounter % TASK_10HZ == 0) 
        {
            Automatictakeland();
            MagRead();
        }//end roop 10 Hz
          if (frameCounter % TASK_5HZ == 0) //TASK_5HZ
        {
         GPS_calc_positionhold();
        }
        if (frameCounter % TASK_10HZ == 0) //TASK_5HZ  TASK_10HZ
        {
            
            
            //Serial.print(CH_THR);Serial.print("\t");
            //Serial.print(CH_AIL);Serial.print("\t");  
            //Serial.print(CH_ELE);Serial.print("\t");
            //Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t"); 
            
            //Serial.print(MagX);Serial.print("\t");
            //Serial.print(MagY);Serial.print("\t");
            //Serial.print(MagZ);Serial.print("\t");  

            //Serial.print(GPS_FIX1);Serial.print("\t");
            //Serial3.print(GPS_LAT1,9);Serial3.print("\t"); 
            //Serial3.print(GPS_LON1,9);Serial3.print("\t");
            //Serial3.print(GPS_speed1);Serial3.print("\t");//cm/s
            //Serial3.print(actual_speedXf);Serial3.print("\t");
            //Serial3.print(actual_speedYf);Serial3.print("\t");
            //Serial3.print(GPS_Distance);Serial3.print("\t");
            //Serial3.print(GPS_ground_course);Serial3.print("\t");
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(Control_XEf);Serial.print("\t");
            //Serial.print(Control_YEf);Serial.print("\t");
            //Serial.print(Control_XBf);Serial.print("\t");
            //Serial.print(Control_YBf);Serial.print("\t");
            
            Serial.print(DCM00);Serial.print("\t");
            Serial.print(DCM01);Serial.print("\t");
            Serial.print(DCM02);Serial.print("\t");
            Serial.print(DCM10);Serial.print("\t");
            Serial.print(DCM11);Serial.print("\t");
            Serial.print(DCM12);Serial.print("\t");
            Serial.print(DCM20);Serial.print("\t");
            Serial.print(DCM21);Serial.print("\t");
            Serial.print(DCM22);Serial.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(GPS_hz);Serial.print("\t"); 
            //Serial.print(Altitude_hat);Serial.print("\t");
            //Serial.print(GPS_vz*10);Serial.print("\t");
            //Serial.print(vz_hat*10);Serial.print("\t");
            
            //Serial.print(motor_Front);Serial.print("\t");
            
            //Serial.print(set_pitch_rate);Serial.print("\t");
            
            //Serial.print(AccX,3);Serial.print("\t");
            //Serial.print(AccXf,3);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccZ,3);Serial.print("\t");
            //Serial.print(AccZf,3);Serial.print("\t");       
            //Serial.print(accrX_cutg);Serial.print("\t");
            //Serial.print(accrY_cutg);Serial.print("\t");
            //Serial.print(accrZ_cutg);Serial.print("\t");
            
            //Serial.print(GyroX*RAD_TO_DEG);Serial.print("\t");
            Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroY*RAD_TO_DEG,3);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
   
            //Serial.print(ahrs_r*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(ahrs_p*RAD_TO_DEG);Serial.print("\t");  
            Serial.print(ahrs_y*RAD_TO_DEG);Serial.print("\t");  
            
            //Serial.print(x_angle,3);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            
            //Serial.print(sensorSamples2);Serial.print("\t");
            //Serial.print(Dt_sensor);Serial.print("\t");
            Serial.print(G_Dt*1000);Serial.print("\t");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
            //Serial3.print("\n");
            ///////////////
            Chack_Command();//Control pid
            ///////////////
        }//end roop 5 Hz 
       // Reset frameCounter back to 0 after reaching 100 (1s)
        if (frameCounter >= TASK_1HZ) {
            frameCounter = 0;
            time_ss++;
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);
            if(GPS_FIX1 == 1)
            {
              digitalWrite(31, !Status_LED);
            }
            else
            {
              digitalWrite(31, LOW);
            }
        }
        //previousTime = micros();
        //sensorPreviousTime = micros();
    }//end roop 100 HZ 
}
