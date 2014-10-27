/*
project_Quad_rotor v1.1  
1. Automatic  Takeoff 
2. 1 waypoint navigation
3. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
//PID By tinnakon_za@hotmail.com
*/
int u_roll = 0;
int u_pitch = 0;
int u_yaw = 0;
float roll_I_rate = 0.0;
float roll_D_rate = 0.0;
float setpoint_rollold = 0.0;
float setpoint_rate_roll = 0.0;
float error_rollold = 0.0;
float error_rate_rollold = 0.0;

float pitch_I_rate = 0.0;
float pitch_D_rate = 0.0;
float setpoint_pitchold = 0.0;
float setpoint_rate_pitch = 0.0;
float error_pitchold = 0.0;
float error_rate_pitchold = 0.0;

float yaw_I_rate = 0.0;
float yaw_D_rate = 0.0;
float error_rate_yawold = 0.0;

//Automatic take-off and landing
int time_ss = 0;
float  h_counter = 0.03;//0.08
float hz_I = 0.0;
uint8_t takeoff = 0;
uint8_t endAuto = 0;


void Control_PIDRate(){
//I-PD By tinnakon
//if(AUX_1 >= 1300)//Flight Modes 2.Stabilize
// ROLL CONTROL///////////
  float setpoint_roll = ((CH_AIL-1500)*0.12) + Control_YBf;//max +-45 deg  ////+-18  + Control_YBf
  applyDeadband(setpoint_roll, 2.4);//1.2
  setpoint_rate_roll = (0.4*setpoint_rate_roll/(0.4+G_Dt)) + ((setpoint_roll-setpoint_rollold)/(0.4+G_Dt));
  setpoint_rollold = setpoint_roll;
  applyDeadband(setpoint_rate_roll, 7.1);//2.5
  float error_roll = setpoint_roll - x_angle;//ahrs_r*ToDeg
  float error_rate_roll = setpoint_rate_roll - GyroXf*RAD_TO_DEG;
  float errorui_roll = error_rate_roll + (error_roll*Kpi_rateRoll);
  roll_I_rate += errorui_roll*Ki_rateRoll*G_Dt;
  roll_I_rate = constrain(roll_I_rate, -300, 300);//+-300
  roll_D_rate = (tar*roll_D_rate/(tar+G_Dt)) + ((error_rate_roll-error_rate_rollold)/(tar+G_Dt));
  error_rate_rollold = error_rate_roll;
  u_roll = Kp_rateRoll*error_rate_roll + roll_I_rate + Kd_rateRoll*roll_D_rate + Kp_levelRoll*error_roll;
  
////// PITCH CONTROL///////////
  float setpoint_pitch = ((CH_ELE-1500)*-0.12) - Control_XBf;//max +-45 deg  ////+-18 - Control_XBf
  applyDeadband(setpoint_pitch, 2.4);//1.2
  setpoint_rate_pitch = (0.4*setpoint_rate_pitch/(0.4+G_Dt)) + ((setpoint_pitch-setpoint_pitchold)/(0.4+G_Dt));
  setpoint_pitchold = setpoint_pitch;
  applyDeadband(setpoint_rate_pitch, 7.1);//6.5
  float error_pitch = setpoint_pitch - y_angle;//ahrs_p*ToDeg
  float error_rate_pitch = setpoint_rate_pitch - GyroYf*RAD_TO_DEG;
  float errorui_pitch = error_rate_pitch + (error_pitch*Kpi_ratePitch);
  pitch_I_rate += errorui_pitch*Ki_ratePitch*G_Dt;
  pitch_I_rate = constrain(pitch_I_rate, -300, 300);//+-300
  pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt)) + ((error_rate_pitch-error_rate_pitchold)/(tar+G_Dt));
  error_rate_pitchold = error_rate_pitch;
  u_pitch = Kp_ratePitch*error_rate_pitch + pitch_I_rate + Kd_ratePitch*pitch_D_rate + Kp_levelPitch*error_pitch;
  
////// YAW CONTROL///////////
  float setpoint_rate_yaw = (CH_RUD-1500)*0.35;
  applyDeadband(setpoint_rate_yaw, 7.1);//6.5
  float error_yaw = 0.0 - z_angle;
  float error_rate_yaw = setpoint_rate_yaw - GyroZf*RAD_TO_DEG;
  float errorui_yaw = error_rate_yaw + (error_yaw*Kpi_rateYaw);
  yaw_I_rate += errorui_yaw*Ki_rateYaw*G_Dt;
  yaw_I_rate = constrain(yaw_I_rate, -100, 100);//+-100
  yaw_D_rate = (tar*yaw_D_rate/(tar+G_Dt)) + ((error_rate_yaw-error_rate_yawold)/(tar+G_Dt));
  error_rate_yawold = error_rate_yaw;
  u_yaw = Kp_rateYaw*error_rate_yaw + yaw_I_rate + Kd_rateYaw*yaw_D_rate + Kp_levelyaw*error_yaw;
  //u_yaw = constrain(u_yaw, -300, 300);
  
  //Altitude control
    float err_hz = h_counter - Altitude_hat;
    hz_I = hz_I + (Ki_altitude*err_hz*G_Dt);//1.9 - 2.9  18.5  22.5
    hz_I = constrain(hz_I, -200, 200);//50
    float uthrottle2 = (err_hz*Kp_altitude) + hz_I - (vz_hat*Kd_altitude);//- (accrZ_cutg*0.129);//- (accrZ_cutg*0.0129) + 9.81; //state feedback control Altitude m = 1290 g
    float cos_roll = cos(ahrs_r);
    float cos_pitch = cos(ahrs_p);
    if(cos_roll < 0.707)
    {
      cos_roll = 0.707;
    }
     if(cos_pitch < 0.707)
    {
      cos_pitch = 0.707;
    }
    uthrottle = uthrottle2*m_quad/(cos_roll*cos_pitch);// = u*m*d/b*(cosr*cosp)     ,m*d/b = 122.33  (cos_roll*cos_pitch)
  
}

void Automatictakeland(){
      //take-off
      if(h_counter == 0.0 && AUX_2 < 1750)
      {
        time_ss = 0;
        target_LAT = GPS_LAT_HOME;
        target_LON = GPS_LON_HOME;
      }
       if(h_counter < h_control && AUX_2 > 1750 && takeoff == 1 && CH_THR > MINCHECK && armed == 1)
      {
        endAuto = 1;
        h_counter = h_counter + 0.033;//0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
      }
       //landing
       if(takeoff == 0)
      {
        h_counter = h_counter - 0.033; //ramp input hz  landing
        if(h_counter <= 0.03)
        {
         h_counter = 0.0;
         endAuto = 0;
        }
      }
}

 void GPS_distance_m_bearing(float lat1, float lon1, float lat2, float lon2, float alt){
  float a, tc1, R, c, d, dLat, dLon;
  lon1=lon1/RAD_TO_DEG;
  lat1=lat1/RAD_TO_DEG;
  lon2=lon2/RAD_TO_DEG;
  lat2=lat2/RAD_TO_DEG;
  R=6371000.0;    //m raio da terra 6371km
  a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
  GPS_ground_course = a*RAD_TO_DEG;
  //if (yaw<0) yaw=360+yaw;
//calculo da distancia entre modelo e home
  dLat = (lat2-lat1);
  dLon = (lon2-lon1);
  a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  c = 2* asin(sqrt(a));  
  d = 6371000 * c;
  //alt=alt-Alt_Home;
  //pitch=atan(alt/d);
  //pitch=pitch*360/(2*PI);
  GPS_Distance = sqrt(alt*alt+d*d);
}

 void Chack_Command(){
      while(Serial.available())
          {
           Read_command = Serial.read();
           
          if(Read_command == 48)//0 Set HOME Quadrotor
          {
           GPS_LAT_HOME = GPS_LAT1;
           GPS_LON_HOME = GPS_LON1;
           //target_LAT = GPS_LAT1;
           //target_LON = GPS_LON1;
          }
           if(Read_command == 49)//1 Set Waypoint 1 Quadrotor
          {
           waypoint1_LAT = GPS_LAT1;
           waypoint1_LON = GPS_LON1;
          }
           if(Read_command == 50)//2 Set Waypoint 2 Quadrotor
          {
           waypoint2_LAT = GPS_LAT1;
           waypoint2_LON = GPS_LON1;
          }
          }//end  while roop
          
          Read_command = 0;
 }
 
  void GPS_calc_positionhold(){
          GPS_distance_m_bearing(GPS_LAT1, GPS_LON1, GPS_LAT_HOME, GPS_LON_HOME, Altitude_hat);
          float error_LAT = (target_LAT - GPS_LAT1)*1000000.0; // X Error
          float error_LON = (target_LON - GPS_LON1)*1000000.0;// Y Error
          error_LAT = constrain(error_LAT,-100,100);//50 = +-5 m
          error_LON = constrain(error_LON,-100,100);
          float target_speedLAT = error_LAT*Kp_speed;//P Control Velocity GPS
          float target_speedLON = error_LON*Kp_speed;//P Control Velocity GPS
          target_speedLAT = constrain(target_speedLAT,-100,100);//+-100 cm/s = 1m/s
          target_speedLON = constrain(target_speedLON,-100,100);
          
          //Cal Velocity GPS
//Calculator Pythagorean theorem , enjoin, actual_speedYf > GPS_speed1 ,enjoin, actual_speedXf > GPS_speed1
          if(actual_speedXf >= GPS_speed1)//X
          {
            actual_speedXf = GPS_speed1;
          }
          if(actual_speedXf < 0.0)//-X
          {
             if(abs(actual_speedXf) >= GPS_speed1)
            {
             actual_speedXf = GPS_speed1*-1.0;
            }
          }//end X
          if(actual_speedYf >= GPS_speed1)//Y
          {
            actual_speedYf = GPS_speed1;
          }
          if(actual_speedYf < 0.0)//-Y
          {
             if(abs(actual_speedYf) >= GPS_speed1)
            {
             actual_speedYf = GPS_speed1*-1.0;
            }
          }//end Y
          //Calculator Pythagorean theorem c^2 = a^2 + b^2
          float actual_speedXff = sqrt(sq(GPS_speed1) - sq(actual_speedYf));//enjoin actual_speedYf > GPS_speed1
          float actual_speedYff = sqrt(sq(GPS_speed1) - sq(actual_speedXf));//enjoin actual_speedXf > GPS_speed1
          if(actual_speedYf < 0.0)//-Y
          {
            actual_speedYff = actual_speedYff*-1.0;
          }
          if(actual_speedXf < 0.0)//-X
          {
            actual_speedXff = actual_speedXff*-1.0;
          }
          
          float error_rate_LAT = target_speedLAT - actual_speedXff;
          float error_rate_LON = target_speedLON - actual_speedYff;
          error_rate_LAT = constrain(error_rate_LAT,-200,200);//+-200 cm/s
          error_rate_LON = constrain(error_rate_LON,-200,200);
          Control_XEf = error_LAT*Kp_gps + error_rate_LAT*Kd_gps;//PD Control angle
          Control_YEf = error_LON*Kp_gps + error_rate_LON*Kd_gps;
          Control_XBf = (DCM00*Control_XEf) + (DCM01*Control_YEf);//Control Body Frame
          Control_YBf = (DCM10*Control_XEf) + (DCM11*Control_YEf);//Control Body Frame
          Control_XBf = constrain(Control_XBf, -20, 20);//+-20 deg
          Control_YBf = constrain(Control_YBf, -20, 20);//+-20 deg
          GPS_LAT1_old = GPS_LAT1;
          GPS_LON1_old = GPS_LON1;
  }
