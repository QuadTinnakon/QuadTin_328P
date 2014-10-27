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
//PID-------------Rate
float Kp_rateRoll = 1.18;//1.18 1.200
float Kpi_rateRoll = 1.32;//1.02 1.32
float Ki_rateRoll = 2.75;//2.25  0.25  -  0.985
float Kd_rateRoll = 0.035;//0.035 0.025 - 0.045

float Kp_ratePitch = 1.18;//1.18 1.200
float Kpi_ratePitch = 1.32;//1.02 1.32
float Ki_ratePitch = 2.75;//2.25 0.5 - 2.8
float Kd_ratePitch = 0.035;//0.025 - 0.045

float Kp_rateYaw = 1.75;//1.75 - 3.450  350.0
float Kpi_rateYaw = 0.0;
float Ki_rateYaw = 3.65;//3.65  2.95
float Kd_rateYaw = 0.065;//0.045 0.065

//PID--------------Stable
float Kp_levelRoll= 5.2;//6.2 
float Ki_levelRoll= 0.00;//0.0
float Kd_levelRoll= 0.00;//0.0

float Kp_levelPitch= 5.2;//6.2 
float Ki_levelPitch= 0.00;
float Kd_levelPitch= 0.00;

float Kp_levelyaw= 0.0;

//PID--------------Altitude
float Kp_altitude = 165.0;//65.0,75.0
float Ki_altitude = 82.0;//32.5,0.0
float Kd_altitude = 120.0;//25.4 ,0.0

//PID GPS
float Kp_gps = 0.101;
float Kd_gps = 0.35;
float Kp_speed = 1.1;//1.1

//GPS //สตาร์ท
float GPS_LAT_HOME = 13.866986274;//13.867000579  13.867021560  13.867017745
float GPS_LON_HOME = 100.483268737; //100.483291625 100.483261108  100.483276367
//ลงจอด
float waypoint1_LAT = 13.866021560;
float waypoint1_LON = 100.484261108;
//
float waypoint2_LAT = 13.867021560;
float waypoint2_LON = 100.483261108;

float GPS_LAT1 = 13.867000579;
float GPS_LON1 = 100.484261108;
// Automatic take-off and landing 
#define h_control 0.7  //0.6 0.9 meter

#define tar 0.01
//Parameter system Quadrotor
#define m_quad 1.1 //kg
#define L_quad 0.25 //m

//magnetometer calibration constants; use the Calibrate example from
// the Pololu library to find the right values for your board
#define M_X_MIN -670
#define M_Y_MIN -540
#define M_Z_MIN -360
#define M_X_MAX 170
#define M_Y_MAX 290
#define M_Z_MAX 390

//Observer hz
float Altitude_hat=0.0;//Observer hx
float vz_hat=0.0;
float h=0.0;
float seth=0.0;//set control
float uthrottle=0.0;
float accrX_cutg = 0.0;
float accrY_cutg = 0.0;
float accrZ_cutg = 0.0;

//GPS
float GPS_speed1 = 0.0;
float actual_speedX = 0.0;
float actual_speedY = 0.0;
float actual_speedXf = 0.0;
float actual_speedYf = 0.0;
float GPS_LAT1_old = GPS_LAT_HOME;
float GPS_LON1_old = GPS_LON_HOME;
float Control_XEf = 0.0;
float Control_YEf = 0.0;
float Control_XBf = 0.0;
float Control_YBf = 0.0;
float target_LAT = 0.0;
float target_LON = 0.0;
byte currentCommand[23];
byte Read_command = 0;
int GPS_FIX1=0;
float GPS_hz = 0.0;
float GPS_vz = 0.0;
float GPS_ground_course = 0.0;
float GPS_Distance = 0.0;

#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_20HZ 5
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100
#define RAD_TO_DEG 57.295779513082320876798154814105

  //direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
  float DCM00 = 0.0;
  float DCM01 = 1.0;
  float DCM02 = 0.0;
  float DCM10 = -1.0;
  float DCM11 = 0.0;
  float DCM12 = 0.0;
  float DCM20 = 0.0;
  float DCM21 = 0.0;
  float DCM22 = 1.0;
  
// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;
byte armed = 0;
float G_Dt = 0.01; 
float Dt_GPS = 0.2; 
long GPS_loopTimer = 0;

long Dt_sensor = 1000;
long Dt_roop = 10000;
int Status_LED = LOW;
int ESC_calibra = 0;
