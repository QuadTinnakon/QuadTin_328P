/* 
copy write tinnakon kheowree
tinnakon_za@hotmail.com
 */
/* We will set the varibles like so, these can also be tuned by the user */
//standard deviation Gyro = 1.103 SD^2 = 1.2166
//standard deviation angle  = 0.11856 SD^2 = 0.01405
float Q_angleX  =  0.0001;//0.001 
float Q_gyroX   =  0.0093;//0.0008 Q_bias 0.003  
float R_angleX  =  0.35;//0.03  

float Q_angleY  =  0.0001; 
float Q_gyroY   =  0.0093; //Q_bias 0.003 
float R_angleY  =  0.35;//0.03  

float Q_angleZ  =  0.1; 
float Q_gyroZ   =  0.0008;//Q_bias 0.003  
float R_angleZ  =  1.3;//0.03  

float x_angle = 0;
float x_bias = 0;
float PX_00 = 1, PX_01 = 0, PX_10 = 0, PX_11 = 1;	
float dtX, yX, SX;
float KX_0 = 0.1;//0.029
float KX_1 = 0.1;//-0.0027

float y_angle = 0;
float y_bias = 0;
float PY_00 = 1, PY_01 = 0, PY_10 = 0, PY_11 = 1;	
float dtY, yY, SY;
float KY_0 = 0.1;
float KY_1 = 0.1;

float z_angle = 0;
float z_bias = 0;
float PZ_00 = 1, PZ_01 = 0, PZ_10 = 0, PZ_11 = 1;	
float dtZ, yZ, SZ;
float KZ_0 = 0.1;
float KZ_1 = 0.1;

//kalman Calculate X
// KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418 - http://www.x-firm.com/?page_id=145
float kalmanCalculateX(float newAngle, float newRate,float looptime) {
    dtX = looptime; 
    // Discrete Kalman filter time update equations - Time Update ("Predict")
// Update xhat - Project the state ahead
    /* Step 1 */
    x_angle = x_angle + dtX * (newRate - x_bias);
// Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    //P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    PX_00 += dtX * (dtX*PX_11 - PX_01 - PX_10 + Q_angleX);
    //PX_00 +=  - dtX * (PX_10 + PX_01) + Q_angleX * dtX;
    PX_01 -= dtX * PX_11;
    PX_10 -= dtX * PX_11;
    PX_11 += Q_gyroX * dtX;
 // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    yX = newAngle - x_angle;
// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
   // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */    
    SX = PX_00 + R_angleX;
    /* Step 5 */
    KX_0 = PX_00 / SX;
    KX_1 = PX_10 / SX;
    /* Step 6 */
    x_angle +=  KX_0 * yX;
    x_bias  +=  KX_1 * yX;
   // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    PX_00 -= KX_0 * PX_00;
    PX_01 -= KX_0 * PX_01;
    PX_10 -= KX_1 * PX_00;
    PX_11 -= KX_1 * PX_01;
    
    return x_angle;
  }
  
//kalman Calculate Y
// KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418 - http://www.x-firm.com/?page_id=145
float kalmanCalculateY(float newAngle, float newRate,float looptime) {
    dtY = looptime; 
    // Discrete Kalman filter time update equations - Time Update ("Predict")
// Update xhat - Project the state ahead
    /* Step 1 */
    y_angle = y_angle + dtY * (newRate - y_bias);
// Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    //P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    PY_00 += dtY * (dtY*PY_11 - PY_01 - PY_10 + Q_angleY);
    //PX_00 +=  - dtX * (PX_10 + PX_01) + Q_angleX * dtX;
    PY_01 -= dtY * PY_11;
    PY_10 -= dtY * PY_11;
    PY_11 += Q_gyroY * dtY;
 // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    yY = newAngle - y_angle;
// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
   // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */    
    SY = PY_00 + R_angleY;
    /* Step 5 */
    KY_0 = PY_00 / SY;
    KY_1 = PY_10 / SY;
    /* Step 6 */
    y_angle +=  KY_0 * yY;
    y_bias  +=  KY_1 * yY;
   // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    PY_00 -= KY_0 * PY_00;
    PY_01 -= KY_0 * PY_01;
    PY_10 -= KY_1 * PY_00;
    PY_11 -= KY_1 * PY_01;
    
    return y_angle;
  }
