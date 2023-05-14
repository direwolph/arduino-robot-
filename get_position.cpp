/*
* This part of the code get the robot position using 3 ways,since we are workng in
* open loop with our servo motor we don't have a precise  way of knowing the robot
* positionning, in other hand, IMU are subject to drift other time, this is why we
* calculate it three time : using servo movement approximation, imu and by merging
* calculation together to reduce the error.
 */
 #include "Arduino_NineAxesMotion.h"
#include "configuration.h"                          //all variable are stored here
#include "robot_shared_data.h"
#include "get_position.h"
#include <math.h>
//================================================================================
//================================== Calibration =================================
//================================================================================
/*
 * the imu need to be calibrate in order to get the best result out of it.
 */
void determine_angle_delta_at_start(){
    /*
    * determine the delta betwin the given robot angle by the imu and the real one
    */
    extern robot robot;
    extern NineAxesMotion IMU_sensor;
    delay(initiale_robot_angle_delay_ms);
    IMU_sensor.updateEuler(); //update robot angle
    robot.position.delta_angle = (IMU_sensor.readEulerHeading())* (1000 / 57296);
    //save in radian
    //fast_blink_confirm(2);
};

//================================================================================
//============================== Raw data acquiring ==============================
//================================================================================
void robot_pos_servo() {
   /*
    * We use this function to determine the robot current position on the map 
    * using servo positioning
    */
    extern robot robot;
    extern NineAxesMotion IMU_sensor;
    float delay_pos = robot.position.delay;

    //determine distance
    float distance_parcourue = (robot.movement.velocity*delay_pos)/1000;
    
    //update robot angle                                
    robot.position.servo.angle = fmod((IMU_sensor.readEulerHeading()+ \
    robot.position.delta_angle),360) * (1000 / 57296);
    //save in radian the angle (modulo 360°)

    //save mesured position
    robot.position.servo.X +=distance_parcourue*cos(robot.position.servo.angle);
    robot.position.servo.Y +=distance_parcourue*sin(robot.position.servo.angle);
    //to do : add kalman filter
};
void robot_pos_imu(){
  /*
   * This function uses an IMU to determine the robot's current position. Normally
   * IMUs are subject to drift. The Arduino 9 - axis board uses a BN055 IMU, which
   * includes internal data filtering (Kalman filter / sensor fusion here). We can
   * use the raw data directly here since filtering has been done by the sensor.
   */
  extern robot robot;
  extern NineAxesMotion IMU_sensor;
  float delay_pos = robot.position.delay;
  
  //read the IMU
  const float acc_X = IMU_sensor.readLinearAccelX();    //accel in robot reference
  const float acc_Y = IMU_sensor.readLinearAccelY();
  const float delata_angle_last = fmod((IMU_sensor.readEulerHeading()+ \
  robot.position.delta_angle - robot.position.IMU.angle),360) \
    * (1000 / 57296);      //save how much robot have rotate since last mesurement
  robot.position.IMU.angle = fmod((IMU_sensor.readEulerHeading()+ \
  robot.position.delta_angle),360) * (1000 / 57296);           //save robot angle

  //puting robot accel in frame referential
  const float acc_frame_x = acc_X * cos(delata_angle_last) + \
    acc_Y * sin(delata_angle_last);
  const float acc_frame_y = acc_Y * cos(delata_angle_last) + \
    acc_X * sin(delata_angle_last);

  //double integration to find position
  robot.position.IMU.X += acc_frame_x * delay_pos * pow(delay_pos,2)/2;
  robot.position.IMU.Y += acc_frame_y * delay_pos * pow(delay_pos,2)/2;
};

//================================================================================
//================================= Sensor fusion ================================
//================================================================================
/* We are controlling the robot in a semi-close loop, i.e . , using servo pos work 
 * on open loop and is prone to error.  In other hand, the second sensor, the IMU, 
 * is subject to drift.  Other times accuracy reduces.  One way to obtain a better 
 * accuracy would be to use sensor fusion.  Each sensor is used to sense the same 
 * parameter, fusing them together helps reduce the drift and get better accuracy.
 */
void robot_pos_fusion() {
    //calculate the delta t betwin 2 mesurements to ensure sincroneous timing of 
    //sensor
  extern robot robot;
  extern NineAxesMotion IMU_sensor;
    unsigned long currentTime_pos = millis();
    unsigned long delay_pos = currentTime_pos - \
    robot.position.previous_pos_time;

    robot.position.delay = delay_pos;
    robot.position.previous_pos_time = currentTime_pos; 
    IMU_sensor.updateEuler();  

    //calculate each estimate position
    robot_pos_servo();
    robot_pos_imu();

    //do fusion mean for now need to be modified to implement kalman filetring
    robot.position.fusion.X = (robot.position.IMU.X + robot.position.servo.X) \
     / 2;
    robot.position.fusion.Y = (robot.position.IMU.Y + robot.position.servo.Y) \
     / 2;
    robot.position.fusion.angle = robot.position.IMU.angle; //same sensor 
    
    robot.position.IMU.X = robot.position.fusion.X;
    robot.position.servo.X = robot.position.fusion.X;
    robot.position.IMU.Y = robot.position.fusion.Y;
    robot.position.servo.Y = robot.position.fusion.Y;
    

 //ajouter un filtre de kalman pour l'instant juste moyenne des deux
};


