
//path fiding lib
//movement lib
//scan lib
//doit modifier fonction get_position pour marcher avec la classe : je devrais surement suppr les classes

//---------------------- trouble shooting --------------------
//led code
// . = short blink 
// _ = long blink

/* Init confirmation :
 *  . : IMU loaded succesfully
 *  .. : delta angle set succesfully
 */

//------------- librairie ----------------------
#include "robot_shared_data.h"                        //contain all robot variable
robot robot;                                             //init the robot object
flag flag;                                                //init the flag object
grid grid;
NineAxesMotion IMU_sensor;                                     //init of the IMU
Servo motorL;                                                       //left motor
Servo motorR;                                                      //right motor
Servo sensorServo;                                               //servo capteur
#include <math.h>                                             //math operation lib
#include "Arduino_NineAxesMotion.h"                                      //imu lib
#include <Wire.h>                                //serial comunication I2C for IMU
#include <Servo.h>
#include "configuration.h"                      //contain all the robot parameters
#include <PID_v1.h>
#include "mapping.h"
#include "movement.h"
#include <vector>
#include <unordered_set>
//#include <iostream>
#include "path_planning.h"
#include "get_position.h"

#ifdef debug_mode
  #include "Serialoutput.h"
#endif

double angleinput, angleoutput, setangle,distinput, distoutput, setdist;
PID PID_angle(&angleinput, &angleoutput, &setangle, Kp_angle, Ki_angle, \
   Kd_angle, DIRECT);
PID PID_speed(&distinput, &distoutput, &setdist, Kp_speed, Ki_speed, Kd_speed, \
   DIRECT);

//-------------- some init needed before opening the struct -------------


void fast_blink_confirm(int number_of_blink){
  /*
   * function blinking onboard led to confirm states
   */
  for(int i=0; i<=number_of_blink; i++){
    digitalWrite(13, HIGH); // turn the LED on
    delay(20);
    digitalWrite(13, LOW); // turn the LED off
    delay(20);
  }
  delay(100);
}



void setup() {
  
  //IMU_sensor.initSensor(); //IMU init
  IMU_sensor.setOperationMode(OPERATION_MODE_NDOF);
  //setting of the operation mode of imu (how the coordonate came)
  IMU_sensor.setUpdateMode(MANUAL); 
  //uptdate the IMU manualy to get all information from the same data stream
  PID_init();
  pinMode(13, OUTPUT); // use pin 13 led as troubleshooting
  Wire1.begin(); //Initialize I2C communication for IMU
  //connecting the servo
  //motorL.attach(pin_motor_left, pwm_min, pwm_max);
  //motorR.attach(pin_motor_right, pwm_min, pwm_max);
  //-------- debugging-----------------
  #ifdef debug_mode  
  Serial.begin(115200); //serial communication for debugging
  #endif
  delay(3000);
  fast_blink_confirm(1);
  determine_angle_delta_at_start();
  fast_blink_confirm(2);
  delay(5000);
  fast_blink_confirm(3);
  map_scan(1);
  delay(500);
  static int robot_x_cell = int((robot_start_x/100) * pathfinding_grid_precision);
  static int robot_y_cell = int((robot_start_y/100) * pathfinding_grid_precision);
  aStar(robot_x_cell, robot_y_cell);
}


void loop() {
  robot_pos_fusion();
  map_scan(0);
  if (robot.sensor.not_a_wall){
    aStar(robot.position.fusion.X, robot.position.fusion.Y);
  }
  PID_control();
  #ifdef debug_mode
  serial_output();
  #endif
}
