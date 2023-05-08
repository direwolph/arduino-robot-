//project ressources and information can be foud at:
// https://github.com/juliusherve/arduino-robot
/*
 * This files whill store all varible and data to be able to have an easy 
 * modification of our constant if needed.
 */
 
#pragma once
 
//---------------------------------- librairies ----------------------------------
#include <math.h>
#include "Arduino_NineAxesMotion.h"                                      //imu lib
#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>

#define debug_mode                     //use to debug and output data to terminale
//================================================================================
//===================================== Pin ======================================
//================================================================================
/*
               +-----+                   +-----+
            +--| USB |-------------------| USB |--+
            |  +-----+                   +-----+  |
            |           GND/RST2  [ ] [ ]         |
            |         MOSI2/SCK2  [ ] [ ]  SCL[ ] | 
            |            5V/MISO2 [ ] [ ]  SDA[ ] | 
            |                             AREF[ ] |
            |                              GND[ ] |
            | [ ]N/C                        13[ ]~| 
            | [ ]IOREF                      12[ ]~| 
            | [ ]RST                        11[ ]~| 
            | [ ]3V3      +----------+      10[ ]~|
            | [ ]5v       | ARDUINO  |       9[ ]~|
            | [ ]GND      |   DUE    |       8[ ]~|
            | [ ]GND      +----------+            |
            | [ ]Vin                         7[ ]~|
            |                                6[ ]~|-- right motor pwm
            | [ ]A0                          5[ ]~|-- left motor pwm
            | [ ]A1                          4[ ]~| 
            | [ ]A2                     INT5/3[ ]~|-- sensor servo pwm
            | [ ]A3                     INT4/2[ ]~|-- IMU INT(Interrupt) 
  IMU SDA --| [ ]A4                       TX>1[ ]~|  
  IMU SDL --| [ ]A5                       RX<0[ ]~|  
            | [ ]A6                               |   
            | [ ]A7                     TX3/14[ ] |  
            |                           RX3/15[ ] |  
            | [ ]A8                     TX2/16[ ] |           
            | [ ]A9                     RX2/17[ ] |  
            | [ ]A10               TX1/INT3/18[ ] |       
            | [ ]A11               RX1/INT2/19[ ] |  
            | [ ]DAC0          I2C-SDA/INT1/20[ ] |        
            | [ ]DAC1          I2C-SCL/INT0/21[ ] |  
            | [ ]CANRX                            |            
            | [ ]CANTX                            |
            |                RST SCK MISO         |
            |         ICSP   [ ] [ ] [ ]          |  
            |                [ ] [ ] [ ]          |
            |                GND MOSI 5V          |  
            | G 5 5 4 4 4 4 4 3 3 3 3 3 2 2 2 2   | (port tens number)
            | N 2 0 8 6 4 2 0 8 6 4 2 0 8 6 4 2 5 | (port ones number)
            | D 1 9 7 5 3 1 9 7 5 3 1 9 7 5 3 1 V |  
            |     DUE                 ____________/ 
              \_______________________/    
     
21-- IMU AD0 (Adress)
29-- HC-SR04 echo pin
25-- HC-SR04 trig pin
  */
//movement
#define pin_motor_left 5
#define pin_motor_right 6

//front sensor
#define pin_servo_sensor 3
#define pin_sensor_echo 29                   //need to be an interrupt//sensor pin
#define pin_sensor_trig 25                               


//================================================================================
//=========================== General robot parameter ============================
//================================================================================
//back servo param
#define pwm_min 1300
#define pwm_max 1700
#define wheel_diam 12 //cm
#define dist_moteur 160 //mm
//movement PID
#define Kp_speed  1.0
#define Ki_speed  1.0
#define Kd_speed  0.0

#define Kp_angle  1.0
#define Ki_angle  1.0
#define Kd_angle  0.0
//front sensor
//servo
#define pulseMin_front 530                  //min delay for controling front servo
#define pulseMax_front 2500                 //max delay for controling front servo
//HC-SR04
#define sound_speed 343                          //speed of sound in the classroom
#define distance_min_scan 10 //cm
#define distance_max_scan 400 //cm
//delay for robot to initialise
#define initiale_robot_angle_delay_ms 500 
//max robot speed
#define robot_max_speed 0.18 //m/S
#define robot_max_angular_speed 4.504 //rad/s

//================================================================================
//====================== Flag and starting point coordonate ======================
//================================================================================
//coordonate system : 
// ---------------------
// |             (0,0))|
// |             ←x--. |
// |                 |y|
// |                 ↓ |
// |                   |
// |      terrain      |
// |                   |
// |                   |
// |                   |
// ---------------------
//flag coordonate
#define flag_x_coordonate 93 //cm
#define flag_y_coordonate 98 //cm
//robot starting point
#define robot_start_x 298 //cm
#define robot_start_y 365 //cm
//map size
#define x_map_size 250 //cm
#define y_map_size 400 //cm

//================================================================================
//======================= Path finding / Mapping parameter =======================
//================================================================================

#define pathfinding_grid_precision 10 //cm
#define scan_angle_precision  10//angle betwin each mapping


