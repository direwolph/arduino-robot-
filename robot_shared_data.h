/*
 * Since we choose to work with multiple file, data need to be save and process in
 * a specific way. each files need to access the data and edit it.This is why this
 * h files is used to store all data. They are saved in form of a struct.
 */

#pragma once
#include "configuration.h"
#ifndef init
#define init

//================================================================================
//===================================== Robot ====================================
//================================================================================
struct robot {

  struct position { 
                                          //position data of robot
    struct servo {             //position data of robot calculatade with open loop
        float X = robot_start_x;                           //calculated X position
        float Y = robot_start_y;                           //calculated Y position
        float angle = 0;                               //calculated angle position
    }; servo servo;

    struct IMU {                     //position data of robot calculatade with IMU
        float X = robot_start_x;                           //calculated X position
        float Y = robot_start_y;                           //calculated Y position
        float angle = 0;                               //calculated angle position
    }; IMU IMU;

    struct fusion {            //position data of robot calculatade with open loop
        float X = robot_start_x;                           //calculated X position
        float Y = robot_start_y;                           //calculated Y position
        float angle = 0;                               //calculated angle position
    }; fusion fusion;

    int previous_pos_time = 0;         //last time where mesurement have been made
    int delay = 0;                             //delay betwin n and n-1 mesurement
    float traveled_distance = 0;                         //distance from flag (cm)
    float distance_from_flag = 0;                         //distance from the flag 
    float dy = 0, dx = 0;                          //distance from flag in x and y
    float delta_angle = 0;    //difference betwin mesured angle and with the table
  }; position position;

  struct movement{
    float velocity = 0;                                 //speed of the robot (m/s)
    float angular_velocity = 0;                            //angular speed (rad/s)
    float targeted_angle = 0;          //desired angle for the robot to head (rad)
  }; movement movement;

  struct sensor {
    double last_mesurement_dist = 0;
    double last_mesurement_x = 0;
    double last_mesurement_y = 0;
    int  last_mesurement_angle = 0;             //angle betwin sensor and x axis
    int last_servo_pose = 0;
    int last_scan = 1;
    bool not_a_wall = 0;
  }; sensor sensor;

  struct pathtofollow {
    int number_of_point = 0;
    int X[100];   //save each sell coordonate to follow
    int Y[100];
    int point_number = 0;
    struct next_point{
      int X;
      int Y;
      int distance;
      int angle;
    };next_point next_point;
  }; pathtofollow pathtofollow;
};

//================================================================================
//===================================== Flag =====================================
//================================================================================
struct flag {
  const float x = flag_x_coordonate;
  const float y = flag_y_coordonate;
};

//================================================================================
//===================================== grid =====================================
//================================================================================
struct grid {
  int matrix[int(x_map_size/pathfinding_grid_precision) + 2] \
  [int(y_map_size/pathfinding_grid_precision) + 2];
};


#endif