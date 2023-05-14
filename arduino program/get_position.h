/*
* This part of the code get the robot position using 3 ways,since we are workng in
* open loop with our servo motor we don't have a precise  way of knowing the robot
* positionning, in other hand, IMU are subject to drift other time, this is why we
* calculate it three time : using servo movement approximation, imu and by merging
* calculation together to reduce the error.
 */
#pragma once
#include <Arduino.h>
#include "robot_shared_data.h"
void determine_angle_delta_at_start();
void robot_pos_servo();
void robot_pos_imu();
void robot_pos_fusion();