/*
 * This part of the code control the robot mouvement using imput data like targete
 * speed and angular velocity
 */
#include "movement.h"
#include "configuration.h"                          //all variable are stored here
#include "robot_shared_data.h"
#include <Servo.h>
#include <PID_v1.h>

void PID_init(){
  // Define the reference trajectory
  extern PID PID_angle;
  extern PID PID_speed;
  extern robot robot;
  int x_ref[100];
  int y_ref[100];
  int n_waypoints = robot.pathtofollow.number_of_point;
  for (int i=0;i<=n_waypoints;i++){
    x_ref[i] = robot.pathtofollow.X[i];
    y_ref[i] = robot.pathtofollow.Y[i];
  }
  // Initialize the PID controller
  
  
  // Initialize the PID controller
  PID_angle.SetMode(AUTOMATIC);
  PID_speed.SetMode(AUTOMATIC);
  PID_angle.SetOutputLimits(-180, 180);
  PID_speed.SetOutputLimits(-180, 180);
};
void PID_control() {
  extern PID PID_angle;
  extern PID PID_speed;
  extern robot robot;
  int i = robot.pathtofollow.point_number;
  if (i > robot.pathtofollow.number_of_point){
    return;
  };
  float delta_x = robot.pathtofollow.X[i] - robot.position.fusion.X;
  float delta_y = robot.pathtofollow.Y[i] - robot.position.fusion.Y;
  if (delta_x < 10 && delta_y < 10){
    i++;
    float delta_x = robot.pathtofollow.X[i] - robot.position.fusion.X;
    float delta_y = robot.pathtofollow.Y[i] - robot.position.fusion.Y;
  };
  float distance_error = sqrt(delta_x * delta_x + delta_y * delta_y);
  float angle_error = atan2(delta_y, delta_x) - robot.position.fusion.angle;
  extern PID PID_angle;
  extern PID PID_speed;
  extern double angleinput;
  extern double angleoutput;
  extern double setangle;
  extern double distinput;
  extern double distoutput;
  extern double setdist;
  angleinput = 0;
  setangle = angle_error;
  distinput = 0;
  setdist = distance_error;
  PID_speed.Compute();
  PID_angle.Compute();
  double desired_distance_change = distoutput;
  double desired_angle_change = angleoutput;
  double motor_speed1 = desired_distance_change - desired_angle_change;
  double motor_speed2 = desired_distance_change + desired_angle_change;
  if (motor_speed1 > 180){
    int diff = motor_speed1 - 180;
    motor_speed1 = 180;
    motor_speed2 -= diff;
  }
  if (motor_speed1 < -180){
    int diff = motor_speed1 + 180;
    motor_speed1 = -180;
    motor_speed2 -= diff;
  }
  if (motor_speed2 > 180){
    int diff = motor_speed2 - 180;
    motor_speed2 = 180;
    motor_speed1 -= diff;
  }
  if (motor_speed2 < -180){
    int diff = motor_speed2 + 180;
    motor_speed2 = -180;
    motor_speed1 -= diff;
  }
  extern Servo motorL;
  extern Servo motorR;
  motorL.write(motor_speed1);
  motorR.write(motor_speed2);
  robot.movement.velocity = map((motor_speed1 + motor_speed2)/2,-180,180, \
  -robot_max_speed,robot_max_speed);
  robot.movement.angular_velocity = map(-(motor_speed1 - motor_speed2)/2,-180, \
  180, -robot_max_angular_speed,robot_max_angular_speed);
};