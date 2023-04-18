//------------- librairie ----------------------
#include <math.h>

//---------------- general parameter ------------------
#define wheel_diam 12 //cm
//flag coordonate
#define flag_x_coordonate 12 //cm
#define flag_y_coordonate 12 //cm
//robot starting point
#define robot_start_x 12 //cm
#define robot_start_y 12 //cm
#define dist_moteur 160 //mm
#define initiale_robot_angle 3.14/2
//max robot speed
#define robot_max_speed 0.18 //m/S
#define robot_max_angular_speed 4.504 //rad/s

//---------------- robot data ------------------
struct robot {
  float x;
  float y;
  float distance;
  float angle;
};

struct flag {
  float x;
  float y;
};

struct PathResult {
  float distance;
  float angle;
};


robot super_robot = { robot_start_x, robot_start_y, 0, initiale_robot_angle };
flag destination = { flag_x_coordonate, flag_y_coordonate };


PathResult pathfinding_no_obstacle(float robot_x, float robot_y, float robot_angle, float flag_x, float flag_y) {
  float distance = 0, dy = 0, dx = 0, angle = 0;
  dx = robot_x-flag_x;
  dy = robot_y-flag_y;
  distance = sqrt(pow(dx,2)+ pow(dy,2));
  angle = atan2(dy, dx) - robot_angle;
  PathResult result = { distance, angle };
  return result;
  //retourne distance, angle a tourner
}

void setup() {
  
}

void loop() {
  PathResult result = pathfinding_no_obstacle(super_robot.x, super_robot.y, super_robot.angle, destination.x, destination.y);
  super_robot.distance = result.distance;
  super_robot.angle = result.angle;
}


int motor_output_from_speed( float desired_speed, float desired_angular_speed, int dir_speed, int dir_rotation) {
  int motorL_outptput = 0;
  int motor_speed = 0;
  int motor_angular_velocity = 0;
  int motorL_output = 0;
  int motorR_output = 0;
  motor_speed = dir_speed * int((desired_speed/robot_max_speed)*180);
  motor_angular_velocity = dir_rotation * int((desired_angular_speed/robot_max_angular_speed)*180);
  motorL_output = motor_speed + motor_angular_velocity;
  motorR_output = -motor_speed + motor_angular_velocity;
  if ((motorL_output >= -180 && motorL_output <= 180) && (motorR_output >= -180 && motorR_output <= 180)) {
    return 0;
    //motorL.write(motorL_outptput);
    //motorR.write(motorR_outptput);
  }
  else {
    return 0; //error to fast
  }
}

float robot_pos() {
  return 0;
}
