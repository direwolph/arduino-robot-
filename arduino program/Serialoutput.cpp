#include "Serialoutput.h"
#include "configuration.h"                          //all variable are stored here
#include "robot_shared_data.h"
void serial_output(){
  extern robot robot;
  extern flag flag;
  extern grid grid;
  //position
  Serial.print(robot.position.servo.X);
  Serial.print(";");
  Serial.print(robot.position.servo.Y);
  Serial.print(";");
  Serial.print(robot.position.servo.angle);
  Serial.print(";");
  Serial.print(robot.position.IMU.X);
  Serial.print(";");
  Serial.print(robot.position.IMU.Y);
  Serial.print(";");
  Serial.print(robot.position.IMU.angle);
  Serial.print(";");
  Serial.print(robot.position.fusion.X);
  Serial.print(";");
  Serial.print(robot.position.fusion.Y);
  Serial.print(";");
  Serial.print(robot.position.fusion.angle);
  Serial.print(";");
  Serial.print(robot.position.distance_from_flag);
  Serial.print(";");
  Serial.print(robot.position.delta_angle);
  Serial.print(";");
  //movement
  Serial.print(robot.movement.velocity);
  Serial.print(";");
  Serial.print(robot.movement.angular_velocity);
  Serial.print(";");
  Serial.print(robot.movement.targeted_angle);
  Serial.print(";");
  //sensor
  Serial.print(robot.sensor.last_mesurement_dist);
  Serial.print(";");
  Serial.print(robot.sensor.last_mesurement_angle);
  Serial.print(";");
  //path
  Serial.print(robot.pathtofollow.number_of_point);
  Serial.print(";");
  Serial.print(robot.pathtofollow.X[0]);
  for (int i = 1; i < robot.pathtofollow.number_of_point; i++) {
    Serial.print(":");
    Serial.print(robot.pathtofollow.X[i]);
  }
  Serial.print(";");
  Serial.print(robot.pathtofollow.Y[0]);
  for (int i = 1; i < robot.pathtofollow.number_of_point; i++) {
    Serial.print(":");
    Serial.print(robot.pathtofollow.Y[i]);
  }

  Serial.print(";");
  Serial.print(flag.x);
  Serial.print(";");
  Serial.print(flag.y);
  Serial.print(";");
  Serial.print(grid.matrix[0][0]);
  for (int i = 0; i < int(x_map_size/pathfinding_grid_precision) + 2; i++) {
    for (int j = 0; j < int(y_map_size/pathfinding_grid_precision) + 2; i++){
      if (i != 0 && j != 0) {
        Serial.print(":");
        Serial.print(grid.matrix[i][j]);
      } 
    }
  }
  Serial.println(";");
};
  