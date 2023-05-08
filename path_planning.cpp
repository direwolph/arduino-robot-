#include "configuration.h"                          //all variable are stored here
#include "robot_shared_data.h"
#include "mapping.h"
#include "path_planning.h"
#include <Servo.h>
#include <math.h>
#include <vector>
#include <unordered_set>
//#include <iostream>


void path_to_flag() {
  extern robot robot;
  extern flag flag;
  robot.position.dx = robot.position.fusion.X - flag.x;
  robot.position.dy = robot.position.fusion.Y - flag.y;
  robot.position.distance_from_flag = sqrt(pow( robot.position.dx,2)+ pow(robot.position.dy,2));
  robot.movement.targeted_angle = atan2(robot.position.dx, robot.position.dy);
 };

 int is_flag_clear(){
   extern robot robot;
   map_scan(1);                          //take a mesurement in the flag direction
   path_to_flag();
   if (robot.sensor.last_mesurement_dist >= robot.position.distance_from_flag){
     return true;
   }else {
     return false;
   }
 };

//================================================================================
//============================= Obstacle pathfinding =============================
//================================================================================

//search is stored in map to ensure less ram usage we need to clean after
void search_erase(){
  extern grid grid;
  int grid_size_x = int(x_map_size/pathfinding_grid_precision) +2 ; 
  //+1 for each wall
  int grid_size_y = int(y_map_size/pathfinding_grid_precision) + 2;
  for (int i = 1; i < grid_size_x - 1 ; i++) {
    for (int j = 1; j < grid_size_y -1 ; j++) {
      if (grid.matrix[i][j] != 0 && grid.matrix[i][j] != 100 && \ 
      grid.matrix[i][j] != -1){
        grid.matrix[i][j] = 0;
      }
    }
  }
};

//we need to erase the old pathfinding first
void old_pathfinding_erase(){
  extern robot robot;
  for (int i = 0; i < 100 ; i++) {
    robot.pathtofollow.X[i] = 0;
    robot.pathtofollow.Y[i] = 0;
  };
  robot.pathtofollow.point_number = 0;
};


//check distance betwin two cell
int manhattanDistance(int cell_a_x, int cell_a_y, int cell_b_x, int cell_b_y) {
  return abs(cell_a_x - cell_b_x) + abs(cell_a_y - cell_b_y);
};



//save ther path inside robot 
void best_path(int cell_robot_x, int cell_robot_y) {
  extern robot robot;
  extern grid grid;
  extern flag flag;
  static int flag_x_cell = int((flag.x/100) * pathfinding_grid_precision);
  static int flag_y_cell = int((flag.y/100) * pathfinding_grid_precision);
  static int best_score = -1000;
  static int best_x;
  static int best_y;
  static int x = cell_robot_x;
  static int y = cell_robot_y;
  static int n = 0; //number of point the robot follow
  while (x != flag_x_cell && y != flag_y_cell) {
    for(int i = -1; i <= 1; i++) {
      for(int j = -1; j <= 1; j++) {
        static int x_new = x + i;
        static int y_new = y + j;
        if (grid.matrix[x_new][y_new] > best_score &&  \
        grid.matrix[x_new][y_new] < -1 ) {
          best_score = grid.matrix[x_new][y_new];
          robot.pathtofollow.X[n] = x_new;
          robot.pathtofollow.Y[n] = y_new;
          n++;
          x = x_new;
          y = y_new;
          grid.matrix[x_new][y_new] = 0;
        }
      }
    }
    if (best_score == -1000){
      robot.pathtofollow.X[n] = flag_x_cell;
      robot.pathtofollow.Y[n] = flag_y_cell;
      search_erase();
      break;
    }
    best_score = -1000;
  }
  search_erase();
};
//We use the A* algorithm
void aStar(int cell_robot_x, int cell_robot_y) {
  //first stape calculate for each cell the robot know is free the distance to
  //flag we save this value as a negativ to know it's our calculus

  extern robot robot;
  extern grid grid;
  extern flag flag;
  old_pathfinding_erase();
  search_erase();
  int grid_size_x = int(x_map_size/pathfinding_grid_precision) +2 ;
  int grid_size_y = int(y_map_size/pathfinding_grid_precision) + 2;
  static int flag_x_cell = int((flag.x/100) * pathfinding_grid_precision);
  static int flag_y_cell = int((flag.y/100) * pathfinding_grid_precision);
  for (int i = 1; i < grid_size_x - 1 ; i++) {
    for (int j = 1; j < grid_size_y -1 ; j++) {
      if (grid.matrix[i][j] == 0) {
        int h = manhattanDistance(i, j, flag_x_cell, flag_y_cell);
        int g = manhattanDistance(i, j, cell_robot_x, cell_robot_y);
        grid.matrix[i][j] = -10 - h - g;
        //-10 is here to ensure that we don't ed up with a value of -1 
      }
    }
  }
  best_path(cell_robot_x, cell_robot_y);
};