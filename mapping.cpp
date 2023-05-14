//en gros je calcul le path direct je tourne le servo et le robot vers la destination;
//si je vois rien, le servo fixe la cible sinon bouge et map la zone, filtre de kalman
//si obstacle appartient table rien et case traversée égale a zero sinon si obstacle focus dessus
//proba et calcul de nouveau path 

//================================================================================
//================================== Initiation ==================================
//================================================================================
#include "configuration.h"                          //all variable are stored here
#include "robot_shared_data.h"
#include "mapping.h"
#include "path_planning.h"
#include <Servo.h>
#include <math.h>
volatile bool measurementStarted = false;
volatile bool measurementComplete = false;
volatile unsigned long measurementStartTime = 0;
volatile unsigned long measurementEndTime = 0;
volatile float dist = 0;

void matrix_init(){
  extern grid grid;
    int grid_size_x = int(x_map_size/pathfinding_grid_precision) +2 ; //+1 for each wall
    int grid_size_y = int(y_map_size/pathfinding_grid_precision) + 2;

    //create a matrix containing all map cell
    // Initialize the matrix with -1 exept wall
    for (int i = 0; i < grid_size_x ; i++) {
      if (i == 0 || i == grid_size_x){
        for (int j = 0; j < grid_size_y ; j++) {
          grid.matrix[i][j] = 100;
        };
      };
      for (int j = 0; j < grid_size_y ; j++) {
        if (j == 0 || j == grid_size_x){
          grid.matrix[i][j] = 100;
        };
        grid.matrix[i][j] = -1;
      };
    };
  };
void echoInterruptHandler();
void map_save();

void sonar_init(){
  extern Servo sensorServo;
  pinMode(pin_sensor_echo, OUTPUT);
  pinMode(pin_sensor_trig, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_sensor_echo), echoInterruptHandler, \
   CHANGE);
  sensorServo.attach(pin_servo_sensor, pulseMin_front, pulseMax_front);
};

//================================================================================
//================================== Filtering ===================================
//================================================================================
double kalman(double U){
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P*H/(H*P*H+R);
  U_hat += K*(U-H*U_hat);
  P = (1-K*H)*P+Q;
  return U_hat;
};

//================================================================================
//============================ Distance calculation ==============================
//================================================================================

float mesure_distance(bool use_kalman){
  //use sensor to mesure distance betwin robot and possibler obstacle
  if (!measurementStarted) {
    // Start a new measurement
    digitalWrite(pin_sensor_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(pin_sensor_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_sensor_trig, LOW);
    measurementStarted = true;
    measurementStartTime = micros();
  }
  if (measurementComplete) {
    unsigned long duration = measurementEndTime - measurementStartTime;
    static float distance = duration * (0.01 * sound_speed) / 2;
    // Reset flags for next measurement
    measurementStarted = false;
    measurementComplete = false;
    if (use_kalman){
      return kalman(distance);
    }else{
      return distance;
    }
  }
};

void echoInterruptHandler() {
  if (digitalRead(pin_sensor_echo) == HIGH) {
    // Rising edge, start of measurement
    measurementStartTime = micros();
  } else {
    // Falling edge, end of measurement
    measurementEndTime = micros();
    measurementComplete = true;
    dist = mesure_distance(1);
    map_save();
  };
};

//================================================================================
//================================== Map saving ==================================
//================================================================================
void map_scan(bool focus_on_flag){
  extern robot robot;
  extern Servo sensorServo;
  if (focus_on_flag){
    //the servo will always be turn to the flag ex if there is no obstacle
    //set the servo angle

    float angle_robot = robot.position.fusion.angle; 
    path_to_flag();
    float angle_robot_flag = robot.movement.targeted_angle;      //absolute angle 
    int servo_angle = (int)degrees(angle_robot_flag-angle_robot);

    if (servo_angle <= -90 || servo_angle >= 90){
      servo_angle = 0;
    }
    sensorServo.write(servo_angle);
    robot.sensor.last_mesurement_angle = radians(servo_angle) + angle_robot;
    robot.sensor.last_servo_pose = servo_angle;
    mesure_distance(1);  //we don't had delay servo is not supposed to move a lot

  } else{
    //scan the environement
    float angle_robot = robot.position.fusion.angle;
    int servo_angle = robot.sensor.last_servo_pose;
    int last_scan = robot.sensor.last_scan;
    if(last_scan == 1 && servo_angle < 90) {
      servo_angle += scan_angle_precision;
    } else if (last_scan == -1 && servo_angle > -90) {
      servo_angle -= scan_angle_precision;
    } else {
      servo_angle = scan_angle_precision + last_scan;
      last_scan = -last_scan;
    }
    sensorServo.write(servo_angle);
    robot.sensor.last_mesurement_angle = radians(servo_angle) + angle_robot;
    robot.sensor.last_servo_pose = servo_angle;    
    robot.sensor.last_scan = last_scan;
    mesure_distance(1);  
  
    if (servo_angle <= -180 || servo_angle >= 180){
      servo_angle = 0;
    }
    sensorServo.write(servo_angle);
    robot.sensor.last_mesurement_angle = servo_angle;
    mesure_distance(1);  //we don't had delay servo is not supposed to move a lot
  }
}

void map_save(){
  extern robot robot;
  extern grid grid;
  //easier to read in the code
  static float last_x = robot.sensor.last_mesurement_x;
  static float last_angle = robot.sensor.last_mesurement_angle;
  static float last_y = robot.sensor.last_mesurement_y;
  static float robot_x = robot.position.fusion.X;
  static float robot_y = robot.position.fusion.Y;
  static float robot_x_grid = (int)((robot_y/100) * pathfinding_grid_precision);
  static float robot_y_grid = (int)((robot_y/100) * pathfinding_grid_precision);
  static float last_dist = dist;
  int grid_size_x = int(x_map_size/pathfinding_grid_precision) +2 ; //+1 for each wall
  int grid_size_y = int(y_map_size/pathfinding_grid_precision) + 2;
  robot.sensor.last_mesurement_dist = dist;

  //define the mesured point position in the grid
  static int point_x = (int)(last_x + last_dist * cos(last_angle))/ \
    pathfinding_grid_precision;
  static int point_y = (int)(last_y + last_dist * sin(last_angle))/ \
    pathfinding_grid_precision;
  if (point_x == 0 || point_x == grid_size_x || point_y == 0 || point_y == grid_size_y ){
    int i = 1;
  }else {
    robot.sensor.not_a_wall = 1;
  }
  // Calculate the slope and intercept of the line between sensor and the obstacle
  float slope = ((float)point_y - (float)robot_y_grid) / \
   ((float)point_x - (float)robot_x_grid);
  float intercept = (float)point_y - slope * (float)point_x;

  // Iterate over all the cells between the sensor and the obstacle
  for (int x = robot_x_grid; x <= point_x; x++) {
    int y = (int)(slope * (float)x + intercept);
    grid.matrix[x][y] = 0;
  }
};