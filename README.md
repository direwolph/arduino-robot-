# Autonomous Arduino Robot
A really fun project for our 4th ingenering school year, an automous robot doing SLAM with basic arduino componant such as HC-SR04 or sg90 servos.

## How to get started
1. Clone the repository:

   ```shell
   git clone https://github.com/juliusherve/arduino-robot.git
   ```
2. Follow the wiring/building scheme (in progress ...)
3. Upload main.ino to the arduino Due
4. Launch "terminal.py" to get all robot data
5. You can edit "configuration.h" to adapt robot to your need

## How does it work ?
   The robot is moved by two linear control servo. Those servo have a speed define by the relation betwin the given pwm and max speed. We can use those parameter to determine robot speed and so position. Working in openloop, only using this technics, the robot accuracy will not be good enouth. This is why we use some sensor fusion with IMU positionning to reduce the positionning inacuraccy. 
   Robot use a SLAM simple algorithm, giving it the possibility to map is environement and so avoid the obstacle while following a A* algorithm to join flags by the shortest road. 
   
## Find a bug?

If you found an issue or would like to submit an improvement to this project, please submit an issue using the issues tab above. If you would like to submit a PR with a fix, reference the issue you created!
