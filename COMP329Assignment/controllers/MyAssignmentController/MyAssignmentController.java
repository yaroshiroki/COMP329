// File:          MyAssignmentController.java
// Date: 15/12/2021
// Description: A controller class dedicated to defining the robots movement and maze solving strategy
// Author: Yaroslav Shiroki (201458436)
// Modifications: Defined the movement of the robot, timer, target velocity

// ==============================================================
// COMP329 2021 Programming Assignment
// ==============================================================
// 
// The aim of the assignment is to move the robot around the arena in such a way
// as to generate an occupancy grid map of the arena itself.  Full details can be
// found on CANVAS for COMP329
//
// Only add code to the controller file - do not modify the other java files in this project.
// You can add code (such as constants, instance variables, initialisation etc) anywhere in 
// the file, but the navigation itself that occurs in the main loop shoudl be done after checking
// the current pose, and having updated the two displays.
//
// Note that the size of the occup[ancy grid can be changed (see below) as well as the update
// frequency of the map, adn whether or not a map is generated.  Changing these values may be
// useful during the debugging phase, but ensure that the solution you submit generates an
// occupancy grid map of size 100x100 cells (with a recommended update frquency of 2).
//
// ==============================================================

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import java.util.*;

public class MyAssignmentController {

  // ---------------------------------------------------------------------------
  // Dimensions of the Robot
  // ---------------------------------------------------------------------------
  // Note that the dimensions of the robot are not strictly circular, as 
  // according to the data sheet the length is 485mm, and width is 381mm
  // so we assume for now the aprox average of the two (i.e. 430mm)
  private final static double ROBOT_RADIUS = 0.215;  // in meters
  public static double wheelRadius = 0.0975;         // in meters
  public static double axelLength = 0.31;            // Distance (in m) between the two wheels
  public static int MAX_NUM_SENSORS = 16;            // Number of sensors on the robot
  // ---------------------------------------------------------------------------
  // Assignment Parameters
  // ---------------------------------------------------------------------------
  // Note that ideally the number of cells in the occupancy grid should be a multiple of the
  // display size (which is 500x500).  So smaller values such as 50x50 or 25x25 could be used
  // to initialise a map with fewer, but larger grid cells 
  private final static int NUMBER_OF_ROWCELLS = 100;   // How many cells across the occupancy grid
  private final static int NUMBER_OF_COLCELLS = 100;   // How many cells down the occupancy grid
  
  // This is the frequency that the map is updated (i.e. the map is updated every GRID_UPDATE_FREQUENCY
  // times the loop is iterated.  Increasing it may make the simulator run faster, but fewer samples
  // will be taken
  private final static int GRID_UPDATE_FREQUENCY = 2;  // How frequently do we sample the world 
  
  // This boolean switches on (or off) the generation of the occupancy grid.  It may be useful to
  // make this false whilst working on the navigation code to speed things up, but any final solution
  // should verify that a valid occupancy grid map is generated.
  private final static boolean GENERATE_OCCUPANCY_GRID = true;

  // ---------------------------------------------------------------------------
  // Robot instance
  // ---------------------------------------------------------------------------
  public static Supervisor robot;
  public static Node robotNode;

  // ==================================================================================
  // Static Methods  
  // ==================================================================================
  // getLocalisedPos()
  //   returns the real position of the robot without the need for localisation through
  //   particle filters etc.  The supervisor mode is used to facilitate this.
  public static Pose getLocalisedPos() {
    double[] realPos = robotNode.getPosition();
    double[] rot = robotNode.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(rot[2], rot[8]);
    double halfPi = Math.PI/2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi)
        theta2 = -(3*halfPi)+theta1;
    
    return new Pose(realPos[0], -realPos[2], theta2);
  }

  public enum MoveState { FORWARD, LEFTTURN, RIGHTTURN };
  // ==================================================================================
  // Main Methods 
  // ==================================================================================  
  public static void main(String[] args) {
    // added a timer in miliseconds in order to turn at desireable locations
    int timeElapsed = 0;
    MoveState state = MoveState.FORWARD;
    // Define Robot Parameters
    long loopCounter = 0;           // Used to count the number of main loop iterations
    // ---------------------------------------------------------------------------
    // create the Supervised Robot instance.
    // ---------------------------------------------------------------------------
    robot = new Supervisor();
    robotNode = robot.getSelf(); // Get a handle to the Robot Node in supervisor mode
    
    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());    
    int step_count = 0;             // We use this to track time in our simulation
    // ---------------------------------------------------------------------------
    // Set up motor devices
    // ---------------------------------------------------------------------------
    Motor leftMotor = robot.getMotor("left wheel");
    Motor rightMotor = robot.getMotor("right wheel");
    
    // Set the target positions of the motors
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    double maxVel = leftMotor.getMaxVelocity();
    // target velocity of approximately 2pir
    double targetVel = 0.3*maxVel;          // Our target velocity
    // Initialise motor velocity
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    

    // ---------------------------------------------------------------------------
    // set up proximity detectors
    // ---------------------------------------------------------------------------
    DistanceSensor[] ps = new DistanceSensor[MAX_NUM_SENSORS];
    String[] psNames = {
      "so0", "so1", "so2", "so3", "so4", "so5", "so6", "so7",
      "so8", "so9", "so10", "so11", "so12", "so13", "so14", "so15",     
    };
    
    // The following array determines the orientation of each sensor, based on the
    // details of the Pioneer Robot Stat sheet.  Note that the positions may be slightly
    // inaccurate as the pioneer is not perfectly round.  Also these values are in degrees
    // and so may require converting to radians.  Finally, we assume that the front of the
    // robot is between so3 and so4.  As the angle between these is 20 deg, we assume that 
    // they are 10 deg each from the robot heading
    double[] psAngleDeg = { 90, 50, 30, 10, -10, -30, -50, -90,
                            -90, -130, -150, -170, 170, 150, 130, 90};

    double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < MAX_NUM_SENSORS; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(timeStep);
    }
    
    // ---------------------------------------------------------------------------
    // Set up occupancy grid
    // ---------------------------------------------------------------------------
    OccupancyGrid grid;                              // Instantiate to generate an occupancy grid
    if (GENERATE_OCCUPANCY_GRID == true) {
      grid = new OccupancyGrid(5.0, 5.0,             // Size of the arena
                                NUMBER_OF_ROWCELLS,  // Number of cells along the x-axis
                                NUMBER_OF_COLCELLS,  // Number of cells along the y-axis
                                ps,                  // Array of distance sensors
                                psAngleDeg,          // Orientation of each sensor (array)
                                ROBOT_RADIUS);       // Radius of the robot body (assumes cylindrical)
    } else {
      grid=null;                                     // No occupancy grid will be generated
    }
  
    // ---------------------------------------------------------------------------
    // Set up display devices
    // ---------------------------------------------------------------------------
    // The sensor view from the Labs is included to assist in debugging
    Display sensorDisplay = robot.getDisplay("sensorDisplay");
    SensorView sensorView = new SensorView(sensorDisplay, ps, psAngleDeg, ROBOT_RADIUS);

    // A variant of the Arena view is used to show the robot position in a map.
    // The current display is configured as a 500x500 display attached to the robot.     
    Display occupancyGridDisplay = robot.getDisplay("occupancyGridDisplay");
    ArenaView gridView = new ArenaView(occupancyGridDisplay, getLocalisedPos(), grid, 5.0, 5.0, ROBOT_RADIUS);

    // ---------------------------------------------------------------------------
    // Main loop:
    // ---------------------------------------------------------------------------
    // perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {
      //increment our timer in miliseconds by our timestep
      timeElapsed += timeStep;
      step_count++; //counter to stop robot
      // ---------------------------------------------------------------------------
      // Get current pose of the robot        
      // ---------------------------------------------------------------------------
      Pose p = getLocalisedPos();
            
      // ---------------------------------------------------------------------------
      // Update the grid map and arena display
      // ---------------------------------------------------------------------------
      if (loopCounter++ % GRID_UPDATE_FREQUENCY == 0) {
        if (GENERATE_OCCUPANCY_GRID == true) {
          grid.occupancy_grid_mapping(p);
        }
        gridView.setPose(p);
        gridView.paintView();
      }

      // ---------------------------------------------------------------------------
      // Update the sensor display
      // ---------------------------------------------------------------------------
      sensorView.setPose(p);
      sensorView.paintView();
      
      // ---------------------------------------------------------------------------
      // Move robot - Assignemnt
      // ---------------------------------------------------------------------------
      // Assignment Solution Here
      // Note - you can add code anywhere in this file, but it is suggested you add the
      // navigation code after the above steps (getting the pose, and updating the displays).
      for (int i = 0; i < MAX_NUM_SENSORS ; i++)
        psValues[i] = (1040 - ps[i].getValue())/2;
      
      // defining obstacles on the left/right side and in front of the robot
      // obstacle on the right hand side using the ps sensors 5-10
      boolean right_obstacle =
        psValues[5] < 45 ||
        psValues[6] < 35 ||
        psValues[7] < 25 ||
        psValues[8] < 25 ||
        psValues[9] < 35 ||
        psValues[10] < 45;
      // obstacle on the left hand side using the ps sensors 0-2 and 13-15
      // this will be the obstacle/wall which we will follow
      boolean left_obstacle =
        psValues[2] < 45 ||
        psValues[0] < 35 ||
        psValues[15] < 35 ||
        psValues[14] < 45 ||
        psValues[13] < 55 ||
        psValues[1] < 55;
      // obstacle in front of the robot using the ps sensors 1-6
      boolean front_obstacle =
        psValues[6] < 25.0 ||
        psValues[1] < 25.0 ||
        psValues[2] < 20.0 ||
        psValues[3] < 15.0 ||
        psValues[4] < 15.0 ||
        psValues[5] < 20.0;
      
      // if too close to the left wall or wall in front of robot, 
      // move away from this wall/obstacle
      boolean left_bump = 
        psValues[2] < 15 ||
        psValues[0] < 10 ||
        psValues[15] < 10 ||
        psValues[13] < 15 ||
        psValues[14] < 12.5 ||
        psValues[1] < 12.5;
      boolean front_bump =
        psValues[6] < 22.5 ||
        psValues[1] < 22.5 ||
        psValues[2] < 17.5 ||
        psValues[3] < 12.5 ||
        psValues[4] < 12.5 ||
        psValues[5] < 17.5;
      
      // print out the coordinates for testing and for finishing near the 
      // coordinates (2.1,2.1,0)
      // *** DON'T NEED TO INCLUDE IN FINAL VERSION OF THE SOLUTION ***
      //System.out.println(p.getX() + ": X, " + p.getY() + ": Y, " + p.getTheta() + ": Theta");
      
      
      // initially define the left wheel velocity and right weel velocity 
      // to approximately 2pi/r
      double leftVel = targetVel;
      double rightVel = targetVel;
      
      
      // attempted to use theta, turn angle and rot in order to calculate an exact 90 degree turn
      
      // these lines of code are not used in the final version of the code, however
      // I have left them here to show that I attempted to implement more complicated mathematcial
      // understandings into this solution. I was limited in my Java programming ability
      
      /////////////////////////////////////
      // double theta = p.getTheta(); // theta from odometry
      // double turnAngle = p.getDeltaTheta(theta);
      // double[] rot = robotNode.getOrientation(); 
            
      // turnAngle = Math.atan2(Math.sin(turnAngle), Math.cos(turnAngle)); // normalize angle -pi to pi
      /////////////////////////////////////
      
      
      //THEORY: Create a robot which follows the wall on its left hand side.
      //if there is an object in front of the robot, turn to the right to follow the left wall
      //if there is no wall on the left hand side, turn left in order to follow the wall that has ended
      //if there is no wall on the left or in front of the robot but there is a wall on the right side,
      // turn left to try find a new wall
      //if robot is too close to the left hand side, turn slightly to the right
      //if the robot is too close to the front wall, reverse to the left slightly
      
      //TURNING FUNCTIONS
      //If too close to left wall, curve to the right with the angular velocity of
      // 9.621 (3dp) radians per second and forwards velocity of 1.65 meters per second
      if (left_bump && left_obstacle && !front_bump && !front_obstacle) {
        leftVel = targetVel*0.5;
        rightVel = targetVel*0.1;
        // following line used for testing
        //System.out.println("LEFT WALL TOO CLOSE");
      }
      //If too close to the left wall and too close to the front wall, reverse in a curve
      // towards the left hand side with the angular velocity of -9.621 radians per second and forwards
      // velocity of -1.65 meters per second
      else if (left_bump && front_bump && front_obstacle && left_obstacle
      ) {
        leftVel = -targetVel*0.1;
        rightVel = -targetVel*0.5;
        // following line used for testing
        //System.out.println("FRONT AND LEFT TOO CLOSE");
      }
      //If there is only a wall on the left hand side then turn on the spot to the left until a wall is found
      // the angular velocity is pi radians/second and the forwards velocity is 0 as the robot is rotating
      // on the spot
      else if (right_obstacle && !left_obstacle && !front_obstacle && !left_bump && !front_bump) {
        leftVel = -targetVel*0.5;
        rightVel = targetVel*0.5;
        // following line used for testing
        //System.out.println("ONLY RIGHT");
      }
      //If there is a wall in front of the robot as well as on the left hand side
      // turn to the right on the spot, following the same wall with an angular 
      // velocity of -pi radians/second and the forwards velocity is 0 ...
      else if (left_obstacle && front_obstacle && !front_bump){
        leftVel = targetVel*0.5;
        rightVel = -targetVel*0.5;
      }
      //If there is a wall on the left hand side and the robot is not too close to the wall
      // then follow this wall and travel at a forwards velocity of 2pi radians/second
      // and an angular velocity of 0
      else if (left_obstacle && !left_bump) {
        leftVel = targetVel;
        rightVel = targetVel;
      }
      //If there is no wall on the left hand side and the robot is not too close
      // to the wall in front of it, turn to the left hand side in a curve with 
      // the angular velocity of 19.242 radians per second and forwards velocity of
      // 3.301 meters per second
      else if (!left_obstacle && !left_bump && !front_bump) {
        leftVel = targetVel*0.2;
        rightVel = targetVel;
        // following line used for testing
        //System.out.println("NO LEFT WALL");
      }
      //If there is no wall on the left hand side however, there is a wall in front
      // of the robot and it is too close, turn to the right on the spot with an angular
      // velocity of -pi radians/second and forwards velocity of 0 ...
      else if(!left_obstacle && front_bump && front_obstacle && !left_bump) {
        leftVel = targetVel*0.5;
        rightVel = -targetVel*0.5;
        // following line used for testing
        //System.out.println("NO LEFT BUT FRONT");
      }
      // This following else if statement was added in order to force the robot to move backwards 
      // in a curve in order to help it avoid getting stuck in some situations when turning to 
      // the right on the spot was not enough 
      
      //Same as before, angular velocity is now different however, I believe it is
      // -pi-1.027 = -4.168 radians per second
      // I believe the forwards velocity is -0.398 meters per second
      else if(!left_obstacle && !left_bump && front_bump && front_obstacle) {
        leftVel = -targetVel*0.1;
        rightVel = -targetVel*0.4;
      }
      
      //TURNING AT SPECIFIC TIMES TO COMPLETE THE MAZE
      // Theory: As the robot only follows the left hand wall, turn to the right 
      // when a lap of the maze is almost completed, in order to fully explore the 
      // maze. Then turn right again when almost in top left corner again, for the
      // robot to complete another lap of the maze.
      if (timeElapsed > 112000 && 
        (left_obstacle || right_obstacle || front_obstacle || left_bump || front_bump) && 
        timeElapsed < 120000 || 
        (timeElapsed > 150000 && timeElapsed < 156000) || 
        (timeElapsed > 224000 && timeElapsed < 227000)) {
        rightVel = targetVel*0.1;
        leftVel = targetVel*0.5;
        // following line used for testing
        //System.out.println("******************");
      }
      // turn right at this point to save time from the robot getting stuck in the
      // bottom right corner of the maze again, it has shown that it can escape this
      // trap on its own already hence, we don't need to watch it do this again
      else if (timeElapsed > 270000 && timeElapsed < 271000) {
        leftVel = -targetVel*0.25;
        rightVel = -targetVel*0.5;
      }
      // again, help the robot avoid the same pit in order to save time.
      else if (timeElapsed > 271000 && timeElapsed < 273000) {
        rightVel = targetVel*0.1;
        leftVel = targetVel*0.5;
      }
      //The following else if statement is used in order to stop the robot near
      // the coordinates (2.1,2.1,0)
      // If the timer gets to 05:11 then stop the robot no matter what
      else if (timeElapsed > 311000 && 
      (left_obstacle || 
      right_obstacle || 
      front_obstacle || 
      left_bump || 
      front_bump)) {
        leftVel = 0;
        rightVel = 0;
        targetVel = 0;
      }
      
      //set our left and right motors to rightVel and leftVel
      leftMotor.setVelocity(leftVel);
      rightMotor.setVelocity(rightVel);
      //STOP RUNNING AT 05:11
    };
    // Enter here exit cleanup code.
  }
}
