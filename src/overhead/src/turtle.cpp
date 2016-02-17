#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tr1/tuple>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>

///////////////////////////////////
////////////// ENUMS //////////////
///////////////////////////////////

// Enums -- In part 1, SLINE just moves the robot forward.
enum PlanningPhase { SLINE, WALL };
char* phaseString[] = { "SLINE", "WALL" };
enum Direction { LEFT, CENTER, RIGHT, NONE };
char* DirectionString[] = {"LEFT", "CENTER", "RIGHT", "NONE" };

///////////////////////////////////
///////// STATE VARIABLES /////////
///////////////////////////////////
int CurrentPhase;         // What planning phase the robot is in
int Num_Points;           // The number of points in the raw scan
float Raw_Scan[1000];     // The raw readings by the scanner
float Smoothed_Scan[3];   // Left, Right, Middle averages
int CurrentScanRep;       // Current repetition of the scan
int FollowWallSide;       // Tells use which side the wall we're following is on/should be

///////////////////////////////////
////////// ROS VARIABLES //////////
///////////////////////////////////
ros::Publisher Motors_publisher;
ros::Publisher Velocity_publisher;
geometry_msgs::Twist Cmd;
kobuki_msgs::MotorPower Msg_motor;

///////////////////////////////////
//////////// CONSTANTS ////////////
///////////////////////////////////
const int RIGHT_INDEX_START = 0;
const int RIGHT_INDEX_END = 150;
const int LEFT_INDEX_START = 450;
const int LEFT_INDEX_END = 610;

const int MAX_SCAN_REP = 5;
const float MOVEMENT_SPEED = 0.25;
const float TURN_RATE  = 0.15;
const float FOLLOW_DISTANCE = 1.5;
const float SAFE_DISTANCE = 0.75;


///////////////////////////////////
/////// METHOD DECLARATIONS ///////
///////////////////////////////////
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void planPath();
void WallPhase();
void SlinePhase();

///////////////////////////////////
///// MAIN AND HELPER METHODS /////
///////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wallrunner");

  // Ros turn on overhead
  ros::NodeHandle n;
	Motors_publisher = n.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
  Velocity_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  Msg_motor.state = 1;
  Motors_publisher.publish(Msg_motor); //enable engine!

  // Robot state initialization
  CurrentPhase = SLINE;
  FollowWallSide = NONE;

  // All systems ready. BEGIN.
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, processLaserScan);
  ros::spin();
}

void forward(float dist)
{
  Cmd.linear.x = dist;
  Cmd.linear.y = 0;
  Cmd.linear.z = 0;
  Cmd.angular.x = 0;
  Cmd.angular.y = 0;
  Cmd.angular.z = 0;
  Velocity_publisher.publish(Cmd);
}

void turn(float rad)
{
  Cmd.linear.x = 0;
  Cmd.linear.y = 0;
  Cmd.linear.z = 0;
  Cmd.angular.x = 0;
  Cmd.angular.y = 0;
  Cmd.angular.z = rad;
  Velocity_publisher.publish(Cmd);
}

// Turn away from the wall being followed
void TurnAway(float rad)
{
  // Turn away from the wall we're following -- we've reached a corner
  int turnModifier = 1;
  if(FollowWallSide == LEFT)
  {
    turnModifier = -1;
  }
  turn(turnModifier * rad);
}

// Returns <dist, side> for the smoothed vision.
// Returns a distance of 9999 and side of NONE if everything is out of range
std::tr1::tuple<float, int> GetClosestPointAndDirection()
{
  float smallest = 9999;
  int side = NONE;

  if(Smoothed_Scan[0] < smallest && Smoothed_Scan[0] != 0)
  {
    Smoothed_Scan[0];
  }

  if(Smoothed_Scan[1] < smallest && Smoothed_Scan[1] != 0)
  {
    smallest = Smoothed_Scan[1];
    side = CENTER;
  }
  if(Smoothed_Scan[2] < smallest && Smoothed_Scan[2] != 0)
  {
    smallest = Smoothed_Scan[2];
    side = RIGHT;
  }
  return std::tr1::make_tuple(smallest, side);
}

// Calculates the average between the Raw Scan reading.
// From start (inclusive) to end (exclusive)
// Called by MakeSmoothScan()
float CalculateAverageBetweenScanPoints(int start, int end)
{
  int countedPoints = 0;
  float total = 0;
  for(int i = start; i < end; i++)
  {
    if(Raw_Scan[i] == Raw_Scan[i])
    {
      total += Raw_Scan[i];
      countedPoints++;
    }
  }
  if(countedPoints == 0)
  {
    return 0;
  }

  return total/countedPoints;
}

// Calculates the averages of the left, right and center points
// for smoothed vision (average of the left right and center points)
void MakeSmoothScan()
{
  // Right side
  Smoothed_Scan[0] = CalculateAverageBetweenScanPoints(LEFT_INDEX_START, LEFT_INDEX_END);
  Smoothed_Scan[1] = CalculateAverageBetweenScanPoints(RIGHT_INDEX_END, LEFT_INDEX_START);
  Smoothed_Scan[2] = CalculateAverageBetweenScanPoints(RIGHT_INDEX_START, RIGHT_INDEX_END);
}


///////////////////////////////////
///////// PLANNER METHODS /////////
///////////////////////////////////

// Scan the number of times specified by maxScanRep before moving. This is to prevent magical NaN misreads.
// Then take the smallest non-NaN reading or use NaN if that was all that was read to plan the next move.
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Length of scan array
  Num_Points =  (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;

  // Copy everything from the first scan into the local scan array
  // After that only copy non non-NaN values.
  if(CurrentScanRep < MAX_SCAN_REP)
  {
    for(int i = 0; i < Num_Points; i++)
    {
      if(CurrentScanRep == 0)
      {
        Raw_Scan[i] = scan->ranges[i];
      }
      else if(Raw_Scan[i] == Raw_Scan[i])
      {
        Raw_Scan[i] = scan->ranges[i];
      }
    }
    CurrentScanRep++;
  }
  else // 5 scans complete
  {
    // Move
    MakeSmoothScan();
    planPath();

    // Reset the scan counter
    CurrentScanRep = 0;
  }
}

void planPath()
{
  // TODO: implement for phase 2 of bug algo under WALL case:
  // IF robot is on SLINE
    // IF robot is facing the goal AND there is a wall in front
      // Follow the wall -- potential bug would be that this gets stuck within tolerances
    // ELSE
      // Change phase to SLINE phase
        // The SLine phase will then turn towards the goal and either
          // Follow the new wall right in front of it
          // Move towards the goal
  // ELSE
    // Follow Wall
  switch(CurrentPhase)
  {
    case SLINE:
      SlinePhase();
      break;

    case WALL:
    default:
      WallPhase();
      break;
  }
}

// In the wall runner code, this moves forward only until a wall comes within following distance.
// In the full bug algorithm, it will follow the SLine.
// Then it will check for a wall in front.
void SlinePhase()
{
  std::tr1::tuple<float, int> point = GetClosestPointAndDirection();
  float smallest = std::tr1::get<0>(point);
  int side = std::tr1::get<1>(point);
  if(smallest > FOLLOW_DISTANCE || smallest == 0) // safe and not following wall
  {
    forward(MOVEMENT_SPEED);
  }
  else if(smallest > SAFE_DISTANCE) // safe < smallest < follow, start following that wall
  {
    CurrentPhase = WALL;

    // Pick a side to put the wall
    if(side == CENTER || side == RIGHT)
    {
      FollowWallSide = RIGHT;
    }
    else
    {
      FollowWallSide = LEFT;
    }
    TurnAway(TURN_RATE * 2); // turn away from the wall
    printf("Turning %s\n", DirectionString[2-FollowWallSide]);
  }
  else // too close to wall -- REVERSE THRUSTERS ACTIVATE
  {
    forward(-1 * MOVEMENT_SPEED);
  }
}

// TODO
void WallPhase()
{
  printf("%f %s\t", Smoothed_Scan[FollowWallSide], DirectionString[FollowWallSide]);

  // Turn away if there is a wall in front
  if(Smoothed_Scan[CENTER] < SAFE_DISTANCE)
  {
    TurnAway(TURN_RATE * 2); // 90 degrees
    printf("Wall in front, turning %s\t", DirectionString[2 - FollowWallSide]);
  }
  else // No wall in front of us
  {
    float wallDist = Smoothed_Scan[FollowWallSide];

    // Turn away if the wall we're following is too close
    if(wallDist < SAFE_DISTANCE)
    {
      TurnAway(TURN_RATE);
      printf("Too close, turning %s\t", DirectionString[2 - FollowWallSide]);
    }
    // Turn towards if the wall we're following is too far
    else if(wallDist > FOLLOW_DISTANCE)
    {
      TurnAway(-1*TURN_RATE);
      printf("Too far. Turning %s\t", DirectionString[FollowWallSide]);
    }
    // Distance is just right
    else
    {
      forward(MOVEMENT_SPEED);
      printf("CHOO CHOO\t");
    }
  }
  printf("\n" );
}
