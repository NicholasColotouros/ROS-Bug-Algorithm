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
enum PlanningPhase { SLINE, WALL, CORNER };
char* phaseString[] = { "SLINE", "WALL", "CORNER" }; // TODO: remove once debugging is complete
enum CornerSubPhase { MOVE, TURN, ADJUST };
char* CornerString[] = {"MOVE", "TURN", "ADJUST"};
enum Direction { LEFT, CENTER, RIGHT, NONE };
char* DirectionString[] = {"LEFT", "CENTER", "RIGHT", "NONE" }; // TODO: remove once debugging is complete

///////////////////////////////////
///////// STATE VARIABLES /////////
///////////////////////////////////
int CurrentPhase;         // What planning phase the robot is in
int CurrentCornerPhase;   // How far we are in our process of turning corners
int Num_Points;           // The number of points in the raw scan
float Raw_Scan[1000];     // The raw readings by the scanner
float Smoothed_Scan[3];   // Left, Right, Middle averages
int CurrentScanRep;       // Current repetition of the scan
int FollowWallSide;       // Tells use which side the wall we're following is on/should be
float PreviousWallDist;

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
const int RIGHT_INDEX_START = 0;  // Which indices determine the vision for left, right and center parts of vision
const int RIGHT_INDEX_END = 150;
const int LEFT_INDEX_START = 450;
const int LEFT_INDEX_END = 610;

const float MOVEMENT_SPEED = 0.05;
const float TURN_RATE  = 0.15;

const float FOLLOW_DISTANCE = 1.5;
const float SAFE_DISTANCE = 1;

const int MAX_SCAN_REP = 5;
const float CORNER_DROPOFF_DELTA = 1; // Used to detect how far of a difference in readings consists of a corner
const float TURN_WAIT_TIME_SECONDS = 0.5;

///////////////////////////////////
/////// METHOD DECLARATIONS ///////
///////////////////////////////////
void ProcessLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void PlanPath();
void WallPhase();
void SlinePhase();
void CornerPhase();

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
  PreviousWallDist = 9999;

  // All systems ready. BEGIN.
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, ProcessLaserScan);
  ros::spin();
}

void Forward(float dist)
{
  Cmd.linear.x = dist;
  Cmd.linear.y = 0;
  Cmd.linear.z = 0;
  Cmd.angular.x = 0;
  Cmd.angular.y = 0;
  Cmd.angular.z = 0;
  Velocity_publisher.publish(Cmd);
}

void Turn(float rad)
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
  Turn(turnModifier * rad);
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

// Finds the closest point in the Raw Scan reading from start (inclusive) to end (exclusive).
// Called by MakeSmoothScan()
float GetClosestPointInRange(int start, int end)
{
  float smallest = 9999;
  for(int i = start; i < end; i++)
  {
    if(Raw_Scan[i] == Raw_Scan[i] && smallest > Raw_Scan[i])
    {
      smallest = Raw_Scan[i];
    }
  }
  return smallest;
}

// Calculates the closest points of the left, right and center points
// for smoothed vision (average of the left right and center points)
void MakeSmoothScan()
{
  // Right side
  Smoothed_Scan[0] = GetClosestPointInRange(LEFT_INDEX_START, LEFT_INDEX_END);
  Smoothed_Scan[1] = GetClosestPointInRange(RIGHT_INDEX_END, LEFT_INDEX_START);
  Smoothed_Scan[2] = GetClosestPointInRange(RIGHT_INDEX_START, RIGHT_INDEX_END);
}


///////////////////////////////////
///////// PLANNER METHODS /////////
///////////////////////////////////

// Scan the number of times specified by maxScanRep before moving. This is to prevent magical NaN misreads.
// Then take the smallest non-NaN reading or use NaN if that was all that was read to plan the next move.
void ProcessLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
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
    PlanPath();

    // Reset the scan counter
    CurrentScanRep = 0;
  }
}

void PlanPath()
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

    case CORNER:
      CornerPhase();
      break;

    case WALL:
    default:
      WallPhase();
      break;
  }
  PreviousWallDist = Smoothed_Scan[FollowWallSide];
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
    Forward(MOVEMENT_SPEED);
  }
  else if(smallest > SAFE_DISTANCE) // safe < smallest < follow, start following that wall
  {
    CurrentPhase = WALL; // Next phase is guaranteed to be wall following

    // Pick a side to put the wall
    if(side == CENTER)
    {
      if(Smoothed_Scan[LEFT] < Smoothed_Scan[RIGHT])
      {
        FollowWallSide = LEFT;
      }
      else
      {
        FollowWallSide = RIGHT;
      }
      printf("Side decided: %s\n", DirectionString[FollowWallSide]);
    }
    else if(side == RIGHT)
    {
      FollowWallSide = RIGHT;
    }
    else
    {
      FollowWallSide = LEFT;
    }

    PreviousWallDist = Smoothed_Scan[FollowWallSide];
    TurnAway(TURN_RATE * 2); // turn away from the wall in front
  }
  else // too close to wall -- REVERSE THRUSTERS ACTIVATE
  {
    Forward(-1 * MOVEMENT_SPEED);
  }
}

// TODO
void WallPhase()
{
  printf("%f %s\t", Smoothed_Scan[FollowWallSide], DirectionString[FollowWallSide]);
  printf("LEFT: %f\t CENTER: %f\t RIGHT: %f\n", Smoothed_Scan[LEFT], Smoothed_Scan[CENTER], Smoothed_Scan[RIGHT]);

  // Turn away if there is a wall in front
  if(Smoothed_Scan[CENTER] < FOLLOW_DISTANCE)
  {
    TurnAway(TURN_RATE * 2); // 90 degrees
    ros::Duration(TURN_WAIT_TIME_SECONDS).sleep();
    Forward(MOVEMENT_SPEED/2);
    printf("Wall in front, turning %s\t", DirectionString[2 - FollowWallSide]);
  }
  else // No wall in front of us
  {
    float wallDist = Smoothed_Scan[FollowWallSide];

    // Turn away if the wall we're following is too close
    if(wallDist < SAFE_DISTANCE)
    {
      TurnAway(TURN_RATE);
      ros::Duration(TURN_WAIT_TIME_SECONDS).sleep();
      Forward(MOVEMENT_SPEED/2);
      printf("Too close, turning %s\t", DirectionString[2 - FollowWallSide]);
    }
    else if(PreviousWallDist - wallDist > CORNER_DROPOFF_DELTA) // Dropoff was too big, we just saw a corner (or similar)
    {
      CurrentPhase = CORNER;
      CurrentCornerPhase = MOVE;
    }
    else if(wallDist > FOLLOW_DISTANCE) // Wall too far but not corner, turn towards it
    {
      TurnAway(-1*TURN_RATE);
      ros::Duration(TURN_WAIT_TIME_SECONDS).sleep();
      Forward(MOVEMENT_SPEED/2);
      printf("Too far. Turning %s\t", DirectionString[FollowWallSide]);
    }
    // Distance is just right
    else
    {
      Forward(MOVEMENT_SPEED);
      printf("CHOO CHOO\t");
    }
  }
  printf("\n" );
}

void CornerPhase()
{
  printf("CORNER: %s\n", CornerString[CurrentCornerPhase]);
  if(CurrentCornerPhase == MOVE)
  {
    Forward(MOVEMENT_SPEED * 5);
    CurrentCornerPhase = TURN;
  }
  else if(CurrentCornerPhase == TURN)
  {
    TurnAway(-3.14/4); // Turn 45 degrees towards where the wall should be
    CurrentCornerPhase = ADJUST;
  }
  else // Adjust phase
  {
    CurrentPhase = WALL;
    CurrentCornerPhase = 0;
  }
}
