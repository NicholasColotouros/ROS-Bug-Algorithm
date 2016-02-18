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
const float MOVEMENT_SPEED = 0.05;
const float TURN_RATE  = 0.15;

const float FOLLOW_DISTANCE = 1.5;
const float SAFE_DISTANCE = 1;

const int MAX_SCAN_REP = 5;
const float CORNER_DROPOFF_DELTA = 0.75;  // Used to detect how far of a difference in readings consists of a corner
const float TURN_WAIT_TIME_SECONDS = 0.5; // A few actions move and turn, this is the delay between the two

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
  const int right_index_start = 0;
  const int right_index_end = 150;
  const int left_index_start = 450;
  const int Left_index_end = 610;

  // Right side
  Smoothed_Scan[0] = GetClosestPointInRange(left_index_start, Left_index_end);
  Smoothed_Scan[1] = GetClosestPointInRange(right_index_end, left_index_start);
  Smoothed_Scan[2] = GetClosestPointInRange(right_index_start, right_index_end);
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
    CurrentPhase = WALL;

    // Pick a side to put the wall and change the phase
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
  }
  else // too close to wall -- REVERSE THRUSTERS ACTIVATE
  {
    Forward(-1 * MOVEMENT_SPEED);
  }
}

// This phase is responsible for following the wall.
// The robot will adjust itself based on its proximity to the wall it has decided to follow.
void WallPhase()
{
  // Turn away if there is a wall in front
  if(Smoothed_Scan[CENTER] < SAFE_DISTANCE)
  {
    TurnAway(TURN_RATE * 2);
  }
  else // No wall in front of us
  {
    float wallDist = Smoothed_Scan[FollowWallSide];

    if( abs(PreviousWallDist - wallDist) > CORNER_DROPOFF_DELTA) // Dropoff was too big, we just saw a corner (or similar)
    {
      CurrentPhase = CORNER;
      CurrentCornerPhase = MOVE;
    }
    // Turn away if the wall we're following is too close
    else if(wallDist < SAFE_DISTANCE)
    {
      TurnAway(TURN_RATE);
    }
    else if(wallDist > FOLLOW_DISTANCE) // Wall too far but not corner, turn towards it
    {
      TurnAway(-1*TURN_RATE);
      ros::Duration(TURN_WAIT_TIME_SECONDS).sleep();
      Forward(MOVEMENT_SPEED/2);
    }
    // Distance is just right
    else
    {
      Forward(MOVEMENT_SPEED);
    }
  }
}

void CornerPhase()
{
  if(CurrentCornerPhase == MOVE)
  {
    float boost_into_doorway_speed = 1.10;
    TurnAway(TURN_RATE * 2);
    ros::Duration(TURN_WAIT_TIME_SECONDS).sleep();
    Forward(boost_into_doorway_speed); // Get a good ways into the doorway
    CurrentCornerPhase = TURN;
  }
  else if(CurrentCornerPhase == TURN)
  {
    // Turn towards the wall until we can see the wall again and then move to adjust phase
    if(Smoothed_Scan[FollowWallSide] < FOLLOW_DISTANCE)
    {
      CurrentCornerPhase = ADJUST;
    }
    else
    {
      TurnAway(-1 * TURN_RATE);
    }
  }
  else // Adjust phase
  {
    CurrentPhase = WALL;
    CurrentCornerPhase = 0;
  }
}
