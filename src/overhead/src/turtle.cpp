#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tr1/tuple>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>


// Enums -- In part 1, SLINE just moves the robot forward.
enum PlanningPhase { SLINE, WALL };
char* phaseString[] = { "SLINE", "WALL" };
enum Direction { LEFT, CENTER, RIGHT, NONE };
char* DirectionString[] = {"LEFT", "CENTER", "RIGHT", "NONE" };
int CurrentPhase;

// Scan vars
int Num_Points;
float Raw_Scan[1000];
float Smoothed_Scan[3]; // Left, Right, Middle averages
float SafeDist = 0.75;

int ScanRep;
int PreviousWallDist;
int PreviousWallSide;
int UpdatePreviousReadings;

// Constants
const int RIGHT_INDEX_START = 0;
const int RIGHT_INDEX_END = 150;
const int LEFT_INDEX_START = 450; // Used for determining left/right/center
const int LEFT_INDEX_END = 610;

const int MAX_SCAN_REP = 5;
const float MOVEMENT_SPEED = 0.25;
const float TURN_RATE  = 0.15;
const float FOLLOW_DISTANCE = 1;
const float SAFE_DISTANCE = 0.75;

ros::Publisher Motors_publisher;
ros::Publisher Velocity_publisher;
geometry_msgs::Twist Cmd;
kobuki_msgs::MotorPower Msg_motor;

// Method declarations
void forward(float dist);
void turn(float rad);
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void planPath();
Direction getObstacleDirection(int pointNo);
float getBeamAngle(int beamNumber, float angleStart, float angleIncrement);
void SmoothScan();
std::tr1::tuple<float,int> GetClosestPointAndDirection();

// Phase methods
void WallPhase();
void SlinePhase();

// Code
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wallrunner");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;
	Motors_publisher = n.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
  Velocity_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  Msg_motor.state = 1;
  Motors_publisher.publish(Msg_motor); //enable engine!

  CurrentPhase = SLINE;
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

// Scan the number of times specified by maxScanRep before moving. This is to prevent magical NaN misreads.
// Then take the smallest non-NaN reading or use NaN if that was all that was read to plan the next move.
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Length of scan array
  Num_Points =  (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;

  // Copy everything from the first scan into the local scan array
  // After that only copy non non-NaN values.
  if(ScanRep < MAX_SCAN_REP)
  {
    for(int i = 0; i < Num_Points; i++)
    {
      if( i == 0)
      {
        Raw_Scan[i] = scan->ranges[i];
      }
      else if(Raw_Scan[i] == Raw_Scan[i])
      {
        Raw_Scan[i] = scan->ranges[i];
      }
    }
    ScanRep++;
  }
  else // 5 scans complete
  {
    // Move
    SmoothScan();
    planPath();

    // Reset the scan counter
    ScanRep = 0;
  }
}

// Calculates the average between the Raw Scan reading.
// From start (inclusive) to end (exclusive)
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
void SmoothScan()
{
  // Right side
  Smoothed_Scan[0] = CalculateAverageBetweenScanPoints(LEFT_INDEX_START, LEFT_INDEX_END);
  Smoothed_Scan[1] = CalculateAverageBetweenScanPoints(RIGHT_INDEX_END, LEFT_INDEX_START);
  Smoothed_Scan[2] = CalculateAverageBetweenScanPoints(RIGHT_INDEX_START, RIGHT_INDEX_END);
  printf("RIGHT: %f\tCENTER: %f\tLEFT: %f\tPHASE: %s\n", Smoothed_Scan[2],Smoothed_Scan[1],Smoothed_Scan[0], phaseString[CurrentPhase]);
}

void planPath()
{
  if(UpdatePreviousReadings)
  {
    // TODO
    UpdatePreviousReadings = 0;
  }
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

// In the wall runner code, this moves forward only until a wall comes within
// following distance.
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
  else if(smallest > SafeDist) // safe < smallest < follow, start following that wall
  {
    CurrentPhase = WALL;
    PreviousWallDist = smallest;
    PreviousWallSide = side;
  }
  else // too close to wall -- REVERSE THRUSTERS ACTIVATE
  {
    forward(-1 * MOVEMENT_SPEED);
  }
}

// Currently following the wall determined by the previously seen wall dist and side.
// If the wall is currently next to us
    // If wallDist < safetyDist
      // Turn away and forward
    // Else if wallDist < wallfollowDist
      // FORWARD
    // Else (wall way too far)
      // turn towards and forward
// If the wall is in front of us
  // Turn

// The above is likely amazingly flawed but it will be a start.
void WallPhase()
{
  std::tr1::tuple<float, int> point = GetClosestPointAndDirection();
  float smallest = std::tr1::get<0>(point);
  int side = std::tr1::get<1>(point);

  if(PreviousWallSide ==  CENTER)
  {
    turn(TURN_RATE);
    PreviousWallSide = RIGHT;
    PreviousWallDist = Smoothed_Scan[CENTER];
  }
  else // following wall is on side
  {
    float currentWallDist = Smoothed_Scan[PreviousWallSide];
    if(currentWallDist < SAFE_DISTANCE)
    {
      if(Smoothed_Scan[CENTER] < FOLLOW_DISTANCE)
      {
        PreviousWallSide = CENTER;
        PreviousWallDist = Smoothed_Scan[CENTER];
      }

      float rotationModifier = 1;
      if(PreviousWallSide == LEFT)
      {
        rotationModifier = -1;
      }
      turn(TURN_RATE * rotationModifier);
    }
    else if(currentWallDist < FOLLOW_DISTANCE)
    {
      forward(MOVEMENT_SPEED);
    }
    else
    {
      float rotationModifier = 1;
      if(PreviousWallSide == RIGHT)
      {
        rotationModifier = -1;
      }
      turn(TURN_RATE * rotationModifier);
    }

    UpdatePreviousReadings = 1;
  }
}

float getBeamAngle(int beamNumber, float angleStart, float angleIncrement )
{
  return beamNumber*angleIncrement+angleStart;
}

Direction getObstacleDirection(int pointNo)
{
  if (pointNo >= 295 && pointNo < 315)
  {
    return CENTER;
  }
  else if (pointNo >=315)
  {
    return LEFT;
  }
  else
  {
    return RIGHT;
  }
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
