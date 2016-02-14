#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>


// Global vars
float ranges[1000];
float closestPoint;
int scanRep;
const int maxScanRep = 5;
float minLength;
int running;

ros::Publisher motors_publisher;
ros::Publisher velocity_publisher;
geometry_msgs::Twist cmd;
kobuki_msgs::MotorPower msg_motor;

// Method declarations
void forward(float dist);
void turn(float rad);
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void planPath();

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
	motors_publisher = n.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  msg_motor.state = 1;
  motors_publisher.publish(msg_motor); //enable engine!
	//Chat Conversation End
  //forward(2.0);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1000,processLaserScan);
  ros::spin();
}

void forward(float dist)
{
  cmd.linear.x = dist;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
  velocity_publisher.publish(cmd);
}

void turn(float rad)
{
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = rad;
  velocity_publisher.publish(cmd);
}

// Scan the number of times specified by maxScanRep before moving. This is to prevent magical NaN misreads. 
// Then take the smallest non-NaN reading or use NaN if that was all that was read to plan the next move.
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int length =  (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;
  if(scanRep < maxScanRep)
  {
    int onlyNaN = 1;
    for(int i = 0; i < length; i++)
    {
      ranges[i] = scan->ranges[i];
      float current = ranges[i];

      if(minLength > current && current == current)
        minLength = current;
      
      // If only NaN, accept it.
      if(current == current)
        onlyNaN = 0;
      //printf("ranges %f\n", ranges[i]);
    }
    if(onlyNaN == 1)
    {
      printf("ONLY NANN\n");
      minLength = ranges[0];
    }

    scanRep++;
    printf("Scan %d: %f\n", scanRep, minLength);
  }
  else
  {
    printf("\n");
    planPath();
    minLength = 9999;
    scanRep = 0;    
  }
}

void planPath()
{
  printf("%f\n", minLength);
  if(0.75 < minLength)
    forward(0.5);
  else
    turn(0.5); 
}