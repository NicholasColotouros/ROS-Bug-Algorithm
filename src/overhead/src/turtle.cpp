#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
ros::Publisher motors_publisher_;
ros::Publisher velocity_publisher_;
geometry_msgs::Twist cmd;
kobuki_msgs::MotorPower msg_motor;
float ranges[1000];
void forward(float dist);

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
    length =  (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;
     if(running == 0){
       for(int i = 0; i < length; i++){
         ranges[i] = scan->ranges[i];
         //printf("ranges %f\n", ranges[i]);
       }
     }
     //path_planner();
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
   	ros::NodeHandle n;
	motors_publisher_ = n.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
    velocity_publisher_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    msg_motor.state = 1;
    motors_publisher_.publish(msg_motor); //enable engine!
	//Chat Conversation End
    //forward(2.0);
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1000,processLaserScan);
    ros::spin();

}

void forward(float dist){
  cmd.linear.x = dist;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
  velocity_publisher_.publish(cmd);
}