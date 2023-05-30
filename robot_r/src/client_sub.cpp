/**
* \file client_sub.cpp
*
* \author Hafiz Muizz Ahmed Sethi
* \version 0.1
* \date 30/05/2023
*
* \details
*
* Subcribes to : <BR>
* 	/odom
*
* Publishes to : <BR>
*	/pod_vel
*
*
* Description:
*This code is a ROS node that subscribes to the "/odom" topic, which provides odometry information. 
*When a new message is received, the callback function "Cbkodom" is triggered. Inside the callback function, the received position and velocity values are assigned to a custom message of type "robot_r::Vel". 
*The assigned values are then printed to the console using "ROS_INFO". 
*The code also publishes the custom message to the "/pod_vel" topic at a rate of 1 Hz. The loop continues as long as the ROS node is running.
**/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <robot_r/Vel.h>

robot_r::Vel msg1;

/**
 * @brief Callback function for the odom topic.
 * @param msg Pointer to the received Odometry message.
 */
void Cbkodom(const nav_msgs::Odometry::ConstPtr& msg)
{
   //Set various values of the custom message
   msg1.x = msg->pose.pose.position.x;
   msg1.y = msg->pose.pose.position.y;
   msg1.velocity_x = msg->twist.twist.linear.x;
   msg1.velocity_y = msg->twist.twist.linear.y;
   
   ROS_INFO("x position: [%f]; y position: [%f], x velocity: [%f]; y velocity: [%f]", msg1.x, msg1.y, msg1.velocity_x, msg1.velocity_y);
}

/**
 * @brief Main function for the client_sub node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer representing the exit status.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "client_sub");
  ros::NodeHandle nh;
  
  // Publisher to /pod_vel, which publishes position and velocity
  ros::Publisher pub = nh.advertise<robot_r::Vel>("/pod_vel", 1);
  
  // Create the subscriber to /odom
  ros::Subscriber sub = nh.subscribe("/odom", 1, Cbkodom);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
  	// Publish /pod_vel
  	pub.publish(msg1);
  	sleep(1);
  	ros::spinOnce();
  	loop_rate.sleep();
  }

  return 0;
}

