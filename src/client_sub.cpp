#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <robot_r/Vel.h>

robot_r::Vel msg1;

// function callback for odom topic
void Cbkodom(const nav_msgs::Odometry::ConstPtr& msg){
   
   // we set various values off custom msg
   msg1.x = msg->pose.pose.position.x;
   msg1.y = msg->pose.pose.position.y;
   msg1.velocity_x = msg->twist.twist.linear.x;
   msg1.velocity_y = msg->twist.twist.linear.y;
   
   ROS_INFO("x position: [%f]; y position: [%f], x velocity: [%f]; y velocity: [%f]  ", msg1.x, msg1.y, msg1.velocity_x, msg1.velocity_y);
   
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "client_sub");
  ros::NodeHandle nh;
  
  // publiher to /pod_vel which publish position and velocity
  ros::Publisher pub = nh.advertise<robot_r::Vel>("/pod_vel", 1);
  
  // create the subscriber to /odom
  ros::Subscriber sub = nh.subscribe("/odom", 1, Cbkodom);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
  	// publish /pod_vel
  	pub.publish(msg1);
  	sleep(1);
  	ros::spinOnce();
  	loop_rate.sleep();
  }

  return 0;
}


