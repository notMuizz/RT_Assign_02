#include "ros/ros.h"
#include "robot_r/Vel.h"
#include "assignment_2_2022/PlanningActionGoal.h"
#include <cmath>
#include <unistd.h>
#include <iostream>

double t =0.0;

double x_g, y_g, x, y, velocity_x, velocity_y, avg_velocity=0, dist=0, dist_left=0;

double x_prev = 0.0, y_prev=0.0, x_in, y_in;

// callback function for subsriber /sub3
void goalCbk(const assignment_2_2022::PlanningActionGoal::ConstPtr& msg)
{
	x_g = msg->goal.target_pose.pose.position.x;
	y_g = msg->goal.target_pose.pose.position.y;
	
	//ROS_INFO("Goal-Target@[%f], Goal-Speed@[%f]", x_g, y_g);
}


// callback function for subsriber /sub4
void posCbk(const robot_r::Vel::ConstPtr& msg1)
{	
	// Set to the initial values of when a new goal position starts
	if(x_prev != x_g || y_prev != y_g)
	{
	 t = 0;
	 x_in = x;
	 y_in = y;
	}
	
	t = t + 1;
	x = msg1->x;
	y = msg1->y;
	velocity_x = msg1->velocity_x;
	velocity_y = msg1->velocity_y;
	
	// Calculate average velocity and distance
	avg_velocity = sqrt(pow(x-x_in,2) + pow(y-y_in,2)) / t;
	dist_left = sqrt(pow(x_g - x,2) + pow(y_g-y,2));
	
	//ROS_INFO("x@[%f], y@[%f]", x, y);
	//std::cout<<std::endl;
	
	ROS_INFO("Distance-Target@[%f], Average-Speed@[%f]", dist_left, avg_velocity);
	std::cout<<std::endl;
	ROS_INFO("Timd@[%f]: " , t);
	
	x_prev = x_g;
	y_prev = y_g;
}


int main (int argc, char **argv)
{
// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
	ros::init(argc, argv, "info_sub");  
	ros::NodeHandle n2, n3;
	// Subsriber to /reaching_goal/goal 
	ros::Subscriber sub3 = n2.subscribe("reaching_goal/goal", 1, goalCbk);
	sleep(2);
	
	// Subsriber to /pod_vel
	ros::Subscriber sub4 = n3.subscribe("/pod_vel", 1, posCbk);
	
	ros::spin();
	return 0;
}
