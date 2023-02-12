#include "ros/ros.h"
#include "robot_r/Counter.h"
#include "assignment_2_2022/PlanningActionResult.h"
#include <iostream>

// counter to count number off reached or cancelled targets and stat for which condition it is 
int counter_reach = 0;
int counter_cancel = 0;
int stat;

// call back function for /reaching_goal/result 
void callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg)
{
	ROS_INFO("I heard: [%d]", msg->status.status);
	std::cout<<std::endl;
	
	stat = msg->status.status;
	    
	if(stat==2)
	  {
	     counter_cancel++;
	  }
	else if(stat==3)
	  {
	     counter_reach++;
	  }    
	  
	ROS_INFO("Goal Reached!: [%d]", counter_reach);
	ROS_INFO("Goal Canceled!: [%d]", counter_cancel);
}

//callback to set values of /info_count topic
bool call_count(robot_r::Counter::Request &req, robot_r::Counter::Response &res)
{
	
	res.num_reached = counter_reach;
	res.num_cancelled = counter_cancel;
	
	return true;
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "info_counter");
	ros::NodeHandle n;
	
	// Subcriber to /reaching_goal/result to status of result 
	ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, callback);
	
	// Service to /info_counter to set values on service
	ros::ServiceServer service = n.advertiseService("/info_counter", call_count);

		
	ros::spin();
   return 0;
}
