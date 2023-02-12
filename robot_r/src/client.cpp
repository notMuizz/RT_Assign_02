#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include <iostream>

int main (int argc, char **argv)
{
  
  ros::init(argc, argv, "client");  
  
  // Node initialization
  ros::NodeHandle nh;
  
  // to check weather user asked to cancel or for new goal.
  int flag=0;
	
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("reaching_goal", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
    
  // set loop rate equal to 1 second
  ros::Rate rate(1);
  
  while(ros::ok()){
  
	  std::cout<<std::endl;
	  
	  ROS_INFO("Press : '1' for setting new goal, '0' for cancel a goal!");
	  std::cin>>flag;
	  
	  // flag is '1' to new goal and '0' for cancel current goal
	  if(flag)
	  {
	  
		  double param_x = 0.0;
		  double param_y = 0.0;
		  
		  std::cout<< "Please enter x_goal and y_goal"<<std::endl;
		  
		  std::cin>>param_x>>param_y;
		  
		  ROS_INFO("Server Activated! Sending goal.");
		  assignment_2_2022::PlanningGoal goal;

		  goal.target_pose.pose.position.x = param_x;
		  goal.target_pose.pose.position.y = param_y;
		  
		  // send a goal to the action
		  ac.sendGoal(goal);
		  
		  // recieved the goal status
		  actionlib::SimpleClientGoalState state = ac.getState();
	    	  ROS_INFO("Goal Status : %s",state.toString().c_str());	    	  
	    	  assignment_2_2022::PlanningResultConstPtr dis_msg= ac.getResult();
	          std::cout << std::endl;

	  }
	  else
	  {
	  
	  	ROS_INFO("Client requested for goal Cancellation!");
	  	ac.cancelGoal(); 
	  	ROS_INFO("Goal has been cancelled!");
	  
	  }
   
   }

  return 0;
}
