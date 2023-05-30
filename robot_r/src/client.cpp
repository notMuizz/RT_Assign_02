/**
* \file client.cpp
* 
* \author Hafiz Muizz Ahmed Sethi
* \version 0.1
* \date 30/05/2023
* \details
*
*
* Discription:
*This node creates a ROS client that interacts with an action server.
*It prompts the user to set a new goal or cancel an existing goal. 
*It sends the goal to the action server and receives the status and result of the goal. 
*The loop continues until the user stops the program.
**/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2022/PlanningAction.h>
#include <iostream>

/**
 * @brief Main function for the client node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer representing the exit status.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");

  
  ros::NodeHandle nh; // Node initialization

 
  int flag = 0;  // To check whether the user asked to cancel or set a new goal.

  
 
  actionlib::SimpleActionClient<assignment_2_2022::PlanningAction> ac("reaching_goal", true);  // True causes the client to spin its own thread

  ROS_INFO("Waiting for action server to start.");
 
  ac.waitForServer(); // Will wait for an infinite time and Wait for the action server to start

  
  ros::Rate rate(1); // Set loop rate equal to 1 second

  while (ros::ok())
  {

    std::cout << std::endl;

    ROS_INFO("Press '1' to set a new goal or '0' to cancel a goal!");
    std::cin >> flag;

    
    if (flag) // Flag is '1' for a new goal and '0' to cancel the current goal
    {

      double param_x = 0.0;
      double param_y = 0.0;

      std::cout << "Please enter x_goal and y_goal" << std::endl;

      std::cin >> param_x >> param_y;

      ROS_INFO("Server Activated! Sending goal.");
      assignment_2_2022::PlanningGoal goal;

      goal.target_pose.pose.position.x = param_x;
      goal.target_pose.pose.position.y = param_y;

      
      ac.sendGoal(goal); // Send a goal to the action

      
      actionlib::SimpleClientGoalState state = ac.getState(); // Receive the goal status
      ROS_INFO("Goal Status: %s", state.toString().c_str());
      assignment_2_2022::PlanningResultConstPtr dis_msg = ac.getResult();
      std::cout << std::endl;
    }
    else
    {

      ROS_INFO("Client requested goal cancellation!");
      ac.cancelGoal();
      ROS_INFO("Goal has been cancelled!");
    }
  }

  return 0;
}

