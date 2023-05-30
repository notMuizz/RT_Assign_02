/**
* \file info_counter.cpp
*  
* \author Hafiz Muizz Ahmed Sethi
* \version 0.1
* \date 30/05/2023
*
* \details
*
* Subscribes To: <BR>
*	/reaching_goal/result
*
* Services To: <BR>
* 	/info_counter
* 
* Description:
*This code is a ROS node that keeps track of the number of reached and canceled goals. 
*It subscribes to the "/reaching_goal/result" topic to receive status messages and updates the counters accordingly. 
*It also provides a service "/info_counter" to retrieve the current counts of reached and canceled goals.
**/

#include "ros/ros.h"
#include "robot_r/Counter.h"
#include "assignment_2_2022/PlanningActionResult.h"
#include <iostream>

// Counter to count the number of reached or cancelled targets and status for which condition it is
int counter_reach = 0;
int counter_cancel = 0;
int stat;

/**
 * @brief Callback function for the /reaching_goal/result topic.
 * @param msg Pointer to the received PlanningActionResult message.
 */
void callback(const assignment_2_2022::PlanningActionResult::ConstPtr& msg)
{
    ROS_INFO("I heard: [%d]", msg->status.status);
    std::cout << std::endl;

    stat = msg->status.status;

    if (stat == 2)
    {
        counter_cancel++;
    }
    else if (stat == 3)
    {
        counter_reach++;
    }

    ROS_INFO("Goal Reached!: [%d]", counter_reach);
    ROS_INFO("Goal Canceled!: [%d]", counter_cancel);
}

/**
 * @brief Service callback function to set values of the /info_counter topic.
 * @param req Request object for the Counter service.
 * @param res Response object for the Counter service.
 * @return True if the service call was successful, false otherwise.
 */
bool call_count(robot_r::Counter::Request &req, robot_r::Counter::Response &res)
{
    res.num_reached = counter_reach;
    res.num_cancelled = counter_cancel;

    return true;
}

/**
 * @brief Main function for the info_counter node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer representing the exit status.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_counter");
    ros::NodeHandle n;

    // Subscriber to /reaching_goal/result for the status of the result
    ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, callback);

    // Service to /info_counter to set values on service
    ros::ServiceServer service = n.advertiseService("/info_counter", call_count);

    ros::spin();
    return 0;
}

