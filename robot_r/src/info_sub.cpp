/**
* \file info_sub.cpp
* 
* \author Hafiz Muizz Ahmed Sethi
* \version 0.1
* \date 30/05/2023
*
* \details
*
*
* Subscribe To : <BR>
* 	reaching_goal/goal
*	/pod_vel
*
* Description:
*This code is a ROS node that subscribes to two topics: "/reaching_goal/goal" and "/pod_vel".
*It extracts the target position from messages received on the "/reaching_goal/goal" topic and calculates the average velocity and remaining distance based on position and velocity information received on the "/pod_vel" topic.
*The node continuously updates and prints these values as new messages are received.
**/

#include "ros/ros.h"
#include "robot_r/Vel.h"
#include "assignment_2_2022/PlanningActionGoal.h"
#include <cmath>
#include <unistd.h>
#include <iostream>

double t = 0.0;

double x_g, y_g, x, y, velocity_x, velocity_y, avg_velocity = 0, dist = 0, dist_left = 0;

double x_prev = 0.0, y_prev = 0.0, x_in, y_in;

/**
 * @brief Callback function for the subscriber to the /reaching_goal/goal topic.
 * @param msg Pointer to the received PlanningActionGoal message.
 */
void goalCbk(const assignment_2_2022::PlanningActionGoal::ConstPtr& msg)
{
    x_g = msg->goal.target_pose.pose.position.x;
    y_g = msg->goal.target_pose.pose.position.y;

    // ROS_INFO("Goal-Target@[%f], Goal-Speed@[%f]", x_g, y_g);
}

/**
 * @brief Callback function for the subscriber to the /pod_vel topic.
 * @param msg1 Pointer to the received Vel message.
 */
void posCbk(const robot_r::Vel::ConstPtr& msg1)
{
    // Set to the initial values when a new goal position starts
    if (x_prev != x_g || y_prev != y_g)
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
    avg_velocity = sqrt(pow(x - x_in, 2) + pow(y - y_in, 2)) / t;
    dist_left = sqrt(pow(x_g - x, 2) + pow(y_g - y, 2));

    // ROS_INFO("x@[%f], y@[%f]", x, y);
    // std::cout<<std::endl;

    ROS_INFO("Distance-Target@[%f], Average-Speed@[%f]", dist_left, avg_velocity);
    std::cout << std::endl;
    ROS_INFO("Time@[%f]: ", t);

    x_prev = x_g;
    y_prev = y_g;
}

/**
 * @brief Main function for the info_sub node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Integer representing the exit status.
 */
int main(int argc, char **argv)
{
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system
    ros::init(argc, argv, "info_sub");
    ros::NodeHandle n2, n3;
    // Subscriber to /reaching_goal/goal
    ros::Subscriber sub3 = n2.subscribe("reaching_goal/goal", 1, goalCbk);
    sleep(2);

    // Subscriber to /pod_vel
    ros::Subscriber sub4 = n3.subscribe("/pod_vel", 1, posCbk);

    ros::spin();
    return 0;
}

