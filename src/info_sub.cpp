#include "ros/ros.h"
#include "control/Vel.h"
#include "assignment_2_2022/PlanningActionGoal.h"
#include <cmath>
#include <unistd.h>
#include <iostream>

double t = 0.0;
double x_g, y_g, x, y, vel_x, vel_y, avg_vel = 0, dist = 0, dist_left = 0;
double x_prev = 0.0, y_prev = 0.0, x_in, y_in;

/**
 * \file info_sub.cpp
 * \brief Node for subscribing to goal and position information and calculating average speed and remaining distance.
 * Subscribes to the /reaching_goal/goal and /pod_vel topics and calculates average speed and remaining distance to the goal.
 * Topics used: /reaching_goal/goal, /pod_vel
 */

/**
 * \brief Callback function for the subscriber /reaching_goal/goal.
 *
 * This function is called whenever a new PlanningActionGoal message is received on the /reaching_goal/goal topic.
 * It updates the target position.
 */
void goalCbk(const assignment_2_2022::PlanningActionGoal::ConstPtr& msg)
{
    x_g = msg->goal.target_pose.pose.position.x;
    y_g = msg->goal.target_pose.pose.position.y;
}

/**
 * \brief Callback function for the subscriber /pod_vel.
 *
 * This function is called whenever a new Vel message is received on the /pod_vel topic.
 * It calculates the average speed and remaining distance to the goal based on the current position.
 */
void posCbk(const control::Vel::ConstPtr& msg1)
{
    if (x_prev != x_g || y_prev != y_g)
    {
        t = 0;
        x_in = x;
        y_in = y;
    }

    t = t + 1;
    x = msg1->x;
    y = msg1->y;
    vel_x = msg1->vel_x;
    vel_y = msg1->vel_y;

    avg_vel = sqrt(pow(x - x_in, 2) + pow(y - y_in, 2)) / t;
    dist_left = sqrt(pow(x_g - x, 2) + pow(y_g - y, 2));

    ROS_INFO("Distance to Target: [%f], Average Speed: [%f]", dist_left, avg_vel);
}

/**
 * \brief Main function of the info_sub node.
 *
 * Initializes the ROS node, subscribes to the /reaching_goal/goal and /pod_vel topics,
 * and starts spinning and processing callbacks.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_sub");
    ros::NodeHandle n2, n3;

    ros::Subscriber sub3 = n2.subscribe("reaching_goal/goal", 1, goalCbk);
    sleep(2);

    ros::Subscriber sub4 = n3.subscribe("/pod_vel", 1, posCbk);

    ros::spin();
    
    return 0;
}



