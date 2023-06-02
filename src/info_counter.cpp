#include "ros/ros.h"
#include "control/Counter.h"
#include "assignment_2_2022/PlanningActionResult.h"
#include <iostream>

int counter_reach = 0;  // Counter to count the number of reached goals
int counter_cancel = 0; // Counter to count the number of cancelled goals
int stat;

/**
 * \file info_counter.cpp
 * \brief Node for counting the number of reached and cancelled goals and providing the count through a service.
 *
 * This node subscribes to the `/reaching_goal/result` topic to receive goal status messages and updates
 * the counters based on the received status. It also provides a service at `/info_counter` to retrieve
 * the current count of reached and cancelled goals.
 *
 * Topics:
 *   - Subscribed Topic: `/reaching_goal/result`
 *     Message Type: `assignment_2_2022/PlanningActionResult`
 *     Description: Subscribes to the topic to receive goal status messages.
 *
 * Services:
 *   - Advertised Service: `/info_counter`
 *     Service Type: `control/Counter`
 *     Description: Provides a service to retrieve the current count of reached and cancelled goals.
 */

/**
 * \brief Callback function for the `/reaching_goal/result` topic.
 *
 * This function is called whenever a new `PlanningActionResult` message is received on the `/reaching_goal/result` topic.
 * It updates the counters based on the status of the goal and prints the current count of reached and cancelled goals.
 *
 * \param msg The received `PlanningActionResult` message.
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

    ROS_INFO("Goals Reached: [%d]", counter_reach);
    ROS_INFO("Goals Cancelled: [%d]", counter_cancel);
}

/**
 * \brief Callback function to set values of the `/info_counter` service response.
 *
 * This function is called when the `/info_counter` service is requested.
 * It sets the response with the current count of reached and cancelled goals.
 *
 * \param req The service request.
 * \param res The service response.
 * \return `true` if the service call was successful, `false` otherwise.
 */
bool call_count(control::Counter::Request &req, control::Counter::Response &res)
{
    res.num_reached = counter_reach;
    res.num_cancelled = counter_cancel;

    return true;
}

/**
 * \brief Main function of the `info_counter` node.
 *
 * Initializes the ROS node, creates a subscriber for the `/reaching_goal/result` topic,
 * creates a service server for the `/info_counter` service, and starts spinning and processing callbacks.
 *
 * \param argc The number of command-line arguments.
 * \param argv An array of command-line arguments.
 * \return The exit status of the program.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_counter");
    ros::NodeHandle n;

    // Subscribe to the `/reaching_goal/result` topic with the callback function `callback`
    ros::Subscriber sub = n.subscribe("/reaching_goal/result", 1, callback);

    // Advertise the `/info_counter` service with the callback function `call_count`
// Service to /info_counter to set values on service
	ros::ServiceServer service = n.advertiseService("/info_counter", call_count);

		
	ros::spin();
   return 0;
}
