#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <control/Vel.h>

control::Vel msg1;

/**
 * \file client_sub.cpp
 * \brief Subscriber node for receiving odometry information and publishing position and velocity.
 */

/**
 * \brief Callback function for the odom topic.
 *description:
 *
 * This function is called whenever a new Odometry message is received on the /odom topic.
 * It extracts position and velocity information from the message and stores it in the custom Vel message.
 *
 */
void Cbkodom(const nav_msgs::Odometry::ConstPtr& msg){
   
   // Set values of the custom Vel message
   msg1.x = msg->pose.pose.position.x;
   msg1.y = msg->pose.pose.position.y;
   msg1.vel_x = msg->twist.twist.linear.x;
   msg1.vel_y = msg->twist.twist.linear.y;
   
   ROS_INFO("x position: [%f]; y position: [%f], x velocity: [%f]; y velocity: [%f]  ", msg1.x, msg1.y, msg1.vel_x, msg1.vel_y);
   
}

/**
 * \brief Main function of the client_sub node.
 *
 * This node subscribes to the /odom topic to receive odometry information and publishes position and velocity
 * information to the /pod_vel topic using the custom Vel message type.
 */
int main(int argc, char **argv) {

  // Initialize the ROS node
  ros::init(argc, argv, "client_sub");
  ros::NodeHandle nh;
  
  // Create a publisher for /pod_vel which publishes position and velocity
  ros::Publisher pub = nh.advertise<control::Vel>("/pod_vel", 1);

  // Create a subscriber for /odom topic with callback function Cbkodom
  ros::Subscriber sub = nh.subscribe("/odom", 1, Cbkodom);

  // Set the loop rate
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    // Publish the message to /pod_velqqqqqqqqq
    pub.publish(msg1);
    sleep(1);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


