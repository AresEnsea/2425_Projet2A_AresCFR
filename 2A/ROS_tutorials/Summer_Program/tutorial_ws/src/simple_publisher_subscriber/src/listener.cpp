#include "ros/ros.h"
#include "std_msgs/String.h"

// Callback function that processes the received message
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "listener");

  // Create a node handle
  ros::NodeHandle nh;

  // Subscribe to the "chatter" topic
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  // Spin to keep the node alive and processing callbacks
  ros::spin();

  return 0;
}

