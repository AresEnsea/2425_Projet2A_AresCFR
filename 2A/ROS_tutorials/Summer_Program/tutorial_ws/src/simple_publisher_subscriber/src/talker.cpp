#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "talker");

  // Create a node handle
  ros::NodeHandle nh;

  // Create a publisher object
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  // Set the loop frequency
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    // Create a message object
    std_msgs::String msg;

    // Create a string to send as data
    std::stringstream ss;
    ss << "Hello ROS World " << count;
    msg.data = ss.str();

    // Log the message to the console
    ROS_INFO("%s", msg.data.c_str());

    // Publish the message
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
