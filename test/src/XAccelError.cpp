#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  /*msg -> linear.acceleration.x
  float linearAccelX = msg
  geometry_msgs::Twist error;
  error.linear.x = 0.1 - linearAccelX;*/
  float error = 0.1 - (msg->linear_acceleration.x);
  ROS_INFO("Imu Linear x: [%f]", error);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/imu/data", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}