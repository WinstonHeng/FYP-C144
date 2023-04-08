#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include "test/IMUdata.h"


_Float64 x_orient_in, y_orient_in, z_orient_in;
_Float64 x_orient_experienced, y_orient_experienced, z_orient_experienced;

_Float64 x_accel_in, y_accel_in, z_accel_in;
_Float64 x_accel_experienced, y_accel_experienced, z_accel_experienced;

_Float64 x_angularvel_in, y_angularvel_in, z_angularvel_in;
_Float64 x_angularvel_experienced, y_angularvel_experienced, z_angularvel_experienced;


bool get_val(test::IMUdata::Request  &req, test::IMUdata::Response &res)
{
    
    ROS_INFO("sending back response");
    ROS_INFO("Imu Linear x: [%f]", x_orient_in);
    res.x_orient_out = x_orient_in;
    res.x_accel_out = x_accel_in - x_accel_experienced;
    return true;

}


void imuCallback(const  sensor_msgs::Imu::ConstPtr& msg)
{
  x_orient_in = msg->orientation.x;
  y_orient_in = msg->orientation.y;
  z_orient_in = msg->orientation.z;

  x_accel_in = msg->linear_acceleration.x;
  y_accel_in = msg->linear_acceleration.y;
  z_accel_in = msg->linear_acceleration.z;
}

void motionGenerator_X_accel(const  sensor_msgs::Imu::ConstPtr& msg)
{
  x_accel_experienced = msg->linear_acceleration.x;
  y_accel_experienced = msg->linear_acceleration.y;
  z_accel_experienced = msg->linear_acceleration.z;


}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_status_server");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/imu/data", 1000, imuCallback);

  ros::NodeHandle nh2;
  ros::Subscriber sub2 = nh2.subscribe("/accelerationPublish", 1000, motionGenerator_X_accel);


  ros::ServiceServer service = nh.advertiseService("imu_status_server", get_val);
  ROS_INFO("Starting server...");
  ros::spin();

  return 0;
}