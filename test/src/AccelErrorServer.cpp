#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <test/IMUdata.h>


_Float64 x_orient_in, y_orient_in, z_orient_in, w_orient_in;
_Float64 x_orient_experienced, y_orient_experienced, z_orient_experienced, w_orient_experienced;

_Float64 x_accel_in, y_accel_in, z_accel_in;
_Float64 x_accel_experienced, y_accel_experienced, z_accel_experienced;

_Float64 x_angularvel_in, y_angularvel_in, z_angularvel_in;
_Float64 x_angularvel_experienced, y_angularvel_experienced, z_angularvel_experienced;


bool get_val(test::IMUdata::Request  &req, test::IMUdata::Response &res)
{
    
    ROS_INFO("sending back response");

    res.x_accel_out = x_accel_in;
    res.x_accel_experienced = x_accel_experienced;

    res.y_accel_out = y_accel_in;
    res.y_accel_experienced = y_accel_experienced;

    res.z_accel_out = z_accel_in;
    res.z_accel_experienced = z_accel_experienced;


    res.x_orient_out = x_orient_in;
    res.x_orient_experienced = x_orient_experienced;

    res.y_orient_out = y_orient_in;
    res.y_orient_experienced = y_orient_experienced;

    res.z_orient_out = z_orient_in;
    res.z_orient_experienced = z_orient_experienced;

    res.w_orient_out = w_orient_in;
    res.w_orient_experienced = w_orient_experienced;


    res.x_angularvel_out = x_angularvel_in;
    res.x_angularvel_experienced = x_angularvel_experienced;

    res.y_angularvel_out = y_angularvel_in;
    res.y_angularvel_experienced = y_angularvel_experienced;

    res.z_angularvel_out = z_angularvel_in;
    res.z_angularvel_experienced = z_angularvel_experienced;

    return true;

}


void motionGenerator(const  sensor_msgs::Imu::ConstPtr& msg)
{
  x_orient_in = msg->orientation.x;
  y_orient_in = msg->orientation.y;
  z_orient_in = msg->orientation.z;
  w_orient_in = msg->orientation.w;

  x_accel_in = msg->linear_acceleration.x;
  y_accel_in = msg->linear_acceleration.y;
  z_accel_in = (msg->linear_acceleration.z)+9.8;

  x_angularvel_in = msg->angular_velocity.x;
  y_angularvel_in = msg->angular_velocity.y;
  z_angularvel_in = msg->angular_velocity.z;
}

void imuCallback(const  sensor_msgs::Imu::ConstPtr& msg)
{
  x_orient_experienced = msg->orientation.x;
  y_orient_experienced = msg->orientation.y;
  z_orient_experienced = msg->orientation.z;
  w_orient_experienced = msg->orientation.w;

  x_accel_experienced = msg->linear_acceleration.x;
  y_accel_experienced = msg->linear_acceleration.y;
  z_accel_experienced = msg->linear_acceleration.z;

  x_angularvel_experienced = msg->angular_velocity.x;
  y_angularvel_experienced = msg->angular_velocity.y;
  z_angularvel_experienced = msg->angular_velocity.z;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_status_server");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/imu/data", 1000, imuCallback);
  ros::Subscriber sub2 = nh.subscribe("/accelerationPublish", 1000, motionGenerator);

  ros::ServiceServer service = nh.advertiseService("imu_status_server", get_val);
  ROS_INFO("Starting server...");
  ros::spin();

  return 0;
}