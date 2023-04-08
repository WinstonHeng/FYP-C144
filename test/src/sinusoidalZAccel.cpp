#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <cmath>
#include <sensor_msgs/Imu.h>

float t = 0.0;
float vel = 0.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::Imu>("accelerationPublish", 1000);

    ros::Rate rate(100); //100 messages per second, 1000 messages in 10seconds

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        sensor_msgs::Imu msg2;

        float omegaT = 2*M_PI*(1.0/10.0)*t;   // 10s per cycle hence frequency = 1/10, velocity = -cos(2*pi*f*t)
        vel = -cos(omegaT); 

        msg.linear.z = vel;
        msg2.linear_acceleration.z = 2*M_PI*(1.0/10.0)*sin(omegaT); // differentiate -cos(2pi*f*t) = 2pi*f*sin(2pi*f*t)
 
        pub.publish(msg);
        pub2.publish(msg2);
        ROS_INFO_STREAM("Sending velocity command:"<< " linear= " << msg.linear.z);
        t += 0.01 ;   // 10ms per loop, 10000ms per cycle
        rate.sleep();


    }
}

