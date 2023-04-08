#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <chrono>
#include <future>
#include <cmath>

float t = 0.0;
float vel = 0.0;

void timer() {
    for(int i=0;i>-1;++i)
    {
        t += ((2*M_PI)/1000);
        vel = cos(t);
        std::this_thread::sleep_for(std::chrono::milliseconds((10)));
    }
}

int main(int argc, char **argv)
{
    auto future = std::async(timer);
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);


    ros::Rate rate(50);

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        pub.publish(msg);
        ROS_INFO_STREAM("Sending velocity command:"<< " linear=" << msg.linear.x << " angular=" << msg.angular.z);
        rate.sleep();
    }
}

