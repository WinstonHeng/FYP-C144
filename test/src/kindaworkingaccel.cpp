#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <cmath>

float t = 0.0;
float vel = 0.0;

float timer()
{
    for(int i=0;i>-1;++i)
        {
            vel = -cos(2*M_PI*(1.0/10.0)*t);
            t += (0.01);
            return vel;
        }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("accelerationPublish", 1000);

    ros::Rate rate(100); //100 messages per second, 1000 messages in 10seconds

    while (ros::ok())
    {
        geometry_msgs::Twist msg;
        geometry_msgs::Twist msg2;

        /*timer();    // 10ms per loop, 10000ms per cycle

        float omegaT = 2*M_PI*(1.0/10.0)*t;   // 10s per cycle hence frequency = 1/10, velocity = -cos(2*pi*f*t)
        vel = -cos(omegaT); 

        //float pubvel = velocityInput();
        msg.linear.x = vel;
        msg2.linear.x = 2*M_PI*(1.0/10.0)*sin(omegaT); // differentiate -cos(2pi*f*t) = 2pi*f*sin(2pi*f*t)*/

        msg.linear.x = timer();
 
        pub.publish(msg);
        pub2.publish(msg2);
        ROS_INFO_STREAM("Sending velocity command:"<< " linear=" << msg.linear.x << " angular=" << msg.angular.z);
        rate.sleep();


    }
}

