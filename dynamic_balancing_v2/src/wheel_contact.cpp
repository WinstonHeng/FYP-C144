#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>
#include <stdio.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Pose odom_pose, gazebo_pose;
double prev_gazebo_x, prev_odom_x, prev_gazebo_y, prev_odom_y;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_pose = msg->pose.pose;
}

void gazeboCallback(const gazebo_msgs::ModelStates &msg)
{

    int index = -1;
    for (unsigned int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == "robot")
        {
            index = i;
            break;
        }
    }
    if (index != -1)
    {
        gazebo_pose = msg.pose[index];
    }
}

void checkPositions()
{
    double gazebo_dx = gazebo_pose.position.x - prev_gazebo_x;
    double odom_dx = odom_pose.position.x - prev_odom_x;
    double real_dx = gazebo_dx - odom_dx;

    double gazebo_dy = gazebo_pose.position.y - prev_gazebo_y;
    double odom_dy = odom_pose.position.y - prev_odom_y;
    double real_dy = gazebo_dy - odom_dy;

    if (std::abs(gazebo_dx) < std::abs(0.50 * (odom_dx)))
    {

        std::cout << "X-Position difference is too large: " << std::abs(real_dx) << std::endl;
        std::cout << "Robot was expected to move: " << std::abs(odom_dx) << "m in x-direction" << std::endl;
        std::cout << "But robot has only moved: " << std::abs(gazebo_dx) << "m in x-direction" << std::endl;
    }

    else if (std::abs(gazebo_dy) < std::abs(0.50 * (odom_dy)))
    {

        std::cout << "Y-Position difference is too large: " << std::abs(real_dy) << std::endl;
        std::cout << "Robot was expected to move: " << std::abs(odom_dy) << "m in y-direction" << std::endl;
        std::cout << "But robot has only moved: " << std::abs(gazebo_dy) << "m in y-direction" << std::endl;
    }

    else
    {
        std::cout << "Robot is moving along just fine!" << std::endl;
    }

    prev_gazebo_x = gazebo_pose.position.x;
    prev_odom_x = odom_pose.position.x;
    prev_gazebo_y = gazebo_pose.position.y;
    prev_odom_y = odom_pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_checker");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/husky_velocity_controller/odom", 10, odomCallback);
    ros::Subscriber gazebo_sub = nh.subscribe("/gazebo/model_states", 10, gazeboCallback);

    while (ros::ok())
    {
        static ros::Time last_time = ros::Time::now(); // initialize the last_time variable on first call
        ros::Time current_time = ros::Time::now();
        double time_diff = (current_time - last_time).toSec();

        if (time_diff > 0.5) // check if 0.5 second has passed since the last update
        {
            checkPositions();
            last_time = current_time;
        }

        ros::spinOnce(); // process any incoming messages and service requests
    }

    return 0;
}
