#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cstdlib>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/GetLinkState.h>
#include <cmath>

// Global variable to store message data
geometry_msgs::Point global_CG_point;
sensor_msgs::Imu experienced_accel;

// Callback function
void cgCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // Update global variable with message data
    global_CG_point = *msg;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    experienced_accel = *msg;
}

int main(int argc, char **argv)
{
    geometry_msgs::Point global_ZMP_point;
    // Initialize ROS node
    ros::init(argc, argv, "locate_ZMP_with_marker");
    ros::NodeHandle n;

    std::cout << "\nPlease input the ROS topic to extract IMU data from: ";
    std::string imu_topic;
    std::cin >> imu_topic;

    ros::Subscriber sub_CG = n.subscribe("CG_Coordinates", 100, cgCallback);
    ros::Subscriber sub_Accel = n.subscribe(imu_topic, 100, imuCallback);

    ros::ServiceClient clientLinkState = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    gazebo_msgs::GetLinkState getLinkState;

    // Set update rate (in Hz)
    ros::Rate loop_rate(100); // Update every 10ms
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_ZMP", 100);
    ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("ZMP_Coordinates", 100);
    visualization_msgs::Marker marker;
    std::cout << "\nPlease input the reference frame where you would like the marker to be displayed in: ";
    std::string reference_frame;
    std::cin >> reference_frame;
    marker.header.frame_id = reference_frame;

    while (ros::ok())
    {
        // Display the point message values in the terminal
        ROS_INFO("CG Point message: x = %f, y = %f, z = %f", global_CG_point.x, global_CG_point.y, global_CG_point.z);
        ROS_INFO("Accel Point message: x = %f, y = %f, z = %f", experienced_accel.linear_acceleration.x, experienced_accel.linear_acceleration.y, experienced_accel.linear_acceleration.z);

        clientLinkState.call(getLinkState);
        getLinkState.request.link_name = "front_left_wheel_link"; //Note: Choose a wheel link's name for accurate z-positioning of ZMP 
        getLinkState.request.reference_frame = reference_frame;

        float btm_up_CG_point_z = (std::abs(getLinkState.response.link_state.pose.position.z) + 0.1651) + global_CG_point.z; // To get height of CG from ground up, (-0.1651) is the radius of the husky's wheels

        global_ZMP_point.x = (global_CG_point.x - ((btm_up_CG_point_z * (experienced_accel.linear_acceleration.x)) / 9.8));
        global_ZMP_point.y = (global_CG_point.y - ((btm_up_CG_point_z * (experienced_accel.linear_acceleration.y)) / 9.8));

        global_ZMP_point.z = (getLinkState.response.link_state.pose.position.z - 0.1651); //Note: (-0.1651) is the radius of the husky wheel, replace with radius of robot's wheel if not husky

        ROS_INFO("ZMP Point message: x = %f, y = %f, z = %f \n", global_ZMP_point.x, global_ZMP_point.y, global_ZMP_point.z);

        marker.header.stamp = ros::Time::now();
        marker.ns = "ZMP_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.pose.position.x = global_ZMP_point.x;
        marker.pose.position.y = global_ZMP_point.y;
        marker.pose.position.z = global_ZMP_point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);

        // Publish ZMP coordinates to a ros topic
        geometry_msgs::Point point;
        point.x = global_ZMP_point.x;
        point.y = global_ZMP_point.y;
        point.z = global_ZMP_point.z;

        // Publish the point message
        point_pub.publish(point);

        // Sleep for the remaining time until the next loop iteration
        loop_rate.sleep();

        // Spin once to handle callbacks
        ros::spinOnce();
    }

    return 0;
}
