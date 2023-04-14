#include <cstdlib>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <iostream>
#include <deque>

const int BUFFER_SIZE = 15;
std::deque<sensor_msgs::Imu> buffer;
sensor_msgs::Imu average_imu;

void calculateMovingAverage()
{
    average_imu.linear_acceleration.x = 0;
    average_imu.linear_acceleration.y = 0;
    average_imu.linear_acceleration.z = 0;
    average_imu.orientation.x = 0;
    average_imu.orientation.y = 0;
    average_imu.orientation.z = 0;

    for (const auto &imu : buffer)
    {
        average_imu.linear_acceleration.x += imu.linear_acceleration.x;
        average_imu.linear_acceleration.y += imu.linear_acceleration.y;
        average_imu.linear_acceleration.z += imu.linear_acceleration.z;
        average_imu.orientation.x += imu.orientation.x;
        average_imu.orientation.y += imu.orientation.y;
        average_imu.orientation.z += imu.orientation.z;
    }

    average_imu.linear_acceleration.x /= buffer.size();
    average_imu.linear_acceleration.y /= buffer.size();
    average_imu.linear_acceleration.z /= buffer.size();
    average_imu.orientation.x /= buffer.size();
    average_imu.orientation.y /= buffer.size();
    average_imu.orientation.z /= buffer.size();
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // Add new data point to the buffer
    buffer.push_back(*msg);

    // Remove the oldest data point if the buffer is full
    if (buffer.size() > BUFFER_SIZE)
    {
        buffer.pop_front();
    }

    // Calculate the moving average of the data points in the buffer
    calculateMovingAverage();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_filter_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    std::cout << "\nPlease input the ROS topic to extract IMU data from: ";
    std::string imu_topic;
    std::cin >> imu_topic;

    std::cout << "\n The filtered ROS topic is named '/filtered_imu'" << std::endl;

    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imuCallback);

    ros::Publisher filtered_imu_pub = nh.advertise<sensor_msgs::Imu>("filtered_imu", 1);

    while (ros::ok())
    {
        filtered_imu_pub.publish(average_imu);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
