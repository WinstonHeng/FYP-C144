#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h> // for using tf2::Quaternion
#include <tf2/LinearMath/Matrix3x3.h>  // for using tf2::Matrix3x3

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // Normalize the orientation quaternion
    tf2::Quaternion quat(msg->orientation.x, msg->orientation.y,
                         msg->orientation.z, msg->orientation.w);
    quat.normalize();

    // Convert the quaternion orientation to pitch angle
    double pitch, roll, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Print the pitch angle in degrees
    ROS_INFO("Pitch angle: %f degrees", pitch * 180.0 / M_PI);
    ROS_INFO("Roll angle: %f degrees", roll * 180.0 / M_PI);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_tilt_angle");
    ros::NodeHandle nh;

    std::cout << "\nPlease input the ROS topic to extract IMU data from: "; //Note: Use IMU topic that is not filtered
    std::string imu_topic;
    std::cin >> imu_topic;

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, imuCallback);

    ros::spin();

    return 0;
}
