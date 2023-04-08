#include "ros/ros.h"
#include <test/IMUdata.h>
#include <cstdlib>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imu_client_node");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<test::IMUdata>("imu_status_server");
    test::IMUdata srv;
    client.call(srv);

    ROS_INFO_STREAM("Linear X-acceleration Input: " << srv.response.x_accel_out);
    ROS_INFO_STREAM("Linear X-acceleration experienced: " << srv.response.x_accel_experienced);
    ROS_INFO_STREAM("Linear X-acceleration Error: " << (srv.response.x_accel_out-srv.response.x_accel_experienced) << "\n");

    ROS_INFO_STREAM("Linear Y-acceleration Input: " << srv.response.y_accel_out);
    ROS_INFO_STREAM("Linear Y-acceleration experienced: " << srv.response.y_accel_experienced);
    ROS_INFO_STREAM("Linear Y-acceleration Error: " << (srv.response.y_accel_out-srv.response.y_accel_experienced) << "\n");

    ROS_INFO_STREAM("Linear Z-acceleration Input: " << srv.response.z_accel_out);
    ROS_INFO_STREAM("Linear Z-acceleration experienced: " << srv.response.z_accel_experienced);
    ROS_INFO_STREAM("Linear Z-acceleration Error: " << (srv.response.z_accel_out-srv.response.z_accel_experienced) << "\n");

    ROS_INFO_STREAM("X-Orientation Input: " << srv.response.x_orient_out);
    ROS_INFO_STREAM("X-Orientation experienced: " << srv.response.x_orient_experienced);
    ROS_INFO_STREAM("X-Orientation Error: " << (srv.response.x_orient_out-srv.response.x_orient_experienced) << "\n");

    ROS_INFO_STREAM("Y-Orientation Input: " << srv.response.y_orient_out);
    ROS_INFO_STREAM("Y-Orientation experienced: " << srv.response.y_orient_experienced);
    ROS_INFO_STREAM("Y-Orientation Error: " << (srv.response.y_orient_out-srv.response.y_orient_experienced) << "\n");

    ROS_INFO_STREAM("Z-Orientation Input: " << srv.response.z_orient_out);
    ROS_INFO_STREAM("Z-Orientation experienced: " << srv.response.z_orient_experienced);
    ROS_INFO_STREAM("Z-Orientation Error: " << (srv.response.z_orient_out-srv.response.z_orient_experienced) << "\n");

    ROS_INFO_STREAM("W-Orientation Input: " << srv.response.w_orient_out);
    ROS_INFO_STREAM("W-Orientation experienced: " << srv.response.w_orient_experienced);
    ROS_INFO_STREAM("W-Orientation Error: " << (srv.response.w_orient_out-srv.response.w_orient_experienced) << "\n");

    ROS_INFO_STREAM("Angular X-acceleration Input: " << srv.response.x_angularvel_out);
    ROS_INFO_STREAM("Angular X-acceleration experienced: " << srv.response.x_angularvel_experienced);
    ROS_INFO_STREAM("Angular X-acceleration Error: " << (srv.response.x_angularvel_out-srv.response.x_angularvel_experienced) << "\n");

    ROS_INFO_STREAM("Angular Y-acceleration Input: " << srv.response.y_angularvel_out);
    ROS_INFO_STREAM("Angular Y-acceleration experienced: " << srv.response.y_angularvel_experienced);
    ROS_INFO_STREAM("Angular Y-acceleration Error: " << (srv.response.y_angularvel_out-srv.response.y_angularvel_experienced) << "\n");

    ROS_INFO_STREAM("Angular Z-acceleration Input: " << srv.response.z_angularvel_out);
    ROS_INFO_STREAM("Angular Z-acceleration experienced: " << srv.response.z_angularvel_experienced);
    ROS_INFO_STREAM("Angular Z-acceleration Error: " << (srv.response.z_angularvel_out-srv.response.z_angularvel_experienced) << "\n");

}