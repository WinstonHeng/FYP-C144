#include <ros/ros.h>
#include <urdf/model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "wheel_position_finder");
    ros::NodeHandle n;

    // Get the name of the robot model from the parameter server
    std::string robot_model;
    std::vector<std::string> link_names;

    std::cout << "\nPlease input the reference frame with respect to the wheels: ";
    std::string source_frame;
    std::cin >> source_frame;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Rate rate(100);

    if (!n.getParam("/robot_description", robot_model))
    {
        ROS_ERROR("Failed to get robot model from parameter server");
        return 1;
    }

    // Parse the robot model using the urdf::Model class
    urdf::Model model;
    if (!model.initString(robot_model))
    {
        ROS_ERROR("Failed to parse robot model");
        return 1;
    }

    // Set the word to search for in the link names
    std::string search_word = "wheel";

    // Search for a link that contains the search word
    for (const auto &link : model.links_)
    {
        // Check if the link name contains the search word
        if (link.second->name.find(search_word) != std::string::npos)
        {
            // Print the link name
            ROS_INFO_STREAM("Link '" << link.second->name << "' contains the word '" << search_word << "'");
            link_names.push_back(link.second->name);
        }
    }

    geometry_msgs::TransformStamped transform;
    ros::Publisher wheel_position_publisher = n.advertise<geometry_msgs::Point>("/wheel_positions", 100);

    while (ros::ok())
    {
        for (int i = 0; i < link_names.size(); i++)
        {
            std::string target_frame = link_names[i];
            if (tf_buffer.canTransform(source_frame, target_frame, ros::Time(0)))
            {
                try
                {
                    transform = tf_buffer.lookupTransform(source_frame, target_frame, ros::Time(0));
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("%s", ex.what());
                    continue;
                }

                // Print the position and orientation of the link
                ROS_INFO_STREAM("Position of link '" << target_frame << "' relative to frame '" << source_frame << "':");

                geometry_msgs::Point msg;
                msg.x = transform.transform.translation.x;
                msg.y = transform.transform.translation.y;
                msg.z = transform.transform.translation.z - 0.1651;

                // Publish the message
                wheel_position_publisher.publish(msg);

                ROS_INFO_STREAM("  point x: " << msg.x);
                ROS_INFO_STREAM("  point y: " << msg.y);
                ROS_INFO_STREAM("  point z: " << msg.z << "\n");
            }
        }
        rate.sleep();
    }
}
