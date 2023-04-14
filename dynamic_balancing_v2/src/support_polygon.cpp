#include <limits>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

std::vector<geometry_msgs::Point> support_polygon;
geometry_msgs::Point zmp;
bool WPcallback_called = false;
bool ZMPcallback_called = false;

void wheelPositionsCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    support_polygon.push_back(*msg);

    // Calculate the center of the support polygon.
    geometry_msgs::Point center;
    center.x = 0.0;
    center.y = 0.0;
    for (int i = 0; i < support_polygon.size(); i++)
    {
        center.x += support_polygon[i].x;
        center.y += support_polygon[i].y;
    }
    center.x /= support_polygon.size();
    center.y /= support_polygon.size();

    // Define a struct that will store the angle between each pair of points and the corresponding point.
    struct AnglePoint
    {
        double angle;
        geometry_msgs::Point point;
    };

    // Calculate the angle between each pair of points and the center of the support polygon.
    std::vector<AnglePoint> angle_points;
    for (int i = 0; i < support_polygon.size(); i++)
    {
        geometry_msgs::Point p = support_polygon[i];
        double dx = p.x - center.x;
        double dy = p.y - center.y;
        double angle = std::atan2(dy, dx);
        AnglePoint ap;
        ap.angle = angle;
        ap.point = p;
        angle_points.push_back(ap);
    }

    // Sort the points based on their angle.
    std::sort(angle_points.begin(), angle_points.end(), [](const AnglePoint &ap1, const AnglePoint &ap2)
              { return ap1.angle < ap2.angle; });

    // Store the sorted points in the support_polygon vector.
    support_polygon.clear();
    for (int i = 0; i < angle_points.size(); i++)
    {
        support_polygon.push_back(angle_points[i].point);
    }
    WPcallback_called = true;
}

void ZMPCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    zmp.x = msg->x;
    zmp.y = msg->y;
    ZMPcallback_called = true;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node and create a subscriber to the /wheel_positions topic
    ros::init(argc, argv, "zmp_distance");
    ros::NodeHandle nh;
    ros::Subscriber sub_wheel_position = nh.subscribe("/wheel_positions", 100, wheelPositionsCallback);
    ros::Subscriber sub_ZMP_coordinates = nh.subscribe("ZMP_Coordinates", 100, ZMPCallback);
    ros::Rate rate(20);
    double shortest_distance = std::numeric_limits<double>::max();

    zmp.x = 0.0;
    zmp.y = 0.0;
    int unstable_count = 0;

    while (ros::ok())
    {
        shortest_distance = std::numeric_limits<double>::max();
        double max_distance = 0;
        double ZMP_distance_to_edge;
        if (WPcallback_called)
        {

            for (int i = 0; i < support_polygon.size(); i++)
            {
                geometry_msgs::Point p1 = support_polygon[i];
                geometry_msgs::Point p2 = support_polygon[(i + 1) % support_polygon.size()];
                ZMP_distance_to_edge = std::abs((p2.y - p1.y) * zmp.x - (p2.x - p1.x) * zmp.y + p2.x * p1.y - p2.y * p1.x) /
                                       std::sqrt((p2.y - p1.y) * (p2.y - p1.y) + (p2.x - p1.x) * (p2.x - p1.x));
                if (ZMP_distance_to_edge < shortest_distance)
                {
                    shortest_distance = ZMP_distance_to_edge;
                }
            }

            // The shortest_distance variable now holds the shortest distance from the ZMP to the edge of the support polygon.
            std::cout << "Shortest distance from ZMP to edge of polygon: " << shortest_distance << std::endl;

            // Calculate the center of the support polygon.
            geometry_msgs::Point center;
            center.x = 0.0;
            center.y = 0.0;
            for (int i = 0; i < support_polygon.size(); i++)
            {
                center.x += support_polygon[i].x;
                center.y += support_polygon[i].y;
            }
            center.x /= support_polygon.size();
            center.y /= support_polygon.size();

            double ZMP_distance_to_center = sqrt(pow((zmp.x - center.x), 2) + pow((zmp.y - center.y), 2));

            // Calculate the maximum distance from the center of the support polygon to the furthest point of the support polygon.

            for (int i = 0; i < support_polygon.size(); i++)
            {
                geometry_msgs::Point p = support_polygon[i];
                double distance_from_center = std::sqrt((p.x - center.x) * (p.x - center.x) + (p.y - center.y) * (p.y - center.y));
                if (distance_from_center > max_distance)
                {
                    max_distance = distance_from_center;
                }
            }

            if (ZMP_distance_to_center > max_distance)
            {
                ROS_INFO("Robot's ZMP has exceeded support polygon, please check on robot.");
                // Output the distance between ZMP (outside) and edge of support polygon
                double distance_percent = (ZMP_distance_to_center / max_distance) * 100.0;
                std::cout << "Distance outside edge of support polygon (%): " << (ZMP_distance_to_center - max_distance) << " (" << distance_percent << "%)" << std::endl;
                unstable_count++;
                std::cout << "Number of times robot was unstable: " << unstable_count << std::endl;
            }

            else
            {
                std::cout << "Max distance to the edge of polygon (From center of polygon): " << max_distance << std::endl;
                // Output the shortest distance as a percentage of the maximum distance.
                double distance_percent = (max_distance - shortest_distance) / max_distance * 100.0;
                std::cout << "Distance left to edge of support polygon (%): " << shortest_distance << " (" << distance_percent << "%)" << std::endl;
            }
        }

        support_polygon.clear();
        ros::spinOnce();
        rate.sleep();
    }
}