#include <ros/ros.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetLinkProperties.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locate_CG_with_marker");
  ros::NodeHandle n;

  // Create publisher for visualization marker
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_CG", 100);
  ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("CG_Coordinates", 100);

  // Create service clients
  ros::ServiceClient clientModelProperties = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
  ros::ServiceClient clientLinkProperties = n.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
  ros::ServiceClient clientLinkState = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  ros::ServiceClient clientWorldProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

  // Retrieve Model Properties
  gazebo_msgs::GetWorldProperties getWorldProperties;
  clientWorldProperties.call(getWorldProperties);
  std::string modelNamesInGazebo[getWorldProperties.response.model_names.size()];

  std::cout << "The models that have been spawned are: \n";
  for (int i = 0; i < getWorldProperties.response.model_names.size(); i++)
  {
    std::cout << getWorldProperties.response.model_names[i] << "\n";
    modelNamesInGazebo[i] = getWorldProperties.response.model_names[i];
  }

  // Asks for user input for the model to be calculated for its CG
  std::cout << "\nPlease input the model name that is to be calculated: ";
  std::string model_name;
  std::cin >> model_name;

  std::string *p;
  p = std::find(modelNamesInGazebo, modelNamesInGazebo + getWorldProperties.response.model_names.size(), model_name);
  if (p != modelNamesInGazebo + getWorldProperties.response.model_names.size())
  {
    std::cout << "Model found in Gazebo: " << *p << '\n';
  }
  else
  {
    std::cout << "Model not found in Gazebo\n";
    return 1;
  }

  // Create visualization marker message
  visualization_msgs::Marker marker;
  // Asks for user input for the model to be calculated for its CG
  std::cout << "\nPlease input the reference frame where you would like the marker to be displayed in: ";
  std::string reference_frame;
  std::cin >> reference_frame;
  marker.header.frame_id = reference_frame;

  // Set update rate (in Hz)
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    // Retrieve model properties and link information in a single call
    gazebo_msgs::GetModelProperties getModelProperties;
    getModelProperties.request.model_name = model_name;
    clientModelProperties.call(getModelProperties);

    std::string bodyNames[getModelProperties.response.body_names.size()];
    float massOfLink[getModelProperties.response.body_names.size()];
    float xCGCoordinates[getModelProperties.response.body_names.size()];
    float yCGCoordinates[getModelProperties.response.body_names.size()];
    float zCGCoordinates[getModelProperties.response.body_names.size()];
    float xBaseLinkCoordinates[getModelProperties.response.body_names.size()];
    float yBaseLinkCoordinates[getModelProperties.response.body_names.size()];
    float zBaseLinkCoordinates[getModelProperties.response.body_names.size()];

    for (int i = 0; i < getModelProperties.response.body_names.size(); i++)
    {
      bodyNames[i] = getModelProperties.response.body_names[i];
      gazebo_msgs::GetLinkProperties getLinkProperties;
      gazebo_msgs::GetLinkState getLinkState;
      getLinkProperties.request.link_name = bodyNames[i];
      getLinkState.request.link_name = bodyNames[i];
      getLinkState.request.reference_frame = reference_frame;

      clientLinkProperties.call(getLinkProperties);
      clientLinkState.call(getLinkState);

      // Retrieve mass of link from link field
      massOfLink[i] = getLinkProperties.response.mass;

      xCGCoordinates[i] = getLinkProperties.response.com.position.x;
      yCGCoordinates[i] = getLinkProperties.response.com.position.y;
      zCGCoordinates[i] = getLinkProperties.response.com.position.z;

      xBaseLinkCoordinates[i] = getLinkState.response.link_state.pose.position.x;
      yBaseLinkCoordinates[i] = getLinkState.response.link_state.pose.position.y;
      zBaseLinkCoordinates[i] = getLinkState.response.link_state.pose.position.z;
    }
    // Calculate CG coordinates for entire model
    float totalMass = 0;
    float xTotal = 0;
    float yTotal = 0;
    float zTotal = 0;
    for (int i = 0; i < getModelProperties.response.body_names.size(); i++)
    {
      xTotal += massOfLink[i] * (xBaseLinkCoordinates[i] + xCGCoordinates[i]);
      yTotal += massOfLink[i] * (yBaseLinkCoordinates[i] + yCGCoordinates[i]);
      zTotal += massOfLink[i] * (zBaseLinkCoordinates[i] + zCGCoordinates[i]);
      totalMass += massOfLink[i];
    }

    float xModelCG = xTotal / totalMass;
    float yModelCG = yTotal / totalMass;
    float zModelCG = zTotal / totalMass;

    std::cout << "The CG for the model is at: (" << xModelCG << ", " << yModelCG << ", " << zModelCG << ")\n";

    marker.header.stamp = ros::Time::now();
    marker.ns = "CG_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.position.x = xModelCG;
    marker.pose.position.y = yModelCG;
    marker.pose.position.z = zModelCG;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration();

    // Publish marker message
    marker_pub.publish(marker);

    geometry_msgs::Point point;
    point.x = xModelCG;
    point.y = yModelCG;
    point.z = zModelCG;

    // Publish the point message
    point_pub.publish(point);

    // Sleep for the desired update rate
    loop_rate.sleep();
  }

  return 0;
}