#include <kdl_parser/kdl_parser.hpp> 
 
KDL::Tree my_tree;
if (!kdl_parser::treeFromFile("/home/winston/husky_ur5e_robot.urdf", my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }