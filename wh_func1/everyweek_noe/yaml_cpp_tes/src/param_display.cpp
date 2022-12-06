#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "param_display"); //rclcpp::init(argc, argv);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~"); //--------------> param node hundle
                            // ~ is relative path of ROS Param. not absolute path
  XmlRpc::XmlRpcValue member_list;
  pnh.getParam("member_list", member_list); //ros::this)node::getName(), ~


