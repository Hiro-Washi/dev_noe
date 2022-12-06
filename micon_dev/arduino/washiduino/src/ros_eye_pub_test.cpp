#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "washi_eye1_pub");
    ros::NodeHandle nh;
    ros::Publisher eye_pub = nh.advertise<std_msgs::String>("eye_topic", 10);
    ros::Rate rate(6000);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "true"; //<<count
        msg.data == ss.str();

        ROS_INFO("%s", msg.data.c_str());////
        eys_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
        ++count;
    }
    return 0;
}
