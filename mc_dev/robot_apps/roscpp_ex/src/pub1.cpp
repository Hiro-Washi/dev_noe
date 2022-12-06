#include "ros/ros.h"
#include "roscpp_ex/adder.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "pub1");
    ros::NodeHandle nh;
    // pub1_pub = rospy.publish("roscpp_ex/adder","pub_in",1000)
    ros::Publisher pub = nh.advertise<roscpp_ex::adder>("pub_in", 1000);
    // 1pub / 1s
    ros::Rate loop_rate(1);
    // defi roscpp_ex::adder type Object
    // inst a,b in adder.msg
    roscpp_ex::adder msg;
    int count = 0;
    // untill ctrl+C, or get unconnected network, or other node do ros::shutdown(), or abandoned ros::NodeHandle
    while (ros::ok()){
        msg.a = count;
        msg.b = count;
        pub.publish(msg);
        printf("a = %d b = %d \n", msg.a, msg.b);
        ros::spinOnce(); //: access to CB (spin(): 1pub / 1access)
        loop_rate.sleep();
        count++;
    }
    //return 0;
}
