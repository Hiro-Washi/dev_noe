#include "ros/ros.h"
#include "roscpp_ex/adder.h"

void charCB(const roscpp_ex::adder msg) {
    int result;
    result = msg.a + msg.b;
    printf("a:%d + b:%d = %d\n", msg.a, msg.b, result);
}

int main(int argc, char **argv) { // argument count , argument vector
                                 // int main(int a, char *b[])
    ros::init(argc, argv, "adder"); /////
    ros::NodeHandle nh;
    // if topic is updated, execute CB
    ros::Subscriber sub = nh.subscribe("pub_in", 1000, charCB);

    ros::spin();
    //return 0;
}
