#include <ros/ros.h>

#include "rgbd_risk/RGBD.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgbd_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    RGBD RGBD(node, private_nh); //instance of RGBD class

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce(); //invokes callback
        loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}