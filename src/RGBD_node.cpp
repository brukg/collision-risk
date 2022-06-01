#include <ros/ros.h>

#include "rgbd_risk/RGBD.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgbd_node"); //node name
    ros::NodeHandle node; //initializes node 
    ros::NodeHandle private_nh("~");

    RGBD RGBD(node, private_nh); //instance of RGBD class

    ros::Rate loop_rate(10); //loop frequency

    while(ros::ok())
    {
        ros::spinOnce(); //invokes callback
        loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}