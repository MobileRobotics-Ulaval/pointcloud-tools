
#include <ros/ros.h>
#include "pointcloud_tools/CloudRecorderService.h"

#define NODE_NAME "cloud_recorder"

int main(int argc, char**argv)
{
    ros::init(argc,argv, NODE_NAME);
    ros::NodeHandle n;

    CloudRecorderService recorder(n);
    ros::spin();

    return 0;
}
