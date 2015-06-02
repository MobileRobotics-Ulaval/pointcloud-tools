
#include <ros/ros.h>
#include <pointcloud_tools/CloudRecorder.h>

#define NODE_NAME "cloud_recorder"

int main(int argc, char**argv)
{
    ros::init(argc,argv, NODE_NAME);
    ros::NodeHandle n;

    CloudRecorder recorder(n);
    recorder.run();

    return 0;
}
