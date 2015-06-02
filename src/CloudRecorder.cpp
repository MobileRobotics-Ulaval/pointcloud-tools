
#include "pointcloud_tools/CloudRecorder.h"

CloudRecorder::CloudRecorder(ros::NodeHandle n)
{
    std::string sourceTopic = "";
    std::string destTopic = "";

    if(!n.getParam(FILE_FORMAT_PARAM, fileFormat)) fileFormat = DEFAULT_FORMAT;

    if(!n.getParam(SOURCE_TOPIC_PARAM, sourceTopic))
    {
        ROS_FATAL("Source topic not specified. Usage: "
                  "rosrun pointcloud_tools cloud_recorder "
                  "_from:=<SOURCE_TOPIC> [_to:=<DEST_TOPIC>] [_format:=<FILE_FORMAT>]");
        ros::shutdown();
    }

    subscriberTopic = n.subscribe(sourceTopic, 100, &CloudRecorder::cloudCallback, this);

    if(n.getParam(DEST_TOPIC_PARAM, destTopic))
    {
        publisherTopic = n.advertise<sensor_msgs::PointCloud2>(destTopic, 100);
    }
}

void CloudRecorder::run()
{

}

void CloudRecorder::cloudCallback(const sensor_msgs::PointCloud2 &msg)
{
}
