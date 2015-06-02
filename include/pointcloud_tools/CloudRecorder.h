
#ifndef CLOUD_RECORDER_H
#define CLOUD_RECORDER_H

#include <exception>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#define FILE_FORMAT_PARAM "format"
#define SOURCE_TOPIC_PARAM "from"
#define DEST_TOPIC_PARAM "to"
#define DEFAULT_FORMAT "vtk"

class CloudRecorder {
public:
    CloudRecorder(ros::NodeHandle n);
    void run();
    void cloudCallback(const sensor_msgs::PointCloud2& msg);

private:
    ros::Subscriber subscriberTopic;
    ros::Publisher publisherTopic;
    std::string fileFormat;
};

#endif
