
#ifndef CLOUD_RECORDER_H
#define CLOUD_RECORDER_H

#include <exception>
#include <string>

#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointcloud_tools/CloudRecorder.h>

#define SERVICE_NAME "cloud_recorder"
#define FILE_FORMAT_PARAM "format"
#define DEST_TOPIC_PARAM "republish"
#define DEFAULT_FORMAT "vtk"

class CloudRecorderService {
public:
    CloudRecorderService(ros::NodeHandle n);
    bool record(pointcloud_tools::CloudRecorderRequest& req,
                pointcloud_tools::CloudRecorderResponse& res);
    static void saveAsVTK(std::string name, sensor_msgs::PointCloud2& cloud);
    static void saveAsPCD(std::string name, sensor_msgs::PointCloud2& cloud);

private:
    ros::Publisher publisherTopic;
    ros::ServiceServer service;
    std::string fileFormat;
    bool republish;
};

#endif
