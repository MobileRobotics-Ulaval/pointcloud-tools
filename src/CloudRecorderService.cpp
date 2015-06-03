
#include "pointcloud_tools/CloudRecorderService.h"

CloudRecorderService::CloudRecorderService(ros::NodeHandle n)
{
    std::string destTopic = "";
    republish = false;

    if(!n.getParam(FILE_FORMAT_PARAM, fileFormat))
    {
        fileFormat = DEFAULT_FORMAT;
    }
    else
    {
        std::string formatList[] = {"vtk", "pcd"};
        std::set<std::string> allowedFormats(formatList, formatList+2);
        if(allowedFormats.find(fileFormat) == allowedFormats.end())
        {
            ROS_WARN_STREAM("Cannot save a cloud in format: " << fileFormat);
            ROS_WARN_STREAM("Reverting to default format: " << DEFAULT_FORMAT);
            fileFormat = DEFAULT_FORMAT;
        }
    }

    if(n.getParam(DEST_TOPIC_PARAM, destTopic))
    {
        republish = true;
        publisherTopic = n.advertise<sensor_msgs::PointCloud2>(destTopic, 100);
    }

    service = n.advertiseService(SERVICE_NAME, &CloudRecorderService::record, this);
}

bool CloudRecorderService::record(pointcloud_tools::CloudRecorderRequest& req,
                                  pointcloud_tools::CloudRecorderResponse& res)
{
    std::stringstream ss;
    ss << req.cloud.header.frame_id << "_" << req.cloud.header.seq;
    std::string filename = ss.str();

    if(fileFormat == "pcd") saveAsPCD(filename += ".pcd", req.cloud);
    else if(fileFormat == "vtk") saveAsVTK(filename += ".vtk", req.cloud);

    res.filename = filename;
    return true;
}

void CloudRecorderService::saveAsPCD(std::string name, sensor_msgs::PointCloud2 &cloud)
{
    pcl::PCLPointCloud2 pclCloud;
    pcl_conversions::toPCL(cloud, pclCloud);
    pcl::io::savePCDFile(name, pclCloud);
}

void CloudRecorderService::saveAsVTK(std::string name, sensor_msgs::PointCloud2 &cloud)
{
    pcl::PCLPointCloud2 pclCloud;
    pcl_conversions::toPCL(cloud, pclCloud);
    pcl::io::saveVTKFile(name, pclCloud);
}
