
#include "pointcloud_tools/CloudRecorder.h"

CloudRecorder::CloudRecorder(ros::NodeHandle n)
{
    std::string publisherTopicName;
    std::string sourceTopicName;
    republish = false;

    if(!n.getParam(SOURCE_TOPIC_PARAM, sourceTopicName))
    {
        ROS_ERROR("You must specify a source topic.");
        ros::shutdown();
    }
    else
    {
        sourceTopic = n.subscribe(sourceTopicName, 100, &CloudRecorder::record, this);
    }

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

    if(n.getParam(DEST_TOPIC_PARAM, publisherTopicName))
    {
        republish = true;
        publisherTopic = n.advertise<sensor_msgs::PointCloud2>(publisherTopicName, 100);
    }

    n.param<std::string>(TARGET_FRAME_PARAM, targetFrame, "");
}

void CloudRecorder::record(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(targetFrame != "")
    {
        transformToFrame(msg, targetFrame, tfListener);
    }

    std::stringstream ss;
    ss << msg->header.seq;
    std::string filename = ss.str();

    sensor_msgs::PointCloud2 cloud = targetFrame == "" ? *msg : transformToFrame(msg, targetFrame, tfListener);

    if(fileFormat == "pcd") saveAsPCD(filename + ".pcd", *msg);
    else if(fileFormat == "vtk") saveAsVTK(filename + ".vtk", *msg);

    ROS_INFO_STREAM("Saved point cloud with name: " << filename << ".");
}

void CloudRecorder::saveAsPCD(std::string name, const sensor_msgs::PointCloud2 &cloud)
{
    pcl::PCLPointCloud2 pclCloud;
    pcl_conversions::toPCL(cloud, pclCloud);
    pcl::io::savePCDFile(name, pclCloud);
}

void CloudRecorder::saveAsVTK(std::string name, const sensor_msgs::PointCloud2 &cloud)
{
    pcl::PCLPointCloud2 pclCloud;
    pcl_conversions::toPCL(cloud, pclCloud);
    pcl::io::saveVTKFile(name, pclCloud);
}

sensor_msgs::PointCloud2 CloudRecorder::transformToFrame(const sensor_msgs::PointCloud2ConstPtr& cloud, std::string targetFrame, tf::TransformListener& tf)
{
    tf::StampedTransform transform;
    try {
        tf.waitForTransform(targetFrame, cloud->header.frame_id, cloud->header.stamp, ros::Duration(3.0));
        tf.lookupTransform(targetFrame, cloud->header.frame_id, cloud->header.stamp, transform);
    } catch(tf::TransformException &ex) {
        ROS_WARN_STREAM("Could not find a transform to " << targetFrame << " from "
                        << cloud->header.frame_id);
    }

    sensor_msgs::PointCloud2 newCloud;
    pcl_ros::transformPointCloud(targetFrame, transform, *cloud, newCloud);
    return newCloud;
}

void CloudRecorder::spin()
{
    ros::spin();
}
