#include <iostream>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include<std_msgs/Float32MultiArray.h>

#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include "TrackAssociation.h"

struct minimum_pt{
    float dis;
    cv::Point3f pt;
};

class ExtractDistance
{
private:
    // Subscriber & Publisher
    ros::NodeHandle node;
    ros::Subscriber subScan_;
    ros::Subscriber subImg_;
    ros::Subscriber subBBox_;
    ros::Publisher pubObject_;
    ros::Subscriber TemiBox;

    // 2D LiDAR Subscriber
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;
    pcl::PointCloud <pcl::PointXYZI> mPoint;

    // Image Subscriber
    cv_bridge::CvImagePtr mImagePtr;
    cv::Mat mImage;

    // Bounding Box Subscriber
    darknet_ros_msgs::BoundingBoxes mBBox;
    std::vector<double> mBBox_temi;
    pcl::PointCloud<pcl::PointXYZI> pre_target_point;

    std::vector<double> compare_distance;
public:
    ExtractDistance();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& subScanMsgs);
    void subImgCallback(const sensor_msgs::Image& subImgMsgs);
    void subBboxCallback(const darknet_ros_msgs::BoundingBoxes& subBoxMsgs);
    void TemiCallback(const std_msgs::Float32MultiArray &msg);
    void run();
    pcl::PointCloud<pcl::PointXYZI> lidarROI(pcl::PointCloud<pcl::PointXYZI> point);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> lidarCluster(pcl::PointCloud<pcl::PointXYZI> point);
    pcl::PointCloud<pcl::PointXYZI> lidarFeatureExtract(std::vector<pcl::PointCloud<pcl::PointXYZI>> cluster);
};
