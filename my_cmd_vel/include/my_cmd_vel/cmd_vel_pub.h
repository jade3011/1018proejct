#ifndef CMD_VEL_PUB_H
#define CMD_VEL_PUB_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

#include <string>
#include <cmath>
#include <vector>

//#define SATURATION_VEL 0.5
//#define SATURATION_ANG 0.2
#define pi 3.14159265358979323846

class controller{
private:
    ros::NodeHandle *nh;
    geometry_msgs::Twist cmd_vel;

    ros::Publisher cmd_vel_publisher;
    ros::Subscriber odom_subscriber;

    ros::Subscriber amcl_pose_subscriber;

    ros::Publisher init_pose;
    ros::Subscriber init_pose_stamped;
    ros::Publisher goal_pose;
    ros::Subscriber goal_pose_stamped;
    ros::Subscriber temi_subscriber;

    tf2_ros::TransformBroadcaster *m_tfServer;
    tf2_ros::Buffer *m_tfBuffer;
    tf2_ros::TransformListener *m_tfListener;
    geometry_msgs::TransformStamped trans;
    geometry_msgs::PoseWithCovarianceStamped amcl_pose;

    // 초기 위치와 목표 위치 값의 입력이 들어왔는지
//    bool is_init = false;
    bool is_goal;
    bool flag;
    

    float vel_x, ang_z;
    float pos_x, pos_y, pos_yaw;

    // 목표 위치와 현재 위치의 오차
    float error_x, error_y;
    // 제어 gain값
    float gain_x, gain_y;

    float goal_x, goal_y, goal_yaw;


    float c;


    float velocity;
    float steer_angle;
    std::vector<double> temp_x;
    std::vector<double>temp_y;
    std::vector<double>compare_object;
    std::vector<double> pre_x;
    std::vector<double> pre_y;
    float compare_tmp;
    int select_object;
public:
    controller(ros::NodeHandle *nh_);
    geometry_msgs::Pose getOdomMaptf(const nav_msgs::Odometry& message);
//    float getEulerAngles( geometry_msgs::Quaternion q);
//    void handle_goal(const geometry_msgs::PoseStamped &goal);
//    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &amcl_pose);
//    void handle_initial_pose(const geometry_msgs::PoseWithCovarianceStamped &pose);
    void messageCallback(const nav_msgs::Odometry& msg);
    void temiCallback(const sensor_msgs::PointCloud2 &msg);
    ~controller();
};


#endif
