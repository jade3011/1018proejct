#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
  
  
void messageCb(const geometry_msgs::Twist& cmd_vel_pub){
    ROS_INFO("linear.x : %f\n", cmd_vel_pub.linear.x);
    ROS_INFO("linear.y : %f\n", cmd_vel_pub.linear.y);
    ROS_INFO("linear.z : %f\n", cmd_vel_pub.linear.z);
    ROS_INFO("angular.x : %f\n", cmd_vel_pub.angular.x);
    ROS_INFO("angular.y : %f\n", cmd_vel_pub.angular.y);
    ROS_INFO("angular.z : %f\n", cmd_vel_pub.angular.z);
}
  
int main(int argc, char **argv){
    ros::init(argc, argv, "cmd_vel_sub");
    ros::NodeHandle nh;
  
    // ros::Subscriber cmd_vel_subscriber(TOPIC_NAME, messageCb);
    ros::Subscriber cmd_vel_subscriber =nh.subscribe("cmd_vel_topic", 10, messageCb);
      
    ros::spin();
  
    return 0;
}