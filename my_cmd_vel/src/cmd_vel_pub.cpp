#include "my_cmd_vel/cmd_vel_pub.h"


double transform_tolerance_ = 0.1;

controller::controller(ros::NodeHandle *nh_):nh(nh_){

    odom_subscriber = nh->subscribe("/odom", 1, &controller::messageCallback, this);
    temi_subscriber=nh->subscribe("/object/position",10,&controller::temiCallback,this);

    cmd_vel_publisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);

    m_tfServer = new tf2_ros::TransformBroadcaster();
    m_tfBuffer = new tf2_ros::Buffer();
    m_tfListener = new tf2_ros::TransformListener(*m_tfBuffer);

    gain_x = 0.2;
    gain_y = 0.2;
    c = 0.214;
    flag = false;
    is_goal=false;
    velocity = 0;
    steer_angle = 0;
    float compare_tmp  = 1000.0;

}

void controller::temiCallback(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
    pcl::fromROSMsg(msg,output_pointcloud);


     goal_x =  output_pointcloud.points.at(0).x;
     goal_y = output_pointcloud.points.at(0).y;
     flag =true;

     if (sqrt((goal_x*goal_x)+(goal_y*goal_y)) <= 1.2)
     {
         is_goal = true;
     }
     else
     {
         is_goal=false;
     }


}

void controller::messageCallback(const nav_msgs::Odometry &msg){
 tf::Quaternion  q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
   tf::Matrix3x3  m(q);
   double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    ang_z = pos_yaw;
    pos_x = 0;
    pos_y =0;



    if (flag == true&&is_goal==false)
    {
        geometry_msgs::Twist cmd_vel; // 속도 파라미터


        error_x = goal_x - pos_x;
        error_y = goal_y - pos_y;




        velocity = cos(ang_z) * gain_x * error_x + sin(ang_z) * gain_y * error_y;
        steer_angle = 1/c*(-sin(ang_z) * gain_x * error_x + cos(ang_z) * gain_y * error_y);


        if(velocity>0.4)
        {
            velocity = 0.4;
        }
        else if(velocity<-0.3)
        {
            velocity = -0.3;
        }
        if(steer_angle>0.2)
        {
            steer_angle =0.2;
        }
        else if(steer_angle<-0.2)
        {
            steer_angle = -0.2;
        }

        cmd_vel.linear.x = velocity;
        cmd_vel.angular.z = steer_angle;
        ROS_INFO("error_x: %2f, error_y: %2f, pose_yaw: %2f", error_x, error_y, ang_z);
        ROS_INFO("vel : %2f", cmd_vel.linear.x);
        ROS_INFO("ang_vel : %f\n", cmd_vel.angular.z);

        cmd_vel_publisher.publish(cmd_vel);
        flag =false;
    }
    else
    {
        std::cout<<"flag :"<<flag<<std::endl;
       std::cout<<"to close"<<is_goal<<std::endl;
    }

}
controller::~controller()
{
    if (m_tfServer)
        delete m_tfServer;
    if (m_tfListener)
        delete m_tfListener;
    if (m_tfBuffer)
        delete m_tfBuffer;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_pub");
    ros::NodeHandle nh;

    controller kine(&nh);
    ros::spin();

    return 0;
}

