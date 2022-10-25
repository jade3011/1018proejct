#include "extractDis.h"

ExtractDistance::ExtractDistance(){
    // 2D LiDAR 데이터 Subscribe
    // "/scan" 이라는 Topic을 subscribe 함.
    subScan_ = node.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &ExtractDistance::scanCallback, this);
    // Camera 데이터 Subscribe
    // "/usb_cam/image_raw" 이라는 Topic을 subscribe 함.
    subImg_ = node.subscribe("/usb_cam/image_raw", 1, &ExtractDistance::subImgCallback, this);
    // Yolo에서 검출된 데이터 Subscribe
    // "/darkent_ros/bounding_boxes" 이라는 Topic을 subscribe 함.
    subBBox_ = node.subscribe("/darknet_ros/bounding_boxes", 1, &ExtractDistance::subBboxCallback, this);
    //ros_demo 에서 나오는 tami 데이터를 subscribe함.
    TemiBox = node.subscribe("/temi_points",10,&ExtractDistance::TemiCallback,this);

    // 최종 객체의 거리를 Publish
    // "/object/position" 이라는 Topic을 Publish 함.
    pubObject_ = node.advertise<sensor_msgs::PointCloud2>("/object/position", 100, false);
}

// 2D LiDAR 데이터를 PointCloud mPoint 변수로 변환하여 받음.
void ExtractDistance::scanCallback(const sensor_msgs::LaserScan::ConstPtr& subScanMsgs){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("laser_frame", *subScanMsgs, cloud, tfListener_);
    pcl::fromROSMsg(cloud, mPoint);
}
// Camera 데이터를 Mat mImage 변수로 변환하여 받음.
void ExtractDistance::subImgCallback(const sensor_msgs::Image &subImgMsgs){
    mImagePtr = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
    mImage = mImagePtr->image;
    mImagePtr->image = mImage;
}
// Yolo 검출 결과를 mBBox 변수로 받음.
void ExtractDistance::subBboxCallback(const darknet_ros_msgs::BoundingBoxes& subBoxMsgs){
    mBBox = subBoxMsgs;
}

void ExtractDistance::TemiCallback(const std_msgs::Float32MultiArray &msg)
{
    for(int i=0;i<msg.data.size();i++)
    {
        mBBox_temi.push_back(msg.data.at(i));
    }
}

// 거리 추정 실행 함수
void ExtractDistance::run(){
    // Yolo에서 검출된 결과가 있고, 라이다 포인트와 카메라 이미지가 Subscribe 된 경우 실행
//    if(mBBox.bounding_boxes.size() > 0 && mPoint.size() != 0 && !mImage.empty()){
      if(mBBox_temi.size() > 0 && mPoint.size() != 0 && !mImage.empty()){
        // Homography Matrix(라이다 & 카메라 융합)
        cv::Mat h = cv::Mat::eye(3, 3, CV_64FC1);
        cv::Mat p = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat r_p = cv::Mat::zeros(3, 1, CV_64FC1);
        h=(cv::Mat1d(3,3) << -6.04234086e-04, -2.79133675e-03, -1.21883167e-01,
            1.53154487e-03, -5.31315204e-04, -7.73137846e-01,
        -3.74501809e-04, -4.55925759e-03,  1.00000000e+00);

        //위에것은 전혁이것. temi 검출 결과를 Homography를이용해 x,y좌표를 변환
        pcl::PointCloud<pcl::PointXYZI> cfeature;
        for(int i=0;i<mBBox_temi.size()-1;i++)
        {
            cv::Point2f pt;
            pt.x = mBBox_temi.at(i);
            pt.y = mBBox_temi.at(i+1);

            p=(cv::Mat1d(3,1) <<pt.x,pt.y,1);
            r_p = h*p;

            pcl::PointXYZI arr;
            arr.x = r_p.row(0).at<double>(0,0)/r_p.row(2).at<double>(0,0);
            arr.y = r_p.row(1).at<double>(0,0)/r_p.row(2).at<double>(0,0);

            cfeature.push_back(arr);
            std::cout<<"cok"<<std::endl;
        }

        // Subscribe한 라이다 데이터 ROI
        pcl::PointCloud<pcl::PointXYZI> roi;
        roi = lidarROI(mPoint);
        // ROI된 라이다 좌표를 군집화
        std::vector<pcl::PointCloud<pcl::PointXYZI>> cluster;
        cluster = lidarCluster(roi);
        // 군집환 된 라이다 좌표계에서 특징점 추출
        pcl::PointCloud<pcl::PointXYZI> lfeature;
        lfeature = lidarFeatureExtract(cluster);

        // 카메라 추출된 (x, y) 좌표 자료형 변경(PointCloud -> cv::Point)
        std::vector<cv::Point3f> carr;
        for(int i = 0; i < cfeature.size(); i++){
            cv::Point3f arr;
            arr.x = cfeature[i].x;
            arr.y = cfeature[i].y;
            arr.z = cfeature[i].z;

            carr.push_back(arr);
            std::cout<<"crok"<<std::endl;
        }
        // 라이다 추출된 (x, y) 좌표 자료형 변경(PointCloud -> cv::Point)
        std::vector<cv::Point3f> larr;
        for(int i = 0; i < lfeature.size(); i++){
            cv::Point3f arr;
            arr.x = lfeature[i].x;
            arr.y = lfeature[i].y;
            arr.z = lfeature[i].z;

            larr.push_back(arr);
            std::cout<<"lrok"<<std::endl;
        }
        // 각 센서에서 추출한 객체 위치를 Hungarian Algorithm를 통해 Matching
        TrackAssociation matchingFeature;
        matchingFeature.Track_pt(carr, larr);
        std::vector<cv::Point3f> matching;
        matching = matchingFeature.matching;

        // Matching 된 결과를 자료형 변환(cv::Point -> PointCloud)
        pcl::PointCloud<pcl::PointXYZI> matchResult;
        for(int i = 0; i < matching.size(); i++)
        {
            pcl::PointXYZI arr;
            if(matching.size()==1)
            {
                arr.x = matching[i].x;
                arr.y = matching[i].y;
                arr.z = matching[i].z;
                arr.intensity = i;

                matchResult.push_back(arr);
                std::cout<<"1 object"<<std::endl;
                  std::cout<<"x : "<<matchResult.at(0).x<<"y :"<<matchResult.at(0).y<<std::endl;
            }
            else if(matching.size() == 0)
            {
                break;
            }
            else
            {
                 //   compare_distance.push_back(pow(matching.at(i).x-pre_target_point.at(0).x,2)+pow(matching.at(i).y-pre_target_point.at(0).y,2));
                compare_distance.push_back((matching[i].x-pre_target_point[0].x)*(matching[i].x-pre_target_point[0].x)+(matching[i].y-pre_target_point[0].y)*(matching[i].y-pre_target_point[0].y));
                    std::cout<<"object more than 1"<<std::endl;
            }

        }

        double  temp = 10000.0;
        int select_num = 0;
        for(int i=0;i<compare_distance.size();i++ )
        {
            if(temp>compare_distance.at(i))
            {
                temp = compare_distance.at(i);
                select_num = i;
            }
            if(i ==compare_distance.size()-1)
            {
                pcl::PointXYZI arr_temp;
                arr_temp.x=0;
                arr_temp.y=0;
                arr_temp.z =0;
                arr_temp.intensity =1;
                matchResult.push_back(arr_temp);
                matchResult.at(0).x = matching[select_num].x;
                matchResult.at(0).y = matching[select_num].y;
                std::cout<<"x : "<<matchResult.at(0).x<<"y :"<<matchResult.at(0).y<<std::endl;
            }
        }
        if(!(matchResult.size()==0))
        {
            pre_target_point = matchResult;
        }
        else
            matchResult = pre_target_point;




         // Final Result Publish
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(matchResult, output);
        output.header.frame_id = "laser_frame";
        pubObject_.publish(output);

        // data release
        mBBox.bounding_boxes.clear();
        mBBox_temi.clear();
        compare_distance.clear();
        mPoint.clear();
        mImage.release();
    }
}

// 2D 라이다 데이터 ROI 함쑤
pcl::PointCloud <pcl::PointXYZI> ExtractDistance::lidarROI(pcl::PointCloud<pcl::PointXYZI> point){
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI> filter;
    pcl::PassThrough <pcl::PointXYZI> pass;

    // X축을 기준으로(0m~20m) 범위 설정
    float minx = 0;
    float maxx = 20;

    // 설정된 범위로 PointCloud ROI
    *cloud = point;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minx, maxx);
    pass.filter(*cloud_filter);
    filter = *cloud_filter;

    return filter;
}

// LiDAR 군집화 함수
std::vector<pcl::PointCloud<pcl::PointXYZI>> ExtractDistance::lidarCluster(pcl::PointCloud<pcl::PointXYZI> point){
    std::vector<pcl::PointCloud<pcl::PointXYZI>> output;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_obstacle = point.makeShared();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_obstacle);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // 점과 점사이의 거리
    ec.setClusterTolerance(0.07);
    // 군집화 최소 Point 갯수
    ec.setMinClusterSize(1);
    // 군집화 최대 Point 갯수
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_obstacle);
    ec.extract(cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI> cluster;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZI pt = cloud_obstacle->points[*pit];
            pt.intensity = j;

            cluster.push_back(pt);
        }
        output.push_back(cluster);
        j++;
    }

    return output;
}

// 로봇과 Point와의 거리
float distance(pcl::PointXYZI pt){
    return sqrt(pow(pt.x,2)+pow(pt.y,2));
}

// 두 Point 사이의 거리
float distance2(pcl::PointXYZI pt, pcl::PointXYZI pt_ori){
    return sqrt(pow(pt.x-pt_ori.x,2)+pow(pt.y-pt_ori.y,2));
}
bool cmp(const minimum_pt & a, const minimum_pt & b)
{
    if (a.dis < b.dis) return true; // 제일 먼저 f를 기준으로 오름차순 정렬

    // 각 경우에 대하여 else를 고려할 필요가 없다.
    return false;
}

// 군집환 된 라이다 PointCloud에서 특징점 추출 함수
pcl::PointCloud<pcl::PointXYZI> ExtractDistance::lidarFeatureExtract(std::vector<pcl::PointCloud<pcl::PointXYZI>> cluster){
    pcl::PointCloud<pcl::PointXYZI> output;

    // 군집화 된 PointCloud의 평균을 구해 평균(x, y)를 특징점으로 사용.
    for(int i = 0; i < cluster.size(); i++){
        pcl::PointXYZI pt_pcl;
        std::vector<minimum_pt> feature_min;

        for(int j = 0; j < cluster.at(i).size(); j++){
            float dis = distance(cluster[i][j]);

            minimum_pt arr;
            arr.dis = dis;
            arr.pt.x = cluster[i][j].x;
            arr.pt.y = cluster[i][j].y;
            arr.pt.z = cluster[i][j].z;
            feature_min.push_back(arr);
        }
        sort(feature_min.begin(), feature_min.end(), cmp);

        float sum_x = 0.0;
        float sum_y = 0.0;

        int count = 10;
        if(feature_min.size() < 10)
            count = feature_min.size();

        for(int j = 0; j < count; j++){
            sum_x += feature_min.at(j).pt.x;
            sum_y += feature_min.at(j).pt.y;
        }

        float avg_x = sum_x/(float)count;
        float avg_y = sum_y/(float)count;

        pt_pcl.x = avg_x;
        pt_pcl.y = avg_y;
        pt_pcl.z = 0.0;
        pt_pcl.intensity = 255;

        output.push_back(pt_pcl);
    }

    return output;
}
