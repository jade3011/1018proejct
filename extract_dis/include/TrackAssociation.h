#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#define MAXN 1000
#define INF 1e9;
#define MAXLIFETIME 10
#define MAXCOST 255.0

static int TrackID = 0;
static double m_time;

using namespace cv;
using namespace std;

// 두 점 사이의 거리 추정 함수
float euclideanDist(Point2f& p, Point2f& q);

// Hungarain Algorithm
class CHungarianAlgorithm
{
public:
    int n, Match_num;                            // worker 수
    float label_x[MAXN], label_y[MAXN];           // label x, y
    int yMatch[MAXN];                           // y와 match되는 x
    int xMatch[MAXN];                           // x와 match되는 y
    bool S[MAXN], T[MAXN];                      // 알고리즘 상에 포함되는 vertex.
    float slack[MAXN];
    float slackx[MAXN];
    int parent[MAXN];                           // alternating path
    float cost[MAXN][MAXN];                       // cost
    float init_cost[MAXN][MAXN];                       // 초기 cost

    void init_labels();
    void update_labels();
    void add_to_tree(int x, int parent_x);
    void augment();
    void hungarian();


public:
    void HAssociation(std::vector<Point3f> &matching, vector<Point3f> &cfeature, vector<Point3f> &lfeature, float DIST_TH);

    // Planning Number
    void Make_a_cost(vector<Point3f> &cfeature, vector<Point3f> &lfeature, bool check_max, float Dis_TH);

    CHungarianAlgorithm();
    ~CHungarianAlgorithm();
};

// Matching
class TrackAssociation
{
public:
    std::vector<Point3f> matching;
public:
    TrackAssociation(){}
    ~TrackAssociation(){}

    void Track_pt(vector<Point3f> &cfeature, vector<Point3f> &lfeature);
};
