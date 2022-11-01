#include "TrackAssociation.h"

// 두 점 사이의 거리 계산 함수
float euclideanDist(Point3f &p, Point3f &q) {
    Point2f a;
    a.x= p.x;
    a.y = p.y;

    Point2f b;
    b.x = q.x;
    b.y = q.y;

    Point2f diff = a - b;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

// 라이다 & 카메라 Matching
void TrackAssociation::Track_pt(vector<Point3f> &cfeature, vector<Point3f> &lfeature){
    float L_minCost = 4.0;
    CHungarianAlgorithm h;
    matching.clear();
    // Hungarian Algorithm 수행
    h.HAssociation(matching, cfeature, lfeature, L_minCost);
}

// HungarianAlgorithm
CHungarianAlgorithm::CHungarianAlgorithm()
{
}
CHungarianAlgorithm::~CHungarianAlgorithm()
{
}

// 라이다 & 카메라의 각 좌표의 cost 생성 함수
void CHungarianAlgorithm::Make_a_cost(vector<Point3f> &cfeature, vector<Point3f> &lfeature, bool check_max, float Dis_TH){
    float xmax = 0.0;
    if(check_max == true){ // 카메라에 추출된 객체의 좌표가 더 많을 때
        // [카메라 검출된 객체의 좌표 , 카메라 검출된 객체의 좌표] 크기의 행렬 생성 
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (j >= lfeature.size())
                {
                    cost[i][j] = 255;
                    init_cost[i][j] = 255;
                    xmax = max(xmax, cost[i][j]);
                }
                else
                {
                    Point3f predict_candi_pt = cfeature[i];
                    Point3f candi_pt = lfeature[j];

                    float Dist = euclideanDist(predict_candi_pt,candi_pt);
                    if(Dist >= Dis_TH){
                        cost[i][j] = 255;
                        init_cost[i][j] = 255;
                        xmax = max(xmax, cost[i][j]);
                    }
                    else{
                        cost[i][j] = Dist;
                        init_cost[i][j] = Dist;
                        xmax = max(xmax, cost[i][j]);
                    }

                }

            }
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                cost[i][j] = xmax - cost[i][j];
            }
        }
    }
    else{ // 라이다에서 추출된 객체의 좌표가 더 많을 때
        // [라이다 검출된 객체의 좌표 , 라이다 검출된 객체의 좌표] 크기의 행렬 생성
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i >= cfeature.size())
                {
                    cost[i][j] = 255;
                    init_cost[i][j] = 255;
                    xmax = max(xmax, cost[i][j]);
                }
                else
                {
                    Point3f predict_candi_pt = cfeature[i];
                    Point3f candi_pt = lfeature[j];
                    float Dist = euclideanDist(predict_candi_pt,candi_pt);
                    if(Dist >= Dis_TH){
                        cost[i][j] = 255;
                        init_cost[i][j] = 255;
                        xmax = max(xmax, cost[i][j]);
                    }
                    else{
                        cost[i][j] = Dist;
                        init_cost[i][j] = Dist;
                        xmax = max(xmax, cost[i][j]);
                    }
                }
            }
        }
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                cost[i][j] = xmax - cost[i][j];
            }
        }
    }
}

void CHungarianAlgorithm::HAssociation(std::vector<Point3f> &matching, vector<Point3f> &cfeature, vector<Point3f> &lfeature, float DIST_TH)
{
    if (cfeature.size() > lfeature.size()) // 카메라에 추출된 객체의 좌표가 더 많을 때
    {
        n = cfeature.size();

        // cost 생성
        Make_a_cost(cfeature, lfeature, true, DIST_TH);
        
        // 생성된 cost를 통해 Matching 수행
        hungarian();

        // 최종 결과 출력
        for (int x = 0; x < n; x++)
        {
            if (init_cost[x][xMatch[x]] < DIST_TH) //  이하이면
            {
                matching.push_back(lfeature[xMatch[x]]);
            }
        }
    }
    else // 라이다에서 추출된 객체의 좌표가 더 많을 떄
    {
        n = lfeature.size();

        // cost 생성
        Make_a_cost(cfeature, lfeature, false, DIST_TH);

        // 생성된 cost를 통해 Matching 수행
        hungarian();

        // 최종 결과 출력
        for (int y = 0; y < n; y++)
        {
            if (init_cost[yMatch[y]][y] < DIST_TH)
            {
                matching.push_back(lfeature[y]);
            }
        }
    }
}

void CHungarianAlgorithm::init_labels()
{
    memset(label_x, 0, sizeof(label_x));
    memset(label_y, 0, sizeof(label_y));      // y label은 모두 0으로 초기화.

    for (int x = 0; x < n; x++)
        for (int y = 0; y < n; y++)
            label_x[x] = max(label_x[x], cost[x][y]);    // cost중에 가장 큰 값을 label 값으로 잡음.
}

void CHungarianAlgorithm::update_labels()
{
    float delta = (float)INF;

    // slack통해서 delta값 계산함.
    for (int y = 0; y < n; y++)
        if (!T[y]) delta = min(delta, slack[y]);

    for (int x = 0; x < n; x++)
        if (S[x]) label_x[x] -= delta;
    for (int y = 0; y < n; y++) {
        if (T[y]) label_y[y] += delta;
        else slack[y] -= delta;
    }
}

void CHungarianAlgorithm::add_to_tree(int x, int parent_x)
{
    S[x] = true;            // S집합에 포함.
    parent[x] = parent_x;   // augmenting 할때 필요.

    for (int y = 0; y < n; y++) {                                   // 새 노드를 넣었으니, slack 갱신해야함.
        if (label_x[x] + label_y[y] - cost[x][y] < slack[y]) {
            slack[y] = label_x[x] + label_y[y] - cost[x][y];
            slackx[y] = x;
        }
    }
}

void CHungarianAlgorithm::augment()
{
    if (Match_num == n) return;
    int root;   // 시작지점.
    queue<int> q;

    memset(S, false, sizeof(S));
    memset(T, false, sizeof(T));
    memset(parent, -1, sizeof(parent));

    // root를 찾음. 아직 매치안된 y값을 찾음ㅇㅇ.
    for (int x = 0; x < n; x++) {
        if (xMatch[x] == -1) {
            q.push(root = x);
            parent[x] = -2;
            S[x] = true;
            break;
        }
    }

    // slack 초기화.
    for (int y = 0; y < n; y++) {
        slack[y] = label_x[root] + label_y[y] - cost[root][y];
        slackx[y] = root;
    }

    int x, y;
    // augment function
    while (1) {
        // bfs cycle로 tree building.
        while (!q.empty()) {
            x = q.front(); q.pop();
            for (y = 0; y < n; y++) {
                if (cost[x][y] == label_x[x] + label_y[y] && !T[y]) {
                    if (yMatch[y] == -1) break;
                    T[y] = true;
                    q.push(yMatch[y]);
                    add_to_tree(yMatch[y], x);
                }
            }
            if (y < n) break;
        }
        if (y < n) break;

        while (!q.empty()) q.pop();

        update_labels(); // 증가경로가 없다면 label 향상ㄱ.

        // label 향상을 통해서 equality graph의 새 edge를 추가함.
        // !T[y] && slack[y]==0 인 경우에만 add 할 수 있음.
        for (y = 0; y < n; y++) {
            if (!T[y] && slack[y] == 0) {
                if (yMatch[y] == -1) {          // 증가경로 존재.
                    x = slackx[y];
                    break;
                }
                else {
                    T[y] = true;
                    if (!S[yMatch[y]]) {
                        q.push(yMatch[y]);
                        add_to_tree(yMatch[y], slackx[y]);
                    }
                }
            }
        }
        if (y < n) break;  // augment path found;
    }

    if (y < n) {        // augment path exist
        Match_num++;

        for (int cx = x, cy = y, ty; cx != -2; cx = parent[cx], cy = ty) {
            ty = xMatch[cx];
            yMatch[cy] = cx;
            xMatch[cx] = cy;
        }
        augment();  // 새 augment path 찾음.
    }
}

void CHungarianAlgorithm::hungarian()
{
    Match_num = 0;

    memset(xMatch, -1, sizeof(xMatch));
    memset(yMatch, -1, sizeof(yMatch));

    init_labels();
    augment();
}
