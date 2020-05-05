#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#include <utility>
#include <algorithm>
#define PI 3.14159265358979323846

double max_alpha = 0.3;
double L = 0.325;
double min_R = L / tan(max_alpha);
int goal_count = 3;
int COUNT_THRES = 50;
double MIN_LIFE = 4.0;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
    root->life_time = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
    //delete[] distribution;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->life_time = 0;

    cv::cvtColor(this->map, this->imgResult, CV_GRAY2BGR);
    cv::resize(imgResult, imgResult, cv::Size(), 2, 2);


    this->p_map = potential_map(map);
    double sum = cv::sum(this->p_map)[0];
    int dist [map.rows * map.cols] = {0,};
    int count;
    int idx = 0;
    for (int i = 0; i < map.rows; i++) {
        for (int j = 0; j < map.cols; j++) {
            count = (int)(1.0 * (p_map.at<float>(i,j)) / sum *  (map.rows * map.cols));
            for (; count > 0; count--){
                dist[idx] = i*map.cols + j;
                idx++;
            }
        }
    }
    this->dist_idx = idx;
    this->distribution = new int [map.rows * map.cols];
    std::memcpy(this->distribution, dist, map.rows * map.cols * sizeof(int));
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;
    int partition = 5;
    int i = this->count - 1;
    idx_parent = this->ptrTable[i]->idx_parent;
    for(int j = 0; j < partition; j++) {
        double alpha = this->ptrTable[i]->alpha;
        double d = this->ptrTable[i]->d;
        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/partition*tan(alpha)/L;
        double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/partition*tan(alpha)/L;
        double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
        double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
        x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
        x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
        cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;
    int partition = 5;
    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
        //cv::circle(imgResult, cv::Point((int)(Res*(path[i].y/res + map_origin_y)), (int)(Res*(path[i].x/res + map_origin_x))), radius, cv::Scalar(0, 255, 0), CV_FILLED);
        double alpha = path[i].alpha;
        if (alpha == 0) {
            x1 = cv::Point((int)(Res * (path[i-1].y / res + map_origin_y)), (int)(Res * (path[i-1].x / res + map_origin_x)));
            x2 = cv::Point((int)(Res * (path[i].y / res + map_origin_y)), (int)(Res * (path[i].x / res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        } else {
            for(int j = 0; j < partition; j++) {
                double d = path[i].d;
                double p1_th = path[i-1].th + d*j/partition*tan(alpha)/L; // R = L/tan(alpha)
                double p2_th = path[i-1].th + d*(j+1)/partition*tan(alpha)/L;
                double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
                double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
                double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
                double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
                x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
                x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
                cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
            }
        }
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

cv::Mat rrtTree::potential_map (cv::Mat origin_map) {
    cv::Mat temp = ~origin_map;
    cv::Mat _voronoi;
    cv::Mat _label;
    temp.copyTo(_voronoi);
    cv::threshold(_voronoi, _voronoi, 50, 255, CV_THRESH_BINARY_INV);
    cv::distanceTransform(_voronoi, _voronoi, _label, CV_DIST_L2, CV_DIST_MASK_PRECISE, CV_DIST_LABEL_CCOMP);
    cv::normalize(_label, _label, 0, 200, cv::NORM_MINMAX);
    cv::normalize(_voronoi, _voronoi, 0, 255, cv::NORM_MINMAX);
    return _voronoi;
}


void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    node *new_tex;
    new_tex = new node;
    new_tex->idx_parent = idx_near;
    new_tex->idx = count;
    new_tex->location = x_new;
    new_tex->rand = x_rand;
    //new_tex->alpha = alpha;
    new_tex->d = d;
    double par_life_time = ptrTable[idx_near]->life_time;
    double my_R = L / tan(alpha);
    double my_th = asin((d/2)/my_R) * 2.;
    //new_tex->life_time = par_life_time + my_R * my_th;
    new_tex->life_time = par_life_time + d;
    new_tex-> alpha = alpha;
    ptrTable[count] = new_tex;
    count = count + 1;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    int i = 0;
    for (; i < K; i++) {
	    point x_rand;
        if (i%goal_count == 0) x_rand = x_goal;
	    else x_rand = randomState(x_max, x_min, y_max, y_min);
        int idx_near = nearestNeighbor(x_rand, MaxStep);
	    double out[5];
	    int valid = randompath(out, ptrTable[idx_near]->location, x_rand, MaxStep);
        if(valid){
            point x_new;
            x_new.x = out[0];
            x_new.y = out[1];
            x_new.th = out[2];
            double alpha = out[3];
            double d = out[4];
            addVertex(x_new, x_rand, idx_near, alpha, d);
            double dist = sqrt(pow(out[0] - x_goal.x, 2) + pow(out[1] - x_goal.y, 2));
            if(dist < 0.2) break;
	        //printf("x, y: %f, %f\n", out[0], out[1]);
	    }
        visualizeTree();
    }
    double a = (ptrTable[count-1]->life_time)/2.;
    double c = sqrt(pow(x_init.x - x_goal.x, 2) + pow(x_init.y - x_goal.y, 2)) / 2;
    if (a > c) {
        double b = sqrt(pow(a, 2) - pow(c, 2));
        double phi = atan2(x_init.y-x_goal.y, x_init.x-x_goal.x);

        for (; i < K; i++) {
            point x_rand;
            if (i%goal_count == 0) x_rand = x_goal;
            else x_rand = randomStateElipse(x_max, x_min, y_max, y_min, a, b, phi);
            int idx_near = nearestNeighbor(x_rand, MaxStep);
            double out[5];
            int valid = randompath(out, ptrTable[idx_near]->location, x_rand, MaxStep);
            if(valid){
                point x_new;
                x_new.x = out[0];
                x_new.y = out[1];
                x_new.th = out[2];
                double alpha = out[3];
                double d = out[4];
                addVertex(x_new, x_rand, idx_near, alpha, d);
                //printf("x, y: %f, %f\n", out[0], out[1]);
            }
            visualizeTree();
        }
    }
    i = 0;
    while(1) {
        int idx_near = nearestNeighbor(x_goal, MaxStep);
        double dist = sqrt(pow(ptrTable[idx_near]->location.x - x_goal.x, 2) + pow(ptrTable[idx_near]->location.y - x_goal.y, 2));
        if(dist < 0.5) break;
        else {
                point x_rand;
                if (i%goal_count ==0) x_rand = x_goal;
                else x_rand = randomState(x_max, x_min, y_max, y_min);
                i = i+1;
                int idx_near = nearestNeighbor(x_rand, MaxStep);
                double out[5];
                int valid = randompath(out, ptrTable[idx_near]->location, x_rand, MaxStep);
                if(valid){
                    int idx_near = nearestNeighbor(x_rand, MaxStep);
                    point x_new;
                    x_new.x = out[0];
                    x_new.y = out[1];
                    x_new.th = out[2];
                    double alpha = out[3];
                    double d = out[4];
                    addVertex(x_new, x_rand, idx_near, alpha, d);
                    double dist = sqrt(pow(out[0] - x_goal.x, 2) + pow(out[1] - x_goal.y, 2));
                    if(dist < 0.5) break;
                    //printf("x, y: %f, %f\n", out[0], out[1]);
                }
        }
        visualizeTree();
    }
}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, int MAX_ITER, double MaxStep) {
    int i = 0;
    int count_addVer = 0;
    for (; i < K; i++) {
	    point x_rand;
        if (i%goal_count == 0) x_rand = x_goal;
	    else x_rand = randomState(x_max, x_min, y_max, y_min);
        int idx_near = nearestNeighbor(x_rand, MaxStep);
	    double out[5];
	    int valid = randompath(out, ptrTable[idx_near]->location, x_rand, MaxStep);
        if(valid){
            count_addVer = 0;
            point x_new;
            x_new.x = out[0];
            x_new.y = out[1];
            x_new.th = out[2];
            double alpha = out[3];
            double d = out[4];
            addVertex(x_new, x_rand, idx_near, alpha, d);
            double dist = sqrt(pow(out[0] - x_goal.x, 2) + pow(out[1] - x_goal.y, 2));
            if(dist < 0.2) break;
	        //printf("x, y: %f, %f\n", out[0], out[1]);
	    }
        else {
            count_addVer++;
        }
        if(count_addVer > COUNT_THRES) {
            visualizeTree();
            return 0;
        }
        visualizeTree();
    }
    double a = (ptrTable[count-1]->life_time)/2.;
    double c = sqrt(pow(x_init.x - x_goal.x, 2) + pow(x_init.y - x_goal.y, 2)) / 2;
    if (a > c) {
        double b = sqrt(pow(a, 2) - pow(c, 2));
        double phi = atan2(x_init.y-x_goal.y, x_init.x-x_goal.x);

        for (; i < K; i++) {
            point x_rand;
            if (i%goal_count == 0) x_rand = x_goal;
            else x_rand = randomStateElipse(x_max, x_min, y_max, y_min, a, b, phi);
            int idx_near = nearestNeighbor(x_rand, MaxStep);
            double out[5];
            int valid = randompath(out, ptrTable[idx_near]->location, x_rand, MaxStep);
            if(valid){
                count_addVer = 0;
                point x_new;
                x_new.x = out[0];
                x_new.y = out[1];
                x_new.th = out[2];
                double alpha = out[3];
                double d = out[4];
                addVertex(x_new, x_rand, idx_near, alpha, d);
                //printf("x, y: %f, %f\n", out[0], out[1]);
            }
            else{
                count_addVer++;
            }
            if(count_addVer > COUNT_THRES) {
                visualizeTree();
                return 0;
            }
            visualizeTree();
        }
    }
    int idx_check = nearestNeighbor(x_goal);
    if(ptrTable[idx_check]->life_time < MIN_LIFE) {visualizeTree(); return 0;}
    while(1) {
        if (i > MAX_ITER) {visualizeTree(); return 0;}
        int idx_near = nearestNeighbor(x_goal, MaxStep);
        double dist = sqrt(pow(ptrTable[idx_near]->location.x - x_goal.x, 2) + pow(ptrTable[idx_near]->location.y - x_goal.y, 2));
        if(dist < 0.5) {visualizeTree(); return 1;}
        else {
            point x_rand;
            if (i%goal_count ==0) x_rand = x_goal;
            else x_rand = randomState(x_max, x_min, y_max, y_min);
            i = i+1;
            int idx_near = nearestNeighbor(x_rand, MaxStep);
            double out[5];
            int valid = randompath(out, ptrTable[idx_near]->location, x_rand, MaxStep);
            if(valid){
                count_addVer = 0;
                int idx_near = nearestNeighbor(x_rand, MaxStep);
                point x_new;
                x_new.x = out[0];
                x_new.y = out[1];
                x_new.th = out[2];
                double alpha = out[3];
                double d = out[4];
                addVertex(x_new, x_rand, idx_near, alpha, d);
                double dist = sqrt(pow(out[0] - x_goal.x, 2) + pow(out[1] - x_goal.y, 2));
                if(dist < 0.5) {visualizeTree(); return 1;}
                //printf("x, y: %f, %f\n", out[0], out[1]);
            } else{
                count_addVer++;
            }
            if(count_addVer > COUNT_THRES) {
                visualizeTree();
                return 0;
            }
        }
        visualizeTree();
    }        
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    //std::random_device rd;
    //std::default_random_engine generator;
    //std::discrete_distribution<int> d(this->distribution);
    //int index = d(generator);
    int rand_num = 1.0 * std::rand() / RAND_MAX * this->dist_idx;
    int x_index = distribution[rand_num] / (this->map).cols;
    int y_index = distribution[rand_num] % (this->map).cols;
    point r_point;
    r_point.x = (double)(x_min + (x_max - x_min) * (double)(x_index) / (this->map).rows);
    r_point.y = (double)(y_min + (y_max - y_min) * (double)(y_index) / (this->map).cols);
    r_point.th = (double)0;

    return r_point;
}

point rrtTree::randomStateElipse(double x_max, double x_min, double y_max, double y_min, double a, double b, double phi) {
    point r_point;
    double r = (double)((double)(std::rand())/RAND_MAX);
    double theta = (double)(2*PI*std::rand()/RAND_MAX);
    double x = r*cos(theta);
    double y = r*sin(theta);
    double x_c = (x_init.x + x_goal.x)/2.;
    double y_c = (x_init.y + x_goal.y)/2.;
    r_point.x = (double)(a*cos(phi)*x-b*sin(phi)*y+x_c);
    r_point.y = (double)(a*sin(phi)*x+b*cos(phi)*y+y_c);
    r_point.th = (double)0;
    if (r_point.x <= x_min || r_point.x >= x_max || r_point.y <= y_min || r_point.y >= y_max){
        return randomStateElipse(x_max, x_min, y_max, y_min, a, b, phi);
    }
    return r_point;
}

std::vector<int> rrtTree::knn(point x_rand, int k) {
   
  std::vector<std::pair<double, int> > v;
   
  for (int i = 0; i < count; i++) {
    double dist = sqrt(pow(ptrTable[i]->location.x - x_rand.x, 2) + pow(ptrTable[i]->location.y - x_rand.y, 2));
    if(v.size() < k){
      v.push_back(std::make_pair(dist, i));
      std::sort(v.begin(), v.end());
    }
    else{
      if(v[k-1].first > dist){
        v.pop_back();
        v.push_back(std::make_pair(dist, i));
        std::sort(v.begin(), v.end());
      }
    }
  }
   
  std::vector<int> ret_v;
  for(int i=0; i<v.size(); i++) ret_v.push_back((v[i].second));
   
  return ret_v;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
  double max_theta = atan2(1 - cos(MaxStep*tan(max_alpha)/L), 
					sin(MaxStep*tan(max_alpha)/L));
  int index = -1;
   
  std::vector<int> k_nei = knn(x_rand, 100);
  for (int k = 0; k < k_nei.size(); k++) {
    int i = k_nei[k];
    double theta = atan2(x_rand.y - ptrTable[i]->location.y, x_rand.x - ptrTable[i]->location.x) - ptrTable[i]->location.th;
    theta = fmod(theta, 2*PI);
    if(theta > PI) theta = theta - 2*PI;
	  if(fabs(max_theta) > fabs(theta)) {
      for(int j = 0; j < 10; j++) {
        double alpha = (double)(-max_alpha + 2*max_alpha*j/9);
        double th_new = ptrTable[i]->location.th + MaxStep*tan(alpha)/L; 
        double x_new = ptrTable[i]->location.x + L/tan(alpha)*(sin(th_new) - sin(ptrTable[i]->location.th));
        double y_new = ptrTable[i]->location.y + L/tan(alpha)*(cos(ptrTable[i]->location.th) - cos(th_new));
        point new_point;
        new_point.x = x_new;
        new_point.y = y_new;
        new_point.th = th_new;
        if(!isCollision(ptrTable[i]->location, new_point, MaxStep, L/tan(alpha))) {
          return i;
        }
	  }
	}
  }
  if(index == -1) {
  	return k_nei[0];
  }
}
int rrtTree::nearestNeighbor(point x_rand) {
    double min_dist = 1E+37; // the largest double number
    int index = -1;
    for (int i = 0; i < count; i++) {
        double dist = sqrt(pow(ptrTable[i]->location.x - x_rand.x, 2) + pow(ptrTable[i]->location.y - x_rand.y, 2));
        if(dist < min_dist) { 
            min_dist = dist;
            index = i;
        }
    }
    return index;
}

int rrtTree::randompath(double *out, point x_near, point x_rand, double MaxStep) {
    double min_dist = 1E+37; // the largest double number
    int index = -1;
    int check = 1;
    int for_idx = 0;
    int valid = 0;
    for(int i = 0; i < 500; i++) {
        double alpha = (double)(-max_alpha + 2*max_alpha*std::rand()/RAND_MAX);
        alpha = alpha;
        x_near.th = x_near.th;
        double d = (double)MaxStep*std::rand()/RAND_MAX;
        double th_new = x_near.th + d*tan(alpha)/L; // R = L/tan(alpha)
        th_new = fmod(th_new, 2*PI);
        if(th_new > PI) th_new = th_new - 2*PI;
        double x_new = x_near.x + L/tan(alpha)*(sin(th_new) - sin(x_near.th));
        double y_new = x_near.y + L/tan(alpha)*(cos(x_near.th) - cos(th_new));
        double dist = sqrt(pow(x_new - x_rand.x,2) + pow(y_new - x_rand.y,2));
        point new_point;
        new_point.x = x_new;
        new_point.y = y_new;
        new_point.th = th_new;
        if(dist < min_dist && !isCollision(x_near, new_point, d, L/tan(alpha))) {
            out[0] = x_new;
            out[1] = y_new;
            out[2] = th_new;
            out[3] = alpha;
            out[4] = d;
            min_dist = dist;
            check = 0;
            valid = 1;
    	}
        x_near.th = x_near.th;
    }
    return valid;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {
    double x_init = x1.x;
    double y_init = x1.y;
    double th_init = x1.th;
    double x_goal = x2.x;
    double y_goal = x2.y;
    double a = this->res;
    double delta_th = d/R;
    int step = (int)(2*d/a);
    int x = x1.x/res + map_origin_x;
    int y = x1.y/res + map_origin_y;
    if (y >= map.cols || x < 0 || x >= map.rows || y < 0) return true;
    if(map.at<uchar>(x,y) == 0) return true;
    for(int i = 0; i < step; i++) {
        double x_cur;
        double y_cur;
        double th_cur;
        if (R == 1E+37) {
            x_cur = x_init + i * cos(x2.th)/2;
            y_cur = y_init + i * sin(x2.th)/2;
        } else {
            th_cur = th_init + i*a/(2*R);
            x_cur = x_init + R*(sin(th_cur) - sin(th_init));
            y_cur = y_init + R*(cos(th_init) - cos(th_cur));
        }
        int x = x_cur/res + map_origin_x; 
        int y = y_cur/res + map_origin_y;
        if (y >= map.cols || x < 0 || x >= map.rows || y < 0) return true;
        if(map.at<uchar>(x,y) == 0) return true;
    }
    x = x_goal/res + map_origin_x;
    y = y_goal/res + map_origin_y;
    if (y >= map.cols || x < 0 || x >= map.rows || y < 0) return true;
    if(map.at<uchar>(x,y) == 0) return true;
    else return false;
}


int rrtTree::getPuppy(point x_rand) {
  int index = -1;
  double min_age = 1E+37; // the largest double number
  int u_index = -1;
  double u_min_age = 1E+37; // the largest double number
   
  for (int i = 0; i < count; i++) {
    double dist = sqrt(pow(ptrTable[i]->location.x - x_rand.x, 2) + pow(ptrTable[i]->location.y - x_rand.y, 2));
    if(dist < 0.2){
      if(ptrTable[i]->life_time < min_age){
        min_age = ptrTable[i]->life_time;
        index = i;
      }
    }
    else if(dist < 0.5) {
      if(ptrTable[i]->life_time < u_min_age){
        u_min_age = ptrTable[i]->life_time;
        u_index = i;
      }
    }
  }
   
  if(index == -1) return u_index;
  else return index;
}

std::vector<traj> rrtTree::backtracking_traj(){
/*
    std::vector<traj> path;
    int partition = 5;
    int idx = getPuppy(x_goal);
    while(idx!=0) {
        
        traj temp_traj;
        temp_traj.x = ptrTable[idx]->location.x;
        temp_traj.y = ptrTable[idx]->location.y;
        temp_traj.th = ptrTable[idx]->location.th;
        temp_traj.alpha = ptrTable[idx]->alpha;
        temp_traj.d = ptrTable[idx]->d;
        path.push_back(temp_traj);
        idx = ptrTable[idx]->idx_parent;
        
    }
    return path;
    

*/

    std::vector<traj> path_temp;
    std::vector<traj> path;
    int partition = 5;
    int idx = getPuppy(x_goal);
    while (idx != 0) {
        double alpha = ptrTable[idx]->alpha;
        for (int j = partition; j > 0; j--) {
            traj temp_traj;
            double d = ptrTable[idx]->d;
            double temp_th = ptrTable[ptrTable[idx]->idx_parent]->location.th + d*j/partition*tan(alpha)/L;
            temp_traj.x = ptrTable[ptrTable[idx]->idx_parent]->location.x + L/tan(alpha)*(sin(temp_th)-sin(ptrTable[ptrTable[idx]->idx_parent]->location.th));
            temp_traj.y = ptrTable[ptrTable[idx]->idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[ptrTable[idx]->idx_parent]->location.th)-cos(temp_th));
            // temp_traj.d = d / partition;
            temp_traj.d = d;
            temp_traj.alpha = alpha;
            temp_traj.th = ptrTable[ptrTable[idx]->idx_parent]->location.th + d*j/partition*tan(alpha)/L;
            path_temp.push_back(temp_traj);
        }
        idx = ptrTable[idx]->idx_parent;
        
    }
    
    int i = path_temp.size()-1;
    int j = 0;
    bool success = false;

    while(i > 0){
        if(i%partition == 0) {
            if(success){
                path_temp[i].d = (j%partition)/((double)(partition)) * path_temp[i].d;
                path.push_back(path_temp[i]);
                success = false;
            } else{
                path.push_back(path_temp[i]);
            }
        }

        for(j = 0; j < i - (i%partition); j++){
            int valid = connectable(path_temp[i], path_temp[j], 0.05);
            if(valid) {
                if (i%5 != 0) {
                    path_temp[i].d = path_temp[i].d * ((partition-i%partition)/((double)(partition)));
                    path.push_back(path_temp[i]);
                }
                path_temp[j].d = sqrt(pow(path_temp[i].x - path_temp[j].x, 2) + pow(path_temp[i].y - path_temp[j].y, 2));
                path_temp[j].th = atan2(path_temp[j].y-path_temp[i].y, path_temp[j].x-path_temp[i].x);
                path_temp[j].alpha = 0;

                path.push_back(path_temp[j]);
                i = j - j%partition;
                success = true;
                break;
            }
        }
        if(!success) i--;
    }
    std::reverse(path.begin(), path.end());
    return path;

}

int rrtTree::connectable (traj t1, traj t2, double max_diff) {
    int valid = 0;
    double theta = atan2(t2.y-t1.y, t2.x-t1.x);
    if (fabs(theta-t1.th) < max_diff && fabs(theta-t2.th) < max_diff) {
        point x1;
        point x2;
        x1.x = t1.x;
        x1.y = t1.y;
        x1.th = theta;
        x2.x = t2.x;
        x2.y = t2.y;
        x2.th = theta;
        double dist = sqrt(pow(x1.x-x2.x, 2) + pow(x1.y-x2.y, 2));
        if(!isCollision(x1, x2, dist, 1E+5)) valid = 1;
        //valid = 1;
    }
    //if (valid) std::cout << "happy" << std::endl;
    return valid;
}
