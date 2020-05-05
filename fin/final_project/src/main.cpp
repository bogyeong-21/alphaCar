//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1
#define PI 3.14159265358979323846

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
int margin = 6;
int K = 200;
double MaxStep = 1.5;
int MAX_ITER = 500;
int waypoint_margin = 6;
double drive_speed = 2.0;
int count = 100;
const int area_count = 5;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();
cv::Mat block_map (cv::Mat map, std::vector<point> waypoints, int track_iter, double block_ratio);


int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/final_project/src/final.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.7;
            world_x_max = 4.7;
            world_y_min = -10.2;
            world_y_max = 10.2;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();
            printf("Initialize ROBOT\n");
            state = RUNNING;

        case RUNNING: {
            //TODO 3
            int current_goal = 1;
            PID pid_ctrl;
            while(ros::ok()) {
                /*
                point temp_goal;
                temp_goal.x = path_RRT[current_goal].x;
                temp_goal.y = path_RRT[current_goal].y;
                temp_goal.th = path_RRT[current_goal].th;
                cmd.drive.steering_angle = pid_ctrl.get_control(robot_pose, temp_goal);
                */
                cmd.drive.steering_angle = pid_ctrl.get_control(robot_pose, path_RRT[current_goal-1], path_RRT[current_goal]);
                
                cmd.drive.speed = pid_ctrl.set_speed(1.0, 1.5, robot_pose, path_RRT[current_goal], cmd.drive.steering_angle);
                //cmd.drive.steering_angle = pid_ctrl.get_control(robot_pose, path_RRT[current_goal-1], path_RRT[current_goal]);
                cmd_vel_pub.publish(cmd);

                float check_x = robot_pose.x - path_RRT[current_goal].x;
                float check_y = robot_pose.y - path_RRT[current_goal].y;
                if (fabs(check_x) < 0.5 && fabs(check_y) < 0.5) {
                    printf("arrived goal : %d with x : %f, y : %f \n", current_goal, fabs(check_x), fabs(check_y));
                    //printf("path_RRT[%d].x=%f;\npath_RRT[%d].y=%f;\npath_RRT[%d].th=%f;", current_goal, path_RRT[current_goal].x, 
                    //    current_goal, path_RRT[current_goal].y, current_goal, path_RRT[current_goal].th);
                    pid_ctrl.reset();
                    current_goal++;
                    if (current_goal == path_RRT.size()) {
                        printf("reached all point");
                        state = FINISH;
                        break;
                    }
                }
                ros::spinOnce();
                control_rate.sleep();
            }
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;
        default: {
        } break;
        }
    }
    return 0;
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    point waypoint_candid[7];

    // Starting point. (Fixed)
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 8.5;


    //TODO 2
    // Set your own waypoints.
    // The car should turn around the outer track once, and come back to the starting point.
    // This is an example.
    waypoint_candid[1].x = 2.2;
    waypoint_candid[1].y = 8.5;
    waypoint_candid[2].x = 2.5;
    waypoint_candid[2].y = -8.5;
    waypoint_candid[3].x = -2.5;
    waypoint_candid[3].y = -8.0;
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 8.5;


    // Waypoints for arbitrary goal points.
    // TA will change this part before scoring.
    // This is an example.
    waypoint_candid[5].x = 1.5;
    waypoint_candid[5].y = 1.5;
    waypoint_candid[6].x = -2;
    waypoint_candid[6].y = -9.0;

    int order[] = {0,1,2,3,4,5,6};
    int order_size = 7;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{   
    //TODO 1
    rrtTree temp_tree;
    point original[7];
    for (int i = 0; i < 7; i++) {
        original[i] = waypoints[i];
    }
    traj temp_traj;
    temp_traj.x = waypoints[0].x;
    temp_traj.y = waypoints[0].y;
    temp_traj.th = waypoints[0].th;	

    path_RRT.push_back(temp_traj);
    int num_traj[waypoints.size()] = {0,};
    int i = 0;
    cv::Mat blocked_map = block_map(map, waypoints, area_count - 1, 0.8);
    while (i < waypoints.size() - 1) {
        if (i < area_count - 1) {
            temp_tree = rrtTree(waypoints[i],waypoints[i+1], blocked_map, map_origin_x, map_origin_y, res, margin);
        } else {
            temp_tree = rrtTree(waypoints[i],waypoints[i+1], map, map_origin_x, map_origin_y, res, margin);
        }
        int valid = temp_tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MAX_ITER, MaxStep);
        if (valid) {
            std::vector<traj> temp_traj = temp_tree.backtracking_traj();
            if(temp_traj.size() > 0) {
                point temp_point;
                temp_point.x = temp_traj[0].x;
                temp_point.y = temp_traj[0].y;
                temp_point.th = temp_traj[0].th;
                waypoints[i+1] = temp_point;
                for(int j = 0; j < temp_traj.size(); j++) {
                    path_RRT.push_back(temp_traj[temp_traj.size() - j - 1]);
                    num_traj[i]++;
                }
                i++;
            }
        } else if (i > 0) {
            for (int j = 0; j < num_traj[i-1]; j++) {
                path_RRT.pop_back();
            }
            i--;
            num_traj[i] = 0;
            waypoints[i+1] = original[i+1];
        }
    }
    temp_tree.visualizeTree(path_RRT);
    sleep(5);
}


cv::Mat block_map (cv::Mat map, std::vector<point> waypoints, int track_iter, double block_ratio) {
    double x0 = fabs(waypoints[0].x);
    double y0 = fabs(waypoints[0].y) * -1;
    double x1 = x0 * -1;
    double y1 = y0 * -1;
    int counting;
    while (true) {
        counting = 0;
        x0 += 0.1;
        y0 -= 0.1;
        x1 -= 0.1;
        y1 += 0.1;
        int i = 0;
        for (i; i < track_iter; i++) {
            if (waypoints[i].x <= x1 && waypoints[i].x >= x0 && waypoints[i].y <= y0 && waypoints[i].y >= y1) {
                counting++;
            }
        }
        if (i == track_iter) break;
        if (counting >= track_iter) break;
    }
    x0 = (int)((x0 * block_ratio) / res + map_origin_x) - margin ;
    y0 = (int)((y0 * block_ratio) / res + map_origin_y) + margin;
    x1 = (int)((x1 * block_ratio) / res + map_origin_x) + margin;
    y1 = (int)((y1 * block_ratio) / res + map_origin_y) - margin;
    cv::Mat blocked_map = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (i > x1 && i < x0 && j > y0 && j < y1) {
                blocked_map.at<uchar>(i, j) = 0;
            }
        }
    }

    return blocked_map;
}