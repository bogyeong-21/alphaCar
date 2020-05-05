#include <cmath>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/traj.h>
#endif

class PID{
public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    float get_control(point car_pose, point goal_pose, bool mode);
    float get_control(point car_pose, point goal_pose);
    float get_control(point car_pose, traj prev_goal, traj cur_goal);
    double set_speed(double min_speed, double max_speed, point car_pose, traj cur_goal, double rad_to_turn);
    void reset();
private:
    float error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
    float W_g;
};
