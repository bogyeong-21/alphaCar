#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 0.8;
    Ki = 0;
    Kd = -0.35;
    /*
    Kp = 1.5;
    Ki = 0;
    Kd = 5;
    */
    W_g = 1.0;
}

void PID::reset() {
    error = 0;
    error_sum = 0;
    error_diff = 0;
}
float PID::get_control(point car_pose, point goal_pose) {
    float ctrl;
    float x_diff = (goal_pose.x - car_pose.x);
    float y_diff = (goal_pose.y - car_pose.y);

    float x_dot = cos(car_pose.th) * x_diff + sin(car_pose.th) * y_diff;
    float y_dot = -sin(car_pose.th) * x_diff + cos(car_pose.th) * y_diff;
    //float pid_angle = atan2(y_dot, x_dot);
    float angle_diff = fmod(goal_pose.th - car_pose.th, 2*PI);
    if(angle_diff > PI) angle_diff = angle_diff - 2*PI;
    //printf("angle_diff : %f\n", angle_diff);
    float pid_angle = atan2(y_dot, x_dot);
    error_diff = pid_angle - error;
    error = pid_angle;
    error_sum += error;

    ctrl = Kp * error + Ki * error_sum + Kd * error_diff;

    if (ctrl> 0.8) {ctrl =0.8;}
    else if (ctrl<-0.8) {ctrl=-0.8;}
    return ctrl;
}

float PID::get_control(point car_pose, traj prev_goal, traj cur_goal) {
    float error_prev = error;
    float MAX_ERROR_SUM = 10;
    float dist = sqrt(pow(car_pose.x - cur_goal.x,2) + pow(car_pose.y - cur_goal.y,2));
    float th_g = atan((car_pose.y - cur_goal.y)/(car_pose.x - cur_goal.x));
    
    if (th_g < 0 && car_pose.x - cur_goal.x > 0) {
        th_g += PI;
    }
    else if (th_g > 0 && car_pose.x - cur_goal.x > 0) {
        th_g -= PI;
    }
    error = th_g - car_pose.th;
    
    if (dist > 1.2)
        error = th_g - car_pose.th;
    else
        error = W_g*th_g + (1-W_g)*cur_goal.th - car_pose.th;
    
    if (error < -PI)
        error += 2*PI;
    else if (error > PI)
        error -= 2*PI;
    error_sum += error;
    error_diff = error - error_prev;
    if (error_sum > MAX_ERROR_SUM){
        error_sum = MAX_ERROR_SUM;
    }
    else if (error_sum < -MAX_ERROR_SUM) {
        error_sum = -MAX_ERROR_SUM;
    }
    float ctrl = Kp * error + Ki * error_sum + Kd * error_diff;
    if (ctrl > 0.8) {
        ctrl = 0.8;
    }
    else if (ctrl < -0.8) {
        ctrl = -0.8;
    }
    return ctrl;
}
double PID::set_speed(double min_speed, double max_speed, point car_pose, traj cur_goal, double rad_to_turn)  {
    double dist = sqrt(pow(car_pose.x - cur_goal.x, 2.0) + pow(car_pose.y - cur_goal.y, 2.0));
    return fmin(min_speed + 2/(1 + 8*fabs(rad_to_turn)), max_speed);   
}
float PID::get_control(point car_pose, point goal_pose, bool mode){
    float ctrl;
    if (mode){
        float x_diff = (goal_pose.x - car_pose.x);
        float y_diff = (goal_pose.y - car_pose.y);

        float x_dot = cos(car_pose.th) * x_diff + sin(car_pose.th) * y_diff;
        float y_dot = -sin(car_pose.th) * x_diff + cos(car_pose.th) * y_diff;
        //float pid_angle = atan2(y_dot, x_dot);
        float angle_diff = fmod(goal_pose.th - car_pose.th, 2*PI);
        if(angle_diff > PI) angle_diff = angle_diff - 2*PI;
        //printf("angle_diff : %f\n", angle_diff);
        float pid_angle = atan2(y_dot, x_dot);
        error_diff = pid_angle - error;
        error = pid_angle;
        error_sum += error;

        ctrl = Kp * error + Ki * error_sum + Kd * error_diff;

        if (ctrl> 0.8) {ctrl =0.8;}
        else if (ctrl<-0.8) {ctrl=-0.8;}
        return ctrl;
    } else {
        float x_diff = -(goal_pose.x - car_pose.x);
        float y_diff = -(goal_pose.y - car_pose.y);

        float x_dot = cos(car_pose.th) * x_diff + sin(car_pose.th) * y_diff;
        float y_dot = -sin(car_pose.th) * x_diff + cos(car_pose.th) * y_diff;
        //float pid_angle = atan2(y_dot, x_dot);
        float angle_diff = fmod(goal_pose.th - car_pose.th, 2*PI);
        if(angle_diff > PI) angle_diff = angle_diff - 2*PI;
        //printf("angle_diff : %f\n", angle_diff);
        float pid_angle = atan2(y_dot, x_dot);
        error_diff = pid_angle - error;
        error = pid_angle;
        error_sum += error;

        ctrl = Kp * error + Ki * error_sum + Kd * error_diff;

        if (ctrl> 0.8) {ctrl =0.8;}
        else if (ctrl<-0.8) {ctrl=-0.8;}
        return -ctrl;
    }
}
