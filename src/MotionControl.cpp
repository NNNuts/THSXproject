#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <Eigen/Dense>
#include <HubMotor_pkg/PID.hpp>
using namespace std;
using namespace Eigen;

enum
{
    Disability,
    Speed,
    Position
};

double ControlHz = 100;

double AGV_states[3] = {10000, 10000, 0};
double AGV_control_state[2] = {0, 0};
// 设置路径点
double Path[100][2] = {0, 0}; 

double PathNum = 1;
int PathTar = 0;
// dic dir
double AGV_ERR[2] = {0, 0}; 
// P I D maxChange maxlimit
double PID_spd[5] = {  1, 0, 0, 0.5,      0.2};
double PID_dir[5] = {  1, 0, 0, 1, EIGEN_PI/4};
double AGV_controlStates[8] = {0, 0, 0, 0, 0, 0, 0, 0};
// 运动模式切换阈值
// double threshold = 0.8;
// 轮毂电机及转向电机使能
int HubMotor_Enable = 0;
int TurnMotor_Enable = 0;

enum
{
    AGV_Move_Stop,
    AGV_Move_Ackermann,
    AGV_Move_Skewing
};

double AGV_Move_State;


PID SpeedPid(PID_spd[0], PID_spd[1], PID_spd[2]);
PID DirectionPid(PID_dir[0], PID_dir[1], PID_dir[2]);


// Quaterniond qua;
void init(void)
{
    ros::Rate delay_rate(1000);
    ROS_INFO("等待AGV定位");
    while(AGV_states[0]>5000 || AGV_states[1]>5000){
        delay_rate.sleep();
        ros::spinOnce();
    }    
    // Path[0][0] = AGV_states[0];
    // Path[0][1] = AGV_states[1];
    AGV_Move_State = AGV_Move_Ackermann;
    HubMotor_Enable = true;
    TurnMotor_Enable = true;
    ROS_INFO("AGV定位成功,当前位置为: %f, %f", AGV_states[0], AGV_states[1]);
}

void LidarOdoCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // cout<<"ok"<<endl;
    Quaterniond qua(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // Eigen::Vector3d eulerAngle=qua.matrix().eulerAngles(2,1,0);
    // AGV_states[0] = 0;
    AGV_states[0] = msg->pose.pose.position.x;
    AGV_states[1] = msg->pose.pose.position.y;

//     AGV_states[1] = 0;
    // AGV_states[2] = eulerAngle[0];

    Matrix4d R,T;
    R.block<3,3>(0,0) = qua.matrix();
    R(3,3) = 1;
    T = Matrix4d::Identity();
    T(0,3) = 10;


    Matrix4d RT = R*T;
    
    AGV_states[2] = atan2(RT(1,3), RT(0,3));
    // ROS_INFO("theta为: %f", AGV_states[2]);
    
    //雷达偏移矫正
    // AGV_states[2] = AGV_states[2] + (48+55)/180*3.1415;
    AGV_states[2] = AGV_states[2] - 135./180*EIGEN_PI;
    if(AGV_states[2] > EIGEN_PI)
        AGV_states[2] -= 2*EIGEN_PI;
    else if(AGV_states[2] < -EIGEN_PI)
        AGV_states[2] += 2*EIGEN_PI;

    
    //double angle_rz = AGV_states[2]*180/3.1415;
    //偏移矫正
    AGV_states[0] -= 0.260 * cos(AGV_states[2]);
    AGV_states[1] -= 0.260 * cos(AGV_states[2]);
    // ROS_INFO("AGV接收位置为: %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    // ROS_INFO("AGV接收四元素为: %f, %f,%f, %f", msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // ROS_INFO("当前位置为: %f, %f, %f", AGV_states[0], AGV_states[1], AGV_states[2]);
    // ROS_INFO("接受RPY为: %f, %f, %f", eulerAngle[0], eulerAngle[1], eulerAngle[2]);
}

std_msgs::Float32MultiArray CotrolCal(void)
{
    std_msgs::Float32MultiArray msg;
    double dir = AGV_control_state[1];
    double speed = AGV_control_state[0]; 
    double speed_left,speed_right;
    double dir_left,dir_right;
    if(dir>0){
        double T = 245/tan(dir);
        speed_left  = (T-235)/T * speed;
        speed_right = (T+235)/T * speed;
        dir_left    = atan2(245,T-235);
        dir_right   = atan2(245,T+235);
    }
    else if(dir<0){
        double T = 245/tan(-dir);
        speed_left  = (T+235)/T * speed;
        speed_right = (T-235)/T * speed;
        dir_left    = -atan2(245,T+235);
        dir_right   = -atan2(245,T-235);
    }
    else{
        speed_left  = speed;
        speed_right = speed;
        dir_right   = 0;
        dir_left    = 0;
    }
    if(HubMotor_Enable == false){
        speed_left  = 0;
        speed_right = 0;
    }
    if(TurnMotor_Enable == false){
        dir_right   = 0;
        dir_left    = 0;
    }
    msg.data.push_back(Speed);
    msg.data.push_back(speed_left);
    msg.data.push_back(speed_left);
    msg.data.push_back(speed_right);
    msg.data.push_back(speed_right);
    msg.data.push_back(dir_left);
    msg.data.push_back(-dir_left);
    msg.data.push_back(dir_right);
    msg.data.push_back(-dir_right);
    return msg;
}

void CalAGVERR(void)
{
    // AGV_states[2] = AGV_states[2] + (48+55)/180*3.1415;
    AGV_ERR[1] = atan2(Path[PathTar][1]-AGV_states[1],Path[PathTar][0]-AGV_states[0]);
    AGV_ERR[1] = AGV_ERR[1] - AGV_states[2];
    if(AGV_ERR[1] > EIGEN_PI)
        AGV_ERR[1] -= 2*EIGEN_PI;
    else if(AGV_ERR[1] < -EIGEN_PI)
        AGV_ERR[1] += 2*EIGEN_PI;
    AGV_ERR[0] = sqrt((Path[PathTar][1]-AGV_states[1]*(Path[PathTar][1]-AGV_states[1]))+(Path[PathTar][0]-AGV_states[0])*(Path[PathTar][0]-AGV_states[0]));
    
    // AGV_ERR[0] 减去阈值
    // AGV_ERR[0] = AGV_ERR[0] - threshold * 0.5;

    
    AGV_ERR[0] = cos(AGV_ERR[1]) * AGV_ERR[0];
    // ROS_INFO("AGV_ERR = %f, %f", AGV_ERR[0], AGV_ERR[1]*180/3.1415);
    // if(fabs(AGV_ERR[0]) > 1)
    //     AGV_Move_State = AGV_Move_Ackermann;
    // else if(fabs(AGV_ERR[0]) > 0.1)
    //     AGV_Move_State = AGV_Move_Skewing;
    // else
    //     AGV_Move_State = AGV_Move_Stop;
    // std::cout << " AGV_ERR[0] = "  << AGV_ERR[0] << std::endl;
}

// 更新 AGV_Move_State
// void judgeAGVState(void)
// {
    
//     if(AGV_Move_State == AGV_Move_Ackermann){
//         if(fabs(AGV_ERR[0]) > threshold)
//             return;
        
//         if(fabs(AGV_ERR[0]) > 0.1)
//             AGV_Move_State = AGV_Move_Skewing;
//         else
//             AGV_Move_State = AGV_Move_Stop;
//     }
// }

// 更新 AGV_control_state
void AGV_ConCal(void)
{
    double desire_speed,desire_dir;

    if(AGV_Move_State == AGV_Move_Stop){
        AGV_control_state[0] = 0;
        AGV_control_state[1] = 0;
        return;
    }
    else if(AGV_Move_State == AGV_Move_Ackermann){
        desire_speed = SpeedPid.pid_control(AGV_ERR[0], 0);
        // spd
        if(desire_speed > AGV_control_state[0]){
            AGV_control_state[0] += PID_spd[3] / ControlHz;
            if(desire_speed < AGV_control_state[0])
                AGV_control_state[0] = desire_speed;
        }
        else if(desire_speed < AGV_control_state[0]){
            AGV_control_state[0] -= PID_spd[3] / ControlHz;
            if(desire_speed > AGV_control_state[0])
                AGV_control_state[0] = desire_speed;
        }
        if(AGV_control_state[0] > PID_spd[4])
            AGV_control_state[0] = PID_spd[4];
        else if(AGV_control_state[0] < -PID_spd[4])
            AGV_control_state[0] = -PID_spd[4];
        // dir
        if(AGV_control_state[0] < 0){
            if(AGV_ERR[1]>0)
                AGV_ERR[1] = AGV_ERR[1] - EIGEN_PI;
            else
                AGV_ERR[1] = AGV_ERR[1] + EIGEN_PI;
            desire_dir = -DirectionPid.pid_control(AGV_ERR[1], 0);
        }
        else
            desire_dir = DirectionPid.pid_control(AGV_ERR[1], 0);
        if(desire_dir > AGV_control_state[1]){
            AGV_control_state[1] += PID_dir[3] / ControlHz;
            if(desire_dir < AGV_control_state[1])
                AGV_control_state[1] = desire_dir;
        }
        else if(desire_dir < AGV_control_state[1]){
            AGV_control_state[1] -= PID_dir[3] / ControlHz;
            if(desire_dir > AGV_control_state[1])
                AGV_control_state[1] = desire_dir;
        }
        if(AGV_control_state[1] > PID_dir[4])
            AGV_control_state[1] = PID_dir[4];
        else if(AGV_control_state[1] < -PID_dir[4])
            AGV_control_state[1] = -PID_dir[4];
        // AGV_control_state[1] = 0;
    }
    else if(AGV_Move_State == AGV_Move_Skewing)
    {
        desire_speed = SpeedPid.pid_control(AGV_ERR[0], 0);
        if(desire_speed > AGV_control_state[0]){
            AGV_control_state[0] += PID_spd[3] / ControlHz;
            if(desire_speed < AGV_control_state[0])
                AGV_control_state[0] = desire_speed;
        }
        else if(desire_speed < AGV_control_state[0]){
            AGV_control_state[0] -= PID_spd[3] / ControlHz;
            if(desire_speed > AGV_control_state[0])
                AGV_control_state[0] = desire_speed;
        }
        if(AGV_control_state[0] > PID_spd[4])
            AGV_control_state[0] = PID_spd[4];
        else if(AGV_control_state[0] < -PID_spd[4])
            AGV_control_state[0] = -PID_spd[4];
        
        // dir
        if(AGV_control_state[0] < 0){
            if(AGV_ERR[1]>0)
                AGV_ERR[1] = AGV_ERR[1] - EIGEN_PI;
            else
                AGV_ERR[1] = AGV_ERR[1] + EIGEN_PI;
            desire_dir = DirectionPid.pid_control(AGV_ERR[1], 0);
        }
        else
            desire_dir = DirectionPid.pid_control(AGV_ERR[1], 0);
        if(desire_dir > AGV_control_state[1]){
            AGV_control_state[1] += PID_dir[3] / ControlHz;
            if(desire_dir < AGV_control_state[1])
                AGV_control_state[1] = desire_dir;
        }
        else if(desire_dir < AGV_control_state[1]){
            AGV_control_state[1] -= PID_dir[3] / ControlHz;
            if(desire_dir > AGV_control_state[1])
                AGV_control_state[1] = desire_dir;
        }
        if(AGV_control_state[1] > PID_dir[4])
            AGV_control_state[1] = PID_dir[4];
        else if(AGV_control_state[1] < -PID_dir[4])
            AGV_control_state[1] = -PID_dir[4];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionControl");
    ros::NodeHandle nh;
 
    ros::Publisher AGV_ontrol_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvControl", 1000);
    ros::Subscriber LidarOdo_sub = nh.subscribe("odom", 1000, LidarOdoCallback);
    setlocale(LC_ALL, "");
    // ROS_INFO("Warning!");
    ROS_INFO("运控启动");
    ros::Rate loop_rate(ControlHz);
    char ch;
    init();
    while (ros::ok())
    {
        float current_angular_speed =  AGV_control_state[1]*180/3.1415;
        float angle_rz = AGV_states[2]*180/3.1415;
        ROS_INFO("当前位置为: %f, %f, %f", AGV_states[0], AGV_states[1], angle_rz);
        ROS_INFO("目标位置为: %f, %f", Path[PathTar][0], Path[PathTar][1]);
        ROS_INFO("control speed is: %f, angular is %f", AGV_control_state[0],current_angular_speed);
        CalAGVERR();
        AGV_ConCal();
        std_msgs::Float32MultiArray msg = CotrolCal();
        AGV_ontrol_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


