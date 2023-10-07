#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"
// #include "sensor_msgs/Joy.h"
#include <iostream>
#include <Eigen/Dense>
#include <signal.h>
#include <HubMotor_pkg/PID.hpp>
#include <HubMotor_pkg/AGV_MotionControl.hpp>

using namespace std;
using namespace Eigen;

/*
// AGV运动模式
enum AGVSportsMode{
    Maintain,
    Disability,
    Speed,
    Position
};

// AGV 运动状态
enum AGV_Move_State{
    AGV_Move_Stop,
    AGV_Move_Ackermann,
    AGV_Move_Skewing,
    AGV_Move_Spin
}AGV_Move_State;
// int AGV_Move_State = AGV_Move_Stop;

// 路径运行状态
enum PathState{
    Path_State_Stop,
    Path_State_Run,
    Path_State_WatingStop
}Path_State;
// int Path_State = Path_State_Stop;

// // AGV 控制模式
// enum{
//     AGV_Control_Handle,
//     AGV_Control_Path
// };
// int AGV_Control_mode = AGV_Control_Path;

double AGV_states[3] = {10000, 10000, 0};
double AGV_control_state[2] = {0, 0};





// 设置路径目标点
double PathGoal[2] = {10, 12};

// 设置路径点
// double Path[100][2] = { 0, 0,
//                         4, 0,
//                         4, 3,
//                         7, 3,
//                         8, 2,
//                        10, 1,
//                        13, 1,
//                        12, 5,
//                         6, 4,
//                         3, 2,
//                         0, 0,
//                         }; 
                        
// // 路径点数
// int PathNum = 10;

// // 设置路径点
// double Path[100][2] = {0, 0,
//                         // 0.14, -2.70,
//                         // 0.14, -2.70,
//                         1.91, -0.62,
//                         3.38,  0.54,
//                         5.07, -0.68,
//                         6.73, -2.09,
//                         8.97, -2.67,
//                         9.71, -1.74,
//                         6.79, -1.28,
//                         3.53, -1.20,
//                         // 0.14, -2.70
//                         }; 
                        
// // 路径点数
// int PathNum = 8;

// // 设置路径点
// double Path[100][2] = {0, 0,
//                        -0.30, -2.33,
//                         3.06, -0.44,
//                         4.74,  2.22,
//                         5.21, -0.02,
//                         6.09, -1.39,
//                         7.55, -2.13,
//                         8.78, -2.05,
//                        11.17, -1.02,
//                         8.78, -2.05,
//                         9.87, -3.81,
//                         8.04, -3.34,
//                         6.09, -2.37,
//                         4.22, -1.08,
//                         3.40, -1.07,
//                        -0.30, -2.33}; 
                        
// // 路径点数
// int PathNum = 15;

// // 设置路径点
// double Path[100][2] = { 0, 0,
//                         0.13, -1.21,
//                         2.49, -0.33,
//                         3.84,  1.55,
//                         5.16, -0.69,
//                         2.46, -1.96,
//                         1.27, -3.58,
//                        -0.02, -3.64,
//                        -0.19, -4.49,
//                        -0.87, -5.00,
//                        -2.10, -6.11,
//                        -1.84, -6.33,
//                        -2.10, -7.38,
//                        -1.90, -6.33,
//                        -0.71, -4.87,
//                         0.51, -3.97}; 
                        
// // 路径点数
// int PathNum = 15;

// // 设置路径点
// double Path[100][2] = { 0, 0,
//                         0.672537, -2.6005,
//                         2.06838, -1.64437, 
//                         3.51479, -0.524376,
//                         4.60231, 1.35237,
//                         5.95491, 2.59055,
//                         7.52865, 2.7031,
//                         9.40416, 2.61769,
//                         10.964, 1.80552, 
//                         11.4356, 0.644309, 
//                         11.1717, -1.02511, 
//                         10.0852, -1.96958, 
//                         7.95116, -1.91122, 
//                         5.39523, -1.52858, 
//                         2.92965, -1.61181, 
//                         0.840819, -2.85833, 
//                         0.578524, -4.22789, 
//                         -0.361215, -4.51916,
//                         -0.922639, -4.79515, 
//                         -2.1043, -6.11061, 
//                         -2.18154, -7.5267, 
//                         -2.26941, -8.7383, 
//                         -2.1043, -6.11061, 
//                         -1.90307, -6.10613, 
//                         -0.748428, -4.85524, 
//                         -0.470572, -4.26269,
//                         0.329268, -3.20783,
//                         0.672537, -2.6005,
//                        }; 
                        
// // 路径点数
// int PathNum = 27;

//  设置路径点
// double Path[100][2] = { 0, 0,
//                         -0.29466, -2.49078,
//                         -0.0438857, -1.53799, 
//                         0.782177, -0.317677, 
//                         2.30554, -0.262623, 
//                         3.84512, 0.58654,
//                         5.00791, 2.05986, 
//                         6.71074, 2.57319, 
//                         8.62965, 2.68011, 
//                         10.5055, 2.18551, 
//                         11.2008, -0.0179896, 
//                         10.4742, -1.64634, 
//                         9.15709, -1.87661, 
//                         7.6311, -1.817,
//                         5.58389, -1.40332, 
//                         3.32741, -1.4187, 
//                         1.79284, -2.97731,
//                        }; 
                        
// // 路径点数
// int PathNum = 16;

//设置路径点
double Path[100][2] = { 0, 0,
                    0.154965, -2.77412, 
                    1.24761, -3.60266, 
                    2.69559, -2.95119, 
                    3.62408, -1.54128, 
                    5.58529, -1.64915, 
                    6.98535, -2.14833, 
                    9.03334, -2.25877, 
                    10.7538, -1.59706, 
                    11.2096, -0.487526, 
                    11.3754, 0.768721, 
                    11.0217, 1.7256, 
                    9.2451, 2.64715, 
                    7.28888, 2.82914, 
                    5.6598, 2.68743, 
                    4.67905, 2.16159, 
                    3.84302, 1.47399, 
                    3.02819, 0.313659, 
                    1.70459, -0.0859179, 
                    0.685601, -0.30961, 
                    0.368457, -1.297, 
                       }; 
                        
// 路径点数
int PathNum = 20;

// 路径更随模式
enum PathFollowingModType{
    PathFollowing_Flesibility,
    PathFollowing_Strict
}PathFollowingState;

// 严格跟随模式参数
//double CurrentTargetPoint[2];


// 柔顺跟随模式参数
double ControlHz = 100;

// 路径速度
double PathSpeed = 0.25;

// 路径时间
double PathTime = 0;

// // 手柄按键信息
// enum{
//     A,
//     B,
//     unknow1,
//     X,
//     Y,
//     unknow2,
//     LT,
//     RT,
//     unknow3,
//     LM,
//     RM,
//     unknow4,
//     LD,
//     RD
// };
// int HandleKey[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// // 手柄摇杆信息
// enum{
//     LY,
//     LX,
//     RY,
//     RX,
//     LB,
//     RB,
//     KY,
//     KX
// };
// double HandleRocker[8] = {0, 0, 0, 0, 0, 0, 0, 0};


// 实时路径点
double realTimePathPoint[2];
int realTimePathPointNum;
// dic dir
double AGV_ERR[2] = {0, 0}; 
//                     P  I  D  maxChange           maxlimit
double PID_spd[5] = {  1, 0, 0,       0.3,        PathSpeed};
double PID_dir[5] = {  1, 0, 0,         2, 30./180*EIGEN_PI};

// 末端斜移阈值
double Skewing_threshold = 1;
// 末端定位精度
double End_position_accuracy = 0.01;

ros::Publisher AGV_control_pub;


// 轮毂电机及转向电机使能
int HubMotor_Enable  = true;
int TurnMotor_Enable = true;

PID SpeedPid(PID_spd[0], PID_spd[1], PID_spd[2]);
PID DirectionPid(PID_dir[0], PID_dir[1], PID_dir[2]);

// 初始化
void pathInit(void)
{
    
    AGV_Move_State = AGV_Move_Ackermann;
    // HubMotor_Enable = true;
    // TurnMotor_Enable = true;
    Path_State = Path_State_Run;
    PathTime = 0;
    realTimePathPointNum = 0;
    // ROS_INFO("AGV定位成功,当前位置为: %f, %f", AGV_states[0], AGV_states[1]);
}

// AGV 控制复位
void AGVControlReset(int mod){
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(mod);
    for(int i=0;i<8;i++)
        msg.data.push_back(0);
    AGV_control_pub.publish(msg);
    ros::spinOnce();
}

void MotionControlExit(int sig)
{
	//这里进行退出前的数据保存、内存清理、告知其他节点等工作
    // rob.canClose();

    AGVControlReset(Maintain);
	ROS_INFO("MotionControl shutting down!");
	ros::shutdown();
    exit(0);
}

// 雷达回调函数
void LidarOdoCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // cout<<"ok"<<endl;
    Quaterniond qua(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // Eigen::Vector3d eulerAngle=qua.matrix().eulerAngles(2,1,0);
    // AGV_states[0] = 0;
    AGV_states[0] = msg->pose.pose.position.x;
    AGV_states[1] = msg->pose.pose.position.y;
    // ROS_INFO("雷达回调位置为：%f, %f", AGV_states[0], AGV_states[1]);

//     AGV_states[1] = 0;
    // AGV_states[2] = eulerAngle[0];

    Matrix4d R,T;
    R.block<3,3>(0,0) = qua.matrix();
    R(3,3) = 1;
    T = Matrix4d::Identity();
    T(0,3) = 10;
    Matrix4d RT = R*T;
    AGV_states[2] = atan2(RT(1,3), RT(0,3));

    
    //雷达偏移矫正
    // AGV_states[2] = AGV_states[2] - 135./180*EIGEN_PI; //leisheng
    AGV_states[2] = AGV_states[2] + 3.5/180*EIGEN_PI; //dajiang
    if(AGV_states[2] > EIGEN_PI)
        AGV_states[2] -= 2*EIGEN_PI;
    else if(AGV_states[2] < -EIGEN_PI)
        AGV_states[2] += 2*EIGEN_PI;

    //偏移矫正
    AGV_states[0] -= 0.260 * cos(AGV_states[2]);
    AGV_states[1] -= 0.260 * cos(AGV_states[2]);
    // ROS_INFO("AGV接收位置为: %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    // ROS_INFO("AGV接收四元素为: %f, %f,%f, %f", msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // ROS_INFO("当前位置为: %f, %f, %f", AGV_states[0], AGV_states[1], AGV_states[2]);
    // ROS_INFO("接受RPY为: %f, %f, %f", eulerAngle[0], eulerAngle[1], eulerAngle[2]);
}

// 计算实时路径点
void realTimePathPointCal(void)
{
    
    if(Path_State != Path_State_Stop){
        PathTime += 1.0/ControlHz;
        double PathDistance = PathSpeed * PathTime;
        for(int i=0; i<PathNum; i++){
            if(PathDistance < sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]))){
                // 实时路径离散点
                // realTimePathPoint[0] = Path[i][0] + PathDistance * (Path[i+1][0]-Path[i][0]) / sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]));
                // realTimePathPoint[1] = Path[i][1] + PathDistance * (Path[i+1][1]-Path[i][1]) / sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]));
                
                //  目标路径点
                realTimePathPoint[0] = Path[i+1][0];
                realTimePathPoint[1] = Path[i+1][1];

                Path_State = Path_State_Run;
                break;
            }
            else{
                PathDistance -= sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]));
                realTimePathPoint[0] = Path[PathNum][0];
                realTimePathPoint[1] = Path[PathNum][1];
                Path_State = Path_State_WatingStop;
            }
        }
    }
    

    // ROS_INFO("realTimePathPoint = %f, %f", realTimePathPoint[0], realTimePathPoint[1]);
}

// AGV控制帧计算
std_msgs::Float32MultiArray CotrolCal(void)
{
    std_msgs::Float32MultiArray msg;
    double speed_left,speed_right;
    double dir_leftFront,dir_rightFront,dir_leftBack,dir_rightBack;
    double dir = AGV_control_state[1];
    double speed = AGV_control_state[0]; 
    if(AGV_Move_State == AGV_Move_Ackermann){ 
        if(dir>0){
            double T = 245/tan(dir);
            speed_left  = (T-235)/T * speed;
            speed_right = (T+235)/T * speed;
            dir_leftFront    = atan2(245,T-235);
            dir_rightFront   = atan2(245,T+235);
            dir_leftBack    = -atan2(245,T-235);
            dir_rightBack   = -atan2(245,T+235);
        }
        else if(dir<0){
            double T = 245/tan(-dir);
            speed_left  = (T+235)/T * speed;
            speed_right = (T-235)/T * speed;
            dir_leftFront    = -atan2(245,T+235);
            dir_rightFront   = -atan2(245,T-235);
            dir_leftBack    = atan2(245,T+235);
            dir_rightBack   = atan2(245,T-235);
        }
        else{
            speed_left  = speed;
            speed_right = speed;
            dir_rightFront = dir_rightBack   = 0;
            dir_leftFront = dir_leftBack   = 0;
        }
        
    }
    else if(AGV_Move_State == AGV_Move_Skewing){
        
        speed_left = speed_right = speed;
        dir_leftFront = dir_leftBack   = dir_rightFront = dir_rightBack   = dir;
    }
    else if(AGV_Move_State == AGV_Move_Spin){
        // std::cout<<"Spin"<<endl;
        // std::cout<<dir_leftFront<<endl;
        speed_left = speed_right = speed;
        dir_leftFront = 3.1415926535/4*3;
        dir_leftBack = -3.1415926535/4*3;
        dir_rightFront = 3.1415926535/4;
        dir_rightBack = -3.1415926535/4;
    }
    else{
        speed_left = speed_right = 0;
        dir_leftFront = dir_leftBack   = dir_rightFront = dir_rightBack   = 0;
    }
    if(HubMotor_Enable == false || Path_State == Path_State_Stop){
        speed_left  = 0;
        speed_right = 0;
    }
    if(TurnMotor_Enable == false || Path_State == Path_State_Stop){
        dir_rightFront = dir_rightBack   = 0;
        dir_leftFront = dir_leftBack    = 0;
    }
    
    msg.data.push_back(Maintain);
    msg.data.push_back(speed_left);
    msg.data.push_back(speed_left);
    msg.data.push_back(speed_left);
    msg.data.push_back(speed_right);
    msg.data.push_back(dir_leftFront);
    msg.data.push_back(dir_leftBack);
    msg.data.push_back(dir_rightFront);
    msg.data.push_back(dir_rightBack);
    
    return msg;
}

// 更新AGV_ERR
void CalAGVERR(void)
{
    // 计算偏差距离
    AGV_ERR[0] = sqrt(((realTimePathPoint[1]-AGV_states[1])*(realTimePathPoint[1]-AGV_states[1]))+(realTimePathPoint[0]-AGV_states[0])*(realTimePathPoint[0]-AGV_states[0]));
    
    // 计算偏差角度
    AGV_ERR[1] = atan2(realTimePathPoint[1]-AGV_states[1],realTimePathPoint[0]-AGV_states[0]);
    AGV_ERR[1] = AGV_ERR[1] - AGV_states[2];
    // ROS_INFO("AGV_ERR %f",180.*AGV_ERR[1]/3.1415926);
    if(AGV_ERR[1] > EIGEN_PI)
        AGV_ERR[1] -= 2*EIGEN_PI;
    else if(AGV_ERR[1] < -EIGEN_PI)
        AGV_ERR[1] += 2*EIGEN_PI;

    // 判断是否需要倒车
    // if(AGV_ERR[1] > EIGEN_PI/2 || AGV_ERR[1] < -EIGEN_PI/2)
    //     AGV_ERR[0] = -AGV_ERR[0];

}

// 更新 AGV_Move_State
void judgeAGVState(void)
{
    if(AGV_Move_State == AGV_Move_Ackermann){
        if(fabs(AGV_ERR[0]) > Skewing_threshold || Path_State != Path_State_WatingStop)
            return;
        AGV_control_state[0] = 0;
        AGV_control_state[1] = 0;
        std_msgs::Float32MultiArray msg = CotrolCal();
        AGV_control_pub.publish(msg);
        ros::Rate delay_rate(1);
        for(int i=0; i<3; i++)
            delay_rate.sleep();
        if(fabs(AGV_ERR[0]) > End_position_accuracy)
            AGV_Move_State = AGV_Move_Skewing;
        else
            AGV_Move_State = AGV_Move_Stop;
    }
    else if(AGV_Move_State == AGV_Move_Skewing){
        if(fabs(AGV_ERR[0]) < End_position_accuracy){
            AGV_Move_State = AGV_Move_Stop;
            Path_State = Path_State_Stop;

            // 连续运行
            Path[0][0] = AGV_states[0];
            Path[0][1] = AGV_states[1];
            pathInit();
        }
    }
}

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

        // 抵达目标点
        // if(Path_State == false){
        //     if(fabs(AGV_ERR[0]) < 0.03){
        //         AGV_control_state[0] = 0;
        //         AGV_control_state[1] = 0;
        //         return;
        //     }
        // }
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
    else if(AGV_Move_State == AGV_Move_Spin)
    {
        // desire_speed = SpeedPid.pid_control(AGV_ERR[0], 0);
        if(AGV_ERR[1] > 0){
            desire_speed = 0.1;
        }
        else{
            desire_speed = -0.1;
        }
        AGV_control_state[0] = desire_speed;
        
    }
}

// 接收路径
void PathResiveCallBack(std_msgs::Float32MultiArray::ConstPtr msg)
{
    if(Path_State == Path_State_Stop){
        PathNum = msg->data.at(0);
        for(int i=0; i<PathNum-1; i++){
            Path[i][0] = msg->data.at(2*i + 1);
            Path[i][1] = msg->data.at(2*i + 2);
        }
        pathInit();
        ROS_INFO("%d", PathNum);
        ROS_INFO("路径接收成功");
    }
}

// // 宇树电机相关
// SerialPort  serial("/dev/ttyUSB0");
// MotorCmd    cmd;
// MotorData   data;

// void setUnitreeMotor(int ID,double rad){
//     cmd.motorType = MotorType::GO_M8010_6;
//     cmd.id    = ID;
//     cmd.mode  = 1;
//     cmd.K_P   = 0.3;
//     cmd.K_W   = 0.01;
//     cmd.Pos   = rad * 6.33;
//     cmd.W     = 0;
//     cmd.T     = 0.00;
//     serial.sendRecv(&cmd,&data);
// }

*/

AGV_MotionControl AGV;

//设置路径点
double Path[100][2] = { 0, 0,
                    0.154965, -2.77412, 
                    1.24761, -3.60266, 
                    2.69559, -2.95119, 
                    3.62408, -1.54128, 
                    5.58529, -1.64915, 
                    6.98535, -2.14833, 
                    9.03334, -2.25877, 
                    10.7538, -1.59706, 
                    11.2096, -0.487526, 
                    11.3754, 0.768721, 
                    11.0217, 1.7256, 
                    9.2451, 2.64715, 
                    7.28888, 2.82914, 
                    5.6598, 2.68743, 
                    4.67905, 2.16159, 
                    3.84302, 1.47399, 
                    3.02819, 0.313659, 
                    1.70459, -0.0859179, 
                    0.685601, -0.30961, 
                    0.368457, -1.297, 
                       }; 
                        
// 路径点数
int PathNum = 20;

// 主程序
int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionControl");
    ros::NodeHandle nh;

    //创建局部句柄，实例化node
    ros::NodeHandle nhPart("~");  

    AGV.systemInit(nh, nhPart);
    AGV.pathSet(PathNum,Path);
    AGV.setPathFollowing_Strict(true);
    AGV.run();

    // AGV_Move_State = AGV_Move_Stop;
    // Path_State     = Path_State_Stop;
    // ros::Rate delay_rate(1000);
    // ROS_INFO("等待AGV定位");
    // while(AGV_states[0]>5000 || AGV_states[1]>5000){
    //     delay_rate.sleep();
    //     ros::spinOnce();
    // }    

    // realTimePathPoint[0] = 0;
    // realTimePathPoint[1] = 0;
    // while(true){
    //     CalAGVERR();
    //     ros::spinOnce();
    // }
    


    // // 发布目标点给RRT
    // ROS_INFO("发送目标点");
    // std_msgs::Float32MultiArray PathGoalSet_msg;
    // PathGoalSet_msg.data.push_back(AGV_states[0]);
    // PathGoalSet_msg.data.push_back(AGV_states[1]);
    // PathGoalSet_msg.data.push_back(PathGoal[0]);
    // PathGoalSet_msg.data.push_back(PathGoal[1]);
    // // PathGoalSet_msg.data.push_back(2);
    // // PathGoalSet_msg.data.push_back(2);
    // // PathGoalSet_msg.data.push_back(10);
    // // PathGoalSet_msg.data.push_back(12);

    // while(true){
    //     PathGoalSet_pub.publish(PathGoalSet_msg);
    //     ros::spinOnce();
    //     ros::Rate delay_rate(1);
    //     delay_rate.sleep();
    //     if(Path_State == Path_State_Run)
    //         break;
    // } 
    // ROS_INFO("接收到路径");

    AGV.pathInit();

    // 设置严格跟随模式
    // AGV.PathFollowingState = AGV.PathFollowing_Strict;
    // AGV.PathFollowingState = AGV.PathFollowing_Flesibility;
    

    // ROS_INFO("Warning!");
    // ROS_INFO("运控启动");
    // ros::Rate loop_rate(AGV.ControlHz);
    // char ch;
    // for(int i=0;i<5;i++)
    //      AGVControlReset(Speed);
    // std_msgs::Float32MultiArray msg;
    // ros::Rate Delay_rate(10);
    
    // while (ros::ok()){
    //     if(PathFollowingState == PathFollowing_Flesibility){
    //         realTimePathPointCal();
    //         CalAGVERR();
    //         judgeAGVState(); 
    //         AGV_ConCal();
    //         // ROS_INFO("PathTime %f", PathTime);
    //     }
    //     else if(PathFollowingState == PathFollowing_Strict){
    //         // 连续运行
    //         if(Path_State == Path_State_Stop){
    //             Path[0][0] = AGV_states[0];
    //             Path[0][1] = AGV_states[1];
    //             pathInit();
    //         }
    //         if(Path_State != Path_State_Stop){
    //             if(!realTimePathPointNum ){
    //                 realTimePathPointNum++;
    //                 realTimePathPoint[0] = Path[realTimePathPointNum][0];
    //                 realTimePathPoint[1] = Path[realTimePathPointNum][1];
    //             }
    //             CalAGVERR();
    //             // ROS_INFO("PathTime %f", PathTime);
    //             if(AGV_Move_State == AGV_Move_Ackermann && sqrt(AGV_ERR[0]*AGV_ERR[0])<0.10)
    //             {
    //                 realTimePathPointNum++;
    //                 if(realTimePathPointNum > PathNum){
    //                     realTimePathPointNum = PathNum;
    //                     Path_State = Path_State_Stop;
                        
    //                 }
    //                 else{
    //                     realTimePathPoint[0] = Path[realTimePathPointNum][0];
    //                     realTimePathPoint[1] = Path[realTimePathPointNum][1];
    //                     AGV_Move_State = AGV_Move_Spin;
    //                     AGV_control_state[0] = 0;
    //                     AGV_control_state[1] = 0;
    //                     msg = CotrolCal();
    //                     // ros::Rate delay_rate(10);
    //                     for(int i=0; i<50; i++){
    //                         AGV_control_pub.publish(msg);
    //                         // std::cout<<msg.data.at(5)<<std::endl;
    //                         ros::spinOnce(); 
    //                         Delay_rate.sleep();
    //                     }
    //                 }
    //             }
    //             CalAGVERR(); 
    //             if(AGV_Move_State == AGV_Move_Spin){
    //                 if(AGV_ERR[1] < 0.02 && AGV_ERR[1] > -0.02){
    //                     AGV_Move_State = AGV_Move_Ackermann;
    //                     AGV_control_state[0] = 0;
    //                     AGV_control_state[1] = 0;
    //                     msg = CotrolCal();
    //                     // ros::Rate delay_rate(10);
    //                     for(int i=0; i<50; i++){
    //                         AGV_control_pub.publish(msg);
    //                         ros::spinOnce();
    //                         Delay_rate.sleep();
    //                     }
    //                 }   
    //             }   
    //             AGV_ConCal();
    //         }
    //     }

        
    //     msg = CotrolCal();
    //     AGV_control_pub.publish(msg);
    //     // ROS_INFO("控制模式为: %d, %d", AGV_Move_State, Path_State);
    //     ROS_INFO("当前位置为: %f, %f, %f", AGV_states[0], AGV_states[1], AGV_states[2]*180/3.1415);
    //     if(Path_State != Path_State_Stop)
    //         ROS_INFO("目标位置为: %f, %f", realTimePathPoint[0], realTimePathPoint[1]);
    //     // ROS_INFO("control speed is: %f, angular is %f", AGV_control_state[0],AGV_control_state[1]*180/3.1415);
    //     else if(Path_State == Path_State_Stop)
    //         ROS_INFO("等待路径发布");
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}