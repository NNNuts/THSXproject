#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"
// #include "sensor_msgs/Joy.h"
#include <iostream>
#include <Eigen/Dense>
#include <signal.h>
#include <HubMotor_pkg/PID.hpp>
using namespace std;
using namespace Eigen;

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
    AGV_Move_Skewing
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

double ControlHz = 100;

double AGV_states[3] = {10000, 10000, 0};
double AGV_control_state[2] = {0, 0};

// 设置路径目标点
double PathGoal[2] = {10, 12};

// // 设置路径点
// double Path[100][2] = {0, 0,
//                        -0.30, -2.33,
//                         1.26, -2.07,
//                         4.24, -0.48,
//                         6.96, -1.42,
//                         8.17, -2.26,
//                         9.83, -2.63,
//                         11.1, -0.71}; 

// 设置路径点
double Path[100][2] = {0, 0,
                        3.51,  0.78,
                        5.11, -2.42,
                        8.01, -2.10,
                        9.71, -1.74,
                        9.45, -3.73,
                        6.61, -1.80,
                        4.64, -0.85,
                        2.30, -1.89,
                        1.32, -3.09}; 
                        
// 路径点数
int PathNum = 9;

// 路径速度
double PathSpeed = 0.3;

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
double realTimePathPoint[2] = {0, 0};
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
    if(AGV_ERR[1] > EIGEN_PI/2 || AGV_ERR[1] < -EIGEN_PI/2)
        AGV_ERR[0] = -AGV_ERR[0];

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
        for(int i=4; i<3; i++)
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
        if(Path_State == false){
            if(fabs(AGV_ERR[0]) < 0.03){
                AGV_control_state[0] = 0;
                AGV_control_state[1] = 0;
                return;
            }
        }
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


// 主程序
int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionControl");
    ros::NodeHandle nh;

    //创建局部句柄，实例化node
    ros::NodeHandle nhPart("~");  
    // Getting ROSParams
    nhPart.getParam("goal_x", PathGoal[0]);
    nhPart.getParam("goal_y", PathGoal[1]);
 
    setlocale(LC_ALL, "");
    signal(SIGINT, MotionControlExit);
    AGV_control_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvControl", 1000);
    // ros::Subscriber LidarOdo_sub = nh.subscribe("odom", 1000, LidarOdoCallback);
    ros::Subscriber LidarOdo_sub = nh.subscribe("Odometry", 1000, LidarOdoCallback);
    
    ros::Subscriber Path_sub = nh.subscribe("Path", 1000, PathResiveCallBack);
    ros::Publisher PathGoalSet_pub = nh.advertise<std_msgs::Float32MultiArray>("start_goal", 1000);

    AGV_Move_State = AGV_Move_Stop;
    Path_State     = Path_State_Stop;
    ros::Rate delay_rate(1000);
    ROS_INFO("等待AGV定位");
    while(AGV_states[0]>5000 || AGV_states[1]>5000){
        delay_rate.sleep();
        ros::spinOnce();
    }    

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

    Path[0][0] = AGV_states[0];
    Path[0][1] = AGV_states[1];
    pathInit();

    // ROS_INFO("Warning!");
    // ROS_INFO("运控启动");
    ros::Rate loop_rate(ControlHz);
    char ch;
    for(int i=0;i<5;i++)
        AGVControlReset(Speed);
    
    while (ros::ok()){
        realTimePathPointCal();
        CalAGVERR();
        judgeAGVState();
        AGV_ConCal();
        std_msgs::Float32MultiArray msg = CotrolCal();
        AGV_control_pub.publish(msg);
        ROS_INFO("控制模式为: %d, %d", AGV_Move_State, Path_State);
        ROS_INFO("当前位置为: %f, %f, %f", AGV_states[0], AGV_states[1], AGV_states[2]*180/3.1415);
        if(Path_State != Path_State_Stop)
            ROS_INFO("目标位置为: %f, %f", realTimePathPoint[0], realTimePathPoint[1]);
        // ROS_INFO("control speed is: %f, angular is %f", AGV_control_state[0],AGV_control_state[1]*180/3.1415);
        else if(Path_State == Path_State_Stop)
            ROS_INFO("等待路径发布");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}