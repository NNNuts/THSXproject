#pragma once
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

class AGV_MotionControl{
public:
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





    
                            
    // // 路径点数
    // int PathNum = 16;

    //设置路径点
    double Path[10000][2];
                            
    // 路径点数
    int PathNum;

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
    double PathSpeed = 0.2;

    // 路径当前速度
    double PathNowSpeed = 0;

    // 路径长度
    double PathLength = 0;

    // 路径加速时间
    double PathATime = 1;

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


    // 实时路径点 上个点和当前点
    double realTimePathPoint[4];
    int realTimePathPointNum;
    //                   x  y  theta
    double AGV_ERR[3] = {0, 0, 0}; 
    //                     P  I  D  maxChange           maxlimit
    double PID_X[5]   = {0.5, 0, 0.1,       1,              0.4};
    double PID_Y[5]   = {  3, 0, 0,         1,       EIGEN_PI/6};
    double PID_dir[5] = {  1, 0, 0,         1, 45./180*EIGEN_PI};

    // 末端斜移阈值
    double Skewing_threshold = 1;
    // 末端定位精度
    double End_position_accuracy = 0.05;

    


    // 轮毂电机及转向电机使能
    int HubMotor_Enable  = true;
    int TurnMotor_Enable = true;

    PID XPid,DirectionPid,YPid;
    // PID DirectionPid;

    ros::Publisher AGV_control_pub;
    ros::Publisher AGV_loc_pub;
    // ros::Publisher PathGoalSet_pub;
    // ros::Subscriber LidarOdo_sub, Path_sub;
    // ros::NodeHandle nh,nhPart;

    // ros::Rate delay_rate(1000);

    

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
        AGV_states[0] -= 0.255 * cos(AGV_states[2]);
        AGV_states[1] -= 0.255 * sin(AGV_states[2]);

        std_msgs::Float32MultiArray Agv_loc_msg;
        Agv_loc_msg.data.push_back(AGV_states[0]);
        Agv_loc_msg.data.push_back(AGV_states[1]);
        Agv_loc_msg.data.push_back(AGV_states[2]);
        AGV_loc_pub.publish(Agv_loc_msg);
        ros::spinOnce;
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
            double PathDistance;

            // 路径长度超过完整加减速过程
            if(PathLength > PathSpeed * PathATime){
                // 超过预计运动时间
                if(PathTime>PathLength/PathSpeed+PathATime){
                    PathNowSpeed = 0;
                    realTimePathPoint[0] = Path[PathNum-2][0];
                    realTimePathPoint[1] = Path[PathNum-2][1];
                    realTimePathPoint[2] = Path[PathNum-1][0];
                    realTimePathPoint[3] = Path[PathNum-1][1];
                    realTimePathPointNum = PathNum;
                    Path_State = Path_State_WatingStop;
                    AGV_Move_State = AGV_Move_Skewing; 
                    return;
                }
                else if(PathTime>PathLength/PathSpeed){
                    PathNowSpeed = (PathLength/PathSpeed+PathATime-PathTime)/PathATime*PathSpeed;
                    PathDistance = PathLength-PathNowSpeed*(PathLength/PathSpeed+PathATime-PathTime)/2;
                }
                else if(PathTime>PathATime){
                    PathNowSpeed = PathSpeed;
                    PathDistance = PathSpeed * PathTime - PathSpeed * PathATime / 2;
                }
                else{
                    PathNowSpeed = PathTime/PathATime*PathSpeed;
                    PathDistance = PathNowSpeed*PathTime/2;
                }
            }
            else{
                double AT = sqrt(PathLength*PathATime/PathSpeed)/2;
                double maxV = AT/PathATime*PathSpeed;
                if(PathTime>2*AT){
                    PathNowSpeed = 0;
                    realTimePathPoint[0] = Path[PathNum-2][0];
                    realTimePathPoint[1] = Path[PathNum-2][1];
                    realTimePathPoint[2] = Path[PathNum-1][0];
                    realTimePathPoint[3] = Path[PathNum-1][1];
                    realTimePathPointNum = PathNum;
                    Path_State = Path_State_WatingStop;
                    return;
                }
                else if(PathTime>AT){
                    PathNowSpeed = (PathLength/maxV+AT-PathTime)/AT*maxV;
                    PathDistance = PathLength-PathNowSpeed*(PathLength/maxV+AT-PathTime)/2;
                }
                else{
                    PathNowSpeed = PathTime/AT*maxV;
                    PathDistance = PathNowSpeed*PathTime/2;
                }
            }
            ROS_DEBUG("PathNowSpeed %f",PathNowSpeed);
            
            for(int i=0; i<PathNum-1; i++){
                if(PathDistance < sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]))){
                    // 实时路径离散点
                    // realTimePathPoint[0] = Path[i][0] + PathDistance * (Path[i+1][0]-Path[i][0]) / sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]));
                    // realTimePathPoint[1] = Path[i][1] + PathDistance * (Path[i+1][1]-Path[i][1]) / sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]));
                    
                    //  目标路径点
                    realTimePathPoint[0] = Path[i][0];
                    realTimePathPoint[1] = Path[i][1];
                    realTimePathPoint[2] = Path[i+1][0];
                    realTimePathPoint[3] = Path[i+1][1];

                    realTimePathPointNum = i+2;

                    Path_State = Path_State_Run;
                    break;
                }
                else{
                    PathDistance -= sqrt((Path[i+1][1]-Path[i][1])*(Path[i+1][1]-Path[i][1])+(Path[i+1][0]-Path[i][0])*(Path[i+1][0]-Path[i][0]));
                    realTimePathPoint[0] = Path[PathNum-2][0];
                    realTimePathPoint[1] = Path[PathNum-2][1];
                    realTimePathPoint[2] = Path[PathNum-1][0];
                    realTimePathPoint[3] = Path[PathNum-1][1];
                    realTimePathPointNum = PathNum;
                    Path_State = Path_State_WatingStop;
                }
            }
            ROS_DEBUG("realTimePathPoint %f %f %f %f",realTimePathPoint[0],realTimePathPoint[1],realTimePathPoint[2],realTimePathPoint[3]);
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
                double T = 280/tan(dir);
                speed_left  = (T-195)/T * speed;
                speed_right = (T+195)/T * speed;
                dir_leftFront    =  atan2(280,T-195);
                dir_rightFront   =  atan2(280,T+195);
                dir_leftBack     = -atan2(280,T-195);
                dir_rightBack    = -atan2(280,T+195);
            }
            else if(dir<0){
                double T = 280/tan(-dir);
                speed_left  = (T+235)/T * speed;
                speed_right = (T-235)/T * speed;
                dir_leftFront    = -atan2(280,T+195);
                dir_rightFront   = -atan2(280,T-195);
                dir_leftBack     =  atan2(280,T+195);
                dir_rightBack    =  atan2(280,T-195);
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
        double thetaTarget = atan2((realTimePathPoint[3] - realTimePathPoint[1]), (realTimePathPoint[2] - realTimePathPoint[0]));
        AGV_ERR[2] = thetaTarget - AGV_states[2];
        double dis = sqrt(((realTimePathPoint[3]-AGV_states[1])*(realTimePathPoint[3]-AGV_states[1]))+(realTimePathPoint[2]-AGV_states[0])*(realTimePathPoint[2]-AGV_states[0]));
        double thetaErr = atan2((realTimePathPoint[3] - AGV_states[1]), (realTimePathPoint[2] - AGV_states[0]));
        // AGV_ERR[0] = dis * cos(thetaErr - AGV_states[2]);
        AGV_ERR[0] = dis;
        // AGV_ERR[1] = dis * sin(thetaErr - AGV_states[2]);
        AGV_ERR[1] = thetaErr - AGV_states[2];
        ROS_DEBUG("AGV_ERR %f %f %f", AGV_ERR[0],AGV_ERR[1],AGV_ERR[2]);
        return;


        // // 计算偏差距离
        // AGV_ERR[0] = sqrt(((realTimePathPoint[1]-AGV_states[1])*(realTimePathPoint[1]-AGV_states[1]))+(realTimePathPoint[0]-AGV_states[0])*(realTimePathPoint[0]-AGV_states[0]));
        
        // // 计算偏差角度
        // AGV_ERR[1] = atan2(realTimePathPoint[1]-AGV_states[1],realTimePathPoint[0]-AGV_states[0]);
        // AGV_ERR[1] = AGV_ERR[1] - AGV_states[2];
        // // ROS_INFO("AGV_ERR %f",180.*AGV_ERR[1]/3.1415926);
        // if(AGV_ERR[1] > EIGEN_PI)
        //     AGV_ERR[1] -= 2*EIGEN_PI;
        // else if(AGV_ERR[1] < -EIGEN_PI)
        //     AGV_ERR[1] += 2*EIGEN_PI;

        // // 判断是否需要倒车
        // if(AGV_ERR[1] > EIGEN_PI/2 || AGV_ERR[1] < -EIGEN_PI/2)
        //     AGV_ERR[0] = -AGV_ERR[0];

    }

    // 更新 AGV_Move_State
    void judgeAGVState(void){
        // if(AGV_Move_State == AGV_Move_Ackermann){
        //     if(sqrt(AGV_ERR[0]*AGV_ERR[0]+AGV_ERR[1]*AGV_ERR[1]) < End_position_accuracy && Path_State == Path_State_WatingStop)
        //         // AGV_Move_State = AGV_Move_Stop;
        //         Path_State     = Path_State_Stop;
        // }
        if(Path_State == Path_State_WatingStop && AGV_ERR[0] < End_position_accuracy){
            Path_State     = Path_State_Stop;
            AGV_Move_State = AGV_Move_Stop;
        }
        ROS_DEBUG("AGV_Move_State %d Path_State %d",AGV_Move_State ,Path_State);



        // if(AGV_Move_State == AGV_Move_Ackermann){
        //     if(fabs(AGV_ERR[0]) > Skewing_threshold || Path_State != Path_State_WatingStop)
        //         return;
        //     AGV_control_state[0] = 0;
        //     AGV_control_state[1] = 0;
        //     std_msgs::Float32MultiArray msg = CotrolCal();
        //     AGV_control_pub.publish(msg);
        //     ros::Rate delay_rate(1);
        //     for(int i=0; i<3; i++)
        //         delay_rate.sleep();
        //     if(fabs(AGV_ERR[0]) > End_position_accuracy)
        //         AGV_Move_State = AGV_Move_Skewing;
        //     else
        //         AGV_Move_State = AGV_Move_Stop;
        // }
        // else if(AGV_Move_State == AGV_Move_Skewing){
        //     if(fabs(AGV_ERR[0]) < End_position_accuracy){
        //         AGV_Move_State = AGV_Move_Stop;
        //         Path_State = Path_State_Stop;

        //         // 连续运行
        //         // Path[0][0] = AGV_states[0];
        //         // Path[0][1] = AGV_states[1];
        //         // pathInit();
        //     }
        // }
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
            desire_speed   = XPid.pid_control(AGV_ERR[0] * cos(AGV_ERR[1]), 0) + PathNowSpeed;
            if(desire_speed > AGV_control_state[0]){
                AGV_control_state[0] += PID_X[3] / ControlHz;
                if(desire_speed < AGV_control_state[0])
                    AGV_control_state[0] = desire_speed;
            }
            else if(desire_speed < AGV_control_state[0]){
                AGV_control_state[0] -= PID_X[3] / ControlHz;
                if(desire_speed > AGV_control_state[0])
                    AGV_control_state[0] = desire_speed;
            }
            if(AGV_control_state[0] > PID_X[4])
                AGV_control_state[0] = PID_X[4];
            else if(AGV_control_state[0] < -PID_X[4])
                AGV_control_state[0] = -PID_X[4];

            double Ychange = YPid.pid_control(AGV_ERR[0] * AGV_ERR[1], 0);
            if(Ychange > PID_Y[4])
                Ychange = PID_Y[4];
            else if(Ychange < -PID_Y[4])
                Ychange = -PID_Y[4];
            desire_dir   = Ychange + DirectionPid.pid_control(AGV_ERR[2], 0);
            if(desire_dir > AGV_control_state[1]){
                AGV_control_state[1] += PID_dir[3] / ControlHz;
                if(desire_dir < AGV_control_state[0])
                    AGV_control_state[1] = desire_dir;
            }
            else if(desire_dir < AGV_control_state[0]){
                AGV_control_state[1] -= PID_dir[3] / ControlHz;
                if(desire_dir > AGV_control_state[0])
                    AGV_control_state[1] = desire_dir;
            }
            if(AGV_control_state[1] > PID_dir[4])
                AGV_control_state[1] = PID_dir[4];
            else if(AGV_control_state[1] < -PID_dir[4])
                AGV_control_state[1] = -PID_dir[4];
        }
        else if(AGV_Move_State == AGV_Move_Skewing){
            if(AGV_ERR[1]>EIGEN_PI/2 || AGV_ERR[1]<-EIGEN_PI/2)
                desire_speed = XPid.pid_control(-AGV_ERR[0], 0);
            else
                desire_speed = XPid.pid_control(AGV_ERR[0], 0);
            if(desire_speed > AGV_control_state[0]){
                AGV_control_state[0] += PID_X[3] / ControlHz;
                if(desire_speed < AGV_control_state[0])
                    AGV_control_state[0] = desire_speed;
            }
            else if(desire_speed < AGV_control_state[0]){
                AGV_control_state[0] -= PID_X[3] / ControlHz;
                if(desire_speed > AGV_control_state[0])
                    AGV_control_state[0] = desire_speed;
            }
            if(AGV_control_state[0] > PID_X[4])
                AGV_control_state[0] = PID_X[4];
            else if(AGV_control_state[0] < -PID_X[4])
                AGV_control_state[0] = -PID_X[4];
            
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
        ROS_DEBUG("AGV_control_state %f %f",AGV_control_state[0],AGV_control_state[1]);
        return;

        // double desire_speed,desire_dir;

        // if(AGV_Move_State == AGV_Move_Stop){
        //     AGV_control_state[0] = 0;
        //     AGV_control_state[1] = 0;
        //     return;
        // }
        // else if(AGV_Move_State == AGV_Move_Ackermann){
        //     desire_speed = SpeedPid.pid_control(AGV_ERR[0], 0);
        //     // spd
        //     if(desire_speed > AGV_control_state[0]){
        //         AGV_control_state[0] += PID_spd[3] / ControlHz;
        //         if(desire_speed < AGV_control_state[0])
        //             AGV_control_state[0] = desire_speed;
        //     }
        //     else if(desire_speed < AGV_control_state[0]){
        //         AGV_control_state[0] -= PID_spd[3] / ControlHz;
        //         if(desire_speed > AGV_control_state[0])
        //             AGV_control_state[0] = desire_speed;
        //     }
        //     if(AGV_control_state[0] > PID_spd[4])
        //         AGV_control_state[0] = PID_spd[4];
        //     else if(AGV_control_state[0] < -PID_spd[4])
        //         AGV_control_state[0] = -PID_spd[4];
        //     // dir
        //     if(AGV_control_state[0] < 0){
        //         if(AGV_ERR[1]>0)
        //             AGV_ERR[1] = AGV_ERR[1] - EIGEN_PI;
        //         else
        //             AGV_ERR[1] = AGV_ERR[1] + EIGEN_PI;
        //         desire_dir = -DirectionPid.pid_control(AGV_ERR[1], 0);
        //     }
        //     else
        //         desire_dir = DirectionPid.pid_control(AGV_ERR[1], 0);
        //     if(desire_dir > AGV_control_state[1]){
        //         AGV_control_state[1] += PID_dir[3] / ControlHz;
        //         if(desire_dir < AGV_control_state[1])
        //             AGV_control_state[1] = desire_dir;
        //     }
        //     else if(desire_dir < AGV_control_state[1]){
        //         AGV_control_state[1] -= PID_dir[3] / ControlHz;
        //         if(desire_dir > AGV_control_state[1])
        //             AGV_control_state[1] = desire_dir;
        //     }
        //     if(AGV_control_state[1] > PID_dir[4])
        //         AGV_control_state[1] = PID_dir[4];
        //     else if(AGV_control_state[1] < -PID_dir[4])
        //         AGV_control_state[1] = -PID_dir[4];
        //     // AGV_control_state[1] = 0;

        //     // 抵达目标点
        //     // if(Path_State == false){
        //     //     if(fabs(AGV_ERR[0]) < 0.03){
        //     //         AGV_control_state[0] = 0;
        //     //         AGV_control_state[1] = 0;
        //     //         return;
        //     //     }
        //     // }
        // }
        // else if(AGV_Move_State == AGV_Move_Skewing)
        // {
        //     desire_speed = SpeedPid.pid_control(AGV_ERR[0], 0);
        //     if(desire_speed > AGV_control_state[0]){
        //         AGV_control_state[0] += PID_spd[3] / ControlHz;
        //         if(desire_speed < AGV_control_state[0])
        //             AGV_control_state[0] = desire_speed;
        //     }
        //     else if(desire_speed < AGV_control_state[0]){
        //         AGV_control_state[0] -= PID_spd[3] / ControlHz;
        //         if(desire_speed > AGV_control_state[0])
        //             AGV_control_state[0] = desire_speed;
        //     }
        //     if(AGV_control_state[0] > PID_spd[4])
        //         AGV_control_state[0] = PID_spd[4];
        //     else if(AGV_control_state[0] < -PID_spd[4])
        //         AGV_control_state[0] = -PID_spd[4];
            
        //     // dir
        //     if(AGV_control_state[0] < 0){
        //         if(AGV_ERR[1]>0)
        //             AGV_ERR[1] = AGV_ERR[1] - EIGEN_PI;
        //         else
        //             AGV_ERR[1] = AGV_ERR[1] + EIGEN_PI;
        //         desire_dir = DirectionPid.pid_control(AGV_ERR[1], 0);
        //     }
        //     else
        //         desire_dir = DirectionPid.pid_control(AGV_ERR[1], 0);
        //     if(desire_dir > AGV_control_state[1]){
        //         AGV_control_state[1] += PID_dir[3] / ControlHz;
        //         if(desire_dir < AGV_control_state[1])
        //             AGV_control_state[1] = desire_dir;
        //     }
        //     else if(desire_dir < AGV_control_state[1]){
        //         AGV_control_state[1] -= PID_dir[3] / ControlHz;
        //         if(desire_dir > AGV_control_state[1])
        //             AGV_control_state[1] = desire_dir;
        //     }
        //     if(AGV_control_state[1] > PID_dir[4])
        //         AGV_control_state[1] = PID_dir[4];
        //     else if(AGV_control_state[1] < -PID_dir[4])
        //         AGV_control_state[1] = -PID_dir[4];
        // }
        // else if(AGV_Move_State == AGV_Move_Spin)
        // {
        //     // desire_speed = SpeedPid.pid_control(AGV_ERR[0], 0);
        //     if(AGV_ERR[1] > 0){
        //         desire_speed = 0.1;
        //     }
        //     else{
        //         desire_speed = -0.1;
        //     }
        //     AGV_control_state[0] = desire_speed;
            
        // }
    }

    // 接收路径
    void PathResiveCallBack(std_msgs::Float32MultiArray::ConstPtr msg)
    {
        if(msg->data.at(0)<0){
            Path_State = Path_State_Stop;
            AGV_Move_State = AGV_Move_Stop;
        }
        else if(Path_State == Path_State_Stop){
            PathNum = msg->data.at(0);
            PathLength = 0;
            Path[0][0] = msg->data.at(1);
            Path[0][1] = msg->data.at(2);
            for(int i=1; i<PathNum; i++){
                Path[i][0] = msg->data.at(2*i + 1);
                Path[i][1] = msg->data.at(2*i + 2);
                // cout<<Path[i][0]<<" "<<Path[i][1]<<endl;
                PathLength += sqrt((Path[i][0]-Path[i-1][0])*(Path[i][0]-Path[i-1][0])+(Path[i][1]-Path[i-1][1])*(Path[i][1]-Path[i-1][1]));
            }
            pathInit();
            
            // ROS_INFO("%d", PathNum);
            ROS_INFO("路径接收成功");
        }
        // cout<<msg->data.size()<<endl;
        // ROS_INFO("%f", Path[PathNum-1][0]);
        // cout<<"Path num"<<
        // exit(0);
    }

    // 设置路径
    void pathSet(int num, double PathSet[100][2]){
        PathNum = num;
        for(int i=0; i<=num; i++){
            Path[i][0] = PathSet[i][0];
            Path[i][1] = PathSet[i][1];
        }
    }

    // 启动严格模式
    void setPathFollowing_Strict(bool Bool = false){
        if(Bool){
            PathFollowingState = PathFollowing_Strict;
            ROS_INFO("切换为严格模式");
            cout<<"切换为严格模式"<<endl;
        }
        else{
            PathFollowingState = PathFollowing_Flesibility;
            ROS_INFO("切换为柔顺模式");
            cout<<"切换为柔顺模式"<<endl;
        }
    }

    // 系统初始化
    // void systemInit(int argc, char **argv){
    //     ros::init(argc, argv, "MotionControl");
    //     ros::NodeHandle nh;

    //     //创建局部句柄，实例化node
    //     ros::NodeHandle nhPart("~");  
    //     // nh = A;
    //     // nhPart = B;
    //     SpeedPid.kp = PID_spd[0];
    //     SpeedPid.ki = PID_spd[1];
    //     SpeedPid.kd = PID_spd[2];
    //     DirectionPid.kp = PID_dir[0];
    //     DirectionPid.ki = PID_dir[1];
    //     DirectionPid.kd = PID_dir[2];
    //     // cout<<PID_spd[0]<<" "<<SpeedPid.kp<<endl;

    //     nhPart.getParam("goal_x", PathGoal[0]);
    //     nhPart.getParam("goal_y", PathGoal[1]);
    
    //     AGV_control_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvControl", 1000);
    //     // ros::Subscriber LidarOdo_sub = nh.subscribe("odom", 1000, LidarOdoCallback);
    //     // ros::Subscriber LidarOdo_sub = nh.subscribe("Odometry", 1000, LidarOdoCallback);
    //     LidarOdo_sub = nh.subscribe("global_localization", 1000, &AGV_MotionControl::LidarOdoCallback, this);
        
    //     Path_sub = nh.subscribe("Path", 1000, &AGV_MotionControl::PathResiveCallBack, this);
    //     PathGoalSet_pub = nh.advertise<std_msgs::Float32MultiArray>("start_goal", 1000);

    //     AGV_Move_State = AGV_Move_Stop;
    //     Path_State     = Path_State_Stop;
    //     PathFollowingState = PathFollowing_Flesibility;
    //     ros::Rate delay_rate(1000);
    //     setlocale(LC_ALL, "");
    //     ROS_INFO("等待AGV定位");
    //     while(AGV_states[0]>5000 || AGV_states[1]>5000){
    //         delay_rate.sleep();
    //         ros::spinOnce();
    //     }    
    //     ROS_INFO("AGV定位成功,当前位置为: %f, %f", AGV_states[0], AGV_states[1]);
    // }

    // 路径初始化
    void pathInit(void)
    {
        AGV_Move_State = AGV_Move_Ackermann;
        // HubMotor_Enable = true;
        // TurnMotor_Enable = true;
        Path_State = Path_State_Run;
        PathTime = 0;
        realTimePathPointNum = 0;
        Path[0][0] = AGV_states[0];
        Path[0][1] = AGV_states[1];
        // ROS_INFO("AGV定位成功,当前位置为: %f, %f", AGV_states[0], AGV_states[1]);
    }

    void run(int argc, char **argv){
        ros::init(argc, argv, "MotionControl");
        ros::NodeHandle nh;

        //创建局部句柄，实例化node
        ros::NodeHandle nhPart("~");  
        // nh = A;
        // nhPart = B;
        XPid.kp = PID_X[0];
        XPid.ki = PID_X[1];
        XPid.kd = PID_X[2];
        YPid.kp = PID_Y[0];
        YPid.ki = PID_Y[1];
        YPid.kd = PID_Y[2];
        DirectionPid.kp = PID_dir[0];
        DirectionPid.ki = PID_dir[1];
        DirectionPid.kd = PID_dir[2];
        // cout<<PID_spd[0]<<" "<<SpeedPid.kp<<endl;

        // nhPart.getParam("goal_x", PathGoal[0]);
        // nhPart.getParam("goal_y", PathGoal[1]);
        bool LogDebugEnable;
        nhPart.getParam("LogDebugEnable", LogDebugEnable);

        if(LogDebugEnable)
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    
        AGV_control_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvControl", 1000);
        AGV_loc_pub = nh.advertise<std_msgs::Float32MultiArray>("Agv_location", 1000);
        // ros::Subscriber LidarOdo_sub = nh.subscribe("odom", 1000, LidarOdoCallback);
        // ros::Subscriber LidarOdo_sub = nh.subscribe("Odometry", 1000, LidarOdoCallback);
        ros::Subscriber LidarOdo_sub = nh.subscribe("cw_loc_imufreq", 1000, &AGV_MotionControl::LidarOdoCallback, this);
        
        ros::Subscriber Path_sub = nh.subscribe("path", 1000, &AGV_MotionControl::PathResiveCallBack, this);
        // PathGoalSet_pub = nh.advertise<std_msgs::Float32MultiArray>("start_goal", 1000);

        AGV_Move_State = AGV_Move_Stop;
        Path_State     = Path_State_Stop;
        // PathFollowingState = PathFollowing_Flesibility;
        ros::Rate delay_rate(1000);
        setlocale(LC_ALL, "");


        ROS_INFO("等待AGV定位");
        while(AGV_states[0]>5000 || AGV_states[1]>5000){
            delay_rate.sleep();
            ros::spinOnce();
        }    
        ROS_INFO("AGV定位成功,当前位置为: %f, %f", AGV_states[0], AGV_states[1]);


        // pathInit();

        AGV_Move_State = AGV_Move_Stop;
        // HubMotor_Enable = true;
        // TurnMotor_Enable = true;
        Path_State = Path_State_Stop;
        // ROS_INFO("Warning!");
        // ROS_INFO("运控启动");
        ros::Rate loop_rate(ControlHz);
        // char ch;
        // for(int i=0;i<5;i++)
        //      AGVControlReset(Speed);
        std_msgs::Float32MultiArray msg;
        ros::Rate Delay_rate(10);
        
        while (ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
            if(PathFollowingState == PathFollowing_Flesibility){
                realTimePathPointCal();
                // ROS_INFO("柔顺模式 当前目标点 第%d个 为 %f %f",realTimePathPointNum, realTimePathPoint[0],realTimePathPoint[1]);
                CalAGVERR();
                judgeAGVState(); 
                AGV_ConCal();
                // ROS_INFO("PathTime %f", PathTime);
            }
            else if(PathFollowingState == PathFollowing_Strict){
                // 连续运行
                if(Path_State == Path_State_Stop){
                    Path[0][0] = AGV_states[0];
                    Path[0][1] = AGV_states[1];
                    pathInit();
                    ROS_INFO("路径重启");
                }
                if(Path_State != Path_State_Stop){
                    if(!realTimePathPointNum ){
                        realTimePathPointNum++;
                        realTimePathPoint[0] = Path[realTimePathPointNum][0];
                        realTimePathPoint[1] = Path[realTimePathPointNum][1];
                        
                    }
                    // ROS_INFO("严格模式 当前目标点 第%d个 为 %f %f",realTimePathPointNum, realTimePathPoint[0],realTimePathPoint[1]);
                    CalAGVERR();
                    // ROS_INFO("PathTime %f", PathTime);
                    if(AGV_Move_State == AGV_Move_Ackermann && sqrt(AGV_ERR[0]*AGV_ERR[0])<0.10)
                    {
                        
                        realTimePathPointNum++;
                        if(realTimePathPointNum > PathNum){
                            realTimePathPointNum = PathNum;
                            Path_State = Path_State_Stop;
                            ROS_INFO("路径结束");
                        }
                        else{
                            ROS_INFO("开始自旋");
                            realTimePathPoint[0] = Path[realTimePathPointNum][0];
                            realTimePathPoint[1] = Path[realTimePathPointNum][1];
                            AGV_Move_State = AGV_Move_Spin;
                            AGV_control_state[0] = 0;
                            AGV_control_state[1] = 0;
                            msg = CotrolCal();
                            // ros::Rate delay_rate(10);
                            for(int i=0; i<50; i++){
                                AGV_control_pub.publish(msg);
                                // std::cout<<msg.data.at(5)<<std::endl;
                                ros::spinOnce(); 
                                Delay_rate.sleep();
                            }
                        }
                    }
                    CalAGVERR(); 
                    if(AGV_Move_State == AGV_Move_Spin){
                        if(AGV_ERR[1] < 0.02 && AGV_ERR[1] > -0.02){
                            AGV_Move_State = AGV_Move_Ackermann;
                            AGV_control_state[0] = 0;
                            AGV_control_state[1] = 0;
                            msg = CotrolCal();
                            // ros::Rate delay_rate(10);
                            for(int i=0; i<50; i++){
                                AGV_control_pub.publish(msg);
                                ros::spinOnce();
                                Delay_rate.sleep();
                            }
                            ROS_INFO("结束自旋");
                        }   
                    }   
                    // ROS_INFO("control speed is: %f, angular is %f", AGV_control_state[0],AGV_control_state[1]*180/3.1415);
                    // cout<<SpeedPid.kp<<endl;
                    AGV_ConCal();
                }
            }

            
            msg = CotrolCal();
            AGV_control_pub.publish(msg);
            ROS_INFO("  ");
            // ROS_INFO("控制模式为: %d, %d", AGV_Move_State, Path_State);
            ROS_INFO("当前位置为: %f, %f, %f", AGV_states[0], AGV_states[1], AGV_states[2]);
            
            // ROS_INFO("当前位置为: %04f, %04f", AGV_states[0], AGV_states[1]);
            if(Path_State != Path_State_Stop){
                ROS_INFO("目标位置为: %04f, %04f", realTimePathPoint[2], realTimePathPoint[3]);
                ROS_INFO("跟随误差为: %04f, %04f, %04f", realTimePathPoint[2]-AGV_states[0], realTimePathPoint[3]-AGV_states[1],sqrt((realTimePathPoint[2]-AGV_states[0])*(realTimePathPoint[2]-AGV_states[0])+ (realTimePathPoint[3]-AGV_states[1])*(realTimePathPoint[3]-AGV_states[1])));
                
                // ROS_INFO("当前速度: %f, 角度 is %f", AGV_control_state[0],AGV_control_state[1]*180/3.1415);
            }
            else if(Path_State == Path_State_Stop)
                ROS_INFO("等待路径发布");
            // ros::spinOnce();
            // loop_rate.sleep();
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

};

