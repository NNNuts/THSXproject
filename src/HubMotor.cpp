#include "HubMotor.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Joy.h"
#include <signal.h>

#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>

#include <pthread.h>
#include <condition_variable>
#include <mutex>


#define msleep(ms)  usleep((ms)*1000)


HubMotor rob;
double ControlHz = 20;
// HubMotor rob1(1);
// double AgvCommond[9] = {0,0,0,0,0,0,0,0,0};
double AgvCommond[9] = {0,0,0,0,0,0,0,0,0};
double odometerPosition[6] = {0, 0, 0, 0, 0, 0};

// 是否Log Debug
bool LogDebugEnable = false;

// AGV 运动模式
enum AGVSportsMode{
    Maintain,
    Disability,
    Speed,
    Position
};
int Mode = Disability;

// AGV 原始数据发布使能
bool AGVDataPubEnable = false;

// AGV 编码器发布使能
bool AGVEncoderOdomPubEnable = false;

// 手柄按键信息
enum HandleBotton{
    A,
    B,
    unknow1,
    X,
    Y,
    unknow2,
    LT,
    RT,
    unknow3,
    unknow4,
    LM,
    RM,
    unknow5,
    LD,
    RD
};
int HandleKey[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// 手柄摇杆信息
enum HandleAxes{
    LY,
    LX,
    RY,
    RX,
    LB,
    RB,
    KY,
    KX
};
double HandleRocker[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// AGV 运动状态
double AGV_states[2] = {0.2, 45./180*EIGEN_PI};

// AGV 控制模式
enum AGVControlMode{
    AGV_Control_Handle,
    AGV_Control_Procedure
}AGV_Control_Mode;

void HubMotorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    // cout<<"ok"<<endl;
    for(int i = 0; i < 9; i++)
        AgvCommond[i] = msg->data.at(i);
}

void HubMotorExit(int sig)
{
	//这里进行退出前的数据保存、内存清理、告知其他节点等工作
    // rob.canClose();
    rob.motorDisEnable(1);
    rob.motorDisEnable(2);
    rob.motorDisEnable(3);
    rob.motorDisEnable(4);
	ROS_INFO("HubMotor shutting down!");
	ros::shutdown();
    exit(0);
}

void odometer(void)
{
    double P[4][2];
    double angle = atan2(0.235,0.245);
    double length = sqrt(0.235*0.235 + 0.245*0.245);
    if(rob.hubMotorRealSpeed[0]>10 || rob.hubMotorRealSpeed[1]>10 || rob.hubMotorRealSpeed[2]>10 || rob.hubMotorRealSpeed[3]>10)
    {
        ROS_WARN("轮毂电机断开连接");
        return;
    }
    if(rob.stepMotorRealPosition[4]>10 || rob.stepMotorRealPosition[5]>10 || rob.stepMotorRealPosition[6]>10 || rob.stepMotorRealPosition[7]>10)
    {
        ROS_WARN("转向电机断开连接");
        return;
    }
    odometerPosition[3] = odometerPosition[0];
    odometerPosition[4] = odometerPosition[1];
    odometerPosition[5] = odometerPosition[2];
    
    P[0][0] = rob.hubMotorRealSpeed[0] / ControlHz * cos(odometerPosition[2] + rob.stepMotorRealPosition[4]) + odometerPosition[0] + length * cos(angle + odometerPosition[2]);
    P[0][1] = rob.hubMotorRealSpeed[0] / ControlHz * sin(odometerPosition[2] + rob.stepMotorRealPosition[4]) + odometerPosition[1] + length * sin(angle + odometerPosition[2]);
    P[1][0] = rob.hubMotorRealSpeed[1] / ControlHz * cos(odometerPosition[2] + rob.stepMotorRealPosition[5]) + odometerPosition[0] - length * cos(angle - odometerPosition[2]);
    P[1][1] = rob.hubMotorRealSpeed[1] / ControlHz * sin(odometerPosition[2] + rob.stepMotorRealPosition[5]) + odometerPosition[1] + length * sin(angle - odometerPosition[2]);
    P[2][0] = rob.hubMotorRealSpeed[2] / ControlHz * cos(odometerPosition[2] + rob.stepMotorRealPosition[6]) + odometerPosition[0] + length * cos(angle - odometerPosition[2]);
    P[2][1] = rob.hubMotorRealSpeed[2] / ControlHz * sin(odometerPosition[2] + rob.stepMotorRealPosition[6]) + odometerPosition[1] - length * sin(angle - odometerPosition[2]);
    P[3][0] = rob.hubMotorRealSpeed[3] / ControlHz * cos(odometerPosition[2] + rob.stepMotorRealPosition[7]) + odometerPosition[0] - length * cos(angle + odometerPosition[2]);
    P[3][1] = rob.hubMotorRealSpeed[3] / ControlHz * sin(odometerPosition[2] + rob.stepMotorRealPosition[7]) + odometerPosition[1] - length * sin(angle + odometerPosition[2]);

    odometerPosition[0] = (P[0][0] + P[1][0] + P[2][0] + P[3][0]) / 4;
    odometerPosition[1] = (P[0][1] + P[1][1] + P[2][1] + P[3][1]) / 4;
    odometerPosition[2] = (atan2((P[0][1] - P[1][1]), (P[0][0] - P[1][0])) + atan2((P[2][1] - P[3][1]), (P[2][0] - P[3][0]))) / 2;
    odometerPosition[3] = (odometerPosition[0] - odometerPosition[3]) * ControlHz;
    odometerPosition[4] = (odometerPosition[1] - odometerPosition[4]) * ControlHz;
    odometerPosition[5] = (odometerPosition[2] - odometerPosition[5]) * ControlHz;
}

// 接收手柄信息
void HandleResiveCallBack(sensor_msgs::Joy::ConstPtr msg){
    for(int i=0; i<15; i++){
        HandleKey[i] = msg->buttons.at(i);
    }
    for(int i=0; i<8; i++){
        HandleRocker[i] = msg->axes.at(i);
    }
    ROS_DEBUG_STREAM("RM %d" << HandleKey[RM]);
}

// 电机模式设置
void motorModeSet(int mode){
    Mode = mode;
    if(mode == Speed){
        // 设置速度控制模式
        rob.motorInit(1, SpeedMod);
        rob.motorInit(2, SpeedMod);
        rob.motorInit(3, SpeedMod);
        rob.motorInit(4, SpeedMod);
        ROS_INFO("设置速度模式");
    }
    // else if(mode == Position){
    //     // 设置位置控制模式
    //     rob.motorInit(1, PositionMod);
    //     rob.motorInit(2, PositionMod);
    //     rob.motorInit(3, PositionMod);
    //     rob.motorInit(4, PositionMod);
    //     ROS_INFO("设置位置模式");
    // }
    else if(mode == Disability){
        // 设置失能模式
        rob.motorDisEnable(1);
        rob.motorDisEnable(2);
        rob.motorDisEnable(3);
        rob.motorDisEnable(4);
        ROS_INFO("设置失能模式");
    }
}

// AGV 运动控制
void AGV_Control(double speed, double dir){
    double speed_left,speed_right;
    double dir_leftFront,dir_rightFront,dir_leftBack,dir_rightBack;
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
        
    if(Mode == Speed){
        rob.motorSetSpeed(1, speed_left);
        rob.motorSetSpeed(2, speed_left);
        rob.motorSetSpeed(3, speed_left);
        rob.motorSetSpeed(4, speed_right);
        // ROS_INFO("Set HubMotor:speed = [%f],[%f]", speed_left, speed_right);
    }
    rob.stepMotorSetPosition(5, dir_leftFront);
    rob.stepMotorSetPosition(6, dir_leftBack);
    rob.stepMotorSetPosition(7, dir_rightFront);
    rob.stepMotorSetPosition(8, dir_rightBack);
    // ROS_INFO("Set StepMotor:position = [%f],[%f],[%f],[%f]", dir_leftFront, dir_leftBack, dir_rightFront, dir_rightBack);
}

// AGV 模式切换
void AGV_Mode_Switching(void){
    // 毫秒延时器
    ros::Rate delay_rate(1000);
    if(HandleKey[RM] == 1){
        if(AGV_Control_Mode == AGV_Control_Procedure){
            AGV_Control_Mode = AGV_Control_Handle;
            ROS_INFO("切换为手柄控制");

        }
        else if(AGV_Control_Mode == AGV_Control_Handle){
            AGV_Control_Mode = AGV_Control_Procedure;
            ROS_INFO("切换为程序控制");
        }
        // AGV_states[0] = 0;
        // AGV_states[1] = 0;
        AGV_Control(0, 0);
        // AGVControlReset();
        // if(Mode == Speed){
        //     rob.motorSetSpeed(1, 0);
        //     rob.motorSetSpeed(2, 0);
        //     rob.motorSetSpeed(3, 0);
        //     rob.motorSetSpeed(4, 0);
        //     // ROS_INFO("Set HubMotor:speed = [%f],[%f],[%f],[%f]", AgvCommond[1], AgvCommond[2], AgvCommond[3], AgvCommond[4]);
        // }
        // rob.stepMotorSetPosition(5, 0);
        // rob.stepMotorSetPosition(6, 0);
        // rob.stepMotorSetPosition(7, 0);
        // rob.stepMotorSetPosition(8, 0);
        while(true){
            delay_rate.sleep();
            ros::spinOnce();
            // cout<<"1"<<endl;
            if(HandleKey[RM] == 0)
                break;
        }
    }
}

ros::Publisher AgvData_pub;
ros::Publisher AgvOdometerPosition_pub;
ros::Publisher AgvOdometerSpeed_pub;


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Agv");  //解析参数，命名结点
    

    //创建全局句柄，实例化node
    ros::NodeHandle nh;  

    //创建局部句柄，实例化node
    ros::NodeHandle nhPart("~");  


    // 设置ROS_INFO中文输出
    setlocale(LC_ALL, "");
    ROS_INFO("HubMotor 启动");

    // Getting ROSParams
    nhPart.getParam("AGVDataPubEnable", AGVDataPubEnable);
    nhPart.getParam("AGVEncoderOdomPubEnable", AGVEncoderOdomPubEnable);
    nhPart.getParam("LogDebugEnable", LogDebugEnable);

    if(LogDebugEnable)
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // cout<<AGVDataPubEnable<<endl;
    // cout<<AGVEncoderOdomPubEnable<<endl;
  
    rob.canOpen();

    rob.motorDisEnable(1);
    rob.motorDisEnable(2);
    rob.motorDisEnable(3);
    rob.motorDisEnable(4);

    // ROS_INFO("轮毂电机已连接");

    signal(SIGINT, HubMotorExit);
    ros::Rate rate(ControlHz);  
    // 毫秒延时器
    ros::Rate delay_rate(1000);

    

    if(AGVDataPubEnable)
        AgvData_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvData", 1000);
    if(AGVEncoderOdomPubEnable){
        AgvOdometerPosition_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvOdometerPosition", 1000);
        AgvOdometerSpeed_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvOdometerSpeed", 1000);
    }
    
    ros::Subscriber Handle_sub = nh.subscribe("joy", 1000, HandleResiveCallBack);
    ros::Subscriber sub = nh.subscribe("AgvControl", 1000, HubMotorCallback);

    AGV_Control_Mode = AGV_Control_Procedure;
    while(true){
        // AGV 模式切换
        AGV_Mode_Switching();

        // 读取电机数据
        if(AGVDataPubEnable || AGVEncoderOdomPubEnable){
            rob.stepMotorReadPosition();
            rob.hubMotorReadPosition();
        }
        
        // 发布AGV原始数据
        if(AGVDataPubEnable){
            std_msgs::Float32MultiArray AgvData;
            AgvData.data.push_back(Mode);
            for(int i = 0; i < 4; i++)
                AgvData.data.push_back(rob.hubMotorRealSpeed[i]);
            for(int i = 0; i < 4; i++)
                AgvData.data.push_back(rob.stepMotorRealPosition[4 + i]);
            AgvData_pub.publish(AgvData);

            if(Mode == Disability)
                ROS_DEBUG("Pub HubMotor:Disability");
            else if(Mode == Speed)
                ROS_DEBUG("Pub HubMotor:Speed = [%f],[%f],[%f],[%f]", rob.hubMotorRealSpeed[0], rob.hubMotorRealSpeed[1], rob.hubMotorRealSpeed[2], rob.hubMotorRealSpeed[3]);
            ROS_DEBUG("Pub StepMotor:position = [%f],[%f],[%f],[%f]", rob.stepMotorRealPosition[4], rob.stepMotorRealPosition[5], rob.stepMotorRealPosition[6], rob.stepMotorRealPosition[7]);
        }

        // 发布编码器里程计
        if(AGVEncoderOdomPubEnable){
            std_msgs::Float32MultiArray AgvOdometerPosition;
            std_msgs::Float32MultiArray AgvOdometerSpeed;
            odometer();
            for(int i = 0; i < 3; i++)
                AgvOdometerPosition.data.push_back(odometerPosition[i]);
            for(int i = 3; i < 6; i++)
                AgvOdometerSpeed.data.push_back(odometerPosition[i]);
            AgvOdometerPosition_pub.publish(AgvOdometerPosition);
            AgvOdometerPosition_pub.publish(AgvOdometerPosition);
            ROS_DEBUG("odometerStatus = [%f],[%f],[%f],[%f],[%f],[%f] ", 
            odometerPosition[0], odometerPosition[1], odometerPosition[2], odometerPosition[3], odometerPosition[4], odometerPosition[5]);
        }
        

        // 程序控制
        if(AGV_Control_Mode == AGV_Control_Procedure){
            // 电机模式设置
            if(AgvCommond[0] != Maintain){
                // ROS_INFO("开始切换模式");
                // Mode = AgvCommond[0];
                motorModeSet(AgvCommond[0]);
            }
            if(Mode == Speed){
                rob.motorSetSpeed(1, AgvCommond[1]);
                rob.motorSetSpeed(2, AgvCommond[2]);
                rob.motorSetSpeed(3, AgvCommond[3]);
                rob.motorSetSpeed(4, AgvCommond[4]);
                ROS_DEBUG("Set HubMotor:speed = [%f],[%f],[%f],[%f]", AgvCommond[1], AgvCommond[2], AgvCommond[3], AgvCommond[4]);
            }
            // else if(Mode == Position)
            // {
            //     rob.motorSetPosition(1, AgvCommond[1]);
            //     rob.motorSetPosition(2, AgvCommond[2]);
            //     rob.motorSetPosition(3, AgvCommond[3]);
            //     rob.motorSetPosition(4, AgvCommond[4]);
            //     // ROS_INFO("Set HubMotor:position = [%f],[%f],[%f],[%f]", AgvCommond[1], AgvCommond[2], AgvCommond[3], AgvCommond[4]);
            // }
            rob.stepMotorSetPosition(5, AgvCommond[5]);
            rob.stepMotorSetPosition(6, AgvCommond[6]);
            rob.stepMotorSetPosition(7, AgvCommond[7]);
            rob.stepMotorSetPosition(8, AgvCommond[8]);
            ROS_DEBUG("Set StepMotor:position = [%f],[%f],[%f],[%f]", AgvCommond[5], AgvCommond[6], AgvCommond[7], AgvCommond[8]);

            // ROS_INFO("--------------------------------------------------\r\n");
        }

        // 手柄控制
        else if(AGV_Control_Mode == AGV_Control_Handle){
            // 电机模式设置
            if(HandleKey[A] == 1){
                motorModeSet(Speed);
                while(true){
                    delay_rate.sleep();
                    ros::spinOnce();
                    if(HandleKey[A] == 0)
                        break;
                }
            }
            if(HandleKey[B] == 1){
                motorModeSet(Disability);
                while(true){
                    delay_rate.sleep();
                    ros::spinOnce();
                    if(HandleKey[B] == 0)
                        break;
                }
            }
            AGV_Control(HandleRocker[LX] * AGV_states[0], HandleRocker[RY] * AGV_states[1]);
            ROS_DEBUG("AGV 运动状态 %f %f", HandleRocker[LX] * AGV_states[0], HandleRocker[RY] * AGV_states[1]);
            // ROS_INFO("--------------------------------------------------\r\n");
        }
        
        // ROS_INFO("--------------------------------------------------\r\n\r\n");

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
} 
       