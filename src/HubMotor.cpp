#include "HubMotor_pkg/HubMotor.hpp"
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
#include "HubMotor_pkg/PID.hpp"
#include "HubMotor_pkg/4M485.h"

#include "HubMotor_pkg/serialPort/SerialPort.h"
#include <thread>


#define msleep(ms)  usleep((ms)*1000)

using namespace dst_ccms_api;

HubMotor rob;
double ControlHz = 20;
// HubMotor rob1(1);
// double AgvCommond[9] = {0,0,0,0,0,0,0,0,0};
double AgvCommond[9] = {0,0,0,0,0,0,0,0,0};
double odometerPosition[6] = {0, 0, 0, 0, 0, 0};

// 是否Log Debug
bool LogDebugEnable = false;

// AGV 轮距
double  TrackWidth[2] = {490, 500};

// AGV 使能模式
enum AGVEnableModeType{
    Maintain,
    Disability,
    Speed,
    Position
};
AGVEnableModeType Mode = Disability;

// AGV 手柄运动模式
enum AGVHandleRunModeType{
    Ackermann,
    Skewing,
    Spin
};
AGVHandleRunModeType AGV_HandleRunMode = Ackermann;

// AGV 原始数据发布使能
bool AGVDataPubEnable = false;

// AGV 编码器发布使能
bool AGVEncoderOdomPubEnable = false;

// // 无线手柄按键信息
// enum HandleBotton{
//     A,
//     B,
//     unknow1,
//     X,
//     Y,
//     unknow2,
//     LT,
//     RT,
//     unknow3,
//     unknow4,
//     LM,
//     RM,
//     unknow5,
//     LD,
//     RD
// };

// 有线按键
enum HandleBotton{
    A,
    B,
    X,
    Y,
    LT,
    RT,
    LM,
    RM,
    unknow5,
    LD,
    RD
};
int HandleKey[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// // 无线手柄摇杆信息
// enum HandleAxes{
//     LY,
//     LX,
//     RY,
//     RX,
//     LB,
//     RB,
//     KY,
//     KX
// };

// 有线手柄摇杆信息
enum HandleAxes{
    LY,
    LX,
    LB,
    RY,
    RX,
    RB,
    KY,
    KX
};
double HandleRocker[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// AGV 手柄运动限制
double AGV_limits[6] = {0.2, 30./180*EIGEN_PI, 1, 20./180*EIGEN_PI, 1, 60./180*EIGEN_PI};

// AGV 手柄运动状态
double AGV_states[2] = {0, 0};

// AGV 控制模式
enum AGVControlMode{
    AGV_Control_Handle,
    AGV_Control_Procedure
}AGV_Control_Mode;

// 宇树电机相关
SerialPort  serial("/dev/ttyUSB0");
MotorCmd    cmd;
MotorData   dataBack;


void setUnitreeMotor(int ID,double rad){
    uint8_t *p;
    
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id    = ID;
    cmd.mode  = 1;
    cmd.K_P   = 1;
    cmd.K_W   = 0.00;
    cmd.Pos   = rad * 6.33;
    cmd.W     = 0;
    cmd.T     = 0.00;
    serial.sendRecv(&cmd,&dataBack);
    p = (uint8_t *)cmd.get_motor_send_data();
    for(int i =0; i<17; i++)
      printf("0X%02X ", *p++);
    cout<<endl;
    
}

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
    for(int i=0; i<11; i++){
        HandleKey[i] = msg->buttons.at(i);
    }
    for(int i=0; i<8; i++){
        HandleRocker[i] = msg->axes.at(i);
    }
    // ROS_DEBUG_STREAM("RM %d" << HandleKey[RM]);
}

// 电机模式设置
void motorModeSet(AGVEnableModeType mode){
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
    if(AGV_HandleRunMode == Ackermann){
        speed *= AGV_limits[0];
        dir *= AGV_limits[1];
        double speed_left,speed_right;
        double dir_leftFront,dir_rightFront,dir_leftBack,dir_rightBack;

        if(speed > AGV_states[0]){
            // 非梯度
            AGV_states[0] = speed;

            // 梯度加速
            // AGV_states[0] += AGV_limits[2] / ControlHz;
            // if(AGV_states[0] > speed)
            //     AGV_states[0] = speed;
        }
        else if(speed < AGV_states[0]){

            // 非梯度
            AGV_states[0] = speed;

            // 梯度降速
            // AGV_states[0] -= AGV_limits[2] / ControlHz;
            // if(AGV_states[0] < speed)
            //     AGV_states[0] = speed;
        }
        speed = AGV_states[0];

        if(dir > AGV_states[1]){
            AGV_states[1] += AGV_limits[3] / ControlHz;
            if(AGV_states[1] > dir)
                AGV_states[1] = dir;
        }
        else if(dir < AGV_states[1]){
            AGV_states[1] -= AGV_limits[3] / ControlHz;
            if(AGV_states[1] < dir)
                AGV_states[1] = dir;
        }
        dir = AGV_states[1];

        if(dir>0){
            double T = TrackWidth[0]/2/tan(dir);
            speed_left  = sqrt((T-TrackWidth[1]/2)*(T-TrackWidth[1]/2) + TrackWidth[0]/2*TrackWidth[0]/2) / sqrt(T*T + TrackWidth[0]/2*TrackWidth[0]/2) * speed;
            speed_right = sqrt((T+TrackWidth[1]/2)*(T+TrackWidth[1]/2) + TrackWidth[0]/2*TrackWidth[0]/2) / sqrt(T*T + TrackWidth[0]/2*TrackWidth[0]/2) * speed;
            dir_leftFront    =  atan2(TrackWidth[0]/2,T-TrackWidth[1]/2);
            dir_rightFront   =  atan2(TrackWidth[0]/2,T+TrackWidth[1]/2);
            dir_leftBack     = -atan2(TrackWidth[0]/2,T-TrackWidth[1]/2);
            dir_rightBack    = -atan2(TrackWidth[0]/2,T+TrackWidth[1]/2);
        }
        else if(dir<0){
            double T = TrackWidth[0]/tan(-dir);
            speed_left  = sqrt((T+TrackWidth[1]/2)*(T+TrackWidth[1]/2) + TrackWidth[0]/2*TrackWidth[0]/2) / sqrt(T*T + TrackWidth[0]/2*TrackWidth[0]/2) * speed;
            speed_right = sqrt((T-TrackWidth[1]/2)*(T-TrackWidth[1]/2) + TrackWidth[0]/2*TrackWidth[0]/2) / sqrt(T*T + TrackWidth[0]/2*TrackWidth[0]/2) * speed;
            dir_leftFront    = -atan2(TrackWidth[0]/2,T+TrackWidth[1]/2);
            dir_rightFront   = -atan2(TrackWidth[0]/2,T-TrackWidth[1]/2);
            dir_leftBack     =  atan2(TrackWidth[0]/2,T+TrackWidth[1]/2);
            dir_rightBack    =  atan2(TrackWidth[0]/2,T-TrackWidth[1]/2);
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

        // 宇树电机
        // setUnitreeMotor(1,dir_leftFront);
        // setUnitreeMotor(2,dir_leftFront);
        // setUnitreeMotor(3,dir_leftFront);
        ROS_DEBUG("AGV 阿克曼运动 %f %f", speed, dir);
    }
    else if(AGV_HandleRunMode == Skewing){
        speed *= AGV_limits[0];
        dir *= AGV_limits[1];
        if(speed > AGV_states[0]){
            AGV_states[0] += AGV_limits[2] / ControlHz;
            if(AGV_states[0] > speed)
                AGV_states[0] = speed;
        }
        else if(speed < AGV_states[0]){
            AGV_states[0] -= AGV_limits[2] / ControlHz;
            if(AGV_states[0] < speed)
                AGV_states[0] = speed;
        }
        speed = AGV_states[0];

        if(dir > AGV_states[1]){
            AGV_states[1] += AGV_limits[3] / ControlHz;
            if(AGV_states[1] > dir)
                AGV_states[1] = dir;
        }
        else if(dir < AGV_states[1]){
            AGV_states[1] -= AGV_limits[3] / ControlHz;
            if(AGV_states[1] < dir)
                AGV_states[1] = dir;
        }
        dir = AGV_states[1];
        if(Mode == Speed){
            rob.motorSetSpeed(1, speed);
            rob.motorSetSpeed(2, speed);
            rob.motorSetSpeed(3, speed);
            rob.motorSetSpeed(4, speed);
            // ROS_INFO("Set HubMotor:speed = [%f],[%f]", speed_left, speed_right);
        }
        rob.stepMotorSetPosition(5, dir);
        rob.stepMotorSetPosition(6, dir);
        rob.stepMotorSetPosition(7, dir);
        rob.stepMotorSetPosition(8, dir);

        // 宇树电机
        // setUnitreeMotor(1,dir);
        // setUnitreeMotor(2,dir);
        // setUnitreeMotor(3,dir);
        ROS_DEBUG("AGV 斜移运动 %f %f", speed, dir);
    }
    else if(AGV_HandleRunMode == Spin){
        speed = dir * AGV_limits[0];
        if(Mode == Speed){
            rob.motorSetSpeed(1, speed);
            rob.motorSetSpeed(2, speed);
            rob.motorSetSpeed(3, speed);
            rob.motorSetSpeed(4, speed);
            // ROS_INFO("Set HubMotor:speed = [%f],[%f]", speed_left, speed_right);
        }
        rob.stepMotorSetPosition(5, 3.1415926535/4*3);
        rob.stepMotorSetPosition(6, -3.1415926535/4*3);
        rob.stepMotorSetPosition(7, 3.1415926535/4);
        rob.stepMotorSetPosition(8, -3.1415926535/4);
        // 宇树电机
        // setUnitreeMotor(1,3.1415926535/4*3);
        // setUnitreeMotor(2,-3.1415926535/4*3);
        // setUnitreeMotor(3,3.1415926535/4);
        ROS_DEBUG("AGV 自旋运动 %f", speed);
    }
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
        AGV_Control(0, 0);
        for(int i=1;i<=8;i++)
            AgvCommond[i] = 0;
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
//     comm_service Uart485;
//     comm_service Uart485_2;
//     cout<<"fd:" << Uart485.CommOpen("/dev/ttyFP0")<<endl;
//     cout<<"comminit:" << Uart485.CommInit(9600, 0, 8, 1, 'N')<<endl;
//     cout<<"fd_2:" << Uart485_2.CommOpen("/dev/ttyFP1")<<endl;
//     cout<<"comminit_2:" << Uart485_2.CommInit(9600, 0, 8, 1, 'N')<<endl;

//     usleep(1000000);

//     char sendData[17] = {1};
//     cout<<Uart485.CommSend(sendData, 17)<<endl;

//  //   cout<<"fd:" << Uart485.CommOpen("/dev/ttyFP1")<<endl;
//   //  cout<<"comminit:" << Uart485.CommInit(4000000, 0, 8, 1, 'N')<<endl;
//     // SerialPort _comUart("/dev/ttyUSB0");
//     char data[100] = {0};

//     for(int i =0; i<17; i++)
//       printf("0X%02X ", sendData[i]);
//     cout<<endl;

//     // setUnitreeMotor(4, 1);
//     usleep(1000000);
//     cout<<Uart485.CommRecv(data, 10)<<endl;

//     // std::thread _thread([]{
//     // setUnitreeMotor(0,1);
//     // });
//     // _thread.detach();
//     // char   _dataBack[100] = {0};

//     // int recvSize = 0;
//     // for(int i = 0; i < 10 && recvSize == 0; i++)
//     // {
//     //     Uart485.CommRecv(_dataBack, 17);
//     //     std::this_thread::sleep_for(std::chrono::seconds(1));
//     // }
   
    
//     cout<< "CommRecv:" << std::endl;//Uart485.CommRecv(data, sizeof(char))<<endl;
//     for(int i =0; i<17; i++)
//       printf("0X%02X ", data[i]);
//     cout<<endl;
//     exit(0);

    setUnitreeMotor(1, 0.8);
    setUnitreeMotor(2, 6);
    setUnitreeMotor(3, 1);
    setUnitreeMotor(4, 2);
    double rad;
    int id;
    while(true){
        cin>>id>>rad;
        cout<<id<<" set "<<rad<<endl;
        setUnitreeMotor(id,rad);
        // 1 0.8
        // 2 -0.1
        // 4 0.05
    }
    

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

    // 宇树电机相关
    // SerialPort  serial("/dev/ttyUSB0");
    // MotorCmd    cmd;
    // MotorData   data;
    // double angle;
    // double change = 0.3;


  
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

    // auto  tp_1 = std::chrono::steady_clock::now();
    // auto  tp_2 = std::chrono::steady_clock::now();
    // auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2- tp_1).count();
    // std::cout << "time:" << track_time << "s" << std::endl;

    while(true){
        
        // 宇树电机相关
        // angle += change;
        // if(angle > 3.1415926 * 6.33){
        //     angle = 3.1415926 * 6.33;
        //     change = -0.3;
        // }
        // else if(angle < 0){
        //     angle = 0;
        //     change = +0.3;
        // }
        // cmd.motorType = MotorType::GO_M8010_6;
        // // cmd.id    = 1;
        // cmd.mode  = 1;
        // cmd.K_P   = 0.3;
        // cmd.K_W   = 0.01;
        // cmd.Pos   = angle;
        // cmd.W     = 0;
        // cmd.T     = 0.00;
        // for(int id=1;id<=3;id++){
        //     cmd.id    = id;
        //     serial.sendRecv(&cmd,&data);
        //     // if(data.correct == true){
        //     //     std::cout <<  std::endl;
        //     //     std::cout <<  "motor.id: "     << cmd.id      << std::endl;
        //     //     std::cout <<  "motor.Pos: "    << data.Pos    << " rad" << std::endl;
        //     //     std::cout <<  "motor.Temp: "   << data.Temp   << " ℃"  << std::endl;
        //     //     std::cout <<  "motor.W: "      << data.W      << " rad/s"<<std::endl;
        //     //     std::cout <<  "motor.T: "      << data.T      << " N.m" << std::endl;
        //     //     std::cout <<  "motor.MError: " << data.MError <<  std::endl;
        //     //     std::cout <<  std::endl;
        //     // }
        // }
        
        


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
            // if(AgvCommond[0] != Maintain){
            //     // ROS_INFO("开始切换模式");
            //     // Mode = AgvCommond[0];
            //     motorModeSet(AgvCommond[0]);
            // }
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
            // tp_1 = std::chrono::steady_clock::now();
            rob.stepMotorSetPosition(8, AgvCommond[8]);
            // tp_2 = std::chrono::steady_clock::now();
            // track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2- tp_1).count();
            // std::cout << "time:" << track_time << "s" << std::endl;
            ROS_DEBUG("Set StepMotor:position = [%f],[%f],[%f],[%f]", AgvCommond[5], AgvCommond[6], AgvCommond[7], AgvCommond[8]);

            // ROS_INFO("--------------------------------------------------\r\n");
            
        }
        // 手柄控制
        else if(AGV_Control_Mode == AGV_Control_Handle){
            // ROS_INFO("RB: %f",HandleRocker[RB]);
            if(HandleRocker[RB]<0){
                // 电机模式设置

                if(HandleKey[A] == 1){
                    if(Mode == Speed)
                        motorModeSet(Disability);
                    else
                        motorModeSet(Speed);
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleKey[A] == 0)
                            break;
                    }
                }

                if(HandleKey[B] == 1){
                    AGV_HandleRunMode = Ackermann;
                    ROS_INFO("切换成阿克曼");
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleKey[B] == 0)
                            break;
                    }
                }
                if(HandleKey[X] == 1){
                    AGV_HandleRunMode = Skewing;
                    ROS_INFO("切换成斜移");
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleKey[X] == 0)
                            break;
                    }
                }
                if(HandleKey[Y] == 1){
                    AGV_HandleRunMode = Spin;
                    ROS_INFO("切换成自旋");
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleKey[Y] == 0)
                            break;
                    }
                }
                
                // 修改速度极限
                if(HandleRocker[KX] > 0.5){
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleRocker[KX] < 0.5)
                            break;
                    }
                    AGV_limits[0] += 0.1;
                    if(AGV_limits[0] > AGV_limits[4])
                        AGV_limits[0] = AGV_limits[4];
                }
                else if(HandleRocker[KX] < -0.5){
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleRocker[KX] > -0.5)
                            break;
                    }
                    AGV_limits[0] -= 0.1;
                    if(AGV_limits[0] < 0.1)
                        AGV_limits[0] = 0.1;
                }

                if(HandleRocker[KY] > 0.5){
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleRocker[KY] < 0.5)
                            break;
                    }
                    AGV_limits[1] += 10./180*EIGEN_PI;
                    if(AGV_limits[1] > AGV_limits[5])
                        AGV_limits[1] = AGV_limits[5];
                }
                else if(HandleRocker[KY] < -0.5){
                    while(true){
                        delay_rate.sleep();
                        ros::spinOnce();
                        if(HandleRocker[KY] > -0.5)
                            break;
                    }
                    AGV_limits[1] -= 10./180*EIGEN_PI;
                    if(AGV_limits[1] < 10./180*EIGEN_PI)
                        AGV_limits[1] = 10./180*EIGEN_PI;
                }

                AGV_Control(HandleRocker[LX], HandleRocker[RY]);
                
                // ROS_INFO("--------------------------------------------------\r\n");
            }
            else
                AGV_Control(0, 0);
            
        }
        
        // ROS_INFO("--------------------------------------------------\r\n\r\n");

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
} 
       