#include "HubMotor.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <signal.h>
#define msleep(ms)  usleep((ms)*1000)


HubMotor rob;
HubMotor rob1(1);
double AgvCommond[9] = {0,0,0,0,0,0,0,0,0};
enum
{
    Disability,
    Speed,
    Position
};
int Mod = Disability;

void HubMotorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    // cout<<"ok"<<endl;
    for(int i = 0; i < 9; i++)
        AgvCommond[i] = msg->data.at(i);
}

void HubMotorExit(int sig)
{
	//这里进行退出前的数据保存、内存清理、告知其他节点等工作
    rob.canClose();
	ROS_INFO("HubMotor shutting down!");
	ros::shutdown();
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Agv");  //解析参数，命名结点
    ros::NodeHandle nh;  //创建句柄，实例化node

    //exit(0);
    // cout << "系统启动" << endl;
    
    setlocale(LC_ALL, "");
    ROS_INFO("Warning!");
    ROS_INFO("退出时，务必使用cril+C进行退出，否则会产生驱动冲突");
    ROS_INFO("冲突时需重启电脑");
    rob.canOpen();
    rob.canInit();
    rob.canStart();

    rob1.canInit();
    rob1.canStart();

    rob.motorDisEnable(1);
    rob.motorDisEnable(2);
    rob.motorDisEnable(3);
    rob.motorDisEnable(4);

    rob1.stepMotorSetPosition(5,0);
    rob1.stepMotorSetPosition(6,0);
    rob1.stepMotorSetPosition(7,0);
    rob1.stepMotorSetPosition(8,0);
    usleep(1000000);
    ROS_INFO("轮毂电机已连接");

    // exit(0);


    signal(SIGINT, HubMotorExit);
    ros::Rate rate(20);  
    ros::Publisher AgvData_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvData", 1000);
    
    ros::Subscriber sub = nh.subscribe("AgvControl", 1000, HubMotorCallback);
    while(true){
        rate.sleep();
        rob1.stepMotorReadPosition();
        rob.hubMotorReadPosition();
        std_msgs::Float32MultiArray AgvData;
        AgvData.data.push_back(Mod);
        for(int i = 0; i < 4; i++)
            AgvData.data.push_back(rob.hubMotorRealSpeed[i]);
        for(int i = 0; i < 4; i++)
            AgvData.data.push_back(rob1.stepMotorRealPosition[4 + i]);
        AgvData_pub.publish(AgvData);
        if(Mod == Disability)
            ROS_INFO("Pub HubMotor:Disability");
        else if(Mod == Speed)
            ROS_INFO("Pub HubMotor:Speed = [%f],[%f],[%f],[%f]", rob.hubMotorRealSpeed[0], rob.hubMotorRealSpeed[1], rob.hubMotorRealSpeed[2], rob.hubMotorRealSpeed[3]);
        ROS_INFO("Pub StepMotor:position = [%f],[%f],[%f],[%f]", rob1.stepMotorRealPosition[4], rob1.stepMotorRealPosition[5], rob1.stepMotorRealPosition[6], rob1.stepMotorRealPosition[7]);
        
        ros::spinOnce();
        if(Mod != AgvCommond[0]){
            // ROS_INFO("开始切换模式");
            Mod = AgvCommond[0];
            if(Mod == Speed){
                // 设置速度控制模式
                rob.motorInit(1, rob.SpeedMod);
                rob.motorInit(2, rob.SpeedMod);
                rob.motorInit(3, rob.SpeedMod);
                rob.motorInit(4, rob.SpeedMod);
                ROS_INFO("设置速度模式");
            }
            else if(Mod == Position){
                // 设置位置控制模式
                rob.motorInit(1, rob.PositionMod);
                rob.motorInit(2, rob.PositionMod);
                rob.motorInit(3, rob.PositionMod);
                rob.motorInit(4, rob.PositionMod);
                ROS_INFO("设置位置模式");
            }
            else if(Mod == Disability){
                // 设置失能模式
                rob.motorDisEnable(1);
                rob.motorDisEnable(2);
                rob.motorDisEnable(3);
                rob.motorDisEnable(4);
                ROS_INFO("设置失能模式");
            }
        }
        if(Mod == Speed){
            rob.motorSetSpeed(1, AgvCommond[1]);
            rob.motorSetSpeed(2, AgvCommond[2]);
            rob.motorSetSpeed(3, AgvCommond[3]);
            rob.motorSetSpeed(4, AgvCommond[4]);
            ROS_INFO("Set HubMotor:speed = [%f],[%f],[%f],[%f]", AgvCommond[1], AgvCommond[2], AgvCommond[3], AgvCommond[4]);
        }
        else if(Mod == Position)
        {
            rob.motorSetPosition(1, AgvCommond[1]);
            rob.motorSetPosition(2, AgvCommond[2]);
            rob.motorSetPosition(3, AgvCommond[3]);
            rob.motorSetPosition(4, AgvCommond[4]);
            ROS_INFO("Set HubMotor:position = [%f],[%f],[%f],[%f]", AgvCommond[1], AgvCommond[2], AgvCommond[3], AgvCommond[4]);
        }
        rob1.stepMotorSetPosition(5, AgvCommond[5]);
        rob1.stepMotorSetPosition(6, AgvCommond[6]);
        rob1.stepMotorSetPosition(7, AgvCommond[7]);
        rob1.stepMotorSetPosition(8, AgvCommond[8]);
        ROS_INFO("Set StepMotor:position = [%f],[%f],[%f],[%f]", AgvCommond[5], AgvCommond[6], AgvCommond[7], AgvCommond[8]);
    }
    return 0;
} 
       