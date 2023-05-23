#include "HubMotor.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <signal.h>
#define msleep(ms)  usleep((ms)*1000)


HubMotor rob;
enum
{
    Disability,
    Speed,
    Position
};
int Mod = Disability;

void HubMotorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(Mod != msg->data.at(0))
    {
        ROS_INFO("开始切换模式");
        Mod = msg->data.at(0);
        rob.motorDisEnable(1);
        rob.motorDisEnable(2);
        rob.motorDisEnable(3);
        rob.motorDisEnable(4);
        usleep(3000000);
        if(Mod == Speed)
        {
            // 设置速度控制模式
            rob.motorInit(1,rob.SpeedMod);
            rob.motorInit(2,rob.SpeedMod);
            rob.motorInit(3,rob.SpeedMod);
            rob.motorInit(4,rob.SpeedMod);
            ROS_INFO("设置速度模式");
        }
        else if(Mod == Position)
        {
            // 设置位置控制模式
            rob.motorInit(1,rob.PositionMod);
            rob.motorInit(2,rob.PositionMod);
            rob.motorInit(3,rob.PositionMod);
            rob.motorInit(4,rob.PositionMod);
            ROS_INFO("设置位置模式");
        }
        else if(Mod == Disability)
        {
            // 设置失能模式
            // rob.motorDisEnable(1);
            // rob.motorDisEnable(2);
            // rob.motorDisEnable(3);
            // rob.motorDisEnable(4);
            ROS_INFO("设置失能模式");
        }
    }
    if(Mod == Speed)
    {
        rob.motorSetSpeed(1,-msg->data.at(1));
        rob.motorSetSpeed(2,-msg->data.at(2));
        rob.motorSetSpeed(3,msg->data.at(3));
        rob.motorSetSpeed(4,msg->data.at(4));
        ROS_INFO("HubMotor:speed = [%f],[%f],[%f],[%f]", msg->data.at(1),msg->data.at(2),msg->data.at(3),msg->data.at(4));
    }
    else if(Mod == Position)
    {
        rob.motorSetPosition(1,-msg->data.at(1));
        rob.motorSetPosition(2,-msg->data.at(2));
        rob.motorSetPosition(3,msg->data.at(3));
        rob.motorSetPosition(4,msg->data.at(4));
        ROS_INFO("HubMotor:position = [%f],[%f],[%f],[%f]", msg->data.at(1),msg->data.at(2),msg->data.at(3),msg->data.at(4));
    }
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
    ros::init(argc, argv, "HubMotor");  //解析参数，命名结点
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
    rob.motorDisEnable(1);
    rob.motorDisEnable(2);
    rob.motorDisEnable(3);
    rob.motorDisEnable(4);
    ROS_INFO("轮毂电机已连接");

    // 设置速度控制模式
    // rob.motorInit(1,rob.SpeedMod);
    // rob.motorInit(2,rob.SpeedMod);
    // rob.motorInit(3,rob.SpeedMod);
    // rob.motorInit(4,rob.SpeedMod);

    signal(SIGINT, HubMotorExit);

    ros::Subscriber sub = nh.subscribe("HubControl", 1000, HubMotorCallback);
    ros::spin();
    return 0;

    // rob.motorChangeTrapezoidalVelocityInPosition(3,50);
    // rob.motorSetPosition(3,10);
    // usleep(3000000);
    // rob.motorSetPosition(3,00);
    // usleep(3000000);
    // rob.motorDisEnable(3);

    // exit(1);

    //speed
    // rob.motorInit(1,rob.SpeedMod);
    // rob.motorInit(2,rob.SpeedMod);
    // rob.motorInit(3,rob.SpeedMod);
    // rob.motorInit(4,rob.SpeedMod);

    // rob.motorChangeUpAccelerationInSpeed(1,20);

    // rob.motorSetSpeed(1,10);
    // rob.motorSetSpeed(2,10);
    // rob.motorSetSpeed(3,10);
    // rob.motorSetSpeed(4,10);
    // usleep(2000000);

    // rob.motorSetSpeed(1,0);
    // rob.motorSetSpeed(2,0);
    // rob.motorSetSpeed(3,0);
    // rob.motorSetSpeed(4,0);
    // usleep(2000000);

    // rob.motorDisEnable(1);
    // rob.motorDisEnable(2);
    // rob.motorDisEnable(3);
    // rob.motorDisEnable(4);

    // return 0;
} 
       