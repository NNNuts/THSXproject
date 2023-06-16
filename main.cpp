#include <iostream>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include "controller.hpp"
#include "kinematics.hpp"
#include<fstream>
#include <Eigen/Dense>
#include<ctime>
#include "CAN2USB.hpp"
#define msleep(ms)  usleep((ms)*1000)

using namespace std; 
using namespace std::chrono;
using namespace Eigen;

Target_data Tar;
TrainComputer TC;
CAN2USB rob;
void MoveL(double X,double Y,double Z,double delay_s);
void MoveJ(double X,double Y,double Z,double delay_s);


void trackRunning(void)
{
    while(true)
    {
        this_thread::sleep_for(std::chrono::microseconds(10000));
        if(Tar.trackMod)
        {
            Tar.MoveL_Trackmod_LadderShaped();
            ofstream OutFile("Test.txt",ios::app); //利用构造函数创建txt文本，并且打开该文本
            OutFile << TC.theta_now[0]<< " " << TC.theta_now[1]<< " "  << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << Tar.present_V[0] << " " << endl; 
            //OutFile << "This is a Test12!" << endl; //把字符串内容"This is a Test!"，写入Test.txt文件
            OutFile.close(); //关闭Test.txt文件
            // exit(0);
        }
    }
}

int main(int argc, char* argv[])
{
    
    cout << "系统启动" << endl;

    rob.canOpen();
    rob.canInit();
    rob.canStart();

    
    // VCI_CAN_OBJ rec[3000];
    // cout<<VCI_Receive(VCI_USBCAN2,0,0,rec,3000,100)<<endl;
    // usleep(1000000);
    // cout<<VCI_Receive(VCI_USBCAN2,0,0,rec,3000,100)<<endl;
    // exit(0);

    // rob.robotReset();
    // rob.robotInit();


    //-------------------------
    // rob.robotSetPosition(1,rob.deg2rad(0));
    // rob.robotSetPosition(2,rob.deg2rad(-90));
    // rob.robotSetPosition(3,rob.deg2rad(0));
    // rob.robotSetPosition(4,rob.deg2rad(-90));
    // rob.robotSetPosition(5,rob.deg2rad(0));
    // rob.robotSetPosition(6,rob.deg2rad(0));

    // exit(0);
    //-------------------------


    // ofstream OutFile("Test.txt",ios::out) ; //利用构造函数创建txt文本，并且打开该文本
    // OutFile.clear();
    // OutFile.close(); //关闭Test.txt文件


    TC.theta_now[0] = 0; 
    TC.theta_now[1] = rob.deg2rad(-60);
    TC.theta_now[2] = rob.deg2rad(-60);
    TC.theta_now[3] = rob.deg2rad(-60);
    TC.theta_now[4] = 0;
    TC.theta_now[5] = 0;

    // TC.theta_now[0] = 0; 
    // TC.theta_now[1] = rob.deg2rad(-60);
    // TC.theta_now[2] = rob.deg2rad(-60);
    // TC.theta_now[3] = rob.deg2rad(-60);
    // TC.theta_now[4] = 0;
    // TC.theta_now[5] = 0;
    // rob.robotSetPositionAll(TC.theta_now);
    // usleep(2000000);
    //rob.robotSetPosition(3,rob.deg2rad(0));
    // exit(0);
    // msleep(5000);

    rob.robotReadPositionAll(TC.theta_now);

    // TC.theta_now[0] = 0;
    cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;
    for(int i = 0;i < 6;i++)
    {
        if(TC.theta_now[i]>10 || TC.theta_now[i]<-10)
        {
            cout<<"通讯失败"<<endl;
            exit(0);
        }
    }
    
    Matrix4d mat = TC.kinematics(TC.theta_now);
    cout << mat << endl;
    Rpy rpy = TC.Matrix2Rpy(mat.block<3, 3>(0, 0));
    cout << rpy << endl << endl;
    // MoveL(-200, -500, 800, 1);
    // MoveJ(-200, -500, 1200, 1);
    // exit(0);
    // MoveL(200, -500, 1100, 1);
    // MoveL(400, -500, 1100, 1);
    // MoveL(400, -300, 1100, 1);
    // MoveL(200, -300, 1100, 1);
    while(true)
    {
        MoveL(-200, -500, 800, 1);
        MoveL(-200,  500, 800, 1);
        MoveL(-400,  500, 800, 1);
        MoveL(-400, -500, 800, 1);
        MoveL(-600, -500, 800, 1);
        MoveL(-600,  500, 800, 1);
        MoveL(-800,  500, 800, 1);
        MoveL(-800, -500, 800, 1);
    }
    // while(true)
    // {
    //     MoveJ(200, -500, 900, 1);
    //     MoveJ(400, -500, 900, 1);
    //     MoveJ(400, -300, 900, 1);
    //     MoveJ(200, -300, 900, 1);
    // }

    // Tar.PathInit();
    // cout << "Init ok" << endl;
    // Tar.SetPathPoint(200, -500, 900, 0, rob.deg2rad(-90), 0,rob.deg2rad(180));
    // cout << "Path Set ok" << endl;
    // clock_t start, end;
    // start = clock();

    // Tar.Move_End_LadderShaped();
    // Tar.Move_MoveJ_CubicPolynomial();

    // end = clock();
    // double endtime = (double)(end - start) / CLOCKS_PER_SEC;
    // cout << "Total time: " << endtime << "s" << endl;		//s为单位

    return 0;
} 

//输入：X，Y，Z 为目标点位置（mm）
//输出：无
//说明：MoveL方式移动
//特别：
void MoveL(double X,double Y,double Z,double delay_s)
{
    double pos[2];
    // TC.kinematics(TC.theta_now);
    Tar.PathInit();
    Tar.SetPathPoint(X, Y, Z, 0, rob.deg2rad(0), 0,rob.deg2rad(0));
    Tar.Move_End_LadderShaped();
    usleep(delay_s * 1000000);
}

//输入：X，Y，Z 为目标点位置（mm）
//输出：无
//说明：MoveJ方式移动
//特别：
void MoveJ(double X,double Y,double Z,double delay_s)
{
    double pos[2];
    // TC.kinematics(TC.theta_now);
    Tar.PathInit();
    Tar.SetPathPoint(X, Y, Z, 0, rob.deg2rad(0), 0,rob.deg2rad(0));
    Tar.Move_MoveJ_CubicPolynomial();
    usleep(delay_s * 1000000);
}