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
//     double a[6] = {0,0,1,0,0,0};
//     double b[6] = {0,0,0.9,0,0,0};
//    Matrix4d T = TC.kinematics(a);
//    cout<<T<<endl;
// //    exit(0);
//    TC.iterate_ikine(b,T,b);
//    cout<<b[0]<<" "<<b[1]<<" "<<b[2]<<" "<<b[3]<<" "<<b[4]<<" "<<b[5]<<endl;
//    T = TC.kinematics(b);
//    cout<<T<<endl;
//    exit(0);

    // rob.canOpen();
    // rob.canInit();
    // rob.canStart();

    // rob.robotReset();
    // rob.robotInit();

    //  rob.robotSetPosition(2,rob.deg2rad(-0));


    // double rad[6] = {0, rob.deg2rad(-90), 0, rob.deg2rad(-90), 0, 0};
    // rob.robotSetPositionAll(rad);
    // rob.robotReadPositionAll(rad);
    // cout << rad[0] << " " << rad[1] << " " << rad[2] << " " << rad[3] << " " << rad[4] << " " << rad[5] << " " << endl;

    // exit(0);


    //exit(0);
    cout << "系统启动" << endl;

    rob.canOpen();
    // rob.canInit();
    rob.canStart();

    // rob.robotReset();
    // rob.robotInit();

    ofstream OutFile("Test.txt",ios::out); //利用构造函数创建txt文本，并且打开该文本
    OutFile.clear();
    OutFile.close(); //关闭Test.txt文件

    double rad[6] = {0, rob.deg2rad(-60), rob.deg2rad(-60), rob.deg2rad(-60), rob.deg2rad(0), 0};
    rob.robotSetPositionAll(rad);
    msleep(10000);

    rob.robotReadPositionAll(TC.theta_now);

    // TC.theta_now[0] = 0;
    cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;

    Matrix4d mat = TC.kinematics(TC.theta_now);
    cout<<mat<<endl;
    Rpy rpy = TC.Matrix2Rpy(mat.block<3, 3>(0, 0));
    cout<<rpy<<endl;

// exit(1);

    // TC.theta_now[0] = 0 * EIGEN_PI / 180;
    // TC.theta_now[1] = -135 * EIGEN_PI / 180;
    // TC.theta_now[2] = 90 * EIGEN_PI / 180;
    // TC.theta_now[3] = 45 * EIGEN_PI / 180;
    // TC.theta_now[4] = 90 * EIGEN_PI / 180;
    // TC.theta_now[5] = 0 * EIGEN_PI / 180;

    
    

    // thread th1(trackRunning);
    // th1.detach();

    // Tar.TrackInit();
    // Tar.setTarget(-100, -100, 200, 90. / 180 * EIGEN_PI, 0., -90. / 180 * EIGEN_PI);
    
    // Tar.trackMod = true;
    // for(int i =0;i<50000;i++)
    // {
    //     this_thread::sleep_for(std::chrono::microseconds(100));
    // }
    // Tar.setTarget(-140, -100, 200, 50. / 180 * EIGEN_PI, 0., -90. / 180 * EIGEN_PI);
    // for(int i =0;i<40000;i++)
    // {
    //     this_thread::sleep_for(std::chrono::microseconds(100));
    // }


    // exit(0);

    Tar.PathInit();
    cout << "Init ok" << endl;
    Tar.SetPathPoint(-100, -400, 1346, 0, rob.deg2rad(-90), 0.,rob.deg2rad(180));
    cout << "Path Set ok" << endl;
    // clock_t start, end;
    // start = clock();

    Tar.Move_End_LadderShaped();
    //Tar.Move_MoveJ_CubicPolynomial();

    // end = clock();
    // double endtime = (double)(end - start) / CLOCKS_PER_SEC;
    // cout << "Total time: " << endtime << "s" << endl;		//s为单位

    return 0;
} 
