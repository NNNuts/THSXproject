#include <iostream>
#include <stdio.h>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include <curses.h>
#include "controller.hpp"
#include "kinematics.hpp"
#include <fstream>
#include <Eigen/Dense>
#include <ctime>
// #include "Can232.hpp"
#include "controlcan.h"
#include "CAN2USB.hpp"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

Target_data Tar;
TrainComputer TC;
// CAN232 rob;
CAN2USB rob;
void DrawM(int times);
void DrawZ(int times);
void UI_MoveJ(int X, int Y,int Z,int delay_s);
void UI_MoveL(int X, int Y,int Z,int delay_s);
void MoveL(double initX,double initY,double initZ,double goX,double goY,double goZ,double delay_s);
void MoveJ(double initX,double initY,double initZ,double goX,double goY,double goZ,double delay_s);
void SpeedUp(int delay_s);
int PositionRecord[3][10];
int PositionNum = 0;

// void trackRunning(void)
// {
//     while (true)
//     {
//         this_thread::sleep_for(std::chrono::microseconds(10000));
//         if (Tar.trackMod)
//         {
//             Tar.MoveL_Trackmod_LadderShaped();
//             // ofstream OutFile("Test.txt",ios::app); //利用构造函数创建txt文本，并且打开该文本
//             // OutFile << TC.theta_now[0]<< " " << TC.theta_now[1]<< " "  << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << Tar.present_V[0] << " " << endl;
//             // OutFile.close(); //关闭Test.txt文件
//             // exit(0);
//         }
//     }
// }

int main(int argc, char *argv[])
{
    cout<<"系统初始化。。。"<<endl;

    rob.canOpen();
    rob.canStart();

    rob.motorChangPID(1,30,0,0,500000);
    rob.motorChangPID(2,30,0,0,500000);

    cout << "系统启动" << endl<< endl;

    // --------------------test--------------------------
    TC.invest(-5000,5000,5000,TC.theta_now);
    auto t1 = std::chrono::high_resolution_clock::now();
    rob.robotSetPositionAll(TC.theta_now);
    auto t3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t3-t1;
    cout<<"实际使用时间 "<<fp_ms.count()<<"ms"<<endl<<endl;
    // exit(0);
    usleep(2000000);
    MoveJ(-5000,5000,5000, 5000,5000,-5000,2);
    MoveL( 5000,5000,-5000,-5000,5000,5000,2);
    exit(0);
    //-----------------测试运行结果-----------------
    // 总计26个控制点
    // 理论使用时间 260 ms
    // 实际使用时间 ??? ms
    // 总计32个控制点
    // 理论使用时间 320 ms
    // 实际使用时间 ??? ms

    //-------------------修改说明-----------------------
    //优化了时间计算方式，重新设计了计时函数，确保了计时及速度计算准确性
    //优化了终端显示，突出了信息显示交互
    //MoveL已取消加减速过程，且电机极限速度已提高了十倍，Pid参数提高十倍，尽可能贴近电机极限
    //当前MoveL设计速度为32 m/s  MoveJ设计速度为6.28 rad/s
    //如果实际使用时间可以和理论时间对应，且机械臂可以与程序同时结束，即看到显示运动结束的同时机械臂同时停止
    //则机械臂完成 6.28rad/s 计划目标

    //------------------正常工作区---------------------
    UI_MoveL(3000,5000,2000,1);
    // SpeedUp(1);
    // DrawM(3);
    // DrawZ(3);
    return 0;
}

void SpeedUp(int delay_s)
{
    TC.invest(-3000,5000,2000,TC.theta_now);
    rob.robotSetPositionAll(TC.theta_now);
    usleep(2000000);
    while(1)
    {
        MoveJ(-3000,5000,2000,3000,5000,2000,delay_s);
        MoveJ(3000,5000,2000,-3000,5000,2000,delay_s);
    }
}



void DrawM(int times)
{
    double x = 0;
    double y = 5000;
    double z = -1000;
    double theta[2];
    TC.invest(x,y,z,TC.theta_now);
    rob.robotSetPositionAll(TC.theta_now);
    usleep(2000000);

    for (int i = 0; i < times; i++)
    {
        MoveL(0, 5000, -1000, 4000, 5000, -2000, 1);
        MoveL(4000, 5000, -2000, 0, 5000, -1000, 1);
    }
}

void DrawZ(int times)
{
    double x = 0;
    double y = 5000;
    double z = 1000;
    double theta[2];
    TC.invest(x,y,z,TC.theta_now);
    rob.robotSetPositionAll(TC.theta_now);
    usleep(2000000);
    for (int i = 0; i < times; i++)
    {
        MoveL(0, 5000, 1000, 1000, 5000, 1000, 1);
        MoveL(1000, 5000, 1000, 0, 5000, -200, 1);
        MoveL(0, 5000, -200, 1000, 5000, -200, 1);

        MoveL(1000, 5000, -200, 0, 5000, -200, 1);
        MoveL(0, 5000, -200, 1000, 5000, 1000, 1);
        MoveL(1000, 5000, 1000, 0, 5000, 1000, 1);

    }
}

void UI_MoveJ(int X, int Y,int Z,int delay_s)
{
    uint ch;
    double theta[2];
    int big = 50;
    int change = big;
    char line[6];
    TC.invest(X,Y,Z,TC.theta_now);
    rob.robotSetPositionAll(TC.theta_now);
    usleep(2000000);
    while (1)
    {
        initscr();
        ch = getch();
        // endwin();
        switch (ch)             
        {
            case 'a':
                X -= change;
            // cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break;

            case 'd':
                X += change;
        //    cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break;

            case 's':
                Z -= change;
            // cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break;

            case 'w':
                Z += change;
            //cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break; 

            case 'q':
                PositionRecord[0][PositionNum] = X;
                PositionRecord[1][PositionNum] = Y;
                PositionRecord[2][PositionNum] = Z;
                PositionNum ++;
                endwin();
                cout<<endl<<"NO."<<PositionNum<<" set finish"<<endl;
                usleep(500000);
                doupdate();
                break;

            case 'z':
                endwin();
                cout<<endl<<"速度已切换"<<endl;
                usleep(500000);
                doupdate();
                change /= 10;
                if(!change)
                {
                    change = big;
                }
                break;

            case 'x':
            {
                endwin();
                if(PositionNum<2)
                {
                    cout<<endl<<"路经点过少，程序退出"<<endl;
                    exit(1);
                }
                cout<<endl<<"保存成功"<<endl;
                usleep(500000);
                doupdate();
                ofstream file("data.txt",ios::out); 
                file << PositionNum << endl; 
                for(int i=0;i<PositionNum;i++)
                {
                    file << PositionRecord[0][i] << endl; 
                    file << PositionRecord[1][i] << endl; 
                    file << PositionRecord[2][i] << endl; 
                }
                file.close(); //关闭Test.txt文件
                break;
            }

            case 'c':
            {
                endwin();
                cout<<endl<<"读取成功"<<endl;
                ifstream file("data.txt",ios::in);
                file.getline(line, 6);
                PositionNum = stoi(line);
                for(int i=0;i<PositionNum;i++)
                {
                    file.getline(line, 6);
                    PositionRecord[0][i] = stoi(line);
                    file.getline(line, 6);
                    PositionRecord[1][i] = stoi(line);
                    file.getline(line, 6);
                    PositionRecord[2][i] = stoi(line);
                }
                file.close();
                TC.invest(PositionRecord[0][0],PositionRecord[1][0],PositionRecord[2][0],TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                usleep(2000000);
                while(1)
                {
                    for(int i=1;i<PositionNum;i++)
                    {
                        MoveJ(PositionRecord[0][i-1], PositionRecord[1][i-1], PositionRecord[2][i-1], PositionRecord[0][i], PositionRecord[1][i], PositionRecord[2][i], delay_s);
                    }
                    MoveJ(PositionRecord[0][PositionNum-1], PositionRecord[1][PositionNum-1], PositionRecord[2][PositionNum-1], PositionRecord[0][0], PositionRecord[1][0], PositionRecord[2][0], delay_s);
                }
                break;
            }

            case 'e':
                endwin();
                if(PositionNum<2)
                {
                    cout<<endl<<"路经点过少，程序退出"<<endl;
                    exit(1);
                }
                TC.invest(PositionRecord[0][0],PositionRecord[1][0],PositionRecord[2][0],TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                usleep(2000000);
                while(1)
                {
                    for(int i=1;i<PositionNum;i++)
                    {
                        MoveJ(PositionRecord[0][i-1], PositionRecord[1][i-1], PositionRecord[2][i-1], PositionRecord[0][i], PositionRecord[1][i], PositionRecord[2][i], delay_s);
                    }
                    MoveJ(PositionRecord[0][PositionNum-1], PositionRecord[1][PositionNum-1], PositionRecord[2][PositionNum-1], PositionRecord[0][0], PositionRecord[1][0], PositionRecord[2][0], delay_s);
                }
        }
    }
}

void UI_MoveL(int X, int Y,int Z,int delay_s)
{
    uint ch;
    double theta[2];
    int big = 50;
    int change = big;
    char line[6];
    TC.invest(X,Y,Z,TC.theta_now);
    rob.robotSetPositionAll(TC.theta_now);
    usleep(2000000);
    while (1)
    {
        initscr();
        ch = getch();
        // endwin();
        switch (ch)             
        {
            case 'a':
                X -= change;
            // cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break;

            case 'd':
                X += change;
        //    cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break;

            case 's':
                Z -= change;
            // cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break;

            case 'w':
                Z += change;
            //cout<<endl<<X<<" "<<Y<<" "<<Z<<endl<<endl;
                TC.invest(X,Y,Z,TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                break; 

            case 'q':
                PositionRecord[0][PositionNum] = X;
                PositionRecord[1][PositionNum] = Y;
                PositionRecord[2][PositionNum] = Z;
                PositionNum ++;
                endwin();
                cout<<endl<<"NO."<<PositionNum<<" set finish"<<endl;
                usleep(500000);
                doupdate();
                break;

            case 'z':
                endwin();
                cout<<endl<<"速度已切换"<<endl;
                usleep(500000);
                doupdate();
                change /= 10;
                if(!change)
                {
                    change = big;
                }
                break;

            case 'x':
            {
                endwin();
                if(PositionNum<2)
                {
                    cout<<endl<<"路经点过少，程序退出"<<endl;
                    exit(1);
                }
                cout<<endl<<"保存成功"<<endl;
                usleep(500000);
                doupdate();
                ofstream file("data.txt",ios::out); 
                file << PositionNum << endl; 
                for(int i=0;i<PositionNum;i++)
                {
                    file << PositionRecord[0][i] << endl; 
                    file << PositionRecord[1][i] << endl; 
                    file << PositionRecord[2][i] << endl; 
                }
                file.close(); //关闭Test.txt文件
                break;
            }

            case 'c':
            {
                endwin();
                cout<<endl<<"读取成功"<<endl;
                ifstream file("data.txt",ios::in);
                file.getline(line, 6);
                PositionNum = stoi(line);
                for(int i=0;i<PositionNum;i++)
                {
                    file.getline(line, 6);
                    PositionRecord[0][i] = stoi(line);
                    file.getline(line, 6);
                    PositionRecord[1][i] = stoi(line);
                    file.getline(line, 6);
                    PositionRecord[2][i] = stoi(line);
                }
                file.close();
                TC.invest(PositionRecord[0][0],PositionRecord[1][0],PositionRecord[2][0],TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                usleep(2000000);
                while(1)
                {
                    for(int i=1;i<PositionNum;i++)
                    {
                        MoveL(PositionRecord[0][i-1], PositionRecord[1][i-1], PositionRecord[2][i-1], PositionRecord[0][i], PositionRecord[1][i], PositionRecord[2][i], delay_s);
                    }
                    MoveL(PositionRecord[0][PositionNum-1], PositionRecord[1][PositionNum-1], PositionRecord[2][PositionNum-1], PositionRecord[0][0], PositionRecord[1][0], PositionRecord[2][0], delay_s);
                }
                break;
            }

            case 'e':
                endwin();
                if(PositionNum<2)
                {
                    cout<<endl<<"路经点过少，程序退出"<<endl;
                    exit(1);
                }
                TC.invest(PositionRecord[0][0],PositionRecord[1][0],PositionRecord[2][0],TC.theta_now);
                rob.robotSetPositionAll(TC.theta_now);
                usleep(2000000);
                while(1)
                {
                    for(int i=1;i<PositionNum;i++)
                    {
                        MoveL(PositionRecord[0][i-1], PositionRecord[1][i-1], PositionRecord[2][i-1], PositionRecord[0][i], PositionRecord[1][i], PositionRecord[2][i], delay_s);
                    }
                    MoveL(PositionRecord[0][PositionNum-1], PositionRecord[1][PositionNum-1], PositionRecord[2][PositionNum-1], PositionRecord[0][0], PositionRecord[1][0], PositionRecord[2][0], delay_s);
                }
        }
    }
}

void MoveL(double initX,double initY,double initZ,double goX,double goY,double goZ,double delay_s)
{
    Tar.PathInit(initX, initY, initZ);
    Tar.SetPathPoint(goX, goY, goZ);
    Tar.Move_End_LadderShaped();
    usleep(delay_s * 1000000);
}

void MoveJ(double initX,double initY,double initZ,double goX,double goY,double goZ,double delay_s)
{
    Tar.PathInit(initX, initY, initZ);
    Tar.SetPathPoint(goX, goY, goZ);
    Tar.Move_MoveJ_CubicPolynomial();
    cout<<"暂停 "<<delay_s<<"s"<<endl<<endl;
    usleep(delay_s * 1000000);
}