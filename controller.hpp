#ifndef _CONTROLLER_
#define _CONTROLLER_
#include <math.h>
#include <Eigen/Dense>
#include "kinematics.hpp"
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "CAN2USB.hpp"

#include <Eigen/Core>
#include <ccd/ccd.h>
#define FCL_EXPORT
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include <iostream>
// #include <Eigen/Dense>

using namespace Eigen;
using namespace std;
using namespace fcl;
using namespace Eigen;

extern TrainComputer TC;
extern CAN2USB rob;
class Target_data
{
public:
    //路径点数目
    int    num = 1;

    //记录路径路径点
    double x[10];
    double y[10];
    double z[10];
    double r[10];
    double rz[10] = { 0 };
    double ry[10] = { 0 };
    double rx[10] = { 0 };

    //末端规划模式使用参数
    double length = 0;//当前运动轨迹长度(mm)
    double len[10] = { 0,0,0,0,0,0,0,0,0,0 };//每段运动轨迹长度

    //记录运行中途路径点对应关节位置
    double Joint_1[10];
    double Joint_2[10];
    double Joint_3[10];
    double Joint_4[10];
    double Joint_5[10];
    double Joint_6[10];

    //运行标志
    int flag = 0;

    //控制间隔时间（ms）
    double ControlIntervalTime = 100;

    //moveL运动速度限制
    double End_A_max = 0.002;//末端加速度(mm/ms2)
    double End_V_max = 0.2;//末端最大速度(mm/ms)
    double End_A_time;//末端加速时间(ms)
    double End_AT;//末端加速距离(mm)
    //double End_Time;//末端运动开始时间

    //关节运动速度限制
    double Joint_V_max = 0.0010;//关节最大速度(rad/ms)

    //实时追踪所需数据
    int trackMod = 0;

    double Target_x = 0;
    double Target_y = 0;
    double Target_z = 0;
    double Target_rx = 0;
    double Target_ry = 0;
    double Target_rz = 0;

    double tarck_maxV = 0.01; // mm/ms
    double tarck_A = 0.00001; // mm/ms2
    double tarck_A_dis = 5;//加减速距离

    double present_V[4] = {0, 0, 0, 0};//速度及速度矢量
    double present_x = 0;
    double present_y = 0;
    double present_z = 0;
    double present_rx = 0;
    double present_ry = 0;
    double present_rz = 0;

    int change_flag ;
    int change_times = 20;
    int change_record;
    double old_V[3] = {0, 0, 0};



    //初始化路径起始点
    //输入：无
    //输出：无
    //特别：无
    void PathInit(void)
    {
        flag = 0;
        Matrix4d T = TC.kinematics(TC.theta_now);

        x[0] = T(0, 3);
        y[0] = T(1, 3);
        z[0] = T(2, 3);
        r[0] = 0;
        Rpy rpy = TC.Matrix2Rpy(T.block<3, 3>(0, 0));
        rx[0] = rpy(0);
        ry[0] = rpy(1);
        rz[0] = rpy(2);

        Joint_1[0] = TC.theta_now[0];
        Joint_2[0] = TC.theta_now[1];
        Joint_3[0] = TC.theta_now[2];
        Joint_4[0] = TC.theta_now[3];
        Joint_5[0] = TC.theta_now[4];
        Joint_6[0] = TC.theta_now[5];

        num = 1;
        length = 0;

        for(int i = 1; i < 10; i++)
        {
            x[i] = y[i] = z[i] = r[i] = rx[i] = ry[i] = rz[i] = 0;
            len[i] = 0;
            Joint_1[i] = Joint_2[i] = Joint_3[i] = Joint_4[i] = Joint_5[i] = Joint_6[i]=0;
        }
    }

    //设置路径中间点
    //输入：X，Y，Z，R，RX，RY，RZ
    //输出：无
    //特别：无
    void SetPathPoint(double X, double Y, double Z, double R, double Rx, double Ry, double Rz)
    {
        //记录每一个中途位姿点
        x[num] = X;
        y[num] = Y;
        z[num] = Z;
        r[num] = R;
        rx[num] = Rx;
        ry[num] = Ry;
        rz[num] = Rz;
        
        //末端规划模式使用的参数
        if (R == 0)
            len[num] = sqrt((x[num] - x[num - 1]) * (x[num] - x[num - 1]) + (y[num] - y[num - 1]) * (y[num] - y[num - 1]) + (z[num] - z[num - 1]) * (z[num] - z[num - 1])) + len[num - 1];
        else
            len[num] = EIGEN_PI * r[num - 1] / 2 + len[num - 1];
        length = len[num];

        //MoveJ规划模式使用的参数
        double Old[6];
        Old[0] = Joint_1[num - 1];
        Old[1] = Joint_2[num - 1];
        Old[2] = Joint_3[num - 1];
        Old[3] = Joint_4[num - 1];
        Old[4] = Joint_5[num - 1];
        Old[5] = Joint_6[num - 1];

        double parameter[6] = {X, Y, Z, Rx, Ry, Rz};

        double Joint[6];
        Pose_ComputerAndJudge_MoveJ(parameter, Old, Joint);
        if(Joint[0] == 10000)
        {
            cout << "当前位置不可达"  << endl;
            exit(0);
        }

        Joint_1[num] = Joint[0];
        Joint_2[num] = Joint[1];
        Joint_3[num] = Joint[2];
        Joint_4[num] = Joint[3];
        Joint_5[num] = Joint[4];
        Joint_6[num] = Joint[5];
        // cout << "Set Target Joints " << Joint[0] << " " << Joint[1] << " " << Joint[2] << " " << Joint[3] << " " << Joint[4] << " " << Joint[5] << endl;
        //中途路径点数目+1
        num++;
    }

    //计算末端梯形速度模式下，当前时间走过的距离
    //输入：当前运行了多久（ms）
    //输出：当前运行了多远（mm）
    //特别：无
    float DisJudge_LadderShaped(double time)
    {
        if (length > End_AT * 2)//若该段轨迹比加减度总长度长，需要划分为三段计算
        {
            if (time < End_A_time)//加速过程
            {
                //dis = End_A_max * time * time / 2;
                return End_A_max * time * time / 2;
                //return dis;
            }
            else
            {
                if ((End_V_max * time - End_AT) > (length - End_AT))//减速过程
                {
                    double more_time = length / End_V_max + End_A_time - time;
                    if (more_time < 0)
                        return length + 1;//代表当前轨迹结束
                    
                    return length - End_A_max * more_time * more_time / 2 ;
                }
                else//匀速过程
                {
                    return End_V_max * time - End_AT;
                }
            }
        }
        else//若该段轨迹比加减度总长度短，只需划分为两段计算
        {
            double b = sqrt(length / End_A_max);
            if (time < b)//加速阶段
                return End_A_max * time * time / 2;
            else//减速阶段
            {
                double more_time = 2 * b - time;
                if (more_time < 0)
                    return length + 1;
                return length - End_A_max * more_time * more_time / 2;
            }
        }
    }

    //计算三次多项式相关参数，
    //输入：Dis 距离（mm）， Time 时间（ms）， V1 初始速度（mm/ms）， V2 末尾速度（mm/ms)
    //输出：A 三次项系数， B 二次项系数， C 一次性系数
    //特别：无
    void Computer_CubicPolynomial(double Dis, double Time, double V1, double V2, double* A, double* B, double* C)
    {
        *B = (3 * Dis - 2 * V1 * Time - V2 * Time) / Time / Time;
        *A = (Dis - V1 * Time - *B * Time * Time) / Time / Time / Time;
        *C = V1;
    }


    //计算MoveJ三次多项式模式下，当前时间走过的距离
    //输入：当前运行了多久（ms），A 三次项系数， B 二次项系数 ，C 一次项系数
    //输出：当前运行了多远（弧度）
    //特别：无
    float DisJudge_MoveJ_CubicPolynomial(double time, double A, double B, double C)
    {
        return A * pow(time, 3) + B * pow(time, 2) + C * pow(time, 1);
    }

    //根据运行过的路径长度，计算当前所在坐标点
    //输入：double 路径长度
    //输出：double 1*3 当前坐标点xyz
    //特别：会判断路径是否结束，结束路径后将运行标志位flag置0
    void GetLocation_End(double dis, double *out)
    {
        double* Out = out;
        int i;
        for (i = 1; i < num; i++)
        {
            if (dis > len[i])//判断当前位于第几段轨迹
                continue;
            if (r[i] != 0)
            {
                double T2 = dis - len[i - 1];
                double T3 = sin(T2 / r[i]);
                double T4 = cos(T2 / r[i]);
                *Out = (x[i] - x[i - 1]) * T3 + x[i - 1]; Out++;
                *Out = (y[i] - y[i - 1]) * T3 + y[i - 1]; Out++;
                *Out = (z[i] - z[i - 1]) * (1 - T4) + z[i - 1]; Out = out;
                return;
            }
            else
            {
                Rpy startRpy(rx[i - 1], ry[i - 1], rz[i - 1]);
                Rpy endRpy(rx[i], ry[i], rz[i]);
                Quaterniond startQuaterniond = TC.Rpy2Quaternion(startRpy);
                startQuaterniond = TC.Matrix2Quaternion(TC.Quaternion2Matrix(startQuaterniond));
                Quaterniond endQuaterniond = TC.Rpy2Quaternion(endRpy);
                endQuaterniond = TC.Matrix2Quaternion(TC.Quaternion2Matrix(endQuaterniond));

                Quaterniond resultQuaterniond = TC.Slerp(startQuaterniond, endQuaterniond, (dis - len[i - 1]) / (len[i] - len[i - 1]));
                // cout<<"resultQuaterniond " << resultQuaterniond.x() << " " << resultQuaterniond.y() << " " << resultQuaterniond.z() << " " << resultQuaterniond.w() << " "<< endl;
                // cout<<TC.Quaternion2Matrix(resultQuaterniond)<<endl;
                //cout<<atan2(2 * (resultQuaterniond.w() * q1 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))<<endl;
                Rpy resultRpy = TC.Quaternion2Rpy(resultQuaterniond);
                // cout<<TC.Rpy2Matrix(resultRpy)<<endl;

                *Out = (x[i] - x[i - 1]) * ((dis - len[i - 1]) / (len[i] - len[i - 1])) + x[i - 1]; Out++;//x坐标
                *Out = (y[i] - y[i - 1]) * ((dis - len[i - 1]) / (len[i] - len[i - 1])) + y[i - 1]; Out++;//y坐标
                *Out = (z[i] - z[i - 1]) * ((dis - len[i - 1]) / (len[i] - len[i - 1])) + z[i - 1]; Out++;//z坐标
                *Out = resultRpy(0); Out++;//rx坐标
                *Out = resultRpy(1); Out++;//ry坐标
                *Out = resultRpy(2); Out++;//rz坐标

                return;
            }
        }

        if (i >= num)
        {
            *Out =  x[num - 1]; Out++;//x坐标
            *Out =  y[num - 1]; Out++;//y坐标
            *Out =  z[num - 1]; Out++;//z坐标
            *Out = rx[num - 1]; Out++;//rx坐标
            *Out = ry[num - 1]; Out++;//ry坐标
            *Out = rz[num - 1]; Out++;//rz坐标
            flag = 0;
            return;
        }
    }

    //根据运行过的路径长度，计算当前所在坐标点
    //输入：dis 路径长度， start 初始位姿，end 结束位姿
    //输出：当前关节数值
    //特别：无
    double GetLocation_MoveJ(double dis, double start, double end)
    {
        double err = end - start;
        if (err < 0)
        {
            if (dis > -err)
                return end;
            else
                return start - dis;
        }
        else
        {
            if (dis > err)
                return end;
            else
                return start + dis;
        }
    }

    //使用末端轨迹规划，梯形速度曲线
    //输入：无
    //输出：无
    //特别：使用的记录好的路径点
    void Move_End_LadderShaped(void)
    {
        End_A_time = End_V_max / End_A_max;
        End_AT = End_V_max * End_A_time / 2;
        // cout << "末端轨迹规划，梯形速度曲线，运动开始" << endl;

        double dis;
        double position[6];
        double joint[6];

        // double JointsRecord[6][500];
        // double PathRecord[6][500];

        int TimeRecord = 0;//时间记录
        flag = 1;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t3 = std::chrono::high_resolution_clock::now();
        // auto t4 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms;
        while(flag)
        {
            //usleep(ControlIntervalTime * 1000);
            while(1)
            {
                t3 = std::chrono::high_resolution_clock::now();
                fp_ms = t3-t1;
                if(fp_ms.count()>ControlIntervalTime*TimeRecord)
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
            dis = DisJudge_LadderShaped(ControlIntervalTime * TimeRecord);

            TimeRecord += 1;
            GetLocation_End(dis, position);
            // cout<<endl<<"position "<<position[0]<<" "<<position[1]<<" "<<position[2]<<" "<<position[3]<<" "<<position[4]<<" "<<position[5]<<" "<<endl;
            // PathRecord[0][TimeRecord] = position[0];
            // PathRecord[1][TimeRecord] = position[1];
            // PathRecord[2][TimeRecord] = position[2];
            // PathRecord[3][TimeRecord] = position[3];
            // PathRecord[4][TimeRecord] = position[4];
            // PathRecord[5][TimeRecord] = position[5];
            // PathRecord[0][0] = TimeRecord;
            // cout << position[0] << " " << position[1] << " " << position[2] << " " << position[3] << " " << position[4] << " " << position[5] << " " << endl;

            TC.InvestAndJudge(position, joint,2);
            if (joint[0] > 100)
            {
                cout << " time:   " << ControlIntervalTime * TimeRecord << " out " << endl;
                // cout<<"position "<<position[0]<<" "<<position[1]<<" "<<position[2]<<" "<<position[3]<<" "<<position[4]<<" "<<position[5]<<" "<<endl<<endl;
                continue;
            }
            TC.theta_now[0] = joint[0];
            TC.theta_now[1] = joint[1];
            TC.theta_now[2] = joint[2];
            TC.theta_now[3] = joint[3];
            TC.theta_now[4] = joint[4];
            TC.theta_now[5] = joint[5];
            // cout << "joint "<< " " <<joint[0] << joint[1] << joint[2]<<joint[3] << joint[4]<< joint[5] << endl;
            rob.robotSetPositionAll(TC.theta_now);
            
            //cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;
            // rob.robotSetPositionAll(TC.theta_now);

            // JointsRecord[0][TimeRecord] = joint[0];
            // JointsRecord[1][TimeRecord] = joint[1];
            // JointsRecord[2][TimeRecord] = joint[2];
            // JointsRecord[3][TimeRecord] = joint[3];
            // JointsRecord[4][TimeRecord] = joint[4];
            // JointsRecord[5][TimeRecord] = joint[5];

            // JointsRecord[0][0] = JointsRecord[1][0] = JointsRecord[2][0] = JointsRecord[3][0] = JointsRecord[4][0] = JointsRecord[5][0] = TimeRecord;       
        }
        // cout << "末端轨迹规划，梯形速度曲线，运动结束" << endl;
        // auto t2 = std::chrono::high_resolution_clock::now();
        // fp_ms = t2 - t1;
        // cout<<"总计"<<TimeRecord-1<<"个控制点"<<endl;
        // cout<<"理论使用时间 "<<(TimeRecord-1)*ControlIntervalTime<<"ms"<<endl;
        // cout<<"实际使用时间 "<<fp_ms.count()<<"ms"<<endl;

        // for (int num = 0; num < 6; num++)
        // {
        //     cout << endl << "JointRecord" << num << endl;
        //     for (int i = 1; i < JointsRecord[0][0]; i++)
        //     {
        //         cout << JointsRecord[num][i] << ", ";
        //     }
        //     cout << endl;
        // }

        // for (int num = 0; num < 6; num++)
        // {
        //     cout << endl << "PathRecord" << num << endl;
        //     for (int i = 1; i < PathRecord[0][0]; i++)
        //     {
        //         cout << PathRecord[num][i] << ", ";
        //     }
        //     cout << endl;
        // }
    }

    //计算并判断MoveJ方式末端位姿
    //输入：Parameter[6] double类型，末端姿态描述参数 Now[6] double 前一位点时的机械臂姿态
    //输出：Joint double* 6位，为0-5号电机角度
    //特别：无
    void Pose_ComputerAndJudge_MoveJ(double Parameter[6], double Now[6], double* Joint)
    {
        TC.MoveJ_InvestAndJudge(Parameter, Now, Joint, 25);
    }

    //使用MoveJ轨迹规划，三次多项式曲线
    //输入：无
    //输出：无
    //特别：使用的记录好的路径点
    void Move_MoveJ_CubicPolynomial(void)
    {
        double A[10][6], B[10][6], C[10][6], Time_record[10];
        double Dis[6], Time[6];

        // double JointsRecord[6][500];
        // double PathRecord[3][500];

        for (int i = 1; i < num; i++)
        {
            Dis[0] = fabs(Joint_1[i] - Joint_1[i - 1]);
            Time[0] = Dis[0] / Joint_V_max;

            Dis[1] = fabs(Joint_2[i] - Joint_2[i - 1]);
            Time[1] = Dis[1] / Joint_V_max;

            Dis[2] = fabs(Joint_3[i] - Joint_3[i - 1]);
            Time[2] = Dis[2] / Joint_V_max;

            Dis[3] = fabs(Joint_4[i] - Joint_4[i - 1]);
            Time[3] = Dis[3] / Joint_V_max;

            Dis[4] = fabs(Joint_5[i] - Joint_5[i - 1]);
            Time[4] = Dis[4] / Joint_V_max;

            Dis[5] = fabs(Joint_6[i] - Joint_6[i - 1]);
            Time[5] = Dis[5] / Joint_V_max;

            Time_record[i - 1] = Time[0];
            for (int t = 0; t < 6; t++)
            {
                if (Time[t] > Time_record[i - 1])
                {
                    Time_record[i - 1] = Time[t];
                }
            }
            Computer_CubicPolynomial(Dis[0], Time_record[i - 1], 0, 0, &A[i - 1][0], &B[i - 1][0], &C[i - 1][0]);
            Computer_CubicPolynomial(Dis[1], Time_record[i - 1], 0, 0, &A[i - 1][1], &B[i - 1][1], &C[i - 1][1]);
            Computer_CubicPolynomial(Dis[2], Time_record[i - 1], 0, 0, &A[i - 1][2], &B[i - 1][2], &C[i - 1][2]);
            Computer_CubicPolynomial(Dis[3], Time_record[i - 1], 0, 0, &A[i - 1][3], &B[i - 1][3], &C[i - 1][3]);
            Computer_CubicPolynomial(Dis[4], Time_record[i - 1], 0, 0, &A[i - 1][4], &B[i - 1][4], &C[i - 1][4]);
            Computer_CubicPolynomial(Dis[5], Time_record[i - 1], 0, 0, &A[i - 1][5], &B[i - 1][5], &C[i - 1][5]);
        }
        cout << "MoveJ轨迹规划，三次多项式曲线，运动开始" << endl;

        double dis;
        double joint[6];
        int TimeRecord = 0;//时间记录
        flag = 1;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t3 = std::chrono::high_resolution_clock::now();
        // auto t4 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms;
        while (flag)
        {
            while(1)
            {
                t3 = std::chrono::high_resolution_clock::now();
                fp_ms = t3-t1;
                if(fp_ms.count()>ControlIntervalTime*TimeRecord)
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
            double now_time = ControlIntervalTime * TimeRecord;
            // usleep(ControlIntervalTime*1000);
            TimeRecord += 1;
            int stage;
            for (stage = 0; stage < num - 1; stage++)
            {
                if (now_time > Time_record[stage])
                {
                    now_time -= Time_record[stage];
                    continue;
                }
                dis = DisJudge_MoveJ_CubicPolynomial(now_time, A[stage][0], B[stage][0], C[stage][0]);
                joint[0] = GetLocation_MoveJ(dis, Joint_1[stage], Joint_1[stage + 1]);

                dis = DisJudge_MoveJ_CubicPolynomial(now_time, A[stage][1], B[stage][1], C[stage][1]);
                joint[1] = GetLocation_MoveJ(dis, Joint_2[stage], Joint_2[stage + 1]);

                dis = DisJudge_MoveJ_CubicPolynomial(now_time, A[stage][2], B[stage][2], C[stage][2]);
                joint[2] = GetLocation_MoveJ(dis, Joint_3[stage], Joint_3[stage + 1]);

                dis = DisJudge_MoveJ_CubicPolynomial(now_time, A[stage][3], B[stage][3], C[stage][3]);
                joint[3] = GetLocation_MoveJ(dis, Joint_4[stage], Joint_4[stage + 1]);

                dis = DisJudge_MoveJ_CubicPolynomial(now_time, A[stage][4], B[stage][4], C[stage][4]);
                joint[4] = GetLocation_MoveJ(dis, Joint_5[stage], Joint_5[stage + 1]);

                dis = DisJudge_MoveJ_CubicPolynomial(now_time, A[stage][5], B[stage][5], C[stage][5]);
                joint[5] = GetLocation_MoveJ(dis, Joint_6[stage], Joint_6[stage + 1]);

                break;
            }
            // cout << "Joints " << joint[0] <<  joint[1] << joint[2] << joint[3] << joint[4] << joint[5] << endl;
            if (stage == num - 1)
            {
                joint[0] = Joint_1[num - 1];
                joint[1] = Joint_2[num - 1];
                joint[2] = Joint_3[num - 1];
                joint[3] = Joint_4[num - 1];
                joint[4] = Joint_5[num - 1];
                joint[5] = Joint_6[num - 1];
                flag = 0;
            }
            TC.theta_now[0] = joint[0];
            TC.theta_now[1] = joint[1];
            TC.theta_now[2] = joint[2];
            TC.theta_now[3] = joint[3];
            TC.theta_now[4] = joint[4];
            TC.theta_now[5] = joint[5];
            rob.robotSetPositionAll(TC.theta_now);
            // cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;

            // JointsRecord[0][TimeRecord] = joint[0];
            // JointsRecord[1][TimeRecord] = joint[1];
            // JointsRecord[2][TimeRecord] = joint[2];
            // JointsRecord[3][TimeRecord] = joint[3];
            // JointsRecord[4][TimeRecord] = joint[4];
            // JointsRecord[5][TimeRecord] = joint[5];

            // JointsRecord[0][0] = JointsRecord[1][0] = JointsRecord[2][0] = JointsRecord[3][0] = JointsRecord[4][0] = JointsRecord[5][0] = TimeRecord;   
        }
        cout << "MoveJ轨迹规划，三次多项式曲线，运动结束" << endl;
        auto t2 = std::chrono::high_resolution_clock::now();
        fp_ms = t2 - t1;
        cout<<"总计"<<TimeRecord-1<<"个控制点"<<endl;
        cout<<"理论使用时间 "<<(TimeRecord-1)*ControlIntervalTime<<"ms"<<endl;
        cout<<"实际使用时间 "<<fp_ms.count()<<"ms"<<endl;
    }


    //设置追踪模式目标参赛
    //输入：X Y Z Rx Ry Rz  目标点的位姿参数
    //输出：无
    //特别：无
    void setTarget(double X, double Y, double Z, double Rx, double Ry, double Rz)
    {
        Target_x = X;
        Target_y = Y;
        Target_z = Z;
        Target_rx = Rx;
        Target_ry = Ry;
        Target_rz = Rz;
        

        if(change_flag >= 0)
        {
            change_record = change_times;
            change_flag = 1;
            
            old_V[0] = present_V[1];
            old_V[1] = present_V[2];
            old_V[2] = present_V[3];
        }
        else
        {
            change_record = change_times;
            change_flag = 0;
        }
        double Vall = sqrt((Target_x - present_x)*(Target_x - present_x) + (Target_y - present_y)*(Target_y - present_y) + (Target_z - present_z)*(Target_z - present_z));
        present_V[1] = (Target_x - present_x) / Vall;
        present_V[2] = (Target_y - present_y) / Vall;
        present_V[3] = (Target_z - present_z) / Vall;
    }

    void TrackInit(void)
    {
        Matrix4d matrix = TC.kinematics(TC.theta_now);
        present_x = matrix(0, 3);
        present_y = matrix(1, 3);
        present_z = matrix(2, 3);

        Rpy rpy = TC.Matrix2Rpy(matrix.block<3,3>(0,0));
        present_rx = rpy.x();
        present_ry = rpy.y();
        present_rz = rpy.z();

        change_flag = -1;
    }

    //追踪模式目标
    //输入：无
    //输出：无
    //特别：无
    void MoveL_Trackmod_LadderShaped(void)
    {

        double Tx = Target_x - present_x;
        double Ty = Target_y - present_y;
        double Tz = Target_z  - present_z;

        double Dis = sqrt(Tx * Tx + Ty * Ty + Tz * Tz );
        if(Dis > 0.001)
        {
            //减速阶段
            if(Dis < 0.5 * present_V[0] * present_V[0] / tarck_A)
            {
                present_V[0] -= tarck_A*ControlIntervalTime;
                if(present_V[0] < 0)
                {
                    present_V[0] = 0;
                }
            }
            //加速阶段
            else if(present_V[0]  < tarck_maxV)
            {
                present_V[0] += tarck_A*ControlIntervalTime;
                if(present_V[0] > tarck_maxV)
                {
                    present_V[0] = tarck_maxV;
                }
            }


            

            double Vnow[3];
            if(change_flag>0)
            {
                // cout<<"Change"<<endl;
                change_record --;
                Vnow[0] = ((change_record * old_V[0]) + ((change_times-change_record) * present_V[1]))/change_times * present_V[0];
                Vnow[1] = ((change_record * old_V[1]) + ((change_times-change_record) * present_V[2]))/change_times * present_V[0];
                Vnow[2] = ((change_record * old_V[2]) + ((change_times-change_record) * present_V[3]))/change_times * present_V[0];
                if(change_record<=0)
                {
                    change_flag = 0;
                    change_record = change_times;
                }
            }
            else
            {
                // cout<<"No "<< change_flag <<endl;
                Vnow[0] = Tx / Dis * present_V[0];
                Vnow[1] = Ty / Dis * present_V[0];
                Vnow[2] = Tz / Dis * present_V[0];
            }
            present_x += Vnow[0] * ControlIntervalTime;
            present_y += Vnow[1] * ControlIntervalTime;
            present_z += Vnow[2] * ControlIntervalTime;

            //
            // present_V[1]  =  Tx / Dis * present_V[0];
            // present_V[2]  =  Ty / Dis * present_V[0];
            // present_V[3]  =  Tz / Dis * present_V[0];

            // present_x += present_V[1] * ControlIntervalTime;
            // present_y += present_V[2] * ControlIntervalTime;
            // present_z += present_V[3] * ControlIntervalTime;

            Rpy  StartRpy(present_rx, present_ry, present_rz);
            Rpy EndRpy(Target_rx, Target_ry, Target_rz);

            Quaterniond StartQuaterniond = TC.Matrix2Quaternion(TC.Rpy2Matrix(StartRpy));
            Quaterniond EndQuaterniond = TC.Matrix2Quaternion(TC.Rpy2Matrix(EndRpy));

            Quaterniond ResultQuaterniond = TC.Slerp(StartQuaterniond, EndQuaterniond, present_V[0] * ControlIntervalTime / Dis);

            Rpy ResultRpy = TC.Quaternion2Rpy(ResultQuaterniond);
            present_rx = ResultRpy.x();
            present_ry = ResultRpy.y();
            present_rz = ResultRpy.z();
            //cout<<ResultRpy<<endl;

            // Matrix4d matrix;
            // matrix.block<3,3>(0,0) = TC.Quaternion2Matrix(ResultQuaterniond);
            // matrix(0, 3) = present_x;
            // matrix(1, 3) = present_y;
            // matrix(2, 3) = present_z;

           
        }
        else
        {
            present_x = Target_x;
            present_y = Target_y;
            present_z = Target_z;
            present_rx = Target_rx;
            present_ry = Target_ry;
            present_rz = Target_rz;
            present_V[0] = present_V[1] = present_V[2] = present_V[2] = 0;
        }
        // else
        // {
        //     return;
        // }
         double T[6] = {present_x, present_y, present_z, present_rx, present_ry, present_rz};

        double Joints[6];
        //cout<<matrix<<endl;
        TC.InvestAndJudge(T,Joints,10);
        if(Joints[0]<100)
        {
            cout<<Joints[0]<<endl;
            TC.theta_now[0] = Joints[0];
            TC.theta_now[1] = Joints[1];
            TC.theta_now[2] = Joints[2];
            TC.theta_now[3] = Joints[3];
            TC.theta_now[4] = Joints[4];
            TC.theta_now[5] = Joints[5];
        }
        else
            cout<<"error"<<endl;
        // Matrix4d T1 = TC.kinematics(Joints); 
        cout << T[0] << "  " << T[1] <<  "  " << T[2] <<  "  " <<  T[3] << "  " << T[4] <<  "  " << T[5] <<  endl;
        // cout<<T1<<endl;
        cout << TC.theta_now[0] << "  " <<TC. theta_now[1] <<  "  " << TC.theta_now[2] <<  "  " <<  TC.theta_now[3] << "  " << TC.theta_now[4] <<  "  " << TC.theta_now[5] <<  endl  <<  endl;
    }

    int isCollide(CollisionObjectd linkA, CollisionObjectd LinkB)
    {
        CollisionRequestd request;
        CollisionResultd result;
        
        // 进行碰撞检测
        collide(&linkA, &LinkB, request, result);
        // collide(&Link1, &Link3, request, result);
        
        // 输出碰撞结果
        if (result.isCollision()) {
            return 1;
        } else {
            return 0;
        }
    }

    int robotIsSelfCollide(double jointTheta[6])
    {
        double err = 40;
        Matrix4d T1;
        T1.setIdentity(4, 4);
        T1(2, 3) = 295.14 / 2;
        // cout << T1 << endl;
        Vector3d vec1(T1(0, 3), T1(1, 3), T1(2, 3));
        Matrix3d mat1 = T1.block<3, 3>(0, 0);
        auto CylinderLink1 = make_shared<Boxd>(152 + err, 152 + err, 295.14 + err);
        CollisionObjectd Link1(CylinderLink1);
        Link1.setTranslation(vec1);
        Link1.setRotation(mat1);

        Matrix4d T2;
        T2.setIdentity(4, 4);
        T2(2, 3) = 196.44 / 2 + 152 / 2;
        T2 = TC.Trans(0,jointTheta[0]) * T2;
        // cout << "T2 " << T2 << endl;
        Vector3d vec2(T2(0, 3), T2(1, 3), T2(2, 3));
        Matrix3d mat2 = T2.block<3, 3>(0, 0);
        auto CylinderLink2 = make_shared<Boxd>(152 + err, 152 + err, 196.44 + err);
        CollisionObjectd Link2(CylinderLink2);
        Link2.setTranslation(vec2);
        Link2.setRotation(mat2);

        Matrix4d T3;
        T3.setIdentity(4, 4);
        T3(0, 3) = (450 / 2 + 152 / 2);
        T3(2, 3) = 206;
        T3.block<3,3>(0,0) = (AngleAxisd(0, Vector3d::UnitZ())
                            * AngleAxisd(EIGEN_PI/2, Vector3d::UnitY())
                            * AngleAxisd(0, Vector3d::UnitX())).toRotationMatrix();

        T3 = TC.Trans(0,jointTheta[0]) * TC.Trans(1,jointTheta[1])  * T3;
        // cout << "T3 " << T3 << endl;
        Vector3d vec3(T3(0, 3), T3(1, 3), T3(2, 3));
        Matrix3d mat3 = T3.block<3, 3>(0, 0);
        auto CylinderLink3 = make_shared<Boxd>(110 + err, 110 + err, 450 + err);
        CollisionObjectd Link3(CylinderLink3);
        Link3.setTranslation(vec3);
        Link3.setRotation(mat3);

        Matrix4d T4;
        T4.setIdentity(4, 4);
        T4(2, 3) = 206 / 2;
        T4 = TC.Trans(0,jointTheta[0]) * TC.Trans(1,jointTheta[1]) * T4;
        // cout << "T4 " << T4 << endl;
        Vector3d vec4(T4(0, 3), T4(1, 3), T4(2, 3));
        Matrix3d mat4 = T4.block<3, 3>(0, 0);
        auto CylinderLink4 = make_shared<Boxd>(120 + err, 120 + err, 349.41 + err);
        CollisionObjectd Link4(CylinderLink4);
        Link4.setTranslation(vec4);
        Link4.setRotation(mat4);

        Matrix4d T5;
        T5.setIdentity(4, 4);
        T5(0, 3) = 571.1 / 2;
        T5(2, 3) = 16;
        T5.block<3,3>(0,0) = (AngleAxisd(0, Vector3d::UnitZ())
                            * AngleAxisd(EIGEN_PI/2, Vector3d::UnitY())
                            * AngleAxisd(0, Vector3d::UnitX())).toRotationMatrix();
        T5 = TC.Trans(0,jointTheta[0]) * TC.Trans(1,jointTheta[1]) * TC.Trans(2,jointTheta[2]) * T5;
        // cout << "T5 " << T5 << endl;
        Vector3d vec5(T5(0, 3), T5(1, 3), T5(2, 3));
        Matrix3d mat5 = T5.block<3, 3>(0, 0);
        auto CylinderLink5 = make_shared<Boxd>(90 + err, 90 + err, 435.6 + err);
        CollisionObjectd Link5(CylinderLink5);
        Link5.setTranslation(vec5);
        Link5.setRotation(mat5);

        Matrix4d T6;
        T6.setIdentity(4, 4);
        T6(2, 3) = 25;
        T6 = TC.Trans(0,jointTheta[0]) * TC.Trans(1,jointTheta[1]) * TC.Trans(2,jointTheta[2]) * T6;
        // cout << "T6 " << T6 << endl;
        Vector3d vec6(T6(0, 3), T6(1, 3), T6(2, 3));
        Matrix3d mat6 = T6.block<3, 3>(0, 0);
        auto CylinderLink6 = make_shared<Boxd>(90 + err, 90 + err, 169.04 + err);
        CollisionObjectd Link6(CylinderLink6);
        Link6.setTranslation(vec6);
        Link6.setRotation(mat6);

        Matrix4d T7;
        T7.setIdentity(4, 4);
        T7(2, 3) = 10;
        T7 = TC.Trans(0,jointTheta[0]) * TC.Trans(1,jointTheta[1]) * TC.Trans(2,jointTheta[2]) * TC.Trans(3,jointTheta[3]) * T7;
        // cout << "T7 " << T7 << endl;
        Vector3d vec7(T7(0, 3), T7(1, 3), T7(2, 3));
        Matrix3d mat7 = T7.block<3, 3>(0, 0);
        auto CylinderLink7 = make_shared<Boxd>(90 + err, 90 + err, 127.19 + err);
        CollisionObjectd Link7(CylinderLink7);
        Link7.setTranslation(vec7);
        Link7.setRotation(mat7);

        Matrix4d T8;
        T8.setIdentity(4, 4);
        T8(2, 3) = 10;
        T8 = TC.Trans(0,jointTheta[0]) * TC.Trans(1,jointTheta[1]) * TC.Trans(2,jointTheta[2]) * TC.Trans(3,jointTheta[3]) * TC.Trans(4,jointTheta[4]) * T8;
        // cout << "T8 " << T8 << endl;
        Vector3d vec8(T8(0, 3), T8(1, 3), T8(2, 3));
        Matrix3d mat8 = T8.block<3, 3>(0, 0);
        auto CylinderLink8 = make_shared<Boxd>(90 + err, 90 + err, 170.4 + err);
        CollisionObjectd Link8(CylinderLink8);
        Link8.setTranslation(vec8);
        Link8.setRotation(mat8);

        if(isCollide(Link1,Link5))
        {
            // cout<<"1 5 碰撞"<<endl;
            return 15;
        }
        if(isCollide(Link1,Link6))
        {
            // cout<<"1 6 碰撞"<<endl;
            return 16;
        }
        if(isCollide(Link1,Link8))
        {
            // cout<<"1 8 碰撞"<<endl;
            return 18;
        }

        if(isCollide(Link2,Link6))
        {
            // cout<<"2 6 碰撞"<<endl;
            return 26;
        }
        if(isCollide(Link2,Link7))
        {
            // cout<<"2 7 碰撞"<<endl;
            return 27;
        }
        if(isCollide(Link2,Link8))
        {
            // cout<<"2 8 碰撞"<<endl;
            return 28;
        }

        if(isCollide(Link3,Link7))
        {
            // cout<<"3 7 碰撞"<<endl;
            return 37;
        }
        if(isCollide(Link3,Link8))
        {
            // cout<<"3 8 碰撞"<<endl;
            return 38;
        }

        if(isCollide(Link5,Link8))
        {
            // cout<<"5 8 碰撞"<<endl;
            return 58;
        }
        return 0;
    }
};

extern Target_data Tar;
#endif