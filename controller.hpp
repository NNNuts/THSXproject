#ifndef _CONTROLLER_
#define _CONTROLLER_
#include <math.h>
#include <Eigen/Dense>
#include "kinematics.hpp"
#include <unistd.h>
#include <fstream>
// #include"Can232.hpp"
#include "CAN2USB.hpp"

using namespace Eigen;

// extern CAN232 rob;
extern CAN2USB rob;
extern TrainComputer TC;
class Target_data
{
public:
    //路径点数目
    int    num = 1;

        //记录路径路径点
    double x[10];
    double y[10];
    double z[10];


    //末端规划模式使用参数
    double length = 0;//当前运动轨迹长度(mm)
    double len[10] = { 0,0,0,0,0,0,0,0,0,0 };//每段运动轨迹长度

    //记录运行中途路径点对应关节位置
    double Joint_1[10];
    double Joint_2[10];

    //运行标志
    int flag = 0;

    //控制间隔时间（ms）
    double ControlIntervalTime = 30;

    //末端运动速度限制movel
    double End_V_max = 32;//末端最大速度(mm/ms)

    double End_A_max_up = 30;//末端加速度(mm/ms2)
    double End_A_time_up;//末端加速时间(ms)
    double End_AT_up;//末端加速距离(mm)

    double End_A_max_down = 30;//末端加速度(mm/ms2)
    double End_A_time_down;//末端加速时间(ms)
    double End_AT_down;//末端加速距离(mm)


    //关节运动速度限制
    double Joint_V_max = 0.00618;//关节最大速度(rad/ms)

    //实时追踪所需数据
    int trackMod = false;

    // double Target_x = 0;
    // double Target_y = 0;
    // double Target_z = 0;

    double TargetJoint_1 = 0;
    double TargetJoint_2 = 0;

    double tarck_maxV = 0.001; //(rad/ms)
    double tarck_A = 0.000001; //(rad/ms2)

    double present_Joint_V1,present_Joint_V2;

    double JointsRecord[2][500] = {0};




    //初始化路径起始点
    //输入：无
    //输出：无
    //特别：无
    void PathInit(double X = 0, double Y = 0, double Z = 0)
    {
        flag = 0;
        x[0] = X;
        y[0] = Y;
        z[0] = Z;

        Joint_1[0] = TC.theta_now[0];
        Joint_2[0] = TC.theta_now[1];
        // cout<<TC.theta_now[0]<<endl;
        // x[0] = y[0] = z[0] = 0;

        num = 1;
        length = 0;

        for(int i = 1; i < 10; i++)
        {
            Joint_1[i] = Joint_2[i]=0;
            x[i] = y[i] = z[i] = 0;
            len[i] = 0;
        }
    }

    //设置路径中间点
    //输入：X，Y，Z
    //输出：无
    //特别：无
    void SetPathPoint(double X, double Y, double Z)
    {
        x[num] = X;
        y[num] = Y;
        z[num] = Z;

        len[num] = sqrt((x[num] - x[num - 1]) * (x[num] - x[num - 1]) + (y[num] - y[num - 1]) * (y[num] - y[num - 1]) + (z[num] - z[num - 1]) * (z[num] - z[num - 1])) + len[num - 1];
        length = len[num];
        double Joint[2];
        TC.invest(X, Y, Z, Joint);

        Joint_1[num] = Joint[0];
        Joint_2[num] = Joint[1];
        // cout<<Joint[0]<<endl;

        //中途路径点数目+1
        num++;
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

    //使用MoveJ轨迹规划，三次多项式曲线
    //输入：无
    //输出：无
    //特别：使用的记录好的路径点
    void Move_MoveJ_CubicPolynomial(void)
    {
        double A[10][6], B[10][6], C[10][6], Time_record[10];
        double Dis[6], Time[6];

        double JointsRecord[6][500];
        //double PathRecord[3][500];

        for (int i = 1; i < num; i++)
        {
            Dis[0] = fabs(Joint_1[i] - Joint_1[i - 1]);
            Time[0] = Dis[0] / Joint_V_max;

            Dis[1] = fabs(Joint_2[i] - Joint_2[i - 1]);
            Time[1] = Dis[1] / Joint_V_max;

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

        }
        cout << "MoveJ轨迹规划，三次多项式曲线，运动开始" << endl;

        double dis;
        double joint[2];
        int TimeRecord = 0;//时间记录
        flag = 1;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t3 = std::chrono::high_resolution_clock::now();
        // auto t4 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms;
        while (flag)
        {
            double now_time = ControlIntervalTime * TimeRecord;

            // usleep(1000*ControlIntervalTime);
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

                break;
            }
            if (stage == num - 1)
            {
                joint[0] = Joint_1[num - 1];
                joint[1] = Joint_2[num - 1];
                flag = 0;
            }
            TC.theta_now[0] = joint[0];
            TC.theta_now[1] = joint[1];
            rob.robotSetPositionAll(TC.theta_now);
            
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        fp_ms = t2 - t1;
        cout<<"总计"<<TimeRecord-1<<"个控制点"<<endl;
        cout<<"理论使用时间 "<<(TimeRecord-1)*ControlIntervalTime<<"ms"<<endl;
        cout<<"实际使用时间 "<<fp_ms.count()<<"ms"<<endl;
        cout << "MoveJ轨迹规划，三次多项式曲线，运动结束" << endl<<endl;
    }


    //设置追踪模式目标参赛
    //输入：X Y Z  目标点的位姿参数
    //输出：无
    //特别：无
    void setTarget(double X, double Y, double Z)
    {
       double Joint[2];
        TC.invest(X, Y, Z, Joint);
        TargetJoint_1 = Joint[0];
        TargetJoint_2 = Joint[1];
    }

    void TrackInit(void)
    {

        present_Joint_V1 = 0;
        present_Joint_V2 = 0;
    }

    //追踪模式目标
    //输入：无
    //输出：无
    //特别：无
    void MoveL_Trackmod_LadderShaped(void)
    {
        double TJ_1 =  TargetJoint_1 -  TC.theta_now[0];
        double TJ_2 =  TargetJoint_2 -  TC.theta_now[1];

        // double Dis = sqrt(Tx * Tx + Ty * Ty + Tz * Tz );
        if(fabs(TJ_1) > 0.001)
        {
            //减速阶段
            if(fabs(TJ_1) < 0.5 * present_Joint_V1* present_Joint_V1 / tarck_A)
            {
                if(TJ_1 > 0)
                {
                    present_Joint_V1 -= tarck_A*ControlIntervalTime;
                    if(present_Joint_V1 < 0)
                    {
                        present_Joint_V1 = 0;
                    }
                }
                else
                {
                    present_Joint_V1 += tarck_A*ControlIntervalTime;
                    if(present_Joint_V1 > 0)
                    {
                        present_Joint_V1 = 0;
                    }
                }
                
            }
            //加速阶段
            else if(present_Joint_V1  < tarck_maxV)
            {
                if(TJ_1 > 0)
                {
                    present_Joint_V1 += tarck_A*ControlIntervalTime;
                    if(present_Joint_V1 > tarck_maxV)
                    {
                        present_Joint_V1 = tarck_maxV;
                    }
                }
                else
                {
                    present_Joint_V1 -= tarck_A*ControlIntervalTime;
                    if(present_Joint_V1 < -tarck_maxV)
                    {
                        present_Joint_V1 = -tarck_maxV;
                    }
                }
            }

            TC.theta_now[0] += present_Joint_V1 * ControlIntervalTime;
        }
        else
        {
            TC.theta_now[0] = TargetJoint_1;
        }

        if(fabs(TJ_2) > 0.001)
        {
            //减速阶段
            if(fabs(TJ_2) < 0.5 * present_Joint_V2* present_Joint_V2 / tarck_A)
            {
                if(TJ_2 > 0)
                {
                    present_Joint_V2 -= tarck_A*ControlIntervalTime;
                    if(present_Joint_V2 < 0)
                    {
                        present_Joint_V2 = 0;
                    }
                }
                else
                {
                    present_Joint_V2 += tarck_A*ControlIntervalTime;
                    if(present_Joint_V2 > 0)
                    {
                        present_Joint_V2 = 0;
                    }
                }
            }
            //加速阶段
            else if(present_Joint_V2  < tarck_maxV)
            {
                if(TJ_2 > 0)
                {
                    present_Joint_V2 += tarck_A*ControlIntervalTime;
                    if(present_Joint_V2 > tarck_maxV)
                    {
                        present_Joint_V2 = tarck_maxV;
                    }
                }
                else
                {
                    present_Joint_V2 -= tarck_A*ControlIntervalTime;
                    if(present_Joint_V2 < -tarck_maxV)
                    {
                        present_Joint_V2 = -tarck_maxV;
                    }
                }
            }
            TC.theta_now[1] += present_Joint_V2 * ControlIntervalTime;
        }
        else
        {
            TC.theta_now[1] = TargetJoint_2;
        }

        //这个是输出角度，需要发送给电机
        cout << TC.theta_now[0] << "  " <<TC. theta_now[1] <<  endl  <<  endl;

    }

    //计算末端梯形速度模式下，当前时间走过的距离
    //输入：当前运行了多久（ms）
    //输出：当前运行了多远（mm）
    //特别：无
    float DisJudge_LadderShaped(double time)
    {
        if (length > End_AT_up + End_AT_down)//若该段轨迹比加减度总长度长，需要划分为三段计算
        {
            if (time < End_A_time_up)//加速过程
            {
                //dis = End_A_max * time * time / 2;
                return End_A_max_up * time * time / 2;
                //return dis;
            }
            else
            {
                if ((End_V_max * time - End_AT_up) > (length - End_AT_down))//减速过程
                {
                    double more_time = length / End_V_max + (End_A_time_up + End_A_time_down)/2 - time;
                    if (more_time < 0)
                        return length + 1;//代表当前轨迹结束
                    
                    return length - End_A_max_down * more_time * more_time / 2 ;
                }
                else//匀速过程
                {
                    return End_V_max * time - End_AT_up;
                }
            }
        }
        else//若该段轨迹比加减度总长度短，只需划分为两段计算
        {
            double b = sqrt(length / End_A_max_up);
            if (time < b)//加速阶段
                return End_A_max_up * time * time / 2;
            else//减速阶段
            {
                double more_time = 2 * b - time;
                if (more_time < 0)
                    return length + 1;
                return length - End_A_max_down * more_time * more_time / 2;
            }
        }
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

            *Out = (x[i] - x[i - 1]) * ((dis - len[i - 1]) / (len[i] - len[i - 1])) + x[i - 1]; Out++;//x坐标
            *Out = (y[i] - y[i - 1]) * ((dis - len[i - 1]) / (len[i] - len[i - 1])) + y[i - 1]; Out++;//y坐标
            *Out = (z[i] - z[i - 1]) * ((dis - len[i - 1]) / (len[i] - len[i - 1])) + z[i - 1]; Out++;//z坐标

            return;
            
        }

        if (i >= num)
        {
            *Out =  x[num - 1]; Out++;//x坐标
            *Out =  y[num - 1]; Out++;//y坐标
            *Out =  z[num - 1]; Out++;//z坐标
            flag = 0;
            return;
        }
    }

    //使用末端轨迹规划，梯形速度曲线
    //输入：无
    //输出：无
    //特别：使用的记录好的路径点
    void Move_End_LadderShaped(void)
    {
        End_A_time_up = End_V_max / End_A_max_up;
        End_AT_up = End_V_max * End_A_time_up / 2;
        End_A_time_down = End_V_max / End_A_max_down;
        End_AT_down = End_V_max * End_A_time_down / 2;

        cout << "末端轨迹规划，梯形速度曲线，运动开始" << endl;

        double dis;
        double position[3];
        double joint[2];

        
        // double PathRecord[6][500];

        int TimeRecord = 0;//时间记录
        flag = 1;
        auto t1 = std::chrono::high_resolution_clock::now();
        auto t3 = std::chrono::high_resolution_clock::now();
        // auto t4 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms;
        while(flag)
        {
            dis = DisJudge_LadderShaped(ControlIntervalTime * TimeRecord);
            // usleep(1000*ControlIntervalTime);
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
            // cout << dis << endl;
            TimeRecord += 1;
            GetLocation_End(dis, position);

            //double position[3];
            TC.invest(position[0], position[1], position[2], joint);
            // if (joint[0] > 100)
            // {
            //     cout << " time:   " << ControlIntervalTime * TimeRecord << " out " << endl;
            //     continue;
            // }
            TC.theta_now[0] = joint[0];
            TC.theta_now[1] = joint[1];
            rob.robotSetPositionAll(TC.theta_now);
 



            JointsRecord[0][TimeRecord] = joint[0];
            JointsRecord[1][TimeRecord] = joint[1];

   
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        fp_ms = t2 - t1;
        cout<<"总计"<<TimeRecord-1<<"个控制点"<<endl;
        cout<<"理论使用时间 "<<(TimeRecord-1)*ControlIntervalTime<<"ms"<<endl;
        cout<<"实际使用时间 "<<fp_ms.count()<<"ms"<<endl;
        
        cout << "末端轨迹规划，梯形速度曲线，运动结束" << endl<< endl;



    }
};

extern Target_data Tar;
#endif