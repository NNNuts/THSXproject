#ifndef _CAN2USB_HPP_
#define _CAN2USB_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include "socketCAN.h"

// #include <linux/can.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <linux/can/raw.h>
// #include <unistd.h>
// #include "controlcan.h"
using namespace Eigen;
using namespace std;

class CAN2USB : public socketCAN                                  
{
private:
    // double motorBias[6] = {0, 104.0/180*EIGEN_PI, -1.0/180*EIGEN_PI, 65.0/180*EIGEN_PI, 147.0/180*EIGEN_PI, 0};
    double motorBias[6] = {-9.0/180*EIGEN_PI, 104.0/180*EIGEN_PI, -2.0/180*EIGEN_PI, 63.0/180*EIGEN_PI, 145.5/180*EIGEN_PI, 0};
    double motorDir[6] = {1, 1, -1, 1, 1, 1};
    double jointsLimit[6][2] = {-180./180*EIGEN_PI, 180./180*EIGEN_PI, 
                                -210./180*EIGEN_PI,  30./180*EIGEN_PI,
                                -150./180*EIGEN_PI, 150./180*EIGEN_PI,
                                -210./180*EIGEN_PI,  30./180*EIGEN_PI,
                                -180./180*EIGEN_PI, 180./180*EIGEN_PI,
                                -180./180*EIGEN_PI, 180./180*EIGEN_PI,};
    // double motorThreshold = {-10,};
    
    // int isOpen = false;
    // int isInit     = false;
    // int isStart  = false;
public:
    int Can = Can0;
    int motorOpenSleep = 5000000;//5s
    int motorSpeed = 10000;//6000/10 = 600rpm

    void canSend(int com, int ID, int Len, long data, unsigned long pass = 0){
        struct can_frame frame;

        long pass_new = 0;
        for(int i = 0;i<4;i++)
        {
            pass_new *= 256;
            pass_new += pass % 256;
            pass /= 256;
        }

        for(int i = Len - 1; i >= 0; i--)
        {
            // canData[canDataNum].Data[i] = data % 256;
            frame.data[i] = data % 256 + pass_new % 256; 
            // cout<<i<<" "<<(int)canData[canDataNum].Data[i]<<" "<<data % 256<<endl;
            data = data / 256;
            pass_new = pass_new / 256;
            // data > 2;
        }

        /************ 写数据 ************/
        frame.can_dlc = Len;  // 设置数据长度（CAN协议规定一帧最多有八个字节的有效数据）
        frame.can_id = ID;    // 设置 ID 号，假设这里 ID 号为1，实际的 ID 号要根据是标准帧（11位）还是拓展帧（29）位来设置
        write(Can_fd[com], &frame, sizeof(frame));  // 写数据
        usleep(CanSendSleep);


    }

    void canInit(void){
        CanPort = 24;
        canOpen();
        canStartPthread(CanAll);
        canSetSendSleep(3000);
    }

    void motorOpen(int ID)
    {
        canSend(Can, ID, 2, 0x1010);
        usleep(motorOpenSleep);
    }

    void motorInit(int ID)
    {
        canSend(Can, 0x600 + ID, 8, 0x2F60600001000000);
        canSend(Can, 0x600 + ID, 8, 0x2381600000000000 + num2Hex(motorSpeed));
        canSend(Can, 0x600 + ID, 8, 0x2B40600080000000);
        canSend(Can, 0x600 + ID, 8, 0x2B40600006000000);
        canSend(Can, 0x600 + ID, 8, 0x2B40600007000000);
        canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);

        // clearCanData();
        // //设置位控模式
        // setCommond(0x600 + ID, 8, 0x2F60600001000000);
        // sendCommond();

        // //设置目标速度
        // setCommond(0x600 + ID, 8, 0x2381600000000000 + num2Hex(motorSpeed));
        // sendCommond();

        // //设置清除异常
        // setCommond(0x600 + ID, 8, 0x2B40600080000000);
        // sendCommond();

        // //设置伺服准备
        // setCommond(0x600 + ID, 8, 0x2B40600006000000);
        // sendCommond();

        // //设置伺服等待使能
        // setCommond(0x600 + ID, 8, 0x2B40600007000000);
        // sendCommond();

        // // 设置伺服使能
        // setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        // sendCommond();

        // // 设置伺服使能
        // setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        // sendCommond();
    }

    void motorSetPosition(int ID,double position)
    {
        // clearCanData();

        canSend(Can, 0x600 + ID, 8, 0x237A600000000000 + rad2Hex(position));
        // sendAck(ID,0x237A600000000000);
        canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);
        // sendAck(ID,0x2B4060002F000000);
        canSend(Can, 0x600 + ID, 8, 0x2B4060003F000000);
        // sendAck(ID,0x2B4060003F000000);

        // setCommond(0x600 + ID, 8, 0x237A600000000000 + rad2Hex(position));
        // sendCommond();

        // setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        // sendCommond();

        // setCommond(0x600 + ID, 8, 0x2B4060003F000000);
        // sendCommond();
    }

    void changeMotorID(int ID, int newID)
    {
        canSend(Can, 0x600 + ID, 8, 0x2F26200000000000 + newID*0x1000000);
        canSend(Can, 0x600 + ID, 8, 0x2310100173617665);
        // clearCanData();
        // setCommond(0x600 + ID, 8, 0x2F26200000000000 + newID*0x1000000);
        // sendCommond();
        // setCommond(0x600 + ID, 8, 0x2310100173617665);
        // sendCommond();
    }

    void changeMotorSpeed(int ID, int speed)
    {
        canSend(Can, 0x600 + ID, 8, 0x2381600000000000 + num2Hex(speed));
        // clearCanData();
        // //设置目标速度
        // setCommond(0x600 + ID, 8, 0x2381600000000000 + num2Hex(speed));
        // sendCommond();
    }

    double motorReadPosition(int ID)
    {
        // VCI_CAN_OBJ rec[100];
        // clearCanData();
        canClean(Can);
        // setCommond(0x600 + ID, 8, 0x4064600000000000);
        // sendCommond();
        canSend(Can, 0x600 + ID, 8, 0x4064600000000000);
        // int Len = canRead(rec);
        // cout<<"Len "<<Len<<endl;
        if(CanReadDataNum[Can])
        {
            // cout<<"Len "<<CanReadDataNum[Can]<<endl;
            for(int i=0;i<CanReadDataNum[Can];i++)
            {
                // cout<<"Read ID "<<CanReadData[Can][i].can_id<<endl;
                if(ackCheck(ID,0x4064600000000000,CanReadData[Can][i]))
                {
                    // cout<<"ReadCheck "<<ID<<endl;
                    double position = CanReadData[Can][i].data[7] * 256 * 256 * 256 + CanReadData[Can][i].data[6] * 256 * 256 + CanReadData[Can][i].data[5] * 256 + CanReadData[Can][i].data[4];
                    // cout<<"position "<<position<<endl;
                    if(position > 0x80000000)
                        position = position - 0x100000000;
                    CanReadDataNum[Can] = 0;
                    return position / 100 / 16384 * EIGEN_PI;
                }
            }
        }
        CanReadDataNum[Can] = 0;
        return 10000;
    }

    int ackCheck(int ID,long data, can_frame OBJ)
    {
        if(OBJ.can_id == 0x580+ID && OBJ.data[1] == (data/0x1000000000000)%256 && OBJ.data[2] == (data/0x10000000000)%256)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int sendAck(int ID,long data)
    {
        if(CanReadDataNum[Can])
        {
            // cout<<"Len "<<CanReadDataNum[Can]<<endl;
            for(int i=0;i<CanReadDataNum[Can];i++)
            {
                // cout<<"Read ID "<<CanReadData[Can][i].can_id<<endl;
                if(ackCheck(ID,data,CanReadData[Can][i]))
                {
                    
                    return 0;
                }
            }
        }
        canClean(Can);
        cout << ID << "号电机发送未确认" <<endl;
        return 1;
    }

    void robotReset(void)
    {
        // int ID;
        // clearCanData();
        for(int ID=1;ID<=6;ID++)
        {
            // setCommond(0x600 + ID, 8, 0x2F08200001000000);
            // sendCommond();
            canSend(Can, 0x600 + ID, 8, 0x2F08200001000000);
        }
        // sendCommond();
        usleep(1000000);
        return;
    }

    void robotInit(void)
    {
        int ID;
        // clearCanData();
        for(ID=1;ID<=6;ID++)
        {
            // setCommond(ID, 2, 0x1010);
            // sendCommond();
            canSend(Can, ID, 2, 0x1010);
        }
        // sendCommond();
        usleep(motorOpenSleep);
        // return;

        //设置位控模式
        for(ID=1;ID<=6;ID++)
        {
            canSend(Can, 0x600 + ID, 8, 0x2F60600001000000);
            // setCommond(0x600 + ID, 8, 0x2F60600001000000);
            // sendCommond();
        }
        // sendCommond();
        

        //设置目标速度
        for(ID=1;ID<=6;ID++)
        {
            canSend(Can, 0x600 + ID, 8, 0x2381600000000000 + num2Hex(motorSpeed));
            // setCommond(0x600 + ID, 8, 0x2381600000000000 + num2Hex(motorSpeed));
            // sendCommond();
        }
        // sendCommond();
        

        //设置清除异常
        for(ID=1;ID<=6;ID++)
        {
            canSend(Can, 0x600 + ID, 8, 0x2B40600080000000);
            // setCommond(0x600 + ID, 8, 0x2B40600080000000);
            // sendCommond();
        }
        // sendCommond();
        // exit(0);

        //设置伺服准备
        for(ID=1;ID<=6;ID++)
        {
            canSend(Can, 0x600 + ID, 8, 0x2B40600006000000);
            // setCommond(0x600 + ID, 8, 0x2B40600006000000);
            // sendCommond();
        }
        // sendCommond();

        //设置伺服等待使能
        for(ID=1;ID<=6;ID++)
        {
            canSend(Can, 0x600 + ID, 8, 0x2B40600007000000);
            // setCommond(0x600 + ID, 8, 0x2B40600007000000);
            // sendCommond();
        }
        // sendCommond();

        // 设置伺服使能
        for(ID=1;ID<=6;ID++)
        {
            canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);
            // setCommond(0x600 + ID, 8, 0x2B4060002F000000);
            // sendCommond();
        }
        // sendCommond();

        // 设置伺服使能
        for(ID=1;ID<=6;ID++)
        {
            canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);
            // setCommond(0x600 + ID, 8, 0x2B4060002F000000);
            // sendCommond();
        }
        usleep(2000000);
        // sendCommond();
    }

    void motorSetPositionAll(double joint1,double joint2,double joint3,double joint4,double joint5,double joint6)
    {
        // clearCanData();
        motorSetPosition(1,joint1);
        motorSetPosition(2,joint2);
        motorSetPosition(3,joint3);
        motorSetPosition(4,joint4);
        motorSetPosition(5,joint5);
        motorSetPosition(6,joint6);
        // canSend(Can, 0x601, 8, 0x237A600000000000 + rad2Hex(joint1));
        // sendAck(1,0x237A600000000000);
        // canSend(Can, 0x602, 8, 0x237A600000000000 + rad2Hex(joint2));
        // canSend(Can, 0x603, 8, 0x237A600000000000 + rad2Hex(joint3));
        // canSend(Can, 0x604, 8, 0x237A600000000000 + rad2Hex(joint4));
        // canSend(Can, 0x605, 8, 0x237A600000000000 + rad2Hex(joint5));
        // canSend(Can, 0x606, 8, 0x237A600000000000 + rad2Hex(joint6));
        // setCommond(0x601, 8, 0x237A600000000000 + rad2Hex(joint1));
        // sendCommond();
        // setCommond(0x602, 8, 0x237A600000000000 + rad2Hex(joint2));
        // sendCommond();
        // setCommond(0x603, 8, 0x237A600000000000 + rad2Hex(joint3));
        // sendCommond();
        // setCommond(0x604, 8, 0x237A600000000000 + rad2Hex(joint4));
        // sendCommond();
        // setCommond(0x605, 8, 0x237A600000000000 + rad2Hex(joint5));
        // sendCommond();
        // setCommond(0x606, 8, 0x237A600000000000 + rad2Hex(joint6));
        // sendCommond();

        // for(int ID = 1; ID <=6; ID++){
        //     canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);
        //     canSend(Can, 0x600 + ID, 8, 0x2B4060003F000000);
        // }


        // setCommond(0x601, 8, 0x2B4060002F000000);
        // sendCommond();
        // setCommond(0x602, 8, 0x2B4060002F000000);
        // sendCommond();
        // setCommond(0x603, 8, 0x2B4060002F000000);
        // sendCommond();
        // setCommond(0x604, 8, 0x2B4060002F000000);
        // sendCommond();
        // setCommond(0x605, 8, 0x2B4060002F000000);
        // sendCommond();
        // setCommond(0x606, 8, 0x2B4060002F000000);
        // sendCommond();

        // setCommond(0x601, 8, 0x2B4060003F000000);
        // sendCommond();
        // setCommond(0x602, 8, 0x2B4060003F000000);
        // sendCommond();
        // setCommond(0x603, 8, 0x2B4060003F000000);
        // sendCommond();
        // setCommond(0x604, 8, 0x2B4060003F000000);
        // sendCommond();
        // setCommond(0x605, 8, 0x2B4060003F000000);
        // sendCommond();
        // setCommond(0x606, 8, 0x2B4060003F000000);
        // sendCommond();
    }

    long rad2Hex(double rad)
    {
        long positionData = rad / EIGEN_PI * 16384 * 100;
        // cout << "Hex " << positionData << endl;
        return num2Hex(positionData);
    }

    long num2Hex(long num)
    {
        long hexData = 0;
        if(num<0)
        {
            num = 0x100000000 + num;
        }
        for(int i = 0; i < 4; i++)
        {
            hexData = hexData * 256 + num % 256;
            num = num / 256;
            // cout<<hexData<<endl;
        }
        // cout<<"hexData "<<hexData<<endl;
        return hexData;
    }

    double deg2rad(double deg)
    {
        double rad;
        rad = deg/180*EIGEN_PI;
        // cout << "rad " << rad << endl; 
        return rad;
    }
    
    void robotSetPosition(int ID, double rad)
    {
        double position = rad*motorDir[ID - 1] + motorBias[ID - 1];
        motorSetPosition(ID,position);
    }

    double robotReadPosition(int ID)
    {
        double position = motorReadPosition(ID);
        position = (position - motorBias[ID - 1]) * motorDir[ID - 1];
        return position;
    }

    void robotSetPositionAll(double rad[6])
    {
        // checkJointLimit(rad);
        if(Tar.robotIsSelfCollide(rad))
        {
            cout<<"发生碰撞"<<endl;
            exit(0);
        }
        double position[6] = {rad[0], rad[1], rad[2], rad[3], rad[4],rad[5]};
        int i;
        for(i = 0;i < 6;i++)
        {
            position[i] = position[i]*motorDir[i] + motorBias[i];
            // cout<<"position "<<position[i]<<endl;
        }
        motorSetPositionAll(position[0], position[1], position[2], position[3], position[4], position[5]);
    }

    void robotReadPositionAll(double* res)
    {
        double rad[6];
        int i;
        for(i = 0;i < 6;i++)
        {
            // canClean(Can);
            rad[i] = motorReadPosition(i + 1);
            // cout<<"rad "<<rad[i]<<endl;
            rad[i] = (rad[i] - motorBias[i]) * motorDir[i];
        }
        *res = rad[0];res++;
        *res = rad[1];res++;
        *res = rad[2];res++;
        *res = rad[3];res++;
        *res = rad[4];res++;
        *res = rad[5];res++;
    }

    void checkJointLimit(double joints[6])
    {
        for(int i=0;i<6;i++)
        {
            if(joints[i]<jointsLimit[i][0])
            {
                cout<<"joint"<<i<<" "<<joints[i]<<" too small"<<endl;
                exit(0);
            }
            if(joints[i]>jointsLimit[i][1])
            {
                cout<<"joint"<<i<<" "<<joints[i]<<" too big"<<endl;
                exit(0);
            }
        }
    }

};




#endif