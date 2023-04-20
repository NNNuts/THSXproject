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
#include "controlcan.h"
using namespace Eigen;
using namespace std;

class CAN2USB
{
private:
    double motorBias[6] = {0, 105.0/180*EIGEN_PI, -10.0/180*EIGEN_PI, 100.0/180*EIGEN_PI, 5.0/180*EIGEN_PI, 0};
    double motorDir[6] = {1, 1, -1, 1, 1, 1};
    // double motorThreshold = {-10,};
    
    // int isOpen = false;
    // int isInit     = false;
    // int isStart  = false;
public:
    VCI_CAN_OBJ canData[100];
    int canDataNum = 0;
    int sendSleep = 10000;//50ms
    int motorOpenSleep = 5000000;//5s
    int motorSpeed = 6000;//6000/10 = 600rpm

    void canOpen(void)
    {
         printf("Can卡开始初始化\r\n");//指示程序已运行
        if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
        {
            printf(">>open deivce CAN success!\n");//打开设备成功
        }else
        {
            printf(">>open deivce CAN error!\n");
            exit(1);
        }
    }

    void canInit(void)
    {
        VCI_INIT_CONFIG config;
        config.AccCode = 0x00000000;
        config.AccMask = 0x00000000;
        config.Filter  = 0x08;//允许所有类型的数据
        config.Timing0 = 0x00;/*波特率125 Kbps  0x03  0x1C*/ /*波特率500 Kbps  0x00  0x1C*/
        config.Timing1 = 0x1C;
        config.Mode    = 0;//正常模式
        if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
        {
            printf(">>Init CAN0 error\n");
		    VCI_CloseDevice(VCI_USBCAN2,0);
		    exit(1);
        }
        else
        {
            printf(">>VCI_InitCAN0 success!\n");
        }
    }

    void canStart(void)
    {
        if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
        {
            printf(">>Start CAN0 error\n");
            VCI_CloseDevice(VCI_USBCAN2,0);
            exit(1);
        }
        else
        {
            printf(">>VCI_StartCAN0 success!\n");
        }
    }

    void canClose(void)
    {
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    int canRead(VCI_CAN_OBJ *rec)
    {
        // VCI_CAN_OBJ rec[100];
        return VCI_Receive(VCI_USBCAN2,0,0,rec,100,100);
    }

    void canClear(void)
    {
        VCI_ClearBuffer(VCI_USBCAN2, 0, 0);
    }

    void setCommond(int ID, int Len, long data)
    {
        if(canDataNum>=100)
        {
            cout << "数据储存溢出" << endl;
            exit(1);
        }
        canData[canDataNum].ID         = ID;
        canData[canDataNum].SendType   = 0;
        canData[canDataNum].RemoteFlag = 0;
        canData[canDataNum].ExternFlag = 0;
        canData[canDataNum].DataLen    = Len;
        for(int i = Len - 1; i >= 0; i--)
        {
            canData[canDataNum].Data[i] = data % 256;
            data = data / 256;
        }
        canDataNum++;
    }

    void sendCommond(void)
    {
        if(!canDataNum)
        {
            cout << "数据储存为空" << endl;
            exit(1);
        }
        VCI_Transmit(VCI_USBCAN2, 0, 0, canData, canDataNum);
        sendNull();
        // cout << "send " << canDataNum << " data" << endl;
        // printf("CAN2 TX ID:0x%08X", canData[0].ID);
		// if(canData[0].ExternFlag==0) printf(" Standard ");
		// if(canData[0].ExternFlag==1) printf(" Extend   ");
		// if(canData[0].RemoteFlag==0) printf(" Data   ");
		// if(canData[0].RemoteFlag==1) printf(" Remote ");
		// printf("DLC:0x%02X",canData[0].DataLen);
		// printf(" data:0x");
		// for(int i = 0; i < canData[0].DataLen; i++)
		// {
		// 	printf(" %02X", canData[0].Data[i]);
		// }
        // printf("\n");
        usleep(sendSleep);
        clearCanData();
    }

    void clearCanData(void)
    {
        canDataNum = 0;
    }

    void sendNull(void)
    {
            VCI_CAN_OBJ nullData[1];
            nullData[0].ID = 0xFFFF;
            nullData[0].ExternFlag = 0;
            nullData[0].RemoteFlag = 0;
            for(int i = 0;i<8;i++)
            {
                nullData[0].Data[i] = 0xFF;
            }
            
            nullData[0].DataLen = 0x08;
            VCI_Transmit(VCI_USBCAN2, 0, 0, nullData, 1);
    }

    void motorOpen(int ID)
    {
        clearCanData();
        setCommond(ID, 0x02, 0x1010);
        sendCommond();
        usleep(motorOpenSleep);
    }

    void motorInit(int ID)
    {
        clearCanData();
        //设置位控模式
        setCommond(0x600 + ID, 8, 0x2F60600001000000);
        sendCommond();

        //设置目标速度
        setCommond(0x600 + ID, 8, 0x2381600000000000 + num2Hex(motorSpeed));
        sendCommond();

        //设置清除异常
        setCommond(0x600 + ID, 8, 0x2B40600080000000);
        sendCommond();

        //设置伺服准备
        setCommond(0x600 + ID, 8, 0x2B40600006000000);
        sendCommond();

        //设置伺服等待使能
        setCommond(0x600 + ID, 8, 0x2B40600007000000);
        sendCommond();

        // 设置伺服使能
        setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        sendCommond();

        // 设置伺服使能
        setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        sendCommond();
    }

    void motorSetPosition(int ID,double position)
    {
        clearCanData();
        // setCommond(0x600 + ID, 8, 0x2B40600080000000);
        // sendCommond();


        setCommond(0x600 + ID, 8, 0x237A600000000000 + rad2Hex(position));
        sendCommond();

        setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        sendCommond();

        setCommond(0x600 + ID, 8, 0x2B4060003F000000);
        sendCommond();
    }

    void changeMotorID(int ID, int newID)
    {
        clearCanData();
        setCommond(0x600 + ID, 8, 0x2F26200000000000 + newID*0x1000000);
        sendCommond();
        setCommond(0x600 + ID, 8, 0x2310100173617665);
        sendCommond();
    }

    void changeMotorSpeed(int ID, int speed)
    {
        clearCanData();
        //设置目标速度
        setCommond(0x600 + ID, 8, 0x2381600000000000 + num2Hex(speed));
        sendCommond();
    }

    double motorReadPosition(int ID)
    {
        VCI_CAN_OBJ rec[100];
        clearCanData();
        canClear();
        setCommond(0x600 + ID, 8, 0x4064600000000000);
        sendCommond();
        int Len = canRead(rec);
        if(Len)
        {
            int i;
            for(i=0;i<Len;i++)
            {
                if(ackCheck(ID,0x4064600000000000,rec[i]))
                {
                    double position = rec[i].Data[7] * 256 * 256 * 256 + rec[i].Data[6] * 256 * 256 + rec[i].Data[5] * 256 + rec[i].Data[4];
                    if(position > 0x80000000)
                        position = position - 0x100000000;
                    return position / 100 / 16384 * EIGEN_PI;
                }
            }
        }
        return 10000;
    }

    int ackCheck(int ID,long data, VCI_CAN_OBJ OBJ)
    {
        if(OBJ.ID == 0x580+ID && OBJ.Data[1] == (data/0x1000000000000)%256 && OBJ.Data[2] == (data/0x10000000000)%256)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void robotReset(void)
    {
        int ID;
        clearCanData();
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2F08200001000000);
        }
        sendCommond();
        usleep(1000000);
        return;
    }

    void robotInit(void)
    {
        int ID;
        clearCanData();
        for(ID=1;ID<=6;ID++)
        {
            setCommond(ID, 2, 0x1010);
        }
        sendCommond();
        usleep(motorOpenSleep);
        // return;

        //设置位控模式
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2F60600001000000);
        }
        sendCommond();

        //设置目标速度
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2381600000000000 + num2Hex(motorSpeed));
        }
        sendCommond();

        //设置清除异常
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2B40600080000000);
        }
        sendCommond();

        //设置伺服准备
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2B40600006000000);
        }
        sendCommond();

        //设置伺服等待使能
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2B40600007000000);
        }
        sendCommond();

        // 设置伺服使能
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        }
        sendCommond();

        // 设置伺服使能
        for(ID=1;ID<=6;ID++)
        {
            setCommond(0x600 + ID, 8, 0x2B4060002F000000);
        }
        sendCommond();
    }

    void motorSetPositionAll(double joint1,double joint2,double joint3,double joint4,double joint5,double joint6)
    {
        clearCanData();
        setCommond(0x601, 8, 0x237A600000000000 + rad2Hex(joint1));
        setCommond(0x602, 8, 0x237A600000000000 + rad2Hex(joint2));
        setCommond(0x603, 8, 0x237A600000000000 + rad2Hex(joint3));
        setCommond(0x604, 8, 0x237A600000000000 + rad2Hex(joint4));
        setCommond(0x605, 8, 0x237A600000000000 + rad2Hex(joint5));
        setCommond(0x606, 8, 0x237A600000000000 + rad2Hex(joint6));
        sendCommond();

        setCommond(0x601, 8, 0x2B4060002F000000);
        setCommond(0x602, 8, 0x2B4060002F000000);
        setCommond(0x603, 8, 0x2B4060002F000000);
        setCommond(0x604, 8, 0x2B4060002F000000);
        setCommond(0x605, 8, 0x2B4060002F000000);
        setCommond(0x606, 8, 0x2B4060002F000000);
        sendCommond();

        setCommond(0x601, 8, 0x2B4060003F000000);
        setCommond(0x602, 8, 0x2B4060003F000000);
        setCommond(0x603, 8, 0x2B4060003F000000);
        setCommond(0x604, 8, 0x2B4060003F000000);
        setCommond(0x605, 8, 0x2B4060003F000000);
        setCommond(0x606, 8, 0x2B4060003F000000);
        sendCommond();
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
        double position[6] = {rad[0], rad[1], rad[2], rad[3], rad[4],rad[5]};
        int i;
        for(i = 0;i < 6;i++)
        {
            position[i] = position[i]*motorDir[i] + motorBias[i];
        }
        motorSetPositionAll(position[0], position[1], position[2], position[3], position[4], position[5]);
    }

    void robotReadPositionAll(double* res)
    {
        double rad[6];
        int i;
        for(i = 0;i < 6;i++)
        {
            rad[i] = motorReadPosition(i + 1);
            rad[i] = (rad[i] - motorBias[i]) * motorDir[i];
        }
        *res = rad[0];res++;
        *res = rad[1];res++;
        *res = rad[2];res++;
        *res = rad[3];res++;
        *res = rad[4];res++;
        *res = rad[5];res++;
    }
};




#endif