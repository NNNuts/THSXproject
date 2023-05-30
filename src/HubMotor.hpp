#ifndef _CAN2USB_HPP_
#define _CAN2USB_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
// #include <stdio.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <pthread.h>

// #include <ctime>
// #include <cstdlib>
#include "unistd.h"
#include "controlcan.h"
using namespace Eigen;
using namespace std;

class HubMotor
{
private:
public:
    int SpeedMod = 0;
    int PositionMod = 1;
    VCI_CAN_OBJ canData[100];
    int canDataNum = 0;
    int sendSleep = 10;//50ms
    float D = 0.2; //m
    int motorNum[8] = {4,2,3,1};

    void canOpen(void)
    {
        printf("Can卡开始初始化\r\n");//指示程序已运行
        int err = VCI_OpenDevice(VCI_USBCAN2,0,0);
        // cout<<err<<endl;
        if(err==1)//打开设备
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

    void setCommond(int ID, int Len, long data, unsigned long pass = 0)
    {
        // cout<<data<<endl;
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
            canData[canDataNum].Data[i] = data % 256 + pass_new % 256; 
            // cout<<i<<" "<<(int)canData[canDataNum].Data[i]<<" "<<data % 256<<endl;
            data = data / 256;
            pass_new = pass_new / 256;
            // data > 2;
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
        
        usleep(sendSleep*1000);
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

    void motorInit(int ID,int mod)
    {
        ID = motorNum[ID-1];
        if(mod == SpeedMod)
        {
            clearCanData();
            //使能电机(锁死)
            setCommond(0x600 + ID, 8, 0x2B4060000F000000);
            sendCommond();

            //设置速度控制模式
            setCommond(0x600 + ID, 8, 0x2F60600003000000);
            sendCommond();
        }
        else if(mod = PositionMod)
        {
            clearCanData();
            
            //使能电机(锁死)
            // setCommond(0x600 + ID, 8, 0x2B4060000F000000);
            // sendCommond();
            //设置位置控制模式
            setCommond(0x600 + ID, 8, 0x2F60600001000000);
            sendCommond();
            //使能电机(锁死)
            // setCommond(0x600 + ID, 8, 0x2B4060001F000000); //绝对位置模式
            setCommond(0x600 + ID, 8, 0x2B4060000F000000);  //相对位置模式
            sendCommond();

            
        }
    }

    

    void motorSetPosition(int ID, double m)
    {
        ID = motorNum[ID-1];
        clearCanData();
        // setCommond(0x600 + ID, 8, 0x237A600000000000, m/EIGEN_PI*4096/D); //绝对位置模式
        setCommond(0x600 + ID, 8, 0x237B600000000000, m/EIGEN_PI*4096/D);  //相对位置模式
        sendCommond();

        //使能电机
        // setCommond(0x600 + ID, 8, 0x2B4060001F000000);
        // sendCommond();
    }

    void motorSetSpeed(int ID,float m_s)
    {
        ID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + ID, 8, 0x2BF02F0900000000, m_s/EIGEN_PI/D*60);
        sendCommond();
    }

    void motorChangeTrapezoidalVelocityInPosition(int ID,int rpm)
    {
        ID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + ID, 8, 0x2B82600000000000, rpm);
        sendCommond();
        setCommond(0x600 + ID, 8, 0x2FE52F0001000000);
        sendCommond();
    }

    void motorChangeUpAccelerationInSpeed(int ID,double rps_s)
    {
        ID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + ID, 8, 0x2383600000000000, rps_s*256*4096/15625);
        sendCommond();
        setCommond(0x600 + ID, 8, 0x2FE52F0001000000);
        sendCommond();
    }

    void motorChangeDownAccelerationInSpeed(int ID,double rps_s)
    {
        ID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + ID, 8, 0x2384600000000000, rps_s*256*4096/15625);
        sendCommond();
        setCommond(0x600 + ID, 8, 0x2FE52F0001000000);
        sendCommond();
    }

    void motorDisEnable(int ID)
    {
        ID = motorNum[ID-1];
        clearCanData();
        //失能电机(解锁)
        setCommond(0x600 + ID, 8, 0x2B40600006000000);
        sendCommond();
    }

    void motorClearWarning(int ID)
    {
        ID = motorNum[ID-1];
        clearCanData();
        //失能电机(解锁)
        setCommond(0x600 + ID, 8, 0x2B40600086000000);
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
};




#endif