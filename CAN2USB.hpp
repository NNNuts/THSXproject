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
    // double motorBias[2] = {30.0/180*EIGEN_PI, 40.0/180*EIGEN_PI};
    double motorBias[2] = {0.0/180*EIGEN_PI, 0.0/180*EIGEN_PI};
    double motorDir[2] = {-1, -1};
    int CanDataSendNum = 0;
    // double motorThreshold = {-10,};
    
    // int isOpen = false;
    // int isInit     = false;
    // int isStart  = false;
public:
    VCI_CAN_OBJ canData[100];
    int canDataNum = 0;
    int sendSleep = 10000;//50ms
    int motorOpenSleep = 5000000;//5s
    int motorSpeed = 10000;//6000/10 = 600rpm

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
        // cout<<"send ";
        for(int i = Len - 1; i >= 0; i--)
        {
            canData[canDataNum].Data[i] = data % 256;
            data = data / 256;
            // cout<<(unsigned)canData[canDataNum].Data[i]<<" ";
        }
        // cout<<endl;
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
        // sendNull();
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

    void robotSetPosition(int ID,double rad)
    {
        VCI_CAN_OBJ rec[100];
        CanDataSendNum ++;
        long data = 0;
        data += 0x0100000000000000 * ID;
        data += 0x01000000000000;
        data += 0x010000000000;
        if(rad >= 0)
        {
            data += ((unsigned)(rad*1000/256))*0x0100000000;
            data += ((unsigned)(rad*1000)%256)*0x01000000;
        }
        else
        {
            data += ((unsigned)(-rad*1000/256+128))*0x0100000000;
            data += ((unsigned)(-rad*1000)%256)*0x01000000;
        }
        // data += ((int)rad*1000%256)*0x01000000;
        // data = 0x0201010200000000;
        setCommond(66,8,data);
        canClear();
        sendCommond();
        usleep(2000);
        int Len = canRead(rec);
        if(Len)
        {
            int i;
            for(i=0;i<Len;i++)
            {
                if(ackCheck(ID+0x10,0x0001010000000000,rec[i]))
                {
                    // cout<<"Ack resive"<<endl;
                }
            }
            // for(i=0;i<Len;i++)
            // {
            //     cout<<(unsigned)rec[0].Data[0]<<" "<<(unsigned)rec[0].Data[1]<<" "<<(unsigned)rec[0].Data[2]<<" "<<(unsigned)rec[0].Data[3]<<" "
            //         <<(unsigned)rec[0].Data[4]<<" "<<(unsigned)rec[0].Data[5]<<" "<<(unsigned)rec[0].Data[6]<<" "<<(unsigned)rec[0].Data[7]<<" "<<endl;
            // }     
            
        }
        // cout<<endl;
        // cout << "CanDataSendNum "<< CanDataSendNum<<endl;
    }

    void robotSetPositionAll(double rad[2])
    {
        //cout<<rad[0]<< " " << rad[1]<<endl;
        robotSetPosition(1,rad[0]*motorDir[0] + motorBias[0]);
        robotSetPosition(2,rad[1]*motorDir[1] + motorBias[1]);
    }

    int ackCheck(int ID,long data, VCI_CAN_OBJ OBJ)
    {
        if(OBJ.Data[0] == ID && OBJ.Data[1] == (data/0x1000000000000)%256 && OBJ.Data[2] == (data/0x10000000000)%256)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void motorChangPID(int ID,unsigned char P,unsigned char I,unsigned char D,int limit)
    {
        long data = 0;
        data += 0x0100000000000000 * ID;
        data += 0x00000000000000;
        data += 0x050000000000;
        data += 0x0100000000 * P;
        data += 0x01000000 * I;
        data += 0x010000 * D;
        data += 0x0100 * (int)(limit/1000/256);
        data += (int)(limit/1000%256);
 
        setCommond(66,8,data);
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