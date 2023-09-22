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

#include <HubMotor_pkg/controlcan.h>

#include <HubMotor_pkg/Can2Usb2.0.h>
using namespace Eigen;
using namespace std;

void CAN2USB::canOpen(void){
    printf("Can卡开始初始化\r\n");//指示程序已运行
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
        printf(">>open deivce CAN success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce CAN error!\n");
        exit(1);
    }
    VCI_INIT_CONFIG config;
    config.AccCode = 0xFFFFFFFF;
    config.AccMask = 0xFFFFFFFF;
    config.Filter  = 0x00;//允许所有类型的数据
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

    if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(1);
    }
    else
    {
        printf(">>VCI_InitCAN1 success!\n");
    }
    if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        exit(1);
    }
    else
    {
        printf(">>VCI_StartCAN1 success!\n");
    }
}

void CAN2USB::canSend(int com,int ID, int Len, long data, unsigned long pass){
    VCI_CAN_OBJ canData;
    canData.ID         = ID;
    canData.SendType   = 0;
    canData.RemoteFlag = 0;
    canData.ExternFlag = 0;
    canData.DataLen    = Len;
    long pass_new = 0;
    for(int i = 0;i<4;i++)
    {
        pass_new *= 256;
        pass_new += pass % 256;
        pass /= 256;
    }
    for(int i = Len - 1; i >= 0; i--)
    {
        canData.Data[i] = data % 256 + pass_new % 256;
        data = data / 256;
        pass_new = pass_new / 256;
    }
    VCI_Transmit(VCI_USBCAN2, 0, com, &canData, 1);
    // sendNull();
    // cout << "send data" << endl;
    // printf("CAN TX ID:0x%08X", canData.ID);
    // if(canData.ExternFlag==0) printf(" Standard ");
    // if(canData.ExternFlag==1) printf(" Extend   ");
    // if(canData.RemoteFlag==0) printf(" Data   ");
    // if(canData.RemoteFlag==1) printf(" Remote ");
    // printf("DLC:0x%02X",canData.DataLen);
    // printf(" data:0x");
    // for(int i = 0; i < canData.DataLen; i++)
    // {
    //     printf(" %02X", canData.Data[i]);
    // }
    // printf("\n");
    usleep(CanSendSleep);
}

void CAN2USB::canSetSendSleep(int us){
    CanSendSleep = us;
}

int CAN2USB::canRead(VCI_CAN_OBJ* frame){
    return VCI_Receive(VCI_USBCAN2,0,0,frame,100,0); 
}
