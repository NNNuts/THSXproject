#ifndef _CAN2USB_OLD_HPP_
#define _CAN2USB_OLD_HPP_

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
    int CanID = 0;
    int SpeedMod = 0;
    int PositionMod = 1;
    VCI_CAN_OBJ canData[100];
    int canDataNum = 0;
    int sendSleep = 2;//50ms
    float D = 0.2; //m
    int motorNum[8] = {1,4,2,3,5,6,7,8};
    int motorDir[8] = {-1,-1,1,1,1,1,-1,-1};
    double stepMotorErr[8] = {0,0,0,0,0.261799,0.157080,-1.570796,-1.570796};
    double stepMotorRealPosition[8] = {100, 100, 100, 100, 100, 100, 100, 100};
    double hubMotorRealSpeed[8] = {100, 100, 100, 100, 100, 100, 100, 100};

    HubMotor(int id)
    {
        this->CanID = id;
    }
    HubMotor() = default;

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
        config.AccMask = 0xFFFFFFFF;
        config.Filter  = 0x08;//允许所有类型的数据
        config.Timing0 = 0x00;/*波特率125 Kbps  0x03  0x1C*/ /*波特率500 Kbps  0x00  0x1C*/ /*波特率1000 Kbps  0x00  0x14*/
        config.Timing1 = 0x1C;
        config.Mode    = 0;//正常模式
        if(VCI_InitCAN(VCI_USBCAN2,0,CanID,&config)!=1)
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
        if(VCI_StartCAN(VCI_USBCAN2,0,CanID)!=1)
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
        return VCI_Receive(VCI_USBCAN2,0,CanID,rec,2500,0);
    }

    void canClear(void)
    {
        VCI_ClearBuffer(VCI_USBCAN2, 0, CanID);
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
        VCI_Transmit(VCI_USBCAN2, 0, CanID, canData, canDataNum);

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
        setCommond(0x600 + ID, 8, 0x237B600000000000, m/EIGEN_PI*4096/D*motorDir[ID - 1]);  //相对位置模式
        sendCommond();

        //使能电机
        // setCommond(0x600 + ID, 8, 0x2B4060001F000000);
        // sendCommond();
    }

    void stepMotorSetPosition(int ID, double rad)
    {
        ID = motorNum[ID-1];
        rad = rad + stepMotorErr[ID-1];
        clearCanData();
        setCommond(0x600 + ID, 8, 0x2B70600000000000, (int)(rad*1000)*motorDir[ID-1]);  //绝对位置模式
        sendCommond();
    }

    void stepMotorReadPosition()
    {
        clearCanData();
        VCI_CAN_OBJ rec[2500];
        setCommond(0x605, 8, 0x4072600000000000);  //绝对位置
        sendCommond();
        setCommond(0x606, 8, 0x4072600000000000);  //绝对位置
        sendCommond();
        setCommond(0x607, 8, 0x4072600000000000);  //绝对位置
        sendCommond();
        setCommond(0x608, 8, 0x4072600000000000);  //绝对位置
        sendCommond();
        // ros::Duration(0,1000000).sleep(); 
        int len = canRead(rec);
        // len = 1;
        // rec[0].ID = 0x586;
        // rec[0].Data[0] = 0x4B;
        // rec[0].Data[1] = 0x72;
        // rec[0].Data[2] = 0x60;
        // rec[0].Data[3] = 0x00;
        // rec[0].Data[4] = 0x00;
        // rec[0].Data[5] = 0x00;
        // rec[0].Data[6] = 0x00;
        // rec[0].Data[7] = 0x00;
        if(len){
            for(int l=0;l<len;l++){
                if(CanDataJudge(rec[l],0x585,0x4B72600000000000)){
                    int16_t data = rec[l].Data[4] + rec[l].Data[5]*256;
                    stepMotorRealPosition[4] = ((double)data) / 1000 * motorDir[5-1] - stepMotorErr[5-1];
                    // cout<<"stepMotorRealPosition[4] " << stepMotorRealPosition[4] << endl;
                }
                else if(CanDataJudge(rec[l],0x586,0x4B72600000000000)){
                    int16_t data = rec[l].Data[4] + rec[l].Data[5]*256;
                    stepMotorRealPosition[5] = ((double)data) / 1000 * motorDir[6-1] - stepMotorErr[6-1];
                    // cout<<"stepMotorRealPosition[5] " << stepMotorRealPosition[5] << endl;
                }
                else if(CanDataJudge(rec[l],0x587,0x4B72600000000000)){
                    int16_t data = rec[l].Data[4] + rec[l].Data[5]*256;
                    stepMotorRealPosition[6] = ((double)data) / 1000 * motorDir[7-1] - stepMotorErr[7-1];
                    // cout<<"stepMotorRealPosition[6] " << stepMotorRealPosition[6] << endl;
                }
                else if(CanDataJudge(rec[l],0x588,0x4B72600000000000)){
                    int16_t data = rec[l].Data[4] + rec[l].Data[5]*256;
                    stepMotorRealPosition[7] = ((double)data) / 1000 * motorDir[8-1] - stepMotorErr[8-1];
                    // cout<<"stepMotorRealPosition[7] " << stepMotorRealPosition[7] << endl;
                }
            }
        }
        else{
            cout<<"StepMotor read no data"<<endl;
        }
    }
    void hubMotorReadPosition()
    {
        clearCanData();
        VCI_CAN_OBJ rec[2500];
        setCommond(0x600 + motorNum[1 - 1], 8, 0x40F9601900000000);  //绝对位置
        sendCommond();
        setCommond(0x600 + motorNum[2 - 1], 8, 0x40F9601900000000);  //绝对位置
        sendCommond();
        setCommond(0x600 + motorNum[3 - 1], 8, 0x40F9601900000000);  //绝对位置
        sendCommond();
        setCommond(0x600 + motorNum[4 - 1], 8, 0x40F9601900000000);  //绝对位置
        sendCommond();
        // ros::Duration(0,1000000).sleep(); 
        int len = canRead(rec);
        if(len){
            for(int l=0;l<len;l++){
                if(CanDataJudge(rec[l],0x580 + motorNum[1 - 1],0x43F9601900000000)){
                    int32_t data = rec[l].Data[4] + rec[l].Data[5]*256 + rec[l].Data[6]*256*256 + rec[l].Data[7]*256*256*256;
                    hubMotorRealSpeed[0] = ((double)data) / 1000 * motorDir[1 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[0] " << hubMotorRealSpeed[0] << endl;
                }
                else if(CanDataJudge(rec[l],0x580 + motorNum[2 - 1],0x43F9601900000000)){
                    int32_t data = rec[l].Data[4] + rec[l].Data[5]*256 + rec[l].Data[6]*256*256 + rec[l].Data[7]*256*256*256;
                    hubMotorRealSpeed[1] = ((double)data) / 1000 * motorDir[2 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[1] " << hubMotorRealSpeed[1] << endl;
                }
                else if(CanDataJudge(rec[l],0x580 + motorNum[3 - 1],0x43F9601900000000)){
                    int32_t data = rec[l].Data[4] + rec[l].Data[5]*256 + rec[l].Data[6]*256*256 + rec[l].Data[7]*256*256*256;
                    hubMotorRealSpeed[2] = ((double)data) / 1000 * motorDir[3 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[2] " << hubMotorRealSpeed[2] << endl;
                }
                else if(CanDataJudge(rec[l],0x580 + motorNum[4 - 1],0x43F9601900000000)){
                    int32_t data = rec[l].Data[4] + rec[l].Data[5]*256 + rec[l].Data[6]*256*256 + rec[l].Data[7]*256*256*256;
                    hubMotorRealSpeed[3] = ((double)data) / 1000 * motorDir[4 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[3] " << hubMotorRealSpeed[3] << endl;
                }
            }
        }
        else{
            cout<<"HubMotor read no data"<<endl;
        }
    }

    int CanDataJudge(VCI_CAN_OBJ canObj,int ID,long data){
        if(canObj.ID != ID)
            return 0;
        data /= 0x100000000;
        for(int i=3;i>=0;i--){
            if(canObj.Data[i] != data % 256)
                return 0;
            data /= 256;
        }
        return 1;
    }

    // void stepMotorArriveJudge(int ID,int delay_s)
    // {
    //     VCI_CAN_OBJ rec[2500];
    //     ID = motorNum[ID-1];
    //     clearCanData();
    //     for(int i=0;i<delay_s*1000;i++)
    //     {
    //         setCommond(0x600 + ID, 8, 0x4071600000000000);  //绝对位置模式
    //         sendCommond();
    //         usleep(30000);
    //         int len = canRead(rec);
    //         // cout<<"len"<<len<<endl;
    //         if(len)
    //         {
    //             for(int l=0;l<len;l++)
    //             {
    //                 if(rec[l].ID == 0x580 + ID)
    //                 {
    //                     // cout<<rec[l].Data[4]<<endl;
    //                     if(rec[l].Data[4])
    //                         return;
    //                 }
    //             }
    //         }
            
    //     }
    //     cout<<"超时未抵达"<<endl;
    // }

    void motorSetSpeed(int ID,float m_s)
    {
        int MotorID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + MotorID, 8, 0x2BF02F0900000000, m_s/EIGEN_PI/D*60*motorDir[ID-1]);
        sendCommond();
    }

    void motorChangeTrapezoidalVelocityInPosition(int ID,int rpm)
    {
        int MotorID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + MotorID, 8, 0x2B82600000000000, rpm);
        sendCommond();
        setCommond(0x600 + MotorID, 8, 0x2FE52F0001000000);
        sendCommond();
    }

    void motorChangeUpAccelerationInSpeed(int ID,double rps_s)
    {
        int MotorID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + MotorID, 8, 0x2383600000000000, rps_s*256*4096/15625);
        sendCommond();
        setCommond(0x600 + MotorID, 8, 0x2FE52F0001000000);
        sendCommond();
    }

    void motorChangeDownAccelerationInSpeed(int ID,double rps_s)
    {
        int MotorID = motorNum[ID-1];
        clearCanData();
        setCommond(0x600 + MotorID, 8, 0x2384600000000000, rps_s*256*4096/15625);
        sendCommond();
        setCommond(0x600 + MotorID, 8, 0x2FE52F0001000000);
        sendCommond();
    }

    void motorDisEnable(int ID)
    {
        int MotorID = motorNum[ID-1];
        clearCanData();
        //失能电机(解锁)
        setCommond(0x600 + MotorID, 8, 0x2B40600006000000);
        sendCommond();
    }

    void motorClearWarning(int ID)
    {
        int MotorID = motorNum[ID-1];
        clearCanData();
        //失能电机(解锁)
        setCommond(0x600 + MotorID, 8, 0x2B40600086000000);
        sendCommond();
    }

    // long rad2Hex(double rad)
    // {
    //     long positionData = rad / EIGEN_PI * 16384 * 100;
    //     // cout << "Hex " << positionData << endl;
    //     return num2Hex(positionData);
    // }

    // long num2Hex(long num)
    // {
    //     long hexData = 0;
    //     if(num<0)
    //     {
    //         num = 0x100000000 + num;
    //     }
    //     for(int i = 0; i < 4; i++)
    //     {
    //         hexData = hexData * 256 + num % 256;
    //         num = num / 256;
    //         // cout<<hexData<<endl;
    //     }
        
    //     return hexData;
    // }

    double deg2rad(double deg)
    {
        double rad;
        rad = deg/180*EIGEN_PI;
        // cout << "rad " << rad << endl; 
        return rad;
    }
};




#endif