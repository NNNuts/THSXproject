#ifndef _CAN2USB_HPP_
#define _CAN2USB_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
// #include <stdio.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <pthread.h>

// #include <ctime>
// #include <cstdlib>
#include "unistd.h"
// #include "lib_emuc_2.h"
using namespace Eigen;
using namespace std;
enum
{
    SpeedMod,
    PositionMod
};

class HubMotor
{
private:
public:
    // int CanID = 0;
    int ComPort = 24;
    int can_fd[2];
    int Hub = 0;
    int Step = 1;
    // int SpeedMod = 0;
    // int PositionMod = 1;
    // CAN_FRAME_INFO canData[100];
    // int canDataNum = 0;
    struct can_frame CanReadData[2][1000];
    int CanReadDataNumMax = 1000;
    int CanReadDataNum[2] = {0, 0};

    int sendSleep = 2;//50ms
    float D = 0.2; //m
    int motorNum[8] = {1,4,2,3,5,6,7,8};
    int motorDir[8] = {-1,-1,1,1,1,1,-1,-1};
    double stepMotorErr[8] = {0,0,0,0,0.261799,0.157080,-1.570796,-1.570796};
    double stepMotorRealPosition[8] = {100, 100, 100, 100, 100, 100, 100, 100};
    double hubMotorRealSpeed[8] = {100, 100, 100, 100, 100, 100, 100, 100};

    // HubMotor(int id)
    // {
    //     this->CanID = id;
    // }
    // HubMotor() = default;

    // void canOpen(void)
    // {
    //     printf("Can卡开始初始化\r\n");//指示程序已运行
    //     int err = EMUCOpenDevice(ComPort);
    //     // cout<<err<<endl;
    //     if(err==0)//打开设备
    //     {
    //         printf(">>open deivce CAN success!\n");//打开设备成功
    //     }else
    //     {
    //         printf(">>open deivce CAN error!\n");
    //         exit(1);
    //     }
    // }

    void *can0_get_status(void){
        pthread_detach(pthread_self());
        // long tid;
        // tid = (long) can_fd;
        int err;
        while(true){
            
            if(CanReadDataNum[0]>=CanReadDataNumMax)
                CanReadDataNum[0] = 0;
            // cout<<num<<endl;
            err = read(can_fd[0], &CanReadData[0][CanReadDataNum[0]], sizeof(CanReadData[0][CanReadDataNum[0]]));
            CanReadDataNum[0] ++;
            // cout<<err<<endl;
            // cout<<"0 "<<(int)CanReadData[0][CanReadDataNum[0]].data[0]<<endl;
        }
    }

    static void* staticCan0_get_status(void *param){
        HubMotor* pThis = (HubMotor*)param;
        pThis->can0_get_status();
    }

    void *can1_get_status(void){
        pthread_detach(pthread_self());
        // long tid;
        // tid = (long) can_fd;
        int err;
        while(true){
            
            if(CanReadDataNum[1]>=CanReadDataNumMax)
                CanReadDataNum[1] = 0;
            // cout<<num<<endl;
            err = read(can_fd[1], &CanReadData[1][CanReadDataNum[1]], sizeof(CanReadData[1][CanReadDataNum[1]]));
            CanReadDataNum[1] ++;
            // cout<<err<<endl;
            // cout<<"1 "<<(int)CanReadData[1][CanReadDataNum[1]].data[0]<<endl;
        }
    }

    static void* staticCan1_get_status(void *param){
        HubMotor* pThis = (HubMotor*)param;
        pThis->can1_get_status();
    }

    void canOpen(void){
        can_fd[0] = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        can_fd[1] = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        if(can_fd < 0)
        {
            perror("socket can creat error!\n");
            exit(0);
        }
        struct ifreq ifr;  // if.h
        strcpy(ifr.ifr_name, "can0");
        ioctl(can_fd[0], SIOCGIFINDEX, &ifr); // 指定编号为 can0 的设备，获取设备索引
        
    
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;  // 指定协议族
        addr.can_ifindex = ifr.ifr_ifindex;  // 设备索引
        // 将套接字与 can0 绑定
        int bind_res = bind(can_fd[0], (struct sockaddr *)&addr, sizeof(addr));
        if(bind_res < 0)
        {
            perror("bind error!");
            exit(0);
        }

        strcpy(ifr.ifr_name, "can1");
        ioctl(can_fd[1], SIOCGIFINDEX, &ifr); // 指定编号为 can0 的设备，获取设备索引
        addr.can_ifindex = ifr.ifr_ifindex;  // 设备索引

        // 将套接字与 can0 绑定
        bind_res = bind(can_fd[1], (struct sockaddr *)&addr, sizeof(addr));
        if(bind_res < 0)
        {
            perror("bind error!");
            exit(0);
        }

        int loopback = 0; //0 表示关闭，1 表示开启(默认) 
        setsockopt(can_fd[0], SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
        setsockopt(can_fd[1], SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

        pthread_t threads;
        int rc;
        long fd;
        fd = can_fd[0];
        rc = pthread_create(&threads, NULL, staticCan0_get_status, (void *)this);
        fd = can_fd[1];
        rc = pthread_create(&threads, NULL, staticCan1_get_status, (void *)this);

        cout<<"Can Open success"<<endl;
    }

    // void canOpen(void)
    // {
    //     int rtn;
    //     printf("Can卡开始初始化\r\n");//指示程序已运行
    //     rtn = EMUCOpenDevice(ComPort);
    //     cout<<rtn<<endl;
    //     if(rtn==0)//打开设备
    //     {
    //         printf(">>open deivce CAN success!\n");//打开设备成功
    //     }else
    //     {
    //         printf(">>open deivce CAN error!\n");
    //         exit(1);
    //     }
    //     /* ----- EMUCSetBaudRate() ----- */
    //     rtn = EMUCSetBaudRate(ComPort, EMUC_BAUDRATE_500K, EMUC_BAUDRATE_1M);
    //     if (rtn)
    //     {
    //         printf("EMUC set baud rate failed !\n");
    //     }
    //     else
    //     {
    //         printf("EMUC set baud rate successfully !\n");
    //         printf("==============================================\n");
    //     }
    //     /* ----- EMUCSetMode() ----- */
    //     rtn = EMUCSetMode(ComPort, EMUC_NORMAL, EMUC_NORMAL);
    //     if (rtn)
    //     {
    //         printf("EMUC set mode failed !\n");
    //     }
    //     else
    //     {
    //         printf("EMUC set mode successfully !\n");
    //         printf("==============================================\n");
    //     }
    //     /* ----- EMUCInitCAN() ----- */
    //     EMUCInitCAN(ComPort, EMUC_ACTIVE, EMUC_ACTIVE);
    // }

    // void canStart(void)
    // {
    //     if(VCI_StartCAN(VCI_USBCAN2,0,CanID)!=1)
    //     {
    //         printf(">>Start CAN0 error\n");
    //         VCI_CloseDevice(VCI_USBCAN2,0);
    //         exit(1);
    //     }
    //     else
    //     {
    //         printf(">>VCI_StartCAN0 success!\n");
    //     }
    // }

    // void canClose(void)
    // {
    //     EMUCCloseDevice(ComPort);
    // }

    // int canRead(int com, CAN_FRAME_INFO *rec)
    // {
    //     // VCI_CAN_OBJ rec[100];
    //     return EMUCReceive(com,rec);
    // }

    // void canClear(void)
    // {
    //     VCI_ClearBuffer(VCI_USBCAN2, 0, CanID);
    // }

    // int canRead(int com,can_frame *frame)
    // {
    //     int num;
    //     num = CanReadDataNum[com];

    // }

    void canClean(int com){
        CanReadDataNum[0] = 0;
    }

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
        write(can_fd[com], &frame, sizeof(frame));  // 写数据
        usleep(sendSleep*1000);
    }

    // void send(int com, int ID, int Len, long data, unsigned long pass = 0)
    // {
    //     // cout<<data<<endl;
    //     // if(canDataNum>=100)
    //     // {
    //     //     cout << "数据储存溢出" << endl;
    //     //     exit(1);
    //     // }
    //     CAN_FRAME_INFO  frame_send;
    //     frame_send.CAN_port = com;
    //     frame_send.id_type = EMUC_SID;
    //     frame_send.rtr = EMUC_DIS_RTR;
    //     frame_send.dlc = 8;
    //     frame_send.id = ID;
    //     long pass_new = 0;

    //     for(int i = 0;i<4;i++)
    //     {
    //         pass_new *= 256;
    //         pass_new += pass % 256;
    //         pass /= 256;
    //     }

    //     for(int i = Len - 1; i >= 0; i--)
    //     {
    //         // canData[canDataNum].Data[i] = data % 256;
    //         frame_send.data[i] = data % 256 + pass_new % 256; 
    //         // cout<<i<<" "<<(int)canData[canDataNum].Data[i]<<" "<<data % 256<<endl;
    //         data = data / 256;
    //         pass_new = pass_new / 256;
    //         // data > 2;
    //     }
    //     EMUCSend(ComPort, &frame_send);
    //     usleep(sendSleep*1000);
    // }

    // void sendCommond(void)
    // {
    //     if(!canDataNum)
    //     {
    //         cout << "数据储存为空" << endl;
    //         exit(1);
    //     }
    //     VCI_Transmit(VCI_USBCAN2, 0, CanID, canData, canDataNum);

    //     // cout << "send " << canDataNum << " data" << endl;
    //     // printf("CAN2 TX ID:0x%08X", canData[0].ID);
	// 	// if(canData[0].ExternFlag==0) printf(" Standard ");
	// 	// if(canData[0].ExternFlag==1) printf(" Extend   ");
	// 	// if(canData[0].RemoteFlag==0) printf(" Data   ");
	// 	// if(canData[0].RemoteFlag==1) printf(" Remote ");
	// 	// printf("DLC:0x%02X",canData[0].DataLen);
	// 	// printf(" data:0x");
	// 	// for(int i = 0; i < canData[0].DataLen; i++)
	// 	// {
	// 	// 	printf(" %02X", canData[0].Data[i]);
	// 	// }
    //     // printf("\n");
        
    //     usleep(sendSleep*1000);
    //     clearCanData();
    // }

    // void clearCanData(void)
    // {
    //     canDataNum = 0;
    // }

    void motorInit(int ID,int mod){
        ID = motorNum[ID-1];
        if(mod == SpeedMod)
        {
            // clearCanData();
            //使能电机(锁死)
            canSend(Hub, 0x600 + ID, 8, 0x2B4060000F000000);
            // sendCommond();

            //设置速度控制模式
            canSend(Hub, 0x600 + ID, 8, 0x2F60600003000000);
            // sendCommond();
        }
        else if(mod = PositionMod)
        {
            // clearCanData();
            //使能电机(锁死)
            // setCommond(0x600 + ID, 8, 0x2B4060000F000000);
            // sendCommond();
            //设置位置控制模式
            canSend(Hub, 0x600 + ID, 8, 0x2F60600001000000);
            // sendCommond();
            //使能电机(锁死)
            // setCommond(0x600 + ID, 8, 0x2B4060001F000000); //绝对位置模式
            canSend(Hub, 0x600 + ID, 8, 0x2B4060000F000000);  //相对位置模式
            // sendCommond();
        }
    }

    

    void motorSetPosition(int ID, double m){
        ID = motorNum[ID-1];
        // clearCanData();
        // setCommond(0x600 + ID, 8, 0x237A600000000000, m/EIGEN_PI*4096/D); //绝对位置模式
        canSend(Hub, 0x600 + ID, 8, 0x237B600000000000, m/EIGEN_PI*4096/D*motorDir[ID - 1]);  //相对位置模式
        // sendCommond();

        //使能电机
        // setCommond(0x600 + ID, 8, 0x2B4060001F000000);
        // sendCommond();
    }

    void stepMotorSetPosition(int ID, double rad){
        ID = motorNum[ID-1];
        rad = rad + stepMotorErr[ID-1];
        // clearCanData();
        canSend(Step, 0x600 + ID, 8, 0x2B70600000000000, (int)(rad*1000)*motorDir[ID-1]);  //绝对位置模式
        // sendCommond();
    }

    void stepMotorReadPosition(){
        int16_t data;

        canClean(Step);
        // usleep(1000);

        canSend(Step, 0x605, 8, 0x4072600000000000);  //绝对位置
        canSend(Step, 0x606, 8, 0x4072600000000000);  //绝对位置
        canSend(Step, 0x607, 8, 0x4072600000000000);  //绝对位置
        canSend(Step, 0x608, 8, 0x4072600000000000);  //绝对位置
        int Len = CanReadDataNum[Step];
        CanReadDataNum[Step] = 0;
        cout<<Len<<endl;
        if(Len){
            for(int l=0;l<Len;l++){
                if(CanDataJudge(CanReadData[Step][l],0x585,0x4B72600000000000)){
                    int16_t data = CanReadData[Step][l].data[4] + CanReadData[Step][l].data[5]*256;
                    stepMotorRealPosition[4] = ((double)data) / 1000 * motorDir[5-1] - stepMotorErr[5-1];
                    // cout<<"stepMotorRealPosition[4] " << stepMotorRealPosition[4] << endl;
                }
                else if(CanDataJudge(CanReadData[Step][l],0x586,0x4B72600000000000)){
                    int16_t data = CanReadData[Step][l].data[4] + CanReadData[Step][l].data[5]*256;
                    stepMotorRealPosition[5] = ((double)data) / 1000 * motorDir[6-1] - stepMotorErr[6-1];
                    // cout<<"stepMotorRealPosition[5] " << stepMotorRealPosition[5] << endl;
                }
                else if(CanDataJudge(CanReadData[Step][l],0x587,0x4B72600000000000)){
                    int16_t data = CanReadData[Step][l].data[4] + CanReadData[Step][l].data[5]*256;
                    stepMotorRealPosition[6] = ((double)data) / 1000 * motorDir[7-1] - stepMotorErr[7-1];
                    // cout<<"stepMotorRealPosition[6] " << stepMotorRealPosition[6] << endl;
                }
                else if(CanDataJudge(CanReadData[Step][l],0x588,0x4B72600000000000)){
                    int16_t data = CanReadData[Step][l].data[4] + CanReadData[Step][l].data[5]*256;
                    stepMotorRealPosition[7] = ((double)data) / 1000 * motorDir[8-1] - stepMotorErr[8-1];
                    // cout<<"stepMotorRealPosition[7] " << stepMotorRealPosition[7] << endl;
                }
            }
        }
        else{
            cout<<"StepMotor read no data"<<endl;
        }

    }

    int CanDataJudge(can_frame canObj,int ID,long data){
        if(canObj.can_id != ID)
            return 0;
        data /= 0x100000000;
        for(int i=3;i>=0;i--){
            if(canObj.data[i] != data % 256)
                return 0;
            data /= 256;
        }
        return 1;
    }

    void hubMotorReadPosition(){
        int32_t data;

        canClean(Hub);

        canSend(Hub, 0x600 + motorNum[1 - 1], 8, 0x40F9601900000000);  //绝对位置
        canSend(Hub, 0x600 + motorNum[2 - 1], 8, 0x40F9601900000000);  //绝对位置
        canSend(Hub, 0x600 + motorNum[3 - 1], 8, 0x40F9601900000000);  //绝对位置
        canSend(Hub, 0x600 + motorNum[4 - 1], 8, 0x40F9601900000000);  //绝对位置
        int Len = CanReadDataNum[Hub];
        CanReadDataNum[Hub] = 0;
        if(Len){
            for(int l=0;l<Len;l++){
                if(CanDataJudge(CanReadData[Hub][l],0x580 + motorNum[1 - 1],0x43F9601900000000)){
                    int32_t data = CanReadData[Hub][l].data[4] + CanReadData[Hub][l].data[5]*256 + CanReadData[Hub][l].data[6]*256*256 + CanReadData[Hub][l].data[7]*256*256*256;
                    hubMotorRealSpeed[0] = ((double)data) / 1000 * motorDir[1 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[0] " << hubMotorRealSpeed[0] << endl;
                }
                else if(CanDataJudge(CanReadData[Hub][l],0x580 + motorNum[2 - 1],0x43F9601900000000)){
                    int32_t data = CanReadData[Hub][l].data[4] + CanReadData[Hub][l].data[5]*256 + CanReadData[Hub][l].data[6]*256*256 + CanReadData[Hub][l].data[7]*256*256*256;
                    hubMotorRealSpeed[1] = ((double)data) / 1000 * motorDir[2 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[1] " << hubMotorRealSpeed[1] << endl;
                }
                else if(CanDataJudge(CanReadData[Hub][l],0x580 + motorNum[3 - 1],0x43F9601900000000)){
                    int32_t data = CanReadData[Hub][l].data[4] + CanReadData[Hub][l].data[5]*256 + CanReadData[Hub][l].data[6]*256*256 + CanReadData[Hub][l].data[7]*256*256*256;
                    hubMotorRealSpeed[2] = ((double)data) / 1000 * motorDir[3 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[2] " << hubMotorRealSpeed[2] << endl;
                }
                else if(CanDataJudge(CanReadData[Hub][l],0x580 + motorNum[4 - 1],0x43F9601900000000)){
                    int32_t data = CanReadData[Hub][l].data[4] + CanReadData[Hub][l].data[5]*256 + CanReadData[Hub][l].data[6]*256*256 + CanReadData[Hub][l].data[7]*256*256*256;
                    hubMotorRealSpeed[3] = ((double)data) / 1000 * motorDir[4 - 1] * EIGEN_PI * D / 60;
                    // cout<<"hubMotorRealSpeed[3] " << hubMotorRealSpeed[3] << endl;
                }
            }
        }
        else{
            cout<<"HubMotor read no data"<<endl;
        }

    }

    // int CanDataJudge(VCI_CAN_OBJ canObj,int ID,long data){
    //     if(canObj.ID != ID)
    //         return 0;
    //     data /= 0x100000000;
    //     for(int i=3;i>=0;i--){
    //         if(canObj.Data[i] != data % 256)
    //             return 0;
    //         data /= 256;
    //     }
    //     return 1;
    // }

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

    void motorSetSpeed(int ID,float m_s){
        int MotorID = motorNum[ID-1];
        canSend(Hub, 0x600 + MotorID, 8, 0x2BF02F0900000000, m_s/EIGEN_PI/D*60*motorDir[ID-1]);
    }

    void motorChangeTrapezoidalVelocityInPosition(int ID,int rpm){
        int MotorID = motorNum[ID-1];

        canSend(Hub, 0x600 + MotorID, 8, 0x2B82600000000000, rpm);

        canSend(Hub, 0x600 + MotorID, 8, 0x2FE52F0001000000);
    }

    void motorChangeUpAccelerationInSpeed(int ID,double rps_s){
        int MotorID = motorNum[ID-1];

        canSend(Hub, 0x600 + MotorID, 8, 0x2383600000000000, rps_s*256*4096/15625);
        
        canSend(Hub, 0x600 + MotorID, 8, 0x2FE52F0001000000);
    }

    void motorChangeDownAccelerationInSpeed(int ID,double rps_s){
        int MotorID = motorNum[ID-1];
        
        canSend(Hub, 0x600 + MotorID, 8, 0x2384600000000000, rps_s*256*4096/15625);
        
        canSend(Hub, 0x600 + MotorID, 8, 0x2FE52F0001000000);
        
    }

    void motorDisEnable(int ID){
        int MotorID = motorNum[ID-1];

        //失能电机(解锁)
        canSend(Hub, 0x600 + MotorID, 8, 0x2B40600006000000);

    }

    void motorClearWarning(int ID){
        int MotorID = motorNum[ID-1];
        
        //失能电机(解锁)
        canSend(Hub, 0x600 + MotorID, 8, 0x2B40600086000000);
        
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

    double deg2rad(double deg){
        double rad;
        rad = deg/180*EIGEN_PI;
        // cout << "rad " << rad << endl; 
        return rad;
    }
};




#endif