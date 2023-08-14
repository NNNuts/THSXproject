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

    int sendSleep = 2000;//50ms
    float D = 0.2; //m
    int motorNum[8] = {1,4,2,3,5,6,7,8};
    int motorDir[8] = {-1,-1,1,1,1,1,-1,-1};
    // double stepMotorErr[8] = {0,0,0,0,0.261799,0.157080,-1.570796,-1.570796};
    double stepMotorErr[8] = {0,0,0,0,0.226893,0.157080,-1.605704,-1.623157};
    double stepMotorRealPosition[8] = {100, 100, 100, 100, 100, 100, 100, 100};
    double hubMotorRealSpeed[8] = {100, 100, 100, 100, 100, 100, 100, 100};

    // void *can0_get_status(void){
    //     pthread_detach(pthread_self());
    //     int err;
    //     while(true){
            
    //         if(CanReadDataNum[0]>=CanReadDataNumMax)
    //             CanReadDataNum[0] = 0;
    //         // cout<<num<<endl;
    //         err = read(can_fd[0], &CanReadData[0][CanReadDataNum[0]], sizeof(CanReadData[0][CanReadDataNum[0]]));
    //         CanReadDataNum[0] ++;
    //    }
    // }

    // static void* staticCan0_get_status(void *param){
    //     HubMotor* pThis = (HubMotor*)param;
    //     pThis->can0_get_status();
    // }

    // void *can1_get_status(void){
    //     pthread_detach(pthread_self());
    //     int err;
    //     while(true){
            
    //         if(CanReadDataNum[1]>=CanReadDataNumMax)
    //             CanReadDataNum[1] = 0;
    //         // cout<<num<<endl;
    //         err = read(can_fd[1], &CanReadData[1][CanReadDataNum[1]], sizeof(CanReadData[1][CanReadDataNum[1]]));
    //         CanReadDataNum[1] ++;
    //         }
    // }

    // static void* staticCan1_get_status(void *param){
    //     HubMotor* pThis = (HubMotor*)param;
    //     pThis->can1_get_status();
    // }

    void *can0_get_status(void);

    static void* staticCan0_get_status(void *param);

    void *can1_get_status(void);

    static void* staticCan1_get_status(void *param);

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


    void canClean(int com){
        CanReadDataNum[com] = 0;
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
        usleep(sendSleep);
    }

    void motorInit(int ID,int mod){
        ID = motorNum[ID-1];
        if(mod == SpeedMod)
        {

            //使能电机(锁死)
            canSend(Hub, 0x600 + ID, 8, 0x2B4060000F000000);

            //设置速度控制模式
            canSend(Hub, 0x600 + ID, 8, 0x2F60600003000000);
        }
        else if(mod = PositionMod)
        {

            //设置位置控制模式
            canSend(Hub, 0x600 + ID, 8, 0x2F60600001000000);
            canSend(Hub, 0x600 + ID, 8, 0x2B4060000F000000);  //相对位置模式
        }
    }

    

    void motorSetPosition(int ID, double m){
        ID = motorNum[ID-1];
        canSend(Hub, 0x600 + ID, 8, 0x237B600000000000, m/EIGEN_PI*4096/D*motorDir[ID - 1]);  //相对位置模式
    }

    void stepMotorSetPosition(int ID, double rad){
        ID = motorNum[ID-1];
        rad = rad + stepMotorErr[ID-1];

        canSend(Step, 0x600 + ID, 8, 0x2B70600000000000, (int)(rad*1000)*motorDir[ID-1]);  //绝对位置模式

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
        // CanReadDataNum[Step] = 0;
        // cout<<Len<<endl;
        if(Len){
            for(int l=0;l<Len;l++){
                // cout<<"Len "<<l<<" ID " << CanReadData[Step][l].can_id << endl;
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
            // cout<<"StepMotor read no data"<<endl;
        }
        canClean(Step);
    }

    int CanDataJudge(can_frame canObj,int ID,long data){
        // cout<<canObj.can_id<<" "<<ID<<endl;
        // int id = canObj.can_id;
        // int err = id - ID;
        // cout<<id<<" "<<err<<endl;
        // if(((int)canObj.can_id ^ ID))
        //     return 0;
        // if(err)
        //     return 0;
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
            // cout<<"HubMotor read no data"<<endl;
        }

    }

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

    double deg2rad(double deg){
        double rad;
        rad = deg/180*EIGEN_PI;
        // cout << "rad " << rad << endl; 
        return rad;
    }
};

void* HubMotor::can0_get_status(void){
    pthread_detach(pthread_self());
    int err;
    struct can_frame frame;
    while(true){
        // CanReadDataNum[0] ++;
        if(CanReadDataNum[0]>=CanReadDataNumMax)
            CanReadDataNum[0] = 0;
        // cout<<num<<endl;
        // err = read(can_fd[0], &CanReadData[0][CanReadDataNum[0]], sizeof(CanReadData[0][CanReadDataNum[0]]));
        err = read(can_fd[0], &frame, sizeof(frame));
        CanReadData[0][CanReadDataNum[0]].can_id = frame.can_id;
        CanReadData[0][CanReadDataNum[0]].can_dlc = frame.can_dlc;
        for(int i=0; i<8; i++)
            CanReadData[0][CanReadDataNum[0]].data[i] = frame.data[i];
        CanReadDataNum[0] ++;
    }
}

void* HubMotor::staticCan0_get_status(void *param){
    HubMotor* pThis = (HubMotor*)param;
    pThis->can0_get_status();
}

void* HubMotor::can1_get_status(void){
    pthread_detach(pthread_self());
    int err;
    struct can_frame frame;
    while(true){
        // CanReadDataNum[1] ++;
        
        if(CanReadDataNum[1]>=CanReadDataNumMax)
            CanReadDataNum[1] = 0;
        // cout<<num<<endl;
        // err = read(can_fd[1], &CanReadData[1][CanReadDataNum[1]], sizeof(CanReadData[1][CanReadDataNum[1]]));
        err = read(can_fd[1], &frame, sizeof(frame));
        CanReadData[1][CanReadDataNum[1]].can_id = frame.can_id;
        CanReadData[1][CanReadDataNum[1]].can_dlc = frame.can_dlc;
        for(int i=0; i<8; i++)
            CanReadData[1][CanReadDataNum[1]].data[i] = frame.data[i];
        // cout<<"read "<<CanReadDataNum[1]<<" ID "<<CanReadData[1][CanReadDataNum[1]].can_id<<endl;
        CanReadDataNum[1] ++;
    }
}

void* HubMotor::staticCan1_get_status(void *param){
    HubMotor* pThis = (HubMotor*)param;
    pThis->can1_get_status();
}


#endif