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

#include <Eigen/Core>
#include <ccd/ccd.h>
#define FCL_EXPORT
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include <iostream>

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
using namespace fcl;

// extern Target_data Tar;
extern TrainComputer TC;
class CAN2USB : public socketCAN                                  
{
private:
    double motorBias[6] = {-9.0/180*EIGEN_PI, 173.0/180*EIGEN_PI, -0.5/180*EIGEN_PI, -26.5/180*EIGEN_PI, -36./180*EIGEN_PI, 0};
    // double motorBias[6] = {-9.0/180*EIGEN_PI, 104.0/180*EIGEN_PI, -2.0/180*EIGEN_PI, 63.0/180*EIGEN_PI, 145.5/180*EIGEN_PI, 0};
    double motorDir[6] = {1, 1, -1, 1, 1, 1};
    double jointsLimit[6][2] = {-180./180*EIGEN_PI, 180./180*EIGEN_PI, 
                                -180./180*EIGEN_PI,  0./180*EIGEN_PI,
                                -180./180*EIGEN_PI, 180./180*EIGEN_PI,
                                -180./180*EIGEN_PI,  30./180*EIGEN_PI,
                                -180./180*EIGEN_PI, 180./180*EIGEN_PI,
                                -180./180*EIGEN_PI, 180./180*EIGEN_PI};
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
        canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + ID, 8, 0x2B4060003F000000);
        // cout<<"send "<<rad2Hex(position)<<endl;

        // canSend(Can, 0x600 + ID, 8, 0x2B4060002F000000);

        // canSend(Can, 0x600 + ID, 8, 0x2B4060003F000000);


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
        canSend(Can, 0x600 + ID, 8, 0x4064600000000000);
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

    double motorReadWarning(int ID)
    {
        // VCI_CAN_OBJ rec[100];
        // clearCanData();
        canClean(Can);
        canSend(Can, 0x600 + ID, 8, 0x400A300000000000);
        if(CanReadDataNum[Can])
        {
            // cout<<"Len "<<CanReadDataNum[Can]<<endl;
            for(int i=0;i<CanReadDataNum[Can];i++)
            {
                // cout<<"Read ID "<<CanReadData[Can][i].can_id<<endl;
                if(ackCheck(ID,0x400A300000000000,CanReadData[Can][i]))
                {
                    // cout<<"ReadCheck "<<ID<<endl;
                    double err = CanReadData[Can][i].data[5] * 256 + CanReadData[Can][i].data[4];
                    // cout<<"err "<<err<<endl;
                    CanReadDataNum[Can] = 0;
                    return err;
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
        // motorSetPosition(1,joint1);
        // motorSetPosition(2,joint2);
        // motorSetPosition(3,joint3);
        // motorSetPosition(4,joint4);
        // motorSetPosition(5,joint5);
        // motorSetPosition(6,joint6);
        
        canSend(Can, 0x600 + 1, 8, 0x237A600000000000 + rad2Hex(joint1));
        canSend(Can, 0x600 + 2, 8, 0x237A600000000000 + rad2Hex(joint2));
        canSend(Can, 0x600 + 3, 8, 0x237A600000000000 + rad2Hex(joint3));
        canSend(Can, 0x600 + 4, 8, 0x237A600000000000 + rad2Hex(joint4));
        canSend(Can, 0x600 + 5, 8, 0x237A600000000000 + rad2Hex(joint5));
        canSend(Can, 0x600 + 6, 8, 0x237A600000000000 + rad2Hex(joint6));
        canSend(Can, 0x600 + 1, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + 2, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + 3, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + 4, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + 5, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + 6, 8, 0x2B4060002F000000);
        canSend(Can, 0x600 + 1, 8, 0x2B4060003F000000);
        canSend(Can, 0x600 + 2, 8, 0x2B4060003F000000);
        canSend(Can, 0x600 + 3, 8, 0x2B4060003F000000);
        canSend(Can, 0x600 + 4, 8, 0x2B4060003F000000);
        canSend(Can, 0x600 + 5, 8, 0x2B4060003F000000);
        canSend(Can, 0x600 + 6, 8, 0x2B4060003F000000);
    }

    long rad2Hex(double rad)
    {
        // cout<<"rad "<<rad<<endl;
        long positionData = rad / EIGEN_PI * 16384 * 100;
        // cout << "Hex " << positionData << endl;
        return num2Hex(positionData);
    }

    long num2Hex(long num)
    {
        long hexData = 0;
        // uint32_t num1 = num;
        // cout<<"num "<<num<<endl;
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
        checkJointLimit(ID, position);
        motorSetPosition(ID,position);
    }

    double robotReadPosition(int ID)
    {
        double position = motorReadPosition(ID);
        position = (position - motorBias[ID - 1]) * motorDir[ID - 1];
        if(position>EIGEN_PI)
            position = position - 2*EIGEN_PI;
        if(position<-EIGEN_PI)
            position = position + 2*EIGEN_PI;
        return position;
    }

    void robotSetPositionAll(double rad[6])
    {
        if(robotIsSelfCollide(rad))
        {
            cout<<"发生碰撞 "<<robotIsSelfCollide(rad)<<endl;
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

    void checkJointLimit(int ID, double joint)
    {
        // for(int i=0;i<6;i++)
        // {
        //     if(joints[i]<jointsLimit[i][0])
        //     {
        //         cout<<"joint"<<i<<" "<<joints[i]<<" too small"<<endl;
        //         exit(0);
        //     }
        //     if(joints[i]>jointsLimit[i][1])
        //     {
        //         cout<<"joint"<<i<<" "<<joints[i]<<" too big"<<endl;
        //         exit(0);
        //     }
        // }
        if(joint<jointsLimit[ID-1][0])
        {
            cout<<"joint"<<ID<<" "<<joint<<" too small"<<endl;
            exit(0);
        }
        if(joint>jointsLimit[ID-1][1])
        {
            cout<<"joint"<<ID<<" "<<joint<<" too big"<<endl;
            exit(0);
        }
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
        double err = 10;
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




#endif