#include <iostream>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include "controller.hpp"
#include "kinematics.hpp"
#include <fstream>
#include <Eigen/Dense>
#include  <ctime>
#include "CAN2USB.hpp"
#include <termios.h>

#include <Eigen/Core>
#include <ccd/ccd.h>
#define FCL_EXPORT
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include <iostream>
#include <Eigen/Dense>

// 读取按键
#include <linux/input.h>
#include <fstream>



#define msleep(ms)  usleep((ms)*1000)

using namespace fcl;
using namespace std; 
using namespace std::chrono;
using namespace Eigen;

Target_data Tar;
TrainComputer TC;
CAN2USB rob;
void MoveL(double X, double Y, double Z, double RX, double RY, double RZ, double delay_s);
void MoveJ(double X, double Y, double Z, double RX, double RY, double RZ, double delay_s);
char getch();
void KeyBoardControl(double X, double Y, double Z, double RX, double RY, double RZ);


void trackRunning(void)
{
    while(true)
    {
        this_thread::sleep_for(std::chrono::microseconds(10000));
        if(Tar.trackMod)
        {
            Tar.MoveL_Trackmod_LadderShaped();
            ofstream OutFile("Test.txt",ios::app); //利用构造函数创建txt文本，并且打开该文本
            OutFile << TC.theta_now[0]<< " " << TC.theta_now[1]<< " "  << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << Tar.present_V[0] << " " << endl; 
            //OutFile << "This is a Test12!" << endl; //把字符串内容"This is a Test!"，写入Test.txt文件
            OutFile.close(); //关闭Test.txt文件
            // exit(0);
        }
    }
}

/*  
 *作用: 找到键盘设备对应的事件文件  例如 event7
 *     linux 插入键盘设备，每一个设备会有一个事件编号,但是目前这个ls -l排序是按照设备名称排，所以
 *     最上面的键盘设备并不会是最新的设备,因此 此函数在键盘设备多的时候 会获取一个不确定的键盘事件文件
        usb-026d_0002-event-if01 -> ../event12
        usb-026d_0002-event-kbd -> ../event6
        usb-413c_Dell_KB216_Wired_Keyboard-event-if01 -> ../event14
        usb-413c_Dell_KB216_Wired_Keyboard-event-kbd -> ../event13     这是最新的键盘设备,但是没有排到最上面
        usb-Logitech_USB_Receiver-if01-event-mouse -> ../event2
        usb-Logitech_USB_Receiver-if01-mouse -> ../mouse0
        usb-SONiX_USB_Keyboard-event-if01 -> ../event4
        usb-SONiX_USB_Keyboard-event-kbd -> ../event3
 */
 
string FindKdbEvet()
{
    FILE *stream;
    FILE *wstream;
    char buf[20];
    memset( buf, '\0', sizeof(buf) );//初始化buf,以免后面写如乱码到文件中
    //将命令的输出 通过管道读取（“r”参数）到FILE* stream
    stream = popen("cd /dev/input/by-id ; ls -l | grep \"kbd\" | head -1 | cut -d \"/\" -f 2", "r"); 
    fread( buf, sizeof(char), sizeof(buf), stream); //将刚刚FILE* stream的数据流读取到buf中
    pclose(stream);
    string str = buf;
    // cout<<"str = "<<str<<endl;
    return str;
}
  
/*  
 *作用: 打开事件文件  
 *输入：  _infile 对应的文件流变量
 *返回值: int 返回0 为打开成功  返回-1 为打开失败
 */
int OpenEventFile(ifstream & _infile)
{
    string kdbevet = FindKdbEvet();   //获取键盘事件例如 event7 
    int isize = kdbevet.size();
    if(isize > 0 && kdbevet[isize-1]) //这里需要去除结尾的换行符号 ascii码是10
    {
        kdbevet = kdbevet.substr(0,kdbevet.size()-1); 
    }
    string shCommand = "/dev/input/" + kdbevet;
 
    _infile.open(shCommand.c_str(),ios_base::in);
    if(!_infile.is_open())
    {
        cout <<"open Keyboard device error, error code = <<" << errno << "!" <<endl;
        return -1;
    }
    return 0;
}
 
 
 

float Data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int key[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
enum
{
    Q,
    A,
    W,
    S,
    E,
    D,
    R,
    F,
    T,
    G,
    Y,
    H,
    Space,
    Z,
    C
};


int main(int argc, char* argv[])
{
    
    cout << "系统启动" << endl;

    rob.canInit();

    // ifstream infile;
    
    
    
    // VCI_CAN_OBJ rec[3000];
    // cout<<VCI_Receive(VCI_USBCAN2,0,0,rec,3000,100)<<endl;
    // usleep(1000000);
    // cout<<VCI_Receive(VCI_USBCAN2,0,0,rec,3000,100)<<endl;
    // exit(0);

    rob.robotReset();
    rob.robotInit();


    //-------------------------
    // cout<<rob.motorReadWarning(4)<<endl;

    // rob.robotSetPosition(1,rob.deg2rad(0));
    // rob.robotSetPosition(2,rob.deg2rad(0));
    // rob.robotSetPosition(3,rob.deg2rad(0));
    // rob.robotSetPosition(4,rob.deg2rad(0));
    // rob.robotSetPosition(5,rob.deg2rad(0));
    // rob.robotSetPosition(6,rob.deg2rad(0));
    // while(true)
    // {
    // //     rob.robotSetPosition(1,rob.deg2rad(5));
    // //     rob.robotSetPosition(2,rob.deg2rad(-5));
    // //     rob.robotSetPosition(3,rob.deg2rad(5));
    //     rob.robotSetPosition(4,rob.deg2rad(-5));
    // //     rob.robotSetPosition(5,rob.deg2rad(5));
    // //     rob.robotSetPosition(6,rob.deg2rad(5));
    //     usleep(1000000);
    //     rob.robotReadPositionAll(TC.theta_now);
    //     cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;
    //     for(int i = 0;i < 6;i++)
    //     {
    //         if(TC.theta_now[i] > 10 || TC.theta_now[i] < -10)
    //         {
    //             cout<< i+1 << "号电机通讯失败" << endl;
    //             // exit(0);
    //         }
    //     }

    // //     rob.robotSetPosition(1,rob.deg2rad(0));
    // //     rob.robotSetPosition(2,rob.deg2rad(-0));
    // //     rob.robotSetPosition(3,rob.deg2rad(0));
    //     rob.robotSetPosition(4,rob.deg2rad(-0));
    // //     rob.robotSetPosition(5,rob.deg2rad(0));
    // //     rob.robotSetPosition(6,rob.deg2rad(0));
    //     usleep(1000000);
    //     rob.robotReadPositionAll(TC.theta_now);
    //     cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;
    //     for(int i = 0;i < 6;i++)
    //     {
    //         if(TC.theta_now[i] > 10 || TC.theta_now[i] < -10)
    //         {
    //             cout<< i+1 << "号电机通讯失败" << endl;
    //             // exit(0);
    //         }
    //     }
    // }


    // exit(0);
    //-------------------------


    // ofstream OutFile("Test.txt",ios::out) ; //利用构造函数创建txt文本，并且打开该文本
    // OutFile.clear();
    // OutFile.close(); //关闭Test.txt文件

    rob.robotReadPositionAll(TC.theta_now);

    // TC.theta_now[0] = 0;
    cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;
    for(int i = 0;i < 6;i++)
    {
        if(TC.theta_now[i] > 10 || TC.theta_now[i] < -10)
        {
            cout<< i+1 << "号电机通讯失败" << endl;
            exit(0);
        }
    }
    // exit(0);

    TC.theta_now[0] = 0; 
    TC.theta_now[1] = rob.deg2rad(-60);
    TC.theta_now[2] = rob.deg2rad(-60);
    TC.theta_now[3] = rob.deg2rad(-60);
    TC.theta_now[4] = 0;
    TC.theta_now[5] = 0;
    // TC.theta_now[0] = 0; 
    // TC.theta_now[1] = rob.deg2rad(0);
    // TC.theta_now[2] = rob.deg2rad(0);
    // TC.theta_now[3] = rob.deg2rad(0);
    // TC.theta_now[4] = 0;
    // TC.theta_now[5] = 0;
    rob.robotSetPositionAll(TC.theta_now);
    // exit(0);
    msleep(5000);
    rob.robotReadPositionAll(TC.theta_now);

    
    Matrix4d mat = TC.kinematics(TC.theta_now);
    // cout << mat << endl;
    Rpy rpy = TC.Matrix2Rpy(mat.block<3, 3>(0, 0));
    // cout << rpy << endl << endl;
    // MoveJ(200, 600, 800, -90, 0, 0, 5);
    // double parameter[6] = {200, 500, 800, rob.deg2rad(90), rob.deg2rad(0), rob.deg2rad(0)};

    // double Joint[6];
    // Tar.Pose_ComputerAndJudge_MoveJ(parameter,TC.theta_now, Joint);
    
    // if(Joint[0] == 10000)
    // {
    //     cout << "当前位置不可达"  << endl;
    // }
    
    // TC.theta_now[0] = Joint[0];
    // TC.theta_now[1] = Joint[1];
    // TC.theta_now[2] = Joint[2];
    // TC.theta_now[3] = Joint[3];
    // TC.theta_now[4] = Joint[4];
    // TC.theta_now[5] = Joint[5];
    // rob.robotSetPositionAll(TC.theta_now);
    // cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;
    // usleep(5000000);
    // rob.robotReadPositionAll(TC.theta_now);
    // // TC.theta_now[0] = 0;
    // cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;
    // for(int i = 0;i < 6;i++)
    // {
    //     if(TC.theta_now[i] > 10 || TC.theta_now[i] < -10)
    //     {
    //         cout<< i+1 << "号电机通讯失败" << endl;
    //         // exit(0);
    //     }
    // }
    // // cout << endl << X << " " << Y << " " << Z << " " << RX << " " << RY << " " << RZ << " " << endl;
    // mat = TC.kinematics(TC.theta_now);
    // cout << mat << endl;
    // rpy = TC.Matrix2Rpy(mat.block<3, 3>(0, 0));
    // cout << rpy << endl << endl;
    // exit(0);
    // TC.theta_now[4] += 2*EIGEN_PI;
    KeyBoardControl(200, 600, 800, -90, 0, 0);
    MoveL(-200, -500, 800, 90, 0, 0, 1);
    // MoveL(-200, -500, 800, 1);
    // MoveJ(-200, -500, 1200, 1);
    // exit(0);
    // MoveL(200, -500, 1100, 1);
    // MoveL(400, -500, 1100, 1);
    // MoveL(400, -300, 1100, 1);
    // MoveL(200, -300, 1100, 1);
    // while(true)
    // {
    //     MoveL(-200, -500, 800, 1);
    //     MoveL(-200,  500, 800, 1);
    //     MoveL(-400,  500, 800, 1);
    //     MoveL(-400, -500, 800, 1);
    //     MoveL(-600, -500, 800, 1);
    //     MoveL(-600,  500, 800, 1);
    //     MoveL(-800,  500, 800, 1);
    //     MoveL(-800, -500, 800, 1);
    // }
    // while(true)
    // {
    //     MoveJ(200, -500, 900, 1);
    //     MoveJ(400, -500, 900, 1);
    //     MoveJ(400, -300, 900, 1);
    //     MoveJ(200, -300, 900, 1);
    // }

    // Tar.PathInit();
    // cout << "Init ok" << endl;
    // Tar.SetPathPoint(200, -500, 900, 0, rob.deg2rad(-90), 0,rob.deg2rad(180));
    // cout << "Path Set ok" << endl;
    // clock_t start, end;
    // start = clock();

    // Tar.Move_End_LadderShaped();
    // Tar.Move_MoveJ_CubicPolynomial();

    // end = clock();
    // double endtime = (double)(end - start) / CLOCKS_PER_SEC;
    // cout << "Total time: " << endtime << "s" << endl;		//s为单位

    return 0;
} 

//输入：X，Y，Z 为目标点位置（mm）RX, RY, RZ 为rpy转角
//输出：无
//说明：MoveL方式移动
//特别：
void MoveL(double X, double Y, double Z, double RX, double RY, double RZ, double delay_s)
{
    double pos[2];
    // TC.kinematics(TC.theta_now);
    Tar.PathInit();
    Tar.SetPathPoint(X, Y, Z, 0, rob.deg2rad(RX), rob.deg2rad(RY), rob.deg2rad(RZ));
    Tar.Move_End_LadderShaped();
    usleep(delay_s * 1000000);
}

//输入：X，Y，Z 为目标点位置（mm）
//输出：无
//说明：MoveJ方式移动
//特别：
void MoveJ(double X, double Y, double Z, double RX, double RY, double RZ, double delay_s)
{
    double pos[2];
    // TC.kinematics(TC.theta_now);
    Tar.PathInit();
    Tar.SetPathPoint(X, Y, Z, 0, rob.deg2rad(RX), rob.deg2rad(RY), rob.deg2rad(RZ));
    Tar.Move_MoveJ_CubicPolynomial();
    usleep(delay_s * 1000000);
}

void KeyBoardControl(double x, double y, double z, double RX, double RY, double RZ)
{
    // MoveL(X, Y, Z, RX, RY, RZ, 3);
    // char ch;
    MoveJ(x, y, z, RX, RY, RZ, 2);
    
    int XYZSpeed = 10;
    // Matrix4d mat;
    // Rpy rpy;
    double num[6];
    struct input_event t;  
    ifstream infile;
    if(OpenEventFile(infile) == -1)
    {
        cout << "OpenEventFile error" << endl;
        exit(0);
    }

    while(true)
    {
        if(infile.read((char *)&t,sizeof(t))){
            if(t.type == EV_KEY){
                // cout<<"key "<<t.code<<" "<<t.value<<endl;
                if(t.code == KEY_W){
                    if(t.value == 1)
                        key[W] = 1;
                    else if(t.value == 0)
                        key[W] = 0;
                }
                else if(t.code == KEY_A){
                    if(t.value == 1)
                        key[A] = 1;
                    else if(t.value == 0)
                        key[A] = 0;
                }
                else if(t.code == KEY_S){
                    if(t.value == 1)
                        key[S] = 1;
                    else if(t.value == 0)
                        key[S] = 0;
                }
                else if(t.code == KEY_D){
                    if(t.value == 1)
                        key[D] = 1;
                    else if(t.value == 0)
                        key[D] = 0;
                }
                else if(t.code == KEY_SPACE){
                    if(t.value == 1)
                        key[Space] = 1;
                    else if(t.value == 0)
                        key[Space] = 0;
                }
                else if(t.code == KEY_R){
                    if(t.value == 1)
                        key[R] = 1;
                    else if(t.value == 0)
                        key[R] = 0;
                }
                else if(t.code == KEY_Z){
                    if(t.value == 1)
                        key[Z] = 1;
                    else if(t.value == 0)
                        key[Z] = 0;
                }
                else if(t.code == KEY_C){
                    if(t.value == 1)
                        key[C] = 1;
                    else if(t.value == 0)
                        key[C] = 0;
                }
            
                if(key[Q] == 1){
                    x += XYZSpeed;
                }
                else if (key[A] == 1){
                    x -= XYZSpeed;
                }if(key[W] == 1){
                    y += XYZSpeed;
                }
                else if (key[S] == 1){
                    y -= XYZSpeed;
                }
                if(key[E] == 1){
                    z += XYZSpeed;
                }
                else if (key[D] == 1){
                    z -= XYZSpeed;
                }
                if(key[R] == 1){
                    RX += 3;
                }
                else if (key[F] == 1){
                    RX -= 3;
                }
                if(key[T] == 1){
                    RY += 3;
                }
                else if (key[G] == 1){
                    RY -= 3;
                }
                if(key[Y] == 1){
                    RZ += 3;
                }
                else if (key[H] == 1){
                    RZ -= 3;
                }
                if(key[Z] == 1){
                    XYZSpeed += 1;
                    if(XYZSpeed>10){
                        XYZSpeed = 10;
                    }
                }
                else if (key[C] == 1){
                    XYZSpeed -= 1;
                    if(XYZSpeed<1){
                        XYZSpeed = 1;
                    }
                }
                if(key[Space] == 1){
                    rob.robotReadPositionAll(num);
                    // TC.theta_now[0] = 0;
                    cout << endl;
                    cout << num[0] << " " << num[1] << " " << num[2] << " " << num[3] << " " << num[4] << " " << num[5] << " " << endl;
                    for(int i = 0;i < 6;i++){
                        if(num[i] > 10 || num[i] < -10){
                            cout<< i+1 << "号电机通讯失败" << endl;
                            // exit(0);
                        }
                    }
                    cout <<  x << " " << y << " " << z << " " << RX << " " << RY << " " << RZ << " " << endl;
                    cout << TC.theta_now[0] << " " << TC.theta_now[1] << " " << TC.theta_now[2] << " " << TC.theta_now[3] << " " << TC.theta_now[4] << " " << TC.theta_now[5] << " " << endl;

                }
            }    
        }
        
        double parameter[6] = {x, y, z, rob.deg2rad(RX), rob.deg2rad(RY), rob.deg2rad(RZ)};

        double Joint[6];
        Tar.Pose_ComputerAndJudge_MoveJ(parameter,TC.theta_now, Joint);
        
        if(Joint[0] == 10000)
        {
            cout << "当前位置不可达"  << endl;
        }
        
        TC.theta_now[0] = Joint[0];
        TC.theta_now[1] = Joint[1];
        TC.theta_now[2] = Joint[2];
        TC.theta_now[3] = Joint[3];
        TC.theta_now[4] = Joint[4];
        TC.theta_now[5] = Joint[5];
        rob.robotSetPositionAll(TC.theta_now);
        // Matrix4d mat = TC.kinematics(TC.theta_now);
        // // cout << mat << endl;
        // Rpy rpy = TC.Matrix2Rpy(mat.block<3, 3>(0, 0));
        // cout << rpy << endl << endl;
    }
}

// char getch()
// {
//     char buf = 0;
//     struct termios old = {0};
//     if(tcgetattr(0,&old) < 0){perror("tcgetattr error");}
//     old.c_lflag &= ~ICANON;
//     old.c_lflag &= ~ECHO;
//     old.c_cc[VMIN] = 0;
//     old.c_cc[VTIME] = 0;
//     if(tcsetattr(0,TCSANOW,&old)<0){
//         perror("tcsetattr error");
//     }    
//     if(read(0,&buf,1)<0){
//         perror("read error");
//     }
    
//     old.c_lflag |= ICANON;
//     old.c_lflag |= ECHO;
    
//     if(tcsetattr(0,TCSADRAIN,&old)<0)
//     {
//         perror("tcsetattr error2");
//     }
//     return (buf);
// }