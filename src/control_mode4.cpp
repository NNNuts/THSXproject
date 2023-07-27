#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
// #include <termios.h>

#include <stdio.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <errno.h>
#include <fstream>
 
using namespace std;
 
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
 
 
 

float Data[9] = {0,0,0,0,0,0,0,0,0};
int key[10] = {0,0,0,0,0,0,0,0,0,0};
enum
{
    W,
    A,
    S,
    D,
    Z,
    C,
    Space,
    R
};

enum
{
    Disability,
    Speed,
    Position
};
// 
char getch();
// int stopFlag = 0;
// int stopNum = 5;
double stopSpeed = 0.02;
double speed = 0;
double runspeed = 0.1;
double speed_left = 1;
double speed_right = 1;
double angle = 0;
double turnSpeed = 150;
// int stopMod = false;
int main(int argc, char **argv)
{
    struct input_event t;  
    ifstream infile;
    
    if(OpenEventFile(infile) == -1)
    {
        return 0;
    }
    ros::init(argc, argv, "KeyBoard");
    ros::NodeHandle nh;
 
    ros::Publisher data_pub = nh.advertise<std_msgs::Float32MultiArray>("AgvControl", 1000);
    setlocale(LC_ALL, "");
    ROS_INFO("Warning!");
//    ROS_INFO("如需使用位置模式，需将四轮离地进行初始化");
    ROS_INFO("键盘控制启动");
    ros::Rate loop_rate(100);
    char ch;
    while (ros::ok())
    {
        std_msgs::Float32MultiArray msg;
        double T;
        if(infile.read((char *)&t,sizeof(t)))
        {
            if(t.type == EV_KEY)
            {
                // cout<<"key "<<t.code<<" "<<t.value<<endl;
                if(t.code == KEY_W)
                {
                    if(t.value == 1)
                        key[W] = 1;
                    else if(t.value == 0)
                        key[W] = 0;
                }
                else if(t.code == KEY_A)
                {
                    if(t.value == 1)
                        key[A] = 1;
                    else if(t.value == 0)
                        key[A] = 0;
                }
                else if(t.code == KEY_S)
                {
                    if(t.value == 1)
                        key[S] = 1;
                    else if(t.value == 0)
                        key[S] = 0;
                }
                else if(t.code == KEY_D)
                {
                    if(t.value == 1)
                        key[D] = 1;
                    else if(t.value == 0)
                        key[D] = 0;
                }
                else if(t.code == KEY_SPACE)
                {
                    if(t.value == 1)
                        key[Space] = 1;
                    else if(t.value == 0)
                        key[Space] = 0;
                }
                else if(t.code == KEY_R)
                {
                    if(t.value == 1)
                        key[R] = 1;
                    else if(t.value == 0)
                        key[R] = 0;
                }
                else if(t.code == KEY_Z)
                {
                    if(t.value == 1)
                        key[Z] = 1;
                    else if(t.value == 0)
                        key[Z] = 0;
                }
                else if(t.code == KEY_C)
                {
                    if(t.value == 1)
                        key[C] = 1;
                    else if(t.value == 0)
                        key[C] = 0;
                }


                if(key[A] == 1 && key[D] == 0)
                {
                    Data[0] = Speed;
                    angle += 3.1415926535/turnSpeed;
                    
                    if(angle > 0)
                    {
                        T = 245/tan(angle);
                        speed_left = (T-235)/T;
                        speed_right = (T+235)/T;
                        Data[5] = atan2(245,T-235);
                        Data[6] = -atan2(245,T-235);
                        Data[7] = atan2(245,T+235);
                        Data[8] = -atan2(245,T+235);
                    }
                    else if(angle < 0)
                    {
                        T = 245/tan(-angle);
                        speed_left = (T+235)/T;
                        speed_right = (T-235)/T;
                        Data[5] = -atan2(245,T+235);
                        Data[6] = atan2(245,T+235);
                        Data[7] = -atan2(245,T-235);
                        Data[8] = atan2(245,T-235);
                    }
                    else
                    {
                        speed_left = 1;
                        speed_right = 1;
                        Data[5] = 0;
                        Data[6] = 0;
                        Data[7] = 0;
                        Data[8] = 0;
                    }
                }
                else if(key[A] == 0 && key[D] == 1)
                {
                    Data[0] = Speed;
                    angle -= 3.1415926535/turnSpeed;
                    if(angle > 0)
                    {
                        T = 245/tan(angle);
                        speed_left = (T-235)/T;
                        speed_right = (T+235)/T;
                        Data[5] = atan2(245,T-235);
                        Data[6] = -atan2(245,T-235);
                        Data[7] = atan2(245,T+235);
                        Data[8] = -atan2(245,T+235);
                    }
                    else if(angle < 0)
                    {
                        T = 245/tan(-angle);
                        speed_left = (T+235)/T;
                        speed_right = (T-235)/T;
                        Data[5] = -atan2(245,T+235);
                        Data[6] = atan2(245,T+235);
                        Data[7] = -atan2(245,T-235);
                        Data[8] = atan2(245,T-235);
                    }
                    else
                    {
                        speed_left = 1;
                        speed_right = 1;
                        Data[5] = 0;
                        Data[6] = 0;
                        Data[7] = 0;
                        Data[8] = 0;
                    }
                }
                if(key[W] == 1 && key[S] == 0)
                {
                    Data[0] = Speed;
                    if(speed>runspeed)
                    {
                        speed -= stopSpeed;
                        if(speed<runspeed)
                            speed = runspeed;
                    }
                    else if(speed<runspeed)
                    {
                        speed += stopSpeed;
                        if(speed>runspeed)
                            speed = runspeed;
                    }
                    else
                        speed = runspeed;
                }
                else if(key[W] == 0 && key[S] == 1)
                {
                    Data[0] = Speed;
                    if(speed>-runspeed)
                    {
                        speed -= stopSpeed;
                        if(speed<-runspeed)
                            speed = -runspeed;
                    }
                    else if(speed<-runspeed)
                    {
                        speed += stopSpeed;
                        if(speed>-runspeed)
                            speed = -runspeed;
                    }
                    else
                        speed = -runspeed;
                }
                else
                {
                    Data[0] = Speed;
                    speed = 0;
                }
                if(key[Z] == 1 && key[C] == 0)
                {
                    runspeed += 0.1;
                    if(runspeed > 1.5)
                        runspeed = 1.5;
                }
                else if(key[Z] == 0 && key[C] == 1)
                {
                    runspeed -= 0.1;
                    if(runspeed < 0)
                        runspeed = 0;
                }
                if(key[Space] == 1)
                {
                    Data[0] = Speed;
                    speed = 0;
                }
                if(key[R] == 1)
                {
                    Data[0] = Speed;
                    angle = 0;
                    speed_left = 1;
                    speed_right = 1;
                    Data[5] = 0;
                    Data[6] = 0;
                    Data[7] = 0;
                    Data[8] = 0;
                }
                
            }
        }

        Data[1] = speed * speed_left;
        Data[2] = speed * speed_left;
        Data[3] = speed * speed_right;
        Data[4] = speed * speed_right;

        msg.data.push_back(Data[0]);
        msg.data.push_back(Data[1]);
        msg.data.push_back(Data[2]);
        msg.data.push_back(Data[3]);
        msg.data.push_back(Data[4]);
        msg.data.push_back(Data[5]);
        msg.data.push_back(Data[6]);
        msg.data.push_back(Data[7]);
        msg.data.push_back(Data[8]);
        data_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
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

