#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <termios.h>

float data[9] = {0,0,0,0,0,0,0,0,0};

enum
{
    Disability,
    Speed,
    Position
};
// 
char getch();
int stopFlag = 0;
int stopNum = 5;
double stopSpeed = 0.02;
double speed = 0;
double runspeed = 0.5;
double turnSpeed = 280;

int stopMod = false;
int main(int argc, char **argv)
{
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
        // initscr();
        // ch = getchar(); //需enter，无延时
        ch = getch(); //无需enter，但有少量延时
        // endwin();
        double t;
        switch (ch)             
        {
            //按住前进
            case 'w':
                stopFlag = 0;
                data[0] = Speed;
                // speed = runspeed;
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
                // speed = 1;
                break;

            //按住后退
            case 's':
                stopFlag = 0;
                data[0] = Speed;
                // speed = -runspeed;
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
                // speed = -1;
                break;

            //按一下前进
            case 't':
                stopFlag = 0;
                speed = runspeed;
                // speed = 1;
                stopMod = 1;
                break;

            //按一下后退
            case 'g':
                stopFlag = 0;
                data[0] = Speed;
                speed = -runspeed;
                // speed = -1;
                stopMod = 1;
                break;

            //加速
            case 'z':
                stopFlag = 0;
                runspeed += 0.1;
                if(runspeed > 1.5)
                    runspeed = 1.5;
                break;

            //减速
            case 'c':
                stopFlag = 0;
                runspeed -= 0.1;
                if(runspeed < 0.1)
                    runspeed = 0.1;
                break;

            //按住左旋转
            case 'a':
                stopFlag = 0;
                data[0] = Speed;
                data[5] += 3.1415926535/turnSpeed;
                data[6] += 3.1415926535/turnSpeed;
                data[7] += 3.1415926535/turnSpeed;
                data[8] += 3.1415926535/turnSpeed;
                break;

            //按住右旋转
            case 'd':
                stopFlag = 0;
                data[0] = Speed;
                data[5] -= 3.1415926535/turnSpeed;
                data[6] -= 3.1415926535/turnSpeed;
                data[7] -= 3.1415926535/turnSpeed;
                data[8] -= 3.1415926535/turnSpeed;
                break;

            case ' ':
                data[0] = Speed;
                speed = 0;
                stopMod = 0;
                break;
            
            case 0:
                if(stopMod == 0)
                    stopFlag ++;
                if(stopFlag>stopNum){
                    data[0] = Speed;
                    // speed = 0;
                    if(speed>0)
                    {
                        speed -= stopSpeed;
                        if(speed<0)
                            speed = 0;
                    }
                    if(speed<0)
                    {
                        speed += stopSpeed;
                        if(speed>0)
                            speed = 0;
                    }
                }
                break;
            
            //轮胎归位
            case 'r':
                stopFlag = 0;
                data[0] = Speed;
                // angle = 0;
                data[5] = 0;
                data[6] = 0;
                data[7] = 0;
                data[8] = 0;
                break;
            
            default:
                stopFlag = 0;
                // ROS_INFO("%c!",ch);
                ros::spinOnce();
                loop_rate.sleep();
                // continue;
        }
        
        data[1] = speed;
        data[2] = speed;
        data[3] = speed;
        data[4] = speed;

        msg.data.push_back(data[0]);
        msg.data.push_back(data[1]);
        msg.data.push_back(data[2]);
        msg.data.push_back(data[3]);
        msg.data.push_back(data[4]);
        msg.data.push_back(data[5]);
        msg.data.push_back(data[6]);
        msg.data.push_back(data[7]);
        msg.data.push_back(data[8]);
        data_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

char getch()
{
    char buf = 0;
    struct termios old = {0};
    if(tcgetattr(0,&old) < 0){perror("tcgetattr error");}
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 0;
    old.c_cc[VTIME] = 0;
    if(tcsetattr(0,TCSANOW,&old)<0){
        perror("tcsetattr error");
    }    
    if(read(0,&buf,1)<0){
        perror("read error");
    }
    
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    
    if(tcsetattr(0,TCSADRAIN,&old)<0)
    {
        perror("tcsetattr error2");
    }
    return (buf);
}

