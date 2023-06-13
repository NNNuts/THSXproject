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
        switch (ch)             
        {
            // case 'p':
            //     data[0] = Disability;
            //     break;
            // case 'q':
            //     data[0] = Speed;
            //     data[1] = 0.25;
            //     data[2] = 0.25;
            //     data[3] = 0.25;
            //     data[4] = 0.25;
            //     break;
            // case 'a':
            //     data[0] = Speed;
            //     data[1] = -0.25;
            //     data[2] = -0.25;
            //     data[3] = -0.25;
            //     data[4] = -0.25;
            //     break;
            // case 'w':
            //     data[0] = Speed;
            //     data[1] = 0.5;
            //     data[2] = 0.5;
            //     data[3] = 0.5;
            //     data[4] = 0.5;
            //     break;
            // case 's':
            //     data[0] = Speed;
            //     data[1] = -0.5;
            //     data[2] = -0.5;
            //     data[3] = -0.5;
            //     data[4] = -0.5;
            //     break;
            // case 'e':

            //     data[5] = 3.1415926535/2;
            //     data[6] = 3.1415926535/2;
            //     data[7] = 3.1415926535/2;
            //     data[8] = 3.1415926535/2;
            //     break;
            // case 'd':

            //     data[5] = -3.1415926535/2;
            //     data[6] = -3.1415926535/2;
            //     data[7] = -3.1415926535/2;
            //     data[8] = -3.1415926535/2;
            //     break;
            // case 'r':
            //     data[0] = Speed;
            //     data[1] = 0;
            //     data[2] = 0;
            //     data[3] = 0;
            //     data[4] = 0;
            //     break;
            // case 't':
            //     data[0] = Speed;
            //     data[1] = 0.25;
            //     data[2] = 0.25;
            //     data[3] = 0.25;
            //     data[4] = 0.25;
            //     break;
            // case 'g':
            //     data[0] = Speed;
            //     data[1] = -0.25;
            //     data[2] = -0.25;
            //     data[3] = -0.25;
            //     data[4] = -0.25;
            //     break;
            
            // case 'y':
            //     data[5] += 3.1415926535/60;
            //     break;
            // case 'h':
            //     data[5] -= 3.1415926535/60;
            //     break;
            // case 'u':
            //     data[6] += 3.1415926535/60;
            //     break;
            // case 'j':
            //     data[6] -= 3.1415926535/60;
            //     break;
            // case 'i':
            //     data[7] += 3.1415926535/60;
            //     break;
            // case 'k':
            //     data[7] -= 3.1415926535/60;
            //     break;
            // case 'o':
            //     data[8] += 3.1415926535/60;
            //     break;
            // case 'l':
            //     data[8] -= 3.1415926535/60;
            //     break;

            // case 'w':
            //     data[0] = Speed;
            //     data[1] += 0.1;
            //     data[2] += 0.1;
            //     data[3] += 0.1;
            //     data[4] += 0.1;
            //     break;
            // case 's':

            //     data[0] = Speed;
            //     data[1] -= 0.1;
            //     data[2] -= 0.1;
            //     data[3] -= 0.1;
            //     data[4] -= 0.1;
            //     break;

            case 'w':
                stopFlag = 0;
                data[0] = Speed;
                data[1] = 1;
                data[2] = 1;
                data[3] = 1;
                data[4] = 1;
                break;

            case 's':
                stopFlag = 0;
                data[0] = Speed;
                data[1] = -1;
                data[2] = -1;
                data[3] = -1;
                data[4] = -1;
                break;

            case 'z':
                stopFlag = 0;
                data[0] = Speed;
                data[1] = 0.1;
                data[2] = 0.1;
                data[3] = 0.1;
                data[4] = 0.1;
                break;

            case 'c':
                stopFlag = 0;
                data[0] = Speed;
                data[1] = -0.1;
                data[2] = -0.1;
                data[3] = -0.1;
                data[4] = -0.1;
                break;

            
            case 'a':

                stopFlag = 0;
                data[0] = Speed;
                data[5] += 3.1415926535/72;
                data[6] += 3.1415926535/72;
                data[7] += 3.1415926535/72;
                data[8] += 3.1415926535/72;
                break;
            
            case 'd':

                stopFlag = 0;
                data[0] = Speed;
                data[5] -= 3.1415926535/72;
                data[6] -= 3.1415926535/72;
                data[7] -= 3.1415926535/72;
                data[8] -= 3.1415926535/72;

                break;

            case 'q':
                stopFlag = 0;
                data[0] = Speed;
                data[5] = 3.1415926535/4*3;
                data[6] = -3.1415926535/4*3;
                data[7] = 3.1415926535/4;
                data[8] = -3.1415926535/4;

                break;
            
            case 'e':
                stopFlag = 0;
                data[0] = Speed;
                data[5] = -3.1415926535/4;
                data[6] = 3.1415926535/4;
                data[7] = -3.1415926535/4*3;
                data[8] = 3.1415926535/4*3;

                break;

            case ' ':

                data[0] = Speed;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                data[4] = 0;
                break;
            
            case 0:
                stopFlag ++;
                if(stopFlag>stopNum){
                    data[0] = Speed;
                    data[1] = 0;
                    data[2] = 0;
                    data[3] = 0;
                    data[4] = 0;
                }
                break;
            
            case 'r':
                stopFlag = 0;
                data[0] = Speed;
                data[5] = 0;
                data[6] = 0;
                data[7] = 0;
                data[8] = 0;
                break;

            case 'f':
                stopFlag = 0;
                data[0] = Speed;
                data[5] = -3.1415926535/4;
                data[6] = 3.1415926535/4;
                data[7] = 3.1415926535/4;
                data[8] = -3.1415926535/4;
                break;
            
            default:
                stopFlag = 0;
                ROS_INFO("%c!",ch);
                ros::spinOnce();
                loop_rate.sleep();
                // continue;
        }
        
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