#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

float data[5] = {0,0,0,0,0};

enum
{
    Disability,
    Speed,
    Position
};
int mod = Disability;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HubControl");
    ros::NodeHandle nh;
 
    ros::Publisher data_pub = nh.advertise<std_msgs::Float32MultiArray>("HubControl", 1000);
    setlocale(LC_ALL, "");
    ROS_INFO("Warning!");
    ROS_INFO("如需使用位置模式，需将四轮离地进行初始化");
    ROS_INFO("键盘控制启动");
    ros::Rate loop_rate(10);
    uint ch;
    while (ros::ok())
    {
        std_msgs::Float32MultiArray msg;
        // initscr();
        ch = getchar();
        // endwin();
        switch (ch)             
        {
            case 'p':
                data[0] = Disability;
                break;
            case 'q':
                data[0] = Speed;
                data[1] = 0.25;
                data[2] = 0.25;
                data[3] = 0.25;
                data[4] = 0.25;
                break;
            case 'a':
                data[0] = Speed;
                data[1] = -0.25;
                data[2] = -0.25;
                data[3] = -0.25;
                data[4] = -0.25;
                break;
            case 'w':
                data[0] = Speed;
                data[1] = 0.5;
                data[2] = 0.5;
                data[3] = 0.5;
                data[4] = 0.5;
                break;
            case 's':
                data[0] = Speed;
                data[1] = -0.5;
                data[2] = -0.5;
                data[3] = -0.5;
                data[4] = -0.5;
                break;
            case 'e':
                data[0] = Speed;
                data[1] = 1;
                data[2] = 1;
                data[3] = 1;
                data[4] = 1;
                break;
            case 'd':
                data[0] = Speed;
                data[1] = -1;
                data[2] = -1;
                data[3] = -1;
                data[4] = -1;
                break;
            case 'r':
                data[0] = Speed;
                data[1] = 0;
                data[2] = 0;
                data[3] = 0;
                data[4] = 0;
                break;
            case 'y':
                data[0] = Position;
                data[1] = 0.25;
                data[2] = 0.25;
                data[3] = 0.25;
                data[4] = 0.25;
                break;
            case 'h':
                data[0] = Position;
                data[1] = -0.25;
                data[2] = -0.25;
                data[3] = -0.25;
                data[4] = -0.25;
                break;
            case 'u':
                data[0] = Position;
                data[1] = 0.5;
                data[2] = 0.5;
                data[3] = 0.5;
                data[4] = 0.5;
                break;
            case 'j':
                data[0] = Position;
                data[1] = -0.5;
                data[2] = -0.5;
                data[3] = -0.5;
                data[4] = -0.5;
                break;
            case 'i':
                data[0] = Position;
                data[1] = 1;
                data[2] = 1;
                data[3] = 1;
                data[4] = 1;
                break;
            case 'k':
                data[0] = Position;
                data[1] = -1;
                data[2] = -1;
                data[3] = -1;
                data[4] = -1;
                break;
            case 'o':
                data[0] = Position;
                data[1] = 2;
                data[2] = 2;
                data[3] = 2;
                data[4] = 2;
                break;
            case 'l':
                data[0] = Position;
                data[1] = -2;
                data[2] = -2;
                data[3] = -2;
                data[4] = -2;
                break;
        }
        msg.data.push_back(data[0]);
        msg.data.push_back(data[1]);
        msg.data.push_back(data[2]);
        msg.data.push_back(data[3]);
        msg.data.push_back(data[4]);
        data_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
