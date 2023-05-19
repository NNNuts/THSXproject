#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
// #include <curses.h>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// using namespace cv;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "HubControl");
    ros::NodeHandle nh;
 
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("HubControl", 1000);
 
    ros::Rate loop_rate(10);
    // initscr();
    uint ch;
    while (ros::ok())
    {
        std_msgs::Float32MultiArray msg;
        // initscr();
        ch = getchar();
        // endwin();
        switch (ch)             
        {
            case 'w':
                msg.data.push_back(1);//自己写的，可行
                msg.data.push_back(1);
                msg.data.push_back(1);
                msg.data.push_back(1);
                chatter_pub.publish(msg);
                break;
            case 'a':
                msg.data.push_back(-1);//自己写的，可行
                msg.data.push_back(-1);
                msg.data.push_back(1);
                msg.data.push_back(1);
                chatter_pub.publish(msg);
                break;
            case 'x':
                msg.data.push_back(-1);//自己写的，可行
                msg.data.push_back(-1);
                msg.data.push_back(-1);
                msg.data.push_back(-1);
                chatter_pub.publish(msg);
                break;
            case 's':
                msg.data.push_back(0);//自己写的，可行
                msg.data.push_back(0);
                msg.data.push_back(0);
                msg.data.push_back(0);
                chatter_pub.publish(msg);
                break;
            case 'd':
                msg.data.push_back(1);//自己写的，可行
                msg.data.push_back(1);
                msg.data.push_back(-1);
                msg.data.push_back(-1);
                chatter_pub.publish(msg);
                break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
