#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
// #include <curses.h>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgproc.hpp>
// using namespace cv;
float speed[4] = {0,0,0,0};
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
            case 'q':
                speed[0] = 0.25;
                speed[1] = 0.25;
                speed[2] = 0.25;
                speed[3] = 0.25;
                
                break;
            case 'a':
                speed[0] = -0.25;
                speed[1] = -0.25;
                speed[2] = -0.25;
                speed[3] = -0.25;
                break;
            case 'w':
                speed[0] = 0.5;
                speed[1] = 0.5;
                speed[2] = 0.5;
                speed[3] = 0.5;
                break;
            case 's':
                speed[0] = -0.5;
                speed[1] = -0.5;
                speed[2] = -0.5;
                speed[3] = -0.5;
                break;
            case 'e':
                speed[0] = 1;
                speed[1] = 1;
                speed[2] = 1;
                speed[3] = 1;
                break;
            case 'd':
                speed[0] = -1;
                speed[1] = -1;
                speed[2] = -1;
                speed[3] = -1;
                break;
            case 'r':
                speed[0] = 0;
                speed[1] = 0;
                speed[2] = 0;
                speed[3] = 0;
                break;
        }
        msg.data.push_back(speed[0]);
        msg.data.push_back(speed[1]);
        msg.data.push_back(speed[2]);
        msg.data.push_back(speed[3]);
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
