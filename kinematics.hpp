#ifndef _KINEMATICS_
#define _KINEMATICS_
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;


#define Rpy Vector3d

//关节数值
class Joints
{
public:
    Joints(double jointsNum[6])
    {
        joint0 = joints[0] = jointsNum[0];
        joint1 = joints[1] = jointsNum[1];
        joint2 = joints[2] = jointsNum[2];
        joint3 = joints[3] = jointsNum[3];
        joint4 = joints[4] = jointsNum[4];
        joint5 = joints[5] = jointsNum[5];
    }


    double joint0;
    double joint1;
    double joint2;
    double joint3;
    double joint4;
    double joint5;

    double joints[6];
};

class TrainComputer
{
public:
    double theta_now[2];//记录理论当前机械臂位置

    // double kinematics(double theta1,double theta2, double y)
    // {
    //     double a = (y+113.3/sin(theta1))*tan(theta1)-(113.3/tan(theta1)+211)*sin(theta1);
    //     double b = y - (113.3/tan(theta1) + 211) * cos(theta1)+113.3/sin(theta1);
    //     double z = sqrt(a*a + b * b) * tan(theta2) + 486;
    //     return z;
    // }

    // int invest(double x, double y, double z, double* ret)
    // {
    //     // double theta1, theta2;
    //     // theta1 = atan2(y, x);
    //     // theta2 = atan2(z - 486, sqrt(x*x + y * y) - 211);
        

    //     int flag = 0;
    //     if((x>-113.3 && y>113.3) || (x>113.3 && y<113.3))
    //     {
    //         flag = true;
    //         x = -x;
    //         y = -y;
    //     }

    //     double theta1,theta2;
    //     theta1 = acos(113.3/(sqrt(x*x + y*y))) + atan2(-x,y) - EIGEN_PI/2;
    //     theta2 = atan2(z - 486,sqrt(x*x+ y*y)*cos(EIGEN_PI/2 - acos(113.3/sqrt(x*x+y*y)))-211);
    //     if(flag)
    //     {
    //         theta1 = theta1 - EIGEN_PI;
    //     }
    //     if(theta1 < -2*EIGEN_PI)
    //     {
    //         theta1 += 2*EIGEN_PI;
    //     }
    //     *ret = theta1; ret++;
    //     *ret = theta2;
    // }

    double kinematics(double theta1,double theta2, double y)
    {
        // double x0 = 150 * cos(theta1);
        // double y0 = 150 * sin(theta1);
        double y1 = (y - 150 * sin(theta1)) / cos(theta1) / cos(theta2);
        // double x  = 150 * cos(theta1) - y1 * cos(theta2) * sin(theta1);
        double z  = 456 + y1 * sin(theta2);

        return z;
    }

    

    int invest(double x, double y, double z, double* ret)
    {
        double theta1,theta2;

        theta1 = EIGEN_PI - (atan2(y,-x) + acos(150./sqrt(x*x+y*y)));
        // theta2 = acos(sqrt(((x-150*cos(theta1))*(x-150*cos(theta1))+(y-150*sin(theta1))*(y-150*sin(theta1)))
        // /((x-150*cos(theta1))*(x-150*cos(theta1))+(y-150*sin(theta1))*(y-150*sin(theta1))+(z-456)*(z-456))));
        theta2 = atan2((z-456),sqrt(x*x+y*y-150*150));

        *ret = theta1; ret++;
        *ret = theta2;
    }
};

 




#endif