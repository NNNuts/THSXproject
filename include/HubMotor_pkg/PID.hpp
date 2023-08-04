#pragma once
#include <iostream>
//位置式PID
class PID
{
private:
    float kp;//比例系数
    float ki;//积分系数
    float kd;//微分系数
    float target;//目标值
    float actual;//实际值
    float e;//误差
    float e_pre;//上一次误差
    float integral;//积分项
public:
    // PID();
    // ~PID(){};
    // PID(float p,float i,float d);
    // float pid_control(float tar,float act);//执行PID控制
    // void pid_show();//显示PID控制器的内部参数
    // //位置PID
    PID():kp(0),ki(0),kd(0),target(0),actual(0),integral(0)
    {
        e=target-actual;
        e_pre=e;
    }
    PID(float p,float i,float d):kp(p),ki(i),kd(d),target(0),actual(0),integral(0)
    {
        e=target-actual;
        e_pre=e;
    }
    float pid_control(float tar,float act)
    {
        float u;
        target=tar;
        actual=act;
        e=target-actual;
        integral+=e;
        u=kp*e+ki*integral+kd*(e-e_pre);
        e_pre=e;
        return u;
    }
    void pid_show()
    {
        using std::cout;
        using std::endl;
        cout<<"The infomation of this position PID controller is as following:"<<endl;
        cout<<"       Kp="<<kp<<endl;
        cout<<"       Ki="<<ki<<endl;
        cout<<"       Kd="<<kd<<endl;
        cout<<" integral="<<integral<<endl;
        cout<<"   target="<<target<<endl;
        cout<<"   actual="<<actual<<endl;
        cout<<"        e="<<e<<endl;
        cout<<"    e_pre="<<e_pre<<endl;
    }
};
