#ifndef _CAN232_HPP_
#define _CAN232_HPP_
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
#include <termios.h>
#include <fcntl.h>
#include <iconv.h>

using namespace Eigen;
using namespace std;

class CAN232
{
public:
    int fd;

    void usartInit(void)
    {
        open_port();
        set_uart_config();
    }

    void set_uart_config(void)
    {
        struct termios opt;
        int speed = B9600;
        if (tcgetattr(fd, &opt) != 0)
        {
            perror("tcgetattr");
        }
    
        cfsetispeed(&opt, speed);
        cfsetospeed(&opt, speed);
        tcsetattr(fd, TCSANOW, &opt);
    
        opt.c_cflag &= ~CSIZE;
        opt.c_cflag |= CS8;

        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能  
    
        opt.c_cflag &= ~CSTOPB;
    
    
        /*处理未接收字符*/
        tcflush(fd, TCIFLUSH);
    
        /*设置等待时间和最小接收字符*/
        opt.c_cc[VTIME] = 1000;
        opt.c_cc[VMIN] = 0;
    
        /*关闭串口回显*/
        opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | NOFLSH);
    
        /*禁止将输入中的回车翻译为新行 (除非设置了 IGNCR)*/
        opt.c_iflag &= ~ICRNL;
        /*禁止将所有接收的字符裁减为7比特*/
        opt.c_iflag &= ~ISTRIP;
    
        /*激活新配置*/
        if ((tcsetattr(fd, TCSANOW, &opt)) != 0)
        {
            perror("tcsetattr");
        }
    }

    /*
    * 打开串口
    */
    void open_port(void)
    {
        /* 使用普通串口 */
        // TODO::在此处添加串口列表
        char* dev = { "/dev/ttyS1" };
    
        //O_NDELAY 同 O_NONBLOCK。
        fd = open(dev, O_RDWR | O_NOCTTY);
        if (fd < 0)
        {
            perror("open serial port");
        }
    
        //恢复串口为阻塞状态 
        //非阻塞：fcntl(fd,F_SETFL,FNDELAY)  
        //阻塞：fcntl(fd,F_SETFL,0) 
        if (fcntl(fd, F_SETFL, 0) < 0)
        {
            perror("fcntl F_SETFL\n");
        }
        /*测试是否为终端设备*/
        if (isatty(STDIN_FILENO) == 0)
        {
            perror("standard input is not a terminal device");
        }
    }

    void motorSendData(long Data)
    {
        unsigned char data[8];
        for(int i=7;i>=0;i--)
        {
            data[i] = Data % 256;
            Data = Data / 256;
        }
        cout<<(unsigned)data[0]<<" "<<(unsigned)data[1]<<" "<<(unsigned)data[2]<<" "<<(unsigned)data[3]<<" "
            <<(unsigned)data[4]<<" "<<(unsigned)data[5]<<" "<<(unsigned)data[6]<<" "<<(unsigned)data[7]<<endl;
        write(fd, data, 8);
    }

    void motorSetPosition(int ID, double rad)
    {
        long data = 0;
        data += 0x0100000000000000 * ID;
        data += 0x01000000000000;
        data += 0x010000000000;
        if(rad >= 0)
        {
            //cout<<(unsigned)(rad*1000/256)<<endl;
            data += ((unsigned)(rad*1000/256))*0x0100000000;
            data += ((unsigned)(rad*1000)%256)*0x01000000;
        }
        else
        {
            //cout<<(unsigned)(-rad*1000/256+128)<<endl;
            data += ((unsigned)(-rad*1000/256+128))*0x0100000000;
            data += ((unsigned)(-rad*1000)%256)*0x01000000;
        }
        //data += ((unsigned)(rad*1000)%256)*0x01000000;
        //cout<<data<<endl;
        motorSendData(data);
    }
};


#endif