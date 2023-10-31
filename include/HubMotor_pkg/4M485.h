 
#ifndef comm_service_h
#define comm_service_h
 
//串口相关的头文件
#include <stdio.h>  /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <string.h>
//宏定义
#define FALSE -1
#define TRUE 0
 
namespace dst_ccms_api
{
    class comm_service
    {
    public:
        comm_service();
        ~comm_service();
 
        /*******************************************************************  
            *名称：             CommOpen  
            *功能：            打开串口并返回串口设备文件描述  
            *param：         port  串口号(ttyS0,ttyS1,ttyS2)
            *return：         void  
        *******************************************************************/
        int CommOpen(char *port);
 
        /*******************************************************************  
            *名称：             CommClose  
            *功能：             关闭串口并返回串口设备文件描述  
            *param：         fd  文件描述符     
            *return：         void  
        *******************************************************************/
        void CommClose();
 
        /*******************************************************************  
            *名称：             CommInit  
            *功能：             串口初始化  
            *param：         fd          串口文件描述符
            *                           speed       串口速度  
            *                           flow_ctrl   数据流控制  (0:不使用 1:硬件流控制 2:软件流控制)
            *                           databits    数据位   取值为 7 或者8  
            *                           stopbits    停止位   取值为 1 或者2  
            *                           parity      效验类型 取值为N(无校验位),E(偶检验),O(奇校验)  
            *return：         正确返回为1，错误返回为-1 
        *******************************************************************/
        int CommInit(int baudrate, int flow_ctrl, int databits, int stopbits, int parity);
 
        /*******************************************************************  
            *名称：             CommRecv  
            *功能：             接收串口数据  
            *param：         rcv_buf  接收串口中数据存入rcv_buf缓冲区中     
            *                           data_len  一帧数据的长度
            *return：         正确返回为1，错误返回为-1   
        *******************************************************************/
        int CommRecv(char *rcv_buf, int data_len);
 
        /*******************************************************************  
            *名称：             CommSend  
            *功能：             发送数据  
            *param：         send_buf  存放串口发送数据     
            *                           data_len  一帧数据的长度
            *return：         正确返回为1，错误返回为-1    
        *******************************************************************/
        int CommSend(char *send_buf, int data_len);
 
        /*******************************************************************  
            *名称：             getCommFD  
            *功能：             获取文件描述  
            *return：         文件描述句柄
        *******************************************************************/
        int CommGetFD() const;
 
        /*******************************************************************  
            *名称：             serialFlush  
            *功能：             清除缓冲区 TCIOFLUSH(刷清输入、输出队列)   TCIFLUSH(刷清输入队列) TCIFLUSH (刷清输出队列)
            *return：         true:开启 flase:关闭
        *******************************************************************/
        void CommFlush(int flush);
 
    private:
        /*******************************************************************  
            *名称：             commSet  
            *功能：             设置串口数据位，停止位和效验位   
            *param：         fd          串口文件描述符
            *                           baudrate    波特率  
            *                           flow_ctrl   数据流控制  
            *                           databits    数据位   取值为 7 或者8  
            *                           stopbits    停止位   取值为 1 或者2  
            *                           parity      效验类型 取值为N,E,O,,S  
            *return：          void  
        *******************************************************************/
        int commSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity);
 
        /*******************************************************************  
            *名称：             commIsOpen  
            *功能：             串口是否开启  
            *return：         true:开启 flase:关闭
        *******************************************************************/
        bool commIsOpen() const;
 
        /*******************************************************************  
            *名称：             serial_set_speci_baud  
            *功能：             非特定波特率设置  
            *param：         options    配置
                                         baud         非标准波特率(如28800..)
            *return：         正确返回为0，错误返回为-1  
        *******************************************************************/
        int serial_set_speci_baud(struct termios &options,int baud);
 
        int m_fd; // 文件描述句柄
    };
} // namespace qh_dst_ccms_api
#endif