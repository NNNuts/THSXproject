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
    // cout<<"str "<<str<<endl;
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


int main ()  
{  
  struct input_event t;  
  ifstream infile;
 
  if(OpenEventFile(infile) == -1)
  {
      return 0;
  }
 
  bool blPressIsCtrl = false;
  bool blPressIsC    = false;
 
  while (1)  
  {
       if(infile.read((char *)&t,sizeof(t)))
       {//读取文件成功
            // cout << "you use key value is "<<t.code<<endl;
           if(t.type == EV_KEY )
           {
               if((t.code == KEY_LEFTCTRL || t.code == KEY_RIGHTCTRL) && t.value == 1)
               {
                   blPressIsCtrl = true;
               }
 
               if ((t.code == KEY_LEFTCTRL || t.code == KEY_RIGHTCTRL) && t.value == 0)
               {
                   blPressIsCtrl = false;
               }
 
               if(t.code == KEY_C && t.value == 1 && blPressIsCtrl == true)
               {
                   cout<<"press ctrl + c"<<endl;
               }
               if(t.value == 0)
               {
                    cout << "松开 "<<t.code<<endl;
                    cout << 's' << endl;
               }
               else if(t.value == 1)
               {
                    cout << "按下 "<<t.code<<endl;
               }
               
            //    cout << "you use key value is "<<t.code<<" value "<<t.value<<endl;
           }
       }
       else
       {//读取文件失败 需要再次进行一次读取
           cout <<"read file error"<<endl;
           infile.close();  
           if(OpenEventFile(infile) == -1)
           {
              return 0;
           }
       }
  }  
  infile.close();  
  return 0;  
}  
