#include <stdio.h>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
 
int main()
{
        int fd_kb;
	
        struct input_event event_kb;
 
        fd_kb = open("/dev/input/event19", O_RDONLY); //键盘输入
        if(fd_kb <= 0)
        {
                printf("open device error\n");
                return 0;
        }
 
        while(1)
        {
                if(read(fd_kb, &event_kb, sizeof(event_kb)) == sizeof(event_kb))
                {
                    if (event_kb.type == EV_KEY)
					{
                        //if (event_kb.value == 0 || event_kb.value == 1)//1表示按下，0表示释放，会检测到两次
						if (event_kb.value == 1)//键按下
                        {
                                //printf("key %d %s\n", event_kb.code, (event_kb.value) ? "Pressed" : "Released");
                                if(event_kb.code == KEY_ESC)
                                        break;
								if(event_kb.code == KEY_Q)
									printf("q\n");                                 
								if(event_kb.code == KEY_W)
                                    printf("w\n");  
								if(event_kb.code == KEY_E)
                                    printf("e\n");  
								if(event_kb.code == KEY_R)
                                    printf("r\n");  
                        }
 
					}
				
                }
				
				
				
        }
        close(fd_kb);
        return 0;
}