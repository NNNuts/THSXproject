#include "test.h"
#include "draw_api.h"
//#include "delay.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"

void ShowStars(void);
void ShowWatch(void);
void ShowTest(void);
void ShowSnow(void);

void demo(void)
{
    int i, j;
    //demo演示
    ClearScreen();
    DrawBitmap(0, 0, Panda, 128, 64);
    UpdateScreen();
    DelayMs(1000);
    DelayMs(1000);
    DelayMs(1000);

    //字符
    ClearScreen();
    SetFontSize(1);
    DrawString(0, 0, "test");
    UpdateScreen();
    DelayMs(1000);

    SetFontSize(2);
    DrawString(0, 8, "test");
    UpdateScreen();
    DelayMs(1000);

    SetFontSize(3);
    DrawString(0, 24, "test");
    UpdateScreen();
    DelayMs(1000);

    //汉字测试
    ClearScreen();
    OLED_ShowCHinese(0, 0, "一只程序缘");
    UpdateScreen();
    DelayMs(1000);
    OLED_ShowCHinese(2, 0, "一只程序缘一只程序缘一只程序缘一只程序缘");
    UpdateScreen();
    DelayMs(1000);
    DelayMs(1000);

    //数字测试
    ClearScreen();

    for(i = 0; i < 300; i++)
    {
        SetFontSize(0);
        DrawNum(0, 96, i, 4);
        SetFontSize(1);
        DrawNum(0, 16, i, 4);
        SetFontSize(2);
        DrawNum(0, 24, i, 4);
        SetFontSize(3);
        DrawNum(0, 40, i, 4);
        UpdateScreen();
        DelayMs(1);
    }

    //串口debug
    DelayMs(1000);
    ClearScreen();

    for(i = 0; i < 256; i++)
    {
        OledPrintf("num:%d cha:%c hex:%x\r\n", i, i, i);
        DelayMs(7);
    }

    DelayMs(1000);

    //划线
    ClearScreen();

    for(i = 0; i < 20; i++)
    {
        DrawLine(0, 0, i * 10, 63);
        UpdateScreen();
        DelayMs(100);
    }

    for(i = 0; i < 20; i++)
    {
        DrawLine(128, 0, 128 - i * 10, 63);
        UpdateScreen();
        DelayMs(100);
    }

    DelayMs(1000);

    //矩形
    for(j = 0; j < 2; j++)
    {
        if(j == 0)
            ClearScreen();

        for(i = 0; i < 31; i += 2)
        {
            DrawRect2(i * 2, i, 128 - i * 4, 64 - 2 * i); //画矩形外框
            UpdateScreen();
            DelayMs(100);
        }

        if(j == 0)
            ClearScreen();

        DelayMs(500);

        for(i = 31; i > 0; i -= 2)
        {
            DrawFillRect2(i * 2, i, 128 - i * 4, 64 - 2 * i); //画实心矩形
            UpdateScreen();
            DelayMs(100);
        }

        SetDrawColor(pix_black);//划线颜色
        SetFillcolor(pix_black);//填充颜色
    }

    SetDrawColor(pix_white);
    SetFillcolor(pix_white);

    //圆角矩形
    for(j = 0; j < 2; j++)
    {
        if(j == 0)
            ClearScreen();

        for(i = 0; i < 25; i += 2)
        {
            DrawRoundRect(i * 2, i, 128 - i * 4, 64 - 2 * i, 8);
            UpdateScreen();
            DelayMs(100);
        }

        if(j == 0)
            ClearScreen();

        DelayMs(500);

        for(i = 25; i > 2; i -= 2)
        {
            DrawfillRoundRect(i * 2, i, 128 - i * 4, 64 - 2 * i, 8);
            UpdateScreen();
            DelayMs(100);
        }

        SetDrawColor(pix_black);
        SetFillcolor(pix_black);
    }

    SetDrawColor(pix_white);
    SetFillcolor(pix_white);
    DelayMs(1000);

    //椭圆
    ClearScreen();
    DrawEllipse(63, 31, 63, 31);
    UpdateScreen();
    DelayMs(1000);
    ClearScreen();
    DrawEllipse(63, 31, 16, 31);
    UpdateScreen();
    DelayMs(1000);
    ClearScreen();
    DrawFillEllipse(63, 31, 63, 31);
    UpdateScreen();
    DelayMs(1000);
    ClearScreen();
    DrawFillEllipse(63, 31, 16, 31);
    UpdateScreen();
    DelayMs(1000);

    //圆
    ClearScreen();
    DrawCircle(63, 31, 30);
    UpdateScreen();
    DelayMs(1000);
    ClearScreen();
    DrawFillCircle(63, 31, 30);
    UpdateScreen();
    DelayMs(1000);

    //三角形
    ClearScreen();
    DrawTriangle(5, 10, 100, 30, 60, 50);
    UpdateScreen();
    DelayMs(1000);
    ClearScreen();
    DrawFillTriangle(5, 10, 100, 30, 60, 50);
    UpdateScreen();
    DelayMs(1000);

    //绘制图形图片 位图
    ClearScreen();
    DrawBitmap(0, 0, BmpTest1, 16, 16);
    UpdateScreen();
    DelayMs(1000);
    DrawBitmap(16, 0, BmpTest2, 32, 32);
    UpdateScreen();
    DelayMs(1000);
    DrawBitmap(48, 0, BmpTest3, 64, 64);
    UpdateScreen();
    DelayMs(1000);

    //圆弧
    ClearScreen();

    for(i = 0; i < 369; i++)
    {
        DrawArc(63, 31, 30, 0, i);	//画圆弧
        UpdateScreen();
        ClearScreen();
    }

    DrawCircle(63, 31, 30);		//画圆
    UpdateScreen();
    DelayMs(500);

    for(i = 0; i < 10; i++)
    {
        DrawFillCircle(63, 31, i);	//在中心填充圆
        DrawCircle(63, 31, 30);
        UpdateScreen();
        ClearScreen();
        DelayMs(100);
    }

    //绕点
    for(i = 0; i < 720; i++) //转720度2圈
    {
        TypeXY temp;
        SetAngle(i);					//设置角度
        SetRotateCenter(63, 31);			//设置圆心
        temp = GetRotateXY(63, 31 + 30);		//讲已知坐标旋转角度
        DrawFillCircle(temp.x, temp.y, 5); //画出算出的位置
        DrawCircle(63, 31, 30);			//画出轨迹
        DrawFillCircle(63, 31, 10);		//填充中心
        UpdateScreen();					//更新屏幕
        ClearScreen();
    }

    //向右边平移
    for(i = 0; i < 95; i++)
    {
        TypeXY temp;					//其实就是上面继续旋转
        SetAngle(720 + i);				//画图的时候x+偏移量
        SetRotateCenter(63 + i, 31);
        temp = GetRotateXY(63 + i, 31 + 30);
        DrawFillCircle(temp.x, temp.y, 5);
        DrawCircle(63 + i, 31, 30);
        DrawFillCircle(63 + i, 31, 10);
        UpdateScreen();
        ClearScreen();
    }

    //动画
    ShowStars();	//星空动画
    ShowWatch();	//时钟动画
    ShowPolygon();	//多边形动画
    DelayMs(1000);

    //结束
    ClearScreen();
    SetFontSize(2);
    DrawString(8, 16, " Show End ");	//显示字符串
    UpdateScreen();
    DelayMs(1000);
    DelayMs(1000);
    ClearScreen();
    DrawBitmap(0, 10, Like, 128, 40);	//三连图片
    UpdateScreen();

    while(1);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//星空动画
void ShowStars(void)
{
    int i;
    int count = 0;
    int fps = 60;
    typedef struct START
    {
        short x;
        short y;
        short speed;
        unsigned char speedcount;
        unsigned char isexist;
    } Star;

    Star star[128] = {0};
    srand(2);

    for(i = 0; i < 128; i++)
    {
        if(star[i].isexist == 0)
        {
            //设置128个()星星的初始信息
            star[i].x = rand() % 127;		//随机生成初始x坐标
            star[i].y = rand() % 64;		//随机生成y的坐标
            star[i].speedcount = 0;
            star[i].speed = rand() % 8 + 1;	//1-8的数
            star[i].isexist = 1;
        }
    }

    while(1)
    {
        if(FrameRateUpdateScreen(fps) == 1)	//在此函数中定时刷新屏
        {
            count++;

            if(count >= fps * 10)   			//10秒钟时间到达之后跳出循环结束动画
                return;
        }

        //此段函数一直在运行
        //依次画出128个星星
        for(i = 0; i < 128; i++)
        {
            //如果这一个星星已经移动到退出屏幕界面
            //则在最左侧重新生成一颗新星星
            if(star[i].isexist == 0)
            {
                star[i].x = 0;
                star[i].y = rand() % 64;
                star[i].speed = rand() % 6 + 1;
                star[i].speedcount = 0;
                star[i].isexist = 1;
            }
            else
            {
                star[i].speedcount++;

                if(star[i].x >= 124)			//标记已经退出屏幕
                    star[i].isexist = 0;

                //清除上一个时刻画的星星(的尾巴) 不管有没有操作 都进行清除操作
                SetDrawColor(pix_black);
                DrawLine(star[i].x, star[i].y, star[i].x, star[i].y);

                SetDrawColor(pix_white);

                if(star[i].speedcount == star[i].speed)	//运行时间到了一定的长度
                {
                    star[i].speedcount = 0;				//复位运行时间并向右移一格
                    star[i].x += 1;						//总之星星的结束需要在这经历124次
                }										//只不过有的更快 就能移动更快

                //从头到尾画出整条星星 不管星星是否已经变化
                DrawLine(star[i].x, star[i].y, star[i].x + (6 / star[i].speed) - 1, star[i].y);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////
//时钟动画
//钟 小时 分钟 秒，
void RoundClock(int hours, int minute, int sec)
{
    unsigned char i = 0;
    TypeXY hourspoint, minutepoint, secpoint, tmp1, tmp2;

    //时针
    SetRotateValue(63, 31, hours * 30 + (minute * 30) / 60, 1);
    hourspoint = GetRotateXY(63 - 14, 31);
    DrawLine(63, 31, hourspoint.x, hourspoint.y);
    //分针
    SetRotateValue(63, 31, minute * 6 + (sec * 6) / 60, 1);
    minutepoint = GetRotateXY(63 - 21, 31);
    DrawLine(63, 31, minutepoint.x, minutepoint.y);
    //秒针
    SetRotateValue(63, 31, sec * 6, 1);
    secpoint = GetRotateXY(63 - 28, 31);
    DrawLine(63, 31, secpoint.x, secpoint.y);

    //表盘
    for(i = 0; i < 12; i++)
    {
        SetRotateValue(63, 31, i * 30, 1);
        tmp1 = GetRotateXY(63 - 29, 31);
        tmp2 = GetRotateXY(63 - 24, 31);
        DrawLine(tmp1.x, tmp1.y, tmp2.x, tmp2.y);
    }

    DrawFillCircle(63, 31, 2);
    DrawCircle(63, 31, 30);
    UpdateScreen();
    ClearScreen();
}

void ShowWatch(void)
{
    int i, j, z;
    int count = 0;

    for(i = 0; i < 12; i++)
        for(j = 0; j < 60; j++)
            for(z = 0; z < 60; z++)
            {
                RoundClock(i, j, z);
                count++;

                if(count >= 800)
                    return;
            }
}

/////////////////////////////////////////////////////////////////////////
//多边形动画
void ShowPolygon(void)
{
    int x0 = 63, y0 = 31;		//正多边形的外接圆的圆心
    unsigned char i = 0, j;
    int n = 1, r = 31;			//画正n边形 大小半径31
    int v = 1, count = 0;		//每个相邻的多边形隔1 画count次后退出
    int x[30], y[30];

    while(1)
    {
        ClearScreen();

        for(i = 0; i < n; i++)
        {
            x[i] = r * cos(2 * 3.1415926 * i / n) + x0;
            y[i] = r * sin(2 * 3.1415926 * i / n) + y0;
        }

        for(i = 0; i <= n - 2; i++)
        {
            for(j = i + 1; j <= n - 1; j++)
                DrawLine(x[i], y[i], x[j], y[j]);
        }

        n += v;

        if(n == 20 || n == 0)
            v = -v;

        DelayMs(100);		//延时停顿100ms
        UpdateScreen();

        if(++count == 90)
        {
            count = 0;
            return ;
        }
    }
}

//下雪的函数 和星空类似 就不展示了
void ShowSnow(void)
{
    int a[66], i, num = 0;
    struct Snow
    {
        short x;
        short y;
        short speed;
    } snow[100];

    srand(1);

    for(i = 0; i < 66; i++)
        a[i] = (i - 2) * 10;

    ClearScreen();

    while(1)
    {
        if(num != 100)
        {
            snow[num].speed = 1 + rand() % 4;
            i = rand() % 66;
            snow[num].x = a[i];
            snow[num].y = 0;
            num++;
        }

        for(i = 0; i < num; i++)
        {
            snow[i].y += snow[i].speed;
            DrawPixel(snow[i].x, snow[i].y + 1);
            DrawPixel(snow[i].x + 1, snow[i].y);
            DrawPixel(snow[i].x, snow[i].y);
            DrawPixel(snow[i].x - 1, snow[i].y);
            DrawPixel(snow[i].x, snow[i].y - 1);

            if(snow[i].y > 63)
            {
                snow[i].y = 0;
            }
        }

        DelayMs(30);
        UpdateScreen();
        ClearScreen();
    }
}

