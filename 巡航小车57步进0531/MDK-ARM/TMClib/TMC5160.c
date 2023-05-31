#include "TMC5160_Register.h"
#include "TMC5160_Mask_Shift.h"
#include "SPI_TMC.h"
#include "TMC5160.h"
#include "main.h"

tmc5160 MotorTmc5160[TotalAxis];
Axis_t  Axis[TotalAxis];//定义有几个轴;
//__IO uint32_t motor_counter;//电机超时计数 需要放到定时器里++即可 1ms时间片

void tmc_delay_us(__IO uint32_t nTime)
{
    u16 i = 0, j;

    for(j = 0; j < nTime; j++)
        for(i = 0; i < 50; i++); //for(i=0; i<10; i++);sfl20220711

}
/*------------------------------------------------------
 * @brief     tmc5130片选使能/失能
 * @param
 * @param
 * @return
 *

------------------------------------------------------*/
void tmc_cs_sel(uint8_t slave, uint8_t status)
{
    switch(slave)
    {
        case 0:
            if(status == low)
                TMC_CS1_L;

            if(status == high)
                TMC_CS1_H;

            break;

        case 1:
            if(status == low)
                TMC_CS2_L;

            if(status == high)
                TMC_CS2_H;

            break;

        case 2:
            if(status == low)
                TMC_CS3_L;

            if(status == high)
                TMC_CS3_H;

            break;

        case 3:
            if(status == low)
                TMC_CS4_L;

            if(status == high)
                TMC_CS4_H;

            break;

        case 4:
            if(status == low)
                TMC_CS5_L;

            if(status == high)
                TMC_CS5_H;

            break;
    }
}


//======================================================



/*------------------------------------------------------
 * @brief通过SPI读取四个字节
  * @param地址寄存器地址
  * @param csPin芯片选择
  * @return状态
------------------------------------------------------*/
uint32_t tmc_spi_readregister(uint8_t slave, uint8_t address)
{
    uint8_t buf[4];
    uint32_t register_value = 0;

    tmc_cs_sel(slave, low);    	//enable slave, low activ  启用从机，低活动--使能芯片  --此处CS低电平
    tmc_delay_us(1);

    // first read cycle to address the register  第一个读周期来寻址寄存器
    tmc_spi_readwritebyte(address);
    tmc_spi_readwritebyte(0x00);
    tmc_spi_readwritebyte(0x00);
    tmc_spi_readwritebyte(0x00);
    tmc_spi_readwritebyte(0x00);
    tmc_delay_us(3);
    tmc_cs_sel(slave, high);    	//disable slave, low activ  禁止从机，低电平使能  --此处CS高电平


    tmc_cs_sel(slave, low);    	//select slave, low activ   选择从机 低电平使能  --此处CS低电平
    tmc_delay_us(1);
    // second read cycle to get the register value  第二个读取周期来获取寄存器值
    tmc_spi_readwritebyte(address);

    tmc_spi_readwritebyte(0x00);//读数据
    buf[3] = tmc_sdo_data[slave];

    tmc_spi_readwritebyte(0x00);//读数据
    buf[2] = tmc_sdo_data[slave];

    tmc_spi_readwritebyte(0x00);//读数据
    buf[1] = tmc_sdo_data[slave];

    tmc_spi_readwritebyte(0x00);//读数据
    buf[0] = tmc_sdo_data[slave];

    tmc_delay_us(3);
    //SPI_CS_PORT |= (1 << slave);	 // disable slave, low activ  禁止从机，低电平使能 --此处CS高电平
    tmc_cs_sel(slave, high);   //
    register_value |= buf[3];                       //数据合并
    register_value = register_value << 8;
    register_value |= buf[2];
    register_value = register_value << 8;
    register_value |= buf[1];
    register_value = register_value << 8;
    register_value |= buf[0];

    return register_value;            //返回数据
}
//======================================================



/*------------------------------------------------------

  * @brief通过SPI发送五个字节并接收状态
  * @param地址寄存器地址
  * @param数据传输数据
  * @param csPin芯片选择
  * @return状态
------------------------------------------------------*/
uint8_t tmc_spi_writeregister(uint8_t slave, uint8_t address, uint32_t data)
{
    uint8_t buf[4];
    uint8_t status = 0;

    buf[0] = data & 0xFF;                   //数据拆散
    buf[1] = (data & 0xFF00) >> 8;
    buf[2] = (data & 0xFF0000) >> 16;
    buf[3] = (data & 0xFF000000) >> 24;

    tmc_cs_sel(slave, low);    			 // 使能芯片
    tmc_delay_us(1);
    tmc_spi_readwritebyte(address | 0x80);   // address register  地址寄存器
    tmc_spi_readwritebyte(buf[3]);
    tmc_spi_readwritebyte(buf[2]);
    tmc_spi_readwritebyte(buf[1]);
    tmc_spi_readwritebyte(buf[0]);
    tmc_delay_us(4);              //此延时时间必须加，数据写入完成后才对CS引脚使能，否则数据掉失。根据速率调整延时。
    tmc_cs_sel(slave, high);   //  disable slave, low activ
    return status;
}
//======================================================

unsigned char bitCount(unsigned char a)//统计二进制1的个数
{
    unsigned char count = 0;

    while (a)
    {
        count++;
        a = a & (a - 1);
    }

    return count;
}

//    u32 vstart;		//启动速度VSTOP≥VSTART
//    u32 a1;       //VSTART 和 V1 之间的加速度
//    u32 v1;				//第一加速/减速阶段阈值速度(无符号)
//    u32 amax ;		//V1 和 VMAX 之间的加速度
//    u32 vmax;			//运动斜坡目标速度(位置模式确保VMAX≥VSTART )
//    u32 dmax;		 	//VMAX 和 V1 之间的减速度
//    u32 d1;				//V1 和 VSTOP 之间的减速度
//    u32 vstop;		//停止速度（接近零）
//加速度的时间参考 ta2: ta2 = 2^41 / (fCLK)2
void tmc5160_SetPara(tmc5160 *motor, uint16_t micro, uint8_t hold, uint8_t run, \
                     uint32_t vstart, uint32_t a1, uint32_t v1, uint32_t amax, uint32_t vmax, \
                     uint32_t dmax, uint32_t d1, uint32_t vstop, uint8_t slave)
{
//	motor->microstep	=	256;
//	motor->hold_current = 5;
//	motor->run_current	=	10;
//	motor->motorcurve.vstart	=	5;
//	motor->motorcurve.a1	=	0;
//	motor->motorcurve.v1	=	0;
//	motor->motorcurve.amax	=	10000;
//	motor->motorcurve.vmax	=	10*MotorTmc5160[slave].microstep*200*tmc_fclk;
//	motor->motorcurve.dmax	=	20000;
//	motor->motorcurve.d1	=	400;
//	motor->motorcurve.vstop	=	10;
    motor->microstep	=	micro;
    motor->hold_current = hold;
    motor->run_current	=	run;
    motor->motorcurve.vstart	=	vstart;
    motor->motorcurve.a1	=	a1;
    motor->motorcurve.v1	=	v1;
    motor->motorcurve.amax	=	amax;
    //motor->motorcurve.vmax	=	vmax*MotorTmc5160[slave].microstep*200*tmc_fclk;//速度的时间参考: t = 2^24 / fCLK
    motor->motorcurve.vmax	=	vmax * 2.65 * 5120; //   /10.0
    motor->motorcurve.dmax	=	dmax;
    motor->motorcurve.d1	=	d1;
    motor->motorcurve.vstop	=	vstop;
//	tmc_spi_writeregister(slave,TMC5160_RAMPMODE	,0x00000000);
    tmc_spi_writeregister(slave, TMC5160_A1				, motor->motorcurve.a1);
    tmc_spi_writeregister(slave, TMC5160_V1				, motor->motorcurve.v1);
    tmc_spi_writeregister(slave, TMC5160_AMAX			, motor->motorcurve.amax);
    tmc_spi_writeregister(slave, TMC5160_VMAX			, motor->motorcurve.vmax);
    tmc_spi_writeregister(slave, TMC5160_DMAX			, motor->motorcurve.dmax);
    tmc_spi_writeregister(slave, TMC5160_D1				, motor->motorcurve.d1);
    tmc_spi_writeregister(slave, TMC5160_VSTOP			, motor->motorcurve.vstop);
}
void tmc5160_InitConfig(tmc5160 *motor, uint8_t slave)
{
    uint32_t value;
    //uint32_t regtemp;
    //SPI数据报示例序列使驱动程序能够进行步进和方向操作并进行初始化 spread Cycle操作的斩波器和<60 RPM的stealthChop：
    tmc_spi_writeregister(slave, TMC5160_CHOPCONF	, 0x000100C3  | (bitCount(~(motor->microstep) + 1) << 24)); //Chopper和驱动程序配置 CHOPCONF: TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
    //12 MHz 时钟设置 TOFF=3.4, i.e. 3 or 4.
    //16 MHz 时钟设置	TOFF=4.6, i.e. 4 or 5.
    tmc_spi_writeregister(slave, TMC5160_TPOWERDOWN	, 0x00000020); 		//  TPOWERDOWN=10: 电机静止到电流减小之间的延时
    tmc_spi_writeregister(slave, TMC5160_COOLCONF		, 0x00000003);		//智能电流控制配置coolConf 堵转
    tmc_spi_writeregister(slave, TMC5160_PWMCONF		, 0x000401C8);		// PWM_CONF: AUTO=1, 1/1024 Fclk, Switch amplitude limit=200, Grad=1   PWM_CONF：AUTO = 1,1 / 1024 Fclk，开关幅度限制= 200，Grad = 1
    tmc_spi_writeregister(slave, 0x00								, 0x00000004);   			//EN_PWM_MODE=1 e nables stealthChop (with default PWM_CONF)  --全局配置  -启用 stealthChop
    tmc_spi_writeregister(slave, TMC5160_TPWMTHRS		, 0x000001F4);		// TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM  TPWM_THRS = 500产生约35000 =约的切换速度。30RPM
    tmc_spi_writeregister(slave, TMC5160_SWMODE			, 0x000000EF);		//开启限位
    tmc_spi_writeregister(slave, TMC5160_RAMPMODE		, 0x00000000);		
    tmc_spi_writeregister(slave, TMC5160_GCONF			, 0x00001084);		//常规配置     --运动方向

    value = SET_IHOLD(motor->hold_current) | SET_IRUN(motor->run_current) | SET_IHOLDDELAY(7);
    tmc_spi_writeregister(slave, TMC5160_IHOLD_IRUN, value);
//	regtemp = tmc_spi_readregister(slave,TMC5160_IHOLD_IRUN);  //读取状态和位置

    tmc_spi_writeregister(slave, TMC5160_A1				, motor->motorcurve.a1);
    tmc_spi_writeregister(slave, TMC5160_V1				, motor->motorcurve.v1);
    tmc_spi_writeregister(slave, TMC5160_AMAX			, motor->motorcurve.amax);
    tmc_spi_writeregister(slave, TMC5160_VMAX			, motor->motorcurve.vmax);
    tmc_spi_writeregister(slave, TMC5160_DMAX			, motor->motorcurve.dmax);
    tmc_spi_writeregister(slave, TMC5160_D1				, motor->motorcurve.d1);
    tmc_spi_writeregister(slave, TMC5160_VSTOP			, motor->motorcurve.vstop);

    tmc_spi_writeregister(slave, TMC5160_XACTUAL		, 0); //XACTUAL = 0    清除电机实际位置
    tmc_spi_writeregister(slave, TMC5160_XTARGET		, 0);	//XTARGET = 0    原点返回步数清零

}

void Axis_init()
{
    MotorTmc5160[motor1].Axis.target_pos_i	=	0;
    MotorTmc5160[motor1].Axis.cur_pos_i	    =	0;
    MotorTmc5160[motor1].Axis.limitflag	    =	limit_nomal;
    MotorTmc5160[motor1].Axis.pitch		    	=	0.79375; //马达转动1圈运动的距离
    MotorTmc5160[motor1].Axis.resolution    =	MotorTmc5160[motor1].Axis.pitch / (MotorTmc5160[motor1].microstep * 200 * 1);
    MotorTmc5160[motor1].Axis.permm		    	=	51200.0 / MotorTmc5160[motor1].Axis.pitch;
    MotorTmc5160[motor1].Axis.target_pos_f	=	0;
    MotorTmc5160[motor1].Axis.cur_pos_f	    =	0;
    MotorTmc5160[motor1].Axis.busyflag	    =	false;
    //MotorTmc5160[motor1].Axis.timeout	    = (int)(30000/speed); //根据速度设置超时时间
    MotorTmc5160[motor1].Axis.timeout	    	= 105000; //根据速度设置超时时间
    MotorTmc5160[motor1].Axis.zeroing       = false ;
    MotorTmc5160[motor1].Axis.es            = false ;
}

//电机限位读取
//变量：motor-电机X
//返回值:无;
void scan_limitstatus(uint8_t  motor)
{


}
//电机立即停止
//变量：motor-电机X
//返回值:无;
void motor_stop(uint8_t motor)
{

}
//运行一步一个脉冲 位置为相对位置
void motor_runstep(uint8_t motor, int step)
{
    int value, regtemp;

    if(MotorTmc5160[motor].Axis.es == 0)
    {
        regtemp = tmc_spi_readregister(motor, TMC5160_XACTUAL);
        value = regtemp + step;
        MotorTmc5160[motor].Axis.target_pos_i = value;
        tmc_spi_writeregister(motor, TMC5160_XTARGET, value);
    }
}
//运行一圈 位置为相对位置
void motor_runlap(uint8_t motor, int lap)
{

}
//电机运行到绝对位置
//变量：motor-电机X
//变量：pos-要运行到的位置
//返回值:无;
void motor_runpos(uint8_t motor, int pos)
{

}
//电机回零
//变量：motor-电机X
//变量：dir-回零方向 根据实际调整 限位和方向
//变量：wait-是否需要等待运行到位
//返回值:无;
void motor_zeroing(uint8_t  motor, uint8_t dir, uint8_t wait)
{

}
//电机回零nowait模式下 定时器判断等待执行到位
//变量：motor-电机X
//返回值:无;
void scan_motor_zeroing(uint8_t  motor, uint8_t dir)
{

}
//电机运行Nmm距离
//变量：motor-电机X
//变量：mm-运行距离
//变量：wait-是否需要等待运行到位
//返回值:无;
void motor_run_mm(uint8_t motor, float mm, u8 posmode, uint8_t wait)
{

}
//电机运行nowait模式下 定时器判断等待执行到位
//变量：motor-电机X
//返回值:无;
void motor_runstatus_check()
{

}
//等待电机执行到位 超时自动退出
//变量：motor-电机X
//返回值:无;
void motor_wait_run_ok(uint8_t motor)
{

}
//等待电机执行到位 超时自动退出
//变量：motor-电机X
//返回值:无;
void motor_wait_zeroing_ok(uint8_t motor)
{

}
//调用方法
/*
//初始化开始
	tmc_spi_init();
//void tmc5160_SetPara(tmc5160 *config,micro,hold,run,\
//	vstart,a1,v1,amax,vmax,\
//	 dmax,d1,vstop,slave)
	MotorTmc5160[motor1].Axis.speed = 5; //尽量设置不要超过12


	tmc5160_SetPara(&MotorTmc5160[motor1],256,4,10,\
	5,0,0,10000,MotorTmc5160[motor1].Axis.speed,\
	10000,500,10,motor1);
	tmc5160_InitConfig(&MotorTmc5160[motor1],motor1);


	Axis_init();
//初始化结束
	motor_zeroing(motor1,reversedir,nowait);
	//等待电机执行到位
	motor_wait_zeroing_ok(motor1);
	while(1){
		motor_runstep(motor1,51200);
		motor_runstep(motor2,51200);
		motor_runstep(motor3,51200);
		motor_runstep(motor4,51200);
		delay_ms(2000);

		motor_zeroing(motor3,reversedir,timeoutwait);
		motor_run_mm(motor1,10.0,absolutepos,nowait);
	}


*/


