/*
 * TMC5160.h
 */

#ifndef TMC5160_H
#define TMC5160_H

//#include "gd32f10x.h"
//#include "stdint_h"

#define TotalAxis	5 //定义电机数量

#define motor1	0x00
#define motor2	0x01
#define motor3	0x02
#define motor4	0x03
#define motor5	0x04


#define forwarddir	0x00	//正向
#define reversedir	0x01	//反向
//run mm 走相对位置还是绝对位置
#define relativepos	0x00	//相对
#define absolutepos	0x01	//绝对

#define nowait			0x00	//不等待
#define timeoutwait	0x01	//超时等待


#define low		0x00
#define high 	0xFF
 
#define  tmc_fclk   1.048576 //速度的时间参考: t = 2^24 / fCLK
//#define  Seedp_conversion  51200*TMC_fCLK
//#define  speed_conversion(slave)  tmc5160config[slave].microstep*200*tmc_fclk
#define  everystep  200*1000
//#define  SingNumber(slave,sing)   (tmc5160config[slave].microstep*200*sing)    //圈数    
//#define  StepNumber(slave,step)   (tmc5160config[slave].microstep*step)        //步数



#define REGISTER_COUNT 128
	
#define SET_IHOLD(a)		  ((a & 0x1F)<<0)
#define SET_IRUN(a)		  	((a & 0x1F)<<8)
#define SET_IHOLDDELAY(a)	((a & 0xF)<<16)
	
	
	
	
/*
电机类型定义开始
*/
//typedef enum {false = 0, true = !false} bool; 
enum limitenum {
    limit_nomal,
		limit_left,
		limit_right	
};

typedef struct{
    uint32_t vstart;		//启动速度VSTOP≥VSTART
    uint32_t a1;       //VSTART 和 V1 之间的加速度
    uint32_t v1;				//第一加速/减速阶段阈值速度(无符号)
    uint32_t amax ;		//V1 和 VMAX 之间的加速度
    uint32_t vmax;			//运动斜坡目标速度(位置模式确保VMAX≥VSTART )
    uint32_t dmax;		 	//VMAX 和 V1 之间的减速度
    uint32_t d1;				//V1 和 VSTOP 之间的减速度
    uint32_t vstop;		//停止速度（接近零）
}ramp_reg;

typedef enum {false = 0, true = !false} mybool; 

typedef struct
{
	__IO int target_pos_i; //目标位置 int
  __IO int cur_pos_i; //当前位置 int
  __IO float pitch; //螺距
	__IO float resolution;//分辨率
	__IO float permm;//每mm 脉冲个数
	__IO float target_pos_f; //目标位置 float
	__IO float cur_pos_f; //当前位置 float
	__IO enum limitenum limitflag;//限位
  __IO mybool busyflag;//繁忙标志
//	__IO char busyflag;//繁忙标志
	__IO int timeout; //超时时间
	__IO unsigned int timeoutcounter; //超时计数
	__IO int speed;
	__IO int max_pulse_num;
	__IO mybool zeroing;//回零中
	__IO mybool working;//是否判断到位
	__IO mybool es;//急停标志
}Axis_t;


typedef struct
{
	uint16_t	microstep;//1-256细分
	uint8_t		run_current;//1-31
	uint8_t		hold_current;//1-31
	ramp_reg	motorcurve; //曲线加速结构体
	Axis_t  Axis; //电机参数结构体
}tmc5160; 
extern tmc5160 MotorTmc5160[TotalAxis];

//extern __IO uint32_t motor_counter;//电机超时计数

uint32_t tmc_spi_readregister(uint8_t slave,uint8_t address);//读TMC5160寄存器
uint8_t tmc_spi_writeregister(uint8_t slave,uint8_t address, uint32_t data);//写TMC5160寄存器
void tmc5160_SetPara(tmc5160 *config,uint16_t micro,uint8_t hold,uint8_t run,\
uint32_t vstart,uint32_t a1,uint32_t v1,uint32_t amax,uint32_t vmax,\
uint32_t dmax,uint32_t d1,uint32_t vstop,uint8_t slave);//TMC5160参数配置 只设置 需要tmc5160_InitConfig初始化才能生效
void tmc5160_InitConfig(tmc5160 *config , uint8_t slave);//TMC5160参数初始化
void Axis_init(void);//常用参数初始化 主要配置螺距pitch和超时时间
void scan_limitstatus(uint8_t  motor);//限位扫描
void motor_runstep(uint8_t motor,int step);//运行一个脉冲
void motor_runlap(uint8_t motor,int lap) ;//运行一圈
void motor_runpos(uint8_t motor,int pos) ;//运行到绝对位置
void motor_zeroing(uint8_t  motor,uint8_t dir,uint8_t wait);//回零
void scan_motor_zeroing(uint8_t  motor,uint8_t dir);//nowait模式下电机回零浏览
void motor_run_mm(uint8_t motor,float mm,u8 posmode,uint8_t wait);//电机运行Nmm距离
void motor_stop(uint8_t motor);//电机立即停止
void motor_runstatus_check(void);//nowait模式下电机运行 定时器判断等待执行到位 发出runmm后需轮训
void motor_wait_run_ok(uint8_t motor);//nowait模式下等待电机执行到位 超时自动退出
void motor_wait_zeroing_ok(uint8_t motor);//nowait模式下等待回零执行到位 超时自动退出
#endif /* TMC5160_H */
