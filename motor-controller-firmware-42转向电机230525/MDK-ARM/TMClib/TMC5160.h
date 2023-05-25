/*
 * TMC5160.h
 */

#ifndef TMC5160_H
#define TMC5160_H

//#include "gd32f10x.h"
//#include "stdint_h"

#define TotalAxis	5 //����������

#define motor1	0x00
#define motor2	0x01
#define motor3	0x02
#define motor4	0x03
#define motor5	0x04


#define forwarddir	0x00	//����
#define reversedir	0x01	//����
//run mm �����λ�û��Ǿ���λ��
#define relativepos	0x00	//���
#define absolutepos	0x01	//����

#define nowait			0x00	//���ȴ�
#define timeoutwait	0x01	//��ʱ�ȴ�


#define low		0x00
#define high 	0xFF
 
#define  tmc_fclk   1.048576 //�ٶȵ�ʱ��ο�: t = 2^24 / fCLK
//#define  Seedp_conversion  51200*TMC_fCLK
//#define  speed_conversion(slave)  tmc5160config[slave].microstep*200*tmc_fclk
#define  everystep  200*1000
//#define  SingNumber(slave,sing)   (tmc5160config[slave].microstep*200*sing)    //Ȧ��    
//#define  StepNumber(slave,step)   (tmc5160config[slave].microstep*step)        //����



#define REGISTER_COUNT 128
	
#define SET_IHOLD(a)		  ((a & 0x1F)<<0)
#define SET_IRUN(a)		  	((a & 0x1F)<<8)
#define SET_IHOLDDELAY(a)	((a & 0xF)<<16)
	
	
	
	
/*
������Ͷ��忪ʼ
*/
//typedef enum {false = 0, true = !false} bool; 
enum limitenum {
    limit_nomal,
		limit_left,
		limit_right	
};

typedef struct{
    uint32_t vstart;		//�����ٶ�VSTOP��VSTART
    uint32_t a1;       //VSTART �� V1 ֮��ļ��ٶ�
    uint32_t v1;				//��һ����/���ٽ׶���ֵ�ٶ�(�޷���)
    uint32_t amax ;		//V1 �� VMAX ֮��ļ��ٶ�
    uint32_t vmax;			//�˶�б��Ŀ���ٶ�(λ��ģʽȷ��VMAX��VSTART )
    uint32_t dmax;		 	//VMAX �� V1 ֮��ļ��ٶ�
    uint32_t d1;				//V1 �� VSTOP ֮��ļ��ٶ�
    uint32_t vstop;		//ֹͣ�ٶȣ��ӽ��㣩
}ramp_reg;

typedef enum {false = 0, true = !false} mybool; 

typedef struct
{
	__IO int target_pos_i; //Ŀ��λ�� int
  __IO int cur_pos_i; //��ǰλ�� int
  __IO float pitch; //�ݾ�
	__IO float resolution;//�ֱ���
	__IO float permm;//ÿmm �������
	__IO float target_pos_f; //Ŀ��λ�� float
	__IO float cur_pos_f; //��ǰλ�� float
	__IO enum limitenum limitflag;//��λ
  __IO mybool busyflag;//��æ��־
//	__IO char busyflag;//��æ��־
	__IO int timeout; //��ʱʱ��
	__IO unsigned int timeoutcounter; //��ʱ����
	__IO int speed;
	__IO int max_pulse_num;
	__IO mybool zeroing;//������
	__IO mybool working;//�Ƿ��жϵ�λ
	__IO mybool es;//��ͣ��־
}Axis_t;


typedef struct
{
	uint16_t	microstep;//1-256ϸ��
	uint8_t		run_current;//1-31
	uint8_t		hold_current;//1-31
	ramp_reg	motorcurve; //���߼��ٽṹ��
	Axis_t  Axis; //��������ṹ��
}tmc5160; 
extern tmc5160 MotorTmc5160[TotalAxis];

//extern __IO uint32_t motor_counter;//�����ʱ����

uint32_t tmc_spi_readregister(uint8_t slave,uint8_t address);//��TMC5160�Ĵ���
uint8_t tmc_spi_writeregister(uint8_t slave,uint8_t address, uint32_t data);//дTMC5160�Ĵ���
void tmc5160_SetPara(tmc5160 *config,uint16_t micro,uint8_t hold,uint8_t run,\
uint32_t vstart,uint32_t a1,uint32_t v1,uint32_t amax,uint32_t vmax,\
uint32_t dmax,uint32_t d1,uint32_t vstop,uint8_t slave);//TMC5160�������� ֻ���� ��Ҫtmc5160_InitConfig��ʼ��������Ч
void tmc5160_InitConfig(tmc5160 *config , uint8_t slave);//TMC5160������ʼ��
void Axis_init(void);//���ò�����ʼ�� ��Ҫ�����ݾ�pitch�ͳ�ʱʱ��
void scan_limitstatus(uint8_t  motor);//��λɨ��
void motor_runstep(uint8_t motor,int step);//����һ������
void motor_runlap(uint8_t motor,int lap) ;//����һȦ
void motor_runpos(uint8_t motor,int pos) ;//���е�����λ��
void motor_zeroing(uint8_t  motor,uint8_t dir,uint8_t wait);//����
void scan_motor_zeroing(uint8_t  motor,uint8_t dir);//nowaitģʽ�µ���������
void motor_run_mm(uint8_t motor,float mm,u8 posmode,uint8_t wait);//�������Nmm����
void motor_stop(uint8_t motor);//�������ֹͣ
void motor_runstatus_check(void);//nowaitģʽ�µ������ ��ʱ���жϵȴ�ִ�е�λ ����runmm������ѵ
void motor_wait_run_ok(uint8_t motor);//nowaitģʽ�µȴ����ִ�е�λ ��ʱ�Զ��˳�
void motor_wait_zeroing_ok(uint8_t motor);//nowaitģʽ�µȴ�����ִ�е�λ ��ʱ�Զ��˳�
#endif /* TMC5160_H */
