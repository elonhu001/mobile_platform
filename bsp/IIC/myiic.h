#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys_config.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

//IO��������
#define SDA_IN()  {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=0<<9*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=1<<9*2;} //PB9���ģʽ
//IO��������	 
#define IIC_SCL    PFout(8) //SCL
#define IIC_SDA    PFout(9) //SDA	 
#define READ_SDA   PFin(9)  //����SDA 

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















