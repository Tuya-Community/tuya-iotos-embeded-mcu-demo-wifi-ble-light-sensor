#ifndef __IO_H
#define __IO_H 		
#include "MY_ST_config.h"
#include "math.h"
#include "stdbool.h"


//IIC_SDA	  PB11
#define IIC_SDA_OUT {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<22);GPIOB->MODER|=1<<22;GPIOB->PUPDR|=1<<22;} 
#define IIC_SDA_IN  {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<22);GPIOB->MODER|=0<<22;} 
#define IIC_SDA_SET GPIOB->ODR|=1<<11
#define IIC_SDA_RESET  GPIOB->ODR&=~(1<<11)
#define IIC_SDA_State ((GPIOB->IDR & 1<<11) == 1<<11)

//IIC_SCL	  PB12
#define IIC_SCL_OUT {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<24);GPIOB->MODER|=1<<24;GPIOB->PUPDR|=1<<24;}  
#define IIC_SCL_IN  {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<24);GPIOB->MODER|=0<<24;} 
#define IIC_SCL_SET GPIOB->ODR|=1<<12
#define IIC_SCL_RESET  GPIOB->ODR&=~(1<<12)
#define IIC_SCL_State ((GPIOB->IDR & 1<<12) == 1<<12)

#if 1
void IIC_Init(void);
void IIC_Start(void);//����IIC��ʼ�ź�
void IIC_Stop(void);//����IICֹͣ�ź�
void IIC_Ack(void);//����ACKӦ��
void IIC_NAck(void);//������ACKӦ��	
uint8_t IIC_Wait_Ack(void);//�ȴ�Ӧ���źŵ���:1,����Ӧ��ʧ��;0,����Ӧ��ɹ�
void IIC_Send_Byte(uint8_t txd);//IIC����һ���ֽ�; �ȷ��͸�λ
uint8_t IIC_Read_Byte(unsigned char ack);//��һ���ֽڣ��ɼ��Ƿ�Ӧ��λ,1��ack��0����ack �Ӹ�λ��ʼ��
uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data);//ֱ��дһ���ֽ�
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);//���ֽ�
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);//��һ��д����ֽ�
#endif

//LED_CTRL4	 	PA5
#define LED_4_OUT {RCC->IOPENR|=1<<0;GPIOA->MODER&=~(3<<10);GPIOA->MODER|=1<<10;} 
#define LED_4_SET GPIOA->ODR|=1<<5
#define LED_4_RESET  GPIOA->ODR&=~(1<<5)
#define LED_4_TOG GPIOA->ODR^=1<<5


#if USING_BH1750FVI/***************************������BH1750FVI���մ�����**************************/
/***************************������BH1750FVI���մ�����**************************/
#if 0		//addr�ߵ�ƽ
#define BHAddWrite     0xB8      //�ӻ���ַ+���д����λ
#define BHAddRead      0xB9      //�ӻ���ַ+��������λ
#else		//addr �͵�ƽ
#define BHAddWrite     0x46      //�ӻ���ַ+���д����λ
#define BHAddRead      0x47      //�ӻ���ַ+��������λ
#endif
#define BHPowDown      0x00      //�ر�ģ��
#define BHPowOn        0x01      //��ģ��ȴ�����ָ��
#define BHReset        0x07      //�������ݼĴ���ֵ��PowerOnģʽ����Ч
#define BHModeH1       0x10      //�����߷ֱ��� ��λ1lx ����ʱ��120ms
#define BHModeH2       0x11      //�߷ֱ���ģʽ2 ��λ0.5lx ����ʱ��120ms
#define BHModeL        0x13      //�ͷֱ��� ��λ4lx ����ʱ��16ms
#define BHSigModeH     0x20      //һ�θ߷ֱ��� ���� ������ģ��ת�� PowerDownģʽ
#define BHSigModeH2    0x21      //ͬ������
#define BHSigModeL     0x23      // ������

#define LITTLE_ENDIAN2
typedef union 
{
  uint16_t u16;               // element specifier for accessing whole i16
  struct {
    #ifdef LITTLE_ENDIAN2  // Byte-order is little endian
    uint8_t u8L;              // element specifier for accessing low u8
    uint8_t u8H;              // element specifier for accessing high u8
    #else                 // Byte-order is big endian
    uint8_t u8H;              // element specifier for accessing low u8
    uint8_t u8L;              // element specifier for accessing high u8
    #endif
  } s16;	
}BH1750FVI_DATA;
/***************************������BH1750FVI���մ�����**************************/
#endif
extern uint8_t F_TASK_BH1750FVI;
void TASK_BH1750FVI(void);


struct ctrl_state
{
	bool flagmax;//�ﵽ�趨ֵ���ޱ�־λ
	bool flagmin;//�ﵽ�趨ֵ���ޱ�־λ
	uint8_t mode;//����ģʽ
	uint16_t range;//�趨��ֵ
	int16_t ctrl;//��������ֵ	
	float now;//��ǰ���ֵ
	float set;//�趨���ֵ	
};
void IO_Init(void);
#endif

