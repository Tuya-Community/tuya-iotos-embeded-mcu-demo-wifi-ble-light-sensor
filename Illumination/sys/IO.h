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
void IIC_Start(void);//产生IIC起始信号
void IIC_Stop(void);//产生IIC停止信号
void IIC_Ack(void);//产生ACK应答
void IIC_NAck(void);//不产生ACK应答	
uint8_t IIC_Wait_Ack(void);//等待应答信号到来:1,接收应答失败;0,接收应答成功
void IIC_Send_Byte(uint8_t txd);//IIC发送一个字节; 先发送高位
uint8_t IIC_Read_Byte(unsigned char ack);//读一个字节，可加是否应答位,1加ack，0不加ack 从高位开始读
uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data);//直接写一个字节
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);//读字节
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);//可一次写多个字节
#endif

//LED_CTRL4	 	PA5
#define LED_4_OUT {RCC->IOPENR|=1<<0;GPIOA->MODER&=~(3<<10);GPIOA->MODER|=1<<10;} 
#define LED_4_SET GPIOA->ODR|=1<<5
#define LED_4_RESET  GPIOA->ODR&=~(1<<5)
#define LED_4_TOG GPIOA->ODR^=1<<5


#if USING_BH1750FVI/***************************以下是BH1750FVI光照传感器**************************/
/***************************以下是BH1750FVI光照传感器**************************/
#if 0		//addr高电平
#define BHAddWrite     0xB8      //从机地址+最后写方向位
#define BHAddRead      0xB9      //从机地址+最后读方向位
#else		//addr 低电平
#define BHAddWrite     0x46      //从机地址+最后写方向位
#define BHAddRead      0x47      //从机地址+最后读方向位
#endif
#define BHPowDown      0x00      //关闭模块
#define BHPowOn        0x01      //打开模块等待测量指令
#define BHReset        0x07      //重置数据寄存器值在PowerOn模式下有效
#define BHModeH1       0x10      //连续高分辨率 单位1lx 测量时间120ms
#define BHModeH2       0x11      //高分辨率模式2 单位0.5lx 测量时间120ms
#define BHModeL        0x13      //低分辨率 单位4lx 测量时间16ms
#define BHSigModeH     0x20      //一次高分辨率 测量 测量后模块转到 PowerDown模式
#define BHSigModeH2    0x21      //同上类似
#define BHSigModeL     0x23      // 上类似

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
/***************************以上是BH1750FVI光照传感器**************************/
#endif
extern uint8_t F_TASK_BH1750FVI;
void TASK_BH1750FVI(void);


struct ctrl_state
{
	bool flagmax;//达到设定值上限标志位
	bool flagmin;//达到设定值下限标志位
	uint8_t mode;//控制模式
	uint16_t range;//设定阈值
	int16_t ctrl;//控制输入值	
	float now;//当前输出值
	float set;//设定输出值	
};
void IO_Init(void);
#endif

