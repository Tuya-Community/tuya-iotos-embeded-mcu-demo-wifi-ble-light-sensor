#include "IO.h"
#include "delay.h"
#include "TIM.h"
#include "math.h"
#include "stdlib.h"
// 2�� 1ͨ�� 1����  1��ˮ 1��ˮ 1��ʪ 2��IIC ��ʪ�� ���� 2��ADC  ˮλ ����ʪ��
#if 1//IIC
void IIC_Init(void)
{
	IIC_SCL_OUT;
	IIC_SDA_OUT;
}
void IIC_Start(void)//����IIC��ʼ�ź�
{
	IIC_SDA_OUT;     //sda�����
	IIC_SDA_SET;//IIC_SDA=1;	  	  
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SDA_RESET;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}
void IIC_Stop(void)//����IICֹͣ�ź�
{
	IIC_SDA_OUT;//sda�����
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	delay_us(5);
	IIC_SDA_SET;//IIC_SDA=1;//����I2C���߽����ź�				   	
}
void IIC_Ack(void)//����ACKӦ��
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//������ACKӦ��	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//�ȴ�Ӧ���źŵ���:1,����Ӧ��ʧ��;0,����Ӧ��ɹ�
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA����Ϊ����     
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);	 
	while(IIC_SDA_State)//���SDA�Ƿ���Ϊ�ߵ�ƽ
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_RESET;//IIC_SCL=0;
	return 0;  
} 
void IIC_Send_Byte(uint8_t txd)//IIC����һ���ֽ�; �ȷ��͸�λ
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_SET;//IIC_SCL=1;
		delay_us(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		delay_us(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//��һ���ֽڣ��ɼ��Ƿ�Ӧ��λ,1��ack��0����ack �Ӹ�λ��ʼ��
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		IIC_SCL_RESET;// IIC_SCL=0; 
		delay_us(5);
		IIC_SCL_SET;//IIC_SCL=1;
		receive<<=1;
		if(IIC_SDA_State)
			receive++;   
		delay_us(5); 
	}					 
		if (ack)
			IIC_Ack(); //����ACK
		else
			IIC_NAck();//����nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//ֱ��дһ���ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //�����ֽ�							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	delay_us(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//���ֽ�
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//���͵�ַ	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //�������ģʽ			   
	ret |= IIC_Wait_Ack();
	while(NumToRead)
	{
		if(NumToRead==1)
		{
			*pBuffer=IIC_Read_Byte(0);	
		}
		else
		{
			*pBuffer=IIC_Read_Byte(1);
		}
		pBuffer++;
		NumToRead--;
	}
	IIC_Stop();//����һ��ֹͣ����	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//��һ��д����ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //�����ֽ�							   
		ret |= IIC_Wait_Ack(); 
		pBuffer++;
	}
	IIC_Stop();
	delay_us(10);
	return ret;
}
#endif

/******************************************************����******************************************************/
uint8_t F_TASK_BH1750FVI=0;
#if USING_BH1750FVI//����
uint8_t bh_data_send(uint8_t DrvAddr,uint8_t Opecode)
{
	uint8_t ret=0;
	IIC_Start();	
	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(Opecode);//���͵�ַ	  
	ret |= IIC_Wait_Ack();
	IIC_Stop();
	return ret;
}
uint8_t bh_data_read(uint8_t DrvAddr,uint8_t BH_Mode,BH1750FVI_DATA* BH_DATA)
{
	uint8_t ret=0;
	IIC_Start();	
	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(BH_Mode);//���͵�ַ	  
	ret |= IIC_Wait_Ack();
	IIC_Stop();
	if((BH_Mode & 0x03)==0x03)
		delay_ms(20);
	else
		delay_ms(130);
	
	IIC_Start();	
	IIC_Send_Byte(DrvAddr+1);	    //����д����
	ret |= IIC_Wait_Ack();
	BH_DATA->s16.u8H=IIC_Read_Byte(1);
	BH_DATA->s16.u8L=IIC_Read_Byte(0);
	IIC_Stop();
	return ret;
}
void BH1750FVI_Init(void)
{
	uint8_t error=0;
	error += bh_data_send(BHAddWrite,BHPowOn);
	error += bh_data_send(BHAddWrite,BHReset);
	error += bh_data_send(BHAddWrite,BHModeL);
}

void TASK_BH1750FVI(void)
{
  uint8_t error=0;
	BH1750FVI_DATA BH_DATA;
	uint16_t light;
	error += bh_data_read(BHAddWrite,BHModeH1,&BH_DATA);
	if(error==0)
	{
		light=(uint16_t)(((float)BH_DATA.u16)/1.2);
		//printf("light:%d lx \r\n",light);	
		mcu_dp_value_update(DPID_BRIGHT_VALUE,light);
	}
}
#endif
void Modules_Init(void)
{
	IIC_Init();
	BH1750FVI_Init();
}
void SwitchIO_Init(void)
{
	LED_4_OUT;	
}
void IO_Init(void)
{
	Modules_Init();
	SwitchIO_Init();
}
