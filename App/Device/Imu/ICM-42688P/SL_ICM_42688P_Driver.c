
#include "SL_ICM_42688P_Driver.h"
#include "common.h"
#include <string.h>

#if SL_Sensor_Algo_Release_Enable==0x00
#include "log.h"
#endif


//I2C SPI选择
#define SL_ICM_42688P_SPI_EN_I2C_DISABLE   0x01 //必须和SL_SPI_IIC_INTERFACE相反；
#define SL_SPI_IIC_INTERFACE           	!SL_ICM_42688P_SPI_EN_I2C_DISABLE //必须和SL_ICM_42688P_SPI_EN_I2C_DISABLE 相反；
//原始数据高通输出使能控制宏定义
#define SL_ICM_42688P_RAWDATA_HPF_ENABLE   0x00
//中断脚默认输出电平控制宏定义
#define SL_ICM_42688P_INT_DEFAULT_LEVEL    0x00
//SDO  上拉电阻控制
#define SL_ICM_42688P_SDO_PullUP_ENABLE    0x01
//AOI唤醒中断检测使能 
#define SL_ICM_42688P_AOI_Wake_Up_ENABLE   0x00

//FIFO_STREAM使能//FIFO_WTM使能
#define SL_ICM_42688P_FIFO_STREAM_WTM  	   0x01//0X00=STREAM MODE 0X01=FIFO MODE

int sensor_status=0;
static void sl_delay(unsigned char sl_i)
{
	delay_us(sl_i * 50);
}


unsigned char SL_ICM_42688P_Check(void)
{
	unsigned char reg_value=0;
	
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7F, 0x00);//goto 0x00
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, ICM_42688P_WHO_AM_I, 1, &reg_value);
#if SL_Sensor_Algo_Release_Enable==0x00
	LogI("0x%x=0x%x\r\n",ICM_42688P_WHO_AM_I,reg_value);
#endif
	if(reg_value==0x6A)
		return 0x01;//ICM_42688P
	else
		return 0x00;//其他芯片
}

unsigned char SL_ICM_42688P_Config(void)
{
	unsigned char Check_Flag=0;
	unsigned char reg_value=0;
	
	while(1)
	{
		if (Check_Flag=SL_ICM_42688P_Check())
		{
			break;
		}
		sl_delay(1000 * 20);
	}
//	Check_Flag=1;//强制初始化
#if SL_Sensor_Algo_Release_Enable==0x00
	LogI( "SL_ICM_42688P_Check=0x%x\r\n",Check_Flag);
#endif	
	if(Check_Flag==1)
	{
		Check_Flag= SL_ICM_42688P_POWER_DOWN();
		sl_delay(10);
	}
#if SL_Sensor_Algo_Release_Enable==0x00
	LogI( "SL_ICM_42688P_POWER_DOWN=0x%x\r\n",Check_Flag);
#endif
	if(Check_Flag==1)
	{
		Check_Flag= SL_ICM_42688P_SOFT_RESET();
		sl_delay(10);
	}
#if SL_Sensor_Algo_Release_Enable==0x00
	LogI( "SL_ICM_42688P_SOFT_RESET=0x%x\r\n",Check_Flag);
#endif
	if(Check_Flag==1)
	{
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7F, 0x00);//goto 0x00
		sl_delay(1);		
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7D, 0x0E);//PWR_CTRL ENABLE ACC+GYR+TEMP
		sl_delay(200);//10ms
#if SL_SPI_IIC_INTERFACE==0x00 //SPI
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7F, 0x00);//goto 0x90
		sl_delay(1);
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x4A, 0x66);
		sl_delay(1);
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7F, 0x83);//goto 0x6F
		sl_delay(1);
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x6F, 0x04);//I2C disable
		sl_delay(1);
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7F, 0x00);//goto 0x6F
		sl_delay(1);
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x4A, 0x00);
		sl_delay(1);
#endif		
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x40, 0x8B);//ACC_CONF 0x08=100Hz  0x0B=800Hz
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x41, 0x01);//ACC_RANGE  ±4G
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x42, 0xCB);//GYR_CONF 0x08=100Hz  0x0B=800Hz
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x43, 0x00);//GYR_RANGE 2000dps
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x05, 0x50);//COM_CFG
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x08, 0x05);//Enable DLPF		

#if SL_ICM_42688P_RAWDATA_HPF_ENABLE ==0x01
		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x08, 1, &reg_value);
		reg_value=reg_value|0xA0;
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x08, reg_value);//HPF_CFG  rawdata hpf
#endif

#if SL_ICM_42688P_AOI_Wake_Up_ENABLE==0x01
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x06, 0x0B);//AOI1-INT1
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x30, 0x2A);//XYZ-ENABLE
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x32, 0x05);//VTH
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x33, 0x02);//TTH
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x3F, 0x10);//HPF FOR AOI1
#endif

		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x06, 1, &reg_value);
#if SL_ICM_42688P_INT_DEFAULT_LEVEL ==0x01
		reg_value=reg_value|0x20;
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x06, reg_value);//defalut high level&& push-pull
#else
		reg_value=reg_value&0xDF;
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x06, reg_value);//defalut low  level&& push-pull	
#endif

		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x05, 1, &reg_value);		
#if SL_ICM_42688P_FIFO_ENABLE==0x01
		reg_value=reg_value&0xEF;//addr_add_auto disable
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x05, reg_value);

		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1C, 0x00);//FIFO_disable
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1C, 0x07);//FIFO_EN //FIFO_Header_Code enable
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1D, 0x00);//fifo:By-Pass mode

		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x1D, 1, &reg_value);
#if SL_ICM_42688P_FIFO_STREAM_WTM==0x00//STREAM
		reg_value=reg_value&0xCF;
		reg_value=reg_value|0x20;//fifo stream mode enable
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1D, reg_value);//fifo stream mode set
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1E, 0xC1);//fifo fth set
		sensor_status=0;
#else //WTM
		reg_value=reg_value&0xCF;
		reg_value=reg_value|0x10;//fifo mode enable
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x06, 0x09);//FIFO_WTM:INT1
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1D, reg_value);//fifo mode set
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1E, 0xC1);//fifo fth set 0XC2=194=(194-2)/6
#endif

#else
		reg_value=reg_value|0x10;//addr_add_auto enable
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x05, reg_value);	
#endif

#if SL_ICM_42688P_SDO_PullUP_ENABLE ==0x01
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE,0x7F, 0x8C);//goto 0x8C
		sl_delay(1);
		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x30, 1, &reg_value);
		reg_value=reg_value&0xFE;//CS PullUP_enable
		reg_value=reg_value&0xFD;//SDO PullUP_enable
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x30, reg_value);

		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE,0x7F, 0x00);//goto 0x00
		sl_delay(1);
#else
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE,0x7F, 0x8C);//goto 0x8C
		sl_delay(1);
		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x30, 1, &reg_value);
		reg_value=reg_value&0xFE;//CS PullUP_enable
		reg_value=reg_value|0x02;//SDO PullUP_disable	
		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x30, reg_value);

		SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE,0x7F, 0x00);//goto 0x00
		sl_delay(1);
#endif

		return 1;
	}
	else
		return 0;
}


//读取时间戳
unsigned int SL_ICM_42688P_TimeStamp_Read(void)
{
	unsigned char  time_data[3];
	unsigned int time_stamp;
	
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x18, 1, &time_data[0]);
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x19, 1, &time_data[1]);
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x20, 1, &time_data[2]);

	time_stamp=(unsigned int)(time_data[0]<<16|time_data[1]<<8|time_data[2]);
	
	return time_stamp;
}

#if SL_ICM_42688P_FIFO_ENABLE ==0x00
int SL_ICM_42688P_RawData_Read(signed short * acc_data_buf,signed short * gyr_data_buf)
{
	unsigned char  raw_data[12];
	unsigned char  drdy_satus;
	unsigned short drdy_cnt=0;
	
	while((drdy_satus&0x03) != 0x03)
	{
		drdy_satus=0x00;
		sl_delay(1);
		if (1 != SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x0B, 1, &drdy_satus))
		{
		}
		drdy_cnt++;
		if(drdy_cnt>40) break;
	}

	if ((drdy_satus&0x03) != 0x03)
	{
		return -1;
	}

	if (HAL_OK != SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x0C, 12, &raw_data[0]))
	{
		return 1;
	}

	acc_data_buf[0] =(signed short)((((unsigned char)raw_data[0])* 256)  + ((unsigned char)raw_data[1]));//ACCX-16位
	acc_data_buf[1] =(signed short)((((unsigned char)raw_data[2])* 256)  + ((unsigned char)raw_data[3]));//ACCY-16位
	acc_data_buf[2] =(signed short)((((unsigned char)raw_data[4])* 256)  + ((unsigned char)raw_data[5]));//ACCZ-16位
	gyr_data_buf[0] =(signed short)((((unsigned char)raw_data[6])* 256)  + ((unsigned char)raw_data[7]));//GYRX-16位
	gyr_data_buf[1] =(signed short)((((unsigned char)raw_data[8])* 256)  + ((unsigned char)raw_data[9]));//GYRY-16位
	gyr_data_buf[2] =(signed short)((((unsigned char)raw_data[10])* 256) + ((unsigned char)raw_data[11]));//GYRZ-16位
#if SL_Sensor_Algo_Release_Enable==0x00
//	USART_printf( USART1, "RawData:AX=%d,AY=%d,AZ=%d,GX=%d,GY=%d,GZ=%d\r\n",acc_data_buf[0],acc_data_buf[1],acc_data_buf[2],gyr_data_buf[0],gyr_data_buf[1],gyr_data_buf[2]);
#endif 
	return 0;
}
#else

#if SL_Sensor_Algo_Release_Enable==0x00
#define SL_ICM_42688P_WAIT_FIFO_LEN_ENABLE 0x00//0x01
#else
#define SL_ICM_42688P_WAIT_FIFO_LEN_ENABLE 0x00
#endif
unsigned char  SL_ICM_42688P_FIFO_DATA[2048];
unsigned char  SL_Header_code_gyrx;
unsigned char  SL_Header_code_gyry;
unsigned char  SL_Header_code_gyrz;
unsigned char  SL_Header_code_accx;
unsigned char  SL_Header_code_accy;
unsigned char  SL_Header_code_accz;
unsigned short SL_ICM_42688P_FIFO_Read(signed short *accx_buf,signed short *accy_buf,signed short *accz_buf,signed short *gyrx_buf,signed short *gyry_buf,signed short *gyrz_buf)
{
	unsigned char  fifo_num1=0;
	unsigned char  fifo_num2=0;
	unsigned short fifo_num=0;
	unsigned short fifo_len=0;
	unsigned short fifo_wait=0;	
	unsigned short j;
	signed short   ag_buf[6];
	
#if SL_Sensor_Algo_Release_Enable==0x00	//user can set to zero
#if SL_ICM_42688P_WAIT_FIFO_LEN_ENABLE==0x00
	while((fifo_num1&0x20)!=0x20)
	{
		sl_delay(200);
		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x1F,1,&fifo_num1);
	}
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x1F,1,&fifo_num1);
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x20,1,&fifo_num2);
	if((fifo_num1&0x10)==0x10)
	{
		fifo_num=2048;
	}
	else
	{
		fifo_num=(fifo_num1&0x0F)*256+fifo_num2;
	}
#else
	while(fifo_num2<194)//32
	{
		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x1F,1,&fifo_num1);
		SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x20,1,&fifo_num2);
		sl_delay(20);
		fifo_wait++;
		if(fifo_wait>30000) break;
	}
	fifo_wait=0;
	fifo_num=fifo_num2;
#endif
	
#else
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x1F,1,&fifo_num1);
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x20,1,&fifo_num2);
	if((fifo_num1&0x10)==0x10)
	{
		fifo_num=2048;
	}
	else
	{
		fifo_num=(fifo_num1&0x0F)*256+fifo_num2;
	}
#endif
	
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x21, fifo_num*2, SL_ICM_42688P_FIFO_DATA);//单地址连续读取 BYTE NUM
	
#if SL_Sensor_Algo_Release_Enable==0x00
//	USART_printf(USART1,"0x1F:0x%x 0x20:0x%x\n",fifo_num1,fifo_num2);
//	USART_printf(USART1,"ICM_42688P_FIFO_NUM1:%d\n",fifo_num);
#endif
	fifo_len=0;
	for(j=4;(j+11)<fifo_num*2;)//194
	{
		SL_Header_code_gyrx =((unsigned char)SL_ICM_42688P_FIFO_DATA[j]&0xE0)>>5;
		SL_Header_code_gyry =((unsigned char)SL_ICM_42688P_FIFO_DATA[j+2]&0xE0)>>5;
		SL_Header_code_gyrz =((unsigned char)SL_ICM_42688P_FIFO_DATA[j+4]&0xE0)>>5;
		SL_Header_code_accx =((unsigned char)SL_ICM_42688P_FIFO_DATA[j+6]&0xE0)>>5;
		SL_Header_code_accy =((unsigned char)SL_ICM_42688P_FIFO_DATA[j+8]&0xE0)>>5;
		SL_Header_code_accz =((unsigned char)SL_ICM_42688P_FIFO_DATA[j+10]&0xE0)>>5;
		if((SL_Header_code_gyrx==5)&&(SL_Header_code_gyry==6)&&(SL_Header_code_gyrz==7)&&(SL_Header_code_accx==2)&&(SL_Header_code_accy==3)&&(SL_Header_code_accz==4))
		{
			gyrx_buf[fifo_len] =(signed short)((((unsigned char)SL_ICM_42688P_FIFO_DATA[j]&0x1F)   * 256 ) + (unsigned char)SL_ICM_42688P_FIFO_DATA[j+1])<<3;//13位
			gyry_buf[fifo_len] =(signed short)((((unsigned char)SL_ICM_42688P_FIFO_DATA[j+2]&0x1F) * 256 ) + (unsigned char)SL_ICM_42688P_FIFO_DATA[j+3])<<3;//13位
			gyrz_buf[fifo_len] =(signed short)((((unsigned char)SL_ICM_42688P_FIFO_DATA[j+4]&0x1F) * 256 ) + (unsigned char)SL_ICM_42688P_FIFO_DATA[j+5])<<3;//13位
			accx_buf[fifo_len] =(signed short)((((unsigned char)SL_ICM_42688P_FIFO_DATA[j+6]&0x1F) * 256 ) + (unsigned char)SL_ICM_42688P_FIFO_DATA[j+7])<<3;//13位
			accy_buf[fifo_len] =(signed short)((((unsigned char)SL_ICM_42688P_FIFO_DATA[j+8]&0x1F) * 256 ) + (unsigned char)SL_ICM_42688P_FIFO_DATA[j+9])<<3;//13位
			accz_buf[fifo_len] =(signed short)((((unsigned char)SL_ICM_42688P_FIFO_DATA[j+10]&0x1F)* 256 ) + (unsigned char)SL_ICM_42688P_FIFO_DATA[j+11])<<3;//13位
#if SL_Sensor_Algo_Release_Enable==0x00
//			USART_printf(USART1,"#1:%x #2:%x #3:%x #4:%x #5:%x #6:%x\r\n",SL_ICM_42688P_FIFO_DATA[j*12+10],SL_ICM_42688P_FIFO_DATA[j*12+11],SL_ICM_42688P_FIFO_DATA[j*12+12],SL_ICM_42688P_FIFO_DATA[j*12+13],SL_ICM_42688P_FIFO_DATA[j*12+14],SL_ICM_42688P_FIFO_DATA[j*12+15]);	
			USART_printf(USART1,"1F=%x,20=%x,Num:%d,Header:%x AX:%d,Header:%x AY:%d,Header:%x AZ:%d,Header:%x GX:%d,Header:%x GY:%d,Header:%x GZ:%d\r\n",fifo_num1,fifo_num2,fifo_len,SL_Header_code_accx,accx_buf[fifo_len],SL_Header_code_accy,accy_buf[fifo_len],SL_Header_code_accz,accz_buf[fifo_len],SL_Header_code_gyrx,gyrx_buf[fifo_len],SL_Header_code_gyry,gyry_buf[fifo_len],SL_Header_code_gyrz,gyrz_buf[fifo_len]);
#endif
			fifo_len++;
			j=j+12;
		}
		else
		{
			j=j+2;
		}
		
		if(SL_Header_code_gyrz!=7)
		{
#if SL_Sensor_Algo_Release_Enable==0x00	
//			USART_printf(USART1,"error #1:%x #2:%x #3:%x #4:%x #5:%x #6:%x\r\n",SL_ICM_42688P_FIFO_DATA[j*12+10],SL_ICM_42688P_FIFO_DATA[j*12+11],SL_ICM_42688P_FIFO_DATA[j*12+12],SL_ICM_42688P_FIFO_DATA[j*12+13],SL_ICM_42688P_FIFO_DATA[j*12+14],SL_ICM_42688P_FIFO_DATA[j*12+15]);	
//			USART_printf(USART1,"error 1F=%x,20=%x,Num:%d,AXHeader:%x,AYHeader:%x,AZHeader:%x,GXHeader:%x,GYHeader:%x,GZHeader:%x\r\n",fifo_num1,fifo_num2,j,SL_Header_code_accx,SL_Header_code_accy,SL_Header_code_accz,SL_Header_code_gyrx,SL_Header_code_gyry,SL_Header_code_gyrz);
#endif
		}	
	}
	

#if SL_ICM_42688P_FIFO_STREAM_WTM==0x00
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1C, 0x00);//FIFO_disable
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1C, 0x07);//FIFO_EN //FIFO_Header_Code enable	
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1D, 0x00);//BY PASS MODE
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1D, 0x20);//Stream MODE
#else
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1C, 0x00);//FIFO_disable
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1C, 0x07);//FIFO_EN //FIFO_Header_Code enable	
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1D, 0x00);//BY PASS MODE
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x1D, 0x10);//FIFO MODE

#endif

	return fifo_len;
}
#endif


unsigned char SL_ICM_42688P_POWER_DOWN(void)
{
	unsigned char SL_Read_Reg  = 0xff;
	
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7F, 0x00);//goto 0x00
	sl_delay(20);
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7D, 0x00);//POWER DOWN
	sl_delay(200);
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x7D, 1,&SL_Read_Reg);
	if(SL_Read_Reg==0x00)   return  1;
	else                    return  0;
}


unsigned char SL_ICM_42688P_SOFT_RESET(void)
{
	unsigned char SL_Read_Reg  = 0xff;
	
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x7F, 0x00);//goto 0x00
	sl_delay(20);
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x08, 0x87);//FLAG
	sl_delay(10);
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x05, 0x80);//BOOT
	SL_ICM_42688P_I2c_Spi_Write(SL_SPI_IIC_INTERFACE, 0x4A, 0xA5);//SOFT_RESET
	sl_delay(200);
	SL_ICM_42688P_I2c_Spi_Read(SL_SPI_IIC_INTERFACE, 0x08, 1,&SL_Read_Reg);
	if(SL_Read_Reg==0x80)   return  1;
	else                    return  0;
	
}


unsigned char SL_ICM_42688P_I2c_Spi_Write(unsigned char sl_spi_iic, unsigned char reg, unsigned char dat)
{
	if(sl_spi_iic)
	{
#if 0
		if (HAL_I2C_Mem_Write(&hi2c1, SL_ICM_42688P_IIC_8BITS_WRITE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &dat, 1, 2) != HAL_OK)
		{
		  LogE("SL_ICM_42688P_I2c_Spi_Write error");
		  return -1;
		}
#endif
	}
	else
	{
		//SPI_SENSOR_WriteReg(reg,dat);
		HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET); // 拉低片选
		delay_us(1);
		uint8_t tx_buff[51];
		uint8_t rx_buff[51];
		tx_buff[0] = reg;
		tx_buff[1] = dat;
		HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi4, tx_buff, rx_buff, 2, 2);
		if (ret != HAL_OK)
		{
		  LogE("ICM_42688P_Write_Reg error: %x", ret);
		}
		HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET); // 释放片选
		return ret;
	}
	return 1;
}
unsigned char SL_ICM_42688P_I2c_Spi_Read(unsigned char sl_spi_iic, unsigned char reg, unsigned short len, unsigned char* buf)
{
	if(sl_spi_iic)
	{
#if 0
		if (HAL_I2C_Mem_Read(&hi2c1, SL_ICM_42688P_IIC_8BITS_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 2) != HAL_OK)
		{
		  LogE("SL_ICM_42688P_I2c_Spi_Read error");
		  return -1;
		}
#endif
	}
	else
	{
		//SPI_SENSOR_ReadBuf(reg|0x80,buf,len);//SC7A20E SC7A20H
		reg |= 0x80; // 设置读标志位（MPU6500要求）

		if (len > 50)
		{
		  LogE("ICM_42688P_Read_Reg len error: %d", len);
		  return HAL_ERROR;
		}

		HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET); // 拉低片选
		delay_us(1);
		uint8_t tx_buff[51];
		uint8_t rx_buff[51];
		tx_buff[0] = reg;
		HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi4, tx_buff, rx_buff, len+1, 2);
		HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET); // 释放片选
		if (ret == HAL_OK)
		{
		  memcpy(buf, &rx_buff[1], len);
		}
		else
		{
		  //LogE("ICM_42688P_Read_Reg error: %x", ret);
			return ret;
		}
	}
	return HAL_OK;

}
