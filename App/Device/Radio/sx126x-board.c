/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX126x driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "radio.h"
#include "sx126x/sx126x.h"
#include "main.h"
#include "spi.h"
#include "sx126x-board.h"
#include <string.h>
#include "common.h"
#include "log.h"

#define BOARD_TCXO_WAKEUP_TIME 5
uint8_t gPaOptSetting = 0;

/*!
 * \brief Holds the internal operating mode of the radio
 */


#define DelayMs(x)  HAL_Delay(x)

HAL_StatusTypeDef Radio_SPI_ReadWrite(uint8_t data, uint8_t *rx_data);
HAL_StatusTypeDef Radio_Read_Reg(uint8_t reg, uint8_t * data, uint16_t len);
HAL_StatusTypeDef Radio_Write_Reg(uint8_t reg, const uint8_t * data, uint16_t len);

void ( *Dio1IrqHandler )( void ) = NULL;


void SX126xIoInit( void )
{
    //GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    //GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &DeviceSel, DEVICE_SEL, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

	/*
	GPIO_InitTypeDef gpio_init_struct;

	SX1262_GPIO_CLK_ENABLE();

	gpio_init_struct.Pin = SX1262MOSI_Pin|SX1262SCK_Pin;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SX1262_PortB, &gpio_init_struct);

	gpio_init_struct.Pin = SX1262Nss_Pin;
	HAL_GPIO_Init(SX1262_PortG, &gpio_init_struct);

	gpio_init_struct.Pin = SX1262MISO_Pin;
	gpio_init_struct.Mode = GPIO_MODE_INPUT;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SX1262_PortB, &gpio_init_struct);


	SetSX1262Nss();
	SetSX1262MOSI();
	ClrSX1262SCK();
*/

	//测试spi读写


}









void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    //GpioSetInterrupt( &SX126x.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, dioIrq );
	Dio1IrqHandler = dioIrq;
}

void SX126xIoDeInit( void )
{
    //GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    //GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void SX126xReset( void )
{
	//__disable_irq();
    DelayMs( 10 );
    //GpioInit( &SX126x.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    HAL_GPIO_WritePin(RF_NRST_GPIO_Port, RF_NRST_Pin, GPIO_PIN_RESET);
    DelayMs( 20 );
    //GpioInit( &SX126x.Reset, RADIO_RESET, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 ); // internal pull-up
    HAL_GPIO_WritePin(RF_NRST_GPIO_Port, RF_NRST_Pin, GPIO_PIN_SET);
    DelayMs( 10 );
    //__enable_irq();
}

int SX126xWaitOnBusy( void )
{
	if (HAL_GPIO_ReadPin( RF_BUSY_GPIO_Port, RF_BUSY_Pin ) == GPIO_PIN_SET)
	{
		return 1;
	}
	return 0;

	/*
    //while( GpioRead( &SX126x.BUSY ) == 1 );
	int i = 0;
	while( HAL_GPIO_ReadPin( RF_BUSY_GPIO_Port, RF_BUSY_Pin ) == GPIO_PIN_SET )
	{
		//taskYIELD();
		DelayMs( 1 );
		i++;
		if (i > 100)
		{
			//break;
		}
	}
	*/
}

void SX126xWakeup( void )
{
    //BoardDisableIrq( );

    //GpioWrite( &SX126x.Spi.Nss, 0 );

    //SpiInOut( &SX126x.Spi, RADIO_GET_STATUS );
    //SpiInOut( &SX126x.Spi, 0x00 );

    //GpioWrite( &SX126x.Spi.Nss, 1 );

    //__disable_irq();

	uint8_t tx_data[2];
	uint8_t rx_data[2];

	tx_data[0] = ( uint8_t )RADIO_GET_STATUS;

	ClrSX1262Nss();

	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1); // 超时1ms
	delay_us(10);

	SetSX1262Nss();

	if (ret != HAL_OK)
	{
		//异常处理
	}

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    //__enable_irq();

    //BoardEnableIrq( );
}

bool SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint32_t wait_tick = HAL_GetTick();
    while(SX126xWaitOnBusy() != 0)
    {
    	if (HAL_GetTick() > (wait_tick + 1))
    	{
    		break;
    	}
    }
    if (SX126xWaitOnBusy( ))
    {
    	LogW("SX126xWriteCommand Busy, command: %x", command);
    	return false;
    }
    if (size > 32)
    {
    	return false;
    }
/*
    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, ( uint8_t )command );

    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }

    GpioWrite( &SX126x.Spi.Nss, 1 );
*/

	uint8_t tx_data[64];
	uint8_t rx_data[64];

	tx_data[0] = ( uint8_t )command;
	memcpy(&tx_data[1], buffer, size);

	ClrSX1262Nss();
	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size+1, 2); // 超时1ms
	delay_us(10);
	SetSX1262Nss();
	if (ret != HAL_OK)
	{
		LogW("SX126xWriteCommand fail, command: %x, ret:%x", command, ret);
		//异常处理
		return false;
	}
	if( command != RADIO_SET_SLEEP )
	{
		SX126xWaitOnBusy( );
	}
	return true;
}



bool SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    uint32_t wait_tick = HAL_GetTick();
    while(SX126xWaitOnBusy() != 0)
    {
    	if (HAL_GetTick() > (wait_tick + 1))
    	{
    		break;
    	}
    }
    if (SX126xWaitOnBusy( ))
    {
    	LogW("SX126xReadCommand Busy, command: %x", command);
    	return false;
    }
    if (size > 32)
    {
    	return false;
    }
/*
    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, ( uint8_t )command );
    SpiInOut( &SX126x.Spi, 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }

    GpioWrite( &SX126x.Spi.Nss, 1 );
*/

    bool ret = false;

	uint8_t tx_data[64];
	uint8_t rx_data[64];
	tx_data[0] = ( uint8_t )command;
	tx_data[1] = 0;

  	ClrSX1262Nss();
  	delay_us(10);

	if( command == RADIO_GET_STATUS )
	{
		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 2); // 超时1ms
		if (status == HAL_OK)
		{
			buffer[0] = rx_data[1];
			ret = true;
		}
		else
		{
			LogW("SX126xReadCommand fail, command: %x, status: %x", command, status);
			buffer[0] = 0;
		}
	}
	else
	{
		HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size+2, 2); // 超时1ms
		if (status == HAL_OK)
		{
			memcpy(buffer, &rx_data[2], size);
			ret = true;
		}
		else
		{
			LogW("SX126xReadCommand fail, command: %x, status: %x", command, status);
			memset(buffer, 0, size);
		}
	}

  	delay_us(10);
  	SetSX1262Nss();

  	SX126xWaitOnBusy( );

  	return ret;
}

bool SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint32_t wait_tick = HAL_GetTick();
    while(SX126xWaitOnBusy() != 0)
    {
    	if (HAL_GetTick() > (wait_tick + 1))
    	{
    		break;
    	}
    }
    if (SX126xWaitOnBusy( ))
    {
    	LogW("SX126xWriteRegisters Busy, address: %x", address);
    	return false;
    }
    if (size > 32)
    {
    	return false;
    }
/*
    GpioWrite( &SX126x.Spi.Nss, 0 );
    
    SpiInOut( &SX126x.Spi, RADIO_WRITE_REGISTER );
    SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    SpiInOut( &SX126x.Spi, address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }

    GpioWrite( &SX126x.Spi.Nss, 1 );
*/
	uint8_t tx_data[64];
	uint8_t rx_data[64];
	tx_data[0] = RADIO_WRITE_REGISTER;
	tx_data[1] = ( address & 0xFF00 ) >> 8;
	tx_data[2] = address & 0x00FF;
	memcpy(&tx_data[3], buffer, size);

  	ClrSX1262Nss();

  	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size+3, 2); // 超时1ms
  	delay_us(10);

  	SetSX1262Nss();

    SX126xWaitOnBusy( );
    if (ret != HAL_OK)
    {
    	LogW("SX126xWriteRegisters fail, address: %x, ret: %x", address, ret);
    }

    return (ret == HAL_OK);
}

bool SX126xWriteRegister( uint16_t address, uint8_t value )
{
    return SX126xWriteRegisters( address, &value, 1 );
}

bool SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    uint32_t wait_tick = HAL_GetTick();
    while(SX126xWaitOnBusy() != 0)
    {
    	if (HAL_GetTick() > (wait_tick + 1))
    	{
    		break;
    	}
    }
    if (SX126xWaitOnBusy( ))
    {
    	LogW("SX126xReadRegisters Busy, address: %x", address);
    	return false;
    }
    if (size > 32)
    {
    	return false;
    }
/*
    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, RADIO_READ_REGISTER );
    SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    SpiInOut( &SX126x.Spi, address & 0x00FF );
    SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
    GpioWrite( &SX126x.Spi.Nss, 1 );
*/

	uint8_t tx_data[64];
	uint8_t rx_data[64];
	tx_data[0] = RADIO_READ_REGISTER;
	tx_data[1] = ( address & 0xFF00 ) >> 8;
	tx_data[2] = address & 0x00FF;
	tx_data[3] = 0;

  	ClrSX1262Nss();

  	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size+4, 2); // 超时1ms
	if (ret == HAL_OK)
	{
		memcpy(buffer, &rx_data[4], size);
	}
  	delay_us(10);

  	SetSX1262Nss();

    SX126xWaitOnBusy( );

    if (ret != HAL_OK)
    {
    	LogW("SX126xReadRegisters fail, address: %x, ret: %x", address, ret);
    }

    return (ret == HAL_OK);
}

bool SX126xReadRegister( uint16_t address, uint8_t *data)
{
    return SX126xReadRegisters( address, data, 1 );
}

bool SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint32_t wait_tick = HAL_GetTick();
    while(SX126xWaitOnBusy() != 0)
    {
    	if (HAL_GetTick() > (wait_tick + 1))
    	{
    		break;
    	}
    }
    if (SX126xWaitOnBusy( ))
    {
    	LogW("SX126xWriteBuffer Busy, offset: %x", offset);
    	return false;
    }
    if (size > 32)
    {
    	return false;
    }
/*
    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, RADIO_WRITE_BUFFER );
    SpiInOut( &SX126x.Spi, offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }
    GpioWrite( &SX126x.Spi.Nss, 1 );
*/
	uint8_t tx_data[64];
	uint8_t rx_data[64];
	tx_data[0] = RADIO_WRITE_BUFFER;
	tx_data[1] = offset;
	memcpy(&tx_data[2], buffer, size);
  	ClrSX1262Nss();

  	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size+2, 2); // 超时1ms
	if (ret == HAL_OK)
	{
	}
  	delay_us(10);

  	SetSX1262Nss();

    SX126xWaitOnBusy( );
    if (ret != HAL_OK)
    {
    	LogW("SX126xWriteBuffer Fail, offset: %x, ret: %x", offset, ret);
    }
    return (ret == HAL_OK);
}

bool SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    uint32_t wait_tick = HAL_GetTick();
    while(SX126xWaitOnBusy() != 0)
    {
    	if (HAL_GetTick() > (wait_tick + 1))
    	{
    		break;
    	}
    }
    if (SX126xWaitOnBusy( ))
    {
    	LogW("SX126xReadBuffer Busy, offset: %x", offset);
    	return false;
    }
    if (size > 32)
    {
    	return false;
    }
/*
    GpioWrite( &SX126x.Spi.Nss, 0 );

    SpiInOut( &SX126x.Spi, RADIO_READ_BUFFER );
    SpiInOut( &SX126x.Spi, offset );
    SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
    GpioWrite( &SX126x.Spi.Nss, 1 );
*/
	uint8_t tx_data[64];
	uint8_t rx_data[64];
	tx_data[0] = RADIO_READ_BUFFER;
	tx_data[1] = offset;
	tx_data[2] = 0;

  	ClrSX1262Nss();

  	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size+3, 2); // 超时1ms
	if (ret == HAL_OK)
	{
		memcpy(buffer, &rx_data[3], size);
	}
  	delay_us(10);

  	SetSX1262Nss();

    SX126xWaitOnBusy( );
    if (ret != HAL_OK)
    {
    	LogW("SX126xReadBuffer Fail, offset: %x, ret: %x", offset, ret);
    }
    return (ret == HAL_OK);
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetPaSelect( uint32_t channel )
{
    //if( GpioRead( &DeviceSel ) == 1 )
    {
        //return SX1261;
    }
    //else
    {
        return SX1262;
    }
}

void SX126xAntSwOn( void )
{
    //GpioInit( &AntPow, ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SX126xAntSwOff( void )
{
    //GpioInit( &AntPow, ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}






