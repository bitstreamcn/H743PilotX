/*
 * CRSFRadio.c
 *
 *  Created on: Jun 12, 2025
 *      Author: bitstream
 */
#include "main.h"
#include "CRSFRadio.h"
#include "application.h"
#include "Config.h"

#include <string.h>
#include <cstdio>
//#include "CRSF.h"
#include "common.h"
#include "log.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "../Device/Radio/radio.h"
#include "../Device/Radio/sx126x/sx126x.h"
#include "../Device/Radio/crsf2.h"
#include "../Device/Radio/sx126x-board.h"

#ifdef __cplusplus
}
#endif


#define DelayMs(x)  HAL_Delay(x)


#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             8//14        // dBm 射频芯片输出功率不能大于9dBm，否则烧毁

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              2//2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        6         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               100e3     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           100e3     // Hz
#define FSK_PREAMBLE_LENGTH                         8         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            100
//#define BUFFER_SIZE                                 255 // Define the payload size here


//uint16_t BufferSize = BUFFER_SIZE;
//uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

//static uint32_t lastRX = 0;


/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );


void OnPacketChannels();
void SendFlightMode();
void OnPacketIn();
void OnPacketFlightMode(enum eFlightMode mode);

static int Init();
static void Loop();
static void GetData(int *_Roll, int *_Pitch, int *_Throttle, int *_Yaw, int *_Mode, int *_Light, int *_ReturnHome, int *_Armed);

CRSFRadio_s CRSFRadio = {
		Init,
		Loop,
		GetData
};

uint32_t recvTime = 0;

static int Init()
{
	Log::I("crsf init");
    CrsfSerialEvent2.onPacketChannels = OnPacketChannels;
    CrsfSerialEvent2.onPacketIn = OnPacketIn;
    CrsfSerialEvent2.onPacketFlightMode = OnPacketFlightMode;

    //CrsfInit(CRSF_BAUDRATE);
    crsf2_begin();

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Log::I("radio init");
    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

    Log::I("radio config");
#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band in the compiler options."
#endif


    Log::I("radio test");
    //下面两行是测试模块通信是否正常
    while(SX126xWaitOnBusy() != 0);
    uint16_t s = SX126xGetIrqStatus();
    while(SX126xWaitOnBusy() != 0);
    RadioError_t err = SX126xGetDeviceErrors();
    (void)err;
    (void)s;
    PacketStatus_t pktStatus;
    while(SX126xWaitOnBusy() != 0);
    SX126xGetPacketStatus( &pktStatus );


    HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_SET);

    while(SX126xWaitOnBusy() != 0);
    SX126xClearIrqStatus( IRQ_RADIO_ALL );

    while(SX126xWaitOnBusy() != 0);
    Radio.RxBoosted( RX_TIMEOUT_VALUE );

    Log::I("radio init done");

    recvTime = TimerGetCurrentTime();

    return 0;
}

volatile bool IsConnect = false;
//每隔N个数据包，遥控器发送一个特定的通道值用于反馈，飞控在指定的时间窗口200ms发送反馈数据包
bool need_response_back = false;
uint32_t req_response_tm = 0;

uint8_t lowpwr_times = 0;

static void Loop()
{
	uint32_t now = TimerGetCurrentTime();

	//发送响应包
    if (need_response_back && (now - req_response_tm) > 10)
    {
    	CALC_FUNC_FREQ("Response");
    	static uint8_t times = 0;
    	if (times == 0)
    	{
    		times = 3; //连发2个响应包
    	}
    	if (times > 0)
    	{
    		SendFlightMode();
    		times--;
    	}
    	if (times == 0)
    	{
    		need_response_back = false;
    	}
    }

    switch( State )
    {
    case RX:
    	//收到遥控信号，继续接收
    case TX:
        //发送成功，继续接收
    {
    	/*
        uint32_t wait_tick = HAL_GetTick();
        while(SX126xWaitOnBusy() != 0)
        {
        	if (HAL_GetTick() > (wait_tick + 1))
        	{
        		break;
        	}
        }
        */
    	lowpwr_times = 0;
    	if (SX126xWaitOnBusy() == 0)
    	{
			HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_SET);
			Radio.RxBoosted( RX_TIMEOUT_VALUE );

    		//SendFlightMode();


			State = LOWPOWER;
        }
        break;
    }
    case TX_TIMEOUT:
    case RX_TIMEOUT:
    case RX_ERROR:
        {
        	/*
            uint32_t wait_tick = HAL_GetTick();
            while(SX126xWaitOnBusy() != 0)
            {
            	if (HAL_GetTick() > (wait_tick + 1))
            	{
            		break;
            	}
            }
            */
        	if (SX126xWaitOnBusy() == 0)
        	{
        		HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_SET);
        		Radio.RxBoosted( RX_TIMEOUT_VALUE );
        		//SendFlightMode();
        		State = LOWPOWER;
        	}
        }
        break;
    case LOWPOWER:
    {
  	    const uint32_t TELEM_INTERVAL_MS = 2000U;
  	    uint32_t timeout = now - recvTime;
  	    if (timeout > TELEM_INTERVAL_MS)
  	    {
  	    	static uint32_t reset_time = 0;
  	    	if ((now - reset_time) > TELEM_INTERVAL_MS)
  	    	{
  	    		reset_time = now;
				if (SX126xWaitOnBusy() == 0)
				{
					SX126xClearIrqStatus( IRQ_RADIO_ALL );
				}
				//recvTime = now;
				lowpwr_times++;
				if (lowpwr_times < 3)
				{
					Log::W("LOWPOWER Timeout:%d", timeout);
				}
  				//超过500MS接收不到数据，就重新开启接收
  				State = RX_TIMEOUT;
  	    	}
			//接收不到数据
			IsConnect = false;
  	    }
  	    else
  	    {
		  //发送遥感数据
		  //SendFlightMode();
  	    }
  	    break;
    }
    default:
        // Set low power
        break;
    }

	  /*
    switch( State )
    {
    case RX:
         {
            if( BufferSize > 0 )
            {
                if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                {
                    // Indicates on a LED that the received frame is a PING
                    //GpioToggle( &Led1 );

                    // Send the reply to the PONG string
                    Buffer[0] = 'P';
                    Buffer[1] = 'O';
                    Buffer[2] = 'N';
                    Buffer[3] = 'G';
                    // We fill the buffer with numbers for the payload
                    for(int i = 4; i < BufferSize; i++ )
                    {
                        Buffer[i] = i - 4;
                    }
                    DelayMs( 1 );
                    //HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_RESET);
                    Radio.Send( Buffer, BufferSize );
                }
            }
        }
        State = LOWPOWER;
        break;
    case TX:
        // Indicates on a LED that we have sent a PING [Master]
        // Indicates on a LED that we have sent a PONG [Slave]
        //GpioToggle( &Led2 );
  	  //HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_SET);
        Radio.Rx( RX_TIMEOUT_VALUE );
        State = LOWPOWER;
        break;
    case RX_TIMEOUT:
    case RX_ERROR:
        {
      	  //HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_SET);
            Radio.Rx( RX_TIMEOUT_VALUE );
        }
        State = LOWPOWER;
        break;
    case TX_TIMEOUT:
  	  //HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_SET);
        Radio.Rx( RX_TIMEOUT_VALUE );
        State = LOWPOWER;
        break;
    case LOWPOWER:
    default:
        // Set low power
        break;
    }
		*/
    //TimerLowPowerHandler( );
    // Process Radio IRQ
    static uint32_t irqfun = (uint32_t)Radio.IrqProcess;
    static bool firstlog = true;
    if (firstlog)
    {
    	Log::I("Radio.IrqProcess: %x", (uint32_t)Radio.IrqProcess);
    	firstlog = false;
    }
    if (irqfun != (uint32_t)Radio.IrqProcess)
    {
    	Log::I("Radio.IrqProcess: %x, Old: %x", (uint32_t)Radio.IrqProcess, irqfun);
    }
    Radio.IrqProcess( );

    //CrsfSerial_loop();
    crsf2_update();
}

void OnPacketIn()
{
	//printf("OnPacketIn\r\n");
	recvTime = TimerGetCurrentTime();
	if (!IsConnect)
	{
		IsConnect = true;
	}
	CALC_FUNC_FREQ("OnPacketIn");
}


void SendFlightMode()
{
	enum eFlightMode mode = app.Mode();
	HAL_GPIO_WritePin(RF_RXEN_GPIO_Port, RF_RXEN_Pin, GPIO_PIN_RESET);
	//CrsfSerial_queuePacket(CRSF_FRAMETYPE_FLIGHT_MODE, &mode, sizeof(mode));
	crsf2_queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_FLIGHT_MODE, &mode, sizeof(mode));
}

volatile int Roll = 1500; // 横滚
volatile int Pitch = 1500; // 俯仰（Pitch）
volatile int Throttle = 1500; // 油门（Throttle）
volatile int Yaw = 1500; // 偏航（Yaw）
volatile int Mode = 1000; // 车/无人机 模式
volatile int Light = 1000; // 车灯
volatile int Armed = 1000; // 解锁
volatile int ReturnHome = 1000; //返航

int rdpackhz = 0;
int rdpackhz_calc = 0;
uint32_t rdpackloop_time = 0;


void OnPacketFlightMode(enum eFlightMode mode)
{
	(void)mode;
	need_response_back = true;
	req_response_tm = HAL_GetTick();
}

void OnPacketChannels()
{
	CALC_FUNC_FREQ("OnPacketChannels");
	//int Prev_Roll = Roll;
	//int Prev_Pitch = Pitch;
	//int Prev_Throttle = Throttle;
	//int Prev_Yaw = Yaw;
	//printf("OnPacketChannels\r\n");
	int newRoll = crsf2_getChannel(1); // Roll横滚
	int newPitch = crsf2_getChannel(2); // 俯仰（Pitch）
	int newThrottle = crsf2_getChannel(3); // 油门（Throttle）
	int newYaw = crsf2_getChannel(4); // 偏航（Yaw）
	int newMode = crsf2_getChannel(5); // 车/无人机 模式
	int newArmed = crsf2_getChannel(6); // 车灯
	int newReturnHome = crsf2_getChannel(7);  // 返航

	int newRoll2 = crsf2_getChannel(11); // Roll横滚
	int newPitch2 = crsf2_getChannel(12); // 俯仰（Pitch）
	int newThrottle2 = crsf2_getChannel(13); // 油门（Throttle）
	int newYaw2 = crsf2_getChannel(14); // 偏航（Yaw）
	int validate_channel = crsf2_getChannel(15); //校验

	int validate = newRoll + newPitch + newThrottle + newYaw;
	validate = CRSF_CHANNEL_VALUE_1000 + validate % (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000);

	if ((newRoll != newRoll2 || newPitch != newPitch2 || newThrottle != newThrottle2 || newYaw != newYaw2)
			|| validate_channel != validate)
	{
		//接收到的数据和发送的数据不一致
		char buff[50];
		snprintf(buff, sizeof(buff), "val err: %d-%d %d-%d %d-%d %d-%d",
				newRoll, newRoll2, newPitch, newPitch2, newThrottle, newThrottle2, newYaw, newYaw2);
		Log::E(buff);
		return;
	}

	newRoll = crsf_map(newRoll, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
	newPitch = crsf_map(newPitch, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
	newThrottle = crsf_map(newThrottle, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
	newYaw = crsf_map(newYaw, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
	newMode = crsf_map(newMode, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
	newArmed = crsf_map(newArmed, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
	newReturnHome = crsf_map(newReturnHome, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);

	//通道值超出极限，不是正常数值
	if (newRoll < (cfg.param.rcl_rol_left - (cfg.param.rcl_rol_dead * 2)) || newRoll > (cfg.param.rcl_rol_right + (cfg.param.rcl_rol_dead * 2)) ||
			newPitch < (cfg.param.rcl_pit_pull - (cfg.param.rcl_pit_dead * 2)) || newPitch > (cfg.param.rcl_pit_push + (cfg.param.rcl_rol_dead * 2)) ||
			newYaw < (cfg.param.rcl_yaw_left - (cfg.param.rcl_yaw_dead * 2)) || newYaw > (cfg.param.rcl_yaw_right + (cfg.param.rcl_yaw_dead * 2)) ||
			newThrottle < (cfg.param.rcl_thr_pull - (cfg.param.rcl_thr_dead * 2)) || newThrottle > (cfg.param.rcl_thr_push + (cfg.param.rcl_thr_dead * 2)))
	{
		char buff[50];
		snprintf(buff, sizeof(buff), "radio err: %d-%d %d-%d %d-%d %d-%d",
				newRoll, newRoll2, newPitch, newPitch2, newThrottle, newThrottle2, newYaw, newYaw2);
		Log::E(buff);
		return;
	}

	//处理异常遥控值，遥控器控制幅度在40，但是收到的数据有时候突然从1800直接变成1300，原因未知，暂时忽略掉异常值
	//之前是因为天线原因，装上天线后应该不再出现这种情况
	/*
	if ((Prev_Throttle - newThrottle) > 300 //&& newThrottle < (1500-50)
			)
	{
		//检测油门异常
		char buff[50];
		snprintf(buff, sizeof(buff), "radio in: %d -> %d", Prev_Throttle, newThrottle);
		Log::W(buff);
		Throttle = Prev_Throttle;
	}
	else
	//*/
	{
		Throttle = newThrottle;
	}
	Roll = newRoll;
	Pitch = newPitch;
	Yaw = newYaw;
	/*
	if ((abs(Prev_Roll - Roll) > 200 && abs(Prev_Roll - 1500) < 100)// || abs(Prev_Roll - Roll) < 5
			)
	{
		Roll = Prev_Roll;
	}
	if ((abs(Prev_Pitch - Pitch) > 200 && abs(Prev_Pitch - 1500) < 100)// || abs(Prev_Pitch - Pitch) < 5
	)
	{
		Pitch = Prev_Pitch;
	}
	if ((abs(Prev_Yaw - Yaw) > 200 && abs(Prev_Yaw - 1500) < 100)// || abs(Prev_Yaw - Yaw) < 5
	)
	{
		Yaw = Prev_Yaw;
	}
	//*/
	Mode = newMode;
	Armed = newArmed;
	ReturnHome = newReturnHome;
	Light = crsf_map(crsf2_getChannel(8), CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);

	uint32_t now = millis();

	//计算频率
	if (rdpackloop_time == 0)
	{
		rdpackloop_time = millis();
		rdpackhz = 0;
	}
	if ((now - rdpackloop_time) > 1000)
	{
		rdpackloop_time = now;
		rdpackhz = rdpackhz_calc;
		rdpackhz_calc = 0;
	}
	else
	{
		rdpackhz_calc++;
	}
	Log::I("Roll: %d, Pitch: %d, Yaw: %d, Throttle: %d", Roll, Pitch, Yaw, Throttle);
}

static void GetData(int *_Roll, int *_Pitch, int *_Throttle, int *_Yaw, int *_Mode, int *_Light, int *_ReturnHome, int *_Armed)
{
	if (!IsConnect)
	{
		*_Roll = 1500;
		*_Pitch = 1500;
		*_Throttle = 1500;
		*_Yaw = 1500;
		*_Mode = Mode;
		*_Light = Light;
		*_ReturnHome = ReturnHome;
		*_Armed = Armed;
	}
	else
	{
		*_Roll = Roll;
		*_Pitch = Pitch;
		int Local_Throttle = Throttle; //避免线程切换取到脏数据
		//if (!((*_Throttle - Local_Throttle) > 100 && Local_Throttle < 1500))
		{
			*_Throttle = Local_Throttle;
		}
		*_Yaw = Yaw;
		*_Mode = Mode;
		*_Light = Light;
		*_ReturnHome = ReturnHome;
		*_Armed = Armed;
	}

    //前避障
    if (app.GetFrontDistance() < 2000 && app.GetFrontDistance() > 200)
    {
    	//*_Pitch = 1400;
    }
}


void OnTxDone( void )
{
    //Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	uint32_t now = TimerGetCurrentTime();
    //Radio.Sleep( );
    //BufferSize = size;
    //if (BufferSize < BUFFER_SIZE)
    {
    	//memcpy( Buffer, payload, BufferSize );
    }
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    recvTime = now;
    //处理遥控信号
    //CrsfSerial_dataIn(payload, size);
    crsf2_handleSerialIn(payload, size);

    CALC_FUNC_FREQ("OnRxDone");
}

void OnTxTimeout( void )
{
    //Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    //Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    //Radio.Sleep( );
    State = RX_ERROR;
}
