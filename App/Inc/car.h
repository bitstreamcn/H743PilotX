/*
 * car.h
 *
 *  Created on: 2026年9月4日
 *      Author: bitstream
 */

#ifndef INC_CAR_H_
#define INC_CAR_H_

class Car
{
public:
	void Loop();

	void setLedOn();
	void setLedOff();
	void setBeepOn();
	void setBeepOff();
	void setMotor(int speed);
	void setServo(int value);

private:
	typedef struct __attribute__((packed)){
		int8_t motor; //电机驱动+-100
		int8_t servo; //方向驱动+-100
		uint8_t led:1; //大灯
		uint8_t beep:1; //BB响
		uint8_t unused:6;
		uint8_t validate; //校验码
	}st_rx;      // 接收的数据

	typedef struct __attribute__((packed)){
		uint8_t voltage; //电压，例11.2V = 112
		uint8_t led:1; //大灯
		uint8_t beep:1; //BB响
		uint8_t unused:6;
		uint8_t validate; //校验码
	}st_tx;

	st_rx rx_data;
	st_tx tx_data;

	float voltage = 0.0;
};

#endif /* INC_CAR_H_ */
