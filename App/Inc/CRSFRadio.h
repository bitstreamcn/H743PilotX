/*
 * CRSFRadio.h
 *
 *  Created on: Jun 12, 2025
 *      Author: bitstream
 */

#ifndef CRSFRADIO_H_
#define CRSFRADIO_H_


typedef struct{
	int ( *Init )();
	void ( *Loop )();
	void ( *GetData )(int *_Roll, int *_Pitch, int *_Throttle, int *_Yaw, int *_Mode, int *_Light, int *_ReturnHome, int *_Armed);
}CRSFRadio_s;

extern CRSFRadio_s CRSFRadio;



#endif /* CRSFRADIO_H_ */
