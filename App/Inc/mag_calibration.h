/*
 * mag_calibration.h
 *
 *  Created on: 2026年9月18日
 *      Author: bitstream
 */

#ifndef INC_MAG_CALIBRATION_H_
#define INC_MAG_CALIBRATION_H_

void MagCalib_CollectData(float x, float y, float z);
int MagCalib_Compute(float *offset_x, float *offset_y, float *offset_z,
                     float *scale_x, float *scale_y, float *scale_z);


#endif /* INC_MAG_CALIBRATION_H_ */
