/*
 * configlink.h
 *
 *  Created on: May 5, 2026
 *      Author: bitstream
 */

#ifndef INC_CONFIGLINK_H_
#define INC_CONFIGLINK_H_

class ConfigLink{
public:
	ConfigLink();
	~ConfigLink();

	void RecvLoop();
private:
	void ConfirmSet(char *name);
	void ConfirmSave();
	void SendConfig(int param_index);
};

#endif /* INC_CONFIGLINK_H_ */
