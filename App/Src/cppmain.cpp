/*
 * cppmain.cpp
 *
 *  Created on: Apr 2, 2026
 *      Author: bitstream
 */
#include "cppmain.h"
#include "application.h"

#ifdef __cplusplus
extern "C" {
#endif


void cppmain(void)
{
	app.Init();
	app.Loop();
}



#ifdef __cplusplus
}
#endif

