
#ifndef _AHERN_PUMP_H_
#define _AHERN_PUMP_H_

#include <linux/ioctl.h>

#define AHERN_PUMP_MAGIC                'A'

#define AHERN_PUMP_IOCTL_AUTHORIZE    	_IO(AHERN_PUMP_MAGIC, 1)
#define AHERN_PUMP_IOCTL_DEAUTHORIZE  	_IO(AHERN_PUMP_MAGIC, 2)

#endif
