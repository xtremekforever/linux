
#ifndef _AHERN_PUMP_H_
#define _AHERN_PUMP_H_

#include <linux/ioctl.h>

#define AHERN_PUMP_MAJOR_NUM  55

#define AHERN_PUMP_IOCTL_AUTHORIZE    	_IO(AHERN_PUMP_MAJOR_NUM, 1)
#define AHERN_PUMP_IOCTL_DEAUTHORIZE  	_IO(AHERN_PUMP_MAJOR_NUM, 2)

#define AHERN_PUMP_1          			1
#define AHERN_PUMP_2          			2

struct pump_interface {
  int enable_pin;
  int pulse_pin;

  int val;
  int prev;

  uint32_t ticks;
};

#endif
