
#ifndef _AHERN_PUMP_H_
#define _AHERN_PUMP_H_

#include <linux/ioctl.h>

#define AHERN_PUMP_MAJOR_NUM  55

#define AHERN_PUMP_IOCTL_1_ENABLE     	_IO(AHERN_PUMP_MAJOR_NUM, 1)
#define AHERN_PUMP_IOCTL_2_ENABLE     	_IO(AHERN_PUMP_MAJOR_NUM, 2)

#define AHERN_PUMP_IOCTL_1_DISABLE    	_IO(AHERN_PUMP_MAJOR_NUM, 3)
#define AHERN_PUMP_IOCTL_2_DISABLE    	_IO(AHERN_PUMP_MAJOR_NUM, 4)

#define AHERN_PUMP_NOT_AUTHORIZED   	-1
#define AHERN_PUMP_INTERFACE_BUSY		-2

struct pump_interface {
  int enable_pin;
  int pulse_pin;

  uint8_t val;
  uint8_t prev;
  uint8_t number;
  uint8_t authorized;

  uint32_t ticks;
};

#endif
