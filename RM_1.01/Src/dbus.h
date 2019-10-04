/*
 * dbus.h
 *
 *  Created on: Oct 4, 2019
 *      Author: Administrator
 */

#ifndef DBUS_H_
#define DBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"

typedef struct
{
  int64_t ch1 :11;
  int64_t ch2 :11;
  int64_t ch3 :11;
  int64_t ch4 :11;
  uint64_t sw2 :2;
  uint64_t sw1 :2;
  int64_t x :16;
  int64_t y :16;
  int64_t z :16;
  uint64_t l :8;
  uint64_t r :8;
  union
  {
    uint16_t key_code;
    struct
    {
      uint16_t W :1;
      uint16_t S :1;
      uint16_t A :1;
      uint16_t D :1;
      uint16_t SHIFT :1;
      uint16_t CTRL :1;
      uint16_t Q :1;
      uint16_t E :1;
      uint16_t R :1;
      uint16_t F :1;
      uint16_t G :1;
      uint16_t Z :1;
      uint16_t X :1;
      uint16_t C :1;
      uint16_t V :1;
      uint16_t B :1;
    } bit;
  } kb;
  uint64_t wheel :16;
}__attribute__ ((packed)) rc_info_t;

typedef struct
{
  uint32_t isOnline;
  rc_info_t rc_info;
  rc_info_t last_rc_info;
} rc_device_t;

extern rc_device_t g_rc_device;

void rc_info_update(rc_device_t *rc_device, uint8_t *buff);

#ifdef __cplusplus
}
#endif


#endif /* DBUS_H_ */
