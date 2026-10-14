#ifndef __sys_H
#define __sys_H

#include "main.h"
#include "NRF24L01.h"
#include "key.h"
#include "Protocol.h"
#include "joystick.h"


void sys_init(void);

void float_to_uint8s(float f, uint8_t *byte0, uint8_t *byte1, uint8_t *byte2, uint8_t *byte3);
float uint8s_to_float(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3);
void split_uint16(uint16_t data, uint8_t *high_byte, uint8_t *low_byte);
uint16_t merge_uint8(uint8_t high_byte, uint8_t low_byte);





#endif

