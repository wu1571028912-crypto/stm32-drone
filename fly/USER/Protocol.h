#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

#define CONTROL_HEADER1  0xAAU
#define CONTROL_HEADER2  0xAFU

#define CTRL_FLAG_DESCENT          0x01U
#define CTRL_FLAG_EMERGENCY_STOP   0x02U
#define CTRL_FLAG_FINE_MODE        0x04U

#pragma pack(push, 1)

typedef struct
{
    uint8_t header1;
    uint8_t header2;

    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;

    uint8_t switch_cfg;
    uint8_t checksum;
} FC_Packet_t;

#pragma pack(pop)

#endif
