#ifndef _CCS_TO_BOT_PROTOCOL_H
#define _CCS_TO_BOT_PROTOCOL_H

#include <stdint.h>


///////////////////////////////////////////////////////////////////////////////////////////////////
enum eMsgType {
    eMsg_diagnostic = 0xFD
  , eMsg_sync = 0xFE
  , eMsg_nack = 0xFF
  , eMsg_ack = 0
  , eMsg_init = 1
  , eMsg_move_vel = 2
  , eMsg_turret_tilt_pan = 3
  , eMsg_shot_detect = 4
  , eMsg_light_effect = 5
};

///////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct stct_ser_msg {
  eMsgType Type;
  uint8_t Len;
  uint8_t Checksum;
  uint8_t Payload[1];
} ser_msg;


///////////////////////////////////////////////////////////////////////////////////////////////////
inline void ser_msg_nullify(ser_msg* Target_msg__p) {
  Target_msg__p->Type = eMsg_sync;
  Target_msg__p->Len = 0;
  Target_msg__p->Checksum = 0 - Target_msg__p->Type;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
inline void ser_msg_add_checksum(ser_msg* Target_msg__p) {
  uint8_t Checksum = Target_msg__p->Type;
  Checksum += Target_msg__p->Len;
  for (uint8_t Payload_idx = 0; Payload_idx < Target_msg__p->Len; Payload_idx++)
  {
    Checksum += Target_msg__p->Payload[Payload_idx];
  }

  Target_msg__p->Checksum = (0 - Checksum);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
inline bool ser_msg_verify_checksum(ser_msg* Target_msg__p) {
  uint8_t Checksum = Target_msg__p->Type;
  Checksum += Target_msg__p->Len;
  for (uint8_t Payload_idx = 0; Payload_idx < Target_msg__p->Len; Payload_idx++)
  {
    Checksum += Target_msg__p->Payload[Payload_idx];
  }

  return ((Checksum + Target_msg__p->Checksum) == 0);
}

#endif//_CCS_TO_BOT_PROTOCOL_H

