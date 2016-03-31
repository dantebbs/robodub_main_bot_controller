///////////////////////////////////////////////////////////////////////////////////////////////////
// Manage the gory details of setup, syncing with the channel, validating and sending messages.

#ifndef _SERIAL_COMMS_H
#define _SERIAL_COMMS_H

#include "ccs_to_bot_protocol.h"


///////////////////////////////////////////////////////////////////////////////////////////////////
// This must be called before the other functions are used.
//
// param[in]  Port  0 = Arduino "Serial", 1 = Arduino "Serial1", etc.
// param[in]  Baud  Any supported baud rate. 57600 is recommended for Arduino.
//
void serial_comms_init(uint8_t Port, int Baud_rate);

///////////////////////////////////////////////////////////////////////////////////////////////////
// This must be called regularly to check for new inputs. If there is too long of a pause between
// calls, messages can be lost. For example, if the most common message is 7 bytes, and the baud
// rate is 57600, then at top speed a new message appears every (7 / (57600 / 10)) = 1.2 ms.
//
// returns  NULL if no valid message is available.
//          a pointer to a structure containing a serial message.
//
const ser_msg* serial_comms_process_input(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
// param[in]  Outgoing_msg  A pointer to a structure where the message type, length and data are
//                          already populated. The checksum field is then overwritten with the
//                          proper checksum and the message is sent out.
void send_message(ser_msg* Outgoing_msg__p);

#endif//_SERIAL_COMMS_H

