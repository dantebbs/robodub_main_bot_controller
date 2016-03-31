#include <Arduino.h>
#include <HardwareSerial.h>
#include "serial_comms.h"


static int Serial_to_use_idx_ = -1;
static HardwareSerial* Serial_to_use__p_ = NULL;
static bool Have_msg_ = false;
static int Curr_rx_buf_idx = 0;
static ser_msg Rx_buffer_;


///////////////////////////////////////////////////////////////////////////////////////////////////
void serial_comms_init(uint8_t Port, int Baud_rate)
{
  Serial_to_use_idx_ = -1;
  Serial_to_use__p_ = NULL;
  Have_msg_ = false;
  ser_msg_nullify(Rx_buffer_);
  
  switch (Port)
  {
    case 0: {
      Serial_to_use__p_ = &Serial;
    } break;
    case 1: {
      Serial_to_use__p_ = &Serial1;
    } break;
    case 2: {
      Serial_to_use__p_ = &Serial2;
    } break;
    case 3: {
      Serial_to_use__p_ = &Serial3;
    } break;
  }

  if (Serial_to_use__p_ != NULL)
  {
    Serial_to_use_idx_ = Port;
    Serial_to_use__p_->begin(Baud_rate);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// This must be called regularly to check for new inputs. If there is too long of a pause between
// calls, messages can be lost. For example, if the most common message is 7 bytes, and the baud
// rate is 57600, then at top speed a new message appears every (7 / (57600 / 10)) = 1.2 ms.
//
// returns  NULL if no valid message is available.
//          a pointer to a structure containing a serial message.
//
const ser_msg* serial_comms_process_input(void) {
  if (Serial_to_use__p_ == NULL) return (const ser_msg*)NULL;

  if (Have_msg_) return Rx_buffer__p_;
  else           return (const ser_msg*)NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// param[in]  Outgoing_msg  A pointer to a structure where the message type, length and data are
//                          already populated. The checksum field is then overwritten with the
//                          proper checksum and the message is sent out.
void send_message(ser_msg* Outgoing_msg__p)
{
  if (Serial_to_use__p_ == NULL) return (const ser_msg*)NULL;

  ser_msg_add_checksum(Outgoing_msp__p);

  Serial_to_use__p_->write((uint8_t*)Outgoing_msg__p, Outgoing_msg__p->Len + 3);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void serialEvent() {
  if (Serial_to_use_idx_ == 0)
  {
    // This data is for us.
  }
}


