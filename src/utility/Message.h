#pragma once

#include "utility/ModbusCommon.h"

#include <stdint.h>

namespace modbus {


enum MessageType {
  MSG_INVALID,
  MSG_REQUEST,
  MSG_RESPONSE,
  MSG_EXCEPTION
};



struct Message
{
  MessageType type;

  uint8_t* buf;
  uint8_t  bufsize;
  uint8_t  length; ///< Length of the message, excluding CRC.

  uint8_t  req[6]; ///< Stores the request header, when type=MSG_RESPONSE|MSG_EXCEPTION

  Message(uint8_t* buf_, uint8_t bufsize_):
      buf(buf_), bufsize(bufsize_)
    {}

  void     save_request();

  uint8_t  get_slave() const { return buf[0]; }
  void     set_slave(uint8_t slave_id) { buf[0] = slave_id; }

  uint8_t  get_fc() const { return buf[1]; }
  void     set_fc(uint8_t function_code) { buf[1] = function_code; }

  int8_t fc_read_coils(uint8_t slave_id, uint16_t addr, uint16_t quantity);
  int8_t fc_read_discrete_inputs(uint8_t slave_id, uint16_t addr, uint16_t quantity);
  int8_t fc_read_holding_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity);
  int8_t fc_read_input_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity);
  int8_t fc_write_single_coil(uint8_t slave_id, uint16_t addr, bool value);
  int8_t fc_write_single_register(uint8_t slave_id, uint16_t addr, uint16_t value);
  int8_t fc_read_exception_status(uint8_t slave_id);
  int8_t fc_diagnostics(uint8_t slave_id, uint16_t subfunction, uint16_t data);
  int8_t fc_get_comm_event_counter(uint8_t slave_id);
  int8_t fc_get_comm_event_log(uint8_t slave_id);
  int8_t fc_write_multiple_coils(uint8_t slave_id, uint16_t addr, uint16_t quantity);
  int8_t fc_write_multiple_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity);
  int8_t fc_write_read_multiple_registers(
      uint8_t slave_id,
      uint16_t write_addr, uint16_t write_quantity,
      uint16_t read_addr,  uint16_t read_quantity
    );

  uint8_t* bit_data()const;
  uint8_t* register_data() const;

  /** Returns the number of variable data, for all types of message. */
  uint16_t get_quantity() const;

  int8_t   get_bit(uint16_t bitnum, bool& value) const;
  bool     get_bit(uint16_t bitnum) const;
  int8_t   set_bit(uint16_t bitnum, bool value);
  int8_t   set_bits(uint8_t* values);
  int8_t   get_register(uint16_t regnum, uint16_t& value) const;
  uint16_t get_register(uint16_t regnum) const;
  int8_t   set_register(uint16_t regnum, uint16_t value);
  int8_t   set_registers(uint16_t* values);

  int8_t verify_response();

  /** Override this method to support extra function codes. */
  virtual int8_t verify_unsupported();

protected:
  /** Create a common message header, with a function code, and 2-byte key/value pair. */
  void _hdr(uint8_t slave_id, uint8_t function_code, uint16_t key, uint16_t value);

  /** Create a common message type, with only a function code. */
  int8_t _pdu1(uint8_t slave_id, uint8_t function_code, uint8_t min_response_bufsize);

  /** Create a common message type, with a function code, and 2-byte key/value pair. */
  int8_t _pdu5(uint8_t slave_id, uint8_t function_code, uint16_t key, uint16_t value, uint8_t min_response_bufsize);
};


} // end namespace modbus
