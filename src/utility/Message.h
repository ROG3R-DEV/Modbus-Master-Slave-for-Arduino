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

  void     save_request() { if(buf) memcpy(req, buf, sizeof(req)); }

  uint8_t  get_slave() const { return buf[0]; }
  void     set_slave(uint8_t slave_id) { buf[0] = slave_id; }

  uint8_t  get_fc() const { return buf[1]; }
  void     set_fc(uint8_t function_code) { buf[1] = function_code; }

  int8_t fc_read_coils(uint8_t slave_id, uint16_t addr, uint16_t quantity)
    {
      return _pdu5(slave_id, MB_FC_READ_COILS, addr, quantity, 3+bitset_size(quantity));
    }

  int8_t fc_read_discrete_inputs(uint8_t slave_id, uint16_t addr, uint16_t quantity)
    {
      return _pdu5(slave_id, MB_FC_READ_DISCRETE_INPUTS, addr, quantity, 3+bitset_size(quantity));
    }

  int8_t fc_read_holding_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity)
    {
      return _pdu5(slave_id, MB_FC_READ_HOLDING_REGISTERS, addr, quantity, 3+quantity*2);
    }

  int8_t fc_read_input_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity)
    {
      return _pdu5(slave_id, MB_FC_READ_INPUT_REGISTERS, addr, quantity, 3+quantity*2);
    }

  int8_t fc_write_single_coil(uint8_t slave_id, uint16_t addr, bool value)
    {
      return _pdu5(slave_id, MB_FC_WRITE_SINGLE_COIL, addr, (value? 0xFF00: 0), 6);
    }

  int8_t fc_write_single_register(uint8_t slave_id, uint16_t addr, uint16_t value)
    {
      return _pdu5(slave_id, MB_FC_WRITE_SINGLE_REGISTER, addr, value, 6);
    }

  int8_t fc_read_exception_status(uint8_t slave_id)
    {
      return _pdu1(slave_id, MB_FC_READ_EXCEPTION_STATUS, 3);
    }

  int8_t fc_diagnostics(uint8_t slave_id, uint16_t subfunction, uint16_t data)
    {
      return _pdu5(slave_id, MB_FC_DIAGNOSTICS, subfunction, data, 6);
    }

  int8_t fc_get_comm_event_counter(uint8_t slave_id)
    {
      return _pdu1(slave_id, MB_FC_GET_COMM_EVENT_COUNTER, 6);
    }

  int8_t fc_get_comm_event_log(uint8_t slave_id)
    {
      return _pdu1(slave_id, MB_FC_GET_COMM_EVENT_LOG, 253); // No idea how large the response might be
    }

  int8_t fc_write_multiple_coils(uint8_t slave_id, uint16_t addr, uint16_t quantity)
    {
      const uint8_t n = bitset_size(quantity);
      if(!buf || bufsize < (7 + n))
          return ERR_PDU;
      _hdr(slave_id, MB_FC_WRITE_MULTIPLE_COILS, addr, quantity);
      buf[6] = n;
      length = 7 + n;
      memset(buf + 7, 0, n);
      return 0;
    }

  int8_t fc_write_multiple_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity)
    {
      const uint8_t n = quantity * 2;
      if(!buf || bufsize < (7 + n))
          return ERR_PDU;
      _hdr(slave_id, MB_FC_WRITE_MULTIPLE_REGISTERS, addr, quantity);
      buf[6] = n;
      length = 7 + n;
      memset(buf + 7, 0, n);
      return 0;
    }

  int8_t fc_write_read_multiple_registers(
        uint8_t slave_id,
        uint16_t write_addr, uint16_t write_quantity,
        uint16_t read_addr,  uint16_t read_quantity
      )
    {
      const uint8_t wbytes = write_quantity * 2;
      const uint8_t rbytes = read_quantity * 2;
      if(!buf || bufsize < (11 + wbytes) || bufsize < (3 + rbytes))
          return ERR_PDU;
      type = MSG_REQUEST;
      set_slave(slave_id);
      set_fc(MB_FC_READ_WRITE_MULTIPLE_REGISTERS);
      marshal_u16(buf + 2, read_addr);
      marshal_u16(buf + 4, read_quantity);
      marshal_u16(buf + 6, write_addr);
      marshal_u16(buf + 8, write_quantity);
      buf[10] = wbytes;
      length = 11 + wbytes;
      memset(buf + 11, 0, wbytes);
      return 0;
    }

  uint8_t* bit_data()
    {
      switch(type)
      {
      case MSG_REQUEST:
          switch(get_fc())
          {
            case MB_FC_WRITE_SINGLE_COIL:    return buf + 4;
            case MB_FC_WRITE_MULTIPLE_COILS: return buf + 7;
            default: break;
          }
          break;

      case MSG_RESPONSE:
          switch(get_fc())
          {
            case MB_FC_READ_COILS:
            case MB_FC_READ_DISCRETE_INPUTS:  return buf + 3;
            case MB_FC_WRITE_SINGLE_COIL:     return buf + 4;
            case MB_FC_READ_EXCEPTION_STATUS: return buf + 2;
            default: break;
          }
          break;

      default:
          break;
      }
      return NULL;
    }

  uint8_t* register_data()
    {
      switch(type)
      {
      case MSG_REQUEST:
          switch(get_fc())
          {
            case MB_FC_WRITE_SINGLE_REGISTER:
            case MB_FC_DIAGNOSTICS:                   return buf + 4;
            case MB_FC_WRITE_MULTIPLE_REGISTERS:      return buf + 7;
            case MB_FC_READ_WRITE_MULTIPLE_REGISTERS: return buf + 11; // write registers
            default: break;
          }
          break;

      case MSG_RESPONSE:
          switch(get_fc())
          {
            case MB_FC_READ_HOLDING_REGISTERS:
            case MB_FC_READ_INPUT_REGISTERS:
            case MB_FC_READ_WRITE_MULTIPLE_REGISTERS: return buf + 3; // read registers
            case MB_FC_WRITE_SINGLE_REGISTER:
            case MB_FC_DIAGNOSTICS:                   return buf + 4;
            case MB_FC_GET_COMM_EVENT_COUNTER:        return buf + 2;
            case MB_FC_READ_FIFO_QUEUE:               return buf + 6;
            default: break;
          }
          break;

      default:
          break;
      }
      return NULL;
    }
  

  /** Returns the number of variable data, for all types of message. */
  uint16_t get_quantity() const
    {
      switch(type)
      {
      case MSG_REQUEST:
          switch(get_fc())
          {
            case MB_FC_WRITE_SINGLE_COIL:
            case MB_FC_WRITE_SINGLE_REGISTER:         return 1;
            case MB_FC_WRITE_MULTIPLE_COILS:
            case MB_FC_WRITE_MULTIPLE_REGISTERS:      return demarshal_u16(buf + 4);
            case MB_FC_DIAGNOSTICS:                   return (length - 3) / 2;
            case MB_FC_READ_WRITE_MULTIPLE_REGISTERS: return demarshal_u16(buf + 8); // write registers
            default: break;
          }
          break;

      case MSG_RESPONSE:
          switch(get_fc())
          {
            case MB_FC_WRITE_SINGLE_COIL:
            case MB_FC_WRITE_SINGLE_REGISTER:         return 1;
            case MB_FC_READ_COILS:
            case MB_FC_READ_DISCRETE_INPUTS:
            case MB_FC_READ_HOLDING_REGISTERS:
            case MB_FC_READ_INPUT_REGISTERS:
            case MB_FC_READ_WRITE_MULTIPLE_REGISTERS: // read registers
            case MB_FC_READ_FIFO_QUEUE:               return demarshal_u16(req + 4);
            case MB_FC_READ_EXCEPTION_STATUS:         return 8;
            case MB_FC_DIAGNOSTICS:                   return (length - 3) / 2;
            case MB_FC_GET_COMM_EVENT_COUNTER:        return 2;
            default: break;
          }
          break;

      default:
          break;
      }
      return 0;
    }

  int8_t get_bit(uint16_t bitnum, bool& value)
    {
      uint8_t* bdata = bit_data();
      if(!bdata)
          return ERR_FUNC_CODE;
      value = bitRead(bdata[bitnum/8], bitnum%8);
      return 0;
    }

  /** Unchecked version! Only call this is you are certain that it will succeed.
   *  If a response has passed verification, the FC is correct,
   *  and bitnum < quantity, then this call is guaranteed to succeed. */
  bool get_bit(uint16_t bitnum)
    {
      uint8_t* bdata = bit_data();
      if(bdata)
          return bitRead(bdata[bitnum/8], bitnum%8);
      else
          return false;
    }

  int8_t set_bit(uint16_t bitnum, bool value)
    {
      uint8_t* bdata = bit_data();
      if(!bdata)
          return ERR_FUNC_CODE;
      bitWrite(bdata[bitnum/8], bitnum%8, value);
      return 0;
    }

  /** <values> array parameter MUST contain at least <quantity> bits. */
  int8_t set_bits(uint8_t* values)
    {
      uint8_t* bdata = bit_data();
      if(!bdata)
          return ERR_FUNC_CODE;
      const uint16_t quantity = get_quantity();
      memcpy(bdata, values, bitset_size(quantity));
      return 0;
    }

  int8_t get_register(uint16_t regnum, uint16_t& value)
    {
      uint8_t* rdata = register_data();
      if(!rdata)
          return ERR_FUNC_CODE;
      value = demarshal_u16(rdata + 2*regnum);
      return 0;
    }

  /** Unchecked version! Only call this is you are certain that it will succeed.
   *  If a response has passed verification, the FC is correct,
   *  and regnum < quantity, then this call is guaranteed to succeed. */
  uint16_t get_register(uint16_t regnum)
    {
      uint8_t* rdata = register_data();
      if(rdata)
          return demarshal_u16(rdata + 2*regnum);
      else
          return UINT16_MAX;
    }

  int8_t set_register(uint16_t regnum, uint16_t value)
    {
      uint8_t* rdata = register_data();
      if(!rdata)
          return ERR_FUNC_CODE;
      marshal_u16(rdata + 2*regnum, value);
      return 0;
    }

  /** <values> array parameter MUST contain at least <quantity> uint16_t values. */
  int8_t set_registers(uint16_t* values)
    {
      uint8_t* rdata = register_data();
      if(!rdata)
          return ERR_FUNC_CODE;
      const uint16_t quantity = get_quantity();
      for(size_t i=0; i<quantity; ++i)
          marshal_u16(rdata + 2*i, values[i]);
      return 0;
    }

  int8_t verify_response()
    {
      type = MSG_RESPONSE;
      if(length < 2) // Exception is 2 bytes.
          return ERR_MALFORMED_MESSAGE;

      // Check slave ID.
      if(get_slave() == 0 || get_slave() != req[0])
          return ERR_BAD_SLAVE_ID;

      // Check function code matches request.
      if((get_fc() & 0x7f) != req[1])
          return ERR_FUNC_CODE; // ?? Need a more specific error for this

      // Check for an exception
      if(get_fc() & 0x80)
      {
        type = MSG_EXCEPTION;
        if (buf[2] <= INT8_MAX)
            return buf[2];
        else
            return ERR_EXCEPTION;
      }

      // Perform FC-specific checks.
      switch(get_fc())
      {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
          {
            if(buf[2] != bitset_size(get_quantity()))
                return ERR_MALFORMED_MESSAGE;
            if(length != 3 + buf[2])
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
          {
            if(buf[2] != (get_quantity() * 2))
                return ERR_MALFORMED_MESSAGE;
            if(length != 3 + buf[2])
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
          {
            if(length != 6 || memcmp(req,buf,6) != 0)
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_READ_EXCEPTION_STATUS:
          {
            if(length != 3)
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_DIAGNOSTICS:
          {
            if(length < 6 || memcmp(req,buf,4) != 0)
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_GET_COMM_EVENT_COUNTER:
          {
            if(length != 6)
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_GET_COMM_EVENT_LOG:
          {
            if(length < 9 || length != (3 + buf[2]))
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
          {
            if(length != 6 || memcmp(req,buf,6) != 0)
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_REPORT_SERVER_ID:
          {
            if(length < 5) // Report server ID response is really freeform.
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_READ_FILE_RECORD:
        case MB_FC_WRITE_FILE_RECORD:
          // Allow the user to implement this.
          return verify_unsupported();

        case MB_FC_MASK_WRITE_REGISTER:
          {
            if(length != 8 || memcmp(req,buf,6) != 0)
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_READ_WRITE_MULTIPLE_REGISTERS:
          {
            // Same as MB_FC_READ_HOLDING_REGISTERS
            if(buf[2] != (get_quantity() * 2))
                return ERR_MALFORMED_MESSAGE;
            if(length != 3 + buf[2])
                return ERR_MALFORMED_MESSAGE;
          }
          break;

        case MB_FC_READ_FIFO_QUEUE:
          {
            if(length < 6 || length != (6 + get_quantity() * 2))
                return ERR_MALFORMED_MESSAGE;
            if(length != 4 + demarshal_u16(buf+2))
                return ERR_MALFORMED_MESSAGE;
          }

        default:
          // Allow the user to implement their own function codes.
          return verify_unsupported();
      }
      return 0; // Checks passed.
    }

  /** Override this method to support extra function codes. */
  virtual int8_t verify_unsupported()
    {
      // Default implementation reports that this function code is unknown.
      return ERR_FUNC_CODE;
    }

protected:
  /** Create a common message header, with a function code, and 2-byte key/value pair. */
  void _hdr(uint8_t slave_id, uint8_t function_code, uint16_t key, uint16_t value)
    {
      type = MSG_REQUEST;
      set_slave(slave_id);
      set_fc(function_code);
      marshal_u16(buf + 2, key);
      marshal_u16(buf + 4, value);
    }

  /** Create a common message type, with only a function code. */
  int8_t _pdu1(uint8_t slave_id, uint8_t function_code, uint8_t min_response_pdu_size)
    {
      if(!buf || bufsize<min_response_pdu_size)
          return ERR_PDU;
      type = MSG_REQUEST;
      set_slave(slave_id);
      set_fc(function_code);
      length = 2;
      return 0;
    }

  /** Create a common message type, with a function code, and 2-byte key/value pair. */
  int8_t _pdu5(uint8_t slave_id, uint8_t function_code, uint16_t key, uint16_t value, uint8_t min_response_pdu_size)
    {
      if(!buf || bufsize<5 || bufsize<min_response_pdu_size)
          return ERR_PDU;
      _hdr(slave_id, function_code, key, value);
      length = 6;
      return 0;
    }

};


} // end namespace modbus



#if 0


void Master::cancel()
{
    u8state = COM_IDLE;
}


int8_t Master::send(Message& msg)
{
  if(u8state != COM_IDLE)
      return setError(ERR_WAITING);

  if(msg.slave() > 247))
      return setError(ERR_BAD_SLAVE_ID);

  sendTxBuffer(msg.buf, msg.length);
  msg.save_request();
  ++u16Counter[CNT_MASTER_QUERY];
  u32timeOut = millis();
  u8state = COM_WAITING;
  return 0;
}


int8_t Master::poll(Message& msg)
{
    if (u8state != COM_WAITING)
        return 0; // Not expecting any incoming messages.

    if (timeOutExpired())
    {
        u8state = COM_IDLE;
        ++u16Counter[CNT_MASTER_TIMEOUT];
        return setError(ERR_TIME_OUT_EXPIRED);
    }

    if (!rxFrameReady())
        return 0;

    // transfer Serial buffer frame to a local buffer.
    const int8_t i8bytes_read = getRxBuffer( msg.buf, msg.bufsize );
    if (i8bytes_read < 0)
    {
        return setError(i8bytes_read); // Pass error on from getRxBuffer().
    }
    msg.length = i8bytes_read;

    ++u16Counter[CNT_MASTER_RESPONSE];

    // validate message: length, exception
    int8_t i8error = msg.validate_response();
    if (i8error == 0)
    {
      u8state = COM_IDLE;
      return 1; // Response received and processed OK.
    }
    else if (i8error > 0)
    {
      u8state = COM_IDLE;
      ++u16Counter[CNT_MASTER_EXCEPTION];
    }
    else
    {
      ++u16Counter[CNT_MASTER_IGNORED];
    }
    return setError(i8error);
}



// ...


modbus::Message msg;

msg.fc_write_read_multiple_registers(1, 10000,10, 20000,3);
for(size_t i=0; i<msg.get_quantity(); ++i)
    msg.set_register(i, 10*i);
master.send(msg);

while(true)
{
  int8_t rv = master.poll(msg);
  if(rv == 0)
  {
    continue;
  }
  else if(rv < 0)
  {
    report_error(rv);
    break;
  }

  for(size_t i=0; i<msg.get_quantity(); ++i)
  {
    process_reg( msg.get_register(i) );
  }
}


msg.fc_write_single_register(1, 40000, digitalRead(A0));
master.send(msg);

#endif
