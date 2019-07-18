#include "Message.h"

#include <Arduino.h>
#include <string.h>

namespace modbus {

namespace {
  const uint8_t CRC = 2;
}


Message::Message(uint8_t bufsize_, uint8_t* buf_):
  buf(buf_), bufsize(bufsize_), own_buf(false)
{
  if(!buf)
  {
    if(bufsize == 0)
        bufsize = INITIAL_BUFSIZE;

    buf = (uint8_t*)malloc(bufsize);
    own_buf = true;
  }
}

Message::~Message()
{
  if(own_buf)
      free(buf);
}


void Message::save_request()
{
  if(buf)
      memcpy(req, buf, sizeof(req));
}


int8_t Message::fc_read_coils(uint8_t slave_id, uint16_t addr, uint16_t quantity)
{
  return _pdu5(slave_id, MB_FC_READ_COILS, addr, quantity, 3+bitset_size(quantity)+CRC);
}


int8_t Message::fc_read_discrete_inputs(uint8_t slave_id, uint16_t addr, uint16_t quantity)
{
  return _pdu5(slave_id, MB_FC_READ_DISCRETE_INPUTS, addr, quantity, 3+bitset_size(quantity)+CRC);
}


int8_t Message::fc_read_holding_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity)
{
  return _pdu5(slave_id, MB_FC_READ_HOLDING_REGISTERS, addr, quantity, 3+quantity*2+CRC);
}


int8_t Message::fc_read_input_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity)
{
  return _pdu5(slave_id, MB_FC_READ_INPUT_REGISTERS, addr, quantity, 3+quantity*2+CRC);
}


int8_t Message::fc_write_single_coil(uint8_t slave_id, uint16_t addr, bool value)
{
  return _pdu5(slave_id, MB_FC_WRITE_SINGLE_COIL, addr, (value? 0xFF00: 0), 6+CRC);
}


int8_t Message::fc_write_single_register(uint8_t slave_id, uint16_t addr, uint16_t value)
{
  return _pdu5(slave_id, MB_FC_WRITE_SINGLE_REGISTER, addr, value, 6+CRC);
}


int8_t Message::fc_read_exception_status(uint8_t slave_id)
{
  return _pdu1(slave_id, MB_FC_READ_EXCEPTION_STATUS, 3+CRC);
}


int8_t Message::fc_diagnostics(uint8_t slave_id, uint16_t subfunction, uint16_t data)
{
  return _pdu5(slave_id, MB_FC_DIAGNOSTICS, subfunction, data, 6+CRC);
}


int8_t Message::fc_get_comm_event_counter(uint8_t slave_id)
{
  return _pdu1(slave_id, MB_FC_GET_COMM_EVENT_COUNTER, 6+CRC);
}


int8_t Message::fc_get_comm_event_log(uint8_t slave_id)
{
  return _pdu1(slave_id, MB_FC_GET_COMM_EVENT_LOG, 253+CRC); // No idea how large the response might be
}


int8_t Message::Message::fc_write_multiple_coils(uint8_t slave_id, uint16_t addr, uint16_t quantity)
{
  const uint8_t n = bitset_size(quantity);
  if(!_bufsize_ok(7 + n))
      return ERR_PDU;
  _hdr(slave_id, MB_FC_WRITE_MULTIPLE_COILS, addr, quantity);
  buf[6] = n;
  length = 7 + n;
  memset(buf + 7, 0, n);
  return 0;
}


int8_t Message::fc_write_multiple_registers(uint8_t slave_id, uint16_t addr, uint16_t quantity)
{
  const uint8_t n = quantity * 2;
  if(!_bufsize_ok(7 + n))
      return ERR_PDU;
  _hdr(slave_id, MB_FC_WRITE_MULTIPLE_REGISTERS, addr, quantity);
  buf[6] = n;
  length = 7 + n;
  memset(buf + 7, 0, n);
  return 0;
}


int8_t Message::fc_write_read_multiple_registers(
    uint8_t slave_id,
    uint16_t write_addr, uint16_t write_quantity,
    uint16_t read_addr,  uint16_t read_quantity
  )
{
  const uint8_t wbytes = write_quantity * 2;
  const uint8_t rbytes = read_quantity * 2;
  if(!_bufsize_ok(max(11 + wbytes, 3 + rbytes + CRC)))
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


uint8_t* Message::bit_data() const
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


uint8_t* Message::register_data() const
{
  uint8_t offset = 0;
  switch(type)
  {
  case MSG_REQUEST:
      switch(get_fc())
      {
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_DIAGNOSTICS:                   offset = 4; break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS:      offset = 7; break;
        case MB_FC_READ_WRITE_MULTIPLE_REGISTERS: offset = 11; break; // write registers
        default: break;
      }
      break;

  case MSG_RESPONSE:
      switch(get_fc())
      {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:          offset = 3; break;
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_DIAGNOSTICS:                   offset = 4; break;
        case MB_FC_GET_COMM_EVENT_COUNTER:        offset = 2; break;
        case MB_FC_READ_WRITE_MULTIPLE_REGISTERS: offset = 3; break; // read registers
        case MB_FC_READ_FIFO_QUEUE:               offset = 6; break;
        default: break;
      }
      break;

  default:
      break;
  }
  if(offset)
      return buf + offset;
  else
      return NULL;
}


/** Returns the number of variable data, for all types of message. */
uint16_t Message::get_quantity() const
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


int8_t Message::get_bit(uint16_t bitnum, bool& value) const
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
bool Message::get_bit(uint16_t bitnum) const
{
  bool result = false;
  get_bit(bitnum, result);
  return result;
}


int8_t Message::set_bit(uint16_t bitnum, bool value)
{
  uint8_t* bdata = bit_data();
  if(!bdata)
      return ERR_FUNC_CODE;
  bitWrite(bdata[bitnum/8], bitnum%8, value);
  return 0;
}


int8_t Message::get_bits(uint8_t* bitset, uint16_t bitset_length)
{
  uint8_t* bdata = bit_data();
  if(!bdata)
      return ERR_FUNC_CODE;
  const uint16_t q = min(get_quantity(), bitset_length);
  memcpy(bitset, bdata, bitset_size(q));
  return 0;
}


/** <values> array parameter MUST contain at least <quantity> bits. */
int8_t Message::set_bits(uint8_t* bitset, uint16_t bitset_length)
{
  uint8_t* bdata = bit_data();
  if(!bdata)
      return ERR_FUNC_CODE;
  const uint16_t q = min(get_quantity(), bitset_length);
  memcpy(bdata, bitset, bitset_size(q));
  return 0;
}


int8_t Message::get_register(uint16_t regnum, uint16_t& value) const
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
uint16_t Message::get_register(uint16_t regnum) const
{
  uint16_t result = UINT16_MAX;
  get_register(regnum, result);
  return result;
}


int8_t Message::set_register(uint16_t regnum, uint16_t value)
{
  uint8_t* rdata = register_data();
  if(!rdata)
      return ERR_FUNC_CODE;
  marshal_u16(rdata + 2*regnum, value);
  return 0;
}


/** <values> array parameter MUST contain at least <quantity> uint16_t values. */
int8_t Message::get_registers(uint16_t* values, uint16_t values_length)
{
  uint8_t* rdata = register_data();
  if(!rdata)
      return ERR_FUNC_CODE;
  const uint16_t q = min(get_quantity(), values_length);
  for(uint16_t i=0; i<q; ++i)
      values[i] = demarshal_u16(rdata + 2*i);
  return 0;
}


/** <values> array parameter MUST contain at least <quantity> uint16_t values. */
int8_t Message::set_registers(uint16_t* values, uint16_t values_length)
{
  uint8_t* rdata = register_data();
  if(!rdata)
      return ERR_FUNC_CODE;
  const uint16_t q = min(get_quantity(), values_length);
  for(uint16_t i=0; i<q; ++i)
      marshal_u16(rdata + 2*i, values[i]);
  return 0;
}


int8_t Message::verify_response()
{
  type = MSG_RESPONSE;
  if(length < 3) // Exception is 3 bytes.
      return ERR_MALFORMED_MESSAGE;

  // Check slave ID.
  if(get_slave() != req[0] || get_slave() == 0)
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

  const uint16_t quantity = get_quantity();

  // Perform FC-specific checks.
  switch(get_fc())
  {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUTS:
      {
        if(buf[2] != bitset_size(quantity))
            return ERR_MALFORMED_MESSAGE;
        if(length != 3 + buf[2])
            return ERR_MALFORMED_MESSAGE;
      }
      break;

    case MB_FC_READ_HOLDING_REGISTERS:
    case MB_FC_READ_INPUT_REGISTERS:
      {
        if(buf[2] != (quantity * 2))
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
        if(buf[2] != (quantity * 2))
            return ERR_MALFORMED_MESSAGE;
        if(length != 3 + buf[2])
            return ERR_MALFORMED_MESSAGE;
      }
      break;

    case MB_FC_READ_FIFO_QUEUE:
      {
        if(length != (6 + quantity * 2))
            return ERR_MALFORMED_MESSAGE;
        if(length != 4 + demarshal_u16(buf+2))
            return ERR_MALFORMED_MESSAGE;
      }
      break;

    default:
      // Allow the user to implement their own function codes.
      return verify_unsupported();
  }
  return 0; // Checks passed.
}


int8_t Message::verify_unsupported()
{
  // Default implementation reports that this function code is unknown.
  return ERR_FUNC_CODE;
}


bool Message::_bufsize_ok(uint8_t min_bufsize)
{
  if(bufsize >= min_bufsize)
      return true;

  if(own_buf)
  {
    // Extend the buffer to fit the message size.
    uint8_t* new_buf = (uint8_t*)realloc(buf, min_bufsize);
    if(new_buf)
    {
      buf = new_buf;
      return true;
    }
  }
  return false;
}


/** Create a common message header, with a function code, and 2-byte key/value pair. */
void Message::_hdr(uint8_t slave_id, uint8_t function_code, uint16_t key, uint16_t value)
{
  type = MSG_REQUEST;
  set_slave(slave_id);
  set_fc(function_code);
  marshal_u16(buf + 2, key);
  marshal_u16(buf + 4, value);
}


/** Create a common message type, with only a function code. */
int8_t Message::_pdu1(uint8_t slave_id, uint8_t function_code, uint8_t min_response_bufsize)
{
  if(!_bufsize_ok(min_response_bufsize))
      return ERR_PDU;
  type = MSG_REQUEST;
  set_slave(slave_id);
  set_fc(function_code);
  length = 2;
  return 0;
}


/** Create a common message type, with a function code, and 2-byte key/value pair. */
int8_t Message::_pdu5(uint8_t slave_id, uint8_t function_code, uint16_t key, uint16_t value, uint8_t min_response_bufsize)
{
  if(!_bufsize_ok(max(6, min_response_bufsize)))
      return ERR_PDU;
  _hdr(slave_id, function_code, key, value);
  length = 6;
  return 0;
}


} // end namespace modbus
