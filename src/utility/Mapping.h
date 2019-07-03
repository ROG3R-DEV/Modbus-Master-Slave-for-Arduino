#pragma once

#include "Block.h"
#include "CoilBlock.h"
#include "ModbusCommon.h"
#include "RegisterBlock.h"

//??#include "Arduino.h"
#include <stdint.h>
//??#include <stdlib.h>

namespace modbus {



class Mapping
{
public:
  Mapping();
  Mapping(RegisterBlock& holding_register_block);
  Mapping(RegisterBlock& holding_register_block, CoilBlock& coil_block);

  /** Return TRUE if any block has been modified. */
  bool is_dirty() const
    { return dirty; }

  /** Clear the "dirty" flag. */
  void set_clean();

  /** For FC7: May be overridden to write a single byte to *dest.
   *  Interpreted as eight server-defined exception status flags. */
  virtual int8_t read_exception_status(uint8_t* dest)
    { (void)dest; return EXC_ILLEGAL_FUNCTION; }

  /** For FC11: May be overridden to return TRUE if the server is busy. */
  virtual bool is_busy()
    { return false; }

  //
  // Coils (read/write boolean values).

  void add_coil_block(CoilBlock& cb);
  bool have_coil_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t write_coils(uint16_t dest_addr, uint8_t* src_byte, uint16_t quantity =1);
  int8_t read_coils(uint8_t* dest_byte, uint16_t src_addr, uint16_t quantity =1) const;

  //
  // Discrete inputs (read-only boolean values).

  void add_discrete_input_block(CoilBlock& cb);
  bool have_discrete_input_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t read_discrete_inputs(uint8_t* dest_byte, uint16_t src_addr, uint16_t quantity =1) const;

  //
  // Input registers (read-only 16-bit words).

  void add_input_register_block(RegisterBlock& rb);
  bool have_input_register_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t write_input_registers(uint16_t dest_addr, uint8_t* src, uint16_t quantity =1);
  int8_t read_input_registers(uint8_t* dest, uint16_t src_addr, uint16_t quantity =1) const;

  //
  // Holding registers (read/write 16-bit words).

  void add_holding_register_block(RegisterBlock& rb);
  bool have_holding_register_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t write_holding_registers(uint16_t dest_addr, uint8_t* src, uint16_t quantity =1);
  int8_t read_holding_registers(uint8_t* dest, uint16_t src_addr, uint16_t quantity =1) const;

private:
  Block*  coil_block_list_head;
  Block*  discrete_input_block_list_head;
  Block*  input_register_block_list_head;
  Block*  holding_register_block_list_head;
  bool    dirty;

private:
  Mapping(const Mapping&); ///< Class is non-copyable.

  void add_block(Block** ptr, Block& new_block);

  /** Helper for implementing have_coil/register_addresses().
   *  Looks for ALL of the required address range.
   *
   * @return  NULL: not found, &Block: block that contains first address.
   */
  Block* find_addresses(
      Block*    block,
      uint16_t  first_addr,
      uint16_t  num_addr
    ) const;

  int8_t write_many(
      Block*   block,
      uint16_t dest_addr,
      uint8_t* src_byte,
      uint16_t quantity
    );

  int8_t read_many(
      Block*   block,
      uint8_t* dest_byte,
      uint16_t src_addr,
      uint16_t quantity
    ) const;
};


} // end namespace modbus
