#include "Mapping.h"

#include <stdlib.h>

namespace modbus {


//
// Mapping

Mapping::Mapping():
  coil_block_list_head(NULL),
  discrete_input_block_list_head(NULL),
  input_register_block_list_head(NULL),
  holding_register_block_list_head(NULL),
  dirty(false)
{}


Mapping::Mapping(RegisterBlock& holding_register_block):
  coil_block_list_head(NULL),
  discrete_input_block_list_head(NULL),
  input_register_block_list_head(NULL),
  holding_register_block_list_head(&holding_register_block),
  dirty(false)
{}


Mapping::Mapping(RegisterBlock& holding_register_block, CoilBlock& coil_block):
  coil_block_list_head(&coil_block),
  discrete_input_block_list_head(NULL),
  input_register_block_list_head(NULL),
  holding_register_block_list_head(&holding_register_block),
  dirty(false)
{}


void Mapping::set_clean()
{
  for(Block* blk=coil_block_list_head; blk; blk=blk->next_block)
      blk->set_clean();

  for(Block* blk=discrete_input_block_list_head; blk; blk=blk->next_block)
      blk->set_clean();

  for(Block* blk=input_register_block_list_head; blk; blk=blk->next_block)
      blk->set_clean();

  for(Block* blk=holding_register_block_list_head; blk; blk=blk->next_block)
      blk->set_clean();

  dirty = false;
}


// COILS

void Mapping::add_coil_block(CoilBlock& cb)
{
  add_block(&coil_block_list_head, cb);
}


bool Mapping::have_coil_addresses(uint16_t first_addr, uint16_t quantity) const
{
  return find_addresses(coil_block_list_head, first_addr, quantity);
}


int8_t Mapping::write_coils(
    uint16_t dest_addr,
    uint8_t* src_byte,
    uint16_t quantity
  )
{
  return write_many(coil_block_list_head, dest_addr, src_byte, quantity);
}


int8_t Mapping::read_coils(
    uint8_t* dest_byte,
    uint16_t src_addr,
    uint16_t quantity
  ) const
{
  *dest_byte = 0;
  return read_many(coil_block_list_head, dest_byte, src_addr, quantity);
}


// DISCRETE INPUTS

void Mapping::add_discrete_input_block(CoilBlock& cb)
{
  add_block(&discrete_input_block_list_head, cb);
}


bool Mapping::have_discrete_input_addresses(uint16_t first_addr, uint16_t quantity) const
{
  return find_addresses(discrete_input_block_list_head, first_addr, quantity);
}


int8_t Mapping::read_discrete_inputs(
    uint8_t* dest_byte,
    uint16_t src_addr,
    uint16_t quantity
  ) const
{
  *dest_byte = 0;
  return read_many(discrete_input_block_list_head, dest_byte, src_addr, quantity);
}


// INPUT REGISTERS

void Mapping::add_input_register_block(RegisterBlock& rb)
{
  add_block(&input_register_block_list_head, rb);
}


bool Mapping::have_input_register_addresses(uint16_t first_addr, uint16_t quantity) const
{
  return find_addresses(input_register_block_list_head, first_addr, quantity);
}


int8_t Mapping::read_input_registers(
    uint8_t*  dest_byte,
    uint16_t  src_addr,
    uint16_t  quantity
  ) const
{
  return read_many(input_register_block_list_head, dest_byte, src_addr, quantity);
}


// HOLDING REGISTERS

void Mapping::add_holding_register_block(RegisterBlock& rb)
{
  add_block(&holding_register_block_list_head, rb);
}


bool Mapping::have_holding_register_addresses(uint16_t first_addr, uint16_t quantity) const
{
  return find_addresses(holding_register_block_list_head, first_addr, quantity);
}


int8_t Mapping::write_holding_registers(
    uint16_t  dest_addr,
    uint8_t*  src_byte,
    uint16_t  quantity
  )
{
  return write_many(holding_register_block_list_head, dest_addr, src_byte, quantity);
}


int8_t Mapping::read_holding_registers(
    uint8_t*  dest_byte,
    uint16_t  src_addr,
    uint16_t  quantity
  ) const
{
  return read_many(holding_register_block_list_head, dest_byte, src_addr, quantity);
}


int8_t Mapping::write_read_multiple_registers(
    uint8_t* write_src, uint16_t write_addr, uint16_t write_quantity,
    uint8_t* read_dest, uint16_t read_addr,  uint16_t read_quantity
  )
{
  // Must perform write operation BEFORE read operation.
  // Check that read addresses are valid, before attempting the write.
  if(!have_holding_register_addresses(read_addr, read_quantity))
      return EXC_ILLEGAL_DATA_ADDRESS;

  int8_t rv = write_many(holding_register_block_list_head, write_addr, write_src, write_quantity);
  if(rv)
      return rv; // error
  else
      return read_many(holding_register_block_list_head, read_dest, read_addr, read_quantity);
}


// PRIVATE

void Mapping::add_block(Block** ptr, Block& new_block)
{
  while(*ptr)
  {
    if(new_block.get_start_address() < (**ptr).get_start_address())
    {
      new_block.next_block = *ptr;
      *ptr = &new_block;
      return;
    }
    ptr = &(**ptr).next_block;
  }
  new_block.next_block = NULL;
  *ptr = &new_block;
}


Block* Mapping::find_addresses(
    Block*   block,
    uint16_t first_addr,
    uint16_t num_addr
  ) const
{
  Block* result = NULL;
  while(block)
  {
    const uint16_t n = block->have_address(first_addr);
    if(n)
    {
      // Result refers to the block that contains the first_addr.
      if(!result)
          result = block;

      if(n >= num_addr)
          return result;

      num_addr -= n;
      first_addr += n;
    }
    else if(block->get_start_address() > first_addr)
    {
      // We have passed the address we are looking for - give up.
      break;
    }
    block = block->next_block;
  }
  return NULL;
}


int8_t Mapping::write_many(
    Block*   block,
    uint16_t dest_addr,
    uint8_t* src_byte,
    uint16_t quantity
  )
{
  // First check that we have all of the addresses.
  block = find_addresses(block, dest_addr, quantity);
  if(block)
  {
    Position src(src_byte);
    dirty = true;
    while(block)
    {
      const int8_t rv = block->write_many(dest_addr, src, quantity);
      if(rv != 0  ||  quantity == 0)
          return rv;

      block = block->next_block;
    }
    // Should never get here.
  }
  return EXC_ILLEGAL_DATA_ADDRESS;
}


int8_t Mapping::read_many(
    Block*   block,
    uint8_t* dest_byte,
    uint16_t src_addr,
    uint16_t quantity
  ) const
{
  // First check that we have all of the addresses.
  block = find_addresses(block, src_addr, quantity);
  if(block)
  {
    Position dest(dest_byte);
    while(block)
    {
      const int8_t rv = block->read_many(dest, src_addr, quantity);
      if(rv != 0  ||  quantity == 0)
          return rv;

      block = block->next_block;
    }
    // Should never get here.
  }
  return EXC_ILLEGAL_DATA_ADDRESS;
}


} // end namespace modbus
