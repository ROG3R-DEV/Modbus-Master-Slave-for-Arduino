#include "ModbusData.h"

namespace modbus {


Block::Block(
    uint16_t  length_,
    uint16_t  start_address_
  ):
    length(length_), ///< The number of coils (bits) in this block.
    start_address(start_address_),
    dirty(false)
{}


/** If addr is a valid address in this block, returns the number of
*  coils (bits) from addr to the end of the block.
*  If addr is not in this block, return zero. */
uint16_t Block::have_address(uint16_t addr) const
{
  if(addr >= start_address && addr < start_address + length)
      return start_address - addr + length;
  else
      return 0;
}


//
// CoilBlock

int8_t CoilBlock::write_many(
    uint16_t& __restrict__ dest_addr,
    Position&              src,
    uint16_t& __restrict__ quantity
  )
{
  int8_t err = 0;
  while(quantity && dest_addr < (start_address + length))
  {
    bool v = bitRead(*src.byte, src.bitn);
    err = write_one(dest_addr, v);
    if(err != 0)
        break;

    src.bitn = 0x07 & (src.bitn + 1);
    if(src.bitn == 0)
    {
      ++src.byte;
    }
    ++dest_addr;
    --quantity;
  }
  return err;
}


int8_t CoilBlock::read_many(
    Position&              dest,
    uint16_t& __restrict__ src_addr,
    uint16_t& __restrict__ quantity
  ) const
{
  int8_t err = 0;
  while(quantity && src_addr < (start_address + length))
  {
    bool v;
    err = read_one(src_addr, v);
    if(err != 0)
        break;

    bitWrite(*dest.byte, dest.bitn, v);
    dest.bitn = 0x07 & (dest.bitn + 1);
    if(dest.bitn == 0)
    {
      ++dest.byte;
      *dest.byte = 0;
    }
    ++src_addr;
    --quantity;
  }
  return err;
}


//
// CoilBlockData

CoilBlockData::CoilBlockData(
    uint8_t*  data_bytes_,
    uint16_t  length_,
    uint16_t  start_address_
  ):
    CoilBlock(length_, start_address_),
    data_bytes(data_bytes_)
{}


CoilBlockData::CoilBlockData(
    uint16_t* data_words_,
    uint16_t  length_,
    uint16_t  start_address_
  ):
    CoilBlock(length_, start_address_),
    data_bytes(reinterpret_cast<uint8_t*>(data_words_))
{}


int8_t CoilBlockData::write_one(uint16_t dest_addr, bool value)
{
  const uint16_t offset    = dest_addr - start_address;
  uint8_t*       dest_byte = data_bytes + (offset / 8);
  uint8_t        dest_bit  = offset % 8;

  dirty = true;
  bitWrite(*dest_byte, dest_bit, value);
  return 0;
}


int8_t CoilBlockData::read_one(uint16_t src_addr, bool& value) const
{
  const uint16_t offset  = src_addr - start_address;
  const uint8_t* src     = data_bytes + (offset / 8);
  uint8_t        src_bit = offset % 8;

  value = bitRead(*src, src_bit);
  return 0;
}


//
// RegisterBlock

int8_t RegisterBlock::write_many(
    uint16_t& __restrict__ dst_addr,
    Position&              src,
    uint16_t& __restrict__ quantity
  )
{
  int8_t err = 0;
  while(quantity && dst_addr < (start_address + length))
  {
    err = write_one(dst_addr, demarshal_u16(src.byte));
    if(err != 0)
        break;

    ++dst_addr;
    src.byte += 2;
    --quantity;
  }
  return err;
}


int8_t RegisterBlock::read_many(
    Position&              dest,
    uint16_t& __restrict__ src_addr,
    uint16_t& __restrict__ quantity
  ) const
{
  int8_t err = 0;
  while(quantity && src_addr < (start_address + length))
  {
    uint16_t value;
    err = read_one(src_addr, value);
    if(err != 0)
        break;

    marshal_u16(dest.byte, value);
    ++src_addr;
    dest.byte += 2;
    --quantity;
  }
  return err;
}


//
// RegisterBlockData

RegisterBlockData::RegisterBlockData(
    uint16_t* data_words_,
    uint16_t  length_, ///< Number of registers (2-byte words) in this block.
    uint16_t  start_address_
  ):
    RegisterBlock(length_, start_address_),
    data_words(data_words_)
{}


//
// No checks are made.
// The caller is assumed to have checked that the address is valid.

int8_t RegisterBlockData::write_one(uint16_t dst_addr, uint16_t value)
{
  data_words[dst_addr - start_address] = value;
  dirty = true;
  return 0;
}


int8_t RegisterBlockData::read_one(uint16_t src_addr, uint16_t& value) const
{
  value = data_words[src_addr - start_address];
  return 0;
}


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
