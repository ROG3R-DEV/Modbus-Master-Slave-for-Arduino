#include "ModbusData.h"

namespace modbus {


//
// CoilBlock

CoilBlock::CoilBlock(
  uint8_t*  data_bytes_,
  uint16_t  length_,
  uint16_t  start_address_
):
  data_bytes(data_bytes_),
  length(length_), ///< The number of coils (bits) in this block.
  start_address(start_address_),
  dirty(false)
{}


/** If addr is a valid address in this block, returns the number of
*  coils (bits) from addr to the end of the block.
*  If addr is not in this block, return zero. */
uint16_t CoilBlock::have_address(uint16_t addr) const
{
  if(addr >= start_address && addr < start_address + length)
      return start_address - addr + length;
  else
      return 0;
}


void CoilBlock::write_one(uint16_t addr, bool value)
{
  dirty = true;
  const uint16_t offset = addr - start_address;
  bitWrite(data_bytes[offset/8], offset%8, value);
}


int8_t CoilBlock::read_many(
  uint8_t*& dest_byte,
  uint8_t&  dest_bit,
  uint16_t& src_addr,
  uint16_t& quantity) const
{
  const uint16_t offset  = src_addr - start_address;
  const uint8_t* src     = data_bytes + (offset / 8);
  uint8_t        src_bit = offset % 8;

  size_t i;
  for(i=0; i<(length - offset) && i<quantity; ++i)
  {
    // ?? Optimise this...
    // ?? Probably best done with some assembly, where bit shifting can
    // ?? be done with carries available.
    bool v = bitRead(*src, src_bit++);
    if(src_bit%8 == 0)
    {
      ++src;
      src_bit = 0;
    }

    bitWrite(*dest_byte, dest_bit++, v);
    if(dest_bit%8 == 0)
    {
      ++dest_byte;
      dest_bit = 0;
    }
  }
  quantity -= i;
  return 0;
}


//
// RegisterBlock

RegisterBlock::RegisterBlock(
  uint16_t* data_words_,
  uint16_t  length_, ///< Number of registers (2-byte words) in this block.
  uint16_t  start_address_
):
  data_words(data_words_),
  length(length_),
  start_address(start_address_),
  dirty(false)
{}


/** If addr is a valid address in this block, returns the number of
*  registers (2-byte words) from addr to the end of the block.
*  If addr is not in this block, return zero. */
uint16_t RegisterBlock::have_address(uint16_t addr) const
{
  if(addr >= start_address && addr < start_address + length)
      return start_address - addr + length;
  else
      return 0;
}


//
// No checks are made.
// The caller is assumed to have checked that the address is valid.

void RegisterBlock::write_one(uint16_t addr, uint16_t value)
{
  dirty = true;
  data_words[ addr - start_address ] = bswap16(value);
}


int8_t RegisterBlock::write_many(
  uint16_t&  addr,
  uint16_t*& src,
  uint16_t&  quantity)
{
  while(quantity && (addr - start_address) < length)
  {
    data_words[addr++ - start_address] = bswap16( *(src++) );
    --quantity;
    dirty = true;
  }
  return 0;
}


int8_t RegisterBlock::read_many(
  uint16_t*& dest,
  uint16_t&  src_addr,
  uint16_t&  quantity) const
{
  while(quantity && (src_addr - start_address) < length)
  {
    *(dest++) = bswap16( data_words[src_addr++ - start_address] );
    --quantity;
  }
  return 0;
}


//
// Mapping

Mapping::Mapping(uint16_t num_coil_blocks, uint16_t num_register_blocks):
  coil_block( (CoilBlock**)calloc(num_coil_blocks, sizeof(CoilBlock*)) ),
  num_coil_blocks(0),
  register_block( (RegisterBlock**)calloc(num_register_blocks, sizeof(RegisterBlock*)) ),
  num_register_blocks(0)
{}


int Mapping::add_coil_block(CoilBlock& cb)
{
  void* ret = realloc(coil_block, num_coil_blocks+1);
  if(ret)
  {
    coil_block = (CoilBlock**)ret;
    coil_block[num_coil_blocks++] = &cb;
    return 0;
  }
  // error
  return -1; // Out of memory
}


int Mapping::add_register_block(RegisterBlock& rb)
{
  void* ret = realloc(register_block, num_register_blocks+1);
  if(ret)
  {
    register_block = (RegisterBlock**)ret;
    register_block[num_register_blocks++] = &rb;
    return 0;
  }
  // error
  return -1; // Out of memory
}


void Mapping::set_clean()
{
  for(size_t c=0; c<num_coil_blocks; ++c)
      coil_block[c]->set_clean();
  for(size_t r=0; r<num_register_blocks; ++r)
      register_block[r]->set_clean();
  dirty = false;
}


bool Mapping::have_coil_addresses(uint16_t first_addr, uint16_t quantity) const
{
  int rv = find_addresses(coil_block, num_coil_blocks, first_addr, quantity);
  return (rv >= 0);
}


bool Mapping::have_register_addresses(uint16_t first_addr, uint16_t quantity) const
{
  int rv = find_addresses(
      register_block, num_register_blocks, first_addr, quantity);
  return (rv >= 0);
}


void Mapping::write_coil(uint16_t addr, bool value)
{
  write_one(coil_block, num_coil_blocks, addr, value);
}


int8_t Mapping::read_coils(
  uint8_t* dest_byte,
  uint8_t  dest_bit,
  uint16_t src_addr,
  uint16_t quantity
) const
{
  // First check that we have all of the addresses.
  int b0 = find_addresses(coil_block, num_coil_blocks, src_addr, quantity);
  if(b0 < 0)
      return EXC_ILLEGAL_DATA_ADDRESS;

  for(size_t b=b0; b<num_coil_blocks; ++b)
  {
    int8_t rv =
        coil_block[b]->read_many(dest_byte, dest_bit, src_addr, quantity);

    if(rv != 0  ||  quantity == 0)
        return rv;
  }
  return EXC_SERVER_DEVICE_FAILURE; // Should never get here.
}


void Mapping::write_register(uint16_t addr, uint16_t value)
{
  write_one(register_block, num_register_blocks, addr, value);
}


int8_t Mapping::read_registers(
  uint16_t* dest,
  uint16_t  src_addr,
  uint16_t  quantity) const
{
  // First check that we have all of the addresses.
  int b0 = find_addresses(register_block, num_register_blocks, src_addr, quantity);
  if(b0 < 0)
      return EXC_ILLEGAL_DATA_ADDRESS;

  for(size_t b=b0; b<num_register_blocks; ++b)
  {
    int8_t rv = register_block[b]->read_many(dest, src_addr, quantity);

    if(rv != 0  ||  quantity == 0)
        return rv;
  }
  return EXC_SERVER_DEVICE_FAILURE; // Should never get here.
}


} // end namespace modbus
