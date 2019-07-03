#include "RegisterBlock.h"

#include "ModbusCommon.h"

namespace modbus {


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


} // end namespace modbus
