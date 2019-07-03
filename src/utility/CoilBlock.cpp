#include "CoilBlock.h"

#include <Arduino.h>

namespace modbus {


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


} // end namespace modbus
