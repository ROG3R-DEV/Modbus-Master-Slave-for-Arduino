#pragma once

#include "Block.h"

namespace modbus {


class CoilBlock: public Block
{
public:
  /** dest_addr is guaranteed to be in the range (start_address, start_address+length].
   *  This method must set the 'dirty' flag, if the object's state has changed. */
  virtual int8_t write_one(uint16_t dest_addr, bool value) = 0;

  /** src_addr is guaranteed to be in the range (start_address, start_address+length]. */
  virtual int8_t read_one(uint16_t src_addr, bool& value) const = 0;

  virtual int8_t write_many(
      uint16_t& dest_addr,
      Position& src,
      uint16_t& quantity
    );

  virtual int8_t read_many(
      Position& dest,
      uint16_t& src_addr,
      uint16_t& quantity
    ) const;

protected:
  CoilBlock(uint16_t l_, uint16_t s_): Block(l_, s_) {}
};


class CoilBlockData: public CoilBlock
{
public:
  CoilBlockData(
      uint8_t*  data_bytes_,
      uint16_t  length_,
      uint16_t  start_address_ = 0
    );

  CoilBlockData(
      uint16_t* data_words_,
      uint16_t  length_,
      uint16_t  start_address_ = 0
    );

  virtual int8_t write_one(uint16_t dest_addr, bool value);
  virtual int8_t read_one(uint16_t src_addr, bool& value) const;

private:
  uint8_t* const  data_bytes;
};


} // end namespace modbus
