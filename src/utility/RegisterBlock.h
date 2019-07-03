#pragma once

#include "Block.h"

#include <stdlib.h>

namespace modbus {


class RegisterBlock: public Block
{
public:
  /** dest_addr is guaranteed to be in the range (start_address, start_address+length].
   *  This method must set the 'dirty' flag, if the object's state has changed. */
  virtual int8_t write_one(uint16_t dest_addr, uint16_t value) = 0;

  /** src_addr is guaranteed to be in the range (start_address, start_address+length]. */
  virtual int8_t read_one(uint16_t src_addr, uint16_t& value) const = 0;

  int8_t write_many(
      uint16_t& dst_addr,
      Position& src,
      uint16_t& quantity
    );

  int8_t read_many(
      Position& dest,
      uint16_t& src_addr,
      uint16_t& quantity
    ) const;

protected:
  RegisterBlock(uint16_t l_, uint16_t s_): Block(l_, s_) {}
};


class RegisterBlockData: public RegisterBlock
{
public:
  RegisterBlockData(
      uint16_t* data_words_,
      uint16_t  length_, ///< Number of registers (2-byte words) in this block.
      uint16_t  start_address_ = 0
    );

  virtual int8_t write_one(uint16_t dest_addr, uint16_t value);
  virtual int8_t read_one(uint16_t src_addr, uint16_t& value) const;

  /** Helper - access the data buffer as an object type.
   *  You can use this with placement new, to use the RegisterBlockData as
   *  arbitrary memory...
   *
   *      // Construct an object of type MyClass at MODBUS address 1234...
   *      MyClass* obj = new (rbd.address2object<MyClass>(1234)) MyClass;
   *
   *      // Access the object later...
   *      rbd.address2object<MyClass>(1234)->myClassMethod();
   */
  template<typename T>
  T* address2object(uint16_t addr)
    {
      // Only perform the cast if the object fits into the buffer.
      if(have_address(addr) * sizeof(uint16_t) >= sizeof(T))
          return static_cast<T*>(data_words + (addr - start_address));
      else
          return NULL;
    }

  /** Default to using the start of the data block. */
  template<typename T>
  T* address2object()
    { return address2object<T>(start_address); }

private:
  uint16_t* const  data_words;
};


} // end namespace modbus
