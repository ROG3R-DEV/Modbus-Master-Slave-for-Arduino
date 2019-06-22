#pragma once

#include "ModbusCommon.h"

#include "Arduino.h"
#include <stdint.h>
#include <stdlib.h>

namespace modbus {


/** Only need to swap endianness on little endian machines. */
inline uint16_t bswap16(uint16_t v) __attribute__((always_inline));
inline uint16_t bswap16(uint16_t v)
{
#if !defined(__BYTE_ORDER__) || !defined(__ORDER_LITTLE_ENDIAN__)
#  error "Cannot determine byte order."
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#  if __GNUC__ > 4 || (__GNUC__ == 4  &&  __GNUC_MINOR__ > 7)
      return __builtin_bswap16(v);
#  else
      return (v<<8)|(v>>8);
#  endif
#else
      return v;
#endif
}


class CoilBlock
{
public:
  CoilBlock(
      uint8_t*  data_bytes_,
      uint16_t  length_,
      uint16_t  start_address_ = 0
    );

  /** Return the first valid address in this block. */
  uint16_t get_start_address() const { return start_address; }

  /** Return the number of coils (bits) in this block. */
  uint16_t get_length() const { return length; }

  /** If addr is a valid address in this block, returns the number of
   *  coils (bits) from addr to the end of the block.
   *  If addr is not in this block, return zero. */
  uint16_t have_address(uint16_t addr) const;

  /** Return TRUE if data in this block has been modified. */
  bool is_dirty() const { return dirty; }

  /** Clear the "dirty" flag. */
  void set_clean() { dirty = false; }

  int8_t write_one(uint16_t addr, bool value);

  int8_t write_many(
      uint16_t& dest_addr,
      uint8_t*& src_byte,
      uint8_t&  src_bit,
      uint16_t& quantity);

  int8_t read_many(
      uint8_t*& dest_byte,
      uint8_t&  dest_bit,
      uint16_t& src_addr,
      uint16_t& quantity) const;

private:
  CoilBlock(const CoilBlock&); ///< Class is non-copyable.

  uint8_t* const  data_bytes;
  const uint16_t  length; ///< The number of coils (bits) in this block.
  const uint16_t  start_address;
  bool            dirty; ///< TRUE if data in this block has been modified.
};


class RegisterBlock
{
public:
  RegisterBlock(
      uint16_t* data_words_,
      uint16_t  length_, ///< Number of registers (2-byte words) in this block.
      uint16_t  start_address_ = 0
    );

  /** Return the first valid address in this block. */
  uint16_t get_start_address() const { return start_address; }

  /** Return the number of registers (2-byte words) in this block. */
  uint16_t get_length() const { return length; }

  /** If addr is a valid address in this block, returns the number of
   *  registers (2-byte words) from addr to the end of the block.
   *  If addr is not in this block, return zero. */
  uint16_t have_address(uint16_t addr) const;

  /** Return TRUE if data in this block has been modified. */
  bool is_dirty() const { return dirty; }

  /** Clear the "dirty" flag. */
  void set_clean() { dirty = false; }

  //
  // No checks are made.
  // The caller is assumed to have checked that the address is valid.

  int8_t write_many(
      uint16_t&  addr,
      uint16_t*& src,
      uint16_t&  quantity);

  int8_t read_many(
      uint16_t*& dest,
      uint16_t&  src_addr,
      uint16_t&  quantity) const;

private:
  RegisterBlock(const RegisterBlock&); ///< Class is non-copyable.

  uint16_t* data_words;
  uint16_t  length; ///< The number of registers (2-byte words) in this block.
  uint16_t  start_address;
  bool      dirty; ///< TRUE if data in this block has been modified.
};




class Mapping
{
public:
  Mapping(uint16_t num_coil_blocks = 0, uint16_t num_register_blocks = 0);
  ~Mapping();
  int add_coil_block(CoilBlock& cb);
  int add_register_block(RegisterBlock& rb);

  /** Return TRUE if any block has been modified. */
  bool is_dirty() const { return dirty; }

  /** Clear the "dirty" flag. */
  void set_clean();

  bool have_coil_addresses(uint16_t first_addr, uint16_t quantity =1) const;

  bool have_register_addresses(uint16_t first_addr, uint16_t quantity =1) const;

  int8_t write_coil(uint16_t addr, bool value);

  int8_t write_coils(
      uint16_t dest_addr,
      uint8_t* src_byte,
      uint8_t  src_bit,
      uint16_t quantity =1
    );

  /** Also used for reading "discrete inputs"
   *  (MODBUS terminology for read-only booleans).
   *
   * @return  0: success, <0: internal error, >0: MODBUS exception
   */
  int8_t read_coils(
      uint8_t* dest_byte,
      uint8_t  dest_bit,
      uint16_t src_addr,
      uint16_t quantity =1
    ) const;

  int8_t write_register(uint16_t addr, uint16_t value);

  int8_t write_registers(
      uint16_t  dest_addr,
      uint16_t* src,
      uint16_t  quantity =1);

  int8_t read_registers(
      uint16_t* dest,
      uint16_t  src_addr,
      uint16_t  quantity =1) const;

private:
  CoilBlock**     coil_block;
  uint16_t        num_coil_blocks;
  RegisterBlock** register_block;
  uint16_t        num_register_blocks;
  bool            dirty;

  Mapping(const Mapping&); ///< Class is non-copyable.

  /** Helper for implementing have_coil/register_addresses().
   *  Looks for ALL of the required address range.
   *
   * @return  <0: not found, >=0: block num that contains first address.
   */
  template<typename T_Block>
  int find_addresses(
      const T_Block* const block, uint16_t num_blocks,
      uint16_t first_addr, uint16_t num_addr) const
    {
      int result = -1;
      for(size_t i=0; i<num_blocks; ++i)
      {
        uint16_t n = block[i]->have_address(first_addr);
        if(n)
        {
          // Result refers to the block that contains the first_addr.
          if(result < 0)
              result = i;

          if(n >= num_addr)
              return result;

          num_addr -= n;
          first_addr += n;
        }
        else if(block[i]->get_start_address() > first_addr)
        {
          // We have passed the address we are looking for - give up.
          break;
        }
      }
      return -2;
    }

  /** Helper for implementing write_coil() & write_register(). */
  template<typename T_Block, typename T_value>
  int8_t write_one(
      T_Block* const block, uint16_t num_blocks,
      uint16_t addr, T_value value)
    {
      for(size_t i=0; i<num_blocks; ++i)
      {
        if(block[i]->have_address(addr))
        {
          dirty = true;
          return block[i]->write_one(addr, value);
        }
      }
      return EXC_ILLEGAL_DATA_ADDRESS;
    }
};


} // end namespace modbus
