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


struct Position
{
  uint8_t* byte;
  uint8_t  bitn;
  Position(uint8_t* byte_): byte(byte_), bitn(0) {}
};


class Block
{
public:
  /** Return the first valid address in this block. */
  uint16_t get_start_address() const { return start_address; }

  /** Return the number of coils (bits) in this block. */
  uint16_t get_length() const { return length; }

  /** If addr is a valid address in this block, returns the number of
   *  data from addr to the end of the block.
   *  If addr is not in this block, return zero. */
  uint16_t have_address(uint16_t addr) const;

  /** Return TRUE if data in this block has been modified. */
  bool is_dirty() const { return dirty; }

  /** Clear the "dirty" flag. */
  void set_clean() { dirty = false; }

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

  virtual ~Block() {}

protected:
  Block(uint16_t length_, uint16_t start_address_ =0);

  Block*         next_block; ///< Collections are implemented as a linked list.
  const uint16_t length; ///< The number of addresses in this block.
  const uint16_t start_address;
  bool           dirty; ///< TRUE if data in this block has been modified.

private:
  Block();                        ///< Class is non-default-constructable.
  Block(const Block&);            ///< Class is non-copyable.
  Block& operator=(const Block&); ///< Class is non-assignable.
  friend class Mapping;
};


class CoilBlock: public Block
{
public:
  CoilBlock(
      uint8_t*  data_bytes_,
      uint16_t  length_,
      uint16_t  start_address_ = 0
    );

  int8_t write_many(
      uint16_t& dest_addr,
      Position& src,
      uint16_t& quantity
    );

  int8_t read_many(
      Position& dest,
      uint16_t& src_addr,
      uint16_t& quantity
    ) const;

private:
  uint8_t* const  data_bytes;
};


class RegisterBlock: public Block
{
public:
  RegisterBlock(
      uint16_t* data_words_,
      uint16_t  length_, ///< Number of registers (2-byte words) in this block.
      uint16_t  start_address_ = 0
    );

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

private:
  RegisterBlock(const RegisterBlock&); ///< Class is non-copyable.

  uint16_t* const  data_words;
};


class Mapping
{
public:
  Mapping();
  void add_coil_block(CoilBlock& cb);
  void add_register_block(RegisterBlock& rb);

  /** Return TRUE if any block has been modified. */
  bool is_dirty() const { return dirty; }

  /** Clear the "dirty" flag. */
  void set_clean();

  bool have_coil_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  bool have_register_addresses(uint16_t first_addr, uint16_t quantity =1) const;

  int8_t write_coils(
      uint16_t dest_addr,
      uint8_t* src_byte,
      uint16_t quantity =1
    );

  /** Also used for reading "discrete inputs"
   *  (MODBUS terminology for read-only booleans).
   *
   * @return  0: success, <0: internal error, >0: MODBUS exception
   */
  int8_t read_coils(
      uint8_t* dest_byte,
      uint16_t src_addr,
      uint16_t quantity =1
    ) const;

  int8_t write_registers(
      uint16_t  dest_addr,
      uint8_t*  src,
      uint16_t  quantity =1
    );

  int8_t read_registers(
      uint8_t*  dest,
      uint16_t  src_addr,
      uint16_t  quantity =1
    ) const;

private:
  Block*  coil_block_list_head;
  Block*  register_block_list_head;
  bool    dirty;

private:
  Mapping(const Mapping&); ///< Class is non-copyable.

  void add_block(Block** ptr, Block& new_block);

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
};


} // end namespace modbus
