#pragma once

#include <stdint.h>

namespace modbus {


/** Helper class, used to keep track of buffer position. */
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
    ) = 0;

  virtual int8_t read_many(
      Position& dest,
      uint16_t& src_addr,
      uint16_t& quantity
    ) const = 0;

  virtual ~Block() {}

protected:
  Block(uint16_t length_, uint16_t start_address_);

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


} // end namespace modbus
