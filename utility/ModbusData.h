#pragma once

#include "ModbusCommon.h"

#include "Arduino.h"
#include <stdint.h>
#include <stdlib.h>

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


class Mapping
{
public:
  Mapping();
  Mapping(RegisterBlock& holding_register_block);
  Mapping(RegisterBlock& holding_register_block, CoilBlock& coil_block);

  /** Return TRUE if any block has been modified. */
  bool is_dirty() const { return dirty; }

  /** Clear the "dirty" flag. */
  void set_clean();

  //
  // Coils (read/write boolean values).

  void add_coil_block(CoilBlock& cb);
  bool have_coil_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t write_coils(uint16_t dest_addr, uint8_t* src_byte, uint16_t quantity =1);
  int8_t read_coils(uint8_t* dest_byte, uint16_t src_addr, uint16_t quantity =1) const;

  //
  // Discrete inputs (read-only boolean values).

  void add_discrete_input_block(CoilBlock& cb);
  bool have_discrete_input_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t read_discrete_inputs(uint8_t* dest_byte, uint16_t src_addr, uint16_t quantity =1) const;

  //
  // Input registers (read-only 16-bit words).

  void add_input_register_block(RegisterBlock& rb);
  bool have_input_register_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t write_input_registers(uint16_t dest_addr, uint8_t* src, uint16_t quantity =1);
  int8_t read_input_registers(uint8_t* dest, uint16_t src_addr, uint16_t quantity =1) const;

  //
  // Holding registers (read/write 16-bit words).

  void add_holding_register_block(RegisterBlock& rb);
  bool have_holding_register_addresses(uint16_t first_addr, uint16_t quantity =1) const;
  int8_t write_holding_registers(uint16_t dest_addr, uint8_t* src, uint16_t quantity =1);
  int8_t read_holding_registers(uint8_t* dest, uint16_t src_addr, uint16_t quantity =1) const;

private:
  Block*  coil_block_list_head;
  Block*  discrete_input_block_list_head;
  Block*  input_register_block_list_head;
  Block*  holding_register_block_list_head;
  bool    dirty;

private:
  Mapping(const Mapping&); ///< Class is non-copyable.

  void add_block(Block** ptr, Block& new_block);

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
};


} // end namespace modbus
