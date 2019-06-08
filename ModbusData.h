#pragma once

#include <stdint.h>
#include <stdlib.h>


namespace modbus {


class CoilBlock
{
private:
  uint8_t* const  data_bytes;
  const uint16_t  length; ///< The number of coils (bits) in this block.
  const uint16_t  start_address;
  bool            dirty; ///< TRUE if data in this block has been modified.

public:
  CoilBlock(
      uint8_t*  data_bytes_,
      uint16_t  length_,
      uint16_t  start_address_ = 0
    ):
      data_bytes(data_bytes_),
      length(length_), ///< The number of coils (bits) in this block.
      start_address(start_address_),
      dirty(false)
    {}

private:
  CoilBlock(const CoilBlock&); ///< Class is non-copyable.

public:
  /** Return the first valid address in this block. */
  uint16_t get_start_address() const
    {
      return start_address;
    }

  /** Return the number of coils (bits) in this block. */
  uint16_t get_length() const
    {
      return length;
    }

  /** If addr is a valid address in this block, returns the number of
   *  coils (bits) from addr to the end of the block.
   *  If addr is not in this block, return zero. */
  uint16_t have_address(uint16_t addr) const
    {
      if(addr >= start_address && addr < start_address + length)
          return start_address - addr + length;
      else
          return 0;
    }

  /** Return TRUE if data in this block has been modified. */
  bool is_dirty() const
    {
      return dirty;
    }

  /** Clear the "dirty" flag. */
  bool set_clean()
    {
      dirty = false;
    }


  //
  // No checks are made.
  // The caller is assumed to have checked that the address is valid.

  void write_one(uint16_t addr, bool value)
    {
      dirty = true;
      const uint16_t offset = addr - start_address;
      bitWrite(data_bytes[offset/8], offset%8, value);
    }

  uint16_t read_many(
      uint16_t addr,
      uint8_t* dest_byte,
      uint8_t  dest_bit,
      uint16_t num_to_read =1) const
    {
      const uint16_t offset = addr - start_address;
      const uint8_t* src = data_bytes + (offset / 8);
      uint8_t        src_bit = offset % 8;

      if(num_to_read > length - offset)
           num_to_read = length - offset;

      for(size_t i=0; i<num_to_read; ++i)
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
      return num_to_read;
    }
};


class RegisterBlock
{
private:
  uint16_t* data_words;
  uint16_t  length; ///< The number of registers (2-byte words) in this block.
  uint16_t  start_address;
  bool      dirty; ///< TRUE if data in this block has been modified.

public:
  RegisterBlock(
      uint16_t* data_words_,
      uint16_t  length_, ///< Number of registers (2-byte words) in this block.
      uint16_t  start_address_ = 0
    ):
      data_words(data_words_),
      length(length_),
      start_address(start_address_),
      dirty(false)
    {}

public:
  /** Return the first valid address in this block. */
  uint16_t get_start_address() const
    {
      return start_address;
    }

  /** Return the number of registers (2-byte words) in this block. */
  uint16_t get_length() const
    {
      return length;
    }

  /** If addr is a valid address in this block, returns the number of
   *  registers (2-byte words) from addr to the end of the block.
   *  If addr is not in this block, return zero. */
  uint16_t have_address(uint16_t addr) const
    {
      if(addr >= start_address && addr < start_address + length)
          return start_address - addr + length;
      else
          return 0;
    }

  /** Return TRUE if data in this block has been modified. */
  bool is_dirty() const
    {
      return dirty;
    }

  /** Clear the "dirty" flag. */
  bool set_clean()
    {
      dirty = false;
    }


  //
  // No checks are made.
  // The caller is assumed to have checked that the address is valid.

  void write_one(uint16_t addr, uint16_t value)
    {
      dirty = true;
      data_words[ addr - start_address ] = value;
    }

  uint16_t read_many(
      uint16_t  addr,
      uint16_t* dest,
      uint16_t  num_to_read =1) const
    {
      const uint16_t offset = addr - start_address;

      if(num_to_read > length - offset)
           num_to_read = length - offset;

      memcpy((void*)dest, data_words + offset, num_to_read * sizeof(uint16_t));
      return num_to_read;
    }
};




class Mapping
{
private:
  CoilBlock**     coil_block;
  uint16_t        num_coil_blocks;

  RegisterBlock** register_block;
  uint16_t        num_register_blocks;

  bool  dirty;

public:
  Mapping(uint16_t num_coil_blocks = 0, uint16_t num_register_blocks = 0):
      coil_block( (CoilBlock**)calloc(num_coil_blocks, sizeof(CoilBlock*)) ),
      num_coil_blocks(0),
      register_block( (RegisterBlock**)calloc(num_coil_blocks, sizeof(RegisterBlock*)) ),
      num_register_blocks(0)
    {}

  int add_coil_block(CoilBlock& cb)
    {
      void* ret = realloc(coil_block, num_coil_blocks+1);
      if(ret)
      {
        coil_block = (CoilBlock**)ret;
        coil_block[num_coil_blocks++] = &cb;
        return 0;
      }
      // error
      return -1;
    }

  int add_register_block(RegisterBlock& rb)
    {
      void* ret = realloc(register_block, num_register_blocks+1);
      if(ret)
      {
        register_block = (RegisterBlock**)ret;
        register_block[num_register_blocks++] = &rb;
        return 0;
      }
      // error
      return -1;
    }

private:
  Mapping(const Mapping&); ///< Class is non-copyable.

public:
  /** Return TRUE if any block has been modified. */
  bool is_dirty() const
    {
      return dirty;
    }

  /** Clear the "dirty" flag. */
  bool set_clean()
    {
      for(size_t c=0; c<num_coil_blocks; ++c)
          coil_block[c]->set_clean();
      for(size_t r=0; r<num_register_blocks; ++r)
          register_block[r]->set_clean();
      dirty = false;
    }

  bool have_coil_addresses(uint16_t first_addr, uint16_t num_coils) const
    {
      return have_addresses(coil_block, num_coil_blocks, first_addr, num_coils);
    }

  bool have_register_addresses(uint16_t first_addr, uint16_t num_coils) const
    {
      return have_addresses(
          register_block, num_register_blocks, first_addr, num_coils);
    }

  void write_coil(uint16_t addr, bool value)
    {
      write_one(coil_block, num_coil_blocks, addr, value);
    }

  /** Also used for reading "discrete inputs"
   *  (MODBUS terminology for read-only booleans). */
  void read_coils(
      uint16_t addr,
      uint8_t* dest_byte,
      uint8_t  dest_bit,
      uint16_t num_to_read =1
    ) const
    {
      for(size_t i=0; i<num_coil_blocks; ++i)
      {
        // If addr is not found, then this algorithm will search through every
        // block, when it could give up earlier. Not bothering with that extra
        // test speeds up the normal case, where addr is found.
        if(coil_block[i]->have_address(addr))
        {
          uint16_t num_read =
              coil_block[i]->read_many(addr, dest_byte, dest_bit, num_to_read);

          if(num_read == num_to_read)
              return;

          num_to_read -= num_read;
          num_read    += dest_bit;
          dest_byte   += num_read/8;
          dest_bit    =  num_read%8;
        }
      }
    }

  void write_register(uint16_t addr, uint16_t value)
    {
      write_one(register_block, num_register_blocks, addr, value);
    }

  void read_registers(
      uint16_t  addr,
      uint16_t* dest,
      uint16_t  num_to_read =1) const
    {
      for(size_t i=0; i<num_register_blocks; ++i)
      {
        // If addr is not found, then this algorithm will search through every
        // block, when it could give up earlier. Not bothering with that extra
        // test speeds up the normal case, where addr is found.
        if(register_block[i]->have_address(addr))
        {
          uint16_t num_read =
              register_block[i]->read_many(addr, dest, num_to_read);

          if(num_read == num_to_read)
              return;

          num_to_read -= num_read;
          dest        += num_read;
        }
      }
    }

private:
  /** Helper for implementing have_coil/register_addresses(). */
  template<typename T_Block>
  bool have_addresses(
      const T_Block* const block, uint16_t num_blocks,
      uint16_t first_addr, uint16_t num_addr) const
    {
      for(size_t i=0; i<num_blocks; ++i)
      {
        uint16_t n = block[i]->have_address(first_addr);
        if(n >= num_addr)
        {
          return true;
        }
        else if(n)
        {
          num_addr -= n;
          first_addr += n;
        }
        else if(block[i]->get_start_address() > first_addr)
        {
          break;
        }
      }
      return false;
    }

  /** Helper for implementing write_coil() & write_register(). */
  template<typename T_Block, typename T_value>
  void write_one(
      T_Block* const block, uint16_t num_blocks,
      uint16_t addr, T_value value)
    {
      for(size_t i=0; i<num_blocks; ++i)
      {
        if(block[i]->have_address(addr))
        {
          dirty = true;
          block[i]->write_one(addr, value);
          return;
        }
      }
      // ?? Error?
    }

};


} // end namespace modbus
