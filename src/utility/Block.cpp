#include "Block.h"

namespace modbus {


Block::Block(
    uint16_t  length_,
    uint16_t  start_address_
  ):
    length(length_), ///< The number of coils (bits) in this block.
    start_address(start_address_),
    dirty(false)
{}


/** If addr is a valid address in this block, returns the number of
*  coils (bits) from addr to the end of the block.
*  If addr is not in this block, return zero. */
uint16_t Block::have_address(uint16_t addr) const
{
  if(addr >= start_address && addr < start_address + length)
      return start_address - addr + length;
  else
      return 0;
}


} // end namespace modbus
