#include "ModbusSlave.h"

#include <avr/wdt.h>

namespace modbus {


/* _____PUBLIC FUNCTIONS_____SLAVE___________________________________________ */

/**
 * @brief
 * Constructor for a Slave.
 *
 * For hardware serial through USB/RS232C/RS485 set port to Serial, Serial1,
 * Serial2, or Serial3. (Numbered hardware serial ports are only available on
 * some boards.)
 *
 * For software serial through RS232C/RS485 set port to a SoftwareSerial object
 * that you have already constructed.
 *
 * Only RS485 needs a pin for flow control. Pins 0 and 1 cannot be used.
 *
 * First call begin() on your serial port, and then start up ModbusRtu by
 * calling start(). You can choose the line speed and other port parameters
 * by passing the appropriate values to the port's begin() function.
 *
 * @param u8id   node address, range: 1..247
 * @param port   serial port used
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 */
Slave::Slave(uint8_t u8id_, Stream& port_, uint8_t u8txenpin_):
    Base(port_, u8txenpin_),
    u8id(u8id_),
    listenOnlyMode(false)
{}


/**
 * @brief
 * Method to write a new slave ID address
 *
 * @param 	u8id	new slave address between 1 and 247
 * @ingroup setup
 */
void Slave::setID( uint8_t u8id)
{
    if (( u8id != 0) && (u8id <= 247))
    {
        this->u8id = u8id;
    }
}


/**
 * @brief
 * Method to read current slave ID address
 *
 * @return u8id	current slave address between 1 and 247
 * @ingroup setup
 */
uint8_t Slave::getID() const
{
    return this->u8id;
}


/**
 * @brief Backward compatibility wrapper for Slave::poll()
 * Constructs a basic MODBUS mapping on the fly from an array of uint16_t.
 *
 * @param *regs  register table for communication exchange
 * @param u8size  size of the register table
 * @return        error: <0, nothing to do: 0, request processed: >0
 * @ingroup loop
 */
int8_t Slave::poll( uint16_t *regs, uint8_t u8size )
{
  RegisterBlockData rb(regs, u8size);
  CoilBlockData cb(regs, 16L*u8size);
  Mapping mapping(rb, cb);
  return poll( mapping );
}


/**
 * @brief
 * This method checks if there is any incoming query
 * Afterwards, it would shoot a validation routine plus a register query
 * Avoid any delay() function !!!!
 * After a successful frame between the Master and the Slave, the time-out timer is reset.
 *
 * @param mapping MODBUS mapping for slave data.
 * @return        error: <0, nothing to do: 0, request processed: >0
 * @ingroup loop
 */
int8_t Slave::poll( Mapping& mapping )
{
    if (!rxFrameReady())
        return 0;

    uint8_t au8Buffer[MAX_BUFFER];
    const int8_t i8bytes_read = getRxBuffer( au8Buffer, MAX_BUFFER );
    if (i8bytes_read < 0)
    {
        return setError(i8bytes_read); // Pass error on from getRxBuffer().
    }
    uint8_t u8BufferSize = i8bytes_read;

    // check slave id
    // (id=0 are broadcast messages, which would pass here, if we supported
    //  any function codes that understood them.)
    if (au8Buffer[ ID ] != u8id)
        return 0; // This message is for another slave.

    ++u16Counter[CNT_SLAVE_MESSAGE];

    if (listenOnlyMode && au8Buffer[FUNC] != MB_FC_DIAGNOSTICS)
    {
        ++u16Counter[CNT_SLAVE_NO_RESPONSE];
        return ERR_LISTEN_ONLY_MODE;
    }

    int8_t i8error = 0;
    switch( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUTS:
        i8error = process_FC1( mapping, au8Buffer, u8BufferSize, MAX_BUFFER );
        break;
    case MB_FC_READ_INPUT_REGISTERS:
    case MB_FC_READ_HOLDING_REGISTERS:
        i8error = process_FC3( mapping, au8Buffer, u8BufferSize, MAX_BUFFER );
        break;
    case MB_FC_WRITE_SINGLE_COIL:
        i8error = process_FC5( mapping, au8Buffer, u8BufferSize );
        break;
    case MB_FC_WRITE_SINGLE_REGISTER:
        i8error = process_FC6( mapping, au8Buffer, u8BufferSize );
        break;
    case MB_FC_READ_EXCEPTION_STATUS:
        i8error = process_FC7( mapping, au8Buffer, u8BufferSize );
        break;
    case MB_FC_DIAGNOSTICS:
        i8error = process_FC8( au8Buffer, u8BufferSize );
        break;
    case MB_FC_GET_COMM_EVENT_COUNTER:
        i8error = process_FC11( mapping, au8Buffer, u8BufferSize );
        break;
    case MB_FC_WRITE_MULTIPLE_COILS:
        i8error = process_FC15( mapping, au8Buffer, u8BufferSize );
        break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS:
        i8error = process_FC16( mapping, au8Buffer, u8BufferSize );
        break;
    case MB_FC_READ_WRITE_MULTIPLE_REGISTERS:
        i8error = process_FC23( mapping, au8Buffer, u8BufferSize, MAX_BUFFER );
        break;
    default:
        i8error = EXC_ILLEGAL_FUNCTION;
        break;
    }

    if (i8error > 0)
    {
        ++u16Counter[CNT_SLAVE_EXCEPTION];
        u8BufferSize = buildException( i8error, au8Buffer );
        sendTxBuffer( au8Buffer, u8BufferSize );
    }
    else if (i8error==0)
    {
        sendTxBuffer( au8Buffer, u8BufferSize );
    }
    else
    {
        ++u16Counter[CNT_SLAVE_NO_RESPONSE];
    }
    return setError(i8error);
}


/* ____PRIVATE FUNCTIONS_____SLAVE___________________________________________ */

/**
 * @brief
 * This method builds an exception message. The buffer is assumed to be large enough
 * for the exception - which is only 3 bytes long.
 *
 * @ingroup buffer
 */
int8_t Slave::buildException( uint8_t u8exception, uint8_t* buf )
{
    switch(u8exception)
    {
        case EXC_NEGATIVE_ACKNOWLEDGE:
            ++u16Counter[CNT_SLAVE_NAK];
            break;

        case EXC_SERVER_DEVICE_BUSY:
            ++u16Counter[CNT_SLAVE_BUSY];
            break;

        default:
            ;
    }

    buf[ FUNC ] += 0x80;
    buf[ 2 ]     = u8exception;
    return 3;
}


/**
 * @brief
 * This method processes functions 1 & 2:
 *   MB_FC_READ_COILS & MB_FC_READ_DISCRETE_INPUTS.
 * This method reads a bit array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC1( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize )
{
    // Validate the request message size.
    if (count != 6)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t addr     = demarshal_u16( buf + ADD_HI );
    const uint16_t quantity = demarshal_u16( buf + NB_HI );

    // Set the message size.
    buf[ 2 ] = (quantity+7)/8;
    count = buf[ 2 ] + 3;

    // Validate the requested quantity.
    if(0 == quantity || quantity > 2000 || count > bufsize-2)
        return EXC_ILLEGAL_DATA_VALUE;

    // Read the requested values into buf[3] onwards.
    if (buf[ FUNC ] == MB_FC_READ_COILS)
        return mapping.read_coils(buf + 3, addr, quantity);
    else // MB_FC_READ_DISCRETE_INPUTS
        return mapping.read_discrete_inputs(buf + 3, addr, quantity);
}


/**
 * @brief
 * This method processes functions 3 & 4:
 *   MB_FC_READ_HOLDING_REGISTERS & MB_FC_READ_INPUT_REGISTERS.
 * This method reads a word array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC3( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize )
{
    // Validate the request message size.
    if (count != 6)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t addr     = demarshal_u16( buf + ADD_HI );
    const uint16_t quantity = demarshal_u16( buf + NB_HI );

    // Set the message size.
    buf[ 2 ] = quantity * 2;
    count = buf[ 2 ] + 3;

    // Validate the requested quantity.
    if(0 == quantity || quantity > 125 || count > bufsize-2)
        return EXC_ILLEGAL_DATA_VALUE;

    // Read the requested values into buf[3] onwards.
    if (buf[ FUNC ] == MB_FC_READ_INPUT_REGISTERS)
        return mapping.read_input_registers(buf + 3, addr, quantity);
    else
        return mapping.read_holding_registers(buf + 3, addr, quantity);
}


/**
 * @brief
 * This method processes function 5: MB_FC_WRITE_SINGLE_COIL
 * This method writes a value assigned by the master to a single bit
 *
 * @return count Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC5( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count != 6)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t addr  = demarshal_u16( buf + ADD_HI );
    const uint16_t value = demarshal_u16( buf + NB_HI );

    // Validate the value.
    if(value != 0x0000  &&  value != 0xFF00)
        return EXC_ILLEGAL_DATA_VALUE;

    // Response is an exact echo of the the request.

    // Set addr = value.
    return mapping.write_coils(addr, buf + NB_HI);
}


/**
 * @brief
 * This method processes function 6: MB_FC_WRITE_SINGLE_REGISTER
 * This method writes a value assigned by the master to a single word
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC6( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count != 6)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t addr = demarshal_u16( buf + ADD_HI );

    // Response is an exact echo of the the request.

    // Set addr = value.
    return mapping.write_holding_registers(addr, buf + NB_HI);
}


/**
 * @brief
 * This method processes function 7: MB_FC_READ_EXCEPTION_STATUS
 * This method read a byte from the slave. The byte is interpreted
 * as eight server-defined exception registers.
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC7( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count != 2)
        return EXC_ILLEGAL_DATA_VALUE;

    count = 3;
    return mapping.read_exception_status(buf + 2);
}


/**
 * @brief
 * This method processes function 8: MB_FC_DIAGNOSTICS
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC8( uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count < 6)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t subfunction = demarshal_u16( buf + ADD_HI );

    if (listenOnlyMode && subfunction != 0x01)
        return ERR_LISTEN_ONLY_MODE;

    // Normal response is just to echo the request.

    switch(subfunction)
    {
    case 0x00: // Return query data
        return 0;

    case 0x01: // Restart communications.
        // Must send reply immediately, before the reboot.
        if (!listenOnlyMode)
            sendTxBuffer(buf, count);
        // Use the watchdog timer to restart the board.
        wdt_disable();
        wdt_enable(WDTO_15MS);
        while (true) {}
        // ...never returns

    case 0x02: // Return Diagnostic Register
    case 0x03: // Change ASCII Input Delimiter
        // We don't support this subfunction
        break;

    case 0x04: // Force listen-only mode.
        listenOnlyMode = true;
        return ERR_LISTEN_ONLY_MODE;

    case 0x0A: // Clear Counters and Diagnostic Register
        clearCounters();
        return 0;

    case 0x0B: // Return Bus Message Count
    case 0x0C: // Return Bus Communication Error Count
    case 0x0D: // Return Bus Exception Error Count
    case 0x0E: // Return Slave Message Count
    case 0x0F: // Return Slave No Response Count
    case 0x10: // Return Slave NAK Count
    case 0x11: // Return Slave Busy Count
    case 0x12: // Return Bus Character Overrun Count
        marshal_u16(buf + 4, u16Counter[subfunction - 0x0A]);
        return 0;

    case 0x14: // Clear Overrun Counter ("and Flag")
        u16Counter[CNT_BUS_CHAR_OVERRUN] = 0;
        // Flush the buffers, while we are about it.
        port->flush();
        while(port->read() >= 0);
        return 0;

    default:
        break;
    };
    return EXC_ILLEGAL_FUNCTION;
}


/**
 * @brief
 * This method processes function 11: MB_FC_GET_COMM_EVENT_COUNTER
 *
 * @return count Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC11( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count != 2)
        return EXC_ILLEGAL_DATA_VALUE;

    count = 6;

    // Status word. (User needs to override Mapping::is_busy() for this.)
    if (mapping.is_busy())
        buf[2] = buf[3] = 0xFF;
    else
        buf[2] = buf[3] = 0x00;
    
    // Counts calls to this function, so they can be excluded from the result.
    ++u16Counter[CNT_CALLS_TO_FC11];

    const uint16_t comm_event_count =
        u16Counter[CNT_SLAVE_MESSAGE] - u16Counter[CNT_SLAVE_EXCEPTION]
        - u16Counter[CNT_SLAVE_NO_RESPONSE] - u16Counter[CNT_CALLS_TO_FC11];

    marshal_u16(buf + 4, comm_event_count);
    return 0;
}


/**
 * @brief
 * This method processes function 15: MB_FC_WRITE_MULTIPLE_COILS
 * This method writes a bit array assigned by the master
 *
 * @return count Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC15( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count < 8)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t addr     = demarshal_u16( buf + ADD_HI );
    const uint16_t quantity = demarshal_u16( buf + NB_HI );
    const uint8_t  n        = buf[ 6 ];

    // Validate the quantity.
    if(0 == quantity || quantity > 0x7B0 || n != (quantity+7)/8)
        return EXC_ILLEGAL_DATA_VALUE;

    if (count != n + 7)
        return EXC_ILLEGAL_DATA_VALUE;

    // Response is just the first 6 bytes of the request.
    count = 6;

    // Write the requested values from buf[7] onwards.
    return mapping.write_coils(addr, buf + 7, quantity);
}


/**
 * @brief
 * This method processes function 16: MB_FC_WRITE_MULTIPLE_REGISTERS
 * This method writes a word array assigned by the master
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC16( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count < 9)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t addr     = demarshal_u16( buf + ADD_HI );
    const uint16_t quantity = demarshal_u16( buf + NB_HI );
    const uint8_t  n        = buf[ 6 ];

    // Validate the quantity.
    if(0 == quantity || quantity > 0x7B || n != quantity*2)
        return EXC_ILLEGAL_DATA_VALUE;

    if (count != n + 7)
        return EXC_ILLEGAL_DATA_VALUE;

    // Response is just the first 6 bytes of the request.
    count = 6;

    // Write the requested values from buf[7] onwards.
    return mapping.write_holding_registers(addr, buf + 7, quantity);
}


/**
 * @brief
 * This method processes function 23: MB_FC_READ_WRITE_MULTIPLE_REGISTERS
 *
 * @return 0: OK. Non-zero: error
 * @ingroup register
 */
int8_t Slave::process_FC23( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize )
{
    // Validate the request message size.
    if (count < 13)
        return EXC_ILLEGAL_DATA_VALUE;

    const uint16_t read_addr      = demarshal_u16( buf + 2 );
    const uint16_t read_quantity  = demarshal_u16( buf + 4 );
    const uint16_t write_addr     = demarshal_u16( buf + 6 );
    const uint16_t write_quantity = demarshal_u16( buf + 8 );
    const uint8_t  n              = buf[ 10 ];

    // Validate the quantities.
    if(0 == read_quantity || read_quantity > 0x7D)
        return EXC_ILLEGAL_DATA_VALUE;

    if(0 == write_quantity || write_quantity > 0x79 || n != write_quantity*2)
        return EXC_ILLEGAL_DATA_VALUE;

    if (count != n + 11)
        return EXC_ILLEGAL_DATA_VALUE;

    // Set the response message size.
    buf[ 2 ] = read_quantity * 2;
    count = buf[ 2 ] + 3;

    if(count > bufsize-2)
        return EXC_ILLEGAL_DATA_VALUE;

    // Despite the name, we must perform the write operation *before* the
    // read operation.
    return mapping.write_read_multiple_registers(
        buf + 3,  write_addr, write_quantity,
        buf + 11, read_addr,  read_quantity
      );
}


} // end namespace modbus
