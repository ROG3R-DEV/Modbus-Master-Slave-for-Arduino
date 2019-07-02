#include "ModbusMaster.h"

namespace modbus {


/* _____PUBLIC FUNCTIONS_____MASTER__________________________________________ */

/**
 * @brief
 * Constructor for a Master.
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
 * First call begin() on your serial port, and then start up the Master by
 * calling start(). You can choose the line speed and other port parameters
 * by passing the appropriate values to the port's begin() function.
 *
 * @param port   serial port used
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 */
Master::Master(Stream& port_, uint8_t u8txenpin_):
    Base(port_, u8txenpin_),
    u8state(COM_IDLE),
    au16regs(NULL),
    u16timeOut(1000),
    u32timeOut(0)
{}


/**
 * @brief
 * Initialize time-out parameter
 *
 * Call once class has been instantiated, typically within setup().
 * The time-out timer is reset each time that there is a successful communication
 * between Master and Slave. It works for both.
 *
 * @param time-out value (ms)
 * @ingroup setup
 */
void Master::setTimeOut( uint16_t u16timeOut)
{
    this->u16timeOut = u16timeOut;
}


/**
 * @brief
 * Return communication Watchdog state.
 * It could be useful to reset outputs if the watchdog is fired.
 *
 * @return TRUE if millis() > u32timeOut
 * @ingroup loop
 */
bool Master::timeOutExpired() const
{
    return ((unsigned long)(millis() -u32timeOut) > (unsigned long)u16timeOut);
}


/**
 * Get modbus master state
 *
 * @return = 0 IDLE, = 1 WAITING FOR ANSWER
 * @ingroup buffer
 */
uint8_t Master::getState() const
{
    return u8state;
}


/**
 * @brief
 * Generate a query to an slave with a modbus_t telegram structure
 * The Master must be in COM_IDLE mode. After it, its state would be COM_WAITING.
 * This method has to be called only in loop() section.
 *
 * @see modbus_t
 * @param modbus_t  modbus telegram structure (id, fct, ...)
 * @return          success: 0, error: <0
 * @ingroup loop
 * @todo finish function 15
 */
int8_t Master::query( modbus_t telegram )
{
    if (u8state != COM_IDLE)
        return setError(ERR_WAITING);

    if ((telegram.u8id==0) || (telegram.u8id>247))
        return setError(ERR_BAD_SLAVE_ID);

    au16regs = telegram.au16reg;

    // The buffer into which we will write the query.
    uint8_t au8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;

    // telegram header
    au8Buffer[ ID ]   = telegram.u8id;
    au8Buffer[ FUNC ] = telegram.u8fct;
    marshal_u16( au8Buffer + ADD_HI, telegram.u16RegAdd );

    switch( telegram.u8fct )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUTS:
    case MB_FC_READ_HOLDING_REGISTERS:
    case MB_FC_READ_INPUT_REGISTERS:
        marshal_u16( au8Buffer + NB_HI, telegram.u16CoilsNo );
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_SINGLE_COIL:
        au8Buffer[ NB_HI ] = ((au16regs[0] > 0) ? 0xff : 0);
        au8Buffer[ NB_LO ] = 0;
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_SINGLE_REGISTER:
        marshal_u16( au8Buffer + NB_HI, au16regs[0] );
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_MULTIPLE_COILS:
        marshal_u16( au8Buffer + NB_HI, telegram.u16CoilsNo );
        au8Buffer[ BYTE_CNT ] = (telegram.u16CoilsNo + 7)/8;
        u8BufferSize = 7 + au8Buffer[ BYTE_CNT ];
        // ?? ASSUMING MACHINE IS LITTLE-ENDIAN ??
        memcpy( au8Buffer + 7, au16regs, au8Buffer[ BYTE_CNT ] );
        break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS:
        marshal_u16( au8Buffer + NB_HI, telegram.u16CoilsNo );
        au8Buffer[ BYTE_CNT ] = static_cast<uint8_t>( telegram.u16CoilsNo * 2 );
        u8BufferSize = 7;

        for (uint16_t i=0; i< telegram.u16CoilsNo; i++)
        {
            marshal_u16( au8Buffer + u8BufferSize, au16regs[ i ] );
            u8BufferSize += 2;
        }
        break;

    default:
        return setError(ERR_FUNC_CODE); // Unrecognised or unsupported function code.
    }

    const int8_t i8bytes_sent = sendTxBuffer( au8Buffer, u8BufferSize, MAX_BUFFER );
    if (i8bytes_sent < 0)
    {
        return setError(i8bytes_sent);
    }
    ++u16Counter[CNT_MASTER_QUERY];
    u32timeOut = millis();
    u8state = COM_WAITING;
    return 0;
}


/**
 * @brief
 * This method checks if there is any incoming answer if pending.
 * If there is no answer, it would change Master state to COM_IDLE.
 * This method must be called only at loop section.
 * Avoid any delay() function.
 *
 * Any incoming data would be redirected to au16regs pointer,
 * as defined in its modbus_t query telegram.
 *
 * @params  nothing
 * @return  error: <0, still waiting: 0, answer arrived: >0
 * @ingroup loop
 */
int8_t Master::poll()
{
    if (u8state != COM_WAITING)
        return 0; // Not expecting any incoming messages.

    if (timeOutExpired())
    {
        u8state = COM_IDLE;
        ++u16Counter[CNT_MASTER_TIMEOUT];
        return setError(ERR_TIME_OUT_EXPIRED);
    }

    if (!rxFrameReady())
        return 0;

    // transfer Serial buffer frame to a local buffer.
    uint8_t au8Buffer[MAX_BUFFER];
    const int8_t i8bytes_read = getRxBuffer( au8Buffer, MAX_BUFFER );
    if (i8bytes_read < 0)
    {
        u8state = COM_IDLE;
        return setError(i8bytes_read); // Pass error on from getRxBuffer().
    }
    uint8_t u8BufferSize = i8bytes_read;

    ++u16Counter[CNT_MASTER_RESPONSE];

    // validate message: length, exception
    int8_t i8error = validateAnswer( au8Buffer, u8BufferSize );

    if (i8error == 0)
    {
        // process answer

        // BUG: Need to check that the response matches the Slave ID and
        //      function code that we are actually waiting for.
        //      If the bus has multiple slaves, the current implementation
        //      could cause chaos.

        switch( au8Buffer[ FUNC ] )
        {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
            // call get_FC1 to transfer the incoming message to au16regs buffer
            // BUG: Need to check bounds of au16regs - response might not fit.
            get_FC1( au8Buffer, u8BufferSize );
            break;
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_READ_HOLDING_REGISTERS :
            // call get_FC3 to transfer the incoming message to au16regs buffer
            // BUG: Need to check bounds of au16regs - response might not fit.
            get_FC3( au8Buffer, u8BufferSize );
            break;
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER :
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS :
            // nothing to do
            break;
        default:
            i8error = ERR_FUNC_CODE;
            break;
        }
    }

    u8state = COM_IDLE;
    if (i8error == 0)
        return 1; // Response received and processed OK.
    else if (i8error > 0)
        ++u16Counter[CNT_MASTER_EXCEPTION];
    else
        ++u16Counter[CNT_MASTER_IGNORED];
    return setError(i8error);
}


/* ____PRIVATE FUNCTIONS_____MASTER__________________________________________ */

/**
 * @brief
 * This method validates master incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
int8_t Master::validateAnswer( const uint8_t* buf, uint8_t count ) const
{
    if (count < 5) // Exception is 3 bytes + 2 bytes CRC.
    {
        return ERR_MALFORMED_MESSAGE;
    }

    // check exception
    if ((buf[ FUNC ] & 0x80) != 0)
    {
        if (buf[ 2 ] <= INT8_MAX)
            return buf[ 2 ];
        else
            return ERR_EXCEPTION;
    }

    // Check for minimum message size.
    if (count < 6) // FC 1 & 2: smallest response is 4 bytes + 2 bytes CRC
    {
        return ERR_MALFORMED_MESSAGE;
    }

    return 0; // OK, no exception code thrown
}


/**
 * This method processes functions 1 & 2 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 * TODO: finish its implementation
 */
void Master::get_FC1( const uint8_t* buf, uint8_t /*count*/ )
{
    // Tell the compiler that buf & au16regs point to different memory.
    const uint8_t* const __restrict__ buf_data  = buf + 3;
    uint16_t*      const __restrict__ user_data = au16regs;

    const uint8_t byte_count = buf[ 2 ];

    // Make sure the high byte of the last word we write is zeroed.
    if (byte_count % 2)
        user_data[ byte_count/2 ] = 0;

    // ?? ASSUMING MACHINE IS LITTLE-ENDIAN ??
    memcpy(user_data, buf_data, byte_count);
}


/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void Master::get_FC3( const uint8_t* __restrict__ buf, uint8_t /*count*/ )
{
    // Tell the compiler that buf & au16regs point to different memory.
    const uint8_t* const __restrict__  buf_data  = buf + 3;
    uint16_t*      const __restrict__  user_data = au16regs;

    const uint8_t num_registers = buf[ 2 ] / 2;
    for (uint8_t i=0; i<num_registers; i++)
    {
        user_data[ i ] = demarshal_u16( buf_data + (i*2) );
    }
}


} // end namespace modbus
