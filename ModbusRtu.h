/**
 * @file        ModbusRtu.h
 * @version     1.22
 * @date        2019.06.29
 * @author      Samuel Marco i Armengol
 * @contact     sammarcoarmengol@gmail.com
 * @contribution Helium6072
 * @contribution gabrielsan
 * @contribution alextingle
 *
 * @description
 *  Arduino library for communicating with Modbus devices
 *  over RS232/USB/485 via RTU protocol.
 *
 *  Further information:
 *  http://modbus.org/
 *  http://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 *
 * @license
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; version
 *  2.1 of the License.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * @defgroup setup Modbus Object Instantiation/Initialization
 * @defgroup loop Modbus Object Management
 * @defgroup buffer Modbus Buffer Management
 * @defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
 * @defgroup register Modbus Function Codes for Holding/Input Registers
 *
 */

#ifndef MODBUS_MODBUS_RTU_H
#define MODBUS_MODBUS_RTU_H

#include "utility/ModbusCommon.h"
#include "utility/ModbusData.h"

#include <inttypes.h>
#include "Arduino.h"

namespace modbus {


/** @class Base
 *  @brief Base class for Master and Slave classes.
 *
 *  You cannot use this class on its own. */
class Base
{
private:
    Base(); //!< Not default constructable.
    Base(const Base&); //!< Not copyable.
    Base& operator= (const Base&); //!< Not assignable.

protected:
    Stream* port; //!< Pointer to Stream class object (Either HardwareSerial or SoftwareSerial)
    uint8_t u8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >1=RS-485 mode
    int8_t  i8lastError;
    int     iLastBytesAvailable;
    uint16_t u16InCnt;
    uint16_t u16OutCnt;
    uint16_t u16errCnt;
    unsigned long  ulT35timer; // Type matches millis() return type.
    uint32_t u32overTime;

protected:
    Base(Stream& port, uint8_t u8txenpin =0);

    int8_t setError( int8_t i8error );
    bool   rxFrameReady();
    int8_t getRxBuffer( uint8_t* buf, uint8_t count );
    int8_t sendTxBuffer( uint8_t* buf, uint8_t count, uint8_t size );

public:
    void start();
    uint16_t getInCnt() const; //!<number of incoming messages
    uint16_t getOutCnt() const; //!<number of outcoming messages
    uint16_t getErrCnt() const; //!<error counter
    int8_t   getLastError() const; //!< Get last error (ERR_XXX) or exception (EXC_XXX) code.
    void     clearLastError(); //!< Set last error to 0.
    void setTxendPinOverTime( uint32_t u32overTime );

    static uint16_t calcCRC( const void* data, uint8_t len );

    friend class Modbus;
};


/**
 * @class Master
 * @brief
 * Arduino class library for communicating with Modbus slave devices over
 * USB/RS232/485 (via RTU protocol).
 */
class Master: public Base
{
private:
    Master(); //!< Not default constructable.
    Master(const Master&); //!< Not copyable.
    Master& operator= (const Master&); //!< Not assignable.

    uint8_t   u8state;
    uint16_t* au16regs;
    uint16_t  u16timeOut;
    uint32_t  u32timeOut; //!< Timestamp of last query (millis).

private:
    int8_t validateAnswer( const uint8_t* buf, uint8_t count ) const;
    void get_FC1( const uint8_t* buf, uint8_t count );
    void get_FC3( const uint8_t* buf, uint8_t count );

public:
    Master(Stream& port, uint8_t u8txenpin =0);

    void setTimeOut( uint16_t u16timeOut); //!<write communication watch-dog timer
    bool timeOutExpired() const; //!<get communication watch-dog timer state
    uint8_t getState() const;

    int8_t query( modbus_t telegram ); //!<only for master
    int8_t poll(); //!<cyclic poll for master
};


/**
 * @class Slave
 * @brief
 * Arduino class library for communicating with Modbus master devices over
 * USB/RS232/485 (via RTU protocol).
 */
class Slave: public Base
{
private:
    Slave(); //!< Not default constructable.
    Slave(const Slave&); //!< Not copyable.
    Slave& operator= (const Slave&); //!< Not assignable.

    uint8_t u8id; //!< Slave ID: 1..247

private:
    int8_t validateRequest( const uint8_t* buf, uint8_t count ) const;
    int8_t buildException( uint8_t u8exception, uint8_t* buf ) const;
    int8_t process_FC1( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize );
    int8_t process_FC3( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize );
    int8_t process_FC5( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC6( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC15( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC16( Mapping& mapping, uint8_t* buf, uint8_t& count );

public:
    Slave(uint8_t u8id, Stream& port, uint8_t u8txenpin =0);

    void setID( uint8_t u8id ); //!<write new ID for the slave
    uint8_t getID() const; //!<get slave ID between 1 and 247

    int8_t poll( uint16_t *regs, uint8_t u8size ); //!<cyclic poll for slave
    int8_t poll( Mapping& mapping );               //!<cyclic poll for slave
};


/**
 * @class Modbus
 * @brief Backwards-compatibility shim for Master and Slave classes.
 */
class Modbus
{
private:
    Modbus(); //!< Not default constructable.
    Modbus(const Modbus&); //!< Not copyable.
    Modbus& operator= (const Modbus&); //!< Not assignable.

    Base* impl; //!< A pointer to a Master or Slave object.
    bool  is_master;

public:
    Modbus(uint8_t u8id, Stream& port, uint8_t u8txenpin =0);

    void start()
        { impl->start(); }

#define CHECK_MASTER(rv) do{ if(!is_master) { return rv; } }while(0)
#define CHECK_SLAVE(rv)  do{ if( is_master) { return rv; } }while(0)

    void setTimeOut( uint16_t u16timeOut) //!<write communication watch-dog tim
        { CHECK_MASTER(); static_cast<Master*>(impl)->setTimeOut(u16timeOut); }
    bool getTimeOutState() //!<get communication watch-dog timer state
        { CHECK_MASTER(false); return static_cast<Master*>(impl)->timeOutExpired(); }
    int8_t query( modbus_t telegram ) //!<only for master
        { CHECK_MASTER(-2); return static_cast<Master*>(impl)->query(telegram); }
    int8_t poll() //!<cyclic poll for master
        { CHECK_MASTER(-2); return static_cast<Master*>(impl)->poll(); }
    int8_t poll( uint16_t *regs, uint8_t u8size ) //!<cyclic poll for slave
        { CHECK_SLAVE(-2); return static_cast<Slave*>(impl)->poll(regs,u8size); }
    uint16_t getInCnt() //!<number of incoming messages
        { return impl->getInCnt(); }
    uint16_t getOutCnt() //!<number of outcoming messages
        { return impl->getOutCnt(); }
    uint16_t getErrCnt() //!<error counter
        { return impl->getErrCnt(); }
    uint8_t getID() //!<get slave ID between 1 and 247
        { CHECK_SLAVE(-2); return static_cast<Slave*>(impl)->getID(); }
    uint8_t getState()
        { CHECK_MASTER(-2); return static_cast<Master*>(impl)->getState(); }
    int8_t getLastError() //!<get last error message
        { return impl->getLastError(); }
    void setID( uint8_t u8id ) //!<write new ID for the slave
        { CHECK_SLAVE(); static_cast<Slave*>(impl)->setID(u8id); }
    void setTxendPinOverTime( uint32_t u32overTime )
        { impl->setTxendPinOverTime(u32overTime); }

#undef CHECK_MASTER
#undef CHECK_SLAVE

    //
    // Deprecated functions

    // Deprecated: Use constructor: "Modbus m(0,Serial,0)" instead.
    Modbus(uint8_t u8id=0, uint8_t u8serno=0, uint8_t u8txenpin=0) __attribute__((deprecated));

    // Deprecated: Use "start()" instead.
    template<typename T_Stream>
    void begin(T_Stream* port_, long u32speed_) __attribute__((deprecated));

    // Deprecated: Use "start()" instead.
    template<typename T_Stream>
    void begin(T_Stream* port_, long u32speed_, uint8_t u8txenpin_) __attribute__((deprecated));

    // Deprecated: Use "start()" instead.
    void begin(long u32speed = 19200) __attribute__((deprecated));
};


/* _____PUBLIC FUNCTIONS_____BASE____________________________________________ */

/**
 * @brief
 * Start-up class object.
 *
 * Call this AFTER calling begin() on the serial port, typically within setup().
 *
 * (If you call this function, then you should NOT call any of
 * Modbus's own begin() functions.)
 *
 * @ingroup setup
 */
void Base::start()
{
    if (u8txenpin > 1)   // pin 0 & pin 1 are reserved for RX/TX
    {
        // return RS485 transceiver to receive mode
        pinMode(u8txenpin, OUTPUT);
        digitalWrite(u8txenpin, LOW);
    }

    while(port->read() >= 0);
    iLastBytesAvailable = 0;
    u16InCnt = u16OutCnt = u16errCnt = 0;
}


/**
 * @brief
 * Get input messages counter value
 * This can be useful to diagnose communication
 *
 * @return input messages counter
 * @ingroup buffer
 */
uint16_t Base::getInCnt() const
{
    return u16InCnt;
}


/**
 * @brief
 * Get transmitted messages counter value
 * This can be useful to diagnose communication
 *
 * @return transmitted messages counter
 * @ingroup buffer
 */
uint16_t Base::getOutCnt() const
{
    return u16OutCnt;
}


/**
 * @brief
 * Get errors counter value
 * This can be useful to diagnose communication
 *
 * @return errors counter
 * @ingroup buffer
 */
uint16_t Base::getErrCnt() const
{
    return u16errCnt;
}


/**
 * Get the last error in the protocol processor
 *
 * @return   ERR_WAITING = -1 Master is still waiting for previous query.
 * @return   ERR_FUNC_CODE = -2 Bad / unsupported function code.
 * @return   ERR_BAD_SLAVE_ID = -3 Telegram specifies invalid slave ID.
 * @return   ERR_BAD_CRC = -4 Received message with incorrect CRC.
 * @return   ERR_TIME_OUT_EXPIRED = -6 Master has given up waiting for answer.
 * @return   ERR_TX_BUFF_OVERFLOW = -7 Buffer too small for outgoing message.
 * @return   ERR_RX_BUFF_OVERFLOW = -8 Buffer too small for incoming message.
 * @return   ERR_MALFORMED_MESSAGE = -9 Incoming message too short.
 * @return   EXC_ILLEGAL_FUNCTION = 1 Function code not available.
 * @return   EXC_ILLEGAL_DATA_ADDRESS = 2  Address beyond available space for Modbus registers
 * @ingroup buffer
 */
int8_t Base::getLastError() const
{
    return i8lastError;
}


/**
 * Clear the last error in the protocol processor
 *
 * @ingroup buffer
 */
void Base::clearLastError()
{
    i8lastError = 0;
}


/**
 * @brief
 * Method to write the overtime count for txend pin.
 * It waits until count reaches 0 after the transfer is done.
 * With this, you can extend the time between txempty and
 * the falling edge if needed.
 *
 * @param 	uint32_t	overtime count for txend pin
 * @ingroup setup
 */
void Base::setTxendPinOverTime( uint32_t u32overTime )
{
    this->u32overTime = u32overTime;
}


/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup buffer
 */
uint16_t Base::calcCRC(const void* data, uint8_t len)
{
    const uint8_t* const bytes = static_cast<const uint8_t*>(data);
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= bytes[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            const bool flag = crc & 0x0001;
            crc >>= 1;
            if (flag)
                crc ^= 0xA001;
        }
    }
    return crc;
}


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
    au8Buffer[ ID ]         = telegram.u8id;
    au8Buffer[ FUNC ]       = telegram.u8fct;
    au8Buffer[ ADD_HI ]     = highByte(telegram.u16RegAdd );
    au8Buffer[ ADD_LO ]     = lowByte( telegram.u16RegAdd );

    switch( telegram.u8fct )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_READ_REGISTERS:
    case MB_FC_READ_INPUT_REGISTER:
        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_COIL:
        au8Buffer[ NB_HI ]      = ((au16regs[0] > 0) ? 0xff : 0);
        au8Buffer[ NB_LO ]      = 0;
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_REGISTER:
        au8Buffer[ NB_HI ]      = highByte(au16regs[0]);
        au8Buffer[ NB_LO ]      = lowByte(au16regs[0]);
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        au8Buffer[ BYTE_CNT ]   = (telegram.u16CoilsNo + 7)/8;
        u8BufferSize = 7;

        for (uint8_t i = 0; i < au8Buffer[ BYTE_CNT ]; i++)
        {
            if(i%2)
            {
                au8Buffer[ u8BufferSize ] = lowByte( au16regs[ i/2 ] );
            }
            else
            {
                au8Buffer[ u8BufferSize ] = highByte( au16regs[ i/2] );
            }          
            u8BufferSize++;
        }
        break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS:
        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        au8Buffer[ BYTE_CNT ]    = (uint8_t) ( telegram.u16CoilsNo * 2 );
        u8BufferSize = 7;

        for (uint16_t i=0; i< telegram.u16CoilsNo; i++)
        {
            au8Buffer[ u8BufferSize ] = highByte( au16regs[ i ] );
            u8BufferSize++;
            au8Buffer[ u8BufferSize ] = lowByte( au16regs[ i ] );
            u8BufferSize++;
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

    // validate message: id, CRC, FCT, exception
    const int8_t i8error = validateAnswer( au8Buffer, u8BufferSize );
    if (i8error != 0)
    {
        u8state = COM_IDLE;
        return setError(i8error);
    }

    // process answer
    switch( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
        // call get_FC1 to transfer the incoming message to au16regs buffer
        get_FC1( au8Buffer, u8BufferSize );
        break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
        // call get_FC3 to transfer the incoming message to au16regs buffer
        get_FC3( au8Buffer, u8BufferSize );
        break;
    case MB_FC_WRITE_COIL:
    case MB_FC_WRITE_REGISTER :
    case MB_FC_WRITE_MULTIPLE_COILS:
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        // nothing to do
        break;
    default:
        break;
    }
    u8state = COM_IDLE;
    // ?? Currently, there is no bounds checking in get_FCXX() functions, so there
    // ?? can be no error to report here.
    return u8BufferSize;
}


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
    u8id(u8id_)
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

    // validate message: CRC, and size
    int8_t i8error = validateRequest( au8Buffer, u8BufferSize );
    if (i8error==0)
    {
        // check slave id
        if (au8Buffer[ ID ] != u8id)
            return 0; // This message is for another slave.

        switch( au8Buffer[ FUNC ] )
        {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
            i8error = process_FC1( mapping, au8Buffer, u8BufferSize, MAX_BUFFER );
            break;
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_READ_REGISTERS :
            i8error = process_FC3( mapping, au8Buffer, u8BufferSize, MAX_BUFFER );
            break;
        case MB_FC_WRITE_COIL:
            i8error = process_FC5( mapping, au8Buffer, u8BufferSize );
            break;
        case MB_FC_WRITE_REGISTER :
            i8error = process_FC6( mapping, au8Buffer, u8BufferSize );
            break;
        case MB_FC_WRITE_MULTIPLE_COILS:
            i8error = process_FC15( mapping, au8Buffer, u8BufferSize );
            break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS :
            i8error = process_FC16( mapping, au8Buffer, u8BufferSize );
            break;
        default:
            i8error = EXC_ILLEGAL_FUNCTION;
            break;
        }
    }
    if (i8error > 0)
    {
        u8BufferSize = buildException( i8error, au8Buffer );
        sendTxBuffer( au8Buffer, u8BufferSize, MAX_BUFFER );
        // Ignore any error reported by sendTxBuffer().
    }
    else if (i8error==0)
    {
      i8error = sendTxBuffer( au8Buffer, u8BufferSize, MAX_BUFFER );
    }
    return setError(i8error);
}


/* _____PUBLIC FUNCTIONS_____MODBUS__________________________________________ */

/**
 * @brief
 * Constructor for a Master/Slave.
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
 * @param u8id   node address 0=master, 1..247=slave
 * @param port   serial port used
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 */
Modbus::Modbus(uint8_t u8id, Stream& port, uint8_t u8txenpin):
    is_master( u8id==0 )
{
  if (is_master)
      impl = new Master(port, u8txenpin);
  else
      impl = new Slave(u8id, port, u8txenpin);
}


/**
 * @brief
 * DEPRECATED constructor for a Master/Slave.
 *
 * THIS CONSTRUCTOR IS ONLY PROVIDED FOR BACKWARDS COMPATIBILITY.
 * USE Modbus(uint8_t, T_Stream&, uint8_t) INSTEAD.
 *
 * @param u8id   node address 0=master, 1..247=slave
 * @param u8serno  serial port used 0..3 (ignored for software serial)
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 * @overload Modbus::Modbus(uint8_t u8id, T_Stream& port, uint8_t u8txenpin)
 */
Modbus::Modbus(uint8_t u8id, uint8_t u8serno, uint8_t u8txenpin):
    is_master( u8id==0 )
{
    Stream* port;

    switch( u8serno )
    {
#if defined(UBRR1H)
    case 1:
        port = &Serial1;
        break;
#endif

#if defined(UBRR2H)
    case 2:
        port = &Serial2;
        break;
#endif

#if defined(UBRR3H)
    case 3:
        port = &Serial3;
        break;
#endif
    case 0:
    default:
        port = &Serial;
        break;
    }

  if (is_master)
      impl = new Master(*port, u8txenpin);
  else
      impl = new Slave(u8id, *port, u8txenpin);
}


/**
 * @brief
 * DEPRECATED Install a serial port, begin() it, and start Modbus.
 *
 * ONLY PROVIDED FOR BACKWARDS COMPATIBILITY.
 * USE Serial.begin(<baud rate>); FOLLOWED BY Modbus.start() INSTEAD.
 *
 * @param install_port pointer to SoftwareSerial or HardwareSerial class object
 * @param u32speed     baud rate, in standard increments (300..115200)
 * @ingroup setup
 */
template<typename T_Stream>
void Modbus::begin(T_Stream* install_port, long u32speed)
{
    impl->port = install_port;
    install_port->begin(u32speed);
    start();
}


/**
 * @brief
 * DEPRECATED. Install a serial port, begin() it, and start Modbus.
 *
 * ONLY PROVIDED FOR BACKWARDS COMPATIBILITY.
 * USE Serial.begin(<baud rate>); FOLLOWED BY Modbus.start() INSTEAD.
 *
 * @param install_port  pointer to SoftwareSerial or HardwareSerial class object
 * @param u32speed      baud rate, in standard increments (300..115200)
 * @param u8txenpin     pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 */
template<typename T_Stream>
void Modbus::begin(T_Stream* install_port, long u32speed, uint8_t u8txenpin)
{
    impl->u8txenpin = u8txenpin;
    impl->port = install_port;
    install_port->begin(u32speed);
    start();
}


/**
 * @brief
 * DEPRECATED. begin() hardware serial port and start Modbus.
 *
 * ONLY PROVIDED FOR BACKWARDS COMPATIBILITY.
 * USE Serial.begin(<baud rate>); FOLLOWED BY Modbus.start() INSTEAD.
 *
 * @see http://arduino.cc/en/Serial/Begin#.Uy4CJ6aKlHY
 * @param speed   baud rate, in standard increments (300..115200). Default=19200
 * @ingroup setup
 */
void Modbus::begin(long u32speed)
{
    // !!Can ONLY do this if port ACTUALLY IS a HardwareSerial object!!
    static_cast<HardwareSerial*>(impl->port)->begin(u32speed);
    start();
}


/* ____PROTECTED FUNCTIONS___BASE____________________________________________ */

/**
 * @brief  Base class constructor.
 * @ingroup setup
 */
Base::Base(Stream& port_, uint8_t u8txenpin_):
    port(&port_),
    u8txenpin(u8txenpin_),
    i8lastError(0),
    iLastBytesAvailable(0),
    u16InCnt(0),
    u16OutCnt(0),
    u16errCnt(0),
    ulT35timer(0),
    u32overTime(0)
{}


/**
 * @brief
 * Increment the error count, and record the last error.
 *
 * @param i8error  The error being reported. >0 - MODBUS exception. <0 Internal error.
 * @return         i8error 0: 0, >0: ERR_EXCEPTION, <0: i8error
 * @ingroup error
 */
int8_t Base::setError( int8_t i8error )
{
  if (i8error)
  {
      ++u16errCnt;
      i8lastError = i8error;
      // Error return values are always <0.
      // If the caller needs to see the specific exception, then they must
      // call getLastError().
      if (i8error > 0)
      {
          return ERR_EXCEPTION;
      }
  }
  return i8error;
}


/**
 * @brief Returns TRUE if there is a frame ready to process in the Rx buffer.
 *
 * Use port->available() to get the amount of data waiting in the receive
 * buffer. When this value is non-zero, and has not changed for two consecutive
 * calls, we assume that the complete frame has arrived.
 *
 * We then wait for T35 milliseconds before processing the message, to ensure
 * that the bus is silent for >3.5 character widths between frames, as required
 * by the MODBUS standard.
 *
 * (This implementation has limited control over the Serial port, so we cannot
 * apply this delay in the transport layer.)
 *
 * @ingroup buffer
 */
bool Base::rxFrameReady()
{
    // Check if there is any incoming frame.
    const int bytesAvailable = port->available();
    if (bytesAvailable > 0)
    {
        const unsigned long now = millis();
        if (bytesAvailable == iLastBytesAvailable)
        {
            // The serial port has now buffered the entire frame.
            if (T35==0 || (now - ulT35timer) >= (unsigned long)T35)
            {
                // The T35 timer has expired.
                // Reset, ready for the next frame.
                iLastBytesAvailable = 0;

                // OK to process the frame.
                return true;
            }
        }
        else
        {
            // The frame is still arriving.
            // Record the current size, and set the T35 timer.
            iLastBytesAvailable = bytesAvailable;
            ulT35timer = now;
        }
    }
    return false;
}


/**
 * @brief
 * This method moves Serial buffer data to out local buffer.
 *
 * @param buf     buffer into which the message should be read.
 * @param bufsize capacity of buffer, in bytes.
 * @return buffer message length if OK,
 *                  ERR_RX_BUFF_OVERFLOW if message is larger than size
 * @ingroup buffer
 */
int8_t Base::getRxBuffer( uint8_t* buf, uint8_t bufsize )
{
    // Pre-condition: We know that (port->available() > 0), because
    // rxFrameReady() has returned TRUE.
    u16InCnt++;
    uint8_t i = 0;
    while(true)
    {
      const int c = port->read();
      if (i < bufsize)
      {
        if (c < 0)
            break;
        buf[ i++ ] = c;
      }
      else if (c < 0)
      {
        return ERR_RX_BUFF_OVERFLOW;
      }
    }
    return i;
}


/**
 * @brief
 * This method transmits au8Buffer to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with UCSRxA register.
 * The CRC is appended to the buffer before starting to send it. The new
 * length of the buffer is returned, or ERR_TX_BUFF_OVERFLOW if the buffer is
 * too short.
 *
 * @param  buf     data buffer.
 * @param  count   message length.
 * @param  bufsize capacity of buffer in bytes.
 * @return         O: OK, ERR_TX_BUFF_OVERFLOW: buffer overflow
 * @ingroup buffer
 */
int8_t Base::sendTxBuffer( uint8_t* buf, uint8_t count, uint8_t bufsize )
{
    // append CRC to message
    if (count+2 > bufsize)
    {
        return ERR_TX_BUFF_OVERFLOW;
    }
    const uint16_t u16crc = calcCRC( buf, count );
    buf[ count   ] = lowByte(  u16crc );
    buf[ count+1 ] = highByte( u16crc );

    if (u8txenpin > 1)
    {
        // set RS485 transceiver to transmit mode
        digitalWrite( u8txenpin, HIGH );
    }

    // transfer buffer to serial line
    port->write( buf, count+2 );

    if (u8txenpin > 1)
    {
        // must wait transmission end before changing pin state
        // soft serial does not need it since it is blocking
        // ...but the implementation in SoftwareSerial does nothing
        // anyway, so no harm in calling it.
        port->flush();
        // return RS485 transceiver to receive mode
        volatile uint32_t u32overTimeCountDown = u32overTime;
        while ( u32overTimeCountDown-- > 0);
        digitalWrite( u8txenpin, LOW );
    }
    while(port->read() >= 0);

    // increase message counter
    u16OutCnt++;

    return 0;
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

    // check message crc vs calculated crc
    const uint16_t u16MsgCRC = word( buf[count - 1], buf[count - 2] );
    if ( calcCRC( buf, count-2 ) != u16MsgCRC )
    {
        return ERR_BAD_CRC;
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

    // check fct code
    boolean isSupported = false;
    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
    {
        if (fctsupported[i] == buf[FUNC])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
        return EXC_ILLEGAL_FUNCTION;
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
     uint8_t u8byte = 3;
     for (uint8_t i=0; i< buf[2]; i++) {
        
        if(i%2)
        {
            au16regs[i/2]= word(buf[i+u8byte], lowByte(au16regs[i/2]));
        }
        else
        {
            au16regs[i/2]= word(0, buf[i+u8byte]); 
        }
        
     }
}


/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void Master::get_FC3( const uint8_t* buf, uint8_t /*count*/ )
{
    uint8_t u8byte, i;
    u8byte = 3;

    for (i=0; i< buf[ 2 ] /2; i++)
    {
        au16regs[ i ] = word(
                            buf[ u8byte ],
                            buf[ u8byte +1 ]);
        u8byte += 2;
    }
}


/* ____PRIVATE FUNCTIONS_____SLAVE___________________________________________ */

/**
 * @brief
 * This method validates slave incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
int8_t Slave::validateRequest( const uint8_t* buf, uint8_t count ) const
{
    if (count <= 3)
        return ERR_MALFORMED_MESSAGE;

    // check message crc vs calculated crc
    const uint16_t u16MsgCRC = word( buf[count - 1], buf[count - 2] );
    if ( calcCRC( buf, count-2 ) != u16MsgCRC )
    {
        return ERR_BAD_CRC;
    }

    return 0; // OK, no exception code thrown
}


/**
 * @brief
 * This method builds an exception message. The buffer is assumed to be large enough
 * for the exception - which is only 3 bytes long.
 *
 * @ingroup buffer
 */
int8_t Slave::buildException( uint8_t u8exception, uint8_t* buf ) const
{
    buf[ FUNC ] += 0x80;
    buf[ 2 ]     = u8exception;
    return EXCEPTION_SIZE;
}


/**
 * @brief
 * This method processes functions 1 & 2
 * This method reads a bit array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC1( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize )
{
    // Validate the request message size.
    if (count != 8)
        return ERR_MALFORMED_MESSAGE;

    const uint16_t addr     = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    const uint16_t quantity = word( buf[ NB_HI  ], buf[ NB_LO  ] );

    // Set the message size.
    buf[ 2 ] = (quantity+7)/8;
    count = buf[ 2 ] + 3;

    // Validate the requested quantity.
    if(0 == quantity || quantity > 2000 || count > bufsize-2)
        return EXC_ILLEGAL_DATA_VALUE;

    // Read the requested values into buf[3] onwards.
    if (buf[ FUNC ] == MB_FC_READ_COILS)
        return mapping.read_coils(buf + 3, addr, quantity);
    else // MB_FC_READ_DISCRETE_INPUT
        return mapping.read_discrete_inputs(buf + 3, addr, quantity);
}


/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC3( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize )
{
    // Validate the request message size.
    if (count != 8)
        return ERR_MALFORMED_MESSAGE;

    const uint16_t addr     = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    const uint16_t quantity = word( buf[ NB_HI  ], buf[ NB_LO  ] );

    // Set the message size.
    buf[ 2 ] = quantity * 2;
    count = buf[ 2 ] + 3;

    // Validate the requested quantity.
    if(0 == quantity || quantity > 125 || count > bufsize-2)
        return EXC_ILLEGAL_DATA_VALUE;

    // Read the requested values into buf[3] onwards.
    if (buf[ FUNC ] == MB_FC_READ_INPUT_REGISTER)
        return mapping.read_input_registers(buf + 3, addr, quantity);
    else
        return mapping.read_holding_registers(buf + 3, addr, quantity);
}


/**
 * @brief
 * This method processes function 5
 * This method writes a value assigned by the master to a single bit
 *
 * @return count Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC5( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count != 8)
        return ERR_MALFORMED_MESSAGE;

    const uint16_t addr  = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    const uint16_t value = word( buf[ NB_HI  ], buf[ NB_LO  ] );

    // Validate the value.
    if(value != 0x0000  &&  value != 0xFF00)
        return EXC_ILLEGAL_DATA_VALUE;

    // Response is just the first 6 bytes of the request.
    count = 6;

    // Set addr = value.
    return mapping.write_coils(addr, buf + NB_HI);
}


/**
 * @brief
 * This method processes function 6
 * This method writes a value assigned by the master to a single word
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC6( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count != 8)
        return ERR_MALFORMED_MESSAGE;

    const uint16_t addr = word( buf[ ADD_HI ], buf[ ADD_LO ] );

    // Response is just the first 6 bytes of the request.
    count = 6;

    // Set addr = value.
    return mapping.write_holding_registers(addr, buf + NB_HI);
}


/**
 * @brief
 * This method processes function 15
 * This method writes a bit array assigned by the master
 *
 * @return count Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC15( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count < 10)
        return ERR_MALFORMED_MESSAGE;

    const uint16_t addr     = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    const uint16_t quantity = word( buf[ NB_HI  ], buf[ NB_LO  ] );
    const uint8_t  n        = buf[ 6 ];

    // Validate the quantity.
    if(0 == quantity || quantity > 0x7B0 || n != (quantity+7)/8)
        return EXC_ILLEGAL_DATA_VALUE;

    if (count != n + 9)
        return ERR_MALFORMED_MESSAGE;

    // Response is just the first 6 bytes of the request.
    count = 6;

    // Write the requested values from buf[7] onwards.
    return mapping.write_coils(addr, buf + 7, quantity);
}


/**
 * @brief
 * This method processes function 16
 * This method writes a word array assigned by the master
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC16( Mapping& mapping, uint8_t* buf, uint8_t& count )
{
    // Validate the request message size.
    if (count < 11)
        return ERR_MALFORMED_MESSAGE;

    const uint16_t addr     = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    const uint16_t quantity = word( buf[ NB_HI  ], buf[ NB_LO  ] );
    const uint8_t  n        = buf[ 6 ];

    // Validate the quantity.
    if(0 == quantity || quantity > 0x7B || n != quantity*2)
        return EXC_ILLEGAL_DATA_VALUE;

    if (count != n + 9)
        return ERR_MALFORMED_MESSAGE;

    // Response is just the first 6 bytes of the request.
    count = 6;

    // Write the requested values from buf[7] onwards.
    return mapping.write_holding_registers(addr, buf + 7, quantity);
}


} // end namespace modbus

#if !defined(USING_MODBUS_NAMESPACE) || USING_MODBUS_NAMESPACE!=0
using namespace modbus;
#endif

#endif // MODBUS_MODBUS_RTU_H
