/**
 * @file 	ModbusRtu.h
 * @version     1.21
 * @date        2019.06.13
 * @author 	Samuel Marco i Armengol
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

#include <inttypes.h>
#include "Arduino.h"

namespace modbus {


/**
 * @struct modbus_t
 * @brief
 * Master query structure:
 * This includes all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct
{
    uint8_t u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
    uint8_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t u16RegAdd;    /*!< Address of the first register to access at slave/s */
    uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
    uint16_t *au16reg;     /*!< Pointer to memory image in master */
}
modbus_t;

enum
{
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};

/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
enum MESSAGE
{
    ID                             = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT  //!< byte counter
};

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
enum MB_FC
{
    MB_FC_NONE                     = 0,   /*!< null operator */
    MB_FC_READ_COILS               = 1,	/*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT      = 2,	/*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS           = 3,	/*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      = 4,	/*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL               = 5,	/*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER           = 6,	/*!< FCT=6 -> write single register */
    MB_FC_WRITE_MULTIPLE_COILS     = 15,	/*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16	/*!< FCT=16 -> write multiple registers */
};

enum COM_STATES
{
    COM_IDLE                     = 0,
    COM_WAITING                  = 1

};

enum ERR_LIST
{
    ERR_WAITING           = -1, //!< Master::query(). Master is still waiting for previous query.
    ERR_FUNC_CODE         = -2, //!< Master::query(). Bad / unsupported function code.
    ERR_BAD_SLAVE_ID      = -3, //!< Master::query(). Telegram specifies invalid slave ID.
    ERR_BAD_CRC           = -4, //!< poll() methods.
    ERR_EXCEPTION         = -5, //!< Master::poll(): Exception received from slave. Slave::poll(): Exception returned to master.
    ERR_TIME_OUT_EXPIRED  = -6, //!< Master::poll(). Master has given up waiting for answer.
    ERR_TX_BUFF_OVERFLOW  = -7, //!< Any Tx method.
    ERR_RX_BUFF_OVERFLOW  = -8, //!< poll() methods.
    ERR_MALFORMED_MESSAGE = -9  //!< poll() methods.
};

enum EXC_LIST
{
    // Standard names, from MODBUS protocol.
    EXC_ILLEGAL_FUNCTION      = 1,
    EXC_ILLEGAL_DATA_ADDRESS  = 2,
    EXC_ILLEGAL_DATA_VALUE    = 3,
    EXC_SERVER_DEVICE_FAILURE = 4,
    EXC_ACKNOWLEDGE           = 5,
    EXC_SERVER_DEVICE_BUSY    = 6,

    EXC_MEMORY_PARITY_ERROR   = 8,
    EXC_GATEWAY_PATH_UNAVAILABLE = 10,
    EXC_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 11,

    // Old, backward compatibility names.
    EXC_FUNC_CODE  = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE    = 4
};

const unsigned char fctsupported[] =
{
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};

#if !defined(T35)
#define T35  5
#endif

#if !defined(MAX_BUFFER)
#define  MAX_BUFFER  64	//!< maximum size for the communication buffer in bytes
#endif


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
    uint8_t u8lastRec;
    uint16_t u16InCnt;
    uint16_t u16OutCnt;
    uint16_t u16errCnt;
    uint32_t u32time;
    uint32_t u32overTime;

protected:
    Base(Stream& port, uint8_t u8txenpin =0);

    int8_t setError( int8_t i8error );
    int8_t getRxBuffer( uint8_t* buf, uint8_t count );
    int8_t sendTxBuffer( uint8_t* buf, uint8_t count, uint8_t size );
    uint16_t calcCRC( const uint8_t* data, uint8_t len ) const;

public:
    void start();
    uint16_t getInCnt(); //!<number of incoming messages
    uint16_t getOutCnt(); //!<number of outcoming messages
    uint16_t getErrCnt(); //!<error counter
    int8_t   getLastError(); //!< Get last error (ERR_XXX) or exception (EXC_XXX) code.
    void     clearLastError(); //!< Set last error to 0.
    void setTxendPinOverTime( uint32_t u32overTime );

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
    int8_t validateAnswer( const uint8_t* buf, uint8_t count );
    void get_FC1( const uint8_t* buf, uint8_t count );
    void get_FC3( const uint8_t* buf, uint8_t count );

public:
    Master(Stream& port, uint8_t u8txenpin =0);

    void setTimeOut( uint16_t u16timeOut); //!<write communication watch-dog timer
    bool timeOutExpired(); //!<get communication watch-dog timer state
    uint8_t getState();

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
    int8_t validateRequest( uint8_t regsize, const uint8_t* buf, uint8_t count );
    int8_t validateCoilAddress( uint8_t regsize, uint16_t startaddr, uint16_t quantity ) const;
    int8_t validateRegAddress( uint8_t regsize, uint16_t startaddr, uint16_t quantity ) const;
    int8_t buildException( uint8_t u8exception, uint8_t* buf, uint8_t size ) const;
    int8_t process_FC1( uint16_t *regs, uint8_t u8size, uint8_t* buf, uint8_t bufsize );
    int8_t process_FC3( uint16_t *regs, uint8_t u8size, uint8_t* buf, uint8_t bufsize );
    int8_t process_FC5( uint16_t *regs, uint8_t u8size, uint8_t* buf, uint8_t bufsize );
    int8_t process_FC6( uint16_t *regs, uint8_t u8size, uint8_t* buf, uint8_t bufsize );
    int8_t process_FC15( uint16_t *regs, uint8_t u8size, uint8_t* buf, uint8_t bufsize );
    int8_t process_FC16( uint16_t *regs, uint8_t u8size, uint8_t* buf, uint8_t bufsize );

public:
    Slave(uint8_t u8id, Stream& port, uint8_t u8txenpin =0);

    void setID( uint8_t u8id ); //!<write new ID for the slave
    uint8_t getID(); //!<get slave ID between 1 and 247

    int8_t poll( uint16_t *regs, uint8_t u8size ); //!<cyclic poll for slave
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
    u8lastRec = 0;
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
uint16_t Base::getInCnt()
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
uint16_t Base::getOutCnt()
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
uint16_t Base::getErrCnt()
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
int8_t Base::getLastError()
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
bool Master::timeOutExpired()
{
    return ((unsigned long)(millis() -u32timeOut) > (unsigned long)u16timeOut);
}


/**
 * Get modbus master state
 *
 * @return = 0 IDLE, = 1 WAITING FOR ANSWER
 * @ingroup buffer
 */
uint8_t Master::getState()
{
    return u8state;
}


/**
 * @brief
 * *** Only Modbus Master ***
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
    uint8_t u8regsno, u8bytesno;
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
        u8regsno = telegram.u16CoilsNo / 16;
        u8bytesno = u8regsno * 2;
        if ((telegram.u16CoilsNo % 16) != 0)
        {
            u8bytesno++;
            u8regsno++;
        }

        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        au8Buffer[ BYTE_CNT ]    = u8bytesno;
        u8BufferSize = 7;

        for (uint16_t i = 0; i < u8bytesno; i++)
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

    int8_t i8bytes_sent = sendTxBuffer( au8Buffer, u8BufferSize, MAX_BUFFER );
    if (i8bytes_sent < 0)
    {
        return setError(i8bytes_sent);
    }
    u32timeOut = millis();
    u8state = COM_WAITING;
    return 0;
}


/**
 * @brief *** Only for Modbus Master ***
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

    // check if there is any incoming frame
    uint8_t u8current = port->available();

    if (timeOutExpired())
    {
        u8state = COM_IDLE;
        return setError(ERR_TIME_OUT_EXPIRED);
    }

    if (u8current == 0)
        return 0; // Still waiting for reply.

    // check T35 after frame end or still no frame end
    if (u8current != u8lastRec)
    {
        u8lastRec = u8current;
        u32time = millis();
        return 0; // Waiting for T35.
    }
    if ((unsigned long)(millis() -u32time) < (unsigned long)T35) return 0;

    // transfer Serial buffer frame to a local buffer.
    u8lastRec = 0;
    uint8_t au8Buffer[MAX_BUFFER];
    int8_t i8bytes_read = getRxBuffer( au8Buffer, MAX_BUFFER );
    if (i8bytes_read < 0)
    {
        u8state = COM_IDLE;
        return setError(i8bytes_read); // Pass error on from getRxBuffer().
    }
    uint8_t u8BufferSize = i8bytes_read;

    // validate message: id, CRC, FCT, exception
    int8_t i8error = validateAnswer( au8Buffer, u8BufferSize );
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
uint8_t Slave::getID()
{
    return this->u8id;
}


/**
 * @brief
 * *** Only for Modbus Slave ***
 * This method checks if there is any incoming query
 * Afterwards, it would shoot a validation routine plus a register query
 * Avoid any delay() function !!!!
 * After a successful frame between the Master and the Slave, the time-out timer is reset.
 *
 * @param *regs  register table for communication exchange
 * @param u8size  size of the register table
 * @return        error: <0, nothing to do: 0, request processed: >0
 * @ingroup loop
 */
int8_t Slave::poll( uint16_t *regs, uint8_t u8size )
{
    // check if there is any incoming frame
    uint8_t u8current = port->available();

    if (u8current == 0)
        return 0; // Still waiting for request.

    // check T35 after frame end or still no frame end
    if (u8current != u8lastRec)
    {
        u8lastRec = u8current;
        u32time = millis();
        return 0; // Waiting for T35.
    }
    if ((unsigned long)(millis() -u32time) < (unsigned long)T35) return 0;

    u8lastRec = 0;
    uint8_t au8Buffer[MAX_BUFFER];
    int8_t i8bytes_read = getRxBuffer( au8Buffer, MAX_BUFFER );
    if (i8bytes_read < 0)
    {
        return setError(i8bytes_read); // Pass error on from getRxBuffer().
    }
    uint8_t u8BufferSize = i8bytes_read;

    // check slave id
    if (ID < u8BufferSize  &&  au8Buffer[ ID ] != u8id)
        return 0; // This message is for another slave.

    // validate message: CRC, FCT, address and size
    int8_t i8error = validateRequest( u8size, au8Buffer, u8BufferSize );
    if (i8error != 0)
    {
        if (i8error > 0)
        {
            u8BufferSize = buildException( i8error, au8Buffer, MAX_BUFFER );
            sendTxBuffer( au8Buffer, u8BufferSize, MAX_BUFFER );
            // Ignore any error reported by sendTxBuffer().
        }
        return setError(i8error);
    }

    // process message
    int8_t i8rv = ERR_FUNC_CODE;
    switch( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
        i8rv = process_FC1( regs, u8size, au8Buffer, MAX_BUFFER );
        break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
        i8rv = process_FC3( regs, u8size, au8Buffer, MAX_BUFFER );
        break;
    case MB_FC_WRITE_COIL:
        i8rv = process_FC5( regs, u8size, au8Buffer, MAX_BUFFER );
        break;
    case MB_FC_WRITE_REGISTER :
        i8rv = process_FC6( regs, u8size, au8Buffer, MAX_BUFFER );
        break;
    case MB_FC_WRITE_MULTIPLE_COILS:
        i8rv = process_FC15( regs, u8size, au8Buffer, MAX_BUFFER );
        break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        i8rv = process_FC16( regs, u8size, au8Buffer, MAX_BUFFER );
        break;
    default:
        break;
    }
    return setError(i8rv);
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
    u8lastRec(0),
    u16InCnt(0),
    u16OutCnt(0),
    u16errCnt(0),
    u32time(0),
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
 * @brief
 * This method moves Serial buffer data to out local buffer.
 *
 * @param buf     buffer into which the message should be read.
 * @param count   length of data buffer, in bytes.
 * @return buffer message length if OK,
 *                  ERR_RX_BUFF_OVERFLOW if message is larger than size
 * @ingroup buffer
 */
int8_t Base::getRxBuffer( uint8_t* buf, uint8_t count )
{
    if (u8txenpin > 1)
        digitalWrite( u8txenpin, LOW );

    u16InCnt++;
    uint8_t i = 0;
    while ( port->available() )
    {
        if (i >= count)
        {
            // Discard the remaining incoming data.
            while(port->read() >= 0);
            return ERR_RX_BUFF_OVERFLOW;
        }
        buf[ i ] = port->read();
        i ++;
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
 * @param  size    capacity of buffer in bytes.
 * @return         message length, or ERR_TX_BUFF_OVERFLOW
 * @ingroup buffer
 */
int8_t Base::sendTxBuffer( uint8_t* buf, uint8_t count, uint8_t size )
{
    // append CRC to message
    if (count+2 > size)
    {
        return ERR_TX_BUFF_OVERFLOW;
    }
    uint16_t u16crc = calcCRC( buf, count );
    buf[ count ] = u16crc >> 8;
    count++;
    buf[ count ] = u16crc & 0x00ff;
    count++;

    if (u8txenpin > 1)
    {
        // set RS485 transceiver to transmit mode
        digitalWrite( u8txenpin, HIGH );
    }

    // transfer buffer to serial line
    port->write( buf, count );

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

    return count;
}


/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup buffer
 */
uint16_t Base::calcCRC(const uint8_t* data, uint8_t len) const
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < len; i++)
    {
        temp = temp ^ data[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}


/* ____PRIVATE FUNCTIONS_____MASTER__________________________________________ */

/**
 * @brief
 * This method validates master incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
int8_t Master::validateAnswer( const uint8_t* buf, uint8_t count )
{
    if (count <= 2)
    {
        return ERR_MALFORMED_MESSAGE;
    }

    // check message crc vs calculated crc
    uint16_t u16MsgCRC =
        ((buf[count - 2] << 8)
         | buf[count - 1]); // combine the crc Low & High bytes
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
    if (count < 6) //7 was incorrect for functions 1 and 2 the smallest frame could be 6 bytes long
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
void Master::get_FC1( const uint8_t* buf, uint8_t count )
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
void Master::get_FC3( const uint8_t* buf, uint8_t count )
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
int8_t Slave::validateRequest(
    uint8_t        regsize,
    const uint8_t* buf,
    uint8_t        count)
{
    if (count <= 2)
    {
        return ERR_MALFORMED_MESSAGE;
    }

    // check message crc vs calculated crc
    uint16_t u16MsgCRC =
        ((buf[count - 2] << 8)
         | buf[count - 1]); // combine the crc Low & High bytes
    if ( calcCRC( buf, count-2 ) != u16MsgCRC )
    {
        return ERR_BAD_CRC;
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

    if (count < 7)
    {
        return ERR_MALFORMED_MESSAGE;
    }

    // check start address & nb range
    switch ( buf[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_WRITE_MULTIPLE_COILS:
      {
        uint16_t u16StartCoil = word( buf[ ADD_HI ], buf[ ADD_LO ] );
        uint16_t u16Coilno    = word( buf[ NB_HI ],  buf[ NB_LO ] );
        return validateCoilAddress(regsize, u16StartCoil, u16Coilno);
      }
    case MB_FC_WRITE_COIL:
      {
        uint16_t u16StartCoil = word( buf[ ADD_HI ], buf[ ADD_LO ] );
        return validateCoilAddress(regsize, u16StartCoil, 1);
      }
    case MB_FC_WRITE_REGISTER :
      {
        uint16_t u16StartAdd = word( buf[ ADD_HI ], buf[ ADD_LO ] );
        return validateRegAddress(regsize, u16StartAdd, 1);
      }
    case MB_FC_READ_REGISTERS :
    case MB_FC_READ_INPUT_REGISTER :
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
      {
        uint16_t u16StartAdd = word( buf[ ADD_HI ], buf[ ADD_LO ] );
        uint16_t u16regsno   = word( buf[ NB_HI ],  buf[ NB_LO ] );
        return validateRegAddress(regsize, u16StartAdd, u16regsno);
      }
    }
    return 0; // OK, no exception code thrown
}


/**
 * @brief
 * Determine the validity of a range of coil (or discrete input) addresses.
 *
 * @param  regsize   number of 16-bit words in the data array.
 *                   Valid addresses are in range [0,regsize*16).
 * @param  startaddr first coil address in range.
 * @param  quantity  number of coil address after startaddr.
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
inline int8_t Slave::validateCoilAddress(
    uint8_t  regsize,
    uint16_t startaddr,
    uint16_t quantity ) const
{
  uint16_t firstword = startaddr/16;
  uint16_t lastword  = (startaddr+quantity-1)/16;
  return validateRegAddress(regsize, firstword, 1+lastword-firstword);
}


/**
 * @brief
 * Determine the validity of a range of register addresses.
 *
 * @param  regsize   number of 16-bit words in the data array.
 *                   Valid addresses are in range [0,regsize).
 * @param  startaddr first register address in range.
 * @param  quantity  number of register address after startaddr.
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
inline int8_t Slave::validateRegAddress(
    uint8_t  regsize,
    uint16_t startaddr,
    uint16_t quantity ) const
{
  uint16_t endaddr = startaddr + quantity;
  if(endaddr > regsize)
      return EXC_ILLEGAL_DATA_ADDRESS;
  else
      return 0;
}


/**
 * @brief
 * This method builds an exception message
 *
 * @ingroup buffer
 */
int8_t Slave::buildException( uint8_t u8exception, uint8_t* buf, uint8_t size ) const
{
    uint8_t u8func = buf[ FUNC ];  // get the original FUNC code

    buf[ ID ]      = u8id;
    buf[ FUNC ]    = u8func + 0x80;
    buf[ 2 ]       = u8exception;
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
int8_t Slave::process_FC1( uint16_t *regs, uint8_t /*u8size*/, uint8_t* buf, uint8_t bufsize )
{
    uint8_t u8currentRegister, u8currentBit, u8bytesno, u8bitsno;
    uint16_t u16currentCoil, u16coil;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    uint16_t u16Coilno = word( buf[ NB_HI ], buf[ NB_LO ] );

    // put the number of bytes in the outcoming message
    u8bytesno = (uint8_t) (u16Coilno / 8);
    if (u16Coilno % 8 != 0) u8bytesno ++;
    buf[ ADD_HI ] = u8bytesno;
    uint8_t count = ADD_LO;
    buf[ count + u8bytesno - 1 ] = 0;

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;

    // Clear all data bits in outgoing message.
    memset( buf+count, 0, MAX_BUFFER-count );

    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) (u16coil % 16);

        bitWrite(
            buf[ count ],
            u8bitsno,
            bitRead( regs[ u8currentRegister ], u8currentBit ) );
        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            count++;
        }
    }

    // send outcoming message
    if (u16Coilno % 8 != 0)
        count ++;

    return sendTxBuffer( buf, count, bufsize );
}


/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC3( uint16_t *regs, uint8_t /*u8size*/, uint8_t* buf, uint8_t bufsize )
{

    uint8_t u8StartAdd = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    uint8_t u8regsno = word( buf[ NB_HI ], buf[ NB_LO ] );
    uint8_t i;

    buf[ 2 ] = u8regsno * 2;
    uint8_t count = 3;

    for (i = u8StartAdd; i < u8StartAdd + u8regsno; i++)
    {
        buf[ count ] = highByte(regs[i]);
        count++;
        buf[ count ] = lowByte(regs[i]);
        count++;
    }
    return sendTxBuffer( buf, count, bufsize );
}


/**
 * @brief
 * This method processes function 5
 * This method writes a value assigned by the master to a single bit
 *
 * @return count Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC5( uint16_t *regs, uint8_t /*u8size*/, uint8_t* buf, uint8_t bufsize )
{
    uint8_t u8currentRegister, u8currentBit;
    uint16_t u16coil = word( buf[ ADD_HI ], buf[ ADD_LO ] );

    // point to the register and its bit
    u8currentRegister = (uint8_t) (u16coil / 16);
    u8currentBit = (uint8_t) (u16coil % 16);

    // write to coil
    bitWrite(
        regs[ u8currentRegister ],
        u8currentBit,
        buf[ NB_HI ] == 0xff );


    // send answer to master
    return sendTxBuffer( buf, 6, bufsize );
}


/**
 * @brief
 * This method processes function 6
 * This method writes a value assigned by the master to a single word
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC6( uint16_t *regs, uint8_t /*u8size*/, uint8_t* buf, uint8_t bufsize )
{

    uint8_t u8add = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    uint16_t u16val = word( buf[ NB_HI ], buf[ NB_LO ] );

    regs[ u8add ] = u16val;

    // keep the same header
    return sendTxBuffer( buf, RESPONSE_SIZE, bufsize );
}


/**
 * @brief
 * This method processes function 15
 * This method writes a bit array assigned by the master
 *
 * @return count Response to master length
 * @ingroup discrete
 */
int8_t Slave::process_FC15( uint16_t *regs, uint8_t /*u8size*/, uint8_t* buf, uint8_t bufsize )
{
    uint8_t u8currentRegister, u8currentBit, u8frameByte, u8bitsno;
    uint16_t u16currentCoil, u16coil;
    boolean bTemp;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word( buf[ ADD_HI ], buf[ ADD_LO ] );
    uint16_t u16Coilno = word( buf[ NB_HI ], buf[ NB_LO ] );


    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;
    u8frameByte = 7;
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {

        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) (u16coil % 16);

        bTemp = bitRead(
                    buf[ u8frameByte ],
                    u8bitsno );

        bitWrite(
            regs[ u8currentRegister ],
            u8currentBit,
            bTemp );

        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8frameByte++;
        }
    }

    // send outcoming message
    // it's just a copy of the incomping frame until 6th byte
    return sendTxBuffer( buf, 6, bufsize );
}


/**
 * @brief
 * This method processes function 16
 * This method writes a word array assigned by the master
 *
 * @return count Response to master length
 * @ingroup register
 */
int8_t Slave::process_FC16( uint16_t *regs, uint8_t /*u8size*/, uint8_t* buf, uint8_t bufsize )
{
    uint8_t u8StartAdd = buf[ ADD_HI ] << 8 | buf[ ADD_LO ];
    uint8_t u8regsno = buf[ NB_HI ] << 8 | buf[ NB_LO ];
    uint8_t i;
    uint16_t temp;

    // build header
    buf[ NB_HI ]   = 0;
    buf[ NB_LO ]   = u8regsno;

    // write registers
    for (i = 0; i < u8regsno; i++)
    {
        temp = word(
                   buf[ (BYTE_CNT + 1) + i * 2 ],
                   buf[ (BYTE_CNT + 2) + i * 2 ]);

        regs[ u8StartAdd + i ] = temp;
    }
    return sendTxBuffer( buf, RESPONSE_SIZE, bufsize );
}


} // end namespace modbus

#if !defined(USING_MODBUS_NAMESPACE) || USING_MODBUS_NAMESPACE!=0
using namespace modbus;
#endif
