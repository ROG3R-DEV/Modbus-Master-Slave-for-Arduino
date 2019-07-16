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


void Master::cancel_request()
{
    u8state = COM_IDLE;
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
int8_t Master::send_request(Message& msg)
{
    if(u8state != COM_IDLE)
        return setError(ERR_WAITING);

    if(msg.get_slave() > 247)
        return setError(ERR_BAD_SLAVE_ID);

    sendTxBuffer(msg.buf, msg.length);
    ++u16Counter[CNT_MASTER_QUERY];
    u32timeOut = millis();

    // Do not wait for broadcast requests.
    if(msg.get_slave())
    {
        msg.save_request();
        u8state = COM_WAITING;
    }
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
int8_t Master::poll(Message& msg)
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
    const int8_t i8bytes_read = getRxBuffer( msg.buf, msg.bufsize );
    if (i8bytes_read < 0)
    {
        return setError(i8bytes_read); // Pass error on from getRxBuffer().
    }
    msg.length = i8bytes_read;

    ++u16Counter[CNT_MASTER_RESPONSE];

    // validate message: length, exception
    int8_t i8error = msg.verify_response();
    if (i8error == 0)
    {
        u8state = COM_IDLE;
        return 1; // Response received and processed OK.
    }
    else if (i8error > 0)
    {
        u8state = COM_IDLE;
        ++u16Counter[CNT_MASTER_EXCEPTION];
    }
    else
    {
        ++u16Counter[CNT_MASTER_IGNORED];
    }
    return setError(i8error);
}


/* ____PRIVATE FUNCTIONS_____MASTER__________________________________________ */


} // end namespace modbus
