#include "ModbusBase.h"

namespace modbus {


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
    clearCounters();
}


/**
 * @brief  Set all counters to zero.
 * Called at start-up, and also by the diagnostic reset.
 *
 * @ingroup buffer
 */
void Base::clearCounters()
{
    memset(u16Counter, 0, NUM_COUNTERS);
}


/**
 * @brief Get counter value.
 *
 * @ingroup buffer
 */
uint16_t Base::getCounter(uint8_t counterId_) const
{
    if (counterId_ < NUM_COUNTERS)
        return u16Counter[counterId_];
    else
        return UINT16_MAX;
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


/* ____PROTECTED FUNCTIONS___BASE____________________________________________ */

/**
 * @brief  Base class constructor.
 * @ingroup setup
 */
Base::Base(Stream& port_, uint8_t u8txenpin_):
    port(&port_),
    u8txenpin(u8txenpin_),
    t35(5), // Default to 5ms inter-frame delay
    i8lastError(0),
    iLastBytesAvailable(0),
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
            if (t35==0 || (now - ulT35timer) >= (unsigned long)t35)
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
    uint8_t count = 0;
    while(true)
    {
      const int c = port->read();
      if (count < bufsize)
      {
        if (c < 0)
            break;
        buf[ count++ ] = c;
      }
      else if (c < 0)
      {
        ++u16Counter[CNT_BUS_CHAR_OVERRUN];
        return ERR_RX_BUFF_OVERFLOW;
      }
    }

    if (count <= 3)
    {
        ++u16Counter[CNT_BUS_COMM_ERROR];
        return ERR_MALFORMED_MESSAGE;
    }

    // check message crc vs calculated crc
    const uint16_t u16MsgCRC = word( buf[count - 1], buf[count - 2] );
    if ( calcCRC( buf, count-2 ) != u16MsgCRC )
    {
        ++u16Counter[CNT_BUS_COMM_ERROR];
        return ERR_BAD_CRC;
    }

    ++u16Counter[CNT_BUS_MESSAGE];
    return count;
}


/**
 * @brief
 * This method transmits buf, followed by its CRC to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 *
 * @param  buf     data buffer.
 * @param  count   message length.
 * @ingroup buffer
 */
void Base::sendTxBuffer( const uint8_t* buf, uint8_t count )
{
    const uint16_t u16crc = calcCRC( buf, count );

    if (u8txenpin > 1)
    {
        // set RS485 transceiver to transmit mode
        digitalWrite( u8txenpin, HIGH );
    }

    // transfer buffer to serial line
    port->write( buf, count );

    // Append the CRC
    port->write( lowByte(  u16crc ) );
    port->write( highByte( u16crc ) );

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
}


} // end namespace modbus
