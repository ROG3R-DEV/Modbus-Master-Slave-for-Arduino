#ifndef MODBUS_MODBUS_BASE_H
#define MODBUS_MODBUS_BASE_H

#include "utility/ModbusCommon.h"

#include <Arduino.h>

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
    uint8_t t35;
    int8_t  i8lastError;
    int     iLastBytesAvailable;
    uint16_t u16Counter[NUM_COUNTERS];
    unsigned long  ulT35timer; // Type matches millis() return type.
    uint32_t u32overTime;

protected:
    Base(Stream& port, uint8_t u8txenpin =0);

    int8_t setError( int8_t i8error );
    bool   rxFrameReady();
    int8_t getRxBuffer( uint8_t* buf, uint8_t count );
    void   sendTxBuffer( const uint8_t* buf, uint8_t count );

public:
    void start();
    void clearCounters();
    uint16_t getCounter(uint8_t counterId_) const;
    int8_t   getLastError() const; //!< Get last error (ERR_XXX) or exception (EXC_XXX) code.
    void     clearLastError(); //!< Set last error to 0.
    void setT35(uint8_t v) { t35 = v; }
    void setTxendPinOverTime( uint32_t u32overTime );

    static uint16_t calcCRC( const void* data, uint8_t len );

    friend class Modbus;
};


} // end namespace modbus

#endif // MODBUS_MODBUS_BASE_H
