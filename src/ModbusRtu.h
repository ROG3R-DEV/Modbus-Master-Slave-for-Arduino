/**
 * @file        ModbusRtu.h
 * @version     1.23
 * @date        2019.07.02
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

#include "ModbusMaster.h"
#include "ModbusSlave.h"

namespace modbus {


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
        { return impl->getCounter(CNT_BUS_MESSAGE); }
    uint16_t getOutCnt(); //!<number of outgoing messages
    uint16_t getErrCnt() //!<error counter
        { return impl->getCounter(CNT_BUS_COMM_ERROR) + impl->getCounter(CNT_BUS_CHAR_OVERRUN); }
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


uint16_t Modbus::getOutCnt()
{
    if (is_master)
        return impl->getCounter(CNT_MASTER_QUERY);
    else
        return impl->getCounter(CNT_SLAVE_MESSAGE) - impl->getCounter(CNT_SLAVE_NO_RESPONSE);
}


} // end namespace modbus

#if !defined(USING_MODBUS_NAMESPACE) || USING_MODBUS_NAMESPACE!=0
using namespace modbus;
#endif

#endif // MODBUS_MODBUS_RTU_H
