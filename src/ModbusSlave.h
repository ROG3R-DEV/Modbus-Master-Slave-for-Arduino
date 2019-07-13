#ifndef MODBUS_MODBUS_SLAVE_H
#define MODBUS_MODBUS_SLAVE_H

/**
 * @file        ModbusSlave.h
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
 */

#include "utility/Mapping.h"
#include "utility/ModbusBase.h"
#include "utility/ModbusCommon.h"

namespace modbus {


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
    bool    listenOnlyMode; //! Slave comms disabled.

private:
    int8_t buildException( uint8_t u8exception, uint8_t* buf );
    int8_t process_FC1( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize );
    int8_t process_FC3( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize );
    int8_t process_FC5( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC6( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC7( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC8( uint8_t* buf, uint8_t& count );
    int8_t process_FC11( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC15( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC16( Mapping& mapping, uint8_t* buf, uint8_t& count );
    int8_t process_FC23( Mapping& mapping, uint8_t* buf, uint8_t& count, uint8_t bufsize );

public:
    Slave(uint8_t u8id, Stream& port, uint8_t u8txenpin =0);

    void setID( uint8_t u8id ); //!<write new ID for the slave
    uint8_t getID() const; //!<get slave ID between 1 and 247
    bool isListenOnly() const { return listenOnlyMode; }

    int8_t poll( uint16_t *regs, uint8_t u8size ); //!<cyclic poll for slave
    int8_t poll( Mapping& mapping );               //!<cyclic poll for slave
};


} // end namespace modbus

#if !defined(USING_MODBUS_NAMESPACE) || USING_MODBUS_NAMESPACE!=0
using namespace modbus;
#endif

#endif // MODBUS_MODBUS_SLAVE_H
