#ifndef MODBUS_MODBUS_MASTER_H
#define MODBUS_MODBUS_MASTER_H

/**
 * @file        ModbusMaster.h
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

#include "utility/ModbusCommon.h"
#include "utility/ModbusBase.h"
#include "utility/Message.h"

namespace modbus {


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
    uint16_t  u16timeOut;
    uint32_t  u32timeOut; //!< Timestamp of last query (millis).

public:
    Master(Stream& port, uint8_t u8txenpin =0);

    void setTimeOut( uint16_t u16timeOut); //!<write communication watch-dog timer
    bool timeOutExpired() const; //!<get communication watch-dog timer state
    uint8_t getState() const;
    void    cancel_request();

    int8_t send_request( Message& msg );
    int8_t poll( Message& msg );
};


} // end namespace modbus

#if !defined(USING_MODBUS_NAMESPACE) || USING_MODBUS_NAMESPACE!=0
using namespace modbus;
#endif

#endif // MODBUS_MODBUS_MASTER_H
