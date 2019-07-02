/**
 *  Modbus slave example 3:
 *  The purpose of this example is to link a data array
 *  from the Arduino to an external device through RS485.
 *
 *  Recommended Modbus Master: QModbus
 *  http://qmodbus.sourceforge.net/
 */

#include <ModbusRtu.h>

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN  4 

// data array for modbus network sharing
uint16_t au16data[16] = {
  3, 1415, 9265, 4, 2, 7182, 28182, 8, 0, 0, 0, 0, 0, 0, 1, (uint16_t)-1 };

// Holding registers traditionally start at address 40000.
RegisterBlockData rb(au16data, 16, 40000);
Mapping mapping(rb);


/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Slave slave(1,Serial,TXEN); // this is slave @1 and RS-485

void setup() {
  // Set serial port to 19200 baud,
  // 8 data bits, Even parity, 1 stop bit, as required by MODBUS standard.
  Serial.begin( 19200, SERIAL_8E1 );
  slave.start();
}

void loop() {
  if (Serial.available()) {
      int8_t retval = slave.poll( mapping );
      if (retval < 0) {
          // Error
          // (See advanced examples for code that blinks the builtin LED
          // to report the error.)
      }
  }
}
