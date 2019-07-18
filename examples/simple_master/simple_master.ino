/**
 *  Modbus master example 1:
 *  The purpose of this example is to query an array of data
 *  from an external Modbus slave device. 
 *  The link media can be USB or RS232.
 *
 *  Recommended Modbus slave: 
 *  diagslave http://www.modbusdriver.com/diagslave.html
 *
 *  In a Linux box, run 
 *  "./diagslave /dev/ttyUSB0 -b 19200 -d 8 -s 1 -p none -m rtu -a 1"
 * 	This is:
 * 		serial port /dev/ttyUSB0 at 19200 baud 8N1
 *		RTU mode and address @1
 */

#include <ModbusMaster.h>

// data array for modbus network sharing
uint16_t au16data[16];

/**
 *  modbus::Master object declaration
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Master master(Serial,0); // this is master and RS-232 or USB-FTDI

/**
 * This is a struct which contains a query to a slave device
 */
Message telegram;

uint8_t u8state;
unsigned long u32time;


void setup() {
  // Set serial port to 19200 baud,
  // 8 data bits, Even parity, 1 stop bit, as required by MODBUS standard.
  Serial.begin( 19200, SERIAL_8E1 );
  master.start();
  master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
  u8state = 1; 
}


void loop() {
  int8_t retval;

  switch( u8state ) {
  case 0: 
    if (millis() - u32time > 2000) {
        u8state++; // wait state
    }
    break;

  case 1:
    // Create request to slave id=1, for register start addr=40000, quantity=4
    telegram.fc_read_holding_registers(1, 40000, 4);

    retval = master.send_request(telegram); // send request (only once)
    if (retval < 0) {
        // Error
        // (See advanced examples for code that blinks the builtin LED
        // to report the error.)

        // Reset
        u8state = 0;
        u32time = millis();
    } else {
        u8state++;
    }
    break;

  case 2:
    retval = master.poll(telegram);
    if (retval < 0) {
        // Error
        // (See advanced examples for code that blinks the builtin LED
        // to report the error.)

        // Reset
        u8state = 0;
        u32time = millis();
    }
    else if (master.getState() == COM_IDLE) {
        // Do something with the data...
        telegram.get_registers(au16data, sizeof(au16data)/sizeof(*au16data));

        // Reset
        u8state = 0;
        u32time = millis();
    }
    break;
  }
}

