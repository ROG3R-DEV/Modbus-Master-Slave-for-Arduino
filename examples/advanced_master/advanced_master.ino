/**
 *  Modbus master example 2:
 *  The purpose of this example is to query several sets of data
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

#include <ModbusRtu.h>

uint16_t au16data[16]; //!< data array for modbus network sharing
uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Master master(Serial,0); // this is master and RS-232 or USB-FTDI

/**
 * This is an struct which contains a query to an slave device
 */
modbus_t telegram[2];

unsigned long u32wait;


/**
 *  LED blinking utility, used for error reporting.
 */
void blink(int8_t times, int delay_ms) {
  pinMode( LED_BUILTIN, OUTPUT );
  for (int8_t i=0; i<times; ++i) {
      digitalWrite( LED_BUILTIN, HIGH );
      delay( delay_ms );
      digitalWrite( LED_BUILTIN, LOW );
      delay( delay_ms );
  }
}


/**
 *  Report error status by blinking the built-in LED.
 *  Number of short flashes = ERR state.
 *  Number of long flashes = exception number.
 */
void report_error(int8_t err) {
  // Error codes are negative.
  if (err < 0) {
      // Blink the LED "err" times, fast.
      blink( -err, 500 );
      // For exceptions, blink the LED "exc" times, slow.
      if (err==ERR_EXCEPTION)
          blink( master.getLastError(), 1000 );
      delay( 5000 );
  }
}


void setup() {
  // telegram 0: read registers
  telegram[0].u8id = 1; // slave address
  telegram[0].u8fct = 3; // function code (this one is registers read)
  telegram[0].u16RegAdd = 0; // start address in slave
  telegram[0].u16CoilsNo = 4; // number of elements (coils or registers) to read
  telegram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // telegram 1: write a single register
  telegram[1].u8id = 1; // slave address
  telegram[1].u8fct = 6; // function code (this one is write a single register)
  telegram[1].u16RegAdd = 4; // start address in slave
  telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
  telegram[1].au16reg = au16data+4; // pointer to a memory array in the Arduino

  // Set serial port to baud-rate at 19200,
  // 8 data bits, Even parity, 1 stop bit, as required by MODBUS standard.
  Serial.begin( 19200, SERIAL_8E1 );
  master.start();
  master.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
  u32wait = millis() + 1000;
  u8state = u8query = 0; 
}

void loop() {
  int8_t retval;
  switch( u8state ) {
  case 0:
    if (millis() > u32wait) u8state++; // wait state
    break;

  case 1:
    retval = master.query( telegram[u8query] ); // send query (only once)
    if (retval < 0) {
        report_error( retval );
        break;
    }
    u8state++;
    u8query++;
    if (u8query > 2) u8query = 0;
    break;

  case 2:
    retval = master.poll(); // check incoming messages
    if (retval < 0) {
        report_error( retval );
        break;
    }
    if (master.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 1000; 
    }
    break;
  }

  au16data[4] = analogRead( 0 );
  
}

