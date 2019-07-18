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

#include <ModbusMaster.h>

uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query

/**
 *  Master object declaration
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Master master(Serial,0); // this is RS-232 or USB-FTDI

/**
 * This is a struct which contains a query to a slave device
 */
Message telegram;

unsigned long u32time;


/**
 *  LED blinking utility, used for data reporting.
 *  Start with a very long flash, followed by the 16 bit pattern in
 *  dots and dashes. This is only intended to signal the presence of data
 *  it's probably too fast to practically read it.
 */
void blink_bits(uint16_t data) {
  // Start with a long flash to signal what's coming...
  digitalWrite( LED_BUILTIN, HIGH ); delay( 1000 );
  digitalWrite( LED_BUILTIN, LOW );  delay( 500 );
  for (int8_t b=0; b<16; ++b) {
      digitalWrite( LED_BUILTIN, HIGH );
      if (bitRead( data, 15 - b )) {
          delay( 250 );
          digitalWrite( LED_BUILTIN, LOW );
          delay( 50 );
      } else {
          delay( 50 );
          digitalWrite( LED_BUILTIN, LOW );
          delay( 250 );
      }
  }
}


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
      blink( -err, 400 );
      // For exceptions, blink the LED "exc" times, slow.
      if (err==ERR_EXCEPTION)
          blink( master.getLastError(), 1000 );
      delay( 5000 );
  }
}


void setup() {
  // Set serial port to baud-rate at 19200,
  // 8 data bits, Even parity, 1 stop bit, as required by MODBUS standard.
  Serial.begin( 19200, SERIAL_8E1 );
  master.start();
  master.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
  u32time = millis();
  u8state = 1;
  u8query = 0;

  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite(LED_BUILTIN, LOW );
}

void loop() {
  int8_t retval;
  switch( u8state ) {
    case 0:
        // Wait for 1000ms.
        if (millis() - u32time > 1000) {
            blink(1,10);
            u8state++;
        }
        break;

    case 1:
        switch(u8query) {
          case 0:
              // Read "input registers" 30000-30001 from slave ID=1.
              telegram.fc_read_input_registers(1, 30000, 2);
              break;

          case 1:
              // Write the value of pin A0 to "holding register" 40000,
              // on slave ID=1
              telegram.fc_write_single_register(1, 40000, analogRead( A0 ));
              break;
        }

        retval = master.send_request( telegram ); // send request (only once)
        if (retval < 0) {
            report_error( retval );
            // Reset
            u8state = 0;
            u32time = millis();
        } else {
            u8state++;
        }
        break;

    case 2:
        retval = master.poll( telegram ); // check incoming messages
        if (retval < 0) {
            report_error( retval );
            // Reset
            u8state = 0;
            u32time = millis();
        }
        else if (master.getState() == COM_IDLE) {
            if (u8query == 0) {
                // Do something with the return data...
                // For example, blink the builtin LED really fast...
                uint16_t return_data = telegram.get_register( 0 );
                blink_bits( return_data );
            }

            // Move on to the next query
            u8query = (u8query + 1) % 2;
            u8state = 0;
            u32time = millis();
        }
        break;
  }

}

