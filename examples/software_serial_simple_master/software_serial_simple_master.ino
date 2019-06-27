/**
 *  Modbus master example 2:
 *  The purpose of this example is to query an array of data
 *  from an external Modbus slave device.
 *  This example is similar to "simple_master", but this example
 *  allows you to use software serial instead of hardware serial
 *  in case that you want to use D1 & D2 for other purposes.
 *  The link media can be USB or RS232.
 
  The circuit:
 * software serial rx(D3) connect to tx pin of another device
 * software serial tx(D4) connect to rx pin of another device
 
 * In this example, we will use two important methods so that we can use
 * software serial.
 *
 * 1. Modbus::Modbus(uint8_t u8id)
 * This is a constructor for a Master/Slave through USB/RS232C via software serial
 * This constructor only specifies u8id (node address) and should be only
 * used if you want to use software serial instead of hardware serial.
 * This method is called if you create a ModBus object with only on parameter "u8id"
 * u8id is the node address of the arduino that will be programmed on,
 * 0 for master and 1..247 for slave
 * for example: Modbus master(0); 
 * If you use this constructor you have to begin ModBus object by
 * using "void Modbus::begin(SoftwareSerial *softPort, long u32speed)".
 * 
 * 2. void Modbus::begin(SoftwareSerial *sPort, long u32speed)
 * Initialize class object.
 * This is the method you have to use if you construct the ModBus object by using 
 * Modbus::Modbus(uint8_t u8id) in order to use software serial and to avoid problems.
 * You have to create a SoftwareSerial object on your own, as shown in the example.
 * sPort is a pointer to your SoftwareSerial object, u32speed is the baud rate, in 
 * standard increments (300..115200)

 created long time ago
 by smarmengol
 modified 29 July 2016
 by Helium6072

 This example code is in the public domain.
 */

#include <ModbusRtu.h>
#include <SoftwareSerial.h>

// data array for modbus network sharing
uint16_t au16data[16];

// Create a SoftwareSerial object so that we can use software serial.
// Search "software serial" on Arduino.cc to find out more details.
#define RX_PIN 3
#define TX_PIN 5
SoftwareSerial mySerial(RX_PIN, TX_PIN);

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Master master(mySerial); // this is master and RS-232 or USB-FTDI via software serial

/**
 * This is a struct which contains a query to a slave device
 */
modbus_t telegram;

uint8_t u8state;
unsigned long u32time;


/**
 * In this example, errors can be reported directly to the Serial port.
 */
void report_error(const char* function, int8_t err) {
  // Error codes are negative.
  if (err < 0) {
      Serial.print(function);
      Serial.print(F("() reported error "));
      Serial.println(err);
      if (err==ERR_EXCEPTION) {
          Serial.print(F("  -> exception code "));
          Serial.println( master.getLastError() );
      }
  }
}


void setup() {
  // Set software serial port to 19200 baud,
  // Arduino's default SoftwareSerial implementation does not implement
  // parity checking, so "8 bits data, no parity, 1 stop bit" is forced.
  // If you want EVEN parity (MODBUS default) then you could try this
  // library (I have not tested it! -AT):
  // https://create.arduino.cc/projecthub/luke-j-barker/softwareserial-with-parity-9ede24
  mySerial.begin( 19200 );

  // start the modbus::Master object.
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
    telegram.u8id = 1; // slave address
    telegram.u8fct = MB_FC_READ_REGISTERS; // function code 3
    telegram.u16RegAdd = 40000; // start address in slave
    telegram.u16CoilsNo = 4; // number of elements (coils or registers) to read
    telegram.au16reg = au16data; // pointer to a memory array in the Arduino

    retval = master.query( telegram ); // send query (only once)
    if (retval < 0) {
        report_error("query", retval);
        // Reset
        u8state = 0;
        u32time = millis();
    } else {
        u8state++;
    }
    break;

  case 2:
    retval = master.poll();
    if (retval < 0) {
        report_error("poll", retval);
        // Reset
        u8state = 0;
        u32time = millis();
    }
    else if (master.getState() == COM_IDLE) {
         // Do something with the data...
        Serial.print(F("Master got:"));
        for (size_t i=0; i<telegram.u16CoilsNo; ++i) {
            Serial.print(" ");
            Serial.print(au16data[i]);
        }
        Serial.println("");
        // Reset
        u8state = 0;
        u32time = millis();
    }
    break;
  }
}
