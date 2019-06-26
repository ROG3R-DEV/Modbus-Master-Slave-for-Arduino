/**
 *  Modbus slave example 2:
 *  The purpose of this example is to link the Arduino digital and analog
 *  pins to an external device.
 *
 *  Recommended Modbus Master: QModbus
 *  http://qmodbus.sourceforge.net/
 *
 *  Editado al espa帽ol por LuxARTS
 */

// ES: Incluye la librer铆a del protocolo Modbus
// EN: Include the MODBUS protocol library.
#include <ModbusRtu.h>
#define ID   1

// ES: Crear instancia
//     - ID del nodo (1-247 para esclavo)
//     - Puerto serie
//     - Protocolo serie. 0 para RS-232 + USB (default), cualquier pin mayor a 1 para RS-485
//
// EN: Create the slave instance.
//     - Node ID (1-247 for slaves)
//     - Serial port
//     - Serial protocol. 0 for RS-232 + USB (default), any pin greater than 1 for RS-485
Slave slave(ID, Serial, 0);

unsigned long tempus;


// ES: La tabla de registros que se desea compartir por la red
// EN: The table of records that you want to share over the network
uint8_t coil[1];
CoilBlock coil_block(coil, 4, 0);

uint8_t discrete_input[1];
CoilBlock discrete_input_block(discrete_input, 4, 10000);

uint16_t input_register[2];
RegisterBlock input_register_block(input_register, 2, 30000);

uint16_t diagnostic_register[3];
RegisterBlock diagnostic_register_block(diagnostic_register, 3, 31000);

uint16_t holding_register[2];
RegisterBlock holding_register_block(holding_register, 2, 40000);

Mapping mapping;


/*********************************************************

  Use               Pins      Type              Addresses     Read  Write
  ----------------  --------  ----------------  -----------   ----  -----
  Digital output    2,3,4,5   coil              0..3          FC:1  FC:5,15
  Digital input     6,7,8,9   discrete input    10000..10003  FC:2  -
  Analogue input    A0,A1     input register    30000..30001  FC:4  -
  Diagnostics       -         input register    31000..31002  FC:4  -
  Analogue output   10,11     holding register  40000..40001  FC:3  FC:6,16

 ES: pin 13 (LED_BUILTIN) reservado para ver el estado de la comunicaci贸n
 EN: pin 13 (LED_BUILTIN) reserved for comms status display.

*********************************************************/
void setup_mapping() {
  // "coil"
  mapping.add_coil_block(coil_block);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  // "discrete input"
  mapping.add_discrete_input_block(discrete_input_block);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);

  // "input register"
  mapping.add_input_register_block(input_register_block);
  mapping.add_input_register_block(diagnostic_register_block);

  // "holding register"
  mapping.add_holding_register_block(holding_register_block);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  analogWrite(10, 0); // PWM 0%
  analogWrite(11, 0); // PWM 0%
}


/*********************************************************
 ES: Enlaza la tabla de registros con los pines
 EN: Link the table of records with the pins.
*********************************************************/
void mapping_poll() {
  // ES: Lee los bits 0..3 de la variable y los pone en las salidas digitales.
  // EN: Read bits 0..3 of the data array, and write them to the digital outputs.
  if (coil_block.is_dirty()) {
      digitalWrite( 2, bitRead( coil[0], 0 ) );
      digitalWrite( 3, bitRead( coil[0], 1 ) );
      digitalWrite( 4, bitRead( coil[0], 2 ) );
      digitalWrite( 5, bitRead( coil[0], 3 ) );
  }

  // ES: Lee las entradas digitales y las guarda en bits de la primera variable
  //     del vector (es lo mismo que hacer una m谩scara).
  // EN: Read the digital inputs and save them in bits 0..3 of the data array.
  //     (Like making a mask.)
  bitWrite( discrete_input[0], 0, digitalRead( 6 ) );
  bitWrite( discrete_input[0], 1, digitalRead( 7 ) );
  bitWrite( discrete_input[0], 2, digitalRead( 8 ) );
  bitWrite( discrete_input[0], 3, digitalRead( 9 ) );

  // ES: Lee las entradas anal贸gicas (ADC).
  // EN: Read the analogue inputs (ADC).
  input_register[0] = analogRead( A0 ); // 0 -> 0V .. 1023 -> 5V
  input_register[1] = analogRead( A1 );

  // ES: Diagn贸stico de la comunicaci贸n (para debug)
  // EN: Comms diagnostics (for debug).
  diagnostic_register[0] = slave.getInCnt();
  diagnostic_register[1] = slave.getOutCnt();
  diagnostic_register[2] = slave.getErrCnt();

  // ES: Cambia el valor del PWM
  // EN: Set the level of the PWN outputs.
  if (holding_register_block.is_dirty()) {
      analogWrite( 10, holding_register[0] ); // 0 -> 0% .. 255 -> 100%
      analogWrite( 11, holding_register[1] );
  }

  // ES: Borra el bit "dirty" (sucio).
  // EN: Clear the "dirty" flag.
  mapping.set_clean();
}


/*********************************************************
  ES: Utilidad de parpadeo de LED, usada para reportar errores.
  EN: LED blinking utility, used for error reporting.
*********************************************************/
void blink(int8_t times, int delay_ms) {
  pinMode( LED_BUILTIN, OUTPUT );
  for (int8_t i=0; i<times; ++i) {
      digitalWrite( LED_BUILTIN, HIGH );
      delay( delay_ms );
      digitalWrite( LED_BUILTIN, LOW );
      delay( delay_ms );
  }
}


/*********************************************************
  ES: Reporte el estado de error parpadeando el LED incorporado.
      N鷐ero de destellos cortos = estado de error interno.
      N鷐ero de destellos largos = n鷐ero de excepci髇.

  EN: Report error status by blinking the built-in LED.
      Number of short flashes = internal error state.
      Number of long flashes = exception number.
*********************************************************/
void report_error(int8_t err) {
  // ES: Los c骴igos de error internos son negativos. (ERR_XXX)
  // EN: Internal error codes are negative. (ERR_XXX)
  if (err < 0) {
      // ES: Parpadea el LED "err" veces, r醦ido.
      // EN: Blink the LED "err" times, fast.
      blink( -err, 500 );
      // ES: Para excepciones, parpadea el LED "exc" veces, lento.
      // EN: For exceptions, blink the LED "exc" times, slow.
      if (err==ERR_EXCEPTION)
          blink( slave.getLastError(), 1000 );
      delay( 5000 );
  }
}


/*********************************************************
 ES: Configuraci贸n del programa
 EN: Program configuration
*********************************************************/
void setup() {
  // ES: configura las entradas y salidas
  // EN: Configure inputs and outputs
  setup_mapping();

  // ES: Abre la comunicaci贸n como esclavo
  // EN: Start comms.
  Serial.begin( 19200, SERIAL_8E1 );
  slave.start();
}


/*********************************************************
 ES: Inicio del programa
 EN: Program start
*********************************************************/
void loop() {
  // ES: Comprueba el buffer de entrada
  // EN: Check the receive buffer
  if (Serial.available()) {
      int8_t retval = slave.poll( mapping );
      if (retval < 0) {
          report_error( retval );
      }
      tempus = millis();
      digitalWrite(LED_BUILTIN, HIGH);
  }

  if (millis() - tempus > 50)
      digitalWrite(LED_BUILTIN, LOW );
  
  // ES: Actualiza los pines de Arduino con la tabla de MODBUS
  // EN: Update the Arduino's pins with the MODBUS table
  mapping_poll();
}
