/**
 * Uses a loopback stream to set up communication between a master and slave on the
 * same machine. Exercises various MODBUS functions, and tests that the results
 * are correct.
 *
 * Test failures are reported to Serial, labelled "FAIL".
 */

//??#include <ModbusMaster.h>
#include <ModbusRtu.h>
#include <ModbusSlave.h>
#include "src/loopback.h"

// Set to 1, to report PASSes as well as FAILures.
// Set to higher numbers for increasing levels of detail.
#define VERBOSE_RESULTS 0

// Set to 1 to quit after the first failure.
#define ABORT_ON_FAIL 0

uint16_t test_count =0;
uint16_t pass_count =0;


//
// Master

const uint16_t master_data_count = 16;
uint16_t master_data[master_data_count+1];
modbus_t telegram;
int8_t master_poll_result;

Loopback master_stream(MAX_BUFFER+1);
Modbus master(0,master_stream,0);


//
// Slave

const uint8_t slave_id = 1;
const uint16_t slave_data_count = 9;
uint16_t slave_data[slave_data_count+1]; ///< Include extra OOB register
CoilBlockData coil_block(slave_data, 16*slave_data_count);
RegisterBlockData reg_block(slave_data, slave_data_count);
Mapping mapping(reg_block, coil_block);
int8_t slave_poll_result;

Loopback slave_stream(MAX_BUFFER+1);
Slave slave(slave_id,slave_stream,0);


void report(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  Serial.print(type);
  Serial.print(F(": "));
  Serial.print(addr);
  Serial.print(F("="));
  Serial.print(val);
  Serial.print(F(" ?"));
  Serial.println(expected);
}
void pass(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  ++test_count;
  ++pass_count;
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=1)
  Serial.print(F("PASS "));
  report(type, addr, val, expected);
#else
  (void)type;
  (void)addr;
  (void)val;
  (void)expected;
#endif
}
void fail(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  ++test_count;
  Serial.print(F("FAIL "));
  report(type, addr, val, expected);
#if ABORT_ON_FAIL
  Serial.println(F(">>> ABORT <<<"));
  Serial.flush();
  abort();
#endif
}

void test_equal(const char* type, uint16_t addr, uint16_t val, uint16_t expected)
{
  if(val!=expected)
      fail(type, addr, val, expected);
  else
      pass(type, addr, val, expected);
}

uint16_t addr2word(uint16_t addr)
{
  return modbus::Base::calcCRC(&addr, sizeof(addr));
}

bool addr2bool(uint16_t addr)
{
  uint16_t index = addr / 16;
  uint16_t crc = modbus::Base::calcCRC(&index, sizeof(index));
  return bitRead(crc, addr % 16);
}

void init_master()
{
  telegram.u8id = slave_id;
  telegram.u8fct = 0;
  telegram.u16RegAdd = 0;
  telegram.u16CoilsNo = 0;
  telegram.au16reg = master_data; // pointer to a memory array in the Arduino
}

void init_slave()
{
  // Also initialise the *extra* register, at the end of the array.
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
  Serial.print(F("\ninit_slave:"));
#endif
  for(size_t i=0; i<=slave_data_count; ++i)
  {
    slave_data[i] = addr2word(i);
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
    Serial.print(" ");
    Serial.print(slave_data[i]);
#endif
  }
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
  Serial.println("");
#endif
}

struct Expect {
  int8_t slave;
  int8_t master;
  
  Expect(): slave(0), master(0) {}
  void reset() { slave=0; master=0; }
  void set_all(int8_t v) { slave=v; master=v; }
} expect;

/** Poll both master and slave. */
void poll()
{
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=4)
      master_stream.print_status(Serial, "master");
      slave_stream.print_status(Serial, "slave");
#endif

  slave_poll_result = slave.poll(mapping);
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
      master_stream.print_status(Serial, "master");
      slave_stream.print_status(Serial, "slave");
#endif

  if(slave_poll_result && slave_poll_result != expect.slave)
  {
    Serial.print(F("Unexpected slave::poll()=")); Serial.print(slave_poll_result);
    if(slave_poll_result == ERR_EXCEPTION)
    {
      Serial.print(F(" last:"));
      Serial.print(slave.getLastError());
    }
    Serial.println("");
  }

  master_poll_result = master.poll();
#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
      master_stream.print_status(Serial, "master");
      slave_stream.print_status(Serial, "slave");
#endif

  if(master_poll_result<0 && master_poll_result != expect.master)
  {
    Serial.print(F("Unexpected master::poll()=")); Serial.print(master_poll_result);
    if(master_poll_result == ERR_EXCEPTION)
    {
      Serial.print(F(" last:"));
      Serial.print(master.getLastError());
    }
    Serial.println("");
  }

#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=2)
      Serial.print(F("  poll: slave=")); Serial.print(slave_poll_result);
      Serial.print(F(" master="));     Serial.print(master_poll_result);
      Serial.print(F(" st:"));   Serial.print(master.getState());
      Serial.print(F(" queries:"));  Serial.print(master.getCounter(CNT_MASTER_QUERY));
      Serial.print(F(" responses:"));   Serial.print(master.getCounter(CNT_MASTER_RESPONSE));
      Serial.print(F(" exceptions:"));  Serial.print(master.getCounter(CNT_MASTER_EXCEPTION));
      Serial.print(F(" last:")); Serial.print(master.getLastError());
      Serial.println("");
#endif
  if(master.getState()!=COM_WAITING)
      expect.reset();
}


//
// HOLDING REGISTERS

uint16_t read_holding_register(uint16_t addr)
{
  telegram.u8fct = MB_FC_READ_HOLDING_REGISTERS;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 1;
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
  return telegram.au16reg[0];
}

void write_holding_register(uint16_t addr, uint16_t val)
{
  telegram.u8fct = MB_FC_WRITE_SINGLE_REGISTER;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 0;
  telegram.au16reg[0] = val;
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
}

/** Holding registers are read/write. */
void test_holding_register(uint16_t reg_addr)
{
  // First check the starting value.
  uint16_t val = read_holding_register(reg_addr);
  uint16_t expected = addr2word(reg_addr);
  test_equal("test_holding_register, starting", reg_addr, val, expected);

  // Test write & read-back.
  uint16_t new_val = addr2word(~expected+321);

  // Write
  write_holding_register(reg_addr, new_val);
  test_equal("test_holding_register, write", reg_addr, slave_data[reg_addr], new_val);

  // Read
  val = read_holding_register(reg_addr);
  test_equal("test_holding_register, read", reg_addr, val, new_val);
}

void test_oob_holding_register()
{
  uint16_t errcnt0 = master.getCounter(CNT_MASTER_EXCEPTION);

  slave.clearLastError();
  master.clearLastError();
  expect.set_all(ERR_EXCEPTION);
  uint16_t val = read_holding_register(slave_data_count);
  test_equal(
      "test_oob_holding_register, non-existent",
      slave_data_count,
      master.getCounter(CNT_MASTER_EXCEPTION),
      errcnt0+1
    );
  test_equal(
      "test_oob_holding_register, non-existent: slave error code",
      slave_data_count,
      slave.getLastError(),
      modbus::EXC_ILLEGAL_DATA_ADDRESS
    );
  test_equal(
      "test_oob_holding_register, non-existent: master error code",
      slave_data_count,
      master.getLastError(),
      modbus::EXC_ILLEGAL_DATA_ADDRESS
    );

  // Check that the slave does not read beyond the bounds of its data array.
  test_equal(
      "test_oob_holding_register, OOB read check",
      slave_data_count,
      val == slave_data[slave_data_count],
      0 ///< Should be false
    );

  // Check that the slave will not write to arbitrary memory.
  slave.clearLastError();
  master.clearLastError();
  uint16_t new_val = 0xBAD;
  expect.set_all(ERR_EXCEPTION);
  write_holding_register(slave_data_count, new_val);
  test_equal(
      "test_oob_holding_register, OOB write check",
      slave_data_count,
      new_val == slave_data[slave_data_count],
      0 ///< Should be false
    );
  test_equal(
      "test_oob_holding_register, OOB write check: slave error code",
      slave_data_count,
      slave.getLastError(),
      modbus::EXC_ILLEGAL_DATA_ADDRESS
    );
  test_equal(
      "test_oob_holding_register, OOB write check: master error code",
      slave_data_count,
      master.getLastError(),
      modbus::EXC_ILLEGAL_DATA_ADDRESS
    );
}

void test_holding_registers()
{
  uint16_t errcnt0 = master.getCounter(CNT_MASTER_EXCEPTION);
  for(uint16_t reg_addr=0; reg_addr<slave_data_count; ++reg_addr)
  {
    test_holding_register(reg_addr);
  }
  test_equal("test_holding_registers, errcnt", 0, master.getCounter(CNT_MASTER_EXCEPTION), errcnt0);

  // Test for correct handling of an out-of-bounds register.
  test_oob_holding_register();
}


//
// COILS

bool read_coil(uint16_t addr)
{
  telegram.u8fct = MB_FC_READ_COILS;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 1;
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
  return telegram.au16reg[0];
}

void write_coil(uint16_t addr, bool val)
{
  telegram.u8fct = MB_FC_WRITE_SINGLE_COIL;
  telegram.u16RegAdd = addr;
  telegram.u16CoilsNo = 0;
  telegram.au16reg[0] = (val? 1: 0);
  master.query( telegram );
  while(master.getState()==COM_WAITING)
  {
    poll();
  }
}

/** Coils are read/write. */
void test_coil(uint16_t reg_addr)
{
  // First check the starting value.
  bool val = read_coil(reg_addr);
  bool expected = addr2bool(reg_addr);
  test_equal("test_coil, starting", reg_addr, val, expected);

  // Test write & read-back.
  bool new_val = reg_addr % 2; // ?? This pattern is too regular - could
                               // ?? be correct by accident.

  // Write
  write_coil(reg_addr, new_val);
  test_equal(
      "test_coil, write",
      reg_addr,
      bitRead(slave_data[reg_addr/16], reg_addr%16),
      new_val
    );

  // Read
  val = read_coil(reg_addr);
  test_equal("test_coil, read", reg_addr, val, new_val);
}

void test_coils()
{
  uint16_t errcnt0 = master.getCounter(CNT_MASTER_EXCEPTION);
  for(uint16_t reg_addr=0; reg_addr<(slave_data_count*16); ++reg_addr)
  {
    test_coil(reg_addr);
  }
  test_equal("test_coils, errcnt", 0, master.getCounter(CNT_MASTER_EXCEPTION), errcnt0);

  // Test for correct handling of a non-existent register.
  // Currently FAILS, because the slave does not check its bounds.
  slave.clearLastError();
  master.clearLastError();
  expect.set_all(ERR_EXCEPTION);
  read_coil(slave_data_count*16);
  test_equal("test_coils, non-existent",0,master.getCounter(CNT_MASTER_EXCEPTION),errcnt0+1);
  test_equal(
      "test_coils, non-existent: slave error code",
      0,
      slave.getLastError(),
      modbus::EXC_ILLEGAL_DATA_ADDRESS
    );
  test_equal(
      "test_coils, non-existent: master error code",
      0,
      master.getLastError(),
      modbus::EXC_ILLEGAL_DATA_ADDRESS
    );
}


void fill_array_with_test_data(uint16_t* arr, uint16_t len)
{
  const char test_data[] = "abcdefghijklmnopqrstuvwxyz0123456789";
  uint8_t* begin = (uint8_t*)arr;
  uint8_t* end = (uint8_t*)(arr + len);
  while(begin < end)
  {
    size_t run = end - begin;
    if(run > sizeof(test_data))
        run = sizeof(test_data);
    memcpy((void*)begin, (void*)test_data, run);
    begin += run;
  }
}


/** Read/write multiple registers. */
void test_multiple_registers()
{
  const uint16_t errcnt0 = master.getCounter(CNT_MASTER_EXCEPTION);
  const uint16_t max_num = min(slave_data_count, master_data_count);
  for(uint16_t num=2; num<max_num; ++num)
  {
    for(uint16_t reg_addr=0; (reg_addr+num)<slave_data_count; ++reg_addr)
    {

      telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS;
      telegram.u16RegAdd = reg_addr;
      telegram.u16CoilsNo = num;
      fill_array_with_test_data(telegram.au16reg, num);
      const uint16_t crc0 = modbus::Base::calcCRC(telegram.au16reg, num*2);

      master.query( telegram );
      while(master.getState()==COM_WAITING)
      {
        poll();
      }

      // Check that the slave data has been set correctly.
      const uint16_t test_id = num*100 + reg_addr;
      const uint16_t crc1 = modbus::Base::calcCRC(slave_data+reg_addr, num*2);
      test_equal("test_multiple_registers, write", test_id, crc1, crc0);
    }
  }
  test_equal("test_multiple_registers, master errCnt",0,master.getCounter(CNT_MASTER_EXCEPTION),errcnt0);
}


bool getBitN(uint8_t* data, uint16_t n)
{
  uint8_t byte = data[n/8];
  return byte & (1<<(n%8));
}


void test_equal_bits(const char* type, uint16_t testno, uint16_t quantity, uint8_t* d0, uint8_t* d1, uint8_t d1_offset)
{
  for(uint16_t q=0; q<quantity; ++q)
  {
    bool v0 = getBitN(d0,q);
    bool v1 = getBitN(d1,q+d1_offset);
    if(v0 != v1)
    {
      fail(type, testno, q, v1);
      return;
    }
  }
  pass(type, testno, 0, 0);
}


/** Read/write multiple coils. */
void test_multiple_coils()
{
  const uint16_t errcnt0 = master.getCounter(CNT_MASTER_EXCEPTION);
  const uint16_t max_num = 16*min(slave_data_count, master_data_count);
  uint16_t test_id = 0;
  for(uint16_t num=2; num<max_num; num+=3)
  {
    for(uint16_t reg_addr=0; (reg_addr+num) < 16*slave_data_count; reg_addr+=3)
    {
      telegram.u8fct = MB_FC_WRITE_MULTIPLE_COILS;
      telegram.u16RegAdd = reg_addr;
      telegram.u16CoilsNo = num;
      uint8_t num_bytes = (num+7)/8;
      fill_array_with_test_data(telegram.au16reg, num_bytes);

#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
      Serial.write((uint8_t*)(telegram.au16reg), num_bytes);
      Serial.println("");
#endif

      // Clear the slave data.
      memset(slave_data, 0, slave_data_count+1);

      master.query( telegram );
      while(master.getState()==COM_WAITING)
      {
        poll();
      }

#if defined(VERBOSE_RESULTS) && (VERBOSE_RESULTS>=3)
      for(size_t i=0; i<slave_data_count+1; ++i)
      {
        Serial.print(((uint8_t*)slave_data)[i], HEX);
        Serial.print(" ");
      }
      Serial.println("");
#endif

      // Check that the slave data has been set correctly.
      ++test_id;
      test_equal_bits("test_multiple_coils, write", test_id, num, (uint8_t*)(telegram.au16reg), (uint8_t*)slave_data, reg_addr);
    }
  }
  test_equal("test_multiple_coils, master errCnt",0,master.getCounter(CNT_MASTER_EXCEPTION),errcnt0);
}


/** Test the MODBUS CRC calculation, by comparing the output of calcCRC() with
 *  a few examples from the MODBUS spec document. */
void test_crc()
{
  // Example from https://en.wikipedia.org/wiki/Modbus
  uint8_t test1[] = {0x01, 0x04, 0x02, 0xFF, 0xFF};
  test_equal("test_crc", 1, modbus::Base::calcCRC(test1,sizeof(test1)), 0x80B8);

  // Examples from https://www.modbustools.com/modbus.html
  uint8_t test2[] = {0x04, 0x01, 0x00, 0x0A, 0x00, 0x0D};
  test_equal("test_crc", 2, modbus::Base::calcCRC(test2,sizeof(test2)), 0x98DD);

  uint8_t test3[] = {0x04, 0x02, 0x00, 0x0A, 0x00, 0x0D};
  test_equal("test_crc", 3, modbus::Base::calcCRC(test3,sizeof(test3)), 0x9899);

  uint8_t test4[] = {0x04, 0x02, 0x02, 0x0A, 0x11};
  test_equal("test_crc", 4, modbus::Base::calcCRC(test4,sizeof(test4)), 0x14B3);
}


void test_loopback()
{
  const char* test_str = "Hello";
  master_stream.write(test_str, (size_t)5);
  size_t i = 0;
  while(slave_stream.available())
  {
    int c = slave_stream.read();
    test_equal("test_loopback", i, c, test_str[i]);
    ++i;
  }
}



void setup()
{
  Serial.begin(115200);
  Serial.println(F(__FILE__ "  Build: " __DATE__ ", " __TIME__));

  // Cut down on inter-frame delays - not needed in loopback.
  master.setT35(0);
  slave.setT35(0);

  master_stream.connect(slave_stream);
}


long delay_seconds = 1;

void loop()
{
  test_count = pass_count = 0;

  test_crc();

  init_master();

  // Reset counters
  master.start();
  slave.start();

  test_loopback();

  init_slave();
  test_holding_registers();

  init_slave();
  test_coils();

  test_multiple_registers();
  test_multiple_coils();

  test_equal("master error count", 0, master.getCounter(CNT_MASTER_EXCEPTION), 3);

  Serial.print(F("DONE. Passed "));
  Serial.print(pass_count);
  Serial.print(F(" / "));
  Serial.println(test_count);
  delay((delay_seconds++)*1000);
}

