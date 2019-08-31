#ifndef MODBUS_MODBUS_COMMON_H
#define MODBUS_MODBUS_COMMON_H

#include <stdint.h>

namespace modbus {


/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
enum MESSAGE
{
    ID                             = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT  //!< byte counter
};

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also modbus_t
 */
enum MB_FC
{
    MB_FC_NONE                     = 0,  /*!< null operator */
    MB_FC_READ_COILS               = 1,  /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUTS     = 2,  /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_HOLDING_REGISTERS   = 3,  /*!< FCT=3 -> read holding registers or analog outputs */
    MB_FC_READ_INPUT_REGISTERS     = 4,  /*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_SINGLE_COIL        = 5,  /*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_SINGLE_REGISTER    = 6,  /*!< FCT=6 -> write single register */
    MB_FC_READ_EXCEPTION_STATUS    = 7,
    MB_FC_DIAGNOSTICS              = 8,

    MB_FC_GET_COMM_EVENT_COUNTER   = 11,
    MB_FC_GET_COMM_EVENT_LOG       = 12,

    MB_FC_WRITE_MULTIPLE_COILS     = 15, /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16, /*!< FCT=16 -> write multiple registers */
    MB_FC_REPORT_SERVER_ID         = 17,

    MB_FC_READ_FILE_RECORD         = 20,
    MB_FC_WRITE_FILE_RECORD        = 21,
    MB_FC_MASK_WRITE_REGISTER      = 22,
    MB_FC_READ_WRITE_MULTIPLE_REGISTERS = 23,
    MB_FC_READ_FIFO_QUEUE          = 24,

    // Old, backward compatibility aliases.
    MB_FC_READ_DISCRETE_INPUT      = 2,
    MB_FC_READ_REGISTERS           = 3,
    MB_FC_READ_INPUT_REGISTER      = 4,
    MB_FC_WRITE_COIL               = 5,
    MB_FC_WRITE_REGISTER           = 6
};

enum COM_STATES
{
    COM_IDLE                     = 0,
    COM_WAITING                  = 1
};

enum ERR_LIST
{
    ERR_WAITING           = -1, //!< Master::query(). Master is still waiting for previous query.
    ERR_FUNC_CODE         = -2, //!< Master::query(). Bad / unsupported function code.
    ERR_BAD_SLAVE_ID      = -3, //!< Master::query(). Telegram specifies invalid slave ID.
    ERR_BAD_CRC           = -4, //!< poll() methods.
    ERR_EXCEPTION         = -5, //!< Master::poll(): Exception received from slave. Slave::poll(): Exception returned to master.
    ERR_TIME_OUT_EXPIRED  = -6, //!< Master::poll(). Master has given up waiting for answer.
    ERR_TX_BUFF_OVERFLOW  = -7, //!< Any Tx method.
    ERR_RX_BUFF_OVERFLOW  = -8, //!< poll() methods.
    ERR_MALFORMED_MESSAGE = -9, //!< poll() methods.
    ERR_OUT_OF_MEMORY     = -10,
    ERR_LISTEN_ONLY_MODE  = -11,// Slave is in listen only mode.
    ERR_PDU               = -12
};

enum EXC_LIST
{
    // Standard names, from MODBUS protocol.
    EXC_ILLEGAL_FUNCTION      = 1,
    EXC_ILLEGAL_DATA_ADDRESS  = 2,
    EXC_ILLEGAL_DATA_VALUE    = 3,
    EXC_SERVER_DEVICE_FAILURE = 4,
    EXC_ACKNOWLEDGE           = 5,
    EXC_SERVER_DEVICE_BUSY    = 6,
    EXC_NEGATIVE_ACKNOWLEDGE  = 7, // http://www.simplymodbus.ca/exceptions.htm
    EXC_MEMORY_PARITY_ERROR   = 8,
    EXC_GATEWAY_PATH_UNAVAILABLE = 10,
    EXC_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 11,

    // Old, backward compatibility names.
    EXC_FUNC_CODE  = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE    = 4
};

/** MODBUS serial line counters.
 *  Defined in Modbus_over_serial_line_V1_02.pdf, section 6.1.1, page 35.
 * (Numbers used here are indexes into a zero-based array. Add one to 
 *  get the MODBUS counter number.)
 *
 *  # Common:
 *  All messages seen on bus =
 *    CNT_BUS_MESSAGE + CNT_BUS_COMM_ERROR + CNT_BUS_CHAR_OVERRUN
 *
 *  # For slaves:
 *  CNT_SLAVE_MESSAGE <= CNT_BUS_MESSAGE
 *  Non-exception responses sent by slave =
 *    CNT_SLAVE_MESSAGE - CNT_SLAVE_EXCEPTION - CNT_SLAVE_NO_RESPONSE
 *  CNT_SLAVE_NAK <= CNT_SLAVE_EXCEPTION
 *  CNT_SLAVE_BUSY <= CNT_SLAVE_EXCEPTION
 *  
 *  # For masters:
 *  
 *  CNT_MASTER_RESPONSE <= CNT_BUS_MESSAGE
 *  Good responses received by master =
 *    CNT_MASTER_RESPONSE - CNT_MASTER_EXCEPTION - CNT_MASTER_IGNORED
 *  Responses still pending =
 *    CNT_MASTER_QUERY - CNT_MASTER_RESPONSE* - CNT_MASTER_TIMEOUT
 *  * - Although rogue stations on the bus might send extra responses
 *      that do not correspond to a query.
 */
enum CounterId
{
    // Event counter used by FC11: Counts calls to FC11.

    CNT_CALLS_TO_FC11     = 0,

    // Counters defined in MODBUS Protocol for slaves:

    CNT_BUS_MESSAGE       = 1, ///< Valid messages seen on the bus.
    CNT_BUS_COMM_ERROR    = 2, ///< CRC fails, and gibberish.
    CNT_SLAVE_EXCEPTION   = 3, ///< Slave: Exception responses sent
    CNT_SLAVE_MESSAGE     = 4, ///< Slave: Messages arrived for this server.
    CNT_SLAVE_NO_RESPONSE = 5, ///< Slave: Messages with no response sent.
    CNT_SLAVE_NAK         = 6, ///< Slave: NAK (Exception 7) responses sent.
    CNT_SLAVE_BUSY        = 7, ///< Slave: Busy (Exception 6) responses sent.
    CNT_BUS_CHAR_OVERRUN  = 8, ///< Too-long messages seen on the bus.

    // Counters used in *this implementation* for masters:

    CNT_MASTER_EXCEPTION = 3, ///< Master: Exception responses received.
    CNT_MASTER_QUERY     = 4, ///< Master: Queries sent.
    CNT_MASTER_RESPONSE  = 5, ///< Master: Responses received
    CNT_MASTER_IGNORED   = 6, ///< Master: Bad responses ignored.
    CNT_MASTER_TIMEOUT   = 7, ///< Master: timeouts.

    NUM_COUNTERS = 9
};


/** Only need to swap endianness on little endian machines. */
inline uint16_t bswap16(uint16_t v) __attribute__((always_inline));
inline uint16_t bswap16(uint16_t v)
{
#if !defined(__BYTE_ORDER__) || !defined(__ORDER_LITTLE_ENDIAN__)
#  error "Cannot determine byte order."
#elif __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#  if __GNUC__ > 4 || (__GNUC__ == 4  &&  __GNUC_MINOR__ > 7)
      return __builtin_bswap16(v);
#  else
      return (v<<8)|(v>>8);
#  endif
#else
      return v;
#endif
}


/** Write a uint16_t into the buffer buf, in big-endian byte order. */
inline void marshal_u16(uint8_t* __restrict__ buf, const uint16_t& __restrict__ v) __attribute__((always_inline));
inline void marshal_u16(uint8_t* __restrict__ buf, const uint16_t& __restrict__ v)
{
    buf[0] = v >> 8;
    buf[1] = v & 0xFF;
}


/** Read a uint16_t from the buffer buf, in big-endian byte order. */
inline uint16_t demarshal_u16(const uint8_t* buf) __attribute__((always_inline));
inline uint16_t demarshal_u16(const uint8_t* buf)
{
    return (buf[0] << 8) | buf[1];
}


/** Calculate the number of bytes required to store a "quantity" of bits. */
inline uint8_t bitset_size(const uint16_t quantity) __attribute__((always_inline));
inline uint8_t bitset_size(const uint16_t quantity)
{
    return (quantity + 7) / 8;
}


#if !defined(MAX_BUFFER)
#define  MAX_BUFFER  64	///!< maximum size for the communication buffer in bytes
#endif


} // end namespace modbus

#endif // MODBUS_MODBUS_COMMON_H
