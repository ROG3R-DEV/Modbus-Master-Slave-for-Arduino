#ifndef MODBUS_MODBUS_COMMON_H
#define MODBUS_MODBUS_COMMON_H

#include <stdint.h>

namespace modbus {


/**
 * @struct modbus_t
 * @brief
 * Master query structure:
 * This includes all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct
{
    uint8_t u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
    uint8_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t u16RegAdd;    /*!< Address of the first register to access at slave/s */
    uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
    uint16_t *au16reg;     /*!< Pointer to memory image in master */
}
modbus_t;

enum
{
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};

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
 * @see also fctsupported
 * @see also modbus_t
 */
enum MB_FC
{
    MB_FC_NONE                     = 0,  /*!< null operator */
    MB_FC_READ_COILS               = 1,  /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUTS     = 2,  /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_HOLDING_REGISTERS   = 3,  /*!< FCT=3 -> read holding registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      = 4,  /*!< FCT=4 -> read analog inputs */
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
    ERR_MALFORMED_MESSAGE = -9,  //!< poll() methods.
    ERR_OUT_OF_MEMORY     = -10
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

    EXC_MEMORY_PARITY_ERROR   = 8,
    EXC_GATEWAY_PATH_UNAVAILABLE = 10,
    EXC_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 11,

    // Old, backward compatibility names.
    EXC_FUNC_CODE  = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE    = 4
};

const unsigned char fctsupported[] =
{
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUTS,
    MB_FC_READ_HOLDING_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_SINGLE_COIL,
    MB_FC_WRITE_SINGLE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
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


#if !defined(T35)
#define T35  5
#endif

#if !defined(MAX_BUFFER)
#define  MAX_BUFFER  64	//!< maximum size for the communication buffer in bytes
#endif


} // end namespace modbus

#endif // MODBUS_MODBUS_COMMON_H
