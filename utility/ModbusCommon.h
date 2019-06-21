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
    MB_FC_NONE                     = 0,   /*!< null operator */
    MB_FC_READ_COILS               = 1,	/*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT      = 2,	/*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS           = 3,	/*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      = 4,	/*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL               = 5,	/*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER           = 6,	/*!< FCT=6 -> write single register */
    MB_FC_WRITE_MULTIPLE_COILS     = 15,	/*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16	/*!< FCT=16 -> write multiple registers */
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
    ERR_MALFORMED_MESSAGE = -9  //!< poll() methods.
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
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};

#if !defined(T35)
#define T35  5
#endif

#if !defined(MAX_BUFFER)
#define  MAX_BUFFER  64	//!< maximum size for the communication buffer in bytes
#endif


} // end namespace modbus

#endif // MODBUS_MODBUS_COMMON_H
