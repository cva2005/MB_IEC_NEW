#pragma once

#include "stdbool.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief IEC transmission modes
 */
typedef enum
{
    IEC_MODE_101 = 0,
    IEC_MODE_104 = 1,
    IEC_SER = 0,
    IEC_TCP = 1,
} iec_comm_mode_t;

/*! \ingroup iec
 * \brief If register should be written or read.
 *
 * This value is passed to the callback functions which support either
 * reading or writing register values. Writing means that the application
 * registers should be updated and reading means that the modbus protocol
 * stack needs to know the current register values.
 *
 * \see iecs_reg_holding_cb(), iecs_reg_coils_cb(), iecs_reg_holding_cb() and
 *   iecs_reg_input_cb().
 */
typedef enum
{
    IEC_REG_READ = 0x0001,   /*!< Read register values and pass to protocol stack. */
    IEC_REG_WRITE = 0x0002,  /*!< Update register values. */
} iec_reg_mode_enum_t;

/*! \ingroup iec
 * \brief Event types used by all function in the protocol stack.
 */
typedef enum _iec_event_enum
{
    _EV_TRANS_START = 0x0001,                   /*!< Start of transaction. */
    _EV_READY = 0x0002,                         /*!< Startup finished. */
    _EV_FRAME_RECEIVED = 0x0004,                /*!< Frame received. */
    _EV_EXECUTE = 0x0008,                       /*!< Execute function. */
    _EV_FRAME_TRANSMIT = 0x0010,                /*!< Transmission started . */
    _EV_FRAME_SENT = 0x0020,                    /*!< Frame sent. */
    _EV_ERROR_PROCESS = 0x0040,                 /*!< Error process state. */
    _EV_MASTER_ERROR_RESPOND_TIMEOUT = 0x0080,  /*!< Request respond timeout. */
    _EV_MASTER_ERROR_RECEIVE_DATA = 0x0100,     /*!< Request receive data error. */
    _EV_MASTER_ERROR_EXECUTE_FUNCTION = 0x0200, /*!< Request execute function error. */
    _EV_MASTER_PROCESS_SUCCESS = 0x0400         /*!< Master error process. */
} iec_event_enum_t;

/*! \ingroup modbus
 * \brief Error event type
 */
typedef enum _iec_err_event_enum
{
    _EV_ERROR_INIT,             /*!< No error, initial state. */
    _EV_ERROR_RESPOND_TIMEOUT,  /*!< Slave respond timeout. */
    _EV_ERROR_RECEIVE_DATA,     /*!< Receive frame data error. */
    _EV_ERROR_EXECUTE_FUNCTION, /*!< Execute function error. */
    _EV_ERROR_OK                /*!< No error, processing completed. */
} iec_err_event_t;

typedef struct _iec_event_t {
    iec_event_enum_t event;      /*!< event itself. */
    uint64_t trans_id;          /*!< unique transaction id */
    uint16_t length;            /*!< length of data accociated with the event */ 
    void *pdata;                /*!< data accociated with the event */
    iec_err_event_t type;        /*!< error type accociated with the event */
    uint64_t post_ts;           /*!< timestamp of event posted */
    uint64_t get_ts;            /*!< timestamp of event receved */
} iec_event_t;

/*! \ingroup modbus
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum
{
    IEC_ENOERR,                  /*!< no error. */
    IEC_ENOREG,                  /*!< illegal register address. */
    IEC_EINVAL,                  /*!< illegal argument. */
    IEC_EPORTERR,                /*!< porting layer error. */
    IEC_ENORES,                  /*!< insufficient resources. */
    IEC_EIO,                     /*!< I/O error. */
    IEC_EILLSTATE,               /*!< protocol stack in illegal state. */
    IEC_ERECVDATA,               /*!< receive data error. */
    IEC_ETIMEDOUT,               /*!< timeout error occurred. */
    IEC_EILLFUNC,                /*!< illegal IEC function. */
    IEC_EBUSY,                   /*!< master is busy now. */
    IEC_ENOCONN                  /*!< peer is not connected. */
} iec_err_enum_t;

/*! \ingroup modbus
 *  \brief TimerMode is Master 3 kind of Timer modes.
 */
typedef enum
{
	IEC_TMODE_T35,                   /*!< Master receive frame T3.5 timeout. */
	IEC_TMODE_RESPOND_TIMEOUT,       /*!< Master wait respond for slave. */
	IEC_TMODE_CONVERT_DELAY          /*!< Master sent broadcast , then delay sometime.*/
} iec_timer_mode_enum_t;

#ifdef __cplusplus
}
#endif

