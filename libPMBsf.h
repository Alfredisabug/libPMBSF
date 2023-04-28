/*
 * libPMBsf.h
 *
 *  Created on: 2023¦~3¤ë24¤é
 *      Author: Alfred Wu
 */

#ifndef LIBRARIES_LIBPMBSF_LIBPMBSF_H_
#define LIBRARIES_LIBPMBSF_LIBPMBSF_H_

#include "stdint.h"
#include "stdbool.h"

#define PMBSF_MAJOR                 0
#define PMBSF_MINOR                 0
#define PMBSF_PATCH                 6
#define PMBSF_SONAME                0.0.6-dev

/**
 * @brief exception code
 */
#define PMBSF_HANDLER_EXCEP_NO_EXCEP            (0x00)

#define PMBSF_HANDLER_EXCEP_TIME_OUT            (0x01)

#define PMBSF_HANDLER_EXCEP_FRAME_CHECK_FAIL    (0x02)

#define PMBSF_HANDLER_EXCEP_SEND_NOT_COMPELET   (0x04)

#define PMBSF_HANDLER_EXCEP_HALT                (0x10)

#define PMBSF_HANDLER_EXCEP_BUFFER_FULL         (0x20)

#define PMBSF_HANDLER_EXCEP_PEC_ERROR           (0x40)

typedef uint16_t PMBSF_HADLER_EXCEPTION_t;

/**
 * @brief handler state machine enumeration
 */
typedef enum {
    PMBSF_STATE_IDLE,
    PMBSF_STATE_RECEIVE,
    PMBSF_STATE_FRAME_CHECK,
    PMBSF_STATE_FRAME_CHECK_OK,
    PMBSF_STATE_SPECIAL_CMD_IN_WATING,
    PMBSF_STATE_WAIT_TRANS,
    PMBSF_STATE_EXEC_CMD,
    PMBSF_STATE_ERROR_CHECK,
    PMBSF_STATE_HALT
} PMBSF_STATE_MACHINE_t;

/** @brief PMBsf data type */
typedef unsigned char   PMBSF_DATA_t;

/** @brief PMBsf data buffer type */
typedef struct {
    int             capacity;
    int             len;
    PMBSF_DATA_t    *data_ptr;
    PMBSF_DATA_t    *arr;
} PMBSF_DATA_BUFFER_t;

typedef bool                            (*PMBsf_I2C_get_SDA_data_fp)(PMBSF_DATA_t*);
typedef bool                            (*PMBsf_frame_check_fp)(const PMBSF_DATA_t* arr, const int len);
typedef bool                            (*PMBsf_I2C_PEC_check_fp)(const PMBSF_DATA_t* arr, const int len);
typedef void                            (*PMBsf_special_cmd_exec_fp)(const PMBSF_DATA_t* arr, const int len);
typedef bool                            (*PMBsf_I2C_put_data_to_SDA_fp)(const PMBSF_DATA_t data);
typedef bool                            (*PMBsf_frame_cmd_exec_fp)(const PMBSF_DATA_t* arr, const int len);
typedef bool                            (*PMBsf_error_check_fp)(const PMBSF_HADLER_EXCEPTION_t);

/** @brief PMBsf handler type */
typedef struct PMBSF_HANDLER_t          PMBSF_HANDLER_t;
struct PMBSF_HANDLER_t {
    /*!< handler last state */
    PMBSF_STATE_MACHINE_t               old_state;
    /*!< handler now state */
    PMBSF_STATE_MACHINE_t               now_state;

    /*!< handler exception */
    PMBSF_HADLER_EXCEPTION_t            exception;

    /*!< handler event register */
    uint16_t                            event;

    /*!< handler rx buffer */
    PMBSF_DATA_BUFFER_t                 *rx_buffer;
    /*!< handler tx buffer */
    PMBSF_DATA_BUFFER_t                 *tx_buffer;
    /*!< handler PMBus PEC used */
    bool                                PEC_used;

    // object function
    /*!< notify handler that MCU gets framer start(PMBus start) signal */
    void                                (*frame_start)(PMBSF_HANDLER_t*);
    /*!< notify handler that get special command signal */
    void                                (*is_special_cmd)(PMBSF_HANDLER_t*);
    /*!< notify handler that the special command execution end signal */
    void                                (*special_cmd_end)(PMBSF_HANDLER_t*);
    /*!< notify handler that MCU gets PMBus read bit signal */
    void                                (*get_query_sign)(PMBSF_HANDLER_t*);
    /*!< notify handler that MCU gets PMBus stop signal */
    void                                (*get_stop_sign)(PMBSF_HANDLER_t*);
    /*!< notify handler that reset handler signal */
    void                                (*reset)(PMBSF_HANDLER_t*);
    /*!< notify handler that I2C communication timeout */
    void                                (*time_out)(PMBSF_HANDLER_t*, bool);
    /*!< set handler whether used PMBus PEC */
    void                                (*set_use_pec)(PMBSF_HANDLER_t*, bool);
    /*!< Put data to handler tx buffer */
    bool                                (*put_transmit_buffer)(PMBSF_HANDLER_t*, const PMBSF_DATA_t);

    // point to user defined functions
    PMBsf_I2C_get_SDA_data_fp           get_SDA_data;
    PMBsf_frame_check_fp                 frame_check;
    PMBsf_I2C_PEC_check_fp              check_pec;
    PMBsf_special_cmd_exec_fp           special_cmd_exec;
    PMBsf_I2C_put_data_to_SDA_fp        put_data_to_SDA;
    PMBsf_frame_cmd_exec_fp             cmd_exec;
    PMBsf_error_check_fp                error_check;
};

// Functions

/* @brief Module initial data buffer */
void    PMBSF_DATA_BUFFER_init(PMBSF_DATA_BUFFER_t* buf, PMBSF_DATA_t* arr, int capacity);

/* @brief Module handler initialize */
void    PMBSF_handler_init(PMBSF_HANDLER_t*,
                           PMBsf_I2C_get_SDA_data_fp,
                           PMBsf_frame_check_fp,
                           PMBsf_I2C_PEC_check_fp,
                           PMBsf_special_cmd_exec_fp,
                           PMBsf_I2C_put_data_to_SDA_fp,
                           PMBsf_frame_cmd_exec_fp,
                           PMBsf_error_check_fp,
                           PMBSF_DATA_BUFFER_t* rxBuffer,
                           PMBSF_DATA_BUFFER_t* txBuffer,
                           bool);

/* @brief Module handler execution function */
void    PMBSF_handler_run(PMBSF_HANDLER_t*);

#endif /* LIBRARIES_LIBPMBSF_LIBPMBSF_H_ */
