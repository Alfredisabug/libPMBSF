/*
 * libPMBsf.c
 *
 *  Created on: 2023¦~3¤ë24¤é
 *      Author: Alfred Wu
 */

#include "libPMBsf.h"

typedef enum {
    PMBsf_event_time_out                = 0x01,
    PMBsf_event_frame_start             = 0x02,
    PMBsf_event_get_query_sign          = 0x04,
    PMBsf_event_get_stop_sign           = 0x08,
    PMBsf_event_get_special_cmd         = 0x10,
    PMBsf_event_special_cmd_in_waiting  = 0x20,
    PMBsf_event_special_cmd_end         = 0x40,
    PMBsf_event_reset                   = 0x80,
} PMBus_handler_event;

// Variables

// functions
static __inline void change_state(PMBSF_HANDLER_t*, PMBSF_STATE_MACHINE_t);
static bool _put_transmit_buffer(PMBSF_HANDLER_t*, const PMBSF_DATA_t);
static void _frame_start(PMBSF_HANDLER_t*);
static void _get_special_cmd(PMBSF_HANDLER_t*);
static void _get_query_sign(PMBSF_HANDLER_t*);
static void _special_cmd_end(PMBSF_HANDLER_t*);
static void _get_stop_sign(PMBSF_HANDLER_t*);
static void _get_reset(PMBSF_HANDLER_t*);
static void _time_out(PMBSF_HANDLER_t*, bool);
static void _set_use_pec(PMBSF_HANDLER_t*, bool);

void
PMBSF_DATA_BUFFER_init(PMBSF_DATA_BUFFER_t* buf, PMBSF_DATA_t* arr, int capacity) {
    int i;
    buf->capacity = capacity;

    for (i = 0; i < capacity; i++) {
        *(arr + i) = 0;
    }

    buf->len = 0;
    buf->arr = arr;
    buf->data_ptr = buf->arr;
}

void
PMBSF_handler_init(PMBSF_HANDLER_t*             handler,
                   PMBsf_I2C_get_SDA_data_fp    get_func,
                   PMBsf_frame_check_fp         frame_check,
                   PMBsf_I2C_PEC_check_fp       check_func,
                   PMBsf_special_cmd_exec_fp    spec_exec_func,
                   PMBsf_I2C_put_data_to_SDA_fp trans_func,
                   PMBsf_frame_cmd_exec_fp      exec_func,
                   PMBsf_error_check_fp         error_check_func,
                   PMBSF_DATA_BUFFER_t*         rx_buffer,
                   PMBSF_DATA_BUFFER_t*         tx_buffer,
                   bool use_pec) {

    handler->old_state = PMBSF_STATE_IDLE;
    handler->now_state = PMBSF_STATE_IDLE;
    handler->exception = 0x00;

    handler->event = 0;

    handler->rx_buffer = rx_buffer;
    handler->tx_buffer = tx_buffer;
    handler->PEC_used = use_pec;

    // behavior
    handler->frame_start = _frame_start;
    handler->is_special_cmd = _get_special_cmd;
    handler->special_cmd_end = _special_cmd_end;
    handler->get_query_sign = _get_query_sign;
    handler->get_stop_sign = _get_stop_sign;
    handler->reset = _get_reset;
    handler->time_out = _time_out;
    handler->set_use_pec = _set_use_pec;
    handler->put_transmit_buffer = _put_transmit_buffer;

    handler->get_SDA_data = get_func;
    handler->frame_check = frame_check;
    handler->check_pec = check_func;
    handler->special_cmd_exec = spec_exec_func;
    handler->put_data_to_SDA = trans_func;
    handler->cmd_exec = exec_func;
    handler->error_check = error_check_func;
}

void
PMBSF_handler_run(PMBSF_HANDLER_t* self) {
    PMBSF_DATA_t temp_rx_data;
    switch (self->now_state) {
        case PMBSF_STATE_IDLE:
            if (self->event & PMBsf_event_frame_start) {
//                // reset stop sign
//                self->event = self->event & ~PMBsf_event_get_stop_sign;
                self->tx_buffer->len = 0;
                self->tx_buffer->data_ptr = self->tx_buffer->arr;
                change_state(self, PMBSF_STATE_RECEIVE);
            } else if (self->event & PMBsf_event_get_query_sign) {
                change_state(self, PMBSF_STATE_WAIT_TRANS);
            }
            break;
        case PMBSF_STATE_RECEIVE:
            if (self->get_SDA_data(&temp_rx_data)) {
                // check whether buffer is full
                if (self->rx_buffer->len >= (self->rx_buffer->capacity - 1)) {
                    self->exception |= PMBSF_HANDLER_EXCEP_BUFFER_FULL;
                } else {
                    self->rx_buffer->len++;
                    *(self->rx_buffer->data_ptr) = temp_rx_data;
                    self->rx_buffer->data_ptr++;
                }
            }

            // state change judgment
            if (self->exception) {
                // if any exception, goto error
                change_state(self, PMBSF_STATE_ERROR_CHECK);
            } else {
                // No error and (get query sign or stop sign)
                if ((self->event & PMBsf_event_get_query_sign) | (self->event & PMBsf_event_get_stop_sign)) {
                    change_state(self, PMBSF_STATE_FRAME_CHECK);
                }
            }
            break;
        case PMBSF_STATE_FRAME_CHECK:
            // check PEC
            if (self->PEC_used) {
                if (!self->check_pec(self->rx_buffer->arr, self->rx_buffer->len)) {
                    // PEC check fail
                    self->exception |= PMBSF_HANDLER_EXCEP_PEC_ERROR;
                }
            }
            // PEC if OK, check frame
            if (!self->exception) {
                if (!self->frame_check(self->rx_buffer->arr, self->rx_buffer->len)) {
                    // check fail: Not support command or not correct
                    self->exception |= PMBSF_HANDLER_EXCEP_FRAME_CHECK_FAIL;
                }
            }
            // state change judgment
            if (self->exception) {
                // if Any exception, wait until get stop sign
                if (self->event & PMBsf_event_get_stop_sign) {
                    change_state(self, PMBSF_STATE_ERROR_CHECK);
                }
            } else {
                // No exception
                change_state(self, PMBSF_STATE_FRAME_CHECK_OK);
            }
            break;
        case PMBSF_STATE_FRAME_CHECK_OK:
            // is special command
            if (self->event & PMBsf_event_get_special_cmd) {
                self->special_cmd_exec(self->rx_buffer->arr,
                                       self->rx_buffer->len);
                self->event |= PMBsf_event_special_cmd_in_waiting;
            }

            if (self->event & PMBsf_event_special_cmd_in_waiting) {
                change_state(self, PMBSF_STATE_SPECIAL_CMD_IN_WATING);
            } else {
                if (self->event & PMBsf_event_get_query_sign) {
                    // transmit operation
                    change_state(self, PMBSF_STATE_WAIT_TRANS);
                } else {
                    // if get stop signal
                    change_state(self, PMBSF_STATE_EXEC_CMD);
                }
            }
            break;
        case PMBSF_STATE_SPECIAL_CMD_IN_WATING:
            if (self->event & PMBsf_event_special_cmd_end) {
                if (self->event & PMBsf_event_get_query_sign) {
                    // transmit operation
                    change_state(self, PMBSF_STATE_WAIT_TRANS);
                } else {
                    // if get stop signal
                    change_state(self, PMBSF_STATE_EXEC_CMD);
                }
            }

            if (self->exception) {
                change_state(self, PMBSF_STATE_ERROR_CHECK);
            }
            break;
        case PMBSF_STATE_EXEC_CMD:
            self->cmd_exec(self->rx_buffer->arr,
                           self->rx_buffer->len);
            // go to check error
            change_state(self, PMBSF_STATE_ERROR_CHECK);
            break;
        case PMBSF_STATE_WAIT_TRANS:
            if (self->tx_buffer->len > 0) {
                while (self->put_data_to_SDA(*(self->tx_buffer->data_ptr)) && (self->tx_buffer->len > 0)) {
                    self->tx_buffer->data_ptr++;
                    self->tx_buffer->len--;
                }
            }

            // goto PMBSF_STATE_ERROR_CHECK if any exception or get stop sign
            // if data not send complete, raise exception
            if (self->exception | (self->event & PMBsf_event_get_stop_sign)) {
                if (self->tx_buffer->len > 0) {
                    self->exception |= PMBSF_HANDLER_EXCEP_SEND_NOT_COMPELET;
                }
                change_state(self, PMBSF_STATE_ERROR_CHECK);
            }
            break;
        case PMBSF_STATE_ERROR_CHECK:
            if (self->error_check(self->exception)) {
                self->exception = PMBSF_HANDLER_EXCEP_NO_EXCEP;
                self->event = 0;
                self->rx_buffer->len = 0;
                self->rx_buffer->data_ptr = self->rx_buffer->arr;
                change_state(self, PMBSF_STATE_IDLE);
            }
            break;
        case PMBSF_STATE_HALT:
            if (self->event == PMBsf_event_reset) {
                change_state(self, PMBSF_STATE_IDLE);
            } else if (self->event == PMBsf_event_get_stop_sign) {
                change_state(self, PMBSF_STATE_ERROR_CHECK);
            }
            break;
        default:
            self->exception = PMBSF_HANDLER_EXCEP_HALT;
            break;
    }
}

__inline void change_state(PMBSF_HANDLER_t* self, PMBSF_STATE_MACHINE_t t) {
    self->old_state = self->now_state;
    self->now_state = t;
}

bool
_put_transmit_buffer(PMBSF_HANDLER_t* self, const PMBSF_DATA_t d) {
    if (self->tx_buffer->len == (self->tx_buffer->capacity -1)) {
        return 0;
    }
    self->tx_buffer->arr[self->tx_buffer->len] = (d & 0xFF);
    self->tx_buffer->len++;
    return 1;
}

void
_frame_start(PMBSF_HANDLER_t* self) {
    self->event |= PMBsf_event_frame_start;
}

void
_get_special_cmd(PMBSF_HANDLER_t* self) {
    self->event |= PMBsf_event_get_special_cmd;
}

void
_get_query_sign(PMBSF_HANDLER_t* self) {
    self->event |= PMBsf_event_get_query_sign;
}

void
_special_cmd_end(PMBSF_HANDLER_t* self) {
    self->event |= PMBsf_event_special_cmd_end;
}

void
_get_stop_sign(PMBSF_HANDLER_t* self) {
    if (self->now_state == PMBSF_STATE_IDLE) {
        return;
    }
    self->event |= PMBsf_event_get_stop_sign;
}

void
_get_reset(PMBSF_HANDLER_t* self) {
    self->event |= PMBsf_event_reset;
}

void
_time_out(PMBSF_HANDLER_t* self, bool s) {
    // if is not in SCL period
//    if (!(self->now_state == PMBSF_STATE_RECEIVE
//        | self->now_state == PMBSF_STATE_WAIT_TRANS
//        | self->now_state == PMBSF_STATE_SPECIAL_CMD_IN_WATING)) {
//        return;
//    }

    if (s) {
        self->event |= PMBsf_event_time_out;
        self->exception |= PMBSF_HANDLER_EXCEP_TIME_OUT;
    }
}

void
_set_use_pec(PMBSF_HANDLER_t* self, bool used) {
    self->PEC_used = used;
}
