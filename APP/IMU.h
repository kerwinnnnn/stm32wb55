#ifndef _IMU_H_
#define	_IMU_H_
#include"lsm6dsv16b_reg.h"

void lsm6dsv16b_fifo(void);
void IMU_init(void);
stmdev_ctx_t IMU_get_ctx(void);
void tx_com(uint8_t *tx_buffer, uint16_t len);

#endif
