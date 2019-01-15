
#ifndef I2C_H_
#define I2C_H_
#include "MKL25Z4.h"

void i2c_set_tx_mode(I2C_MemMapPtr p);
void i2c_set_rx_mode(I2C_MemMapPtr p);
void i2c_set_slave_mode(I2C_MemMapPtr p);
void i2c_set_master_mode(I2C_MemMapPtr p);
void i2c_give_nack(I2C_MemMapPtr p);
void i2c_give_ack(I2C_MemMapPtr p);
void i2c_repeated_start(I2C_MemMapPtr p);
void i2c_write_byte(I2C_MemMapPtr p, uint8_t data);
uint8_t i2c_read_byte(I2C_MemMapPtr p);
void i2c_start(I2C_MemMapPtr p);
void i2c_stop(I2C_MemMapPtr p);
void i2c_wait(I2C_MemMapPtr p);
_Bool i2c_get_ack(I2C_MemMapPtr p);
void hal_i2c0_init(I2C_MemMapPtr p);
void hal_i2c1_init(I2C_MemMapPtr p);
void hal_i2c0_deinit(I2C_MemMapPtr p);
void hal_i2c1_deinit(I2C_MemMapPtr p);
#define I2C_READ  1
#define I2C_WRITE 0

#endif /* I2C_H_ */
