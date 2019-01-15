#ifndef MMA8451_H_
#define MMA8451_H_

#include "i2c.h"
#include "angle_cal.h"

#define MMA8451_I2C_ADDRESS (0x1d<<1)
#define I2C0_B  I2C0_BASE_PTR

extern int16_t resultx, resulty, resultz;

void hal_dev_mma8451_init(void); //prepare I2C port
uint8_t   hal_dev_mma8451_read_reg(uint8_t addr);
void hal_dev_mma8451_write_reg(uint8_t addr, uint8_t data);
uint16_t hal_dev_mma8451_test_reg_rw(void);
void hal_dev_mma8451_test_function(void);
void accel_init(void);
void accel_read(void);


#endif /* MMA8451_H_ */
