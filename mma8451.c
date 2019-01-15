#include "mma8451.h"

int16_t accel_x, accel_y, accel_z;
int16_t resultx, resulty, resultz;

void accel_init(void)
{
    unsigned char tmp;
    //printf("****Accelerometor init ****\n");
    //Configure MMA8451 sensor //MMA8451Q
    hal_dev_mma8451_init();      //Initialize I2C modules
    tmp = hal_dev_mma8451_read_reg(0x2a);
    hal_dev_mma8451_write_reg(0x2a,tmp|0x01);
}

void accel_read(void)
{
    if((hal_dev_mma8451_read_reg(0x00)&0xf) != 0)
    {
        accel_x   = hal_dev_mma8451_read_reg(0x01)<<8;
        accel_x  |= hal_dev_mma8451_read_reg(0x02);
        accel_x >>= 2;

        accel_y   = hal_dev_mma8451_read_reg(0x03)<<8;
        accel_y  |= hal_dev_mma8451_read_reg(0x04);
        accel_y >>= 2;

        accel_z   = hal_dev_mma8451_read_reg(0x05)<<8;
        accel_z  |= hal_dev_mma8451_read_reg(0x06);
        accel_z >>= 2;

        resultx   = hal_dev_mma8451_read_reg(0x01)<<8;
        resultx  |= hal_dev_mma8451_read_reg(0x02);
        resultx >>= 8;

        resulty   = hal_dev_mma8451_read_reg(0x03)<<8;
        resulty  |= hal_dev_mma8451_read_reg(0x04);
        resulty >>= 8;

        resultz   = hal_dev_mma8451_read_reg(0x05)<<8;
        resultz  |= hal_dev_mma8451_read_reg(0x06);
        resultz >>= 8;
        
        //angle_calculation(); //-900  to  900
        //detect_fall_detection();
    }
}

void hal_dev_mma8451_init(void)
{
  hal_i2c0_init(I2C0_B);
}
// this delay is very important, it may cause w-r operation failure.
static void pause(void)
{
    int n;
    for(n=0; n<40; n++)
        asm("nop");
}

uint8_t hal_dev_mma8451_read_reg(uint8_t addr)
{
    uint8_t result;

    i2c_start(I2C0_B);
    i2c_write_byte(I2C0_B, MMA8451_I2C_ADDRESS | I2C_WRITE);
    
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    i2c_write_byte(I2C0_B, addr);
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    i2c_repeated_start(I2C0_B);
    i2c_write_byte(I2C0_B, MMA8451_I2C_ADDRESS | I2C_READ);
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    i2c_set_rx_mode(I2C0_B);

    i2c_give_nack(I2C0_B);
    result = i2c_read_byte(I2C0_B);
    i2c_wait(I2C0_B);

    i2c_stop(I2C0_B);
    result = i2c_read_byte(I2C0_B);
    pause();
    return result;
}

void hal_dev_mma8451_write_reg(uint8_t addr, uint8_t data)
{
    i2c_start(I2C0_B);

    i2c_write_byte(I2C0_B, MMA8451_I2C_ADDRESS|I2C_WRITE);
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    i2c_write_byte(I2C0_B, addr);
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    i2c_write_byte(I2C0_B, data);
    i2c_wait(I2C0_B);
    i2c_get_ack(I2C0_B);

    i2c_stop(I2C0_B);
    pause();
}
