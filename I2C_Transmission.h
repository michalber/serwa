#include "MKL25Z4.h"

#define I2C_READ 1 
#define I2C_WRITE 0

void I2C_Init(void);
void I2C_WriteByte(uint8_t data);
uint8_t I2C_ReadByte(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Wait(void);
void I2C_RepeatedStart(void);
void I2C_SendAck(void);
uint8_t I2C_GetAck(void);
void I2C_SetTXmode(void);
void I2C_SetRXmode(void);
void I2C_SendNack(void);
void I2C_WaitForAck(void);
void I2C_Write(uint8_t data);
