 /*
								Spis portów
--------------------TPM0-------------------------------
PTA3-CH0 PTA4-CH1 PTA5-CH2 PTA0-CH5
PTC1-CH0 PTC2-CH1 PTC3-CH2 PTC4-CH3 PTC8-CH4 PTC9-CH5
PTD0-CH0 PTD1-CH1 PTD2-CH2 PTD3-CH3 PTD4-CH4 PTD5-CH5
PTE24-CH0 PTE25-CH1 PTE29-CH2 PTE30-CH3 PTE31-CH4
	
--------------------TPM1-------------------------------
PTA12-CH0 PTA13-CH1
PTB0-CH0 PTB1-CH1
PTE20-CH0 PTE21-CH1

--------------------TPM2-------------------------------
PTA1-CH0 PTA2-CH1
PTB2-CH0 PTB3-CH1 PTB18-CH0 PTB19-CH1
PTE22-CH0 PTE23-CH1
 */
#ifndef PWM_H_
#define PWM_H_
 
#include "MKL25Z4.h" 
#include "stdbool.h"
 
// TPM clock source select
// Selects the clock source for the TPM counter clock
#define TPM_CLK_DIS 	0 	// Clock disabled
#define TPM_PLLFLL 		1 	// MCGFLLCLK clock or MCGPLLCLK/2 clock
#define TPM_OSCERCLK 	2 	// OSCERCLK clock
#define TPM_MCGIRCLK 	3 	// MCGIRCLK clock
// counter is disabled
#define TPM_CNT_DIS 	0 
// counter increments on every LPTPM counter clock
#define TPM_CLK 		1
// counter increments on rising edge of LPTPM_EXTCLK synchronized to the LPTPM counter clock
#define TPM_EXT_CLK 	2 
 
#define PS_1 			0 // Prescale Factor Selection
#define PS_2 			1
#define PS_4 			2
#define PS_8 			3
#define PS_16 			4
#define PS_32 			5
#define PS_64 			6
#define PS_128 			7
 
#define TPM_OC_TOGGLE 	TPM_CnSC_MSA_MASK|TPM_CnSC_ELSA_MASK
#define TPM_OC_CLR 		TPM_CnSC_MSA_MASK|TPM_CnSC_ELSB_MASK
#define TPM_OC_SET 		TPM_CnSC_MSA_MASK|TPM_CnSC_ELSA_MASK|TPM_CnSC_ELSB_MASK
#define TPM_OC_OUTL 	TPM_CnSC_MSB_MASK|TPM_CnSC_MSA_MASK|TPM_CnSC_ELSB_MASK
#define TPM_OC_OUTH 	TPM_CnSC_MSB_MASK|TPM_CnSC_MSA_MASK|TPM_CnSC_ELSA_MASK
 
#define TPM_PWM_H 		TPM_CnSC_MSB_MASK|TPM_CnSC_ELSB_MASK
#define TPM_PWM_L 		TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK
 
#define EDGE_PWM 		0
#define CENTER_PWM 		1
 
bool pwm_tpm_Init(TPM_MemMapPtr tpm, uint16_t clk, uint16_t module, 
uint8_t clock_mode,uint8_t ps, bool counting_mode);

//void pwm_tpm_Ch_Init(TPM_MemMapPtr tpm, uint16_t channel, uint8_t mode);

bool pwm_tpm_Ch_Init(TPM_MemMapPtr tpm, uint16_t channel, uint8_t mode,
GPIO_MemMapPtr gpio,uint8_t pin);

void pwm_tpm_CnV(TPM_MemMapPtr TPMx, uint16_t channel, uint16_t value);
 
#endif /* PWM_H_ */