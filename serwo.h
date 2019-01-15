#ifndef SERWO_H_
#define SERWO_H_

#include "pwm.h"     
#include "extra.h"

#define TPM_MODULE 4095		// PWM period - 12bit
#define T0_DUTY_MIN 115				// PWM min pulse width
#define T0_DUTY_MAX 410			// PWM max pulse width
#define T0_DUTY_ZERO 250

#define T1_DUTY_MIN 90				// PWM min pulse width
#define T1_DUTY_MAX 370			// PWM max pulse width
#define T1_DUTY_ZERO 230

typedef struct Serwo {
		_Bool type;
    TPM_MemMapPtr p_tpm;
		uint16_t p_channel;
		GPIO_MemMapPtr p_gpio;
		uint8_t p_pin;
		uint16_t p_registerValue;
		uint16_t currentPosition;
} Serwo;

void initSerwo(Serwo *self, _Bool type, TPM_MemMapPtr tpm_base, TPM_MemMapPtr tpm_ch, uint16_t channel, GPIO_MemMapPtr gpio,uint8_t pin);     // constructor
void setPosition(Serwo *self, signed angle);
void rotateFor(Serwo *self, signed angle);
void resetPosition(Serwo *self); 
void goToPosition(Serwo *self, signed angle, uint32_t time); 
void goToZero(Serwo *self, signed int angle); 


#endif /* SERWO_H_ */