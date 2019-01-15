 /*----------------------------------------------------------------------------
 *      
 *----------------------------------------------------------------------------
 *      Name:    serwo.h
 *      Purpose: Control serwos'
 *----------------------------------------------------------------------------
 *      
 *---------------------------------------------------------------------------*/
#include "serwo.h"
 /**
@brief Function initializes Serwo.
@note  
@author Michal Berdzik
@version 1.0 2018-12-11
@param none
@retval none
*/
void initSerwo(Serwo *self, _Bool type, TPM_MemMapPtr tpm_base, TPM_MemMapPtr tpm_ch, uint16_t channel, GPIO_MemMapPtr gpio,uint8_t pin) {
	self->type = type;
	self->p_tpm=tpm_ch;
	self->p_channel=channel;
	self->p_gpio=gpio;
	self->p_pin=pin;
	
	pwm_tpm_Init(tpm_base, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, CENTER_PWM);
	pwm_tpm_Ch_Init(tpm_ch, channel, TPM_PWM_H,gpio,pin);
}
 /**
@brief Function to set Serwo's position
@note  
@author Michal Berdzik
@version 0.1 2018-12-27
@param Serwo *self - reference to Serwo's struct, signed angle - angle to go to (from -90 to 90 deg)
@retval none
*/
void setPosition(Serwo *self, signed int angle) {
	if(self->type == 0) {
		self->p_registerValue=1.638888889*angle+258.3333333;	//funkcja liniowa CnV(angle)
	}
	else if(self->type == 1) {
		self->p_registerValue=1.555555556*angle+230;	//funkcja liniowa CnV(angle)
	}
	asm("nop");
	self->currentPosition = angle;
	pwm_tpm_CnV(self->p_tpm, self->p_channel, self->p_registerValue);
} 
 /**
@brief Function to set Serwo's position
@note  
@author Michal Berdzik
@version 0.1 2018-12-27
@param Serwo *self - reference to Serwo's struct, signed angle - angle to go to (from -90 to 90 deg)
@retval none
*/
void rotateFor(Serwo *self, signed int angle) {
	if(self->type == 0) {
		self->p_registerValue=1.638888889*(self->currentPosition + angle)+258.3333333;	//funkcja liniowa CnV(angle)
	}
	else if(self->type == 1) {
		self->p_registerValue=1.555555556*(self->currentPosition + angle)+230;	//funkcja liniowa CnV(angle)
	}
	asm("nop");
	self->currentPosition += angle;
	pwm_tpm_CnV(self->p_tpm, self->p_channel, self->p_registerValue);
}
 /**
@brief Function resetting Serwo's position to 0.
@note  
@author Michal Berdzik
@version 1.0 2018-12-11
@param none
@retval none
*/
void resetPosition(Serwo *self) {
	if(self->type == 0) {
		pwm_tpm_CnV(self->p_tpm, self->p_channel, T0_DUTY_ZERO);
	}
	else if(self->type == 1) {
		pwm_tpm_CnV(self->p_tpm, self->p_channel, T1_DUTY_ZERO);
	}
	self->currentPosition = 0;
}
 /**
@brief Function that control movement of Serwo to set position
@note  
@author Michal Berdzik
@version 1.0 2018-12-11
@param none
@retval none
*/
void goToPosition(Serwo *self, signed angle, uint32_t time) {
	uint32_t x, dt;
	dt = time/(1.638888889*angle+258.3333333 - self->currentPosition);						
	x = self->currentPosition;
	while(x<1.6*angle+94) {																	
		pwm_tpm_CnV(self->p_tpm, self->p_channel, x);
		x += 1;
		delay_mc(dt);
	}
}

 /**
@brief Function that control movement of Serwo to set position
@note  
@author Michal Berdzik
@version 1.0 2018-12-11
@param none
@retval none
*/
void goToZero(Serwo *self, signed int angle) {
	rotateFor(self,1);
}

 /**
@brief Function that control movement of Serwo to set position
@note  
@author Michal Berdzik
@version 1.0 2018-12-11
@param none
@retval none
*/
signed int buffer = 0;
void rotateSerwoControl(Serwo *self, signed int angle) {                 
	if(__fabs(angle-buffer < 5)) {
		if((buffer - angle > 0)) {
			rotateFor(self,-2);
		}
		else if((buffer - angle < 0)) {
			rotateFor(self,2);
		}
	}
	else {
		setPosition(self,angle);	
	}
	buffer = angle;
}
