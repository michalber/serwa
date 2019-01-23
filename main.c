//#include "MKL25Z4.h"

///******************************************************************************
//    * Public Function prototypes
//    ******************************************************************************/
//    void PWM_Init(void);
//    void PWM_Set_Duty_Cycle(int8_t chan,int8_t dutycycle);
//    void PWM_Disable(int8_t disftmchan);
//    void PWM_Enable(int8_t enftmchan);

//    /******************************************************************************
//    * PWM_Init(void)
//    *
//    * Description: Sets up the Flex Timer channels as center aligned PWM outputs on
//    *              the Kinetis K02.
//    *
//    * Parameters: None
//    ******************************************************************************/
//		void PWM_Init(void){
//		}

//int main() {
//	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; 
//	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
//	
//	PORTD->PCR[5] |= PORT_PCR_MUX(4);
//	PORTE->PCR[29] |= PORT_PCR_MUX(3); //enabling clock to PORT D and E
//	
//	//enabling clock to TPM0
//	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
//	//load mod value to counter (doubt:why this specific value is given)
//	//TPM0->MOD = 60124;
//	TPM0->MOD = 4095; //Select up counting (CPWMS=0) ,divide by 128 prescaler (PS=111) and clock mode is given in such as way that TPM counter increments on every TPM counter clock (CMOD=01)
//	TPM0->SC |= (~TPM_SC_CPWMS_MASK | TPM_SC_CMOD(1) | TPM_SC_PS(7));
//	
//	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK;
//	
//	// Set channels 5 and 2 to edge-aligned high-true PWM. This is done by setting MSB:MSA as 10 and ELSB:ELSA as 10
//	TPM0->CONTROLS[5].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK;
//	TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

//	//Set control values for channel 5 and 2
//	TPM0->CONTROLS[5].CnV=0;
//	TPM0->CONTROLS[2].CnV=0;
//	TPM0->SC |= TPM_SC_CMOD(1);
//	
//	
//	
//	
//	return 0;
//}

#include "MKL25Z4.h"
#include "pwm.h"
#include "serwo.h"
#include "Accelometer.h"
#include "extra.h"
#include "lptmr.h"
#include "Accelometer.h"
#include "I2C_Transmission.h"
#include "mma8451.h"

/*
0 - servo test
1 - debug
2 - gimbal test
*/
#define MODE 4

int main()
{
	Serwo serwoX;	
	Serwo serwoY;	
	Serwo serwoZ;	
	initSerwo(&serwoX,0,TPM1_BASE_PTR, TPM1,0,GPIOB,0);
	initSerwo(&serwoY,0,TPM1_BASE_PTR, TPM1,1,GPIOB,1);
	initSerwo(&serwoZ,1,TPM0_BASE_PTR, TPM0,3,GPIOE,30);
	resetPosition(&serwoX);
	resetPosition(&serwoY);
	resetPosition(&serwoZ);
	time_delay_ms(10);	// 10ms delay
	
#if MODE == 2 || MODE == 3
	uart_Init((UART_MemMapPtr)UART0,3,115200);
#endif
	//I2C_Init();
	accel_init();
	

	/*
	time_delay_ms(10);
	accel_read();
	*/
	
	//Gyro_R_Read(results);
	time_delay_ms(5);
	//uint32_t pos0 = resultx;
	#if MODE == 2 || MODE == 3
		uart_String((UART_MemMapPtr)UART0, "Hello from KL25Z \r\n");
	#endif

	//setPosition(&serwoX,-90);
	//I2C_Init();
	
	while(1) {
#if MODE == 0		
		setPosition(&serwoX,0);
		setPosition(&serwoY,0);
		setPosition(&serwoZ,0);
		delay_mc(500);
		setPosition(&serwoX,-90);
		setPosition(&serwoY,-90);
		setPosition(&serwoZ,-90);
		delay_mc(500);
		setPosition(&serwoX,90);
		setPosition(&serwoY,90);
		setPosition(&serwoZ,90);
		delay_mc(500);
		resetPosition(&serwoX);	
		resetPosition(&serwoY);	
		resetPosition(&serwoZ);	
		delay_mc(500);
		rotateFor(&serwoX,45);
		rotateFor(&serwoY,45);
		rotateFor(&serwoZ,45);
		delay_mc(500);
		rotateFor(&serwoX,-90);
		rotateFor(&serwoY,-90);
		rotateFor(&serwoZ,-90);
		delay_mc(500);
#elif MODE == 1
// test I2C - send WHO AM I
		uart_String((UART_MemMapPtr)UART0, "Reading reg \n\r");
		uart_Put((UART_MemMapPtr)UART0, WHO_AM_I_MPU9250);
		uart_String((UART_MemMapPtr)UART0, "\n\r"); 
		uart_String((UART_MemMapPtr)UART0, "My name is: \n\r");
		data = hal_dev_mma8451_read_reg(WHO_AM_I_MPU9250);
		asm("nop");
		uart_Put((UART_MemMapPtr)UART0, data);
		uart_String((UART_MemMapPtr)UART0, "\n\r"); 
		asm("nop");
#elif MODE == 2
		accel_read();
		asm("nop");
		if(serwoX.currentPosition < -2) {
			rotateFor(&serwoX,1);	
			time_delay_ms(8);			
		}
		
#elif MODE == 3
// test read registers of MPU6500
	Gyro_R_Read();	
	
	uart_Put((UART_MemMapPtr)UART0, mpu_results[0]);
	uart_String((UART_MemMapPtr)UART0, "\n\r");
	uart_Put((UART_MemMapPtr)UART0, mpu_results[1]);
	uart_String((UART_MemMapPtr)UART0, "\n\r");
	uart_Put((UART_MemMapPtr)UART0, mpu_results[2]);
	uart_String((UART_MemMapPtr)UART0, "\n\r");
	uart_String((UART_MemMapPtr)UART0, "\n\r");
//	uart_Put((UART_MemMapPtr)UART0, results[1]);
//	uart_String((UART_MemMapPtr)UART0, "\n\r");
//	uart_Put((UART_MemMapPtr)UART0, results[2]);
//	uart_String((UART_MemMapPtr)UART0, "\n\r");
	time_delay_ms(200);
	
#elif MODE == 4
	Gyro_R_Read();	
	rotateFor(&serwoX,psi_x()/2);
#endif
	}
	return 0;
}
