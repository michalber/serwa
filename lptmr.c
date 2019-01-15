/*! @addtogroup LPTMR Low-Power Timer
 *  @{
 *  @ingroup 	DEVICE_DRIVERS 
 *  @file    	lptmr.c
 *	@brief		LPTMR driver for Freescale Kinetis L - series devices
 * 
 * 
 * @}
 */

#include "lptmr.h"

/*! @brief Initialize the Low-Power Timer to provide a delay measured in ms.	
 *
 *@param[in] count_val	-Number of ms to delay
*/
void time_delay_ms(uint32_t count_val)
{
  /* Make sure the clock to the LPTMR is enabled */
  SIM_SCGC5|=SIM_SCGC5_LPTMR_MASK;

  /* Reset LPTMR settings */
  LPTMR0_CSR=0;

  /* Set the compare value to the number of ms to delay */
  LPTMR0_CMR = count_val;

  /* Set up LPTMR to use 1kHz LPO with no prescaler as its clock source */
  LPTMR0_PSR = LPTMR_PSR_PCS(1)|LPTMR_PSR_PBYP_MASK;

  /* Start the timer */
  LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;

  /* Wait for counter to reach compare value */
  while (!(LPTMR0_CSR & LPTMR_CSR_TCF_MASK));

  /* Disable counter and Clear Timer Compare Flag */
  LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;

  return;
}
/*! @brief Initialize the Low-Power Timer
 * 
 * @param count			-Number of pulse to count
 * @param clock_source	-Can be configured to select four different clock source.
 */
void lptmr_init(uint32_t count, uint8_t clock_source, _Bool mode)
{
	/* Make sure the clock to the LPTMR is enabled */
	SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;

    LPTMR0_PSR = ( LPTMR_PSR_PRESCALE(0) 			// 0000 is div 2
                 | LPTMR_PSR_PBYP_MASK  			// LPO feeds directly to LPT
                 | LPTMR_PSR_PCS(clock_source)) ; 	// use the choice of clock
    
    /* Set the compare value to the number of count*/
    LPTMR0_CMR = LPTMR_CMR_COMPARE(count);  

    LPTMR0_CSR =  LPTMR_CSR_TCF_MASK	// Clear any pending interrupt
                 | LPTMR_CSR_TPS(0)		//TMR pin select
                 |!LPTMR_CSR_TPP_MASK	//TMR Pin polarity
                 |!LPTMR_CSR_TFC_MASK	// Timer Free running counter is reset whenever TMR counter equals compare
                 |!LPTMR_CSR_TMS_MASK;	//LPTMR0 as Timer
    if(mode)
    	LPTMR0_CSR |= LPTMR_CSR_TIE_MASK; //Enable LPTMR Interrupt
                
    LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;   //Turn on LPT and start counting
}
