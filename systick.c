/*
 * systick.c
 *
 *  Created on: Oct 22, 2013
 *      Author: B34443
 */

#include "systick.h"

extern volatile long long msTicks; 

void systick_init(uint32_t sysclk)
{
	/* Set the core clk as clk source and pend the SysTick interrupt */
	SYST_CSR |= SysTick_CSR_CLKSOURCE_MASK | SysTick_CSR_TICKINT_MASK;

	/* set the reload value */
	SYST_RVR = sysclk;
	   
	/* Enable the SysTick */
	SYST_CSR |= SysTick_CSR_ENABLE_MASK;
}

void systick_delay_ms(uint32_t ms)
{
	long long bp;
	bp = ms + msTicks;
	while(msTicks < bp);
}
