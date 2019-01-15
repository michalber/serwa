
#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "MKL25Z4.h"

void systick_init(uint32_t sysclk);
void systick_delay_ms(uint32_t ms);

#endif /* SYSTICK_H_ */
