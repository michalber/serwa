/*! @addtogroup LPTMR Low-Power Timer
 *  @{
 *  @ingroup 	DEVICE_DRIVERS
 * 	@file		lptmr.h
 *  @brief 		LPTMR Header File.
 *  @}
 */

#ifndef __LPTMR_H__
#define __LPTMR_H__

#include "MKL25Z4.h"

/*!MCG Internal Reference Clock*/
#define LPTMR_USE_IRCLK 	0
/*!Low-Power Oscillator 1KHz*/
#define LPTMR_USE_LPOCLK 	1
/*!External Reference Clock can be LPO, RTC_CLKIN or OSC32KCLK*/
#define LPTMR_USE_ERCLK32 	2
/*!External Reference Clock*/ 
#define LPTMR_USE_OSCERCLK 	3

/*!Polling mode*/
#define POLLING		0
/*!interrupt mode*/
#define	INTERRUPT	1

void time_delay_ms(uint32_t count_val);
void lptmr_init(uint32_t count, uint8_t clock_source, _Bool mode);

/********************************************************************/

#endif /* __LPTMR_H__ */
