#include "extra.h"												//Declarations

 /**
@brief Function making delay of value * 10000 clock cycles
@note  
@author Michal Berdzik
@version 1.0 2018-12-11
@param none
@retval none
*/
void delay_mc(uint32_t value){
	uint32_t delay, x;
	
	for(x=0; x < value; x++){
		for(delay=0; delay < 10000; delay++){};
	}
}
