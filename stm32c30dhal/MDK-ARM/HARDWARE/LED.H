#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"


/*--------Buzzer control pin--------*/
#define Buzzer PAout(8)
/*----------------------------------*/

/*--------LED control pin--------*/
#define LED PAout(12) 
/*----------------------------------*/
 
void Led_Flash(u16 time);
void Buzzer_Alarm(u16 count);
extern int Led_Count;
#endif
