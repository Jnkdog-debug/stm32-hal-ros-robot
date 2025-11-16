#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#include "oled.h"
#include "system.h"


extern int Buzzer_count;  

void show_task(void);
void oled_show(void);
void APP_Show(void);
void OLED_ShowCheckConfirming(void);
void OLED_ShowChecking(void);
void OLED_ShowCheckResult(void);
void OLED_Show_CCD(void);
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t);
void oled_show_once(void);
#endif
