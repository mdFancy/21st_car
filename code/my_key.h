#ifndef __MY_KEY_H__
#define __MY_KEY_H__

#include "zf_common_headfile.h"

#define CONFIG_KEY_DOWN_PIN  P18_3
#define CONFIG_KEY_LEFT_PIN  P18_1
#define CONFIG_KEY_RIGHT_PIN P18_4
#define CONFIG_KEY_CEN_PIN   P18_5
#define CONFIG_KEY_UP_PIN    P22_0
#define KeyDown  	1
#define KeyUp 		2
#define KeyLeft         3
#define KeyRight        4
#define KeyCen  	5
#define KeyNone  	0
void KEY_Init(void);

uint8 Key_Scan(void);

#endif


