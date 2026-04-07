
#include "zf_common_headfile.h"


void KEY_Init(void)              
{                                                                      
	gpio_init(CONFIG_KEY_CEN_PIN,GPI ,0 ,GPI_PULL_UP);           
	gpio_init(CONFIG_KEY_UP_PIN,GPI, 0,GPI_PULL_UP);       
	gpio_init(CONFIG_KEY_DOWN_PIN,GPI,0,GPI_PULL_UP);     
	gpio_init(CONFIG_KEY_LEFT_PIN,GPI ,0, GPI_PULL_UP);      
	gpio_init(CONFIG_KEY_RIGHT_PIN,GPI ,0,GPI_PULL_UP);
}


uint8 Key_Scan(void)               
{
	if(!gpio_get_level(CONFIG_KEY_DOWN_PIN)	)  
	{	
		system_delay_ms(10);
		while(!gpio_get_level(CONFIG_KEY_DOWN_PIN));
		system_delay_ms(10);
		return KeyDown;
		
	}
	if(!gpio_get_level(CONFIG_KEY_LEFT_PIN))	  
	{	
		system_delay_ms(10);
		while(!gpio_get_level(CONFIG_KEY_LEFT_PIN));
		system_delay_ms(10);
		return KeyLeft;
		
	}
	if(!gpio_get_level(CONFIG_KEY_RIGHT_PIN))	  
	{	
		system_delay_ms(10);
		while(!gpio_get_level(CONFIG_KEY_RIGHT_PIN));
		system_delay_ms(10);
		return KeyRight;
		
	}
	if(!gpio_get_level(CONFIG_KEY_UP_PIN))	  
	{	
		system_delay_ms(10);
		while(!gpio_get_level(CONFIG_KEY_UP_PIN));
		system_delay_ms(10);
		return KeyUp;
	}
	if(!gpio_get_level(CONFIG_KEY_CEN_PIN))	  
	{	
		system_delay_ms(10);
		while(!gpio_get_level(CONFIG_KEY_CEN_PIN));
		system_delay_ms(10);
		return KeyCen;
	}
	
	return KeyNone;
}
