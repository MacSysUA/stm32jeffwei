#include "stm32f10x_lib.h"
	
	
#define	BLACK	0x0000
#define	BLUE	0x001F
#define	RED 	0xF800
#define	GREEN 	0x07E0
#define CYAN	0x07FF
#define MAGENTA 0xF81F
#define YELLOW	0xFFE0
#define WHITE	0xFFFF		



extern int	LCD_Init(void);
extern void     LCD_CtrlLinesConfig(void);
extern void     LCD_FSMCConfig(void);
extern void     LCD_Clear(u16 Color);




