/*
 * GUI.h
 *
 *  Created on: Aug 17, 2025
 *      Author: BOLO
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include "inttypes.h"
#include "types_h.h"
#include "fonts.h"

typedef struct
{
	uint32_t 	TextColor;
	uint32_t 	BackColor;
	sFONT*    	pFont;
	uint8_t		TextWrap;
}lcdFontPropTypeDef;

typedef struct
{
	unsigned short	x;
	unsigned short	y;
}lcdCursorPosTypeDef;





void LCD_DisALoop(u16 x0, u16 y0, u8 r, u16 Color);
void LCD_DisASquare(uint16_t x0, uint16_t y0, uint16_t wide, uint16_t Color);
void LCD_DisARectangular(u16 x0, u16 y0, u16 x1, u16 y1, u16 Color);

//uint16_t LCD_RandColor(void);
void LCD_DrawLine(uint16_t color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_No_Fill_Draw(u16 color, u16 x0, u16 y0, u16 x1, u16 y1);

void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length) ;

//**************
void lcdSetCursor(unsigned short x, unsigned short y);
void lcdPrintf(const char *fmt, ...);
void lcdDrawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg);
void lcdSetTextColor(uint16_t c, uint16_t b);
void lcdSetTextFont(sFONT* font);

u16  BSP_LCD_GetTextColor(void) ;
void BSP_LCD_FillRect(u16 x, u16 y, u16 width, u16 height) ;
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length) ;
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length) ;
void BSP_LCD_FillCircle(int16_t x0, int16_t y0, int16_t r);
void BSP_LCD_SetTextColor(u16 color);



//*********************
#endif /* INC_GUI_H_ */
