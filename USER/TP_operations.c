/*
 * TP_operations.c
 *
 *  Created on: Aug 27, 2025
 *      Author: BOLO
 */

#include "TP_operations.h"
#include "ili9341.h"
#include "stdbool.h"
#include "XPT2046_touch.h"

extern SPI_HandleTypeDef hspi2;

//==========================================
// Domyślnie są to dane touchtype=0.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;
u16 x01, y01;
//================================================
#define READ_TIMES 5    // ilość odczytów
#define LOST_VAL 1      // odrzucone wartości
#define ERR_RANGE 50 // Zakres tolerancji
//===================================================
extern POINT PIXEL_displaySample[];
extern POINT RAW_ADC_screenSample[];
extern MATRIX  matrix ;
extern uint8_t idx[];
extern uint8_t idy[];
extern int my_itoa(uint8_t * buf, uint32_t data);
extern int my_utoa(uint8_t * buf, uint32_t data);
extern void my_htoa32(uint8_t * buf, uint32_t data);
//===============================================================
void draw_cross(uint16_t x, uint16_t y, uint16_t color){
  uint16_t i;

  for(i = x-15; i<x+15; i++ ) { LCD_Put_Pixel(i, y, color); }
  for(i = y-15; i<y+15; i++ ) { LCD_Put_Pixel(x, i, color); }

}
//==========================================
void delay_us(u16 delay) {
	if(delay == 0) { return;}
	  TIM6->CR1 = 0;
	  TIM6->CNT = 0;
	  TIM6->CR1 = 1;
	  while(TIM6->CNT < delay){;}
	  TIM6->CR1 = 0;
}
//===============================================================
void draw_bigPixel(uint16_t x, uint16_t y, uint16_t color){
	LCD_Put_Pixel(x, y, color);
	LCD_Put_Pixel(x+1, y, color);
	LCD_Put_Pixel(x, y+1, color);
	LCD_Put_Pixel(x+1, y+1, color);
}
//--------------------------------------------------------------------------------------------------------
void lets_calibrate_ts(uint8_t orientation)
{
	u32 n;

	lcdSetOrientation(orientation);

	LCD_ClrScr(COLOR_565_BLACK);
	for(n=0; n<3; n++) {
		TCS = 1;

		draw_cross(PIXEL_displaySample[n].x , PIXEL_displaySample[n].y  , 0xffff); // display cross


		while(XPT2046_TouchPressed() != true){;} // wait for press touch screen
		HAL_Delay(1000); // debounce tactic delay

		while( XPT2046_GetFastCoordinates(&x01 , &y01) != true);

		RAW_ADC_screenSample[n].x= x01   ;
		RAW_ADC_screenSample[n].y= y01   ;


		draw_cross(PIXEL_displaySample[n].x , PIXEL_displaySample[n].y  , 0x0000); // clear cross
		while(XPT2046_TouchPressed() != false){;} // wait for releasse TS
		HAL_Delay(1000);

		my_utoa(&idx[0], RAW_ADC_screenSample[n].x);
		my_utoa(&idy[0], RAW_ADC_screenSample[n].y);

	}
	setCalibrationMatrix( &PIXEL_displaySample[0], &RAW_ADC_screenSample[0], &matrix ) ;
}
//**************************************************************************************
