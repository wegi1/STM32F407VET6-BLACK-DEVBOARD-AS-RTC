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

#include "GUI.h"


extern void make_fade(void);
extern SPI_HandleTypeDef hspi2;
extern lcdPropertiesTypeDef  lcdProperties;
//==========================================
// DomyĹ›lnie sÄ… to dane touchtype=0.
u8 CMD_RDX=0XD0;
u8 CMD_RDY=0X90;
u16 x01, y01;
//================================================
#define READ_TIMES 5    // iloĹ›Ä‡ odczytĂłw
#define LOST_VAL 1      // odrzucone wartoĹ›ci
#define ERR_RANGE 50 // Zakres tolerancji



extern const unsigned char digital_7_ttf[] ;
extern const unsigned char dum1_ttf[];

extern int lcd_text_boxed(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height);
extern int lcd_mono_text_boxed(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height);


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
void live_show_ts(void);
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
//--------------------------------------------------------------------------------------------------------
void lets_calibrate_ts(uint8_t orientation)
{
	u32 n;
	uint8_t bckp = lcdProperties.orientation;
	lcdSetOrientation(orientation);

	LCD_ClrScr(COLOR_565_BLACK);
	for(n=0; n<3; n++) {
		TCS = 1;

		draw_cross(PIXEL_displaySample[n].x , PIXEL_displaySample[n].y  , 0xffff); // display cross


		while(XPT2046_TouchPressed() == false){;} // wait for press touch screen
		HAL_Delay(2000); // debounce tactic delay

		while( XPT2046_GetFastCoordinates(&x01 , &y01) != true);

		RAW_ADC_screenSample[n].x= x01   ;
		RAW_ADC_screenSample[n].y= y01   ;


		draw_cross(PIXEL_displaySample[n].x , PIXEL_displaySample[n].y  , 0x0000); // clear cross
		while(XPT2046_TouchPressed() == true){;} // wait for releasse TS
		HAL_Delay(1000);

		my_utoa(&idx[0], RAW_ADC_screenSample[n].x);
		my_utoa(&idy[0], RAW_ADC_screenSample[n].y);


uint16_t x_tab[3]  = { 20 , 140, 250};
uint16_t y_tab[3]  = { 20 , 170, 80};
uint16_t y_tab2[3] = { 50 , 200, 110};

        lcd_mono_text_boxed(x_tab[n], y_tab[n], (char*)&idx[0], digital_7_ttf, 20);
		lcd_mono_text_boxed(x_tab[n], y_tab2[n],(char*)&idy[0], digital_7_ttf, 20);

	}
	lcdSetOrientation(bckp);
	while(XPT2046_TouchPressed() == true){;} // wait for releasse TS
	HAL_Delay(22);
	while(XPT2046_TouchPressed() == false){;} // wait for press touch screen
	HAL_Delay(22);
	setCalibrationMatrix( &PIXEL_displaySample[0], &RAW_ADC_screenSample[0], &matrix ) ;
	LCD_ClrScr(COLOR_565_BLACK);


	my_htoa32(&idx[0], (uint32_t) &matrix.An);

	lcd_mono_text_boxed(35, 75,"MATRIX ADDRESS", digital_7_ttf, 20);
	lcd_mono_text_boxed(70, 115,(char*)&idx[0], digital_7_ttf, 20);

	while(XPT2046_TouchPressed() == true){;} // wait for releasse TS
	HAL_Delay(22);
	while(XPT2046_TouchPressed() == false){;} // wait for press touch screen
	HAL_Delay(22);
	LCD_ClrScr(COLOR_565_BLACK);

	lcd_mono_text_boxed(30, 5,"AN", digital_7_ttf, 20);
	lcd_mono_text_boxed(30, 35,"BN", digital_7_ttf, 20);
	lcd_mono_text_boxed(30, 65,"CN", digital_7_ttf, 20);
	lcd_mono_text_boxed(30, 95,"DN", digital_7_ttf, 20);
	lcd_mono_text_boxed(30, 125,"EN", digital_7_ttf, 20);
	lcd_mono_text_boxed(30, 155,"FN", digital_7_ttf, 20);
	lcd_mono_text_boxed(30, 200,"DIV", digital_7_ttf, 20);

	my_htoa32(&idx[0], matrix.An);
	lcd_mono_text_boxed(100, 5,(char*)&idx[0], digital_7_ttf, 20);
	my_htoa32(&idx[0], matrix.Bn);
	lcd_mono_text_boxed(100, 35,(char*)&idx[0], digital_7_ttf, 20);
	my_htoa32(&idx[0], matrix.Cn);
	lcd_mono_text_boxed(100, 65,(char*)&idx[0], digital_7_ttf, 20);
	my_htoa32(&idx[0], matrix.Dn);
	lcd_mono_text_boxed(100, 95,(char*)&idx[0], digital_7_ttf, 20);
	my_htoa32(&idx[0], matrix.En);
	lcd_mono_text_boxed(100, 125,(char*)&idx[0], digital_7_ttf, 20);
	my_htoa32(&idx[0], matrix.Fn);
	lcd_mono_text_boxed(100, 155,(char*)&idx[0], digital_7_ttf, 20);

	my_htoa32(&idx[0], matrix.Divider);
	lcd_mono_text_boxed(100, 200,(char*)&idx[0], digital_7_ttf, 20);

	while(XPT2046_TouchPressed() == true){;} // wait for releasse TS
	HAL_Delay(22);
	while(XPT2046_TouchPressed() == false){;} // wait for press touch screen
	HAL_Delay(22);
	while(XPT2046_TouchPressed() == true){;} // wait for releasse TS
	HAL_Delay(22);
	lcdSetOrientation(bckp);
	live_show_ts();
	lcdSetOrientation(bckp);
}
//**************************************************************************************

//===================================================================================
void live_show_ts(void)
{

	u16 x01, y01;
	POINT   COORDS ;
	POINT RAW_DATA ;

	TCS = 1;

	if(lcdProperties.orientation != LCD_ORIENTATION_LANDSCAPE_ROTATE) {
		lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);
	}
	LCD_ClrScr(COLOR_565_BLACK);


	lcd_mono_text_boxed(10, 60, "PRESS TS", digital_7_ttf, 40);


	while(XPT2046_TouchPressed() == false){;} // wait for press touch screen

	LCD_ClrScr(COLOR_565_BLACK);

	lcd_text_boxed(250, 203, "END", digital_7_ttf, 25);
	LCD_DisARectangular(239,200,319,202, COLOR_565_WHITE);
	LCD_DisARectangular(239,200,241,239, COLOR_565_WHITE);

	draw_cross(160, 120, 0xffff);

	draw_cross(290, 30, 0xffff);
	draw_cross(30, 210, 0xffff);
	draw_cross(160, 210, 0xffff);
	draw_cross(30, 120, 0xffff);
	draw_cross(290, 120, 0xffff);

	while(1) {

		while(XPT2046_TouchPressed() == false){;} // wait for press touch screen
		while( XPT2046_GetFastCoordinates(&x01 , &y01) != true);

		RAW_DATA.x= x01   ;
		RAW_DATA.y= y01   ;

		HAL_Delay(2);

		my_utoa(&idx[0], RAW_DATA.x);
		my_utoa(&idy[0], RAW_DATA.y);


		LCD_OpenWin(20, 20, 115, 80);
		for(uint32_t ix = 0; ix < 5700 ; ix++) { LCD_RAM = 0 ;}

        lcd_mono_text_boxed(20, 20, (char*)&idx[0], digital_7_ttf, 20);
		lcd_mono_text_boxed(20, 50, (char*)&idy[0], digital_7_ttf, 20);


		getDisplayPoint( &COORDS, &RAW_DATA, &matrix ) ;




		if(COORDS.x > lcdProperties.width) { COORDS.x = lcdProperties.width-1;}
		if(COORDS.y > lcdProperties.height) { COORDS.x = lcdProperties.height-1;}

		if(lcdProperties.orientation == LCD_ORIENTATION_LANDSCAPE) {
			COORDS.x = 320 - COORDS.x;
			COORDS.y = 240 - COORDS.y;
		}
		my_utoa(&idx[0], COORDS.x);
		my_utoa(&idy[0], COORDS.y);

		LCD_OpenWin(120, 20, 215, 80);
		for(uint32_t ix = 0; ix < 5700 ; ix++) { LCD_RAM = 0 ;}

        lcd_mono_text_boxed(120, 20, (char*)&idx[0], digital_7_ttf, 20);
		lcd_mono_text_boxed(120, 50, (char*)&idy[0], digital_7_ttf, 20);

		if(COORDS.y > 200) {
			if(COORDS.x > 240) {
				while(XPT2046_TouchPressed() == true ){;} // wait for releasse TS
				HAL_Delay(250);
				LCD_ClrScr(0);
				make_fade();
				return;
			}
		}

	}
}
//===================================================================================
