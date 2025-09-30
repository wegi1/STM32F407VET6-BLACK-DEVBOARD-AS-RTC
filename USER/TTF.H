/*
 * TTF.H
 *
 *  Created on: Sep 1, 2025
 *      Author: BOLO
 */

#ifndef TTF_H_
#define TTF_H_

//================================================================================
#include "main.h"
#include "stdio.h"
#include "stb_truetype.h"
#include "string.h"
#include "fonty.h"
#include "dum.h"
#include "ili9341.h"
//#include "inttypes.h"
//================================================================================
const uint16_t RGB565_GRAYSCALE_32[] =
{ 0x0000, 0x0841, 0x1062, 0x18C3, 0x2104, 0x2945, 0x3166, 0x39C7, 0x4208, 0x4A49, 0x52AA, 0x5AAB, 0x630C, 0x6B4D, 0x738e, 0x7bcf, 0x8410, 0x8c51, 0x94b2, 0x9cf3, 0xa514, 0xAd55, 0xb596, 0xbdd7, 0xc638, 0xce79, 0xd6ba, 0xdedb, 0xe73c, 0xef5d, 0xf7be, 0xFFFF };

//================================================================================
stbtt_fontinfo font;  // łatwiejszy dostęp :P
uint16_t lcd_text_color = COLOR_565_WHITE;
uint16_t lcd_background_color = COLOR_565_BLACK;
char tekst[30];
uint16_t *bitmap2;  // tymczasowa bitmapa aka framebuffer xD
int efect_dx = 1;
uint8_t fade_effect = 0;

extern uint8_t LCD_WORK_ORIENTATION;
extern uint8_t LCD_NOT_WORK_ORIENTATION ;
extern uint8_t LCD_PORTRAIT_WORK_ORIENTATION ;
extern uint8_t LCD_PORTRAIT_NOT_WORK_ORIENTATION ;

extern lcdPropertiesTypeDef  lcdProperties;
//===========================================================================================
#define delay(x) HAL_Delay(x)

//=====================================================================================================================================

//===================================================================================
/**
 * @brief Fast draw bitmap rgb565
 * @param x
 * @param y
 * @param szerokosc
 * @param wysokosc
 * @param data
 */
extern void lcd_Draw16BitBitmap(uint16_t x, uint16_t y, uint16_t szerokosc, uint16_t wysokosc, const uint16_t *data)
{
	LCD_OpenWin(x, y, x + szerokosc - 1, y + wysokosc - 1);

	for (uint32_t t = 0; t < szerokosc * wysokosc; t++)
		LCD_WriteData(data[t]);
}
//===================================================================================

/**
 * @brief Kolor tekstu w formacie rgb565.
 * @param kolor
 */
extern void lcd_set_text_color(uint16_t kolor)
{
	lcd_text_color = kolor;
}
//===================================================================================

/**
 * @brief Kolor tła tekstu w formacie rgb565.
 * @param kolor
 */
extern void lcd_set_background_color(uint16_t kolor)
{
	lcd_background_color = kolor;
}
//===================================================================================


extern int render_biggestFont_portait(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height)
{
 	//lcd_Direction(ROTATE_0);
 	if (!stbtt_InitFont(&font, font_data, stbtt_GetFontOffsetForIndex(font_data, 0)))
 	{
 		return 0;  // nie uda�o sie zainicjowa� font�w :(
 	}

 	float scale = stbtt_ScaleForPixelHeight(&font, pixel_height);

 	int ascent, descent, line_gap;
 	stbtt_GetFontVMetrics(&font, &ascent, &descent, &line_gap);
 	int baseline = (int) (ascent * scale);

 	// Ustalamy staly rozmiar glifu (monospace)
 	int adv_width, lsb;
 	stbtt_GetCodepointHMetrics(&font, 'M', &adv_width, &lsb);  // szeroko�� najwi�kszego znaku
 	int mono_w = (int) (adv_width * scale);
 	int text_len = strlen(text);

 	int x0, y0, x1, y1;
 	stbtt_GetCodepointBitmapBox(&font, 'M', scale, scale, &x0, &y0, &x1, &y1);
 	int total_h = baseline + y1;


 	// Renderowanie znak�w jeden po drugim
 	int pen_x = 0;
 	for (int i = 0; i < text_len; i++)
 	{
 		int cp = text[i];
 		int x0, y0, x1, y1;
 		stbtt_GetCodepointBitmapBox(&font, cp, scale, scale, &x0, &y0, &x1, &y1);

 		int gw = x1 - x0;
 		int gh = y1 - y0;

 		u16 winx_start = (((240 + mono_w) >> 1)  - gw - ((240 - mono_w) >> 1)) ; // center monospace in LCD
 		LCD_OpenWin(winx_start, (baseline + y0), (winx_start+gw-1), (baseline + y0 + gh));
extern uint8_t idx[];
extern uint8_t idy[];
 		my_utoa(&idx[0], mono_w);
 		my_utoa(&idy[0], gw);

 		unsigned char *g_bitmap = stbtt_GetCodepointBitmap(&font, scale, scale, cp, &gw, &gh, 0, 0);


 		for (int row = 0; row < gh; row++)
 		{
 			for (int col = 0; col < gw; col++)
 			{
 				unsigned char val = g_bitmap[row * gw + col];

 				if (val > 0)
 				{
 					LCD_RAM = lcd_text_color;
 				} else {
 					LCD_RAM = lcd_background_color;
 				}
 			}
 		}
 		stbtt_FreeBitmap(g_bitmap, NULL);
 		pen_x += mono_w;
 	}
 	return total_h;
 }


//================================================================================
extern void Big_TTF_Demo(void)
{

	lcdSetOrientation(LCD_PORTRAIT_NOT_WORK_ORIENTATION);
	LCD_ClrScr(lcd_background_color);
	for (uint8_t t=1;t<10;t++)
	{
		sprintf(tekst, "%d", t);
//		lcd_ClearScreen(lcd_background_color);
		render_biggestFont_portait(0, 0, tekst, digital_7_ttf, 230);
		delay(500);
	}
	sprintf(tekst, "%d", 0);
	render_biggestFont_portait(0, 0, tekst, digital_7_ttf, 230);
	delay(500);

	lcdSetOrientation(LCD_WORK_ORIENTATION);
	LCD_ClrScr(lcd_background_color);

}
//===================================================================================

extern int render_text_monospace(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height)
{

	if (!stbtt_InitFont(&font, font_data, stbtt_GetFontOffsetForIndex(font_data, 0)))
	{
		return 0;
	}

	float scale = stbtt_ScaleForPixelHeight(&font, pixel_height);

	int ascent, descent, line_gap;
	stbtt_GetFontVMetrics(&font, &ascent, &descent, &line_gap);
	int baseline = (int) (ascent * scale);

	//  Ustalamy rozmiar glifu (monospace)
	int adv_width, lsb;
	stbtt_GetCodepointHMetrics(&font, 'M', &adv_width, &lsb);  //
	int mono_w = (int) (adv_width * scale);
	int text_len = strlen(text);

	int x0, y0, x1, y1;
	stbtt_GetCodepointBitmapBox(&font, 'M', scale, scale, &x0, &y0, &x1, &y1);
	int total_h = baseline + y1;

	//  Renderowanie znaeden po drugim
	int pen_x = 0;
	for (int i = 0; i < text_len; i++)
	{
		int cp = text[i];
		int x0, y0, x1, y1;
		stbtt_GetCodepointBitmapBox(&font, cp, scale, scale, &x0, &y0, &x1, &y1);

		int gw = x1 - x0;
		int gh = y1 - y0;
		unsigned char *g_bitmap = stbtt_GetCodepointBitmap(&font, scale, scale, cp, &gw, &gh, 0, 0);





		uint16_t center_x =  (pozx+pen_x);  // X center in monospace fields
		uint16_t center_x_end = (center_x+mono_w-1);

		uint16_t center_y = ((total_h - gh) >> 1) + pozy; // Y center in monospace fields

		uint16_t buforek[320];


		LCD_OpenWin( center_x, center_y, center_x_end , center_y+gh-1); // open window big as TTF fonts
		for (int row = 0; row < gh; row++)
		{
			for(uint16_t it = 0; it< (mono_w) ; it++) { buforek[it] = lcd_background_color  ; }
			for (int col = 0; col < gw; col++)
			{
				unsigned char val = g_bitmap[row * gw + col];
				int px = col + (mono_w - gw)  ;  // centrowanie w polu monospace
				if (val > 0) {	buforek[px] = lcd_text_color; }
			}
			for(uint16_t it = 0 ; it < mono_w ; it++) { LCD_RAM = buforek[it];}
		}
		stbtt_FreeBitmap(g_bitmap, NULL);
		pen_x += mono_w;
	}

	return total_h;
}
//===================================================================================

// Główna funkcja renderująca tekst

extern void lcd_text_bounds(stbtt_fontinfo *font, const char *text, float pixel_height, int *out_width, int *out_height)
{
	float scale = stbtt_ScaleForPixelHeight(font, pixel_height);

	int ascent, descent, line_gap;
	stbtt_GetFontVMetrics(font, &ascent, &descent, &line_gap);

	int baseline = (int) (ascent * scale);
	int min_x = 999999, min_y = 999999;
	int max_x = -999999, max_y = -999999;

	int x = 0;
	for (const char *p = text; *p; p++)
	{

		int glyph = stbtt_FindGlyphIndex(font, *p);

		int ax, lsb;
		stbtt_GetGlyphHMetrics(font, glyph, &ax, &lsb);

		int x0, y0, x1, y1;
		stbtt_GetGlyphBitmapBox(font, glyph, scale, scale, &x0, &y0, &x1, &y1);

		int gx0 = x + x0;
		int gy0 = baseline + y0;
		int gx1 = x + x1;
		int gy1 = baseline + y1;

		if (gx0 < min_x) min_x = gx0;
		if (gy0 < min_y) min_y = gy0;
		if (gx1 > max_x) max_x = gx1;
		if (gy1 > max_y) max_y = gy1;

		int kern = stbtt_GetGlyphKernAdvance(font, glyph, *(p + 1));
		x += (int) (ax * scale) + (int) (kern * scale);
	}

	*out_width = (max_x - min_x);
	*out_height = (max_y - min_y) + 3;
}
//===================================================================================

/**
 * @brief procedura rysująca
 * @param pozx
 * @param pozy
 * @param text
 * @param font_data
 * @param pixel_height
 * @param fast
 * @param boxed
 */
int lcd_render_text(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height, uint8_t fast, uint8_t boxed)
{

	int text_width, text_height;

	if (!stbtt_InitFont(&font, font_data, stbtt_GetFontOffsetForIndex(font_data, 0)))
	{
		return 0;  //
	}

	lcd_text_bounds(&font, text, pixel_height, &text_width, &text_height);
	float scale = stbtt_ScaleForPixelHeight(&font, pixel_height);

	int ascent, descent, line_gap;
	stbtt_GetFontVMetrics(&font, &ascent, &descent, &line_gap);

	int baseline = (int) (ascent * scale);
	int x = 0;



	for (const char *p = text; *p; p++)
	{
		int glyph = stbtt_FindGlyphIndex(&font, *p);

		int ax, lsb;
		stbtt_GetGlyphHMetrics(&font, glyph, &ax, &lsb);

		int x0, y0, x1, y1;
		stbtt_GetGlyphBitmapBox(&font, glyph, scale, scale, &x0, &y0, &x1, &y1);

		int width = x1 - x0;
		int height = y1 - y0;

		unsigned char *bitmap = stbtt_GetCodepointBitmap(&font, scale, scale, *p, &width, &height, &x0, &y0);

		LCD_OpenWin((pozx + x), (pozy+baseline + y0), (pozx + x+width-1), (pozy+baseline + y0 + height-1));

		for (int row = 0; row < height; row++)
		{
			for (int col = 0; col < width; col++)
			{
				uint8_t value = bitmap[row * width + col];
				if (value > 0)
				{
					LCD_RAM = lcd_text_color;
				}
				else
				{
					LCD_RAM = lcd_background_color;
				}
			}
		}

		stbtt_FreeBitmap(bitmap, NULL);

		int kern = stbtt_GetGlyphKernAdvance(&font, glyph, *(p + 1));
		x += (int) (ax * scale) + (int) (kern * scale);
	}

	return text_height;
}

//===================================================================================

extern int lcd_text_boxed(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height)
{
	return pozy + lcd_render_text(pozx, pozy, text, font_data, pixel_height, 0, 0);
}
//===================================================================================

extern int lcd_mono_text_boxed(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height)
{
	return pozy + render_text_monospace(pozx, pozy, text, font_data, pixel_height);
}
//===================================================================================

extern int lcd_fade_text(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height)
{
	uint16_t backup = lcd_text_color;
	lcd_set_text_color(RGB565_GRAYSCALE_32[fade_effect]);
	uint32_t tmp = pozy + lcd_render_text(pozx, pozy, text, font_data, pixel_height, 0, 0);
	lcd_set_text_color(backup);
	fade_effect += efect_dx;
	if ((fade_effect == 30) || fade_effect == 0) efect_dx = -efect_dx;
	return tmp;
}


//===================================================================================
extern void draw_End(void){
	if(lcdProperties.orientation > 3 ) { lcdSetOrientation( LCD_ORIENTATION_LANDSCAPE ); }
	lcd_text_boxed(170, 203, "CLS", digital_7_ttf, 25);
	lcd_text_boxed(250, 203, "END", digital_7_ttf, 25);

}
//===================================================================================
//=====================================================================================
extern void make_fade(void){

	if(lcdProperties.orientation > 3 ) { lcdSetOrientation( LCD_ORIENTATION_LANDSCAPE ); }
	LCD_ClrScr(0);

	for(uint32_t fader = 0; fader < 120; fader ++){
		lcd_fade_text(10, 60, "FA:DE" , digital_7_ttf, 86);
	}
}

//=====================================================================================
//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================

//===================================================================================



#endif /* TTF_H_ */
