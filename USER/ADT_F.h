/*
 * ADT_F.h
 *
 *  Created on: Sep 23, 2025
 *      Author: BOLO
 */

#ifndef ADT_F_H_
#define ADT_F_H_

extern const unsigned char digital_7_ttf[];
extern const unsigned char dum1_ttf[];

extern void make_fade(void);
extern void Big_TTF_Demo(void);
extern int render_biggestFont_portait(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height);
extern int lcd_text_boxed(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height);

extern int lcd_mono_text_boxed(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height);
extern int render_text_monospace(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height);
extern int lcd_render_text(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height, uint8_t fast, uint8_t boxed);

extern void draw_End(void);
//int lcd_fade_text(int pozx, int pozy, const char *text, const unsigned char *font_data, float pixel_height);

extern void Timer_Demo(void) ;
extern void lcd_set_text_color(uint16_t kolor);
extern void lcd_set_background_color(uint16_t kolor);
extern uint8_t test_stop(void);

extern void lcd_setup_picture(uint8_t pic_nr);
extern uint32_t readPicFromFlash(uint8_t pic_nr);

#endif /* ADT_F_H_ */
