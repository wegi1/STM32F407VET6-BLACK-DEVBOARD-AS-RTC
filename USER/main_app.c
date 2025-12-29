/*
 * main_app.c
 * 1024|512 v dots with a HUUUGE... ZOOM
 *  Created on: 3 mar 2022
 *      Author: BOLO
 */
//====================

#include"table_data.h"
#include <inttypes.h>
#include "ili9341.h"
#include "demos.h"
#include "main.h"
#include "stdlib.h"
#include "multable_data.h"
#include "x_data_valid.h"
#include "y_data_valid.h"
//======================================
#define vector_dots 512
//**************************************
//***************************************
//****************************************
//======================================
#if vector_dots == 512
    #define many_circles 8
#else
    #define many_circles 16
#endif
#define many_plots many_circles*2*32
//======================================


#define screen_X 320
#define screen_Y 240
#define zooming ((uint16_t)0x1000)
#define frames_count 1200

#define x_min (2048 - screen_X/2)
#define x_max (2048 + screen_X/2)

#define y_min (2048 - screen_Y/2)
#define y_max (2048 + screen_Y/2)

#define x_offset (2048 - screen_X/2)
#define y_offset (2048 - screen_Y/2)
//======================================
void insert_dots(void);
void calculate_matrix(void);
void calculate_8_dots(void);
void calc_all_dots_01(void);

extern void main_app(void);
extern uint8_t test_stop(void);
//======================================
int8_t mat_A, mat_B, mat_C, mat_D, mat_E, mat_F ;

int8_t tb_fact1[8]; // circle prescale factors 4 circles top and 4 mirroring to bottom
int8_t tb_fact2[8]; //

int8_t v_5tab[many_circles]; //
int8_t v_6tab[many_circles]; //
int8_t v_7tab[many_circles*2]; //

//uint8_t x_data_valid[4096]; // maximum size calculated dot from - 2048 to 2047
//uint8_t y_data_valid[4096]; // after translate to 2D max values are from 0 to 4095

//uint8_t multable[128][256]; // a prescale table

int8_t wynik_X[16]; // X poss first 8 plots
int8_t wynik_Y[16]; // Y poss first 8 plots
int8_t plot_1X[many_plots]; // first buffer many_plots plot X values
int8_t plot_1Y[many_plots]; // first buffer many_plots plot Y values

uint16_t plots01_X[many_plots];
uint16_t plots01_Y[many_plots];

uint16_t plots02_X[many_plots];
uint16_t plots02_Y[many_plots];

uint32_t pl01_calc = 1;
uint32_t pl02_calc = 1;

uint16_t zoom_scale = zooming;
uint8_t zoom_side = 0x01;

uint8_t zoom = 128 ;//128;
uint32_t frames =0;

//=============================================
extern void half_sec_delay(void);
//---------------------------------------------
void main_app(void)
{

    uint32_t i;
    plots02_X[0] = 0U;
    plots02_Y[0] = 0U;
    LCD_ClrScr(COLOR_565_BLACK);  // init screen as black
    insert_dots();


    while(1)
    {
        frames++;
        if(test_stop() != 0) {
        	LCD_ClrScr(COLOR_565_BLACK);
        	half_sec_delay();
        	frames = 3333 ;
        } // check the stop action
        // huge zoom manage
        //====================================================================
        if(frames > frames_count)
        {
            if ((zoom_scale < 0x4000)&&(zoom_side != 0))
            {
                zoom_scale += 0x0040;
                if (zoom_scale == 0x4000){zoom_side = 0;}
            }
            else
            {
                zoom_scale -= 0x0040;
                if (zoom_scale < 0x0040){zoom_side = 1;}
            }

            if(frames > 3000)
            {
            	LCD_ClrScr(COLOR_565_BLACK);  // init screen as black
                zoom_scale = zooming;
                zoom_side = 1;
                frames = 0;
                return;
            }
        }
        //====================================================================

        calculate_matrix();
        calculate_8_dots();
        calc_all_dots_01(); // store plots values in first buffer

        // clear old plots wait for line #screen_Y
        while(getscanline() != 285){};
        //while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));

        uint16_t x,y;

        for(i = 0; i < pl02_calc ; i++)
        {
            x = plots02_X[i];
            y = plots02_Y[i];

            LCD_REG = (ILI_CASET);
            LCD_RAM = (y>>8);
            LCD_RAM = (y);
            LCD_RAM = (y>>8);
            LCD_RAM = (y);
            LCD_REG = (ILI_PASET);
            LCD_RAM = (x>>8);
            LCD_RAM = (x);
            LCD_RAM = (x>>8);
            LCD_RAM = (x);
            LCD_REG = (ILI_RAMWR);
            LCD_RAM = (0);
        }
        // set new plots
        for(i = 0; i < pl01_calc ; i++)
        {

            x = plots01_X[i];
            y = plots01_Y[i];

            LCD_REG = (ILI_CASET);
            LCD_RAM = (y>>8);
            LCD_RAM = (y);
            LCD_RAM = (y>>8);
            LCD_RAM = (y);
            LCD_REG = (ILI_PASET);
            LCD_RAM = (x>>8);
            LCD_RAM = (x);
            LCD_RAM = (x>>8);
            LCD_RAM = (x);
            LCD_REG = (ILI_RAMWR);
            LCD_RAM = (0xFFFFU);
        }
        for(i=0 ; i<many_plots ; i++)
        {
            plots02_X[i] = plots01_X[i];
            plots02_Y[i] = plots01_Y[i];
        }
        pl02_calc = pl01_calc;
    }
}
  //=======================================================================
  void insert_dots()
  {
    volatile uint8_t i, i2,  t9, t10, sx, sy;
//    volatile uint8_t i3, dana ;
//    volatile uint16_t wynik, x;
    volatile uint8_t t_x;



    //*************************************************
    // make prescale/multiply table
//    dana = 0;
//    for(i=0 ; i<128 ; i++)
//    {
//      wynik = 0;
//      i3 = 0;
//      for(i2=0 ; i2<128 ; i2++) {
//        multable[i][i2] = (uint8_t) (((wynik) >> 8) & 0xFF);
//        wynik = wynik + dana;
//        i3--;
//        multable[i][i3] = (uint8_t)  (0 - multable[i][i2]);
//      }
//      dana = dana +2;
//    }
    //*************************************************
//    for(x=0 ; x<4096 ; x++)
//    {
//      x_data_valid[x] = 0;
//      y_data_valid[x] = 0;
//      if((x > x_min)&&(x < x_max)){x_data_valid[x] = 1;}
//      if((x > y_min)&&(x < y_max)){y_data_valid[x] = 2;}
//    }
    //*************************************************
    //*************************************************
    // calculate and place dots
    t_x = 0x44;
    i2 = 0;
    for(i=7 ; i!=0xff ; i--)
    {
      t_x = t_x - 0x08;
      sx = t_x & 0x3F;
      sy = 0;
      //;---------------------
      //; F = invsin(t9)-invsin(t10)
      //;---------------------
      t9  = sy-sx;
      t10 = sy+sx;
      tb_fact1[i] = (uint8_t) ((sinus[128+t9] - sinus[128+t10]) >> 8) & 0xFF;
      tb_fact2[i2] = tb_fact1[i];
      i2++;
    }
    //*************************************************
    // calc data next object #3
#if many_circles == 8
    t_x = 0x44;
#else
    t_x = 0x42;
#endif
    i2 = 0;
    for(i=many_circles-1 ; i!=0xff ; i--)
    {
#if many_circles == 8
    t_x = t_x - 0x08;
#else
    t_x = t_x - 0x04;
#endif
      sx = t_x & 0x3F;
      sy = 0;
      //;---------------------
      //; F = invsin(t9)-invsin(t10)
      //;---------------------
      t9  = sy-sx;
      t10 = sy+sx;

      v_5tab[i] = (uint8_t) ((sinus[128+t9] - sinus[128+t10]) >> 8) & 0xFF;
      v_6tab[i2] = v_5tab[i];
      i2++;
    }
  }
  //===========================================================================
  void calculate_matrix()
  {
    uint8_t t1, t2, t3, t4, t5, t6, t7, t8, t9, t10 ;

    static uint16_t delta_x = 0x0070;
    static uint16_t delta_y = 0x00c0;
    static uint16_t delta_z = 0x0060;

    static uint16_t angle_x = 0x00;
    static uint16_t angle_y = 0x00;
    static uint16_t angle_z = 0x00;

    static uint8_t sx;
    static uint8_t sy;
    static uint8_t sz;

    /*    update_angles   */
    angle_x = angle_x + delta_x;
    angle_y = angle_y + delta_y;
    angle_z = angle_z + delta_z;

    sx = (uint8_t)  ((angle_x)>>8) & 0xFF;
    sy = (uint8_t)  ((angle_y)>>8) & 0xFF;
    sz = (uint8_t)  ((angle_z)>>8) & 0xFF;

    t1 = sy-sz;
    t2 = sy+sz;
    t3 = sx+sz;
    t4 = sx-sz;
    t5 = sx+sy+sz;
    t6 = sx-sy+sz;
    t7 = sx+sy-sz;
    t8 = sy+sz-sx;
    t9  = sy-sx;
    t10 = sy+sx;

    /*
// cosinus(x)   =  sinus[x+64]
// invsinus(x)  =  sinus[x+128]
;--------------------
; A = cos(t1)+cos(t2)
;--------------------
; B = sin(t1)-sin(t2)
;--------------
; C = invsin(sy)*2
;----------------------------------------------------------
; D = sin(t3)-sin(t4) + (cos(t6)-cos(t5)+cos(t8)-cos(t7))/2
;----------------------------------------------------------
; E = cos(t3)+cos(t4) + (sin(t5)-sin(t6)-sin(t7)-sin(t8))/2
;----------------------------------------------------------
; F = invsin(t9)-invsin(t10)
;---------------------
     */
    mat_A = (uint8_t) ((sinus[64 + t1] + sinus[64 + t2]) >> 8) & 0xFF;
    mat_B = (uint8_t) ((sinus[t1] - sinus[t2]) >> 8) & 0xFF;
    mat_C = (uint8_t) ((sinus[128+sy] + sinus[128+sy]) >> 8) & 0xFF;
    mat_D = (uint8_t) ((sinus[t3] - sinus[t4] +  ((sinus[64+t6] - sinus[64+t5] + sinus[64+t8] - sinus[64+t7]) / 2)) >> 8) & 0xFF;
    mat_E = (uint8_t) ((sinus[64+t3] + sinus[64+t4] + ((sinus[t5] - sinus[t6] - sinus[t7] - sinus[t8]) / 2)) >> 8) & 0xFF;
    mat_F = (uint8_t) ((sinus[128 + t9] - sinus[128 + t10]) >> 8) & 0xFF;
    if (zoom < 128)
    {
      mat_C = multable[zoom][mat_C];
      mat_F = multable[zoom][mat_F];
    }
  }
  //====================================================================================================
  void calc_all_dots_01()
  {
    uint16_t row1, row2, i, i2, i3;
    uint8_t add_x , add_y ;
    int32_t data1;
    uint32_t helper_y, helper_x;
    int16_t plot_16X;
    int16_t plot_16Y;

    i2 = 0;
    // 4 times calculate and mirroring circles = 4*16*2 = 128 vectordots
    for(i3 = 0; i3 < many_circles ; i3++) {
      row1 = v_6tab[i3]; // prescale factor for X/Y offset data (add_x;add_y)

      add_x = multable[row1][mat_C]; // X offset  after prescale circle = possitoin X;Y 0;0
      add_y = multable[row1][mat_F]; // Y offset  after prescale circle

      row2 = v_5tab[i3]; // prescale factor for actual circle

      // calculate 32 X;Y values of plots in mirroring 2nd circles
      for(i = 0; i < 16 ; i++) {
        plot_1X[i2]   = ((multable[row2][wynik_X[i]] + add_x) );
        plot_1X[i2+1] = (plot_1X[i2] ^ 255);
        plot_1Y[i2]   = ((multable[row2][wynik_Y[i]] + add_y) );
        plot_1Y[i2+1] = (plot_1Y[i2] ^ 255);

        plot_1X[i2+2] = ((multable[row2][wynik_X[i]] - add_x) );
        plot_1X[i2+3] = (plot_1X[i2+2] ^ 255);
        plot_1Y[i2+2] = ((multable[row2][wynik_Y[i]] - add_y) );
        plot_1Y[i2+3] = (plot_1Y[i2+2] ^ 255);
        i2 = i2 + 4;
      }
    }
    pl01_calc = 0;
    for(i=0 ; i<many_plots ; i++)
    {
      plot_16X = plot_1X[i]*16;
      plot_16Y = plot_1Y[i]*16;
      data1 = plot_16X * zoom_scale;
      plot_16X = data1 >> 16;
      data1 = plot_16Y * zoom_scale;
      plot_16Y = data1 >> 16;
      plot_16X += 2048;
      plot_16Y += 2048;

      //validate pixel range
      if((x_data_valid[plot_16X] | y_data_valid[plot_16Y]) == 0x03)
      {
        helper_y = plot_16Y - y_offset;
        helper_x = plot_16X - x_offset;
        plots01_X[pl01_calc] = helper_x  ;
        plots01_Y[pl01_calc] = helper_y  ;
        pl01_calc++;
      }
    }
  }
  //==================================================================================
  void calculate_8_dots()
  {
    uint8_t row1, row2;
    uint8_t  v1, v2, v4, v5;
    uint8_t i, i2;

    // calculate first 8 dots to following prescale and mirroring
    i2 = 0;
    for(i = 0 ; i < 8 ; i++)
    {
      row1 = tb_fact1[i];
      row2 = tb_fact2[i];

      v5 = multable[row1][mat_E];
      v2 = multable[row1][mat_B];
      v4 = multable[row2][mat_D];
      v1 = multable[row2][mat_A];

      wynik_X[i2]   = v1 + v2;
      wynik_Y[i2]   = v4 + v5;
      wynik_X[i2+1] = v1 - v2;
      wynik_Y[i2+1] = v4 - v5;
      i2 = i2 +2;
    }
  }
  //===============================================================================


  //===============================================================================

