


#define _SAMPLE_C_



/****************************************************/
/*                                                  */
/* Included files                                   */
/*                                                  */
/****************************************************/

#include "Calibrate.h"
#include "GUI.h"
#include "XPT2046_touch.h"
#include "ili9341.h"
#include "TP_operations.h"



/****************************************************/
/*                                                  */
/* Local Definitions                                */
/*                                                  */
/****************************************************/

extern void draw_End(void);

extern void my_htoa32(uint8_t * buf, uint32_t data);
extern int my_utoa(uint8_t * buf, uint32_t data);
extern void draw_bigPixel(uint16_t x, uint16_t y, uint16_t color);
extern uint8_t idx[];
extern uint8_t idy[];
extern uint8_t LCD_WORK_ORIENTATION;
extern uint8_t LCD_NOT_WORK_ORIENTATION ;
extern lcdPropertiesTypeDef  lcdProperties ;
/****************************************************/
/*                                                  */
/* Local Variables                                  */
/*                                                  */
/****************************************************/

        /* NOTE: Even though the validity of the calibration/translation method  */
        /*        proposed has been verified with empirical data from several    */
        /*        actual touch screen enabled displays, for the convenience of   */
        /*        this exercise, the raw and expected data used and presented    */
        /*        below are artificial.  When used with actual data the          */
        /*        functions presented yield results that may be off by a larger  */
        /*        but still small percentage (~1-3%) due to electrical noise and */
        /*        human error (i.e., the hand touching a screen target and       */
        /*        missing by a small amount.)                                    */


		/* The array of input points.  The first three are used for calibration. */
        /* These set of points assume that the touchscreen has vertical and      */
        /*  horizontal resolutions of 1024 pixels (10-bit digitizer.)            */ 

// ADC RAW VALUES
POINT RAW_ADC_screenSample[] =	{
        { 903, 873 },
		{ 516, 145 },
		{ 115, 506 },
} ;

		/* The array of expected "right answers."  The values selected assume a  */
        /*  vertical and horizontal display resolution of 240 pixels.            */

// SCREEN PIXEL VALUES
POINT PIXEL_displaySample[] =	{
		{  30,  30 },
		{ 160, 210 },
		{ 290, 120 }
} ;



//=========================
//= matrix members values =
//=========================
//===============================
// An           =      0x00022902
// Bn           =      0x0000071C
// Cn           =      0xF783853C
// Dn           =      0xFFFFFDE4
// En           =      0x0001A202
// Fn           =      0xF9B061B8
// Divider      =      0xFFF969ED
//===============================
MATRIX  matrix = {
		0x00022902,
		0x0000071C,
		0xF783853C,
		0xFFFFFDE4,
		0x0001A202,
		0xF9B061B8,
		0xFFF969ED
};
/****************************************************/
/*                                                  */
/* Forward Declaration of local functions           */
/*                                                  */
/****************************************************/

void do_calibrate(void) ;
void take_displayPoint(void);
void start_Paint(void);
void Paint_Init(void);
uint16_t Paint_Color = COLOR_565_WHITE;
POINT   Paint_Display ;
//===========================================
//===========================================
void Paint_Init(void){




	LCD_ClrScr(0);
	draw_End();


	LCD_DisARectangular(0,200,40,239, COLOR_565_GREEN); // COLOR_565_GREEN
	LCD_DisARectangular(40,200,80,239, COLOR_565_RED);
	LCD_DisARectangular(80,200,120,239, COLOR_565_BLUE);
	LCD_DisARectangular(120,200,160,239, COLOR_565_YELLOW);

	LCD_DisARectangular(160,200,319,202, COLOR_565_WHITE);
	LCD_DisARectangular(239,200,241,239, COLOR_565_WHITE);
}
//===========================================
extern void lets_calibrate_ts(uint8_t orientation);
//===========================================
void start_Paint(void){
uint8_t      make_cal = 0;
uint32_t     n_time, tst_time;
n_time = HAL_GetTick();
n_time += 500;
uint8_t bckp = lcdProperties.orientation;

uint8_t goahead = 0;

    if(bckp != LCD_ORIENTATION_LANDSCAPE_ROTATE) {bckp = LCD_ORIENTATION_LANDSCAPE;}
    lcdSetOrientation(bckp);

    LCD_ClrScr(0);
    draw_cross((lcdProperties.width >> 1), (lcdProperties.height >> 1), 0xffff);

	while(make_cal == 0) {

		if(HAL_GetTick() == n_time) { make_cal=1;}
		if(XPT2046_TouchPressed() == true) {
			LCD_ClrScr(0);
			HAL_Delay(20);
			while(XPT2046_TouchPressed() == true); // RELEASSE TS
			HAL_Delay(30);

			lets_calibrate_ts(LCD_ORIENTATION_LANDSCAPE_ROTATE);

			lcdSetOrientation(bckp);
			make_cal = 1;

		}
	}
	Paint_Init();

	while(1){
		n_time = HAL_GetTick();
		goahead=0;

		while(goahead == 0) {
			tst_time = HAL_GetTick();
			if((tst_time - n_time) > 2000) { // idle time overflow
				LCD_ClrScr(0);
				HAL_Delay(10);
				Paint_Color = COLOR_565_WHITE;
				return; // EXIT
			}
			if(XPT2046_TouchPressed() == true) { goahead = 1 ;}
		}

		take_displayPoint(); // GET COORDS PRESSED PIXEL
		if(Paint_Display.y > 200) { // Y CHOICE STRIP REACHED?
			if(Paint_Display.x > 241) { // REINIT, CLEAR SCREEN ETC...
				while(XPT2046_TouchPressed() == true); // RELEASSE TS
				LCD_ClrScr(0);
				HAL_Delay(10);
				Paint_Color = COLOR_565_WHITE;
				return; // EXIT
			}
			if(Paint_Display.x > 160) { // CLS KEY REGION REACHED ?
				Paint_Init();
				while(XPT2046_TouchPressed() == true); // RELEASSE TS
				HAL_Delay(10);
			} // TEST COLOR KEY
			if(Paint_Display.x < 160) { Paint_Color = COLOR_565_YELLOW; }
			if(Paint_Display.x < 120) { Paint_Color = COLOR_565_BLUE; }
			if(Paint_Display.x < 80) { Paint_Color = COLOR_565_RED; }
			if(Paint_Display.x < 40) { Paint_Color = COLOR_565_GREEN; }
		}
		if(Paint_Display.y < 200) { draw_bigPixel( (Paint_Display.x) , (Paint_Display.y)  , Paint_Color);}
	}// END while(1) LOOP
}
//===========================================

//=======================================================
void take_displayPoint(void){

	uint32_t dana;

	// ADC RAW VALUES
	POINT OneSample;

	while(XPT2046_TouchPressed() != true);
	while((XPT2046_GetFastCoordinates(&OneSample.x, &OneSample.y)) != true ){;}

	getDisplayPoint( &Paint_Display, &OneSample, &matrix ) ;
	if(LCD_WORK_ORIENTATION == LCD_ORIENTATION_LANDSCAPE) {
		Paint_Display.x = 319 - Paint_Display.x;
		Paint_Display.y = 239 - Paint_Display.y;
	}

	dana = Paint_Display.x;
    my_utoa(&idx[0], dana);
    dana = Paint_Display.y;
    my_utoa(&idy[0], dana);
}
/*************************************/




void do_calibrate(void)
{

        /*                 NOTE!    NOTE!    NOTE!                     */
        /*                                                             */
        /*  setCalibrationMatrix() and getDisplayPoint() will do fine  */
        /*  for you as they are, provided that your digitizer          */
        /*  resolution does not exceed 10 bits (1024 values).  Higher  */
        /*  resolutions may cause the integer operations to overflow   */
        /*  and return incorrect values.  If you wish to use these     */
        /*  functions with digitizer resolutions of 12 bits (4096      */
        /*  values) you will either have to a) use 64-bit signed       */
        /*  integer variables and math, or b) judiciously modify the   */
        /*  operations to scale results by a factor of 2 or even 4.    */
        /*                                                             */


    


        /* Call the function once more to obtain the calibration       */
        /*  factors you will use until you calibrate again.            */
    setCalibrationMatrix( &PIXEL_displaySample[0], &RAW_ADC_screenSample[0], &matrix ) ;

    my_htoa32(&idx[0], (uint32_t) &matrix);


        // example for getdisplaypoint - convert RAW ADC data to piexels
        //getDisplayPoint( &display, &screenSample[0], &matrix ) ;

} // end of do_calibrate()


