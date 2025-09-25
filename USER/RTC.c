/*
 * RTC.c
 *
 *  Created on: Sep 21, 2025
 *      Author: BOLO
 */

#include "RTC.h"

#include <time.h>
#include "demos.h"
#include "ili9341.h"
#include "w25qxx.h"
#include "TP_operations.h"
#include "XPT2046_touch.h"
#include "TTF.H"

//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================

extern uint8_t idx[13];
extern uint8_t idy[13];
extern lcdPropertiesTypeDef  lcdProperties;
extern uint16_t lcd_text_color ;
extern uint16_t lcd_background_color;
extern uint16_t Paint_Color;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
//extern
//extern
//extern
//extern
//extern
//extern
//extern
//extern
//extern
//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================


uint8_t change_colors = 0;
uint8_t run_slideshow = 0;
uint8_t mode_night = 0;
uint8_t max_day; // maximum day in month
int32_t SECONDS_TO_CALIBRATE = 0;
uint32_t HOURS_CALIBRATE_PERIOD = 1;
uint8_t  UPDATE_FLAG = 0;
int32_t century = 21;

RTC_TimeTypeDef my_time;
RTC_TimeTypeDef old_time;
RTC_DateTypeDef my_date;

RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp;
struct tm currTime;


POINT   Pressed_Point ;

uint8_t LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE;
uint8_t LCD_NOT_WORK_ORIENTATION ;
uint8_t LCD_PORTRAIT_WORK_ORIENTATION ;
uint8_t LCD_PORTRAIT_NOT_WORK_ORIENTATION ;
//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================

extern void main_app(void);
extern void do_calibrate(void);
extern void start_Paint(void);
extern MATRIX  matrix ;
extern void TIM4_IRQHandler(void);
//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================



//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================


//=====================================================================================================================

/**
 * @brief test key1 (PE3) or screen (PC5) pressed
 *
 * @return != 0 = PRESSED
 */
uint8_t test_stop(void) {
	uint8_t result = 0;
	if((KEY1_GPIO_Port->IDR & KEY1_Pin) == 0){ result = 1 ;} // key1 pressed?
	if((T_PEN_GPIO_Port->IDR & T_PEN_Pin) == 0){ result = 1 ;} // screen pressed ?
	if(result != 0){ HAL_Delay(20) ;}
	return result;
}
//=====================================================================================
void lcd_setup_picture(uint8_t pic_nr)
{
	uint8_t dana ;
	lcdOrientationTypeDef dana2;



	const lcdOrientationTypeDef setup_pic[] = {
			LCD_WORK_ORIENTATION,
			LCD_PORTRAIT_NOT_WORK_ORIENTATION,
			LCD_PORTRAIT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_WORK_ORIENTATION,
			LCD_WORK_ORIENTATION,
			LCD_WORK_ORIENTATION,
#define NOGITH
#ifdef GITH
			LCD_PORTRAIT_NOT_WORK_ORIENTATION,
			LCD_PORTRAIT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION
#else

			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION,
			LCD_NOT_WORK_ORIENTATION
#endif
	};


//---
extern	void check_work_orientation(void);
//---


    check_work_orientation();
	lcdSetOrientation(LCD_WORK_ORIENTATION);
	dana = pic_nr;
	if (dana > 12) {dana =0;}
	dana2 =  setup_pic[dana];
	lcdSetOrientation(dana2);

	LCD_OpenWin(0, 0, lcdProperties.width, lcdProperties.height);


}
//=====================================================================================================================
uint32_t readPicFromFlash(uint8_t pic_nr)
{
	uint8_t page_data[256];
	uint32_t i, i2, ADDR_PIC;
	uint16_t* DATA_PAGE  = (uint16_t*) &page_data[0];
	uint32_t command = 0x03;
	uint8_t * tmpr1 = (uint8_t*) &ADDR_PIC;
	uint8_t * tmpr2 = (uint8_t*) &command;

	lcd_setup_picture(pic_nr);
	ADDR_PIC = pic_nr * (600*256);

	i2 = 3;
    for(i=0; i<3; i++){
    	tmpr2[i2] = tmpr1[i];
    	i2--;
    }

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);

	HAL_SPI_Transmit(&hspi1,  tmpr2, 4, 5555);

	for(i = 0; i < 600; i++) {
		HAL_SPI_Receive(&hspi1, &page_data[0], 256, 5555);
		for(i2 = 0; i2 < 128; i2++){
			LCD_WriteData(DATA_PAGE[i2]);
		}
	}

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);

  return 0;
}


//=====================================================================================================================

/**
  * @brief  Returns the name of the day of the week.
  *         This function takes a number (1-7) and returns the corresponding
  *         day of the week as a string.
  * @param  number: The day of the week as a number (1 for Monday, 2 for Tuesday, etc.).
  * @retval char*: The name of the corresponding day of the week.
  */
char* getDayofweek(uint8_t number)
{
	if(number == 0) { number = 1;}
	if(number  > 7) { number = 7;}
	char *weekday[] = { "MONDAY", "TUESDAY", "WEDNESDAY","THURSDAY","FRIDAY","SATURDAY" , "SUNDAY"};
	return weekday[number-1];
}
//=====================================================================================================================
int calc_dow(int y, int m,int d) {
	int result;

	y-=m<3;
	result = (y+y/4-y/100+y/400+"-bed=pen+mad."[m]+d)%7;
	if(result == 0) {result = 7;}
	return result;
}
//=====================================================================================================================
void set_RTC_DATE(void){
	check_RTC_DATE();
	HAL_RTC_SetDate(&hrtc, &my_date, RTC_FORMAT_BIN);
}
void check_RTC_DATE(void){
	static const uint8_t month_maxdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31} ;
	uint8_t luty_add = 0;

	if(my_date.Year > 99) { my_date.Year = my_date.Year % 100;}
	if((my_date.Year %4) == 0) {luty_add =1;}
	if(my_date.Month == 0) {my_date.Month =1;}
	if(my_date.Month > 12) {my_date.Month =12;}
	max_day = month_maxdays[my_date.Month - 1];
	if(my_date.Month == 2) { max_day = max_day + luty_add;}
	if(my_date.Date == 0 ) {my_date.Date = 1;}
	if(my_date.Date > max_day) {my_date.Date = max_day;}
	my_date.WeekDay = calc_dow((int) (((century - 1) * 100) + my_date.Year)  , (int) my_date.Month , (int) my_date.Date );


}
//=====================================================================================================================

//=====================================================================================================================
void colon_print(void){
		TIM4->CR1 = TIM4->CR1 | 1;
		HAL_TIM_Base_Start_IT(&htim4);
		LCD_DisASquare(158, 36, 12, lcd_text_color);
		LCD_DisASquare(158, 78, 12, lcd_text_color);
}
//=====================================================================================================================
void read_RTC(void){
	  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	  HAL_RTC_WaitForSynchro(&hrtc);
	  HAL_RTC_GetTime(&hrtc, &my_time, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &my_date, RTC_FORMAT_BIN);
}
//=====================================================================================================================
void check_night_mode(void) {

#define nigh_value 20

	int hh;
	read_RTC();
	hh = my_time.Hours;

	if((mode_night &1) == 0) {
		if(hh >= nigh_value) {
			set_night_PWM();
			return;
		}
		if(hh < 6)  {
			set_night_PWM();
			return;
		}
	}
	set_PWM(); // out of night mode turn on the light as day mode

}
//=====================================================================================================================
void print_RTC(void) {

	lcdSetOrientation(LCD_WORK_ORIENTATION);
//	LCD_ClrScr(lcd_background_color);
	check_night_mode();
	int hh = my_time.Hours, mm = my_time.Minutes, ss = my_time.Seconds;



	set_color_RTC();

	old_time.Seconds = my_time.Seconds;
	sprintf(tekst, "%02d", hh);
	lcd_mono_text_boxed(2, 0, tekst, digital_7_ttf, 86);

	sprintf(tekst, "%02d", mm);
	lcd_mono_text_boxed(167, 0, tekst, digital_7_ttf, 86);
	sprintf(tekst, "%02d", ss);
	lcd_mono_text_boxed(0, 125, tekst, digital_7_ttf, 40);


	LCD_DisARectangular(200, 125, 319, 185, lcd_background_color); // clear for short name
	lcd_text_boxed(88, 140, getDayofweek(my_date.WeekDay), dum1_ttf, 42);

	hh = my_date.Year + ((century - 1) * 100);
	sprintf(tekst, "%04d-%02d-%02d", hh, my_date.Month, my_date.Date);
	if(century > -10) {
		lcd_mono_text_boxed(5, 185, tekst, digital_7_ttf, 36);
	} else {
		lcd_mono_text_boxed(5, 185, tekst, digital_7_ttf, 30);
	}

	colon_print();
}
//=====================================================================================================================
void set_color_RTC(void)
{
  lcd_text_color =  0xffff ;
  lcd_background_color = 0x0000;

  if ((change_colors & 0x01)!= 0) { return;}

  if(my_time.Minutes < 15) {
	  lcd_text_color = COLOR_565_GREEN;
	  return;
  }
  if(my_time.Minutes < 30) {
	  lcd_text_color = COLOR_565_RED;
	  return;
  }
  if(my_time.Minutes < 45) {
	  lcd_text_color = COLOR_565_MAGENTA;
	  return;
  }
}

//=====================================================================================================================
void test_print_RTC(void){
	int32_t tmpr1;
	uint8_t tmpr2, tmpr3;
	struct tm * czas_lokal;
	//---
	read_RTC();
	tmpr2 = my_time.Minutes;

	int ss = my_time.Seconds;
	set_color_RTC();
	if(my_time.Seconds != old_time.Seconds) {
		old_time.Seconds = my_time.Seconds;


		if(ss == 3) {
			if(UPDATE_FLAG == 0){ compare_EPOCH(); }
		}
		//TODO ADD TIME
		if(UPDATE_FLAG == 1) {
			if(my_time.Seconds == 4) {

				UPDATE_FLAG = 0; // clear update flag
				tmpr1 = SECONDS_TO_CALIBRATE; // get for compare and calculations seconds calibrate value

//------------------
				if(tmpr1 != 0) { // exist calibration seconds value != 0 ?

					read_timestamp();
					timestamp = timestamp + tmpr1;
					czas_lokal = gmtime(&timestamp);

					my_date.Year     =  czas_lokal->tm_year % 100 ;
					my_date.Date     =  czas_lokal->tm_mday ;
					my_date.Month    =  czas_lokal->tm_mon +1 ;
					my_date.WeekDay  =  czas_lokal->tm_mday ;

					my_time.Hours    =  czas_lokal->tm_hour ;
					my_time.Minutes  =  czas_lokal->tm_min ;
					my_time.Seconds  =  czas_lokal->tm_sec ;

					set_RTC_DATE();
					HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN); // store setup value
					ss = my_time.Seconds;
					print_RTC(); // immediatelly RTC print
//---
				}
			}
		}

		if(my_time.Seconds == 0) {
			print_RTC();
			return;
		}


		//TODO image
		sprintf(tekst, "%02d", ss);
		lcd_mono_text_boxed(0, 125, tekst, digital_7_ttf, 40);
		if(ss == 25) {
			if(run_slideshow == 0) {
				if(tmpr2 %5 == 0) {

					tmpr3 = tmpr2 / 5;
					TIM4_IRQHandler(); // stop colon blinking

					readPicFromFlash(tmpr3);
					HAL_Delay(10000);
					lcdSetOrientation(LCD_WORK_ORIENTATION);
					LCD_ClrScr(lcd_background_color);

					print_RTC();
					return;
				}
			}
		}
		colon_print();
	}
}
//=====================================================================================================================
//=====================================================================================================================




//=====================================================================================================================
bool setting_globals(void) {

	if((test_pressed_point(70, 319, 125, 185)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);

		while(1){

			lcd_text_boxed(0, 0, "GLOBAL FUNCTIONS", dum1_ttf, 36);

			lcd_text_boxed(25, 50, "PAINT", dum1_ttf, 42);
			lcd_text_boxed(175, 50, "BIGTTF", dum1_ttf, 42);
			lcd_text_boxed(25, 100, "LIGHT", dum1_ttf, 42);
			lcd_text_boxed(165,100, "COLORS", dum1_ttf, 42);
			lcd_text_boxed(22, 150, "AUTO", dum1_ttf, 42);
			lcd_text_boxed(170,150, "CALIBR.", dum1_ttf, 42);
			lcd_text_boxed(22,200, "EXIT", dum1_ttf, 42);
			lcd_text_boxed(190,200, "NEXT", dum1_ttf, 42);

			LCD_No_Fill_Draw(0xffff, 0, 45, 159, 95); // paint if(test_pressed_point(0, 159, 45, 95) == true)
			LCD_No_Fill_Draw(0xffff, 159, 45, 319, 95); // ttf demo if(test_pressed_point(159, 319, 45, 95) == true)
			LCD_No_Fill_Draw(0xffff, 0, 95, 159, 145); // light if((test_pressed_point(0, 159, 95, 145)) == true)
			LCD_No_Fill_Draw(0xffff, 159, 95, 319, 145); // colors if((test_pressed_point(159, 319, 95, 145)) == true)
			LCD_No_Fill_Draw(0xffff, 0, 145, 159, 195); // auto if((test_pressed_point(0, 159, 145, 195)) == true)

			LCD_No_Fill_Draw(0xffff, 159, 145, 319, 195); // TIM5 CALIBR  if((test_pressed_point(159, 319, 145, 195)) == true)

			LCD_No_Fill_Draw(0xffff, 0, 195, 159, 239);  // EXIT  if((test_pressed_point(0, 159, 195, 239)) == true)
			LCD_No_Fill_Draw(0xffff, 159, 195, 319, 239); // NEXT  if((test_pressed_point(159, 319, 195, 239)) == true)

			get_pressed_point();
// TODO GLOBALS
			TIM_CALIBRATE_SETUP();
			paint_test();
			TTF_test();
			set_LIGHT();
			COLOR_FONTS();
			night_mode();
			if( go_to_back() == true) { return true;}
			test_page();
		}
	}
	return false; // doesn't choose setting_globals
}
//=====================================================================================================================

//=====================================================================================================================
void next_page(void) {


	while(1){

		lcd_text_boxed(10, 0, "OTHER FUNCTIONS", dum1_ttf, 36);

		lcd_text_boxed(3, 50, "CAL_TS", dum1_ttf, 42);
		lcd_text_boxed(175, 50, "SLIDES", dum1_ttf, 42);

					lcd_text_boxed(10, 100, "DEMOS", dum1_ttf, 42);
					lcd_text_boxed(165,100, "V_BALL", dum1_ttf, 42);
					lcd_text_boxed(5, 155, "ROTATE", dum1_ttf, 38);
					lcd_text_boxed(162,154, "CENTURY", dum1_ttf, 37);
					lcd_text_boxed(22,200, "EXIT", dum1_ttf, 42);
					lcd_text_boxed(190,200, "PICS", dum1_ttf, 42);

		// TODO place where try obtains press parameter area
		LCD_No_Fill_Draw(0xffff, 0, 45, 159, 95); // CAL_TS if(test_pressed_point(0, 159, 45, 95) == true)
		LCD_No_Fill_Draw(0xffff, 159, 45, 319, 95); // SLIDES demo if(test_pressed_point(159, 319, 45, 95) == true)
		LCD_No_Fill_Draw(0xffff, 0, 95, 159, 145); // DEMOS if((test_pressed_point(0, 159, 95, 145)) == true)
		LCD_No_Fill_Draw(0xffff, 159, 95, 319, 145); // BALL  if((test_pressed_point(159, 319, 95, 145)) == true)
		LCD_No_Fill_Draw(0xffff, 0, 145, 159, 195); // ROTATION if((test_pressed_point(40, 280, 145, 195)) == true)

		LCD_No_Fill_Draw(0xffff, 159, 145, 319, 195); // CENTURY  if((test_pressed_point(159, 319, 145, 195)) == true)

		LCD_No_Fill_Draw(0xffff, 0, 195, 159, 239);  // EXIT  if((test_pressed_point(0, 159, 195, 239)) == true)
		LCD_No_Fill_Draw(0xffff, 159, 195, 319, 239); // NEXT  if((test_pressed_point(159, 319, 195, 239)) == true)


// TODO NEXT_PAGE
		get_pressed_point();
		CAL_TS();
		SLIDE_SHOW();
		DEMOS_RUN();
		V_BALL();
		ROTATION_TEST();
		slideshow_disable();
		SET_CENTURY();
		if( go_to_back() == true) { return ; }



	}
}
//=====================================================================================================================


//============================
// TODO LIST: ADD FUNC TO LOOP
// -
//============================
//=====================================================================================================================



//=====================================================================================================================
// TODO: ACTUAL FUNCTION MAKING
//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================
void SET_CENTURY(void) {

	uint8_t action ;
	read_RTC();
	READ_CENTURY();


	if((test_pressed_point(159, 319, 145, 195)) == true)  {

		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(55, 5, "SET CENTURY", dum1_ttf, 42);
//===================================================================================================
			lcd_mono_text_boxed(220, 50, "+", digital_7_ttf, 86);
			lcd_mono_text_boxed(0, 50, "-", digital_7_ttf, 86);
			sprintf(tekst, "%d", (int) century);
			lcd_text_color = 0;
			lcd_mono_text_boxed(80, 78, "000", digital_7_ttf, 50);
			lcd_text_color = 0xffff;

			if(century < 0) {
				lcd_mono_text_boxed(80, 78, tekst, digital_7_ttf, 50);
			} else {
				lcd_mono_text_boxed(123, 78, tekst, digital_7_ttf, 50);
			}
			lcd_text_boxed(115, 180, "DONE", dum1_ttf, 42);
			LCD_No_Fill_Draw(lcd_text_color, 80, 172, 240, 230);
			LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 165);
			LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 165);
//===================================================================================================
			check_RTC_DATE();
			action = test_action();
			if((action == 3)) { // DONE pressed

				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10,  (uint32_t) century);
				read_RTC();
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				HAL_Delay(100);
				return ;
			} else {
				if(action == 2) { // PLUS pressed
					if(century == 99) { century = -99;} else { century++;}
				} else { // MINUS pressed
					if(century == -99) { century = 99;} else { century--;}
				}
			}
		}
	}
}

//=====================================================================================================================

//=====================================================================================================================
void READ_CENTURY(void) {
	century = (int32_t) HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR10);
	if(century > 99) {
		century = 21;
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10,  (uint32_t) century);
	}
	if(century < -99) {
		century = 21;
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10,  (uint32_t) century);
	}
}
//=====================================================================================================================




//=====================================================================================================================
void compare_EPOCH() {
	uint64_t tmpr01;
	time_t pgm_timestamp;
	double elapsed;
	uint32_t hours, tmpr1, tmpr_hi, tmpr_lo;

	tmpr01 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR9); // read high 32 bits
	tmpr01 = tmpr01 << 32;
	tmpr01 = tmpr01 |  HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR8); // add low 32 bits

	pgm_timestamp = (time_t) tmpr01;
	read_timestamp(); // read RTC to timestamp
	elapsed = difftime(timestamp, pgm_timestamp); // = TIMESTAMP - PGM_TIMESTAMP

	// ELAPSED = ACTUAL TIME - PGM_TIMESTAMP
	if(elapsed > 0) {
		UPDATE_FLAG = 1;
		hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4); // iterval hours calibrate event

		tmpr1 = hours * 3600;


		timestamp = pgm_timestamp;
		timestamp = timestamp + tmpr1;

		tmpr_hi = (uint32_t) (timestamp >> 32);
		tmpr_lo = (uint32_t) (timestamp & 0xFFFFFFFF);

		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8,  tmpr_lo);  // LO timestamp
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9, tmpr_hi);  // HI timestamp

	}
}

//=====================================================================================================================
void set_EPOCH(void) { // setting the EPOCH date to calibrate seconds date
	uint32_t hours, tmpr1, tmpr_hi, tmpr_lo;

	hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4); // iterval hours calibrate event

	tmpr1 = hours * 3600;

	read_timestamp(); // read RTC to timestamp

	timestamp = timestamp + tmpr1;

	tmpr_hi = (uint32_t) (timestamp >> 32);
	tmpr_lo = (uint32_t) (timestamp & 0xFFFFFFFF);

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8,  tmpr_lo);  // LO timestamp
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9, tmpr_hi);  // HI timestamp

	UPDATE_FLAG = 0;
}
//=====================================================================================================================
void read_timestamp(void) {
	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);

	currTime.tm_year = currentDate.Year + ((century -20) * 100);  // In fact: 2000 + 18 - 1900
	currTime.tm_mday = currentDate.Date;
	currTime.tm_mon  = currentDate.Month - 1;

	currTime.tm_hour = currentTime.Hours;
	currTime.tm_min  = currentTime.Minutes;
	currTime.tm_sec  = currentTime.Seconds;

	timestamp = mktime(&currTime);
}
//=====================================================================================================================
void night_light(void){
	uint8_t action;
	uint32_t LIGHT_VAL = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7);
	if(LIGHT_VAL > 10) {LIGHT_VAL = 10;}

	set_night_PWM();
	LCD_ClrScr(lcd_background_color);

	while(1){
		lcd_text_boxed(85, 5, "SET NIGHT", dum1_ttf, 42);
		sprintf(tekst, "%02d", (int) LIGHT_VAL);
		lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
		put_DATE_number();
		action = test_action();
		set_night_PWM();

		Pressed_Point.x =0;
		Pressed_Point.y =0;
		if((action == 3)) { // DONE pressed
			wait_for_releasseTS(); // wait for releasse TS
			LCD_ClrScr(lcd_background_color);
			check_night_mode();
			return ;
		} else {
			if(action == 2) { // PLUS pressed
				if(LIGHT_VAL < 10) { LIGHT_VAL++;}
			} else { // MINUS pressed
				if(LIGHT_VAL >  0)  { LIGHT_VAL--;}
			}
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, LIGHT_VAL);
			set_night_PWM();
		}

	}




}

//=====================================================================================================================
void slideshow_disable(void){
	uint8_t s_mode, action;
	if((test_pressed_point(159, 319, 195, 239)) == true){
		LCD_ClrScr(lcd_background_color);
		check_slideshow(); // read slide setup
		s_mode = run_slideshow;

		while(1){

			s_mode = s_mode &0x01;

			lcd_text_boxed(25, 5, "SLIDE MODE", dum1_ttf, 42);
			sprintf(tekst, "YES");
			if(s_mode  != 0) { sprintf(tekst, "NO!"); }

			lcd_text_color = 0;
			lcd_mono_text_boxed(105, 85, "000", digital_7_ttf, 36);
			lcd_text_color = 0xffff;

			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				read_RTC();
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, (uint32_t) s_mode);
				run_slideshow = s_mode ; // read back data from backup register to variable

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				return ;
			} else {
				wait_for_releasseTS(); // wait for releasse TS
				if(action == 2) { // PLUS pressed
					s_mode++;
				} else { // MINUS pressed
					s_mode--;
				}
			}

		}
	}
}
//=====================================================================================================================

//=====================================================================================================================
void check_slideshow() {
	run_slideshow = (uint8_t) (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6) & 0x01 );
}
//=====================================================================================================================
void ROTATION_TEST(void) {
	uint8_t action;
	uint32_t R_MODE;



	if((test_pressed_point(0, 159, 145, 195)) == true)  {
		LCD_ClrScr(lcd_background_color);
		R_MODE= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
		if(R_MODE != 0) { R_MODE = 1;}


		while(1){



			R_MODE = R_MODE & 0x01;

			if(R_MODE != 0)
			{
				LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE ;
			} else {
				LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE;
			}
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, LCD_WORK_ORIENTATION);
			check_work_orientation();
			lcdSetOrientation(LCD_WORK_ORIENTATION);
			LCD_ClrScr(lcd_background_color);

			lcd_text_boxed(25, 5, "SET ROTATION", dum1_ttf, 42);
			sprintf(tekst, " 0 ");
			if(R_MODE != 0) { sprintf(tekst, "180"); }

			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);

				return ;
			} else {
				wait_for_releasseTS(); // wait for releasse TS
				if(action == 2) { // PLUS pressed
					R_MODE++;
				} else { // MINUS pressed
					R_MODE--;
				}
			}

		}
	}
}
//=====================================================================================================================
void check_work_orientation(void){
	LCD_WORK_ORIENTATION = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
	if((LCD_WORK_ORIENTATION &3) != 0) {
		LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE;
		LCD_NOT_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE ;
		LCD_PORTRAIT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT_ROTATE_MIRROR;
		LCD_PORTRAIT_NOT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT;
	} else {
		LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE;
		LCD_NOT_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE ;
		LCD_PORTRAIT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT;
		LCD_PORTRAIT_NOT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT_ROTATE_MIRROR;
	}
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, LCD_WORK_ORIENTATION);
}
//=====================================================================================================================
void V_BALL(void) {

	if((test_pressed_point(159, 319, 95, 145)) == true) {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 65535);

		lcdSetOrientation(LCD_ORIENTATION_PORTRAIT);
		main_app();

		set_PWM();
		lcdSetOrientation(LCD_WORK_ORIENTATION);
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================
void DEMOS_RUN(void) {

	if((test_pressed_point(0, 159, 95, 145)) == true) {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 65535);

		start_demos();
		set_PWM();
		lcdSetOrientation(LCD_WORK_ORIENTATION);
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================


//=====================================================================================================================
void SLIDE_SHOW(void) {
	uint8_t i, bckp;

	bckp = lcdProperties.orientation;
	if(test_pressed_point(159, 319, 45, 95) == true)  {
		wait_for_releasseTS(); // wait for releasse TS


		LCD_ClrScr(lcd_background_color);

		for( i=0; i<12; i++) {
			check_work_orientation();
			lcdSetOrientation(LCD_WORK_ORIENTATION);

			readPicFromFlash(i);
			get_pressed_point();
			wait_for_releasseTS(); // wait for releasse TS
		}
		lcdSetOrientation(bckp);

		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================

//=====================================================================================================================
void CAL_TS(void) {
	if(test_pressed_point(0, 159, 45, 95) == true)  {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 65535);
		lets_calibrate_ts(LCD_ORIENTATION_LANDSCAPE_ROTATE);
		LCD_ClrScr(lcd_background_color);
		Paint_Color = COLOR_565_WHITE;
		start_Paint();

		set_PWM();
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
	}
}
//=====================================================================================================================
void test_page(void) {
	if((test_pressed_point(159, 319, 195, 239)) == true)  {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);

		next_page();

		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
	}
}
//=====================================================================================================================
//=====================================================================================================================
void TIM_CALIBRATE_SETUP(void) {
	uint8_t action ;
	if((test_pressed_point(159, 319, 145, 195)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);


		while(1){
			lcd_text_boxed(0, 5, "SET TIM CALIBR.", dum1_ttf, 42);


			lcd_text_color = lcd_background_color;
			lcd_mono_text_boxed(130, 55, "000", digital_7_ttf, 36); // clear field by spaces
			lcd_mono_text_boxed(135, 180,"000", digital_7_ttf, 36);
			lcd_text_color = 0xFFFF;
			sprintf(tekst, "%d", ((int)SECONDS_TO_CALIBRATE));


			if(SECONDS_TO_CALIBRATE < 0) {
				lcd_mono_text_boxed(130, 55, tekst, digital_7_ttf, 36); // print seconds
			} else {
				lcd_mono_text_boxed(145, 55, tekst, digital_7_ttf, 36); // print seconds
			}



			sprintf(tekst, "%d", ((int)HOURS_CALIBRATE_PERIOD));
			lcd_mono_text_boxed(135, 180, tekst, digital_7_ttf, 36); // print seconds
//==================================================================
			lcd_mono_text_boxed(260, 55, "+", digital_7_ttf, 36);
			lcd_mono_text_boxed(20, 55, "-", digital_7_ttf, 36);
			lcd_mono_text_boxed(260, 180, "+", digital_7_ttf, 36);
			lcd_mono_text_boxed(20, 180, "-", digital_7_ttf, 36);

			lcd_text_boxed(85, 68, "SEC", dum1_ttf, 28);
			lcd_text_boxed(85, 180, "HRS", dum1_ttf, 28);

			lcd_text_boxed(110, 118, "DONE", dum1_ttf, 42);
			LCD_No_Fill_Draw(lcd_text_color, 83, 114, 230, 165); // DONE


			LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 110); // minus sec
			LCD_No_Fill_Draw(lcd_text_color, 0, 172, 80, 230); // minus hrs

			LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 110); // plus sec
			LCD_No_Fill_Draw(lcd_text_color, 235, 172, 318, 230); // plus hrs
//===================================================================

			action = calibr_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
//TODO EPOCH
			if((action == 5)) { // DONE pressed

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, HOURS_CALIBRATE_PERIOD);
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, (uint32_t) SECONDS_TO_CALIBRATE);
				set_EPOCH();

				return ;
			} else {
				if(action == 2) { // PLUS sec pressed
					if(SECONDS_TO_CALIBRATE < 99)  { SECONDS_TO_CALIBRATE++;} else { SECONDS_TO_CALIBRATE = -99;}
				}
				if (action == 1) { // MINUS sec pressed
					if(SECONDS_TO_CALIBRATE > -99)  { SECONDS_TO_CALIBRATE--;} else { SECONDS_TO_CALIBRATE = 99;}
				}
				if(action == 4) { // PLUS hrs pressed
					if(HOURS_CALIBRATE_PERIOD < 744)  { HOURS_CALIBRATE_PERIOD++;} else { HOURS_CALIBRATE_PERIOD = 1 ;}
				}
				if(action == 3) { // PLUS hrs pressed
					if(HOURS_CALIBRATE_PERIOD > 1)  { HOURS_CALIBRATE_PERIOD--;} else { HOURS_CALIBRATE_PERIOD = 744 ;}
				}
			}

		}

	}

}
//=====================================================================================================================
uint8_t calibr_action(void) {
	while(1) {
		get_pressed_point();
		// DONE test
		if((test_pressed_point(80, 240, 115, 165)) == true) { return 5;} // DONE

		if((test_pressed_point(0, 80, 55, 110)) == true) { return 1;} // minus sec
		if((test_pressed_point(0, 80, 172, 230)) == true) { return 3;} // minus hrs

		if((test_pressed_point(235, 318, 55, 110)) == true) { return 2;} // plus sec
		if((test_pressed_point(235, 318, 172, 230)) == true) { return 4;} // plus hrs

	}
}
//if((test_pressed_point(80, 240, 172, 230)) == true) { return 5;} // DONE
//
//if((test_pressed_point(0, 80, 55, 110)) == true) { return 1;} // minus sec
//if((test_pressed_point(0, 80, 115, 165)) == true) { return 3;} // minus hrs
//
//if((test_pressed_point(235, 318, 55, 110)) == true) { return 2;} // plus sec
//if((test_pressed_point(235, 318, 110, 165)) == true) { return 4;} // plus hrs
//=====================================================================================================================

//=====================================================================================================================
void CHECK_CALIBRATE_TIME_VALUES(void){
	uint32_t hours ;
	int32_t  seconds;
	hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4); // iterval hours calibrate event
	seconds = (int32_t) HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5); // seconds to add calibrate

	if(hours == 0)      { hours   = 1;   }
	if(hours > 744)     { hours   = 744; }
	if(seconds > 99)    { seconds = 0;   } // seconds to calibrate
	if(seconds < (-99)) { seconds = 0;   } // seconds to calibrate

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, (uint32_t) seconds);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, hours);
	HOURS_CALIBRATE_PERIOD = hours;
	SECONDS_TO_CALIBRATE   = seconds;
}
//=====================================================================================================================
bool go_to_back(void) {
	if((test_pressed_point(0, 159, 195, 239)) == true)  {
		wait_for_releasseTS(); // wait for releasse TS
		LCD_ClrScr(lcd_background_color);
		HAL_Delay(100);
		return true;
	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
void night_mode(void) {
	uint8_t action;
	uint32_t n_mode;



	if((test_pressed_point(0, 159, 145, 195)) == true)  {
		LCD_ClrScr(lcd_background_color);
		n_mode = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);

		while(1){

			n_mode = n_mode &0x01;

			lcd_text_color = 0;
			lcd_mono_text_boxed(105, 85, "000", digital_7_ttf, 36);
			lcd_text_color = 0xffff;

			lcd_text_boxed(25, 5, "NIGHT MODE", dum1_ttf, 42);
			sprintf(tekst, "YES");
			if(n_mode  != 0) { sprintf(tekst, "NO!"); }

			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				read_RTC();
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, n_mode);
				mode_night = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3); // read back data from backup register to variable

				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				if(mode_night == 0) {night_light();}
				check_night_mode();
				return ;
			} else {
				wait_for_releasseTS(); // wait for releasse TS
				if(action == 2) { // PLUS pressed
					n_mode++;
				} else { // MINUS pressed
					n_mode--;
				}
			}

		}

	}

}
//=====================================================================================================================

//=====================================================================================================================
void COLOR_FONTS(void) {
	uint8_t action;
	uint32_t font_VAL = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);


	if((test_pressed_point(159, 319, 95, 145)) == true)  {
		LCD_ClrScr(lcd_background_color);

		while(1){
			lcd_text_boxed(25, 5, "COLOR FONTS", dum1_ttf, 42);

			lcd_text_color = 0;
			lcd_mono_text_boxed(105, 85, "000", digital_7_ttf, 36);
			lcd_text_color = 0xffff;

			sprintf(tekst, "YES");
			if((font_VAL &1) != 0) { sprintf(tekst, "NO!"); }
			lcd_mono_text_boxed(105, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, font_VAL);
				change_colors = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
				wait_for_releasseTS(); // wait for releasse TS
				LCD_ClrScr(lcd_background_color);
				HAL_Delay(100);
				return ;
			} else {
				if(action == 2) { // PLUS pressed
					font_VAL++;
				} else { // MINUS pressed
					font_VAL--;
				}
			}
		}
	}
}
//=====================================================================================================================

//=====================================================================================================================
void set_LIGHT(void) {
	uint8_t action;
	uint32_t LIGHT_VAL = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	if(LIGHT_VAL > 10) {LIGHT_VAL = 10;}

	if((test_pressed_point(0, 159, 95, 145)) == true)  {
		LCD_ClrScr(lcd_background_color);

		while(1){
			lcd_text_boxed(85, 5, "SET LIGHT", dum1_ttf, 42);
			sprintf(tekst, "%02d", (int) LIGHT_VAL);
			lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();

			Pressed_Point.x =0;
			Pressed_Point.y =0;
			if((action == 3)) { // DONE pressed
				wait_for_releasseTS(); // wait for releasse TS
				HAL_Delay(100);
				LCD_ClrScr(lcd_background_color);
				return ;
			} else {
				if(action == 2) { // PLUS pressed
					if(LIGHT_VAL < 10) { LIGHT_VAL++;}
				} else { // MINUS pressed
					if(LIGHT_VAL >  0)  { LIGHT_VAL--;}
				}
				HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, LIGHT_VAL);
				set_PWM();
			}

		}

	}
}
//=====================================================================================================================
void set_PWM(void) {
	uint16_t light;
	uint32_t light_val = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	if(light_val == 0) { light = 4096;}
	if(light_val > 10) { light_val = 10;}
	if(light_val > 0)  { light = light_val * 6553; }
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, light_val);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, light);
}

void set_night_PWM(void) {
	uint16_t light;
	uint32_t light_val = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7);
	if(light_val == 0) { light = 4096;}
	if(light_val > 10) { light_val = 10;}
	if(light_val > 0)  { light = light_val * 6553; }
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, light_val);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, light);
}
//=====================================================================================================================
void TTF_test(void) {
	extern uint16_t Paint_Color;

	if(test_pressed_point(159, 319, 45, 95) == true)  {
		Paint_Color = COLOR_565_WHITE;
		Big_TTF_Demo();
		wait_for_releasseTS(); // wait for releasse TS
		Pressed_Point.x =0;
		Pressed_Point.y =0;
		check_work_orientation();
		lcdSetOrientation(LCD_WORK_ORIENTATION);
	}

}
//=====================================================================================================================

//=====================================================================================================================
void paint_test(void) {
	if(test_pressed_point(0, 159, 45, 95) == true)  {
		Paint_Color = COLOR_565_WHITE;
		start_Paint();
		wait_for_releasseTS(); // wait for releasse TS
		Pressed_Point.x =0;
		Pressed_Point.y =0;
	}
}
//=====================================================================================================================


//=====================================================================================================================


//=====================================================================================================================



//=====================================================================================================================
bool set_day(void) {
	uint8_t action;
	if((test_pressed_point(230, 319, 185, 239)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		check_RTC_DATE(); // get max day in month
		while(1){
			lcd_text_boxed(85, 5, "SET DAY", dum1_ttf, 42);
			sprintf(tekst, "%02d", (my_date.Date));
			lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();
			check_RTC_DATE();
			if((action == 3)) { // DONE pressed
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_date.Date == max_day) { my_date.Date = 1;} else { my_date.Date++;}
				} else { // MINUS pressed
					if(my_date.Date == 1) { my_date.Date = max_day;} else { my_date.Date--;}
				}
			}

		}

	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
bool set_month(void) {
	uint8_t action;
	if((test_pressed_point(130, 230, 185, 239)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(55, 5, "SET MONTH", dum1_ttf, 42);
			sprintf(tekst, "%02d", (my_date.Month));
			lcd_mono_text_boxed(125, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();
			check_RTC_DATE();
			if((action == 3)) { // DONE pressed
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_date.Month == 12) { my_date.Month = 1;} else { my_date.Month++;}
				} else { // MINUS pressed
					if(my_date.Month == 1) { my_date.Month = 12;} else { my_date.Month--;}
				}
			}

		}

	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
bool set_year(void) {
	uint8_t action;
	if((test_pressed_point(0, 130, 185, 239)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(65, 5, "SET YEAR", dum1_ttf, 42);
			sprintf(tekst, "%04d", (int) (my_date.Year + ((century - 1) * 100)));
			lcd_mono_text_boxed(95, 85, tekst, digital_7_ttf, 36);
			put_DATE_number();
			action = test_action();
			check_RTC_DATE();
			if((action == 3)) { // DONE pressed
				set_RTC_DATE();
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_date.Year == 99) { my_date.Year = 0;} else { my_date.Year++;}
				} else { // MINUS pressed
					if(my_date.Year == 0) { my_date.Year = 99;} else { my_date.Year--;}
				}
			}

		}

	}
	return false; // doesn't choose set year
}
//=====================================================================================================================
void get_pressed_point(void){
	POINT OneSample;
	while(XPT2046_TouchPressed() != true);
	while((XPT2046_GetFastCoordinates(&OneSample.x, &OneSample.y)) != true ){ ; }
	getDisplayPoint( &Pressed_Point, &OneSample, &matrix ) ;
	if(LCD_WORK_ORIENTATION == LCD_ORIENTATION_LANDSCAPE) {
		Pressed_Point.x = 319 - Pressed_Point.x;
		Pressed_Point.y = 239 - Pressed_Point.y;
	}
	my_utoa(&idx[0], Pressed_Point.x);
	my_utoa(&idy[0], Pressed_Point.y);
	wait_for_releasseTS(); // wait for releasse TS
	HAL_Delay(20);
}
//=====================================================================================================================
void test_setup(void)
{

	if((XPT2046_TouchPressed()) == false) { return; } // no touch pressed so good bye

	get_pressed_point();  // get pressed points coords

	if(clear_ss() == true) { return; } // clear seconds test
	lcd_text_color = 0xFFFF;
	// todo KAFAR
	if(set_hours() == true) { return; }
	if(set_minuts() == true) { return; }
	if(set_year() == true) { return; }
	if(set_month() == true) { return; }
	if(set_day() == true) { return; }
	if(setting_globals() == true) {
		print_RTC();
		wait_for_releasseTS(); // wait for releasse TS
		return;
	}

	wait_for_releasseTS(); // wait for releasse TS if pressed
}
//=====================================================================================================================
bool set_minuts(void) {

	uint8_t action , bckp, chk_min;
	read_RTC();
	chk_min = my_time.Minutes;
	if((test_pressed_point(180, 319, 0, 115)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(35, 5, "SET MINUTS", dum1_ttf, 42);
			put_number( my_time.Minutes);
			action = test_action();
			if((action == 3)) { // DONE pressed
				bckp = my_time.Minutes;
				read_RTC();
				my_time.Minutes = bckp;

				if(chk_min != my_time.Minutes) {

					my_time.Seconds ++;
					if(my_time.Seconds > 59) { my_time.Seconds = 0;}
					__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
					HAL_RTC_WaitForSynchro(&hrtc);
					HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN); // store setup value
				}

				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_time.Minutes == 59) { my_time.Minutes = 0;} else { my_time.Minutes++;}
				} else { // MINUS pressed
					if(my_time.Minutes == 0) { my_time.Minutes = 59;} else { my_time.Minutes--;}
				}
			}

		}

	}
	return false; // doesn't choose set hours
}
//=====================================================================================================================
void wait_for_releasseTS(void){
	while(XPT2046_TouchPressed() == true); // wait for releasse TS
	HAL_Delay(20);
}
//=====================================================================================================================
bool clear_ss(void){

	if(Pressed_Point.x < 70) {
		if(Pressed_Point.y > 125) {
			if(Pressed_Point.y < 185) {
				read_RTC();
				my_time.Seconds = 0;
				HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN);
				print_RTC();
				wait_for_releasseTS(); // wait for releasse TS
				return true;
			}
		}
	}
	return false;
}
//=====================================================================================================================
bool set_hours(void) {

	uint8_t action, bckp, chk_hrs;
	read_RTC();
	chk_hrs = my_time.Hours;
	if((test_pressed_point(0, 150, 0, 115)) == true)  {
		TIM4_IRQHandler(); // disable timer4 and IRQ
		LCD_ClrScr(lcd_background_color);
		while(1){
			lcd_text_boxed(55, 5, "SET HOURS", dum1_ttf, 42);
			put_number( my_time.Hours);
			action = test_action();
			if((action == 3)) { // DONE pressed
				bckp = my_time.Hours;
				read_RTC();
				my_time.Hours = bckp;
				if(chk_hrs != my_time.Hours) {
					HAL_RTC_SetTime(&hrtc, &my_time, RTC_FORMAT_BIN); // store setup value
				}
				LCD_ClrScr(lcd_background_color);
				print_RTC();
				return true;
			} else {
				if(action == 2) { // PLUS pressed
					if(my_time.Hours == 23) { my_time.Hours = 0;} else { my_time.Hours++;}
				} else { // MINUS pressed
					if(my_time.Hours == 0) { my_time.Hours = 23;} else { my_time.Hours--;}
				}
			}

		}

	}
	return false; // doesn't choose set hours
}
//=====================================================================================================================
void put_number(uint8_t data){

	lcd_mono_text_boxed(220, 50, "+", digital_7_ttf, 86);
	lcd_mono_text_boxed(0, 50, "-", digital_7_ttf, 86);
	sprintf(tekst, "%02d", data);
	lcd_mono_text_boxed(80, 50, tekst, digital_7_ttf, 86);
	lcd_text_boxed(115, 180, "DONE", dum1_ttf, 42);
	LCD_No_Fill_Draw(lcd_text_color, 80, 172, 240, 230);
	LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 165);
	LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 165);
}
//=====================================================================================================================
void put_DATE_number(void){

	lcd_mono_text_boxed(220, 50, "+", digital_7_ttf, 86);
	lcd_mono_text_boxed(0, 50, "-", digital_7_ttf, 86);

	lcd_text_boxed(115, 180, "DONE", dum1_ttf, 42);
	LCD_No_Fill_Draw(lcd_text_color, 80, 172, 240, 230);
	LCD_No_Fill_Draw(lcd_text_color, 0, 55, 80, 165);
	LCD_No_Fill_Draw(lcd_text_color, 235, 55, 318, 165);
}
//=====================================================================================================================
uint8_t test_action(void) {
	while(1) {
		get_pressed_point();
		// DONE test
		if((test_pressed_point(80, 240, 172, 230)) == true) { return 3;}
		if((test_pressed_point(235, 318, 55, 165)) == true) { return 2;}
		if((test_pressed_point(0, 80, 55, 165)) == true) { return 1;}
	}
}
//=====================================================================================================================
bool test_pressed_point(uint16_t xmin, uint16_t xmax, uint16_t ymin, uint16_t ymax){
	if(Pressed_Point.x > xmin) {
		if(Pressed_Point.x < xmax){
			if(Pressed_Point.y > ymin){
				if(Pressed_Point.y < ymax){
					return true;
				}
			}
		}
	}
	return false;
}
//=====================================================================================================================



//=====================================================================================================================
//=====================================================================================================================

